/*
 * Copyright (c) 2024, Mario Bălănică <mariobalanica02@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <common/debug.h>
#include <common/runtime_svc.h>
#include <drivers/scmi-msg.h>
#include <lib/mmio.h>

#include <platform_def.h>
#include <rockchip_sip_svc.h>
#include <soc.h>

#include <scmi_clock.h>
#include <rk3588_clk.h>

#include "plat_sip_sdmmc.h"

/* This is board-specific (see rk3588_reference_pmic) */
#pragma weak plat_rk3588_sdmmc_set_signal_voltage

#define DIV_ROUND_CLOSEST(x, divisor) (((x) + ((divisor) / 2)) / (divisor))

#define PICOSECONDS_PER_SECOND	1000000000000ULL

#define CRU_SD_CON_SEL			BIT(11)
#define CRU_SD_CON_DELAYNUM_SHIFT	3
#define CRU_SD_CON_DELAYNUM_MASK 	GENMASK_32(10, 3)
#define CRU_SD_CON_DEGREE_SHIFT		1
#define CRU_SD_CON_DEGREE_MASK		GENMASK_32(2, 1)
#define CRU_SD_CON_INIT_STATE		BIT(0)
#define CRU_SD_CON_MASK			GENMASK_32(11, 0)

#define CRU_SD_CON_DELAYNUM_MAX      255
#define CRU_SD_CON_DEGREE_STEP       90

/*
 * RK3588 TRM-Part2 3.6.7:
 * The delay time of every element is in the range of 36ps~68ps,
 * varying with different voltage and temperature.
 */
#define CRU_SD_DELAY_ELEMENT_PS_MIN	36
#define CRU_SD_DELAY_ELEMENT_PS_MAX	68
#define CRU_SD_DELAY_ELEMENT_PS  \
	((CRU_SD_DELAY_ELEMENT_PS_MIN + CRU_SD_DELAY_ELEMENT_PS_MAX) / 2)

#define CRU_SD_CLKGEN_DIV	2

static unsigned int cru_sd_get_phase(unsigned int con_reg,
				     unsigned int rate_hz)
{
	unsigned int val;
	unsigned int phase_degrees;
	unsigned long long delaynum;

	if (rate_hz == 0) {
		return 0;
	}

	val = mmio_read_32(con_reg);

	phase_degrees = (val & CRU_SD_CON_DEGREE_MASK) >> CRU_SD_CON_DEGREE_SHIFT;
	phase_degrees *= CRU_SD_CON_DEGREE_STEP;

	if ((val & CRU_SD_CON_SEL) != 0) {
		delaynum = (val & CRU_SD_CON_DELAYNUM_MASK) >> CRU_SD_CON_DELAYNUM_SHIFT;
		delaynum *= CRU_SD_DELAY_ELEMENT_PS * rate_hz * 360ULL;

		phase_degrees += DIV_ROUND_CLOSEST(delaynum, PICOSECONDS_PER_SECOND);
	}

	return phase_degrees % 360;
}

static void cru_sd_set_phase(unsigned int con_reg,
			     unsigned int rate_hz,
			     unsigned int phase_degrees)
{
	unsigned int degree_sel, remaining_degrees;
	unsigned long long delaynum;
	unsigned int val = 0;

	if (rate_hz == 0) {
		return;
	}

	degree_sel = phase_degrees / CRU_SD_CON_DEGREE_STEP;
	remaining_degrees = (phase_degrees % CRU_SD_CON_DEGREE_STEP);

	delaynum = DIV_ROUND_CLOSEST(PICOSECONDS_PER_SECOND * remaining_degrees,
			CRU_SD_DELAY_ELEMENT_PS * rate_hz * 360ULL);

	delaynum = MIN(delaynum, (unsigned long long)CRU_SD_CON_DELAYNUM_MAX);

	if (delaynum) {
		val |= CRU_SD_CON_SEL;
	}
	val |= (delaynum << CRU_SD_CON_DELAYNUM_SHIFT) & CRU_SD_CON_DELAYNUM_MASK;
	val |= (degree_sel << CRU_SD_CON_DEGREE_SHIFT) & CRU_SD_CON_DEGREE_MASK;

	mmio_write_32(con_reg, (CRU_SD_CON_MASK << 16) | val);
}

static int get_mshc_phase_shift_cru_reg(uintptr_t controller_address,
					unsigned int id,
					unsigned int *cru_reg)
{
	switch (controller_address) {
	case SDMMC_BASE:
		switch (id) {
		case RK_SIP_SDMMC_CLOCK_ID_MSHC_CIU_DRIVE:
			*cru_reg = CRU_BASE + CRU_SDMMC_CON0;
			return RK_SIP_E_SUCCESS;
		case RK_SIP_SDMMC_CLOCK_ID_MSHC_CIU_SAMPLE:
			*cru_reg = CRU_BASE + CRU_SDMMC_CON1;
			return RK_SIP_E_SUCCESS;
		}
		break;
	case SDIO_BASE:
		switch (id) {
		case RK_SIP_SDMMC_CLOCK_ID_MSHC_CIU_DRIVE:
			*cru_reg = CRU_BASE + CRU_SDIO_CON0;
			return RK_SIP_E_SUCCESS;
		case RK_SIP_SDMMC_CLOCK_ID_MSHC_CIU_SAMPLE:
			*cru_reg = CRU_BASE + CRU_SDIO_CON1;
			return RK_SIP_E_SUCCESS;
		}
		break;
	}

	return RK_SIP_E_NOT_IMPLEMENTED;
}

static int get_sdmmc_card_clock_scmi_id(uintptr_t controller_address,
					unsigned int id,
					unsigned int *scmi_id)
{
	switch (controller_address) {
	case SDMMC_BASE:
		switch (id) {
		case RK_SIP_SDMMC_CLOCK_ID_MSHC_CIU:
			*scmi_id = SCMI_CCLK_SD;
			return RK_SIP_E_SUCCESS;
		}
		break;
	case SDIO_BASE:
		switch (id) {
		case RK_SIP_SDMMC_CLOCK_ID_MSHC_CIU:
			*scmi_id = SCMI_CCLK_SDIO;
			NOTICE("SDIO_CLK_PATCH: SDIO clock service active\n");
			return RK_SIP_E_SUCCESS;
		}
		break;
	case EMMC_BASE:
		switch (id) {
		case RK_SIP_SDMMC_CLOCK_ID_EMMC_CCLK:
			*scmi_id = SCMI_CCLK_EMMC;
			return RK_SIP_E_SUCCESS;
		}
		break;
	}

	return RK_SIP_E_NOT_IMPLEMENTED;
}

static int rk_sip_sdmmc_clock_rate_get(uintptr_t controller_address,
				       unsigned int id,
				       unsigned int *rate_hz)
{
	int ret;
	unsigned int scmi_id;

	ret = get_sdmmc_card_clock_scmi_id(controller_address, id, &scmi_id);
	if (ret) {
		return ret;
	}

	*rate_hz = plat_scmi_clock_get_rate(0, scmi_id);
	if (*rate_hz == 0) {
		return RK_SIP_E_DEVICE_ERROR;
	}

	return RK_SIP_E_SUCCESS;
}

static int rk_sip_sdmmc_clock_rate_set(uintptr_t controller_address,
				       unsigned int id,
				       unsigned int rate_hz)
{
	int ret;
	unsigned int scmi_id;

	ret = get_sdmmc_card_clock_scmi_id(controller_address, id, &scmi_id);
	if (ret) {
		return ret;
	}

	ret = plat_scmi_clock_set_rate(0, scmi_id, rate_hz);
	if (ret) {
		return RK_SIP_E_DEVICE_ERROR;
	}

	return RK_SIP_E_SUCCESS;
}

static int rk_sip_sdmmc_clock_phase_get(uintptr_t controller_address,
					unsigned int id,
					unsigned int *phase_degrees)
{
	int ret;
	unsigned int cru_reg;
	unsigned int rate_hz;

	ret = get_mshc_phase_shift_cru_reg(controller_address, id, &cru_reg);
	if (ret) {
		return ret;
	}

	ret = rk_sip_sdmmc_clock_rate_get(controller_address,
					  RK_SIP_SDMMC_CLOCK_ID_MSHC_CIU,
					  &rate_hz);
	if (ret) {
		return ret;
	}

	rate_hz /= CRU_SD_CLKGEN_DIV;

	*phase_degrees = cru_sd_get_phase(cru_reg, rate_hz);

	return RK_SIP_E_SUCCESS;
}

static int rk_sip_sdmmc_clock_phase_set(uintptr_t controller_address,
					unsigned int id,
					unsigned int phase_degrees)
{
	int ret;
	unsigned int cru_reg;
	unsigned int rate_hz;

	ret = get_mshc_phase_shift_cru_reg(controller_address, id, &cru_reg);
	if (ret) {
		return ret;
	}

	ret = rk_sip_sdmmc_clock_rate_get(controller_address,
					  RK_SIP_SDMMC_CLOCK_ID_MSHC_CIU,
					  &rate_hz);
	if (ret) {
		return ret;
	}

	rate_hz /= CRU_SD_CLKGEN_DIV;

	cru_sd_set_phase(cru_reg, rate_hz, phase_degrees);

	return RK_SIP_E_SUCCESS;
}

static int rk_sip_sdmmc_regulator_voltage_get(uintptr_t controller_address,
					      unsigned int id,
					      unsigned int *microvolts)
{
	return RK_SIP_E_NOT_IMPLEMENTED;
}

static int rk_sip_sdmmc_regulator_voltage_set(uintptr_t controller_address,
					      unsigned int id,
					      unsigned int microvolts)
{
	int ret;

	switch (controller_address) {
	case SDMMC_BASE:
	case SDIO_BASE:	/* W5-A: SDIO 控制器走同样的 PLDO5 信号电压路径 */
		switch (id) {
		case RK_SIP_SDMMC_REGULATOR_ID_SIGNAL:
			ret = plat_rk3588_sdmmc_set_signal_voltage(microvolts);
			if (ret) {
				return RK_SIP_E_DEVICE_ERROR;
			}
			return RK_SIP_E_SUCCESS;
		}
		break;
	}

	return RK_SIP_E_NOT_IMPLEMENTED;
}

static int rk_sip_sdmmc_regulator_enable_get(uintptr_t controller_address,
					     unsigned int id,
					     bool *enable)
{
	return RK_SIP_E_NOT_IMPLEMENTED;
}

static int rk_sip_sdmmc_regulator_enable_set(uintptr_t controller_address,
					     unsigned int id,
					     bool enable)
{
	return RK_SIP_E_NOT_IMPLEMENTED;
}

int plat_rk3588_sdmmc_set_signal_voltage(unsigned int microvolts)
{
	return RK_SIP_E_NOT_IMPLEMENTED;
}

/*
 * YL debug (2026-08-27): one-shot dump of the three values that both the
 * firmware (RockchipPlatformLib SdioWifiInit/SdioBusInit) and the Windows
 * ACPI PINC._REG method rely on, to prove what they mean on silicon.
 * Triggered at EL3 by the dwcmshc regulator service chain
 *   MshcRockchipSetVoltage -> RkSipSdmmcRegulatorEnableSet ->
 *   RK_SIP_SDMMC_REGULATOR_ENABLE_SET (0x82000027),
 * i.e. BEFORE Windows evaluates _REG -- so it validates the register
 * layout/value semantics, not the ACPI-time write itself.
 *
 *  1. G3AL = GPIO3A_IOMUX_SEL_L @ 0xFD5F8060
 *       expect low16 == 0x2222 : GPIO3_A0~A3 mux=2 (SDIO_D0~D3)
 *  2. G3AH = GPIO3A_IOMUX_SEL_H @ 0xFD5F8064
 *       expect low16 bits[7:0] == 0x22 : GPIO3_A4/A5 mux=2 (SDIO_CMD/CLK)
 *       (bits[15:8] belong to A6/A7 which we never touch)
 *  3. WL_REG_ON = GPIO0_C7 (pin 23, released-high):
 *       SWPORT_DDR_H @ 0xFD8A000C bit7 = 1 (direction = output)
 *       SWPORT_DR_H  @ 0xFD8A0004 bit7 = 1 (latched level = high/ON)
 *       EXT_PORT     @ 0xFD8A0070 bit23 = actual pad level
 */
static void sdio_dump_gpio_regs_once(void)
{
	static unsigned int dumped;
	unsigned int sel_l, sel_h, ddr_h, dr_h, ext_port;

	// if (dumped != 0U) {
	// 	return;
	// }
	// dumped = 1U;

	sel_l = mmio_read_32(0xFD5F8060U);
	sel_h = mmio_read_32(0xFD5F8064U);
	ddr_h = mmio_read_32(0xFD8A000CU);
	dr_h = mmio_read_32(0xFD8A0004U);
	ext_port = mmio_read_32(0xFD8A0070U);

	NOTICE("SDIO_CLK_PATCH(0x82000027): Times [%ud]\n", dumped);
	NOTICE("SDIO_CLK_PATCH(0x82000027): G3AL(GPIOM.sel_l @0xFD5F8060)=0x%08x exp_low16=0x2222\n",
	       sel_l);
	NOTICE("SDIO_CLK_PATCH(0x82000027): G3AH(GPIOM.sel_h @0xFD5F8064)=0x%08x exp_bits[7:0]=0x22\n",
	       sel_h);
	NOTICE("SDIO_CLK_PATCH(0x82000027): WL_REG_ON(GPIO0_C7) dir_out=%u latch_high=%u pad_high=%u (exp 1/1/1)\n",
	       (ddr_h >> 7) & 1U, (dr_h >> 7) & 1U, (ext_port >> 23) & 1U);
}

uintptr_t rk_sip_sdmmc_handler(uint32_t smc_fid,
			       u_register_t x1,
			       u_register_t x2,
			       u_register_t x3,
			       u_register_t x4,
			       void *cookie,
			       void *handle,
			       u_register_t flags)
{
	int ret;

	switch (smc_fid) {
	case RK_SIP_SDMMC_CLOCK_RATE_GET: {
		unsigned int rate_hz = 0;

		ret = rk_sip_sdmmc_clock_rate_get(x1, x2, &rate_hz);
		if (!ret) {
			SMC_RET2(handle, RK_SIP_E_SUCCESS, rate_hz);
		} else {
			SMC_RET1(handle, ret);
		}
		break;
	}
	case RK_SIP_SDMMC_CLOCK_RATE_SET: {
		ret = rk_sip_sdmmc_clock_rate_set(x1, x2, x3);
		SMC_RET1(handle, ret);
		break;
	}
	case RK_SIP_SDMMC_CLOCK_PHASE_GET: {
		unsigned int phase_degrees = 0;

		ret = rk_sip_sdmmc_clock_phase_get(x1, x2, &phase_degrees);
		if (!ret) {
			SMC_RET2(handle, RK_SIP_E_SUCCESS, phase_degrees);
		} else {
			SMC_RET1(handle, ret);
		}
		break;
	}
	case RK_SIP_SDMMC_CLOCK_PHASE_SET: {
		ret = rk_sip_sdmmc_clock_phase_set(x1, x2, x3);
		SMC_RET1(handle, ret);
		break;
	}
	case RK_SIP_SDMMC_REGULATOR_VOLTAGE_GET: {
		unsigned int microvolts = 0;

		ret = rk_sip_sdmmc_regulator_voltage_get(x1, x2, &microvolts);
		if (!ret) {
			SMC_RET2(handle, RK_SIP_E_SUCCESS, microvolts);
		} else {
			SMC_RET1(handle, ret);
		}
		break;
	}
	case RK_SIP_SDMMC_REGULATOR_VOLTAGE_SET: {
		ret = rk_sip_sdmmc_regulator_voltage_set(x1, x2, x3);
		SMC_RET1(handle, ret);
		break;
	}
	case RK_SIP_SDMMC_REGULATOR_ENABLE_GET: {
		bool enable = false;
		if ((uintptr_t)x1 == SDIO_BASE) {
			sdio_dump_gpio_regs_once();
		}

		ret = rk_sip_sdmmc_regulator_enable_get(x1, x2, &enable);
		if (!ret) {
			SMC_RET2(handle, RK_SIP_E_SUCCESS, enable);
		} else {
			SMC_RET1(handle, ret);
		}
		break;
	}
	case RK_SIP_SDMMC_REGULATOR_ENABLE_SET: {
		if ((uintptr_t)x1 == SDIO_BASE) {
			sdio_dump_gpio_regs_once();
		}

		ret = rk_sip_sdmmc_regulator_enable_set(x1, x2, x3);
		SMC_RET1(handle, ret);
		break;
	}
	default:
		ERROR("%s: unhandled SMC (0x%x)\n", __func__, smc_fid);
		SMC_RET1(handle, SMC_UNK);
	}

	SMC_RET1(handle, SMC_UNK);
}
