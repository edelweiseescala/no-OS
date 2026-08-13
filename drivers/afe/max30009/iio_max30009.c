/***************************************************************************//**
 *   @file   iio_max30009.c
 *   @brief  Implementation of IIO interface for the MAX30009 BioZ AFE.
 *   @author Edelweise Escala (edelweise.escala@analog.com)
********************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "iio_max30009.h"
#include "max30009.h"
#include "no_os_alloc.h"
#include "no_os_circular_buffer.h"
#include "no_os_util.h"

#define MAX30009_TAG_BIOZ_I	0x1u
#define MAX30009_TAG_BIOZ_Q	0x2u
#define MAX30009_TAG_MASK	NO_OS_GENMASK(23, 20)
#define MAX30009_DATA_MASK	NO_OS_GENMASK(19, 0)

/* Bit flags returned by the status attribute */
#define MAX30009_STATUS_DRV_OOR		NO_OS_BIT(0) /* Drive signal out of range */
#define MAX30009_STATUS_OVERFLOW	NO_OS_BIT(1) /* ADC positive overflow */
#define MAX30009_STATUS_UNDERFLOW	NO_OS_BIT(2) /* ADC negative overflow */

#define MAX30009_SIGN_EXT(v)	no_os_sign_extend32((v), 19)


enum max30009_iio_attr_id {
	/* FIFO */
	MAX30009_IIO_FIFO_DATA_COUNT,
	MAX30009_IIO_FIFO_WATERMARK,
	MAX30009_IIO_FIFO_ROLLOVER,
	MAX30009_IIO_FIFO_A_FULL_TYPE,
	/* PLL */
	MAX30009_IIO_MDIV,
	MAX30009_IIO_NDIV,
	MAX30009_IIO_KDIV,
	MAX30009_IIO_REF_CLK_SEL,
	MAX30009_IIO_CLK_FREQ_SEL,
	MAX30009_IIO_CLK_FINE_TUNE,
	MAX30009_IIO_PLL_ENABLED,
	MAX30009_IIO_PLL_LOCK_STATUS,
	MAX30009_IIO_PLL_LOCK_WNDW,
	/* BioZ TX */
	MAX30009_IIO_BIOZ_DRV_MODE,
	MAX30009_IIO_BIOZ_VDRV_MAG,
	MAX30009_IIO_BIOZ_IDRV_RGE,
	/* BioZ RX */
	MAX30009_IIO_BIOZ_GAIN,
	MAX30009_IIO_BIOZ_DAC_OSR,
	MAX30009_IIO_BIOZ_ADC_OSR,
	MAX30009_IIO_BIOZ_ENABLED,
	MAX30009_IIO_BIOZ_INA_MODE,
	MAX30009_IIO_BIOZ_DM_DIS,
	/* BioZ band-gap (device-level — required before PLL enable) */
	MAX30009_IIO_BIOZ_BG_ENABLED,
	/* BioZ filters */
	MAX30009_IIO_BIOZ_AHPF,
	MAX30009_IIO_BIOZ_DHPF,
	MAX30009_IIO_BIOZ_DLPF,
	MAX30009_IIO_BIOZ_EXT_CAP,
	MAX30009_IIO_BIOZ_DC_RESTORE,
	/* BioZ advanced */
	MAX30009_IIO_BIOZ_AMP_RGE,
	MAX30009_IIO_BIOZ_AMP_BW,
	MAX30009_IIO_BIOZ_FAST_MANUAL,
	MAX30009_IIO_BIOZ_FAST_START_EN,
	MAX30009_IIO_BIOZ_DRV_RESET,
	MAX30009_IIO_BIOZ_DAC_RESET,
	MAX30009_IIO_BIOZ_STBYON,
	MAX30009_IIO_BIOZ_INA_CHOP_EN,
	MAX30009_IIO_BIOZ_CH_FSEL,
	MAX30009_IIO_BIOZ_I_CLK_PHASE,
	MAX30009_IIO_BIOZ_Q_CLK_PHASE,
	/* BioZ thresholds */
	MAX30009_IIO_BIOZ_THRESH_EN,
	MAX30009_IIO_BIOZ_HI_THRESH,
	MAX30009_IIO_BIOZ_LO_THRESH,
	MAX30009_IIO_BIOZ_CMP_MODE,
	/* BioZ MUX */
	MAX30009_IIO_BMUX_DRVP,
	MAX30009_IIO_BMUX_DRVN,
	MAX30009_IIO_BMUX_BIP,
	MAX30009_IIO_BMUX_BIN,
	MAX30009_IIO_BMUX_MUX_EN,
	MAX30009_IIO_BMUX_CAL_EN,
	MAX30009_IIO_BMUX_RSEL,
	MAX30009_IIO_BMUX_GSR_RSEL,
	MAX30009_IIO_BMUX_CONNECT_CAL_ONLY,
	MAX30009_IIO_BMUX_BIST_EN,
	MAX30009_IIO_BMUX_GSR_LOAD_EN,
	MAX30009_IIO_BMUX_EXT_INLOAD_EN,
	MAX30009_IIO_BMUX_INT_INLOAD_EN,
	/* Lead detection */
	MAX30009_IIO_LON_DETECT_EN,
	MAX30009_IIO_LOFF_DETECT_EN,
	MAX30009_IIO_LOFF_EXT_EN,
	MAX30009_IIO_LOFF_DRV_OOR_EN,
	MAX30009_IIO_LOFF_IPOL,
	MAX30009_IIO_LOFF_IMAG,
	MAX30009_IIO_LOFF_THRESH,
	/* Lead bias */
	MAX30009_IIO_RBIAS_VALUE,
	MAX30009_IIO_RBIAS_BIP_EN,
	MAX30009_IIO_RBIAS_BIN_EN,
	/* Status (read-only) */
	MAX30009_IIO_LEAD_OFF_STATUS,
	MAX30009_IIO_FREQ_LOCK_STATUS,
	MAX30009_IIO_PHASE_LOCK_STATUS,
	MAX30009_IIO_FIFO_LEVEL,
	MAX30009_IIO_STATUS,
	MAX30009_IIO_OVERRUN_COUNT,
	/* Pin configuration */
	MAX30009_IIO_INT_FCFG,
	MAX30009_IIO_INT_OCFG,
	MAX30009_IIO_TRIG_ICFG,
	MAX30009_IIO_TRIG_OCFG,
	/* System */
	MAX30009_IIO_MASTER_MODE,
	MAX30009_IIO_POWER_MODE,
};

static int max30009_iio_get_attr(void *device, char *buf, uint32_t len,
				 const struct iio_ch_info *channel,
				 intptr_t priv)
{
	struct max30009_iio_dev *iio_dev = device;
	struct max30009_dev *dev = iio_dev->max30009_dev;
	struct max30009_pll_config pll_cfg;
	struct max30009_bioz_config bioz_cfg;
	struct max30009_bioz_mux_config mux_cfg;
	struct max30009_lead_detect_config ld_cfg;
	struct max30009_status status;
	int32_t val = 0;
	uint8_t reg_val;
	uint16_t fifo_count;
	int ret;

	switch ((enum max30009_iio_attr_id)priv) {
	/* ---- FIFO ---- */
	case MAX30009_IIO_FIFO_DATA_COUNT:
		ret = max30009_fifo_get_count(dev, &fifo_count);
		if (ret)
			return ret;
		val = fifo_count;
		break;

	case MAX30009_IIO_FIFO_WATERMARK: {
		uint8_t wm;

		ret = max30009_get_fifo_watermark(dev, &wm);
		if (ret)
			return ret;
		val = wm;
		break;
	}

	case MAX30009_IIO_FIFO_ROLLOVER:
		ret = max30009_reg_read(dev, MAX30009_REG_FIFO_CONFIGURATION1,
					&reg_val);
		if (ret)
			return ret;
		val = no_os_field_get(MAX30009_FIFO_RO_MSK, reg_val);
		break;

	case MAX30009_IIO_FIFO_A_FULL_TYPE:
		ret = max30009_reg_read(dev, MAX30009_REG_FIFO_CONFIGURATION2,
					&reg_val);
		if (ret)
			return ret;
		val = no_os_field_get(MAX30009_A_FULL_TYPE_MSK, reg_val);
		break;

	/* ---- PLL ---- */
	case MAX30009_IIO_MDIV:
		ret = max30009_get_pll_config(dev, &pll_cfg);
		if (ret)
			return ret;
		val = pll_cfg.mdiv;
		break;

	case MAX30009_IIO_NDIV:
		ret = max30009_get_pll_config(dev, &pll_cfg);
		if (ret)
			return ret;
		val = pll_cfg.ndiv;
		break;

	case MAX30009_IIO_KDIV:
		ret = max30009_get_pll_config(dev, &pll_cfg);
		if (ret)
			return ret;
		val = pll_cfg.kdiv;
		break;

	case MAX30009_IIO_REF_CLK_SEL:
		ret = max30009_get_pll_config(dev, &pll_cfg);
		if (ret)
			return ret;
		val = pll_cfg.use_external_clk;
		break;

	case MAX30009_IIO_CLK_FREQ_SEL:
		ret = max30009_get_pll_config(dev, &pll_cfg);
		if (ret)
			return ret;
		val = pll_cfg.use_32768_hz;
		break;

	case MAX30009_IIO_CLK_FINE_TUNE:
		ret = max30009_get_pll_config(dev, &pll_cfg);
		if (ret)
			return ret;
		val = pll_cfg.clk_fine_tune;
		break;

	case MAX30009_IIO_PLL_ENABLED:
		ret = max30009_reg_read(dev, MAX30009_REG_PLL_CONFIGURATION1,
					&reg_val);
		if (ret)
			return ret;
		val = no_os_field_get(MAX30009_PLL_EN_MSK, reg_val);
		break;

	case MAX30009_IIO_PLL_LOCK_STATUS:
		ret = max30009_get_status(dev, &status);
		if (ret)
			return ret;
		val = status.phase_lock;
		break;

	case MAX30009_IIO_PLL_LOCK_WNDW:
		ret = max30009_reg_read(dev, MAX30009_REG_PLL_CONFIGURATION1,
					&reg_val);
		if (ret)
			return ret;
		val = no_os_field_get(MAX30009_PLL_LOCK_WNDW_MSK, reg_val);
		break;

	/* ---- BioZ TX ---- */
	case MAX30009_IIO_BIOZ_DRV_MODE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.drv_mode;
		break;

	case MAX30009_IIO_BIOZ_VDRV_MAG:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.vdrv_mag;
		break;

	case MAX30009_IIO_BIOZ_IDRV_RGE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.idrv_rge;
		break;

	/* ---- BioZ RX ---- */
	case MAX30009_IIO_BIOZ_GAIN:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.gain;
		break;

	case MAX30009_IIO_BIOZ_DAC_OSR:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.dac_osr;
		break;

	case MAX30009_IIO_BIOZ_ADC_OSR:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.adc_osr;
		break;

	case MAX30009_IIO_BIOZ_ENABLED:
		ret = max30009_reg_read(dev, MAX30009_REG_BIOZ_CONFIGURATION1,
					&reg_val);
		if (ret)
			return ret;
		val = no_os_field_get(MAX30009_BIOZ_I_EN_MSK, reg_val) &&
		      no_os_field_get(MAX30009_BIOZ_Q_EN_MSK, reg_val) &&
		      no_os_field_get(MAX30009_BIOZ_BG_EN_MSK, reg_val);
		break;

	case MAX30009_IIO_BIOZ_BG_ENABLED:
		ret = max30009_reg_read(dev, MAX30009_REG_BIOZ_CONFIGURATION1,
					&reg_val);
		if (ret)
			return ret;
		val = no_os_field_get(MAX30009_BIOZ_BG_EN_MSK, reg_val);
		break;

	case MAX30009_IIO_BIOZ_INA_MODE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.ina_mode;
		break;

	case MAX30009_IIO_BIOZ_DM_DIS:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.dm_dis;
		break;

	/* ---- BioZ filters ---- */
	case MAX30009_IIO_BIOZ_AHPF:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.ahpf;
		break;

	case MAX30009_IIO_BIOZ_DHPF:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.dhpf;
		break;

	case MAX30009_IIO_BIOZ_DLPF:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.dlpf;
		break;

	case MAX30009_IIO_BIOZ_EXT_CAP:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.ext_cap;
		break;

	case MAX30009_IIO_BIOZ_DC_RESTORE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.dc_restore;
		break;

	/* ---- BioZ advanced ---- */
	case MAX30009_IIO_BIOZ_AMP_RGE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.amp_rge;
		break;

	case MAX30009_IIO_BIOZ_AMP_BW:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.amp_bw;
		break;

	case MAX30009_IIO_BIOZ_FAST_MANUAL:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.fast_manual;
		break;

	case MAX30009_IIO_BIOZ_FAST_START_EN:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.fast_start_en;
		break;

	case MAX30009_IIO_BIOZ_DRV_RESET:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.drv_reset;
		break;

	case MAX30009_IIO_BIOZ_DAC_RESET:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.dac_reset;
		break;

	case MAX30009_IIO_BIOZ_STBYON:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.stbyon;
		break;

	case MAX30009_IIO_BIOZ_INA_CHOP_EN:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.ina_chop_en;
		break;

	case MAX30009_IIO_BIOZ_CH_FSEL:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.ch_fsel;
		break;

	case MAX30009_IIO_BIOZ_I_CLK_PHASE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.i_clk_phase;
		break;

	case MAX30009_IIO_BIOZ_Q_CLK_PHASE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.q_clk_phase;
		break;

	/* ---- BioZ thresholds ---- */
	case MAX30009_IIO_BIOZ_THRESH_EN:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.en_bioz_thresh;
		break;

	case MAX30009_IIO_BIOZ_HI_THRESH:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.hi_thresh;
		break;

	case MAX30009_IIO_BIOZ_LO_THRESH:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.lo_thresh;
		break;

	case MAX30009_IIO_BIOZ_CMP_MODE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		val = bioz_cfg.cmp_mode;
		break;

	/* ---- BioZ MUX ---- */
	case MAX30009_IIO_BMUX_DRVP:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.drvp_assign;
		break;

	case MAX30009_IIO_BMUX_DRVN:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.drvn_assign;
		break;

	case MAX30009_IIO_BMUX_BIP:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.bip_assign;
		break;

	case MAX30009_IIO_BMUX_BIN:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.bin_assign;
		break;

	case MAX30009_IIO_BMUX_MUX_EN:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.mux_en;
		break;

	case MAX30009_IIO_BMUX_CAL_EN:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.cal_en;
		break;

	case MAX30009_IIO_BMUX_RSEL:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.bmux_rsel;
		break;

	case MAX30009_IIO_BMUX_GSR_RSEL:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.bmux_gsr_rsel;
		break;

	case MAX30009_IIO_BMUX_CONNECT_CAL_ONLY:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.connect_cal_only;
		break;

	case MAX30009_IIO_BMUX_BIST_EN:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.bmux_bist_en;
		break;

	case MAX30009_IIO_BMUX_GSR_LOAD_EN:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.gsr_load_en;
		break;

	case MAX30009_IIO_BMUX_EXT_INLOAD_EN:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.en_ext_inload;
		break;

	case MAX30009_IIO_BMUX_INT_INLOAD_EN:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		val = mux_cfg.en_int_inload;
		break;

	/* ---- Lead detection ---- */
	case MAX30009_IIO_LON_DETECT_EN:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		val = ld_cfg.en_lon_det;
		break;

	case MAX30009_IIO_LOFF_DETECT_EN:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		val = ld_cfg.en_loff_det;
		break;

	case MAX30009_IIO_LOFF_EXT_EN:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		val = ld_cfg.en_ext_loff;
		break;

	case MAX30009_IIO_LOFF_DRV_OOR_EN:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		val = ld_cfg.en_drv_oor;
		break;

	case MAX30009_IIO_LOFF_IPOL:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		val = ld_cfg.loff_ipol;
		break;

	case MAX30009_IIO_LOFF_IMAG:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		val = ld_cfg.loff_imag;
		break;

	case MAX30009_IIO_LOFF_THRESH:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		val = ld_cfg.loff_thresh;
		break;

	/* ---- Lead bias ---- */
	case MAX30009_IIO_RBIAS_VALUE:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		val = ld_cfg.rbias_value;
		break;

	case MAX30009_IIO_RBIAS_BIP_EN:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		val = ld_cfg.en_rbias_bip;
		break;

	case MAX30009_IIO_RBIAS_BIN_EN:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		val = ld_cfg.en_rbias_bin;
		break;

	/* ---- Status (read-only) ---- */
	case MAX30009_IIO_LEAD_OFF_STATUS:
		ret = max30009_get_status(dev, &status);
		if (ret)
			return ret;
		val = status.leads_on ? 0 : 1;
		break;

	case MAX30009_IIO_FREQ_LOCK_STATUS:
		ret = max30009_get_status(dev, &status);
		if (ret)
			return ret;
		val = status.freq_lock;
		break;

	case MAX30009_IIO_PHASE_LOCK_STATUS:
		ret = max30009_get_status(dev, &status);
		if (ret)
			return ret;
		val = status.phase_lock;
		break;

	case MAX30009_IIO_FIFO_LEVEL:
		ret = max30009_fifo_get_count(dev, &fifo_count);
		if (ret)
			return ret;
		val = fifo_count;
		break;

	case MAX30009_IIO_STATUS:
		/* Bit 0 = drive out-of-range, bit 1 = ADC overflow, bit 2 = ADC underflow */
		ret = max30009_get_status(dev, &status);
		if (ret)
			return ret;
		val = (status.drv_oor   ? NO_OS_BIT(0) : 0) |
		      (status.bioz_over ? NO_OS_BIT(1) : 0) |
		      (status.bioz_undr ? NO_OS_BIT(2) : 0);
		break;

	case MAX30009_IIO_OVERRUN_COUNT:
		return snprintf(buf, len, "%lu",
				(unsigned long)iio_dev->overrun_count);

	/* ---- Pin configuration ---- */
	case MAX30009_IIO_INT_FCFG:
		ret = max30009_reg_read(dev,
					MAX30009_REG_PIN_FUNC_CONFIGURATION,
					&reg_val);
		if (ret)
			return ret;
		val = no_os_field_get(MAX30009_INT_FCFG_MSK, reg_val);
		break;

	case MAX30009_IIO_INT_OCFG:
		ret = max30009_reg_read(dev,
					MAX30009_REG_OUTPUT_PIN_CONFIGURATION,
					&reg_val);
		if (ret)
			return ret;
		val = no_os_field_get(MAX30009_INT_OCFG_MSK, reg_val);
		break;

	case MAX30009_IIO_TRIG_ICFG:
		ret = max30009_reg_read(dev,
					MAX30009_REG_PIN_FUNC_CONFIGURATION,
					&reg_val);
		if (ret)
			return ret;
		val = no_os_field_get(MAX30009_TRIG_ICFG_MSK, reg_val);
		break;

	case MAX30009_IIO_TRIG_OCFG:
		ret = max30009_reg_read(dev,
					MAX30009_REG_OUTPUT_PIN_CONFIGURATION,
					&reg_val);
		if (ret)
			return ret;
		val = no_os_field_get(MAX30009_TRIG_OCFG_MSK, reg_val);
		break;

	/* ---- System ---- */
	case MAX30009_IIO_MASTER_MODE:
		ret = max30009_reg_read(dev,
					MAX30009_REG_SYSTEM_CONFIGURATION1,
					&reg_val);
		if (ret)
			return ret;
		val = no_os_field_get(MAX30009_MASTER_MSK, reg_val);
		break;

	case MAX30009_IIO_POWER_MODE:
		ret = max30009_reg_read(dev,
					MAX30009_REG_SYSTEM_CONFIGURATION1,
					&reg_val);
		if (ret)
			return ret;
		/* 0 = active (SHDN cleared), 1 = shutdown */
		val = no_os_field_get(MAX30009_SHDN_MSK, reg_val);
		break;

	default:
		return -EINVAL;
	}

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

static int max30009_iio_set_attr(void *device, char *buf, uint32_t len,
				 const struct iio_ch_info *channel,
				 intptr_t priv)
{
	struct max30009_iio_dev *iio_dev = device;
	struct max30009_dev *dev = iio_dev->max30009_dev;
	struct max30009_pll_config pll_cfg;
	struct max30009_bioz_config bioz_cfg;
	struct max30009_bioz_mux_config mux_cfg;
	struct max30009_lead_detect_config ld_cfg;
	int32_t val;
	int ret;

	ret = iio_parse_value(buf, IIO_VAL_INT, &val, NULL);
	if (ret)
		return ret;

	switch ((enum max30009_iio_attr_id)priv) {
	/* ---- FIFO ---- */
	case MAX30009_IIO_FIFO_WATERMARK:
		return max30009_set_fifo_watermark(dev, (uint16_t)val);

	case MAX30009_IIO_FIFO_ROLLOVER:
		return max30009_set_fifo_rollover(dev, (bool)val);

	case MAX30009_IIO_FIFO_A_FULL_TYPE:
		return max30009_set_a_full_type(dev, (bool)val);

	/* ---- PLL ---- */
	case MAX30009_IIO_MDIV:
		ret = max30009_get_pll_config(dev, &pll_cfg);
		if (ret)
			return ret;
		pll_cfg.mdiv = (uint16_t)val;
		return max30009_set_pll_config(dev, &pll_cfg);

	case MAX30009_IIO_NDIV:
		ret = max30009_get_pll_config(dev, &pll_cfg);
		if (ret)
			return ret;
		pll_cfg.ndiv = (enum max30009_pll_ndiv)val;
		return max30009_set_pll_config(dev, &pll_cfg);

	case MAX30009_IIO_KDIV:
		ret = max30009_get_pll_config(dev, &pll_cfg);
		if (ret)
			return ret;
		pll_cfg.kdiv = (enum max30009_pll_kdiv)val;
		return max30009_set_pll_config(dev, &pll_cfg);

	case MAX30009_IIO_REF_CLK_SEL:
		ret = max30009_get_pll_config(dev, &pll_cfg);
		if (ret)
			return ret;
		pll_cfg.use_external_clk = (bool)val;
		return max30009_set_pll_config(dev, &pll_cfg);

	case MAX30009_IIO_CLK_FREQ_SEL:
		ret = max30009_get_pll_config(dev, &pll_cfg);
		if (ret)
			return ret;
		pll_cfg.use_32768_hz = (bool)val;
		return max30009_set_pll_config(dev, &pll_cfg);

	case MAX30009_IIO_CLK_FINE_TUNE:
		return max30009_clock_fine_tune(dev, (uint8_t)val);

	case MAX30009_IIO_PLL_ENABLED:
		return max30009_pll_enable(dev, (bool)val);

	case MAX30009_IIO_PLL_LOCK_WNDW:
		return max30009_pll_set_lock_window(dev, (bool)val);

	/* ---- BioZ TX ---- */
	case MAX30009_IIO_BIOZ_DRV_MODE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.drv_mode = (enum max30009_drive_mode)val;
		return max30009_set_bioz_config(dev, &bioz_cfg);

	case MAX30009_IIO_BIOZ_VDRV_MAG:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.vdrv_mag = (uint8_t)val;
		return max30009_set_bioz_config(dev, &bioz_cfg);

	case MAX30009_IIO_BIOZ_IDRV_RGE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.idrv_rge = (uint8_t)val;
		return max30009_set_bioz_config(dev, &bioz_cfg);

	/* ---- BioZ RX ---- */
	case MAX30009_IIO_BIOZ_GAIN:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.gain = (enum max30009_bioz_gain)val;
		return max30009_set_bioz_config(dev, &bioz_cfg);

	case MAX30009_IIO_BIOZ_DAC_OSR:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.dac_osr = (enum max30009_dac_osr)val;
		return max30009_set_bioz_config(dev, &bioz_cfg);

	case MAX30009_IIO_BIOZ_ADC_OSR:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.adc_osr = (enum max30009_adc_osr)val;
		return max30009_set_bioz_config(dev, &bioz_cfg);

	case MAX30009_IIO_BIOZ_ENABLED:
		return max30009_bioz_enable(dev, (bool)val);

	case MAX30009_IIO_BIOZ_BG_ENABLED:
		return max30009_bioz_bg_enable(dev, (bool)val);

	case MAX30009_IIO_BIOZ_INA_MODE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.ina_mode = (bool)val;
		return max30009_set_bioz_config(dev, &bioz_cfg);

	case MAX30009_IIO_BIOZ_DM_DIS:
		return max30009_disable_differential_mode(dev, (bool)val);

	/* ---- BioZ filters ---- */
	case MAX30009_IIO_BIOZ_AHPF:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.ahpf = (enum max30009_ahpf)val;
		return max30009_set_bioz_config(dev, &bioz_cfg);

	case MAX30009_IIO_BIOZ_DHPF:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.dhpf = (enum max30009_dhpf)val;
		return max30009_set_bioz_config(dev, &bioz_cfg);

	case MAX30009_IIO_BIOZ_DLPF:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.dlpf = (enum max30009_dlpf)val;
		return max30009_set_bioz_config(dev, &bioz_cfg);

	case MAX30009_IIO_BIOZ_EXT_CAP:
		return max30009_enable_external_cap(dev, (bool)val);

	case MAX30009_IIO_BIOZ_DC_RESTORE:
		return max30009_enable_dc_restore(dev, (bool)val);

	/* ---- BioZ advanced ---- */
	case MAX30009_IIO_BIOZ_AMP_RGE:
		return max30009_set_amp_range(dev, (uint8_t)val);

	case MAX30009_IIO_BIOZ_AMP_BW:
		return max30009_set_amp_bandwidth(dev, (uint8_t)val);

	case MAX30009_IIO_BIOZ_FAST_MANUAL:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.fast_manual = (bool)val;
		return max30009_set_bioz_config(dev, &bioz_cfg);

	case MAX30009_IIO_BIOZ_FAST_START_EN:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		return max30009_enable_fast_start(dev, (bool)bioz_cfg.fast_manual,
						  (bool)val);

	case MAX30009_IIO_BIOZ_DRV_RESET:
		return max30009_enable_drive_reset(dev, (bool)val);

	case MAX30009_IIO_BIOZ_DAC_RESET:
		return max30009_enable_dac_reset(dev, (bool)val);

	case MAX30009_IIO_BIOZ_STBYON:
		return max30009_enable_standby_mode(dev, (bool)val);

	case MAX30009_IIO_BIOZ_INA_CHOP_EN:
		return max30009_enable_ina_chopping(dev, (bool)val);

	case MAX30009_IIO_BIOZ_CH_FSEL:
		return max30009_set_channel_freq_select(dev, (uint8_t)val);

	case MAX30009_IIO_BIOZ_I_CLK_PHASE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.i_clk_phase = (bool)val;
		return max30009_set_demod_clk_phases(dev, bioz_cfg.i_clk_phase,
						     bioz_cfg.q_clk_phase);

	case MAX30009_IIO_BIOZ_Q_CLK_PHASE:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.q_clk_phase = (bool)val;
		return max30009_set_demod_clk_phases(dev, bioz_cfg.i_clk_phase,
						     bioz_cfg.q_clk_phase);

	/* ---- BioZ thresholds ---- */
	case MAX30009_IIO_BIOZ_THRESH_EN:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		bioz_cfg.en_bioz_thresh = (bool)val;
		return max30009_set_bioz_config(dev, &bioz_cfg);

	case MAX30009_IIO_BIOZ_HI_THRESH:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		return max30009_set_bioz_thresholds(dev, (uint16_t)val,
						    bioz_cfg.lo_thresh);

	case MAX30009_IIO_BIOZ_LO_THRESH:
		ret = max30009_get_bioz_config(dev, &bioz_cfg);
		if (ret)
			return ret;
		return max30009_set_bioz_thresholds(dev, bioz_cfg.hi_thresh,
						    (uint16_t)val);

	case MAX30009_IIO_BIOZ_CMP_MODE:
		return max30009_set_comparison_mode(dev, (bool)val);

	/* ---- BioZ MUX ---- */
	case MAX30009_IIO_BMUX_DRVP:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		mux_cfg.drvp_assign = (uint8_t)val;
		return max30009_set_bioz_mux_config(dev, &mux_cfg);

	case MAX30009_IIO_BMUX_DRVN:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		mux_cfg.drvn_assign = (uint8_t)val;
		return max30009_set_bioz_mux_config(dev, &mux_cfg);

	case MAX30009_IIO_BMUX_BIP:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		mux_cfg.bip_assign = (uint8_t)val;
		return max30009_set_bioz_mux_config(dev, &mux_cfg);

	case MAX30009_IIO_BMUX_BIN:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		mux_cfg.bin_assign = (uint8_t)val;
		return max30009_set_bioz_mux_config(dev, &mux_cfg);

	case MAX30009_IIO_BMUX_MUX_EN:
		ret = max30009_get_bioz_mux_config(dev, &mux_cfg);
		if (ret)
			return ret;
		mux_cfg.mux_en = (bool)val;
		return max30009_set_bioz_mux_config(dev, &mux_cfg);

	case MAX30009_IIO_BMUX_CAL_EN:
		return max30009_enable_calibration(dev, (bool)val);

	case MAX30009_IIO_BMUX_RSEL:
		return max30009_set_bia_load_resistor(dev,
					(enum max30009_bia_rsel)val);

	case MAX30009_IIO_BMUX_GSR_RSEL:
		return max30009_set_gsr_load_resistor(dev,
					(enum max30009_gsr_rsel)val);

	case MAX30009_IIO_BMUX_CONNECT_CAL_ONLY:
		return max30009_calibration_connect_only(dev, (bool)val);

	case MAX30009_IIO_BMUX_BIST_EN:
		return max30009_enable_bia_bist(dev, (bool)val);

	case MAX30009_IIO_BMUX_GSR_LOAD_EN:
		return max30009_enable_gsr_load(dev, (bool)val);

	case MAX30009_IIO_BMUX_EXT_INLOAD_EN:
		return max30009_enable_external_load(dev, (bool)val);

	case MAX30009_IIO_BMUX_INT_INLOAD_EN:
		return max30009_enable_internal_load(dev, (bool)val);

	/* ---- Lead detection ---- */
	case MAX30009_IIO_LON_DETECT_EN:
		return max30009_enable_ultra_low_power_lead_on(dev, (bool)val);

	case MAX30009_IIO_LOFF_DETECT_EN:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		ld_cfg.en_loff_det = (bool)val;
		return max30009_set_lead_detect_config(dev, &ld_cfg);

	case MAX30009_IIO_LOFF_EXT_EN:
		return max30009_enable_external_lead_off(dev, (bool)val);

	case MAX30009_IIO_LOFF_DRV_OOR_EN:
		return max30009_enable_drive_oor_detect(dev, (bool)val);

	case MAX30009_IIO_LOFF_IPOL:
		return max30009_set_lead_off_polarity(dev, (bool)val);

	case MAX30009_IIO_LOFF_IMAG:
		return max30009_set_lead_off_current(dev,
					(enum max30009_loff_current)val);

	case MAX30009_IIO_LOFF_THRESH:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		ld_cfg.loff_thresh = (uint8_t)val;
		return max30009_set_lead_detect_config(dev, &ld_cfg);

	/* ---- Lead bias ---- */
	case MAX30009_IIO_RBIAS_VALUE:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		ld_cfg.rbias_value = (enum max30009_rbias_val)val;
		return max30009_set_lead_detect_config(dev, &ld_cfg);

	case MAX30009_IIO_RBIAS_BIP_EN:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		ld_cfg.en_rbias_bip = (bool)val;
		return max30009_set_lead_detect_config(dev, &ld_cfg);

	case MAX30009_IIO_RBIAS_BIN_EN:
		ret = max30009_get_lead_detect_config(dev, &ld_cfg);
		if (ret)
			return ret;
		ld_cfg.en_rbias_bin = (bool)val;
		return max30009_set_lead_detect_config(dev, &ld_cfg);

	/* ---- Pin configuration ---- */
	case MAX30009_IIO_INT_FCFG:
		return max30009_reg_update(dev,
					   MAX30009_REG_PIN_FUNC_CONFIGURATION,
					   MAX30009_INT_FCFG_MSK,
					   (uint8_t)val);

	case MAX30009_IIO_INT_OCFG:
		return max30009_reg_update(dev,
					   MAX30009_REG_OUTPUT_PIN_CONFIGURATION,
					   MAX30009_INT_OCFG_MSK,
					   (uint8_t)val);

	case MAX30009_IIO_TRIG_ICFG:
		return max30009_reg_update(dev,
					   MAX30009_REG_PIN_FUNC_CONFIGURATION,
					   MAX30009_TRIG_ICFG_MSK,
					   (uint8_t)val);

	case MAX30009_IIO_TRIG_OCFG:
		return max30009_reg_update(dev,
					   MAX30009_REG_OUTPUT_PIN_CONFIGURATION,
					   MAX30009_TRIG_OCFG_MSK,
					   (uint8_t)val);

	/* ---- System ---- */
	case MAX30009_IIO_MASTER_MODE:
		return max30009_set_master_mode(dev, (bool)val);

	case MAX30009_IIO_POWER_MODE:
		return max30009_set_power_mode(dev, (bool)val);

	default:
		return -EINVAL;
	}
}

static int max30009_iio_reg_read(void *dev, uint32_t reg, uint32_t *val)
{
	struct max30009_iio_dev *iio_dev = dev;
	uint8_t read_val;
	int ret;

	if (reg > 0xFF)
		return -EINVAL;

	ret = max30009_reg_read(iio_dev->max30009_dev, (uint8_t)reg, &read_val);
	if (ret)
		return ret;

	*val = read_val;
	return 0;
}

static int max30009_iio_reg_write(void *dev, uint32_t reg, uint32_t val)
{
	struct max30009_iio_dev *iio_dev = dev;

	if (reg > 0xFF || val > 0xFF)
		return -EINVAL;

	return max30009_reg_write(iio_dev->max30009_dev, (uint8_t)reg,
				  (uint8_t)val);
}

/**
 * @brief Called by the IIO layer before enabling the buffer.
 *
 * Flushes the hardware FIFO and circular buffer, then enables BioZ and arms
 * the interrupt handler so it starts filling the circular buffer.
 */
static int max30009_iio_pre_enable(void *dev, uint32_t mask)
{
	struct max30009_iio_dev *iio_dev = (struct max30009_iio_dev *)dev;
	int ret;

	if (!iio_dev || !iio_dev->max30009_dev)
		return -EINVAL;

	ret = max30009_fifo_flush(iio_dev->max30009_dev);
	if (ret)
		return ret;

	no_os_cb_cfg(iio_dev->fifo_buf, iio_dev->fifo_buf->buff,
		     iio_dev->fifo_buf->size);
	iio_dev->samples_available = 0;
	iio_dev->active_mask = mask;
	iio_dev->buffer_active = true;

	return 0;
}

/**
 * @brief Called by the IIO layer after disabling the buffer.
 *
 * Disarms the interrupt handler so it stops writing to the circular buffer.
 */
static int max30009_iio_post_disable(void *dev)
{
	struct max30009_iio_dev *iio_dev = (struct max30009_iio_dev *)dev;

	if (!iio_dev)
		return -EINVAL;

	iio_dev->buffer_active = false;

	return 0;
}

/**
 * @brief Transfer acquired samples from the circular buffer to the IIO layer.
 */
static int max30009_iio_submit(struct iio_device_data *dev_data)
{
	struct max30009_iio_dev *iio_dev;
	uint32_t n_scans;
	uint32_t raw;
	uint32_t scan[2]; /* [0] = I component, [1] = Q component */
	bool has_i, has_q;
	uint32_t i;
	uint32_t drain_limit;
	int ret;

	if (!dev_data)
		return -EINVAL;

	iio_dev = (struct max30009_iio_dev *)dev_data->dev;
	if (!iio_dev->max30009_dev)
		return -EINVAL;

	if (!(iio_dev->active_mask & NO_OS_BIT(0)))
		return 0;

	n_scans = dev_data->buffer->samples;

	drain_limit = n_scans * 8;

	for (i = 0; i < n_scans; i++) {
		has_i = false;
		has_q = false;
		scan[0] = 0;
		scan[1] = 0;

		/*
		 * Consume up to two tagged FIFO words to form one scan.
		 * The FIFO interleaves I then Q; route each by tag into the
		 * appropriate scan slot. Missing tags are zero-filled.
		 */
		while ((!has_i || !has_q) &&
		       iio_dev->samples_available > 0 &&
		       drain_limit > 0) {
			ret = no_os_cb_read(iio_dev->fifo_buf, &raw, 3);
			if (ret)
				break;

			iio_dev->samples_available--;
			drain_limit--;

			switch (no_os_field_get(MAX30009_TAG_MASK, raw)) {
			case MAX30009_TAG_BIOZ_I:
				scan[0] = raw & MAX30009_DATA_MASK;
				has_i = true;
				break;
			case MAX30009_TAG_BIOZ_Q:
				scan[1] = raw & MAX30009_DATA_MASK;
				has_q = true;
				break;
			default:
				/* Marker or reserved tag — discard and continue */
				break;
			}
		}

		if (drain_limit == 0 && (!has_i || !has_q))
			iio_dev->overrun_count++;

		ret = iio_buffer_push_scan(dev_data->buffer, scan);
		if (ret)
			return ret;
	}

	return dev_data->buffer->size / dev_data->buffer->bytes_per_scan;
}

/*
 * Device-level attributes: operational status and buffer health only.
 * All configuration is applied at init time via max30009_iio_init_param.
 */
static struct iio_attribute max30009_iio_global_attrs[] = {
	/* PLL lock status */
	MAX30009_ATTR_RO("pll_locked",			MAX30009_IIO_PLL_LOCK_STATUS),
	/* Lead detection status */
	MAX30009_ATTR_RO("lead_off_detected",		MAX30009_IIO_LEAD_OFF_STATUS),
	/* Fault status bitmask (mirrors interrupt status registers) */
	/* bit 0 = drv_oor, bit 1 = adc_overflow, bit 2 = adc_underflow; 0 = ok */
	MAX30009_ATTR_RO("status",			MAX30009_IIO_STATUS),
	/* Buffer health */
	MAX30009_ATTR_RO("overrun_count",		MAX30009_IIO_OVERRUN_COUNT),

	END_ATTRIBUTES_ARRAY
};

/*
 * Debug attributes: full configuration access for development and calibration.
 * In production, configuration is done at init time via struct fields.
 */
static struct iio_attribute max30009_iio_debug_attrs[] = {
	/* System */
	MAX30009_ATTR("power_mode",			MAX30009_IIO_POWER_MODE),
	MAX30009_ATTR("master_mode",			MAX30009_IIO_MASTER_MODE),
	/* FIFO */
	MAX30009_ATTR_RO("fifo_data_count",		MAX30009_IIO_FIFO_DATA_COUNT),
	MAX30009_ATTR_RO("fifo_level",			MAX30009_IIO_FIFO_LEVEL),
	MAX30009_ATTR("fifo_watermark",			MAX30009_IIO_FIFO_WATERMARK),
	MAX30009_ATTR("fifo_rollover",			MAX30009_IIO_FIFO_ROLLOVER),
	MAX30009_ATTR("fifo_a_full_type",		MAX30009_IIO_FIFO_A_FULL_TYPE),
	/* PLL */
	MAX30009_ATTR("pll_enabled",			MAX30009_IIO_PLL_ENABLED),
	MAX30009_ATTR_RO("freq_lock_status",		MAX30009_IIO_FREQ_LOCK_STATUS),
	MAX30009_ATTR_RO("phase_lock_status",		MAX30009_IIO_PHASE_LOCK_STATUS),
	MAX30009_ATTR("pll_lock_wndw",			MAX30009_IIO_PLL_LOCK_WNDW),
	MAX30009_ATTR("mdiv",				MAX30009_IIO_MDIV),
	MAX30009_ATTR("ndiv",				MAX30009_IIO_NDIV),
	MAX30009_ATTR("kdiv",				MAX30009_IIO_KDIV),
	MAX30009_ATTR("ref_clk_sel",			MAX30009_IIO_REF_CLK_SEL),
	MAX30009_ATTR("clk_freq_sel",			MAX30009_IIO_CLK_FREQ_SEL),
	MAX30009_ATTR("clk_fine_tune",			MAX30009_IIO_CLK_FINE_TUNE),
	/* Pin configuration */
	MAX30009_ATTR("int_fcfg",			MAX30009_IIO_INT_FCFG),
	MAX30009_ATTR("int_ocfg",			MAX30009_IIO_INT_OCFG),
	MAX30009_ATTR("trig_icfg",			MAX30009_IIO_TRIG_ICFG),
	MAX30009_ATTR("trig_ocfg",			MAX30009_IIO_TRIG_OCFG),
	MAX30009_ATTR("bg_enabled",			MAX30009_IIO_BIOZ_BG_ENABLED),
	/* BioZ TX */
	MAX30009_ATTR("drv_mode",			MAX30009_IIO_BIOZ_DRV_MODE),
	MAX30009_ATTR("vdrv_mag",			MAX30009_IIO_BIOZ_VDRV_MAG),
	MAX30009_ATTR("idrv_rge",			MAX30009_IIO_BIOZ_IDRV_RGE),
	MAX30009_ATTR("drv_reset",			MAX30009_IIO_BIOZ_DRV_RESET),
	MAX30009_ATTR("dac_reset",			MAX30009_IIO_BIOZ_DAC_RESET),
	/* BioZ RX */
	MAX30009_ATTR("gain",				MAX30009_IIO_BIOZ_GAIN),
	MAX30009_ATTR("dac_osr",			MAX30009_IIO_BIOZ_DAC_OSR),
	MAX30009_ATTR("adc_osr",			MAX30009_IIO_BIOZ_ADC_OSR),
	MAX30009_ATTR("ina_mode",			MAX30009_IIO_BIOZ_INA_MODE),
	MAX30009_ATTR("dm_dis",				MAX30009_IIO_BIOZ_DM_DIS),
	/* BioZ filters */
	MAX30009_ATTR("ahpf",				MAX30009_IIO_BIOZ_AHPF),
	MAX30009_ATTR("dhpf",				MAX30009_IIO_BIOZ_DHPF),
	MAX30009_ATTR("dlpf",				MAX30009_IIO_BIOZ_DLPF),
	MAX30009_ATTR("ext_cap",			MAX30009_IIO_BIOZ_EXT_CAP),
	MAX30009_ATTR("dc_restore",			MAX30009_IIO_BIOZ_DC_RESTORE),
	/* BioZ advanced */
	MAX30009_ATTR("amp_rge",			MAX30009_IIO_BIOZ_AMP_RGE),
	MAX30009_ATTR("amp_bw",				MAX30009_IIO_BIOZ_AMP_BW),
	MAX30009_ATTR("fast_manual",			MAX30009_IIO_BIOZ_FAST_MANUAL),
	MAX30009_ATTR("fast_start_en",			MAX30009_IIO_BIOZ_FAST_START_EN),
	MAX30009_ATTR("stbyon",				MAX30009_IIO_BIOZ_STBYON),
	MAX30009_ATTR("ina_chop_en",			MAX30009_IIO_BIOZ_INA_CHOP_EN),
	MAX30009_ATTR("ch_fsel",			MAX30009_IIO_BIOZ_CH_FSEL),
	MAX30009_ATTR("i_clk_phase",			MAX30009_IIO_BIOZ_I_CLK_PHASE),
	MAX30009_ATTR("q_clk_phase",			MAX30009_IIO_BIOZ_Q_CLK_PHASE),
	/* BioZ thresholds */
	MAX30009_ATTR("thresh_en",			MAX30009_IIO_BIOZ_THRESH_EN),
	MAX30009_ATTR("hi_thresh",			MAX30009_IIO_BIOZ_HI_THRESH),
	MAX30009_ATTR("lo_thresh",			MAX30009_IIO_BIOZ_LO_THRESH),
	MAX30009_ATTR("cmp_mode",			MAX30009_IIO_BIOZ_CMP_MODE),
	/* Calibration MUX */
	MAX30009_ATTR("bmux_drvp",			MAX30009_IIO_BMUX_DRVP),
	MAX30009_ATTR("bmux_drvn",			MAX30009_IIO_BMUX_DRVN),
	MAX30009_ATTR("bmux_bip",			MAX30009_IIO_BMUX_BIP),
	MAX30009_ATTR("bmux_bin",			MAX30009_IIO_BMUX_BIN),
	MAX30009_ATTR("bmux_mux_en",			MAX30009_IIO_BMUX_MUX_EN),
	MAX30009_ATTR("bmux_cal_en",			MAX30009_IIO_BMUX_CAL_EN),
	MAX30009_ATTR("bmux_rsel",			MAX30009_IIO_BMUX_RSEL),
	MAX30009_ATTR("bmux_gsr_rsel",			MAX30009_IIO_BMUX_GSR_RSEL),
	MAX30009_ATTR("bmux_connect_cal_only",		MAX30009_IIO_BMUX_CONNECT_CAL_ONLY),
	MAX30009_ATTR("bmux_bist_en",			MAX30009_IIO_BMUX_BIST_EN),
	MAX30009_ATTR("bmux_gsr_load_en",		MAX30009_IIO_BMUX_GSR_LOAD_EN),
	MAX30009_ATTR("bmux_ext_inload_en",		MAX30009_IIO_BMUX_EXT_INLOAD_EN),
	MAX30009_ATTR("bmux_int_inload_en",		MAX30009_IIO_BMUX_INT_INLOAD_EN),
	/* Lead detection */
	MAX30009_ATTR("lead_detect_lon_en",		MAX30009_IIO_LON_DETECT_EN),
	MAX30009_ATTR("lead_detect_loff_en",		MAX30009_IIO_LOFF_DETECT_EN),
	MAX30009_ATTR("lead_detect_loff_ext_en",	MAX30009_IIO_LOFF_EXT_EN),
	MAX30009_ATTR("lead_detect_drv_oor_en",		MAX30009_IIO_LOFF_DRV_OOR_EN),
	MAX30009_ATTR("lead_detect_ipol",		MAX30009_IIO_LOFF_IPOL),
	MAX30009_ATTR("lead_detect_imag",		MAX30009_IIO_LOFF_IMAG),
	MAX30009_ATTR("lead_detect_thresh",		MAX30009_IIO_LOFF_THRESH),
	/* Lead bias */
	MAX30009_ATTR("rbias_val",			MAX30009_IIO_RBIAS_VALUE),
	MAX30009_ATTR("rbias_bip_en",			MAX30009_IIO_RBIAS_BIP_EN),
	MAX30009_ATTR("rbias_bin_en",			MAX30009_IIO_RBIAS_BIN_EN),

	END_ATTRIBUTES_ARRAY
};

/*
 * Channel attribute: raw single-shot read only.
 * All BioZ configuration is in debug_attrs; config is applied at init time.
 */
static struct iio_attribute max30009_iio_bioz0_attrs[] = {
	MAX30009_ATTR("enable",				MAX30009_IIO_BIOZ_ENABLED),
	END_ATTRIBUTES_ARRAY
};

/*
 * BioZ produces a complex impedance phasor — one measurement, two components
 * (I = real, Q = imaginary). All hardware configuration registers are shared;
 * there is no per-component register. Expose as a single IIO_RESISTANCE channel
 * (bioz0) whose scan delivers both components: scan[0] = I, scan[1] = Q.
 * The index (0) allows for future devices with more than one BioZ frontend.
 */
static struct scan_type max30009_scan_type = {
	.sign = 's',
	.realbits = 20,
	.storagebits = 32,
	.shift = 0,
	.is_big_endian = true,
};

static struct iio_channel max30009_channels[] = {
	{
		.name = "bioz0",
		.ch_type = IIO_RESISTANCE,
		.channel = 0,
		.indexed = true,
		.ch_out = false,
		.scan_index = 0,
		.scan_type = &max30009_scan_type,
		.attributes = max30009_iio_bioz0_attrs,
	},
};

static struct iio_device max30009_iio_device = {
	.num_ch = NO_OS_ARRAY_SIZE(max30009_channels),
	.channels = max30009_channels,
	.attributes = max30009_iio_global_attrs,
	.debug_attributes = max30009_iio_debug_attrs,
	.pre_enable = max30009_iio_pre_enable,
	.post_disable = max30009_iio_post_disable,
	.submit = max30009_iio_submit,
	.debug_reg_read = max30009_iio_reg_read,
	.debug_reg_write = max30009_iio_reg_write,
};

/**
 * @brief Initialise the MAX30009 IIO wrapper.
 * @param iio_dev     Output: pointer to the allocated descriptor.
 * @param init_param  Initialisation parameters.
 * @return 0 on success, negative error code on failure.
 */
int max30009_iio_init(struct max30009_iio_dev **iio_dev,
		      struct max30009_iio_init_param *init_param)
{
	struct max30009_iio_dev *desc;
	int ret;

	if (!iio_dev || !init_param)
		return -EINVAL;

	desc = no_os_calloc(1, sizeof(*desc));
	if (!desc)
		return -ENOMEM;

	ret = max30009_init(&desc->max30009_dev, &init_param->max30009_init);
	if (ret)
		goto error_dev;

	ret = no_os_cb_init(&desc->fifo_buf,
			    (uint32_t)init_param->fifo_buf_size *
			    MAX30009_FIFO_DATA_SIZE);
	if (ret)
		goto error_cb;

	if (init_param->pll) {
		ret = max30009_set_pll_config(desc->max30009_dev,
					      init_param->pll);
		if (ret)
			goto error_cfg;

		ret = max30009_pll_enable(desc->max30009_dev, true);
		if (ret)
			goto error_cfg;
	}

	if (init_param->bioz) {
		ret = max30009_set_bioz_config(desc->max30009_dev,
					       init_param->bioz);
		if (ret)
			goto error_cfg;

		ret = max30009_bioz_enable(desc->max30009_dev, true);
		if (ret)
			goto error_cfg;
	}

	if (init_param->fifo_watermark) {
		ret = max30009_set_fifo_watermark(desc->max30009_dev,
						  init_param->fifo_watermark);
		if (ret)
			goto error_cfg;
	}

	ret = max30009_enable_interrupt(desc->max30009_dev,
					MAX30009_INT_A_FULL, true);
	if (ret)
		goto error_cfg;

	ret = max30009_enable_interrupt(desc->max30009_dev,
					MAX30009_INT_FIFO_DATA_RDY, true);
	if (ret)
		goto error_cfg;

	desc->iio_dev = &max30009_iio_device;
	desc->samples_available = 0;
	*iio_dev = desc;

	return 0;

error_cfg:
	no_os_cb_remove(desc->fifo_buf);
error_cb:
	max30009_remove(desc->max30009_dev);
error_dev:
	no_os_free(desc);
	return ret;
}

/**
 * @brief Release all resources allocated by max30009_iio_init().
 * @param iio_dev  Descriptor returned by max30009_iio_init().
 * @return 0 on success, negative error code on failure.
 */
int max30009_iio_remove(struct max30009_iio_dev *iio_dev)
{
	int ret;

	if (!iio_dev)
		return -EINVAL;

	no_os_cb_remove(iio_dev->fifo_buf);

	ret = max30009_remove(iio_dev->max30009_dev);
	if (ret)
		return ret;

	no_os_free(iio_dev);
	return 0;
}

/**
 * @brief GPIO interrupt handler for the INTB pin.
 * @param ctx  Pointer to the max30009_iio_dev descriptor.
 */
void max30009_intb_handler(void *ctx)
{
	struct max30009_iio_dev *iio_dev = (struct max30009_iio_dev *)ctx;
	struct max30009_dev *dev;
	uint8_t status1;
	uint16_t fifo_count;
	uint8_t *fifo_data;
	uint32_t sample;
	uint16_t i;
	int ret;

	if (!iio_dev || !iio_dev->max30009_dev)
		return;

	if (!iio_dev->buffer_active) {
		max30009_clear_status(iio_dev->max30009_dev);
		return;
	}

	dev = iio_dev->max30009_dev;

	ret = max30009_reg_read(dev, MAX30009_REG_STATUS1, &status1);
	if (ret)
		return;

	if (!(status1 & (MAX30009_STATUS1_A_FULL_MSK | MAX30009_STATUS1_FIFO_DATA_RDY_MSK)))
		return;

	ret = max30009_fifo_get_count(dev, &fifo_count);
	if (ret || fifo_count == 0) {
		max30009_clear_status(dev);
		return;
	}

	fifo_data = dev->fifo_read_buf;

	ret = max30009_read_fifo_data(dev, fifo_data,
				      fifo_count * MAX30009_FIFO_DATA_SIZE);
	if (ret) {
		max30009_clear_status(dev);
		return;
	}

	for (i = 0; i < fifo_count; i++) {
		sample = ((uint32_t)fifo_data[i * 3] << 16) |
			 ((uint32_t)fifo_data[i * 3 + 1] << 8) |
			  (uint32_t)fifo_data[i * 3 + 2];

		ret = no_os_cb_write(iio_dev->fifo_buf, &sample, 3);
		if (ret) {
			iio_dev->overrun_count += fifo_count - i;
			break;
		}

		iio_dev->samples_available++;
	}

	max30009_clear_status(dev);
}
