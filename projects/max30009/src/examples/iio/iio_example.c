/***************************************************************************//**
 *   @file   iio_example.c
 *   @brief  IIO streaming example for the MAX30009 BioZ AFE.
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

#include "iio_max30009.h"
#include "iio_app.h"
#include "common_data.h"
#include "no_os_irq.h"
#include "parameters.h"

/*
 * Capture buffer for IIO streaming.
 *
 * Layout per scan (see max30009_channels[] in iio_max30009.c):
 *   slot 0 — bioz0 I component : 32-bit signed (20-bit data, sign-extended)
 *   slot 1 — bioz0 Q component : 32-bit signed (20-bit data, sign-extended)
 *
 * DATA_BUFFER_SIZE controls how many complete I+Q scans can be buffered
 * before libiio drains them over UART to the host.
 */
#ifndef DATA_BUFFER_SIZE
#define DATA_BUFFER_SIZE 256
#endif

#define FW_VERSION "1.0.0"

static uint8_t iio_data_buffer[DATA_BUFFER_SIZE * 2 * sizeof(uint32_t)];

static struct iio_ctx_attr context_attributes[] = {
	{.name = "hw_mezzanine", .value = MAX30009_BOARD_NAME },
	{.name = "hw_carrier", .value = MAX30009_HW_CARRIER },
	{.name = "hw_name", .value = MAX30009_ACTIVE_DEVICE_NAME },
	{.name = "hw_vendor", .value = MAX30009_ACTIVE_DEVICE_VENDOR },
	{.name = "fw_version", .value = FW_VERSION },
};

int example_main(void)
{
	struct max30009_iio_dev *max30009_iio_desc;
	struct max30009_iio_init_param max30009_iio_ip;
	struct iio_app_desc *app;
	struct iio_app_init_param app_init_param = { 0 };
	struct no_os_irq_ctrl_desc *irq_desc;
	int ret;

	struct iio_data_buffer bioz_buff = {
		.buff = (void *)iio_data_buffer,
		.size = sizeof(iio_data_buffer),
	};

	ret = no_os_irq_ctrl_init(&irq_desc, &max30009_gpio_irq_ip);
	if (ret)
		return ret;

	struct max30009_pll_config pll_cfg = {
		.mdiv		= 511,
		.ndiv		= MAX30009_NDIV_1024,
		.use_external_clk = false,
		.use_32768_hz	= true,
	};

	struct max30009_bioz_config bioz_cfg = {
		.adc_osr	= MAX30009_ADC_OSR_1024,
		.dac_osr	= MAX30009_DAC_OSR_256,
		.gain		= MAX30009_BIOZ_GAIN_10,
		.ahpf		= MAX30009_AHPF_500HZ,
		.dhpf		= MAX30009_DHPF_BYPASS,
		.dlpf		= MAX30009_DLPF_BYPASS,
	};

	max30009_iio_ip.max30009_init	= max30009_init_params;
	max30009_iio_ip.fifo_buf_size	= DATA_BUFFER_SIZE * 2;
	max30009_iio_ip.pll		= &pll_cfg;
	max30009_iio_ip.bioz		= &bioz_cfg;
	max30009_iio_ip.fifo_watermark	= 1;

	ret = max30009_iio_init(&max30009_iio_desc, &max30009_iio_ip);
	if (ret)
		goto error_irq;

	struct no_os_callback_desc intb_cb = {
		.callback = max30009_intb_handler,
		.ctx = max30009_iio_desc,
		.event = NO_OS_EVT_GPIO,
		.peripheral = NO_OS_GPIO_IRQ,
		.handle = MAX30009_GPIO_CB_HANDLE,
	};

	ret = no_os_irq_register_callback(irq_desc, MAX30009_GPIO_TRIG_IRQ_ID,
					  &intb_cb);
	if (ret)
		goto error_iio;

	ret = no_os_irq_trigger_level_set(irq_desc, MAX30009_GPIO_TRIG_IRQ_ID,
					  NO_OS_IRQ_EDGE_FALLING);
	if (ret)
		goto error_iio;

	ret = no_os_irq_set_priority(irq_desc, MAX30009_GPIO_TRIG_IRQ_ID, 1);
	if (ret)
		goto error_iio;

	ret = no_os_irq_enable(irq_desc, MAX30009_GPIO_TRIG_IRQ_ID);
	if (ret)
		goto error_iio;

	struct iio_app_device iio_devices[] = {
		{
			.name = "max30009",
			.dev = max30009_iio_desc,
			.dev_descriptor = max30009_iio_desc->iio_dev,
			.read_buff = &bioz_buff,
		}
	};

	app_init_param.devices = iio_devices;
	app_init_param.nb_devices = NO_OS_ARRAY_SIZE(iio_devices);
	app_init_param.uart_init_params = max30009_uart_ip;
	app_init_param.ctx_attrs = context_attributes;
	app_init_param.nb_ctx_attr = NO_OS_ARRAY_SIZE(context_attributes);

	ret = iio_app_init(&app, app_init_param);
	if (ret)
		goto error_iio;

	return iio_app_run(app);

error_iio:
	max30009_iio_remove(max30009_iio_desc);
error_irq:
	no_os_irq_ctrl_remove(irq_desc);
	return ret;
}
