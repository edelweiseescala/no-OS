/***************************************************************************//**
 *   @file   parameters.c
 *   @brief  Definition of Maxim platform data used by admt4000 project.
 *   @author Kister Genesis Jimenez (kister.jimenez@analog.com)
 *******************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ******************************************************************************/

#include "parameters.h"

struct max_uart_init_param uart_extra_ip = {
	.flow = MAX_UART_FLOW_DIS,
};

/*
 * The Maxim GPIO driver only needs the I/O voltage domain; direction and
 * output type are selected through the generic no_os_gpio API. The three
 * descriptors below are kept separate so that the common initialization data
 * stays identical across platforms.
 */
struct max_gpio_init_param gpio_extra_ip = {
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};

struct max_gpio_init_param gpio_extra_pp_ip = {
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};

struct max_gpio_init_param gpio_in_extra_ip = {
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};

/* Used as the extra parameter of the GPIO interrupt controller (ACALC port) */
struct max_gpio_init_param gpio_acalc_extra_ip = {
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};

struct max_spi_init_param spi_extra_ip = {
	.num_slaves = 2,
	.polarity = SPI_SS_POL_LOW,
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};

#if (TMC)
struct max_spi_init_param spi_tmc_extra_ip = {
	.num_slaves = 2,
	.polarity = SPI_SS_POL_LOW,
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};
#endif

struct max_i2c_init_param i2c_extra_ip = {
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};

/* Arduino header I2C1 map-A pins: P2.17 (SDA), P2.18 (SCL) */
static const mxc_gpio_cfg_t arduino_i2c_pins = {
	.port = MXC_GPIO2,
	.mask = (MXC_GPIO_PIN_17 | MXC_GPIO_PIN_18),
	.func = MXC_GPIO_FUNC_ALT1,
	.pad = MXC_GPIO_PAD_NONE,
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};

void admt4000_i2c_pins_fixup(void)
{
	MXC_GPIO_Config(&arduino_i2c_pins);
}
