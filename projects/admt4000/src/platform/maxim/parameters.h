/***************************************************************************//**
 *   @file   parameters.h
 *   @brief  Definitions specific to Maxim platform used by admt4000 project.
 *   @author Kister Genesis Jimenez (kister.jimenez@analog.com)
 *******************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ******************************************************************************/

#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#include "maxim_irq.h"
#include "maxim_i2c.h"
#include "maxim_spi.h"
#include "maxim_gpio.h"
#include "maxim_uart.h"
#include "maxim_uart_stdio.h"
#include "no_os_gpio.h"
#include "no_os_uart.h"

/*
 * Pin mapping for the AD-APARD32690-SL (MAX32690) carrier with an
 * EVAL-ADMT4000ARD1Z / EVAL-ADMT4000ARD2Z Arduino shield plugged into the
 * Arduino headers.
 *
 * The Arduino-header-to-MAX32690 mapping below matches the upstream
 * AD-APARD32690-SL board definition:
 *
 *   A0 -> P3.0    D2  -> P2.13   D8  -> P1.31   SDA -> P2.17 (I2C1)
 *   A1 -> P3.1    D3  -> P2.15   D9  -> P1.30   SCL -> P2.18 (I2C1)
 *   A2 -> P3.2    D4  -> P0.8    D10 -> P1.23 (SPI1 SS0)
 *   A3 -> P3.3    D5  -> P0.7    D11 -> P1.29 (SPI1 MOSI)
 *                 D6  -> P1.24   D12 -> P1.28 (SPI1 MISO)
 *                 D7  -> P1.25   D13 -> P1.26 (SPI1 SCK)
 *                      (SPI1 SS2)     (SPI1 SS1 is D7/P1.25)
 *
 * All Arduino-header signals live in the 3.3 V VDDIOH domain.
 */

/* Console UART - routed to the on-board USB-to-UART bridge */
#define UART_DEVICE_ID			0
#define UART_BAUDRATE			230400
#define UART_EXTRA			&uart_extra_ip
#define UART_OPS			&max_uart_ops
#define UART_IRQ_ID			UART0_IRQn

/* SPI1 is the Arduino header SPI, SS0 (P1.23) is Arduino D10 / CS_MT */
#define SPI_DEVICE_ID			1
#define SPI_BAUDRATE			2000000
#define SPI_OPS				&max_spi_ops
#define SPI_CS_PORT			1
#define SPI_CS				0
#define SPI_EXTRA			&spi_extra_ip

#define GPIO_OPS			&max_gpio_ops
#define GPIO_EXTRA			&gpio_extra_ip
#define GPIO_PP_EXTRA			&gpio_extra_pp_ip
#define GPIO_IN_EXTRA			&gpio_in_extra_ip

/* Arduino D2 - ADMT4000 GPIO2 */
#define GPIO_GPIO2_PORT			2
#define GPIO_GPIO2			13

/* Arduino D3 - ADMT4000 GPIO0 / BUSY */
#define GPIO_BUSY_PORT			2
#define GPIO_BUSY			15

/* Arduino D8 - ADMT4000 GPIO4 / FAULT */
#define GPIO_FAULT_PORT			1
#define GPIO_FAULT			31

/* Arduino A0 - ADMT4000 GPIO5 / BOOTLOAD */
#define GPIO_BOOTLOAD_PORT		3
#define GPIO_BOOTLOAD			0

/* Arduino D5 - ADMT4000 GPIO1 / CNV */
#define GPIO_CNV_PORT			0
#define GPIO_CNV			7

/* Arduino D6 - ADMT4000 GPIO3 / ACALC (also the IIO trigger source) */
#define GPIO_ACALC_PORT			1
#define GPIO_ACALC			24

/* Arduino D9 - TMC5240 DIAG1 */
#define GPIO_DIAG1_PORT			1
#define GPIO_DAIG1			30

/* Arduino A1 - Coil driver shutdown */
#define GPIO_SHDN_N_PORT		3
#define GPIO_SHDN_N			1

/* Arduino A2 - ADMT4000 reset */
#define GPIO_RSTB_PORT			3
#define GPIO_RSTB			2

/* Arduino D4 - Coil drive for magnetic reset */
#define GPIO_COIL_RS_PORT		0
#define GPIO_COIL_RS			8

/* Arduino A3 - ADMT4000 supply enable */
#define GPIO_V_EN_PORT			3
#define GPIO_V_EN			3

/*
 * The IIO trigger is driven by the ACALC pin. On Maxim the GPIO interrupt
 * controller is instantiated per port, and the trigger "irq id" is the pin
 * number within that port.
 */
#define GPIO_IRQ_ID			GPIO_ACALC_PORT
#define GPIO_IRQ_OPS			&max_gpio_irq_ops
#define GPIO_IRQ_EXTRA			&gpio_acalc_extra_ip
#define GPIO_IRQ_INTR_PRIORITY		GPIO1_IRQn

#define ADMT4000_GPIO_CB_HANDLE		MXC_GPIO_GET_GPIO(GPIO_ACALC_PORT)
#define ADMT4000_GPIO_TRIG_IRQ_ID	GPIO_ACALC

/* I2C1 is the Arduino header I2C (P2.17/P2.18), used by the shield EEPROM */
#define I2C_DEVICE_ID			1
#define I2C_TIMING			0 /* (Unused on Maxim) */
#define I2C_OPS				&max_i2c_ops
#define I2C_EXTRA			&i2c_extra_ip

#define HW_CARRIER_NAME			"AD-APARD32690-SL"
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION			"1.0.0"
#endif

#define NVIC_GPIO_IRQ			GPIO1_IRQn

#if (TMC)
/* Arduino D7 - TMC5240 chip select, which is SPI1 SS1 (P1.25) */
#define TMC_CS_PORT			1
#define TMC_CS_PIN			1
#define SPI_TMC_EXTRA			&spi_tmc_extra_ip
#endif

extern struct max_uart_init_param uart_extra_ip;
extern struct max_spi_init_param spi_extra_ip;
extern struct max_spi_init_param spi_tmc_extra_ip;
extern struct max_i2c_init_param i2c_extra_ip;
extern struct max_gpio_init_param gpio_extra_ip;
extern struct max_gpio_init_param gpio_extra_pp_ip;
extern struct max_gpio_init_param gpio_in_extra_ip;
extern struct max_gpio_init_param gpio_acalc_extra_ip;

void admt4000_i2c_pins_fixup(void);

#endif /* __PARAMETERS_H__ */
