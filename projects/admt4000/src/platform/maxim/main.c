/***************************************************************************//**
 *   @file   main.c
 *   @brief  Main function for Maxim platform of admt4000 project.
 *   @author Kister Genesis Jimenez (kister.jimenez@analog.com)
 *******************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ******************************************************************************/

#include "parameters.h"
#include "common_data.h"
#include "no_os_error.h"
#include "no_os_print_log.h"
#include "no_os_uart.h"

extern int example_main();

int main()
{
	int ret;
	struct no_os_irq_ctrl_desc *nvic_desc;
	struct no_os_irq_init_param nvic_desc_param = {
		.platform_ops = &max_irq_ops,
	};

	ret = no_os_irq_ctrl_init(&nvic_desc, &nvic_desc_param);
	if (ret)
		return ret;

	ret = no_os_irq_enable(nvic_desc, NVIC_GPIO_IRQ);
	if (ret)
		goto irq_remove;

#ifdef IIO_SUPPORT
	ret = example_main();
	goto irq_remove;
#else
	struct no_os_uart_desc *uart_desc;

	/* Initialize UART interface */
	ret = no_os_uart_init(&uart_desc, &admt4000_uart_ip);
	if (ret)
		goto irq_remove;

	no_os_uart_stdio(uart_desc);

	ret = example_main();
	pr_info("Example main returned %d\n", ret);
	goto irq_remove;
#endif

irq_remove:
	no_os_irq_ctrl_remove(nvic_desc);

	return ret;
}
