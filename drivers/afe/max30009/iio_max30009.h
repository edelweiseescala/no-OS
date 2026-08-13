/***************************************************************************//**
 *   @file   iio_max30009.h
 *   @brief  Header file for MAX30009 IIO interface
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

#ifndef _IIO_MAX30009_H_
#define _IIO_MAX30009_H_

#include <stdbool.h>
#include "iio.h"
#include "max30009.h"

#define MAX30009_ATTR(_name, _priv) {			\
	.name = _name,					\
	.show = max30009_iio_get_attr,			\
	.store = max30009_iio_set_attr,			\
	.priv = _priv,					\
}

#define MAX30009_ATTR_RO(_name, _priv) {		\
	.name = _name,					\
	.show = max30009_iio_get_attr,			\
	.store = NULL,					\
	.priv = _priv,					\
}

#define MAX30009_ATTR_SHARED(_name, _priv) {		\
	.name = _name,					\
	.shared = IIO_SHARED_BY_TYPE,			\
	.show = max30009_iio_get_attr,			\
	.store = max30009_iio_set_attr,			\
	.priv = _priv,					\
}

#define MAX30009_ATTR_SHARED_RO(_name, _priv) {		\
	.name = _name,					\
	.shared = IIO_SHARED_BY_TYPE,			\
	.show = max30009_iio_get_attr,			\
	.store = NULL,					\
	.priv = _priv,					\
}

struct max30009_iio_dev {
	struct max30009_dev *max30009_dev;
	struct iio_device *iio_dev;
	struct no_os_circular_buffer *fifo_buf;
	volatile uint32_t samples_available;
	/* Samples dropped due to circular buffer overflow in intb_handler. */
	volatile uint32_t overrun_count;
	/* Set by pre_enable, cleared by post_disable; gates intb_handler writes. */
	volatile bool buffer_active;
	/* Channel mask from pre_enable; bit N set means channel N is active. */
	uint32_t active_mask;
};

struct max30009_iio_init_param {
	struct max30009_init_param	max30009_init;
	uint16_t			fifo_buf_size;
	struct max30009_pll_config	*pll;
	struct max30009_bioz_config	*bioz;
	uint8_t				fifo_watermark;
};

/** Initialise the MAX30009 IIO wrapper. */
int max30009_iio_init(struct max30009_iio_dev **iio_dev,
		      struct max30009_iio_init_param *init_param);

/** Release all resources allocated by max30009_iio_init(). */
int max30009_iio_remove(struct max30009_iio_dev *iio_dev);

/** GPIO interrupt handler — call from the platform IRQ callback. */
void max30009_intb_handler(void *ctx);

#endif /* _IIO_MAX30009_H_ */
