/*
 * Copyright 2017-2022 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#include <linux/version.h>
#include <linux/of.h>

#if KERNEL_VERSION(7, 0, 0) <= LINUX_VERSION_CODE
#include <linux/gpio/consumer.h>
#else
#include <linux/of_gpio.h>
#endif

#include "of.h"

#if KERNEL_VERSION(7, 0, 0) <= LINUX_VERSION_CODE
#define MORSE_OF_GPIO_PROP(_name) _name
static int morse_of_get_gpio(struct device_node *np, const char *prop, int index)
{
	struct gpio_desc *desc;
	int gpio;

	desc = fwnode_gpiod_get_index(of_fwnode_handle(np), prop, index, GPIOD_ASIS,
				      "morse-of");
	if (IS_ERR(desc))
		return PTR_ERR(desc);

	gpio = desc_to_gpio(desc);
	gpiod_put(desc);

	return gpio;
}
#else
#define MORSE_OF_GPIO_PROP(_name) _name "-gpios"
static int morse_of_get_gpio(struct device_node *np, const char *prop, int index)
{
	return of_get_named_gpio(np, prop, index);
}
#endif

int morse_of_probe(struct device *dev, struct morse_gpios *gpios,
		   const struct of_device_id *match_table)
{
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_id;
	int gpio_pin;

	if (np) {
		of_id = of_match_node(match_table, np);
		if (of_id) {
			dev_info(dev, "%s: Reading gpio pins configuration from device tree\n",
				 __func__);
			gpio_pin = morse_of_get_gpio(np, MORSE_OF_GPIO_PROP("power"), 0);
			gpios->wake = gpio_pin;

			gpio_pin = morse_of_get_gpio(np, MORSE_OF_GPIO_PROP("power"), 1);
			gpios->busy = gpio_pin;

			/* Don't error on no PS config - this might be valid, e.g. AP only device */
			if (!morse_hw_ps_gpios_are_supported(gpios))
				dev_err(dev, "%s: optional property power-gpios incomplete, powersave won't be supported\n",
					__func__);

			/* Don't error on missing reset-gpios, user may be using mmc-pwrseq */
			gpio_pin = morse_of_get_gpio(np, MORSE_OF_GPIO_PROP("reset"), 0);
			gpios->reset = gpio_pin;

			/* Don't error on spi-irq-gpios - this might not be a SPI device */
			gpio_pin = morse_of_get_gpio(np, MORSE_OF_GPIO_PROP("spi-irq"), 0);
			gpios->spi_irq = gpio_pin;
			return 0;
		}
		dev_err(dev, "%s: Couldn't match device table\n", __func__);
	} else {
		dev_err(dev, "%s: Device node not found\n", __func__);
	}
	dev_info(dev, "%s: No pin configs found\n", __func__);
	return -ENOENT;
}
