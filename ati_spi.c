/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2018 Luc Verhaegen <libv@skynet.be>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * This adds support for the ATI/AMD Radeon SPI interfaces.
 */

#include "programmer.h"
#include "spi.h"

struct ati_spi_pci_private {
	int io_bar;
};

static const struct ati_spi_pci_private r600_spi_pci_private = {
	.io_bar = 2,
};

const struct flashrom_pci_match ati_spi_pci_devices[] = {
	{0x1002, 0x958d, NT, &r600_spi_pci_private},
	{},
};

int
ati_spi_init(void)
{
	struct flashrom_pci_device *device;

	device = flashrom_pci_init(ati_spi_pci_devices);
	if (!device)
		return 1;

	return 0;
}
