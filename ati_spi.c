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

#include <stdlib.h>

#include "programmer.h"
#include "spi.h"

/* improve readability */
#define mmio_read(reg) flashrom_pci_mmio_long_read(device, (reg))
#define mmio_read_byte(reg) flashrom_pci_mmio_byte_read(device, (reg))
#define mmio_write(reg, val) flashrom_pci_mmio_long_write(device, (reg), (val))
#define mmio_mask(reg, val, mask) flashrom_pci_mmio_long_mask(device, (reg), (val), (mask))

struct ati_spi_pci_private {
	int io_bar;

	int (*save) (struct flashrom_pci_device *device);
	int (*restore) (struct flashrom_pci_device *device);
	int (*enable) (struct flashrom_pci_device *device);

	struct spi_master *master;
};

#define R600_GENERAL_PWRMGT		0x0618

#define R600_LOWER_GPIO_ENABLE		0x0710
#define R600_CTXSW_VID_LOWER_GPIO_CNTL	0x0718
#define R600_HIGH_VID_LOWER_GPIO_CNTL	0x071c
#define R600_MEDIUM_VID_LOWER_GPIO_CNTL	0x0720
#define R600_LOW_VID_LOWER_GPIO_CNTL	0x0724

#define R600_ROM_CNTL			0x1600
#define R600_PAGE_MIRROR_CNTL		0x1604
#define R600_ROM_SW_CNTL		0x1618
#define R600_ROM_SW_STATUS		0x161C
#define R600_ROM_SW_COMMAND		0x1620
#define R600_ROM_SW_DATA_0x00		0x1624
/* ... */
#define R600_ROM_SW_DATA_0xFC		0x1720
#define R600_ROM_SW_DATA(off)		(R600_ROM_SW_DATA_0x00 + (off))

#define R600_GPIOPAD_MASK		0x1798
#define R600_GPIOPAD_A			0x179C
#define R600_GPIOPAD_EN			0x17A0

#define R600_ROM_SW_STATUS_LOOP_COUNT 1000

#define R600_SPI_TRANSFER_SIZE 0x100

struct r600_spi_data {
	uint32_t reg_general_pwrmgt;
	uint32_t reg_lower_gpio_enable;
	uint32_t reg_ctxsw_vid_lower_gpio_cntl;
	uint32_t reg_high_vid_lower_gpio_cntl;
	uint32_t reg_medium_vid_lower_gpio_cntl;
	uint32_t reg_low_vid_lower_gpio_cntl;

	uint32_t reg_rom_cntl;
	uint32_t reg_page_mirror_cntl;
	uint32_t reg_gpiopad_mask;
	uint32_t reg_gpiopad_a;
	uint32_t reg_gpiopad_en;
};

/*
 * Save for later restore.
 */
static int
r600_spi_save(struct flashrom_pci_device *device)
{
	struct r600_spi_data *data;

	msg_pdbg("%s();\n", __func__);

	if (device->private_data) {
		msg_perr("%s: device->private_data is already assigned.\n",
			 __func__);
		return -1;
	}

	data = calloc(1, sizeof(struct r600_spi_data));

	data->reg_general_pwrmgt = mmio_read(R600_GENERAL_PWRMGT);

	data->reg_lower_gpio_enable = mmio_read(R600_LOWER_GPIO_ENABLE);
	data->reg_ctxsw_vid_lower_gpio_cntl =
		mmio_read(R600_CTXSW_VID_LOWER_GPIO_CNTL);
	data->reg_high_vid_lower_gpio_cntl =
		mmio_read(R600_HIGH_VID_LOWER_GPIO_CNTL);
	data->reg_medium_vid_lower_gpio_cntl =
		mmio_read(R600_MEDIUM_VID_LOWER_GPIO_CNTL);
	data->reg_low_vid_lower_gpio_cntl =
		mmio_read(R600_LOW_VID_LOWER_GPIO_CNTL);

	data->reg_rom_cntl = mmio_read(R600_ROM_CNTL);
	data->reg_page_mirror_cntl = mmio_read(R600_PAGE_MIRROR_CNTL);

	data->reg_gpiopad_mask = mmio_read(R600_GPIOPAD_MASK);
	data->reg_gpiopad_a = mmio_read(R600_GPIOPAD_A);
	data->reg_gpiopad_en = mmio_read(R600_GPIOPAD_EN);

	device->private_data = data;

	return 0;
}

/*
 * Restore saved registers, in the order of enable writes.
 */
static int
r600_spi_restore(struct flashrom_pci_device *device)
{
	struct r600_spi_data *data = device->private_data;

	msg_pdbg("%s();\n", __func__);

	if (!data) {
		msg_perr("%s: device->private_data is not assigned.\n",
			 __func__);
		return -1;
	}

	mmio_write(R600_ROM_CNTL, data->reg_rom_cntl);

	mmio_write(R600_GPIOPAD_A, data->reg_gpiopad_a);
	mmio_write(R600_GPIOPAD_EN, data->reg_gpiopad_en);
	mmio_write(R600_GPIOPAD_MASK, data->reg_gpiopad_mask);

	mmio_write(R600_GENERAL_PWRMGT, data->reg_general_pwrmgt);

	mmio_write(R600_CTXSW_VID_LOWER_GPIO_CNTL,
		   data->reg_ctxsw_vid_lower_gpio_cntl);
	mmio_write(R600_HIGH_VID_LOWER_GPIO_CNTL,
		   data->reg_high_vid_lower_gpio_cntl);
	mmio_write(R600_MEDIUM_VID_LOWER_GPIO_CNTL,
		   data->reg_medium_vid_lower_gpio_cntl);
	mmio_write(R600_LOW_VID_LOWER_GPIO_CNTL,
		   data->reg_low_vid_lower_gpio_cntl);

	mmio_write(R600_LOWER_GPIO_ENABLE, data->reg_lower_gpio_enable);

	mmio_write(R600_PAGE_MIRROR_CNTL, data->reg_page_mirror_cntl);

	free(data);
	device->private_data = NULL;

	return 0;
}

/*
 * Enable SPI Access.
 */
static int
r600_spi_enable(struct flashrom_pci_device *device)
{
	int i;

	msg_pdbg("%s();\n", __func__);

	/* software enable clock gating and set sck divider to 1 */
	mmio_mask(R600_ROM_CNTL, 0x10000002, 0xF0000002);

	/* set gpio7,8,9 low */
	mmio_mask(R600_GPIOPAD_A, 0, 0x0700);
	/* gpio7 is input, gpio8/9 are output */
	mmio_mask(R600_GPIOPAD_EN, 0x0600, 0x0700);
	/* only allow software control on gpio7,8,9 */
	mmio_mask(R600_GPIOPAD_MASK, 0x0700, 0x0700);

	/* disable open drain pads */
	mmio_mask(R600_GENERAL_PWRMGT, 0, 0x0800);

	mmio_mask(R600_CTXSW_VID_LOWER_GPIO_CNTL, 0, 0x0400);
	mmio_mask(R600_HIGH_VID_LOWER_GPIO_CNTL, 0, 0x0400);
	mmio_mask(R600_MEDIUM_VID_LOWER_GPIO_CNTL, 0, 0x0400);
	mmio_mask(R600_LOW_VID_LOWER_GPIO_CNTL, 0, 0x0400);

	mmio_mask(R600_LOWER_GPIO_ENABLE, 0x0400, 0x0400);

	programmer_delay(1000);

	mmio_mask(R600_GPIOPAD_MASK, 0, 0x700);
	mmio_mask(R600_GPIOPAD_EN, 0, 0x700);
	mmio_mask(R600_GPIOPAD_A, 0, 0x00080000);

	/* page mirror usage */
	mmio_mask(R600_PAGE_MIRROR_CNTL, 0x04000000, 0x0C000000);

	if (mmio_read(R600_ROM_SW_STATUS)) {
		for (i = 0; i < R600_ROM_SW_STATUS_LOOP_COUNT; i++) {
			mmio_write(R600_ROM_SW_STATUS, 0);
			programmer_delay(1000);
			if (!mmio_read(R600_ROM_SW_STATUS))
				break;
		}

		if (i == R600_ROM_SW_STATUS_LOOP_COUNT) {
			msg_perr("%s: failed to clear R600_ROM_SW_STATUS\n",
				 __func__);
			return -1;
		}
	}

	return 0;
}

/*
 *
 */
static int
r600_spi_command(struct flashctx *flash,
		 unsigned int writecnt, unsigned int readcnt,
		 const unsigned char *writearr, unsigned char *readarr)
{
	const struct spi_master spi_master = flash->mst->spi;
	struct flashrom_pci_device *device =
		(struct flashrom_pci_device *) spi_master.data;
	uint32_t command, control;
	int i, command_size;

	msg_pdbg("%s(%p(%p), %d, %d, %p (0x%02X), %p);\n", __func__, flash,
		 device, writecnt, readcnt, writearr, writearr[0], readarr);

	if (!device) {
		msg_perr("%s: no device specified!\n", __func__);
		return -1;
	}

	command = writearr[0];
	if (writecnt > 1)
		command |= writearr[1] << 24;
	if (writecnt > 2)
		command |= writearr[2] << 16;
	if (writecnt > 3)
		command |= writearr[3] << 8;

	if (writecnt < 4)
		command_size = writecnt;
	else
		command_size = 4;

	mmio_write(R600_ROM_SW_COMMAND, command);

	/*
	 * For some reason, we have an endianness difference between reading
	 * and writing. Also, ati hw only does 32bit register write accesses.
	 * If you write 8bits, the upper bytes will be nulled. Reading is fine.
	 * Furthermore, due to flashrom infrastructure, we need to skip the
	 * command in the writearr.
	 */
	for (i = 4; i < writecnt; i += 4) {
		uint32_t value = 0;
		int remainder = writecnt - i;

		if (remainder > 4)
			remainder = 4;

		if (remainder > 0)
			value |= writearr[i + 0] << 24;
		if (remainder > 1)
			value |= writearr[i + 1] << 16;
		if (remainder > 2)
			value |= writearr[i + 2] << 8;
		if (remainder > 3)
			value |= writearr[i + 3] << 0;

		mmio_write(R600_ROM_SW_DATA(i - 4), value);
	}

	control = (command_size - 1) << 0x10;
	if (readcnt)
		control |= 0x40000 | readcnt;
	else if (writecnt > 4)
		control |= writecnt - 4;
	mmio_write(R600_ROM_SW_CNTL, control);

	for (i = 0; i < R600_ROM_SW_STATUS_LOOP_COUNT; i++) {
		if (mmio_read(R600_ROM_SW_STATUS))
			break;
		programmer_delay(1000);
	}

	if (i == R600_ROM_SW_STATUS_LOOP_COUNT) {
		msg_perr("%s: still waiting for R600_ROM_SW_STATUS\n",
			 __func__);
		return -1;
	}
	mmio_write(R600_ROM_SW_STATUS, 0);

	for (i = 0; i < readcnt; i++)
		readarr[i] = mmio_read_byte(R600_ROM_SW_DATA(i));

	return 0;
}

static struct spi_master r600_spi_master = {
	.type = SPI_CONTROLLER_ATI,
	.features = 0,
	.max_data_read = R600_SPI_TRANSFER_SIZE,
	.max_data_write = R600_SPI_TRANSFER_SIZE + 1,
	.command = r600_spi_command,
	.multicommand = default_spi_send_multicommand,
	.read = default_spi_read,
	.write_256 = default_spi_write_256,
	.write_aai = default_spi_write_aai,
	.data = NULL, /* make this our flashrom_pci_device... */
};

/*
 * Used by all Rx6xx and RV710/RV770/RV710. RV730/RV740 use a slightly
 * different enable.
 */
static const struct ati_spi_pci_private r600_spi_pci_private = {
	.io_bar = 2,
	.save = r600_spi_save,
	.restore = r600_spi_restore,
	.enable = r600_spi_enable,
	.master = &r600_spi_master,
};

const struct flashrom_pci_match ati_spi_pci_devices[] = {
	{0x1002, 0x9400, NT, &r600_spi_pci_private},
	{0x1002, 0x9401, NT, &r600_spi_pci_private},
	{0x1002, 0x9402, NT, &r600_spi_pci_private},
	{0x1002, 0x9403, NT, &r600_spi_pci_private},
	{0x1002, 0x9405, NT, &r600_spi_pci_private},
	{0x1002, 0x940A, NT, &r600_spi_pci_private},
	{0x1002, 0x940B, NT, &r600_spi_pci_private},
	{0x1002, 0x940F, NT, &r600_spi_pci_private},
	{0x1002, 0x9440, NT, &r600_spi_pci_private},
	{0x1002, 0x9441, NT, &r600_spi_pci_private},
	{0x1002, 0x9442, NT, &r600_spi_pci_private},
	{0x1002, 0x9443, NT, &r600_spi_pci_private},
	{0x1002, 0x9444, NT, &r600_spi_pci_private},
	{0x1002, 0x9446, NT, &r600_spi_pci_private},
	{0x1002, 0x944A, NT, &r600_spi_pci_private},
	{0x1002, 0x944B, NT, &r600_spi_pci_private},
	{0x1002, 0x944C, NT, &r600_spi_pci_private},
	{0x1002, 0x944e, NT, &r600_spi_pci_private},
	{0x1002, 0x9450, NT, &r600_spi_pci_private},
	{0x1002, 0x9452, NT, &r600_spi_pci_private},
	{0x1002, 0x9456, NT, &r600_spi_pci_private},
	{0x1002, 0x945A, NT, &r600_spi_pci_private},
	{0x1002, 0x9460, NT, &r600_spi_pci_private},
	{0x1002, 0x9462, NT, &r600_spi_pci_private},
	{0x1002, 0x946A, NT, &r600_spi_pci_private},
	{0x1002, 0x94C1, NT, &r600_spi_pci_private},
	{0x1002, 0x94C3, OK, &r600_spi_pci_private},
	{0x1002, 0x94C4, NT, &r600_spi_pci_private},
	{0x1002, 0x94C5, NT, &r600_spi_pci_private},
	{0x1002, 0x94C6, NT, &r600_spi_pci_private},
	{0x1002, 0x94C7, NT, &r600_spi_pci_private},
	{0x1002, 0x94C8, NT, &r600_spi_pci_private},
	{0x1002, 0x94C9, NT, &r600_spi_pci_private},
	{0x1002, 0x94CB, NT, &r600_spi_pci_private},
	{0x1002, 0x94CC, NT, &r600_spi_pci_private},
	{0x1002, 0x9500, NT, &r600_spi_pci_private},
	{0x1002, 0x9501, NT, &r600_spi_pci_private},
	{0x1002, 0x9504, NT, &r600_spi_pci_private},
	{0x1002, 0x9505, NT, &r600_spi_pci_private},
	{0x1002, 0x9506, NT, &r600_spi_pci_private},
	{0x1002, 0x9507, NT, &r600_spi_pci_private},
	{0x1002, 0x9508, NT, &r600_spi_pci_private},
	{0x1002, 0x9509, NT, &r600_spi_pci_private},
	{0x1002, 0x950F, NT, &r600_spi_pci_private},
	{0x1002, 0x9511, OK, &r600_spi_pci_private},
	{0x1002, 0x9513, NT, &r600_spi_pci_private},
	{0x1002, 0x9515, NT, &r600_spi_pci_private},
	{0x1002, 0x9519, NT, &r600_spi_pci_private},
	{0x1002, 0x9540, NT, &r600_spi_pci_private},
	{0x1002, 0x954F, NT, &r600_spi_pci_private},
	{0x1002, 0x9552, NT, &r600_spi_pci_private},
	{0x1002, 0x9553, NT, &r600_spi_pci_private},
	{0x1002, 0x9555, NT, &r600_spi_pci_private},
	{0x1002, 0x9557, NT, &r600_spi_pci_private},
	{0x1002, 0x955F, NT, &r600_spi_pci_private},
	{0x1002, 0x9580, NT, &r600_spi_pci_private},
	{0x1002, 0x9581, NT, &r600_spi_pci_private},
	{0x1002, 0x9583, NT, &r600_spi_pci_private},
	{0x1002, 0x9586, NT, &r600_spi_pci_private},
	{0x1002, 0x9587, NT, &r600_spi_pci_private},
	{0x1002, 0x9588, NT, &r600_spi_pci_private},
	{0x1002, 0x9589, OK, &r600_spi_pci_private},
	{0x1002, 0x958A, NT, &r600_spi_pci_private},
	{0x1002, 0x958B, NT, &r600_spi_pci_private},
	{0x1002, 0x958C, NT, &r600_spi_pci_private},
	{0x1002, 0x958D, OK, &r600_spi_pci_private},
	{0x1002, 0x958E, NT, &r600_spi_pci_private},
	{0x1002, 0x9591, NT, &r600_spi_pci_private},
	{0x1002, 0x9593, NT, &r600_spi_pci_private},
	{0x1002, 0x9595, NT, &r600_spi_pci_private},
	{0x1002, 0x9596, NT, &r600_spi_pci_private},
	{0x1002, 0x9597, NT, &r600_spi_pci_private},
	{0x1002, 0x9598, NT, &r600_spi_pci_private},
	{0x1002, 0x9599, NT, &r600_spi_pci_private},
	{0x1002, 0x95C0, NT, &r600_spi_pci_private},
	{0x1002, 0x95C2, NT, &r600_spi_pci_private},
	{0x1002, 0x95C4, NT, &r600_spi_pci_private},
	{0x1002, 0x95C5, NT, &r600_spi_pci_private},
	{0x1002, 0x95C6, NT, &r600_spi_pci_private},
	{0x1002, 0x95C9, NT, &r600_spi_pci_private},
	{0x1002, 0x95CC, NT, &r600_spi_pci_private},
	{0x1002, 0x95CD, NT, &r600_spi_pci_private},
	{0x1002, 0x95CF, NT, &r600_spi_pci_private},
	{},
};

/*
 *
 */
static int
ati_spi_shutdown(void *data)
{
	struct flashrom_pci_device *device = data;
	const struct ati_spi_pci_private *private = device->private;

	return private->restore(device);
}

int
ati_spi_init(void)
{
	struct flashrom_pci_device *device;
	const struct ati_spi_pci_private *private;
	int ret;

	device = flashrom_pci_init(ati_spi_pci_devices);
	if (!device)
		return 1;

	private = device->private;

	if (flashrom_pci_mmio_map(device, private->io_bar))
		return 1;

	register_shutdown(ati_spi_shutdown, device);

	ret = private->save(device);
	if (ret)
		return ret;

	ret = private->enable(device);
	if (ret)
		return ret;

	private->master->data = device;
	if (register_spi_master(private->master))
		return 1;

	return 0;
}
