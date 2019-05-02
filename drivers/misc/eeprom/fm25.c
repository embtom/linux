/*
 * fm25.c -- support SPI FRAMs, such as Cypress FM25 models
 * 
 * This file is part of the EMBTOM project
 * Copyright (c) 2018-2019 Thomas Willetal 
 * (https://github.com/tom3333)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>

#include <linux/nvmem-provider.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/property.h>
#include <linux/of.h>

/* FRAM SPI commands */
#define	FM25_WRSR		0x01		/* write status register */
#define	FM25_WRITE		0x02		/* write byte(s)/sector */
#define	FM25_READ		0x03		/* read byte(s) */
#define	FM25_WRDI		0x04		/* reset the write enable */
#define	FM25_RDSR		0x05		/* read status register */
#define	FM25_WREN		0x06		/* latch the write enable */

/* FRAM SR Bits */
#define	FM25_SR_WEN		0x02		/* write enable (latched) */
#define	FM25_SR_BP0		0x04		/* BP for software writeprotect */
#define	FM25_SR_BP1		0x08
#define	FM25_SR_WPEN	0x80		/* writeprotect enable */

#define FM25_MAXADDRLEN	3		/* 24 bit addresses */

#define	io_limit	PAGE_SIZE	/* bytes */

struct fm25_data {
	struct spi_device	    *spi;
	struct mutex		    lock;
	struct spi_eeprom	    chip;
	unsigned		        addrlen;
	struct nvmem_config	    nvmem_config;
	struct nvmem_device	    *nvmem;
};

static int fm25_data_read(void *priv, unsigned offset, void *val, size_t count)
{
	struct 	fm25_data *fm25 	= priv;
	char   *buf 				= val;
	u8	    command[FM25_MAXADDRLEN + 1];
	u8		instr;
	u8	   *cp;
	int		status;
	struct spi_transfer t[2];
	struct spi_message	m;

	if (unlikely(offset >= fm25->chip.byte_len))
		return -EINVAL;
	if ((offset + count) > fm25->chip.byte_len)
		count = fm25->chip.byte_len - offset;
	if (unlikely(!count))
		return -EINVAL;

	cp = command;
	instr = FM25_READ;
	*cp++ = instr;

	/* 8/16/24-bit address is written MSB first */
	switch (fm25->addrlen) {
	default:	/* case 3 */
		*cp++ = offset >> 16;
		/* fall through */
	case 2:
		*cp++ = offset >> 8;
		/* fall through */
	case 1:
	case 0:	/* can't happen: for better codegen */
		*cp++ = offset >> 0;
	}

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	t[0].tx_buf = command;
	t[0].len = fm25->addrlen + 1;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = count;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&fm25->lock);

	status = spi_sync(fm25->spi, &m);
	dev_dbg(&fm25->spi->dev, "read %zu bytes at %d --> %zd\n",count, offset, status);

	mutex_unlock(&fm25->lock);
	return status;
}

static int fm25_data_write(void *priv, unsigned int off, void *val, size_t count)
{
	struct fm25_data *fm25 = priv;
	const char *buf = val;
	int			status = 0;
	unsigned	buf_size;
	u8			*bounce;

	if (unlikely(off >= fm25->chip.byte_len))
		return -EFBIG;
	if ((off + count) > fm25->chip.byte_len)
		count = fm25->chip.byte_len - off;
	if (unlikely(!count))
		return -EINVAL;

	/* Temp buffer starts with command and address */
	if (fm25->chip.byte_len > io_limit) {
		buf_size = io_limit;
	}
	else {
		buf_size = fm25->chip.byte_len;
	}
	buf_size += fm25->addrlen +1; 
	bounce = kmalloc(buf_size, GFP_KERNEL);
	if (bounce == NULL) {
		return -ENOMEM;
	}
	
	mutex_lock(&fm25->lock);
	do {
		unsigned int segment;
		unsigned int offset = off;
		u8 *cp = bounce;
		
		*cp = FM25_WREN;
		status = spi_write(fm25->spi, bounce, 1);
		if (status < 0) {
			mutex_unlock(&fm25->lock);
			kfree(bounce);
			dev_dbg(&fm25->spi->dev, "WREN --> %d\n",(int) status);
			return status;
		}

		*cp++ = FM25_WRITE;
	
		/* 8/16/24-bit address is written MSB first */
		switch (fm25->addrlen) 
		{
			default:	/* case 3 */
	 			*cp++  = offset >> 16;
			case 2:
	 			*cp++ = offset >> 8;
	 		case 1:
	 		case 0:	/* can't happen: for better codegen */
	 			*cp++ = offset >> 0;
		}

		/* Write as much of a page as we can */
 		segment = buf_size - (off % buf_size);
		if (segment > count) {
			segment = count;
		}
	
		memcpy(cp, buf, segment);
		status = spi_write(fm25->spi, bounce, segment + fm25->addrlen + 1);
		dev_dbg(&fm25->spi->dev,"write %u bytes at %u --> %d\n", segment, off, (int) status);
		if (status < 0) {
			mutex_unlock(&fm25->lock);
			kfree(bounce);
			return status;
		}

		off += segment;
		buf += segment;
		count -= segment;
	} while(count > 0);

	mutex_unlock(&fm25->lock);

	kfree(bounce);
	return status;
}

/*-------------------------------------------------------------------------*/

static int fm25_np_to_chip(struct device *dev, struct spi_eeprom *chip)
{
	u32 val;
	int ret;

	strncpy(chip->name, dev->of_node->name, sizeof(chip->name));

	ret = device_property_read_u32(dev, "size", &val);
	if(ret < 0) {
		dev_err(dev, "Error: missing \"size\" property\n");
		return -ENODEV;
	}
	chip->byte_len = val;

	ret = device_property_read_u32(dev, "address-width", &val);
	if(ret < 0) {
		dev_err(dev, "Error: missing \"address-width\" property\n");
		return -ENODEV;
	}

	switch (val) 
	{
		case 8:
			chip->flags |= EE_ADDR1;
			break;
		case 16:
			chip->flags |= EE_ADDR2;
			break;
		case 24:
			chip->flags |= EE_ADDR3;
			break;
		default:
			dev_err(dev, "Error: bad \"address-width\" property: %u\n", val);
			return -ENODEV;
	}

	if (device_property_present(dev, "read-only")) {
		chip->flags |= EE_READONLY;
	}
	return 0;
}

static int fm25_probe(struct spi_device *spi)
{
	struct fm25_data	*fm25 = NULL;
	struct spi_eeprom	chip = {0};
	int	        		ret;
	int 				sr;
	int					addrlen;

	/* Get chip description, check if platform data is available otherwise check device tree*/
	if (spi->dev.platform_data == NULL) {
		ret = fm25_np_to_chip(&spi->dev, &chip);
		if (ret < 0) {
			return ret;
		}
	}
	else {
		chip = *(struct spi_eeprom *)spi->dev.platform_data;
	}

	/* For now we only support 8/16/24 bit addressing */
	if (chip.flags & EE_ADDR1)
		addrlen = 1;
	else if (chip.flags & EE_ADDR2)
		addrlen = 2;
	else if (chip.flags & EE_ADDR3)
		addrlen = 3;
	else {
		dev_dbg(&spi->dev, "unsupported address type\n");
		return -EINVAL;
	}

	/* Ping the chip ... the status register is pretty portable,
	 * unlike probing manufacturer IDs.  We do expect that system
	 * firmware didn't write it in the past few milliseconds!
	 */
	sr = spi_w8r8(spi, FM25_RDSR);
	if (sr < 0 ) {
		dev_dbg(&spi->dev, "rdsr --> %d (%02x)\n", sr, sr);
		return -ENXIO;
	}

	fm25 = devm_kzalloc(&spi->dev, sizeof(struct fm25_data), GFP_KERNEL);
	if (fm25 == NULL) {
	 	return -ENOMEM;	
	}

	mutex_init(&fm25->lock);
	fm25->chip = chip;
	fm25->spi = spi;
	spi_set_drvdata(spi, fm25);
	fm25->addrlen = addrlen;
	
	fm25->nvmem_config.name = dev_name(&spi->dev);
	fm25->nvmem_config.dev = &spi->dev;
	fm25->nvmem_config.read_only = chip.flags & EE_READONLY;
	fm25->nvmem_config.root_only = true;
	fm25->nvmem_config.owner = THIS_MODULE;
	fm25->nvmem_config.compat = true;
	fm25->nvmem_config.base_dev = &spi->dev;
	fm25->nvmem_config.reg_read = fm25_data_read;
	fm25->nvmem_config.reg_write = fm25_data_write;
	fm25->nvmem_config.priv = fm25;
	fm25->nvmem_config.stride = 4;
	fm25->nvmem_config.word_size = 1;
	fm25->nvmem_config.size = chip.byte_len;

	fm25->nvmem = devm_nvmem_register(&spi->dev, &fm25->nvmem_config);
	if (IS_ERR(fm25->nvmem)) {
		return PTR_ERR(fm25->nvmem);
	}

	dev_info(&spi->dev, "%d %s %s fram%s\n",
		(chip.byte_len < 1024) ? chip.byte_len : (chip.byte_len / 1024),
		(chip.byte_len < 1024) ? "Byte" : "KByte",
		fm25->chip.name,
		(chip.flags & EE_READONLY) ? " (readonly)" : "");
	return 0;
}

static int fm25_remove(struct spi_device *spi)
{
	struct fm25_data	*fm25;
	fm25 = spi_get_drvdata(spi);
	nvmem_unregister(fm25->nvmem);
	return 0;
}

/*-------------------------------------------------------------------------*/

static const struct of_device_id fm25_of_match[] = {
	{ .compatible = "fm25", },
	{ }
};
MODULE_DEVICE_TABLE(of, fm25_of_match);

static struct spi_driver fm25_driver = {
	.driver = {
		.name		= "fm25",
		.owner		= THIS_MODULE,
		.of_match_table = fm25_of_match,
	},
	.probe		= fm25_probe,
	.remove		= fm25_remove,
};

module_spi_driver(fm25_driver);

MODULE_DESCRIPTION("Driver for Cypress SPI FRAMs");
MODULE_AUTHOR("Thomas Willetal");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:fram");