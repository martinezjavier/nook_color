/*
 * Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) SPI touchscreen driver.
 * For use with Cypress Txx3xx parts.
 * Supported parts include:
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009, 2010, 2011 Cypress Semiconductor, Inc.
 * Copyright (C) 2011 Javier Martinez Canillas <martinez.javier@gmail.com>
 *
 * Multi-touch protocol type B support and cleanups by Javier Martinez Canillas
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */

#include "cyttsp_core.h"

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/spi/spi.h>

#define CY_SPI_WR_OP      0x00 /* r/~w */
#define CY_SPI_RD_OP      0x01
#define CY_SPI_CMD_BYTES  4
#define CY_SPI_SYNC_BYTE  2
#define CY_SPI_SYNC_ACK1  0x62 /* from protocol v.2 */
#define CY_SPI_SYNC_ACK2  0x9D /* from protocol v.2 */
#define CY_SPI_DATA_SIZE  128
#define CY_SPI_DATA_BUF_SIZE (CY_SPI_CMD_BYTES + CY_SPI_DATA_SIZE)
#define CY_SPI_BITS_PER_WORD 8

struct cyttsp_spi {
	struct spi_device *spi;
	void *ttsp_client;
	u8 wr_buf[CY_SPI_DATA_BUF_SIZE];
	u8 rd_buf[CY_SPI_DATA_BUF_SIZE];
};

static int cyttsp_spi_xfer(u8 op, struct cyttsp_spi *ts,
			   u8 reg, u8 *buf, int length)
{
	struct spi_message msg;
	struct spi_transfer xfer[2];
	u8 *wr_buf = ts->wr_buf;
	u8 *rd_buf = ts->rd_buf;
	int retval;

	if (length > CY_SPI_DATA_SIZE) {
		dev_dbg(&ts->spi->dev,
			"%s: length %d is too big.\n",
			__func__, length);
		return -EINVAL;
	}

	memset(wr_buf, 0, CY_SPI_DATA_BUF_SIZE);
	memset(rd_buf, 0, CY_SPI_DATA_BUF_SIZE);

	wr_buf[0] = 0x00; /* header byte 0 */
	wr_buf[1] = 0xFF; /* header byte 1 */
	wr_buf[2] = reg;  /* reg index */
	wr_buf[3] = op;   /* r/~w */
	if (op == CY_SPI_WR_OP)
		memcpy(wr_buf + CY_SPI_CMD_BYTES, buf, length);

	memset((void *)xfer, 0, sizeof(xfer));
	spi_message_init(&msg);

	/*
	  We set both TX and RX buffers because Cypress TTSP
	  requires full duplex operation.
	*/
	xfer[0].tx_buf = wr_buf;
	xfer[0].rx_buf = rd_buf;
	if (op == CY_SPI_WR_OP) {
		xfer[0].len = length + CY_SPI_CMD_BYTES;
		spi_message_add_tail(&xfer[0], &msg);
	} else if (op == CY_SPI_RD_OP) {
		xfer[0].len = CY_SPI_CMD_BYTES;
		spi_message_add_tail(&xfer[0], &msg);

		xfer[1].rx_buf = buf;
		xfer[1].len = length;
		spi_message_add_tail(&xfer[1], &msg);
	}

	retval = spi_sync(ts->spi, &msg);
	if (retval < 0) {
		dev_dbg(&ts->spi->dev,
			"%s: spi_sync() error %d, len=%d, op=%d\n",
			__func__, retval, xfer[1].len, op);

		/*
		 * do not return here since was a bad ACK sequence
		 * let the following ACK check handle any errors and
		 * allow silent retries
		 */
	}

	if ((rd_buf[CY_SPI_SYNC_BYTE] == CY_SPI_SYNC_ACK1) &&
	    (rd_buf[CY_SPI_SYNC_BYTE+1] == CY_SPI_SYNC_ACK2))
		retval = 0;
	else {
		int i;
		for (i = 0; i < (CY_SPI_CMD_BYTES); i++)
			dev_dbg(&ts->sp->dev,
				"%s: test rd_buf[%d]:0x%02x\n",
				__func__, i, rd_buf[i]);
		for (i = 0; i < (length); i++)
			dev_dbg(&ts->spi->dev,
				"%s: test buf[%d]:0x%02x\n",
				__func__, i, buf[i]);

		/* signal ACK error so silent retry */
		retval = 1;
	}

	return retval;
}

static int ttsp_spi_read_block_data(struct device *dev,
				    u8 addr, u8 length, void *data)
{
	struct spi_device *spi = to_spi_device(dev);
	struct cyttsp_spi *ts = spi_get_drvdata(spi);
	int retval;

	retval = cyttsp_spi_xfer(CY_SPI_RD_OP, ts, addr, data, length);
	if (retval < 0)
		pr_err("%s: ttsp_spi_read_block_data failed\n",
		       __func__);

	/*
	 * Do not print the above error if the data sync bytes were not found.
	 * This is a normal condition for the bootloader loader startup and need
	 * to retry until data sync bytes are found.
	 */
	if (retval > 0)
		retval = -EIO;  /* now signal fail; so retry can be done */

	return retval;
}

static int ttsp_spi_write_block_data(struct device *dev,
				     u8 addr, u8 length, const void *data)
{
	struct spi_device *spi = to_spi_device(dev);
	struct cyttsp_spi *ts = spi_get_drvdata(spi);
	int retval;

	retval = cyttsp_spi_xfer(CY_SPI_WR_OP, ts, addr, (void *)data, length);
	if (retval < 0)
		pr_err("%s: ttsp_spi_write_block_data failed\n",
		       __func__);

	/*
	 * Do not print the above error if the data sync bytes were not found.
	 * This is a normal condition for the bootloader loader startup and need
	 * to retry until data sync bytes are found.
	 */
	if (retval > 0)
		retval = -EIO;  /* now signal fail; so retry can be done */

	return retval;
}

static const struct cyttsp_bus_ops cyttsp_spi_bus_ops = {
	.bustype        = BUS_SPI,
	.write          = ttsp_spi_write_block_data,
	.read           = ttsp_spi_read_block_data,
};

static int __devinit cyttsp_spi_probe(struct spi_device *spi)
{
	struct cyttsp_spi *ts;
	int retval;

	/* Set up SPI*/
	spi->bits_per_word = CY_SPI_BITS_PER_WORD;
	spi->mode = SPI_MODE_0;
	retval = spi_setup(spi);
	if (retval < 0) {
		dev_dbg(&spi->dev, "%s: SPI setup error %d\n",
			__func__, retval);
		return retval;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		dev_dbg(&spi->dev, "%s: Error, kzalloc\n", __func__);
		return -ENOMEM;
	}

	ts->spi = spi;
	spi_set_drvdata(spi, ts);

	ts->ttsp_client = cyttsp_core_init(&cyttsp_spi_bus_ops, &spi->dev,
					   spi->irq);
	if (IS_ERR(ts->ttsp_client)) {
		int retval = PTR_ERR(ts->ttsp_client);
		kfree(ts);
		return retval;
	}

	dev_dbg(&ts->spi->dev, "%s: Registration complete\n", __func__);

	return 0;
}

static int __devexit cyttsp_spi_remove(struct spi_device *spi)
{
	struct cyttsp_spi *ts = dev_get_drvdata(&spi);

	cyttsp_core_release(ts->ttsp_client);
	kfree(ts);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cyttsp_spi_suspend(struct device *dev)
{
	struct cyttsp_spi *ts = dev_get_drvdata(dev);

	return cyttsp_suspend(ts->ttsp_client);
}

static int cyttsp_spi_resume(struct device *dev)
{
	struct cyttsp_spi *ts = dev_get_drvdata(dev);

	return cyttsp_resume(ts->ttsp_client);
}
#endif

static SIMPLE_DEV_PM_OPS(cyttsp_spi_pm, cyttsp_spi_suspend, cyttsp_spi_resume);

static struct spi_driver cyttsp_spi_driver = {
	.driver = {
		.name = CY_SPI_NAME,
		.owner = THIS_MODULE,
		.pm = &cyttsp_spi_pm,
	},
	.probe = cyttsp_spi_probe,
	.remove = __devexit_p(cyttsp_spi_remove),
};

static int __init cyttsp_spi_init(void)
{
	return spi_register_driver(&cyttsp_spi_driver);
}
module_init(cyttsp_spi_init);

static void __exit cyttsp_spi_exit(void)
{
	spi_unregister_driver(&cyttsp_spi_driver);
}
module_exit(cyttsp_spi_exit);

MODULE_ALIAS("spi:cyttsp");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) SPI driver");
MODULE_AUTHOR("Cypress");

