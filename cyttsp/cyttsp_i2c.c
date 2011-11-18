/*
 * Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) I2C touchscreen driver.
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

#include <linux/i2c.h>
#include <linux/input.h>

#define CY_I2C_DATA_SIZE  128

static int cyttsp_i2c_read_block_data(struct device *dev,
				      u8 addr, u8 length, void *values)
{
	struct i2c_client *client = to_i2c_client(dev);
	int retval;

	retval = i2c_master_send(client, &addr, 1);
	if (retval < 0)
		return retval;

	retval = i2c_master_recv(client, values, length);

	if (retval > 0 && retval != length)
		return -EIO;

	return (retval < 0) ? retval : 0;
}

static int cyttsp_i2c_write_block_data(struct device *dev,
				       u8 addr, u8 length, const void *values)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cyttsp *ts = i2c_get_clientdata(client);
	int retval;


	ts->xfer_buf[0] = addr;
	memcpy(&ts->xfer_buf[1], values, length);
	retval = i2c_master_send(client, ts->xfer_buf, length + 1);

	if (retval != length)
		return -EIO;

	return 0;
}

static const struct cyttsp_bus_ops cyttsp_i2c_bus_ops = {
	.bustype        = BUS_I2C,
	.write          = cyttsp_i2c_write_block_data,
	.read           = cyttsp_i2c_read_block_data,
};

static int __devinit cyttsp_i2c_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	struct cyttsp *ts;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EIO;

	ts = cyttsp_probe(&cyttsp_i2c_bus_ops, &client->dev, client->irq,
			  CY_I2C_DATA_SIZE);
	if (IS_ERR(ts))
		return PTR_ERR(ts);

	i2c_set_clientdata(client, ts);

	return 0;
}

static int __devexit cyttsp_i2c_remove(struct i2c_client *client)
{
	struct cyttsp *ts = i2c_get_clientdata(client);

	cyttsp_remove(ts);

	return 0;
}

static const struct i2c_device_id cyttsp_i2c_id[] = {
	{ CY_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cyttsp_i2c_id);

static struct i2c_driver cyttsp_i2c_driver = {
	.driver = {
		.name   = CY_I2C_NAME,
		.owner  = THIS_MODULE,
		.pm     = &cyttsp_pm_ops,
	},
	.probe          = cyttsp_i2c_probe,
	.remove         = __devexit_p(cyttsp_i2c_remove),
	.id_table       = cyttsp_i2c_id,
};

static int __init cyttsp_i2c_init(void)
{
	return i2c_add_driver(&cyttsp_i2c_driver);
}
module_init(cyttsp_i2c_init);

static void __exit cyttsp_i2c_exit(void)
{
	return i2c_del_driver(&cyttsp_i2c_driver);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) I2C driver");
MODULE_AUTHOR("Cypress");
