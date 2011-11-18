/*
 * Core Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) touchscreen drivers.
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
#include <linux/input/mt.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

/* Bootloader number of command keys */
#define CY_NUM_BL_KEYS    8

/* helpers */
#define GET_NUM_TOUCHES(x)          ((x) & 0x0F)
#define IS_LARGE_AREA(x)            (((x) & 0x10) >> 4)
#define IS_BAD_PKT(x)               ((x) & 0x20)
#define IS_VALID_APP(x)             ((x) & 0x01)
#define IS_OPERATIONAL_ERR(x)       ((x) & 0x3F)
#define GET_HSTMODE(reg)            ((reg & 0x70) >> 4)
#define GET_BOOTLOADERMODE(reg)     ((reg & 0x10) >> 4)

#define CY_REG_BASE                 0x00
#define CY_REG_ACT_DIST             0x1E
#define CY_REG_ACT_INTRVL           0x1D
#define CY_REG_TCH_TMOUT            (CY_REG_ACT_INTRVL+1)
#define CY_REG_LP_INTRVL            (CY_REG_TCH_TMOUT+1)
#define CY_MAXZ                     255
#define CY_DELAY_DFLT               20 /* ms */
#define CY_DELAY_MAX                (500/CY_DELAY_DFLT) /* half second */
#define CY_ACT_DIST_DFLT            0xF8
#define CY_HNDSHK_BIT               0x80
/* device mode bits */
#define CY_OPERATE_MODE             0x00
#define CY_SYSINFO_MODE             0x10
/* power mode select bits */
#define CY_SOFT_RESET_MODE          0x01 /* return to Bootloader mode */
#define CY_DEEP_SLEEP_MODE          0x02
#define CY_LOW_POWER_MODE           0x04

/* Slots management */
#define CY_MAX_FINGER               4
#define CY_MAX_ID                   16

static const u8 bl_command[] = {
	0x00,			/* file offset */
	0xFF,			/* command */
	0xA5,			/* exit bootloader command */
	0, 1, 2, 3, 4, 5, 6, 7	/* default keys */
};

static int ttsp_read_block_data(struct cyttsp *ts, u8 command,
	u8 length, void *buf)
{
	int retval = -1;
	int tries;

	if (!buf || !length)
		return -EINVAL;

	for (tries = 0; tries < CY_NUM_RETRY && (retval < 0); tries++) {
		retval = ts->bus_ops->read(ts->dev, command, length, buf);
		if (retval)
			msleep(CY_DELAY_DFLT);
	}

	return retval;
}

static int ttsp_write_block_data(struct cyttsp *ts, u8 command,
	u8 length, void *buf)
{
	int retval = -1;
	int tries;

	if (!buf || !length)
		return -EINVAL;

	for (tries = 0; tries < CY_NUM_RETRY && (retval < 0); tries++) {
		retval = ts->bus_ops->write(ts->dev, command, length, buf);
		if (retval)
			msleep(CY_DELAY_DFLT);
	}

	return retval;
}

static int cyttsp_load_bl_regs(struct cyttsp *ts)
{
	memset(&(ts->bl_data), 0, sizeof(struct cyttsp_bootloader_data));

	ts->bl_data.bl_status = 0x10;

	return  ttsp_read_block_data(ts, CY_REG_BASE, sizeof(ts->bl_data),
				     &ts->bl_data);
}

static int cyttsp_bl_app_valid(struct cyttsp *ts)
{
	int retval;

	retval = cyttsp_load_bl_regs(ts);

	if (retval < 0) {
		retval = -ENODEV;
		goto done;
	}

	if (GET_BOOTLOADERMODE(ts->bl_data.bl_status)) {
		if (IS_VALID_APP(ts->bl_data.bl_status))
			return 0;
		else
			return -ENODEV;
	}

	if (GET_HSTMODE(ts->bl_data.bl_file) == CY_OPERATE_MODE) {
		if (!(IS_OPERATIONAL_ERR(ts->bl_data.bl_status)))
			return 1;
		else
			return -ENODEV;
	}

	retval = -ENODEV;
done:
	return retval;
}

static int cyttsp_exit_bl_mode(struct cyttsp *ts)
{
	int retval;
	int tries;
	u8 bl_cmd[sizeof(bl_command)];

	memcpy(bl_cmd, bl_command, sizeof(bl_command));
	if (ts->pdata->bl_keys)
		memcpy(&bl_cmd[sizeof(bl_command) - CY_NUM_BL_KEYS],
			ts->pdata->bl_keys, sizeof(bl_command));

	retval = ttsp_write_block_data(ts, CY_REG_BASE,
		sizeof(bl_cmd), (void *)bl_cmd);

	if (retval < 0)
		return retval;

	/* wait for TTSP Device to complete switch to Operational mode */
	tries = 0;
	do {
		msleep(CY_DELAY_DFLT);
		retval = cyttsp_load_bl_regs(ts);
	} while ((retval || GET_BOOTLOADERMODE(ts->bl_data.bl_status)) &&
		(tries++ < CY_DELAY_MAX));

	if (tries >= CY_DELAY_MAX)
		return -ENODEV;

	return retval;
}

static int cyttsp_set_operational_mode(struct cyttsp *ts)
{
	struct cyttsp_xydata xy_data;
	int retval;
	int tries = 0;
	u8 cmd = CY_OPERATE_MODE;

	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);

	if (retval < 0)
		return retval;

	/* wait for TTSP Device to complete switch to Operational mode */
	do {
		retval = ttsp_read_block_data(ts, CY_REG_BASE,
			sizeof(xy_data), &(xy_data));
	} while ((retval || xy_data.act_dist != CY_ACT_DIST_DFLT) &&
		 (tries++ < CY_DELAY_MAX));

	if (tries >= CY_DELAY_MAX)
		return -EAGAIN;

	return retval;
}

static int cyttsp_set_sysinfo_mode(struct cyttsp *ts)
{
	int retval;
	int tries;
	u8 cmd = CY_SYSINFO_MODE;

	memset(&(ts->sysinfo_data), 0, sizeof(struct cyttsp_sysinfo_data));

	/* switch to sysinfo mode */
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	if (retval < 0)
		return retval;

	/* read sysinfo registers */
	tries = 0;
	do {
		msleep(CY_DELAY_DFLT);
		retval = ttsp_read_block_data(ts, CY_REG_BASE,
			sizeof(ts->sysinfo_data), &ts->sysinfo_data);
	} while ((retval || (!ts->sysinfo_data.tts_verh &&
			     !ts->sysinfo_data.tts_verl)) &&
		 (tries++ < CY_DELAY_MAX));

	if (tries >= CY_DELAY_MAX)
		return -EAGAIN;

	return retval;
}

static int cyttsp_set_sysinfo_regs(struct cyttsp *ts)
{
	int retval = 0;

	if (ts->pdata->act_intrvl != CY_ACT_INTRVL_DFLT ||
		ts->pdata->tch_tmout != CY_TCH_TMOUT_DFLT ||
		ts->pdata->lp_intrvl != CY_LP_INTRVL_DFLT) {

		u8 intrvl_ray[3];

		intrvl_ray[0] = ts->pdata->act_intrvl;
		intrvl_ray[1] = ts->pdata->tch_tmout;
		intrvl_ray[2] = ts->pdata->lp_intrvl;

		/* set intrvl registers */
		retval = ttsp_write_block_data(ts,
				CY_REG_ACT_INTRVL,
				sizeof(intrvl_ray), intrvl_ray);

		msleep(CY_DELAY_DFLT);
	}

	return retval;
}

static int cyttsp_soft_reset(struct cyttsp *ts)
{
	int retval;
	u8 cmd = CY_SOFT_RESET_MODE;
	long wait_jiffies = msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX);
	/* wait for interrupt to set ready completion */
	INIT_COMPLETION(ts->bl_ready);

	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	if (retval < 0)
		return retval;

	return wait_for_completion_timeout(&ts->bl_ready, wait_jiffies);
}

static int cyttsp_act_dist_setup(struct cyttsp *ts)
{
	int retval;
	u8 act_dist_setup;

	/* Init gesture; active distance setup */
	act_dist_setup = ts->pdata->act_dist;
	retval = ttsp_write_block_data(ts, CY_REG_ACT_DIST,
		sizeof(act_dist_setup), &act_dist_setup);

	return retval;
}

static int cyttsp_hndshk(struct cyttsp *ts, u8 hst_mode)
{
	u8 cmd;

	cmd = hst_mode ^ CY_HNDSHK_BIT;

	return ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), (u8 *)&cmd);
}

static void cyttsp_report_slot(struct input_dev *dev, int slot,
			       int x, int y, int z)
{
	input_mt_slot(dev, slot);
	input_mt_report_slot_state(dev, MT_TOOL_FINGER, true);
	input_report_abs(dev, ABS_MT_POSITION_X, x);
	input_report_abs(dev, ABS_MT_POSITION_Y, y);
	input_report_abs(dev, ABS_MT_TOUCH_MAJOR, z);
}

static void cyttsp_report_slot_empty(struct input_dev *dev, int slot)
{
	input_mt_slot(dev, slot);
	input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
}

static void cyttsp_extract_track_ids(struct cyttsp_xydata *xy_data, int *ids)
{
	ids[0] = xy_data->touch12_id >> 4;
	ids[1] = xy_data->touch12_id & 0xF;
	ids[2] = xy_data->touch34_id >> 4;
	ids[3] = xy_data->touch34_id & 0xF;
}

static const struct cyttsp_tch *cyttsp_get_tch(struct cyttsp_xydata *xy_data,
					       int idx)
{
	switch (idx) {
	case 0:
		return &xy_data->tch1;
	case 1:
		return &xy_data->tch2;
	case 2:
		return &xy_data->tch3;
	case 3:
		return  &xy_data->tch4;
	default:
		return NULL;
	}
}

static int cyttsp_handle_tchdata(struct cyttsp *ts)
{
	struct cyttsp_xydata xy_data;
	u8 num_cur_tch;
	int i;
	int ids[4];
	const struct cyttsp_tch *tch = NULL;
	int x, y, z;
	int used = 0;

	/* Get touch data from CYTTSP device */
	if (ttsp_read_block_data(ts,
		CY_REG_BASE, sizeof(struct cyttsp_xydata), &xy_data))
		return 0;

	/* provide flow control handshake */
	if (ts->pdata->use_hndshk)
		if (cyttsp_hndshk(ts, xy_data.hst_mode))
			return 0;

	/* determine number of currently active touches */
	num_cur_tch = GET_NUM_TOUCHES(xy_data.tt_stat);

	/* check for any error conditions */
	if (ts->power_state == CY_IDLE_STATE)
		return 0;
	else if (GET_BOOTLOADERMODE(xy_data.tt_mode)) {
		return -1;
	} else if (IS_LARGE_AREA(xy_data.tt_stat) == 1) {
		/* terminate all active tracks */
		num_cur_tch = 0;
		dev_dbg(ts->dev, "%s: Large area detected\n", __func__);
	} else if (num_cur_tch > CY_MAX_FINGER) {
		/* terminate all active tracks */
		num_cur_tch = 0;
		dev_dbg(ts->dev, "%s: Num touch error detected\n", __func__);
	} else if (IS_BAD_PKT(xy_data.tt_mode)) {
		/* terminate all active tracks */
		num_cur_tch = 0;
		dev_dbg(ts->dev, "%s: Invalid buffer detected\n", __func__);
	}

	cyttsp_extract_track_ids(&xy_data, ids);

	for (i = 0; i < num_cur_tch; i++) {
		used |= (1 << ids[i]);

		tch = cyttsp_get_tch(&xy_data, i);

		x = be16_to_cpu(tch->x);
		y = be16_to_cpu(tch->y);
		z = tch->z;

		cyttsp_report_slot(ts->input, ids[i], x, y, z);
	}

	for (i = 0; i < CY_MAX_ID; i++)
		if (!(used & (1 << i)))
			cyttsp_report_slot_empty(ts->input, i);

	input_sync(ts->input);

	return 0;
}

static void cyttsp_pr_state(struct cyttsp *ts)
{
	static char *cyttsp_powerstate_string[] = {
		"IDLE",
		"ACTIVE",
		"LOW_PWR",
		"SLEEP",
		"BOOTLOADER",
		"INVALID"
	};

	dev_info(ts->dev, "%s: %s\n", __func__,
		ts->power_state < CY_INVALID_STATE ?
		cyttsp_powerstate_string[ts->power_state] :
		"INVALID");
}

static irqreturn_t cyttsp_irq(int irq, void *handle)
{
	struct cyttsp *ts = handle;
	int retval;

	if (ts->power_state == CY_BL_STATE)
		complete(&ts->bl_ready);
	else {
		/* process the touches */
		retval = cyttsp_handle_tchdata(ts);

		if (retval < 0) {
			/*
			 * TTSP device has reset back to bootloader mode.
			 * Restore to operational mode.
			 */
			retval = cyttsp_exit_bl_mode(ts);
			if (retval)
				ts->power_state = CY_IDLE_STATE;
			else
				ts->power_state = CY_ACTIVE_STATE;
			cyttsp_pr_state(ts);
		}
	}

	return IRQ_HANDLED;
}

static int cyttsp_power_on(struct cyttsp *ts)
{
	int retval = 0;

	ts->power_state = CY_BL_STATE;
	enable_irq(ts->irq);

	retval = cyttsp_soft_reset(ts);
	if (retval == 0)
		goto bypass;

	retval = cyttsp_bl_app_valid(ts);
	if (retval < 0)
		goto bypass;
	else if (retval > 0)
		goto no_bl_bypass;

	retval = cyttsp_exit_bl_mode(ts);

	if (retval < 0)
		goto bypass;

	ts->power_state = CY_IDLE_STATE;

no_bl_bypass:
	retval = cyttsp_set_sysinfo_mode(ts);
	if (retval < 0)
		goto bypass;

	retval = cyttsp_set_sysinfo_regs(ts);
	if (retval < 0)
		goto bypass;

	retval = cyttsp_set_operational_mode(ts);
	if (retval < 0)
		goto bypass;

	/* init active distance */
	retval = cyttsp_act_dist_setup(ts);
	if (retval < 0)
		goto bypass;

	ts->power_state = CY_ACTIVE_STATE;
	retval = 0;

bypass:
	cyttsp_pr_state(ts);
	return retval;
}

#ifdef CONFIG_PM_SLEEP
static int cyttsp_resume(struct device *dev)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int retval = 0;
	struct cyttsp_xydata xydata;

	if (ts->pdata->use_sleep && ts->power_state != CY_ACTIVE_STATE) {

		if (ts->pdata->wakeup)
			retval = ts->pdata->wakeup();
		else
			retval = -ENOSYS;

		if (retval >= 0) {
			retval = ttsp_read_block_data(ts, CY_REG_BASE,
						      sizeof(xydata),
						      &xydata);
			if (retval >= 0 &&
			    !GET_HSTMODE(xydata.hst_mode))
				ts->power_state = CY_ACTIVE_STATE;
		}
	}

	return retval;
}

static int cyttsp_suspend(struct device *dev)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	u8 sleep_mode = 0;
	int retval = 0;

	if (ts->pdata->use_sleep && ts->power_state == CY_ACTIVE_STATE) {
		sleep_mode = ts->platform_data->use_sleep;
		retval = ttsp_write_block_data(ts,
			CY_REG_BASE, sizeof(sleep_mode), &sleep_mode);
		if (retval >= 0)
			ts->power_state = CY_SLEEP_STATE;
	}

	return retval;
}
#endif

SIMPLE_DEV_PM_OPS(cyttsp_pm_ops, cyttsp_suspend, cyttsp_resume);
EXPORT_SYMBOL_GPL(cyttsp_pm_ops);

static int cyttsp_open(struct input_dev *dev)
{
	struct cyttsp *ts = input_get_drvdata(dev);

	return cyttsp_power_on(ts);
}

static void cyttsp_close(struct input_dev *dev)
{
	struct cyttsp *ts = input_get_drvdata(dev);

	disable_irq(ts->irq);
}

struct cyttsp *cyttsp_probe(const struct cyttsp_bus_ops *bus_ops,
			    struct device *dev, int irq, size_t xfer_buf_size)
{
	const struct cyttsp_platform_data *pdata = dev->platform_data;
	struct cyttsp *ts;
	struct input_dev *input_dev;
	int error;

	if (!dev || !bus_ops || !pdata || !pdata->name || irq <= 0) {
		error = -EINVAL;
		goto err_out;
	}

	ts = kzalloc(sizeof(*ts) + xfer_buf_size, GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	ts->dev = dev;
	ts->input = input_dev;
	ts->pdata = dev->platform_data;
	ts->bus_ops = bus_ops;
	ts->irq = irq;

	init_completion(&ts->bl_ready);
	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(dev));

	if (pdata->init) {
		error = pdata->init();
		if (error) {
			dev_err(ts->dev, "platform init failed, err: %d\n",
				error);
			goto err_free_mem;
		}
	}

	input_dev->name = pdata->name;
	input_dev->phys = ts->phys;
	input_dev->id.bustype = bus_ops->bustype;
	input_dev->dev.parent = ts->dev;

	input_dev->open = cyttsp_open;
	input_dev->close = cyttsp_close;

	input_set_drvdata(input_dev, ts);

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, pdata->maxx, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, pdata->maxy, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, CY_MAXZ, 0, 0);

	input_mt_init_slots(input_dev, CY_MAX_ID);

	error = request_threaded_irq(ts->irq, NULL, cyttsp_irq,
				     IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				     pdata->name, ts);
	if (error) {
		dev_err(ts->dev, "failed to request IRQ %d, err: %d\n",
			ts->irq, error);
		goto err_platform_exit;
	}
	disable_irq(ts->irq);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(ts->dev, "failed to register input device: %d\n",
			error);
		goto err_free_irq;
	}

	return ts;

err_free_irq:
	free_irq(ts->irq, ts);
err_platform_exit:
	if (pdata->exit)
		pdata->exit();
err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
err_out:
	return ERR_PTR(error);
}
EXPORT_SYMBOL_GPL(cyttsp_probe);

void cyttsp_remove(struct cyttsp *ts)
{
	free_irq(ts->irq, ts);
	input_unregister_device(ts->input);
	if (ts->pdata->exit)
		ts->pdata->exit();
	kfree(ts);
}
EXPORT_SYMBOL_GPL(cyttsp_remove);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver core");
MODULE_AUTHOR("Cypress");

