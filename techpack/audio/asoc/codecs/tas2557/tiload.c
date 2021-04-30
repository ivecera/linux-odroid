/*
 * =============================================================================
 * Copyright (c) 2016  Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * File:
 *     tiload.c
 *
 * Description:
 *     utility for TAS2557 Android in-system tuning
 *
 * =============================================================================
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#include "tiload.h"

static struct cdev *tiload_cdev;
static int tiload_major; /* Dynamic allocation of Mjr No. */
static int tiload_opened; /* Dynamic allocation of Mjr No. */
static struct tas2557_priv *g_TAS2557;
struct class *tiload_class;
static unsigned int magic_num;

static char gPage;
static char gBook;

/*
 * Function : tiload_open
 *
 * Purpose  : open method for tiload programming interface
 */
static int tiload_open(struct inode *in, struct file *filp)
{
	struct tas2557_priv *pTAS2557 = g_TAS2557;

	if (tiload_opened) {
		dev_info(pTAS2557->dev, "%s device is already opened\n",
			 "tiload");
		return -EINVAL;
	}

	filp->private_data = pTAS2557;
	tiload_opened++;

	return 0;
}

/*
 * Function : tiload_release
 *
 * Purpose  : close method for tiload programming interface
 */
static int tiload_release(struct inode *in, struct file *filp)
{
	struct tas2557_priv *pTAS2557 = filp->private_data;

	dev_info(pTAS2557->dev, "%s\n", __func__);
	filp->private_data = NULL;
	tiload_opened--;

	return 0;
}

#define MAX_LENGTH 128

/*
 * Function : tiload_read
 *
 * Purpose  : read from codec
 */
static ssize_t tiload_read(struct file *filp, char __user *buf, size_t count,
			   loff_t *offset)
{
	struct tas2557_priv *pTAS2557 = filp->private_data;
	unsigned int nCompositeRegister = 0, Value = 0;
	static char rd_data[MAX_LENGTH + 1];
	char reg_addr;
	size_t size;
	int ret = 0;

	if (count > MAX_LENGTH) {
		dev_err(pTAS2557->dev, "Max %d bytes can be read\n",
			MAX_LENGTH);
		return -EINVAL;
	}

	/* copy register address from user space  */
	size = copy_from_user(&reg_addr, buf, 1);
	if (!size) {
		dev_err(pTAS2557->dev, "read: copy_from_user failure\n");
		return -EINVAL;
	}

	size = count;

	nCompositeRegister = BPR_REG(gBook, gPage, reg_addr);
	if (count == 1) {
		ret = pTAS2557->read(pTAS2557, 0x80000000 | nCompositeRegister,
				     &Value);
		if (ret >= 0)
			rd_data[0] = (char)Value;
	} else if (count > 1) {
		ret = pTAS2557->bulk_read(pTAS2557,
					  0x80000000 | nCompositeRegister,
					  rd_data, size);
	}
	if (ret < 0)
		dev_err(pTAS2557->dev, "ret=%d, count=%zu error happen!\n", ret,
			count);

	dev_dbg(pTAS2557->dev, "read size = %zd, reg_addr= %x , count = %zd\n",
		size, reg_addr, count);

	if (size != count)
		dev_err(pTAS2557->dev, "read %zd registers from the codec\n",
			size);

	if (copy_to_user(buf, rd_data, size) != 0) {
		dev_err(pTAS2557->dev, "copy_to_user failed\n");
		return -EINVAL;
	}

	return size;
}

/*
 * Function : tiload_write
 *
 * Purpose  : write to codec
 */
static ssize_t tiload_write(struct file *filp, const char __user *buf,
			    size_t count, loff_t *offset)
{
	struct tas2557_priv *pTAS2557 = filp->private_data;
	static char wr_data[MAX_LENGTH + 1];
	unsigned int nCompositeRegister = 0;
	unsigned int nRegister;
	char *pData = wr_data;
	int ret = 0;

	if (count > MAX_LENGTH) {
		dev_err(pTAS2557->dev, "max %d bytes can be read\n",
			MAX_LENGTH);
		return -EINVAL;
	}

	/* copy buffer from user space  */
	if (copy_from_user(wr_data, buf, count)) {
		dev_err(pTAS2557->dev,
			"failed to copy %zu bytes from userspace\n", count);
		return -EINVAL;
	}

	dev_dbg(pTAS2557->dev, "write size = %zu\n", count);

	nRegister = wr_data[0];
	if ((nRegister == 127) && (gPage == 0)) {
		gBook = wr_data[1];
		return count;
	}

	if (nRegister == 0) {
		gPage = wr_data[1];
		pData++;
		count--;
	}

	nCompositeRegister = BPR_REG(gBook, gPage, nRegister);
	if (count == 2) {
		ret = pTAS2557->write(pTAS2557, 0x80000000 | nCompositeRegister,
				      pData[1]);
	} else if (count > 2) {
		ret = pTAS2557->bulk_write(pTAS2557,
					   0x80000000 | nCompositeRegister,
					   &pData[1], count - 1);
	}

	if (ret < 0)
		dev_err(pTAS2557->dev, "failed to write %zu bytes (ret = %d)\n",
			count, ret);

	return count;
}

static void tiload_route_IO(struct tas2557_priv *pTAS2557, unsigned int bLock)
{
	if (bLock)
		pTAS2557->write(pTAS2557, 0xAFFEAFFE, 0xBABEBABE);
	else
		pTAS2557->write(pTAS2557, 0xBABEBABE, 0xAFFEAFFE);
}

static long tiload_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct tas2557_priv *pTAS2557 = filp->private_data;
	void __user *argp = (void __user *)arg;
	struct BPR bpr;
	long num = 0;
	int val;

	dev_info(pTAS2557->dev, "TILOAD_IOCTL: cmd=%0x\n", cmd);

	switch (cmd) {
	case TILOAD_IOMAGICNUM_GET:
		dev_info(pTAS2557->dev, "TILOAD_IOMAGICNUM_GET: %u\n",
			 magic_num);
		num = copy_to_user(argp, &magic_num, sizeof(int));
		break;
	case TILOAD_IOMAGICNUM_SET:
		dev_info(pTAS2557->dev, "TILOAD_IOMAGICNUM_SET: %u\n",
			 magic_num);
		num = copy_from_user(&magic_num, argp, sizeof(int));
		tiload_route_IO(pTAS2557, magic_num);
		break;
	case TILOAD_BPR_READ:
		break;
	case TILOAD_BPR_WRITE:
		dev_info(pTAS2557->dev,
			 "TILOAD_BPR_WRITE: %#02x, %#02x, %#02x\n\r", bpr.nBook,
			 bpr.nPage, bpr.nRegister);
		num = copy_from_user(&bpr, argp, sizeof(struct BPR));
		break;
	case TILOAD_IOCTL_SET_CHL:
		break;
	case TILOAD_IOCTL_SET_CONFIG:
		dev_info(pTAS2557->dev, "TILOAD_IOCTL_SET_CONFIG: %d\n", val);
		num = copy_from_user(&val, argp, sizeof(val));
		pTAS2557->set_config(pTAS2557, val);
		break;
	case TILOAD_IOCTL_SET_CALIBRATION:
		dev_info(pTAS2557->dev, "TILOAD_IOCTL_SET_CALIBRATION: %d\n",
			 val);
		num = copy_from_user(&val, argp, sizeof(val));
		pTAS2557->set_calibration(pTAS2557, val);
		break;
	default:
		break;
	}

	return num;
}

#ifdef CONFIG_COMPAT
static long tiload_compat_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	return tiload_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

/*********** File operations structure for tiload *************/
static const struct file_operations tiload_fops = {
	.owner = THIS_MODULE,
	.open = tiload_open,
	.release = tiload_release,
	.read = tiload_read,
	.write = tiload_write,
	.unlocked_ioctl = tiload_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tiload_compat_ioctl,
#endif
};

/*
 * Function : tiload_driver_init
 *
 * Purpose  : Register a char driver for dynamic tiload programming
 */
int tiload_driver_init(struct tas2557_priv *pTAS2557)
{
	dev_t dev = MKDEV(tiload_major, 0);
	int result;

	result = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	if (result < 0) {
		dev_err(pTAS2557->dev, "failed to alloc major number %d\n",
			tiload_major);
		return result;
	}

	tiload_class = class_create(THIS_MODULE, DEVICE_NAME);
	tiload_major = MAJOR(dev);
	dev_info(pTAS2557->dev, "allocated imajor number: %d\n", tiload_major);

	tiload_cdev = cdev_alloc();
	cdev_init(tiload_cdev, &tiload_fops);
	tiload_cdev->owner = THIS_MODULE;
	tiload_cdev->ops = &tiload_fops;

	if (!device_create(tiload_class, NULL, dev, NULL, "tiload_node"))
		dev_err(pTAS2557->dev, "Device creation failed\n");

	result = cdev_add(tiload_cdev, dev, 1) < 0;
	if (result < 0) {
		dev_err(pTAS2557->dev, "failed to add char device\n");
		unregister_chrdev_region(dev, 1);
		tiload_cdev = NULL;
		return result;
	}
	dev_info(pTAS2557->dev, "registered TiLoad driver\n");

	return 0;
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("Utility for TAS2557 Android in-system tuning");
MODULE_LICENSE("GPL v2");
