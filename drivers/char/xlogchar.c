/* Copyright (C) 2018 XiaoMi, Inc.
 * Copyright (C) 2020 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/ratelimit.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/current.h>
#include <asm/div64.h>
#include "xlogchar.h"

#define XLOGCHAR_DEVNAME	"xlog"
#define XLOGCHAR_CLASSNAME	"xlog"

static struct class *xlogchar_class;
static int xlogchar_major;

static int xlogchar_open(struct inode *inode, struct file *file)
{
	struct xlogchar *xlc;

	xlc = container_of(inode->i_cdev, struct xlogchar, cdev);
	file->private_data = xlc;

	nonseekable_open(inode, file);

	return 0;
}

static int xlogchar_close(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

static ssize_t xlogchar_read(struct file *file, char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct xlogchar *xlc = file->private_data;
	size_t copy_bytes;
	u64 temp = count;
	int err = 0;

	dev_dbg(xlc->dev, "requested to read %zu bytes\n", count);

	if (do_div(temp, XLOGPKG_SIZE) || (count > XLOGBUF_SIZE)) {
		dev_err(xlc->dev, "invalid count %zu\n", count);
		return -EBADMSG;
	}

	if (!buf)
		return -EFAULT;

	while (err != -ERESTARTSYS) {
		mutex_lock(&xlc->lock);
		if (xlc->used_size >= count)
			break;
		mutex_unlock(&xlc->lock);

		dev_dbg(xlc->dev, "going to sleep\n");
		err = wait_event_interruptible(xlc->wait_q,
					       xlc->used_size >= count);
		dev_dbg(xlc->dev, "wakeup");
	}

	if (err == -ERESTARTSYS) {
		dev_err(xlc->dev, "wake up by signal\n");
		return err;
	}

	while (count) {
		copy_bytes = min(XLOGBUF_SIZE - xlc->read_idx, count);
		err = copy_to_user(buf, xlc->buf + xlc->read_idx, copy_bytes);
		if (err) {
			dev_err(xlc->dev, "failed to copy data\n");
			return err;
		}

		xlc->used_size -= copy_bytes;
		xlc->read_idx += copy_bytes;
		if (xlc->read_idx >= XLOGBUF_SIZE)
			xlc->read_idx -= XLOGBUF_SIZE;

		count -= copy_bytes;
	}

	mutex_unlock(&xlc->lock);

	return count;
}


static ssize_t xlogchar_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct xlogchar *xlc = file->private_data;
	size_t copy_bytes;
	u64 temp = count;
	int err = 0;

	dev_info(xlc->dev, "requested to write %zu bytes\n", count);

	if (do_div(temp, XLOGPKG_SIZE) || (count > XLOGBUF_SIZE)) {
		dev_err(xlc->dev, "invalid count %zu\n", count);
		return -EBADMSG;
	}

	mutex_lock(&xlc->lock);

	if ((XLOGBUF_SIZE - xlc->used_size) < count) {
		dev_err(xlc->dev, "no more space to write [free: %zu, count %zu]\n",
		       XLOGBUF_SIZE - xlc->used_size, count);
		mutex_unlock(&xlc->lock);
		return  -EIO;
	}

	while (count) {
		copy_bytes = min(XLOGBUF_SIZE - xlc->write_idx, count);
		err = copy_from_user(xlc->buf + xlc->write_idx, buf,
				     copy_bytes);
		if (err) {
			dev_err(xlc->dev, "failed to copy data\n");
			return err;
		}

		xlc->used_size += copy_bytes;
		xlc->write_idx += copy_bytes;
		if (xlc->write_idx >= XLOGBUF_SIZE)
			xlc->write_idx -= XLOGBUF_SIZE;

		count -= copy_bytes;
	}

	mutex_unlock(&xlc->lock);

	dev_dbg(xlc->dev, " waking up reader\n");
	wake_up_interruptible(&xlc->wait_q);

	return count;
}

static unsigned int xlogchar_poll(struct file *file, poll_table *wait)
{
	int masks = 0;

	return masks;
}


static const struct file_operations xlogchar_fops = {
	.owner = THIS_MODULE,
	.read = xlogchar_read,
	.write = xlogchar_write,
	.poll = xlogchar_poll,
	.open = xlogchar_open,
	.release = xlogchar_close
};

static int xlogchar_setup_cdev(struct xlogchar *xlc, dev_t devno)
{
	int err;

	cdev_init(&xlc->cdev, &xlogchar_fops);
	xlc->cdev.owner = THIS_MODULE;

	err = cdev_add(&xlc->cdev, devno, 1);
	if (err < 0) {
		pr_err("failed to add char device to system\n");
		return err;
	}

	xlc->dev = device_create(xlogchar_class, NULL, devno, xlc,
				 XLOGCHAR_DEVNAME);
	if (xlc->dev) {
		pr_err("failed to create device\n");
		cdev_del(&xlc->cdev);

		return -EIO;
	}

	return 0;
}

static int __init xlogchar_init(void)
{
	struct xlogchar *xlc;
	int ret = 0;
	dev_t dev;

	pr_info("initialize\n");

	xlc = kzalloc(sizeof(struct xlogchar), GFP_KERNEL);
	if (!xlc) {
		pr_err("failed to allocate memory\n");
		return -ENOMEM;
	}

	xlc->buf = kzalloc(XLOGBUF_SIZE, GFP_KERNEL);
	if (!xlc->buf) {
		pr_err("failed to allocate data buffer\n");
		ret = -ENOMEM;
		goto error_buffer;
	}

	mutex_init(&xlc->lock);
	init_waitqueue_head(&xlc->wait_q);

	/* Get major number from kernel and initialize */
	ret = alloc_chrdev_region(&dev, 1, 1, XLOGCHAR_DEVNAME);
	if (ret < 0) {
		pr_err("failed to alloc char device region\n");
		goto error_chrdev;
	}
	xlogchar_major = MAJOR(dev);

	/* Create devices class */
	xlogchar_class = class_create(THIS_MODULE, XLOGCHAR_CLASSNAME);
	if (IS_ERR(xlogchar_class)) {
		pr_err("failed to create device class\n");
		ret = PTR_ERR(xlogchar_class);
		goto error_class;
	}

	ret = xlogchar_setup_cdev(xlc, dev);
	if (ret)
		goto error_cdev;

	pr_info("init done\n");

	return 0;

error_cdev:
	class_destroy(xlogchar_class);
error_class:
	unregister_chrdev_region(MKDEV(xlogchar_major, 0), 1);
error_chrdev:
	kfree(xlc->buf);
error_buffer:
	kfree(xlc);

	return ret;
}

static int xlogchar_destroy_one(struct device *dev, void *data)
{
	struct xlogchar *xlc = dev_get_drvdata(dev);

	device_destroy(xlogchar_class, dev->devt);
	cdev_del(&xlc->cdev);
	kfree(xlc->buf);
	kfree(xlc);

	return 0;
}

static void xlogchar_exit(void)
{
	class_for_each_device(xlogchar_class, NULL, NULL,
			      xlogchar_destroy_one);
	class_destroy(xlogchar_class);

	pr_info("exit\n");
}

core_initcall(xlogchar_init);
module_exit(xlogchar_exit);

MODULE_DESCRIPTION("Xlog Char Driver");
MODULE_LICENSE("GPL v2");
