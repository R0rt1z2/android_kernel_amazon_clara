/*
 * Copyright 2017  Amazon Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include "internal.h"

static DEFINE_MUTEX(tvmsg_lock);
#define TVMSG_MAX_BOUNCE_BUFFER_SIZE (2*PAGE_SIZE)
#define TIMESTAMP_HEADER_SIZE 32


static ssize_t write_tvmsg(struct file *file, const char __user *buf,
			  size_t count, loff_t *ppos)
{
	size_t i, buffer_size;
	char *buffer;

	if (!count)
		return 0;

	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;

	buffer_size = count;
	if (buffer_size > TVMSG_MAX_BOUNCE_BUFFER_SIZE)
		buffer_size = TVMSG_MAX_BOUNCE_BUFFER_SIZE;
	buffer = vmalloc(buffer_size);
	if (!buffer) {
		pr_err("[TVMSG]: Failed to allocate buffer\n");
		return -ENOMEM;
	}

	mutex_lock(&tvmsg_lock);

	for (i = 0; i < count; ) {
		size_t c = min(count - i, buffer_size);
		long ret;
		u64 id;
		u64 ts;
		unsigned long rem_nsec;
		int ts_len;
		char ts_buf[TIMESTAMP_HEADER_SIZE];

		ret = __copy_from_user(buffer, buf + i, c);
		if (unlikely(ret != 0)) {
			mutex_unlock(&tvmsg_lock);
			vfree(buffer);
			return -EFAULT;
		}

		/* print kernel time stamp */
		ts = local_clock();
		rem_nsec = do_div(ts, 1000000000);
		ts_len = snprintf(ts_buf, TIMESTAMP_HEADER_SIZE, "[%5lu.%06lu] ",
				(unsigned long)ts, rem_nsec / 1000);
		psinfo->write_buf(PSTORE_TYPE_TVMSG, 0, &id, 0, ts_buf, 0, ts_len,
				psinfo);

		/* print userspace log */
		psinfo->write_buf(PSTORE_TYPE_TVMSG, 0, &id, 0, buffer, 0, c,
				psinfo);

		i += c;
	}

	mutex_unlock(&tvmsg_lock);
	vfree(buffer);
	return count;
}

static const struct file_operations tvmsg_fops = {
	.owner		= THIS_MODULE,
	.llseek		= noop_llseek,
	.write		= write_tvmsg,
};

static struct class *tvmsg_class;
static int tvmsg_major;
#define TVMSG_NAME "tvmsg"
#undef pr_fmt
#define pr_fmt(fmt) TVMSG_NAME ": " fmt

static char *tvmsg_devnode(struct device *dev, umode_t *mode)
{
	if (mode)
		*mode = 0220;
	return NULL;
}

void pstore_register_tvmsg(void)
{
	struct device *tvmsg_device;

	tvmsg_major = register_chrdev(0, TVMSG_NAME, &tvmsg_fops);
	if (tvmsg_major < 0) {
		pr_err("register_chrdev failed\n");
		goto err;
	}

	tvmsg_class = class_create(THIS_MODULE, TVMSG_NAME);
	if (IS_ERR(tvmsg_class)) {
		pr_err("device class file already in use\n");
		goto err_class;
	}
	tvmsg_class->devnode = tvmsg_devnode;

	tvmsg_device = device_create(tvmsg_class, NULL, MKDEV(tvmsg_major, 0),
					NULL, "%s%d", TVMSG_NAME, 0);
	if (IS_ERR(tvmsg_device)) {
		pr_err("failed to create device\n");
		goto err_device;
	}
	return;

err_device:
	class_destroy(tvmsg_class);
err_class:
	unregister_chrdev(tvmsg_major, TVMSG_NAME);
err:
	return;
}

void pstore_unregister_tvmsg(void)
{
	device_destroy(tvmsg_class, MKDEV(tvmsg_major, 0));
	class_destroy(tvmsg_class);
	unregister_chrdev(tvmsg_major, TVMSG_NAME);
}
