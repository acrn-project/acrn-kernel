/*
 * ACRN Project
 * Virtio Backend Service (VBS) for ACRN hypervisor
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright (c) 2018 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Contact Information:
 *
 * BSD LICENSE
 *
 * Copyright (c) 2018 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *  VBS-K Reference Driver: virtio-echo
 *  - Each VBS-K driver exports a char device to /dev/, e.g. /dev/vbs_echo;
 *  - Each VBS-K driver uses Virtqueue APIs to interact with the virtio
 *    frontend driver in guest;
 *  - Each VBS-K driver registers itelf as VHM (Virtio and Hypervisor
 *    service Module) client, which enables in-kernel handling of register
 *    access of virtio device;
 *  - Each VBS-K driver could maintain the connections, from VBS-U, in a
 *    list/table, so that it could serve multiple guests.
 */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/hw_random.h>
#include <linux/uio.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <linux/vbs/vq.h>
#include <linux/vbs/vbs.h>
#include <linux/hashtable.h>

enum {
	VBS_K_ECHO_RXQ = 0,
	VBS_K_ECHO_TXQ = 1,
	VBS_K_ECHO_MAXQ = 2
};

#define VTRND_RINGSZ 64

/* VBS-K features if any */
/*
 *enum {
 *	VBS_K_RNG_FEATURES = VBS_K_FEATURES |
 *			 (1ULL << VIRTIO_F_VERSION_1),
 *};
 */

/**
 * struct vbs_echo - Backend of virtio-echo based on VBS-K
 *
 * @dev		: instance of struct virtio_dev_info
 * @vqs		: instances of struct virtio_vq_info
 * @node	: hashtable maintaining multiple connections
 *		  from multiple guests/devices
 */
struct vbs_echo {
	struct virtio_dev_info dev;
	struct virtio_vq_info vqs[VBS_K_ECHO_MAXQ];
	int buf_size;
	int count;
	int cur_cnt;
	bool reset;
	ktime_t start;
	s64 total_us;
	s64 min_us;
	s64 max_us;
	/*
	 * Each VBS-K module might serve multiple connections
	 * from multiple guests/device models/VBS-Us, so better
	 * to maintain the connections in a list, and here we
	 * use hashtable as an example.
	 */
	struct hlist_node node;
};

#define ECHO_MAX_HASH_BITS 4		/* MAX is 2^4 */
#define HASH_NAME vbs_echo_hash

DECLARE_HASHTABLE(HASH_NAME, ECHO_MAX_HASH_BITS);
static int vbs_echo_hash_initialized = 0;
static int vbs_echo_connection_cnt = 0;

/* function declarations */
static int handle_kick(int client_id, unsigned long *ioreqs_map);
static long vbs_echo_reset(struct vbs_echo *echo);
static void vbs_echo_stop(struct vbs_echo *echo);
static void vbs_echo_flush(struct vbs_echo *echo);

/* hash table related functions */
static void vbs_echo_hash_init(void)
{
	if (vbs_echo_hash_initialized)
		return;

	hash_init(HASH_NAME);
	vbs_echo_hash_initialized = 1;
}

static int vbs_echo_hash_add(struct vbs_echo *entry)
{
	if (!vbs_echo_hash_initialized) {
		pr_err("RNG hash table not initialized!\n");
		return -1;
	}

	hash_add(HASH_NAME, &entry->node, virtio_dev_client_id(&entry->dev));
	return 0;
}

static struct vbs_echo *vbs_echo_hash_find(int client_id)
{
	struct vbs_echo *entry;
	int bkt;

	if (!vbs_echo_hash_initialized) {
		pr_err("RNG hash table not initialized!\n");
		return NULL;
	}

	hash_for_each(HASH_NAME, bkt, entry, node)
		if (virtio_dev_client_id(&entry->dev) == client_id)
			return entry;

	pr_err("Not found item matching client_id!\n");
	return NULL;
}

static int vbs_echo_hash_del(int client_id)
{
	struct vbs_echo *entry;
	int bkt;

	if (!vbs_echo_hash_initialized) {
		pr_err("RNG hash table not initialized!\n");
		return -1;
	}

	hash_for_each(HASH_NAME, bkt, entry, node)
		if (virtio_dev_client_id(&entry->dev) == client_id) {
			hash_del(&entry->node);
			return 0;
		}

	pr_err("%s failed, not found matching client_id!\n",
	       __func__);
	return -1;
}

static int vbs_echo_hash_del_all(void)
{
	struct vbs_echo *entry;
	int bkt;

	if (!vbs_echo_hash_initialized) {
		pr_err("RNG hash table not initialized!\n");
		return -1;
	}

	hash_for_each(HASH_NAME, bkt, entry, node)
		hash_del(&entry->node);

	return 0;
}

/* send data to guest */
static noinline void vbs_echo_notify_rx_vq(struct vbs_echo *echo,
	struct virtio_vq_info *vq)
{
	struct iovec iov;
	int n, size;
	uint16_t idx;

	if (echo->reset) {
		pr_err("%s[%d], rx_buf = %p, len = 0x%x\n",
			__func__, echo->cur_cnt, iov.iov_base, size);
		echo->reset = false;
	}

	if (echo->cur_cnt < echo->count) {
		echo->start = ktime_get();
		if (virtio_vq_has_descs(vq)) {
			n = virtio_vq_getchain(vq, &idx, &iov, 1, NULL);
			BUG_ON(n != 1);
			size = echo->buf_size <= iov.iov_len
				? echo->buf_size
				: iov.iov_len;
			memset(iov.iov_base, 0xAA, size);
			trace_printk("%s[%d], rx_buf = %p, len = 0x%x\n",
				__func__, echo->cur_cnt, iov.iov_base, size);
			virtio_vq_relchain(vq, idx, size);
			echo->cur_cnt++;
		}
		virtio_vq_endchains(vq, 1);
	} else {
		pr_err("%s, total_us = %lld, count = %d\n",
			__func__, echo->total_us, echo->cur_cnt);
		pr_err("%s, min_us = %lld, max_us = %lld\n",
			__func__, echo->min_us, echo->max_us);
		pr_err("%s, average_us = %lld\n",
			__func__, echo->total_us/echo->cur_cnt);
	}
}

/* receive data from guest */
static noinline void vbs_echo_notify_tx_vq(struct vbs_echo *echo,
	struct virtio_vq_info *vq)
{
	struct iovec iov;
	int n;
	uint16_t idx;
	ktime_t end;
	s64 this_us;

	if (virtio_vq_has_descs(vq)) {
		n = virtio_vq_getchain(vq, &idx, &iov, 1, NULL);
		BUG_ON(n != 1);
		end = ktime_get();
		this_us = ktime_to_us(ktime_sub(end, echo->start));
		echo->total_us += this_us;
		if (this_us < echo->min_us)
			echo->min_us = this_us;
		if (this_us > echo->max_us)
			echo->max_us = this_us;
		trace_printk("%s[%d], ns[%lld], tx_buf = %p, len = 0x%lx\n",
			__func__, echo->cur_cnt, this_us,
			iov.iov_base, iov.iov_len);
		virtio_vq_relchain(vq, idx, iov.iov_len);
	}
	virtio_vq_endchains(vq, 1);
}

static void handle_vq_kick(struct vbs_echo *echo, int vq_idx)
{
	struct virtio_vq_info *vq;

	BUG_ON(!echo);
	vq = &echo->vqs[vq_idx];
	if (vq_idx == VBS_K_ECHO_RXQ)
		vbs_echo_notify_rx_vq(echo, vq);
	else if (vq_idx == VBS_K_ECHO_TXQ)
		vbs_echo_notify_tx_vq(echo, vq);
	else
		BUG();
}

static int handle_kick(int client_id, unsigned long *ioreqs_map)
{
	int val = -1;
	struct vbs_echo *echo;

	if (unlikely(bitmap_empty(ioreqs_map, VHM_REQUEST_MAX))) {
		pr_err("%s: bitmap_empty!\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: handle kick: 0x%lx\n", __func__, *ioreqs_map);

	echo = vbs_echo_hash_find(client_id);
	if (echo == NULL) {
		pr_err("%s: client %d not found!\n",
				__func__, client_id);
		return -EINVAL;
	}

	val = virtio_vq_index_get(&echo->dev, ioreqs_map);

	if (val >= 0)
		handle_vq_kick(echo, val);

	return 0;
}

static int vbs_echo_open(struct inode *inode, struct file *f)
{
	struct vbs_echo *echo;
	struct virtio_dev_info *dev;
	struct virtio_vq_info *vqs;
	int i;

	echo = kzalloc(sizeof(*echo), GFP_KERNEL);
	if (echo == NULL) {
		pr_err("Failed to allocate memory for vbs_echo!\n");
		return -ENOMEM;
	}

	echo->total_us = 0;
	echo->min_us = LLONG_MAX;
	echo->max_us = 0;
	echo->cur_cnt = 0;

	dev = &echo->dev;
	strncpy(dev->name, "vbs_echo", VBS_NAME_LEN);
	dev->dev_notify = handle_kick;
	vqs = (struct virtio_vq_info *)&echo->vqs;

	for (i = 0; i < VBS_K_ECHO_MAXQ; i++) {
		vqs[i].dev = dev;
		/*
		 * Currently relies on VHM to kick us,
		 * thus vq_notify not used
		 */
		vqs[i].vq_notify = NULL;
	}

	/* link dev and vqs */
	dev->vqs = vqs;

	virtio_dev_init(dev, vqs, VBS_K_ECHO_MAXQ);

	f->private_data = echo;

	/* init a hash table to maintain multi-connections */
	vbs_echo_hash_init();

	pr_err("%s, successful\n", __func__);

	return 0;
}

static int vbs_echo_release(struct inode *inode, struct file *f)
{
	struct vbs_echo *echo = f->private_data;

	if (!echo)
		pr_err("%s: UNLIKELY echo NULL!\n",
		       __func__);

	vbs_echo_stop(echo);
	vbs_echo_flush(echo);

	/* device specific release */
	vbs_echo_reset(echo);

	pr_err("vbs_echo_connection cnt is %d\n",
			vbs_echo_connection_cnt);

	if (echo && vbs_echo_connection_cnt--)
		vbs_echo_hash_del(virtio_dev_client_id(&echo->dev));
	if (!vbs_echo_connection_cnt) {
		pr_debug("vbs_echo remove all hash entries\n");
		vbs_echo_hash_del_all();
	}

	kfree(echo);

	pr_err("%s done\n", __func__);
	return 0;
}

static long vbs_echo_ioctl(struct file *f, unsigned int ioctl,
			    unsigned long arg)
{
	struct vbs_echo *echo = f->private_data;
	void __user *argp = (void __user *)arg;
	/*u64 __user *featurep = argp;*/
	/*u64 features;*/
	int r;

	switch (ioctl) {
/*
 *	case VHOST_GET_FEATURES:
 *		features = VHOST_NET_FEATURES;
 *		if (copy_to_user(featurep, &features, sizeof features))
 *			return -EFAULT;
 *		return 0;
 *	case VHOST_SET_FEATURES:
 *		if (copy_from_user(&features, featurep, sizeof features))
 *			return -EFAULT;
 *		if (features & ~VHOST_NET_FEATURES)
 *			return -EOPNOTSUPP;
 *		return vhost_net_set_features(n, features);
 */
	case VBS_SET_VQ:
		/*
		 * we handle this here because we want to register VHM client
		 * after handling VBS_K_SET_VQ request
		 */
		r = virtio_vqs_ioctl(&echo->dev, ioctl, argp);
		if (r == -ENOIOCTLCMD) {
			pr_err("VBS_K_SET_VQ: virtio_vqs_ioctl failed!\n");
			return -EFAULT;
		}
		/* Register VHM client */
		if (virtio_dev_register(&echo->dev) < 0) {
			pr_err("failed to register VHM client!\n");
			return -EFAULT;
		}
		/* Added to local hash table */
		if (vbs_echo_hash_add(echo) < 0) {
			pr_err("failed to add to hashtable!\n");
			return -EFAULT;
		}
		/* Increment counter */
		vbs_echo_connection_cnt++;

		pr_err("%s, VBS_SET_VQ\n", __func__);

		return r;

	case VBS_RESET_DEV:
		pr_err("VBS_RESET_DEV ioctl:\n");
		vbs_echo_stop(echo);
		vbs_echo_flush(echo);
		r = vbs_echo_reset(echo);
		return r;

	case VBS_ECHO_SET_BUFSIZE: {
		int buf_size;

		if (copy_from_user(&buf_size, argp, sizeof(int)))
			return -EFAULT;
		echo->buf_size = buf_size;
		pr_err("%s, VBS_ECHO_SET_BUFSIZE, %d\n", __func__, buf_size);
		return 0;
	}

	case VBS_ECHO_SET_COUNT: {
		int count;

		if (copy_from_user(&count, argp, sizeof(int)))
			return -EFAULT;
		echo->count = count;
		pr_err("%s, VBS_ECHO_SET_COUNT, %d\n", __func__, count);
		return 0;
	}

	default:
		/*mutex_lock(&n->dev.mutex);*/
		pr_err("%s, 0x%x\n", __func__, ioctl);
		r = virtio_dev_ioctl(&echo->dev, ioctl, argp);
		if (r == -ENOIOCTLCMD)
			r = virtio_vqs_ioctl(&echo->dev, ioctl, argp);
		else
			vbs_echo_flush(echo);
		/*mutex_unlock(&n->dev.mutex);*/
		return r;
	}
}

/* device specific function to cleanup itself */
static long vbs_echo_reset(struct vbs_echo *echo)
{
	pr_err("%s, cur_cnt = %d, total_cnt = %d\n",
		__func__, echo->cur_cnt, echo->count);
	echo->reset = true;
	return virtio_dev_reset(&echo->dev);
}

/* device specific function */
static void vbs_echo_stop(struct vbs_echo *echo)
{
	virtio_dev_deregister(&echo->dev);
}

/* device specific function */
static void vbs_echo_flush(struct vbs_echo *echo)
{
}

static const struct file_operations vbs_echo_fops = {
	.owner          = THIS_MODULE,
	.release        = vbs_echo_release,
	.unlocked_ioctl = vbs_echo_ioctl,
	.open           = vbs_echo_open,
	.llseek		= noop_llseek,
};

static struct miscdevice vbs_echo_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vbs_echo",
	.fops = &vbs_echo_fops,
};

static int vbs_echo_init(void)
{
	return misc_register(&vbs_echo_misc);
}
module_init(vbs_echo_init);

static void vbs_echo_exit(void)
{
	misc_deregister(&vbs_echo_misc);
}
module_exit(vbs_echo_exit);

MODULE_VERSION("0.1");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL and additional rights");
MODULE_DESCRIPTION("Virtio Backend Service reference driver on ACRN hypervisor");
