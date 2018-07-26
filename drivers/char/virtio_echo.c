/*
 * virtio echo frontend driver
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright (c) 2017 Intel Corporation. All rights reserved.
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
 * BSD LICENSE
 *
 * Copyright (C) 2017 Intel Corporation. All rights reserved.
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
 * Jian Jun Chen <jian.jun.chen@intel.com>
 *
 */

#include <linux/err.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <uapi/linux/virtio_ids.h>

static int virtio_echo_with_copy = 1;
module_param(virtio_echo_with_copy, int, 0444);
MODULE_PARM_DESC(virtio_echo_with_copy, "virtio_echo_with_copy. (default: 1)");

#define VIRTIO_ECHO_BUF_SIZE		(2 * 1024 * 1024)

struct virtio_echo_info {
	struct virtio_device	*vdev;
	struct virtqueue	*rx, *tx;
	spinlock_t		lock;
	struct delayed_work	fill_rx_work;
	char			*rx_buf;
	char			*tx_buf;
	unsigned int		buf_size;
	bool			ready;
	bool			with_copy;
};

/* should be called with vi->lock held */
static noinline void virtio_echo_fill_rx(struct virtio_echo_info *vi,
	char *buf, unsigned int size)
{
	struct scatterlist sg[1];
	int rc;

	sg_init_one(sg, buf, size);

	rc = virtqueue_add_inbuf(vi->rx, sg, 1, buf, GFP_ATOMIC);
	BUG_ON(rc);
	virtqueue_kick(vi->rx);
}

/* should be called with vi->lock held */
static noinline void virtio_echo_fill_tx(struct virtio_echo_info *vi,
	char *buf, unsigned int size)
{
	struct scatterlist sg[1];
	int rc;

	sg_init_one(sg, buf, size);

	rc = virtqueue_add_outbuf(vi->tx, sg, 1, buf, GFP_ATOMIC);
	BUG_ON(rc);
	virtqueue_kick(vi->tx);
}

static void virtio_echo_fill_rx_work(struct work_struct *work)
{
	struct virtio_echo_info *vi =
		container_of(work, struct virtio_echo_info, fill_rx_work.work);
	unsigned long flags;

	pr_err("%s, rx_buf = %p, len = 0x%x\n",
		__func__, vi->rx_buf, vi->buf_size);

	spin_lock_irqsave(&vi->lock, flags);
	virtio_echo_fill_rx(vi, vi->rx_buf, vi->buf_size);
	spin_unlock_irqrestore(&vi->lock, flags);
}

static void virtio_echo_handle_rx(struct virtqueue *vq)
{
	struct virtio_echo_info *vi = vq->vdev->priv;
	unsigned long flags;
	unsigned int len;
	char *buf;

	spin_lock_irqsave(&vi->lock, flags);
	if (vi->ready) {
		/* get buf from rx queue */
		buf = virtqueue_get_buf(vi->rx, &len);
		spin_unlock_irqrestore(&vi->lock, flags);

		trace_printk("%s, [%d], rx_buf = %p, len = 0x%x\n",
			__func__, virtio_echo_with_copy, buf, len);

		/* here we emulate some memory operation */
		if (virtio_echo_with_copy) {
			if (unlikely(len > vi->buf_size))
				len = vi->buf_size;
			/* copy data from rx buf to tx buf */
			memcpy(vi->tx_buf, buf, len);
		}

		trace_printk("%s, [%d], tx_buf = %p, len = 0x%x\n",
			__func__, virtio_echo_with_copy, vi->tx_buf, len);

		spin_lock_irqsave(&vi->lock, flags);
		/* fill tx queue and kick tx */
		if (virtio_echo_with_copy)
			virtio_echo_fill_tx(vi, vi->tx_buf, len);
		else
			virtio_echo_fill_tx(vi, vi->rx_buf, len);
	}
	spin_unlock_irqrestore(&vi->lock, flags);
}

static void virtio_echo_handle_tx(struct virtqueue *vq)
{
	struct virtio_echo_info *vi = vq->vdev->priv;
	unsigned long flags;
	unsigned int len;
	char *buf;

	spin_lock_irqsave(&vi->lock, flags);

	/* we pop tx buffer first */
	buf = virtqueue_get_buf(vi->tx, &len);

	trace_printk("%s, rx_buf = %p, len = 0x%x\n",
			__func__, buf, len);

	/* we then push rx buffer again and kick rx queue */
	virtio_echo_fill_rx(vi, vi->rx_buf, vi->buf_size);

	spin_unlock_irqrestore(&vi->lock, flags);
}

static int virtio_echo_init_vqs(struct virtio_echo_info *vi)
{
	struct virtqueue *vqs[2];
	vq_callback_t *cbs[] = {virtio_echo_handle_rx,
				virtio_echo_handle_tx};
	static const char * const names[] = { "echo_rx", "echo_tx" };
	int err;

	err = virtio_find_vqs(vi->vdev, 2, vqs, cbs, names, NULL);
	if (err)
		return err;
	vi->rx = vqs[0];
	vi->tx = vqs[1];

	return 0;
}

static int virtio_echo_probe_common(struct virtio_device *vdev)
{
	int err;
	struct virtio_echo_info *vi = NULL;

	vi = kzalloc(sizeof(struct virtio_echo_info), GFP_KERNEL);
	if (!vi)
		return -ENOMEM;

	vi->buf_size = VIRTIO_ECHO_BUF_SIZE;
	vi->rx_buf = kmalloc(vi->buf_size * 2, GFP_KERNEL);
	if (!vi->rx_buf) {
		err = -ENOMEM;
		goto err_alloc_buf;
	}
	vi->tx_buf = vi->rx_buf + vi->buf_size;

	pr_err("%s, rx_buf = %p, tx_buf = %p\n",
		__func__, vi->rx_buf, vi->tx_buf);

	vdev->priv = vi;
	vi->vdev = vdev;
	spin_lock_init(&vi->lock);

	err = virtio_echo_init_vqs(vi);
	if (err)
		goto err_init_vq;

	INIT_DELAYED_WORK(&vi->fill_rx_work, virtio_echo_fill_rx_work);
	schedule_delayed_work(&vi->fill_rx_work, 30 * HZ);

	vi->with_copy = true;
	vi->ready = true;

	return 0;

err_init_vq:
	kfree(vi->rx_buf);
err_alloc_buf:
	kfree(vi);
	return err;
}

static void virtio_echo_remove_common(struct virtio_device *vdev)
{
	struct virtio_echo_info *vi = vdev->priv;
	unsigned long flags;

	cancel_delayed_work_sync(&vi->fill_rx_work);

	spin_lock_irqsave(&vi->lock, flags);
	vi->ready = false;
	spin_unlock_irqrestore(&vi->lock, flags);

	vdev->config->reset(vdev);
	vdev->config->del_vqs(vdev);

	kfree(vi->rx_buf);
	kfree(vi);
}

static int virtio_echo_probe(struct virtio_device *vdev)
{
	return virtio_echo_probe_common(vdev);
}

static void virtio_echo_remove(struct virtio_device *vdev)
{
	virtio_echo_remove_common(vdev);
}

#ifdef CONFIG_PM_SLEEP
static int virtio_echo_freeze(struct virtio_device *vdev)
{
	virtio_echo_remove_common(vdev);
	return 0;
}

static int virtio_echo_restore(struct virtio_device *vdev)
{
	return virtio_echo_probe_common(vdev);
}
#endif

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_ECHO, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static struct virtio_driver virtio_echo_driver = {
	.driver.name =	KBUILD_MODNAME,
	.driver.owner =	THIS_MODULE,
	.id_table =	id_table,
	.probe =	virtio_echo_probe,
	.remove =	virtio_echo_remove,
#ifdef CONFIG_PM_SLEEP
	.freeze =	virtio_echo_freeze,
	.restore =	virtio_echo_restore,
#endif
};

module_virtio_driver(virtio_echo_driver);
MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("Virtio echo driver");
MODULE_LICENSE("GPL");
