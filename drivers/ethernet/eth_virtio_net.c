/*
 * Copyright (c) 2011 IBM Corporation
 * Copyright (c) 2020 Hesham Almatary
 * Copyright (c) 2022 Huawei Technologies
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 */

#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/net/ethernet.h>
#include <ethernet/eth_stats.h>

#define LOG_MODULE_NAME eth_virtio_net
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/drivers/virtio/virtio.h>
#include "eth_virtio_net_priv.h"

#define NET_GUEST_FEATURE_SUPPORT	(VIRTIO_NET_F_MAC)

static uint16_t last_rx_idx;	/* Last index in RX "used" ring */

/* Callbacks for the virtqueue */
static void vnet_rx_cb(struct virtio_device *vdev);
static void vnet_tx_cb(struct virtio_device *vdev);

/**
 * @brief Internal function to probe features the net devices has
 * and that Zephyr can support.
 *
 * @param vdev the virtio device
 * @return uint64_t the features that Zephyr can support
 */
static uint64_t vnet_probe_features(struct virtio_device *vdev)
{
	ARG_UNUSED(vdev);

	/* Currently our driver is very minimal so all the fancy features
	 * are ignored.
	 */

	return 0;
}

/**
 * @brief Initialize the virtio-net device in the virtio sense
 *
 * @param dev the zephyr device associated with the subsystem
 * @return int
 */
static int vnet_dev_init(const struct device *dev)
{
	int i;
	int status = VIRTIO_STAT_ACKNOWLEDGE | VIRTIO_STAT_DRIVER;
	struct virtio_net *vnet = (struct virtio_net *)dev->data;
	struct vqs *vq_tx = NULL, *vq_rx = NULL;
	struct virtio_device *vdev;

	/* Device already running */
	if (vnet->running != 0)
		return -EALREADY;

	/* Find the first NET virtio device */
	SYS_SLIST_FOR_EACH_CONTAINER(&virtio_vdevs, vdev, node) {
		if (vdev->dev_type == VIRTIO_DEV_ID_NET)
			break;
	}

	if (!vdev) {
		LOG_ERR("%s: vdev null\n", __func__);
		return -EINVAL;
	}
	vnet->vdev	= vdev;
	vnet->eth_dev	= dev;

	/* Reset device */
	virtio_reset_device(vdev);

	/* Acknowledge device */
	virtio_set_status(vdev, VIRTIO_STAT_ACKNOWLEDGE);

	/* Tell HV that we know how to drive the device. */
	virtio_set_status(vdev, status);

	/* Device specific setup */
	if (vdev->features & VIRTIO_F_VERSION_1) {
		uint64_t guest_features = vnet_probe_features(vnet->vdev);

		if (virtio_negotiate_guest_features(vdev,
		    guest_features | NET_GUEST_FEATURE_SUPPORT))
			goto dev_error;
		vnet->net_hdr_size = sizeof(struct virtio_net_hdr_v1);
		status = virtio_get_status(vdev);
	} else {
		vnet->net_hdr_size = sizeof(struct virtio_net_hdr);
		virtio_set_guest_features(vdev, 0);
	}

	/* We are only interested in the receive and transmit queue here
	 * XXX: The size value comes from QEMU's implementation. Unsure how to probe it.
	 */
	vq_rx = virtio_queue_init(vdev, VQ_RX, vnet_rx_cb, vnet, 256);
	vq_tx = virtio_queue_init(vdev, VQ_TX, vnet_tx_cb, vnet, 256);
	if (!vq_rx || !vq_tx) {
		LOG_ERR("virtionet: Failed to vq_rx/vq_tx!\n");
		virtio_set_status(vdev,
			VIRTIO_STAT_ACKNOWLEDGE | VIRTIO_STAT_DRIVER | VIRTIO_STAT_FAILED);
		return -ENODEV;
	}

	/* Allocate memory for multiple receive buffers */
	vq_rx->buf_mem = k_aligned_alloc(8,
				(BUFFER_ENTRY_SIZE + vnet->net_hdr_size) * RX_QUEUE_SIZE);
	if (!vq_rx->buf_mem) {
		LOG_ERR("Failed to allocate rx buffers!");
		goto dev_error;
	}

	/* Prepare receive buffer queue */
	for (i = 0; i < RX_QUEUE_SIZE; i++) {
		uint64_t addr = (uint64_t)vq_rx->buf_mem
					+ i * (BUFFER_ENTRY_SIZE + vnet->net_hdr_size);
		uint32_t id = i * 2;

		/* Descriptor for net_hdr: */
		virtio_fill_desc(vq_rx, id, vdev->features, addr, vnet->net_hdr_size,
				 VRING_DESC_F_NEXT | VRING_DESC_F_WRITE, id + 1);

		/* Descriptor for data: */
		virtio_fill_desc(vq_rx, id + 1, vdev->features, addr + vnet->net_hdr_size,
				 BUFFER_ENTRY_SIZE, VRING_DESC_F_WRITE, 0);

		vq_rx->avail->ring[i] = virtio_cpu_to_modern16(vdev, id);
	}
	barrier_dmem_fence_full();

	vq_rx->avail->flags = virtio_cpu_to_modern16(vdev, 0);
	vq_rx->avail->idx = virtio_cpu_to_modern16(vdev, RX_QUEUE_SIZE);

	last_rx_idx = virtio_modern16_to_cpu(vdev, vq_rx->used->idx);

	vq_tx->avail->flags = virtio_cpu_to_modern16(vdev, 0);
	vq_tx->avail->idx = 0;

	/* Tell HV that setup succeeded */
	status |= VIRTIO_STAT_DRIVER_OK;
	virtio_set_status(vdev, status);

	/* Tell HV that RX/TX queues are ready */
	virtio_queue_ready(vdev, VQ_RX);
	virtio_queue_ready(vdev, VQ_TX);

	vnet->running = 1;
	for (i = 0; i < (int)sizeof(vnet->config.mac); i++)
		virtio_get_config(vdev, i, (uintptr_t)&vnet->config.mac[i], 1);

	LOG_DBG("(%02x:%02x:%02x:%02x:%02x:%02x)",
		vnet->config.mac[0], vnet->config.mac[1],
		vnet->config.mac[2], vnet->config.mac[3],
		vnet->config.mac[4], vnet->config.mac[5]);

	return 0;

dev_error:
	/* Free vq memory */
	virtio_queue_term(vdev, vq_rx);
	virtio_queue_term(vdev, vq_tx);
	/* Set the device to failed */
	status |= VIRTIO_STAT_FAILED;
	virtio_set_status(vdev, status);
	LOG_ERR("Failed to setup the virtio-net device.");

	return -ENODEV;
}

/**
 * @brief Transmit a packet using virtio-net
 *
 * @param vnet the virtio-net device
 * @param buf the buffer to send
 * @param len length of the buffer
 * @return int
 */
static int vnet_xmit(struct virtio_net *vnet, void *buf, size_t len)
{
	int id, idx;
	static struct virtio_net_hdr_v1 nethdr_v1 =  {0};
	static struct virtio_net_hdr nethdr_legacy = {0};
	void *nethdr = &nethdr_legacy;
	struct virtio_device *vdev = vnet->vdev;
	struct vqs *vq_tx = &vdev->vq[VQ_TX];

	if (len > BUFFER_ENTRY_SIZE) {
		LOG_ERR("Packet too big!\n");
		return -ENOMEM;
	}

	if (vdev->features & VIRTIO_F_VERSION_1)
		nethdr = &nethdr_v1;

	/* Determine descriptor index */
	idx = virtio_modern16_to_cpu(vdev, vq_tx->avail->idx);
	id = (idx * 2) % vq_tx->size;

	/* Set up virtqueue descriptor for header */
	virtio_fill_desc(vq_tx, id, vdev->features, (uint64_t)nethdr,
				 vnet->net_hdr_size, VRING_DESC_F_NEXT, id + 1);

	/* Set up virtqueue descriptor for data */
	virtio_fill_desc(vq_tx, id + 1, vdev->features, (uint64_t)buf, len, 0, 0);

	vq_tx->avail->ring[idx % vq_tx->size] = virtio_cpu_to_modern16(vdev, id);
	barrier_dmem_fence_full();
	vq_tx->avail->idx = virtio_cpu_to_modern16(vdev, idx + 1);
	barrier_dmem_fence_full();

	/* Kick the HV to notify packet transmition */
	virtio_queue_notify(vdev, VQ_TX);

	LOG_DBG("packet at %p, %ld bytes", buf, len + vnet->net_hdr_size);

	return 0;
}

/*
 * Bottom part of this file is to interface with Zephyr
 */

/**
 * @brief Send api for the ethernet api
 *
 * @param ddev the device as kwnown for the ethernet layer
 * @param pkt the packet to send
 * @return int
 */
static int vnet_send(const struct device *ddev, struct net_pkt *pkt)
{
	struct virtio_net *vnet = (struct virtio_net *)ddev->data;
	size_t len = net_pkt_get_len(pkt);

	if (net_pkt_read(pkt, vnet->txb, len))
		return -EIO;

	return vnet_xmit(vnet, vnet->txb, len);
}

/**
 * @brief Receive a packet using virtio-net
 *
 * @param vnet the virtio net device
 * @param buf the buffer to copy data to
 * @param maxlen max length supported by the buffer
 * @return int
 */
static int vnet_receive(struct virtio_net *vnet, void *buf, size_t maxlen)
{
	uint32_t len = 0;
	uint32_t id, idx;
	uint16_t avail_idx;
	struct virtio_device *vdev = vnet->vdev;
	struct vqs *vq_rx = &vnet->vdev->vq[VQ_RX];
	void *desc_buf_addr = NULL;

	idx = virtio_modern16_to_cpu(vdev, vq_rx->used->idx);

	if (last_rx_idx == idx) {
		/* Nothing received yet */
		return 0;
	}

	id = (virtio_modern32_to_cpu(vdev, vq_rx->used->ring[last_rx_idx % vq_rx->size].id) + 1)
		% vq_rx->size;
	len = virtio_modern32_to_cpu(vdev, vq_rx->used->ring[last_rx_idx % vq_rx->size].len)
				     - vnet->net_hdr_size;
	LOG_DBG("last_rx_idx=%i, vq_rx->used->idx=%i,"
		" id=%i, len=%i", last_rx_idx, vq_rx->used->idx, id, len);

	if (len > (uint32_t)maxlen) {
		LOG_WRN("Receive buffer not big enough!\n");
		len = maxlen;
	}

	/* Get the address given by the descriptor from the device */
	desc_buf_addr = (void *) virtio_desc_addr(vdev, VQ_RX, id);
	/* Copy data to destination buffer.
	 * XXX: This is pretty useless but will remove later.
	 */
	memcpy(buf, desc_buf_addr, len);

	/* Move indices to next entries */
	last_rx_idx = last_rx_idx + 1;

	avail_idx = virtio_modern16_to_cpu(vdev, vq_rx->avail->idx);
	vq_rx->avail->ring[avail_idx % vq_rx->size] = virtio_cpu_to_modern16(vdev, id - 1);
	vq_rx->avail->idx = virtio_cpu_to_modern16(vdev, avail_idx + 1);
	barrier_dmem_fence_full();

	/* Tell HV that RX queue entry is ready */
	virtio_queue_notify(vdev, VQ_RX);
	return len;
}

/**
 * @brief Function to receive and allocate a packet
 *
 * @param vnet the virtio-net device
 * @return struct net_pkt*
 */
static struct net_pkt *vnet_rx(struct virtio_net *vnet)
{
	struct net_pkt *pkt = NULL;
	ssize_t len = vnet_receive(vnet, vnet->rxb, NET_ETH_MTU);

	if (!len) {
		goto out;
	}

	pkt = net_pkt_rx_alloc_with_buffer(vnet->iface, len, AF_UNSPEC, 0, K_NO_WAIT);

	if (!pkt) {
		LOG_ERR("Out of net_pkt buffers");
		goto out;
	}

	if (net_pkt_write(pkt, vnet->rxb, len)) {
		LOG_ERR("Out of memory for received frame");
		net_pkt_unref(pkt);
		pkt = NULL;
	}
out:
	return pkt;
}

/**
 * @brief Callback function for the rx virtqueue
 *
 * @param vdev the virtio device associated to the virtio-net device
 * This uses virtio device to respect generic API
 */
static void vnet_rx_cb(struct virtio_device *vdev)
{
	struct virtio_net *vnet = (struct virtio_net *)vdev->vq[VQ_RX].top_vdev;
	struct net_pkt *rpkt = vnet_rx(vnet);

	if (rpkt) {
		/* Push the packet to the upper level */
		net_recv_data(vnet->iface, rpkt);
	} else {
		eth_stats_update_errors_rx(vnet->iface);
	}
}

/**
 * @brief Callback function for the tx virtqueue
 *
 * @param vdev the virtio device associated to the virtio-net deviec
 */
static void vnet_tx_cb(struct virtio_device *vdev)
{
	/* The callback for the transmit queue usually does some freeing
	 * of descriptors. We do nothing for now.
	 */
	ARG_UNUSED(vdev);
}

/**
 * @brief Function to stop the device
 *
 * @param ddev the device as known to the ethernet API layer
 * @return int
 */
static int vnet_stop(const struct device *ddev)
{
	struct virtio_net *vnet = (struct virtio_net *)ddev->data;
	struct virtio_device *vdev = vnet->vdev;
	struct vqs *vq_rx = &vnet->vdev->vq[VQ_RX];
	struct vqs *vq_tx = &vnet->vdev->vq[VQ_TX];

	if (vnet->running == 0) {
		LOG_ERR("vnet device not running or terminated.");
		return 0;
	}

	LOG_DBG("Terminating vnet.");

	/* Quiesce the device */
	virtio_set_status(vdev, VIRTIO_STAT_FAILED);

	/* Reset the device */
	virtio_reset_device(vdev);

	/* Free buffer memory */
	k_free(vq_rx->buf_mem);

	virtio_queue_term(vdev, vq_rx);
	virtio_queue_term(vdev, vq_tx);

	return 0;
}

static void vnet_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct virtio_net *vnet = ((struct virtio_net *)dev->data);

	vnet->iface = iface;
	ethernet_init(iface);

	/* Assign link local address */
	net_if_set_link_addr(iface,
			     vnet->config.mac, 6, NET_LINK_ETHERNET);
}

static const struct ethernet_api vnet_if_api = {
	.iface_api.init = vnet_iface_init,
	.send		= vnet_send,
	.stop		= vnet_stop,
};

static struct virtio_net virtio_net_0 = {
	.running	= 0,
};

ETH_NET_DEVICE_INIT(vnet_0,
		    "eth-virtio-net",
		    vnet_dev_init,
		    NULL,
		    &virtio_net_0, /* Data */
		    NULL,	   /* Config */
		    CONFIG_ETH_INIT_PRIORITY,
		    &vnet_if_api,
		    NET_ETH_MTU);
