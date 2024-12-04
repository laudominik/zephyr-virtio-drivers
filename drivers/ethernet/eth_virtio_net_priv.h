/*
 * Copyright (c) 2022 Huawei Technologies
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 */

#ifndef ETH_VIRTIO_NET_PRIV_H_
#define ETH_VIRTIO_NET_PRIV_H_

#ifdef __cplusplus
extern "C" {
#endif

/* VIRTIO NET Status bits */
#define VIRTIO_NET_S_LINK_UP	1
#define VIRTIO_NET_S_ANNOUNCE	2

/* VIRTIO_NET Feature bits */
#define VIRTIO_NET_F_CSUM		BIT64(0)
#define VIRTIO_NET_F_GUEST_CSUM		BIT64(1)
#define VIRTIO_NET_F_MTU		BIT64(3)
#define VIRTIO_NET_F_MAC		BIT64(5)
#define VIRTIO_NET_F_STATUS		BIT64(16)
#define VIRTIO_NET_F_CTRL_VQ		BIT64(17)
#define VIRTIO_NET_F_CTRL_RX		BIT64(18)
#define VIRTIO_NET_F_GUEST_ANNOUNCE	BIT64(21)
#define VIRTIO_NET_F_MQ			BIT64(22)

/* This is the default value of the queue size from QEMU */
#define RX_QUEUE_SIZE		128
#define BUFFER_ENTRY_SIZE	1514

enum {
	VQ_RX = 0,	/* Receive Queue */
	VQ_TX = 1,	/* Transmit Queue */
};

struct virtio_net_config {
	uint8_t		mac[6];
	uint16_t	status;
	uint16_t	max_virtqueues_pairs;
	uint16_t	mtu;
};

struct virtio_net {
	uint8_t				txb[NET_ETH_MTU];
	uint8_t				rxb[NET_ETH_MTU];
	bool				running;
	struct virtio_device		*vdev;
	struct virtio_net_config	config;

	/* Putting some pointers to useful devices */
	const struct device		*eth_dev; /* Device created by eth layer */
	struct net_if			*iface;
	uint32_t			net_hdr_size;
};

/* See Virtio Spec, appendix C, "Device Operation" */
struct virtio_net_hdr {
	uint8_t		flags;
	uint8_t		gso_type;
	uint16_t	hdr_len;
	uint16_t	gso_size;
	uint16_t	csum_start;
	uint16_t	csum_offset;
	/* uint16_t  num_buffers;	Only if VIRTIO_NET_F_MRG_RXBUF */
};

struct virtio_net_hdr_v1 {
	uint8_t		flags;
	uint8_t		gso_type;
	le16		hdr_len;
	le16		gso_size;
	le16		csum_start;
	le16		csum_offset;
	le16		num_buffers;
};

#ifdef __cplusplus
}
#endif

#endif /* ETH_VIRTIO_NET_PRIV_H_ */
