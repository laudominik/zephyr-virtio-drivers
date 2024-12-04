/*
 * Copyright (c) 2014 Ruslan Bukin <br@bsdpad.com>
 * Copyright (c) 2022 Huawei Technologies
 *
 * SPDX-License-Identifier: BSD-2 clause
 */

#ifndef	ZEPHYR_INCLUDE_DRIVERS_VIRTIO_MMIO_H_
#define	ZEPHYR_INCLUDE_DRIVERS_VIRTIO_MMIO_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief VirtIO MMIO Interface
 * @defgroup virtio_mmio_interface VirtIO MMIO Interface
 * @ingroup io_interfaces
 * @{
 */

#define	VIRTIO_MMIO_MAGIC_VALUE		0x000
#define	VIRTIO_MMIO_VERSION		0x004
#define	VIRTIO_MMIO_DEVICE_ID		0x008
#define	VIRTIO_MMIO_VENDOR_ID		0x00c
#define	VIRTIO_MMIO_HOST_FEATURES	0x010
#define	VIRTIO_MMIO_HOST_FEATURES_SEL	0x014
#define	VIRTIO_MMIO_GUEST_FEATURES	0x020
#define	VIRTIO_MMIO_GUEST_FEATURES_SEL	0x024
#define	VIRTIO_MMIO_GUEST_PAGE_SIZE	0x028	/* version 1 only */
#define	VIRTIO_MMIO_QUEUE_SEL		0x030
#define	VIRTIO_MMIO_QUEUE_NUM_MAX	0x034
#define	VIRTIO_MMIO_QUEUE_NUM		0x038
#define	VIRTIO_MMIO_QUEUE_ALIGN		0x03c	/* version 1 only */
#define	VIRTIO_MMIO_QUEUE_PFN		0x040	/* version 1 only */
#define	VIRTIO_MMIO_QUEUE_READY		0x044	/* requires version 2 */
#define	VIRTIO_MMIO_QUEUE_NOTIFY	0x050
#define	VIRTIO_MMIO_INTERRUPT_STATUS	0x060
#define	VIRTIO_MMIO_INTERRUPT_ACK	0x064
#define	VIRTIO_MMIO_STATUS		0x070
#define	VIRTIO_MMIO_QUEUE_DESC_LOW	0x080	/* requires version 2 */
#define	VIRTIO_MMIO_QUEUE_DESC_HIGH	0x084	/* requires version 2 */
#define	VIRTIO_MMIO_QUEUE_AVAIL_LOW	0x090	/* requires version 2 */
#define	VIRTIO_MMIO_QUEUE_AVAIL_HIGH	0x094	/* requires version 2 */
#define	VIRTIO_MMIO_QUEUE_USED_LOW	0x0a0	/* requires version 2 */
#define	VIRTIO_MMIO_QUEUE_USED_HIGH	0x0a4	/* requires version 2 */
#define	VIRTIO_MMIO_CONFIG_GENERATION	0x100	/* requires version 2 */
#define	VIRTIO_MMIO_CONFIG		0x100
#define	VIRTIO_MMIO_INT_VRING		(1 << 0)
#define	VIRTIO_MMIO_INT_CONFIG		(1 << 1)
#define	VIRTIO_MMIO_VRING_ALIGN		4096

#define VIRTIO_MMIO_LEGACY_PG_SIZE	0x1000

struct virtio_mmio_generic_config {
	DEVICE_MMIO_ROM;
};
struct virtio_mmio_generic_data {
	DEVICE_MMIO_RAM;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_VIRTIO_MMIO_H_ */
