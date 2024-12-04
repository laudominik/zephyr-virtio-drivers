/*
 * Copyright (c) 2011 IBM Corporation
 * Copyright (c) 2022 Huawei Technologies
 *
 * SPDX-License-Identifier: BSD-2 clause
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_VIRTIO_H_
#define ZEPHYR_INCLUDE_DRIVERS_VIRTIO_H_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Originally in a biteorder.h header */
typedef uint16_t le16;
typedef uint32_t le32;
typedef uint64_t le64;

/* Device id definitions */
#define VIRTIO_DEV_ID_NET		1
#define VIRTIO_DEV_ID_SERIAL		3

/* Device status bits */
#define VIRTIO_STAT_ACKNOWLEDGE		1
#define VIRTIO_STAT_DRIVER		2
#define VIRTIO_STAT_DRIVER_OK		4
#define VIRTIO_STAT_FEATURES_OK		8
#define VIRTIO_STAT_NEEDS_RESET		0x40
#define VIRTIO_STAT_FAILED		0x80

/* VIRTIO 1.0 Device independent feature bits */
#define VIRTIO_F_RING_INDIRECT_DESC	BIT(28)
#define VIRTIO_F_RING_EVENT_IDX		BIT(29)
#define VIRTIO_F_VERSION_1		((uint64_t) BIT(32))
#define VIRTIO_F_IOMMU_PLATFORM		((uint64_t) BIT(33))
#define VIRTIO_F_NOTIFICATION_DATA	((uint64_t) BIT(38))

#define VIRTIO_TIMEOUT			5000 /* 5 sec timeout */

/* Definitions for vring_desc.flags */
#define VRING_DESC_F_NEXT		1 /* buffer continues via the next field */
#define VRING_DESC_F_WRITE		2 /* buffer is write-only (otherwise read-only) */
#define VRING_DESC_F_INDIRECT		4 /* buffer contains a list of buffer descriptors */

#ifdef VIRTIO_USE_PCI
#error "VIRTIO_USE_PCI isn't supported by Zephyr yet"
#endif

#ifndef VIRTIO_USE_MMIO
#define VIRTIO_USE_MMIO 1
#endif

/* List of existing devices (virtio.c) */
extern sys_slist_t virtio_vdevs;

struct virtio_device;

/* The Host uses this in used->flags to advise the Guest: don't kick me when
 * you add a buffer.  It's unreliable, so it's simply an optimization.  Guest
 * will still kick if it's out of buffers.
 */
#define VRING_USED_F_NO_NOTIFY  1
/* The Guest uses this in avail->flags to advise the Host: don't interrupt me
 * when you consume a buffer.  It's unreliable, so it's simply an
 * optimization.
 */
#define VRING_AVAIL_F_NO_INTERRUPT  1

/* Descriptor table entry - see Virtio Spec chapter 2.3.2 */
struct vring_desc {
	uint64_t addr;		/* Address (guest-physical) */
	uint32_t len;		/* Length */
	uint16_t flags;		/* The flags as indicated above */
	uint16_t next;		/* Next field if flags & NEXT */
};

/* Available ring - see Virtio Spec chapter 2.6.6 */
struct vring_avail {
	uint16_t flags;
	uint16_t idx;
	uint16_t ring[];
};

struct vring_used_elem {
	uint32_t id;	/* Index of start of used descriptor chain */
	uint32_t len;	/* Total length of the descriptor chain which was used */
};

struct vring_used {
	uint16_t flags;
	uint16_t idx;
	struct vring_used_elem ring[];
};

/* Structure shared with SLOF and is 16bytes */
struct virtio_cap {
	void *addr;
	uint8_t bar;
	uint8_t is_io;
	uint8_t cap_id;
};

struct vqs {
	bool ready;
	void (*callback)(struct virtio_device *vdev);
	void *top_vdev; /* Pointer to the top device e.g. virtio_net */
	void *buf_mem;
	struct vring_desc *desc;
	struct vring_avail *avail;
	struct vring_used *used;
	void **desc_gpas; /* to get gpa from desc->addr (which is ioba) */
	uint64_t bus_desc;
	uint32_t index;
	uint32_t size;
};

struct virtio_device {
	const struct device	*dev;
	uint32_t		version;
	uint8_t			dev_type;
	uint64_t		features;
	struct vqs		vq[3];

	/* Chain the virtio devices */
	sys_snode_t		node;
};

/* Parts of the virtqueue are aligned on a 4096 byte page boundary */
#define VQ_ALIGN(addr)	(((addr) + 0xfff) & ~0xfff)

static inline uint16_t virtio_cpu_to_modern16(struct virtio_device *dev, uint16_t val)
{
	return (dev->features & VIRTIO_F_VERSION_1) ? sys_cpu_to_le16(val) : val;
}

static inline uint32_t virtio_cpu_to_modern32(struct virtio_device *dev, uint32_t val)
{
	return (dev->features & VIRTIO_F_VERSION_1) ? sys_cpu_to_le32(val) : val;
}

static inline uint64_t virtio_cpu_to_modern64(struct virtio_device *dev, uint64_t val)
{
	return (dev->features & VIRTIO_F_VERSION_1) ? sys_cpu_to_le64(val) : val;
}

static inline uint16_t virtio_modern16_to_cpu(struct virtio_device *dev, uint16_t val)
{
	return (dev->features & VIRTIO_F_VERSION_1) ? sys_le16_to_cpu(val) : val;
}

static inline uint32_t virtio_modern32_to_cpu(struct virtio_device *dev, uint32_t val)
{
	return (dev->features & VIRTIO_F_VERSION_1) ? sys_le32_to_cpu(val) : val;
}

static inline uint64_t virtio_modern64_to_cpu(struct virtio_device *dev, uint64_t val)
{
	return (dev->features & VIRTIO_F_VERSION_1) ? sys_le64_to_cpu(val) : val;
}

extern unsigned long virtio_vring_size(unsigned int qsize);
extern unsigned int virtio_get_qsize(struct virtio_device *vdev, uint32_t queue);
extern struct vring_desc *virtio_get_vring_desc(struct virtio_device *vdev, uint32_t queue);
extern struct vring_avail *virtio_get_vring_avail(struct virtio_device *vdev, uint32_t queue);
extern struct vring_used *virtio_get_vring_used(struct virtio_device *vdev, uint32_t queue);
extern void virtio_fill_desc(struct vqs *vq, int id, uint64_t features,
			     uint64_t addr, uint32_t len,
			     uint16_t flags, uint16_t next);
extern void virtio_free_desc(struct vqs *vq, int id, uint64_t features);
extern size_t virtio_desc_addr(struct virtio_device *vdev, uint32_t queue, int id);
extern struct vqs *virtio_queue_init(struct virtio_device *vdev, uint32_t queue,
					void (*callback)(struct virtio_device *),
					void *arg, uint64_t size);
extern void virtio_queue_term(struct virtio_device *vdev, struct vqs *vq);
extern void virtio_queue_ready(struct virtio_device *vdev, uint32_t queue);
extern void virtio_queue_notify(struct virtio_device *vdev, uint32_t queue);

extern struct virtio_device *virtio_setup_vd(const struct device *dev);
extern void virtio_reset_device(struct virtio_device *vdev);
extern void virtio_set_status(struct virtio_device *vdev, uint8_t status);
extern uint8_t virtio_get_status(struct virtio_device *vdev);
extern void virtio_set_guest_features(struct virtio_device *vdev, uint64_t features);
extern uint64_t virtio_get_host_features(struct virtio_device *vdev);
extern int virtio_negotiate_guest_features(struct virtio_device *vdev, uint64_t guest_features);
extern void virtio_get_config(struct virtio_device *vdev, uint32_t offset,
			      uintptr_t buf, uint32_t size);

extern bool virtio_host_has_feature(struct virtio_device *vdev, unsigned int fbit);

#ifdef VIRTIO_USE_MMIO
extern void virtio_mmio_print_configs(struct virtio_device *vdev);
#endif


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_VIRTIO_H_ */
