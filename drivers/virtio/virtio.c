/*
 * Copyright (c) 2011 IBM Corporation
 * Copyright (c) 2020 Hesham Almatary
 * Copyright (c) 2022 Huawei Technologies
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 */

#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/virtio/virtio.h>
#include <zephyr/drivers/virtio/virtio-config.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(virtio, LOG_LEVEL_DBG);

sys_slist_t virtio_vdevs = SYS_SLIST_STATIC_INIT(&virtio_vdevs);

/**
 * @brief Allocate the virtio_device in memory.
 *
 * @param dev The device in the Zephyr sense created by the DTS
 * @return struct virtio_device*
 */
struct virtio_device *virtio_setup_vd(const struct device *dev)
{
	struct virtio_device *vdev;

	vdev = k_malloc(sizeof(struct virtio_device));
	if (!vdev)
		return NULL;

	/* Link the `const struct device *` to `struct virtio_device` */
	vdev->dev = dev;

	return vdev;
}

/**
 * @brief Calculate the ring size which includes desc, avail and used.
 *
 * @param qsize The size of the queue items
 * @return unsigned long
 */
unsigned long virtio_vring_size(unsigned int qsize)
{
	/* `sizeof(uint16_t) * qsize` corresponds to the avail ring */
	return VQ_ALIGN(sizeof(struct vring_desc) * qsize +
			sizeof(struct vring_avail) + sizeof(uint16_t) * qsize) +
		VQ_ALIGN(sizeof(struct vring_used) +
			 sizeof(struct vring_used_elem) * qsize);
}

/**
 * @brief Get number of elements in a vring
 *
 * @param   vdev  pointer to virtio device information
 * @param   queue virtio queue number
 * @return  number of elements
 */
unsigned int virtio_get_qsize(struct virtio_device *vdev, uint32_t queue)
{
	uint32_t size = vdev->vq[queue].size;
	const struct virtio_config_api *api = vdev->dev->api;

	if (size != 0)
		return size;
	else
		return api->get_queue_max(vdev->dev, queue);
}

/**
 * @brief Get address of descriptor vring
 *
 * @param   dev  pointer to virtio device information
 * @param   queue virtio queue number
 * @return struct vring_desc * pointer to the descriptor ring
 */
struct vring_desc *virtio_get_vring_desc(struct virtio_device *dev, uint32_t queue)
{
	return dev->vq[queue].desc;
}

/**
 * @brief Get address of "available" vring
 *
 * @param   dev  pointer to virtio device information
 * @param   queue virtio queue number
 * @return  pointer to the "available" ring
 */
struct vring_avail *virtio_get_vring_avail(struct virtio_device *dev, uint32_t queue)
{
	return dev->vq[queue].avail;
}

/**
 * Get address of "used" vring
 * @param   dev  pointer to virtio device information
 * @param   queue virtio queue number
 * @return  pointer to the "used" ring
 */
struct vring_used *virtio_get_vring_used(struct virtio_device *dev, uint32_t queue)
{
	return dev->vq[queue].used;
}

/**
 * @brief Fill the virtio descriptor depending on the mode (legacy or not)
 *
 * @param vq The virtual queue to retrieve the descriptor from
 * @param id The id of the descriptor to fill
 * @param features The features of the device
 * @param addr Address to copy data from
 * @param len Length of the data
 * @param flags Flags for the descriptor
 * @param next Id when the descriptor know if it's chained
 */
void virtio_fill_desc(struct vqs *vq, int id, uint64_t features,
		      uint64_t addr, uint32_t len,
		      uint16_t flags, uint16_t next)
{
	struct vring_desc *desc;

	id %= vq->size;
	desc = &vq->desc[id];
	next %= vq->size;

	if (features & VIRTIO_F_VERSION_1) {
		if (features & VIRTIO_F_IOMMU_PLATFORM) {
			if (!vq->desc_gpas) {
				LOG_ERR("IOMMU setup has not been done!\n");
				return;
			}
		}
		desc->addr = sys_cpu_to_le64(addr);
		desc->len = sys_cpu_to_le32(len);
		desc->flags = sys_cpu_to_le16(flags);
		desc->next = sys_cpu_to_le16(next);
	} else {
		desc->addr = addr;
		desc->len = len;
		desc->flags = flags;
		desc->next = next;
	}
}

/**
 * @brief When using gpas we might need to free descriptors for further use.
 *
 * @param vq The vq from where to free the descriptors
 * @param id The id of the descriptors
 * @param features Features of the virtio_device
 */
void virtio_free_desc(struct vqs *vq, int id, uint64_t features)
{
	struct vring_desc *desc;

	id %= vq->size;
	desc = &vq->desc[id];

	if (!(features & VIRTIO_F_VERSION_1) ||
	    !(features & VIRTIO_F_IOMMU_PLATFORM))
		return;

	if (!vq->desc_gpas[id])
		return;
	/* XXX Empty as only used if IOMMU and VIRTIO_VERSION1 are supported */
	vq->desc_gpas[id] = NULL;
}

/**
 * @brief Returns the descriptor address associated to an id from the virtqueue
 *
 * @param vdev the virtio device
 * @param queue the id of the queue we want to retrieve
 * @param id the id of the descriptor we want to access
 * @return size_t
 */
size_t virtio_desc_addr(struct virtio_device *vdev, uint32_t queue, int id)
{
	struct vqs *vq = &vdev->vq[queue];

	if (vq->desc_gpas)
		return (size_t) vq->desc_gpas[id];

	return (size_t) virtio_modern64_to_cpu(vdev, vq->desc[id].addr);
}

/**
 * @brief Reset the device in the virtio sense
 *
 * @param vdev the virtio device
 */
void virtio_reset_device(struct virtio_device *vdev)
{
	const struct virtio_config_api *api = vdev->dev->api;

	api->reset(vdev->dev);
}

/**
 * @brief Notify the hypervisor that the virtqueue is ready and operational.
 * As far as I understand this only needs to be done when setting up the virtqueue
 * the first time. (Or in case of changes in the configuration)
 *
 * @param vdev the virtio device
 * @param queue the id of the queue that became ready
 */
void virtio_queue_ready(struct virtio_device *vdev, uint32_t queue)
{
	const struct virtio_config_api *api = vdev->dev->api;

	return api->ready_vq(vdev->dev, queue);
}

/**
 * @brief Send a notification to the hypervisor that there has been some operation in
 * the virtqueue.
 *
 * @param vdev the virtio device
 * @param queue the id of the queue that would like to send the notification
 */
void virtio_queue_notify(struct virtio_device *vdev, uint32_t queue)
{
	const struct virtio_config_api *api = vdev->dev->api;

	return api->notify_hp(vdev->dev, queue);
}

/**
 * @brief Register the queue address on the virtio device. This chunk of memory comes from
 * Zephyr.
 *
 * @param vdev the virtio device
 * @param vq the virtqueue we want to register
 */
static void virtio_set_qaddr(struct virtio_device *vdev, struct vqs *vq)
{
	const struct virtio_config_api *api = vdev->dev->api;

	api->setup_vq(vdev->dev, vq);
}

/**
 * @brief Initialize the virtqueue
 *
 * @param vdev the virtio device
 * @param id the id of the queue we want to initialize
 * @param callback the callback function to register for the virtqueue
 * @param arg private argument, used to link end virtio device (e.g. virito-net) to vdev
 * @param size size of the queue we want to initialize
 * @return struct vqs*
 */
struct vqs *virtio_queue_init(struct virtio_device *vdev, uint32_t id,
			      void (*callback)(struct virtio_device *),
			      void *arg, uint64_t size)
{
	struct vqs *vq;

	if (id > ARRAY_SIZE(vdev->vq)) {
		printk("Queue index is too big!\n");
		return NULL;
	}
	vq = &vdev->vq[id];

	memset(vq, 0, sizeof(*vq));

	/* XXX Function `virtio_get_qsize()` was used here.
	 * The problem is that the theoretical maximum can be different from the actual
	 * impementation (eg. 1024 max theoretical, 256 on the actual virtio device).
	 */
	vq->size = size;
	/* We allocate a chunk of memory to put the desc, avail and used */
	vq->desc = k_aligned_alloc(4096, virtio_vring_size(vq->size));
	if (!vq->desc) {
		LOG_ERR("vq->desc: memory allocation failed!\n");
		return NULL;
	}

	/* Move the avail pointer after desc entries */
	vq->avail = (void *) ((size_t) vq->desc + vq->size * sizeof(struct vring_desc));

	/* Move the used pointer after the avail entries */
	vq->used = (void *) ((size_t) VQ_ALIGN((size_t) vq->avail +
		sizeof(struct vring_avail) +
		sizeof(uint16_t) * vq->size));

	memset(vq->desc, 0, virtio_vring_size(vq->size));
	vq->callback	= callback;
	vq->top_vdev	= arg;
	vq->index	= id;
	virtio_set_qaddr(vdev, vq);

	vq->avail->flags = virtio_cpu_to_modern16(vdev, VRING_AVAIL_F_NO_INTERRUPT);
	vq->avail->idx	 = 0;
	if (vdev->features & VIRTIO_F_IOMMU_PLATFORM) {
		LOG_ERR("vq->desc_gpas: IOMMU not implemented yet");
		virtio_queue_term(vdev, vq);
		return NULL;
	}

	vq->ready = true;

	return vq;
}

/**
 * @brief Terminate the virtqueue
 *
 * @param vdev the virtio device
 * @param vq the virtqueue to be terminated
 */
void virtio_queue_term(struct virtio_device *vdev, struct vqs *vq)
{
	const struct virtio_config_api *api = vdev->dev->api;
	int key;

	if (vq->desc_gpas) {
		uint32_t i;

		for (i = 0; i < vq->size; ++i)
			virtio_free_desc(vq, i, vdev->features);

		k_free(vq->desc_gpas);
	}

	if (vq->desc) {
		k_free(vq->desc);
	}
	memset(vq, 0, sizeof(*vq));
	/* Disable interrupts so we don't delete a queue that might get notified */
	key = irq_lock();
	api->del_vq(vdev->dev, vq->index);
	irq_unlock(key);
}


/**
 * @brief Set the status bit(s) on the virtio device
 *
 * @param vdev the virtio device
 * @param status the status bit(s) to be set
 */
void virtio_set_status(struct virtio_device *vdev, uint8_t status)
{
	const struct virtio_config_api *api = vdev->dev->api;

	api->set_status(vdev->dev, status);
}


/**
 * @brief Read the status bits from the virtio device
 *
 * @param vdev the virtio device
 * @return uint8_t the status bit that were set on the device
 */
uint8_t virtio_get_status(struct virtio_device *vdev)
{
	const struct virtio_config_api *api = vdev->dev->api;

	return api->get_status(vdev->dev);
}

/**
 * @brief Set the guest feature bits so the host knows what we are capable of supporting
 *
 * @param vdev the virtio device
 * @param features the features we want to set
 */
void virtio_set_guest_features(struct virtio_device *vdev, uint64_t features)
{
	const struct virtio_config_api *api = vdev->dev->api;

	/* We are trying to set without negotiation, this is for legacy mode only */
	if (features & VIRTIO_F_VERSION_1) {
		LOG_ERR("Non legacy version 0x%llx", features);
		return;
	}

	api->set_guest_features(vdev->dev, features);
}

/**
 * @brief Read the host features
 *
 * @param vdev the virtio device
 * @return uint64_t the features that the host supports
 */
uint64_t virtio_get_host_features(struct virtio_device *vdev)
{
	const struct virtio_config_api *api = vdev->dev->api;

	return api->get_host_features(vdev->dev);
}

/**
 * @brief The negotiation function between guest and host to determine
 * which features will be supported for the virtio device
 *
 * @param vdev the virtio device
 * @param wanted_features some necessary features we want to support.
 * This comes from the final virtio device (e.g. virtio-net)
 * @return int
 */
int virtio_negotiate_guest_features(struct virtio_device *vdev, uint64_t wanted_features)
{
	const struct virtio_config_api *api = vdev->dev->api;
	uint64_t host_features = api->get_host_features(vdev->dev);
	uint64_t guest_features = wanted_features;
	uint8_t status;

	/* We enter negotiation with some features wanted from the driver
	 *
	 * XXX: Testing for device independent features we may want.
	 * We currently only test for version.
	 *
	 * VIRTIO_F_RING_INDIRECT_DESC
	 * VIRTIO_F_RING_EVENT_IDX
	 * VIRTIO_F_IOMMU_PLATFORM
	 * VIRTIO_F_NOTIFICATION_DATA
	 */
	if (host_features & VIRTIO_F_VERSION_1)
		guest_features |= VIRTIO_F_VERSION_1;

	api->set_guest_features(vdev->dev, guest_features);
	host_features = api->get_host_features(vdev->dev);
	if ((host_features & guest_features) != guest_features) {
		LOG_ERR("Features error %llx", guest_features);
		return -EIO;
	}

	/* After setting the features, we set the status of the driver */
	status = api->get_status(vdev->dev);
	status |= VIRTIO_STAT_FEATURES_OK;
	api->set_status(vdev->dev, status);

	/* Read back to verify the STAT_FEATURES_OK bit */
	status = api->get_status(vdev->dev);
	if (!(status & VIRTIO_STAT_FEATURES_OK))
		return -EIO;

	/* Write the guest features supported in kernel space */
	vdev->features = guest_features;

	return 0;
}


/**
 * @brief Depending on the virtio device, there is a field containing additional information
 * This functions allows to read that field starting at a certain offset.
 *
 * @param vdev The virtio device
 * @param offset The offset where the config reading should start
 * @param buf The buffer to put the data that is being read
 * @param len The len of the data to read in bytes
 */
void virtio_get_config(struct virtio_device *vdev, uint32_t offset,
			   uintptr_t buf, uint32_t len)
{
	const struct virtio_config_api *api = vdev->dev->api;

	api->get_config(vdev->dev, offset, buf, len);
}

/**
 * @brief Test if the virtio_device has a certain feature bit
 *
 * @param vdev the virtio_device
 * @param fbit the feature bit we want to test
 * @return true when the feature bit is present
 * @return false otherwise
 */
bool virtio_host_has_feature(struct virtio_device *vdev, unsigned int fbit)
{
	if (virtio_get_host_features(vdev) & fbit)
		return true;
	return false;
}
