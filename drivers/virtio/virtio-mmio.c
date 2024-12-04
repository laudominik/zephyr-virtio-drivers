/*
 * Copyright (c) 2022 Huawei Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT virtio_mmio

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(virtio_mmio, LOG_LEVEL_DBG);

#include <zephyr/drivers/virtio/virtio.h>
#include <zephyr/drivers/virtio/virtio-config.h>
#include <zephyr/drivers/virtio/virtio-mmio.h>

static uint64_t vio_mmio_get_host_features(const struct device *dev);

void virtio_mmio_print_configs(struct virtio_device *vdev)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(vdev->dev);
	uint64_t features = virtio_get_host_features(vdev);

	LOG_DBG("MagicValue:\t\t 0x%" PRIx32, sys_read32(base_address + VIRTIO_MMIO_MAGIC_VALUE));
	LOG_DBG("Version:\t\t 0x%" PRIx32, sys_read32(base_address + VIRTIO_MMIO_VERSION));
	LOG_DBG("DeviceID:\t\t 0x%" PRIx32, sys_read32(base_address + VIRTIO_MMIO_DEVICE_ID));
	LOG_DBG("VendorID:\t\t 0x%" PRIx32, sys_read32(base_address +  VIRTIO_MMIO_VENDOR_ID));

	LOG_DBG("DeviceFeatures:\t 0x%" PRIx64, features);
	if (!(features & VIRTIO_F_VERSION_1))
		LOG_DBG("PageSize:\t\t 0x%x",
			sys_read32(base_address + VIRTIO_MMIO_GUEST_PAGE_SIZE));

	sys_write32(0x0, base_address + VIRTIO_MMIO_QUEUE_SEL);
	LOG_DBG("QueueNumMax[0]:\t 0x%" PRIx32,
		sys_read32(base_address + VIRTIO_MMIO_QUEUE_NUM_MAX));
	LOG_DBG("QueueReady[0]:\t 0x%" PRIx32, sys_read32(base_address +  VIRTIO_MMIO_QUEUE_READY));
	sys_write32(0x1, base_address + VIRTIO_MMIO_QUEUE_SEL);
	LOG_DBG("QueueNumMax[1]:\t 0x%" PRIx32,
		sys_read32(base_address +  VIRTIO_MMIO_QUEUE_NUM_MAX));
	LOG_DBG("QueueReady[1]:\t 0x%" PRIx32, sys_read32(base_address +  VIRTIO_MMIO_QUEUE_READY));
	if (!(features & VIRTIO_F_VERSION_1))
		LOG_DBG("QueuePFN:\t\t 0x%" PRIx32,
			sys_read32(base_address + VIRTIO_MMIO_QUEUE_PFN));

	LOG_DBG("InterruptStatus:\t 0x%" PRIx32,
		sys_read32(base_address + VIRTIO_MMIO_INTERRUPT_STATUS));
	LOG_DBG("Status:\t\t 0x%" PRIx32, sys_read32(base_address +  VIRTIO_MMIO_STATUS));
	LOG_DBG("ConfigGeneration:\t 0x%" PRIx32, sys_read32(base_address +
		VIRTIO_MMIO_CONFIG_GENERATION));
	LOG_DBG("Config:\t\t 0x%" PRIx32, sys_read32(base_address + VIRTIO_MMIO_CONFIG));
}


static uint8_t vio_mmio_get_version(const struct device *dev)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	return sys_read32(base_address + VIRTIO_MMIO_VERSION) & 0xff;
}

static void vio_mmio_get_config(const struct device *dev, uint32_t offset,
				uintptr_t buf, uint32_t len)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);
	uint8_t b;
	uint16_t w;
	uint32_t l;

	base_address += VIRTIO_MMIO_CONFIG;

	if (vio_mmio_get_version(dev) == 1) {
		uint8_t *ptr = (uint8_t *)buf;
		int i;

		for (i = 0; i < len; i++)
			ptr[i] = sys_read8(base_address + offset + i);
		return;
	}

	switch (len) {
	case 1:
		b = sys_read8(base_address + offset);
		memcpy((void *)buf, &b, sizeof(b));
		break;
	case 2:
		w = sys_cpu_to_le16(sys_read16(base_address + offset));
		memcpy((void *)buf, &w, sizeof(w));
		break;
	case 4:
		l = sys_cpu_to_le32(sys_read32(base_address + offset));
		memcpy((void *)buf, &l, sizeof(l));
		break;
	case 8:
		l = sys_cpu_to_le32(sys_read32(base_address + offset));
		memcpy((void *)buf, &l, sizeof(l));
		l = sys_cpu_to_le32(sys_read32(base_address + offset + sizeof(l)));
		memcpy((void *)(buf + sizeof(l)), &l, sizeof(l));
		break;
	default:
		LOG_ERR("Wrong size to read the config");
		break;
	}
}

static void vio_mmio_set_config(const struct device *dev, uint32_t offset,
				const uintptr_t buf, uint32_t len)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);
	uint8_t b;
	uint16_t w;
	uint32_t l;

	base_address += VIRTIO_MMIO_CONFIG;

	if (vio_mmio_get_version(dev) == 1) {
		const uint8_t *ptr = (uint8_t *)buf;
		int i;

		for (i = 0; i < len; i++)
			sys_write8(ptr[i], base_address + offset + i);
		return;
	}

	switch (len) {
	case 1:
		memcpy(&b, (void *)buf, sizeof(b));
		sys_write8(b, base_address + offset);
		break;
	case 2:
		memcpy(&w, (void *)buf, sizeof(w));
		sys_write16(sys_le16_to_cpu(w), base_address + offset);
		break;
	case 4:
		memcpy(&l, (void *)buf, sizeof(l));
		sys_write32(sys_le32_to_cpu(l), base_address + offset);
		break;
	case 8:
		memcpy(&l, (void *)buf, sizeof(l));
		sys_write32(sys_le32_to_cpu(l), base_address + offset);
		memcpy(&l, (void *)(buf + sizeof(l)), sizeof(l));
		sys_write32(sys_le32_to_cpu(l), base_address + offset + sizeof(l));
		break;
	default:
		LOG_ERR("Wrong size to write a config");
		break;
	}
}

static uint32_t vio_mmio_generation(const struct device *dev)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	if (vio_mmio_get_version(dev) == 1)
		return 0;
	else
		return sys_read32(base_address + VIRTIO_MMIO_CONFIG_GENERATION);
}

static uint8_t vio_mmio_get_status(const struct device *dev)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	return sys_read32(base_address +  VIRTIO_MMIO_STATUS) & 0xff;
}

static void vio_mmio_set_status(const struct device *dev, uint8_t status)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	__ASSERT(status != 0, "Status should never be set to 0", __func__);

	sys_write32(status, base_address + VIRTIO_MMIO_STATUS);
}

static void vio_mmio_reset(const struct device *dev)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	sys_write32(0, base_address + VIRTIO_MMIO_STATUS);
}

static uint32_t vio_mmio_get_queue_max(const struct device *dev, uint32_t queue)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	sys_write32(queue, base_address + VIRTIO_MMIO_QUEUE_SEL);
	return sys_read32(base_address + VIRTIO_MMIO_QUEUE_NUM_MAX);
}

static void vio_mmio_setup_vq(const struct device *dev, struct vqs *vq)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);
	uint32_t queue = vq->index;

	sys_write32(queue, base_address + VIRTIO_MMIO_QUEUE_SEL);

	if (vio_mmio_get_host_features(dev) & VIRTIO_F_VERSION_1) {
		uint64_t q_desc  = (uint64_t)vq->desc;
		uint64_t q_avail = (uint64_t)vq->avail;
		uint64_t q_used  = (uint64_t)vq->used;

		sys_write32((uint32_t)(q_desc >> 32),
			    base_address + VIRTIO_MMIO_QUEUE_DESC_HIGH);
		sys_write32((uint32_t)q_desc,
			    base_address + VIRTIO_MMIO_QUEUE_DESC_LOW);

		sys_write32((uint32_t)(q_avail >> 32),
			    base_address +  VIRTIO_MMIO_QUEUE_AVAIL_HIGH);
		sys_write32((uint32_t)q_avail,
			    base_address + VIRTIO_MMIO_QUEUE_AVAIL_LOW);

		sys_write32((uint32_t)(q_used >> 32),
			    base_address + VIRTIO_MMIO_QUEUE_USED_HIGH);
		sys_write32((uint32_t)q_used,
			    base_address + VIRTIO_MMIO_QUEUE_USED_LOW);

	} else {
		/* XXX: The following writes are only used by legacy mode.
		 * Since it is not the default for Zephyr, leaving the values as is
		 * for now.
		 */
		uint64_t qaddr = (uint64_t)vq;

		sys_write32((uint32_t)(qaddr >> 12),
			    base_address + VIRTIO_MMIO_QUEUE_PFN);
		sys_write32(1024,
			    base_address + VIRTIO_MMIO_QUEUE_NUM);
		sys_write32(VIRTIO_MMIO_LEGACY_PG_SIZE,
			    base_address + VIRTIO_MMIO_QUEUE_ALIGN);
	}
}

static void vio_mmio_set_ready_vq(const struct device *dev, uint32_t queue)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	sys_write32(queue, base_address + VIRTIO_MMIO_QUEUE_SEL);
	sys_write32(0x1, base_address + VIRTIO_MMIO_QUEUE_READY);
}

static void vio_mmio_notify_hp(const struct device *dev, uint32_t queue)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	sys_write32(queue, base_address + VIRTIO_MMIO_QUEUE_NOTIFY);
}

static void vio_mmio_del_vq(const struct device *dev, uint32_t queue)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	/* Select and deactivate the queue */
	sys_write32(queue, base_address + VIRTIO_MMIO_QUEUE_SEL);
	if (vio_mmio_get_version(dev) == 1) {
		/* Legacy version */
		sys_write32(0, base_address + VIRTIO_MMIO_QUEUE_PFN);
		sys_write32(0, base_address + VIRTIO_MMIO_QUEUE_NUM);
	} else {
		sys_write32(0, base_address + VIRTIO_MMIO_QUEUE_READY);
	}
}

static uint64_t vio_mmio_get_host_features(const struct device *dev)
{
	uint64_t features;
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	sys_write32(1, base_address + VIRTIO_MMIO_HOST_FEATURES_SEL);
	features = sys_read32(base_address + VIRTIO_MMIO_HOST_FEATURES);
	features <<= 32;

	sys_write32(0, base_address + VIRTIO_MMIO_HOST_FEATURES_SEL);
	features |= sys_read32(base_address + VIRTIO_MMIO_HOST_FEATURES);

	return features;
}

static void vio_mmio_set_guest_features(const struct device *dev,
					uint64_t features)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	/* XXX: Linux here clears unwanted bits for the vring
	 * They also try to detect cases where devices are "mixed".
	 */
	sys_write32(1, base_address + VIRTIO_MMIO_GUEST_FEATURES_SEL);
	sys_write32((uint32_t)(features >> 32),
		    base_address + VIRTIO_MMIO_GUEST_FEATURES);
	sys_write32(0, base_address + VIRTIO_MMIO_GUEST_FEATURES_SEL);
	sys_write32((uint32_t)features,
		    base_address + VIRTIO_MMIO_GUEST_FEATURES);
}

static const struct virtio_config_api vio_mmio_api = {
	.get_config		= vio_mmio_get_config,
	.set_config		= vio_mmio_set_config,
	.generation		= vio_mmio_generation,
	.get_status		= vio_mmio_get_status,
	.set_status		= vio_mmio_set_status,
	.reset			= vio_mmio_reset,
	.get_queue_max		= vio_mmio_get_queue_max,
	.setup_vq		= vio_mmio_setup_vq,
	.ready_vq		= vio_mmio_set_ready_vq,
	.notify_hp		= vio_mmio_notify_hp,
	.del_vq			= vio_mmio_del_vq,
	.get_host_features	= vio_mmio_get_host_features,
	.set_guest_features	= vio_mmio_set_guest_features,
};

static uint8_t vio_mmio_get_irq_status(const struct device *dev)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	return (uint8_t)sys_read32(base_address + VIRTIO_MMIO_INTERRUPT_STATUS);
}

static void vio_mmio_set_irq_ack(const struct device *dev, uint8_t ack)
{
	mem_addr_t base_address = DEVICE_MMIO_GET(dev);

	sys_write32(ack, base_address + VIRTIO_MMIO_INTERRUPT_ACK);
}

static int virtio_mmio_isr(struct virtio_device *vdev)
{
	uint8_t status;

	/* Read and acknowledge interrupts */
	status = vio_mmio_get_irq_status(vdev->dev);
	vio_mmio_set_irq_ack(vdev->dev, status);

	/* The configuration of the device changed */
	if (unlikely(status & VIRTIO_MMIO_INT_CONFIG)) {
		LOG_WRN("Configuration of the device changed, not handled");
	}

	/* There has been changed on the vrings */
	if (likely(status & VIRTIO_MMIO_INT_VRING)) {
		int i, key;
		/* We currently support 3 vqs only */
		for (i = 0; i < 3; i++) {
			if (vdev->vq[i].ready) {
				key = irq_lock();
				vdev->vq[i].callback(vdev);
				irq_unlock(key);
			}
		}
	}
	return 0;
}

static int virtio_mmio_init(const struct device *dev)
{
	struct virtio_device *vdev;
	mem_addr_t base_address;
	uint32_t magic;

	/* Map the device so that accesses to it do not fault */
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	/* Spawn the virtio_device struct in memory space */
	vdev = virtio_setup_vd(dev);
	if (!vdev) {
		LOG_ERR("Could not allocate virtio_device instance");
		return -ENOMEM;
	}

	/* Probe and check for irregularities */
	base_address = DEVICE_MMIO_GET(dev);
	magic = sys_read32(base_address + VIRTIO_MMIO_MAGIC_VALUE);
	if (magic != ('v' | 'i' << 8 | 'r' << 16 | 't' << 24)) {
		LOG_ERR("Wrong magic value 0x%08x", magic);
		k_free(vdev);
		return -ENODEV;
	}

	vdev->version = sys_read32(base_address + VIRTIO_MMIO_VERSION);
	if (vdev->version < 1 || vdev->version > 2) {
		LOG_ERR("Version %d not supported", vdev->version);
		k_free(vdev);
		return -ENXIO;
	}

	if (vio_mmio_get_version(dev) == 1) {
		/* Legacy mode requires to write the guest page size */
		sys_write32(VIRTIO_MMIO_LEGACY_PG_SIZE,
			    base_address + VIRTIO_MMIO_GUEST_PAGE_SIZE);
	}

	/* We do a minimal setup when probing/initing the device */
	vdev->dev_type = sys_read32(base_address + VIRTIO_MMIO_DEVICE_ID);
	vdev->features = vio_mmio_get_host_features(dev) & VIRTIO_F_VERSION_1;
	/* Append to the list of virtio devices */
	sys_slist_append(&virtio_vdevs, &vdev->node);

	/* Interrupts initialized dynamically */
	irq_connect_dynamic(DT_INST_IRQN(0), 0,
		(void (*)(const void *))virtio_mmio_isr, vdev, 0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

static const struct virtio_mmio_generic_config virtio_mmio_config_0 = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
};

static struct virtio_mmio_generic_data virtio_mmio_data_0;

/* XXX: Replace this single instance with a better macro */
DEVICE_DT_INST_DEFINE(0,
	virtio_mmio_init, "virtio_mmio_0",
	&virtio_mmio_data_0, &virtio_mmio_config_0,
	POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	&vio_mmio_api);
