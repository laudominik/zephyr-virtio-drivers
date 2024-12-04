/*
 * Copyright (c) 2022 Huawei Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_VIRTIO_CONFIG_H_
#define ZEPHYR_INCLUDE_DRIVERS_VIRTIO_CONFIG_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*vio_get_config)(const struct device *dev, uint32_t offset,
				    uintptr_t buf, uint32_t len);

typedef void (*vio_set_config)(const struct device *dev, uint32_t offset,
				    const uintptr_t buf, uint32_t len);

typedef uint32_t (*vio_generation)(const struct device *dev);

typedef uint8_t (*vio_get_status)(const struct device *dev);

typedef void (*vio_set_status)(const struct device *dev, uint8_t status);

typedef void (*vio_reset)(const struct device *dev);

typedef uint32_t (*vio_get_queue_max)(const struct device *dev, uint32_t queue);

typedef void (*vio_setup_vq)(const struct device *dev, struct vqs *vq);

typedef void (*vio_set_ready_vq)(const struct device *dev, uint32_t queue);

typedef void (*vio_notify_hp)(const struct device *dev, uint32_t queue);

typedef void (*vio_del_vq)(const struct device *dev, uint32_t queue);

typedef uint64_t (*vio_get_host_features)(const struct device *dev);

typedef void (*vio_set_guest_features)(const struct device *dev, uint64_t features);

/* XXX The whole api interfacing with MMIO addresses is in the transport layer
 * currently.
 * We could part of it (virtqueues) to a different api layer in the future.
 */
__subsystem struct virtio_config_api {
	vio_get_config		get_config;
	vio_set_config		set_config;
	vio_generation		generation;
	vio_get_status		get_status;
	vio_set_status		set_status;
	vio_reset		reset;
	vio_get_queue_max	get_queue_max;
	vio_setup_vq		setup_vq;
	vio_set_ready_vq	ready_vq;
	vio_notify_hp		notify_hp;
	vio_del_vq		del_vq;
	vio_get_host_features	get_host_features;
	vio_set_guest_features	set_guest_features;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_VIRTIO_CONFIG_H_ */
