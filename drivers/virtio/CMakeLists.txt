# Copyright (c) 2022 Huawei Technologies
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_VIRTIO		virtio.c)
zephyr_library_sources_ifdef(CONFIG_VIRTIO_USE_MMIO	virtio-mmio.c)
