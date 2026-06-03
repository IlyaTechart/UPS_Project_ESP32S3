/*
 * SPDX-FileCopyrightText: 2020-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "frames_structure.h"

#ifdef __cplusplus
extern "C" {
#endif


#define CDC_CMD_COMAND_SIZE 8

#define ID_AVE_FRAME_START   (const uint32_t)0x22446688
#define ID_DUMP_FRAME_START  (const uint32_t)0x336699FF

#define ID_TAIL_FRMES        (const uint32_t)0x55AA55AA

#define USB_TX_TIMEOUT_MS 2000 // 2 секунды на ожидание хоста (важно для 1 МБ)

/// @brief Структура для отправки сообщений по USB 
typedef struct {
	uint32_t head_frames;
	uint32_t count_elements;
	ModulData_t *buffer;
	uint32_t time_event;
	uint32_t tail_frames;
}__attribute__((packed))Package_t;


/**
 * @brief Initialize serial bridge
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t serial_bridge_init(void);

#ifdef __cplusplus
}
#endif
