/**
 * @file spi_handler_v2.h
 * @brief SPI Slave (v2): причёсанный вариант с препроцессором SPI2 / SPI2+SPI3.
 *
 * Одна точка входа: spi_slave_init() — инициализирует GPIO, буферы, оба SPI (по конфигу),
 * очереди, семафоры и все задачи FreeRTOS (драйверы + обработка).
 *
 * Варианты сборки (задать до включения заголовка или в -D):
 *   CONFIG_SPI_SLAVE_SPI2_ENABLED=1  — использовать SPI2 (по умолчанию 1)
 *   CONFIG_SPI_SLAVE_SPI3_ENABLED=0  — только SPI2; =1 — SPI2 и SPI3
 */

#pragma once

#include "esp_err.h"
#include "frames_structure.h"
#include "wifi_control.h"

/* ----------------------------------------------------------------------------
 * Конфигурация: какой SPI включать в сборку
 * Переопределить в sdkconfig, CMake или -D при компиляции.
 * ---------------------------------------------------------------------------- */
#ifndef CONFIG_SPI_SLAVE_SPI2_ENABLED
#define CONFIG_SPI_SLAVE_SPI2_ENABLED  1
#endif

#ifndef CONFIG_SPI_SLAVE_SPI3_ENABLED
#define CONFIG_SPI_SLAVE_SPI3_ENABLED  0
#endif

#if !CONFIG_SPI_SLAVE_SPI2_ENABLED && !CONFIG_SPI_SLAVE_SPI3_ENABLED
#error "At least one of CONFIG_SPI_SLAVE_SPI2_ENABLED or CONFIG_SPI_SLAVE_SPI3_ENABLED must be 1"
#endif

/* ----------------------------------------------------------------------------
 * Пины SPI2 (используются при CONFIG_SPI_SLAVE_SPI2_ENABLED)
 * ---------------------------------------------------------------------------- */
#define SPI_SLAVE_SPI2_GPIO_MOSI  11
#define SPI_SLAVE_SPI2_GPIO_MISO  13
#define SPI_SLAVE_SPI2_GPIO_SCLK  12
#define SPI_SLAVE_SPI2_GPIO_CS    10

/* ----------------------------------------------------------------------------
 * Пины SPI3 (используются при CONFIG_SPI_SLAVE_SPI3_ENABLED)
 * ---------------------------------------------------------------------------- */
#if CONFIG_SPI_SLAVE_SPI3_ENABLED
#define SPI_SLAVE_SPI3_GPIO_MOSI  35
#define SPI_SLAVE_SPI3_GPIO_MISO  36
#define SPI_SLAVE_SPI3_GPIO_SCLK  9
#define SPI_SLAVE_SPI3_GPIO_CS    14
#endif

/* ----------------------------------------------------------------------------
 * Размер буфера и DMA
 * ---------------------------------------------------------------------------- */
#define SPI_SLAVE_DMA_CHAN         SPI_DMA_CH_AUTO
/* Размер буфера SPI: не меньше пакета из frames_structure.h (примерно 82 байта), берём с запасом */
#define SPI_SLAVE_PAYLOAD_SIZE     256

/* ----------------------------------------------------------------------------
 * Буферы и сообщения (внутренние типы, без изменения контракта пакетов)
 * ---------------------------------------------------------------------------- */
typedef struct {
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    uint8_t *internal_tx;
    uint8_t *internal_rx;
} spi_slave_buffers_t;

typedef struct {
    uint8_t  data[SPI_SLAVE_PAYLOAD_SIZE];
    uint32_t len;
} spi_slave_message_t;


/* Для совместимости с wifi_control.c: глобальный буфер данных SPI2 (определён в spi_handler_v2.c при CONFIG_SPI_SLAVE_SPI2_ENABLED) */
#if CONFIG_SPI_SLAVE_SPI2_ENABLED
extern volatile ModulData_t ModulData;
#endif

/* ----------------------------------------------------------------------------
 * Публичный API
 * ---------------------------------------------------------------------------- */

/**
 * Инициализация SPI Slave: GPIO, DMA-буферы, шины SPI2/SPI3 (по конфигу),
 * очереди, семафоры и задачи FreeRTOS (драйверы приёма + одна задача обработки).
 * Вызывать один раз из app_main().
 */
void spi_slave_init(void);
