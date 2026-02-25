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
#include "wifi_control.h"

/* ----------------------------------------------------------------------------
 * Конфигурация: какой SPI включать в сборку
 * Переопределить в sdkconfig, CMake или -D при компиляции.
 * ---------------------------------------------------------------------------- */
#ifndef CONFIG_SPI_SLAVE_SPI2_ENABLED
#define CONFIG_SPI_SLAVE_SPI2_ENABLED  1
#endif

#ifndef CONFIG_SPI_SLAVE_SPI3_ENABLED
#define CONFIG_SPI_SLAVE_SPI3_ENABLED  1
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
#define SPI_SLAVE_BUFFER_SIZE_KB  1
#define SPI_SLAVE_BYTES_PER_KB     1024
#define SPI_SLAVE_PAYLOAD_SIZE     (SPI_SLAVE_BUFFER_SIZE_KB * SPI_SLAVE_BYTES_PER_KB)

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

/* ----------------------------------------------------------------------------
 * Структуры пакета (ModulData_t и входящие в неё — не трогаем)
 * ---------------------------------------------------------------------------- */
#pragma pack(push, 1)

typedef struct {
    uint16_t grid_status;
    uint16_t bypass_grid_status;
    uint16_t rectifier_status;
    uint16_t inverter_status;
    uint16_t pwr_via_inverter;
    uint16_t pwr_via_bypass;
    uint16_t sync_status;
    uint16_t load_mode;
    uint16_t sound_alarm;
    uint16_t battery_status;
    uint16_t ups_mode;
} GroupStatus_t;

typedef struct {
    uint16_t err_low_input_vol;
    uint16_t err_high_dc_bus;
    uint16_t err_low_bat_charge;
    uint16_t err_bat_not_conn;
    uint16_t err_inv_fault;
    uint16_t err_inv_overcurrent;
    uint16_t err_high_out_vol;
    uint16_t err_fan_fault;
    uint16_t err_replace_bat;
    uint16_t err_rect_overheat;
    uint16_t err_inv_overheat;
} GroupAlarms_t;

typedef struct {
    float v_in_AB;
    float v_in_BC;
    float v_in_CA;
    float v_bypass_A;
    float v_bypass_B;
    float v_bypass_C;
    float i_in_A;
    float i_in_B;
    float i_in_C;
    float freq_in;
} GroupInput_t;

typedef struct {
    float v_out_A;
    float v_out_B;
    float v_out_C;
    float freq_out;
    float i_out_A;
    float i_out_B;
    float i_out_C;
    float p_active_A;
    float p_active_B;
    float p_active_C;
    float p_apparent_A;
    float p_apparent_B;
    float p_apparent_C;
    float load_pct_A;
    float load_pct_B;
    float load_pct_C;
    float event_count;
} GroupOutput_t;

typedef struct {
    float bat_voltage;
    float bat_capacity;
    float bat_groups_count;
    float dc_bus_voltage;
    float bat_current;
    float backup_time;
} GroupBattery_t;

typedef struct {
    uint32_t start_marker;
    uint32_t packet_counter;
    GroupStatus_t  status;
    GroupAlarms_t  alarms;
    GroupInput_t   input;
    GroupOutput_t  output;
    GroupBattery_t battery;
    uint32_t crc32;
} FpgaToEspPacket_t;

#pragma pack(pop)

typedef union {
    FpgaToEspPacket_t packet;
    uint8_t Tx_Buffer[sizeof(FpgaToEspPacket_t) + 4];
} ModulData_t;

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
