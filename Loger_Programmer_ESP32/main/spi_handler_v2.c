/**
 * @file spi_handler_v2.c
 * @brief SPI Slave v2: реализация с препроцессором CONFIG_SPI_SLAVE_SPI2_ENABLED / CONFIG_SPI_SLAVE_SPI3_ENABLED.
 */

#include "spi_handler_v2.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#define CRC32_POLY  0xEDB88320
#define TAG         "spi_slave_v2"
#define TAG_UPS     "spi_ups"

/* ----------------------------------------------------------------------------
 * Очереди и семафоры
 * ---------------------------------------------------------------------------- */
#if CONFIG_SPI_SLAVE_SPI2_ENABLED
static QueueHandle_t s_spi2_evt_queue;
static SemaphoreHandle_t s_spi2_driver_sem;
#endif

#if CONFIG_SPI_SLAVE_SPI3_ENABLED
static QueueHandle_t s_spi3_evt_queue;
static SemaphoreHandle_t s_spi3_driver_sem;
static QueueSetHandle_t s_spi_evt_queue_set;
#endif

/* ----------------------------------------------------------------------------
 * Буферы и данные по шинам
 * ---------------------------------------------------------------------------- */
#if CONFIG_SPI_SLAVE_SPI2_ENABLED
static volatile spi_slave_buffers_t s_spi2_buffers;
static volatile spi_slave_message_t s_spi2_msg;
/* Глобальный буфер SPI2: используется wifi_control.c для generate_ups_json_string */
volatile ModulData_t ModulData;
#endif

#if CONFIG_SPI_SLAVE_SPI3_ENABLED
static volatile spi_slave_buffers_t s_spi3_buffers;
static volatile spi_slave_message_t s_spi3_msg;
static volatile ModulData_t s_modul_data_spi3;
#endif

/* ----------------------------------------------------------------------------
 * Прототипы внутренних функций
 * ---------------------------------------------------------------------------- */
static void spi_slave_alloc_buffers(void);
static void spi_slave_gpio_init_spi2(void);
#if CONFIG_SPI_SLAVE_SPI3_ENABLED
static void spi_slave_gpio_init_spi3(void);
#endif

static void spi_slave_driver_task_spi2(void *pvParameters);
#if CONFIG_SPI_SLAVE_SPI3_ENABLED
static void spi_slave_driver_task_spi3(void *pvParameters);
#endif
static void spi_slave_processing_task(void *pvParameters);

void IRAM_ATTR spi_slave_post_setup_cb_spi2(spi_slave_transaction_t *trans);
void IRAM_ATTR spi_slave_post_trans_cb_spi2(spi_slave_transaction_t *trans);
#if CONFIG_SPI_SLAVE_SPI3_ENABLED
void IRAM_ATTR spi_slave_post_setup_cb_spi3(spi_slave_transaction_t *trans);
void IRAM_ATTR spi_slave_post_trans_cb_spi3(spi_slave_transaction_t *trans);
#endif

static void spi_slave_print_ups_packet(volatile FpgaToEspPacket_t *pkt, const char *source_tag);
static uint32_t spi_slave_crc32(const void *data, size_t len);

/* ----------------------------------------------------------------------------
 * Единая точка входа: инициализация всего
 * ---------------------------------------------------------------------------- */
void spi_slave_init(void)
{
    spi_slave_alloc_buffers();

#if CONFIG_SPI_SLAVE_SPI2_ENABLED
    spi_slave_gpio_init_spi2();
    gpio_set_pull_mode(SPI_SLAVE_SPI2_GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SPI_SLAVE_SPI2_GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SPI_SLAVE_SPI2_GPIO_CS,   GPIO_PULLUP_ONLY);
#endif

#if CONFIG_SPI_SLAVE_SPI3_ENABLED
    spi_slave_gpio_init_spi3();
    gpio_set_pull_mode(SPI_SLAVE_SPI3_GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SPI_SLAVE_SPI3_GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SPI_SLAVE_SPI3_GPIO_CS,   GPIO_PULLUP_ONLY);
#endif

    /* Очереди: одна для SPI2, при включённом SPI3 — ещё одна и набор очередей */
#if CONFIG_SPI_SLAVE_SPI2_ENABLED
    s_spi2_evt_queue = xQueueCreate(2, sizeof(spi_slave_message_t));
    if (s_spi2_evt_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create SPI2 event queue");
        return;
    }
#endif

#if CONFIG_SPI_SLAVE_SPI3_ENABLED
    s_spi3_evt_queue = xQueueCreate(2, sizeof(spi_slave_message_t));
    s_spi_evt_queue_set = xQueueCreateSet(4);
    if (s_spi3_evt_queue == NULL || s_spi_evt_queue_set == NULL) {
        ESP_LOGE(TAG, "Failed to create SPI3 queue(s)");
        return;
    }
    xQueueAddToSet(s_spi2_evt_queue, s_spi_evt_queue_set);
    xQueueAddToSet(s_spi3_evt_queue, s_spi_evt_queue_set);
#endif

    /* Инициализация шины SPI2 */
#if CONFIG_SPI_SLAVE_SPI2_ENABLED
    {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = SPI_SLAVE_SPI2_GPIO_MOSI,
            .miso_io_num = SPI_SLAVE_SPI2_GPIO_MISO,
            .sclk_io_num = SPI_SLAVE_SPI2_GPIO_SCLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        };
        spi_slave_interface_config_t slv_cfg = {
            .mode = 0,
            .spics_io_num = SPI_SLAVE_SPI2_GPIO_CS,
            .queue_size = 3,
            .flags = 0,
            .post_setup_cb = spi_slave_post_setup_cb_spi2,
            .post_trans_cb = spi_slave_post_trans_cb_spi2,
        };
        esp_err_t ret = spi_slave_initialize(SPI2_HOST, &bus_cfg, &slv_cfg, SPI_SLAVE_DMA_CHAN);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI2 init failed: %s", esp_err_to_name(ret));
            return;
        }
        s_spi2_driver_sem = xSemaphoreCreateBinary();
        if (s_spi2_driver_sem == NULL) {
            ESP_LOGE(TAG, "Failed to create SPI2 driver semaphore");
            return;
        }
        xSemaphoreGive(s_spi2_driver_sem);
        if (xTaskCreate(spi_slave_driver_task_spi2, "spi2_drv", 4096, NULL, 5, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create SPI2 driver task");
        }
    }
#endif

    /* Инициализация шины SPI3 */
#if CONFIG_SPI_SLAVE_SPI3_ENABLED
    {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = SPI_SLAVE_SPI3_GPIO_MOSI,
            .miso_io_num = SPI_SLAVE_SPI3_GPIO_MISO,
            .sclk_io_num = SPI_SLAVE_SPI3_GPIO_SCLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        };
        spi_slave_interface_config_t slv_cfg = {
            .mode = 0,
            .spics_io_num = SPI_SLAVE_SPI3_GPIO_CS,
            .queue_size = 3,
            .flags = 0,
            .post_setup_cb = spi_slave_post_setup_cb_spi3,
            .post_trans_cb = spi_slave_post_trans_cb_spi3,
        };
        esp_err_t ret = spi_slave_initialize(SPI3_HOST, &bus_cfg, &slv_cfg, SPI_SLAVE_DMA_CHAN);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI3 init failed: %s", esp_err_to_name(ret));
        } else {
            s_spi3_driver_sem = xSemaphoreCreateBinary();
            if (s_spi3_driver_sem != NULL) {
                xSemaphoreGive(s_spi3_driver_sem);
                if (xTaskCreate(spi_slave_driver_task_spi3, "spi3_drv", 4096, NULL, 5, NULL) != pdPASS) {
                    ESP_LOGE(TAG, "Failed to create SPI3 driver task");
                }
            }
        }
    }
#endif

    /* Одна задача обработки: обрабатывает приём с обоих SPI и выводит в терминал */
    if (xTaskCreate(spi_slave_processing_task, "spi_proc", 4096, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SPI processing task");
    }

    ESP_LOGI(TAG, "SPI Slave init done (SPI2=%d, SPI3=%d)",
             (int)CONFIG_SPI_SLAVE_SPI2_ENABLED, (int)CONFIG_SPI_SLAVE_SPI3_ENABLED);
}

/* ----------------------------------------------------------------------------
 * Задача драйвера SPI2: подаёт буферы в spi_slave_transmit
 * ---------------------------------------------------------------------------- */
#if CONFIG_SPI_SLAVE_SPI2_ENABLED
static void spi_slave_driver_task_spi2(void *pvParameters)
{
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = SPI_SLAVE_PAYLOAD_SIZE * 8;

    for (;;) {
        memset(s_spi2_buffers.rx_buffer, 0, SPI_SLAVE_PAYLOAD_SIZE);
        memcpy(s_spi2_buffers.tx_buffer, ModulData.Tx_Buffer, sizeof(ModulData.Tx_Buffer));
        t.tx_buffer = s_spi2_buffers.tx_buffer;
        t.rx_buffer = s_spi2_buffers.rx_buffer;

        esp_err_t ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI2 transmit failed: %s", esp_err_to_name(ret));
        }
        xSemaphoreTake(s_spi2_driver_sem, portMAX_DELAY);
    }
}
#endif

/* ----------------------------------------------------------------------------
 * Задача драйвера SPI3
 * ---------------------------------------------------------------------------- */
#if CONFIG_SPI_SLAVE_SPI3_ENABLED
static void spi_slave_driver_task_spi3(void *pvParameters)
{
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = SPI_SLAVE_PAYLOAD_SIZE * 8;

    for (;;) {
        memset(s_spi3_buffers.rx_buffer, 0, SPI_SLAVE_PAYLOAD_SIZE);
        memcpy(s_spi3_buffers.tx_buffer, s_modul_data_spi3.Tx_Buffer, sizeof(s_modul_data_spi3.Tx_Buffer));
        t.tx_buffer = s_spi3_buffers.tx_buffer;
        t.rx_buffer = s_spi3_buffers.rx_buffer;

        esp_err_t ret = spi_slave_transmit(SPI3_HOST, &t, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI3 transmit failed: %s", esp_err_to_name(ret));
        }
        xSemaphoreTake(s_spi3_driver_sem, portMAX_DELAY);
    }
}
#endif

/* ----------------------------------------------------------------------------
 * Колбэки SPI2 (post_trans копирует данные в очередь)
 * ---------------------------------------------------------------------------- */
#if CONFIG_SPI_SLAVE_SPI2_ENABLED
void IRAM_ATTR spi_slave_post_setup_cb_spi2(spi_slave_transaction_t *trans)
{
    (void)trans;
}

void IRAM_ATTR spi_slave_post_trans_cb_spi2(spi_slave_transaction_t *trans)
{
    BaseType_t woken = pdFALSE;
    uint32_t n = trans->trans_len / 8;
    if (n > sizeof(s_spi2_msg.data)) n = sizeof(s_spi2_msg.data);
    memcpy(s_spi2_msg.data, trans->rx_buffer, n);
    s_spi2_msg.len = n;
    xQueueSendFromISR(s_spi2_evt_queue, &s_spi2_msg, &woken);
    if (woken) portYIELD_FROM_ISR();
}
#endif

/* ----------------------------------------------------------------------------
 * Колбэки SPI3
 * ---------------------------------------------------------------------------- */
#if CONFIG_SPI_SLAVE_SPI3_ENABLED
void IRAM_ATTR spi_slave_post_setup_cb_spi3(spi_slave_transaction_t *trans)
{
    (void)trans;
}

void IRAM_ATTR spi_slave_post_trans_cb_spi3(spi_slave_transaction_t *trans)
{
    BaseType_t woken = pdFALSE;
    uint32_t n = trans->trans_len / 8;
    if (n > sizeof(s_spi3_msg.data)) n = sizeof(s_spi3_msg.data);
    memcpy(s_spi3_msg.data, trans->rx_buffer, n);
    s_spi3_msg.len = n;
    xQueueSendFromISR(s_spi3_evt_queue, &s_spi3_msg, &woken);
    if (woken) portYIELD_FROM_ISR();
}
#endif

/* ----------------------------------------------------------------------------
 * Задача обработки: одна на оба SPI; при SPI3 — ожидание через queue set
 * ---------------------------------------------------------------------------- */
static void spi_slave_processing_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "SPI processing task started");

    spi_slave_message_t msg;

#if CONFIG_SPI_SLAVE_SPI3_ENABLED
    /* Оба SPI: ждём любое событие из набора очередей */
    for (;;) {
        QueueHandle_t active = (QueueHandle_t)xQueueSelectFromSet(s_spi_evt_queue_set, portMAX_DELAY);
        if (active == NULL) continue;
        if (xQueueReceive(active, &msg, 0) != pdPASS) continue;

        bool from_spi2 = (active == s_spi2_evt_queue);
        SemaphoreHandle_t sem = from_spi2 ? s_spi2_driver_sem : s_spi3_driver_sem;
        const char *tag = from_spi2 ? "SPI2" : "SPI3";

        if (msg.len < 4) {
            ESP_LOGW(TAG, "[%s] Too short packet: %lu bytes", tag, (unsigned long)msg.len);
            xSemaphoreGive(sem);
            continue;
        }

        if (from_spi2) {
            memcpy(ModulData.Tx_Buffer, msg.data, msg.len);
            if (ModulData.packet.crc32 != spi_slave_crc32(ModulData.Tx_Buffer, msg.len - 4)) {
                ESP_LOGE(TAG, "[%s] CRC error", tag);
            } else {
                spi_slave_print_ups_packet(&ModulData.packet, tag);
            }
        } else {
            memcpy(s_modul_data_spi3.Tx_Buffer, msg.data, msg.len);
            if (s_modul_data_spi3.packet.crc32 != spi_slave_crc32(s_modul_data_spi3.Tx_Buffer, msg.len - 4)) {
                ESP_LOGE(TAG, "[%s] CRC error", tag);
            } else {
                spi_slave_print_ups_packet(&s_modul_data_spi3.packet, tag);
            }
        }
        xSemaphoreGive(sem);
    }
#else
    /* Только SPI2: одна очередь */
    for (;;) {
        if (xQueueReceive(s_spi2_evt_queue, &msg, portMAX_DELAY) != pdPASS) continue;

        if (msg.len < 4) {
            ESP_LOGW(TAG, "[SPI2] Too short packet: %lu bytes", (unsigned long)msg.len);
            xSemaphoreGive(s_spi2_driver_sem);
            continue;
        }
        memcpy(ModulData.Tx_Buffer, msg.data, msg.len);
        if (ModulData.packet.crc32 != spi_slave_crc32(ModulData.Tx_Buffer, msg.len - 4)) {
            ESP_LOGE(TAG, "[SPI2] CRC error");
        } else {
            spi_slave_print_ups_packet(&ModulData.packet, "SPI2");
        }
        xSemaphoreGive(s_spi2_driver_sem);
    }
#endif
}

/* ----------------------------------------------------------------------------
 * Выделение DMA-буферов
 * ---------------------------------------------------------------------------- */
static void spi_slave_alloc_buffers(void)
{
#if CONFIG_SPI_SLAVE_SPI2_ENABLED
    s_spi2_buffers.rx_buffer = (uint8_t *)spi_bus_dma_memory_alloc(SPI2_HOST, SPI_SLAVE_PAYLOAD_SIZE, MALLOC_CAP_DMA);
    s_spi2_buffers.tx_buffer = (uint8_t *)spi_bus_dma_memory_alloc(SPI2_HOST, SPI_SLAVE_PAYLOAD_SIZE, MALLOC_CAP_DMA);
    if (!s_spi2_buffers.rx_buffer || !s_spi2_buffers.tx_buffer) {
        ESP_LOGE(TAG, "SPI2 DMA buffer alloc failed");
        return;
    }
    memset(s_spi2_buffers.rx_buffer, 0, SPI_SLAVE_PAYLOAD_SIZE);
    memset(s_spi2_buffers.tx_buffer, 0, SPI_SLAVE_PAYLOAD_SIZE);
    s_spi2_buffers.internal_tx = NULL;
    s_spi2_buffers.internal_rx = NULL;
    ESP_LOGI(TAG, "SPI2 RX @ %p TX @ %p", (void *)s_spi2_buffers.rx_buffer, (void *)s_spi2_buffers.tx_buffer);
#endif

#if CONFIG_SPI_SLAVE_SPI3_ENABLED
    s_spi3_buffers.rx_buffer = (uint8_t *)spi_bus_dma_memory_alloc(SPI3_HOST, SPI_SLAVE_PAYLOAD_SIZE, MALLOC_CAP_DMA);
    s_spi3_buffers.tx_buffer = (uint8_t *)spi_bus_dma_memory_alloc(SPI3_HOST, SPI_SLAVE_PAYLOAD_SIZE, MALLOC_CAP_DMA);
    if (!s_spi3_buffers.rx_buffer || !s_spi3_buffers.tx_buffer) {
        ESP_LOGE(TAG, "SPI3 DMA buffer alloc failed");
        return;
    }
    memset(s_spi3_buffers.rx_buffer, 0, SPI_SLAVE_PAYLOAD_SIZE);
    memset(s_spi3_buffers.tx_buffer, 0, SPI_SLAVE_PAYLOAD_SIZE);
    s_spi3_buffers.internal_tx = NULL;
    s_spi3_buffers.internal_rx = NULL;
    ESP_LOGI(TAG, "SPI3 RX @ %p TX @ %p", (void *)s_spi3_buffers.rx_buffer, (void *)s_spi3_buffers.tx_buffer);
#endif
}

/* ----------------------------------------------------------------------------
 * GPIO: инициализация пинов SPI2
 * ---------------------------------------------------------------------------- */
#if CONFIG_SPI_SLAVE_SPI2_ENABLED
static void spi_slave_gpio_init_spi2(void)
{
    gpio_reset_pin(SPI_SLAVE_SPI2_GPIO_MOSI);
    gpio_reset_pin(SPI_SLAVE_SPI2_GPIO_MISO);
    gpio_reset_pin(SPI_SLAVE_SPI2_GPIO_SCLK);
    gpio_reset_pin(SPI_SLAVE_SPI2_GPIO_CS);

    gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << SPI_SLAVE_SPI2_GPIO_CS) | (1ULL << SPI_SLAVE_SPI2_GPIO_SCLK) | (1ULL << SPI_SLAVE_SPI2_GPIO_MOSI),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io);

    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL << SPI_SLAVE_SPI2_GPIO_MISO);
    gpio_config(&io);
    gpio_set_level(SPI_SLAVE_SPI2_GPIO_MISO, 0);
    ESP_LOGI(TAG, "SPI2 GPIO: MOSI=%d MISO=%d SCLK=%d CS=%d",
             SPI_SLAVE_SPI2_GPIO_MOSI, SPI_SLAVE_SPI2_GPIO_MISO, SPI_SLAVE_SPI2_GPIO_SCLK, SPI_SLAVE_SPI2_GPIO_CS);
}
#endif

/* ----------------------------------------------------------------------------
 * GPIO: инициализация пинов SPI3
 * ---------------------------------------------------------------------------- */
#if CONFIG_SPI_SLAVE_SPI3_ENABLED
static void spi_slave_gpio_init_spi3(void)
{
    gpio_reset_pin(SPI_SLAVE_SPI3_GPIO_MOSI);
    gpio_reset_pin(SPI_SLAVE_SPI3_GPIO_MISO);
    gpio_reset_pin(SPI_SLAVE_SPI3_GPIO_SCLK);
    gpio_reset_pin(SPI_SLAVE_SPI3_GPIO_CS);

    gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << SPI_SLAVE_SPI3_GPIO_CS) | (1ULL << SPI_SLAVE_SPI3_GPIO_SCLK) | (1ULL << SPI_SLAVE_SPI3_GPIO_MOSI),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io);

    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL << SPI_SLAVE_SPI3_GPIO_MISO);
    gpio_config(&io);
    gpio_set_level(SPI_SLAVE_SPI3_GPIO_MISO, 0);
    ESP_LOGI(TAG, "SPI3 GPIO: MOSI=%d MISO=%d SCLK=%d CS=%d",
             SPI_SLAVE_SPI3_GPIO_MOSI, SPI_SLAVE_SPI3_GPIO_MISO, SPI_SLAVE_SPI3_GPIO_SCLK, SPI_SLAVE_SPI3_GPIO_CS);
}
#endif

/* ----------------------------------------------------------------------------
 * Вывод пакета UPS в лог (с префиксом источника SPI2/SPI3)
 * ---------------------------------------------------------------------------- */
static void spi_slave_print_ups_packet(volatile FpgaToEspPacket_t *pkt, const char *source_tag)
{
    if (pkt->start_marker != 0xAA55AA55) {
        ESP_LOGW(TAG_UPS, "[%s] Bad start marker 0x%08lX", source_tag, (unsigned long)pkt->start_marker);
    }

    ESP_LOGI(TAG_UPS, "[%s] === UPS DATA (Cnt: %lu) ===", source_tag, (unsigned long)pkt->packet_counter);
    ESP_LOGI(TAG_UPS, "[%s] [STATUS] Grid:%d Byp:%d Rect:%d Inv:%d PwrInv:%d PwrByp:%d Sync:%d",
             source_tag, pkt->status.grid_status, pkt->status.bypass_grid_status,
             pkt->status.rectifier_status, pkt->status.inverter_status,
             pkt->status.pwr_via_inverter, pkt->status.pwr_via_bypass, pkt->status.sync_status);
    ESP_LOGI(TAG_UPS, "[%s] [STATUS] Load:%d Sound:%d BatSt:%d UpsMode:%d",
             source_tag, pkt->status.load_mode, pkt->status.sound_alarm,
             pkt->status.battery_status, pkt->status.ups_mode);
    ESP_LOGI(TAG_UPS, "[%s] [ALARM] LowIn:%d HighDC:%d LowBat:%d NoBat:%d InvF:%d InvOC:%d",
             source_tag, pkt->alarms.err_low_input_vol, pkt->alarms.err_high_dc_bus,
             pkt->alarms.err_low_bat_charge, pkt->alarms.err_bat_not_conn,
             pkt->alarms.err_inv_fault, pkt->alarms.err_inv_overcurrent);
    ESP_LOGI(TAG_UPS, "[%s] [INPUT] V_in: A=%.1f B=%.1f C=%.1f | V_byp: A=%.1f B=%.1f C=%.1f",
             source_tag, pkt->input.v_in_AB, pkt->input.v_in_BC, pkt->input.v_in_CA,
             pkt->input.v_bypass_A, pkt->input.v_bypass_B, pkt->input.v_bypass_C);
    ESP_LOGI(TAG_UPS, "[%s] [INPUT] I_in: A=%.1f B=%.1f C=%.1f Freq: %.2f Hz",
             source_tag, pkt->input.i_in_A, pkt->input.i_in_B, pkt->input.i_in_C, pkt->input.freq_in);
    ESP_LOGI(TAG_UPS, "[%s] [OUTPUT] V_out: A=%.1f B=%.1f C=%.1f Freq: %.2f Hz",
             source_tag, pkt->output.v_out_A, pkt->output.v_out_B, pkt->output.v_out_C, pkt->output.freq_out);
    ESP_LOGI(TAG_UPS, "[%s] [OUTPUT] I_out: A=%.1f B=%.1f C=%.1f",
             source_tag, pkt->output.i_out_A, pkt->output.i_out_B, pkt->output.i_out_C);
    ESP_LOGI(TAG_UPS, "[%s] [OUTPUT] P_Act: A=%.1f B=%.1f C=%.1f kW",
             source_tag, pkt->output.p_active_A, pkt->output.p_active_B, pkt->output.p_active_C);
    ESP_LOGI(TAG_UPS, "[%s] [OUTPUT] P_App: A=%.1f B=%.1f C=%.1f kVA",
             source_tag, pkt->output.p_apparent_A, pkt->output.p_apparent_B, pkt->output.p_apparent_C);
    ESP_LOGI(TAG_UPS, "[%s] [OUTPUT] Load: A=%.1f%% B=%.1f%% C=%.1f%% Events: %.0f",
             source_tag, pkt->output.load_pct_A, pkt->output.load_pct_B, pkt->output.load_pct_C, pkt->output.event_count);
    ESP_LOGI(TAG_UPS, "[%s] [BAT] Vol: %.1f V Cap: %.0f Ah Grp: %.0f DC: %.1f V",
             source_tag, pkt->battery.bat_voltage, pkt->battery.bat_capacity,
             pkt->battery.bat_groups_count, pkt->battery.dc_bus_voltage);
    ESP_LOGI(TAG_UPS, "[%s] [BAT] Cur: %.1f A Backup: %.0f min",
             source_tag, pkt->battery.bat_current, pkt->battery.backup_time);
    ESP_LOGI(TAG_UPS, "[%s] [CRC] 0x%08lX", source_tag, (unsigned long)pkt->crc32);
    ESP_LOGI(TAG_UPS, "[%s] =================================", source_tag);
}

/* ----------------------------------------------------------------------------
 * CRC-32
 * ---------------------------------------------------------------------------- */
static uint32_t spi_slave_crc32(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ CRC32_POLY : (crc >> 1);
        }
    }
    return ~crc;
}
