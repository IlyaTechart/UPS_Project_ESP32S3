/*
 * SPDX-FileCopyrightText: 2020-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "serial_bridge.h"
#include "serial_handler.h"
#include "tusb_config.h"
#include "tusb.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "util.h"
#include "debug_probe.h"
#include "logger_handler.h"


#define USB_SEND_RINGBUFFER_SIZE (2 * 1024)

static const char *TAG = "serial_bridge";

static uint8_t cmd_buf[8];
static uint8_t cmd_buf_pos = 0;

QueueHandle_t queue_serial_RX;

TaskHandle_t DumpTask_Handler;

extern SemaphoreHandle_t bufferMutex;

static RingbufHandle_t usb_sendbuf;
static SemaphoreHandle_t usb_tx_requested = NULL;
static SemaphoreHandle_t usb_tx_done = NULL;
static esp_timer_handle_t state_change_timer;


static void error_handler(esp_err_t r) {
    switch (r) {
        case ESP_OK:
            ESP_LOGI(TAG, "ESP_OK: Успешное выполнение");
            break;
        case ESP_FAIL:
            ESP_LOGE(TAG, "ESP_FAIL: Общая ошибка");
            break;
        case ESP_ERR_NO_MEM:
            ESP_LOGE(TAG, "ESP_ERR_NO_MEM: Недостаточно памяти (Out of memory)");
            break;
        case ESP_ERR_INVALID_ARG:
            ESP_LOGE(TAG, "ESP_ERR_INVALID_ARG: Неверный аргумент функции");
            break;
        case ESP_ERR_INVALID_STATE:
            ESP_LOGE(TAG, "ESP_ERR_INVALID_STATE: Неверное состояние (периферия не инициализирована и т.п.)");
            break;
        case ESP_ERR_INVALID_SIZE:
            ESP_LOGE(TAG, "ESP_ERR_INVALID_SIZE: Неверный размер данных");
            break;
        case ESP_ERR_NOT_FOUND:
            ESP_LOGE(TAG, "ESP_ERR_NOT_FOUND: Запрошенный ресурс не найден");
            break;
        case ESP_ERR_NOT_SUPPORTED:
            ESP_LOGE(TAG, "ESP_ERR_NOT_SUPPORTED: Операция не поддерживается");
            break;
        case ESP_ERR_TIMEOUT:
            ESP_LOGE(TAG, "ESP_ERR_TIMEOUT: Таймаут ожидания операции (например, на шине SPI)");
            break;
        case ESP_ERR_INVALID_RESPONSE:
            ESP_LOGE(TAG, "ESP_ERR_INVALID_RESPONSE: Получен неверный или неожиданный ответ");
            break;
        case ESP_ERR_INVALID_CRC:
            ESP_LOGE(TAG, "ESP_ERR_INVALID_CRC: Ошибка контрольной суммы (CRC)");
            break;
        default:
            // ПРО-СОВЕТ: В ESP-IDF есть встроенная функция esp_err_to_name(), 
            // которая переводит любой код ошибки в строку.
            // Используем её для всех остальных (менее частых) кодов.
            ESP_LOGE(TAG, "Неизвестная ошибка: %s (код: 0x%x)", esp_err_to_name(r), r);
            break;
    }
}

/// @brief Отправка данных байтовым полем нужной длины
/// @param data Указатель на отправляемые данные
/// @param len Длина отправляемых данных 
/// @return При успешном завершении вернёт - ESP_OK, при тайм-ауте - ESP_ERR_TIMEOUT, при обрыве цпи во время передачи - ESP_ERR_INVALID_STATE
static esp_err_t usb_cdc_write_blocking(const uint8_t *data, size_t len) 
{
    size_t written = 0;
    uint32_t timeout_counter = 0;

    while (written < len) {
        // Проверяем, не отвалился ли кабель/программа на ПК
        if (!tud_cdc_connected()) {
            ESP_LOGE(TAG, "Обрыв связи USB во время передачи!");
            return ESP_ERR_INVALID_STATE;
        }

        uint32_t available = tud_cdc_write_available();
        if (available > 0) {
            uint32_t to_write = (len - written < available) ? (len - written) : available;
            uint32_t chunk_written = tud_cdc_write(data + written, to_write);
            
            written += chunk_written;
            timeout_counter = 0; // Сбрасываем таймаут, данные идут

            // Если заполнили буфер TinyUSB — проталкиваем его к хосту
            if (tud_cdc_write_available() == 0) {
                tud_cdc_write_flush();
            }
        } else {
            // FIFO буфер TinyUSB забит. Ждем 1 мс, пока хост (ПК) его не вычитает.
            vTaskDelay(pdMS_TO_TICKS(1));
            timeout_counter++;

            if (timeout_counter >= USB_TX_TIMEOUT_MS) {
                ESP_LOGE(TAG, "Таймаут USB! Хост перестал принимать данные.");
                return ESP_ERR_TIMEOUT;
            }
        }
    }
    return ESP_OK;
}

/// @brief Функция отправки сообщения со средними значениями по USB 
/// @param AVE_Sendler - Указатель на отправляемую структуру в которой упкован список 
/// @return 
esp_err_t send_usb_ave_frame(Package_t *AVE_Sendler){

    if (AVE_Sendler == NULL){
        ESP_LOGE(TAG, "Ошибка: Указатель на структуру срендних значений (NULL)!");
        return ESP_FAIL;
    }
    
    if (!tud_cdc_connected()){
        ESP_LOGE(TAG, "Ошибка: USB хост не подключён");
        return ESP_ERR_INVALID_STATE;  
    }

    tud_cdc_write_clear();

    esp_err_t err;

    err = usb_cdc_write_blocking((uint8_t*)&AVE_Sendler->head_frames , sizeof(AVE_Sendler->head_frames));
    if (err != ESP_OK) return err;

    err = usb_cdc_write_blocking((uint8_t*)&AVE_Sendler->count_elements , sizeof(AVE_Sendler->count_elements));
    if (err != ESP_OK) return err;

    err = usb_cdc_write_blocking((uint8_t*)AVE_Sendler->buffer , sizeof(ModulData_t));
    if (err != ESP_OK) return err;

    // err = usb_cdc_write_blocking((uint8_t*)&AVE_Sendler->time_event , sizeof(AVE_Sendler->time_event));
    // if (err != ESP_OK) return err;

     err = usb_cdc_write_blocking((uint8_t*)&AVE_Sendler->tail_frames , sizeof(AVE_Sendler->tail_frames));
    if (err != ESP_OK) return err;

    return ESP_OK;
}


/// @brief Функция отправки дампа 
/// @param rb Указатель на кольцевой буфер дампа 
/// @return При успешной передачи вернёт - ESP_OK
static esp_err_t dump_ringbuf_to_usb_cdc(Package_t *rb)
{
    if (rb == NULL || rb->buffer == NULL){
        ESP_LOGE(TAG, "Ошибка: Кольцевой буфер не инициализирован (NULL)!");
        return ESP_FAIL;
    }
    
    if (!tud_cdc_connected()){
        ESP_LOGE(TAG, "Ошибка: USB хост не подключён");
        return ESP_ERR_INVALID_STATE;  
    }

    size_t count = rb->count_elements;
    size_t idx = RingBuffModulData.tail;
    ESP_LOGI(TAG, "Начало выгрузки. Элементов: %zu", count);

    tud_cdc_write_clear();

    esp_err_t err;

    // 1. Отправляем head_frames
    err = usb_cdc_write_blocking((uint8_t*)&rb->head_frames, sizeof(rb->head_frames));
    if (err != ESP_OK) return err;

    // 2. Отправляем count_elements 
    err = usb_cdc_write_blocking((uint8_t*)&rb->count_elements, sizeof(rb->count_elements));
    if (err != ESP_OK) return err;

    uint32_t bytes_written = 0;

    // 3. Отправляем сами кадры из кольцевого буфера
    for (size_t i = 0; i < count; i++) {
        ModulData_t *frame = &rb->buffer[idx];

        err = usb_cdc_write_blocking((uint8_t*)frame, sizeof(ModulData_t));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка отправки кадра %zu", i);
            return err;
        }

        bytes_written += sizeof(ModulData_t);
        idx = (idx + 1) % RingBuffModulData.cnt_cpyes;

        // Опционально: сбрасывать Watchdog (WDT), если дамп огромный (1МБ+)
        vTaskDelay(0); //может помочь, если таск ест 100% ядра долгое время
    }

    // 4. Отправляем tail_frames
    err = usb_cdc_write_blocking((uint8_t*)&rb->tail_frames, sizeof(rb->tail_frames));
    if (err != ESP_OK) return err;

    // 5. Финальный flush, чтобы убедиться, что последние байты ушли из FIFO на ПК
    tud_cdc_write_flush();

    ESP_LOGI(TAG, "Успешная выгрузка! Отправлено полезных байт: %lu, кадров: %zu", 
             bytes_written, count);

    return ESP_OK;
}

// Transport data received callback - called by serial handler when data arrives
static void transport_data_received_callback(const uint8_t *data, size_t len)
{
    // With the new API, the callback is only called when bridge mode is active
    // (i.e., when flashing is not in progress), so we don't need to check mode
    ESP_LOGD(TAG, "Transport -> USB ringbuffer (%zu bytes)", len);
    ESP_LOG_BUFFER_HEXDUMP("Transport -> USB", data, len, ESP_LOG_DEBUG);

    // Send received transport data to USB CDC
    if (xRingbufferSend(usb_sendbuf, data, len, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGV(TAG, "Cannot write to ringbuffer (free %zu of %zu)!",
                 xRingbufferGetCurFreeSize(usb_sendbuf),
                 (size_t)USB_SEND_RINGBUFFER_SIZE);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t usb_wait_for_tx(const uint32_t block_time_ms)
{
    if (xSemaphoreTake(usb_tx_done, pdMS_TO_TICKS(block_time_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void usb_sender_task(void *pvParameters)
{
    while (1) {
        size_t ringbuf_received;
        uint8_t *buf = xRingbufferReceiveUpTo(usb_sendbuf, &ringbuf_received, pdMS_TO_TICKS(100),
                                              CFG_TUD_CDC_TX_BUFSIZE);

        if (buf) {
            uint8_t int_buf[CFG_TUD_CDC_TX_BUFSIZE];
            memcpy(int_buf, buf, ringbuf_received);
            vRingbufferReturnItem(usb_sendbuf, (void *) buf);

            for (int transferred = 0, to_send = ringbuf_received; transferred < ringbuf_received;) {
                xSemaphoreGive(usb_tx_requested);
                const int wr_len = tud_cdc_write(int_buf + transferred, to_send);
                /* tinyusb might have been flushed the data. In case not flushed, we are flushing here.
                    2nd attempt might return zero, meaning there is no data to transfer. So it is safe to call it again.
                */
                tud_cdc_write_flush();
                if (usb_wait_for_tx(50) != ESP_OK) {
                    xSemaphoreTake(usb_tx_requested, 0);
                    tud_cdc_write_clear(); /* host might be disconnected. drop the buffer */
                    ESP_LOGV(TAG, "usb tx timeout");
                    break;
                }
                ESP_LOGD(TAG, "USB ringbuffer -> USB CDC (%d bytes)", wr_len);
                transferred += wr_len;
                to_send -= wr_len;
            }
        } else {
            ESP_LOGD(TAG, "usb_sender_task: nothing to send");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
    }
    vTaskDelete(NULL);
}

static void dump_task(void *pvParameters)
{
    BaseType_t xResult;
    uint32_t parm = 0;

    ModulData_t ModulData_SendAve;

    while(1)
    {
        parm = 0;
        xResult = xTaskNotifyWait(0x00, UINT32_MAX, &parm, portMAX_DELAY);
        if (xResult == pdTRUE){

            switch (parm)
            {
            case SEND_AVE_COMAND:

                Package_t ave_sender;
                TickType_t tick_exit = xTaskGetTickCount();

                memcpy(&ModulData_SendAve, &ModulDataFromExtend, sizeof(ModulData_t));
                ave_sender.buffer = &ModulData_SendAve;
                ave_sender.head_frames = ID_AVE_FRAME_START;
                ave_sender.time_event = (uint32_t)tick_exit;
                ave_sender.count_elements = 1;
                ave_sender.tail_frames = ID_TAIL_FRMES;

                if (send_usb_ave_frame(&ave_sender) == ESP_OK)
                {
                    ESP_LOGI(TAG, "SEND AVE FRAME");
                }else{
                    ESP_LOGI(TAG, "send_usb_aveFrame: ERROR!");
                }
                break;

            case SEND_DUMP_COMAND:
                
                if (xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE)
                {
                    esp_err_t err = ESP_ERR_INVALID_STATE;
                    TickType_t tick_exit = xTaskGetTickCount();

                    while( err != ESP_OK )
                    {
                        if((xTaskGetTickCount() - tick_exit) > 3000)
                        {
                            ESP_LOGI(TAG, "Dump timeout!");
                            break;
                        }
                        ESP_LOGI(TAG, "Start DUMP!");
                        Package_t DumpData = {0}; 
                        DumpData.head_frames = ID_DUMP_FRAME_START;
                        DumpData.count_elements = (uint32_t)get_elements_count(&RingBuffModulData);
                        DumpData.buffer = RingBuffModulData.buffer;
                        DumpData.time_event = (uint32_t)xTaskGetTickCount();
                        DumpData.tail_frames = ID_TAIL_FRMES;
                        err = dump_ringbuf_to_usb_cdc(&DumpData); 
                        if(err != ESP_OK) continue;;

                        ESP_LOGI(TAG, "END DUMP!");

                        memset(RingBuffModulData.buffer, 0x00, RingBuffModulData.size_byte);
                        RingBuffModulData.tail = 0;
                        RingBuffModulData.head = 0;
                        tick_exit = xTaskGetTickCount();
                    }
                }
                xSemaphoreGive(bufferMutex);
                break;
            
            default:
                break;
            }
        }
        // После дампа — "глотаем" все уведомления которые накопились
        // пока шёл дамп, чтобы не делать повторный дамп сразу
        ulTaskNotifyTake(pdTRUE, 0); // — неблокирующий сброс
        vTaskDelay(pdMS_TO_TICKS(5));

    }
    vTaskDelete(NULL);
}

void tud_cdc_tx_complete_cb(const uint8_t itf)
{
    if (xSemaphoreTake(usb_tx_requested, 0) != pdTRUE) {
        /* Semaphore should have been given before write attempt.
            Sometimes tinyusb can send one more cb even xfer_complete len is zero
        */
        return;
    }

    xSemaphoreGive(usb_tx_done);
}

void tud_cdc_rx_cb(const uint8_t itf)
{
    BaseType_t woken = pdFALSE;
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];

    const uint32_t rx_size = tud_cdc_n_read(itf, buf, CFG_TUD_CDC_RX_BUFSIZE);
    ESP_LOGI(TAG, "Total RX CDC USB Byte: %d", rx_size);
    if (rx_size > 0) {
        ESP_LOGD(TAG, "USB CDC -> Transport (%" PRIu32 " bytes)", rx_size);
        ESP_LOG_BUFFER_HEXDUMP("USB CDC -> Transport", buf, rx_size, ESP_LOG_DEBUG);  // <<-- Работаем в этом месте 

        if(rx_size != CDC_CMD_COMAND_SIZE){
            // Send to transport (could be UART, SPI, I2C, etc.)
            serial_handler_send_data(buf, rx_size);
        }else{
            for(uint8_t i = 0; i < CFG_TUD_CDC_RX_BUFSIZE; i++)
            {
                if( buf[i] == 0xAA )
                {
                    if(i < (CFG_TUD_CDC_RX_BUFSIZE - CDC_CMD_COMAND_SIZE) ) xQueueSend(queue_serial_RX, buf + i, 0);
                    break;
                }
            }
            serial_handler_send_data(buf, rx_size);
        }
        
    } else {
        ESP_LOGW(TAG, "tud_cdc_rx_cb receive error");
    }
}

void tud_cdc_line_coding_cb(const uint8_t itf, cdc_line_coding_t const *p_line_coding)
{
    if (serial_handler_set_baudrate(p_line_coding->bit_rate) != ESP_OK) {
        ESP_LOGE(TAG, "Could not set the baudrate to %" PRIu32, p_line_coding->bit_rate);
        eub_abort();
    }
}

void tud_cdc_line_state_cb(const uint8_t itf, const bool dtr, const bool rts)
{
    // The following transformation of DTR & RTS signals to BOOT & RST is done based on auto reset circutry shown in
    // schematics of ESP boards.

    // defaults for ((dtr && rts) || (!dtr && !rts))
    bool rst = true;
    bool boot = true;

    if (!dtr && rts) {
        rst = false;
        boot = true;
    } else if (dtr && !rts) {
        rst = true;
        boot = false;
    }

    esp_timer_stop(state_change_timer);  // maybe it is not started so not check the exit value

    if (dtr & rts) {
        // The assignment of BOOT=1 and RST=1 is postponed and it is done only if no other state change occurs in time
        // period set by the timer.
        // This is a patch for Esptool. Esptool generates DTR=0 & RTS=1 followed by DTR=1 & RTS=0. However, a callback
        // with DTR = 1 & RTS = 1 is received between. This would prevent to put the target chip into download mode.
        ESP_ERROR_CHECK(esp_timer_start_once(state_change_timer, 10 * 1000 /*us*/));

    } else {
        ESP_LOGI(TAG, "DTR = %d, RTS = %d -> BOOT = %d, RST = %d", dtr, rts, boot, rst);

        serial_handler_set_boot_reset_pins(boot, rst);

        if (!rst) {
            const uint32_t default_baud = 115200;
            if (serial_handler_set_baudrate(default_baud) != ESP_OK) {
                eub_abort();
            }
        }

        // On ESP32, TDI jtag signal is on GPIO12, which is also a strapping pin that determines flash voltage.
        // If TDI is high when ESP32 is released from external reset, the flash voltage is set to 1.8V, and the chip will fail to boot.
        // As a solution, MTDI signal forced to be low when RST is about to go high.
        if (boot) {
            debug_probe_handle_esp32_tdi_bootstrapping(!rst);
        }
    }
}

static void state_change_timer_cb(void *arg)
{
    ESP_LOGI(TAG, "BOOT = 1, RST = 1");
    serial_handler_set_boot_reset_pins(true, true); // BOOT=1, RST=1 (not in reset)
}

static void init_state_change_timer(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = state_change_timer_cb,
        .name = "serial_bridge_state_change"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &state_change_timer));
}

esp_err_t serial_bridge_init(void)
{
    // Create ring buffer for USB sending
    usb_sendbuf = xRingbufferCreate(USB_SEND_RINGBUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!usb_sendbuf) {
        ESP_LOGE(TAG, "Cannot create ringbuffer for USB sender");
        return ESP_ERR_NO_MEM;
    }

    // Create semaphores for USB TX synchronization
    usb_tx_done = xSemaphoreCreateBinary();
    usb_tx_requested = xSemaphoreCreateBinary();
    if (!usb_tx_done || !usb_tx_requested) {
        ESP_LOGE(TAG, "Cannot create USB TX semaphores");
        return ESP_ERR_NO_MEM;
    }

    // Register callback for transport data
    esp_err_t ret = serial_handler_register_data_callback(transport_data_received_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register transport data callback");
        return ret;
    }

    // Initialize state change timer
    init_state_change_timer();

    // Start USB sender task
    if( xTaskCreatePinnedToCore(usb_sender_task, "usb_sender_task", 4 * 1024, NULL, SERIAL_HANDLER_TASK_PRI, NULL, 0) != pdPASS )
    {
        ESP_LOGE(TAG, "Failed to create USB SENDER task");
    }
    
    if( xTaskCreatePinnedToCore(dump_task, "dump_sender_task", 6 * 1024, NULL, 7, &DumpTask_Handler, 1) != pdPASS )
    {
        ESP_LOGE(TAG, "Failed to create DUMP SSENDER task");
    }

    queue_serial_RX = xQueueCreate(4, CDC_CMD_COMAND_SIZE );

    if(queue_serial_RX == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue_serial_RX");
    }

    ESP_LOGI(TAG, "Serial bridge initialized");
    return ESP_OK;
}
