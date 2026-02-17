#include "spi_handler.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
//#include "esp_cache.h"

static const char *TAG = "SPI_HANDLER";

// Очередь должна быть определена в .c файле
static QueueHandle_t spi_evt_queue;

SemaphoreHandle_t sema_for_driverTask;

// Буферы для драйвера: HEAP_CAPS_DMA обязателен для DMA-capable памяти на ESP32S3
// Выравнивание 16 байт требуется для L1CACHE на ESP32S3

volatile spi_buffers_t spi_buffers;
//EXT_RAM_BSS_ATTR volatile spi_message_t msg;
volatile spi_message_t msg;
volatile TransferBuffer_t TransferBuffer;


//Переменные для логов 
uint8_t LogBuffer[11] = {0};
uint8_t LogFromDriverTask = 0;
uint8_t LogFromISR = 0;
uint8_t LogFromProcessingTask = 0;


// Прототипы функций (чтобы init их видел)
void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans);
void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans);
static void spi_driver_task(void *pvParameters); // Внутренняя задача драйвера
void memory_allocate(void);

void spi_slave_init(void) {

     memory_allocate();

    // 1. Создаем очередь
    spi_evt_queue = xQueueCreate(2, sizeof(spi_message_t));

    if (spi_evt_queue == NULL) {
        ESP_LOGE(TAG, "Error creating queue");
        return;
    }
    
    // 2. Настраиваем шину SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
        //.max_transfer_sz = CURRENT_SIZE + 1,
        //.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO // Лучше AUTO
    };

    // 3. Настраиваем интерфейс слейва
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb, 
        .post_trans_cb = my_post_trans_cb
    };

    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    // ВАЖНО: Используем SPI2_HOST (RCV_HOST определен в хедере)
    esp_err_t ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init SPI: %s", esp_err_to_name(ret));
        return;
    }

    sema_for_driverTask = xSemaphoreCreateBinary();
    if (sema_for_driverTask == NULL) {
        ESP_LOGE(TAG, "Error creating semaphore");
        return;
    }
    
    // Выдаём семафор один раз, чтобы задача могла начать работу
    xSemaphoreGive(sema_for_driverTask);

    // 5. ЗАПУСК ДРАЙВЕРА (Обязательно!)
    // Без этой задачи контроллер SPI не будет знать, куда принимать данные
    BaseType_t TaskReturned = xTaskCreate(spi_driver_task, "spi_driver", 4096, NULL, 5, NULL);
    if(TaskReturned != pdPASS)
    {
        ESP_LOGE(TAG, "IS NOT CREATED: spi_driver");
    }


    ESP_LOGI(TAG, "SPI Init done.");
}

// Задача, которая постоянно "кормит" драйвер буферами
static void spi_driver_task(void *pvParameters) {
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    while (1) {
        // Очищаем буфер приема
        memset(spi_buffers.pssram_rx_buffer, 0, CURRENT_SIZE);

        // Настраиваем транзакцию
        t.length =  CURRENT_SIZE * 8; // Размер в битах!
        t.tx_buffer = spi_buffers.pssram_tx_buffer;
        t.rx_buffer = spi_buffers.pssram_rx_buffer;
//        t.flags = SPI_SLAVE_TRANS_DMA_BUFFER_ALIGN_AUTO; // Разрешаем автопереаллокацию если нужна
        LogFromDriverTask++;

        // Ждем данные от мастера
        volatile esp_err_t ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI trans failed: %s", esp_err_to_name(ret));
        }

        xSemaphoreTake(sema_for_driverTask, portMAX_DELAY);
    }
}

// Обработчик прерывания (вызывается ПОСЛЕ приема данных)
void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(GPIO_HANDSHAKE, 1);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Вычисляем сколько байт реально пришло
    volatile uint32_t bytes_rcv = trans->trans_len / 8;
    if (bytes_rcv > sizeof(msg.data)) bytes_rcv = sizeof(msg.data);
    
    // Копируем данные из буфера драйвера в сообщение очереди
    //memcpy(msg.data, trans->rx_buffer, bytes_rcv);
    for(uint32_t i = 0; i < 1024; i++)
    {
        msg.data[i] = *(uint8_t*)(trans->rx_buffer + i);
    }
    LogFromISR++;

    msg.len = bytes_rcv;

    // Отправляем в очередь
    xQueueSendFromISR(spi_evt_queue, &msg, &xHigherPriorityTaskWoken);
    //gpio_set_level(GPIO_HANDSHAKE, 0);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// Пустышка для setup (вызывается ПЕРЕД транзакцией)
void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans) {
//    gpio_set_level(GPIO_HANDSHAKE, 0);
    // Здесь можно выставить GPIO в 1, чтобы сигнализировать Мастеру "Я готов"
}

// Задача обработки (Пользовательская логика)
void spi_processing_task(void *pvParameters) {
    ESP_LOGI(TAG, "SPI Processing task started");
    uint32_t cnt_msg;

    while(1) {
        // Ждем сообщения из очереди
        LogFromProcessingTask++;
        *(msg.data + 1023) = '\0';
        if((xQueueReceive(spi_evt_queue, &msg, portMAX_DELAY)) == pdPASS) 
        {
            memcpy(TransferBuffer.buffer, msg.data, CURRENT_SIZE);
            // Фаза A
            ESP_LOGI(TAG, "Phase A: Vin=%.2f V, Iin=%.2f A | P_Act=%.2f kW, P_Rea=%.2f kVar",
                TransferBuffer.ups_data[0].v_in_a,
                TransferBuffer.ups_data[0].c_in_a,
                TransferBuffer.ups_data[0].p_act_a,
                TransferBuffer.ups_data[0].p_rea_a
            );

            // Фаза B
            ESP_LOGI(TAG, "Phase B: Vin=%.2f V, Iin=%.2f A | P_Act=%.2f kW, P_Rea=%.2f kVar",
                TransferBuffer.ups_data[0].v_in_b,
                TransferBuffer.ups_data[0].c_in_b,
                TransferBuffer.ups_data[0].p_act_b,
                TransferBuffer.ups_data[0].p_rea_b
            );

            // Фаза C
            ESP_LOGI(TAG, "Phase C: Vin=%.2f V, Iin=%.2f A | P_Act=%.2f kW, P_Rea=%.2f kVar",
                TransferBuffer.ups_data[0].v_in_c,
                TransferBuffer.ups_data[0].c_in_c,
                TransferBuffer.ups_data[0].p_act_c,
                TransferBuffer.ups_data[0].p_rea_c
            );
            //ESP_LOG_BUFFER_HEX(TAG, msg.data, sizeof(msg.data));
        }

        xSemaphoreGive(sema_for_driverTask);
        //ESP_LOG_BUFFER_HEX(TAG, msg.data, msg.len);
            
        
    }
}

void memory_allocate(void)
{
    // Выделяем DMA-буферы из внутренней DRAM
    //spi_buffers.pssram_rx_buffer = (uint8_t*)heap_caps_aligned_alloc(32, CURRENT_SIZE,  MALLOC_CAP_DMA);
    //spi_buffers.pssram_tx_buffer = (uint8_t*)heap_caps_aligned_alloc(32, CURRENT_SIZE,  MALLOC_CAP_DMA);

    spi_buffers.pssram_rx_buffer = (uint8_t*)spi_bus_dma_memory_alloc(RCV_HOST, CURRENT_SIZE, MALLOC_CAP_DMA);
    spi_buffers.pssram_tx_buffer = (uint8_t*)spi_bus_dma_memory_alloc(RCV_HOST, CURRENT_SIZE, MALLOC_CAP_DMA);

    if ((spi_buffers.pssram_rx_buffer == NULL) || (spi_buffers.pssram_tx_buffer == NULL)) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffers");
        return;
    }
    
    memset(spi_buffers.pssram_rx_buffer, 0, CURRENT_SIZE);
    memset(spi_buffers.pssram_tx_buffer, 0, CURRENT_SIZE);

    ESP_LOGI(TAG, "RX buffer @ 0x%p, TX buffer @ 0x%p", spi_buffers.pssram_rx_buffer, spi_buffers.pssram_tx_buffer);
    
    // Проверяем, что буферы действительно в DMA-capable памяти
//     multi_heap_info_t info = {0};
//     heap_caps_get_info(&info, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
//     ESP_LOGI(TAG, "DMA-capable SPIRAM: total=%zu, free=%zu", info.total_allocated_bytes + info.total_free_bytes, info.total_free_bytes);
}

