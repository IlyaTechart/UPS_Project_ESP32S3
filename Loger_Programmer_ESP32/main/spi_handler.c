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


#define CRC32_POLY 0xEDB88320

static const char *TAG = "SPI_HANDLER";
static const char *TAG_SPI = "TX_SPI";

// Очередь должна быть определена в .c файле
static QueueHandle_t spi_evt_queue;

SemaphoreHandle_t sema_for_driverTask;

// Буферы для драйвера: HEAP_CAPS_DMA обязателен для DMA-capable памяти на ESP32S3
// Выравнивание 16 байт требуется для L1CACHE на ESP32S3

volatile spi_buffers_t spi_buffers;
//EXT_RAM_BSS_ATTR volatile spi_message_t msg;
volatile spi_message_t msg;
volatile ModulData_t ModulData;


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
static void GPIO_Init_SPI2(void);
static void PrintUpsPacket(volatile FpgaToEspPacket_t *pkt);
uint32_t calculate_crc32(const void *data, size_t len);

void spi_slave_init(void) {

    memory_allocate();

    GPIO_Init_SPI2();

        gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    gpio_io_config_t gpio_io_config = {0};
    gpio_get_io_config(GPIO_CS, &gpio_io_config);

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
        memcpy(spi_buffers.pssram_tx_buffer, ModulData.Tx_Buffer, sizeof(ModulData.Tx_Buffer));

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
    memcpy(msg.data, trans->rx_buffer, bytes_rcv);
    // for(uint32_t i = 0; i < 1024; i++)
    // {
    //     msg.data[i] = *(uint8_t*)(trans->rx_buffer + i);
    // }
    
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
        //*(msg.data + 1023) = '\0';
        if((xQueueReceive(spi_evt_queue, &msg, portMAX_DELAY)) == pdPASS) 
        {
            if (msg.len < 4)
            {
            ESP_LOGW(TAG, "Received too short packet: %lu bytes", msg.len);
            // Пропускаем обработку, но обязательно отдаем семафор!
            xSemaphoreGive(sema_for_driverTask);
            continue; 
            }

            memcpy(ModulData.Tx_Buffer, msg.data, msg.len);

            if( ModulData.packet.crc32 != calculate_crc32(ModulData.Tx_Buffer, msg.len - 4) )
            {
                ESP_LOGE(TAG, "CRC is not correct!");
            }else{
                PrintUpsPacket(&ModulData.packet);
            }
            //PrintUpsPacket(&ModulData.packet);

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

static void GPIO_Init_SPI2(void)
{
    gpio_reset_pin(GPIO_MOSI);
    gpio_reset_pin(GPIO_MISO);
    gpio_reset_pin(GPIO_SCLK);
    gpio_reset_pin(GPIO_CS);

    // 2. Настраиваем структуру конфигурации
    gpio_config_t io_conf = {};

    // --- Настройка входов (CS, CLK, MOSI) ---
    // В режиме Slave ESP32 только СЛУШАЕТ эти линии.
    // Мы отключаем подтяжки (pull-up/down), чтобы проверить, 
    // сможет ли ПЛИС сама управлять линией.
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT; 
    io_conf.pin_bit_mask = (1ULL << GPIO_CS) | (1ULL << GPIO_SCLK) | (1ULL << GPIO_MOSI);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;   // Отключаем подтяжку, пусть ПЛИС рулит уровнем
    gpio_config(&io_conf);

    // --- Настройка выхода (MISO) ---
    // Это единственная линия, которую драйвит ESP32 в режиме Slave
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_MISO);
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    // Уставим MISO в 0 для начала
    gpio_set_level(GPIO_MISO, 0);

    printf("DEBUG: Pins configured as raw GPIO inputs (Floating).\n");

}

// Функция для красивого вывода данных (принимает указатель на структуру с float)
static void PrintUpsPacket(volatile FpgaToEspPacket_t *pkt) {
    if (pkt->start_marker != 0xAA55AA55) { // Проверка маркера (если он у вас такой)
        ESP_LOGW(TAG_SPI, "Invalid Start Marker: 0x%08lX", pkt->start_marker);
        // Можно не выходить, если хотите видеть мусор
    }

    ESP_LOGI(TAG_SPI, "=== UPS DATA (Cnt: %lu) ===", pkt->packet_counter);

    // 1. Статусы (uint16_t)
    ESP_LOGI(TAG_SPI, "[STATUS] Grid:%d | Byp:%d | Rect:%d | Inv:%d | PwrInv:%d | PwrByp:%d | Sync:%d",
             pkt->status.grid_status, pkt->status.bypass_grid_status,
             pkt->status.rectifier_status, pkt->status.inverter_status,
             pkt->status.pwr_via_inverter, pkt->status.pwr_via_bypass,
             pkt->status.sync_status);
    
    ESP_LOGI(TAG_SPI, "[STATUS] Load:%d | Sound:%d | BatSt:%d | UpsMode:%d",
             pkt->status.load_mode, pkt->status.sound_alarm,
             pkt->status.battery_status, pkt->status.ups_mode);

    // 2. Аварии (uint16_t)
    ESP_LOGI(TAG_SPI, "[ALARM] LowIn:%d | HighDC:%d | LowBat:%d | NoBat:%d | InvF:%d | InvOC:%d",
             pkt->alarms.err_low_input_vol, pkt->alarms.err_high_dc_bus,
             pkt->alarms.err_low_bat_charge, pkt->alarms.err_bat_not_conn,
             pkt->alarms.err_inv_fault, pkt->alarms.err_inv_overcurrent);

    // 3. Вход (Float)
    ESP_LOGI(TAG_SPI, "[INPUT] V_in: A=%.1f B=%.1f C=%.1f | V_byp: A=%.1f B=%.1f C=%.1f",
             pkt->input.v_in_AB, pkt->input.v_in_BC, pkt->input.v_in_CA,
             pkt->input.v_bypass_A, pkt->input.v_bypass_B, pkt->input.v_bypass_C);
    
    ESP_LOGI(TAG_SPI, "[INPUT] I_in: A=%.1f B=%.1f C=%.1f | Freq: %.2f Hz",
             pkt->input.i_in_A, pkt->input.i_in_B, pkt->input.i_in_C,
             pkt->input.freq_in);

    // 4. Выход (Float)
    ESP_LOGI(TAG_SPI, "[OUTPUT] V_out: A=%.1f B=%.1f C=%.1f | Freq: %.2f Hz",
             pkt->output.v_out_A, pkt->output.v_out_B, pkt->output.v_out_C,
             pkt->output.freq_out);

    ESP_LOGI(TAG_SPI, "[OUTPUT] I_out: A=%.1f B=%.1f C=%.1f",
             pkt->output.i_out_A, pkt->output.i_out_B, pkt->output.i_out_C);
    
    ESP_LOGI(TAG_SPI, "[OUTPUT] P_Act: A=%.1f B=%.1f C=%.1f kW",
             pkt->output.p_active_A, pkt->output.p_active_B, pkt->output.p_active_C);
    
    ESP_LOGI(TAG_SPI, "[OUTPUT] P_App: A=%.1f B=%.1f C=%.1f kVA",
             pkt->output.p_apparent_A, pkt->output.p_apparent_B, pkt->output.p_apparent_C);
    
    ESP_LOGI(TAG_SPI, "[OUTPUT] Load: A=%.1f%% B=%.1f%% C=%.1f%% | Events: %.0f",
             pkt->output.load_pct_A, pkt->output.load_pct_B, pkt->output.load_pct_C,
             pkt->output.event_count);

    // 5. Батарея (Float)
    ESP_LOGI(TAG_SPI, "[BAT] Vol: %.1f V | Cap: %.0f Ah | Grp: %.0f | DC: %.1f V",
             pkt->battery.bat_voltage, pkt->battery.bat_capacity,
             pkt->battery.bat_groups_count, pkt->battery.dc_bus_voltage);
    
    ESP_LOGI(TAG_SPI, "[BAT] Cur: %.1f A | Backup: %.0f min",
             pkt->battery.bat_current, pkt->battery.backup_time);

    ESP_LOGI(TAG_SPI, "[CRC] 0x%08lX", pkt->crc32);
    ESP_LOGI(TAG_SPI, "=================================");
}

/**
 * Функция вычисления CRC-32 (побитовый метод).
 * Этот метод компактный по коду, но медленнее табличного метода.
 *
 * @param data Указатель на буфер данных
 * @param len Длина буфера в байтах
 * @return Результат CRC-32
 */
uint32_t calculate_crc32(const void *data, size_t len) {
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFF; // Начальное значение (стандарт)

    for (size_t i = 0; i < len; i++) {

        crc ^= bytes[i];

        // Обработка 8 бит
        for (int j = 0; j < 8; j++) {
            // Если младший бит равен 1, делаем сдвиг и XOR с полиномом
            if (crc & 1) {
                crc = (crc >> 1) ^ CRC32_POLY;
            } else {
                crc = (crc >> 1);
            }
        }
    }

    crc = ~crc;

    return (uint32_t)crc;
}
