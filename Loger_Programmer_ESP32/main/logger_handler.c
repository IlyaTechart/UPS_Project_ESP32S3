#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "esp_err.h"
#include "frames_structure.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "logger_handler.h"


char *TAG = "Logger";

RingBuffModulData_t RingBuffModulData;
static SemaphoreHandle_t bufferMutex = NULL; // Мутекс для защиты памяти

RingBuffStatus_t RingBuffStatus = RINGBUF_OK;

// Глобальная структура для хранения RMS-значений
FpgaRmsData_t gFpgaRmsData;

static void logger_proc_task(void *pvParameters);
static void calculate_rms_from_buffer(void);

void logger_Inint(void)
{
    bufferMutex = xSemaphoreCreateMutex();        // Создаём тьютек
    if (bufferMutex == NULL) {
        ESP_LOGE(TAG, "Mutex create failed");
        return;
    }

    RingBuffModulData.buffer = (ModulData_t *)heap_caps_calloc(SIZE_OF_CIRCULAR_BUFFER, sizeof(ModulData_t), MALLOC_CAP_DEFAULT); // выделяем память 
    if (RingBuffModulData.buffer == NULL) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return;
    } 
    // Размер кольцевого буфера в элементах (ячейках), а не в байтах
    RingBuffModulData.size = SIZE_OF_CIRCULAR_BUFFER;
    RingBuffModulData.cell_size = sizeof(ModulData_t);
    RingBuffModulData.head = 0;
    RingBuffModulData.tail = 0;
    RingBuffModulData.is_full = false;

    if (xTaskCreate(logger_proc_task, "logger", 4096, NULL, 10, NULL) != pdPASS) {        // Создаём задачу
        ESP_LOGE(TAG, "Failed to create LOGGER task");
    }else{
        ESP_LOGI(TAG, "Logger Init Success");
    }
    
}

RingBuffStatus_t RingBuffWrite(ModulData_t* ModulData)
{
    if(RingBuffModulData.buffer == NULL || bufferMutex == NULL) return RINGBUF_NULL_POINTER;

    if(xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(10)) == pdTRUE)                                     //<<--!Осторожно временная задержка котораяя может сильно ограничивать скорость 
    {
        uint16_t next_head = (RingBuffModulData.head + 1) % RingBuffModulData.size;

        if(next_head == RingBuffModulData.tail)
        {
            //Можно здесь отбрасывать приянтые данные, но щас релизована перезапись старых
            RingBuffModulData.tail = (RingBuffModulData.tail + 1) % RingBuffModulData.size;
            // Теперь место под head освободилось (за счет потери старого элемента)
            memcpy(&RingBuffModulData.buffer[RingBuffModulData.head], ModulData, sizeof(ModulData_t));
            RingBuffModulData.head = next_head;
            xSemaphoreGive(bufferMutex);
            return RINGBUF_OVERFLOW;
        }else{
            memcpy(&RingBuffModulData.buffer[RingBuffModulData.head], ModulData, sizeof(ModulData_t));
            RingBuffModulData.head = next_head;
            xSemaphoreGive(bufferMutex);
            return RINGBUF_OK;
        }


    }else{
        return RINGBUF_MUTEX_NOT_GIVE;
    }

}


static void logger_proc_task(void *pvParameters)
{
    uint32_t last_time = (uint32_t)(esp_timer_get_time() / 1000); 
    for(;;)
    {


    }
}

