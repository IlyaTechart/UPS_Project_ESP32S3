#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_err.h"
#include "frames_structure.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "logger_handler.h"



char *TAG = "LOGER:";
static const char *TAG_RMS = "LOG_AVG";

// Для дебага 
RingBuffStatus_t RingBuffStatus;

// Переменная кольцевого буфера 
RingBuffModulData_t RingBuffModulData;

// Переменная сумматора (накопление в uint64)
AveSummator_t AVESummator;

// Буфер кадров в который копируется данные из расширенного буфера uint64, сюда 
ModulData_t ModulDataFromExtend = {0};

// Мутекс для защиты памяти
SemaphoreHandle_t bufferMutex = NULL; 

// Хендел задачи дампа 
extern TaskHandle_t DumpTask_Handler;
// Хендел задачи логера 
static TaskHandle_t TaskHeandler_Logger;


extern void logger_print_one_frame(const ModulData_t *m, size_t frame_index);

static void logger_proc_task(void *pvParameters);

void logger_Inint(void)
{
    bufferMutex = xSemaphoreCreateMutex();        // Создаём мьютекс
    if (bufferMutex == NULL) {
        ESP_LOGE(TAG, "Mutex create failed");
        return;
    }

    RingBuffModulData.buffer = (ModulData_t *)heap_caps_calloc(SIZE_OF_CIRCULAR_BUFFER, sizeof(ModulData_t), MALLOC_CAP_SPIRAM); // выделяем память 
    if (RingBuffModulData.buffer == NULL) {
        ESP_LOGE(TAG, "Memory allocation failed in PSRAM");
        return;
    } 
    RingBuffModulData.size_byte = sizeof(ModulData_t) * SIZE_OF_CIRCULAR_BUFFER; // Размер в байтах 
    RingBuffModulData.cnt_cpyes = SIZE_OF_CIRCULAR_BUFFER;
    RingBuffModulData.head = 0;
    RingBuffModulData.tail = 0;
    RingBuffModulData.count = 0;
    RingBuffModulData.is_full = false;

    memset(&AVESummator, 0, sizeof(AVESummator));

    if (xTaskCreatePinnedToCore(logger_proc_task, "logger", 8192, NULL, 5, &TaskHeandler_Logger, 1) != pdPASS) {        // Создаём задачу (стек 8KB — расчёт RMS + много ESP_LOGI)
        ESP_LOGE(TAG, "Failed to create LOGGER task");
    }else{
        ESP_LOGI(TAG, "Logger Init Success");
    }

    size_t heap_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    
    ESP_LOGI(TAG, "Free heap size: %u", heap_free);
   
}


// Функция возвращает текущее количество данных в буфере
size_t get_elements_count(RingBuffModulData_t *rb) {
    size_t capacity = rb->cnt_cpyes; // Общая вместимость буфера (максимальное кол-во элементов)
    
    if (rb->is_full) {
        return capacity;
    }
    
    if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    } else {
        return capacity - rb->tail + rb->head;
    }
}


void add_sample_in_average(ModulData_t* ModulData)
{
    if (ModulData == NULL) {
        return;
    }

    FpgaToEspPacket_t *p = &ModulData->packet;

    // Группа INPUT
    AVESummator.input.v_in_AB     += p->input.v_in_AB;
    AVESummator.input.v_in_BC     += p->input.v_in_BC;
    AVESummator.input.v_in_CA     += p->input.v_in_CA;
    AVESummator.input.v_bypass_A  += p->input.v_bypass_A;
    AVESummator.input.v_bypass_B  += p->input.v_bypass_B;
    AVESummator.input.v_bypass_C  += p->input.v_bypass_C;
    AVESummator.input.i_in_A      += p->input.i_in_A;
    AVESummator.input.i_in_B      += p->input.i_in_B;
    AVESummator.input.i_in_C      += p->input.i_in_C;
    AVESummator.input.freq_in     += p->input.freq_in;

    // Группа OUTPUT
    AVESummator.output.v_out_A      += p->output.v_out_A;
    AVESummator.output.v_out_B      += p->output.v_out_B;
    AVESummator.output.v_out_C      += p->output.v_out_C;
    AVESummator.output.freq_out     += p->output.freq_out;
    AVESummator.output.i_out_A      += p->output.i_out_A;
    AVESummator.output.i_out_B      += p->output.i_out_B;
    AVESummator.output.i_out_C      += p->output.i_out_C;
    AVESummator.output.p_active_A   += p->output.p_active_A;
    AVESummator.output.p_active_B   += p->output.p_active_B;
    AVESummator.output.p_active_C   += p->output.p_active_C;
    AVESummator.output.p_apparent_A += p->output.p_apparent_A;
    AVESummator.output.p_apparent_B += p->output.p_apparent_B;
    AVESummator.output.p_apparent_C += p->output.p_apparent_C;
    AVESummator.output.load_pct_A   += p->output.load_pct_A;
    AVESummator.output.load_pct_B   += p->output.load_pct_B;
    AVESummator.output.load_pct_C   += p->output.load_pct_C;
    AVESummator.output.event_count  += p->output.event_count;

    // Группа BATTERY
    AVESummator.battery.bat_voltage      += p->battery.bat_voltage;
    AVESummator.battery.bat_capacity     += p->battery.bat_capacity;
    AVESummator.battery.bat_groups_count += p->battery.bat_groups_count;
    AVESummator.battery.dc_bus_voltage   += p->battery.dc_bus_voltage;
    AVESummator.battery.bat_current      += p->battery.bat_current;
    AVESummator.battery.backup_time      += p->battery.backup_time;
}


void sub_sample_from_average(ModulData_t* ModulData)
{
    if (ModulData == NULL) {
        return;
    }

    FpgaToEspPacket_t *p = &ModulData->packet;

    // Группа INPUT
    AVESummator.input.v_in_AB     -= p->input.v_in_AB;
    AVESummator.input.v_in_BC     -= p->input.v_in_BC;
    AVESummator.input.v_in_CA     -= p->input.v_in_CA;
    AVESummator.input.v_bypass_A  -= p->input.v_bypass_A;
    AVESummator.input.v_bypass_B  -= p->input.v_bypass_B;
    AVESummator.input.v_bypass_C  -= p->input.v_bypass_C;
    AVESummator.input.i_in_A      -= p->input.i_in_A;
    AVESummator.input.i_in_B      -= p->input.i_in_B;
    AVESummator.input.i_in_C      -= p->input.i_in_C;
    AVESummator.input.freq_in     -= p->input.freq_in;

    // Группа OUTPUT
    AVESummator.output.v_out_A      -= p->output.v_out_A;
    AVESummator.output.v_out_B      -= p->output.v_out_B;
    AVESummator.output.v_out_C      -= p->output.v_out_C;
    AVESummator.output.freq_out     -= p->output.freq_out;
    AVESummator.output.i_out_A      -= p->output.i_out_A;
    AVESummator.output.i_out_B      -= p->output.i_out_B;
    AVESummator.output.i_out_C      -= p->output.i_out_C;
    AVESummator.output.p_active_A   -= p->output.p_active_A;
    AVESummator.output.p_active_B   -= p->output.p_active_B;
    AVESummator.output.p_active_C   -= p->output.p_active_C;
    AVESummator.output.p_apparent_A -= p->output.p_apparent_A;
    AVESummator.output.p_apparent_B -= p->output.p_apparent_B;
    AVESummator.output.p_apparent_C -= p->output.p_apparent_C;
    AVESummator.output.load_pct_A   -= p->output.load_pct_A;
    AVESummator.output.load_pct_B   -= p->output.load_pct_B;
    AVESummator.output.load_pct_C   -= p->output.load_pct_C;
    AVESummator.output.event_count  -= p->output.event_count;

    // Группа BATTERY
    AVESummator.battery.bat_voltage      -= p->battery.bat_voltage;
    AVESummator.battery.bat_capacity     -= p->battery.bat_capacity;
    AVESummator.battery.bat_groups_count -= p->battery.bat_groups_count;
    AVESummator.battery.dc_bus_voltage   -= p->battery.dc_bus_voltage;
    AVESummator.battery.bat_current      -= p->battery.bat_current;
    AVESummator.battery.backup_time      -= p->battery.backup_time;
}


// Расчёт скользящего среднего по окну из последних выборок (суммы в AVESummator).
static void calculate_moving_average(FpgaToEspPacket_t *pkt)
{

    if (RingBuffModulData.buffer == NULL) {
        return;
    }

    if (get_elements_count(&RingBuffModulData) < (SIZE_OF_CIRCULAR_BUFFER - NUMBER_OF_REMAINING_EMPTY)) {
        return;
    }

    size_t samples = RingBuffModulData.count;
    if (samples == 0) {
        return;
    }

    // Средние значения → поля пакета ModulDataFromExtend
    pkt->input.v_in_AB     = (uint16_t)(AVESummator.input.v_in_AB     / samples);
    pkt->input.v_in_BC     = (uint16_t)(AVESummator.input.v_in_BC     / samples);
    pkt->input.v_in_CA     = (uint16_t)(AVESummator.input.v_in_CA     / samples);
    pkt->input.v_bypass_A  = (uint16_t)(AVESummator.input.v_bypass_A  / samples);
    pkt->input.v_bypass_B  = (uint16_t)(AVESummator.input.v_bypass_B  / samples);
    pkt->input.v_bypass_C  = (uint16_t)(AVESummator.input.v_bypass_C  / samples);
    pkt->input.i_in_A      = (uint16_t)(AVESummator.input.i_in_A      / samples);
    pkt->input.i_in_B      = (uint16_t)(AVESummator.input.i_in_B      / samples);
    pkt->input.i_in_C      = (uint16_t)(AVESummator.input.i_in_C      / samples);
    pkt->input.freq_in     = (uint16_t)(AVESummator.input.freq_in     / samples);

    pkt->output.v_out_A      = (uint16_t)(AVESummator.output.v_out_A      / samples);
    pkt->output.v_out_B      = (uint16_t)(AVESummator.output.v_out_B      / samples);
    pkt->output.v_out_C      = (uint16_t)(AVESummator.output.v_out_C      / samples);
    pkt->output.freq_out     = (uint16_t)(AVESummator.output.freq_out     / samples);
    pkt->output.i_out_A      = (uint16_t)(AVESummator.output.i_out_A      / samples);
    pkt->output.i_out_B      = (uint16_t)(AVESummator.output.i_out_B      / samples);
    pkt->output.i_out_C      = (uint16_t)(AVESummator.output.i_out_C      / samples);
    pkt->output.p_active_A   = (uint16_t)(AVESummator.output.p_active_A   / samples);
    pkt->output.p_active_B   = (uint16_t)(AVESummator.output.p_active_B   / samples);
    pkt->output.p_active_C   = (uint16_t)(AVESummator.output.p_active_C   / samples);
    pkt->output.p_apparent_A = (uint16_t)(AVESummator.output.p_apparent_A / samples);
    pkt->output.p_apparent_B = (uint16_t)(AVESummator.output.p_apparent_B / samples);
    pkt->output.p_apparent_C = (uint16_t)(AVESummator.output.p_apparent_C / samples);
    pkt->output.load_pct_A   = (uint16_t)(AVESummator.output.load_pct_A   / samples);
    pkt->output.load_pct_B   = (uint16_t)(AVESummator.output.load_pct_B   / samples);
    pkt->output.load_pct_C   = (uint16_t)(AVESummator.output.load_pct_C   / samples);
    pkt->output.event_count  = (uint16_t)(AVESummator.output.event_count  / samples);

    pkt->battery.bat_voltage      = (uint16_t)(AVESummator.battery.bat_voltage      / samples);
    pkt->battery.bat_capacity     = (uint16_t)(AVESummator.battery.bat_capacity     / samples);
    pkt->battery.bat_groups_count = (uint16_t)(AVESummator.battery.bat_groups_count / samples);
    pkt->battery.dc_bus_voltage   = (uint16_t)(AVESummator.battery.dc_bus_voltage   / samples);
    pkt->battery.bat_current      = (uint16_t)(AVESummator.battery.bat_current      / samples);
    pkt->battery.backup_time      = (uint16_t)(AVESummator.battery.backup_time      / samples);

    pkt->system_time_ms = xTaskGetTickCount();
}

RingBuffStatus_t RingBuffWrite(ModulData_t* ModulData)
{
    if(RingBuffModulData.buffer == NULL || bufferMutex == NULL) return RINGBUF_NULL_POINTER;

    if(xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(10)) == pdTRUE)                                     
    {
        size_t next_head = (RingBuffModulData.head + 1) % RingBuffModulData.cnt_cpyes;
        if(next_head == RingBuffModulData.tail)
        {
            sub_sample_from_average(&RingBuffModulData.buffer[RingBuffModulData.tail]);
            add_sample_in_average(ModulData);
            memcpy(&RingBuffModulData.buffer[RingBuffModulData.tail], ModulData, sizeof(ModulData_t));

            RingBuffModulData.tail = (RingBuffModulData.tail + 1) % RingBuffModulData.cnt_cpyes;
            RingBuffModulData.head = next_head;
            RingBuffModulData.is_full = true;
            xSemaphoreGive(bufferMutex);
            return RINGBUF_OVERFLOW;
        }else{
            add_sample_in_average(ModulData);
            RingBuffModulData.count++;
            memcpy(&RingBuffModulData.buffer[RingBuffModulData.head], ModulData, sizeof(ModulData_t));
            RingBuffModulData.head = next_head;
            RingBuffModulData.is_full = false;
            xSemaphoreGive(bufferMutex);
            return RINGBUF_OK;
        }


    }else{
        return RINGBUF_MUTEX_NOT_GIVE;
    }

}


void time_calculate_DEBUG(RingBuffModulData_t *rb)
{
    char* TAG_TIME = "Time sub";

    FpgaToEspPacket_t* pkTail = &rb->buffer[rb->tail].packet;

    size_t last_written_idx = (rb->head + rb->cnt_cpyes - 1) % rb->cnt_cpyes;
    FpgaToEspPacket_t* pkHead = &rb->buffer[last_written_idx].packet;

    // Вычитаем из НОВОГО времени СТАРОЕ (а не наоборот, чтобы не было переполнения uint32_t)
    uint32_t deltaTime_ms = pkHead->system_time_ms - pkTail->system_time_ms;
    uint32_t deltaTime_sec = deltaTime_ms / 1000; 
    uint32_t deltaTime_min = deltaTime_sec / 60; 
    
    ESP_LOGI(TAG_TIME, "Delta: %u ms, Elements: %d, Buf_status: %s", (unsigned)deltaTime_ms, get_elements_count(rb), rb->is_full? "FULL" : "NOT FULL");
    ESP_LOGI(TAG_TIME, "Delta Sec: %u sec, Minutes %u,%u min", (unsigned)deltaTime_sec, (unsigned)deltaTime_min, (unsigned)deltaTime_sec % 60 );
}

static void logger_proc_task(void *pvParameters)
{
    const TickType_t xFreqCalcMovAverage =  pdMS_TO_TICKS(PERIOD_MS_MOV_AVRAGE);
    const TickType_t xFreqSendFrames =  pdMS_TO_TICKS(PERIOD_MS_SAND_FRAMES);
    const TickType_t xFreqDebugPrint =  pdMS_TO_TICKS(PERIOD_MS_DEBUG_PRINT);

    TickType_t xLastCalcMovAverage = xTaskGetTickCount();
    TickType_t xLastSendFrames = xTaskGetTickCount();
    TickType_t xLastDebugPrint = xTaskGetTickCount();



    for(;;)
    {
        TickType_t xCurrentTick = xTaskGetTickCount();
        /* ── Безопасная точка остановки ──────────────────────────────────
         * Мьютекс НЕ захвачен. Проверяем уведомление от logger_suspend().
         * ulTaskNotifyTake(pdTRUE, 0) — неблокирующая проверка:
         *   - если уведомление есть (> 0) → сбрасываем счётчик и засыпаем;
         *   - если нет (== 0) → продолжаем работу.
         * После vTaskResume() снаружи задача продолжит отсюда. */
        if (ulTaskNotifyTake(pdTRUE, 0) > 0)
        {
            ESP_LOGI(TAG, "Logger: suspending itself safely");
            vTaskSuspend(NULL);
            /* ── возобновление после vTaskResume() ── */
            ESP_LOGI(TAG, "Logger: resumed");
        }
        if ( (xCurrentTick - xLastCalcMovAverage) >= xFreqCalcMovAverage )
        {
            xLastCalcMovAverage = xTaskGetTickCount();
            if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                calculate_moving_average(&ModulDataFromExtend.packet);
                xSemaphoreGive(bufferMutex);
                // UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
                // ESP_LOGI(TAG, "logger stack free: %u bytes", (unsigned)(hwm * sizeof(StackType_t)));
            } 
        }

        if (RingBuffModulData.head > 0)
        {
            if(RingBuffModulData.buffer[RingBuffModulData.head - 1].packet.alarms.raw)
            {
                xTaskNotify(DumpTask_Handler, SEND_DUMP_COMAND, eSetValueWithOverwrite);
                ESP_LOGI(TAG, "Send Task DUMP Notify");
            }
        }
        //TODO   (часть для отправки фрейма со средними значениями нужно сделать в синхронном режиме для USB устройств)
        if ( (xCurrentTick - xLastSendFrames) >= xFreqSendFrames )
        {
            xLastSendFrames = xTaskGetTickCount();
            if(RingBuffModulData.count > 0)
            {
                xTaskNotify(DumpTask_Handler, SEND_AVE_COMAND, eSetValueWithOverwrite);
                ESP_LOGI(TAG, "Send Task AVE Notify");
            
            }
        }
        
        // Участок для логирования информации раз в 1 сек. 
        /////////////////////////////////////////////////////////////////////////////////////////////
        if ( (xCurrentTick - xLastDebugPrint) >= xFreqDebugPrint ) {
            xLastDebugPrint = xTaskGetTickCount();
            if(RingBuffModulData.head > 0)
            {
                    logger_print_one_frame(&RingBuffModulData.buffer[RingBuffModulData.head - 1], 1);
            }

            time_calculate_DEBUG(&RingBuffModulData);
            //logger_print_avg_data(&snapshot);
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        // Если задача требует боольшого процессорного рвемя, то может сработать WDT и убить её, поскольку 
        // в idl задачи не будет выделено время для сброса WDT 
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}