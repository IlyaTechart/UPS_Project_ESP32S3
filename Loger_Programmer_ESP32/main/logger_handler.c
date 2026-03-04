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


char *TAG = "LOGER";
static const char *TAG_RMS = "LOG_AVG";

// Переменная кольцевого буфера 
RingBuffModulData_t RingBuffModulData;

static SemaphoreHandle_t bufferMutex = NULL; // Мутекс для защиты памяти

//Состояние кольцевого буфера
RingBuffStatus_t RingBuffStatus = RINGBUF_OK;

// Состояние UPS
UpsRegisterFlags_t UpsRegisterFlags;

// Глобальная структура для средних значений по данным из кольцевого буфера
FpgaRmsData_t gFpgaAvrData;

static size_t get_elements_count(RingBuffModulData_t *rb);
void add_sample_in_average(ModulData_t* ModulData);
void sub_sample_from_average(ModulData_t* ModulData);
static void calculate_moving_average_from_buffer(void);
static void logger_print_avg_data(const FpgaRmsData_t *avg);
static void print_error_flag_frame(RingBuffModulData_t *RingBuffModulData, UpsRegisterFlags_t *UpsRegisterFlags);
void time_calculate_DEBUG(RingBuffModulData_t *RingBuffModulData);
static void logger_proc_task(void *pvParameters);

void logger_Inint(void)
{
    bufferMutex = xSemaphoreCreateMutex();        // Создаём тьютек
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

    if (xTaskCreate(logger_proc_task, "logger", 8192, NULL, 5, NULL) != pdPASS) {        // Создаём задачу (стек 8KB — расчёт RMS + много ESP_LOGI)
        ESP_LOGE(TAG, "Failed to create LOGGER task");
    }else{
        ESP_LOGI(TAG, "Logger Init Success");
    }

    size_t heap_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    
    ESP_LOGI(TAG, "Free heap size: %u", heap_free);

    
}

// Функция возвращает текущее количество данных в буфере
static size_t get_elements_count(RingBuffModulData_t *rb) {
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
    RingBuffModulData.FPGA_mov_averge.input.v_in_AB     += p->input.v_in_AB;
    RingBuffModulData.FPGA_mov_averge.input.v_in_BC     += p->input.v_in_BC;
    RingBuffModulData.FPGA_mov_averge.input.v_in_CA     += p->input.v_in_CA;
    RingBuffModulData.FPGA_mov_averge.input.v_bypass_A  += p->input.v_bypass_A;
    RingBuffModulData.FPGA_mov_averge.input.v_bypass_B  += p->input.v_bypass_B;
    RingBuffModulData.FPGA_mov_averge.input.v_bypass_C  += p->input.v_bypass_C;
    RingBuffModulData.FPGA_mov_averge.input.i_in_A      += p->input.i_in_A;
    RingBuffModulData.FPGA_mov_averge.input.i_in_B      += p->input.i_in_B;
    RingBuffModulData.FPGA_mov_averge.input.i_in_C      += p->input.i_in_C;
    RingBuffModulData.FPGA_mov_averge.input.freq_in     += p->input.freq_in;

    // Группа OUTPUT
    RingBuffModulData.FPGA_mov_averge.output.v_out_A      += p->output.v_out_A;
    RingBuffModulData.FPGA_mov_averge.output.v_out_B      += p->output.v_out_B;
    RingBuffModulData.FPGA_mov_averge.output.v_out_C      += p->output.v_out_C;
    RingBuffModulData.FPGA_mov_averge.output.freq_out     += p->output.freq_out;
    RingBuffModulData.FPGA_mov_averge.output.i_out_A      += p->output.i_out_A;
    RingBuffModulData.FPGA_mov_averge.output.i_out_B      += p->output.i_out_B;
    RingBuffModulData.FPGA_mov_averge.output.i_out_C      += p->output.i_out_C;
    RingBuffModulData.FPGA_mov_averge.output.p_active_A   += p->output.p_active_A;
    RingBuffModulData.FPGA_mov_averge.output.p_active_B   += p->output.p_active_B;
    RingBuffModulData.FPGA_mov_averge.output.p_active_C   += p->output.p_active_C;
    RingBuffModulData.FPGA_mov_averge.output.p_apparent_A += p->output.p_apparent_A;
    RingBuffModulData.FPGA_mov_averge.output.p_apparent_B += p->output.p_apparent_B;
    RingBuffModulData.FPGA_mov_averge.output.p_apparent_C += p->output.p_apparent_C;
    RingBuffModulData.FPGA_mov_averge.output.load_pct_A   += p->output.load_pct_A;
    RingBuffModulData.FPGA_mov_averge.output.load_pct_B   += p->output.load_pct_B;
    RingBuffModulData.FPGA_mov_averge.output.load_pct_C   += p->output.load_pct_C;
    RingBuffModulData.FPGA_mov_averge.output.event_count  += p->output.event_count;

    // Группа BATTERY
    RingBuffModulData.FPGA_mov_averge.battery.bat_voltage      += p->battery.bat_voltage;
    RingBuffModulData.FPGA_mov_averge.battery.bat_capacity     += p->battery.bat_capacity;
    RingBuffModulData.FPGA_mov_averge.battery.bat_groups_count += p->battery.bat_groups_count;
    RingBuffModulData.FPGA_mov_averge.battery.dc_bus_voltage   += p->battery.dc_bus_voltage;
    RingBuffModulData.FPGA_mov_averge.battery.bat_current      += p->battery.bat_current;
    RingBuffModulData.FPGA_mov_averge.battery.backup_time      += p->battery.backup_time;
}

void sub_sample_from_average(ModulData_t* ModulData)
{
    if (ModulData == NULL) {
        return;
    }

    FpgaToEspPacket_t *p = &ModulData->packet;

    // Группа INPUT
    RingBuffModulData.FPGA_mov_averge.input.v_in_AB     -= p->input.v_in_AB;
    RingBuffModulData.FPGA_mov_averge.input.v_in_BC     -= p->input.v_in_BC;
    RingBuffModulData.FPGA_mov_averge.input.v_in_CA     -= p->input.v_in_CA;
    RingBuffModulData.FPGA_mov_averge.input.v_bypass_A  -= p->input.v_bypass_A;
    RingBuffModulData.FPGA_mov_averge.input.v_bypass_B  -= p->input.v_bypass_B;
    RingBuffModulData.FPGA_mov_averge.input.v_bypass_C  -= p->input.v_bypass_C;
    RingBuffModulData.FPGA_mov_averge.input.i_in_A      -= p->input.i_in_A;
    RingBuffModulData.FPGA_mov_averge.input.i_in_B      -= p->input.i_in_B;
    RingBuffModulData.FPGA_mov_averge.input.i_in_C      -= p->input.i_in_C;
    RingBuffModulData.FPGA_mov_averge.input.freq_in     -= p->input.freq_in;

    // Группа OUTPUT
    RingBuffModulData.FPGA_mov_averge.output.v_out_A      -= p->output.v_out_A;
    RingBuffModulData.FPGA_mov_averge.output.v_out_B      -= p->output.v_out_B;
    RingBuffModulData.FPGA_mov_averge.output.v_out_C      -= p->output.v_out_C;
    RingBuffModulData.FPGA_mov_averge.output.freq_out     -= p->output.freq_out;
    RingBuffModulData.FPGA_mov_averge.output.i_out_A      -= p->output.i_out_A;
    RingBuffModulData.FPGA_mov_averge.output.i_out_B      -= p->output.i_out_B;
    RingBuffModulData.FPGA_mov_averge.output.i_out_C      -= p->output.i_out_C;
    RingBuffModulData.FPGA_mov_averge.output.p_active_A   -= p->output.p_active_A;
    RingBuffModulData.FPGA_mov_averge.output.p_active_B   -= p->output.p_active_B;
    RingBuffModulData.FPGA_mov_averge.output.p_active_C   -= p->output.p_active_C;
    RingBuffModulData.FPGA_mov_averge.output.p_apparent_A -= p->output.p_apparent_A;
    RingBuffModulData.FPGA_mov_averge.output.p_apparent_B -= p->output.p_apparent_B;
    RingBuffModulData.FPGA_mov_averge.output.p_apparent_C -= p->output.p_apparent_C;
    RingBuffModulData.FPGA_mov_averge.output.load_pct_A   -= p->output.load_pct_A;
    RingBuffModulData.FPGA_mov_averge.output.load_pct_B   -= p->output.load_pct_B;
    RingBuffModulData.FPGA_mov_averge.output.load_pct_C   -= p->output.load_pct_C;
    RingBuffModulData.FPGA_mov_averge.output.event_count  -= p->output.event_count;

    // Группа BATTERY
    RingBuffModulData.FPGA_mov_averge.battery.bat_voltage      -= p->battery.bat_voltage;
    RingBuffModulData.FPGA_mov_averge.battery.bat_capacity     -= p->battery.bat_capacity;
    RingBuffModulData.FPGA_mov_averge.battery.bat_groups_count -= p->battery.bat_groups_count;
    RingBuffModulData.FPGA_mov_averge.battery.dc_bus_voltage   -= p->battery.dc_bus_voltage;
    RingBuffModulData.FPGA_mov_averge.battery.bat_current      -= p->battery.bat_current;
    RingBuffModulData.FPGA_mov_averge.battery.backup_time      -= p->battery.backup_time;
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
            //Можно здесь отбрасывать приянтые данные, но щас релизована перезапись старых
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

// Расчёт скользящего среднего по окну из последних выборок,
// используя накопленные суммы в FPGA_mov_averge.
static void calculate_moving_average_from_buffer(void)
{
    if (RingBuffModulData.buffer == NULL){
        return;
    }

    // Ждём, пока в буфере наберётся достаточно выборок
    if (get_elements_count(&RingBuffModulData) < (SIZE_OF_CIRCULAR_BUFFER - NUMBER_OF_REMAINING_EMPTY)) {
        return;
    }

    size_t samples = RingBuffModulData.count;
    if (samples == 0) {
        return;
    }

    // Средние значения по группам input/output/battery
    gFpgaAvrData.input.v_in_AB      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.input.v_in_AB      / samples);
    gFpgaAvrData.input.v_in_BC      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.input.v_in_BC      / samples);
    gFpgaAvrData.input.v_in_CA      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.input.v_in_CA      / samples);
    gFpgaAvrData.input.v_bypass_A   = (uint16_t)(RingBuffModulData.FPGA_mov_averge.input.v_bypass_A   / samples);
    gFpgaAvrData.input.v_bypass_B   = (uint16_t)(RingBuffModulData.FPGA_mov_averge.input.v_bypass_B   / samples);
    gFpgaAvrData.input.v_bypass_C   = (uint16_t)(RingBuffModulData.FPGA_mov_averge.input.v_bypass_C   / samples);
    gFpgaAvrData.input.i_in_A       = (uint16_t)(RingBuffModulData.FPGA_mov_averge.input.i_in_A       / samples);
    gFpgaAvrData.input.i_in_B       = (uint16_t)(RingBuffModulData.FPGA_mov_averge.input.i_in_B       / samples);
    gFpgaAvrData.input.i_in_C       = (uint16_t)(RingBuffModulData.FPGA_mov_averge.input.i_in_C       / samples);
    gFpgaAvrData.input.freq_in      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.input.freq_in      / samples);

    gFpgaAvrData.output.v_out_A      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.v_out_A      / samples);
    gFpgaAvrData.output.v_out_B      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.v_out_B      / samples);
    gFpgaAvrData.output.v_out_C      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.v_out_C      / samples);
    gFpgaAvrData.output.freq_out     = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.freq_out     / samples);
    gFpgaAvrData.output.i_out_A      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.i_out_A      / samples);
    gFpgaAvrData.output.i_out_B      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.i_out_B      / samples);
    gFpgaAvrData.output.i_out_C      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.i_out_C      / samples);
    gFpgaAvrData.output.p_active_A   = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.p_active_A   / samples);
    gFpgaRmsData.output.p_active_B   = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.p_active_B   / samples);
    gFpgaAvrData.output.p_active_C   = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.p_active_C   / samples);
    gFpgaAvrData.output.p_apparent_A = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.p_apparent_A / samples);
    gFpgaAvrData.output.p_apparent_B = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.p_apparent_B / samples);
    gFpgaAvrData.output.p_apparent_C = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.p_apparent_C / samples);
    gFpgaAvrData.output.load_pct_A   = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.load_pct_A   / samples);
    gFpgaAvrData.output.load_pct_B   = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.load_pct_B   / samples);
    gFpgaAvrData.output.load_pct_C   = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.load_pct_C   / samples);
    gFpgaAvrData.output.event_count  = (uint16_t)(RingBuffModulData.FPGA_mov_averge.output.event_count  / samples);

    gFpgaAvrData.battery.bat_voltage      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.battery.bat_voltage      / samples);
    gFpgaAvrData.battery.bat_capacity     = (uint16_t)(RingBuffModulData.FPGA_mov_averge.battery.bat_capacity     / samples);
    gFpgaAvrData.battery.bat_groups_count = (uint16_t)(RingBuffModulData.FPGA_mov_averge.battery.bat_groups_count / samples);
    gFpgaAvrData.battery.dc_bus_voltage   = (uint16_t)(RingBuffModulData.FPGA_mov_averge.battery.dc_bus_voltage   / samples);
    gFpgaAvrData.battery.bat_current      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.battery.bat_current      / samples);
    gFpgaAvrData.battery.backup_time      = (uint16_t)(RingBuffModulData.FPGA_mov_averge.battery.backup_time      / samples);
}

// Вывод средних значений по образу spi_slave_print_ups_packet,
// но без полей заголовка/CRC, только группы status/alarms/input/output/battery.
static void logger_print_avg_data(const FpgaRmsData_t *r)
{
    const char *src = "AVG";

    ESP_LOGI(TAG_RMS, "[%s] === UPS AVG DATA ===", src);
    ESP_LOGI(TAG_RMS, "[%s] [STATUS] Grid:%u Byp:%u Rect:%u Inv:%u PwrInv:%u PwrByp:%u Sync:%u",
             src,
             (unsigned)r->status.grid_status, (unsigned)r->status.bypass_grid_status,
             (unsigned)r->status.rectifier_status, (unsigned)r->status.inverter_status,
             (unsigned)r->status.pwr_via_inverter, (unsigned)r->status.pwr_via_bypass,
             (unsigned)r->status.sync_status);
    ESP_LOGI(TAG_RMS, "[%s] [STATUS] Load:%u Sound:%u BatSt:%u UpsMode:%u",
             src,
             (unsigned)r->status.load_mode, (unsigned)r->status.sound_alarm,
             (unsigned)r->status.battery_status, (unsigned)r->status.ups_mode);

    ESP_LOGI(TAG_RMS, "[%s] [ALARM] LowIn:%u HighDC:%u LowBat:%u NoBat:%u InvF:%u InvOC:%u HiOut:%u Fan:%u ReplBat:%u RectHot:%u InvHot:%u",
             src,
             (unsigned)r->alarms.err_low_input_vol, (unsigned)r->alarms.err_high_dc_bus,
             (unsigned)r->alarms.err_low_bat_charge, (unsigned)r->alarms.err_bat_not_conn,
             (unsigned)r->alarms.err_inv_fault, (unsigned)r->alarms.err_inv_overcurrent,
             (unsigned)r->alarms.err_high_out_vol, (unsigned)r->alarms.err_fan_fault,
             (unsigned)r->alarms.err_replace_bat, (unsigned)r->alarms.err_rect_overheat,
             (unsigned)r->alarms.err_inv_overheat);

    // INPUT: x0.1 для напряжений и токов, x0.01 для частоты
    ESP_LOGI(TAG_RMS, "[%s] [INPUT] V_in: A=%u.%u B=%u.%u C=%u.%u | V_byp: A=%u.%u B=%u.%u C=%u.%u",
             src,
             (unsigned)(r->input.v_in_AB / 10u),     (unsigned)(r->input.v_in_AB % 10u),
             (unsigned)(r->input.v_in_BC / 10u),     (unsigned)(r->input.v_in_BC % 10u),
             (unsigned)(r->input.v_in_CA / 10u),     (unsigned)(r->input.v_in_CA % 10u),
             (unsigned)(r->input.v_bypass_A / 10u),  (unsigned)(r->input.v_bypass_A % 10u),
             (unsigned)(r->input.v_bypass_B / 10u),  (unsigned)(r->input.v_bypass_B % 10u),
             (unsigned)(r->input.v_bypass_C / 10u),  (unsigned)(r->input.v_bypass_C % 10u));

    ESP_LOGI(TAG_RMS, "[%s] [INPUT] I_in: A=%u.%u B=%u.%u C=%u.%u Freq: %u.%02u Hz",
             src,
             (unsigned)(r->input.i_in_A / 10u), (unsigned)(r->input.i_in_A % 10u),
             (unsigned)(r->input.i_in_B / 10u), (unsigned)(r->input.i_in_B % 10u),
             (unsigned)(r->input.i_in_C / 10u), (unsigned)(r->input.i_in_C % 10u),
             (unsigned)(r->input.freq_in / 100u), (unsigned)(r->input.freq_in % 100u));

    // OUTPUT: x0.1 для напряжений/токов/мощностей/процентов, x0.01 для частоты
    ESP_LOGI(TAG_RMS, "[%s] [OUTPUT] V_out: A=%u.%u B=%u.%u C=%u.%u Freq: %u.%02u Hz",
             src,
             (unsigned)(r->output.v_out_A / 10u), (unsigned)(r->output.v_out_A % 10u),
             (unsigned)(r->output.v_out_B / 10u), (unsigned)(r->output.v_out_B % 10u),
             (unsigned)(r->output.v_out_C / 10u), (unsigned)(r->output.v_out_C % 10u),
             (unsigned)(r->output.freq_out / 100u), (unsigned)(r->output.freq_out % 100u));

    ESP_LOGI(TAG_RMS, "[%s] [OUTPUT] I_out: A=%u.%u B=%u.%u C=%u.%u",
             src,
             (unsigned)(r->output.i_out_A / 10u), (unsigned)(r->output.i_out_A % 10u),
             (unsigned)(r->output.i_out_B / 10u), (unsigned)(r->output.i_out_B % 10u),
             (unsigned)(r->output.i_out_C / 10u), (unsigned)(r->output.i_out_C % 10u));

    ESP_LOGI(TAG_RMS, "[%s] [OUTPUT] P_Act: A=%u.%u B=%u.%u C=%u.%u kW",
             src,
             (unsigned)(r->output.p_active_A / 10u), (unsigned)(r->output.p_active_A % 10u),
             (unsigned)(r->output.p_active_B / 10u), (unsigned)(r->output.p_active_B % 10u),
             (unsigned)(r->output.p_active_C / 10u), (unsigned)(r->output.p_active_C % 10u));

    ESP_LOGI(TAG_RMS, "[%s] [OUTPUT] P_App: A=%u.%u B=%u.%u C=%u.%u kVA",
             src,
             (unsigned)(r->output.p_apparent_A / 10u), (unsigned)(r->output.p_apparent_A % 10u),
             (unsigned)(r->output.p_apparent_B / 10u), (unsigned)(r->output.p_apparent_B % 10u),
             (unsigned)(r->output.p_apparent_C / 10u), (unsigned)(r->output.p_apparent_C % 10u));

    ESP_LOGI(TAG_RMS, "[%s] [OUTPUT] Load: A=%u.%u%% B=%u.%u%% C=%u.%u%% Events: %u",
             src,
             (unsigned)(r->output.load_pct_A / 10u), (unsigned)(r->output.load_pct_A % 10u),
             (unsigned)(r->output.load_pct_B / 10u), (unsigned)(r->output.load_pct_B % 10u),
             (unsigned)(r->output.load_pct_C / 10u), (unsigned)(r->output.load_pct_C % 10u),
             (unsigned)r->output.event_count);

    // Battery: bat_voltage, dc_bus x0.1; bat_current x0.1 знаковый; capacity, backup_time — целые
    {
        uint16_t v  = r->battery.bat_voltage;
        uint16_t dc = r->battery.dc_bus_voltage;
        int16_t  cur = (int16_t)r->battery.bat_current;
        uint16_t cur_abs = (uint16_t)(cur < 0 ? -cur : cur);

        ESP_LOGI(TAG_RMS, "[%s] [BAT] Vol: %u.%u V Cap: %u Ah Grp: %u DC: %u.%u V",
                 src,
                 (unsigned)(v / 10u), (unsigned)(v % 10u),
                 (unsigned)r->battery.bat_capacity, (unsigned)r->battery.bat_groups_count,
                 (unsigned)(dc / 10u), (unsigned)(dc % 10u));

        ESP_LOGI(TAG_RMS, "[%s] [BAT] Cur: %s%u.%u A Backup: %u min",
                 src,
                 cur < 0 ? "-" : "",
                 (unsigned)(cur_abs / 10u), (unsigned)(cur_abs % 10u),
                 (unsigned)r->battery.backup_time);
    }

    ESP_LOGI(TAG_RMS, "[%s] =============================", src);
}

// Вывод флагов ошибок/статусов UPS из последнего принятого кадра и запись в UpsRegisterFlags_t
static void print_error_flag_frame(RingBuffModulData_t *rb, UpsRegisterFlags_t *out)
{
    static const char *TAG_ERR = "UPS_FLAG";

    if (rb->buffer == NULL || out == NULL) {
        return;
    }

    size_t count = get_elements_count(rb);
    if (count == 0) {
        return;
    }

    // Индекс последнего записанного кадра (head — следующая позиция записи)
    size_t last_idx = (rb->head + rb->cnt_cpyes - 1) % rb->cnt_cpyes;
    FpgaToEspPacket_t *pkt = &rb->buffer[last_idx].packet;

    if (pkt->start_marker != 0xAA55AA55u) {
        return;
    }

    uint16_t st = pkt->status.raw;
    uint16_t al = pkt->alarms.raw;

    memset(out, 0, sizeof(UpsRegisterFlags_t));

    /* Статусы (10001–10011), биты 0–10 */
    for (unsigned i = 0; i <= 10; i++) {
        if (!(st & (1u << i))) continue;
        switch (i) {
            case 0:  out->grid_status        = 1; ESP_LOGW(TAG_ERR, "[10001] Grid: авария на входе");       break;
            case 1:  out->bypass_grid_status = 1; ESP_LOGW(TAG_ERR, "[10002] Bypass grid: авария");         break;
            case 2:  out->rectifier_status   = 1; ESP_LOGI(TAG_ERR, "[10003] Выпрямитель: работает");      break;
            case 3:  out->inverter_status    = 1; ESP_LOGI(TAG_ERR, "[10004] Инвертор: работает");         break;
            case 4:  out->pwr_via_inverter   = 1; ESP_LOGI(TAG_ERR, "[10005] Питание через инвертор");      break;
            case 5:  out->pwr_via_bypass     = 1; ESP_LOGI(TAG_ERR, "[10006] Питание по байпасу");         break;
            case 6:  out->sync_status        = 1; ESP_LOGW(TAG_ERR, "[10007] Синхронизация: рассогласование"); break;
            case 7:  out->load_mode          = 1; ESP_LOGI(TAG_ERR, "[10008] Нагрузка от инвертора");      break;
            case 8:  out->sound_alarm        = 1; ESP_LOGW(TAG_ERR, "[10009] Звуковая сигнализация");      break;
            case 9:  out->battery_status     = 1; ESP_LOGI(TAG_ERR, "[10010] АКБ: разряд");                break;
            case 10: out->ups_mode           = 1; ESP_LOGI(TAG_ERR, "[10011] ИБП: работа от батареи");     break;
            default: break;
        }
    }

    /* Аварии (10012–10022), биты 0–10 */
    for (unsigned i = 0; i <= 10; i++) {
        if (!(al & (1u << i))) continue;
        switch (i) {
            case 0:  out->err_low_input_vol    = 1; ESP_LOGE(TAG_ERR, "[10012] Низкое напряжение на входе ИБП"); break;
            case 1:  out->err_high_dc_bus      = 1; ESP_LOGE(TAG_ERR, "[10013] Высокое напряжение DC шины");     break;
            case 2:  out->err_low_bat_charge   = 1; ESP_LOGE(TAG_ERR, "[10014] Низкий заряд АКБ");               break;
            case 3:  out->err_bat_not_conn     = 1; ESP_LOGE(TAG_ERR, "[10015] АКБ не подключены");              break;
            case 4:  out->err_inv_fault        = 1; ESP_LOGE(TAG_ERR, "[10016] Неисправность инвертора");        break;
            case 5:  out->err_inv_overcurrent  = 1; ESP_LOGE(TAG_ERR, "[10017] Перегрузка инвертора по току"); break;
            case 6:  out->err_high_out_vol     = 1; ESP_LOGE(TAG_ERR, "[10018] Высокое напряжение на выходе");  break;
            case 7:  out->err_fan_fault        = 1; ESP_LOGE(TAG_ERR, "[10019] Неисправность вентилятора");     break;
            case 8:  out->err_replace_bat      = 1; ESP_LOGE(TAG_ERR, "[10020] Необходимо заменить АКБ");       break;
            case 9:  out->err_rect_overheat    = 1; ESP_LOGE(TAG_ERR, "[10021] Перегрев выпрямителя");          break;
            case 10: out->err_inv_overheat     = 1; ESP_LOGE(TAG_ERR, "[10022] Перегрев инвертора");            break;
            default: break;
        }
    }

    if (st != 0 || al != 0) {
        ESP_LOGI(TAG_ERR, "--- флаги записаны в UpsRegisterFlags (status=0x%04X alarms=0x%04X) ---", (unsigned)st, (unsigned)al);
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
    
    ESP_LOGI(TAG_TIME, "Delta: %u ms, Elements: %d, Buf_status: %s", (unsigned)deltaTime_ms, get_elements_count(rb), rb->is_full? "FULL" : "NOT FULL");
}

static void logger_proc_task(void *pvParameters)
{
    uint64_t last_print_ms = 0;
     uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000); 

    for(;;)
    {
        if( ((uint32_t)(esp_timer_get_time() / 1000) - now_ms) > 500)
        {
            time_calculate_DEBUG(&RingBuffModulData);
            now_ms = (uint32_t)(esp_timer_get_time() / 1000); 
        }
        if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Пересчитываем RMS по текущему окну скользящего среднего
            calculate_moving_average_from_buffer();
            xSemaphoreGive(bufferMutex);
            // UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);                                    // <<-- !Проверка утечки стека 
            // ESP_LOGI(TAG, "logger stack free: %u bytes", (unsigned)(hwm * sizeof(StackType_t)));
        }
        //print_error_flag_frame(&RingBuffModulData, &UpsRegisterFlags);
        //ESP_LOGI(TAG, "Circular buf: tail: %u  head: %u flag: %u", RingBuffModulData.tail, RingBuffModulData.head, RingBuffModulData.is_full );
        // Раз в 1 секунду выводим текущие RMS-значения
        if (now_ms - last_print_ms >= 1000) {
            last_print_ms = now_ms;

            // Делаем локальную копию, чтобы вывод не зависел от мьютекса
            FpgaRmsData_t snapshot;
            memcpy(&snapshot, &gFpgaRmsData, sizeof(snapshot));
            //logger_print_avg_data(&snapshot);
            //ESP_LOGI(TAG, "Current time: %u", now_ms);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

