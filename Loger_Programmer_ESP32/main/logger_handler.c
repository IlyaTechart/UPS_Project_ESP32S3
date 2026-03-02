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
#include "esp_timer.h"
#include "logger_handler.h"


char *TAG = "LOGER";
static const char *TAG_RMS = "LOG_RMS";

// Переменная кольцевого буфера 
RingBuffModulData_t RingBuffModulData;

static SemaphoreHandle_t bufferMutex = NULL; // Мутекс для защиты памяти

//Состояние кольцевого буфера
RingBuffStatus_t RingBuffStatus = RINGBUF_OK;

// Состояние UPS
UpsRegisterFlags_t UpsRegisterFlags;

// Глобальная структура для RMS-значений по данным из кольцевого буфера
FpgaRmsData_t gFpgaRmsData;

static size_t get_elements_count(RingBuffModulData_t *rb);
static void calculate_rms_from_buffer(void);
static void logger_print_rms_data(const FpgaRmsData_t *rms);
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

    RingBuffModulData.buffer = (ModulData_t *)heap_caps_calloc(SIZE_OF_CIRCULAR_BUFFER, sizeof(ModulData_t), MALLOC_CAP_DEFAULT); // выделяем память 
    if (RingBuffModulData.buffer == NULL) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return;
    } 
    RingBuffModulData.size_byte = sizeof(ModulData_t) * SIZE_OF_CIRCULAR_BUFFER; // Размер в байтах 
    RingBuffModulData.size_cpyes = SIZE_OF_CIRCULAR_BUFFER;
    RingBuffModulData.head = 0;
    RingBuffModulData.tail = 0;
    RingBuffModulData.is_full = false;

    if (xTaskCreate(logger_proc_task, "logger", 8192, NULL, 10, NULL) != pdPASS) {        // Создаём задачу (стек 8KB — расчёт RMS + много ESP_LOGI)
        ESP_LOGE(TAG, "Failed to create LOGGER task");
    }else{
        ESP_LOGI(TAG, "Logger Init Success");
    }

    size_t heap_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    
    ESP_LOGI(TAG, "Free heap size: %u", heap_free);

    
}

// Функция возвращает текущее количество данных в буфере
static size_t get_elements_count(RingBuffModulData_t *rb) {
    size_t capacity = rb->size_cpyes; // Общая вместимость буфера (максимальное кол-во элементов)
    
    if (rb->is_full) {
        return capacity;
    }
    
    if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    } else {
        return capacity - rb->tail + rb->head;
    }
}

RingBuffStatus_t RingBuffWrite(ModulData_t* ModulData)
{
    if(RingBuffModulData.buffer == NULL || bufferMutex == NULL) return RINGBUF_NULL_POINTER;

    if(xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(10)) == pdTRUE)                                     //<<--!Осторожно временная задержка котораяя может сильно ограничивать скорость 
    {
        uint16_t next_head = (RingBuffModulData.head + 1) % RingBuffModulData.size_cpyes;

        if(next_head == RingBuffModulData.tail)
        {
            //Можно здесь отбрасывать приянтые данные, но щас релизована перезапись старых
            RingBuffModulData.tail = (RingBuffModulData.tail + 1) % RingBuffModulData.size_cpyes;
            // Теперь место под head освободилось (за счет потери старого элемента)
            memcpy(&RingBuffModulData.buffer[RingBuffModulData.head], ModulData, sizeof(ModulData_t));
            RingBuffModulData.head = next_head;
            RingBuffModulData.is_full = true;
            xSemaphoreGive(bufferMutex);
            return RINGBUF_OVERFLOW;
        }else{
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

// Расчёт RMS по всем элементам групп status/alarms/input/output/battery
// на основе всех валидных записей в кольцевом буфере.
static void calculate_rms_from_buffer(void)
{
    if (RingBuffModulData.buffer == NULL){
        return;
    }

    if( get_elements_count(&RingBuffModulData) < (SIZE_OF_CIRCULAR_BUFFER - NUMBER_OF_REMAINING_EMPTY) )
    {
        return;
    }

    float status_raw_sq   = 0.0;
    float alarms_raw_sq   = 0.0;

    float v_in_AB_sq      = 0.0;
    float v_in_BC_sq      = 0.0;
    float v_in_CA_sq      = 0.0;
    float v_bypass_A_sq   = 0.0;
    float v_bypass_B_sq   = 0.0;
    float v_bypass_C_sq   = 0.0;
    float i_in_A_sq       = 0.0;
    float i_in_B_sq       = 0.0;
    float i_in_C_sq       = 0.0;
    float freq_in_sq      = 0.0;

    float v_out_A_sq      = 0.0;
    float v_out_B_sq      = 0.0;
    float v_out_C_sq      = 0.0;
    float freq_out_sq     = 0.0;
    float i_out_A_sq      = 0.0;
    float i_out_B_sq      = 0.0;
    float i_out_C_sq      = 0.0;
    float p_active_A_sq   = 0.0;
    float p_active_B_sq   = 0.0;
    float p_active_C_sq   = 0.0;
    float p_apparent_A_sq = 0.0;
    float p_apparent_B_sq = 0.0;
    float p_apparent_C_sq = 0.0;
    float load_pct_A_sq   = 0.0;
    float load_pct_B_sq   = 0.0;
    float load_pct_C_sq   = 0.0;
    float event_count_sq  = 0.0;

    float bat_voltage_sq      = 0.0;
    float bat_capacity_sq     = 0.0;
    float bat_groups_count_sq = 0.0;
    float dc_bus_voltage_sq   = 0.0;
    float bat_current_sq      = 0.0;
    float backup_time_sq      = 0.0;

    size_t samples = 0;

    // Проходим по всей реальной длине буфера (кол-во элементов),
    // не используя head/tail, и берём только валидные пакеты.
    for (size_t i = 0; i < SIZE_OF_CIRCULAR_BUFFER; ++i) {
        ModulData_t *entry = &RingBuffModulData.buffer[i];

        // Валидный кадр: есть маркер начала
        if (entry->packet.start_marker == 0) {
            continue;
        }

        FpgaToEspPacket_t *p = &entry->packet;
        samples++;

        // Статусы и аварии по полю raw
        status_raw_sq += (float)p->status.raw * (float)p->status.raw;
        alarms_raw_sq += (float)p->alarms.raw * (float)p->alarms.raw;

        v_in_AB_sq      += (float)p->input.v_in_AB      * (float)p->input.v_in_AB;
        v_in_BC_sq      += (float)p->input.v_in_BC      * (float)p->input.v_in_BC;
        v_in_CA_sq      += (float)p->input.v_in_CA      * (float)p->input.v_in_CA;
        v_bypass_A_sq   += (float)p->input.v_bypass_A   * (float)p->input.v_bypass_A;
        v_bypass_B_sq   += (float)p->input.v_bypass_B   * (float)p->input.v_bypass_B;
        v_bypass_C_sq   += (float)p->input.v_bypass_C   * (float)p->input.v_bypass_C;
        i_in_A_sq       += (float)p->input.i_in_A       * (float)p->input.i_in_A;
        i_in_B_sq       += (float)p->input.i_in_B       * (float)p->input.i_in_B;
        i_in_C_sq       += (float)p->input.i_in_C       * (float)p->input.i_in_C;
        freq_in_sq      += (float)p->input.freq_in      * (float)p->input.freq_in;

        v_out_A_sq      += (float)p->output.v_out_A      * (float)p->output.v_out_A;
        v_out_B_sq      += (float)p->output.v_out_B      * (float)p->output.v_out_B;
        v_out_C_sq      += (float)p->output.v_out_C      * (float)p->output.v_out_C;
        freq_out_sq     += (float)p->output.freq_out     * (float)p->output.freq_out;
        i_out_A_sq      += (float)p->output.i_out_A      * (float)p->output.i_out_A;
        i_out_B_sq      += (float)p->output.i_out_B      * (float)p->output.i_out_B;
        i_out_C_sq      += (float)p->output.i_out_C      * (float)p->output.i_out_C;
        p_active_A_sq   += (float)p->output.p_active_A   * (float)p->output.p_active_A;
        p_active_B_sq   += (float)p->output.p_active_B   * (float)p->output.p_active_B;
        p_active_C_sq   += (float)p->output.p_active_C   * (float)p->output.p_active_C;
        p_apparent_A_sq += (float)p->output.p_apparent_A * (float)p->output.p_apparent_A;
        p_apparent_B_sq += (float)p->output.p_apparent_B * (float)p->output.p_apparent_B;
        p_apparent_C_sq += (float)p->output.p_apparent_C * (float)p->output.p_apparent_C;
        load_pct_A_sq   += (float)p->output.load_pct_A   * (float)p->output.load_pct_A;
        load_pct_B_sq   += (float)p->output.load_pct_B   * (float)p->output.load_pct_B;
        load_pct_C_sq   += (float)p->output.load_pct_C   * (float)p->output.load_pct_C;
        event_count_sq  += (float)p->output.event_count  * (float)p->output.event_count;

        bat_voltage_sq      += (float)p->battery.bat_voltage      * (float)p->battery.bat_voltage;
        bat_capacity_sq     += (float)p->battery.bat_capacity     * (float)p->battery.bat_capacity;
        bat_groups_count_sq += (float)p->battery.bat_groups_count * (float)p->battery.bat_groups_count;
        dc_bus_voltage_sq   += (float)p->battery.dc_bus_voltage   * (float)p->battery.dc_bus_voltage;
        bat_current_sq      += (float)p->battery.bat_current      * (float)p->battery.bat_current;
        backup_time_sq      += (float)p->battery.backup_time      * (float)p->battery.backup_time;
    }

    if (samples == 0) {
        return;
    }

    float inv_n = 1.0 / (float)samples;

    gFpgaRmsData.status.raw  = (uint16_t)sqrt(status_raw_sq * inv_n);
    gFpgaRmsData.alarms.raw  = (uint16_t)sqrt(alarms_raw_sq * inv_n);

    gFpgaRmsData.input.v_in_AB      = (uint16_t)sqrt(v_in_AB_sq * inv_n);
    gFpgaRmsData.input.v_in_BC      = (uint16_t)sqrt(v_in_BC_sq * inv_n);
    gFpgaRmsData.input.v_in_CA      = (uint16_t)sqrt(v_in_CA_sq * inv_n);
    gFpgaRmsData.input.v_bypass_A   = (uint16_t)sqrt(v_bypass_A_sq * inv_n);
    gFpgaRmsData.input.v_bypass_B   = (uint16_t)sqrt(v_bypass_B_sq * inv_n);
    gFpgaRmsData.input.v_bypass_C   = (uint16_t)sqrt(v_bypass_C_sq * inv_n);
    gFpgaRmsData.input.i_in_A       = (uint16_t)sqrt(i_in_A_sq * inv_n);
    gFpgaRmsData.input.i_in_B       = (uint16_t)sqrt(i_in_B_sq * inv_n);
    gFpgaRmsData.input.i_in_C       = (uint16_t)sqrt(i_in_C_sq * inv_n);
    gFpgaRmsData.input.freq_in      = (uint16_t)sqrt(freq_in_sq * inv_n);

    gFpgaRmsData.output.v_out_A      = (uint16_t)sqrt(v_out_A_sq * inv_n);
    gFpgaRmsData.output.v_out_B      = (uint16_t)sqrt(v_out_B_sq * inv_n);
    gFpgaRmsData.output.v_out_C      = (uint16_t)sqrt(v_out_C_sq * inv_n);
    gFpgaRmsData.output.freq_out     = (uint16_t)sqrt(freq_out_sq * inv_n);
    gFpgaRmsData.output.i_out_A      = (uint16_t)sqrt(i_out_A_sq * inv_n);
    gFpgaRmsData.output.i_out_B      = (uint16_t)sqrt(i_out_B_sq * inv_n);
    gFpgaRmsData.output.i_out_C      = (uint16_t)sqrt(i_out_C_sq * inv_n);
    gFpgaRmsData.output.p_active_A   = (uint16_t)sqrt(p_active_A_sq * inv_n);
    gFpgaRmsData.output.p_active_B   = (uint16_t)sqrt(p_active_B_sq * inv_n);
    gFpgaRmsData.output.p_active_C   = (uint16_t)sqrt(p_active_C_sq * inv_n);
    gFpgaRmsData.output.p_apparent_A = (uint16_t)sqrt(p_apparent_A_sq * inv_n);
    gFpgaRmsData.output.p_apparent_B = (uint16_t)sqrt(p_apparent_B_sq * inv_n);
    gFpgaRmsData.output.p_apparent_C = (uint16_t)sqrt(p_apparent_C_sq * inv_n);
    gFpgaRmsData.output.load_pct_A   = (uint16_t)sqrt(load_pct_A_sq * inv_n);
    gFpgaRmsData.output.load_pct_B   = (uint16_t)sqrt(load_pct_B_sq * inv_n);
    gFpgaRmsData.output.load_pct_C   = (uint16_t)sqrt(load_pct_C_sq * inv_n);
    gFpgaRmsData.output.event_count  = (uint16_t)sqrt(event_count_sq * inv_n);

    gFpgaRmsData.battery.bat_voltage      = (uint16_t)sqrt(bat_voltage_sq * inv_n);
    gFpgaRmsData.battery.bat_capacity     = (uint16_t)sqrt(bat_capacity_sq * inv_n);
    gFpgaRmsData.battery.bat_groups_count = (uint16_t)sqrt(bat_groups_count_sq * inv_n);
    gFpgaRmsData.battery.dc_bus_voltage   = (uint16_t)sqrt(dc_bus_voltage_sq * inv_n);
    gFpgaRmsData.battery.bat_current      = (uint16_t)sqrt(bat_current_sq * inv_n);
    gFpgaRmsData.battery.backup_time      = (uint16_t)sqrt(backup_time_sq * inv_n);

    RingBuffModulData.tail = (RingBuffModulData.tail + 1) % RingBuffModulData.size_cpyes;
    RingBuffModulData.is_full = false;
}

// Вывод RMS-значений по образу spi_slave_print_ups_packet,
// но без полей заголовка/CRC, только группы status/alarms/input/output/battery.
static void logger_print_rms_data(const FpgaRmsData_t *r)
{
    const char *src = "RMS";

    ESP_LOGI(TAG_RMS, "[%s] === UPS RMS DATA ===", src);
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
    size_t last_idx = (rb->head + rb->size_cpyes - 1) % rb->size_cpyes;
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

    size_t last_written_idx = (rb->head + rb->size_cpyes - 1) % rb->size_cpyes;
    FpgaToEspPacket_t* pkHead = &rb->buffer[last_written_idx].packet;

    // Вычитаем из НОВОГО времени СТАРОЕ (а не наоборот, чтобы не было переполнения uint32_t)
    uint32_t deltaTime_ms = pkHead->system_time_ms - pkTail->system_time_ms; 
    
    ESP_LOGI(TAG_TIME, "Delta: %u ms, Elements: %d, Buf_statusЖ %s", (unsigned)deltaTime_ms, get_elements_count(rb), rb->is_full? "FULL" : "NOT FULL");
}

static void logger_proc_task(void *pvParameters)
{
    uint64_t last_print_ms = 0;

    for(;;)
    {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000); 
        time_calculate_DEBUG(&RingBuffModulData);
        if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Пересчитываем RMS по всем валидным кадрам в буфере
            calculate_rms_from_buffer();
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
            //logger_print_rms_data(&snapshot);
            //ESP_LOGI(TAG, "Current time: %u", now_ms);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

