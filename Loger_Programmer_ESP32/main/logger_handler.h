#pragma once

#include "esp_err.h"
#include "frames_structure.h"
#include "wifi_control.h"

#define SIZE_OF_CIRCULAR_BUFFER  10  //Колличесвто структур-кадров которые будут храниться в кольцевом буфере
#define NUMBER_OF_REMAINING_EMPTY  5

#if (SIZE_OF_CIRCULAR_BUFFER - 1) <= NUMBER_OF_REMAINING_EMPTY
#error "The number of frames must be greater than the number of unused frames in the ring buffer."
#endif

typedef enum{
    RINGBUF_OK,
    RINGBUF_ERR,
    RINGBUF_PARAM_ERR,
    RINGBUF_OVERFLOW,
    RINGBUF_NULL_POINTER,
    RINGBUF_MUTEX_NOT_GIVE,
}RingBuffStatus_t;           // Должен отражать состояние кольцевого буфера 


typedef struct{
    uint8_t grid_status        ;  // 10001: Состояние электросети на входе (0: Норма, 1: Авария)
    uint8_t bypass_grid_status ;  // 10002: Состояние электросети на входе байпаса
    uint8_t rectifier_status   ;  // 10003: Состояние выпрямителя (1: Работает, 0: Выкл)
    uint8_t inverter_status    ;  // 10004: Состояние инвертора (1: Работает, 0: Выкл)
    uint8_t pwr_via_inverter   ;  // 10005: Питание через инвертор (1: Да)
    uint8_t pwr_via_bypass     ;  // 10006: Питание по байпас (1: Да)
    uint8_t sync_status        ;  // 10007: Синхронизация инвертора и байпаса (0: Синхронизированы)
    uint8_t load_mode          ;  // 10008: Режим питания нагрузки (1: Инвертор, 0: Байпас)
    uint8_t sound_alarm        ;  // 10009: Аварийный звуковой сигнал (1: Включен)
    uint8_t battery_status     ;  // 10010: Состояние АКБ (0: Заряд, 1: Разряд)
    uint8_t ups_mode           ;  // 10011: Режим работы ИБП (0: Сеть, 1: Батарея)


    uint8_t err_low_input_vol     ;   // 10012: Низкое напряжение на входе ИБП
    uint8_t err_high_dc_bus       ;   // 10013: Высокое напряжение на DC шине
    uint8_t err_low_bat_charge    ;   // 10014: Низкий заряд АКБ
    uint8_t err_bat_not_conn      ;   // 10015: АКБ не подключены
    uint8_t err_inv_fault         ;   // 10016: Неисправность инвертора
    uint8_t err_inv_overcurrent   ;   // 10017: Перегрузка инвертора по току
    uint8_t err_high_out_vol      ;   // 10018: Высокое напряжение на выходе ИБП
    uint8_t err_fan_fault         ;   // 10019: Неисправность вентилятора
    uint8_t err_replace_bat       ;   // 10020: Необходимо заменить АКБ
    uint8_t err_rect_overheat     ;   // 10021: Перегрев выпрямителя
    uint8_t err_inv_overheat      ;   // 10022: Перегрев инвертор

}UpsRegisterFlags_t;

// Для расчёта скользящего среднего 

// --- Группа 3: Входные параметры (30001-30010) ---
// Передаем как int, множители (x0.1 и т.д.) применяются при отображении
typedef struct {
	uint64_t v_in_AB;             // 30001: Входное напр. AB (x0.1 В)
	uint64_t v_in_BC;             // 30002: Входное напр. BC (x0.1 В)
	uint64_t v_in_CA;             // 30003: Входное напр. CA (x0.1 В)
	uint64_t v_bypass_A;          // 30004: Входное напр. байпаса фазы A (x0.1 В)
	uint64_t v_bypass_B;          // 30005: Входное напр. байпаса фазы B (x0.1 В)
	uint64_t v_bypass_C;          // 30006: Входное напр. байпаса фазы C (x0.1 В)
	uint64_t i_in_A;              // 30007: Входной ток фазы A (x0.1 А)
	uint64_t i_in_B;              // 30008: Входной ток фазы B (x0.1 А)
	uint64_t i_in_C;              // 30009: Входной ток фазы C (x0.1 А)
	uint64_t freq_in;             // 30010: Входная частота (x0.01 Гц) !!! Внимание: 0.01
} GroupInput_x64_t;

// --- Группа 4: Выходные параметры (30011-30027) ---
typedef struct {
	uint64_t v_out_A;             // 30011: Выходное напр. фазы A (x0.1 В)
	uint64_t v_out_B;             // 30012: Выходное напр. фазы B (x0.1 В)
	uint64_t v_out_C;             // 30013: Выходное напр. фазы C (x0.1 В)
	uint64_t freq_out;            // 30014: Выходная частота (x0.01 Гц) !!! Внимание: 0.01
	uint64_t i_out_A;             // 30015: Выходной ток фазы A (x0.1 А)
	uint64_t i_out_B;             // 30016: Выходной ток фазы B (x0.1 А)
	uint64_t i_out_C;             // 30017: Выходной ток фазы C (x0.1 А)
	uint64_t p_active_A;          // 30018: Вых. активная мощность фазы A (x0.1 кВт)
	uint64_t p_active_B;          // 30019: Вых. активная мощность фазы B (x0.1 кВт)
	uint64_t p_active_C;          // 30020: Вых. активная мощность фазы C (x0.1 кВт)
	uint64_t p_apparent_A;        // 30021: Вых. полная мощность фазы A (x0.1 кВА)
	uint64_t p_apparent_B;        // 30022: Вых. полная мощность фазы B (x0.1 кВА)
	uint64_t p_apparent_C;        // 30023: Вых. полная мощность фазы C (x0.1 кВА)
	uint64_t load_pct_A;          // 30024: Нагрузка фазы A (x0.1 %)
	uint64_t load_pct_B;          // 30025: Нагрузка фазы B (x0.1 %)
	uint64_t load_pct_C;          // 30026: Нагрузка фазы C (x0.1 %)
	uint64_t event_count;         // 30027: Кол-во зарегистрированных событий (Целое)
} GroupOutput_x64_t;

// --- Группа 5: Параметры АКБ (30028-30033) ---
typedef struct {
	uint64_t bat_voltage;         // 30028: Напряжение АКБ (x0.1 В)
	uint64_t bat_capacity;        // 30029: Емкость АКБ (x1 А*ч)
	uint64_t bat_groups_count;    // 30030: Число батарейных групп (Целое)
	uint64_t dc_bus_voltage;      // 30031: Напряжение DC шины (x0.1 В)
	uint64_t bat_current;         // 30032: Ток заряда/разряда (x0.1 А) !!! int16_t для знака
	uint64_t backup_time;         // 30033: Расчетное время автономии (x1 мин)
} GroupBattery_x64_t;

typedef struct {
    GroupInput_x64_t   input;         // 10 регистров = 20 байт
    GroupOutput_x64_t  output;        // 17 регистров = 34 байта
    GroupBattery_x64_t battery;       // 6 регистров = 12 байт
} FPGA_mov_acrage_x64_t;


typedef struct{
    ModulData_t *buffer;                // Буфер кадров
    volatile size_t tail;               // Точка чтения 
    volatile size_t head;               // Точка записи 
    volatile size_t size_byte;          // Размер буфера 
    volatile size_t cnt_cpyes;         // Количество копий
    volatile size_t count;              // ТЕКУЩЕЕ количество элементов в буфере
    volatile size_t cell_size;          // Размер одной ячейки (байт)
    volatile bool is_full;              // Флаг переполнения

    volatile FPGA_mov_acrage_x64_t FPGA_mov_averge;  // Накопительная сумма (размер типа должен исключать переполнение!)

}RingBuffModulData_t;

void logger_Inint(void);

RingBuffStatus_t RingBuffWrite(ModulData_t* ModulData);

// Глобальная структура с RMS-значениями по данным от FPGA
extern FpgaRmsData_t gFpgaRmsData;