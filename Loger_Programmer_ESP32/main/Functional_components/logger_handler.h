#pragma once

#include "esp_err.h"
#include "frames_structure.h"
#include "wifi_control.h"

#define SIZE_OF_CIRCULAR_BUFFER  1000  //Колличесвто структур-кадров которые будут храниться в кольцевом буфере
#define NUMBER_OF_REMAINING_EMPTY  5
#define START_MARKER_ID          0xFFAA2211

#if (SIZE_OF_CIRCULAR_BUFFER - 1) <= NUMBER_OF_REMAINING_EMPTY
#error "The number of frames must be greater than the number of unused frames in the ring buffer."
#endif

#define SEND_AVE_COMAND    (uint32_t)0X01
#define SEND_DUMP_COMAND   (uint32_t)0X02

#define PERIOD_MS_MOV_AVRAGE   1000
#define PERIOD_MS_SAND_FRAMES  1000
#define PERIOD_MS_DEBUG_PRINT  1000


typedef enum{
    RINGBUF_OK,
    RINGBUF_ERR,
    RINGBUF_PARAM_ERR,
    RINGBUF_OVERFLOW,
    RINGBUF_NULL_POINTER,
    RINGBUF_MUTEX_NOT_GIVE,
}RingBuffStatus_t;           // Должен отражать состояние кольцевого буфера 


/// @brief Структура кольцевого буфера 
typedef struct{
    ModulData_t *buffer;                // Буфер кадров
    volatile size_t tail;               // Точка чтения 
    volatile size_t head;               // Точка записи 
    volatile size_t size_byte;          // Размер буфера 
    volatile size_t cnt_cpyes;          // Количество копий
    volatile size_t count;              // ТЕКУЩЕЕ количество элементов в буфере
    volatile size_t cell_size;          // Размер одной ячейки (байт)
    volatile bool is_full;              // Флаг переполнения

}RingBuffModulData_t;



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

/// @brief Структура сумматора (uint64 для накопления скользящего среднего)
typedef struct {
    GroupInput_x64_t   input;
    GroupOutput_x64_t  output;
    GroupBattery_x64_t battery;
} AveSummator_t;


extern RingBuffModulData_t RingBuffModulData;
extern ModulData_t ModulDataFromExtend;



void logger_Inint(void);
size_t get_elements_count(RingBuffModulData_t *rb);
RingBuffStatus_t RingBuffWrite(ModulData_t* ModulData);