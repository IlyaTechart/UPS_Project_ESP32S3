#pragma once

#include "esp_err.h"
#include "frames_structure.h"
#include "wifi_control.h"

#define SIZE_OF_CIRCULAR_BUFFER  1000  //Колличесвто структур-кадров которые будут храниться в кольцевом буфере
#define NUMBER_OF_REMAINING_EMPTY  50

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




typedef struct{
    ModulData_t *buffer;                // Буфер кадров
    volatile size_t tail;               // Точка чтения 
    volatile size_t head;               // Точка записи 
    volatile size_t size_byte;          // Размер буфера 
    volatile size_t size_cpyes;         // Количество копий
    volatile size_t cell_size;          // Размер одной ячейки (байт)
    volatile bool is_full;              // Флаг переполнения
}RingBuffModulData_t;

void logger_Inint(void);

RingBuffStatus_t RingBuffWrite(ModulData_t* ModulData);

// Глобальная структура с RMS-значениями по данным от FPGA
extern FpgaRmsData_t gFpgaRmsData;