/*
 * wi-fi_hande.h
 *
 *  Created on: 17 февр. 2026 г.
 *      Author: q
 */

#ifndef INC_MSP_H_
#define INC_MSP_H_

#include<stdio.h>
#include<stdint.h>
#include<stdlib.h>
#include<string.h>



typedef struct {
    // Вход (Левая колонка)
    float v_in_a; float v_in_b; float v_in_c; // Напряжение
    float c_in_a; float c_in_b; float c_in_c; // Ток
    float f_in_a; float f_in_b; float f_in_c; // Частота

    // Выход (Правая колонка)
    float p_act_a; float p_act_b; float p_act_c; // Активная мощность (kW)
    float p_rea_a; float p_rea_b; float p_rea_c; // Реактивная мощность (kVar)
    float load_a;  float load_b;  float load_c;  // Нагрузка (%)

    int fan_time;
    int module_id;
} UpsData_t;

#pragma pack(push, 1)

// Директива для плотной упаковки данных (важно для передачи структур байт-в-байт через SPI)

// --- Группа 1: Статусы (Регистры 10001-10011) ---
typedef struct {
    uint16_t grid_status;         // 10001: Состояние электросети на входе (0: Норма, 1: Авария)
    uint16_t bypass_grid_status;  // 10002: Состояние электросети на входе байпаса
    uint16_t rectifier_status;    // 10003: Состояние выпрямителя (1: Работает, 0: Выкл)
    uint16_t inverter_status;     // 10004: Состояние инвертора (1: Работает, 0: Выкл)
    uint16_t pwr_via_inverter;    // 10005: Питание через инвертор (1: Да)
    uint16_t pwr_via_bypass;      // 10006: Питание по байпас (1: Да)
    uint16_t sync_status;         // 10007: Синхронизация инвертора и байпаса (0: Синхронизированы)
    uint16_t load_mode;           // 10008: Режим питания нагрузки (1: Инвертор, 0: Байпас)
    uint16_t sound_alarm;         // 10009: Аварийный звуковой сигнал (1: Включен)
    uint16_t battery_status;      // 10010: Состояние АКБ (0: Заряд, 1: Разряд)
    uint16_t ups_mode;            // 10011: Режим работы ИБП (0: Сеть, 1: Батарея)
} GroupStatus_t;

// --- Группа 2: Аварии (Регистры 10012-10022) ---
// Значение 1 = Авария, 0 = Норма
typedef struct {
    uint16_t err_low_input_vol;   // 10012: Низкое напряжение на входе ИБП
    uint16_t err_high_dc_bus;     // 10013: Высокое напряжение на DC шине
    uint16_t err_low_bat_charge;  // 10014: Низкий заряд АКБ
    uint16_t err_bat_not_conn;    // 10015: АКБ не подключены
    uint16_t err_inv_fault;       // 10016: Неисправность инвертора
    uint16_t err_inv_overcurrent; // 10017: Перегрузка инвертора по току
    uint16_t err_high_out_vol;    // 10018: Высокое напряжение на выходе ИБП
    uint16_t err_fan_fault;       // 10019: Неисправность вентилятора
    uint16_t err_replace_bat;     // 10020: Необходимо заменить АКБ
    uint16_t err_rect_overheat;   // 10021: Перегрев выпрямителя
    uint16_t err_inv_overheat;    // 10022: Перегрев инвертора
} GroupAlarms_t;

// --- Группа 3: Входные параметры (30001-30010) ---
// Передаем как int, множители (x0.1 и т.д.) применяются при отображении
typedef struct {
	float v_in_AB;             // 30001: Входное напр. AB (x0.1 В)
	float v_in_BC;             // 30002: Входное напр. BC (x0.1 В)
	float v_in_CA;             // 30003: Входное напр. CA (x0.1 В)
	float v_bypass_A;          // 30004: Входное напр. байпаса фазы A (x0.1 В)
	float v_bypass_B;          // 30005: Входное напр. байпаса фазы B (x0.1 В)
	float v_bypass_C;          // 30006: Входное напр. байпаса фазы C (x0.1 В)
	float i_in_A;              // 30007: Входной ток фазы A (x0.1 А)
	float i_in_B;              // 30008: Входной ток фазы B (x0.1 А)
	float i_in_C;              // 30009: Входной ток фазы C (x0.1 А)
	float freq_in;             // 30010: Входная частота (x0.01 Гц) !!! Внимание: 0.01
} GroupInput_t;

// --- Группа 4: Выходные параметры (30011-30027) ---
typedef struct {
	float v_out_A;             // 30011: Выходное напр. фазы A (x0.1 В)
	float v_out_B;             // 30012: Выходное напр. фазы B (x0.1 В)
	float v_out_C;             // 30013: Выходное напр. фазы C (x0.1 В)
	float freq_out;            // 30014: Выходная частота (x0.01 Гц) !!! Внимание: 0.01
    float i_out_A;             // 30015: Выходной ток фазы A (x0.1 А)
    float i_out_B;             // 30016: Выходной ток фазы B (x0.1 А)
    float i_out_C;             // 30017: Выходной ток фазы C (x0.1 А)
    float p_active_A;          // 30018: Вых. активная мощность фазы A (x0.1 кВт)
    float p_active_B;          // 30019: Вых. активная мощность фазы B (x0.1 кВт)
    float p_active_C;          // 30020: Вых. активная мощность фазы C (x0.1 кВт)
    float p_apparent_A;        // 30021: Вых. полная мощность фазы A (x0.1 кВА)
    float p_apparent_B;        // 30022: Вых. полная мощность фазы B (x0.1 кВА)
    float p_apparent_C;        // 30023: Вых. полная мощность фазы C (x0.1 кВА)
    float load_pct_A;          // 30024: Нагрузка фазы A (x0.1 %)
    float load_pct_B;          // 30025: Нагрузка фазы B (x0.1 %)
    float load_pct_C;          // 30026: Нагрузка фазы C (x0.1 %)
    float event_count;         // 30027: Кол-во зарегистрированных событий (Целое)
} GroupOutput_t;

// --- Группа 5: Параметры АКБ (30028-30033) ---
typedef struct {
	float bat_voltage;         // 30028: Напряжение АКБ (x0.1 В)
	float bat_capacity;        // 30029: Емкость АКБ (x1 А*ч)
	float bat_groups_count;    // 30030: Число батарейных групп (Целое)
	float dc_bus_voltage;      // 30031: Напряжение DC шины (x0.1 В)
	float bat_current;         // 30032: Ток заряда/разряда (x0.1 А) !!! int16_t для знака
	float backup_time;         // 30033: Расчетное время автономии (x1 мин)
} GroupBattery_t;

// === ГЛАВНАЯ СТРУКТУРА ПАКЕТА SPI ===
// Эту структуру целиком отправляем/принимаем по SPI
typedef struct {
    uint32_t start_marker;        // 0xAA55AA55 или подобный Magic Number
    uint32_t packet_counter;      // Инкрементальный счетчик для отладки

    // Вложенные структуры данных (порядок важен!)
    GroupStatus_t  status;        // 11 регистров = 22 байта
    GroupAlarms_t  alarms;        // 11 регистров = 22 байта
    GroupInput_t   input;         // 10 регистров = 20 байт
    GroupOutput_t  output;        // 17 регистров = 34 байта
    GroupBattery_t battery;       // 6 регистров = 12 байт

    uint32_t crc32;               // Контрольная сумма пакета
} FpgaToEspPacket_t;


#pragma pack(pop)

typedef union {
    FpgaToEspPacket_t packet;
    uint8_t Tx_Buffer[sizeof(FpgaToEspPacket_t)];
}ModulData_t;



void SetVelueInStruckt(ModulData_t *ups_data);


#endif /* INC_MSP_H_ */
