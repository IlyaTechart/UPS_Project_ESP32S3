/*
 * wi-fi_hande.h
 *
 *  Created on: 17 февр. 2026 г.
 *      Author: q
 */

#ifndef INC_WI_FI_HANDE_H_
#define INC_WI_FI_HANDE_H_

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


void SetVelueInStruckt(UpsData_t *ups_data);


#endif /* INC_WI_FI_HANDE_H_ */
