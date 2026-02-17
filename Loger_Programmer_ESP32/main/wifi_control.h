#pragma once


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


// Инициализация Wi-Fi и запуск веб-сервера
void wifi_web_init(void);

// Функция для обновления данных, которые мы хотим видеть в браузере (пример)
void set_web_status_message(const char* msg);