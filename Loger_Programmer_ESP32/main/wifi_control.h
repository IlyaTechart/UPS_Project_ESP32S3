#pragma once




// Инициализация Wi-Fi и запуск веб-сервера
void wifi_web_init(void);

// Функция для обновления данных, которые мы хотим видеть в браузере (пример)
void set_web_status_message(const char* msg);