#pragma once




// Инициализация Wi-Fi и запуск веб-сервера (полная, с выделением памяти)
void wifi_web_init(void);

// Приостановить WiFi и HTTP-сервер (без освобождения памяти драйвера)
void wifi_web_suspend(void);

// Возобновить WiFi и HTTP-сервер после wifi_web_suspend()
void wifi_web_resume(void);

// Полная деинициализация с освобождением всех ресурсов
void wifi_web_Deinit(void);

// Функция для обновления данных, которые мы хотим видеть в браузере (пример)
void set_web_status_message(const char* msg);