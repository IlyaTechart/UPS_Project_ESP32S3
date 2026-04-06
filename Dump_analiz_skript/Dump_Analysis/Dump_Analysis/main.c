/*
 * Dump_Analysis — приём дампа от ESP32S3 по USB CDC и сохранение в CSV
 *
 * Протокол: бинарные кадры FpgaToEspPacket_t (86 байт, pragma pack(1))
 * Маркер начала кадра: 0xAA55AA55
 * Таймаут тишины (DUMP_IDLE_TIMEOUT_MS): если данных нет дольше этого
 * времени — дамп считается завершённым, файл сохраняется.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <locale.h>
#include <Windows.h>
#include "main.h"

/* ─── Размер кадра FpgaToEspPacket_t (pragma pack(1)) ─────────────────────
 * uint32 start_marker       = 4
 * uint32 packet_counter     = 4
 * uint16 status_raw         = 2
 * uint16 alarms_raw         = 2
 * GroupInput_t:   10 x uint16 = 20
 * GroupOutput_t:  17 x uint16 = 34
 * GroupBattery_t:  6 x uint16 = 12
 * uint32 crc32              = 4
 * uint32 system_time_ms     = 4
 * ИТОГО:                      86 байт
 * ─────────────────────────────────────────────────────────────────────── */
#define FRAME_SIZE          86
#define START_MARKER        0xAA55AA55UL
#define DUMP_IDLE_TIMEOUT_MS 3000
#define MAX_FRAMES_PER_DUMP 100000
#define RX_BUF_SIZE         (FRAME_SIZE * 8)


/* Проверка размера на этапе компиляции */
typedef char _static_assert_frame_size[(sizeof(FpgaToEspPacket_t) == FRAME_SIZE) ? 1 : -1];

/* ─── Буфер накопленных кадров ─────────────────────────────────────────── */
typedef struct {
    FpgaToEspPacket_t *frames;
    size_t             count;
    size_t             capacity;
} DumpBuffer_t;

static bool dump_buffer_init(DumpBuffer_t *db)
{
    db->frames   = (FpgaToEspPacket_t *)malloc(sizeof(FpgaToEspPacket_t) * 1024);
    db->count    = 0;
    db->capacity = 1024;
    return db->frames != NULL;
}

static bool dump_buffer_push(DumpBuffer_t *db, const FpgaToEspPacket_t *frame)
{
    if (db->count >= MAX_FRAMES_PER_DUMP) return false;
    if (db->count >= db->capacity) {
        size_t new_cap = db->capacity * 2;
        FpgaToEspPacket_t *tmp = (FpgaToEspPacket_t *)realloc(
            db->frames, sizeof(FpgaToEspPacket_t) * new_cap);
        if (!tmp) return false;
        db->frames   = tmp;
        db->capacity = new_cap;
    }
    db->frames[db->count++] = *frame;
    return true;
}

static void dump_buffer_clear(DumpBuffer_t *db)
{
    db->count = 0;
}

static void dump_buffer_free(DumpBuffer_t *db)
{
    free(db->frames);
    db->frames   = NULL;
    db->count    = 0;
    db->capacity = 0;
}

/* ─── Вспомогательные функции ──────────────────────────────────────────── */

/* bat_current — знаковое int16 в uint16 контейнере */
static int16_t bat_current_signed(uint16_t raw)
{
    return (int16_t)raw;
}

/* Строка флагов статуса и аварий через '|' */
static void build_flags_string(const FpgaToEspPacket_t *p, char *buf, size_t buf_size)
{
    buf[0] = '\0';
    size_t pos = 0;

#define APPEND(s) do { \
    size_t _l = strlen(s); \
    if (pos + _l + 2 < buf_size) { \
        if (pos > 0) { buf[pos++] = '|'; } \
        memcpy(buf + pos, s, _l); pos += _l; buf[pos] = '\0'; \
    } \
} while(0)

    uint16_t st = p->status.raw;
    uint16_t al = p->alarms.raw;

    if (st & (1 << 0))  APPEND("WARN:grid_fault");
    if (st & (1 << 1))  APPEND("WARN:bypass_grid_fault");
    if (st & (1 << 2))  APPEND("INFO:rectifier_on");
    if (st & (1 << 3))  APPEND("INFO:inverter_on");
    if (st & (1 << 4))  APPEND("INFO:pwr_via_inverter");
    if (st & (1 << 5))  APPEND("INFO:pwr_via_bypass");
    if (st & (1 << 6))  APPEND("WARN:sync_mismatch");
    if (st & (1 << 7))  APPEND("INFO:load_from_inverter");
    if (st & (1 << 8))  APPEND("WARN:sound_alarm");
    if (st & (1 << 9))  APPEND("INFO:bat_discharge");
    if (st & (1 << 10)) APPEND("INFO:ups_on_battery");

    if (al & (1 << 0))  APPEND("ERR:low_input_vol");
    if (al & (1 << 1))  APPEND("ERR:high_dc_bus");
    if (al & (1 << 2))  APPEND("ERR:low_bat_charge");
    if (al & (1 << 3))  APPEND("ERR:bat_not_conn");
    if (al & (1 << 4))  APPEND("ERR:inv_fault");
    if (al & (1 << 5))  APPEND("ERR:inv_overcurrent");
    if (al & (1 << 6))  APPEND("ERR:high_out_vol");
    if (al & (1 << 7))  APPEND("ERR:fan_fault");
    if (al & (1 << 8))  APPEND("WARN:replace_bat");
    if (al & (1 << 9))  APPEND("ERR:rect_overheat");
    if (al & (1 << 10)) APPEND("ERR:inv_overheat");

#undef APPEND

    if (buf[0] == '\0') strcpy_s(buf, buf_size, "OK");
}

/* ─── Сохранение дампа в CSV ───────────────────────────────────────────── */
static void save_dump_csv(const DumpBuffer_t *db, int dump_number)
{
    char filename[256];
    time_t now = time(NULL);
    struct tm tm_info;
    localtime_s(&tm_info, &now);

    snprintf(filename, sizeof(filename),
             "dump_%04d%02d%02d_%02d%02d%02d_#%d.csv",
             tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
             tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
             dump_number);

    FILE *fp = NULL;
    if (fopen_s(&fp, filename, "w") != 0 || fp == NULL) {
        printf("  [ERROR] Не удалось создать файл: %s\n", filename);
        return;
    }

    /* Заголовок CSV */
    fprintf(fp,
        "frame_idx,"
        "pkt_counter,"
        "system_time_ms,"
        "system_time_s,"
        "status_raw,"
        "alarms_raw,"
        "flags,"
        /* Вход */
        "v_in_AB_V,"
        "v_in_BC_V,"
        "v_in_CA_V,"
        "v_bypass_A_V,"
        "v_bypass_B_V,"
        "v_bypass_C_V,"
        "i_in_A_A,"
        "i_in_B_A,"
        "i_in_C_A,"
        "freq_in_Hz,"
        /* Выход */
        "v_out_A_V,"
        "v_out_B_V,"
        "v_out_C_V,"
        "freq_out_Hz,"
        "i_out_A_A,"
        "i_out_B_A,"
        "i_out_C_A,"
        "p_active_A_kW,"
        "p_active_B_kW,"
        "p_active_C_kW,"
        "p_apparent_A_kVA,"
        "p_apparent_B_kVA,"
        "p_apparent_C_kVA,"
        "load_pct_A,"
        "load_pct_B,"
        "load_pct_C,"
        "event_count,"
        /* АКБ */
        "bat_voltage_V,"
        "bat_capacity_Ah,"
        "bat_groups_count,"
        "dc_bus_voltage_V,"
        "bat_current_A,"
        "backup_time_min,"
        /* Служебное */
        "crc32\n");

    char flags_buf[512];
    size_t error_frames = 0;

    for (size_t i = 0; i < db->count; i++) {
        const FpgaToEspPacket_t *p = &db->frames[i];
        build_flags_string(p, flags_buf, sizeof(flags_buf));

        bool has_alarm = (p->alarms.raw != 0);
        if (has_alarm) error_frames++;

        int16_t bat_cur = bat_current_signed(p->battery.bat_current);

        fprintf(fp,
            "%zu,"           /* frame_idx */
            "%u,"            /* pkt_counter */
            "%u,"            /* system_time_ms */
            "%.3f,"          /* system_time_s */
            "%u,"            /* status_raw */
            "%u,"            /* alarms_raw */
            "%s,"            /* flags */
            /* Вход */
            "%.1f,%.1f,%.1f,"
            "%.1f,%.1f,%.1f,"
            "%.1f,%.1f,%.1f,"
            "%.2f,"
            /* Выход */
            "%.1f,%.1f,%.1f,"
            "%.2f,"
            "%.1f,%.1f,%.1f,"
            "%.1f,%.1f,%.1f,"
            "%.1f,%.1f,%.1f,"
            "%.1f,%.1f,%.1f,"
            "%u,"
            /* АКБ */
            "%.1f,"
            "%u,"
            "%u,"
            "%.1f,"
            "%.1f,"
            "%u,"
            /* CRC */
            "0x%08X\n",
            i + 1,
            p->packet_counter,
            p->system_time_ms,
            p->system_time_ms / 1000.0,
            (unsigned)p->status.raw,
            (unsigned)p->alarms.raw,
            flags_buf,
            /* Вход */
            p->input.v_in_AB   / 10.0,
            p->input.v_in_BC   / 10.0,
            p->input.v_in_CA   / 10.0,
            p->input.v_bypass_A / 10.0,
            p->input.v_bypass_B / 10.0,
            p->input.v_bypass_C / 10.0,
            p->input.i_in_A    / 10.0,
            p->input.i_in_B    / 10.0,
            p->input.i_in_C    / 10.0,
            p->input.freq_in   / 100.0,
            /* Выход */
            p->output.v_out_A      / 10.0,
            p->output.v_out_B      / 10.0,
            p->output.v_out_C      / 10.0,
            p->output.freq_out     / 100.0,
            p->output.i_out_A      / 10.0,
            p->output.i_out_B      / 10.0,
            p->output.i_out_C      / 10.0,
            p->output.p_active_A   / 10.0,
            p->output.p_active_B   / 10.0,
            p->output.p_active_C   / 10.0,
            p->output.p_apparent_A / 10.0,
            p->output.p_apparent_B / 10.0,
            p->output.p_apparent_C / 10.0,
            p->output.load_pct_A   / 10.0,
            p->output.load_pct_B   / 10.0,
            p->output.load_pct_C   / 10.0,
            (unsigned)p->output.event_count,
            /* АКБ */
            p->battery.bat_voltage      / 10.0,
            (unsigned)p->battery.bat_capacity,
            (unsigned)p->battery.bat_groups_count,
            p->battery.dc_bus_voltage   / 10.0,
            bat_cur                     / 10.0,
            (unsigned)p->battery.backup_time,
            /* CRC */
            p->crc32);
    }

    fclose(fp);

    printf("\n  [OK] Дамп #%d сохранён -> %s\n", dump_number, filename);
    printf("       Кадров всего: %zu  |  Кадров с авариями: %zu\n",
           db->count, error_frames);
}

/* ─── Сканирование COM-портов ──────────────────────────────────────────── */
#define MAX_PORTS 20

static uint8_t  available_ports[MAX_PORTS];
static uint16_t available_ports_count = 0;

static void scan_com_ports(void)
{
    char port_name[20];
    available_ports_count = 0;

    printf("--- Сканирование COM-портов ---\n");

    for (int i = 1; i < 256 && available_ports_count < MAX_PORTS; i++) {
        sprintf_s(port_name, sizeof(port_name), "\\\\.\\COM%d", i);

        HANDLE h = CreateFileA(port_name, GENERIC_READ | GENERIC_WRITE,
                               0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (h != INVALID_HANDLE_VALUE) {
            printf("  [%u] COM%d — свободен\n", available_ports_count, i);
            available_ports[available_ports_count++] = (uint8_t)i;
            CloseHandle(h);
        } else if (GetLastError() == ERROR_ACCESS_DENIED) {
            printf("  [%u] COM%d — занят\n", available_ports_count, i);
            available_ports[available_ports_count++] = (uint8_t)i;
        }
    }

    if (available_ports_count == 0) {
        printf("  COM-портов не найдено. Проверь USB-кабель.\n");
    } else {
        printf("  Найдено: %u\n", available_ports_count);
    }
}

/* ─── Открытие и настройка COM-порта ──────────────────────────────────── */
static HANDLE open_com_port(int com_number, uint32_t baudrate)
{
    char port_name[20];
    sprintf_s(port_name, sizeof(port_name), "\\\\.\\COM%d", com_number);

    HANDLE h = CreateFileA(port_name,
                           GENERIC_READ | GENERIC_WRITE,
                           0, NULL, OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL, NULL);

    if (h == INVALID_HANDLE_VALUE) {
        printf("  [ERROR] Не удалось открыть COM%d (код: %lu)\n",
               com_number, GetLastError());
        return INVALID_HANDLE_VALUE;
    }

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) {
        printf("  [ERROR] GetCommState failed\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }
    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    if (!SetCommState(h, &dcb)) {
        printf("  [ERROR] SetCommState failed\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }

    /* Таймауты: ReadIntervalTimeout = MAXDWORD + обнуление остальных
     * означает «вернуть сразу то, что уже есть в буфере (или 0 байт)».
     * Это позволяет нам самим управлять таймаутом тишины через GetTickCount. */
    COMMTIMEOUTS to = { 0 };
    to.ReadIntervalTimeout         = MAXDWORD;
    to.ReadTotalTimeoutConstant    = 0;
    to.ReadTotalTimeoutMultiplier  = 0;
    SetCommTimeouts(h, &to);

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    /* DTR=1 — TinyUSB считает хост подключённым только когда DTR=1 (бит 0 line_state).
     * RTS=0 — не трогаем сброс/загрузку ESP32. */
    EscapeCommFunction(h, SETDTR);
    EscapeCommFunction(h, CLRRTS);

    printf("  COM%d открыт @ %lu бод  [DTR=1, RTS=0]\n", com_number, baudrate);
    return h;
}

/* ─── Обработчик Ctrl+C ────────────────────────────────────────────────── */
static volatile bool g_stop = false;

static BOOL WINAPI console_ctrl_handler(DWORD ctrl_type)
{
    if (ctrl_type == CTRL_C_EVENT || ctrl_type == CTRL_BREAK_EVENT) {
        printf("\n\n  Остановлено пользователем.\n");
        g_stop = true;
        return TRUE;
    }
    return FALSE;
}

/* ─── Основной цикл приёма дампа ───────────────────────────────────────── */
static void receive_dump(HANDLE hPort)
{
    uint8_t    rx_buf[RX_BUF_SIZE];
    uint8_t    frame_buf[RX_BUF_SIZE * 2];
    size_t     frame_buf_len = 0;

    DumpBuffer_t db;
    if (!dump_buffer_init(&db)) {
        printf("  [ERROR] Нет памяти для буфера кадров\n");
        return;
    }

    int       dump_number    = 0;
    size_t    total_frames   = 0;
    ULONGLONG last_data_tick = 0;
    bool      got_data_ever  = false;

    printf("\nОжидание данных от ESP32... (Ctrl+C для остановки)\n");
    printf("Таймаут тишины: %d мс -> сохранение дампа\n\n", DUMP_IDLE_TIMEOUT_MS);

    while (!g_stop) {
        DWORD bytes_read = 0;
        (void)ReadFile(hPort, rx_buf, sizeof(rx_buf), &bytes_read, NULL);

        if (bytes_read > 0) {
            /* Защита от переполнения скользящего буфера */
            if (frame_buf_len + bytes_read > sizeof(frame_buf)) {
                /* Сдвигаем: оставляем последние (sizeof(frame_buf)/2) байт */
                size_t keep = sizeof(frame_buf) / 2;
                memmove(frame_buf, frame_buf + frame_buf_len - keep, keep);
                frame_buf_len = keep;
            }
            memcpy(frame_buf + frame_buf_len, rx_buf, bytes_read);
            frame_buf_len += bytes_read;

            last_data_tick = GetTickCount64();
            got_data_ever  = true;

            /* Разбираем все полные кадры из буфера */
            while (frame_buf_len >= FRAME_SIZE) {
                /* Ищем маркер */
                size_t marker_pos = (size_t)-1;
                for (size_t i = 0; i + 4 <= frame_buf_len; i++) {
                    uint32_t candidate;
                    memcpy(&candidate, frame_buf + i, 4);
                    if (candidate == START_MARKER) {
                        marker_pos = i;
                        break;
                    }
                }

                if (marker_pos == (size_t)-1) {
                    /* Маркер не найден — выбрасываем всё кроме последних 3 байт */
                    if (frame_buf_len > 3) {
                        size_t keep = 3;
                        memmove(frame_buf, frame_buf + frame_buf_len - keep, keep);
                        frame_buf_len = keep;
                    }
                    break;
                }

                /* Выбрасываем мусор перед маркером */
                if (marker_pos > 0) {
                    memmove(frame_buf, frame_buf + marker_pos,
                            frame_buf_len - marker_pos);
                    frame_buf_len -= marker_pos;
                }

                if (frame_buf_len < FRAME_SIZE) break;

                /* Извлекаем кадр */
                FpgaToEspPacket_t pkt;
                memcpy(&pkt, frame_buf, FRAME_SIZE);

                /* Сдвигаем буфер */
                memmove(frame_buf, frame_buf + FRAME_SIZE,
                        frame_buf_len - FRAME_SIZE);
                frame_buf_len -= FRAME_SIZE;

                dump_buffer_push(&db, &pkt);
                total_frames++;

                printf("\r  Получено кадров: %zu (дамп #%d)   ",
                       db.count, dump_number + 1);
                fflush(stdout);

                /* Если есть аварии — сразу выводим в консоль */
                if (pkt.alarms.raw != 0) {
                    char flags_buf[512];
                    build_flags_string(&pkt, flags_buf, sizeof(flags_buf));
                    printf("\n  [!] Пакет #%u  t=%.3f с  АВАРИИ: %s\n",
                           pkt.packet_counter,
                           pkt.system_time_ms / 1000.0,
                           flags_buf);
                }
            }
        } else {
            /* Нет новых байт */
            if (got_data_ever && db.count > 0) {
                ULONGLONG now     = GetTickCount64();
                ULONGLONG elapsed = now - last_data_tick;
                if (elapsed >= DUMP_IDLE_TIMEOUT_MS) {
                    /* Тишина — сохраняем дамп */
                    printf("\n\n  Тишина %llu мс — сохраняю дамп...\n", elapsed);
                    dump_number++;
                    save_dump_csv(&db, dump_number);
                    dump_buffer_clear(&db);
                    got_data_ever = false;
                    printf("\n  Ожидание следующего дампа...\n\n");
                }
            } else if (!got_data_ever) {
                printf("\r  Ожидание данных...                 ");
                fflush(stdout);
            }
            Sleep(10);
        }
    }

    /* Сюда попадаем только при Ctrl+C (через ExitProcess) */
    if (db.count > 0) {
        dump_number++;
        save_dump_csv(&db, dump_number);
    }
    dump_buffer_free(&db);
}

/* ─── main ─────────────────────────────────────────────────────────────── */
int main(void)
{
    SetConsoleCP(65001);
    SetConsoleOutputCP(65001);
    SetConsoleCtrlHandler(console_ctrl_handler, TRUE);

    printf("========================================\n");
    printf("  UPS Dump Analyzer  (ESP32S3 -> CSV)\n");
    printf("  Размер кадра FpgaToEspPacket_t: %d байт\n", FRAME_SIZE);
    printf("========================================\n\n");

    scan_com_ports();

    if (available_ports_count == 0) {
        printf("\nПодключите ESP32 и перезапустите программу.\n");
        printf("Нажмите Enter для выхода...\n");
        (void)getchar();
        return 1;
    }

    int choice = 0;
    if (available_ports_count == 1) {
        choice = 0;
        printf("\nАвтовыбор: COM%d\n", available_ports[0]);
    } else {
        printf("\nВведите номер порта из списка (0..%d): ", available_ports_count - 1);
        while (scanf_s("%d", &choice) != 1 ||
               choice < 0 || choice >= (int)available_ports_count) {
            printf("Неверный ввод. Повторите (0..%d): ", available_ports_count - 1);
            /* Очищаем stdin */
            int c; while ((c = getchar()) != '\n' && c != EOF) {}
        }
    }

    uint32_t baudrate = 115200;
    printf("Скорость [бод, Enter = 115200]: ");
    /* Очищаем остаток строки после scanf */
    { int c; while ((c = getchar()) != '\n' && c != EOF) {} }

    char baud_str[32] = "";
    if (fgets(baud_str, sizeof(baud_str), stdin) && baud_str[0] != '\n') {
        uint32_t b = (uint32_t)strtoul(baud_str, NULL, 10);
        if (b > 0) baudrate = b;
    }

    HANDLE hPort = open_com_port(available_ports[choice], baudrate);
    if (hPort == INVALID_HANDLE_VALUE) {
        printf("Нажмите Enter для выхода...\n");
        (void)getchar();
        return 1;
    }

    receive_dump(hPort);

    CloseHandle(hPort);
    return 0;
}
