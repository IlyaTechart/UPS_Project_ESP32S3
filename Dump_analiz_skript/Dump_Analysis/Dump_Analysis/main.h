#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* ─── Структуры пакета (pragma pack(1), little-endian) ─────────────────── */
#pragma pack(push, 1)

typedef struct {
    union {
        uint16_t raw;
        struct {
            uint8_t grid_status : 1;
            uint8_t bypass_grid_status : 1;
            uint8_t rectifier_status : 1;
            uint8_t inverter_status : 1;
            uint8_t pwr_via_inverter : 1;
            uint8_t pwr_via_bypass : 1;
            uint8_t sync_status : 1;
            uint8_t load_mode : 1;
            uint8_t sound_alarm : 1;
            uint8_t battery_status : 1;
            uint8_t ups_mode : 1;
            uint8_t _reserved : 5;
        };
    };
} GroupStatus_t;

typedef struct {
    union {
        uint16_t raw;
        struct {
            uint8_t err_low_input_vol : 1;
            uint8_t err_high_dc_bus : 1;
            uint8_t err_low_bat_charge : 1;
            uint8_t err_bat_not_conn : 1;
            uint8_t err_inv_fault : 1;
            uint8_t err_inv_overcurrent : 1;
            uint8_t err_high_out_vol : 1;
            uint8_t err_fan_fault : 1;
            uint8_t err_replace_bat : 1;
            uint8_t err_rect_overheat : 1;
            uint8_t err_inv_overheat : 1;
            uint8_t _reserved : 5;
        };
    };
} GroupAlarms_t;

typedef struct {
    uint16_t v_in_AB;
    uint16_t v_in_BC;
    uint16_t v_in_CA;
    uint16_t v_bypass_A;
    uint16_t v_bypass_B;
    uint16_t v_bypass_C;
    uint16_t i_in_A;
    uint16_t i_in_B;
    uint16_t i_in_C;
    uint16_t freq_in;
} GroupInput_t;

typedef struct {
    uint16_t v_out_A;
    uint16_t v_out_B;
    uint16_t v_out_C;
    uint16_t freq_out;
    uint16_t i_out_A;
    uint16_t i_out_B;
    uint16_t i_out_C;
    uint16_t p_active_A;
    uint16_t p_active_B;
    uint16_t p_active_C;
    uint16_t p_apparent_A;
    uint16_t p_apparent_B;
    uint16_t p_apparent_C;
    uint16_t load_pct_A;
    uint16_t load_pct_B;
    uint16_t load_pct_C;
    uint16_t event_count;
} GroupOutput_t;

typedef struct {
    uint16_t bat_voltage;
    uint16_t bat_capacity;
    uint16_t bat_groups_count;
    uint16_t dc_bus_voltage;
    uint16_t bat_current;
    uint16_t backup_time;
} GroupBattery_t;

typedef struct {
    uint32_t      start_marker;
    uint32_t      packet_counter;
    GroupStatus_t status;
    GroupAlarms_t alarms;
    GroupInput_t  input;
    GroupOutput_t output;
    GroupBattery_t battery;
    uint32_t      crc32;
    uint32_t      system_time_ms;
} FpgaToEspPacket_t;

#pragma pack(pop)
