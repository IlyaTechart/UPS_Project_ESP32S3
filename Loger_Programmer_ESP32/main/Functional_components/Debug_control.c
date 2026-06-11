#pragma once

#include <inttypes.h>
#include "esp_err.h"
#include "frames_structure.h"



void logger_print_one_frame(const ModulData_t *m, size_t frame_index)
{
    const FpgaToEspPacket_t *p = &m->packet;

    printf("\n========== frame #%zu ==========\n", frame_index);
    printf("start_marker:   0x%08" PRIx32 " (%" PRIu32 ")\n", p->start_marker, p->start_marker);

    printf("[STATUS] raw=0x%04x | Grid:%u BypGrid:%u Rect:%u Inv:%u PwrInv:%u PwrByp:%u Sync:%u Load:%u Sound:%u BatSt:%u UpsMode:%u\n",
           (unsigned)p->status.raw,
           (unsigned)p->status.grid_status, (unsigned)p->status.bypass_grid_status,
           (unsigned)p->status.rectifier_status, (unsigned)p->status.inverter_status,
           (unsigned)p->status.pwr_via_inverter, (unsigned)p->status.pwr_via_bypass,
           (unsigned)p->status.sync_status, (unsigned)p->status.load_mode,
           (unsigned)p->status.sound_alarm, (unsigned)p->status.battery_status,
           (unsigned)p->status.ups_mode);

    printf("[ALARM] raw=0x%04x | LowIn:%u HiDC:%u LowBat:%u NoBat:%u InvF:%u InvOC:%u HiOut:%u Fan:%u ReplBat:%u RectHot:%u InvHot:%u\n",
           (unsigned)p->alarms.raw,
           (unsigned)p->alarms.err_low_input_vol, (unsigned)p->alarms.err_high_dc_bus,
           (unsigned)p->alarms.err_low_bat_charge, (unsigned)p->alarms.err_bat_not_conn,
           (unsigned)p->alarms.err_inv_fault, (unsigned)p->alarms.err_inv_overcurrent,
           (unsigned)p->alarms.err_high_out_vol, (unsigned)p->alarms.err_fan_fault,
           (unsigned)p->alarms.err_replace_bat, (unsigned)p->alarms.err_rect_overheat,
           (unsigned)p->alarms.err_inv_overheat);

    printf("[INPUT] V_in AB/BC/CA (x0.1 V): %u %u %u | V_byp A/B/C: %u %u %u\n",
           (unsigned)p->input.v_in_AB, (unsigned)p->input.v_in_BC, (unsigned)p->input.v_in_CA,
           (unsigned)p->input.v_bypass_A, (unsigned)p->input.v_bypass_B, (unsigned)p->input.v_bypass_C);
    printf("[INPUT] I_in A/B/C (x0.1 A): %u %u %u | freq_in (x0.01 Hz): %u\n",
           (unsigned)p->input.i_in_A, (unsigned)p->input.i_in_B, (unsigned)p->input.i_in_C,
           (unsigned)p->input.freq_in);

    printf("[OUTPUT] V_out A/B/C (x0.1 V): %u %u %u | freq_out (x0.01 Hz): %u\n",
           (unsigned)p->output.v_out_A, (unsigned)p->output.v_out_B, (unsigned)p->output.v_out_C,
           (unsigned)p->output.freq_out);
    printf("[OUTPUT] I_out A/B/C (x0.1 A): %u %u %u\n",
           (unsigned)p->output.i_out_A, (unsigned)p->output.i_out_B, (unsigned)p->output.i_out_C);
    printf("[OUTPUT] P_active A/B/C (x0.1 kW): %u %u %u | P_apparent A/B/C (x0.1 kVA): %u %u %u\n",
           (unsigned)p->output.p_active_A, (unsigned)p->output.p_active_B, (unsigned)p->output.p_active_C,
           (unsigned)p->output.p_apparent_A, (unsigned)p->output.p_apparent_B, (unsigned)p->output.p_apparent_C);
    printf("[OUTPUT] Load %% A/B/C (x0.1): %u %u %u | event_count: %u\n",
           (unsigned)p->output.load_pct_A, (unsigned)p->output.load_pct_B, (unsigned)p->output.load_pct_C,
           (unsigned)p->output.event_count);

    {
        int16_t cur = (int16_t)p->battery.bat_current;
        uint16_t cur_abs = (uint16_t)(cur < 0 ? -cur : cur);
        printf("[BAT] Vbat (x0.1 V): %u | Cap (Ah): %u | groups: %u | DC (x0.1 V): %u\n",
               (unsigned)p->battery.bat_voltage, (unsigned)p->battery.bat_capacity,
               (unsigned)p->battery.bat_groups_count, (unsigned)p->battery.dc_bus_voltage);
        printf("[BAT] I (x0.1 A, signed): %s%u.%u | backup (min): %u\n",
               cur < 0 ? "-" : "",
               (unsigned)(cur_abs / 10u), (unsigned)(cur_abs % 10u),
               (unsigned)p->battery.backup_time);
    }

    printf("crc32:          0x%08" PRIx32 "\n", p->crc32);
    printf("system_time_ms: %" PRIu32 "\n", p->system_time_ms);

}



