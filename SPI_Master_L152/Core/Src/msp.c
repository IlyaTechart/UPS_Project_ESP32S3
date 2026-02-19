/*
 * wi-fi_hande.c
 *
 *  Created on: 17 февр. 2026 г.
 *      Author: q
 */


#include <msp.h>


UpsData_t ups_data[27] = {0};


void SetVelueInStruckt(ModulData_t *ups_data)
{
//	for(uint8_t i = 0; i < 12; i++)
//	{
//	    ups_data[i].v_in_a = 228.0 + ((float)(rand() % 20) / 10.0);
//	    ups_data[i].v_in_b = 228.0 + ((float)(rand() % 20) / 10.0);
//	    ups_data[i].v_in_c = 228.0 + ((float)(rand() % 20) / 10.0);
//
//	    ups_data[i].c_in_a = ((float)(rand() % 20) / 5.0);
//	    ups_data[i].c_in_b = ((float)(rand() % 20) / 5.0);
//	    ups_data[i].c_in_c = ((float)(rand() % 20) / 5.0);
//
//	    ups_data[i].f_in_a = 50.0 + ((float)(rand() % 3) / 2.0);
//	    ups_data[i].f_in_b = 50.0 + ((float)(rand() % 3) / 2.0);
//	    ups_data[i].f_in_c = 50.0 + ((float)(rand() % 3) / 2.0);
//	}

    // Генерируем фалги состояний
	for(uint8_t i = 0; i < 11; i++)
	{
		*((uint16_t*)&ups_data->packet.status + i) = (rand() % 2); // Более читаемый вариант
		*((uint16_t*)&ups_data->packet.alarms + i) = (rand() % 2); // Более читаемый вариант
	}

	// Генерируем входные значения
	ups_data->packet.input.v_in_AB = 380.0 + ((float)(rand() % 20) / 10.0);
	ups_data->packet.input.v_in_BC = 380.0 + ((float)(rand() % 20) / 10.0);
	ups_data->packet.input.v_in_CA = 380.0 + ((float)(rand() % 20) / 10.0);
	ups_data->packet.input.v_bypass_A = 380.0 + ((float)(rand() % 20) / 10.0);
	ups_data->packet.input.v_bypass_B = 380.0 + ((float)(rand() % 20) / 10.0);
	ups_data->packet.input.v_bypass_C = 380.0 + ((float)(rand() % 20) / 10.0);
	ups_data->packet.input.i_in_A = 380.0 + ((float)(rand() % 20) / 10.0);
	ups_data->packet.input.i_in_B = 380.0 + ((float)(rand() % 20) / 10.0);
	ups_data->packet.input.i_in_C = 380.0 + ((float)(rand() % 20) / 10.0);

	// Генерируем выходные значения
	ups_data->packet.output.v_out_A = 380.0 + ((float)(rand() % 20) / 10.0);
	ups_data->packet.output.v_out_B = 380.0 + ((float)(rand() % 20) / 10.0);
	ups_data->packet.output.v_out_C = 380.0 + ((float)(rand() % 20) / 10.0);
	ups_data->packet.output.freq_out = 50 + ((float)(rand() % 3));
	ups_data->packet.output.i_out_A = 10 + (float)(rand() % 2);
	ups_data->packet.output.i_out_B = 10 + (float)(rand() % 2);
	ups_data->packet.output.i_out_C = 10 + (float)(rand() % 2);
	ups_data->packet.output.p_active_A = ups_data->packet.output.v_out_A * ups_data->packet.output.i_out_A;
	ups_data->packet.output.p_active_B = ups_data->packet.output.v_out_B * ups_data->packet.output.i_out_B;
	ups_data->packet.output.p_active_C = ups_data->packet.output.v_out_C * ups_data->packet.output.i_out_C;
	ups_data->packet.output.p_apparent_A = ups_data->packet.output.v_out_A * ups_data->packet.output.i_out_A;
	ups_data->packet.output.p_apparent_B = ups_data->packet.output.v_out_B * ups_data->packet.output.i_out_B;
	ups_data->packet.output.p_apparent_C = ups_data->packet.output.v_out_C * ups_data->packet.output.i_out_C;
	ups_data->packet.output.load_pct_A = 80 + (float)(rand() % 5);
	ups_data->packet.output.load_pct_B = 80 + (float)(rand() % 5);
	ups_data->packet.output.load_pct_C = 80 + (float)(rand() % 5);
	ups_data->packet.output.event_count = 7 + (float)(rand() % 5);

	// Генерация параметров АКБ
	ups_data->packet.battery.bat_voltage = 120 + (float)(rand() % 5);
	ups_data->packet.battery.bat_capacity = 80 + (float)(rand() % 5);
	ups_data->packet.battery.bat_groups_count = 1000 + (float)(rand() % 100);
	ups_data->packet.battery.dc_bus_voltage = 120 + (float)(rand() % 5);
	ups_data->packet.battery.bat_current = 10 + (float)(rand() % 5);
	ups_data->packet.battery.backup_time = 10 + (float)(rand() % 30);


}
