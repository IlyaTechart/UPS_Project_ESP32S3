/*
 * wi-fi_hande.c
 *
 *  Created on: 17 февр. 2026 г.
 *      Author: q
 */


#include"wi-fi_hande.h"


UpsData_t ups_data[27] = {0};


void SetVelueInStruckt(UpsData_t *ups_data)
{
	for(uint8_t i = 0; i < 12; i++)
	{
	    ups_data[i].v_in_a = 228.0 + ((float)(rand() % 20) / 10.0);
	    ups_data[i].v_in_b = 228.0 + ((float)(rand() % 20) / 10.0);
	    ups_data[i].v_in_c = 228.0 + ((float)(rand() % 20) / 10.0);

	    ups_data[i].c_in_a = ((float)(rand() % 20) / 5.0);
	    ups_data[i].c_in_b = ((float)(rand() % 20) / 5.0);
	    ups_data[i].c_in_c = ((float)(rand() % 20) / 5.0);

	    ups_data[i].f_in_a = 50.0 + ((float)(rand() % 3) / 2.0);
	    ups_data[i].f_in_b = 50.0 + ((float)(rand() % 3) / 2.0);
	    ups_data[i].f_in_c = 50.0 + ((float)(rand() % 3) / 2.0);
	}


}
