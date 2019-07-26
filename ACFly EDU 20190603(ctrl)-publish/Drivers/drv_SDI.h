#pragma once

#include <stdint.h>

void Uart3_Send( const uint8_t* data , uint16_t length );
uint16_t read_Uart3( uint8_t* data , uint16_t length );
uint16_t Uart3_DataAvailable();
struct Patrol
{
	float cross_x;
	float cross_y;
	float turn_yaw;
	int cx_mean;
	char Langing_flag;
};

extern struct Patrol Patrol;	
void init_drv_SDI();