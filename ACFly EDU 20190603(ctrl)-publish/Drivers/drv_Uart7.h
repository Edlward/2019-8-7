#pragma once

#include <stdint.h>




void init_drv_Uart7();
uint16_t Uart7_DataAvailable();
uint16_t read_UART7( uint8_t* data , uint16_t length );
void Uart7_Send( const uint8_t* data , uint16_t length );
uint16_t UART7_DataAvailable();

extern  uint16_t Distance; 
