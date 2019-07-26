#include "Basic.h"
#include "drv_OpticalFlow.h"
#include "Sensors_Backend.h"
#include "MeasurementSystem.h"

#include "TM4C123GH6PM.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "hw_types.h"
#include "hw_ints.h"
#include "debug.h"
#include "fpu.h"
#include "gpio.h"
#include "pin_map.h"
#include "sysctl.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"
#include "hw_gpio.h"
#include "udma.h"

#include "drv_Uart1.h"

static void UART1_Handler();

void init_drv_Uart1()
{
	//ʹ��Uart1���ţ�Rx:PB0 Tx:PB1��
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//ʹ��UART1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	
	//����GPIO
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIOB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
	
	//����Uart
	//UARTЭ������ ������115200 8λ 1ֹͣλ  ��У��λ	
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet() , 115200,			
	                   (UART_CONFIG_WLEN_8 
										| UART_CONFIG_STOP_ONE 
										|	UART_CONFIG_PAR_NONE));	
	
	UARTIntEnable(UART1_BASE,UART_INT_RX);//ʹ��UART0�����ж�		
	UARTIntRegister(UART1_BASE,UART1_Handler);//UART�жϵ�ַע��	
	IntPrioritySet(INT_UART1, INT_PRIO_7);
	UARTFIFODisable( UART1_BASE );
}


void Uart3_Send( const uint8_t* data , uint16_t length )
{
	IntDisable( INT_Uart1 );
	
	//��ȡʣ��Ļ������ռ�
	int16_t buffer_space = RingBuf_uint8_t_get_Freesize( &Tx_RingBuf );
	//��ȡDMA�д����͵��ֽ���
	int16_t DMA_Remain = uDMAChannelSizeGet( udma_ch );
	
	//����Ҫ���͵��ֽ���
	int16_t max_send_count = buffer_space - DMA_Remain;
	if( max_send_count < 0 )
		max_send_count = 0;
	uint16_t send_count = ( length < max_send_count ) ? length : max_send_count;
	
	//���������ֽ�ѹ�뻺����
	RingBuf_uint8_t_push_length( &Tx_RingBuf , data , send_count );
//	for( uint8_t i = 0 ; i < send_count ; ++i )
//		RingBuf_uint8_t_push( &Tx_RingBuf , data[i] );
	
	//��ȡDMA�����Ƿ����
	if( uDMAChannelIsEnabled( UDMA_CH17_UART3TX ) == false )
	{
		//DMA�����
		//���Լ�������
		uint16_t length;
		uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Tx_RingBuf , &length );
		if( length )
		{
			uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH17_UART3TX , \
				UDMA_MODE_BASIC , p , (void*)&UART3->DR , length );
			uDMAChannelEnable( UDMA_CH17_UART3TX );
		}
	}
	IntEnable( INT_UART3 );
}
static void UART1_Handler()
{
	UARTIntClear( UART1_BASE , UART_INT_OE );
	UARTRxErrorClear( UART1_BASE );
	while( ( Uart1->FR & (1<<4) ) == false	)
	{
		//����
		uint8_t rdata = Uart1->DR & 0xff;
		RingBuf_uint8_t_push( &Rx_RingBuf , rdata );
	}
	
	if( uDMAChannelIsEnabled( UDMA_CH17_UART3TX ) == false )
	{
		uint16_t length;
		uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Tx_RingBuf , &length );
		if( length )
		{
			uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH17_UART3TX , \
				UDMA_MODE_BASIC , p , (void*)&UART3->DR , length );
			uDMAChannelEnable( UDMA_CH17_UART3TX );
		}
	}
}
uint16_t read_Uart3( uint8_t* data , uint16_t length )
{
	IntDisable( INT_UART3 );
	uint8_t read_bytes = RingBuf_uint8_t_pop_length( &Rx_RingBuf , data , length );
	IntEnable( INT_UART3 );
	return read_bytes;
}
uint16_t Uart3_DataAvailable()
{
	IntDisable( INT_UART3 );
	uint16_t bytes2read = RingBuf_uint8_t_get_Bytes2read( &Rx_RingBuf );
	IntEnable( INT_UART3 );
	return bytes2read;
}