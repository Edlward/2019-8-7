#include "drv_Uart7.h"

#include "Quaternion.h"
#include "MeasurementSystem.h"

#include "Commulink.h"
#include "STS.h"
#include "Sensors_Backend.h"
#include "RingBuf.h"

#include "TM4C123GH6PM.h"
#include "uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_gpio.h"
#include "Timer.h"
#include "udma.h"


//#define Position_3  //ʹ��3�㶨λ����

#define Position_2_tag   //ʹ��������ǩ��һ����վ�Ķ�λ����


#ifdef Position_2_tag 
   #define TAG0_TAG1  200
	 
#else 
   #include "trilateration.h"
#endif

/*��վ�ͱ�ǩʾ��ͼ
           

*/

static bool UWB_RCTrigger( unsigned int Task_ID );
static void UWB_Server( unsigned int Task_ID );

#ifdef Position_2_tag 
		/*�����ǩ�������վ�ľ���*/
		struct ANTHOR
		{
			int Anothor0;
			int Anothor1;
			int Anothor2;
			int Anothor3;
			char Flag;
			char rc_step;
		};
		struct ANTHOR TAG0 = {0,0,0,0,0};


#else 
    struct TAG
		{
		  int dist0;
		  int dist0;
		  int dist2;
			int dist3;
	  };
		
		struct TAG TAG0;

#endif
//�����ж�
static void UART7_Handler();

/*���ͻ�����*/
	#define TX7_BUFFER_SIZE 30
	static uint8_t tx7_buffer[TX7_BUFFER_SIZE];
	static RingBuf_uint8_t Tx7_RingBuf;
/*���ͻ�����*/

/*���ջ�����*/
	#define RX7_BUFFER_SIZE 30
	static uint8_t rx7_buffer[RX7_BUFFER_SIZE];
	static RingBuf_uint8_t Rx7_RingBuf;
/*���ջ�����*/

static void Counter(unsigned int Task_ID);

void init_drv_Uart7()
{
		//ʹ��Uart7���ţ�Rx:PD6 Tx:PD7��
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//ʹ��Uart7
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	
	//PE1����
	GPIOE->LOCK = GPIO_LOCK_KEY;
	GPIOE->CR |= (1<<1);
	GPIOE->LOCK = 0;
	
	//����GPIO
	GPIOPinConfigure(GPIO_PE0_U7RX);
	GPIOPinConfigure(GPIO_PE1_U7TX);
	GPIOPinTypeUART(GPIOE_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
		
	//����Uart
	UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet() , 115200,			
	                   (UART_CONFIG_WLEN_8 
										| UART_CONFIG_STOP_ONE 
										|	UART_CONFIG_PAR_NONE));	
	UARTFIFOEnable( UART7_BASE );
	
	//��ʼ��������
	RingBuf_uint8_t_init( &Tx7_RingBuf , tx7_buffer , TX7_BUFFER_SIZE );
	RingBuf_uint8_t_init( &Rx7_RingBuf , rx7_buffer , RX7_BUFFER_SIZE );
	
	//���ô��ڽ����ж�
	UARTIntEnable( UART7_BASE , UART_INT_RX | UART_INT_RT );
	UARTIntRegister( UART7_BASE , UART7_Handler );
	
	//����DMA����
	uDMAChannelControlSet( UDMA_PRI_SELECT | UDMA_CH21_UART7TX , \
		UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1 );
	UARTDMAEnable( UART7_BASE , UART_DMA_TX );
	UARTIntRegister( UART7_BASE , UART7_Handler );	
	uDMAChannelAssign(UDMA_CH21_UART7TX  );	
	
	//���ж�
	IntPrioritySet( INT_UART7 , INT_PRIO_2 );
	IntEnable( INT_UART7 );
	
	//ע��˿�
	Port uart_port;
	uart_port.read = read_UART7;
	uart_port.DataAvailable = UART7_DataAvailable;
	uart_port.write = Uart7_Send;
	PortRegister( uart_port );
	
	//	//ע�ᴫ����
	PositionSensorRegister( default_uwb_sensor_index ,
	                        Position_Sensor_Type_RelativePositioning ,
													Position_Sensor_DataType_s_xy ,
													Position_Sensor_frame_BodyHeading , 
													0.001f , 
													false );
	//��Ӽ򵥶��ο���Э���������
	STS_Add_Task( STS_Task_Trigger_Mode_Custom , 0 , UWB_RCTrigger , UWB_Server );
}

static bool UWB_RCTrigger( unsigned int Task_ID )
{
	if( Uart7_DataAvailable() )
		return true;
	return false;
}

void Uart7_Send( const uint8_t* data , uint16_t length )
{
	IntDisable( INT_UART7 );
	
	//��ȡʣ��Ļ������ռ�
	int16_t buffer_space = RingBuf_uint8_t_get_Freesize( &Tx7_RingBuf );
	//��ȡDMA�д����͵��ֽ���
	int16_t DMA_Remain = uDMAChannelSizeGet( UDMA_CH21_UART7TX );
	
	//����Ҫ���͵��ֽ���
	int16_t max_send_count = buffer_space - DMA_Remain;
	if( max_send_count < 0 )
		max_send_count = 0;
	uint16_t send_count = ( length < max_send_count ) ? length : max_send_count;
	
	//���������ֽ�ѹ�뻺����
	RingBuf_uint8_t_push_length( &Tx7_RingBuf , data , send_count );
//	for( uint8_t i = 0 ; i < send_count ; ++i )
//		RingBuf_uint8_t_push( &Tx7_RingBuf , data[i] );
	
	//��ȡDMA�����Ƿ����
	if( uDMAChannelIsEnabled( UDMA_CH21_UART7TX ) == false )
	{
		//DMA�����
		//���Լ�������
		uint16_t length;
		uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Tx7_RingBuf , &length );
		if( length )
		{
			uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH21_UART7TX , \
				UDMA_MODE_BASIC , p , (void*)&UART7->DR , length );
			uDMAChannelEnable( UDMA_CH21_UART7TX );
		}
	}
	IntEnable( INT_UART7 );
}

uint16_t read_UART7( uint8_t* data , uint16_t length )
{
	IntDisable( INT_UART7 );
	uint8_t read_bytes = RingBuf_uint8_t_pop_length( &Rx7_RingBuf , data , length );
	IntEnable( INT_UART7 );
	return read_bytes;
}
uint16_t UART7_DataAvailable()
{
	IntDisable( INT_UART7 );
	uint16_t bytes2read = RingBuf_uint8_t_get_Bytes2read( &Rx7_RingBuf );
	IntEnable( INT_UART7 );
	return bytes2read;
}



static void UART7_Handler()
{
	UARTIntClear( UART7_BASE , UART_INT_OE );
	UARTRxErrorClear( UART7_BASE );
	while( ( UART7->FR & (1<<4) ) == false	)
	{
		//����
		uint8_t rdata = UART7->DR;
		RingBuf_uint8_t_push( &Rx7_RingBuf , rdata );
	}
	
	if( uDMAChannelIsEnabled( UDMA_CH21_UART7TX ) == false )
	{
		uint16_t length;
		uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Tx7_RingBuf , &length );
		if( length )
		{
			uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH21_UART7TX , \
				UDMA_MODE_BASIC , p , (void*)&UART7->DR , length );
			uDMAChannelEnable( UDMA_CH21_UART7TX );
		}
	}
}


static uint8_t  UWB_RC_OVER=0; //���ռ���
static uint8_t Low_eight, Hight_eight;
static int last0,last1,last2,last3;


static void UWB_Server( unsigned int Task_ID )
{
	//�򵥶��ο���Э�����
	
	/*״̬������*/
		static uint8_t distance;	   
		uint8_t rc_buf[20];
	uint8_t length = read_UART7( rc_buf , 20 );

	for( uint8_t i = 0 ; i < length ; ++i )
	{
		uint8_t r_data = rc_buf[i];
		//����
		
		switch(TAG0.rc_step)
		{
			case 0: if(r_data == 'm')  /*֡ͷ���*/
							{
								TAG0.rc_step++; 
								break;
							}
							else  
							{
								TAG0.rc_step=0;
								break;
							}
							
			case 1: if(r_data == 'r') 
							{
								TAG0.rc_step++; 
								break;
							}
							else 
							{
								TAG0.rc_step=0;
								break;
							}
			case 2: 
				      if(r_data == 0x02)  /*�汾�ż��*/
							{
								TAG0.rc_step++; 
								break;
							}
							else 
							{
								TAG0.rc_step=0;
								break;
							}
		  case 3:
              if(r_data == 0x0f)  /*��ǩ���б�*/
							{
								//UWB_flag=0;
								TAG0.rc_step++; 
								break;
							}
							else 
							{
								TAG0.rc_step=0;
								break;
							}
			case 4: TAG0.rc_step++; break;
			case 5: TAG0.rc_step++; break;				
		  /*���ܻ�վ0��������ǩ�ľ���*/
			case 6: Low_eight = r_data; TAG0.rc_step++; break;				
			case 7: 
						Hight_eight = r_data; 
						TAG0.Anothor0 = ((uint16_t)Hight_eight<<8) | Low_eight; 
			      if (TAG0.Anothor0 > 20000)  
						{
							TAG0.rc_step = 0;
							TAG0.Anothor0 = last0;
						}
			      else 
						{
							TAG0.rc_step++;
							last0 = TAG0.Anothor0;
						}
						break;
			case 8: Low_eight = r_data; TAG0.rc_step++; break;				
			case 9: 
						Hight_eight = r_data; 
						TAG0.Anothor1 = ((uint16_t)Hight_eight<<8) | Low_eight; 
			      if (TAG0.Anothor1 > 20000) 
						{
							TAG0.rc_step = 0;
							TAG0.Anothor1 = last1;
						}
			      else
						{
							TAG0.rc_step++;
							last1 = TAG0.Anothor1;
						}
						break;
			case 10: Low_eight = r_data; TAG0.rc_step++; break;				
			case 11: 
						Hight_eight = r_data; 
						TAG0.Anothor2 = ((uint16_t)Hight_eight<<8) | Low_eight; 
			      if (TAG0.Anothor2 > 20000)
						{
							TAG0.rc_step = 0;
							TAG0.Anothor2 = last2;
						}
			      else 
						{
							TAG0.rc_step++;
							last2 = TAG0.Anothor2;
						}
						break;
			case 12: Low_eight = r_data; TAG0.rc_step++; break;				
			case 13: 
						Hight_eight = r_data; 
						TAG0.Anothor3 = ((uint16_t)Hight_eight<<8) | Low_eight; 
			      if (TAG0.Anothor3 > 20000)
						{
							TAG0.rc_step = 0;
							TAG0.Anothor3 = last3;
						}
			      else 
						{
							TAG0.rc_step++;
							last3 = TAG0.Anothor3;
						}
						break;
			case 14: 
				      if(r_data == '\n')  
							{
								TAG0.rc_step++; 
								break;
							}
							else 
							{
								TAG0.rc_step=0;
								break;
							}
			case 15: 
						if(r_data == '\r')  /*һ���������ݽ������*/
						{	
	            Counter(0);							
						}
						TAG0.rc_step = 0;
						break;
			default: TAG0.rc_step=0;break;
		}
	}
}


 /*ʹ��������ǩһ����վ�Ķ�λ���㷽��*/
static void Counter(unsigned int Task_ID)
{
          vector3_float Base_station0, Error;
					float angle0_0,angle0_1,h;
					vector3_float  Base_station1,Mid_station,TAG_station;
					h = get_Position().z - 20;
					TAG0.Anothor0 = sqrt((pow(TAG0.Anothor0,2) - pow(h,2)));
					TAG0.Anothor1 = sqrt((pow(TAG0.Anothor1,2) - pow(h,2)));
					TAG0.Anothor2 = sqrt((pow(TAG0.Anothor2,2) - pow(h,2)));
					TAG0.Anothor3 = sqrt((pow(TAG0.Anothor3,2) - pow(h,2)));
          
	        //ʹ��DOA�㷨��ȡ���꣨��վ����Ϊ����վ0 (0,0)����վ1(500,0), ��վ2 (250,500).���뵥λcm ��
          Base_station0.y = ((pow(TAG0.Anothor0,2) - pow(TAG0.Anothor1, 2))/1000 + 250);
					Base_station0.x = ((pow(TAG0.Anothor0,2) + pow(TAG0.Anothor1, 2) - 2*pow(TAG0.Anothor2,2))/2000 + 187.5);
	       
	
//	        //ʹ��Chan�㷨��ȡ���꣨��վ����Ϊ����վ0 (0,0)����վ1(500,0), ��վ2 (250,500).���뵥λcm ��
//	        Base_station0.y = TAG0.Anothor0*(TAG0.Anothor0/500.0 - TAG0.Anothor1/500.0) - pow((TAG0.Anothor0-TAG0.Anothor1),2)/1000.0 + 250;
//	        Base_station0.x = pow((TAG0.Anothor0-TAG0.Anothor1),2)/2000.0 - pow((TAG0.Anothor0 - TAG0.Anothor2),2)/1000.0 + TAG0.Anothor0*(TAG0.Anothor0+TAG0.Anothor1-2*TAG0.Anothor2)/1000 + 187.5;
//	       
	         Base_station0.z = 0;
					if(fabsf(Base_station0.x)<10000 && fabsf(Base_station0.y) < 10000)
					{
						 PositionSensorUpdatePosition(default_uwb_sensor_index, Base_station0, true, -1);
					}
					else
						PositionSensorSetInavailable( default_uwb_sensor_index );
}


uint16_t Uart7_DataAvailable()
{
	IntDisable( INT_UART7 );
	uint16_t bytes2read = RingBuf_uint8_t_get_Bytes2read( &Rx7_RingBuf );
	IntEnable( INT_UART7 );
	return bytes2read;
}
