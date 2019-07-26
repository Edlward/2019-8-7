#include "Basic.h"
#include "STS.h"
#include "drv_Ultrasonic.h"
#include "Sensors_Backend.h"
#include "MeasurementSystem.h"

#include "TM4C123GH6PM.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "debug.h"
#include "fpu.h"
#include "gpio.h"
#include "pin_map.h"
#include "pwm.h"
#include "rom.h"
#include "sysctl.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"
#include "hw_gpio.h"

static void Ultrasonic_Handler();
static void Ultrasonic_Server( unsigned int Task_ID );
/*״̬��*/
	//���������״̬
	//255��δ��ʼ
	//0���ѷ��Ϳ�ʼ�ź�
	//1���Ѽ�¼��һ������
	static uint8_t Ultrasoinc_counter = 255;
/*״̬��*/

void init_drv_Ultrasonic()
{
	//��GPIOF��Դ��PF0��Echo PF1��Trig��
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	//PF0����
	GPIOF->LOCK = GPIO_LOCK_KEY;
	GPIOF->CR |= 0x1;
	GPIOF->LOCK = 0;
	
	//����PF1Ϊ���ģʽ
	GPIOPinTypeGPIOOutput( GPIOF_BASE , GPIO_PIN_1 );
	GPIOPinWrite( GPIOF_BASE , GPIO_PIN_1 , 0 );
	
	//����PF0Ϊ���벶������
	GPIOPinTypeTimer(GPIOF_BASE, GPIO_PIN_0);
	GPIOPinConfigure(GPIO_PF0_T0CCP0);
	
	//������ʱ��0(T0CCP0)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	
	//���ö�ʱ��0AΪ˫���ز���
	TimerConfigure( TIMER0_BASE ,TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP ); 
	TimerControlEvent(TIMER0_BASE,TIMER_A,TIMER_EVENT_BOTH_EDGES);	
	TimerLoadSet( TIMER0_BASE , TIMER_A , 0xffff );
	TimerPrescaleSet( TIMER0_BASE , TIMER_A , 0xff );
	
	//������ʱ���ж�
	TimerIntRegister(TIMER0_BASE,  TIMER_A , Ultrasonic_Handler);	
	IntPrioritySet( INT_TIMER0A , INT_PRIO_7);
	TimerIntEnable( TIMER0_BASE , TIMER_CAPA_EVENT);
	TimerEnable( TIMER0_BASE, TIMER_A );
	IntEnable( INT_TIMER0A );
	
	//ע�ᴫ����
	PositionSensorRegister( default_ultrasonic_sensor_index , \
													Position_Sensor_Type_RangePositioning , \
													Position_Sensor_DataType_s_z , \
													Position_Sensor_frame_ENU , \
													0.05f , \
													false );
	
	//��ӳ�������������
	STS_Add_Task( STS_Task_Trigger_Mode_RoughTime , 1.0f/10 , 0 , Ultrasonic_Server );
}

static void Ultrasonic_Server( unsigned int Task_ID )
{
	if( Ultrasoinc_counter != 255 )
		PositionSensorSetInavailable( default_ultrasonic_sensor_index );
	
	GPIOPinWrite( GPIOF_BASE , GPIO_PIN_1 , GPIO_PIN_1 );
	delay( 10e-6f );
	Ultrasoinc_counter = 0;
	GPIOPinWrite( GPIOF_BASE , GPIO_PIN_1 , 0 );
}

static void Ultrasonic_Handler()
{
	TimerIntClear( TIMER0_BASE , TIMER_CAPA_EVENT );
	
	static uint32_t last_value = 0;

	
	switch( Ultrasoinc_counter )
	{
		case 0:
			//�ѷ��Ϳ�ʼ�ź�
			last_value = TimerValueGet( TIMER0_BASE , TIMER_A );
			++Ultrasoinc_counter;
			break;
		
		case 1:
		{
			//�Ѽ�¼��һ������
			uint32_t trig_value = TimerValueGet( TIMER0_BASE , TIMER_A );
			float t;
			if( trig_value > last_value )
				t = ( trig_value - last_value ) * ( 1e-6f / 80 );
			else
				t = ( trig_value + 0xffffff - last_value ) * ( 1e-6f / 80 );
			
			vector3_float position;
			position.z = t * 17000.0f;
			if( position.z > 1 && position.z < 600 )
			{
				float lean_cosin = get_lean_angle_cosin();
				position.z *= lean_cosin;
				PositionSensorUpdatePosition( default_ultrasonic_sensor_index , position , true , -1 );
			}
			else
				PositionSensorSetInavailable( default_ultrasonic_sensor_index );
			
			Ultrasoinc_counter = 255;
			
			break;
		}
	}
}