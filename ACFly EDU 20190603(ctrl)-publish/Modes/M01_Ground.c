#include "Modes.h"
#include "Basic.h"
#include <stdlib.h>
#include <stdio.h>
#include "M01_Ground.h"

#include "Receiver.h"
#include "InteractiveInterface.h"
#include "OLED_Screens.h"
#include "MeasurementSystem.h"
#include "Quaternion.h"
#include "Sensors.h"

static void M01_Ground_MainFunc();
static void M01_Ground_enter();
static void M01_Ground_exit();
const Mode M01_Ground = 
{
	50 , //mode frequency
	M01_Ground_enter , //enter
	M01_Ground_exit ,	//exit
	M01_Ground_MainFunc ,	//mode main func
};

typedef struct
{
	//������ģʽ
	uint16_t pre_enter_mode;
	//����ģʽ������
	uint16_t pre_enter_mode_counter;
}MODE_INF;
static MODE_INF* Mode_Inf;

static void M01_Ground_enter()
{
	Led_setStatus( LED_status_ready1 );
	
	//��ʼ��ģʽ����
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Mode_Inf->pre_enter_mode_counter = 0;
	
	OLED_Clear();
	OLED_Draw_Str8x6( "Ground" , 0 , 0);	OLED_Draw_Str8x6( "page:01" , 0 , 128-7*6);
	OLED_Screen_BasicInf_init();
	//OLED_Screen_RawRC_init();
	OLED_Update();
}

static void M01_Ground_exit()
{
	free( Mode_Inf );
	OLED_Clear();
	OLED_Update();
}

static void M01_Ground_MainFunc()
{
	//ˢ����ʾ��
	OLED_Screen_BasicInf_Refresh();
	//OLED_Screen_RawRC_Refresh();
	OLED_Update();
	
	const Receiver* rc = get_current_Receiver();
	if( rc->available == false )
	{
		//���ջ�������
		Mode_Inf->pre_enter_mode_counter = 0;
		return;
	}
	float throttle = rc->data[0];
	float yaw_stick = rc->data[1];
	float pitch_stick = rc->data[2];
	float roll_stick = rc->data[3];
	/*�жϽ���ģʽ*/
		uint8_t pre_enter_mode = 0;
			
		if( (throttle < 5.0f) && (yaw_stick < 5.0f) && (pitch_stick < 5.0f) && (roll_stick < 5.0f) )
			pre_enter_mode = 12;	//���ٶ�У׼ģʽ
		else if( (throttle < 5.0f) && (yaw_stick < 5.0f) && (pitch_stick > 45) && (pitch_stick < 55) && (roll_stick < 5.0f) )
			pre_enter_mode = 15;	//ˮƽУ׼ģʽ
		else if( (throttle < 5.0f) && (yaw_stick < 5.0f) && (pitch_stick > 95.0f) && (roll_stick < 5.0f) )
			pre_enter_mode = 13;	//������У׼ģʽ
		else if( (throttle < 5.0f) && (yaw_stick > 95.0f) && (pitch_stick < 5.0f) && (roll_stick < 5.0f) )
		{
			if( rc->data[4] < 40 )
				pre_enter_mode = 30;
			else if( rc->data[4] > 70 )
				pre_enter_mode = 32;
			else
				pre_enter_mode = 35;
		}
		
		if( pre_enter_mode != Mode_Inf->pre_enter_mode || pre_enter_mode == 0 )
		{
			Mode_Inf->pre_enter_mode = pre_enter_mode;
			Mode_Inf->pre_enter_mode_counter = 0;
		}
		else
		{
			if( ++Mode_Inf->pre_enter_mode_counter >= 50 )
			{
				change_Mode( pre_enter_mode );
				if( rc->data[4] < 40 )
					Led_setSignal( LED_signal_1 );
				else if( rc->data[4] > 70 )
					Led_setSignal( LED_signal_3 );
				else
					Led_setSignal( LED_signal_2 );
			}
		}
	/*�жϽ���ģʽ*/
}