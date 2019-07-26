#include "Modes.h"
#include "Basic.h"
#include "stdlib.h"
#include "M30_Att.h"

#include "AC_Math.h"
#include "Receiver.h"
#include "InteractiveInterface.h"
#include "ControlSystem.h"

static void M30_Att_MainFunc();
static void M30_Att_enter();
static void M30_Att_exit();
const Mode M30_Att = 
{
	50 , //mode frequency
	M30_Att_enter , //enter
	M30_Att_exit ,	//exit
	M30_Att_MainFunc ,	//mode main func
};

typedef struct
{
	//�˳�ģʽ������
	uint16_t exit_mode_counter;
}MODE_INF;
static MODE_INF* Mode_Inf;

static void M30_Att_enter()
{
	Led_setStatus( LED_status_running1 );
	
	//��ʼ��ģʽ����
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Attitude_Control_Enable();
}

static void M30_Att_exit()
{
	Attitude_Control_Disable();
	
	free( Mode_Inf );
}

static void M30_Att_MainFunc()
{
	const Receiver* rc = get_current_Receiver();
		
	if( rc->available == false )
	{
		//���ջ�������
		Attitude_Control_set_Throttle( 0 );
		return;
	}
	
	//��ȡң��ҡ��λ��
	float throttle_stick = rc->data[0];
	float yaw_stick = rc->data[1];
	float pitch_stick = rc->data[2];
	float roll_stick = rc->data[3];
	
	/*�ж��˳�ģʽ*/
		if( throttle_stick < 5 && yaw_stick < 5 && pitch_stick < 5 && roll_stick > 95 )
		{
			if( ++Mode_Inf->exit_mode_counter >= 50 )
			{
				change_Mode( 1 );
				return;
			}
		}
		else
			Mode_Inf->exit_mode_counter = 0;
	/*�ж��˳�ģʽ*/
	
	//���Ÿ�ֱ�ӿ�����
	Attitude_Control_set_Throttle( throttle_stick );
	//��������˿ظ������
	Attitude_Control_set_Target_RollPitch( ( roll_stick - 50.0f )*0.015f , ( pitch_stick - 50.0f )*0.015f );
	//ƫ�������м���ƫ��
	//�����м����ƫ���ٶ�
	if( in_symmetry_range_offset_float( yaw_stick , 5 , 50 ) )
		Attitude_Control_set_YawLock();
	else
		Attitude_Control_set_Target_YawRate( ( 50.0f - yaw_stick )*0.05f );
}