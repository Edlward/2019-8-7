#include "Modes.h"
#include "Basic.h"
#include "stdlib.h"
#include "M32_PosCtrl.h"

#include "AC_Math.h"
#include "Receiver.h"
#include "InteractiveInterface.h"
#include "ControlSystem.h"
#include "MeasurementSystem.h"

static void M32_PosCtrl_MainFunc();
static void M32_PosCtrl_enter();
static void M32_PosCtrl_exit();
const Mode M32_PosCtrl = 
{
	50 , //mode frequency
	M32_PosCtrl_enter , //enter
	M32_PosCtrl_exit ,	//exit
	M32_PosCtrl_MainFunc ,	//mode main func
};

typedef struct
{
	//�˳�ģʽ������
	uint16_t exit_mode_counter;
}MODE_INF;
static MODE_INF* Mode_Inf;

static void M32_PosCtrl_enter()
{
	Led_setStatus( LED_status_running1 );
	
	//��ʼ��ģʽ����
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Altitude_Control_Enable();
}

static void M32_PosCtrl_exit()
{
	Altitude_Control_Disable();
	Attitude_Control_Disable();
	
	free( Mode_Inf );
}

static void M32_PosCtrl_MainFunc()
{
	const Receiver* rc = get_current_Receiver();
		
	if( rc->available == false )
	{
		//���ջ�������
		Attitude_Control_set_Throttle( 0 );
		return;
	}
	float throttle = rc->data[0];
	float yaw_stick = rc->data[1];
	float pitch_stick = rc->data[2];
	float roll_stick = rc->data[3];
	
	
	
	/*�ж��˳�ģʽ*/
		if( throttle < 5 && yaw_stick < 5 && pitch_stick < 5 && roll_stick > 95 )
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
		
	if( rc->data[4] > 60 )
	{
		if( get_Position_Control_Enabled() == false )
		{
			Position_Control_Enable();
			if( get_Position_Control_Enabled() == true )
				Led_setSignal( LED_signal_3 );
		}
	}
	else if( rc->data[4] < 40 )
	{
		if( get_Position_Control_Enabled() == true )
		{			
			Position_Control_Disable();
			if( get_Position_Control_Enabled() == false )
				Led_setSignal( LED_signal_1 );
		}
	}
		
	//������������ת��Bodyheading����ϵ
	float Yaw = Quaternion_getYaw( get_Airframe_attitude() );
	float Yaw_sin , Yaw_cos;
	arm_sin_cos_f32( rad2degree(Yaw) , &Yaw_sin , &Yaw_cos );
	float WindDisturbance_bodyheading_x = map_ENU2BodyHeading_x( get_WindDisturbance_x() , get_WindDisturbance_y() , Yaw_sin , Yaw_cos  );
	float WindDisturbance_bodyheading_y = map_ENU2BodyHeading_y( get_WindDisturbance_x() , get_WindDisturbance_y() , Yaw_sin , Yaw_cos  );
		
	//�߶ȿ�������
	if( in_symmetry_range_offset_float( throttle , 5 , 50 ) )
		Position_Control_set_ZLock();
	else
		Position_Control_set_TargetVelocityZ( ( apply_deadband_float( throttle - 50.0f , 5 ) ) * 6 );
	
	if( get_Position_Control_Enabled() )
	{
		//ˮƽ����
		if( in_symmetry_range_offset_float( roll_stick , 5 , 50 ) && in_symmetry_range_offset_float( pitch_stick , 5 , 50 ) )
			Position_Control_set_XYLock();
		else
		{
			float roll_sitck_d = apply_deadband_float( roll_stick - 50.0f , 5 );
			float pitch_sitck_d = apply_deadband_float( pitch_stick - 50.0f , 5 );
			Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
				pitch_sitck_d * 20 ,\
				-roll_sitck_d * 20 , \
				fabsf( roll_sitck_d  )*0.015f, \
				fabsf( pitch_sitck_d )*0.015f \
			);
		}
	}
	else
	{	//����
		//Roll Pitch�������루����������
		Attitude_Control_set_Target_RollPitch( \
			( roll_stick 	- 50.0f )*0.015f - -atan2f( WindDisturbance_bodyheading_y , constG ), \
			( pitch_stick - 50.0f )*0.015f - atan2f( WindDisturbance_bodyheading_x , constG ) );
	}
	
	//ƫ����������
	if( in_symmetry_range_offset_float( yaw_stick , 5 , 50 ) )
		Attitude_Control_set_YawLock();
	else
		Attitude_Control_set_Target_YawRate( ( 50.0f - yaw_stick )*0.05f );
}