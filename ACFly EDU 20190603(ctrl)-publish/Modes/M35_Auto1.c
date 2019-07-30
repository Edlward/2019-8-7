#include "Modes.h"
#include "Basic.h"
#include "stdlib.h"
#include <stdio.h>
#include "M35_Auto1.h"

#include "AC_Math.h"
#include "Receiver.h"
#include "InteractiveInterface.h"
#include "ControlSystem.h"
#include "MeasurementSystem.h"

#include "drv_SDI.h"
#include "drv_Uart7.h"

static void M35_Auto1_MainFunc();
static void M35_Auto1_enter();
static void M35_Auto1_exit();
const Mode M35_Auto1 = 
{
	50 , //mode frequency
	M35_Auto1_enter , //enter
	M35_Auto1_exit ,	//exit
	M35_Auto1_MainFunc ,	//mode main func
};

typedef struct
{
	//�˳�ģʽ������
	uint16_t exit_mode_counter;
	
	//�Զ�����״̬��
	uint8_t auto_step1;	//0-��¼��ťλ��
											//1-�ȴ���ť������� 
											//2-�ȴ������� 
											//3-�ȴ�2��
											//4-����
											//5-�ȴ��������
	uint16_t auto_counter;
	float last_button_value;
	
	float last_height;
}MODE_INF;
static MODE_INF* Mode_Inf;

static uint8_t Locking_flag = 0;

static void M35_Auto1_enter()
{
	Led_setStatus( LED_status_running1 );
	
	//��ʼ��ģʽ����
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Mode_Inf->exit_mode_counter = 0;
	Mode_Inf->auto_step1 = Mode_Inf->auto_counter = 0;
	Altitude_Control_Enable();
}

static void M35_Auto1_exit()
{
	Altitude_Control_Disable();
	Attitude_Control_Disable();
	
	free( Mode_Inf );
}

static void M35_Auto1_MainFunc()
{
	const Receiver* rc = get_current_Receiver();
		
	if( rc->available == false )
	{
		//���ջ�������
		//����
		Position_Control_set_XYLock();
		Position_Control_set_TargetVelocityZ( -50 );
		return;
	}
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
		
	//�ж�ҡ���Ƿ����м�
	bool sticks_in_neutral = 
		in_symmetry_range_offset_float( throttle_stick , 5 , 50 ) && \
		in_symmetry_range_offset_float( yaw_stick , 5 , 50 ) && \
		in_symmetry_range_offset_float( pitch_stick , 5 , 50 ) && \
		in_symmetry_range_offset_float( roll_stick , 5 , 50 );
	
	extern vector3_float SDI_Point;
	extern TIME SDI_Time;
	
	if( sticks_in_neutral && get_Position_Measurement_System_Status() == Measurement_System_Status_Ready )
	{
		//ҡ�����м�
		//ִ���Զ�����		
		//ֻ����λ����Чʱ��ִ���Զ�����
		
		//��ˮƽλ�ÿ���
		Position_Control_Enable();
		switch( Mode_Inf->auto_step1 )
		{
			case 0:
				Mode_Inf->last_button_value = rc->data[5];
				++Mode_Inf->auto_step1;
				Mode_Inf->auto_counter = 0;
				break;
			
			case 1:
				//�ȴ���ť�������
				if( get_is_inFlight() == false && fabsf( rc->data[5] - Mode_Inf->last_button_value ) > 15 )
				{
					Position_Control_Takeoff_HeightRelative( 100.0f );
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				break;
				
			case 2:
				//�ȴ�������
				if( get_Altitude_ControlMode() == Position_ControlMode_Position )
				{
					Mode_Inf->last_button_value = rc->data[5];
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
					Patrol.Langing_flag = 0;
				}
				break;
				
			case 3:	
//			   if(Patrol.Langing_flag == 4 && Time_isValid(SDI_Time) && get_pass_time(SDI_Time) < 2.0f )
//				 {
//					 if(fabs(SDI_Point.x ) < 20)
//					 {
//						 Attitude_Control_set_Target_YawRate( -degree2rad(constrain_float( SDI_Point.x * 1.0f , 45 ) ));
//						 // Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( 15.0f , -constrain_float( SDI_Point.y * 3.0f , 100 )*5 , 0.08f , 0.08f	);
//							 Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
//															10 ,	\
//															constrain_float( SDI_Point.y*0.7  , 50 ) ,	\
//															0.2f ,	\
//															0.2f	\
//														);
//					 }
//					 else if(fabs(SDI_Point.x ) > 20 && (fabs(SDI_Point.x ) < 80))
//					 {
//						 
//						 
//						 if(++Mode_Inf->auto_counter < 100)
//						 {
//							 Attitude_Control_set_Target_YawRate( -degree2rad(constrain_float( SDI_Point.x * 1.0f , 50 ) ));
//						 // Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( 15.0f , -constrain_float( SDI_Point.y * 3.0f , 100 )*5 , 0.08f , 0.08f	);
//							 Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
//															3 ,	\
//															0 ,	\
//															0.1f ,	\
//															0.1f	\
//														);
//						 }
//						 else if(++Mode_Inf->auto_counter < 200)
//						 {
//									Position_Control_set_XYLock();
//									Mode_Inf->auto_counter = 0;
//							 
//					 }
//					 else if(fabs(SDI_Point.x ) > 80)
//					 {
//						 Attitude_Control_set_Target_YawRate( -degree2rad(constrain_float( SDI_Point.x * 1.30f , 70 ) )); 
//						 Position_Control_set_XYLock();
//						 
//					 }
//				 }
//			 }
//			 else
//			 { 
//						 Position_Control_set_XYLock();
//			 }

       if(++Mode_Inf->auto_counter > 100 )
			 {
				 Position_Control_set_XYLock();
				 if(Mode_Inf->auto_counter < 500)
				  Mode_Inf->auto_counter = 0;
			 }
			 else if(Patrol.Langing_flag == 4 && Time_isValid(SDI_Time) && get_pass_time(SDI_Time) < 2.0f )
			 { 
					 if(fabs(SDI_Point.x ) < 20)
					 {
						 Attitude_Control_set_Target_YawRate( -degree2rad(constrain_float( SDI_Point.x * 1.0f , 45 ) ));
						 // Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( 15.0f , -constrain_float( SDI_Point.y * 3.0f , 100 )*5 , 0.08f , 0.08f	);
							 Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
															10 ,	\
															constrain_float( SDI_Point.y*0.7  , 50 ) ,	\
															0.2f ,	\
															0.2f	\
														);
					 }
					 else if(fabs(SDI_Point.x ) > 20 && (fabs(SDI_Point.x ) < 80))
					 {
						 
						 
//						 if(++Mode_Inf->auto_counter < 100)
//						 {
							 Attitude_Control_set_Target_YawRate( -degree2rad(constrain_float( SDI_Point.x * 1.0f , 50 ) ));
						 // Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( 15.0f , -constrain_float( SDI_Point.y * 3.0f , 100 )*5 , 0.08f , 0.08f	);
							 Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
															3 ,	\
															0 ,	\
															0.1f ,	\
															0.1f	\
														);
//						 }
//						 else if(++Mode_Inf->auto_counter < 200)
//						 {
//									Position_Control_set_XYLock();
									//Mode_Inf->auto_counter = 0;
							 
					 }
					 else if(fabs(SDI_Point.x ) > 80)
					 {
						 Attitude_Control_set_Target_YawRate( -degree2rad(constrain_float( SDI_Point.x * 1.30f , 70 ) )); 
						 Position_Control_set_XYLock();
						 
					 }
				 }
			 else
			 { 
						 Position_Control_set_XYLock();
			 }
			 
			 
			 
			 
//					else
//					{
//						 	Attitude_Control_set_Target_YawRate( -degree2rad(constrain_float( SDI_Point.x * 1.0f , 45 ) ));
//						 // Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( 15.0f , -constrain_float( SDI_Point.y * 3.0f , 100 )*5 , 0.08f , 0.08f	);
//							 Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
//															10 ,	\
//															constrain_float( SDI_Point.y  , 50 ) ,	\
//															0.2f ,	\
//															0.2f	\
//														);
//					}
//					  Patrol.Langing_flag=0;
//				 }
//				 else if(Locking_flag == 0)
//				{
//					Locking_flag=1;
//					Mode_Inf->auto_counter = 0;
//					//Attitude_Control_set_YawLock();
//					//Position_Control_set_XYLock();
//				}
				
//				if(++Mode_Inf->auto_counter < 
					 
				//if(SDI_Point.x == 200)
//				else if(Locking_flag == 0) 
//				{
//					Locking_flag=1;
//					Attitude_Control_set_YawLock();
//					Position_Control_set_XYLock();
//					Position_Control_set_TargetVelocityZ( -50 );
//				}
//				if(Patrol.Langing_flag == 1)
//				{
//					Position_Control_set_XYLock();
//					++Mode_Inf->auto_step1;
//					Mode_Inf->auto_counter = 0;
//				}
			
			
				

//         //Ѳ�ߺ��Զ����䲿��
//         if(Patrol.Langing_flag != 1 )
//				 {
//					Attitude_Control_set_Target_YawRate( -degree2rad(constrain_float( SDI_Point.x * 1.0f , 70 ) ));
//			    Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( 15.0f , -constrain_float( SDI_Point.y * 3.0f , 100 )*5 , 0.08f , 0.08f	);
//				 }
//           if( Time_isValid(SDI_Time) && get_pass_time(SDI_Time) < 2.0f && Patrol.Langing_flag == 0)
//						{		
//									if(Patrol.Langing_flag == 3)   //Patrol.Langing_flag�ñ�־����3��ʱ����ж��㽵��
//									{
//										Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
//													constrain_float( SDI_Point.x  , 50 ) ,	\
//													constrain_float( SDI_Point.y  , 50 ) ,	\
//													0.2f ,	\
//													0.2f	\
//												);
//										Patrol.Langing_flag = 0;
//										Locking_flag = 0;
//									}
//									else if(Patrol.Langing_flag == 4)   //Patrol.Langing_flag�ñ�־����4��ʱ�����Ѳ��
//									{
//										Locking_flag = 0;
//										Patrol.Langing_flag = 0;
//										Mode_Inf->auto_counter = 0;
//										
//										Attitude_Control_set_Target_YawRate( -degree2rad(constrain_float( SDI_Point.x * 1.0f , 70 ) ));
//			              Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( 15.0f , -constrain_float( SDI_Point.y * 3.0f , 100 )*5 , 0.08f , 0.08f	);
//									}
//				          else if(Locking_flag == 0)
//									{
//										    Locking_flag = 1;
//											  Mode_Inf->auto_counter = 0;
//												Attitude_Control_set_YawLock();
//												Position_Control_set_XYLock();
//									}
//									
//						}
//						else
//						{
//							 if(Locking_flag == 0)
//							 {
//								Locking_flag = 1;
//								Mode_Inf->auto_counter = 0;
//								Attitude_Control_set_YawLock();
//								Position_Control_set_XYLock();
//							 }
//						 }
//						
//						if (++Mode_Inf->auto_counter > 250)
//						{
//								Position_Control_set_XYLock();
//								++Mode_Inf->auto_step1;
//								Mode_Inf->auto_counter = 0;
//						}
						
					//�ȴ���ť����
				if( fabsf( rc->data[5] - Mode_Inf->last_button_value ) > 15 )
				{
					Position_Control_set_XYLock();
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				break;
				
			case 4:
				//����
			  //if( ++Mode_Inf->auto_counter >= 250)
				//{
					Position_Control_set_TargetVelocityZ( -50 );
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
					break;
				//}
			case 5:
				//�ȴ��������
				if( get_is_inFlight() == false )
				{
					Mode_Inf->auto_step1 = 0;
					Mode_Inf->auto_counter = 0;
				}
				break;
		}
	}
	else
	{
		ManualControl:
		//ҡ�˲����м�
		//�ֶ�����
		Mode_Inf->auto_step1 = Mode_Inf->auto_counter = 0;
		
		char str[16];
		double Lat , Lon;
		get_LatLon_From_Point( get_Position().x , get_Position().y , &Lat , &Lon );
		sprintf( str , "%d", (int)(Lat*1e7) );
		OLED_Draw_Str8x6( str , 0 , 0 );
		sprintf( str , "%d", (int)(Lon*1e7) );
		OLED_Draw_Str8x6( str , 1 , 0 );
		OLED_Update();
		
		//�ر�ˮƽλ�ÿ���
		Position_Control_Disable();
		
		//�߶ȿ�������
		if( in_symmetry_range_offset_float( throttle_stick , 5 , 50 ) )
			Position_Control_set_ZLock();
		else
			Position_Control_set_TargetVelocityZ( ( throttle_stick - 50.0f ) * 6 );

		//ƫ����������
		if( in_symmetry_range_offset_float( yaw_stick , 5 , 50 ) )
			Attitude_Control_set_YawLock();
		else
			Attitude_Control_set_Target_YawRate( ( 50.0f - yaw_stick )*0.05f );
		
		//Roll Pitch��������
		Attitude_Control_set_Target_RollPitch( \
			( roll_stick 	- 50.0f )*0.015f, \
			( pitch_stick - 50.0f )*0.015f );
	}
}