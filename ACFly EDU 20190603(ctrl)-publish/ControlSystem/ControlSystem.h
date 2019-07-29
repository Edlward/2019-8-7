#pragma once

#include <stdbool.h>


	//      front
			//       x
			//       ^
			//       |
			// y < --O
			
			
			
/*�۲���*/
	//��ȡ��ͣ����
	float get_hover_throttle();
	//��ȡ�Ƿ��ڷ���
	bool get_is_inFlight();
	//��ȡENU x��������ɵļ��ٶ�cm/s^2��
	float get_WindDisturbance_x();
	//��ȡENU y��������ɵļ��ٶ�cm/s^2��
	float get_WindDisturbance_y();
/*�۲���*/

/*��̬����*/
	typedef enum
	{
		Attitude_ControlMode_Angle ,
		Attitude_ControlMode_AngularRate ,
		Attitude_ControlMode_Locking ,
	}Attitude_ControlMode;
	
	//�򿪹ر���̬������
	bool Attitude_Control_Enable();
	bool Attitude_Control_Disable();

	//��ȡ��ǰ����
	float get_Target_Throttle();
	//�趨����
	bool Attitude_Control_set_Throttle( float thr );
	//��ȡĿ��Roll
	float Attitude_Control_get_Target_Roll();
	//��ȡĿ��Pitch
	float Attitude_Control_get_Target_Pitch();
	//��ȡĿ��Yaw
	float Attitude_Control_get_Target_Yaw();
	//�趨Ŀ��Roll Pitch
	bool Attitude_Control_set_Target_RollPitch( float Roll , float Pitch );

	//�趨Ŀ��Yaw
	bool Attitude_Control_set_Target_Yaw( float Yaw );
	bool Attitude_Control_set_Target_YawRelative( float Yaw );
	//�趨Ŀ��Yaw�ٶ�
	bool Attitude_Control_set_Target_YawRate( float YawRate );
	//����Yaw��ɲ�������Ƕȣ�
	bool Attitude_Control_set_YawLock();
/*��̬����*/
	
/*λ�ÿ���*/
	typedef enum
	{				
		Position_ControlMode_Velocity = 11 ,	//�ٶȿ���ģʽ
		Position_ControlMode_Locking = 10 ,	//ɲ������λ��
		
		Position_ControlMode_RouteLine = 20 ,	//Ѳ��ģʽ
		Position_ControlMode_Position = 21 ,	//λ������ģʽ
		Position_ControlMode_Takeoff = 22 ,	//���ģʽ
	}Position_ControlMode;
	#define Is_PositionControlMode(x) (x >=20 && x<=29)
	
	//�򿪹رո߶ȿ�����
	bool Altitude_Control_Enable();
	bool Altitude_Control_Disable();
	
	/*�߶�*/
		//�趨Ŀ��߶�
		bool Position_Control_set_TargetPositionZ( float posz );
		bool Position_Control_set_TargetPositionZRelative( float posz );
		//�趨Ŀ�괹ֱ�ٶ�
		bool Position_Control_set_TargetVelocityZ( float velz );
		//ɲ�������߶�
		bool Position_Control_set_ZLock();
	
		//��ȡ��ǰ�߶ȿ���ģʽ
		Position_ControlMode get_Altitude_ControlMode();
	
		//��ɵ���ǰ�߶��Ϸ���height�߶�
		bool Position_Control_Takeoff_HeightRelative( float height );
	/*�߶�*/
	
	/*ˮƽλ��*/
		//�򿪹ر�ˮƽλ�ÿ���
		bool get_Position_Control_Enabled();
		bool Position_Control_Enable();
		bool Position_Control_Disable();
		
		//��ȡ��ǰˮƽλ�ÿ���ģʽ
		Position_ControlMode get_Position_ControlMode();
	
		//�趨Ŀ��ˮƽλ��
		bool Position_Control_set_TargetPositionXY( float posx , float posy );
		//�趨Ŀ��ˮƽλ�ã���Ե�ǰ���꣩
		bool Position_Control_set_TargetPositionXYRelative( float posx , float posy );
		//�趨Ŀ��ˮƽλ�ã���Ե�ǰ������ƫ����Bodyheadingϵ�£�
		bool Position_Control_set_TargetPositionXYRelativeBodyHeading( float posx , float posy );
		//���ݾ�γ���趨Ŀ��ˮƽλ��
		bool Position_Control_set_TargetPositionXY_LatLon( double Lat , double Lon );
		
		//�趨Ŀ��ˮƽ�ٶȣ�Bodyheading����
		bool Position_Control_set_TargetVelocityBodyHeadingXY( float velx , float vely );
		//�趨Ŀ��ˮƽ�ٶȣ�Bodyheading���򣩲��������Ƕȣ�����������ĽǶȣ�
		bool Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( float velx , float vely , float maxRoll , float maxPitch );
		//ɲ��������ˮƽλ��
		bool Position_Control_set_XYLock();
	/*ˮƽλ��*/
/*λ�ÿ���*/