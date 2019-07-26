#pragma once

#include <stdbool.h>
#include "Quaternion.h"

/*����ϵͳ״̬*/
	typedef enum
	{
		Measurement_System_Status_Initializing ,
		
		Measurement_System_Status_Ready ,
	}MeasurementSystem_Status;
	
	MeasurementSystem_Status get_Attitude_Measurement_System_Status();
	MeasurementSystem_Status get_Altitude_Measurement_System_Status();
	MeasurementSystem_Status get_Position_Measurement_System_Status();
/*����ϵͳ״̬*/

/*��ȡ�˲�����*/
	vector3_float get_AngularRateFilted();
	vector3_float get_AccelerationFilted();
	vector3_float get_AccelerationCtrl();
	vector3_float get_AngularRateCtrl();
/*��ȡ�˲�����*/

/*��������*/
	//��ȡ��б�ǵ�cos
	float get_lean_angle_cosin();
	//��ȡָ���������Ĺ���λ��
	bool get_Estimated_Sensor_Position_z( float* pos_z , unsigned char index );
	bool get_Estimated_Sensor_Position_xy( float* pos_x , float* pos_y , unsigned char index );
	bool get_Point_From_LatLon( float* pos_x , float* pos_y , double Lat , double Lon );
	bool get_LatLon_From_Point( float pos_x , float pos_y , double* Lat , double* Lon );
/*��������*/
	
/*��ȡ��ʷ����*/
	Quaternion get_attitude();
	Quaternion get_Airframe_attitude();
	
	vector3_float get_Position();
	vector3_float get_VelocityENU();
	vector3_float get_AccelerationENU();
	
	Quaternion get_history_attitude( float t );
	Quaternion get_history_Airframe_attitude( float t );
	vector3_float get_history_acceleration( float t );
	
	vector3_float get_history_position( float delay );
	vector3_float get_history_velocityENU( float delay );
/*��ȡ��ʷ����*/
	
	
/*��ȡ������֤*/
	//��ȡ������֤�Ƿ�ɹ�
	bool MS_Attitude_WGA();
	//��ȡ������֤��ʶ��
	unsigned int MS_Attitude_get_WGA( unsigned char index );
	
	//��ȡ������֤�Ƿ�ɹ�
	bool MS_Position_WGA();
	//��ȡ������֤��ʶ��
	unsigned int MS_Position_get_WGA( unsigned char index );
/*��ȡ������֤*/