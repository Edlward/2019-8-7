#pragma once

#include "Sensors.h"

extern IMU_Sensor Accelerometer[IMU_Sensors_Count];
extern IMU_Sensor Gyroscope[IMU_Sensors_Count];
extern IMU_Sensor Magnetometer[IMU_Sensors_Count];

extern Position_Sensor Position_Sensors[ Position_Sensors_Count ];

/*IMU*/

	/*IMU������ע�ắ��*/
		bool IMUAccelerometerRegister( unsigned char index , float sensitivity );
		bool IMUAccelerometerUnRegister( unsigned char index );

		bool IMUGyroscopeRegister( unsigned char index , float sensitivity );
		bool IMUGyroscopeUnRegister( unsigned char index );
		
		bool IMUMagnetometerRegister( unsigned char index , float sensitivity );
		bool IMUMagnetometerUnRegister( unsigned char index );
	/*IMU������ע�ắ��*/
	
	/*IMU���������º���*/
		bool IMUAccelerometerUpdate( unsigned char index , vector3_int data );
		bool IMUGyroscopeUpdate( unsigned char index , vector3_int data );
		bool IMUMagnetometerUpdate( unsigned char index , vector3_int data );
	/*IMU���������º���*/

/*IMU*/

/*λ�ô�����*/
	//������ϸ�����Sensors.h��λ�ô�����ע��
		
	/*λ�ô�����ע�ắ��*/
		//safe���������Ƿ�ȫ�����ݻ����仯���ᷢ������ ����ע�⣡���粻ȷ����Ҫ����Ϊsafe��
		bool PositionSensorRegister( 
			unsigned char index ,\
			Position_Sensor_Type sensor_type ,\
			Position_Sensor_DataType sensor_data_type ,\
			Position_Sensor_frame sensor_vel_frame ,\
			float delay ,\
			bool safe \
		);
		//ע��������
		bool PositionSensorUnRegister( unsigned char index );
	/*λ�ô�����ע�ắ��*/
				
	//����λ�ô�����DataType
	bool PositionSensorChangeDataType( unsigned char index , Position_Sensor_DataType datatype );
			
	/*λ�ô��������º���*/
		//delay����С��0�򲻻�ı�delay		
		bool PositionSensorUpdatePositionGlobal( unsigned char index , vector3_double position_Global , bool available , float delay );
		bool PositionSensorUpdatePosition( unsigned char index , vector3_float position , bool available , float delay );
		bool PositionSensorUpdatePositionGlobalVel( unsigned char index , vector3_double position_Global , vector3_float vel , bool available , float delay );
		bool PositionSensorUpdatePositionVel( unsigned char index , vector3_float position , vector3_float vel , bool available , float delay );
		bool PositionSensorUpdateVel( unsigned char index , vector3_float vel , bool available , float delay );
		//λ�ô�����ʧЧ����
		bool PositionSensorSetInavailable( unsigned char index );
	/*IMU���������º���*/
		
/*λ�ô�����*/	