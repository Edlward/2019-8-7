#pragma once

#include "Basic.h"
#include "AC_Math.h"
#include "map_projection.h"
#include "vector_3.h"

#define IMU_Sensors_Count 3
#define Position_Sensors_Count 8

/*IMU����������*/
	typedef struct
	{
		bool present;	//�������Ƿ����
		TIME last_update_time;	//�ϴθ���ʱ��
		float sample_time;	//����ʱ��
		
		float sensitivity;	//�����ȣ�ԭʼ����->ʵ�ʵ�λ ���ݣ�rad/s ���ٶȣ�cm/s^2 �ų���gauss��
		
		vector3_int data_raw;	//ԭʼ����
		vector3_float data;	//ʵ�ʵ�λ����
	}IMU_Sensor;
/*IMU����������*/

/*λ�ô���������*/
	typedef enum
	{
		Position_Sensor_Type_GlobalPositioning ,	//ȫ��λ����γ�ȶ�λ����GPS��
		Position_Sensor_Type_RelativePositioning ,	//��Զ�λ������ѹ�ƣ������ﲻ��ı䣩
		Position_Sensor_Type_RangePositioning ,	//���붨λ����ඨλ���糬��������������ܻ�仯��
	}Position_Sensor_Type;
	typedef enum
	{
		//s-λ�� v-�ٶ�
		//��sv_xy��ʾ�ô��������У�λ���ٶȵ�xy����
		Position_Sensor_DataType_s_xy ,
		Position_Sensor_DataType_s_z ,
		Position_Sensor_DataType_s_xyz ,
		
		Position_Sensor_DataType_v_xy ,
		Position_Sensor_DataType_v_z ,
		Position_Sensor_DataType_v_xyz ,
		
		Position_Sensor_DataType_sv_xy ,
		Position_Sensor_DataType_sv_z ,
		Position_Sensor_DataType_sv_xyz ,
	}Position_Sensor_DataType;
	typedef enum
	{
		Position_Sensor_frame_ENU ,	//�ٶ�������ENU����ϵ��
		Position_Sensor_frame_BodyHeading ,	//�ٶ�����xΪ��ͷ���������ƽ�У���yΪ�����ͷ�󷽣������ƽ�У���zΪ�Ϸ�
	}Position_Sensor_frame;
	typedef struct
	{
		bool publishing;	//�Ƿ����ڸ���
		
		bool present;	//�������Ƿ����
		bool available;	//�������Ƿ����
		TIME last_update_time;	//�ϴθ���ʱ��
		TIME inavailable_start_time;	//�����������ÿ�ʼʱ��
		float delay;	//��������ʱ
		float sample_time;	//����ʱ��
		
		bool safe;	//�������Ƿ�ȫ�����ݻ����仯���ᷢ������ ����ע�⣡���粻ȷ����Ҫ����Ϊsafe��
		Position_Sensor_Type sensor_type;	//���������ͣ���ö��ע�ͣ�
		Position_Sensor_DataType sensor_DataType;	//�������������ͣ���ö��ע�ͣ�
		Position_Sensor_frame velocity_data_frame;	//�ٶ���������ϵ����ö��ע�ͣ�
		
		vector3_double position_Global;	//��γ��
		vector3_float position;	//λ��(cm)
		vector3_float velocity;	//�ٶ�(cm/s)
	}Position_Sensor;

	//��ȡ��ǰ��γ�ȶ�λ�ɲ�����
	bool get_MP_Available();
	//��ȡ��ǰ��γ��תƽ����Ϣ
	const Map_Projection* get_MP();
	
	extern const unsigned char default_ultrasonic_sensor_index;
	extern const unsigned char  default_optical_flow_index;
	extern const unsigned char  internal_baro_sensor_index;
	extern const unsigned char  default_gps_sensor_index;
	extern const unsigned char default_uwb_sensor_index; 
/*λ�ô���������*/
	
/*IMU*/

	/*IMU��������ȡ����*/
		const IMU_Sensor* GetAccelerometer( unsigned char index );
		const IMU_Sensor* GetGyroscope( unsigned char index );
		const IMU_Sensor* GetMagnetometer( unsigned char index );
	/*IMU������ע�ắ��*/
	
	extern const unsigned char External_Magnetometer_Index;
	extern const unsigned char Internal_Magnetometer_Index;
	
/*IMU*/
	
/*λ�ô�����*/
	
	/*λ�ô�������ȡ����*/
		const Position_Sensor* GetPositionSensor( unsigned char index );
	/*λ�ô�������ȡ����*/
		
/*λ�ô�����*/

/*���*/
	float getBatteryVoltage();
/*���*/