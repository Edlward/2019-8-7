#include "Basic.h"
#include "Configurations.h"
#include <string.h>

#include "eeprom.h"
#include "flash.h"
#include "sysctl.h"

#include "Sensors.h"

#define CfgLength 512
static uint32_t configurations[ CfgLength ];

//��ҳ���ü�excel��ACFly EDU EEPROM����
//ÿ�����÷�Ϊ����������
//Cfg_set_initial_xxx �����ڳ�ʼ������
//Cfg_get_xxx �����ڶ�ȡ����
//Cfg_update_xxx �����ڸ������ò�д��eeprom

/*���ٶȼ�У׼*/
	//10��ʼΪ���ٶ�У׼����
	#define Accelerometer_Calib_Start 10
	#define Accelerometer_Calib_Group_Length 10
	#define Accelerometer_Calib_Offset_StartIndex 0
	#define Accelerometer_Calib_Sensivitity_StartIndex 3
	
	void Cfg_set_initial_AccelerometerOffset( uint8_t sensor , vector3_float offset )
	{
		if( sensor >= IMU_Sensors_Count )
			return;
		memcpy( &configurations[ Accelerometer_Calib_Start + sensor*Accelerometer_Calib_Group_Length + Accelerometer_Calib_Offset_StartIndex ] ,\
			&offset , 3*sizeof(float) );
	}
	vector3_float Cfg_get_AccelerometerOffset( uint8_t sensor )
	{
		vector3_float offset = { 0, 0, 0 };
		if( sensor >= IMU_Sensors_Count )
			return offset;
		memcpy( &offset , \
			&configurations[ Accelerometer_Calib_Start + sensor*Accelerometer_Calib_Group_Length + Accelerometer_Calib_Offset_StartIndex ] ,\
			3*sizeof(float) );
		return offset;
	}
	bool Cfg_update_AccelerometerOffset( uint8_t sensor , vector3_float offset )
	{
		if( sensor >= IMU_Sensors_Count )
			return false;
		uint16_t cfg_num = Accelerometer_Calib_Start + sensor*Accelerometer_Calib_Group_Length + Accelerometer_Calib_Offset_StartIndex;
		memcpy( &configurations[ cfg_num ] , &offset , 3*sizeof(float) );
		return !EEPROMProgram( &configurations[cfg_num] , cfg_num*4 , 3*sizeof(float) );
	}
	
	void Cfg_set_initial_AccelerometerSensivitity( uint8_t sensor , vector3_float Sensivitity )
	{
		if( sensor >= IMU_Sensors_Count )
			return;
		memcpy( &configurations[ Accelerometer_Calib_Start + sensor*Accelerometer_Calib_Group_Length + Accelerometer_Calib_Sensivitity_StartIndex ] ,\
			&Sensivitity , 3*sizeof(float) );
	}
	vector3_float Cfg_get_AccelerometerSensivitity( uint8_t sensor )
	{
		vector3_float Sensivitity = { 0, 0, 0 };
		if( sensor >= IMU_Sensors_Count )
			return Sensivitity;
		memcpy( &Sensivitity , \
			&configurations[ Accelerometer_Calib_Start + sensor*Accelerometer_Calib_Group_Length + Accelerometer_Calib_Sensivitity_StartIndex ] ,\
			3*sizeof(float) );
		return Sensivitity;
	}
	bool Cfg_update_AccelerometerSensivitity( uint8_t sensor , vector3_float Sensivitity )
	{
		if( sensor >= IMU_Sensors_Count )
			return false;
		uint16_t cfg_num = Accelerometer_Calib_Start + sensor*Accelerometer_Calib_Group_Length + Accelerometer_Calib_Sensivitity_StartIndex;
		memcpy( &configurations[ cfg_num ] , &Sensivitity , 3*sizeof(float) );
		return !EEPROMProgram( &configurations[cfg_num] , cfg_num*4 , 3*sizeof(float) );
	}
/*���ٶȼ�У׼*/
	
/*����У׼*/
	//40��ʼΪ����У׼����
	#define Gyroscope_Calib_Start 40
	#define Gyroscope_Calib_Group_Length 10
	#define Gyroscope_Calib_Offset_StartIndex 0
	#define Gyroscope_Calib_Sensivitity_StartIndex 3
	
	void Cfg_set_initial_GyroscopeOffset( uint8_t sensor , vector3_float offset )
	{
		if( sensor >= IMU_Sensors_Count )
			return;
		memcpy( &configurations[ Gyroscope_Calib_Start + sensor*Gyroscope_Calib_Group_Length + Gyroscope_Calib_Offset_StartIndex ] ,\
			&offset , 3*sizeof(float) );
	}
	vector3_float Cfg_get_GyroscopeOffset( uint8_t sensor )
	{
		vector3_float offset = { 0, 0, 0 };
		if( sensor >= IMU_Sensors_Count )
			return offset;
		memcpy( &offset , \
			&configurations[ Gyroscope_Calib_Start + sensor*Gyroscope_Calib_Group_Length + Gyroscope_Calib_Offset_StartIndex ] ,\
			3*sizeof(float) );
		return offset;
	}
	bool Cfg_update_GyroscopeOffset( uint8_t sensor , vector3_float offset )
	{
		if( sensor >= IMU_Sensors_Count )
			return false;
		uint16_t cfg_num = Gyroscope_Calib_Start + sensor*Gyroscope_Calib_Group_Length + Gyroscope_Calib_Offset_StartIndex;
		memcpy( &configurations[ cfg_num ] , &offset , 3*sizeof(float) );
		return !EEPROMProgram( &configurations[cfg_num] , cfg_num*4 , 3*sizeof(float) );
	}
	
	void Cfg_set_initial_GyroscopeSensivitity( uint8_t sensor , vector3_float Sensivitity )
	{
		if( sensor >= IMU_Sensors_Count )
			return;
		memcpy( &configurations[ Gyroscope_Calib_Start + sensor*Gyroscope_Calib_Group_Length + Gyroscope_Calib_Sensivitity_StartIndex ] ,\
			&Sensivitity , 3*sizeof(float) );
	}
	vector3_float Cfg_get_GyroscopeSensivitity( uint8_t sensor )
	{
		vector3_float Sensivitity = { 0, 0, 0 };
		if( sensor >= IMU_Sensors_Count )
			return Sensivitity;
		memcpy( &Sensivitity , \
			&configurations[ Gyroscope_Calib_Start + sensor*Gyroscope_Calib_Group_Length + Gyroscope_Calib_Sensivitity_StartIndex ] ,\
			3*sizeof(float) );
		return Sensivitity;
	}
	bool Cfg_update_GyroscopeSensivitity( uint8_t sensor , vector3_float Sensivitity )
	{
		if( sensor >= IMU_Sensors_Count )
			return false;
		uint16_t cfg_num = Gyroscope_Calib_Start + sensor*Gyroscope_Calib_Group_Length + Gyroscope_Calib_Sensivitity_StartIndex;
		memcpy( &configurations[ cfg_num ] , &Sensivitity , 3*sizeof(float) );
		return !EEPROMProgram( &configurations[cfg_num] , cfg_num*4 , 3*sizeof(float) );
	}
/*����У׼*/
	
/*������У׼*/
	//70��ʼΪ������У׼����
	#define Magnetometer_Calib_Start 70
	#define Magnetometer_Calib_Group_Length 10
	#define Magnetometer_Calib_Offset_StartIndex 0
	#define Magnetometer_Calib_Sensivitity_StartIndex 3
	
	void Cfg_set_initial_MagnetometerOffset( uint8_t sensor , vector3_float offset )
	{
		if( sensor >= IMU_Sensors_Count )
			return;
		memcpy( &configurations[ Magnetometer_Calib_Start + sensor*Magnetometer_Calib_Group_Length + Magnetometer_Calib_Offset_StartIndex ] ,\
			&offset , 3*sizeof(float) );
	}
	vector3_float Cfg_get_MagnetometerOffset( uint8_t sensor )
	{
		vector3_float offset = { 0, 0, 0 };
		if( sensor >= IMU_Sensors_Count )
			return offset;
		memcpy( &offset , \
			&configurations[ Magnetometer_Calib_Start + sensor*Magnetometer_Calib_Group_Length + Magnetometer_Calib_Offset_StartIndex ] ,\
			3*sizeof(float) );
		return offset;
	}
	bool Cfg_update_MagnetometerOffset( uint8_t sensor , vector3_float offset )
	{
		if( sensor >= IMU_Sensors_Count )
			return false;
		uint16_t cfg_num = Magnetometer_Calib_Start + sensor*Magnetometer_Calib_Group_Length + Magnetometer_Calib_Offset_StartIndex;
		memcpy( &configurations[ cfg_num ] , &offset , 3*sizeof(float) );
		return !EEPROMProgram( &configurations[cfg_num] , cfg_num*4 , 3*sizeof(float) );
	}
	
	void Cfg_set_initial_MagnetometerSensivitity( uint8_t sensor , vector3_float Sensivitity )
	{
		if( sensor >= IMU_Sensors_Count )
			return;
		memcpy( &configurations[ Magnetometer_Calib_Start + sensor*Magnetometer_Calib_Group_Length + Magnetometer_Calib_Sensivitity_StartIndex ] ,\
			&Sensivitity , 3*sizeof(float) );
	}
	vector3_float Cfg_get_MagnetometerSensivitity( uint8_t sensor )
	{
		vector3_float Sensivitity = { 0, 0, 0 };
		if( sensor >= IMU_Sensors_Count )
			return Sensivitity;
		memcpy( &Sensivitity , \
			&configurations[ Magnetometer_Calib_Start + sensor*Magnetometer_Calib_Group_Length + Magnetometer_Calib_Sensivitity_StartIndex ] ,\
			3*sizeof(float) );
		return Sensivitity;
	}
	bool Cfg_update_MagnetometerSensivitity( uint8_t sensor , vector3_float Sensivitity )
	{
		if( sensor >= IMU_Sensors_Count )
			return false;
		uint16_t cfg_num = Magnetometer_Calib_Start + sensor*Magnetometer_Calib_Group_Length + Magnetometer_Calib_Sensivitity_StartIndex;
		memcpy( &configurations[ cfg_num ] , &Sensivitity , 3*sizeof(float) );
		return !EEPROMProgram( &configurations[cfg_num] , cfg_num*4 , 3*sizeof(float) );
	}
/*������У׼*/

/*���ջ�У׼*/

	//100��ʼΪSbusУ׼����
	//120ΪPPMУ׼����
	#define RC_Calib_Start 100
	#define RC_Group_Length 20

	//Sbus���ջ�У׼��
	//100-101��ͨ��ӳ��
	//102-109��rc1-rc8ͨ����Сֵ
	//110-117��rc1-rc8ͨ��У��ϵ��	
	
	#define RC_Channel_Reflection_Start_index 0
	#define RC_Channel_Min_Start_index 2
	#define RC_Channel_Scale_Start_index 10
	void Cfg_set_initial_Channel_Reflection( RC_Type rc , uint8_t assign[8] )
	{
		memcpy( &configurations[ RC_Calib_Start + rc*RC_Group_Length + RC_Channel_Reflection_Start_index ] , assign , 8 );
	}
	const uint8_t* Cfg_get_Channel_Reflection( RC_Type rc )
	{
		return (uint8_t*)&configurations[ RC_Calib_Start + rc*RC_Group_Length + RC_Channel_Reflection_Start_index ];
	}
	bool Cfg_update_Channel_Reflection( RC_Type rc , uint8_t assign[8] )
	{
		uint16_t cfg_num = RC_Calib_Start + rc*RC_Group_Length + RC_Channel_Reflection_Start_index;
		memcpy( &configurations[cfg_num] , assign , 8 );
		return !EEPROMProgram( &configurations[cfg_num] , cfg_num*4 , 8 );
	}
	
	void Cfg_set_initial_Channel_Min( RC_Type rc , float channels_Min[8] )
	{
		memcpy( &configurations[ RC_Calib_Start + rc*RC_Group_Length + RC_Channel_Min_Start_index ] , channels_Min , 8*sizeof(float) );
	}
	const float* Cfg_get_Channel_Min( RC_Type rc )
	{
		return (float*)&configurations[ RC_Calib_Start + rc*RC_Group_Length + RC_Channel_Min_Start_index ];
	}
	bool Cfg_update_Channel_Min( RC_Type rc , float channels_Min[8] )
	{
		uint16_t cfg_num = RC_Calib_Start + rc*RC_Group_Length + RC_Channel_Min_Start_index;
		memcpy( &configurations[cfg_num] , channels_Min , 8*sizeof(float) );
		return !EEPROMProgram( &configurations[cfg_num] , cfg_num*4 , 8*sizeof(float) );
	}
	
	void Cfg_set_initial_Channel_Scale( RC_Type rc , float channels_Scale[8] )
	{
		memcpy( &configurations[ RC_Calib_Start + rc*RC_Group_Length + RC_Channel_Scale_Start_index ] , channels_Scale , 8*sizeof(float) );
	}
	const float* Cfg_get_Channel_Scale( RC_Type rc )
	{
		return (float*)&configurations[ RC_Calib_Start + rc*RC_Group_Length + RC_Channel_Scale_Start_index ];
	}
	bool Cfg_update_Channel_Scale( RC_Type rc , float channels_Scale[8] )
	{
		uint16_t cfg_num = RC_Calib_Start + rc*RC_Group_Length + RC_Channel_Scale_Start_index;
		memcpy( &configurations[cfg_num] , channels_Scale , 8*sizeof(float) );
		return !EEPROMProgram( &configurations[cfg_num] , cfg_num*4 , 8*sizeof(float) );
	}
/*���ջ�У׼*/

/*ˮƽУ׼*/
	//180-183:ˮƽУ׼��Ԫ��qw qx qy qz
	#define Horizontal_Calibration_Start 180
	void Cfg_set_initial_Horizontal_Calibration( Quaternion quat )
	{
		memcpy( &configurations[Horizontal_Calibration_Start] , &quat , 4*sizeof(float) );
	}
	Quaternion Cfg_get_Horizontal_Calibration()
	{
		Quaternion quat;
		memcpy( &quat , &configurations[Horizontal_Calibration_Start] , 4*sizeof(float) );
		return quat;
	}
	bool Cfg_update_Horizontal_Calibration( Quaternion quat )
	{
		memcpy( &configurations[Horizontal_Calibration_Start] , &quat , 4*sizeof(float) );
		return !EEPROMProgram( &configurations[Horizontal_Calibration_Start] , Horizontal_Calibration_Start*4 , 4*sizeof(float) );
	}
/*ˮƽУ׼*/

/*����������*/
	//����������
	#define UAVType_Start 200
	void Cfg_set_initial_UAVType( UAVType type )
	{
		( (uint8_t*)&configurations[ UAVType_Start ] )[0] = type;
	}
	UAVType Cfg_get_UAVType()
	{
		return ( (uint8_t*)&configurations[ UAVType_Start ] )[0];
	}
	bool Cfg_update_UAVType( UAVType type )
	{
		( (uint8_t*)&configurations[ UAVType_Start ] )[0] = type;
		return !EEPROMProgram( &configurations[UAVType_Start] , UAVType_Start*4 , 4 );
	}
/*����������*/

/*�������*/
	//�����ת����
	#define MotorCfg_Start 203
	void Cfg_set_initial_MotorStartingThrottle( uint8_t thr )
	{
		( (uint8_t*)&configurations[ MotorCfg_Start ] )[0] = thr;
	}
	uint8_t Cfg_get_MotorStartingThrottle()
	{
		return ( (uint8_t*)&configurations[ MotorCfg_Start ] )[0];
	}
	bool Cfg_update_MotorStartingThrottle( uint8_t thr )
	{
		( (uint8_t*)&configurations[ MotorCfg_Start ] )[0] = thr;
		return !EEPROMProgram( &configurations[MotorCfg_Start] , MotorCfg_Start*4 , 4 );
	}
	
	//�������������ϵ��
	void Cfg_set_initial_MotorNonlineFactor( float factor )
	{
		( (uint8_t*)&configurations[ MotorCfg_Start ] )[1] = factor * 100;
	}
	float Cfg_get_MotorNonlineFactor()
	{
		return ( (uint8_t*)&configurations[ MotorCfg_Start ] )[1] * 0.01f;
	}
	bool Cfg_update_MotorNonlineFactor( float factor )
	{
		( (uint8_t*)&configurations[ MotorCfg_Start ] )[1] = factor * 100;
		return !EEPROMProgram( &configurations[MotorCfg_Start] , MotorCfg_Start*4 , 4 );
	}
	
	//�����������ű���
	void Cfg_set_initial_MotorFullThrottleRatio( float ratio )
	{
		( (uint8_t*)&configurations[ MotorCfg_Start ] )[2] = ratio * 100;
	}
	float Cfg_get_MotorFullThrottleRatio()
	{
		return ( (uint8_t*)&configurations[ MotorCfg_Start ] )[2] * 0.01f;
	}
	bool Cfg_update_MotorFullThrottleRatio( float ratio )
	{
		( (uint8_t*)&configurations[ MotorCfg_Start ] )[2] = ratio * 100;
		return !EEPROMProgram( &configurations[MotorCfg_Start] , MotorCfg_Start*4 , 4 );
	}
	
	//������ٹ���ʱ��T
	#define MotorT_Start 204
	void Cfg_set_initial_MotorT( float T )
	{
		memcpy( &configurations[MotorT_Start] , &T , sizeof(float) );
	}
	float Cfg_get_MotorT()
	{
		return *(float*)&configurations[MotorT_Start];
	}
	bool Cfg_update_MotorT( float T )
	{
		memcpy( &configurations[MotorT_Start] , &T , sizeof(float) );
		return !EEPROMProgram( &configurations[MotorT_Start] , MotorT_Start*4 , sizeof(float) );
	}
/*�������*/		
	
/*BAT---��ز���*/
	//��ػ�׼��ѹ
	#define BAT_STVoltage_Start 206
	void Cfg_set_initial_BatSTVoltage( float T )
	{
		memcpy( &configurations[BAT_STVoltage_Start] , &T , sizeof(float) );
	}
	float Cfg_get_BatSTVoltage()
	{
		return *(float*)&configurations[BAT_STVoltage_Start];
	}
	bool Cfg_update_BatSTVoltage( float T )
	{
		memcpy( &configurations[BAT_STVoltage_Start] , &T , sizeof(float) );
		return !EEPROMProgram( &configurations[BAT_STVoltage_Start] , BAT_STVoltage_Start*4 , sizeof(float) );
	}
	
	//��ԴADC�Ŵ���
	#define BAT_VoltageADCMag_Start 207
	void Cfg_set_initial_BatVoltageADCMag( float T )
	{
		memcpy( &configurations[BAT_VoltageADCMag_Start] , &T , sizeof(float) );
	}
	float Cfg_get_BatVoltageADCMag()
	{
		return *(float*)&configurations[BAT_VoltageADCMag_Start];
	}
	bool Cfg_update_BatVoltageADCMag( float T )
	{
		memcpy( &configurations[BAT_VoltageADCMag_Start] , &T , sizeof(float) );
		return !EEPROMProgram( &configurations[BAT_VoltageADCMag_Start] , BAT_VoltageADCMag_Start*4 , sizeof(float) );
	}
/*BAT---��ز���*/
	
/*��̬���Ʋ���*/
	//��̬����жȼ�����
	//RPY��0-Roll 1-Pitch 2-Yaw
	//0���ж�
	//1����������P1
	//2����������P2
	//3����������P3
	//4����������P4
	//5��ǰ������P1
	//6��ǰ������P2
	//7��ǰ������P3
	//8��ǰ������P4
	#define RPYCtrl_Start 210
	#define RPYCtrl_Group_Length 10
	void Cfg_set_initial_RPYCtrl_b( uint8_t RPY , float b )
	{
		if( RPY > 2 )
			return;
		memcpy( &configurations[ RPYCtrl_Start + RPY*RPYCtrl_Group_Length + 0 ] , &b , sizeof(float) );
	}
	float Cfg_get_RPYCtrl_b( uint8_t RPY )
	{
		if( RPY > 2 )
			return 0;
		return *(float*)&configurations[ RPYCtrl_Start + RPY*RPYCtrl_Group_Length + 0 ];
	}
	bool Cfg_update_RPYCtrl_b( uint8_t RPY , float b )
	{
		if( RPY > 2 )
			return false;
		uint16_t cfg_num = RPYCtrl_Start + RPY*RPYCtrl_Group_Length + 0;
		memcpy( &configurations[ cfg_num ] , &b , sizeof(float) );
		return !EEPROMProgram( &configurations[cfg_num] , cfg_num*4 , sizeof(float) );
	}
	
	void Cfg_set_initial_RPYCtrl_Pn( uint8_t RPY , float P , uint8_t n )
	{
		if( RPY > 2 )
			return;
		if( n == 0 || n > 4 )
			return;
		memcpy( &configurations[ RPYCtrl_Start + RPY*RPYCtrl_Group_Length + 1 + n - 1 ] , &P , sizeof(float) );
	}
	float Cfg_get_RPYCtrl_Pn( uint8_t RPY , uint8_t n )
	{
		if( RPY > 2 )
			return 0;
		if( n == 0 || n > 4 )
			return 0;
		return *(float*)&configurations[ RPYCtrl_Start + RPY*RPYCtrl_Group_Length + 1 + n - 1 ];
	}
	bool Cfg_update_RPYCtrl_Pn( uint8_t RPY , float P , uint8_t n )
	{
		if( RPY > 2 )
			return false;
		if( n == 0 || n > 4 )
			return false;
		uint16_t cfg_num = RPYCtrl_Start + RPY*RPYCtrl_Group_Length + 1 + n - 1;
		memcpy( &configurations[ cfg_num ] , &P , sizeof(float) );
		return !EEPROMProgram( &configurations[cfg_num] , cfg_num*4 , sizeof(float) );
	}
	
	void Cfg_set_initial_RPYCtrl_TD4Pn( uint8_t RPY , float P , uint8_t n )
	{
		if( RPY > 2 )
			return;
		if( n == 0 || n > 4 )
			return;
		memcpy( &configurations[ RPYCtrl_Start + RPY*RPYCtrl_Group_Length + 5 + n - 1 ] , &P , sizeof(float) );
	}
	float Cfg_get_RPYCtrl_TD4Pn( uint8_t RPY , uint8_t n )
	{
		if( RPY > 2 )
			return 0;
		if( n == 0 || n > 4 )
			return 0;
		return *(float*)&configurations[ RPYCtrl_Start + RPY*RPYCtrl_Group_Length + 5 + n - 1 ];
	}
	bool Cfg_update_RPYCtrl_TD4Pn( uint8_t RPY , float P , uint8_t n )
	{
		if( RPY > 2 )
			return false;
		if( n == 0 || n > 4 )
			return false;
		uint16_t cfg_num = RPYCtrl_Start + RPY*RPYCtrl_Group_Length + 5 + n - 1;
		memcpy( &configurations[ cfg_num ] , &P , sizeof(float) );
		return !EEPROMProgram( &configurations[cfg_num] , cfg_num*4 , sizeof(float) );
	}
/*��̬���Ʋ���*/	
	
/*������֤*/	
	uint32_t Cfg_get_WGA( uint8_t index )
	{
		if( index >= 3 )
			return 0;
		return configurations[ 509 + index ];
	}
	bool Cfg_update_WGA( uint32_t wga[3] )
	{
		memcpy( &configurations[ 509 ] , wga , 3*4 );
		return !EEPROMProgram( &configurations[509] , 509*4 , 3*4 );
	}
/*������֤*/
	
	
/*Params*/
	/*UAV ����������*/
		//����������
		static bool Param_set_UAVType( float type )
		{
			UAVType uavtype = *(UAVType*)&type;
			if( uavtype != UAVType_Rotor4_X && \
					uavtype != UAVType_Rotor6_X && \
					uavtype != UAVType_Rotor8_X && \
					uavtype != UAVType_Rotor4_C && \
					uavtype != UAVType_Rotor42_C )
				return false;
			return Cfg_update_UAVType( uavtype );
		}
		static float Param_get_UAVType()
		{
			ALIGN4 UAVType type = Cfg_get_UAVType();
			return *(float*)&type;
		}
	/*UAV ����������*/
	
	/*MT---Motor�������*/		
		//��ת����
		static bool Param_set_MotorStartingThrottle( float thr )
		{
			if( thr > 40 || thr < 0 )
				return false;
			return Cfg_update_MotorStartingThrottle( *(unsigned char*)&thr );
		}
		static float Param_get_MotorStartingThrottle()
		{
			ALIGN4 unsigned char thr = Cfg_get_MotorStartingThrottle();
			return *(float*)&thr;
		}
		
		//�������������ϵ��
		static bool Param_set_MotorNonlineFactor( float factor )
		{
			if( factor > 1 || factor < 0 )
				return false;
			return Cfg_update_MotorNonlineFactor( factor );
		}
		static float Param_get_MotorNonlineFactor()
		{
			return Cfg_get_MotorNonlineFactor();
		}
		
		//�����������ű���
		static bool Param_set_MotorFullThrottleRatio( float ratio )
		{
			if( ratio > 1 || ratio < 0.6f )
				return false;
			return Cfg_update_MotorFullThrottleRatio( ratio );
		}
		static float Param_get_MotorFullThrottleRatio()
		{
			return Cfg_get_MotorFullThrottleRatio();
		}
		
		static bool Param_set_MotorT( float T )
		{
			return Cfg_update_MotorT( T );
		}
		static float Param_get_MotorT()
		{
			return Cfg_get_MotorT();
		}
	/*MT---Motor�������*/	
		
	/*BAT---�������*/
		//��ر�׼��ѹ
		static bool Param_set_BatSTVoltage( float V )
		{
			return Cfg_update_BatSTVoltage( V );
		}
		static float Param_get_BatSTVoltage()
		{
			return Cfg_get_BatSTVoltage();
		}
		
		//��ص�ѹADC�Ŵ���
		static bool Param_set_BatVADCMag( float Mag )
		{
			return Cfg_update_BatVoltageADCMag( Mag );
		}
		static float Param_get_BatVADCMag()
		{
			return Cfg_get_BatVoltageADCMag();
		}
	/*BAT---�������*/
		
	/*AC---Attitude Control��̬���Ʋ���*/
		//Roll
		static bool Param_set_RollCtrl_b( float b )
		{
			return Cfg_update_RPYCtrl_b( 0 , b );
		}
		static float Param_get_RollCtrl_b()
		{
			return Cfg_get_RPYCtrl_b( 0 );
		}
		static bool Param_set_RollCtrl_P1( float P1 )
		{
			return Cfg_update_RPYCtrl_Pn( 0 , P1 , 1 );
		}
		static float Param_get_RollCtrl_P1()
		{
			return Cfg_get_RPYCtrl_Pn( 0 , 1 );
		}
		static bool Param_set_RollCtrl_P2( float P2 )
		{
			return Cfg_update_RPYCtrl_Pn( 0 , P2 , 2 );
		}
		static float Param_get_RollCtrl_P2()
		{
			return Cfg_get_RPYCtrl_Pn( 0 , 2 );
		}
		static bool Param_set_RollCtrl_P3( float P3 )
		{
			return Cfg_update_RPYCtrl_Pn( 0 , P3 , 3 );
		}
		static float Param_get_RollCtrl_P3()
		{
			return Cfg_get_RPYCtrl_Pn( 0 , 3 );
		}
		static bool Param_set_RollCtrl_P4( float P4 )
		{
			return Cfg_update_RPYCtrl_Pn( 0 , P4 , 4 );
		}
		static float Param_get_RollCtrl_P4()
		{
			return Cfg_get_RPYCtrl_Pn( 0 , 4 );
		}
		static bool Param_set_RollCtrl_TD4P1( float TD4P )
		{
			return Cfg_update_RPYCtrl_TD4Pn( 0 , TD4P , 1 );
		}
		static float Param_get_RollCtrl_TD4P1()
		{
			return Cfg_get_RPYCtrl_TD4Pn( 0 , 1 );
		}
		static bool Param_set_RollCtrl_TD4P2( float TD4P )
		{
			return Cfg_update_RPYCtrl_TD4Pn( 0 , TD4P , 2 );
		}
		static float Param_get_RollCtrl_TD4P2()
		{
			return Cfg_get_RPYCtrl_TD4Pn( 0 , 2 );
		}
		static bool Param_set_RollCtrl_TD4P3( float TD4P )
		{
			return Cfg_update_RPYCtrl_TD4Pn( 0 , TD4P , 3 );
		}
		static float Param_get_RollCtrl_TD4P3()
		{
			return Cfg_get_RPYCtrl_TD4Pn( 0 , 3 );
		}
		static bool Param_set_RollCtrl_TD4P4( float TD4P )
		{
			return Cfg_update_RPYCtrl_TD4Pn( 0 , TD4P , 4 );
		}
		static float Param_get_RollCtrl_TD4P4()
		{
			return Cfg_get_RPYCtrl_TD4Pn( 0 , 4 );
		}
		
		//Pitch
		static bool Param_set_PitchCtrl_b( float b )
		{
			return Cfg_update_RPYCtrl_b( 1 , b );
		}
		static float Param_get_PitchCtrl_b()
		{
			return Cfg_get_RPYCtrl_b( 1 );
		}
		static bool Param_set_PitchCtrl_P1( float P1 )
		{
			return Cfg_update_RPYCtrl_Pn( 1 , P1 , 1 );
		}
		static float Param_get_PitchCtrl_P1()
		{
			return Cfg_get_RPYCtrl_Pn( 1 , 1 );
		}
		static bool Param_set_PitchCtrl_P2( float P2 )
		{
			return Cfg_update_RPYCtrl_Pn( 1 , P2 , 2 );
		}
		static float Param_get_PitchCtrl_P2()
		{
			return Cfg_get_RPYCtrl_Pn( 1 , 2 );
		}
		static bool Param_set_PitchCtrl_P3( float P3 )
		{
			return Cfg_update_RPYCtrl_Pn( 1 , P3 , 3 );
		}
		static float Param_get_PitchCtrl_P3()
		{
			return Cfg_get_RPYCtrl_Pn( 1 , 3 );
		}
		static bool Param_set_PitchCtrl_P4( float P4 )
		{
			return Cfg_update_RPYCtrl_Pn( 1 , P4 , 4 );
		}
		static float Param_get_PitchCtrl_P4()
		{
			return Cfg_get_RPYCtrl_Pn( 1 , 4 );
		}
		static bool Param_set_PitchCtrl_TD4P1( float TD4P )
		{
			return Cfg_update_RPYCtrl_TD4Pn( 1 , TD4P , 1 );
		}
		static float Param_get_PitchCtrl_TD4P1()
		{
			return Cfg_get_RPYCtrl_TD4Pn( 1 , 1 );
		}
		static bool Param_set_PitchCtrl_TD4P2( float TD4P )
		{
			return Cfg_update_RPYCtrl_TD4Pn( 1 , TD4P , 2 );
		}
		static float Param_get_PitchCtrl_TD4P2()
		{
			return Cfg_get_RPYCtrl_TD4Pn( 1 , 2 );
		}
		static bool Param_set_PitchCtrl_TD4P3( float TD4P )
		{
			return Cfg_update_RPYCtrl_TD4Pn( 1 , TD4P , 3 );
		}
		static float Param_get_PitchCtrl_TD4P3()
		{
			return Cfg_get_RPYCtrl_TD4Pn( 1 , 3 );
		}
		static bool Param_set_PitchCtrl_TD4P4( float TD4P )
		{
			return Cfg_update_RPYCtrl_TD4Pn( 1 , TD4P , 4 );
		}
		static float Param_get_PitchCtrl_TD4P4()
		{
			return Cfg_get_RPYCtrl_TD4Pn( 1 , 4 );
		}
		
		//Yaw
		static bool Param_set_YawCtrl_b( float b )
		{
			return Cfg_update_RPYCtrl_b( 2 , b );
		}
		static float Param_get_YawCtrl_b()
		{
			return Cfg_get_RPYCtrl_b( 2 );
		}
		static bool Param_set_YawCtrl_P1( float P1 )
		{
			return Cfg_update_RPYCtrl_Pn( 2 , P1 , 1 );
		}
		static float Param_get_YawCtrl_P1()
		{
			return Cfg_get_RPYCtrl_Pn( 2 , 1 );
		}
		static bool Param_set_YawCtrl_P2( float P2 )
		{
			return Cfg_update_RPYCtrl_Pn( 2 , P2 , 2 );
		}
		static float Param_get_YawCtrl_P2()
		{
			return Cfg_get_RPYCtrl_Pn( 2 , 2 );
		}
		static bool Param_set_YawCtrl_P3( float P3 )
		{
			return Cfg_update_RPYCtrl_Pn( 2 , P3 , 3 );
		}
		static float Param_get_YawCtrl_P3()
		{
			return Cfg_get_RPYCtrl_Pn( 2 , 3 );
		}
		static bool Param_set_YawCtrl_P4( float P4 )
		{
			return Cfg_update_RPYCtrl_Pn( 2 , P4 , 4 );
		}
		static float Param_get_YawCtrl_P4()
		{
			return Cfg_get_RPYCtrl_Pn( 2 , 4 );
		}
		static bool Param_set_YawCtrl_TD4P1( float TD4P )
		{
			return Cfg_update_RPYCtrl_TD4Pn( 2 , TD4P , 1 );
		}
		static float Param_get_YawCtrl_TD4P1()
		{
			return Cfg_get_RPYCtrl_TD4Pn( 2 , 1 );
		}
		static bool Param_set_YawCtrl_TD4P2( float TD4P )
		{
			return Cfg_update_RPYCtrl_TD4Pn( 2 , TD4P , 2 );
		}
		static float Param_get_YawCtrl_TD4P2()
		{
			return Cfg_get_RPYCtrl_TD4Pn( 2 , 2 );
		}
		static bool Param_set_YawCtrl_TD4P3( float TD4P )
		{
			return Cfg_update_RPYCtrl_TD4Pn( 2 , TD4P , 3 );
		}
		static float Param_get_YawCtrl_TD4P3()
		{
			return Cfg_get_RPYCtrl_TD4Pn( 2 , 3 );
		}
		static bool Param_set_YawCtrl_TD4P4( float TD4P )
		{
			return Cfg_update_RPYCtrl_TD4Pn( 2 , TD4P , 4 );
		}
		static float Param_get_YawCtrl_TD4P4()
		{
			return Cfg_get_RPYCtrl_TD4Pn( 2 , 4 );
		}
	/*AC---Attitude Control��̬���Ʋ���*/	
		
	//�����б�
	const Param Params[] = 
	{
		//Type                  , Name               , Update Function           , get Param
		
		//UAV
		{ MAV_PARAM_TYPE_UINT8  , "UAV_Type"     , Param_set_UAVType         , Param_get_UAVType } ,
		
		//MT---Motor
		{ MAV_PARAM_TYPE_UINT8  , "MT_STThr"     , Param_set_MotorStartingThrottle, Param_get_MotorStartingThrottle } ,
		{ MAV_PARAM_TYPE_REAL32 , "MT_NonlinF"   , Param_set_MotorNonlineFactor, Param_get_MotorNonlineFactor } ,
		{ MAV_PARAM_TYPE_REAL32 , "MT_FullThrR"  , Param_set_MotorFullThrottleRatio, Param_get_MotorFullThrottleRatio } ,
		{ MAV_PARAM_TYPE_REAL32 , "MT_T"         , Param_set_MotorT      , Param_get_MotorT } ,

		//BAT---Battery
		{ MAV_PARAM_TYPE_REAL32 , "BAT_STVoltage"    , Param_set_BatSTVoltage, Param_get_BatSTVoltage } ,
		{ MAV_PARAM_TYPE_REAL32 , "BAT_VADCMag"      , Param_set_BatVADCMag, Param_get_BatVADCMag } ,
		
		//AC---Attitude Control
			//Roll
		{ MAV_PARAM_TYPE_REAL32 , "AC_Roll_b"     , Param_set_RollCtrl_b     , Param_get_RollCtrl_b } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Roll_P1"    , Param_set_RollCtrl_P1    , Param_get_RollCtrl_P1 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Roll_P2"    , Param_set_RollCtrl_P2    , Param_get_RollCtrl_P2 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Roll_P3"    , Param_set_RollCtrl_P3    , Param_get_RollCtrl_P3 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Roll_P4"    , Param_set_RollCtrl_P4    , Param_get_RollCtrl_P4 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Roll_TD4P1"    , Param_set_RollCtrl_TD4P1    , Param_get_RollCtrl_TD4P1 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Roll_TD4P2"    , Param_set_RollCtrl_TD4P2    , Param_get_RollCtrl_TD4P2 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Roll_TD4P3"    , Param_set_RollCtrl_TD4P3    , Param_get_RollCtrl_TD4P3 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Roll_TD4P4"    , Param_set_RollCtrl_TD4P4    , Param_get_RollCtrl_TD4P4 } ,
			//Pitch
		{ MAV_PARAM_TYPE_REAL32 , "AC_Pitch_b"     , Param_set_PitchCtrl_b     , Param_get_PitchCtrl_b } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Pitch_P1"    , Param_set_PitchCtrl_P1    , Param_get_PitchCtrl_P1 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Pitch_P2"    , Param_set_PitchCtrl_P2    , Param_get_PitchCtrl_P2 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Pitch_P3"    , Param_set_PitchCtrl_P3    , Param_get_PitchCtrl_P3 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Pitch_P4"    , Param_set_PitchCtrl_P4    , Param_get_PitchCtrl_P4 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Pitch_TD4P1"    , Param_set_PitchCtrl_TD4P1    , Param_get_PitchCtrl_TD4P1 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Pitch_TD4P2"    , Param_set_PitchCtrl_TD4P2    , Param_get_PitchCtrl_TD4P2 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Pitch_TD4P3"    , Param_set_PitchCtrl_TD4P3    , Param_get_PitchCtrl_TD4P3 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Pitch_TD4P4"    , Param_set_PitchCtrl_TD4P4    , Param_get_PitchCtrl_TD4P4 } ,
			//Yaw
		{ MAV_PARAM_TYPE_REAL32 , "AC_Yaw_b"     , Param_set_YawCtrl_b       , Param_get_YawCtrl_b } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Yaw_P1"    , Param_set_YawCtrl_P1    , Param_get_YawCtrl_P1 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Yaw_P2"    , Param_set_YawCtrl_P2    , Param_get_YawCtrl_P2 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Yaw_P3"    , Param_set_YawCtrl_P3    , Param_get_YawCtrl_P3 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Yaw_P4"    , Param_set_YawCtrl_P4    , Param_get_YawCtrl_P4 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Yaw_TD4P1"    , Param_set_YawCtrl_TD4P1    , Param_get_YawCtrl_TD4P1 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Yaw_TD4P2"    , Param_set_YawCtrl_TD4P2    , Param_get_YawCtrl_TD4P2 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Yaw_TD4P3"    , Param_set_YawCtrl_TD4P3    , Param_get_YawCtrl_TD4P3 } ,
		{ MAV_PARAM_TYPE_REAL32 , "AC_Yaw_TD4P4"    , Param_set_YawCtrl_TD4P4    , Param_get_YawCtrl_TD4P4 } ,
	};
	//��������
	const uint16_t Params_Count = sizeof( Params ) / sizeof( Param );
	//������Ѱ�Ҳ���
	int16_t Params_Find( char name[17] )
	{
		for( uint16_t i = 0 ; i < Params_Count ; ++ i )
		{
			if( strcmp( name , Params[i].name ) == 0 )
				return i;
		}
		return -1;
	}
/*Params*/
	
void init_Configurations()
{
	uint32_t buf[10];
	//��ȡ�洢�汾
	EEPROMRead( buf , 0 , 4 );
	//����汾��һ�£�����д���ʼ����
	if( buf[0] != ( ( 0xAC << 24 ) | CFG_Version ) )
	{
		buf[0] = ( 0xAC << 24 ) | CFG_Version;
		EEPROMProgram( buf , 0 , 4 );
		//��Ҫ�������3���ֵ�����У����Ϣ
		EEPROMProgram( &configurations[1] , 1*4 , (CfgLength - 1 - 3)*4 );
	}
	else
	{
		//�洢�汾����
		//��ȡ���д洢��Ϣ
		EEPROMRead( configurations , 0 , (CfgLength)*4 );
	}
}