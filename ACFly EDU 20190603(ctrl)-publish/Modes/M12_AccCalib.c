#include "Modes.h"
#include "Basic.h"
#include "M12_AccCalib.h"
#include <stdlib.h>

#include "MeasurementSystem.h"
#include "InteractiveInterface.h"
#include "Sensors.h"
#include "AC_Math.h"
#include "Configurations.h"

//���ٶȼ�����У׼
//ʹ��������������򷽳�
//����Ҫ��ȫ��ƽ

//���Ľ� 20181226
//����������ҵ��;
//������Ϯ�ؾ�����

/*
	Ellipsoid equation :
		A( x - a )^2 + B( y - b )^2 + ( c - z )^2 - r = 0;

	es = [ a ]
			 [ b ]
			 [ c ]
			 [ A ]
			 [ B ]
			 [ r ]
	p = [ x1 y1 z1 ]
			[ x2 y2 z2 ]
			[ x3 y3 z3 ]
			[ x4 y4 z4 ]
			[ x5 y5 z5 ]
			[ x6 y6 z6 ]
*/

static void M12_AccCalib_MainFunc();
static void M12_AccCalib_enter();
static void M12_AccCalib_exit();
const Mode M12_AccCalib = 
{
	50 , //mode frequency
	M12_AccCalib_enter , //enter
	M12_AccCalib_exit ,	//exit
	M12_AccCalib_MainFunc ,	//mode main func
};

//����ÿ��ƽ�棨���ٶȵ㣩ƽ������
#define Calibration_Time 200
static const float inv_Calibration_Time = 1.0f / Calibration_Time;

typedef struct
{
	//�жϰ����Ƿ�ֹ
	vector3_float Calibration_Acc_Max , Calibration_Acc_Min;
	vector3_float Calibration_Gyro_Max , Calibration_Gyro_Min;
	//У׼������
	unsigned short Calibration_Counter;
	//��У׼�˵�ƽ������һ��6�棩
	unsigned char calibrated_surface_count;
	//��¼��У׼��ƽ��
	bool calibrated_surface[6];
	//У׼�ۼ���
	vector3_int Acc_Sum[IMU_Sensors_Count] , Gyro_Sum[IMU_Sensors_Count];
	vector3_int Gyro_PSum[IMU_Sensors_Count];
	
	//ţ�ٵ��� �ſɱȾ���
	float dF_matrix[ 6 * 6 ];
	float F_matrix[ 6 * 1 ];
	float dx_matrix[ 6 * 1 ];

	//��ǰУ׼�Ĵ�����
	unsigned char current_Calibrate_Sensor;
	//У׼���
	float es[ 6 * 1 ];
	//���ٶȵ�
	float p[ IMU_Sensors_Count ][ 6 * 3 ];
}MODE_INF;
static MODE_INF* Mode_Inf;

static inline void clear_Sums()
{
	for( unsigned char i = 0 ; i < IMU_Sensors_Count ; ++ i )
	{
		Mode_Inf->Acc_Sum[i].x = Mode_Inf->Acc_Sum[i].y = Mode_Inf->Acc_Sum[i].z = 0;
		Mode_Inf->Gyro_Sum[i].x = Mode_Inf->Gyro_Sum[i].y = Mode_Inf->Gyro_Sum[i].z = 0;
	}
}
static inline void clear_All_Sums()
{
	for( unsigned char i = 0 ; i < IMU_Sensors_Count ; ++ i )
	{
		Mode_Inf->Acc_Sum[i].x = Mode_Inf->Acc_Sum[i].y = Mode_Inf->Acc_Sum[i].z = 0;
		Mode_Inf->Gyro_Sum[i].x = Mode_Inf->Gyro_Sum[i].y = Mode_Inf->Gyro_Sum[i].z = 0;
		Mode_Inf->Gyro_PSum[i].x = Mode_Inf->Gyro_PSum[i].y = Mode_Inf->Gyro_PSum[i].z = 0;
	}
}

static void M12_AccCalib_enter()
{
	//����״̬��
	Led_setProgress(0);
	
	//��ʼ��ģʽ����
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Mode_Inf->Calibration_Acc_Max.x = Mode_Inf->Calibration_Acc_Max.y = Mode_Inf->Calibration_Acc_Max.z = 0;	
	Mode_Inf->Calibration_Acc_Min.x = Mode_Inf->Calibration_Acc_Min.y = Mode_Inf->Calibration_Acc_Min.z = 0;
	Mode_Inf->Calibration_Gyro_Max.x = Mode_Inf->Calibration_Gyro_Max.y = Mode_Inf->Calibration_Gyro_Max.z = 0;	
	Mode_Inf->Calibration_Gyro_Min.x = Mode_Inf->Calibration_Gyro_Min.y = Mode_Inf->Calibration_Gyro_Min.z = 0;
	clear_All_Sums();
	Mode_Inf->Calibration_Counter = 0;
	Mode_Inf->calibrated_surface_count = 0;
	Mode_Inf->calibrated_surface[0] = Mode_Inf->calibrated_surface[1] = Mode_Inf->calibrated_surface[2] = Mode_Inf->calibrated_surface[3] = Mode_Inf->calibrated_surface[4] = Mode_Inf->calibrated_surface[5] = false;
	Mode_Inf->current_Calibrate_Sensor = 0;
}

static void M12_AccCalib_exit()
{
	free( Mode_Inf );
}

static inline void get_F_dF( const float* es , const float* p );
static void M12_AccCalib_MainFunc()
{
	if( Mode_Inf->calibrated_surface_count < 6 )
	{
		
		const IMU_Sensor* acc0 = GetAccelerometer( 0 );
		const IMU_Sensor* gyro0 = GetGyroscope( 0 );
		
		vector3_int acc_data_raw = acc0->data_raw;
		vector3_int gyro_data_raw = gyro0->data_raw;
		
		vector3_float acc_f32 = { acc_data_raw.x ,acc_data_raw.y ,acc_data_raw.z };
		acc_f32 = vector3_float_mult( acc_f32 , acc0->sensitivity );
		
		vector3_float gyro_f32 = { gyro_data_raw.x ,gyro_data_raw.y ,gyro_data_raw.z };
		gyro_f32 = vector3_float_mult( gyro_f32 , gyro0->sensitivity );
		
		//�жϷɿ���������
		signed char Current_Surface = -1;
		if( acc_f32.x > 600.0f ) Current_Surface = 0;
		else if( acc_f32.x < -600.0f ) Current_Surface = 1;
		else if( acc_f32.y > 600.0f ) Current_Surface = 2;
		else if( acc_f32.y < -600.0f ) Current_Surface = 3;
		else if( acc_f32.z > 600.0f ) Current_Surface = 4;
		else if( acc_f32.z < -600.0f ) Current_Surface = 5;
		
		if( Current_Surface == -1 )
		{
			Led_setProgress(0);
			Mode_Inf->Calibration_Counter = 0;
			clear_Sums();
			return;
		}
			
		if( Mode_Inf->calibrated_surface[ Current_Surface ] == true )
		{
			//��ǰ����У׼
			Led_setProgress(0);
			Mode_Inf->Calibration_Counter = 0;
			clear_Sums();
		}
		else
		{
			vector3_float acc_filted = get_AccelerationFilted();
			vector3_float gyro_filted = get_AngularRateCtrl();
			if( Mode_Inf->Calibration_Counter == 0 )
			{
				Led_setProgress(0);
				Mode_Inf->Calibration_Acc_Max = Mode_Inf->Calibration_Acc_Min = acc_filted;
				Mode_Inf->Calibration_Gyro_Max = Mode_Inf->Calibration_Gyro_Min = gyro_filted;
				clear_Sums();
			}
			else if( Mode_Inf->Calibration_Counter <= Calibration_Time )
			{
				//ƽ���㻹û�ɼ����
				//�жϷɿؾ�ֹ�ɼ�������ƽ��
				
				//�жϷɿ��Ƿ�ֹ
				if( acc_filted.x > Mode_Inf->Calibration_Acc_Max.x ) Mode_Inf->Calibration_Acc_Max.x = acc_filted.x;
				else if( acc_filted.x < Mode_Inf->Calibration_Acc_Min.x ) Mode_Inf->Calibration_Acc_Min.x = acc_filted.x;
				if( acc_filted.y > Mode_Inf->Calibration_Acc_Max.y ) Mode_Inf->Calibration_Acc_Max.y = acc_filted.y;
				else if( acc_filted.y < Mode_Inf->Calibration_Acc_Min.y ) Mode_Inf->Calibration_Acc_Min.y = acc_filted.y;
				if( acc_filted.z > Mode_Inf->Calibration_Acc_Max.z ) Mode_Inf->Calibration_Acc_Max.z = acc_filted.z;
				else if( acc_filted.z < Mode_Inf->Calibration_Acc_Min.z ) Mode_Inf->Calibration_Acc_Min.z = acc_filted.z;
				
				if( gyro_filted.x > Mode_Inf->Calibration_Gyro_Max.x ) Mode_Inf->Calibration_Gyro_Max.x = gyro_filted.x;
				else if( gyro_filted.x < Mode_Inf->Calibration_Gyro_Min.x ) Mode_Inf->Calibration_Gyro_Min.x = gyro_filted.x;
				if( gyro_filted.y > Mode_Inf->Calibration_Gyro_Max.y ) Mode_Inf->Calibration_Gyro_Max.y = gyro_filted.y;
				else if( gyro_filted.y < Mode_Inf->Calibration_Gyro_Min.y ) Mode_Inf->Calibration_Gyro_Min.y = gyro_filted.y;
				if( gyro_filted.z > Mode_Inf->Calibration_Gyro_Max.z ) Mode_Inf->Calibration_Gyro_Max.z = gyro_filted.z;
				else if( gyro_filted.z < Mode_Inf->Calibration_Gyro_Min.z ) Mode_Inf->Calibration_Gyro_Min.z = gyro_filted.z;
				
				float acc_fluctuation_range;	float gyro_fluctuation_range;
				arm_sqrt_f32( vector3_float_square( vector3_float_subtract( Mode_Inf->Calibration_Acc_Max , Mode_Inf->Calibration_Acc_Min ) ), &acc_fluctuation_range );
				arm_sqrt_f32( vector3_float_square( vector3_float_subtract( Mode_Inf->Calibration_Gyro_Max , Mode_Inf->Calibration_Gyro_Min) ) , &gyro_fluctuation_range );
				if( ( acc_fluctuation_range > 10.0f ) || ( gyro_fluctuation_range > 0.03f ) )
				{
					//���Ӳ��Ǿ�ֹ��
					Led_setProgress(0);
					Mode_Inf->Calibration_Counter = 0;
					clear_Sums();
					return;
				}
				else
					Led_setProgress( Mode_Inf->Calibration_Counter * 100 / Calibration_Time );
				
				for( unsigned char i = 0 ; i < IMU_Sensors_Count ; ++ i )
				{
					const IMU_Sensor* sensor = GetAccelerometer(i);
					
					if( sensor->present )
						Mode_Inf->Acc_Sum[i] = vector3_int_plus( Mode_Inf->Acc_Sum[i] , sensor->data_raw );
					
					sensor = GetGyroscope(i);
					if( sensor->present )
						Mode_Inf->Gyro_Sum[i] = vector3_int_plus( Mode_Inf->Gyro_Sum[i] , sensor->data_raw );
				}
			}
			else
			{
				//��ǰ���Ѳɼ����
				Led_setSignal( LED_signal_continue );
				
				//��¼��
				for( unsigned char i = 0 ; i < IMU_Sensors_Count ; ++ i )
				{
					Mode_Inf->p[ i ][ Current_Surface * 3 ] = Mode_Inf->Acc_Sum[ i ].x * inv_Calibration_Time;
					Mode_Inf->p[ i ][ Current_Surface * 3 + 1 ] = Mode_Inf->Acc_Sum[ i ].y * inv_Calibration_Time;
					Mode_Inf->p[ i ][ Current_Surface * 3 + 2 ] = Mode_Inf->Acc_Sum[ i ].z * inv_Calibration_Time;
					
					Mode_Inf->Gyro_PSum[i] = vector3_int_plus( Mode_Inf->Gyro_PSum[i] , Mode_Inf->Gyro_Sum[i] );
				}
				++Mode_Inf->calibrated_surface_count;
				Mode_Inf->calibrated_surface[ Current_Surface ] = true;		
				
				Mode_Inf->Calibration_Counter = 0;
				clear_Sums();
				return;
			}
			++Mode_Inf->Calibration_Counter;
		}
		
	}	//if( calibrated_surface_count < 6 )
	else
	{
		//ȫ���㶼�ɼ����
		Led_setStatus( LED_status_running2 );
		if( GetAccelerometer( Mode_Inf->current_Calibrate_Sensor )->present == false )
		{
			++Mode_Inf->current_Calibrate_Sensor;
			if( Mode_Inf->current_Calibrate_Sensor >= IMU_Sensors_Count )
			{
				float Gyro_Sum_Scale = 0.16666666666666666666666666666667f * inv_Calibration_Time;
				for( unsigned char i = 0 ; i < IMU_Sensors_Count ; ++ i )
				{
					if( GetGyroscope(i)->present )
					{
						vector3_float GyroCalib = { \
							Mode_Inf->Gyro_PSum[i].x * Gyro_Sum_Scale , \
							Mode_Inf->Gyro_PSum[i].y * Gyro_Sum_Scale , \
							Mode_Inf->Gyro_PSum[i].z * Gyro_Sum_Scale };
						Cfg_update_GyroscopeOffset( i , GyroCalib );
					}
				}
				change_Mode( 01 );
				return;
			}
		}
		if( Mode_Inf->Calibration_Counter == 0 )
		{
			//��ʼ����������
			Mode_Inf->es[0] = Mode_Inf->es[1] = Mode_Inf->es[2] = 0;
			Mode_Inf->es[3] = Mode_Inf->es[4] = 1;	
			Mode_Inf->es[5] = constG / GetAccelerometer(Mode_Inf->current_Calibrate_Sensor)->sensitivity;
			Mode_Inf->es[5] *= Mode_Inf->es[5];
			++Mode_Inf->Calibration_Counter;
		}
		else
		{
			get_F_dF( Mode_Inf->es , Mode_Inf->p[Mode_Inf->current_Calibrate_Sensor] );
			
			bool status;
			status = Matrix_Inverse( Mode_Inf->dF_matrix , 6 );
			Mode_Inf->dx_matrix[0] = Mode_Inf->dF_matrix[0]*Mode_Inf->F_matrix[0] + Mode_Inf->dF_matrix[1]*Mode_Inf->F_matrix[1] + Mode_Inf->dF_matrix[2]*Mode_Inf->F_matrix[2] + Mode_Inf->dF_matrix[3]*Mode_Inf->F_matrix[3] + Mode_Inf->dF_matrix[4]*Mode_Inf->F_matrix[4] + Mode_Inf->dF_matrix[5]*Mode_Inf->F_matrix[5];
			Mode_Inf->dx_matrix[1] = Mode_Inf->dF_matrix[6]*Mode_Inf->F_matrix[0] + Mode_Inf->dF_matrix[7]*Mode_Inf->F_matrix[1] + Mode_Inf->dF_matrix[8]*Mode_Inf->F_matrix[2] + Mode_Inf->dF_matrix[9]*Mode_Inf->F_matrix[3] + Mode_Inf->dF_matrix[10]*Mode_Inf->F_matrix[4] + Mode_Inf->dF_matrix[11]*Mode_Inf->F_matrix[5];
			Mode_Inf->dx_matrix[2] = Mode_Inf->dF_matrix[12]*Mode_Inf->F_matrix[0] + Mode_Inf->dF_matrix[13]*Mode_Inf->F_matrix[1] + Mode_Inf->dF_matrix[14]*Mode_Inf->F_matrix[2] + Mode_Inf->dF_matrix[15]*Mode_Inf->F_matrix[3] + Mode_Inf->dF_matrix[16]*Mode_Inf->F_matrix[4] + Mode_Inf->dF_matrix[17]*Mode_Inf->F_matrix[5];
			Mode_Inf->dx_matrix[3] = Mode_Inf->dF_matrix[18]*Mode_Inf->F_matrix[0] + Mode_Inf->dF_matrix[19]*Mode_Inf->F_matrix[1] + Mode_Inf->dF_matrix[20]*Mode_Inf->F_matrix[2] + Mode_Inf->dF_matrix[21]*Mode_Inf->F_matrix[3] + Mode_Inf->dF_matrix[22]*Mode_Inf->F_matrix[4] + Mode_Inf->dF_matrix[23]*Mode_Inf->F_matrix[5];
			Mode_Inf->dx_matrix[4] = Mode_Inf->dF_matrix[24]*Mode_Inf->F_matrix[0] + Mode_Inf->dF_matrix[25]*Mode_Inf->F_matrix[1] + Mode_Inf->dF_matrix[26]*Mode_Inf->F_matrix[2] + Mode_Inf->dF_matrix[27]*Mode_Inf->F_matrix[3] + Mode_Inf->dF_matrix[28]*Mode_Inf->F_matrix[4] + Mode_Inf->dF_matrix[29]*Mode_Inf->F_matrix[5];
			Mode_Inf->dx_matrix[5] = Mode_Inf->dF_matrix[30]*Mode_Inf->F_matrix[0] + Mode_Inf->dF_matrix[31]*Mode_Inf->F_matrix[1] + Mode_Inf->dF_matrix[32]*Mode_Inf->F_matrix[2] + Mode_Inf->dF_matrix[33]*Mode_Inf->F_matrix[3] + Mode_Inf->dF_matrix[34]*Mode_Inf->F_matrix[4] + Mode_Inf->dF_matrix[35]*Mode_Inf->F_matrix[5];
						
			Mode_Inf->es[0] -= Mode_Inf->dx_matrix[0];
			Mode_Inf->es[1] -= Mode_Inf->dx_matrix[1];
			Mode_Inf->es[2] -= Mode_Inf->dx_matrix[2];
			Mode_Inf->es[3] -= Mode_Inf->dx_matrix[3];
			Mode_Inf->es[4] -= Mode_Inf->dx_matrix[4];
			Mode_Inf->es[5] -= Mode_Inf->dx_matrix[5];
			
			if( Mode_Inf->Calibration_Counter > 200 && \
				( Mode_Inf->dx_matrix[0]*Mode_Inf->dx_matrix[0]\
				+ Mode_Inf->dx_matrix[1]*Mode_Inf->dx_matrix[1]\
				+ Mode_Inf->dx_matrix[2]*Mode_Inf->dx_matrix[2]\
				+ Mode_Inf->dx_matrix[3]*Mode_Inf->dx_matrix[3]\
				+ Mode_Inf->dx_matrix[4]*Mode_Inf->dx_matrix[4] ) < 0.1f )
			{
				Led_setSignal( LED_signal_success );
				
				arm_sqrt_f32( Mode_Inf->es[5] / Mode_Inf->es[3] , &Mode_Inf->es[3] );
				arm_sqrt_f32( Mode_Inf->es[5] / Mode_Inf->es[4] , &Mode_Inf->es[4] );
				arm_sqrt_f32( Mode_Inf->es[5] , &Mode_Inf->es[5] );
				float Gravity_DP = constG / GetAccelerometer(Mode_Inf->current_Calibrate_Sensor)->sensitivity;
				vector3_float acc_calib;
				acc_calib.x = Mode_Inf->es[0];	acc_calib.y = Mode_Inf->es[1];	acc_calib.z = Mode_Inf->es[2];
				Cfg_update_AccelerometerOffset( Mode_Inf->current_Calibrate_Sensor , acc_calib );
				acc_calib.x = Gravity_DP / Mode_Inf->es[3];	acc_calib.y = Gravity_DP / Mode_Inf->es[4];	acc_calib.z = Gravity_DP / Mode_Inf->es[5];
				Cfg_update_AccelerometerSensivitity( Mode_Inf->current_Calibrate_Sensor , acc_calib );
				
				//����У׼��һ��������
				++Mode_Inf->current_Calibrate_Sensor;
				if( Mode_Inf->current_Calibrate_Sensor < IMU_Sensors_Count )
				{				
					Mode_Inf->Calibration_Counter = 0;
					return;
				}				
				
				float Gyro_Sum_Scale = 0.16666666666666666666666666666667f * inv_Calibration_Time;
				for( unsigned char i = 0 ; i < IMU_Sensors_Count ; ++ i )
				{
					if( GetGyroscope(i)->present )
					{
						vector3_float GyroCalib = { \
							Mode_Inf->Gyro_PSum[i].x * Gyro_Sum_Scale , \
							Mode_Inf->Gyro_PSum[i].y * Gyro_Sum_Scale , \
							Mode_Inf->Gyro_PSum[i].z * Gyro_Sum_Scale };
						Cfg_update_GyroscopeOffset( i , GyroCalib );
					}
				}
				change_Mode( 01 );
				return;
			}	
			++Mode_Inf->Calibration_Counter;
		}
	}
}

/*
	Ellipsoid equation :
		A( x - a )^2 + B( y - b )^2 + ( c - z )^2 - r = 0;

	es = [ a ]
			 [ b ]
			 [ c ]
			 [ A ]
			 [ B ]
			 [ r ]
	p = [ x1 y1 z1 ]
			[ x2 y2 z2 ]
			[ x3 y3 z3 ]
			[ x4 y4 z4 ]
			[ x5 y5 z5 ]
			[ x6 y6 z6 ]
*/

//get dF from matrix es[6,1] and matrix p[6,6]
static inline void get_F_dF( const float* es , const float* p )
{
	float a_x1 = es[0] - p[0];
	float a_x2 = es[0] - p[3];
	float a_x3 = es[0] - p[6];
	float a_x4 = es[0] - p[9];
	float a_x5 = es[0] - p[12];
	float a_x6 = es[0] - p[15];
	
	float b_y1 = es[1] - p[1];
	float b_y2 = es[1] - p[4];
	float b_y3 = es[1] - p[7];
	float b_y4 = es[1] - p[10];
	float b_y5 = es[1] - p[13];
	float b_y6 = es[1] - p[16];
	
	float c_z1 = es[2] - p[2];
	float c_z2 = es[2] - p[5];
	float c_z3 = es[2] - p[8];
	float c_z4 = es[2] - p[11];
	float c_z5 = es[2] - p[14];
	float c_z6 = es[2] - p[17];
	
	Mode_Inf->F_matrix[0] = es[3]*a_x1*a_x1 + es[4]*b_y1*b_y1 + c_z1*c_z1 - es[5];
	Mode_Inf->F_matrix[1] = es[3]*a_x2*a_x2 + es[4]*b_y2*b_y2 + c_z2*c_z2 - es[5];
	Mode_Inf->F_matrix[2] = es[3]*a_x3*a_x3 + es[4]*b_y3*b_y3 + c_z3*c_z3 - es[5];
	Mode_Inf->F_matrix[3] = es[3]*a_x4*a_x4 + es[4]*b_y4*b_y4 + c_z4*c_z4 - es[5];
	Mode_Inf->F_matrix[4] = es[3]*a_x5*a_x5 + es[4]*b_y5*b_y5 + c_z5*c_z5 - es[5];
	Mode_Inf->F_matrix[5] = es[3]*a_x6*a_x6 + es[4]*b_y6*b_y6 + c_z6*c_z6 - es[5];
	
	float A2  = es[3] * 2.0f;
	float B2  = es[4] * 2.0f;
	Mode_Inf->dF_matrix[00] = A2 * a_x1;	Mode_Inf->dF_matrix[01] = B2 * b_y1;	Mode_Inf->dF_matrix[02] = 2 * c_z1;	Mode_Inf->dF_matrix[03] = a_x1 * a_x1;	Mode_Inf->dF_matrix[04] = b_y1 * b_y1;	Mode_Inf->dF_matrix[05] = -1;
	Mode_Inf->dF_matrix[06] = A2 * a_x2;	Mode_Inf->dF_matrix[07] = B2 * b_y2;	Mode_Inf->dF_matrix[8] = 2 * c_z2;	Mode_Inf->dF_matrix[9] = a_x2 * a_x2;	Mode_Inf->dF_matrix[10] = b_y2 * b_y2;	Mode_Inf->dF_matrix[11] = -1;
	Mode_Inf->dF_matrix[12] = A2 * a_x3;	Mode_Inf->dF_matrix[13] = B2 * b_y3;	Mode_Inf->dF_matrix[14] = 2 * c_z3;	Mode_Inf->dF_matrix[15] = a_x3 * a_x3;	Mode_Inf->dF_matrix[16] = b_y3 * b_y3;	Mode_Inf->dF_matrix[17] = -1;
	Mode_Inf->dF_matrix[18] = A2 * a_x4;	Mode_Inf->dF_matrix[19] = B2 * b_y4;	Mode_Inf->dF_matrix[20] = 2 * c_z4;	Mode_Inf->dF_matrix[21] = a_x4 * a_x4;	Mode_Inf->dF_matrix[22] = b_y4 * b_y4;	Mode_Inf->dF_matrix[23] = -1;
	Mode_Inf->dF_matrix[24] = A2 * a_x5;	Mode_Inf->dF_matrix[25] = B2 * b_y5;	Mode_Inf->dF_matrix[26] = 2 * c_z5;	Mode_Inf->dF_matrix[27] = a_x5 * a_x5;	Mode_Inf->dF_matrix[28] = b_y5 * b_y5;	Mode_Inf->dF_matrix[29] = -1;
	Mode_Inf->dF_matrix[30] = A2 * a_x6;	Mode_Inf->dF_matrix[31] = B2 * b_y6;	Mode_Inf->dF_matrix[32] = 2 * c_z6;	Mode_Inf->dF_matrix[33] = a_x6 * a_x6;	Mode_Inf->dF_matrix[34] = b_y6 * b_y6;	Mode_Inf->dF_matrix[35] = -1;
}
