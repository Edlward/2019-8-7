#include "Modes.h"
#include "Basic.h"
#include "M10_RCCalib.h"
#include <stdlib.h>

#include "InteractiveInterface.h"
#include "Receiver.h"
#include "Configurations.h"
#include "AC_Math.h"

//���ջ�У׼
//��У׼������ң��ͨ��˳��
//ӳ�䵽��
//ͨ��1������
//ͨ��2��ƫ��
//ͨ��3������
//ͨ��4�����
//ͨ��5678������

static void M10_RCCalib_MainFunc();
static void M10_RCCalib_enter();
static void M10_RCCalib_exit();
const Mode M10_RCCalib = 
{
	50 , //mode frequency
	M10_RCCalib_enter , //enter
	M10_RCCalib_exit ,	//exit
	M10_RCCalib_MainFunc ,	//mode main func
};

typedef struct
{
	//ҪУ׼�Ľ��ջ�
	RC_Type Calib_RC;
	
	bool Calibration_Started;
	unsigned char Calibration_Stage;
	unsigned char Calibration_Stage2;
	bool RC_Channel_Calibrated[ 16 ];
	float RC_Calibration_Point[ 3 ][ 16 ];
	uint8_t RC_Calibration_Reflection[8];
	
	float max_rc[16] , min_rc[16] , avg_rc[16];
}MODE_INF;
static MODE_INF* Mode_Inf;

static void M10_RCCalib_enter()
{
	//����״̬��
	Led_setStatus( LED_status_waiting );
	
	//��ʼ��ģʽ����
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Mode_Inf->Calibration_Started = false;
	Mode_Inf->Calib_RC = get_current_Receiver_Type();
	const Receiver* rc = get_Receiver( Mode_Inf->Calib_RC );
	if( rc->connected == false )
	{
		//�޽��ջ�����
		Led_setSignal( LED_signal_error );
		change_Mode( 1 );
		return;
	}
	
	for( unsigned char i = 0 ; i < 16 ; ++i )
	{
		Mode_Inf->max_rc[i] = Mode_Inf->min_rc[i] = Mode_Inf->avg_rc[i] = rc->raw_data[i];
		Mode_Inf->RC_Channel_Calibrated[i] = false;
	}
	Mode_Inf->Calibration_Stage = Mode_Inf->Calibration_Stage2 = 0;
}

static void M10_RCCalib_exit()
{
	free( Mode_Inf );
}

static void M10_RCCalib_MainFunc()
{
	const Receiver* rc = get_Receiver( Mode_Inf->Calib_RC );
	if( rc->connected == false )
	{
		//���ջ��Ͽ�
		Led_setSignal( LED_signal_error );
		change_Mode( 1 );
		return;
	}
	
	#define avg_time 50	
	#define reset_constant_check for( unsigned char j = 0 ; j < 16 ; ++j )\
														Mode_Inf->max_rc[j] = Mode_Inf->min_rc[j] = Mode_Inf->avg_rc[j] = rc->raw_data[j];\
												 Mode_Inf->Calibration_Stage2 = 1
	//���ͨ���Ƿ�ά�ֲ���
	for( unsigned char i = 0 ; i < 16 ; ++i )
	{
		if( rc->raw_data[i] > Mode_Inf->max_rc[i] )
			Mode_Inf->max_rc[i] = rc->raw_data[i];
		else if( rc->raw_data[i] < Mode_Inf->min_rc[i] )
			Mode_Inf->min_rc[i] = rc->raw_data[i];
		
		if( Mode_Inf->max_rc[i] - Mode_Inf->min_rc[i] > 1.0f )
		{
			//ͨ���ڱ仯
			//ͨ����һ�α仯��ʼУ׼
			Mode_Inf->Calibration_Started = true;
			Led_setProgress( 0 );
			reset_constant_check;
			return;
		}	
		Mode_Inf->avg_rc[i] += rc->raw_data[i];
	}
	++Mode_Inf->Calibration_Stage2;
	
	if( Mode_Inf->Calibration_Started == false )
	{
		for( unsigned char j = 0 ; j < 16 ; ++j )
		{
			Mode_Inf->avg_rc[j] = rc->raw_data[j];
			Mode_Inf->Calibration_Stage2 = 1;
		}
		Led_setStatus( LED_status_error );
		return;
	}
	
	switch( Mode_Inf->Calibration_Stage )
	{
		
		case 0:	//��¼�е�
		{		
			Led_setProgress( Mode_Inf->Calibration_Stage2 * 100 / avg_time );
				
			if( Mode_Inf->Calibration_Stage2 >= avg_time )
			{
				Led_setSignal( LED_signal_continue );
				Mode_Inf->Calibration_Stage = 1;
				Mode_Inf->Calibration_Stage2 = 0;
				for( unsigned char i = 0 ; i < 16 ; ++i )
					Mode_Inf->RC_Calibration_Point[ 1 ][ i ] = Mode_Inf->avg_rc[i] * ( 1.0f / avg_time );
				for( unsigned char i = 0 ; i < 8 ; ++i )
					Mode_Inf->RC_Calibration_Reflection[i] = 255;
				reset_constant_check;
			}
				
			break;
		}	//case 0
		
		case 1:	//��¼һ����ͨ��Сֵ
		case 2:
		case 3:
		case 4:
		{
			Led_setProgress( Mode_Inf->Calibration_Stage2 * 100 / avg_time );
			if( Mode_Inf->Calibration_Stage2 >= avg_time )
			{
				//�Ҹı��˵�ͨ��
				unsigned char channel_change_count = 0;
				unsigned char channel_index;
				for( unsigned char i = 0 ; i < 16 ; ++i )
				{
					if( fabsf( Mode_Inf->avg_rc[i] * ( 1.0f / avg_time ) - Mode_Inf->RC_Calibration_Point[ 1 ][ i ] ) > 15.0f )
					{
						channel_index = i;
						++channel_change_count;
					}
				}
				
				//����ı��ͨ����ֹһ��
				//���߸�ͨ���Ѿ�У׼
				//���¿�ʼ
				if( (channel_change_count != 1) || (Mode_Inf->RC_Channel_Calibrated[channel_index] == true) )
				{
					reset_constant_check;
					return;
				}
				else
				{
					Mode_Inf->RC_Calibration_Point[ 0 ][ channel_index ] = Mode_Inf->avg_rc[channel_index] * ( 1.0f / avg_time );
					Mode_Inf->RC_Calibration_Reflection[ Mode_Inf->Calibration_Stage - 1 ] = channel_index;
					Mode_Inf->RC_Channel_Calibrated[channel_index] = true;
					
					Led_setSignal( LED_signal_continue );
					reset_constant_check;
					++Mode_Inf->Calibration_Stage;
				}
			}
			break;			
		}	//case 1 2 3 4
		
		case 5:	//��¼һ����ͨ���ֵ
		case 6:
		case 7:
		case 8:
		{
			Led_setProgress( Mode_Inf->Calibration_Stage2 * 100 / avg_time );
			if( Mode_Inf->Calibration_Stage2 >= avg_time )
			{
				//Ѱ�Ҹı��ͨ��
				unsigned char channel_change_count = 0;
				unsigned char channel_index;
				for( unsigned char i = 0 ; i < 16 ; ++i )
				{
					if( fabsf( Mode_Inf->avg_rc[i] * ( 1.0f / avg_time ) - Mode_Inf->RC_Calibration_Point[ 1 ][ i ] ) > 15.0f )
					{
						channel_index = i;
						++channel_change_count;
					}
				}
				
				//����ı��ͨ����ֹһ��
				//���߸�ͨ���Ѿ�У׼
				//���¿�ʼ
				if( channel_change_count != 1 )
				{
					reset_constant_check;
					return;
				}
				else
				{
					//���ͨ����У׼��Сֵʱ�Ĳ�һ��
					//���¿�ʼ
					if( channel_index != Mode_Inf->RC_Calibration_Reflection[ Mode_Inf->Calibration_Stage - 5 ] )
					{
						reset_constant_check;
						return;
					}
					
					Mode_Inf->RC_Calibration_Point[ 2 ][ channel_index ] = Mode_Inf->avg_rc[channel_index] * ( 1.0f / avg_time );
					
					Led_setSignal( LED_signal_continue );
					reset_constant_check;
					++Mode_Inf->Calibration_Stage;
				}
			}
			break;			
		}	//case 5 6 7 8
		
		case 9:	//button1
		case 10:	//button2
		{
			Led_setProgress( Mode_Inf->Calibration_Stage2 * 100 / avg_time );
			if( Mode_Inf->Calibration_Stage2 >= avg_time )
			{
				//Ѱ�Ҹı��ͨ��
				unsigned char channel_change_count = 0;
				unsigned char channel_index;
				for( unsigned char i = 0 ; i < 16 ; ++i )
				{
					if( fabsf( Mode_Inf->avg_rc[i] * ( 1.0f / avg_time ) - Mode_Inf->RC_Calibration_Point[ 1 ][ i ] ) > 15.0f )
					{
						channel_index = i;
						++channel_change_count;
					}
				}
				
				//����ı��ͨ����ֹһ��
				//���߸�ͨ���Ѿ�У׼
				//���¿�ʼ
				if( (channel_change_count != 1) || (Mode_Inf->RC_Channel_Calibrated[channel_index] == true) )
				{
					reset_constant_check;
					return;
				}
				
				Mode_Inf->RC_Calibration_Reflection[ Mode_Inf->Calibration_Stage - 9 + 4 ] = channel_index;
				Mode_Inf->RC_Calibration_Point[ 0 ][channel_index] = Mode_Inf->avg_rc[channel_index] * ( 1.0f / avg_time );
				Mode_Inf->RC_Channel_Calibrated[channel_index] = true;
				
				Led_setSignal( LED_signal_continue );
				reset_constant_check;
				++Mode_Inf->Calibration_Stage;
			}
			break;
		}
		
		case 11:	//aux3
		case 12:	//aux4
		{
			Led_setProgress( Mode_Inf->Calibration_Stage2 * 100 / avg_time );
			if( Mode_Inf->Calibration_Stage2 >= avg_time )
			{
				//Ѱ�Ҹı��ͨ��
				unsigned char channel_change_count = 0;
				unsigned char channel_index;
				for( unsigned char i = 0 ; i < 16 ; ++i )
				{
					if( fabsf( Mode_Inf->avg_rc[i] * ( 1.0f / avg_time ) - Mode_Inf->RC_Calibration_Point[ 1 ][ i ] ) > 15.0f )
					{
						channel_index = i;
						++channel_change_count;
					}
				}
				
				//����ı��ͨ����ֹһ��
				//���¿�ʼ
				if( (channel_change_count != 1) )
				{
					reset_constant_check;
					return;
				}
				
				//�ж������������У׼
				if( channel_index == Mode_Inf->RC_Calibration_Reflection[ 0 ] )
				{
					bool thr_low;
					thr_low = fabsf( Mode_Inf->avg_rc[channel_index] * ( 1.0f / avg_time ) -  Mode_Inf->RC_Calibration_Point[ 0 ][ channel_index ] ) < 5.0f;
					
					if( thr_low )
					{
						Led_setSignal( LED_signal_continue );
						reset_constant_check;
						Mode_Inf->Calibration_Stage = 13;
						break;
					}
				}
				
				//�����ͨ���Ѿ�У׼
				//���¿�ʼ
				if( (Mode_Inf->RC_Channel_Calibrated[channel_index] == true) )
				{
					reset_constant_check;
					return;
				}
				
				Mode_Inf->RC_Calibration_Reflection[ Mode_Inf->Calibration_Stage - 9 + 4 ] = channel_index;
				Mode_Inf->RC_Calibration_Point[ 0 ][channel_index] = Mode_Inf->avg_rc[channel_index] * ( 1.0f / avg_time );
				Mode_Inf->RC_Channel_Calibrated[channel_index] = true;
				
				Led_setSignal( LED_signal_continue );
				reset_constant_check;
				++Mode_Inf->Calibration_Stage;
			}
			break;
		}
		
		case 13:	//����У׼���
		{
			float min_RCs[8];
			float RC_scales[8];
			for( unsigned char i = 0 ; i < 4 ; ++i )
			{
				unsigned char reflection_ch = Mode_Inf->RC_Calibration_Reflection[ i ];

				//failed if mid point not correct
				float m_side = Mode_Inf->RC_Calibration_Point[ 2 ][ reflection_ch ] - Mode_Inf->RC_Calibration_Point[ 1 ][ reflection_ch ];
				float l_side = Mode_Inf->RC_Calibration_Point[ 1 ][ reflection_ch ] - Mode_Inf->RC_Calibration_Point[ 0 ][ reflection_ch ];
				if( fabsf( m_side - l_side ) > 10.0f )
				{
					Led_setSignal( LED_signal_error );
					change_Mode( 01 );
					return;
				}
				float side = ( fabsf(m_side) > fabsf(l_side) ) ? m_side : l_side;
				min_RCs[i] = Mode_Inf->RC_Calibration_Point[ 1 ][ reflection_ch ] - side;
				RC_scales[i] = 50.0f / side;
			}
			
			for( unsigned char i = 4 ; i < 8 ; ++i )
			{
				if( Mode_Inf->RC_Calibration_Reflection[ i ] < 16 )
				{
					unsigned char reflection_ch = Mode_Inf->RC_Calibration_Reflection[ i ];
					RC_scales[i] = 100.0f / ( Mode_Inf->RC_Calibration_Point[ 1 ][ reflection_ch ] - Mode_Inf->RC_Calibration_Point[ 0 ][ reflection_ch ] );
					min_RCs[i] = Mode_Inf->RC_Calibration_Point[ 0 ][ reflection_ch ];
				}
			}
			
			Led_setSignal( LED_signal_success );			
			Cfg_update_Channel_Reflection( Mode_Inf->Calib_RC , Mode_Inf->RC_Calibration_Reflection );
			Cfg_update_Channel_Min( Mode_Inf->Calib_RC , min_RCs );
			Cfg_update_Channel_Scale( Mode_Inf->Calib_RC , RC_scales );

			change_Mode( 01 );
			return;
			
			break;
		}
		
	}
}