#include "CommuLink.h"
#include "mavlink.h"
#include "MavlinkRCProcess.h"
#include "MavlinkSendFuncs.h"

#include "AC_Math.h"
#include "STS.h"
#include "Configurations.h"
#include "Modes.h"
#include "MeasurementSystem.h"
#include "eeprom.h"

//�˿�
static Port CommuPorts[ MAVLINK_COMM_NUM_BUFFERS ] = {0};

/*����״̬*/
	//���Ͳ����б�
	static int16_t Params_send = -1;
	void Send_Param_List()
	{
		Params_send = Params_Count - 1;
	}
	
	//������Ϣ�б�
	typedef struct
	{
		uint16_t Msg;
		uint8_t Rate;
		uint8_t Counter;
	}SendMsgInf;
	#define MAX_SEND_MSGS 10
	static SendMsgInf SendMsgs[ MAVLINK_COMM_NUM_BUFFERS ][ MAX_SEND_MSGS ] = {0};
	//���÷�����Ϣ
	//port_index:�˿����
	//Msg:��Ϣ���
	//Rate:����Ƶ��(hz) 0��ʾ������
	//�����Ƿ����óɹ�
	bool SetMsgRate( uint8_t port_index , uint16_t Msg , uint16_t Rate )
	{
		if( Msg >= Mavlink_Send_Funcs_Count )
			return false;
		if( Mavlink_Send_Funcs[ Msg ] == 0 )
			return false;
		
		int16_t exist_msg = -1;
		for( uint16_t i = 0 ; i < MAX_SEND_MSGS ; ++i )
		{
			if( SendMsgs[ port_index ][ i ].Msg == Msg )
			{
				exist_msg = i;
				break;
			}
		}
		
		uint8_t Rate10ms = 0;
		if( Rate > 0 )
		{
			Rate10ms = 100.0f / (float)Rate;
			if( Rate10ms < 1 )
				Rate10ms = 1;
		}
		
		if( exist_msg >= 0 )
		{	//�Ѵ��ڸ���Ϣ
			if( Rate > 0 )
				//������ϢƵ��
				SendMsgs[ port_index ][ exist_msg ].Rate = Rate10ms;
			else
				//ȡ����Ϣ����
				SendMsgs[ port_index ][ exist_msg ].Msg = 0;
		}
		else if( Rate > 0 )
		{	//����Ϣ�����ڣ���Ҫ�½���
			//�ҿ�λ
			for( uint16_t i = 0 ; i < MAX_SEND_MSGS ; ++i )
			{
				if( SendMsgs[ port_index ][ i ].Msg == 0 )
				{
					exist_msg = i;
					break;
				}
			}
			
			//û�п�λʧ��
			if( exist_msg < 0 )
				return false;
			
			SendMsgs[ port_index ][ exist_msg ].Msg = Msg;
			SendMsgs[ port_index ][ exist_msg ].Rate = Rate10ms;
		}
		
		return true;
	}
/*����״̬*/

//ע��˿�����Э��ͨ��
bool PortRegister( Port port )
{
	if( port.read == 0 && port.write == 0 )
		return false;
	if( (port.read == 0) ^ (port.DataAvailable == 0) )
		return false;
	
	//Ѱ�ҿ��õ�λ��
	int8_t p_index = -1;
	for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
	{
		if( CommuPorts[ i ].read == 0 && CommuPorts[ i ].write == 0 )
		{
			p_index = i;
			break;
		}
	}
	//������
	if( p_index < 0 )
		return false;
	
	CommuPorts[ p_index ] = port;
	mavlink_set_proto_version( p_index , 1 );
	SetMsgRate( p_index , MAVLINK_MSG_ID_ATTITUDE_QUATERNION , 50 );
	SetMsgRate( p_index , MAVLINK_MSG_ID_LOCAL_POSITION_NED , 50 );
	return true;
}

//��ȡ�˿�
const Port* get_Port( uint8_t port )
{
	return &CommuPorts[port];
}

//�˿ڽ��շ��񴥷��жϺ���
static bool CommuLink_RCTrigger( unsigned int Task_ID )
{
	for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
	{
		if( CommuPorts[i].DataAvailable != 0 )
		{
			if( CommuPorts[i].DataAvailable() )
				return true;
		}
	}
	return false;
}
//�˿ڽ��շ�����
static void CommuLink_RCServer( unsigned int Task_ID )
{
	mavlink_message_t msg;
	for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
	{
		if( CommuPorts[i].read != 0 )
		{
			//ÿ�ν���10���ֽڽ��д���
			uint8_t buf[10];
			uint8_t length = CommuPorts[i].read( buf , 10 );
			
			bool parsed = false;
			for( uint8_t k = 0 ; k < length ; ++k )
			{
				//��Ϣ���
				if( mavlink_parse_char( i , buf[k] , &msg , NULL ) )
				{
					//��Ϣ������
					parsed = true;
					
					//�����Ϣ����������
					//������Ϣ
					if( msg.msgid < Mavlink_RC_Process_Count )
					{
						if( Mavlink_RC_Process[ msg.msgid ] != 0 )
							Mavlink_RC_Process[ msg.msgid ]( i , &msg );
					}
				}
			}
			//����Ѿ�������һ����Ϣ
			//�˳�������һ���ٴ��������˿�
			if( parsed )
				return;
		}
	}
}
//�˿ڷ��ͷ�����
static void CommuLink_SendServer( unsigned int Task_ID )
{
	static uint16_t HeartBeat_counter = 0;
	mavlink_message_t msg_sd;
	
	//����������
	if( ++HeartBeat_counter >= 100 )
	{
		HeartBeat_counter = 0;
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{
			if( CommuPorts[i].write != 0 )
			{
				mavlink_msg_heartbeat_pack_chan( 
					1 ,	//system id
					MAV_COMP_ID_AUTOPILOT1 ,	//component id
					i	,	//chan
					&msg_sd,
					MAV_TYPE_QUADROTOR ,	//type
					MAV_AUTOPILOT_GENERIC ,	//autopilot
					MAV_MODE_PREFLIGHT ,	//base mode
					0xffff ,	//custom mode
					MAV_STATE_STANDBY	//sys status
				);
				mavlink_msg_to_send_buffer(CommuPorts[i].write, &msg_sd);
			}
		}
	}
	
	//������Ϣ��
	for( uint8_t port_id = 0 ; port_id < MAVLINK_COMM_NUM_BUFFERS ; ++port_id )
	{
		if( CommuPorts[port_id].write != 0 )
		{
			for( uint8_t i = 0 ; i < MAX_SEND_MSGS ; ++i )
			{
				if( SendMsgs[ port_id ][ i ].Msg != 0 )
				{
					if( ++SendMsgs[ port_id ][ i ].Counter >= SendMsgs[ port_id ][ i ].Rate )
					{
						SendMsgs[ port_id ][ i ].Counter = 0;
						
						if( Mavlink_Send_Funcs[ SendMsgs[ port_id ][ i ].Msg ]( port_id , &msg_sd ) )
							mavlink_msg_to_send_buffer(CommuPorts[port_id].write, &msg_sd);
					}
				}
			}
		}
	}
	
	//���Ͳ����б�
	if( Params_send >= 0 )
	{
	
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{
			const Port* port = &CommuPorts[ i ];
			if( port->write != 0 )
			{	
				mavlink_msg_param_value_pack_chan( 
					1 ,	//system id
					MAV_COMP_ID_AUTOPILOT1 ,	//component id
					i ,	//chan
					&msg_sd,
					Params[ Params_send ].name,	//param id
					Params[ Params_send ].get_param() ,	//param value
					Params[ Params_send ].type ,	//param type
					Params_Count ,	//param count
					Params_send	//param index
				);
				mavlink_msg_to_send_buffer(port->write, &msg_sd);
				
			}
		}
		--Params_send;
		
	}
}

void init_CommuLink()
{
	STS_Add_Task( STS_Task_Trigger_Mode_Custom , 0 , CommuLink_RCTrigger , CommuLink_RCServer );
	STS_Add_Task( STS_Task_Trigger_Mode_RoughTime , 0.01f , 0 , CommuLink_SendServer );
}