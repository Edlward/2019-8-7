#pragma once

#include <stdint.h>
#include <stdbool.h>

//�˿ڶ���
typedef struct
{
	//д�˿ں���
	void (*write)( const uint8_t* data , uint16_t length );
	//���˿ں���
	uint16_t (*read)( uint8_t* data , uint16_t length );
	//��ȡ�˿ڴ����ֽ���������
	uint16_t (*DataAvailable)();
}Port;

//ע��˿�����Э��ͨ��
bool PortRegister( Port port );

//��ȡ�˿�
const Port* get_Port( uint8_t port );

/*����״̬*/
	//���Ͳ����б�
	void Send_Param_List();

	//���÷�����Ϣ
	//port_index:�˿����
	//Msg:��Ϣ���
	//Rate:����Ƶ��(hz) 0��ʾ������
	//�����Ƿ����óɹ�
	bool SetMsgRate( uint8_t port_index , uint16_t Msg , uint16_t Rate );
/*����״̬*/


void init_CommuLink();