#pragma once

void init_drv_USB();

//��˿�д������
void write_UsbUart( const uint8_t* data , uint16_t length );

//�����˿�����
//����ֵ��ʵ�ʶ����������ֽ���
uint16_t read_UsbUart( uint8_t* data , uint16_t length );

//���ض˿��ж����ֽڴ���������
uint16_t UsbUart_DataAvailable();