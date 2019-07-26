#pragma once

#include "Basic.h"
#include <stdbool.h>

//���ջ�����
typedef struct
{
	bool present;	//�Ƿ����
	bool connected;	//�Ƿ�������
	bool available;	//�Ƿ����
	TIME last_update_time;	//�ϴθ���ʱ��
	float update_time;	//����ʱ����
	
	float raw_data[16];	//ԭʼ����
	float data[8];	//У׼�������
}Receiver;

#define Receivers_Count 3
typedef enum
{
	RC_Type_Sbus = 0 ,
	RC_Type_PPM = 1 ,
}RC_Type;

//��ȡָ���Ľ��ջ�
const Receiver* get_Receiver( RC_Type rc );
//��ȡ��ǰʹ�õĽ��ջ�
const Receiver* get_current_Receiver();
//��ȡ��ǰʹ�õĽ��ջ�
RC_Type get_current_Receiver_Type();