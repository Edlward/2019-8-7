#pragma once

#include "Receiver.h"

//��ȡ���ջ�
Receiver* get_Receiver_NC( RC_Type rc );

//���½��ջ�����
void Receiver_Update( RC_Type _rc , bool connected , float raw_data[16] , uint8_t channel_count );