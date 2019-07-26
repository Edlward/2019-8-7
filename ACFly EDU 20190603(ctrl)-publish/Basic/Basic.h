#pragma once

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define SYSTIMERCLK 80e+6

void init_Basic();

#define  INT_PRIO_0  0x00
#define  INT_PRIO_1  0x20
#define  INT_PRIO_2  0x40
#define  INT_PRIO_3  0x60
#define  INT_PRIO_4  0x80
#define  INT_PRIO_5  0xA0
#define  INT_PRIO_6  0xD0
#define  INT_PRIO_7  0xE0

#pragma region TIME

	extern const float TIM2sec_scale;

	//ʱ���Ϊ��s��λ
	typedef struct 
	{
		uint64_t t;
	}TIME;

	//��ȡ��ǰʱ��
	TIME get_TIME_now();
	
	//�������last_time��ʱ��
	float get_pass_time( TIME last_time );
	//�������time_a��time_b������ʱ��
	float get_time_difference( TIME time_a , TIME time_b );
	//����ϵͳ����ʱ��
	float get_System_Run_Time();
	
	//�������last_time��ʱ��
	//����last_time����Ϊ��ǰʱ��
	//���ھ�ȷ��ʱ
	float get_pass_time_st( TIME* last_time );
	
	//��ʱ������Ϊ������
	//��Ҫ�����ж�
	void Time_set_inValid( TIME* t );
	bool Time_isValid( TIME t );
	
	//��ʱ
	void delay( float t );

#pragma endpragma