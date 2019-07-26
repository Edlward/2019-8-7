#pragma once

#include <stdbool.h>

typedef struct
{
	float mode_frequency;	//ģʽ����Ƶ��
	void (*mode_enter)();	//����ģʽ��������
	void (*mode_exit)();	//�뿪ģʽ��������
	void (*mode_main_func)();	//ģʽ������������Ƶ��ִ�У�
}Mode;

void init_Modes();

bool change_Mode( unsigned char mode );
unsigned char get_current_Mode();

#define Is_Calibration_Mode(m) ( m>=10 && m<=19 )
#define Is_Flight_Mode(m) ( m>=30 && m<=39 )