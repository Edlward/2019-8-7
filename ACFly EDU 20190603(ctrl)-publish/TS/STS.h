#pragma once

#include "Basic.h"

//Simple Task Scheduling
//�������������
//���Ľ� 20181224

//����������
#define STS_MAX_TASK_COUNT 20

typedef enum
{
	STS_Task_Trigger_Mode_RoughTime ,	//����ʱ�䴥������
	STS_Task_Trigger_Mode_PreciseTime ,	//��ȷʱ�䴥������
	
	STS_Task_Trigger_Mode_Custom ,	//�Զ��庯����������
}STS_Task_Trigger_Mode;

//�������������������ID
//������ģʽΪʱ�䴥����tΪ����ʱ����
//������ģʽΪ�Զ��庯��������trigger_funcΪ�Զ��庯��
//mainfuncΪ����������
unsigned int STS_Add_Task( STS_Task_Trigger_Mode mode , float t , bool (*trigger_func)( unsigned int Task_ID ) , void (*mainfunc)( unsigned int Task_ID ) );

//�������񴥷�ģʽ
bool STS_Change_Task_Mode( unsigned int Task_ID , STS_Task_Trigger_Mode mode , float t , bool (*trigger_func)( unsigned int Task_ID ) );

//��������������
bool STS_Change_Task_MainFunc( unsigned int Task_ID , void (*mainfunc)( unsigned int Task_ID ) );

//ɾ������
bool STS_Remove_Task( unsigned int Task_ID );

//�ڱ�����
bool STS_Shield_Task( unsigned int Task_ID );

//�����ڱε�����
bool STS_Manifest_Task( unsigned int Task_ID );

void STS_Run();