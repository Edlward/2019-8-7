#include "STS.h"

typedef struct
{
	STS_Task_Trigger_Mode mode;	//���񴥷�ģʽ
	
	TIME last_trigger_time;	//�ϴδ�������ʱ��
	float t;
	bool (*trigger_func)(  unsigned int Task_ID );	//�Զ��庯������ ����ָ��
	
	bool shielded;	//�����Ƿ�����
	
	void (*mainfunc)( unsigned int Task_ID );	//����ִ�к���ָ��
}STS_Task;

static STS_Task tasks[ STS_MAX_TASK_COUNT ] = {0};

//�������������������ID
//������ģʽΪʱ�䴥����tΪ����ʱ����
//������ģʽΪ�Զ��庯��������trigger_funcΪ�Զ��庯��
//mainfuncΪ����������
unsigned int STS_Add_Task( STS_Task_Trigger_Mode mode , float t , bool (*trigger_func)( unsigned int Task_ID ) , void (*mainfunc)( unsigned int Task_ID ) )
{
	int16_t first_available_id = -1;
	for( uint16_t i = 0 ; i < STS_MAX_TASK_COUNT ; ++i )
	{
		if( tasks[ i ].mainfunc == 0 )
		{
			first_available_id = i;
			break;
		}
	}
	//�������������ټӣ�����0
	if( first_available_id < 0 )
		return 0;
	
	if( mode == STS_Task_Trigger_Mode_Custom && trigger_func == 0 )
		return 0;
	else if( t < 0 )
		return 0;
	
	tasks[ first_available_id ].mode = mode;
	tasks[ first_available_id ].t = t;
	tasks[ first_available_id ].trigger_func = trigger_func;
	tasks[ first_available_id ].mainfunc = mainfunc;
	tasks[ first_available_id ].shielded = false;
	return first_available_id;
}

//ɾ������
bool STS_Remove_Task( unsigned int Task_ID )
{
	if( Task_ID >= STS_MAX_TASK_COUNT )
		return false;
	if( tasks[ Task_ID ].mainfunc != 0 )
		return false;
	tasks[ Task_ID ].mainfunc = 0;
	return true;	
}

//�ڱ�����
bool STS_Shield_Task( unsigned int Task_ID )
{
	if( Task_ID >= STS_MAX_TASK_COUNT )
		return false;
	if( tasks[ Task_ID ].mainfunc == 0 )
		return false;
	tasks[ Task_ID ].shielded = true;
	return true;	
}

//�����ڱε�����
bool STS_Manifest_Task( unsigned int Task_ID )
{
	if( Task_ID >= STS_MAX_TASK_COUNT )
		return false;
	if( tasks[ Task_ID ].mainfunc == 0 )
		return false;
	tasks[ Task_ID ].shielded = false;
	return true;	
}

void STS_Run()
{
	while(1)
	{
		for( uint16_t i = 0 ; i < STS_MAX_TASK_COUNT ; ++i )
		{
			if( (tasks[ i ].mainfunc != 0) && (tasks[ i ].shielded != false) )
			{
				switch( tasks[i].mode )
				{
					//����ʱ�䴥��
					case STS_Task_Trigger_Mode_RoughTime:
					{
						float pass_time = get_pass_time( tasks[i].last_trigger_time );
						if( pass_time >= tasks[i].t )
						{
							tasks[i].mainfunc( i );
							tasks[i].last_trigger_time = get_TIME_now();
						}
						break;
					}
					
					//��ȷʱ�䴥��
					case STS_Task_Trigger_Mode_PreciseTime:
					{
						float pass_time = get_pass_time_st( &tasks[i].last_trigger_time );
						if( pass_time >= tasks[i].t )
							tasks[i].mainfunc( i );
						break;
					}
					
					//�Զ��庯������
					case STS_Task_Trigger_Mode_Custom:
					{
						if( tasks[i].trigger_func( i ) )
						{
							tasks[i].mainfunc( i );
							tasks[i].last_trigger_time = get_TIME_now();
						}
						break;
					}
				}
			}
		}
	}
}