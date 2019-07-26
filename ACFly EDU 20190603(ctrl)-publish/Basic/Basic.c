#include "Basic.h"

#include "sysctl.h"
#include "systick.h"
#include "fpu.h"
#include "hw_ints.h"
#include "interrupt.h"

#include "TM4C123GH6PM.h"

static volatile uint32_t current_time_part_1 = 0;
static const unsigned int systick_load = 16777000;
const float TIM2sec_scale = 1.0f / (float)SYSTIMERCLK;
static const float TIM2sec_part1_scale = (float)4294967296 / (float)SYSTIMERCLK;

static void Systick_IRQHandler()
{
	++current_time_part_1;
}


#pragma region TIME

	//��ȡ��ǰʱ��
	TIME get_TIME_now()
	{
		TIME t_now;
				
		uint32_t part1;
		uint32_t part2;
		do
		{
			part1 = current_time_part_1;
			part2 = systick_load - SysTick->VAL;
		}while( part1 != current_time_part_1 );
		t_now.t = (uint64_t)part1*(uint64_t)systick_load + (uint64_t)part2;
		return t_now;
	}
	
	//�������last_time��ʱ��
	float get_pass_time( TIME last_time )
	{
		TIME current_time = get_TIME_now();
		typedef union
		{
			uint64_t long_data;
			unsigned int int_data[2];
		}long_int_union;
		if( current_time.t > last_time.t )
		{
			uint64_t time_diff = current_time.t - last_time.t;
			long_int_union* convert = (long_int_union*)&time_diff;
			return TIM2sec_part1_scale * convert->int_data[1] + TIM2sec_scale * convert->int_data[0];
		}
		else
		{
			uint64_t time_diff = last_time.t - current_time.t;
			long_int_union* convert = (long_int_union*)&time_diff;
			return -TIM2sec_part1_scale * convert->int_data[1] - TIM2sec_scale * convert->int_data[0];
		}
	}
	
	//�������time_b��time_a������ʱ��
	float get_time_difference( TIME time_a , TIME time_b )
	{
		typedef union
		{
			uint64_t long_data;
			unsigned int int_data[2];
		}long_int_union;
		if( time_a.t > time_b.t )
		{
			uint64_t time_diff = time_a.t - time_b.t;
			long_int_union* convert = (long_int_union*)&time_diff;
			return TIM2sec_part1_scale * convert->int_data[1] + TIM2sec_scale * convert->int_data[0];
		}
		else
		{
			uint64_t time_diff = time_b.t - time_a.t;
			long_int_union* convert = (long_int_union*)&time_diff;
			return -TIM2sec_part1_scale * convert->int_data[1] - TIM2sec_scale * convert->int_data[0];
		}
	}
	
	//�������last_time��ʱ��
	//����last_time����Ϊ��ǰʱ��
	//���ھ�ȷ��ʱ
	float get_pass_time_st( TIME* last_time )
	{
		TIME current_time = get_TIME_now();
		typedef union
		{
			uint64_t long_data;
			unsigned int int_data[2];
		}long_int_union;
		if( current_time.t > last_time->t )
		{
			uint64_t time_diff = current_time.t - last_time->t;
			last_time->t = current_time.t;
			long_int_union* convert = (long_int_union*)&time_diff;
			return TIM2sec_part1_scale * convert->int_data[1] + TIM2sec_scale * convert->int_data[0];
		}
		else
		{
			uint64_t time_diff = last_time->t - current_time.t;
			last_time->t = current_time.t;
			long_int_union* convert = (long_int_union*)&time_diff;
			return -TIM2sec_part1_scale * convert->int_data[1] - TIM2sec_scale * convert->int_data[0];
		}
	}
	//����ϵͳ����ʱ��
	float get_System_Run_Time()
	{
		TIME current_time = get_TIME_now();
		typedef union
		{
			uint64_t long_data;
			unsigned int int_data[2];
		}long_int_union;
		
		uint64_t time_diff = current_time.t;
		long_int_union* convert = (long_int_union*)&time_diff;
		return TIM2sec_part1_scale * convert->int_data[1] + TIM2sec_scale * convert->int_data[0];
	}
	
	//��ʱ������Ϊ������
	//��Ҫ�����ж�
	void Time_set_inValid( TIME* t )
	{
		t->t = 0;
	}
	bool Time_isValid( TIME t )
	{
		return (t.t != 0);
	}

	void delay( float t )
	{
		TIME start_time = get_TIME_now();
		while( get_pass_time(start_time) <= t );
	}
	
#pragma endpragma


void init_Basic()
{	
	//����ϵͳʱ�� 80mhz
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);			
	
	//�����������㵥Ԫ	
	FPULazyStackingEnable();	
	FPUEnable();
	
	//�ر�BOR Reset
	SYSCTL->PBORCTL = 0;
	
	//�����ж����ȼ����
	//16����ռ���ȼ�
	NVIC_SetPriorityGrouping(3);
	
	//����Systick��ʱ����ʱ
	SysTickPeriodSet(systick_load); // 1000 for milliseconds & 1000000 for microseconds
	SysTickIntRegister(Systick_IRQHandler);
	NVIC_SetPriority( SysTick_IRQn , 0 );
	SysTickIntEnable();
	SysTickEnable();
	
	delay( 0.5f );
}