#include "TM4C123GH6PM.h"

#include <stdio.h>
#include <rt_heap.h>
#include <stdlib.h>

#include "Basic.h"
#include "drv_main.h"
#include "MS_main.h"
#include "STS.h"
#include "Modes.h"
#include "DebugComm.h"
#include "ctrl_Main.h"
#include "Configurations.h"
#include "CommuLink.h"
#include "drv_PWMOut.h"
#include "drv_EEPROM.h"

#if 1 //���û����Σ�����Ҫ��targetѡ����ѡ��ʹ��USE microLIB

	__asm(".global __use_no_semihosting\n\t") ;//ע�ͱ���, ����1
		FILE __stdout;

		void _sys_exit(int x)
		{
			x = x;
		}

		//__use_no_semihosting was requested, but _ttywrch was referenced, �������º���, ����2
		void _ttywrch(int ch)
		{
			ch = ch;
		}
		
		char *_sys_command_string(char *cmd, int len)
		{
				return 0;
		}
#endif

int main()
{
	init_Basic();
	init_drv_EEPROM();
	init_MS();
	init_ControlSystem();
	init_Drivers();	
	
	init_Configurations();
	
	init_Modes();
	init_CommuLink();
	
	init_Debug();
	//while(1);
	STS_Run();
}

void HardFault_Handler()
{
	//�����ж������������
	PWM_PullDownAll();
}