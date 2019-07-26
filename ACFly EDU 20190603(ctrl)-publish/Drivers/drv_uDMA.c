#include "Basic.h"
#include "drv_uDMA.h"

#include "sysctl.h"
#include "udma.h"

//uDMA Control Table�ڴ�
//ע�⣺������alternate DMA�����轫�ռ�����һ��
#if defined(ewarm)
	#pragma data_alignment=1024
	uint8_t uDMA_buf[1024];
#elif defined(ccs)
	#pragma DATA_ALIGN(ui8ControlTable, 1024)
	uint8_t uDMA_buf[1024];
#else
	uint8_t uDMA_buf[1024] __attribute__ ((aligned(1024)));
#endif

void init_drv_uDMA()
{
	//����uDMAʱ�� masterenable
	SysCtlPeripheralEnable( SYSCTL_PERIPH_UDMA );
	uDMAEnable();
	
	//�趨uDMA Control Table
	uDMAControlBaseSet( uDMA_buf );
}