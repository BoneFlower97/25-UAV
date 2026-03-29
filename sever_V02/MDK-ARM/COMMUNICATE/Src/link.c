#include "link.h"
#include "usart.h"
/*FreeRTOS?????*/
#include "FreeRTOS.h"
#include "task.h"

void linkTxTask(void *param)
{
	my2printf("Debug is begin:\r\n");
    while(1)
    {
        vTaskDelay(150);
				//my2printf("test time\r\n");
    }
}
