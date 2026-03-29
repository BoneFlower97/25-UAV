#include "link.h"
#include "usart.h"
/*FreeRTOS?????*/
#include "global.h"

bool link_sta;
void linkTxTask(void *param)
{
	my2printf("Debug is begin:\r\n");
	uint8_t step = 0;
    while(1)
    {
			step++;
			vTaskDelay(100);
			if(link_sta == true)
			{
				HAL_GPIO_WritePin(LED_STA_GPIO_Port,LED_STA_Pin,GPIO_PIN_SET);	//输出高电平
					
				vTaskDelay(150);
				HAL_GPIO_WritePin(LED_STA_GPIO_Port,LED_STA_Pin,GPIO_PIN_RESET);	//输出高电平
				vTaskDelay(150);
			}
			if(step>10)
			{
				link_sta = false;
			}
				//my2printf("test time\r\n");
    }
}
