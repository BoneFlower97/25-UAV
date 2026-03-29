#include "lcdprocess.h"
#include "global.h"
#include "lcd.h"
#include "gui.h"
#include "GUIInterface.h"
void lcdTask(void *param) 
{
	vTaskDelay(2000);
	//static uint32_t test;
	uint8_t test1,test2,test3;
	LCD_Init();	   //液晶屏初始化
	rc_control_command_t rc_control;
//	Show_Str(10,90,GREEN,BLUE,"4.0\" ST7796S 320X480",12,1);//居中显示
//	Show_Str(10,120,BLUE,BLUE,"xiaoFeng@QDtech 2019-09-25",24,1);//居中显示
//	POINT_COLOR = WHITE;
//	LCD_DrawFillRectangle(300, 0, 301, 480); // 画实心矩形2
	uint16_t Rectangle_x = 1;
	uint8_t Rectangle_num;
	
	while(1)
	{
	//LCD_DrawFillRectangle(200, 0, 240, 480); // 画实心矩形2
		
		Show_Str(10,10,GREEN,BLUE,"PITHCH:",24,1);//居中显示
		LCD_ShowNum(100,10,test1,3,24);//居中显示
		Show_Str(10,60,GREEN,BLUE,"ROLL:",24,1);//居中显示
		LCD_ShowNum(100,120,200,3,24);//居中显示
		Show_Str(10,120,GREEN,BLUE,"YAW:",24,1);//居中显示
		test1++;
		//Show_Str(10,120,BLUE,BLUE,"xiaoFeng@QDtech 2019-09-25",24,1);//居中显示
		POINT_COLOR = WHITE;
		LCD_DrawFillRectangle(200, 0, 201, 480); // 画实心矩形2	
		//LCD_DrawRectangle(200,0,350,40);
		//LCD_DrawFillRectangle(200, 0, 201, 480); // 画实心矩形2	
		if(pdTRUE == xQueueReceive(rccontrolDataQueue, &rc_control, 0))
		{
			
			if(rc_control.up == true)
			{
				if(Rectangle_num>1)
				{
					POINT_COLOR = BLACK;
					LCD_DrawRectangle(200,Rectangle_x,350,Rectangle_x+40);
					POINT_COLOR = WHITE;
					Rectangle_x-=40;
					Rectangle_num--;

				}
				else
				{
					POINT_COLOR = BLACK;
					LCD_DrawRectangle(200,Rectangle_x,350,Rectangle_x+40);
					POINT_COLOR = WHITE;
					Rectangle_num = 7;
					Rectangle_x=1;
				}
					
			}
			if(rc_control.down == true)
			{
				if(Rectangle_num<7)
				{
					//POINT_COLOR = BLACK;
					//LCD_DrawRectangle(200,Rectangle_x,350,Rectangle_x+40);
					//POINT_COLOR = WHITE;
					Rectangle_x+=40;
					Rectangle_num++;
				}
				
			}
			LCD_DrawRectangle(200,Rectangle_x,350,Rectangle_x+40);
		}
		vTaskDelay(1);
	}
}
