/*!
 * \file      sx1262mbxcas-board.c
 *
 * \brief     Target board SX1262MBXCAS shield driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdlib.h>
#include "project_config.h"
//#include "delay.h"
#include "radio.h"
#include "sx126x-board.h"
#include "spi.h"
#include "stdio.h"
#include "tim.h"
#include "global.h"
#include "gpio.h"

typedef struct __softTimer_s{
	uint8_t isRun;		//0停止，1运行
	uint32_t delayMs;	//定时器需要多长时间后唤醒
	uint32_t startMs;	//定时器开始时间
}SoftTimer_t;

static DioIrqHandler *dio1IrqCallback=NULL;	//记录DIO1的回调函数句柄
static uint32_t timer_count=0;
static SoftTimer_t txTimerHandle,rxTimerHandle;

/* 初始化sx126x需要用到的GPIO初始化，将 BUSY 引脚设置为输入模式
 */
void SX126xIoInit( void )
{
//	uint8_t u8_ret=255;
//	GPIO_InitTypeDef  GPIO_InitStructure;

//	//初始化BUSY为上拉输入模式
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	    //使能指定端口时钟
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO口速度为50MHz，这里不用传参，直接写死用最大速度50MHZ
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//上拉输入
//	GPIO_InitStructure.GPIO_Pin = RADIO_DIO4_BUSY_PIN;
//	GPIO_Init(RADIO_DIO4_BUSY_PORT, &GPIO_InitStructure);	//初始化GPIO
//	
//	//下面这几个引脚没用用到，设置为浮空输入模式
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//浮空输入
//	
//	GPIO_InitStructure.GPIO_Pin = RADIO_DIO0_TXEN_PIN;
//	GPIO_Init(RADIO_DIO0_TXEN_PORT, &GPIO_InitStructure);	//初始化GPIO

//	GPIO_InitStructure.GPIO_Pin = RADIO_DIO2_PIN;
//	GPIO_Init(RADIO_DIO2_PORT, &GPIO_InitStructure);	//初始化GPIO
//	
//	GPIO_InitStructure.GPIO_Pin = RADIO_DIO3_PIN;
//	GPIO_Init(RADIO_DIO3_PORT, &GPIO_InitStructure);	//初始化GPIO
//	
//	GPIO_InitStructure.GPIO_Pin = RADIO_DIO5_RXEN_PIN;
//	GPIO_Init(RADIO_DIO5_RXEN_PORT, &GPIO_InitStructure);	//初始化GPIO
//	
//	u8_ret=Spi1Init();
//	if(u8_ret){
//		printf("spi1 init error\r\n");
//	}else{
//		//printf("SX126xIoInit init sucess\r\n");
//	}
}

/* 初始化 DIO1
 * 将DIO1设置为外部中断(上升沿触发)，并且回调函数为 dioIrq 函数原型 void RadioOnDioIrq( void* context )
 */
void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
	dio1IrqCallback=dioIrq;
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;
// 	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	dio1IrqCallback=dioIrq;
//	
//	//gpio(DIO1)初始化为下拉输入模式
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	    //使能指定端口时钟
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO口速度为50MHz，这里不用传参，直接写死用最大速度50MHZ
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;//下拉输入
//	GPIO_InitStructure.GPIO_Pin = RADIO_DIO1_PIN;
//	GPIO_Init(RADIO_DIO1_PORT, &GPIO_InitStructure);	//初始化GPIO
//	
//	//中断配置
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟
//	//GPIOE.2 中断线以及中断初始化配置   上升沿触发
//  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource13);
//	EXTI_InitStructure.EXTI_Line=EXTI_Line13;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//上升沿触发
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

//	//设置中断优先级
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键WK_UP所在的外部中断通道
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
//	NVIC_Init(&NVIC_InitStructure);
//	EXTI_ClearITPendingBit(EXTI_Line13);  //清除LINE11上的中断标志位 
}

//在中断回调函数中回调 void SX126xIoIrqInit( DioIrqHandler dioIrq ) 中注册的 dioIrq 回调
void dio_EXTI(void){

		if(NULL!=dio1IrqCallback){
			dio1IrqCallback(NULL);
		}
}

void SX126xIoDeInit( void )
{
    //GPIO去初始化代码
}

//复位按键功能
void SX126xReset( void )
{
//	GPIO_InitTypeDef  GPIO_InitStructure;

//	SX126xDelayMs( 10 );
//	
//	//将RST输出低电平
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	    //使能指定端口时钟
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO口速度为50MHz，这里不用传参，直接写死用最大速度50MHZ
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	//推挽输出
//	GPIO_InitStructure.GPIO_Pin = RADIO_nRESET_PIN;
//	GPIO_Init(RADIO_nRESET_PORT, &GPIO_InitStructure);	//初始化GPIO
//	GPIO_WriteBit( RADIO_nRESET_PORT, RADIO_nRESET_PIN,Bit_RESET);	//RST设为低电平输出
//	
//	SX126xDelayMs( 20 );
//	
//	//将RST设置为上拉输入模式
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
//	GPIO_Init(RADIO_nRESET_PORT, &GPIO_InitStructure);	//初始化GPIO
//	
//	SX126xDelayMs( 10 );
}

//读取Busy引脚电平状态
void SX126xWaitOnBusy( void )
{
	uint32_t u32_count=0;
	
	while( HAL_GPIO_ReadPin(RADIO_DIO4_BUSY_PORT,RADIO_DIO4_BUSY_PIN) == 1 ){
		if(u32_count++>1000){
			//printf("wait busy pin timeout\r\n");
			u32_count=0;
		}
		SX126xDelayMs(1);
	}
}

//设置SPI的NSS引脚电平，0低电平，非零高电平
void SX126xSetNss(uint8_t lev ){
	if(lev){
		HAL_GPIO_WritePin(RADIO_NSS_PORT,RADIO_NSS_PIN,GPIO_PIN_SET);	//输出高电平
	}else{
		HAL_GPIO_WritePin(RADIO_NSS_PORT,RADIO_NSS_PIN,GPIO_PIN_RESET);	//输出低电平
	}
}

//spi传输一个字节的数据
uint8_t SX126xSpiInOut(uint8_t data){
	return HALSpi2InOut(data);

}

//检查频率是否符合要求，如果不需要判断则可以直接返回true
bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

//毫秒延时
void SX126xDelayMs(uint32_t ms){
	vTaskDelay(ms);
}

//tx/rx定时器操作

//定时器3中断服务程序
//tx定时结束需要回调 RadioOnTxTimeoutIrq
//rx定时结束需要回调 RadioOnRxTimeoutIrq
//void TIM3_IRQHandler(void)   //TIM3中断
//{
//	uint32_t diffMs=0;
//	
//	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
//		timer_count++;
//		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志
//		
//		//printf("timer_count=%d\r\n",timer_count);
//		
//		//处理tx定时器
//		if(txTimerHandle.isRun){
//			if(timer_count>=txTimerHandle.startMs){
//				diffMs=timer_count-txTimerHandle.startMs;
//			}else{//溢出了
//				diffMs=0xffffffff - txTimerHandle.startMs + timer_count;
//			}
//			if(diffMs>txTimerHandle.delayMs){
//				//计时结束
//				//printf("----------------- tx timer irq ---------------\r\n");
//				SX126xTxTimerStop();
//				txTimerHandle.startMs=timer_count;
//				RadioOnTxTimeoutIrq(NULL);	//tx定时结束需要回调 RadioOnTxTimeoutIrq
//			}
//		}
//		
//		//处理rx定时器
//		if(rxTimerHandle.isRun){
//			if(timer_count>=rxTimerHandle.startMs){
//				diffMs=timer_count-rxTimerHandle.startMs;
//			}else{//溢出了
//				diffMs=0xffffffff - rxTimerHandle.startMs + timer_count;
//			}
//			if(diffMs>rxTimerHandle.delayMs){
//				//计时结束
//				//printf("----------------- rx timer irq ---------------\r\n");
//				SX126xRxTimerStop();
//				rxTimerHandle.startMs=timer_count;
//				RadioOnRxTimeoutIrq(NULL);	//rx定时结束需要回调 RadioOnRxTimeoutIrq
//			}
//		}
//	}
//}
void SX126xTimerIrqHandler()
{
    uint32_t diffMs = 0;

        timer_count++;

        /* ========== TX Timer 处理 ========== */
        if(txTimerHandle.isRun)
        {
            if(timer_count >= txTimerHandle.startMs)
                diffMs = timer_count - txTimerHandle.startMs;
            else
                diffMs = 0xffffffff - txTimerHandle.startMs + timer_count;

            if(diffMs > txTimerHandle.delayMs)
            {
                SX126xTxTimerStop();
                txTimerHandle.startMs = timer_count;
                RadioOnTxTimeoutIrq(NULL);
            }
        }

        /* ========== RX Timer 处理 ========== */
        if(rxTimerHandle.isRun)
        {
            if(timer_count >= rxTimerHandle.startMs)
                diffMs = timer_count - rxTimerHandle.startMs;
            else
                diffMs = 0xffffffff - rxTimerHandle.startMs + timer_count;

            if(diffMs > rxTimerHandle.delayMs)
            {
                SX126xRxTimerStop();
                rxTimerHandle.startMs = timer_count;
                RadioOnRxTimeoutIrq(NULL);
            }
        }
    }

//初始化定时器(这里用TIM3定时器模拟出了两个ms定时器)
//tx定时结束需要回调 RadioOnTxTimeoutIrq
//rx定时结束需要回调 RadioOnRxTimeoutIrq
void SX126xTimerInit(void){
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	//使能TIM3
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
// 
//	//TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

//	//中断优先级NVIC设置
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
//	
//	//定时器TIM3初始化
//	TIM_TimeBaseStructure.TIM_Period = 10; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
//	TIM_TimeBaseStructure.TIM_Prescaler =SystemCoreClock/10000; //设置用来作为TIMx时钟频率除数的预分频值(1s10000次，也就是1次0.1ms)
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
//	
//	//关闭软件定时器
//	txTimerHandle.isRun=0;
//	rxTimerHandle.isRun=0;
//	
//	TIM_Cmd(TIM3, ENABLE);  //使能TIMx
//	TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志
//	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM2中断,允许更新中断
}

void SX126xSetTxTimerValue(uint32_t nMs){
	//printf("[%s()-%d]set timer out %d ms\r\n",__func__,__LINE__,nMs);
	txTimerHandle.delayMs=nMs;
}

void SX126xTxTimerStart(void){
	//printf("[%s()-%d]start timer\r\n",__func__,__LINE__);
	txTimerHandle.startMs=timer_count;
	txTimerHandle.isRun=1;
}

void SX126xTxTimerStop(void){
	//printf("[%s()-%d]stop timer\r\n",__func__,__LINE__);
	txTimerHandle.isRun=0;
}

void SX126xSetRxTimerValue(uint32_t nMs){
	//printf("[%s()-%d]set timer out %d ms\r\n",__func__,__LINE__,nMs);
	rxTimerHandle.delayMs=nMs;
}

void SX126xRxTimerStart(void){
	//printf("[%s()-%d]start timer\r\n",__func__,__LINE__);
	rxTimerHandle.startMs=timer_count;
	rxTimerHandle.isRun=1;
}

void SX126xRxTimerStop(void){
	//printf("[%s()-%d]stop timer\r\n",__func__,__LINE__);
	rxTimerHandle.isRun=0;
}
