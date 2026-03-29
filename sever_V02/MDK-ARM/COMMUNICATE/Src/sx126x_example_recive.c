#include "sx126x_example_recive.h"
#include "radio.h"
#include "project_config.h"
#include "stdio.h"
//#include "stm32f10x_it.h"
//#include "delay.h"
#include "string.h"
#include "global.h"

/*!
 * Radio events function pointer
 * 这个是传参进入其他函数中了，所以用全局变量(局部变量使用完了内存释放可能导致异常)
 */
uint8_t sendbuff[32];
uint8_t sendflag = 0;

static RadioEvents_t SX126xRadioEvents;

static void SX126xOnTxDone( void );
static void SX126xOnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
static void SX126xOnTxTimeout( void );
static void SX126xOnRxTimeout( void );
static void SX126xOnRxError( void );

//static void sendWirelessData(uint8_t com)
//{
//	uint8_t i = 0;
//	uint8_t j = 0;
//	uint8_t k = 0;
//	uint8_t sumcheck = 0;
//	uint8_t addcheck = 0;
//	uint16_t PAR_ID;
//  uint32_t PAR_VAL;
//	
//	sendbuff[i++]   = 0xAA;
//	sendbuff[i++] = 0xFF;
//	sendbuff[i++] = com;
//	switch(com)
//	{
//		default:
//		case 0xE0:
//			
//		break;
//		case 0xE2:
//			sendbuff[i++] = 0x06;
//			
//			sendbuff[i++] = 0x05;
//			sendbuff[i++] = 0x00;
//			
//		  sendbuff[i++] = 0x00;
//			sendbuff[i++] = 0x00;
//			sendbuff[i++] = 0x00;
//			sendbuff[i++] = 0x00;
//		break;		
//	}
//	for(j = 0;j<sendbuff[3]+4;j++)
//	{
//		sumcheck += sendbuff[j];
//		addcheck += sumcheck;
//	}
//	sendbuff[i++] = sumcheck;
//	sendbuff[i++] = addcheck;
//	global_usart.tx_len = i;
//	global_usart.tx_flag = 1;
//	//Radio.Send((uint8_t *)sendbuff,i);
//	//Radio.Send((uint8_t *)sendbuff,i);
//	//Radio.Send((uint8_t *)sendData1,strlen(sendData1));
//	
//	//Radio.Rx( FSK_RX_TIMEOUT_VALUE );
////	printf("sendbuff: ");
////	if (i > 0) {
////			for (k = 0; k < i; k++) {
////					printf("%02X ", sendbuff[k]);  // %02X 表示两位十六进制，不足两位补0
////					// 每16个字节换行，便于阅读
////					if ((k + 1) % 16 == 0 && k != i - 1) {
////							printf("\r\n");
////					}
////			}
////			printf("\r\n");
////		}
//}
	
static void receiveWirelessData(uint8_t *payload, uint16_t size)
{
	uint8_t step = 0;
	uint8_t i = 0;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint16_t PAR_ID;
  uint32_t PAR_VAL;
	if(size>5)
	{
//				sendbuff[step++]   = 0xAA;
//				sendbuff[step++] = 0xFF;
//				switch(payload[2])
//				{
//					case 0xE0:
//						sendbuff[step++] = 0X00;
//						sendbuff[step++] = 0X03;
//						sendbuff[step++] = payload[2];
//						sendbuff[step++] = payload[15];
//						sendbuff[step++] = payload[16];
//					break;
//					case 0xE1:
//						PAR_ID  = ((uint16_t)payload[5] << 8) | payload[4];
//						if(PAR_ID ==	0x00)	PAR_VAL = 0;
//						else if(PAR_ID ==	0x01) PAR_VAL = 0x05;
//						else PAR_VAL = 0xFF;
//						sendbuff[step++] = 0XE2;
//						sendbuff[step++] = 0x06;
//						
//						sendbuff[step++] = payload[4];
//						sendbuff[step++] = payload[5];
//						
//						sendbuff[step++] = (uint8_t)(PAR_VAL & 0xFF);;
//						sendbuff[step++] = (uint8_t)((PAR_VAL >> 8)  & 0xFF);
//						sendbuff[step++] = (uint8_t)((PAR_VAL >> 16) & 0xFF);
//						sendbuff[step++] = (uint8_t)((PAR_VAL >> 24) & 0xFF);	
//					break;
//			}
//			for(i = 0;i<sendbuff[3]+4;i++)
//			{
//				sumcheck += sendbuff[i];
//				addcheck += sumcheck;
//			}
//			sendbuff[step++] = sumcheck;
//			sendbuff[step++] = addcheck;
//			sendflag = 0;
//			
		}
	}


//开启一个定时发送任务，每隔1S发送一条数据
void wirelessTask(void *param){
	uint8_t OCP_Value = 0;
	vTaskDelay(50);
	uint32_t lastWakeTime = xTaskGetTickCount();
	//printf("start %s() FSK example\r\n",__func__);
	SX126xRadioEvents.TxDone = SX126xOnTxDone;
	SX126xRadioEvents.RxDone = SX126xOnRxDone;
	SX126xRadioEvents.TxTimeout = SX126xOnTxTimeout;
	SX126xRadioEvents.RxTimeout = SX126xOnRxTimeout;
	SX126xRadioEvents.RxError = SX126xOnRxError;

	Radio.Init( &SX126xRadioEvents );
	// 1. 设置FSK频率（复用原LoRa频率宏，或替换为FSK_FRE）
	Radio.SetChannel(FSK_FRE);
	
	// 2. 修正RadioSetTxConfig调用（匹配参数数量+FSK配置）
	// 参数顺序：modem → power → fdev → bandwidth → datarate → coderate → preambleLen → fixLen → crcOn → freqHopOn → hopPeriod → iqInverted → timeout
	Radio.SetTxConfig( 
	    MODEM_FSK,                // [1] 调制模式：FSK（替换MODEM_LORA）
	    FSK_TX_OUTPUT_POWER,      // [2] 发射功率
	    FSK_FDEV,                 // [3] FSK频偏（LoRa此处填0，FSK必填）
	    FSK_BANDWIDTH,            // [4] FSK带宽（LoRa此处是带宽，FSK填接收带宽）
	    FSK_BITRATE,              // [5] FSK比特率（LoRa此处是扩频因子）
	    FSK_CODINGRATE,           // [6] FSK无编码率，填0（LoRa此处是编码率）
	    FSK_PREAMBLE_LENGTH,      // [7] 前导码长度
	    FSK_FIX_LENGTH_PAYLOAD,   // [8] 是否固定payload长度
	    FSK_CRC_EN,               // [9] CRC使能
	    FSK_FREQ_HOP_ON,          // [10] 关闭跳频
	    FSK_HOP_PERIOD,           // [11] 跳频周期（关闭则填0）
	    FSK_IQ_INVERSION,         // [12] IQ反转（FSK填false）
	    FSK_TX_TIMEOUT            // [13] 发射超时
	);

	OCP_Value = Radio.Read(REG_OCP);
	//printf("[%s()-%d]read OCP register value:0x%04X\r\n",__func__,__LINE__,OCP_Value);
	
	// 3. 修正RadioSetRxConfig调用（匹配参数数量+FSK配置）
	// 参数顺序：modem → bandwidth → datarate → coderate → bandwidthAfc → preambleLen → symbTimeout → fixLen → payloadLen → crcOn → freqHopOn → hopPeriod → iqInverted → rxContinuous
	Radio.SetRxConfig( 
	    MODEM_FSK,                // [1] 调制模式：FSK
	    FSK_BANDWIDTH,            // [2] 接收带宽
	    FSK_BITRATE,              // [3] 比特率
	    FSK_CODINGRATE,           // [4] 编码率（FSK填0）
	    FSK_BANDWIDTH_AFC,        // [5] AFC带宽（和接收带宽一致）
	    FSK_PREAMBLE_LENGTH,      // [6] 前导码长度
	    FSK_SYMBOL_TIMEOUT,       // [7] 符号超时（FSK填0）
	    FSK_FIX_LENGTH_PAYLOAD,   // [8] 固定payload长度
	    FSK_PAYLOAD_LEN,          // [9] payload长度（非固定填0xFF）
	    FSK_CRC_EN,               // [10] CRC使能
	    FSK_FREQ_HOP_ON,          // [11] 跳频使能
	    FSK_HOP_PERIOD,           // [12] 跳频周期
	    FSK_IQ_INVERSION,         // [13] IQ反转
	    FSK_RX_CONTINUOUS         // [14] 连续接收
	);
	
	// 打印FSK配置（替换原LoRa打印）
	//printf("FSK config\r\n");
	//printf("freq: %lu\r\n Tx power: %d\r\n FDEV: %d\r\n bandwidth: %d\r\n bitrate: %d\r\n preamble len: %d\r\n",
	 //      FSK_FRE,FSK_TX_OUTPUT_POWER,FSK_FDEV,FSK_BANDWIDTH,FSK_BITRATE,FSK_PREAMBLE_LENGTH);
	Radio.Rx( FSK_RX_TIMEOUT_VALUE );	//进入接收模式
	// 4. 发送数据（FSK模式下Send接口无需修改）
	
	while(1){
		vTaskDelayUntil(&lastWakeTime, 1); /*1ms周期延时*/
		//Radio.Send((uint8_t *)"123456",FSK_PAYLOAD_LEN);
		Radio.IrqProcess(); // Process Radio IRQ
		
		
		if(sendflag == 1)
		{
			Radio.Standby();
			Radio.Send((uint8_t *)sendbuff,FSK_PAYLOAD_LEN);
			sendflag = 0;
			//Radio.Rx( FSK_RX_TIMEOUT_VALUE );
		}
	}
}

static void SX126xOnTxDone( void )
{

	//printf("TxDone\r\n");
	Radio.Standby();
	Radio.Rx( FSK_RX_TIMEOUT_VALUE );
	//发送完成闪烁一下led提示
//	GPIO_SetBits(GREEN_LED_PORT,GREEN_LED_PIN);
//	delay_ms(100);
//	GPIO_ResetBits(GREEN_LED_PORT,GREEN_LED_PIN);
}

static void SX126xOnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	uint16_t i;
	//printf("RxDone\r\n");
	Radio.Standby();
	//printf("size:%d\r\nrssi:%d\r\nsnr:%d\r\npayload:%s\r\n",size,rssi,snr,payload);
	//printf("size:%d\r\nrssi:%d\r\nsnr:%d\r\n",size,rssi,snr);
	
//	printf("payload: ");
//	if (payload != NULL && size > 0) {
//			for (i = 0; i < size; i++) {
//					printf("%02X ", payload[i]);  // %02X 表示两位十六进制，不足两位补0
//					// 每16个字节换行，便于阅读
//					if ((i + 1) % 16 == 0 && i != size - 1) {
//							printf("\r\n");
//					}
//			}
//			printf("\r\n");
//	};
	//memcpy(&global_usart.payload, payload, size);
	receiveWirelessData(payload,size);
	Radio.Rx( FSK_RX_TIMEOUT_VALUE );
//	GPIO_SetBits(GREEN_LED_PORT,GREEN_LED_PIN);
//	delay_ms(100);
//	GPIO_ResetBits(GREEN_LED_PORT,GREEN_LED_PIN);
}

static void SX126xOnTxTimeout( void )
{
	//printf("TxTimeout\r\n");
}

static void SX126xOnRxTimeout( void )
{
	Radio.Standby();
	//printf("RxTimeout retry recive\r\n");
	Radio.Rx( FSK_RX_TIMEOUT_VALUE ); 
}

static void SX126xOnRxError( void )
{
	Radio.Standby();
	//printf("RxError retry recive\r\n");
	Radio.Rx(FSK_RX_TIMEOUT_VALUE); 
}
