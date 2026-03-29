#ifndef __PROJECT_CONFIG_H__
#define __PROJECT_CONFIG_H__

#include "main.h"
//基于semtech官网驱动移植
//官网驱动网址 https://github.com/Lora-net/LoRaMac-node/tree/master/src/radio  下载日期 2021/2/3
#define SOFT_VERSION	"sx126x driver for stm32f103 V0.0.0"

//--------------------------------------------- 测试默认配置 ---------------------------------------------
#define LORA_FRE									                  470500000	  // 收发频率
#define LORA_TX_OUTPUT_POWER                        22          // 测试默认使用的发射功率，126x发射功率0~22dbm，127x发射功率2~20dbm
#define LORA_BANDWIDTH                              0           // [0: 125 kHz,	测试默认使用的带宽，sx126x：[0: 125 kHz,1: 250 kHz,2: 500 kHz,3: Reserved]
#define LORA_SPREADING_FACTOR                       9           // 测试默认使用的扩频因子范围7~12
#define LORA_CODINGRATE                             1           // 测试默认使用的纠错编码率[1: 4/5,2: 4/6,3: 4/7,4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8           // 前导码长度
#define LORA_SX126X_SYMBOL_TIMEOUT                  0           // Symbols(sx126x用到的是0,127x用到的是5)
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false	      // 是否为固定长度包(暂时只是sx126x用到了)
#define LORA_IQ_INVERSION_ON                        false		    // 这个应该是设置是否翻转中断电平的(暂时只是sx126x用到了)
#define LORA_RX_TIMEOUT_VALUE                       5000        // 接收超时时间
#define LORA_DATA_SEND_OR_RECEIV_MODE              	0           // 数据收发模式，1为发送模式，0为接收模式


// FSK核心配置（发送/接收端必须完全一致）
#define FSK_FRE                 470500000UL  // FSK工作频率
#define FSK_TX_OUTPUT_POWER     12           // 发射功率（dBm）
#define FSK_FDEV                9600         // FSK频偏（Hz，建议=比特率/2）
#define FSK_BANDWIDTH           78200        // FSK接收/发射带宽（Hz）
#define FSK_BITRATE             19200         // FSK比特率（bps）
#define FSK_CODINGRATE          0            // FSK无编码率，填0
#define FSK_PREAMBLE_LENGTH     4            // 前导码长度（字节）
#define FSK_FIX_LENGTH_PAYLOAD  true        // 非固定payload长度
#define FSK_CRC_EN              true         // 使能CRC
#define FSK_FREQ_HOP_ON         false        // 关闭跳频
#define FSK_HOP_PERIOD          0            // 跳频周期（关闭则填0）
#define FSK_IQ_INVERSION        false        // FSK无IQ反转，填false
#define FSK_TX_TIMEOUT          1000         // 发射超时（ms）
#define FSK_SYMBOL_TIMEOUT      0            // FSK符号超时（填0即可）
#define FSK_PAYLOAD_LEN         0x20         // payload长度（非固定填0xFF）
#define FSK_BANDWIDTH_AFC       78200        // AFC带宽（和接收带宽一致）
#define FSK_RX_CONTINUOUS       false        // 非连续接收
#define FSK_RX_TIMEOUT_VALUE    20000         // 接收超时（ms）

/*!
 * Board MCU pins definitions
 */
//SPI
#define RADIO_NSS_PIN       SX_CS_Pin 
#define RADIO_NSS_PORT      SX_CS_GPIO_Port
//#define RADIO_MOSI_PIN      GPIO_Pin_7
//#define RADIO_MOSI_PORT     GPIOA
//#define RADIO_MISO_PIN      GPIO_Pin_6
//#define RADIO_MISO_PORT     GPIOA
//#define RADIO_SCK_PIN       GPIO_Pin_5
//#define RADIO_SCK_PORT      GPIOA
//RST复位脚
//#define RADIO_nRESET_PIN    GPIO_Pin_14
//#define RADIO_nRESET_PORT   GPIOB
//DIO1 引脚
#define RADIO_DIO1_PIN      DIO1_Pin
#define RADIO_DIO1_PORT     DIO1_GPIO_Port
//BUSY 引脚
#define RADIO_DIO4_BUSY_PIN      BUSY_Pin
#define RADIO_DIO4_BUSY_PORT     BUSY_GPIO_Port

//LoRa LED
//#define GREEN_LED_PIN GPIO_Pin_8
//#define GREEN_LED_PORT GPIOA


//下面这几个引脚没用用到，设置为浮空输入模式
//TXEN
//#define RADIO_DIO0_TXEN_PIN      GPIO_Pin_10
//#define RADIO_DIO0_TXEN_PORT     GPIOB
////DIO2
//#define RADIO_DIO2_PIN      GPIO_Pin_8
//#define RADIO_DIO2_PORT     GPIOB
////DIO3
//#define RADIO_DIO3_PIN      GPIO_Pin_9
//#define RADIO_DIO3_PORT     GPIOB
////RXEN
//#define RADIO_DIO5_RXEN_PIN      GPIO_Pin_1
//#define RADIO_DIO5_RXEN_PORT     GPIOA

#endif // __PROJECT_CONFIG_H__
