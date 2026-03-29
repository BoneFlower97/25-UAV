#include "wireless.h"
#include "sx126x_example_recive.h"
#include "radio.h"
#include "project_config.h"
#include "global.h"
#define RX_DATA_MAX_SIAZ    50



uint8_t rx_data[RX_DATA_MAX_SIAZ];
uint8_t tx_data[RX_DATA_MAX_SIAZ];
uint8_t Rx_to_Tx_data[RX_DATA_MAX_SIAZ];
static uint8_t flag;
static uint32_t param_write_cmd(uint16_t PAR_ID)
{
	uint32_t  PAR_VAL;
    switch (PAR_ID)
    {

    case 0X00:
        /* code */
        PAR_VAL = 0;
        break;
    case 0x01:
        PAR_VAL = 0X05;
        break;
    break;
    case 0x02:
        PAR_VAL = 0XFF;
    break;
    case 0x03:
        PAR_VAL = 0XFF;
    break;
    case 0x04:
        PAR_VAL = 0XFF;
    break;
    default:
        PAR_VAL = 0;
        break;
    }
    return PAR_VAL;
}
static void DataProcess(const uint8_t *buf)
{
    uint8_t  step = 0;
    uint8_t  expression;
    uint8_t  data_size;

    uint16_t PAR_ID;
    uint32_t PAR_VAL;

    uint16_t sumcheck = 0;
    uint16_t addcheck = 0;

    memset(Rx_to_Tx_data, 0x00, sizeof(Rx_to_Tx_data));

    expression = buf[2];
    data_size  = buf[3];

    switch (expression)
    {
        case 0x00:
            /* 保留 */
            break;

        case 0xE1:
        {
            PAR_ID  = ((uint16_t)buf[5] << 8) | buf[4];
            PAR_VAL = param_write_cmd(PAR_ID);

            Rx_to_Tx_data[step++] = 0xAA;
            Rx_to_Tx_data[step++] = 0xFF;
            Rx_to_Tx_data[step++] = 0xE2;
            Rx_to_Tx_data[step++] = 0x06;

            Rx_to_Tx_data[step++] = buf[4];
            Rx_to_Tx_data[step++] = buf[5];

            Rx_to_Tx_data[step++] = (uint8_t)(PAR_VAL & 0xFF);
            Rx_to_Tx_data[step++] = (uint8_t)((PAR_VAL >> 8)  & 0xFF);
            Rx_to_Tx_data[step++] = (uint8_t)((PAR_VAL >> 16) & 0xFF);
            Rx_to_Tx_data[step++] = (uint8_t)((PAR_VAL >> 24) & 0xFF);

            for (uint8_t i = 0; i < Rx_to_Tx_data[3] + 4; i++)
            {
                sumcheck += Rx_to_Tx_data[i];
                addcheck += sumcheck;
            }


        }
        break;

        case 0xE0:
        {
            Rx_to_Tx_data[step++] = 0xAA;
            Rx_to_Tx_data[step++] = 0xFF;
            Rx_to_Tx_data[step++] = 0x00;
            Rx_to_Tx_data[step++] = 0x03;

            Rx_to_Tx_data[step++] = buf[2];
            Rx_to_Tx_data[step++] = buf[15];
            Rx_to_Tx_data[step++] = buf[16];

            for (uint8_t i = 0; i < Rx_to_Tx_data[3] + 4; i++)
            {
                sumcheck += Rx_to_Tx_data[i];
                addcheck += sumcheck;
            }


        }
        break;

        default:
            /* 未定义命令 */
            break;
    }
		            Rx_to_Tx_data[step++] = (uint8_t)sumcheck;
            Rx_to_Tx_data[step++] = (uint8_t)addcheck;

//            usart1sendbuff(Rx_to_Tx_data, step);
}

void wirelessTxTask(void *param)
{
		uint32_t lastWakeTime = xTaskGetTickCount();
		rc_ctrl_t rc_ctrl;
		control_command_t	con;
	  uint8_t tx_data[RX_DATA_MAX_SIAZ];
		uint8_t step;
	  uint16_t sumcheck = 0; 
    uint16_t addcheck = 0; 
    while(1)
    {
			vTaskDelayUntil(&lastWakeTime, 50); /*1ms周期延时*/
			if(pdTRUE == xQueueReceive(controlcommandDataQueue, &con, 0))
			{
				step = 0;
				tx_data[step++] = 0xAA;
				tx_data[step++] = 0xFF;
				tx_data[step++] = 0xE0;
				tx_data[step++] = 11;
				tx_data[step++] = 0x10;
				tx_data[step++] = 0x00;
				if(con.motor_lock == true) tx_data[step++] = 0x01;
				else if(con.motor_lock == false) tx_data[step++] = 0x02;
				tx_data[step++] = 0x00;
				tx_data[step++] = 0x00;
				tx_data[step++] = 0x00;
				tx_data[step++] = 0x00;
				tx_data[step++] = 0x00;
				tx_data[step++] = 0x00;
				tx_data[step++] = 0x00;
				tx_data[step++] = 0x00;
				sumcheck = addcheck = 0;
				for(uint8_t i = 0; i<tx_data[3]+4;i++)
				{
						sumcheck += tx_data[i];  
						addcheck += sumcheck; 
				}
				tx_data[step++] = (uint8_t)sumcheck;
				tx_data[step++] = (uint8_t)addcheck;
				//usart1sendbuff(tx_data,step);				
			}
			else if(pdTRUE == xQueueReceive(rcctrlDataQueue, &rc_ctrl, 0))
			{
				step = 0;
				tx_data[step++] = 0xAA;
				tx_data[step++] = 0xFF;
				tx_data[step++] = 0x41;
				tx_data[step++] = 14;
				tx_data[step++] = (int8_t)(rc_ctrl.ctrl_rol & 0xFF);
				tx_data[step++] = (int8_t )((rc_ctrl.ctrl_rol >> 8) & 0xFF);
				tx_data[step++] = (int8_t )(rc_ctrl.ctrl_pit & 0xFF);
				tx_data[step++] = (int8_t )((rc_ctrl.ctrl_pit >> 8) & 0xFF);
				tx_data[step++] = (int8_t )(rc_ctrl.ctrl_thr & 0xFF);
				tx_data[step++] = (int8_t )((rc_ctrl.ctrl_thr >> 8) & 0xFF);
				tx_data[step++] = (int8_t )(rc_ctrl.ctrl_yawdps & 0xFF);
				tx_data[step++] = (int8_t )((rc_ctrl.ctrl_yawdps >> 8) & 0xFF);
				if(con.motor_lock == true) tx_data[step++] = 0x01;
				else if(con.motor_lock == false) 
				{
					tx_data[step++] = 0x00;
				}
				tx_data[step++] = 0;
				tx_data[step++] = 0;
				tx_data[step++] = 0;
				tx_data[step++] = 0;
				tx_data[step++] = 0;
				sumcheck = addcheck = 0;
				for(uint8_t i = 0; i<tx_data[3]+4;i++)
				{
						sumcheck += tx_data[i];  
						addcheck += sumcheck; 
				}
				tx_data[step++] = (uint8_t)sumcheck;
				tx_data[step++] = (uint8_t)addcheck;
				Radio.Send((uint8_t *)tx_data,FSK_PAYLOAD_LEN);
				//usart1sendbuff(tx_data,step);		
			}
			//my1printf("123");
							
    }
}

void wirelessRxTask(void *param)
{

//    uart_frame_t frame;
//    control_t control;
//	HAL_UART_Receive_DMA(&huart1, rx_data, sizeof(rx_data));  
    while(1)
    {
			vTaskDelay(150);
//        if(pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
//        {
						//HAL_UART_Receive_DMA(&huart1, rx_data, sizeof(rx_data));
//            //xSemaphoreTake(uart1Mutex, portMAX_DELAY);
//			//my1printf("123");
//            memcpy(frame.buf, rx_data, RX_DATA_MAX_SIAZ);
//            frame.len = RX_DATA_MAX_SIAZ;
//            xQueueSend(uartRxQueue, &frame, portMAX_DELAY);
//			
//            //if(rx_data[0] == 0xAA && rx_data[2] == 0xE0)
//					if(rx_data[2] == 0xE0)
//            {
//                if(rx_data[6] == 0x05)  control.accelerator=10;
//                else if(rx_data[6] == 0x06)  control.accelerator=0;
//                xQueueOverwrite(controlQueue, &control);
//            }
            //xSemaphoreGive(uart1Mutex);
//        }
    }
}
