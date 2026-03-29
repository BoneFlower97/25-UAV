#include "wireless.h"
#include "global.h"
#include "usart.h"
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

            usart1sendbuff(Rx_to_Tx_data, step);
}
static void Lora_Init(void)
{
    uint8_t     rx_data_num; 
    uint8_t     i;
	uint8_t     step = 1;
	

    switch (step)
    {
    case 1:
        /* code */
        HAL_UART_Receive_DMA(&huart1, rx_data, sizeof(rx_data));    
        my1printf("+++\r\n");
        vTaskDelay(10);
        while(i<RX_DATA_MAX_SIAZ - 1)
        {
            if(rx_data[i] == '\r' && rx_data[i+1] == '\n')
            {
                if(memcmp(rx_data, "Entry AT\r\n", i+1))
                {
                    step = 2;
                }
            }
        i++;
        }
    case 2:
        /* code */
        HAL_UART_Receive_DMA(&huart1, rx_data, sizeof(rx_data));    
        my1printf("AT+LEVEL5\r\n");
        vTaskDelay(10);
        while(i<RX_DATA_MAX_SIAZ - 1)
        {
            if(rx_data[i] == '\r' && rx_data[i+1] == '\n')
            {
                if(memcmp(rx_data, "OK\r\n", i+1))
                {
                            my1printf("AT+RESET\r\n");
                            step = 3;
                }
            }
        i++;
        }
        case 3:
        /* code */
        HAL_UART_Receive_DMA(&huart1, rx_data, sizeof(rx_data));    
        my1printf("AT+BAUD7\r\n");
        vTaskDelay(10);
        while(i<RX_DATA_MAX_SIAZ - 1)
        {
            if(rx_data[i] == '\r' && rx_data[i+1] == '\n')
            {
                if(memcmp(rx_data, "OK\r\n", i+1))
                {
                            my1printf("AT+RESET\r\n");
                            step = 4;
                            vTaskDelay(10);
                }
            }
        i++;
        }
        case 4:
            vTaskDelay(10);
            //LORA_USART1_UART_Init();
            step = 5;
        case 5:
        /* code */
        HAL_UART_Receive_DMA(&huart1, rx_data, sizeof(rx_data));    
        my1printf("+++\r\n");
        vTaskDelay(10);
        while(i<RX_DATA_MAX_SIAZ - 1)
        {
            if(rx_data[i] == '\r' && rx_data[i+1] == '\n')
            {
                if(memcmp(rx_data, "Entry AT\r\n", i+1))
                {
                    my1printf("AT+RESET\r\n");
                    vTaskDelay(10);
									//my1printf("hello");
                }
            }
        i++;
        }
    break;
    default:
        break;
    }

}
void wirelessTxTask(void *param)
{
    static uint8_t step;
    uint16_t    ROL = 20 *100;
    uint16_t    PIT = 34 *100;
    uint16_t    YAW = 54 *100;
    static uint16_t sumcheck = 0; 
    static uint16_t addcheck = 0; 
	static state_t 		stateToSend;		/*������̬*/
    QueueSetMemberHandle_t activatedQueue;
    uart_frame_t frame;
    while(1)
    {
		//activatedQueue = xQueueSelectFromSet(wirelessQueueSet, portMAX_DELAY);
        //if (xQueueReceive(stateDataQueue, &stateToSend, portMAX_DELAY) == pdTRUE)	
        if(activatedQueue == stateDataQueue)
        {
            //my1printf("123");
            xQueueReceive(stateDataQueue, &stateToSend, 0);
           // xSemaphoreTake(uart1Mutex, portMAX_DELAY);
            ROL = (int16_t )(stateToSend.attitude.roll*100);
            PIT = (int16_t )(stateToSend.attitude.pitch*100);
            YAW = (int16_t )(stateToSend.attitude.yaw*100);
            step = 0;
            tx_data[step++] = 0xAA;
            tx_data[step++] = 0xFF ;
            tx_data[step++] = 0x03;
            tx_data[step++] = 7;
            tx_data[step++] = (int8_t)(ROL & 0xFF);
            tx_data[step++] = (int8_t )((ROL >> 8) & 0xFF);
            tx_data[step++] = (int8_t )(PIT & 0xFF);
            tx_data[step++] = (int8_t )((PIT >> 8) & 0xFF);
            tx_data[step++] = (int8_t )(YAW & 0xFF);
            tx_data[step++] = (int8_t )((YAW >> 8) & 0xFF);
            tx_data[step++] = 1;
            sumcheck = addcheck = 0;
            for(uint8_t i = 0; i<tx_data[3]+4;i++)
            {
                sumcheck += tx_data[i];  
                addcheck += sumcheck; 
            }
            tx_data[step++] = (uint8_t)sumcheck;
            tx_data[step++] = (uint8_t)addcheck;
            //usart1sendbuff(tx_data,step);
						//vTaskDelay(100);
           // xSemaphoreGive(uart1Mutex);
        }
        else if (activatedQueue == uartRxQueue)
        {
            
//DataProcess(frame.buf);
					xQueueReceive(uartRxQueue, &frame, 0);
        }
        

    }
}

void wirelessRxTask(void *param)
{

	//vTaskDelay(150);
	//Lora_Init();
	//vTaskDelay(150);
    uart_frame_t frame;
    rc_ctrl_t rc_ctrl;
		control_command_t con;
		uint8_t err_flag_test;
		HAL_UART_Receive_DMA(&huart1, rx_data, sizeof(rx_data));  
    while(1)
    {
			//vTaskDelay(10);
        if(pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
        {
						HAL_UART_Receive_DMA(&huart1, rx_data, sizeof(rx_data));
            //xSemaphoreTake(uart1Mutex, portMAX_DELAY);
			//my1printf("123");
            memcpy(frame.buf, rx_data, RX_DATA_MAX_SIAZ);
            frame.len = RX_DATA_MAX_SIAZ;
            xQueueSend(uartRxQueue, &frame, portMAX_DELAY);
			
//						if(rx_data[2] == 0xE0)
//            {
//                if(rx_data[6] == 0x05)  control.accelerator=10;
//                else if(rx_data[6] == 0x06)  control.accelerator=0;
//            }
						
						if(rx_data[2] == 0xE0)
            {
								if(rx_data[6] == 0x01) con.motor_lock = true;
								else if(rx_data[6] == 0x00) con.motor_lock = false;
                xQueueOverwrite(rcctrlDataQueue, &con);
            }	
						else if(rx_data[2] == 0x41)
						{
							rc_ctrl.ctrl_rol = (int16_t)((rx_data[5] << 8) | (rx_data[4] & 0xFF));
							rc_ctrl.ctrl_pit = (int16_t)((rx_data[7] << 8) | (rx_data[6] & 0xFF));
							rc_ctrl.ctrl_thr = (int16_t)((rx_data[9] << 8) | (rx_data[8] & 0xFF));
							rc_ctrl.ctrl_yawdps = (int16_t)((rx_data[9] << 10) | (rx_data[9] & 0xFF));
							if(rx_data[12] == 0x01) 
							{
								err_flag_test = 0;
								con.motor_lock = true;
							}
							else if(rx_data[12] == 0x00) 
							{
								err_flag_test++;
								
							}
							if(err_flag_test>2)
							{
								con.motor_lock = false;
							}
							xQueueOverwrite(rcctrlDataQueue, &con);
							xQueueOverwrite(controlcommandDataQueue, &rc_ctrl);
						}
						memset(rx_data, 0x00, sizeof(rx_data));
						
        }
    }
}
