#include "processState.h"
#include "global.h"
#include "adc.h"

#include "gpio.h"
#include <string.h>
#include <stdlib.h>

#define ADC_LPF_SHIFT  4            // 2^4 = 16，值越大越平滑

typedef struct {
    int roll_trim_angle;             // 横滚配平角 (°，左倾为正，补偿机身侧倾)
    int pitch_trim_angle;            // 俯仰配平角 (°，前俯为正，补偿机身前倾/后倾)
    int yaw_trim_angle;              // 偏航配平角 (°，右偏为正，补偿机身自转趋势)	
		int motor_trim_throttle;
} QuadrotorTrimParams;
typedef struct
{
    int16_t in_min;     // ADC 最小值
    int16_t in_max;     // ADC 最大值
    int16_t out_min;    // 输出最小值
    int16_t out_max;    // 输出最大值
} rc_map_t;

rc_map_t RC_THR_MAP =
{
    .in_min  = 1000,
    .in_max  = 3100,
    .out_min = 200,
    .out_max = 100
};

rc_map_t RC_YAW_MAP =
{
    .in_min  = 1000,
    .in_max  = 3200,
    .out_min = -100,
    .out_max = 100
};

rc_map_t RC_ROL_MAP =
{
    .in_min  = 800,
    .in_max  = 3200,
    .out_min = -1000,
    .out_max = 1000
};

rc_map_t RC_PIT_MAP =
{
    .in_min  = 800,
    .in_max  = 3200,
    .out_min = -1000,
    .out_max = 1000
};


// 状态机定义
typedef enum {
    KEY_SCAN_PHASE_COL1,
    KEY_SCAN_PHASE_COL2,
    KEY_DEBOUNCE
} KeyScanState_t;

// 按键状态定义
typedef struct {
    uint8_t current[8];      // 当前读取的8个按键状态（4行×2列）
    uint8_t stable[8];       // 稳定状态的按键
    uint8_t debounce[8];     // 消抖计数
    uint8_t pressed[8];      // 按键按下事件
    uint8_t released[8];     // 按键释放事件
} KeyMatrix_t;

// TW状态定义
typedef struct {
    uint8_t current[8];      // 当前状态
    uint8_t stable[8];       // 稳定状态
    uint8_t debounce[8];     // 消抖计数
    uint8_t changed[8];      // 状态变化事件
} TWState_t;

// 全局变量
static uint16_t adcbuf[4];

static KeyScanState_t keyScanState = KEY_SCAN_PHASE_COL1;
static KeyMatrix_t keys;
static TWState_t tws;
static uint32_t scanTimer = 0;
static uint8_t scanColumn = 0;  // 当前扫描的列
static uint16_t adc_filt[4] = {0};   // 滤波后结果

// ==================== Flash 配置参数 ====================
// STM32F103C8 Flash特性：
// - 主存储区起始地址：0x08000000
// - 总容量：64KB (0x08000000 ~ 0x08010000)
// - 每页大小：1KB (0x400 字节)
// 注意：选择Flash末尾的页存储（避免覆盖程序代码），这里选第63页（0x0800FC00 ~ 0x0800FFFF）
#define FLASH_STORE_ADDR    0x0800FC00  // 存储起始地址（第63页起始）
#define FLASH_PAGE_SIZE     0x400       // 1KB/页

// ==================== 核心函数实现 ====================
/**
 * @brief  解锁STM32F103 Flash（写入/擦除前必须解锁）
 */
void Flash_Unlock(void) {
    // 解锁序列：先写KEY1，再写KEY2
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
}

/**
 * @brief  锁定STM32F103 Flash（操作完成后锁定，防止误写）
 */
void Flash_Lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

/**
 * @brief  擦除指定地址的Flash页
 * @param  page_addr: 要擦除的页起始地址（必须是页对齐地址）
 * @retval 0: 擦除成功, 1: 擦除失败
 */
uint8_t Flash_ErasePage(uint32_t page_addr) {
    // 等待上一次操作完成
    while (FLASH->SR & FLASH_SR_BSY);
    
    // 擦除页操作
    FLASH->CR |= FLASH_CR_PER;        // 使能页擦除
    FLASH->AR = page_addr;            // 设置要擦除的页地址
    FLASH->CR |= FLASH_CR_STRT;       // 启动擦除
    
    // 等待擦除完成
    while (FLASH->SR & FLASH_SR_BSY);
    
    // 检查擦除是否成功
    if (FLASH->SR & FLASH_SR_EOP) {
        FLASH->SR |= FLASH_SR_EOP;    // 清除EOP标志
        FLASH->CR &= ~FLASH_CR_PER;   // 关闭页擦除
        return 0;
    } else {
        FLASH->CR &= ~FLASH_CR_PER;   // 关闭页擦除
        return 1;
    }
}

/**
 * @brief  向Flash写入半字（2字节，STM32F1 Flash最小写入单位）
 * @param  addr: 写入地址（必须是2字节对齐）
 * @param  data: 要写入的半字数据
 * @retval 0: 写入成功, 1: 写入失败
 */
uint8_t Flash_WriteHalfWord(uint32_t addr, uint16_t data) {
    // 等待上一次操作完成
    while (FLASH->SR & FLASH_SR_BSY);
    
    // 使能编程（写入）
    FLASH->CR |= FLASH_CR_PG;
    
    // 写入半字（必须通过指针操作，直接赋值）
    *(uint16_t*)addr = data;
    
    // 等待写入完成
    while (FLASH->SR & FLASH_SR_BSY);
    
    // 检查写入是否成功
    if (FLASH->SR & FLASH_SR_EOP) {
        FLASH->SR |= FLASH_SR_EOP;    // 清除EOP标志
        FLASH->CR &= ~FLASH_CR_PG;    // 关闭编程
        return 0;
    } else {
        FLASH->CR &= ~FLASH_CR_PG;    // 关闭编程
        return 1;
    }
}

/**
 * @brief  将配平参数结构体写入Flash
 * @param  trim_params: 要存储的配平参数结构体指针
 * @retval 0: 存储成功, 1: 存储失败
 */
uint8_t QuadTrim_StoreToFlash(QuadrotorTrimParams *trim_params) {
    uint8_t ret = 0;
    uint32_t write_addr = FLASH_STORE_ADDR;
    uint16_t *data_ptr = (uint16_t*)trim_params;
    uint16_t data_len = sizeof(QuadrotorTrimParams) / 2;  // 按半字计算长度
    
    // 1. 解锁Flash
    Flash_Unlock();
    
    // 2. 擦除目标页（Flash写入前必须先擦除，擦除后所有位为1）
    ret = Flash_ErasePage(FLASH_STORE_ADDR);
    if (ret != 0) {
        Flash_Lock();
        return 1;
    }
    
    // 3. 逐半字写入结构体数据
    for (uint16_t i = 0; i < data_len; i++) {
        ret = Flash_WriteHalfWord(write_addr, data_ptr[i]);
        if (ret != 0) {
            Flash_Lock();
            return 1;
        }
        write_addr += 2;  // 地址+2（半字）
    }
    
    // 4. 锁定Flash
    Flash_Lock();
    
    return 0;
}

/**
 * @brief  从Flash读取配平参数结构体
 * @param  trim_params: 存储读取结果的结构体指针
 * @retval 0: 读取成功, 1: 读取失败（数据异常）
 */
uint8_t QuadTrim_LoadFromFlash(QuadrotorTrimParams *trim_params) {
    uint32_t read_addr = FLASH_STORE_ADDR;
    uint16_t *data_ptr = (uint16_t*)trim_params;
    uint16_t data_len = sizeof(QuadrotorTrimParams) / 2;
    
    // 逐半字读取数据到结构体
    for (uint16_t i = 0; i < data_len; i++) {
        data_ptr[i] = *(uint16_t*)read_addr;
        read_addr += 2;
    }
    
    // 简单校验：参数值是否在合理范围（可根据实际需求调整）
    if ((trim_params->roll_trim_angle < -1000 || trim_params->roll_trim_angle > 1000) ||
        (trim_params->pitch_trim_angle < -1000 || trim_params->pitch_trim_angle > 1000) ||
        (trim_params->yaw_trim_angle < -1000 || trim_params->yaw_trim_angle > 1000) ||
        (trim_params->motor_trim_throttle < -1000 || trim_params->motor_trim_throttle > 1000)) {
        // 数据超出合理范围，判定为读取失败
        return 1;
    }
    
    return 0;
}

void ADC_Filter(uint16_t adcbuf[4])
{
    for (int i = 0; i < 4; i++)
    {
        adc_filt[i] += ((int32_t)adcbuf[i] - adc_filt[i]) >> ADC_LPF_SHIFT;
    }
}

// 初始化函数
void initKeyScan(void)
{
    memset(&keys, 0, sizeof(KeyMatrix_t));
    memset(&tws, 0, sizeof(TWState_t));
    
    // 初始化列输出为高电平（假设高电平是不激活）
    HAL_GPIO_WritePin(KEY_IN1_GPIO_Port, KEY_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(KEY_IN2_GPIO_Port, KEY_IN2_Pin, GPIO_PIN_SET);
}

// 扫描矩阵键盘
void scanMatrixKeys(void)
{
    uint8_t rowPins[4];
    
    switch(keyScanState)
    {
        case KEY_SCAN_PHASE_COL1:
            // 扫描第一列：设置COL1为低电平，COL2为高电平
            HAL_GPIO_WritePin(KEY_IN1_GPIO_Port, KEY_IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(KEY_IN2_GPIO_Port, KEY_IN2_Pin, GPIO_PIN_SET);
            
            scanColumn = 0;  // 第一列对应按键0-3
            keyScanState = KEY_DEBOUNCE;
            scanTimer = 2;   // 2ms消抖时间
            break;
            
        case KEY_SCAN_PHASE_COL2:
            // 扫描第二列：设置COL1为高电平，COL2为低电平
            HAL_GPIO_WritePin(KEY_IN1_GPIO_Port, KEY_IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(KEY_IN2_GPIO_Port, KEY_IN2_Pin, GPIO_PIN_RESET);
            
            scanColumn = 4;  // 第二列对应按键4-7
            keyScanState = KEY_DEBOUNCE;
            scanTimer = 2;   // 2ms消抖时间
            break;
            
        case KEY_DEBOUNCE:
            if(scanTimer > 0) {
                scanTimer--;
                
                if(scanTimer == 0) {
                    // 读取行输入
                    uint8_t key1 = HAL_GPIO_ReadPin(KEY_OUT1_GPIO_Port, KEY_OUT1_Pin);
                    uint8_t key2 = HAL_GPIO_ReadPin(KEY_OUT2_GPIO_Port, KEY_OUT2_Pin);
                    uint8_t key3 = HAL_GPIO_ReadPin(KEY_OUT3_GPIO_Port, KEY_OUT3_Pin);
                    uint8_t key4 = HAL_GPIO_ReadPin(KEY_OUT4_GPIO_Port, KEY_OUT4_Pin);
                    
                    // 处理按键状态（假设高电平表示按下）
                    uint8_t rowStates[4] = {
                        (key1 == GPIO_PIN_RESET) ? 0 : 1,
                        (key2 == GPIO_PIN_RESET) ? 0 : 1,
                        (key3 == GPIO_PIN_RESET) ? 0 : 1,
                        (key4 == GPIO_PIN_RESET) ? 0 : 1
                    };
                    
                    // 更新当前列的所有按键状态
                    for(int i = 0; i < 4; i++) {
                        int keyIndex = scanColumn + i;
                        
                        // 更新当前状态
                        keys.current[keyIndex] = rowStates[i];
                        
                        // 消抖处理
                        if(keys.current[keyIndex] != keys.stable[keyIndex]) {
                            keys.debounce[keyIndex]++;
                            
                            // 消抖时间到（例如5个周期）
                            if(keys.debounce[keyIndex] >= 5) {
                                if(keys.current[keyIndex]) {
                                    keys.pressed[keyIndex] = 1;  // 按键按下事件
                                } else {
                                    keys.released[keyIndex] = 1; // 按键释放事件
                                }
                                keys.stable[keyIndex] = keys.current[keyIndex];
                                keys.debounce[keyIndex] = 0;
                            }
                        } else {
                            keys.debounce[keyIndex] = 0;
                        }
                    }
                    
                    // 转换到下一列
                    if(scanColumn == 0) {  // 刚扫描完第一列，切换到第二列
                        keyScanState = KEY_SCAN_PHASE_COL2;
                    } else {  // 刚扫描完第二列，切换回第一列
                        keyScanState = KEY_SCAN_PHASE_COL1;
                        
                        // 这里可以处理完成一轮扫描后的逻辑
                        // 例如：发送按键事件、更新显示等
                    }
                }
            }
            break;
    }
}

// 扫描TW输入
void scanTWInputs(void)
{
    // 直接读取TW引脚状态
    uint8_t twStates[8] = {
        HAL_GPIO_ReadPin(TW1_GPIO_Port, TW1_Pin),
        HAL_GPIO_ReadPin(TW2_GPIO_Port, TW2_Pin),
        HAL_GPIO_ReadPin(TW3_GPIO_Port, TW3_Pin),
        HAL_GPIO_ReadPin(TW4_GPIO_Port, TW4_Pin),
        HAL_GPIO_ReadPin(TW5_GPIO_Port, TW5_Pin),
        HAL_GPIO_ReadPin(TW6_GPIO_Port, TW6_Pin),
        HAL_GPIO_ReadPin(TW7_GPIO_Port, TW7_Pin),
        HAL_GPIO_ReadPin(TW8_GPIO_Port, TW8_Pin)
    };
    
    // 假设低电平有效
    for(int i = 0; i < 8; i++) {
        uint8_t currentState = (twStates[i] == GPIO_PIN_RESET) ? 1 : 0;
        
        // 更新当前状态
        tws.current[i] = currentState;
        
        // 消抖处理
        if(tws.current[i] != tws.stable[i]) {
            tws.debounce[i]++;
            
            // 消抖时间到
            if(tws.debounce[i] >= 5) {
                tws.changed[i] = 1;
                tws.stable[i] = tws.current[i];
                tws.debounce[i] = 0;
            }
        } else {
            tws.debounce[i] = 0;
        }
    }
}

// 获取按键状态函数（供外部调用）
uint8_t getKeyState(uint8_t keyIndex)
{
    if(keyIndex < 8) {
        return keys.stable[keyIndex];
    }
    return 0;
}

uint8_t isKeyPressed(uint8_t keyIndex)
{
    if(keyIndex < 8) {
        if(keys.pressed[keyIndex]) {
            keys.pressed[keyIndex] = 0;  // 清除事件
            return 1;
        }
    }
    return 0;
}

uint8_t isKeyReleased(uint8_t keyIndex)
{
    if(keyIndex < 8) {
        if(keys.released[keyIndex]) {
            keys.released[keyIndex] = 0;  // 清除事件
            return 1;
        }
    }
    return 0;
}

// 获取TW状态函数
uint8_t getTWState(uint8_t twIndex)
{
    if(twIndex < 8) {
        return tws.stable[twIndex];
    }
    return 0;
}

uint8_t isTWChanged(uint8_t twIndex)
{
    if(twIndex < 8) {
        if(tws.changed[twIndex]) {
            tws.changed[twIndex] = 0;  // 清除事件
            return 1;
        }
    }
    return 0;
}
uint8_t isTWPressed(uint8_t twIndex)
{
    if(twIndex < 8 && tws.changed[twIndex]) {
        // 只有状态从 0(释放) 变 1(按下) 时，才判定为“单次按下”
        if(tws.stable[twIndex] == 1) {
            tws.changed[twIndex] = 0; // 清除事件标志，避免重复触发
            return 1; // 返回1表示检测到单次按下
        }
        // 如果是从 1 变 0（释放），仅清除标志，不返回按下
        tws.changed[twIndex] = 0;
    }
    return 0; // 无按下事件
}

int16_t RC_Map(int16_t input, const rc_map_t *map)
{
    int32_t out;

    /* 输入限幅 */
    if (input < map->in_min)
        input = map->in_min;
    else if (input > map->in_max)
        input = map->in_max;

    /* 线性映射（用 int32 防溢出） */
    out = (int32_t)(input - map->in_min) *
          (map->out_max - map->out_min) /
          (map->in_max - map->in_min) +
          map->out_min;

    return (int16_t)out;
}

// 更新后的任务函数
void updateStateTask(void *param) 
{
	
		HeapStats_t xHeapStats;
	
		static uint32_t stack_watermark;
    uint32_t lastWakeTime = xTaskGetTickCount();
    static bool last_motor_lock;
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcbuf, 4);
	
	
    // 初始化键盘扫描
    initKeyScan();
    rc_ctrl_t rc_ctrl;
		control_command_t	con;
		
		//从Flash读取参数
		uint8_t ret = 1;
		QuadrotorTrimParams trim_data;
		//ret = QuadTrim_LoadFromFlash(&trim_data);
		if (ret == 0) {
				// 读取成功，trim_data中为Flash存储的参数
		} else {
				// 读取失败，使用默认值
				trim_data.roll_trim_angle = 0;
				trim_data.pitch_trim_angle = 0;
				trim_data.yaw_trim_angle = -0;
				trim_data.motor_trim_throttle = 0;
		}
		trim_data.roll_trim_angle = 40;
		trim_data.pitch_trim_angle = 40;
		trim_data.yaw_trim_angle = 111;
    while(1) 
    {
			vPortGetHeapStats(&xHeapStats);
        // 扫描矩阵键盘（每1ms执行一次）
        scanMatrixKeys();
        
				if(HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port,POWER_KEY_Pin) == GPIO_PIN_SET)
				{
					HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
				}
				
				if(isKeyPressed(0) == 1)
				{
					trim_data.yaw_trim_angle+=5;
				}
				else if(isKeyPressed(1) == 1)
				{
					trim_data.yaw_trim_angle-=5;
				}
				else if(isKeyPressed(2) == 1)
				{
					trim_data.motor_trim_throttle-=1;
				}
				else if(isKeyPressed(3) == 1)
				{
					trim_data.motor_trim_throttle+=1;
				}
				else if(isKeyPressed(4) == 1)
				{
					trim_data.roll_trim_angle+=20;
				}
				else if(isKeyPressed(5) == 1)
				{
					trim_data.roll_trim_angle-=20;
				}
				else if(isKeyPressed(6) == 1)
				{
					trim_data.pitch_trim_angle-=20;
				}
				else if(isKeyPressed(7) == 1)
				{
					trim_data.pitch_trim_angle+=20;
				}
        // 每5ms扫描一次TW输入（减少CPU占用）
        static uint32_t twScanCounter = 0;
        if(++twScanCounter >= 5) {
            twScanCounter = 0;
            scanTWInputs();
        }

				if(tws.stable[2]==1 && tws.stable[7]==1)
				{
					//解锁
					con.motor_lock = true;
				}
				else if(tws.stable[3]==1 && tws.stable[6]==1)
				{
					//闭锁
					con.motor_lock = false;
				}
				if(con.motor_lock != last_motor_lock)
				{
					last_motor_lock = con.motor_lock;
					QuadTrim_StoreToFlash(&trim_data);
					xQueueOverwrite(controlcommandDataQueue, &con);
				}
				
				//if(tws.changed[0] == 0x01 ||tws.changed[1] == 0x01 ||tws.changed[4] == 0x01 ||tws.changed[5] == 0x01)
				//if(getTWState(0) == 0 || getTWState(1) == 0 ||getTWState(4) == 0 ||getTWState(5) == 0 )
				{
					rc_control_command_t rc_control;
					if(isTWPressed(0) == 0x01)
					{
						trim_data.yaw_trim_angle -= RC_Map(adc_filt[0], &RC_YAW_MAP);
						rc_control.re = true;
						xQueueOverwrite(rccontrolDataQueue, &rc_control);
					}
					else 
					{
						rc_control.re = false;
					}
					if(isTWPressed(1) == 0x01)
					{
						rc_control.right = true;
						trim_data.yaw_trim_angle = 111;
						xQueueOverwrite(rccontrolDataQueue, &rc_control);
					}
					else 
					{
						rc_control.right = false;
					}
					if(isTWPressed(4) == 0x01)
					{
						rc_control.up = true;
						xQueueOverwrite(rccontrolDataQueue, &rc_control);
					}
					else 
					{
						rc_control.up = false;
					}
					if(isTWPressed(5) == 0x01)
					{
						rc_control.down = true;
						xQueueOverwrite(rccontrolDataQueue, &rc_control);
					}
					else 
					{
						rc_control.down = false;
					}
					//xQueueOverwrite(rccontrolDataQueue, &rc_control);
				}
        // 其他处理...
        ADC_Filter(adcbuf);
				rc_ctrl.ctrl_yawdps = (-1*RC_Map(adc_filt[0], &RC_YAW_MAP)) + trim_data.yaw_trim_angle;
				rc_ctrl.ctrl_thr = RC_Map(adc_filt[1], &RC_THR_MAP) + trim_data.motor_trim_throttle;
				rc_ctrl.ctrl_rol = (-1*RC_Map(adc_filt[2], &RC_ROL_MAP)) + trim_data.roll_trim_angle;
				rc_ctrl.ctrl_pit = RC_Map(adc_filt[3], &RC_PIT_MAP) + trim_data.pitch_trim_angle;
				xQueueOverwrite(rcctrlDataQueue, &rc_ctrl);
				stack_watermark = uxTaskGetStackHighWaterMark(NULL); // NULL 表示当前任务
        vTaskDelayUntil(&lastWakeTime, 5); /*1ms周期延时*/
    }
}