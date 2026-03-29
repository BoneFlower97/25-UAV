/*!
 * \file      sx126x-board.h
 *
 * \brief     Target board SX126x driver implementation
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
#ifndef __SX126x_BOARD_H__
#define __SX126x_BOARD_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "sx126x.h"

//进入和退出临界区保护方法
#if HAVE_OS
	#define CRITICAL_SECTION_BEGIN( )	vPortEnterCritical()
	#define CRITICAL_SECTION_END( )	vPortExitCritical()
#else
	#define CRITICAL_SECTION_BEGIN( )
	#define CRITICAL_SECTION_END( )
#endif

/*!
 * \brief Initializes the radio I/Os pins interface
 */
void SX126xIoInit( void );

/*!
 * \brief Initializes DIO IRQ handlers
 *
 * \param [IN] irqHandlers Array containing the IRQ callback functions
 */
void SX126xIoIrqInit( DioIrqHandler dioIrq );

/*!
 * \brief De-initializes the radio I/Os pins interface.
 *
 * \remark Useful when going in MCU low power modes
 */
void SX126xIoDeInit( void );

/*!
 * \brief Initializes the radio debug pins.
 */
void SX126xIoDbgInit( void );

/*!
 * \brief HW Reset of the radio
 */
void SX126xReset( void );

/*!
 * \brief Blocking loop to wait while the Busy pin in high
 */
void SX126xWaitOnBusy( void );

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool SX126xCheckRfFrequency( uint32_t frequency );

void SX126xDelayMs(uint32_t ms);

/*
 * 定时器初始化 
 */
void SX126xTimerInit(void);
void SX126xSetTxTimerValue(uint32_t nMs);
void SX126xTxTimerStart(void);
void SX126xTxTimerStop(void);
void SX126xSetRxTimerValue(uint32_t nMs);
void SX126xRxTimerStart(void);
void SX126xRxTimerStop(void);

void SX126xSetNss(uint8_t lev );
uint8_t SX126xSpiInOut(uint8_t data);

/*!
 * Radio hardware and global parameters
 */
extern SX126x_t SX126x;

#ifdef __cplusplus
}
#endif

#endif // __SX126x_BOARD_H__
