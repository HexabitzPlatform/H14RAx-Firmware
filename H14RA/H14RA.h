/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved
 
 File Name     : H14RA.h
 Description   : Header file for module H14RA.
 (Description_of_module)

 (Description of Special module peripheral configuration):
 >>
 >>
 >>

 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H14RA_H
#define H14RA_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H14RA_MemoryMap.h"
#include "H14RA_uart.h"
#include "H14RA_gpio.h"
#include "H14RA_dma.h"
#include "H14RA_inputs.h"
#include "H14RA_eeprom.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H14RA

/* Port-related Definitions */
#define	NUM_OF_PORTS	6
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define Available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5
#define _P6

/* Define Available USARTs */
#define _USART1
#define _USART2
#define _USART3
#define _USART4
#define _USART5
#define _USART6

/* Port-UART mapping */
#define UART_P1 &huart4
#define UART_P2 &huart2
#define UART_P3 &huart6
#define UART_P4 &huart3
#define UART_P5 &huart1
#define UART_P6 &huart5

/* Module-specific Hardware Definitions ************************************/
/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

#define	USART6_TX_PIN		GPIO_PIN_4
#define	USART6_RX_PIN		GPIO_PIN_5
#define	USART6_TX_PORT		GPIOA
#define	USART6_RX_PORT		GPIOA
#define	USART6_AF			GPIO_AF3_USART6

/* Indicator LED */
#define _IND_LED_PORT		GPIOB
#define _IND_LED_PIN		GPIO_PIN_15

/* Module-specific Macro Definitions ***************************************/
#define NUM_MODULE_PARAMS		1
/*
 *  ESC_FPWM_CLK =  F_clk /((Period(ARR) + 1 ) * (Prescaler + 1))
 *  50 HZ = 64MHZ / ((Period + 1 ) * ( 6399 + 1 )) --> Period = 200 - 1 = 199‬
 */
#define TIMER_PRESCALER     ((uint16_t)6399)
/*Auto Reload Register(ARR)*/
#define TIMER_PERIOD        ((uint16_t)199)
/*Frequency of ESC(Electrical Speed Control)circuit for BLDC motors[HZ]*/
#define ESC_FPWM_CLK        ((uint8_t)50)
/*Period of ESC(Electrical Speed Control) circuit for BLDC motors[Second]*/
#define ESC_TPWM_PERIOD     (1.0f/ESC_FPWM_CLK)
/*This value corresponds to the MAX width of pulse = 2 ms*/
#define MAX_ESC_TIME_PERIOD ((uint8_t)2)
/*This value corresponds to the MIN width of pulse = 1 ms*/
#define MIN_ESC_TIME_PERIOD ((uint8_t)1)
/*This value of CCR(Capture Compare Register Timer) which corresponds to the MAX width of pulse,
 * and which will make the Motor run at the highest speed.
 * Duty cycle[%] = CCRx / ARR(Period) ---> CCRx(MAX) = Duty cycle[%] * ARR(Period) = MAX_ESC_TIME_PERIOD[ms]/ESC_TPWM_PERIOD[ms] * ARR(Period)
 **/
#define MAX_ESC_CCR_VALUE   round((MAX_ESC_TIME_PERIOD/(ESC_TPWM_PERIOD*1000))* TIMER_PERIOD)
/*This value of CCR(Capture Compare Register Timer) which corresponds to the MIN width of pulse,
 * and which will make the Motor run at the lowest speed.
 * Duty cycle[%] = CCRx / ARR(Period) ---> CCRx(MIN) = Duty cycle[%] * ARR(Period) = MIN_ESC_TIME_PERIOD[ms]/ESC_TPWM_PERIOD[ms] * ARR(Period)
 **/
#define MIN_ESC_CCR_VALUE   round((MIN_ESC_TIME_PERIOD/(ESC_TPWM_PERIOD*1000))* TIMER_PERIOD)
/*Numbers of Motors and output of PWM can be connected to this module*/
#define NUM_MOTORS          ((uint8_t) 6)
#define NUM_OUTS            ((uint8_t) 6)
#define MAX_DUTY_CYCLE      ((uint8_t)100)
#define MIN_DUTY_CYCLE      ((uint8_t)0)
/*htim timers handlers*/
#define TIMER_HANDLE_OUT1 htim1
#define TIMER_HANDLE_OUT2 htim15
#define TIMER_HANDLE_OUT3 htim3
#define TIMER_HANDLE_OUT4 htim2
#define TIMER_HANDLE_OUT5 htim3
#define TIMER_HANDLE_OUT6 htim4
/*Channels of the timers*/
#define TIMER_CHANAL_OUT1 TIM_CHANNEL_4
#define TIMER_CHANAL_OUT2 TIM_CHANNEL_1
#define TIMER_CHANAL_OUT3 TIM_CHANNEL_4
#define TIMER_CHANAL_OUT4 TIM_CHANNEL_1
#define TIMER_CHANAL_OUT5 TIM_CHANNEL_2
#define TIMER_CHANAL_OUT6 TIM_CHANNEL_4
/*CCR(Capture Compare Register) of the Timers*/
#define TIMER_CCR_OUT1 TIM1->CCR4
#define TIMER_CCR_OUT2 TIM15->CCR1
#define TIMER_CCR_OUT3 TIM3->CCR4
#define TIMER_CCR_OUT4 TIM2->CCR1
#define TIMER_CCR_OUT5 TIM3->CCR2
#define TIMER_CCR_OUT6 TIM4->CCR4
/*PINs and PORTs of the Timers*/
#define TIMER_OUT1_PIN  GPIO_PIN_11
#define TIMER_OUT1_PORT GPIOA
#define TIMER_OUT2_PIN  GPIO_PIN_14
#define TIMER_OUT2_PORT GPIOB
#define TIMER_OUT3_PIN  GPIO_PIN_1
#define TIMER_OUT3_PORT GPIOB
#define TIMER_OUT4_PIN  GPIO_PIN_15
#define TIMER_OUT4_PORT GPIOA
#define TIMER_OUT5_PIN  GPIO_PIN_5
#define TIMER_OUT5_PORT GPIOB
#define TIMER_OUT6_PIN  GPIO_PIN_9
#define TIMER_OUT6_PORT GPIOB
/* Module-specific Enumeration Definitions *********************************/
typedef enum {
    MOTOR_1 = 0,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4,
    MOTOR_5,
    MOTOR_6
} Motor_t;
typedef enum {
    OUT_1 = 0,
	OUT_2,
	OUT_3,
	OUT_4,
	OUT_5,
	OUT_6
} ChannelOut_t;
/* Module-specific Type Definition *****************************************/
/* Module-status Type Definition */
typedef enum {
	H14RA_OK = 0,
	H14RA_ERR_UNKNOWNMESSAGE,
	H14RA_ERR_WRONGPARAMS,
	H14RA_ERR_INVALID_MOTOR,
	H14RA_ERR_INVALID_OUT_CHANNEL,
	H14RA_ERR_INVALID_FREQ,
	H14RA_ERROR = 255
} Module_Status;

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/


#endif /* H14RA_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
