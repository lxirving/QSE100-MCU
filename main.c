/******************************************************************************
 *                  Shanghai ChipON Micro-Electronic Co.,Ltd                  *
 ******************************************************************************
 *  $File Name$       : main.c                                         	      *
 *  $Author$          : ChipON AE/FAE Group                                   *
 *  $Data$            : 2021-07-8                                             *
 *  $Hard Version     : KF32A156-MINI-EVB_V1.2                                *
 *  $Description$     : This file provides a reference for interrupt usart    *
 *                    asynchronous transceiver application routines		      *
 ******************************************************************************
 *  Copyright (C) by Shanghai ChipON Micro-Electronic Co.,Ltd                 *
 *  All rights reserved.                                                      *
 *                                                                            *
 *  This software is copyright protected and proprietary to                    *
 *  Shanghai ChipON Micro-Electronic Co.,Ltd.                                 *
 ******************************************************************************
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  *
 *                     		REVISON HISTORY                               	  *
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  *
 *  Data       Version  Author        Description                             *
 *  ~~~~~~~~~~ ~~~~~~~~ ~~~~~~~~~~~~  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  *
 *  2021-07-08 00.02.00 FAE Group     Version 2.0 update                      *
 *                                                                            *
 *                                                                            *
 *****************************************************************************/

/******************************************************************************
*                        		Include Files                                 *
******************************************************************************/
#include "system_init.h"
#include "Usart.h"
#include "Usart_Config.h"

#include <stdio.h>
#include <string.h>

/******************************************************************************
*                 	      Static function declaration                         *
******************************************************************************/
static void Led_Gpio_Init(void);
static void Led_Flip(void);

/******************************************************************************
*                           Array Definition                                  *
******************************************************************************/
//uint8_t  Usart_Send_Sdu[] = {"ChipON\r\n"};
/* Usart send data array and receive data array */
uint8_t UsartTxBuffer[100];
uint8_t Usart0RxBuffer[100];
uint8_t Usart1RxBuffer[100];
uint8_t Usart2RxBuffer[100];

extern volatile uint32_t TransmitCount ;
extern volatile uint32_t UART2Count ;
extern volatile uint32_t UART1Count ;
/******************************************************************************
*                        	Initialization function                           *
******************************************************************************/
/**
 *  @brief :Initializes the GPIO of the LED
 *  @param in :None
 *  @param out :None
 *  @retval :PD12--LED1
 *  		 PH3 --LED2
 *  		 PA4 --LED3
 *  		 PF7 --LED4
 */
static void Led_Gpio_Init(void)
{
	/*Configure the GPIO PA4/PD12/PF7/PH3 as IO port output mode */
	GPIO_Write_Mode_Bits(GPIOA_SFR, GPIO_PIN_MASK_4 , GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOD_SFR, GPIO_PIN_MASK_12 , GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_7 , GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOH_SFR, GPIO_PIN_MASK_3 , GPIO_MODE_OUT);
}

/**
 *  @brief :Led flip
 *  @param in :None
 *  @param out :None
 *  @retval :PD12--LED1
 *  		 PH3 --LED2
 *  		 PA4 --LED3
 *  		 PF7 --LED4
 */
static void Led_Flip(void)
{
	/*Configure the GPIO port PA4/PD12/PF7/PH3 to specify the output data of the pins to be inverted*/
	GPIO_Toggle_Output_Data_Config(GPIOA_SFR, GPIO_PIN_MASK_4);
	GPIO_Toggle_Output_Data_Config(GPIOD_SFR, GPIO_PIN_MASK_12);
	GPIO_Toggle_Output_Data_Config(GPIOF_SFR, GPIO_PIN_MASK_7);
	GPIO_Toggle_Output_Data_Config(GPIOH_SFR, GPIO_PIN_MASK_3);
}

int32_t  qse_flag = 0;

/*****************************************************************************
*                              	main function                                *
******************************************************************************/
int main()
{
	/* Initialize the system clock */
	SystemInit(120);
	/* Setup SysTick Timer as delay function */
	systick_delay_init(120);
	/*
	 * Initialize led IOs
	 * Configure the GPIO PA4/PD12/PF7/PH3 as IO port output mode
	 */
	Led_Gpio_Init();
	/*
	 *  Set Usart Tx and Rx PINS
	 *  PA3 -- USART0_TX0
	 *      - Configure PA3 remap mode
     *      - Push–pull output
	 *  PE7 -- USART0_RX
	 *      - Configure PE7 remap mode
	 *      - Configured in pull-up mode
	 */
	Usart_Gpio_Init();
	Usart_Gpio_Init1();
	Usart_Gpio_Init2();

	/*
	 * Set Usart0 working mode :
	 *    - Set Usart0 to async mode
	 *    - Set baudrate to 115200
	 *    - Set Usart to send enable
	 */
	USART_Mode_Config(USART0_SFR);
	USART_Mode_Config(USART1_SFR);
	USART_Mode_Config(USART2_SFR);

	/* Set Usart intterrupt
	 *   - Resetting USARTx sends the BUF interrupt bit
	 *   - SET USARTx RDR interrupt enable
	 *   - SET USARTx TFE interrupt enable
	 *   - Interrupt enables a peripheral or kernel interrupt vector
	 *   - Global shielded interrupt enable bit
	 */
	USART_Int_Config(USART0_SFR,INT_USART0);
	USART_Int_Config(USART1_SFR,INT_USART1);
	USART_Int_Config(USART2_SFR,INT_USART2);

	/* Configure interrupt priority group, default is 3VS1 */
	INT_Priority_Group_Config(INT_PRIORITY_GROUP_3VS1);
	/* Global shielded interrupt enable bit */
	INT_All_Enable(TRUE);

	systick_delay_ms(250);
	qse100_init();   //给qse100发初始化命令报文80E1
	systick_delay_ms(250);

	/* Sends a string of characters and light up the led */
//  USART_Send(USART0_SFR, Usart_Send_Sdu, sizeof(Usart_Send_Sdu));

//	Led_Flip();
//	qse100_init();//MCU给qse100发初始化80E1



	while(1)
	{

//
//		if (UART2Count)
//				{
//
//					/* Copy data from UsartRxBuffer to TransmitCount */
//					memcpy(UsartTxBuffer, UsartRxBuffer, UART2Count);
//					USART_Send(USART0_SFR, UsartTxBuffer, UART2Count);
//				//	USART_Send(USART0_SFR, Usart_Send_Sdu, sizeof(Usart_Send_Sdu));
//
//				//	USART_Send(USART1_SFR, UsartTxBuffer, TransmitCount);
//				//	USART_Send(USART1_SFR, Usart_Send_Sdu, sizeof(Usart_Send_Sdu));
//
//					/* Clear TransmitCount */
//					UART2Count = 0;
//					//	Led_Flip();
//					systick_delay_ms(250);
//
//
//				}
//        if(qse_flag == 1)
//			{
//        	    printf("-----");
//        		prepare_session_key(); //给qse100发准备会话密钥命令报文80D0
//        		printf("-----");
//        		qse_flag = 0;
//
//			}

	}		
}
