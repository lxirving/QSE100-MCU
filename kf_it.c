/******************************************************************************
 *                  Shanghai ChipON Micro-Electronic Co.,Ltd                  *
 ******************************************************************************
 *  $File Name$       : kf_it.c                                        	      *
 *  $Project Name$	  : KF32A156_USART										  *
 *  $Author$          : ChipON AE/FAE Group                                   *
 *  $Data$            : 2021-07-20- 18:04:36                                  *
 *  $Hard Version     : KF32A156-MINI-EVB_V1.2                                *
 *  $Description$     : This file provides a reference for interrupt usart    *
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
#include"system_init.h"
#include "Usart.h"
#include <stdio.h>
#include <string.h>

#include "Can.h"
#include "Task.h"

//*****************************************************************************************
//                                 NMI Interrupt Course
//*****************************************************************************************	
void __attribute__((interrupt)) _NMI_exception (void)
{	

}

//*****************************************************************************************
//                               HardFault Interrupt Course
//*****************************************************************************************	

void __attribute__((interrupt)) _HardFault_exception (void)
{

}

//*****************************************************************************************
//                               StackFault Interrupt Course
//*****************************************************************************************	
void __attribute__((interrupt)) _StackFault_exception (void)
{

}

//*****************************************************************************************
//                               SVC Interrupt Course
//*****************************************************************************************	
void __attribute__((interrupt)) _SVC_exception (void)
{

}

//*****************************************************************************************
//                              SoftSV Interrupt Course
//*****************************************************************************************	
void __attribute__((interrupt)) _SoftSV_exception (void)
{

}

//*****************************************************************************************
//                              SysTick Interrupt Course
//*****************************************************************************************	
void __attribute__((interrupt)) _SysTick_exception (void)
{
	
}

//*****************************************************************************************
//                               USART0 Interrupt Course
//*****************************************************************************************
/* Usart0 receive data count */
volatile uint32_t TransmitCount = 0;
volatile uint32_t UART2Count = 0;
volatile uint32_t UART1Count = 0;
extern uint8_t UsartTxBuffer[];
extern uint8_t Usart0RxBuffer[];
extern uint8_t Usart1RxBuffer[];
extern uint8_t Usart2RxBuffer[];

extern int32_t  qse_flag ;
//volatile uint8_t index = 0 ;

Can_Pdu_TypeDef Receive_Can_Pdu;
struct
{
	uint8_t  Msg_Rcvd_Flag[3];          //to flag the receiving of ID 0x0101 0x0102 0x0103
	Usart_Sdu_Type  Usart_Send_Sdu[3];	//to storage the Msg 0x0101 0x0102 0x0103
}Usart_Send_Struct;

//uint8_t Full_Car_Msg[24] = {0};

typedef union
{
 struct
 {
  unsigned char low_byte;
  unsigned char mlow_byte;
  unsigned char mhigh_byte;
  unsigned char high_byte;
 }FLOAT_BYTE;

 unsigned char float_byte[4];

 float  value;
}FLAOT_UNION;


typedef struct
{
	uint8_t  Msg_Rcvd_Flag[3];
	uint8_t Full_Car_Msg[24];
	struct UART_SEND_CARDATA
	{
		uint8_t CarData[60];
		uint8_t DataLength;
	}Uart_Send_CarData;
	struct CAR_DATA
	{
		float Xaxle_V;
		float Yaxle_V;
		float Zaxle_V;
		float Xaxle_A;
		float Yaxle_A;
		float Zaxle_A;
		float X_Angular;
		float Y_Angular;
		float Z_Angular;
		float Battery_Voltage;
	}Car_Data;
}Rcvd_Car_Data_Type;
Rcvd_Car_Data_Type RcvdCarData = {0};

void __attribute__((interrupt)) _USART0_exception (void)
{
	static uint32_t ReceiveCount = 0;
	printf("000");
	/*	If gets the status of the USART data ready interrupt flag */
	if(USART_Get_Receive_BUFR_Ready_Flag(USART0_SFR))
	{
		Usart0RxBuffer[ReceiveCount++] = USART_ReceiveData(USART0_SFR);

		//printf("%c",Usart0RxBuffer[ReceiveCount-1]);
	}
	/* If gets the USART interrupt flag for receiving idle frames */
	if (USART_Get_Receive_Frame_Idel_Flag(USART0_SFR))
	{
		/* Clear idle interrupt flag */
		USART_Clear_Idle_INT_Flag(USART0_SFR);
		/* Gets the length of the data received */
		TransmitCount = ReceiveCount;
		ReceiveCount = 0;
	}

	USART_Send(USART0_SFR, Usart0RxBuffer, TransmitCount);
	TransmitCount = 0;
}
void __attribute__((interrupt)) _USART1_exception (void)
{

	static uint32_t ReceiveCount = 0;
	/*	If gets the status of the USART data ready interrupt flag */

	if(USART_Get_Receive_BUFR_Ready_Flag(USART1_SFR))
	{
		Usart1RxBuffer[ReceiveCount++] = USART_ReceiveData(USART1_SFR);

	}


	/* If gets the USART interrupt flag for receiving idle frames */
	if (USART_Get_Receive_Frame_Idel_Flag(USART1_SFR))
	{
		/* Clear idle interrupt flag */
		USART_Clear_Idle_INT_Flag(USART1_SFR);
		/* Gets the length of the data received */
		UART1Count = ReceiveCount;
		ReceiveCount = 0;
	}

	//USART_Send(USART0_SFR, Usart1RxBuffer, UART1Count);
	//UART1Count = 0;

}




void __attribute__((interrupt)) _USART2_exception (void)
{
	//printf("222");
	static uint32_t ReceiveCount = 0;
	/*	If gets the status of the USART data ready interrupt flag */

	if(USART_Get_Receive_BUFR_Ready_Flag(USART2_SFR))
	{
		Usart2RxBuffer[ReceiveCount++] = USART_ReceiveData(USART2_SFR);

	}



	/* If gets the USART interrupt flag for receiving idle frames */
	if (USART_Get_Receive_Frame_Idel_Flag(USART2_SFR))
	{
		/* Clear idle interrupt flag */
		USART_Clear_Idle_INT_Flag(USART2_SFR);
		/* Gets the length of the data received */
		UART2Count = ReceiveCount;
		ReceiveCount = 0;



		/* 验证串口1可以收到qse100针对于初始化指令的 响应报文    A0 A1 A2 A3 00 02 90 00 00 90 A4 A5 A6 A7 */
		if(qse_flag == 0)
			{
			 if((Usart2RxBuffer[0]==0xA0)&&(Usart2RxBuffer[1]==0xA1)&&(Usart2RxBuffer[2]==0xA2)&&(Usart2RxBuffer[3]==0xA3)&&(Usart2RxBuffer[4]==0x00)&&(Usart2RxBuffer[5]==0x02)&&(Usart2RxBuffer[6]==0x90)&&(Usart2RxBuffer[7]==0x00)&&(Usart2RxBuffer[8]==0x00)&&(Usart2RxBuffer[9]==0x90)&&(Usart2RxBuffer[10]==0xA4)&&(Usart2RxBuffer[11]==0xA5)&&(Usart2RxBuffer[12]==0xA6)&&(Usart2RxBuffer[13]==0xA7))
				{
//					printf("QSE100_init success !\n");
					qse_flag++;
//					printf("qse_flag = %d\n", qse_flag);
//					USART_Send(USART0_SFR, Usart2RxBuffer, UART2Count);
					UART2Count = 0;
				}
			else
				{
//					printf("QSE100_init error !\n");
//					USART_Send(USART0_SFR, Usart2RxBuffer, UART2Count);
					UART2Count = 0;
				}
	        }
         if(qse_flag == 1)
			 {
        	    //printf("-----");
        		prepare_session_key();  //给qse100发准备会话密钥命令报文80D0
        		qse_flag++;

			 }
         if(qse_flag == 2)
         {
        	 if((Usart2RxBuffer[150]==0x90)&&(Usart2RxBuffer[151]==0x00))
        	 {
//        		 printf("QSE100_prepare_session_key success !\n");
        		 qse_flag++;
//        		 printf("qse_flag = %d\n", qse_flag);

        		 unsigned char KeyTag[128] ;
        		 memcpy(KeyTag, Usart2RxBuffer+6,128);
//        		 printf("KeyTag is : \r\n");
//        		 printf("----------");
//        		 USART_Send(USART0_SFR, KeyTag, 128);    //调试用


        		 unsigned char KeySN[4] ;
        		 memcpy(KeySN, Usart2RxBuffer+134,4);
//        		 printf("KeySN is : \r\n");
//        		 printf("----------");
//        		 USART_Send(USART0_SFR, KeySN, 4);       //调试用


        		 unsigned char KeyLen[4] ;
        		 memcpy(KeyLen, Usart2RxBuffer+138,4);
//        		 printf("KeyLen is : \r\n");
//        		 printf("----------");
//        		 USART_Send(USART0_SFR, KeyLen, 4);      //调试用

        		 USART_Send(USART0_SFR, Usart2RxBuffer+6, 136);   //---将Key_ID通过串口0发送给MPU的串口1
        	 }
         }
         if(qse_flag == 3)
         {
        	// if(index == 56)
        	 //{
        	 encrypt_ins(RcvdCarData.Uart_Send_CarData.CarData , 56);
        	 qse_flag ++;
        	// }
         }
         if(qse_flag == 4)
         {
            if((Usart2RxBuffer[72]==0x90)&&(Usart2RxBuffer[73]==0x00))
            {
//            	printf("encryption success !\n");
            	unsigned char encrypt_data[64] ;
            	memcpy(encrypt_data,Usart2RxBuffer+8,64);
//            	printf("encrypt_data is : \r\n");
//            	printf("----------");
//            	USART_Send(USART0_SFR, encrypt_data, 64);    //调试用
                USART_Send(USART0_SFR, encrypt_data, 64); 	 //----将encrypt_data通过串口0发送给MPU的串口1
            }
         }

	}

//	USART_Send(USART0_SFR, Usart2RxBuffer, UART2Count);
//	UART2Count = 0;


}


//void Send_Uart_Msg ()
//{
//	if(Usart_Send_Struct.Msg_Rcvd_Flag[0] == 1 && Usart_Send_Struct.Msg_Rcvd_Flag[1] == 1 && Usart_Send_Struct.Msg_Rcvd_Flag[2] == 1 )
//			{
//
//
//
//				//send usart data
//				USART_Send(USART0_SFR,&Usart_Send_Struct.Usart_Send_Sdu[0],1);
//				USART_Send(USART0_SFR,&Usart_Send_Struct.Usart_Send_Sdu[1],1);
//				USART_Send(USART0_SFR,&Usart_Send_Struct.Usart_Send_Sdu[2],1);
//
//				// clear flag
//				Usart_Send_Struct.Msg_Rcvd_Flag[0] = 0;
//				Usart_Send_Struct.Msg_Rcvd_Flag[1] = 0;
//				Usart_Send_Struct.Msg_Rcvd_Flag[2] = 0;
//
//			}
//}

void Send_Uart_Message ()
{
	if(RcvdCarData.Msg_Rcvd_Flag[0] ==1 && RcvdCarData.Msg_Rcvd_Flag[1] == 1 &&RcvdCarData.Msg_Rcvd_Flag[2] == 1)
	{
		//clear flag
		RcvdCarData.Msg_Rcvd_Flag[0] = 0;
		RcvdCarData.Msg_Rcvd_Flag[1] = 0;
		RcvdCarData.Msg_Rcvd_Flag[2] = 0;




		unsigned char temp1, temp2;
		unsigned short tempdata;
		temp1 = RcvdCarData.Full_Car_Msg[2];
		temp2 = RcvdCarData.Full_Car_Msg[3];
		tempdata = temp1 * 256 + temp2;
		short tem = tempdata;
		float temm = tem;
		RcvdCarData.Car_Data.Xaxle_V = tempdata / 1000 + (tempdata % 1000)*0.001;

		temp1 = RcvdCarData.Full_Car_Msg[4];
		temp2 = RcvdCarData.Full_Car_Msg[5];
		tempdata = temp1 * 256 + temp2;
		tem = tempdata;
		temm = tem;
		RcvdCarData.Car_Data.Yaxle_V = tempdata / 1000 + (tempdata % 1000)*0.001;

		temp1 = RcvdCarData.Full_Car_Msg[6];
		temp2 = RcvdCarData.Full_Car_Msg[7];
		tempdata = temp1 * 256 + temp2;
		tem = tempdata;
		temm = tem;
		RcvdCarData.Car_Data.Zaxle_V = tempdata / 1000 + (tempdata % 1000)*0.001;


		temp1 = RcvdCarData.Full_Car_Msg[8];
		temp2 = RcvdCarData.Full_Car_Msg[9];
		tempdata = temp1 * 256 + temp2;
		tem = tempdata;
		temm = tem;

		RcvdCarData.Car_Data.Xaxle_A = temm/ 1671.84f;

		temp1 = RcvdCarData.Full_Car_Msg[10];
		temp2 = RcvdCarData.Full_Car_Msg[11];
		tempdata = temp1 * 256 + temp2;
		tem = tempdata;
		temm = tem;
		RcvdCarData.Car_Data.Yaxle_A = temm/ 1671.84f;

		temp1 = RcvdCarData.Full_Car_Msg[12];
		temp2 = RcvdCarData.Full_Car_Msg[13];
		tempdata = temp1 * 256 + temp2;
		tem = tempdata;
		temm = tem;
		RcvdCarData.Car_Data.Zaxle_A = temm/ 1671.84f;

		temp1 = RcvdCarData.Full_Car_Msg[14];
		temp2 = RcvdCarData.Full_Car_Msg[15];
		tempdata = temp1 * 256 + temp2;
		tem = tempdata;
		temm = tem;
		RcvdCarData.Car_Data.X_Angular = temm* 0.00026644f;

		temp1 = RcvdCarData.Full_Car_Msg[16];
		temp2 = RcvdCarData.Full_Car_Msg[17];
		tempdata = temp1 * 256 + temp2;
		tem = tempdata;
		temm = tem;
		RcvdCarData.Car_Data.Y_Angular = temm*0.00026644f;

		temp1 = RcvdCarData.Full_Car_Msg[18];
		temp2 = RcvdCarData.Full_Car_Msg[19];
		tempdata = temp1 * 256 + temp2;
		tem = tempdata;
		temm = tem;
		RcvdCarData.Car_Data.Z_Angular = temm*0.00026644f;

		temp1 = RcvdCarData.Full_Car_Msg[20];
		temp2 = RcvdCarData.Full_Car_Msg[21];
		tempdata = temp1 * 256 + temp2;
		tem = tempdata;
		//temm = tem;
		RcvdCarData.Car_Data.Battery_Voltage = tem/1000 + (tem % 1000)*0.001;

		//RcvdCarData.Uart_Send_CarData.DataLength = 0;
		uint8_t index = 0 ;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0xFF;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0x00;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0x01;

		FLAOT_UNION FloatThansmiter = {0};
		FloatThansmiter.value = RcvdCarData.Car_Data.Xaxle_V;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0xA0;

		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[3];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[2];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[1];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[0];

		FloatThansmiter.value = RcvdCarData.Car_Data.Yaxle_V;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0xA1;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[3];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[2];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[1];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[0];

		FloatThansmiter.value = RcvdCarData.Car_Data.Zaxle_V;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0xA2;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[3];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[2];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[1];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[0];

		FloatThansmiter.value = RcvdCarData.Car_Data.Xaxle_A;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0xA3;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[3];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[2];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[1];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[0];

		FloatThansmiter.value = RcvdCarData.Car_Data.Yaxle_A;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0xA4;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[3];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[2];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[1];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[0];

		FloatThansmiter.value = RcvdCarData.Car_Data.Zaxle_A;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0xA5;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[3];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[2];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[1];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[0];

		FloatThansmiter.value = RcvdCarData.Car_Data.X_Angular;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0xA6;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[3];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[2];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[1];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[0];

		FloatThansmiter.value = RcvdCarData.Car_Data.Y_Angular;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0xA7;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[3];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[2];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[1];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[0];

		FloatThansmiter.value = RcvdCarData.Car_Data.Z_Angular;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0xA8;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[3];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[2];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[1];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[0];

		FloatThansmiter.value = RcvdCarData.Car_Data.Battery_Voltage;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0xA9;
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[3];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[2];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[1];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = FloatThansmiter.float_byte[0];


		union
		{
			uint16_t  value;
			uint8_t		data[2];
		}CRC_Type = {0};

		//CRC_Type.value = crc16(&RcvdCarData.Uart_Send_CarData.CarData,index);

		RcvdCarData.Uart_Send_CarData.CarData[index++] = CRC_Type.data[0];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = CRC_Type.data[1];
		RcvdCarData.Uart_Send_CarData.CarData[index++] = 0xFE;

		RcvdCarData.Uart_Send_CarData.CarData[1] = index;
		RcvdCarData.Uart_Send_CarData.DataLength = index;


//		 for (uint32_t Send_data_count = 0; Send_data_count < RcvdCarData.Uart_Send_CarData.DataLength; Send_data_count++)
//		  {
//		    USART_SendData(USART0_SFR, RcvdCarData.Uart_Send_CarData.CarData[Send_data_count]);
//		    while (!USART_Get_Transmitter_Empty_Flag(USART0_SFR))
//		      ;
//		  }
//
//		 for (uint32_t Send_data_count = 0; Send_data_count < 24; Send_data_count++)
//		 				  {
//		 				    USART_SendData(USART0_SFR, RcvdCarData.Full_Car_Msg[Send_data_count]);
//		 				    while (!USART_Get_Transmitter_Empty_Flag(USART0_SFR))
//		 				      ;
//		 				  }
	}

}

void __attribute__((interrupt)) _CAN4_exception (void)
{
	volatile uint32_t Can_Rcr = 0x00;

	uint8_t datatest[10] = {0};
	Usart_Sdu_Type  Usart_Send_Sdu = {{'H','F','U','T',' ','Y','Y','\r','\n'},10};

	if(CAN_Get_INT_Flag(CAN4_SFR,CAN_INT_ERROR_ALARM))
	{
		CAN_Clear_INT_Flag(CAN4_SFR,CAN_INT_ERROR_ALARM);
	}

	if(CAN_Get_INT_Flag(CAN4_SFR,CAN_INT_ERROR_NEGATIVE))
	{
	CAN_Clear_INT_Flag(CAN4_SFR,CAN_INT_ERROR_NEGATIVE);
	}

	if(CAN_Get_INT_Flag(CAN4_SFR,CAN_INT_TRANSMIT))
	{
	CAN_Clear_INT_Flag(CAN4_SFR,CAN_INT_TRANSMIT);
	}

	if(CAN_Get_INT_Flag(CAN4_SFR,CAN_INT_BUS_ERROR))
	{
		CAN_Clear_INT_Flag(CAN4_SFR,CAN_INT_BUS_ERROR);
		Can_Rcr = CAN4_SFR->RCR;
	}

	if(CAN_Get_INT_Flag(CAN4_SFR,CAN_INT_BUS_OFF))
	{
		CAN_Clear_INT_Flag(CAN4_SFR,CAN_INT_BUS_OFF);
		CAN4_SFR->CTLR &= ~(0x01);
	}

	if(CAN_Get_INT_Flag(CAN4_SFR,CAN_INT_RECEIVE))
	{

//		CAN_Clear_INT_Flag(CAN4_SFR,CAN_INT_RECEIVE);
//		CAN_Int_Config(&Can_Interrupt1);
//		/* CAN receive message */
		CAN_Receive_Message(&Receive_Can_Pdu);


		for (uint8_t i = 0 ; i < Receive_Can_Pdu.Frame_length ; i++)
		{
			Usart_Send_Sdu.Length  = Receive_Can_Pdu.CAN_Message[i].m_DataLength;//Receive_Can_Pdu.CAN_Message[0].m_DataLength;

			Usart_Send_Sdu.Data[0] = Receive_Can_Pdu.CAN_Message[i].m_Can_ID>>8;
			Usart_Send_Sdu.Data[1] = Receive_Can_Pdu.CAN_Message[i].m_Can_ID;

			for (uint8_t j = 0 ; j < Usart_Send_Sdu.Length ; j++)
			{
				Usart_Send_Sdu.Data[j + 2] = Receive_Can_Pdu.CAN_Message[i].m_Data[j];
			}
			Usart_Send_Sdu.Length += 2;


		//	USART_Send(USART0_SFR,&Usart_Send_Sdu);

			if(Usart_Send_Sdu.Data[0] == 0x01 && Usart_Send_Sdu.Data[1] == 0x01)
			{
				RcvdCarData.Msg_Rcvd_Flag[0] = 1;
				for(uint8_t i = 0; i<8 ; i++)
				{
					//RcvdCarData.Full_Car_Msg[i] = Usart_Send_Sdu[2+i];
					RcvdCarData.Full_Car_Msg[i] = Usart_Send_Sdu.Data[2+i];
				}


				Usart_Send_Struct.Msg_Rcvd_Flag[0] = 1;
				Usart_Send_Struct.Usart_Send_Sdu[0] = Usart_Send_Sdu;
			}
			else if(Usart_Send_Sdu.Data[0] == 0x01 && Usart_Send_Sdu.Data[1] == 0x02)
			{
				RcvdCarData.Msg_Rcvd_Flag[1] = 1;
				for(uint8_t i = 0; i<8 ; i++)
				{
					RcvdCarData.Full_Car_Msg[i+8] = Usart_Send_Sdu.Data[2+i];
				}


				Usart_Send_Struct.Msg_Rcvd_Flag[1] = 1;
				Usart_Send_Struct.Usart_Send_Sdu[1] = Usart_Send_Sdu;
			}
			else if(Usart_Send_Sdu.Data[0] == 0x01 && Usart_Send_Sdu.Data[1] == 0x03)
			{
				RcvdCarData.Msg_Rcvd_Flag[2] = 1;
				for(uint8_t i = 0; i<8 ; i++)
				{
					RcvdCarData.Full_Car_Msg[i+16] = Usart_Send_Sdu.Data[2+i];
				}


				Usart_Send_Struct.Msg_Rcvd_Flag[2] = 1;
				Usart_Send_Struct.Usart_Send_Sdu[2] = Usart_Send_Sdu;
			}


		}
		Send_Uart_Message();

		//Send_Uart_Msg (); //yz


        //CAN_Int_Config(&Can_Interrupt2);
	}
}


