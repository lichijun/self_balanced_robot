/**			                                                    
		   ____                    _____ _______ _____       @���˴���
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   �����˿�����
									 
  ****************************************************************************** 
  *           
  * ��Ȩ���У� @���˴���  ��Ȩ���У�����ؾ�
  * ��˾��վ�� www.xtark.cn   www.tarkbot.com
  * �Ա����̣� https://xtark.taobao.com  
  * ����΢�ţ� ���˴��£���ע���ںţ���ȡ���¸�����Ѷ��
  *      
  ******************************************************************************
  * @��  ��  Musk Han@XTARK
  * @��  ��  LED�ƿ���
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_LED_H
#define __AX_LED_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//�ӿں���
void AX_LED_Init(void);

#define AX_LED_Red_Off()  	     GPIO_SetBits(GPIOC, GPIO_Pin_4)      //LEDG��ɫϨ��
#define AX_LED_Red_On()		     GPIO_ResetBits(GPIOC, GPIO_Pin_4)    //LEDG��ɫ����
#define AX_LED_Red_Toggle()      GPIO_WriteBit(GPIOC, GPIO_Pin_4, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)))	//LEDG��ɫ״̬��ת

#define AX_LED_Green_Off()  	 GPIO_SetBits(GPIOA, GPIO_Pin_8)      //LEDG��ɫϨ��
#define AX_LED_Green_On()		 GPIO_ResetBits(GPIOA, GPIO_Pin_8)    //LEDG��ɫ����
#define AX_LED_Green_Toggle()    GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)))	//LEDG��ɫ״̬��ת

#define AX_LED_Off()  	     GPIO_SetBits(GPIOC, GPIO_Pin_12)      //LEDG��ɫϨ��
#define AX_LED_On()		     GPIO_ResetBits(GPIOC, GPIO_Pin_12)    //LEDG��ɫ����

#endif 

/******************* (C) ��Ȩ 2023 XTARK **************************************/
