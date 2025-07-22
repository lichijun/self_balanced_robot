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
  * @˵  ��
  *
  * 
  ******************************************************************************
  */

#include "ax_led.h"

/**
  * @��  ��  LED ��ʼ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_LED_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//GPIO����
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//�ر�LED��
	GPIO_SetBits(GPIOC,GPIO_Pin_4);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);	
}

/******************* (C) ��Ȩ 2023 XTARK **************************************/
