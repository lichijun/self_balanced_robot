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
  * @��  ��  V2.0
  * @��  ��  2022-7-26
  * @��  ��  SBUS��ģң������������ 
  *
  ******************************************************************************
  */

#include "ax_sbus.h"
#include <stdio.h>
#include "ax_robot.h"

//SBUS����֡�궨��
#define SBUS_RECV_MAX    25    //SBUS���յ������
#define SBUS_START       0x0F  //SBUS��ʼ����
#define SBUS_END         0x00  //SBUS��������


static uint8_t  uart_sbus_rx_ok = 0;      //���ճɹ���־
static uint8_t  uart_sbus_rx_con=0;       //���ռ�����
static uint8_t  uart_sbus_rx_buf[40];     //���ջ��壬��������С�ڵ���32Byte

//�����ж�
static uint16_t sbus_buf[2];             //ͨ������

/**
  * @��  ��  SBUS���ڳ�ʼ��
  * @��  ��  ��
  * @����ֵ	 ��
  */
void AX_SBUS_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//**USART����******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //�򿪴���GPIO��ʱ��

	//��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  //�򿪴��������ʱ��

	//����USART����
	USART_InitStructure.USART_BaudRate = 100000;		//������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_Init(USART3, &USART_InitStructure);

	//����USARTΪ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //�������ȼ�	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);//��ʼ������NVIC

	//ʹ�ܴ��ڽ����ж�
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

	//ʹ�� USART�� �������
	USART_Cmd(USART3, ENABLE);
}

/**
  * @��  ��  �����жϷ������
  * @��  ��  ��
  * @����ֵ  ��
  */
void USART3_IRQHandler(void)                	
{
	uint8_t Res;
	
	//�����ж�
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{
		Res =USART_ReceiveData(USART3);	
		
		if(uart_sbus_rx_con == 0)
		{
			//������ݿ�ʼ�ֽ�
			if(Res == SBUS_START)
			{
				uart_sbus_rx_buf[uart_sbus_rx_con] = Res;
				
				uart_sbus_rx_con++; 
			}			
		}
		else
		{
			//������ݽ����ֽ�
			if((uart_sbus_rx_con >= (SBUS_RECV_MAX-1)) && (Res ==  SBUS_END))
			{
				//һ֡���ݽ������
				uart_sbus_rx_ok = 1;
				
				if(ax_control_mode != CTL_RMS)
				{
					//��ȡͨ��4��ͨ��8����
					sbus_buf[0] = ((int16_t)uart_sbus_rx_buf[ 3] >> 6 | ((int16_t)uart_sbus_rx_buf[ 4] << 2 )  | (int16_t)uart_sbus_rx_buf[ 5] << 10 ) & 0x07FF;
					sbus_buf[1] = ((int16_t)uart_sbus_rx_buf[10] >> 5 | ((int16_t)uart_sbus_rx_buf[11] << 3 )) & 0x07FF;
					
					//�Ҳ�ҡ�ˣ�ͨ��2�������϶ˣ�����SWD-8���أ�ͨ��8���������¶ˣ����뺽ģң��������ģʽ
					if((sbus_buf[0] > 1500) && (sbus_buf[1] > 1500))
					{
						//�л�����ģң����SBUS����
						ax_control_mode = CTL_RMS;
						
						//ִ�з�����������ʾ
						ax_beep_ring = BEEP_SHORT;
					}					
				}
				
				//��λ���ռ�����
				uart_sbus_rx_con = 0;
			}
			else
			{
				uart_sbus_rx_buf[uart_sbus_rx_con] = Res;
				
				uart_sbus_rx_con++; 
			}
		}			
	}
}

/**
  * @��  ��  ��ȡ���յ�����
  * @��  ��  *pbuf��ͨ������
  * @����ֵ	 0-�����ݽ��գ�1-������
  */
uint8_t AX_SBUS_GetRxData(uint16_t *pbuf)
{

	if(uart_sbus_rx_ok != 0)
	{
		//����ң����ͨ������
		*(pbuf+0) = ((int16_t)uart_sbus_rx_buf[ 1] >> 0 | ((int16_t)uart_sbus_rx_buf[ 2] << 8 )) & 0x07FF;
		*(pbuf+1) = ((int16_t)uart_sbus_rx_buf[ 2] >> 3 | ((int16_t)uart_sbus_rx_buf[ 3] << 5 )) & 0x07FF;
		*(pbuf+2) = ((int16_t)uart_sbus_rx_buf[ 3] >> 6 | ((int16_t)uart_sbus_rx_buf[ 4] << 2 )  | (int16_t)uart_sbus_rx_buf[ 5] << 10 ) & 0x07FF;
		*(pbuf+3) = ((int16_t)uart_sbus_rx_buf[ 5] >> 1 | ((int16_t)uart_sbus_rx_buf[ 6] << 7 )) & 0x07FF;
		*(pbuf+4) = ((int16_t)uart_sbus_rx_buf[ 6] >> 4 | ((int16_t)uart_sbus_rx_buf[ 7] << 4 )) & 0x07FF;
		*(pbuf+5) = ((int16_t)uart_sbus_rx_buf[ 7] >> 7 | ((int16_t)uart_sbus_rx_buf[ 8] << 1 )  | (int16_t)uart_sbus_rx_buf[9] <<  9 ) & 0x07FF;
		*(pbuf+6) = ((int16_t)uart_sbus_rx_buf[9] >> 2 | ((int16_t)uart_sbus_rx_buf[10] << 6 )) & 0x07FF;
		*(pbuf+7) = ((int16_t)uart_sbus_rx_buf[10] >> 5 | ((int16_t)uart_sbus_rx_buf[11] << 3 )) & 0x07FF;
		
//		*(pbuf+8) = ((int16_t)uart_sbus_rx_buf[12] << 0 | ((int16_t)uart_sbus_rx_buf[13] << 8 )) & 0x07FF;
//		*(pbuf+9) = ((int16_t)uart_sbus_rx_buf[13] >> 3 | ((int16_t)uart_sbus_rx_buf[14] << 5 )) & 0x07FF;
//		*(pbuf+10) = ((int16_t)uart_sbus_rx_buf[14] >> 6 | ((int16_t)uart_sbus_rx_buf[15] << 2 )  | (int16_t)uart_sbus_rx_buf[16] << 10 ) & 0x07FF;
//		*(pbuf+11) = ((int16_t)uart_sbus_rx_buf[16] >> 1 | ((int16_t)uart_sbus_rx_buf[17] << 7 )) & 0x07FF;
//		*(pbuf+12) = ((int16_t)uart_sbus_rx_buf[17] >> 4 | ((int16_t)uart_sbus_rx_buf[18] << 4 )) & 0x07FF;
//		*(pbuf+13) = ((int16_t)uart_sbus_rx_buf[18] >> 7 | ((int16_t)uart_sbus_rx_buf[19] << 1 )  | (int16_t)uart_sbus_rx_buf[20] <<  9 ) & 0x07FF;
//		*(pbuf+14) = ((int16_t)uart_sbus_rx_buf[20] >> 2 | ((int16_t)uart_sbus_rx_buf[21] << 6 )) & 0x07FF;
//		*(pbuf+15) = ((int16_t)uart_sbus_rx_buf[21] >> 5 | ((int16_t)uart_sbus_rx_buf[22] << 3 )) & 0x07FF;
		
		//��λ��־λ
		uart_sbus_rx_ok = 0;
		return 1;
	}
	else
	{
		return 0;
	}	
}

/******************* (C) ��Ȩ 2023 XTARK **************************************/
