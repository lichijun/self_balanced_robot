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
  * @��  ��  Ԥ������ͨ��
  *
  ******************************************************************************
  * @˵  ��
  *
  * 1.����ʹ�þ���X-ProtocolЭ��������ݷ���
  * 2.����UART�Ĵ������ݽ��չ��ܣ�ʹ���жϷ�ʽ��X-ProtocolЭ��ͨ��
  *
  * X-ProtocolЭ����ܣ���֡����
  * ֡���壺AA 55 | 0B  | 01  | 03 E8 FC 18 00 0A | 14
  *        ֡ͷ   ֡��   ֡��  ����                У���
  * ֡  ͷ��˫֡ͷ��������ǿ
  * ֡  �����������ݳ����趨
  * ֡  �룺�û����ݹ����趨����ʶ֡��Ψһ��
  * ��  �ݣ���λ��ǰ�����ȿɱ䣬�����������8λ��16λ��32λ����
  * У��ͣ�ǰ�������ۼӺ͵ĵ�8λ
  * ֡ʾ����( AA 55 0B 01 03 E8 FC 18 00 0A 14 ) ���ݣ�1000��-1000,10,
  * 
  ******************************************************************************
  */

#include "ax_uart5.h"
#include <stdio.h>

static uint8_t uart5_flag_rx_ok = 0; //���ճɹ���־
static uint8_t uart5_rx_con=0;  //���ռ�����
static uint8_t uart5_rx_checksum; //֡ͷ����У���
static uint8_t uart5_rx_buf[40];  //���ջ��壬��������С�ڵ���32Byte
static uint8_t uart5_tx_buf[40];  //���ͻ���

/**
  * @��  ��  ���ڳ�ʼ��
  * @��  ��  baud�� ����������
  * @����ֵ	 ��
  */
void AX_UART5_Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//**USART����******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);  //�򿪴���GPIO��ʱ��

	//��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);  //�򿪴��������ʱ��

	//����USART����
	USART_InitStructure.USART_BaudRate = baud;		//������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure);

	//����USARTΪ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //�������ȼ�	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);//��ʼ������NVIC

	//ʹ�ܴ��ڽ����ж�
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

	//ʹ�� USART�� �������
	USART_Cmd(UART5, ENABLE);
}

/**
  * @��  ��  �����жϷ������
  * @��  ��  ��
  * @����ֵ  ��
  */
void UART5_IRQHandler(void)                	
{
	uint8_t Res;
	
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  //�����ж�
	{
		  //printf("Get Data!\r\n");
			Res =USART_ReceiveData(UART5);	
		
			if(uart5_rx_con < 3)    //==����֡ͷ + ����
			{
				if(uart5_rx_con == 0)  //����֡ͷ1 0xAA
				{
					if(Res == 0xAA)
					{
						uart5_rx_buf[0] = Res;
						uart5_rx_con = 1;					
					}
					else
					{
						
					}
				}else if(uart5_rx_con == 1) //����֡ͷ2 0x55
				{
					if(Res == 0x55)
					{
						uart5_rx_buf[1] = Res;
						uart5_rx_con = 2;
					}
					else
					{
						uart5_rx_con = 0;						
					}				
				}
				else  //�������ݳ���
				{
					uart5_rx_buf[2] = Res;
					uart5_rx_con = 3;
					uart5_rx_checksum = (0xAA+0x55) + Res;	//����У���
				}
			}
			else    //==��������
			{
				if(uart5_rx_con < (uart5_rx_buf[2]-1) )
				{
					uart5_rx_buf[uart5_rx_con] = Res;
					uart5_rx_con++;
					uart5_rx_checksum = uart5_rx_checksum + Res;					
				}
				else    //�ж����1λ
				{
					//����У��
					if( Res == uart5_rx_checksum )  //У����ȷ
					{	
						//=====�˴��������ݽ���=========
						uart5_flag_rx_ok = 1;
						
						//������ɣ��ָ���ʼ״̬
						uart5_rx_con = 0;					
					}	
					
				}
			}
			
      USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	} 
}

/**
  * @��  ��  ��ȡ���յ�����
  * @��  ��  *pbuf����������ָ��,��1���ֽ�Ϊ֡���룬����Ϊ����
  * @����ֵ	 0-�����ݽ��գ�other-��Ҫ��ȡ�������ֽڸ���
  */
uint8_t AX_UART5_GetData(uint8_t *pbuf)
{
	uint8_t cnt,i;
	
	if(uart5_flag_rx_ok != 0)
	{
		cnt = uart5_rx_buf[2]-4;
		
		for(i=0; i<cnt; i++)
		{
			*(pbuf+i) = uart5_rx_buf[3+i];
		}
		
		uart5_flag_rx_ok = 0;
		return cnt;
	}
	else
	{
		return 0;
	}	
}

/**
  * @��  ��  �������ݣ�X-ProtocolЭ�飩
  * @��  ��  *pbuf����������ָ��
  *          len���������ݳ��ȸ�������27 (32-5)
  *          num��֡�ţ�֡����
  * @����ֵ	 ��
  */
void AX_UART5_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i,cnt;	
    uint8_t tx_checksum = 0;//����У���
	
	if(len <= 50)
	{
		/******��ȡ����******/
		uart5_tx_buf[0] = 0xAA;    //֡ͷ
		uart5_tx_buf[1] = 0x55;    //
		uart5_tx_buf[2] = len+5;  //����������ȼ���֡����
		uart5_tx_buf[3] = num;    //֡����
		
		for(i=0; i<len; i++)
		{
			uart5_tx_buf[4+i] = *(pbuf+i);
		}
		
		/******����У���******/	
		cnt = 4+len;
		for(i=0; i<cnt; i++)
		{
			tx_checksum = tx_checksum + uart5_tx_buf[i];
		}
		uart5_tx_buf[i] = tx_checksum;
		
		
		/******��������******/	
		cnt = 5+len;
		
		//��ѯ���䷽ʽ
		for(i=0; i<cnt; i++)
		{
			USART_SendData(UART5, uart5_tx_buf[i]);
			while(USART_GetFlagStatus(UART5,USART_FLAG_TC) != SET);
		}	
	}
}



/******************* (C) ��Ȩ 2023 XTARK  **************************************/
