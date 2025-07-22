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
  * @��  ��  ���Դ���ͨ��
  *
  ******************************************************************************
  * @˵  ��
  *
  * 1.USB����ͨ�ţ�printf�����Ѷ��򵽸ô��ڣ��������������Ϣ
  * 2.����ʹ�þ���X-ProtocolЭ��������ݷ��͡�
  * 3.����UART�Ĵ������ݽ��չ��ܣ�ʹ���жϷ�ʽ��X-ProtocolЭ��ͨ��
  * 4.��ͨ��AX_UART1_GetRxData()�����ж��Ƿ������ݽ���
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

#include "ax_uart1.h"
#include <stdio.h>

#include "ax_robot.h"

static uint8_t uart1_rx_con=0;       //���ռ�����
static uint8_t uart1_rx_checksum;    //֡ͷ����У���
static uint8_t uart1_rx_buf[40];     //���ջ��壬��������С�ڵ���32Byte
static uint8_t uart1_tx_buf[40];     //���ͻ���

/**
  * @��  ��  UART ���ڳ�ʼ��
  * @��  ��  baud�� ����������
  * @����ֵ	 ��
  */
void AX_UART1_Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//**���Դ���USART����******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //�򿪴���GPIO��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  //�򿪴��������ʱ��
	
	//��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//����USART����
	USART_InitStructure.USART_BaudRate = baud; //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	//����USARTΪ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //�������ȼ�	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);//��ʼ������NVIC
	
	//ʹ�ܴ��ڽ����ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	//ʹ�� USART�� �������
	USART_Cmd(USART1, ENABLE);
}

/**
  * @��  ��  DBUART �����жϷ�����
  * @��  ��  �� 
  * @����ֵ  ��
  */
void USART1_IRQHandler(void)
{
	uint8_t Res;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
	{
		Res =USART_ReceiveData(USART1);	
		
		if(uart1_rx_con < 3)    //==����֡ͷ + ����
		{
			if(uart1_rx_con == 0)  //����֡ͷ1 0xAA
			{
				if(Res == 0xAA)
				{
					uart1_rx_buf[0] = Res;
					uart1_rx_con = 1;					
				}
				else
				{
					
				}
			}else if(uart1_rx_con == 1) //����֡ͷ2 0x55
			{
				if(Res == 0x55)
				{
					uart1_rx_buf[1] = Res;
					uart1_rx_con = 2;
				}
				else
				{
					uart1_rx_con = 0;						
				}				
			}
			else  //�������ݳ���
			{
				uart1_rx_buf[2] = Res;
				uart1_rx_con = 3;
				uart1_rx_checksum = (0xAA+0x55) + Res;	//����У���
			}
		}
		else    //==��������
		{
			if(uart1_rx_con < (uart1_rx_buf[2]-1) )
			{
				uart1_rx_buf[uart1_rx_con] = Res;
				uart1_rx_con++;
				uart1_rx_checksum = uart1_rx_checksum + Res;					
			}
			else    //�ж����1λ
			{
				//������ɣ��ָ���ʼ״̬
				uart1_rx_con = 0;	
				
				//����У��
				if( Res == uart1_rx_checksum )  //У����ȷ
				{	

					//�ٶȿ���֡
					if(uart1_rx_buf[3] == ID_URX_VEL)
					{
						ax_robot_vx = (int16_t)((uart1_rx_buf[4]<<8) | uart1_rx_buf[5]);
						ax_robot_vw = (int16_t)((uart1_rx_buf[8]<<8) | uart1_rx_buf[9]);
						
						//ת���ٶȻ���ת��Ϊ�Ƕ�
						ax_robot_vw = ax_robot_vw*((180/PI)*0.001f);
						
						//����ΪROS����ģʽ
						ax_control_mode = CTL_ROS;
					}
					else
					{
						//ֱ��ƽ��PID����
						if(uart1_rx_buf[3] == ID_URX_BLC)
						{
							ax_balance_kp = (int16_t)((uart1_rx_buf[4]<<8) | uart1_rx_buf[5]);
							ax_balance_kd = (int16_t)((uart1_rx_buf[6]<<8) | uart1_rx_buf[7]);
						}
						
						//�ٶȿ���PID����
						if(uart1_rx_buf[3] == ID_URX_BLV)
						{
							ax_velocity_kp = (int16_t)((uart1_rx_buf[4]<<8) | uart1_rx_buf[5]);
							ax_velocity_ki = (int16_t)((uart1_rx_buf[6]<<8) | uart1_rx_buf[7]);
						}						
						
						//ת�����PID����
						if(uart1_rx_buf[3] == ID_URX_BLT)
						{
							ax_turn_kp = (int16_t)((uart1_rx_buf[4]<<8) | uart1_rx_buf[5]);
							ax_turn_kd = (int16_t)((uart1_rx_buf[6]<<8) | uart1_rx_buf[7]);
						}							
						
						//ִ�з�����������ʾ
						ax_beep_ring = BEEP_SHORT;
					}	
				}
			}
		}
	} 
}


/**
  * @��  ��  UART �������ݣ�X-ProtocolЭ�飩
  * @��  ��  *pbuf����������ָ��
  *          len���������ݳ��ȸ�������27 (32-5)
  *          num��֡�ţ�֡����
  * @����ֵ	 ��
  */
void AX_UART1_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i,cnt;	
  uint8_t tx_checksum = 0;//����У���
	
	if(len <= 32)
	{
		/******��ȡ����******/
		uart1_tx_buf[0] = 0xAA;    //֡ͷ
		uart1_tx_buf[1] = 0x55;    //
		uart1_tx_buf[2] = len+5;  //����������ȼ���֡����
		uart1_tx_buf[3] = num;    //֡����
		
		for(i=0; i<len; i++)
		{
			uart1_tx_buf[4+i] = *(pbuf+i);
		}
		
		/******����У���******/	
		cnt = 4+len;
		for(i=0; i<cnt; i++)
		{
			tx_checksum = tx_checksum + uart1_tx_buf[i];
		}
		uart1_tx_buf[i] = tx_checksum;
		
		
		/******��������******/	
		cnt = 5+len;
		
		//��ѯ���䷽ʽ
		for(i=0; i<cnt; i++)
		{
			USART_SendData(USART1, uart1_tx_buf[i]);
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC) != SET);
		}
	}
}

/**************************���ڴ�ӡ��غ����ض���********************************/
/**
  * @��  ��  �ض���putc������USART1��	
  */
int fputc(int ch, FILE *f)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART1, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}

	return ch;
}

/**
  * @��  ��  �ض���getc������USART1��	
  */
int fgetc(FILE *f)
{
	/* �ȴ�����1�������� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
	{}

	return (int)USART_ReceiveData(USART1);
}

/******************* (C) ��Ȩ 2023 XTARK **************************************/
