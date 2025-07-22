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
  * @��  ��  Musk Han@
  * @��  ��  V1.0
  * @��  ��  2022-7-26
  * @��  ��  ���������
  *
  ******************************************************************************
  * @˵  ��
  *
  ******************************************************************************
  */

#include "ax_ursc.h" 

#include "ax_delay.h" 

static uint32_t ultrasonic_distance;  //����������ֵ
static uint32_t ultrasonic_tim;

uint8_t  US_TIM_CAPTURE_STA = 0;	//ͨ��1���벶���־������λ�������־����6λ�������־		
uint16_t US_TIM_CAPTURE_UPVAL;
uint16_t US_TIM_CAPTURE_DOWNVAL;


/**
  * @��  ��  ��������ʼ��	
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_URSC_Init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	NVIC_InitTypeDef NVIC_InitStructure;	
	
	//GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	//���ó�����ECHO�˿�
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//���ó�����TRIG�˿�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;       //���͵�ƽ����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�������
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1 );
	
	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
	//���ö�ʱ��
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period=0xffff; //(65535-1); ��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ         ������1000Ϊ1ms
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;  //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  1M�ļ���Ƶ�� 1US����
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	//��ʼ��TIM5���벶����� ͨ��1
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM5, &TIM_ICInitStructure);	
	
	//��ʱ���ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;  //��ռ���ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);   //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

    //����������жϣ�����CC1IE�����ж�	
	TIM_ITConfig(TIM5, TIM_IT_CC1 , ENABLE);   
	
	//ʹ�ܶ�ʱ��	
	TIM_Cmd(TIM5, ENABLE); 			
}

/**
  * @��  ��  ��ʱ���жϷ������,����������ͨ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void TIM5_IRQHandler(void)
{
	if ((US_TIM_CAPTURE_STA & 0X80) == 0) 		//��δ�ɹ�����	
	{
		//����1���������¼�
		if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET) 		
		{   
			//����жϱ�־λ
			TIM_ClearITPendingBit(TIM5, TIM_IT_CC1); 	

			//����һ���½���
			if (US_TIM_CAPTURE_STA & 0X40)		
			{
				//��¼�´�ʱ�Ķ�ʱ������ֵ
				US_TIM_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM5);
				
				
				//�����������ʼֵ����ĩβֵ��˵�������������
				if (US_TIM_CAPTURE_DOWNVAL < US_TIM_CAPTURE_UPVAL)
				{
					ultrasonic_tim = 65535;
				}
				else
				{
					ultrasonic_tim = 0;
				}
				
				//�õ��ܵĸߵ�ƽ��ʱ��
				ultrasonic_distance = US_TIM_CAPTURE_DOWNVAL - US_TIM_CAPTURE_UPVAL + ultrasonic_tim;	
				
				//�������&&UltrasonicWave_Distance<85
				ultrasonic_distance = ultrasonic_distance *17/1000;			
				
				//�����־λ���㣬��һ������Ҫ��
				US_TIM_CAPTURE_STA = 0;	

				//����Ϊ�����ز���		 
				TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Rising);  
				
			}
			else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				//��ȡ����������
				US_TIM_CAPTURE_UPVAL = TIM_GetCapture1(TIM5);	
				
				//����Ѳ���������
				US_TIM_CAPTURE_STA |= 0X40;		
				
				//����Ϊ�½��ز���
				TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Falling);
			}
		}
	}
}

/**
  * @��  ��  �������������
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_URSC_StartUp(void)
{
	  //�����źţ���Ϊ�����ź�
	  GPIO_SetBits(GPIOA, GPIO_Pin_1);
	
	  //�ߵ�ƽ�źų���10us
	  AX_Delayus(20);
	  GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

/**
  * @��  ��  ��ȡ�������������
  * @��  ��  ��
  * @����ֵ  ���������ֵ����λcm
  */
uint32_t AX_URSC_GetDistance(void)
{
    //���ؾ���ֵ	
	return ultrasonic_distance;
	
}

/******************* (C) ��Ȩ 2023 XTARK **************************************/
