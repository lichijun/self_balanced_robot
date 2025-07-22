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
  * @��  ��  PWM�ӿڶ������
  *
  ******************************************************************************
  * @˵  ��
  *
  ******************************************************************************
  */

#include "ax_servo.h" 

/**
  * @��  ��  ����ӿڳ�ʼ��	
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_SERVO_S1234_Init(void)
{ 
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 
	
	//��ʱ��ͨ��IO����
	//GPIO�����ù���ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);	

	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//TIMʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	//��ʱ������	PWM����ģʽ��Ƶ��50Hz,����20ms  
	//ռ�ձȵ��ڷ�Χ��0-1.5ms-2.5ms 0-1500-2500	��ʼ��Ϊ1500
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;  //��ʱ����Ƶ����Ƶ���Ƶ��Ϊ1M
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	//PWM1 Mode configuration
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1500;	    //ռ�ձȳ�ʼ����90��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

	//PWM1 Mode configuration
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	//PWM1 Mode configuration
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	//PWM1 Mode configuration
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8, ENABLE);

	//ʹ�ܶ�ʱ��
	TIM_Cmd(TIM8, ENABLE);	
	
	//ʹ��MOEλ
	TIM_CtrlPWMOutputs(TIM8,ENABLE);		
	
}

/**
  * @��  ��  �������
  * @��  ��  angle ����ĽǶȣ���Χ��-900~900������ϵ��0.1,
  *          �ر�˵�������ֶ��ʵ�ʿ��ƽǶ�С��90�ȣ���ע�ⷶΧ����
  * @����ֵ  ��
  */
void AX_SERVO_S1_SetAngle(int16_t angle)
{
	if(angle >  900) angle =  900;
	if(angle < -900) angle = -900;
	
	TIM_SetCompare4(TIM8,(1.111f*angle + 1500));
}

/**
  * @��  ��  �������
  * @��  ��  angle ����ĽǶȣ���Χ��-900~900������ϵ��0.1,
  *          �ر�˵�������ֶ��ʵ�ʿ��ƽǶ�С��90�ȣ���ע�ⷶΧ����
  * @����ֵ  ��
  */
void AX_SERVO_S2_SetAngle(int16_t angle)
{
	if(angle >  900) angle =  900;
	if(angle < -900) angle = -900;
	
	TIM_SetCompare3(TIM8,(1.111f*angle + 1500));
	
}

/**
  * @��  ��  �������
  * @��  ��  angle ����ĽǶȣ���Χ��-900~900������ϵ��0.1,
  *          �ر�˵�������ֶ��ʵ�ʿ��ƽǶ�С��90�ȣ���ע�ⷶΧ����
  * @����ֵ  ��
  */
void AX_SERVO_S3_SetAngle(int16_t angle)
{
	if(angle >  900) angle =  900;
	if(angle < -900) angle = -900;
	
	TIM_SetCompare2(TIM8,(1.111f*angle + 1500));
	
}

/**
  * @��  ��  �������
  * @��  ��  angle ����ĽǶȣ���Χ��-900~900������ϵ��0.1,
  *          �ر�˵�������ֶ��ʵ�ʿ��ƽǶ�С��90�ȣ���ע�ⷶΧ����
  * @����ֵ  ��
  */
void AX_SERVO_S4_SetAngle(int16_t angle)
{
	if(angle >  900) angle =  900;
	if(angle < -900) angle = -900;
	
	TIM_SetCompare1(TIM8,(1.111f*angle + 1500));
	
}

/**
  * @��  ��  ����ӿڳ�ʼ��	
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_SERVO_S56_Init(void)
{ 
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 	
	
	//GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);


	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	//��ʱ������	PWM����ģʽ��Ƶ��50Hz,����20ms  ռ�ձȵ��ڷ�Χ��0-1.5ms-2.5ms 0-1500-2500	��ʼ��Ϊ1500
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;  //��ʱ����Ƶ����Ƶ���Ƶ��Ϊ1M
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	//PWM1 Mode configuration: Channel
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1500;	    //ռ�ձȳ�ʼ����90��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM5, ENABLE);

	//ʹ�ܶ�ʱ��
	TIM_Cmd(TIM5, ENABLE);		
}

/**
  * @��  ��  �������
  * @��  ��  angle ����ĽǶȣ���Χ��-900~900������ϵ��0.1,
  *          �ر�˵�������ֶ��ʵ�ʿ��ƽǶ�С��90�ȣ���ע�ⷶΧ����
  * @����ֵ  ��
  */
void AX_SERVO_S5_SetAngle(int16_t angle)
{
	if(angle >  900) angle =  900;
	if(angle < -900) angle = -900;
	
	TIM_SetCompare1(TIM5,(1.111f*angle + 1500));
	
}

/**
  * @��  ��  �������
  * @��  ��  angle ����ĽǶȣ���Χ��-900~900������ϵ��0.1,
  *          �ر�˵�������ֶ��ʵ�ʿ��ƽǶ�С��90�ȣ���ע�ⷶΧ����
  * @����ֵ  ��
  */
void AX_SERVO_S6_SetAngle(int16_t angle)
{
	if(angle >  900) angle =  900;
	if(angle < -900) angle = -900;
	
	TIM_SetCompare2(TIM5,(1.111f*angle + 1500));
	
}

/******************* (C) ��Ȩ 2023 XTARK **************************************/
