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
  * @��  ��  VIN�����ѹ���
  *
  ******************************************************************************
  * @˵  ��
  *
  * 1.ʹ��ADC2��VIN�����ѹ��������ѯ��ʽ
	* 2.�����ѹ��⹦�ܣ���ʵ�ֶԵ�ص���������
  * 
  ******************************************************************************
  */

#include "ax_vin.h"

#define ADC_REVISE  99.9

/**
  * @��  ��  VIN �����ѹ����ʼ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_VIN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	//����GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//**����ADC******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	//ADCģʽ����
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);	

	//����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	//����ADCͨ����ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5);

	//ʹ��ADC
	ADC_Cmd(ADC2, ENABLE);

	//��ʼ��ADC У׼�Ĵ���  
	ADC_ResetCalibration(ADC2);

	//�ȴ�У׼�Ĵ�����ʼ�����
	while(ADC_GetResetCalibrationStatus(ADC2));

	//ADC��ʼУ׼
	ADC_StartCalibration(ADC2);

	//�ȴ�У׼���
	while(ADC_GetCalibrationStatus(ADC2)); 	
}


/**
  * @��  ��  VIN ��������ѹ
  * @��  ��  ��	  
  * @����ֵ  ��ѹֵ������100��������7.2V���Ϊ720��
  */
uint16_t AX_VIN_GetVol_X100(void)
{
	uint16_t Input_Vol,temp;

	//�������ת������ 
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);  

	//�ȴ�ת������ 
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));

	temp = ADC_GetConversionValue(ADC2);
	
	//��ѹ����1/11
	Input_Vol = (uint16_t)((3.3*11.0*ADC_REVISE * temp)/4095);	
	
	return Input_Vol;
}


/******************* (C) ��Ȩ 2023 XTARK **************************************/
