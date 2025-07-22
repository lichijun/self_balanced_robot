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
  * @��  ��  CCDѲ�ߴ�����
  ******************************************************************************
  * @˵  ��
  *
  * 
  ******************************************************************************
  */

#include "ax_ccd.h"
#include "ax_sys.h"
#include <stdio.h>

//���Ŷ���
#define TSL_CLK   PBout(13)   //CLK 
#define TSL_SI    PBout(12)   //SI  

//�ڲ���������
static void CCD_Delay_us(void);  //CCDר����ʱ����
static uint16_t CCD_GetAdcData(void);  //��ȡADC����ֵ

/**
  * @��  ��  CCDѲ�ߴ�������ʼ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_CCD_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	//����GPIO AO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	//����GPIO SI,CLK
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	//**����ADC******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	//ADCģʽ����
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);	

	//����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	//����ADCͨ����ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);

	//ʹ��ADC
	ADC_Cmd(ADC1, ENABLE);

	//��ʼ��ADC У׼�Ĵ���  
	ADC_ResetCalibration(ADC1);

	//�ȴ�У׼�Ĵ�����ʼ�����
	while(ADC_GetResetCalibrationStatus(ADC1));

	//ADC��ʼУ׼
	ADC_StartCalibration(ADC1);

	//�ȴ�У׼���
	while(ADC_GetCalibrationStatus(ADC1)); 		
}

/**
  * @��  ��  CCD��ȡһ֡����
  * @��  ��  *pbuf��CCDһ֡���ݣ����ݸ���128
  * @����ֵ  ���ݽ����
  */
void AX_CCD_GetData(uint16_t *pbuf)
{
	uint8_t i;
	
	//���ó�ʼ״̬
	TSL_CLK=1;
    TSL_SI=0; 
    CCD_Delay_us();
    
	//�������������
    TSL_SI=1; 
    TSL_CLK=0;
    CCD_Delay_us();
	
	//���õȴ�����
    TSL_CLK=1;
    TSL_SI=0;
    CCD_Delay_us(); 
	
	//��ȡ128�����ص��ѹֵ
	for(i=0;i<128;i++)					
	{   
		//�½�����������
		TSL_CLK=0; 
		CCD_Delay_us();  //�����ع�ʱ��
        
		//��ȡת�����
		*(pbuf+i) = (CCD_GetAdcData())>>4;
		TSL_CLK=1;
		CCD_Delay_us();	
	}
}

/**
  * @��  ��  ��ȡ��������߾����м�λ��ƫ��
  * @��  ��  ��
  * @����ֵ  ƫ�������м�Ϊ0����Χ��64��
  */
int16_t AX_CCD_GetOffset(void)
{
	uint8_t i;
	static uint16_t ccd[128]= {0};
	static uint16_t ccd_min,ccd_max;
	static uint16_t ccd_threshold;
	static uint16_t ccd_left,ccd_right;
	static int16_t ccd_offset;
	
	//��ȡCCDһ֡��128������
	AX_CCD_GetData(ccd);
	
//	//����ʹ�ã��鿴CCDԭʼ����	
//	for(i=0; i<128; i++) printf("%d ",ccd[i]);
//	printf("\r\n");

	//CCD�ɼ�128���������ݣ�ÿ����������ֵ���бȽϣ�����ֵ��Ϊ��ɫ������ֵСΪ��ɫ
	//��̬��ֵ�㷨��һ֡���ݵ�������Сֵ���м�ֵ��Ϊ��ֵ 
	
	//-----���㶯̬��ֵ-------
	ccd_min = ccd[0];
	ccd_max = ccd[0];
	
	//�������飬Ѱ����ֵ
	for(i=0; i<128; i++)
	{
		//Ѱ����Сֵ
		if(ccd_min > ccd[i])
		{
			ccd_min = ccd[i];
		}
		
		//Ѱ�����ֵ
		if(ccd_max < ccd[i])
		{
			ccd_max = ccd[i];
		}	
	}
	
	//������ֵ
	ccd_threshold =(ccd_min + (ccd_max-ccd_min)/2);	  
	
	
	//���Һ�����߱��أ������غ�����4���������жϱ�����
	for(i=0; i<125; i++)
	{
		//
		if(ccd[i] < ccd_threshold)
		{
			//��������������Ϊ������
			if((ccd[i+1]<ccd_threshold) && (ccd[i+2]<ccd_threshold) && (ccd[i+3]<ccd_threshold))
			{
				ccd_left = i;
				break;
			}
		}
	}
	
	//���Һ����ұ߱��أ�������ǰ������4���������жϱ���
	for(i=128; i>2; i--)
	{
		//
		if(ccd[i] < ccd_threshold)
		{
			//ǰ��������������Ϊ����
			if((ccd[i-1]<ccd_threshold) && (ccd[i-2]<ccd_threshold) && (ccd[i-3]<ccd_threshold))
			{
				ccd_right = i;
				break;
			}
		}
	}
	
	//����ƫ��ֵ
	ccd_offset = (ccd_left+ccd_right)/2 - 64;
	
	//��ӡ������Ϣ
	//printf("%d  %d \r\n",ccd_threshold, ccd_offset);
	
	return ccd_offset;
}

/**
  * @��  ��  CCD��ȡADC����ֵ
  * @��  ��  ��	  
  * @����ֵ  ADCת�������
  */
static uint16_t CCD_GetAdcData(void)
{
	//�������ת������ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);  

	//�ȴ�ת������ 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
	
	return ADC_GetConversionValue(ADC1);
}

/**
  * @��  ��  CCDר����ʱ����
  * @��  ��  ��	  
  * @����ֵ  ��
  */
static void CCD_Delay_us(void)
{
   int i;   

   for(i=0;i<30;i++);      
}



/******************* (C) ��Ȩ 2023 XTARK **************************************/
