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
  * @��  ��  ������ƽ�����
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_BALANCE_H
#define __AX_BALANCE_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

#define LQR_4_STATES
// #define LQR_6_STATES

typedef struct
{
  float x;           // λ��(m)
  float x_dot;       // �ٶ�(m/s)
  float phi;         // ���(rad) ǰ��Ϊ��
  float phi_dot;     // ��ǽ��ٶ�(rad/s)
  float theta;       // �����(rad)
  float theta_dot;   // ����ǽ��ٶ�(rad/s)
  int16_t odoNumL;  // ���λ�Ƽ��㾫�ȣ�����ͣתodoNum����
  int16_t odoNumR;
  float wheel_vel_l; // �����ٶ�(m/s)
  float wheel_vel_r; // �����ٶ�(m/s)
}StateVariable;

typedef struct 
{
  // tau = a * x_dot + b * pwm
  float a;
  float b;
}MotorCharacterCoef;






#endif

/******************* (C) ��Ȩ 2023 XTARK **************************************/
