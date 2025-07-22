/**			                                                    
		   ____                    _____ _______ _____       @塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： @塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @内  容  机器人平衡控制
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_BALANCE_H
#define __AX_BALANCE_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

// #define LQR_4_STATES
#define LQR_6_STATES

typedef struct
{
  float x;           // 位移(m)
  float x_dot;       // 速度(m/s)
  float phi;         // 倾角(rad) 前倾为负
  float phi_dot;     // 倾角角速度(rad/s)
  float theta;       // 航向角(rad)
  float theta_dot;   // 航向角角速度(rad/s)
  int16_t odoNumL;  // 提高位移计算精度，轮子停转odoNum清零
  int16_t odoNumR;
  float wheel_vel_l; // 左轮速度(m/s)
  float wheel_vel_r; // 右轮速度(m/s)
}StateVariable;

typedef struct 
{
  // x_2dot = a * x_dot + b * pwm + c
  float a;
  float b;
  float c;
}MotorCharacterCoef;






#endif

/******************* (C) 版权 2023 XTARK **************************************/
