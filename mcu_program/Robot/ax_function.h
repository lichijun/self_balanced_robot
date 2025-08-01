/**			                                                    
		   ____                    _____ _______ _____       XTARK@塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @内  容  机器人功能处理文件
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_FUNCTION_H
#define __AX_FUNCTION_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"


extern int16_t ax_ccd_speed; 
extern int16_t ax_ccd_offset;   
extern int16_t ax_ccd_kp;
extern int16_t ax_ccd_kd;  


//功能函数
void AX_FUN_List1(void);
void AX_FUN_List2(void);
void AX_FUN_List3(void);
void AX_FUN_List4(void);
void AX_FUN_List5(void);

#endif

/******************* (C) 版权 2023 XTARK **************************************/
