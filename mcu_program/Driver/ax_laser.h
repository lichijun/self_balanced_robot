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
  * @内  容  乐动STL-19P雷达处理
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_LASER_H
#define __AX_LASER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//雷达帧相关定义
#define LS_HEADER     0x54   //帧头
#define LS_F_LEN      47     //帧长度
#define LS_VERLEN     0x2C   //低五位是一帧数据接收到的点数，目前固定是12，高三位预留
#define LS_PNAM       12     //一帧测量点数量

#define LS_DISABLE     0   //串口中断打开
#define LS_ENABLE      1   //串口中断关闭

//雷达点结构体定义
typedef struct
{
	uint16_t      angle;     //角度
	uint16_t   distance;     //距离

}LaserPointTypeDef;

extern LaserPointTypeDef ax_ls_point[250];


//接口函数
void AX_LASER_Init(void);    //雷达串口初始化
void AX_LASER_SetEnable(uint8_t state);  //设置雷达串口接收是否打开

#endif 

/******************* (C) 版权 2023 XTARK **************************************/
