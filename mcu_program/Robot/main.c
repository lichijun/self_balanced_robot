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
  * @内  容  主函数体
  * 
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include "ax_robot.h"

#include "ax_oled_chinese.h" //OLED汉字库
#include "ax_oled_picture.h" //OLED 图片库

//任务句柄
//启动任务
#define START_TASK_PRIO		1
#define START_STK_SIZE 		256  
TaskHandle_t StartTask_Handler = NULL;
void Start_Task(void *pvParameters);

//控制和功能任务
#define CONTROL_TASK_PRIO		3     
#define CONTROL_STK_SIZE 		256 
TaskHandle_t Control_Task_Handle = NULL;
void Control_Task(void *pvParameters);

//按键处理任务
#define KEY_TASK_PRIO		4     
#define KEY_STK_SIZE 		128   
TaskHandle_t Key_Task_Handle = NULL;
void Key_Task(void *pvParameters);

//RGB灯效任务
#define LIGHT_TASK_PRIO		5     
#define LIGHT_STK_SIZE 		128   
TaskHandle_t Light_Task_Handle = NULL;
void Light_Task(void *pvParameters);

//OLED显示任务
#define DISP_TASK_PRIO		6     
#define DISP_STK_SIZE 		128   
TaskHandle_t Disp_Task_Handle = NULL;
void Disp_Task(void *pvParameters);

//琐事管理任务
#define TRIVIA_TASK_PRIO		10     
#define TRIVIA_STK_SIZE 		128   
TaskHandle_t Trivia_Task_Handle = NULL;
void Trivia_Task(void *pvParameters);

//PS2数据获取任务
#define PS2_TASK_PRIO		11     
#define PS2_STK_SIZE 		128   
TaskHandle_t Ps2_Task_Handle = NULL;
void Ps2_Task(void *pvParameters);


/**
  * @简  述  程序主函数
  * @参  数  无
  * @返回值  无
  */
int main(void)
{	

	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);   
	
	//舵机初始化
	AX_SERVO_S1234_Init();	

	//电机初始化
	AX_MOTOR_Init();
	
	//延时函数初始化
	AX_DELAY_Init();  
	
	//JTAG初始化
	AX_JTAG_Set(JTAG_SWD_DISABLE);    	
	AX_JTAG_Set(SWD_ENABLE);      
    
	//LED初始化
	AX_LED_Init();  
	
	//KEY按键检测初始化
	AX_KEY_Init();
	
	//电池电压检测初始化
	AX_VIN_Init();
	
	//蜂鸣器初始化
	AX_BEEP_Init();  
	
	//调试串口初始化
	AX_UART1_Init(230400);	
	
	//激光雷达传感器初始化
	AX_LASER_Init();
	
	//蓝牙串口初始化
	AX_UART2_Init(115200);
	
	//航模遥控器SBUS串口初始化
	AX_SBUS_Init();
	
	//PS2手柄初始化
	AX_PS2_Init();
	
	//编码器初始化
	AX_ENCODER_A_Init();  
	AX_ENCODER_B_Init(); 

	//RGB彩灯
	AX_RGB_Init();	
	AX_RGB_SetFullColor(0, 0, 0xff);
	
	//OLED屏幕初始化
	AX_OLED_Init();	
	AX_OLED_DispPicture(0, 0, 128, 8, PIC64X128_XTARK, 0); 
	
	//开机提示信息
	AX_BEEP_On();
	AX_Delayms(100);	
	AX_BEEP_Off();
	AX_Delayms(1900);
	
	//MPU6050初始化
	AX_MPU6050_Init();  //MPU6050初始化  
	AX_MPU6050_DMP_Init();  //DMP初始化
	
	//CCD传感器初始化
	AX_CCD_Init();

	//创建AppTaskCreate任务
	xTaskCreate((TaskFunction_t )Start_Task,  /* 任务入口函数 */
								 (const char*    )"Start_Task",/* 任务名字 */
								 (uint16_t       )START_STK_SIZE,  /* 任务栈大小 */
								 (void*          )NULL,/* 任务入口函数参数 */
								 (UBaseType_t    )START_TASK_PRIO, /* 任务的优先级 */
								 (TaskHandle_t*  )&StartTask_Handler);/* 任务控制块指针 */ 
							
	//启动任务，开启调度						 
	vTaskStartScheduler(); 

	//循环
	while (1);
}


/**
  * @简  介  创建任务函数
  * @参  数  无
  * @返回值  无
  */
void Start_Task(void *pvParameters)
{
	/******机器人启动流程************************************************/
	
	//默认灯效参数
	R_Light.M  = LEFFECT2;  //呼吸效果
	R_Light.S  = 0;
	R_Light.T  = 0;
	R_Light.R  = 0x00;
	R_Light.G  = 0xFF;
	R_Light.B  = 0xFF;	
	
	//设置舵机角度，加入零偏矫正
	AX_SERVO_S1_SetAngle(JOINTA_ANGLE_OFFSET);
	AX_SERVO_S2_SetAngle(JOINTB_ANGLE_OFFSET);		
	
	//开机启动完成，绿灯点亮，蜂鸣器提示
	AX_LED_Green_On();	
	AX_BEEP_On();
	AX_Delayms(100);	
	AX_BEEP_Off();	
	
	//显示主窗口界面
	AX_OLED_ClearScreen();  //清除OLED启动画面显示
	AX_OLED_DispStr(0, 0, "   * TARKBOT B680 *   ", 0);	
	AX_OLED_DispStr(0, 1, "---------------------", 0);	
	AX_OLED_DispStr(0, 2, " Vol:12.2V Mod:ROS   ", 0);	
	AX_OLED_DispStr(0, 3, " Agl:00.00 Gyo:00000 ", 0);
	AX_OLED_DispStr(0, 4, " MTA:00.00 MTB:-0.00 ", 0);	
	AX_OLED_DispStr(0, 5, "---------------------", 0);	
	AX_OLED_DispStr(0, 6, "                     ", 0);	

	//进入临界区
	taskENTER_CRITICAL();           
  
	//创建机器人控制任务
	xTaskCreate((TaskFunction_t )Control_Task, /* 任务入口函数 */
			 (const char*    )"Control_Task",/* 任务名字 */
			 (uint16_t       )CONTROL_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )CONTROL_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Control_Task_Handle);/* 任务控制块指针 */
			 	 								 
	//创建按键处理任务
	xTaskCreate((TaskFunction_t )Key_Task, /* 任务入口函数 */
			 (const char*    )"Key_Task",/* 任务名字 */
			 (uint16_t       )KEY_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )KEY_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Key_Task_Handle);/* 任务控制块指针 */
			 
	//RGB灯效任务
	xTaskCreate((TaskFunction_t )Light_Task, /* 任务入口函数 */
			 (const char*    )"Light_Task",/* 任务名字 */
			 (uint16_t       )LIGHT_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )LIGHT_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Light_Task_Handle);/* 任务控制块指针 */			

	//OLED屏显示任务
	xTaskCreate((TaskFunction_t )Disp_Task, /* 任务入口函数 */
			 (const char*    )"Disp_Task",/* 任务名字 */
			 (uint16_t       )DISP_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )DISP_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Disp_Task_Handle);/* 任务控制块指针 */	
			 
	//琐事管理任务
	xTaskCreate((TaskFunction_t )Trivia_Task, /* 任务入口函数 */
			 (const char*    )"Trivia_Task",/* 任务名字 */
			 (uint16_t       )TRIVIA_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )TRIVIA_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Trivia_Task_Handle);/* 任务控制块指针 */

	//PS2手柄数据读取任务
	xTaskCreate((TaskFunction_t )Ps2_Task, /* 任务入口函数 */
			 (const char*    )"Ps2_Task",/* 任务名字 */
			 (uint16_t       )PS2_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )PS2_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Ps2_Task_Handle);/* 任务控制块指针 */				 
			 
						  
	//删除AppTaskCreate任务				
	vTaskDelete(StartTask_Handler); 

	//退出临界区
	taskEXIT_CRITICAL();  		 
						 							
}

/******************* (C) 版权 2023 XTARK **************************************/

