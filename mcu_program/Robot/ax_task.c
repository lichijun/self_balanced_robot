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
  * @内  容  机器人任务处理文件
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ax_task.h"
#include "ax_robot.h"

#include "ax_light.h"
#include "ax_control.h"
#include "ax_function.h"

//函数定义
void JOINT_Control(void);       //舵机云台处理

/**
  * @简  述  机器人控制和功能管理任务
  * @参  数  无
  * @返回值  无
  */
void Control_Task(void* parameter)
{	
	while(1)
	{		
		//控制模式，控制频率约50HZ
		if(ax_control_mode < CTL_FN1)
		{
			//控制方式选择，ROS模式不执行任何操作
			if(ax_control_mode == CTL_PS2)          AX_CTL_Ps2();    //PS2手柄			
			else if (ax_control_mode == CTL_APP)    AX_CTL_App();    //APP控制		
			else if (ax_control_mode == CTL_RMS)    AX_CTL_Rms();    //航模遥控器控制
			
			//舵机云台控制
			JOINT_Control();
		
			//延时
			vTaskDelay(20); 
		}
		else  //功能模式
		{
			//判断是否开启功能
			if(ax_function_enable != 0)
			{
				//不同模式差异化显示
				switch(ax_control_mode)
				{
					case CTL_FN1://功能-雷达避障行驶
						AX_FUN_List1();
						break;
					case CTL_FN2://功能-雷达跟随
						AX_FUN_List2();
						break;
					case CTL_FN3://功能-超声波避障
						AX_FUN_List3();
						break;			
					case CTL_FN4://功能-超声波跟随
						AX_FUN_List4();
						break;				
					case CTL_FN5://功能-CCD巡线
						AX_FUN_List5();
						break;
					default://空循环
						vTaskDelay(100);   
						break;
				}			
			}
			else
			{
				//延时
				vTaskDelay(100);   
			}			
		}                                
	}	
}

/**
  * @简  述  舵机云台控制函数
  * @参  数  无 
  * @返回值  无
  */
void JOINT_Control(void)   
{
	//幅度限制保护
	if(ax_joint_angle[0] > JOINTA_UP_LIMIT)	   ax_joint_angle[0] = JOINTA_UP_LIMIT;
	if(ax_joint_angle[0] < JOINTA_LOW_LIMIT)   ax_joint_angle[0] = JOINTA_LOW_LIMIT;
	
	if(ax_joint_angle[1] > JOINTB_UP_LIMIT)	   ax_joint_angle[1] = JOINTB_UP_LIMIT;
	if(ax_joint_angle[1] < JOINTB_LOW_LIMIT)   ax_joint_angle[1] = JOINTB_LOW_LIMIT;
	
	//设置舵机角度
	AX_SERVO_S1_SetAngle( ax_joint_angle[0] + JOINTA_ANGLE_OFFSET);
	AX_SERVO_S2_SetAngle( -ax_joint_angle[1] - JOINTB_ANGLE_OFFSET);	
	
	//查看舵机角度
	//printf("@%d %d \r\n ", ax_joint_angle[0], ax_joint_angle[1]);		
}

/**
  * @简  述  琐事管理任务
  * @参  数  无
  * @返回值  无
  */
void Trivia_Task(void* parameter)
{	
	//计数变量
	static uint16_t ax_bat_vol_cnt = 0; 
	

	while (1)
	{	
		
		/*****电池管理***********************************/
		
		//采集电池电压
	    ax_battery_vel = AX_VIN_GetVol_X100();
		
		//调试输出电池电压数据
        //printf("@ %d  \r\n",ax_battery_vel);		
		
		//电量低于40%
		if(ax_battery_vel < VBAT_40P)  
		{
			//红灯开始闪烁警示
			AX_LED_Red_Toggle();
			
			//电量低于20%
			if(ax_battery_vel < VBAT_20P)
			{
				//红灯常亮
				AX_LED_Red_On();
				
				//电量低于10%，关闭系统进入保护状态
				if(ax_battery_vel < VBAT_10P)
				{
					//低压时间计数
					ax_bat_vol_cnt++;
					
					//超过10次，进入关闭状态
					if(ax_bat_vol_cnt > 10 )
					{
						//关闭绿灯，红灯常亮
						AX_LED_Green_Off();
						AX_LED_Red_On();
						
						//任务挂起
						vTaskSuspend(Disp_Task_Handle);
						
						//关闭电机控制
						ax_robot_move_enable = 0; 
						
						//清除OLED启动画面显示
						AX_OLED_ClearScreen();  
						
						//蜂鸣器鸣叫报警
						while(1)
						{	
							//显示机器人停止信息
							AX_OLED_DispStr(0, 3, "     Low power      ", 0);	
							AX_OLED_DispStr(0, 5, "  Robot has stopped ", 0);	
							AX_BEEP_On();
							vTaskDelay(30);
							AX_BEEP_Off();
							
							vTaskDelay(1000);	
							AX_OLED_ClearScreen(); 
							vTaskDelay(1000);						
						}								
					}
				}
				else
				{
					ax_bat_vol_cnt = 0;
				}				
			}
		}
		else
		{
			//红灯关闭
			AX_LED_Red_Off();
		}				
		
		/*****蜂鸣器鸣叫管理***********************************/
		if(ax_beep_ring != 0)
		{
			if(ax_beep_ring == BEEP_SHORT)
			{
				//短鸣叫一声
				AX_BEEP_On();
				vTaskDelay(200); 
				AX_BEEP_Off();
				
				//鸣叫一声标志复位
				ax_beep_ring = 0;
			}
			else //
			{
				//长鸣叫一声
				AX_BEEP_On();
				vTaskDelay(1000); 
				AX_BEEP_Off();
				
				//鸣叫一声标志复位
				ax_beep_ring = 0;				
			}
		}
		
		//LED系统心跳指示
		AX_LED_Green_Toggle();	
		
		
        //循环周期500ms
		vTaskDelay(500); 
	}			
}

/**
  * @简  述  按键处理任务
  * @参  数  无
  * @返回值  无
  */
void Key_Task(void* parameter)
{	
	uint8_t  i;
	//int16_t  temp;
	
	while (1)
	{		
		//按键扫描
		if(AX_KEY_Scan() != 0)
		{
			//软件延时
			vTaskDelay(50);  
			
			//确定按键按下
			if(AX_KEY_Scan() != 0)
			{
				//等待按键抬起
				for(i=0; i<200; i++)
				{

					vTaskDelay(50);
					
					if(AX_KEY_Scan() == 0)
					{
						break;
					}
					
					//按键时长3S时，给出提示音
					if(i == 60)
					{
						AX_BEEP_On();
						vTaskDelay(200);
						AX_BEEP_Off();
					}
				}

				//短按检测,小于1S
				if(i < 20)
				{
					//功能模式关闭
					ax_function_enable = 0;
					AX_BEEP_Off();
					
					//速度设置为零
					ax_robot_vx = 0;
					ax_robot_vw = 0;
					
					//切换功能模式
					if(ax_control_mode < CTL_FN1)
					{
						ax_control_mode = CTL_FN1;
					}
					else
					{
						if(ax_control_mode < CTL_FN_MAX)
							ax_control_mode++;
						else
							ax_control_mode = CTL_FN1;
					}
					
					if(ax_control_mode <= CTL_FN4)
					{
						//打开雷达接收中断
						AX_LASER_SetEnable(LS_ENABLE);
					}
					else
					{
						//关闭雷达接收中断
						AX_LASER_SetEnable(LS_DISABLE);
					}
						
				}
				
				//中按检测,大于3S，小于10S
				if(i>60 && i<200)
				{
					//功能模式启动
					ax_function_enable = 1;
				}				
				
					
				//长按检测，大于10S
				if(i == 200)
				{
					//任务挂起					
				}
			}
		}
		
		//循环周期
		vTaskDelay(50);    
	}			
}


/**
  * @简  述  灯光处理任务
  * @参  数  无
  * @返回值  无
  */
void Light_Task(void* parameter)
{	

	while (1)
	{	
        //延时30ms，30HZ显示频率		
		vTaskDelay(30); 
		
		//判断前灯是否打开
		if(ax_light_enable != 0)
		{
			//正常显示
			AX_LIGHT_Show();
		}
		else
		{
			//关闭状态
			AX_RGB_SetFullColor(0x00, 0x00, 0x00);
		}
	}			
}


/**
  * @简  述  按键处理任务
  * @参  数  无
  * @返回值  无
  */
void Disp_Task(void* parameter)
{	

	while (1)
	{	
		//延时	
		vTaskDelay(100); 
		
		//显示电池电压，陀螺仪Z轴数据,舵机偏角,
		AX_OLED_DispValue(30, 2, (ax_battery_vel*0.1), 2, 1, 0);
		
		//显示角度和角速度
		AX_OLED_DispValue(30, 3, (ax_balance_angle), 2, 2, 0);
		AX_OLED_DispValue(90, 3, (ax_balance_gyro), 6, 0, 0);		
		
		//显示轮子实时速度
		AX_OLED_DispValue(30, 4, (ax_wheel_vel_l*100), 2, 2, 0);
		AX_OLED_DispValue(90, 4, (ax_wheel_vel_r*100), 2, 2, 0);
		
		//调试信息
		AX_OLED_DispValue(30, 7, (stX.x*1000), 4, 0, 0);
		AX_OLED_DispValue(90, 7, (stX.theta/PI*180), 4, 0, 0);
		
		//延时	
		vTaskDelay(100); 
		
		//不同模式差异化显示
		switch(ax_control_mode)
		{
			case CTL_PS2://PS2控制
				AX_OLED_DispStr(90, 2, "PS2", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);
				break;
			case CTL_APP://APP控制
				AX_OLED_DispStr(90, 2, "APP", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;
			case CTL_RMS://航模遥控器控制
				AX_OLED_DispStr(90, 2, "RMS", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;
			case CTL_ROS://ROS控制
				AX_OLED_DispStr(90, 2, "ROS", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;
			case CTL_FN1://功能-雷达避障行驶
				AX_OLED_DispStr(90, 2, "FN1", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;
			case CTL_FN2://功能-雷达跟随
				AX_OLED_DispStr(90, 2, "FN2", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;
			case CTL_FN3://功能-
				AX_OLED_DispStr(90, 2, "FN3", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;			
			case CTL_FN4://功能-
				AX_OLED_DispStr(90, 2, "FN4", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;				
			case CTL_FN5://功能-CCD巡线
				AX_OLED_DispStr(90, 2, "FN5", 0);  
			    AX_OLED_DispStr(0, 6, "                     ", 0);	
		        AX_OLED_DispStr((ax_ccd_offset+64), 6, "#", 0);	
				break;
			default:
				break;
		}
	}			
}

/**
  * @简  述  PS2数据获取任务
  * @参  数  无
  * @返回值  无
  */
void Ps2_Task(void* parameter)
{	
	while(1)
	{
		//读取PS2手柄键值
		AX_PS2_ScanKey(&ax_joystick);
		
		//判断是否开启PS2手柄控制
		//START按键被按下后，左边摇杆上推，进入PS2控制模式
		if((ax_joystick.btn1 == PS2_BT1_START) && (ax_joystick.LJoy_UD == 0x00))
		{
			//切换到PS2模式
			ax_control_mode = CTL_PS2;	

			//执行蜂鸣器鸣叫提示
			ax_beep_ring = BEEP_SHORT;
		}

		
		//延时	
		vTaskDelay(20); 
		
//		//打印手柄键值
//		printf("MODE:%2x BTN1:%2x BTN2:%2x RJOY_LR:%2x RJOY_UD:%2x LJOY_LR:%2x LJOY_UD:%2x\r\n",
//		ax_joystick.mode, ax_joystick.btn1, ax_joystick.btn2, 
//		ax_joystick.RJoy_LR, ax_joystick.RJoy_UD, ax_joystick.LJoy_LR, ax_joystick.LJoy_UD);
	}
}



/******************* (C) 版权 2023 XTARK **************************************/

