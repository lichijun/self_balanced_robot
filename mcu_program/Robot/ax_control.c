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
  * @内  容  机器人控制处理文件
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ax_control.h"
#include "ax_robot.h"

/**
  * @简  述  处理PS2手柄控制命令
  * @参  数  无
  * @返回值  无
  */
void AX_CTL_Ps2(void)
{
	//按键按下标记
	static uint8_t btn_select_flag = 0;
	static uint8_t btn_joyl_flag = 0;
	static uint8_t btn_joyr_flag = 0;
	static uint8_t btn_l1_flag = 0;
	static uint8_t btn_l2_flag = 0;
	
	static uint8_t  robot_amplitude = 5;   //运行速度，1~6
	static uint16_t joint_amplitude = 15;  //云台关节速度
	
	//红绿灯模式下，执行控制操作
	if(ax_joystick.mode ==  0x73)
	{
		//机器人速度控制
		ax_robot_vx = (int16_t)(      robot_amplitude * (0x80 - ax_joystick.RJoy_UD));
		ax_robot_vw = (int16_t)(0.3 * robot_amplitude * (0x80 - ax_joystick.LJoy_LR));
		
		//SELECT按键，切换RGB灯效模式
		if(ax_joystick.btn1 & PS2_BT1_SELECT)
		{
			btn_select_flag = 1;
		}
		else
		{
			if(btn_select_flag)
			{
				//灯光效果切换
				if(R_Light.M < LEFFECT6)
					R_Light.M++;
				else
					R_Light.M = LEFFECT1;
				
				//复位标记
				btn_select_flag = 0;
			}
		}
		
		//左摇杆按键，减速
		if(ax_joystick.btn1 & PS2_BT1_JOY_L)
		{
			btn_joyl_flag = 1;
		}
		else
		{
			if(btn_joyl_flag)
			{
				
				//速度减小
				if(robot_amplitude > 1)
				{
					robot_amplitude--;
				}
				else
				{
					robot_amplitude = 1;
					
					//蜂鸣器鸣叫提示
					ax_beep_ring = BEEP_SHORT;
				}
					
				//复位标记
				btn_joyl_flag = 0;
			}
		}
		
		//右摇杆按键，加速
		if(ax_joystick.btn1 & PS2_BT1_JOY_R)
		{
			btn_joyr_flag = 1;
		}
		else
		{
			if(btn_joyr_flag)
			{
				//速度增加
				if(robot_amplitude < 6)
				{
					robot_amplitude++;
				}
				else
				{
					robot_amplitude = 6;
					
					//蜂鸣器鸣叫提示
					ax_beep_ring = BEEP_SHORT;					
				}
					
				//复位标记
				btn_joyr_flag = 0;
			}
		}
		
		
		
		//左右按键控制关节1
		if(ax_joystick.btn1 & PS2_BT1_LEFT)
		{
			ax_joint_angle[0] += joint_amplitude; 
		}
		else if(ax_joystick.btn1 & PS2_BT1_RIGHT)
		{
			ax_joint_angle[0] -= joint_amplitude; 
		}
			
		//上下按键控制关节2
		if(ax_joystick.btn1 & PS2_BT1_DOWN)
		{
			ax_joint_angle[1] += joint_amplitude; 
		}
		else if(ax_joystick.btn1 & PS2_BT1_UP)
		{
			ax_joint_angle[1] -= joint_amplitude; 
		}
		
		//L1按键，机械臂加速
		if(ax_joystick.btn2 & PS2_BT2_L1)
		{
			btn_l1_flag = 1;
		}
		else
		{
			if(btn_l1_flag)
			{
				//执行动作，
				if(joint_amplitude < 25)
				{
					joint_amplitude =  joint_amplitude + 5;
				}
				else
				{
					joint_amplitude = 25;
					
					//蜂鸣器鸣叫提示
					ax_beep_ring = BEEP_SHORT;					
				}
				
				//复位标记
				btn_l1_flag = 0;
			}
		}
		
		//L2按键，机械臂减速
		if(ax_joystick.btn2 & PS2_BT2_L2)
		{
			btn_l2_flag = 1;
		}
		else
		{
			if(btn_l2_flag)
			{
				//执行动作
				if(joint_amplitude > 5)
				{
					joint_amplitude =  joint_amplitude -5;
				}
				else
				{
					joint_amplitude = 5;
					
					//蜂鸣器鸣叫提示
					ax_beep_ring = BEEP_SHORT;
				}
				
				//复位标记
				btn_l2_flag = 0;
			}
		}	
	}
}	

/**
  * @简  述  处理手机APP控制命令
  * @参  数  无
  * @返回值  无
  */
void AX_CTL_App(void)
{
	static uint8_t comdata[16];
	
	//接收蓝牙APP串口数据
	if(AX_UART2_GetData(comdata))
	{
		//摇杆模式运动控制帧
		if((comdata[0] == ID_BLERX_YG))
		{
			ax_robot_vx = (int16_t)(  5*(int8_t)comdata[4] );
			ax_robot_vw = (int16_t)( -2*(int8_t)comdata[1] );
		}
		
		//手柄模式运动控制帧
		if((comdata[0] == ID_BLERX_SB))
		{
			ax_robot_vx = (int16_t)(  5*(int8_t)comdata[4] );
			ax_robot_vw = (int16_t)( -2*(int8_t)comdata[1] );
		}
		
		//重力模式运动控制帧
		if(comdata[0] == ID_BLERX_ZL)
		{
			ax_robot_vx = (int16_t)( -5*(int8_t)comdata[3] );
			ax_robot_vw = (int16_t)( -2*(int8_t)comdata[2] );
		}
		
		//灯光效果控制帧
		if(comdata[0] == ID_BLERX_LG)
		{
			R_Light.M  = comdata[1];
			R_Light.S  = comdata[2];
			R_Light.T  = comdata[3];
			R_Light.R  = comdata[4];
			R_Light.G  = comdata[5];
			R_Light.B  = comdata[6];
		}
	}
}

/**
  * @简  述  处理SBUS航模遥控器控制命令
  * @参  数  无
  * @返回值  无
  */
void AX_CTL_Rms(void)
{
	static uint16_t sbusdata[8];
	static float amplitude;
	
	//获取SBUS解码数据
	if(AX_SBUS_GetRxData(sbusdata))
	{
		//SWD-8（通道7）拨杆到上方生效
		if(sbusdata[7] < 500)
		{
			//SWA-5拨杆设置速度
			if(sbusdata[4] < 500)          amplitude = 1.0;
			else if(sbusdata[4] < 1000)    amplitude = 0.7;
			else                           amplitude = 0.5;
			
			//运动控制
			ax_robot_vx = (int16_t)(       amplitude * (sbusdata[1] - 992));
			ax_robot_vw = (int16_t)(-0.4 * amplitude * (sbusdata[3] - 992));
		}
		
		//查看各个通道数值
		//printf("c1=%04d c2=%04d c3=%04d ch4=%04d ch5=%04d ch6=%04d ch7=%04d ch8=%04d \r\n",sbusdata[0],
	    //       sbusdata[1],sbusdata[2],sbusdata[3],sbusdata[4],sbusdata[5],sbusdata[6],sbusdata[7]);
	}		
		
	
}

/******************* (C) 版权 2023 XTARK **************************************/

