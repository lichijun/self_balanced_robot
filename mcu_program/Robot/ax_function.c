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

/* Includes ------------------------------------------------------------------*/
#include "ax_function.h"
#include "ax_robot.h"

/********雷达避障相关**************************/

#define LS_AVOID_Distance        400     //避障有效距离
#define LS_AVOID_Distance_min    250     //后退距离
#define LS_AVOID_Speed           300     //行驶速度
#define LS_AVOID_Turn            100     //转向速度

/**
  * @简  述  激光雷达避障行驶
  * @参  数  无
  * @返回值  无
  */
void AX_FUN_List1(void)
{
	uint8_t i;
	
	uint16_t min_distance = 65000;   //障碍物最小距离	
	uint16_t min_angle = 0;          //障碍物最小角度	
	uint16_t min_distance_cnt = 0;   //最小距离计数	
	
	//扫描前方障碍物角度和距离
	for(i = 0; i < 250; i++)
	{
		//避障有效角度300-60之间
		if((ax_ls_point[i].angle>30000) || (ax_ls_point[i].angle<6000))  
		{	
			//检测小于避障有效距离的数据
			if((ax_ls_point[i].distance >0)  && (ax_ls_point[i].distance < LS_AVOID_Distance))
			{
				//寻找最近障碍物
				if(ax_ls_point[i].distance < min_distance && ax_ls_point[i].distance>100)
				{
					//记录距离和角度
					min_distance = ax_ls_point[i].distance;
					min_angle = ax_ls_point[i].angle;
				}
				
				//记录距离小于后退距离的点数
				if(ax_ls_point[i].distance < LS_AVOID_Distance_min)
					min_distance_cnt++;
			}
		}
	}
	
	//打印调试信息
	//printf("@%d %d \r\n",min_angle, min_distance);
	
	//小车前进
	ax_robot_vx = LS_AVOID_Speed;
	
	//进入避障控制
	if(min_distance < LS_AVOID_Distance)
	{
		//距离太近小车倒退
		if(min_distance_cnt > 3)
		{
			ax_robot_vx = -LS_AVOID_Speed;
			ax_robot_vw = 0;
		}
		else 
		{
			//障碍物在右边
			if(min_angle < 6000)
			{
				ax_robot_vw = LS_AVOID_Turn;
			}
			
			//障碍物在左边
			if(min_angle > 30000)
			{
				ax_robot_vw = -LS_AVOID_Turn;
			}
		}
	}
	else
	{
		//前方没有障碍物，直行
		ax_robot_vw = 0;
	}

	//延时
	vTaskDelay(100); 
}




/********雷达警报相关**************************/

#define LS_ALARM_DistanceMin 150   //警报最小距离
#define LS_ALARM_DistanceMax 500   //警报最大距离

/**
  * @简  述  雷达警报功能
  * @参  数  无
  * @返回值  无
  */
void AX_FUN_List2(void)
{
	uint8_t i;
	
	uint16_t min_distance = 65000;   //最近物体距离，单位mm	
	uint16_t min_angle = 0;          //最近物体角度	
    int16_t  follow_angle = 0;		 //跟随角度	
	
	//找出最近物体
	for(i = 0; i < 250; i++)
	{
		//跟随有效角度300-60之间
		if((ax_ls_point[i].angle>30000) || (ax_ls_point[i].angle<6000))  
		{	
			//检测有效距离内的数据
			if((ax_ls_point[i].distance >LS_ALARM_DistanceMin) && (ax_ls_point[i].distance < LS_ALARM_DistanceMax))
			{	
				//寻找最近物体
				if(ax_ls_point[i].distance < min_distance)
				{
					min_distance = ax_ls_point[i].distance;
					min_angle = ax_ls_point[i].angle;
				}
			}
		}
	}
	
	//转换到±180°，单位度每秒
	if(min_angle > 18000)
		follow_angle = (min_angle - 36000)/100.0;	
	else
		follow_angle = min_angle/100.0;
	
	ax_robot_vx = 0;
	
	//判断是否有可疑物体
	if(min_angle != 0)
	{
		//蜂鸣器报警
		AX_BEEP_On();
		
		//比例控制
		ax_robot_vw = -3*follow_angle;
	}
	else
	{
		//蜂鸣器报警关闭
		AX_BEEP_Off();
		
		ax_robot_vw = 0;
	}
	
	//打印调试信息
	//printf("@%d %d \r\n",follow_angle, ax_robot_vw);

	//延时
	vTaskDelay(100); 
}



/********雷达跟随相关**************************/

#define LS_FOLLOW_DistanceMin  200   //有效最小距离
#define LS_FOLLOW_DistanceMax  1500   //有效最大距离
#define LS_FOLLOW_DistanceKeep 350   //雷达与物体保持距离

float ls_fangle_kp = 3.0f;   //跟随角度PID KP参数
float ls_fangle_kd = 1.0f;   //跟随角度PID KD参数
float ls_fdistance_kp = 1.0f;  //跟随距离PID KP参数
float ls_fdistance_kd = 0.5f;   //跟随距离PID KD参数

int16_t FOLLOW_Angle_PIDControl(float target, float current);  //跟随角度PID控制函数
int16_t FOLLOW_Distance_PIDControl(float target, float current);  //跟随距离PID控制函数

/**
  * @简  述  激光雷达跟随
  * @参  数  无
  * @返回值  无
  */
void AX_FUN_List3(void)
{
	uint8_t i;
	
	uint16_t min_distance = 65000;   //跟随距离	
	uint16_t min_angle = 0;          //跟随角度		
	
	static float follow_angle = 0;			//跟随角度
	static float follow_distance = 0;		//跟随距离	
	
	//找出跟随物体角度
	for(i = 0; i < 250; i++)
	{
		//跟随有效角度270-90之间
		if((ax_ls_point[i].angle>30000) || (ax_ls_point[i].angle<6000))  
		{	
			//检测有效距离内的数据
			if((ax_ls_point[i].distance >200) && (ax_ls_point[i].distance < 1000))
			{	
				//寻找最近物体
				if(ax_ls_point[i].distance < min_distance)
				{
					min_distance = ax_ls_point[i].distance;
					min_angle = ax_ls_point[i].angle;
				}
			}
		}
	}
	
	//转换到±180°，单位度每秒
	if(min_angle > 18000)
		follow_angle = (min_angle - 36000)/100.0;	
	else
		follow_angle = min_angle/100.0;
		
	//跟随距离改为浮点类型，单位mm
	follow_distance = min_distance;	
	 
	//打印调试信息
	//printf("@%d %d \r\n",follow_angle*100, follow_distance*100);
	
	//判断是否有有效跟随物体
	if(min_angle != 0)
	{
		//PID跟随程序
		ax_robot_vx = FOLLOW_Distance_PIDControl(LS_FOLLOW_DistanceKeep, follow_distance);  //跟随距离PID控制
		ax_robot_vw = FOLLOW_Angle_PIDControl(0, follow_angle);  //跟随角度PID控制
	}
	else
	{
		ax_robot_vx = 0;
		ax_robot_vw = 0;
	}

	//延时
	vTaskDelay(100); 
}

/**
  * @简  述  转向角度控制PID函数
  * @参  数  target  目标角度，单位度
  *          current 当前角度，单位度
  * @返回值  无
  */
int16_t FOLLOW_Angle_PIDControl(float target, float current)
{
	static float bias, output, last_bias;
	
	//计算偏差	
	bias = target - current;
	
	//计算PID输出转向控制量
	output = ls_fangle_kp*bias + ls_fangle_kd*(bias-last_bias);
	
	//记录上次偏差
	last_bias = bias;
	
	//打印调试信息
	//printf("%f %f \r\n",bias, output);
	
	return  (output);
}

/**
  * @简  述  跟随距离控制PID函数
  * @参  数  target  目标距离，单位mm
  *          current 当前距离，单位mm
  * @返回值  无
  */
int16_t FOLLOW_Distance_PIDControl(float target, float current)
{
	static float bias, output, last_bias;
	
	//计算偏差	
	bias = target - current;
	
	//计算PID输出转向控制量
	output = -ls_fdistance_kp*bias - ls_fdistance_kd*(bias-last_bias);
	
	//记录上次偏差
	last_bias = bias;
	
	//打印调试信息
	//printf("@%f %f \r\n",bias, output);
	
	return  (output);
	
}


/********雷达沿墙直线行驶相关**************************/

#define LS_LINE_Speed          200     //行驶速度
float ls_line_kp = 0.2f;  //直线行驶PID KP参数
float ls_line_kd = 0.1f;   //直线行驶PID KD参数

int16_t LINE_PIDControl(float target, float current);  //雷达直线行驶PID控制函数

/**
  * @简  述  激光雷达直线行驶
  * @参  数  无
  * @返回值  无
  */
void AX_FUN_List4(void)
{
	uint8_t i;
	static uint8_t cnt=0;
	
	//距离直线物体当前距离
	static uint16_t current_distance = 200; 
	static uint16_t last_distance = 200;
	
	//目标跟随距离
	static uint16_t target_distance = 200; 
	
	//找出跟随物体角度
	for(i = 0; i < 250; i++)
	{
		//取出小车右前方75度左右的雷达探测数据
		if((ax_ls_point[i].angle>7200) && (ax_ls_point[i].angle<7800))  
		{	
			//检测有效距离内的数据
			if(ax_ls_point[i].distance < (target_distance + 300))
			{	
				//有效测量
				current_distance = ax_ls_point[i].distance;			
				last_distance = current_distance;
			}
			else
			{   //无效测量数据，等于上一次测距数据
				current_distance = last_distance;
			}
		}
	}
	
	if(cnt > 30)
	{
		//小车前进
		ax_robot_vx = LS_LINE_Speed;
		
		//控制小车PID走直线
		ax_robot_vw = LINE_PIDControl(target_distance, current_distance);  
		 
		//打印调试信息
		//printf("@%d %d \r\n",current_distance,ax_robot_vw*10);		
	}
	else
	{
		//当前测量距离为设定墙壁距离
		target_distance = current_distance;
		
		cnt++;
	}
	
	//延时
	vTaskDelay(100); 
}

/**
  * @简  述  雷达直线行驶PID函数
  * @参  数  target  目标距离，单位mm
  *          current 当前距离，单位mm
  * @返回值  无
  */
int16_t LINE_PIDControl(float target, float current)
{
	static float bias, output, last_bias;
	
	//计算偏差	
	bias = target - current;
	
	//计算PID输出转向控制量
	output = ls_line_kp*bias + ls_line_kd*(bias-last_bias);
	
	//记录上次偏差
	last_bias = bias;
	
	//打印调试信息
	//printf("@%f %f \r\n",bias, output);
	
	return  (output);
	
}


/********CCD巡线行驶相关**************************/
 
int16_t ax_ccd_speed = 300;  //CCD巡线速度，单位mm/s
int16_t ax_ccd_offset;   //CCD巡线黑线位置偏差
int16_t ax_ccd_kp = 50;  //CCD巡线PID参数kp
int16_t ax_ccd_kd = 20;  //CCD巡线PID参数kd


/**
  * @简  述  功能-CCD巡线功能
  * @参  数  无
  * @返回值  无
  */
void AX_FUN_List5(void)
{
	static float bias,bias_last;
	float move_w=0;   

	//设置CCD巡线速度
	ax_robot_vx = ax_ccd_speed;
	
	//获取CCD采集并计算的黑线偏差值
	ax_ccd_offset = AX_CCD_GetOffset();
	bias = ax_ccd_offset;
	
	//计算PID输出转向控制量
	move_w = -ax_ccd_kp*bias*0.01f - ax_ccd_kd*(bias-bias_last)*0.01f;
	
	//计算转向速度，与前进速度相关
	ax_robot_vw  =  0.01f* move_w*ax_ccd_speed;	
	
	//记录上次偏差
	bias_last = bias;
	
	//打印调试信息
	//printf("@%d %d \r\n",ax_ccd_offset, ax_robot_vw);
	
	//延时
	vTaskDelay(20); 
}

/******************* (C) 版权 2023 XTARK **************************************/

