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

/* Includes ------------------------------------------------------------------*/
#include "ax_balance.h"
#include "ax_robot.h"

//内部函数定义
int16_t Balance_AngleCtl(int16_t target, float angle, float gyro);  
int16_t Balance_VelocityCtl(int16_t t_vx, float r_vx ); 
int16_t Balance_TurnCtl(int16_t t_vw, float r_vw);  
void Balance_FallProtect(void);
void Balance_PutDown(void);
void ROS_SendDataHandle(void); 
void DBG_SendDataHandle(void); 

void SetStateVar(StateVariable* pState, int16_t odoNumL, int16_t odoNumR, 
			     float wheel_vel_l, float wheel_vel_r, float phi, float phi_dot,
				 float theta, float theta_dot)
{
	pState->odoNumL += odoNumL;
	pState->odoNumR += odoNumR;
	pState->x = (pState->odoNumL + pState->odoNumR) / 2 * (PI*WHEEL_DIAMETER/WHEEL_RESOLUTION);
	pState->wheel_vel_l = wheel_vel_l;
	pState->wheel_vel_r = wheel_vel_r;
	pState->x_dot = (wheel_vel_l + wheel_vel_r) / 2;
	pState->phi = phi / 180 * PI;
	pState->phi_dot = phi_dot / 180 * PI;
	pState->theta = theta / 180 * PI;
	pState->theta_dot = theta_dot / 180 * PI;
}

void ResetStateVar(StateVariable* pState)
{
	memset(pState, 0, sizeof(StateVariable));
}

// pwm: -100~100
void BalanceCtrlLqr(StateVariable* pState, float* pPwmL, float* pPwmR)
{
	float wheel_mess = 0.104; // 两个轮子的重量
	float accL, accR;
	MotorCharacterCoef motorParaL = {-17.41, 0.25, -0.24}; // 空载测试数据回归得到
	MotorCharacterCoef motorParaR = {-19.93, 0.26, -0.04}; // 空载测试数据回归得到
#ifdef LQR_4_STATES
	// float K[4] = {-5.4650, -3.2293, 16.7392, 0.8214}; // Q dia[50, 0, 20, 0]
	float K[4] = {-3.4470, -2.5848, 16.8864, 0.7930}; // Q dia[20, 0, 40, 0]
	float u = - K[0] * pState->x - K[1] * pState->x_dot 
			  - K[2] * pState->phi - K[3] * pState->phi_dot; 
	accL = u / wheel_mess / 2; // 每个轮子需要提供的加速度
	accR = u / wheel_mess / 2;
#elif defined(LQR_6_STATES)
	// float K_Row1[6] = {-5.4650, -3.2293, 16.7392, 0.8214, 0, 0}; // Q dia[50, 0, 20, 0, 10, 0]
	// float K_Row2[6] = {0, 0, 0, 0, 2.9062, 0.3589};
	float K_Row1[6] = {-3.4470, -2.5848, 16.8864, 0.7930, 0, 0}; // Q dia[20, 0, 40, 0, 20, 0]
	float K_Row2[6] = {0, 0, 0, 0, 4.0449, 0.4234};
	float u1 = - K_Row1[0] * pState->x - K_Row1[1] * pState->x_dot 
			   - K_Row1[2] * pState->phi - K_Row1[3] * pState->phi_dot
			   - K_Row1[4] * pState->theta - K_Row1[5] * pState->theta_dot; 
	float u2 = - K_Row2[0] * pState->x - K_Row2[1] * pState->x_dot 
			   - K_Row2[2] * pState->phi - K_Row2[3] * pState->phi_dot
			   - K_Row2[4] * pState->theta - K_Row2[5] * pState->theta_dot; 
	accL = (u1/2 - u2) / wheel_mess; // 每个轮子需要提供的加速度
	accR = (u1/2 + u2) / wheel_mess;
#endif
	*pPwmL = (accL - motorParaL.a * pState->wheel_vel_l - motorParaL.c) / motorParaL.b;
	*pPwmR = (accR - motorParaR.a * pState->wheel_vel_r - motorParaR.c) / motorParaR.b;
	if (*pPwmL > 100)
		*pPwmL = 100;
	if (*pPwmR > 100)
		*pPwmR = 100;
	printf("pwm %.3f,%.3f wheel %.3f,%.3f\r\n", *pPwmL, *pPwmR, pState->wheel_vel_l, pState->wheel_vel_r);
	printf("x %.3f,%.3f phi %.3f,%.3f\r\n", pState->x, pState->x_dot, pState->phi, pState->phi_dot);
}

/**
  * @简  述  平衡小车控制核心代码
             MPU6050的INT中断触发，10ms一次，严格时序同步
  * @参  数  无
  * @返回值  无
  */
void EXTI9_5_IRQHandler(void)
{
	//平衡角度，角速度相关浮点数变量
	static float balance_angle,balance_gyro,turn_gyro,turn_angle;
	static float wheel_vel;
	static uint8_t cnt=0;
	int16_t wheel_pwm_l,wheel_pwm_r;
	int16_t odoNumL, odoNumR;
	
	//确认中断,进入控制周期
	if(EXTI_GetITStatus(EXTI_Line5) != RESET) 
	{		
		//清除中断标志位
		EXTI_ClearITPendingBit(EXTI_Line5);
		
		//获取IMU数据
		AX_MPU6050_DMP_GetData(ax_gyro_data,ax_acc_data,ax_angle_data);  //读取IMU数据
		ax_balance_angle = ax_angle_data[0];  //平衡倾角，前倾负，单位0.01度
		ax_balance_gyro  = ax_gyro_data[0];   //平衡角速度，前倾负，量程±2000度/s，范围±32768，一个数字代表2000/32768=0.061，可查手册
		ax_turn_angle    = ax_angle_data[2];  //航向角，单位0.01度
		ax_turn_gyro     = ax_gyro_data[2];   //转向角速度，逆时针正，量程±2000度/s，范围±32768，一个数字代表2000/32768=0.061，可查手册

		//换算到实际浮点数
		balance_angle = 0.01f * ax_balance_angle;   //倾角浮点数，单位度    
		balance_gyro  = 0.061f * ax_balance_gyro;   //平衡角速度浮点数，单位度/s
		turn_angle    = 0.01f * ax_turn_angle;   //航向角浮点数，单位度  
		turn_gyro     = 0.061f * ax_turn_gyro;      //转动角速度浮点数，单位度/s	
		
		//通过编码器获取车轮实时转速m/s，前进正，后退负
		ax_wheel_vel_l = (float) ((int16_t)AX_ENCODER_A_GetCounter()*WHEEL_SCALE);  //左轮转速
		odoNumL = (int16_t)AX_ENCODER_A_GetCounter();
		AX_ENCODER_A_SetCounter(0);
		ax_wheel_vel_r = (float)-((int16_t)AX_ENCODER_B_GetCounter()*WHEEL_SCALE);  //右轮转速
		odoNumR = -(int16_t)AX_ENCODER_B_GetCounter();
		AX_ENCODER_B_SetCounter(0);	
		
		//计算小车速度
		wheel_vel = (ax_wheel_vel_l + ax_wheel_vel_r)/2;
		ax_velocity = wheel_vel*1000;  //转化为整数，代表实时速度数据

		//PID运动控制
		ax_balance_out  = Balance_AngleCtl(ax_middle_angle,balance_angle, balance_gyro);  //角度控制
	    ax_velocity_out = Balance_VelocityCtl(ax_robot_vx, wheel_vel);   //行驶速度控制
	    ax_turn_out     = Balance_TurnCtl(ax_robot_vw, turn_gyro);	   //转向速度控制	

		//计算左右轮速度值
		wheel_pwm_l  = ax_balance_out + ax_velocity_out - ax_turn_out;
		wheel_pwm_r  = ax_balance_out + ax_velocity_out + ax_turn_out;
		
		//LQR运动控制
		if(ax_robot_move_enable == 1)
		{
			float pwmL, pwmR;
			SetStateVar(&stX, odoNumL, odoNumR, ax_wheel_vel_l, ax_wheel_vel_r, 
				        balance_angle, balance_gyro, turn_angle, turn_gyro);
			BalanceCtrlLqr(&stX, &pwmL, &pwmR);
			wheel_pwm_l = pwmL / 100 * 3600;
			wheel_pwm_r = pwmR / 100 * 3600;
		}
		else
		{
			ResetStateVar(&stX);
		}
			
		//电机是否运行由该标志位控制
		if(ax_robot_move_enable == 1)
		{	
			AX_MOTOR_A_SetSpeed( -wheel_pwm_l);//控制左侧电机
			AX_MOTOR_B_SetSpeed(  wheel_pwm_r);//控制右侧电机
		}
		else
		{
			//电机速度设置为0
			AX_MOTOR_A_SetSpeed(0);
			AX_MOTOR_B_SetSpeed(0);
		}
		
		//安全放下启动，小车直立（角度小于5度），轮子有转动
		Balance_PutDown();
		
		//异常动作摔倒保护，拿起保护（角度大于45停止）
		Balance_FallProtect();
		
		//发送数据, 50HZ发送频率
		if(cnt != 0)
		{
			//ROS模式下发送ROS数据
			if(ax_control_mode == CTL_ROS)
			{
				ROS_SendDataHandle();
			}
			
			//非ROS，控制模式下发送平衡调试数据，功能模式不发送
			else
			{
				//判断是否在控制模式下
				if(ax_control_mode < CTL_ROS)
				{
					DBG_SendDataHandle();
				}
			}
			
		   cnt = 0;
		}
		else
		{
			cnt++;
		}
		
		
	    //调试数据查看
		//printf("@%d %d %d \r\n ", ax_balance_angle, ax_balance_gyro, ax_balance_out); //小车角度数据
	    //printf("@%d %d %d \r\n ", ax_balance_angle, ax_balance_gyro, ax_turn_gyro); //小车角度数据
		//printf("@%d %d %d  \r\n ", ax_balance_out, ax_velocity_out,ax_turn_out); //小车控制输出数据
	}
}


/**
  * @简  述  平衡小车直立PID控制
  * @参  数  target 目标平衡角度，小车机械中间角度，默认为0
  *          angle  当前平衡角度，单位度
  *          gyro   当前平衡角速度，单位度/s
  * @返回值  PID控制输出值电机PWM值
  */
int16_t Balance_AngleCtl(int16_t target, float angle, float gyro)
{ 
	
	float angle_bias,gyro_bias;  //角度和角速度偏差
	int16_t p_out,d_out,out;    //PID输出值
	
	//计算平衡角度偏差
	angle_bias = target - angle;  
	
	//计算平衡角速度偏差
	gyro_bias  = 0 - gyro;  
	
	//计算平衡角度控制PWM输出，pk，pd为系数，out = kp*e(k) + kd*[e(k)-e(k-1)]
	p_out = ax_balance_kp * angle_bias * 0.1f;
	d_out = ax_balance_kd * gyro_bias * 0.1f;
	
	//计算总输出值
	out =  p_out + d_out;
	
	//调试数据查看
	//printf("@%d %d %d \r\n ",out, p_out, d_out); 
	
	return out;
}

/**
  * @简  述  速度PID控制
  * @参  数  t_vx 目标速度，单位为mm/s，范围0~700
  *          r_vx 实时速度，单位m/s
  * @返回值  PID控制输出值电机PWM值
  */
int16_t Balance_VelocityCtl(int16_t t_vx, float r_vx)
{ 
	
	static float velocity_bias = 0, velocity_bias_update;
	static float velocity_integral = 0;
	float  velocity_target;  //速度目标值
	int16_t p_out,i_out,out; //PID输出值
	
	//目标速度限幅
	if(t_vx > 700) t_vx = 700;
	else if(t_vx <-700) t_vx = -700;
	
	//计算速度偏差，目标速度为0
	velocity_bias_update = 0 - r_vx;
	
	//一阶低通滤波器, 减小速度波动对平衡控制影响
	velocity_bias = (velocity_bias*0.6) + (velocity_bias_update*0.4); 
	
	//速度积分计算出位移，单位m，积分时间10ms
	velocity_integral += velocity_bias*0.01f;  
	
	//计算目标速度（单位为m/s）,vx单位为mm/s,积分时间0.01s
	velocity_target = t_vx*0.001f*0.01f;
	
	//加入目标速度控制，控制行驶速度
	velocity_integral += velocity_target; 
	
	//如果电机停止，则清除积分项
	if(ax_robot_move_enable == 0)
	{
		velocity_integral = 0;
	}
	
	//限制积分饱和
	if(velocity_integral > 1.5)  velocity_integral = 1.5;
	else if(velocity_integral < -1.5)  velocity_integral = -1.5;

	//计算速度控制PWM输出，pk，pi为系数
	p_out = -ax_velocity_kp * velocity_bias;
	i_out = -ax_velocity_ki * velocity_integral;
	
	//计算总输出值
	out =  p_out + i_out;
	
    //调试数据查看
	//printf("@%d %d %d \r\n ", out, p_out,i_out);   
	//printf("@%d %d \r\n ",vx, (int)(-velocity_bias_update*1000) );
	
	return out;

}


/**
  * @简  述  转向PID控制
  * @参  数  t_vw 目标旋转速度，逆时针为正，单位度/s
                  范围0~360相对安全，过快速度会破坏平衡
  *          r_vw 实时旋转速度，陀螺仪转向速度，单位度/s
  * @返回值  PID控制输出值电机PWM值
  */
int16_t Balance_TurnCtl(int16_t t_vw, float r_vw)
{
	static float bias = 0;
	static float bias_last = 0;
	
	//PID输出值
	static int16_t p_out=0,d_out=0,out=0;  


	//如果电机停止，则清除转向输出
	if(ax_robot_move_enable == 0)
	{
		out = 0;
	}	

	//计算转向速度偏差
	bias = t_vw - r_vw;
	
	//计算平衡转向控制PWM输出
	p_out =  ax_turn_kp * bias * 0.001f;  
    d_out =  ax_turn_kd * (bias-bias_last) * 0.001f;
	
	//计算总输出值
	out +=  p_out + d_out;
	
	//记录上次偏差
	bias_last = bias;
	
    //调试数据查看
	//printf("@%d %d %d \r\n ", (int)(bias*10), (int)(t_vw*10), (int)(r_vw*10)); 
	//printf("@%d %d %d \r\n ", out, p_out,d_out);   
	
	return out;
}

/**
  * @简  述  平衡小车放下检测
  *          小车直立放下（角度小于5度）,轮子有轻微转动
  * @参  数  无
  * @返回值  无
  */
void Balance_PutDown(void)
{
	static uint16_t cont;
	int16_t wheel_l,wheel_r;
	
	//获取左右轮转速
	wheel_l = (int16_t)(ax_wheel_vel_l*1000);
	wheel_r = (int16_t)(ax_wheel_vel_r*1000);
	
	//小车平衡系统关闭状态下
	if(ax_robot_move_enable == 0)
	{
		if(cont == 0)
		{
			//判断1，角度小于5度，编码器速度为0
			if(ax_balance_angle<500 && ax_balance_angle>-500 && wheel_l==0 &&  wheel_r==0)
			{
				cont++;
			}		
		}
		else
		{			
			if(cont<50)
			{
				cont++;
				
				//判断2，轻轻前后推动，左右轮产生速度
				if(wheel_l>1 || wheel_l<-1 || wheel_r>1 || wheel_r<-1) 
				{
					//速度设置为零
					ax_robot_vx = 0;
					ax_robot_vw = 0;
					
					//打开电机使能
					ax_robot_move_enable = 1;   
					
					//清理计数器
					cont = 0;
				}
			}
			else  //超时500ms退出
			{
				//清理计数器
				cont = 0;
			}
		}		
	}
}

/**
  * @简  述  小车摔倒保护，角度大于45度，认为小车摔倒
  * @参  数  无
  * @返回值  无
  */
void Balance_FallProtect(void)
{
	//倾角大于45度进入保护状态
	if((ax_balance_angle>4500)||(ax_balance_angle<-4500))
	{
		//电机运动使能
		ax_robot_move_enable=0;
	}
}

/**
  * @简  述  ROS处理函数，包含IMU数据封装、速度正解和数据发送
  * @参  数  无
  * @返回值  无
  */
void ROS_SendDataHandle(void)   
{
	int16_t temp[3];
	
	//串口发送数据
	static uint8_t comdata[20]; 	

	//加速度 = (ax_acc/32768) * 2G 
	temp[0] =  ax_acc_data[1];   //ROS坐标X轴对应IMU的Y轴
	temp[1] = -ax_acc_data[0];   //ROS坐标Y轴对应IMU的X轴反向
	temp[2] =  ax_acc_data[2];   //ROS坐标Z轴对应IMU的Z轴
	
	comdata[0] = (u8)( temp[0] >> 8 );  
	comdata[1] = (u8)( temp[0] );
	comdata[2] = (u8)( temp[1] >> 8 );
	comdata[3] = (u8)( temp[1] );
	comdata[4] = (u8)( temp[2] >> 8 );
	comdata[5] = (u8)( temp[2] );
	
	//陀螺仪角速度 = (ax_gyro/32768) * 2000
	temp[0] =  ax_gyro_data[1];   //ROS坐标X轴对应IMU的Y轴
	temp[1] = -ax_gyro_data[0];   //ROS坐标Y轴对应IMU的X轴反向
	temp[2] =  ax_gyro_data[2];   //ROS坐标Z轴对应IMU的Z轴	

	comdata[6] = (u8)( temp[0] >> 8 );  
	comdata[7] = (u8)( temp[0] );
	comdata[8] = (u8)( temp[1] >> 8 );
	comdata[9] = (u8)( temp[1] );
	comdata[10] = (u8)( temp[2] >> 8 );
	comdata[11] = (u8)( temp[2] );
	
	//运动学正解析，由机器人轮子速度计算机器人速度
	//机器人速度值 单位为m/s，放大1000倍
	temp[0] = ((ax_wheel_vel_l + ax_wheel_vel_r)/2)*1000;  //机器人实时X速度
	temp[1] = 0;  //机器人实Y速度
	temp[2] = ((-ax_wheel_vel_l + ax_wheel_vel_r)/WHEEL_BASE)*1000;		//机器人实W速度
	
	comdata[12] = (u8)( temp[0] >> 8 );  
	comdata[13] = (u8)( temp[0] );
	comdata[14] = (u8)( temp[1] >> 8 );
	comdata[15] = (u8)( temp[1] );
	comdata[16] = (u8)( temp[2] >> 8 );
	comdata[17] = (u8)( temp[2] );	

	//电池电压
	comdata[18] = (u8)( ax_battery_vel >> 8 );
	comdata[19] = (u8)( ax_battery_vel );	
	
	//USB串口发送数据（与调试串口共用，需要调试时可注释掉该句）
    // AX_UART1_SendPacket(comdata, 20, ID_UTX_ROS);	
		
}

/**
  * @简  述  调试数据，包含平衡角度、角速度、控制量等数据发送
  * @参  数  无
  * @返回值  无
  */
void DBG_SendDataHandle(void)   
{
	int16_t temp[2];
	
	//串口发送数据
	static uint8_t comdata[14]; 	

	//平衡控制相关
	temp[0] =  ax_balance_angle;  //平衡角度
	temp[1] =  ax_balance_gyro;   //平衡角度度，陀螺仪测量
	
	comdata[0] = (u8)( temp[0] >> 8 );  
	comdata[1] = (u8)( temp[0] );
	comdata[2] = (u8)( temp[1] >> 8 );
	comdata[3] = (u8)( temp[1] );

	//速度控制数据
	temp[0] =  ax_robot_vx;   //目标速度数据，设定值
	temp[1] =  ax_velocity;   //实际速度数据，编码器测量值

	comdata[4] = (u8)( temp[0] >> 8 );  
	comdata[5] = (u8)( temp[0] );
	comdata[6] = (u8)( temp[1] >> 8 );
	comdata[7] = (u8)( temp[1] );
	
	//转向控制数据
	temp[0] = ax_robot_vw * 10;  //目标转向速度数据，设定值
	temp[1] = 0.61f * ax_turn_gyro;  //实际转向速度数据，陀螺仪测量值
	
	comdata[8] = (u8)( temp[0] >> 8 );  
	comdata[9] = (u8)( temp[0] );
	comdata[10] = (u8)( temp[1] >> 8 );
	comdata[11] = (u8)( temp[1] );
	
	//USB串口发送数据（与调试串口共用，需要调试时可注释掉该句）
    // AX_UART1_SendPacket(comdata, 12, ID_UTX_DBG);	
		
}

/******************* (C) 版权 2023 XTARK **************************************/

