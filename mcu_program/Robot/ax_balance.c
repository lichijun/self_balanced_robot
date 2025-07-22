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

/* Includes ------------------------------------------------------------------*/
#include "ax_balance.h"
#include "ax_robot.h"

//�ڲ���������
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
	float wheel_mess = 0.104; // �������ӵ�����
	float accL, accR;
	MotorCharacterCoef motorParaL = {-17.41, 0.25, -0.24}; // ���ز������ݻع�õ�
	MotorCharacterCoef motorParaR = {-19.93, 0.26, -0.04}; // ���ز������ݻع�õ�
#ifdef LQR_4_STATES
	// float K[4] = {-5.4650, -3.2293, 16.7392, 0.8214}; // Q dia[50, 0, 20, 0]
	float K[4] = {-3.4470, -2.5848, 16.8864, 0.7930}; // Q dia[20, 0, 40, 0]
	float u = - K[0] * pState->x - K[1] * pState->x_dot 
			  - K[2] * pState->phi - K[3] * pState->phi_dot; 
	accL = u / wheel_mess / 2; // ÿ��������Ҫ�ṩ�ļ��ٶ�
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
	accL = (u1/2 - u2) / wheel_mess; // ÿ��������Ҫ�ṩ�ļ��ٶ�
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
  * @��  ��  ƽ��С�����ƺ��Ĵ���
             MPU6050��INT�жϴ�����10msһ�Σ��ϸ�ʱ��ͬ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void EXTI9_5_IRQHandler(void)
{
	//ƽ��Ƕȣ����ٶ���ظ���������
	static float balance_angle,balance_gyro,turn_gyro,turn_angle;
	static float wheel_vel;
	static uint8_t cnt=0;
	int16_t wheel_pwm_l,wheel_pwm_r;
	int16_t odoNumL, odoNumR;
	
	//ȷ���ж�,�����������
	if(EXTI_GetITStatus(EXTI_Line5) != RESET) 
	{		
		//����жϱ�־λ
		EXTI_ClearITPendingBit(EXTI_Line5);
		
		//��ȡIMU����
		AX_MPU6050_DMP_GetData(ax_gyro_data,ax_acc_data,ax_angle_data);  //��ȡIMU����
		ax_balance_angle = ax_angle_data[0];  //ƽ����ǣ�ǰ�㸺����λ0.01��
		ax_balance_gyro  = ax_gyro_data[0];   //ƽ����ٶȣ�ǰ�㸺�����̡�2000��/s����Χ��32768��һ�����ִ���2000/32768=0.061���ɲ��ֲ�
		ax_turn_angle    = ax_angle_data[2];  //����ǣ���λ0.01��
		ax_turn_gyro     = ax_gyro_data[2];   //ת����ٶȣ���ʱ���������̡�2000��/s����Χ��32768��һ�����ִ���2000/32768=0.061���ɲ��ֲ�

		//���㵽ʵ�ʸ�����
		balance_angle = 0.01f * ax_balance_angle;   //��Ǹ���������λ��    
		balance_gyro  = 0.061f * ax_balance_gyro;   //ƽ����ٶȸ���������λ��/s
		turn_angle    = 0.01f * ax_turn_angle;   //����Ǹ���������λ��  
		turn_gyro     = 0.061f * ax_turn_gyro;      //ת�����ٶȸ���������λ��/s	
		
		//ͨ����������ȡ����ʵʱת��m/s��ǰ���������˸�
		ax_wheel_vel_l = (float) ((int16_t)AX_ENCODER_A_GetCounter()*WHEEL_SCALE);  //����ת��
		odoNumL = (int16_t)AX_ENCODER_A_GetCounter();
		AX_ENCODER_A_SetCounter(0);
		ax_wheel_vel_r = (float)-((int16_t)AX_ENCODER_B_GetCounter()*WHEEL_SCALE);  //����ת��
		odoNumR = -(int16_t)AX_ENCODER_B_GetCounter();
		AX_ENCODER_B_SetCounter(0);	
		
		//����С���ٶ�
		wheel_vel = (ax_wheel_vel_l + ax_wheel_vel_r)/2;
		ax_velocity = wheel_vel*1000;  //ת��Ϊ����������ʵʱ�ٶ�����

		//PID�˶�����
		ax_balance_out  = Balance_AngleCtl(ax_middle_angle,balance_angle, balance_gyro);  //�Ƕȿ���
	    ax_velocity_out = Balance_VelocityCtl(ax_robot_vx, wheel_vel);   //��ʻ�ٶȿ���
	    ax_turn_out     = Balance_TurnCtl(ax_robot_vw, turn_gyro);	   //ת���ٶȿ���	

		//�����������ٶ�ֵ
		wheel_pwm_l  = ax_balance_out + ax_velocity_out - ax_turn_out;
		wheel_pwm_r  = ax_balance_out + ax_velocity_out + ax_turn_out;
		
		//LQR�˶�����
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
			
		//����Ƿ������ɸñ�־λ����
		if(ax_robot_move_enable == 1)
		{	
			AX_MOTOR_A_SetSpeed( -wheel_pwm_l);//���������
			AX_MOTOR_B_SetSpeed(  wheel_pwm_r);//�����Ҳ���
		}
		else
		{
			//����ٶ�����Ϊ0
			AX_MOTOR_A_SetSpeed(0);
			AX_MOTOR_B_SetSpeed(0);
		}
		
		//��ȫ����������С��ֱ�����Ƕ�С��5�ȣ���������ת��
		Balance_PutDown();
		
		//�쳣����ˤ�����������𱣻����Ƕȴ���45ֹͣ��
		Balance_FallProtect();
		
		//��������, 50HZ����Ƶ��
		if(cnt != 0)
		{
			//ROSģʽ�·���ROS����
			if(ax_control_mode == CTL_ROS)
			{
				ROS_SendDataHandle();
			}
			
			//��ROS������ģʽ�·���ƽ��������ݣ�����ģʽ������
			else
			{
				//�ж��Ƿ��ڿ���ģʽ��
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
		
		
	    //�������ݲ鿴
		//printf("@%d %d %d \r\n ", ax_balance_angle, ax_balance_gyro, ax_balance_out); //С���Ƕ�����
	    //printf("@%d %d %d \r\n ", ax_balance_angle, ax_balance_gyro, ax_turn_gyro); //С���Ƕ�����
		//printf("@%d %d %d  \r\n ", ax_balance_out, ax_velocity_out,ax_turn_out); //С�������������
	}
}


/**
  * @��  ��  ƽ��С��ֱ��PID����
  * @��  ��  target Ŀ��ƽ��Ƕȣ�С����е�м�Ƕȣ�Ĭ��Ϊ0
  *          angle  ��ǰƽ��Ƕȣ���λ��
  *          gyro   ��ǰƽ����ٶȣ���λ��/s
  * @����ֵ  PID�������ֵ���PWMֵ
  */
int16_t Balance_AngleCtl(int16_t target, float angle, float gyro)
{ 
	
	float angle_bias,gyro_bias;  //�ǶȺͽ��ٶ�ƫ��
	int16_t p_out,d_out,out;    //PID���ֵ
	
	//����ƽ��Ƕ�ƫ��
	angle_bias = target - angle;  
	
	//����ƽ����ٶ�ƫ��
	gyro_bias  = 0 - gyro;  
	
	//����ƽ��Ƕȿ���PWM�����pk��pdΪϵ����out = kp*e(k) + kd*[e(k)-e(k-1)]
	p_out = ax_balance_kp * angle_bias * 0.1f;
	d_out = ax_balance_kd * gyro_bias * 0.1f;
	
	//���������ֵ
	out =  p_out + d_out;
	
	//�������ݲ鿴
	//printf("@%d %d %d \r\n ",out, p_out, d_out); 
	
	return out;
}

/**
  * @��  ��  �ٶ�PID����
  * @��  ��  t_vx Ŀ���ٶȣ���λΪmm/s����Χ0~700
  *          r_vx ʵʱ�ٶȣ���λm/s
  * @����ֵ  PID�������ֵ���PWMֵ
  */
int16_t Balance_VelocityCtl(int16_t t_vx, float r_vx)
{ 
	
	static float velocity_bias = 0, velocity_bias_update;
	static float velocity_integral = 0;
	float  velocity_target;  //�ٶ�Ŀ��ֵ
	int16_t p_out,i_out,out; //PID���ֵ
	
	//Ŀ���ٶ��޷�
	if(t_vx > 700) t_vx = 700;
	else if(t_vx <-700) t_vx = -700;
	
	//�����ٶ�ƫ�Ŀ���ٶ�Ϊ0
	velocity_bias_update = 0 - r_vx;
	
	//һ�׵�ͨ�˲���, ��С�ٶȲ�����ƽ�����Ӱ��
	velocity_bias = (velocity_bias*0.6) + (velocity_bias_update*0.4); 
	
	//�ٶȻ��ּ����λ�ƣ���λm������ʱ��10ms
	velocity_integral += velocity_bias*0.01f;  
	
	//����Ŀ���ٶȣ���λΪm/s��,vx��λΪmm/s,����ʱ��0.01s
	velocity_target = t_vx*0.001f*0.01f;
	
	//����Ŀ���ٶȿ��ƣ�������ʻ�ٶ�
	velocity_integral += velocity_target; 
	
	//������ֹͣ�������������
	if(ax_robot_move_enable == 0)
	{
		velocity_integral = 0;
	}
	
	//���ƻ��ֱ���
	if(velocity_integral > 1.5)  velocity_integral = 1.5;
	else if(velocity_integral < -1.5)  velocity_integral = -1.5;

	//�����ٶȿ���PWM�����pk��piΪϵ��
	p_out = -ax_velocity_kp * velocity_bias;
	i_out = -ax_velocity_ki * velocity_integral;
	
	//���������ֵ
	out =  p_out + i_out;
	
    //�������ݲ鿴
	//printf("@%d %d %d \r\n ", out, p_out,i_out);   
	//printf("@%d %d \r\n ",vx, (int)(-velocity_bias_update*1000) );
	
	return out;

}


/**
  * @��  ��  ת��PID����
  * @��  ��  t_vw Ŀ����ת�ٶȣ���ʱ��Ϊ������λ��/s
                  ��Χ0~360��԰�ȫ�������ٶȻ��ƻ�ƽ��
  *          r_vw ʵʱ��ת�ٶȣ�������ת���ٶȣ���λ��/s
  * @����ֵ  PID�������ֵ���PWMֵ
  */
int16_t Balance_TurnCtl(int16_t t_vw, float r_vw)
{
	static float bias = 0;
	static float bias_last = 0;
	
	//PID���ֵ
	static int16_t p_out=0,d_out=0,out=0;  


	//������ֹͣ�������ת�����
	if(ax_robot_move_enable == 0)
	{
		out = 0;
	}	

	//����ת���ٶ�ƫ��
	bias = t_vw - r_vw;
	
	//����ƽ��ת�����PWM���
	p_out =  ax_turn_kp * bias * 0.001f;  
    d_out =  ax_turn_kd * (bias-bias_last) * 0.001f;
	
	//���������ֵ
	out +=  p_out + d_out;
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
    //�������ݲ鿴
	//printf("@%d %d %d \r\n ", (int)(bias*10), (int)(t_vw*10), (int)(r_vw*10)); 
	//printf("@%d %d %d \r\n ", out, p_out,d_out);   
	
	return out;
}

/**
  * @��  ��  ƽ��С�����¼��
  *          С��ֱ�����£��Ƕ�С��5�ȣ�,��������΢ת��
  * @��  ��  ��
  * @����ֵ  ��
  */
void Balance_PutDown(void)
{
	static uint16_t cont;
	int16_t wheel_l,wheel_r;
	
	//��ȡ������ת��
	wheel_l = (int16_t)(ax_wheel_vel_l*1000);
	wheel_r = (int16_t)(ax_wheel_vel_r*1000);
	
	//С��ƽ��ϵͳ�ر�״̬��
	if(ax_robot_move_enable == 0)
	{
		if(cont == 0)
		{
			//�ж�1���Ƕ�С��5�ȣ��������ٶ�Ϊ0
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
				
				//�ж�2������ǰ���ƶ��������ֲ����ٶ�
				if(wheel_l>1 || wheel_l<-1 || wheel_r>1 || wheel_r<-1) 
				{
					//�ٶ�����Ϊ��
					ax_robot_vx = 0;
					ax_robot_vw = 0;
					
					//�򿪵��ʹ��
					ax_robot_move_enable = 1;   
					
					//���������
					cont = 0;
				}
			}
			else  //��ʱ500ms�˳�
			{
				//���������
				cont = 0;
			}
		}		
	}
}

/**
  * @��  ��  С��ˤ���������Ƕȴ���45�ȣ���ΪС��ˤ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void Balance_FallProtect(void)
{
	//��Ǵ���45�Ƚ��뱣��״̬
	if((ax_balance_angle>4500)||(ax_balance_angle<-4500))
	{
		//����˶�ʹ��
		ax_robot_move_enable=0;
	}
}

/**
  * @��  ��  ROS������������IMU���ݷ�װ���ٶ���������ݷ���
  * @��  ��  ��
  * @����ֵ  ��
  */
void ROS_SendDataHandle(void)   
{
	int16_t temp[3];
	
	//���ڷ�������
	static uint8_t comdata[20]; 	

	//���ٶ� = (ax_acc/32768) * 2G 
	temp[0] =  ax_acc_data[1];   //ROS����X���ӦIMU��Y��
	temp[1] = -ax_acc_data[0];   //ROS����Y���ӦIMU��X�ᷴ��
	temp[2] =  ax_acc_data[2];   //ROS����Z���ӦIMU��Z��
	
	comdata[0] = (u8)( temp[0] >> 8 );  
	comdata[1] = (u8)( temp[0] );
	comdata[2] = (u8)( temp[1] >> 8 );
	comdata[3] = (u8)( temp[1] );
	comdata[4] = (u8)( temp[2] >> 8 );
	comdata[5] = (u8)( temp[2] );
	
	//�����ǽ��ٶ� = (ax_gyro/32768) * 2000
	temp[0] =  ax_gyro_data[1];   //ROS����X���ӦIMU��Y��
	temp[1] = -ax_gyro_data[0];   //ROS����Y���ӦIMU��X�ᷴ��
	temp[2] =  ax_gyro_data[2];   //ROS����Z���ӦIMU��Z��	

	comdata[6] = (u8)( temp[0] >> 8 );  
	comdata[7] = (u8)( temp[0] );
	comdata[8] = (u8)( temp[1] >> 8 );
	comdata[9] = (u8)( temp[1] );
	comdata[10] = (u8)( temp[2] >> 8 );
	comdata[11] = (u8)( temp[2] );
	
	//�˶�ѧ���������ɻ����������ٶȼ���������ٶ�
	//�������ٶ�ֵ ��λΪm/s���Ŵ�1000��
	temp[0] = ((ax_wheel_vel_l + ax_wheel_vel_r)/2)*1000;  //������ʵʱX�ٶ�
	temp[1] = 0;  //������ʵY�ٶ�
	temp[2] = ((-ax_wheel_vel_l + ax_wheel_vel_r)/WHEEL_BASE)*1000;		//������ʵW�ٶ�
	
	comdata[12] = (u8)( temp[0] >> 8 );  
	comdata[13] = (u8)( temp[0] );
	comdata[14] = (u8)( temp[1] >> 8 );
	comdata[15] = (u8)( temp[1] );
	comdata[16] = (u8)( temp[2] >> 8 );
	comdata[17] = (u8)( temp[2] );	

	//��ص�ѹ
	comdata[18] = (u8)( ax_battery_vel >> 8 );
	comdata[19] = (u8)( ax_battery_vel );	
	
	//USB���ڷ������ݣ�����Դ��ڹ��ã���Ҫ����ʱ��ע�͵��þ䣩
    // AX_UART1_SendPacket(comdata, 20, ID_UTX_ROS);	
		
}

/**
  * @��  ��  �������ݣ�����ƽ��Ƕȡ����ٶȡ������������ݷ���
  * @��  ��  ��
  * @����ֵ  ��
  */
void DBG_SendDataHandle(void)   
{
	int16_t temp[2];
	
	//���ڷ�������
	static uint8_t comdata[14]; 	

	//ƽ��������
	temp[0] =  ax_balance_angle;  //ƽ��Ƕ�
	temp[1] =  ax_balance_gyro;   //ƽ��Ƕȶȣ������ǲ���
	
	comdata[0] = (u8)( temp[0] >> 8 );  
	comdata[1] = (u8)( temp[0] );
	comdata[2] = (u8)( temp[1] >> 8 );
	comdata[3] = (u8)( temp[1] );

	//�ٶȿ�������
	temp[0] =  ax_robot_vx;   //Ŀ���ٶ����ݣ��趨ֵ
	temp[1] =  ax_velocity;   //ʵ���ٶ����ݣ�����������ֵ

	comdata[4] = (u8)( temp[0] >> 8 );  
	comdata[5] = (u8)( temp[0] );
	comdata[6] = (u8)( temp[1] >> 8 );
	comdata[7] = (u8)( temp[1] );
	
	//ת���������
	temp[0] = ax_robot_vw * 10;  //Ŀ��ת���ٶ����ݣ��趨ֵ
	temp[1] = 0.61f * ax_turn_gyro;  //ʵ��ת���ٶ����ݣ������ǲ���ֵ
	
	comdata[8] = (u8)( temp[0] >> 8 );  
	comdata[9] = (u8)( temp[0] );
	comdata[10] = (u8)( temp[1] >> 8 );
	comdata[11] = (u8)( temp[1] );
	
	//USB���ڷ������ݣ�����Դ��ڹ��ã���Ҫ����ʱ��ע�͵��þ䣩
    // AX_UART1_SendPacket(comdata, 12, ID_UTX_DBG);	
		
}

/******************* (C) ��Ȩ 2023 XTARK **************************************/

