/**			                                                    
		   ____                    _____ _______ _____       XTARK@���˴���
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   �����˿�����
									 
  ****************************************************************************** 
  *           
  * ��Ȩ���У� XTARK@���˴���  ��Ȩ���У�����ؾ�
  * ��˾��վ�� www.xtark.cn   www.tarkbot.com
  * �Ա����̣� https://xtark.taobao.com  
  * ����΢�ţ� ���˴��£���ע���ںţ���ȡ���¸�����Ѷ��
  *      
  ******************************************************************************
  * @��  ��  Musk Han@XTARK
  * @��  ��  �����˹��ܴ����ļ�
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ax_function.h"
#include "ax_robot.h"

/********�״�������**************************/

#define LS_AVOID_Distance        400     //������Ч����
#define LS_AVOID_Distance_min    250     //���˾���
#define LS_AVOID_Speed           300     //��ʻ�ٶ�
#define LS_AVOID_Turn            100     //ת���ٶ�

/**
  * @��  ��  �����״������ʻ
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_FUN_List1(void)
{
	uint8_t i;
	
	uint16_t min_distance = 65000;   //�ϰ�����С����	
	uint16_t min_angle = 0;          //�ϰ�����С�Ƕ�	
	uint16_t min_distance_cnt = 0;   //��С�������	
	
	//ɨ��ǰ���ϰ���ǶȺ;���
	for(i = 0; i < 250; i++)
	{
		//������Ч�Ƕ�300-60֮��
		if((ax_ls_point[i].angle>30000) || (ax_ls_point[i].angle<6000))  
		{	
			//���С�ڱ�����Ч���������
			if((ax_ls_point[i].distance >0)  && (ax_ls_point[i].distance < LS_AVOID_Distance))
			{
				//Ѱ������ϰ���
				if(ax_ls_point[i].distance < min_distance && ax_ls_point[i].distance>100)
				{
					//��¼����ͽǶ�
					min_distance = ax_ls_point[i].distance;
					min_angle = ax_ls_point[i].angle;
				}
				
				//��¼����С�ں��˾���ĵ���
				if(ax_ls_point[i].distance < LS_AVOID_Distance_min)
					min_distance_cnt++;
			}
		}
	}
	
	//��ӡ������Ϣ
	//printf("@%d %d \r\n",min_angle, min_distance);
	
	//С��ǰ��
	ax_robot_vx = LS_AVOID_Speed;
	
	//������Ͽ���
	if(min_distance < LS_AVOID_Distance)
	{
		//����̫��С������
		if(min_distance_cnt > 3)
		{
			ax_robot_vx = -LS_AVOID_Speed;
			ax_robot_vw = 0;
		}
		else 
		{
			//�ϰ������ұ�
			if(min_angle < 6000)
			{
				ax_robot_vw = LS_AVOID_Turn;
			}
			
			//�ϰ��������
			if(min_angle > 30000)
			{
				ax_robot_vw = -LS_AVOID_Turn;
			}
		}
	}
	else
	{
		//ǰ��û���ϰ��ֱ��
		ax_robot_vw = 0;
	}

	//��ʱ
	vTaskDelay(100); 
}




/********�״ﾯ�����**************************/

#define LS_ALARM_DistanceMin 150   //������С����
#define LS_ALARM_DistanceMax 500   //����������

/**
  * @��  ��  �״ﾯ������
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_FUN_List2(void)
{
	uint8_t i;
	
	uint16_t min_distance = 65000;   //���������룬��λmm	
	uint16_t min_angle = 0;          //�������Ƕ�	
    int16_t  follow_angle = 0;		 //����Ƕ�	
	
	//�ҳ��������
	for(i = 0; i < 250; i++)
	{
		//������Ч�Ƕ�300-60֮��
		if((ax_ls_point[i].angle>30000) || (ax_ls_point[i].angle<6000))  
		{	
			//�����Ч�����ڵ�����
			if((ax_ls_point[i].distance >LS_ALARM_DistanceMin) && (ax_ls_point[i].distance < LS_ALARM_DistanceMax))
			{	
				//Ѱ���������
				if(ax_ls_point[i].distance < min_distance)
				{
					min_distance = ax_ls_point[i].distance;
					min_angle = ax_ls_point[i].angle;
				}
			}
		}
	}
	
	//ת������180�㣬��λ��ÿ��
	if(min_angle > 18000)
		follow_angle = (min_angle - 36000)/100.0;	
	else
		follow_angle = min_angle/100.0;
	
	ax_robot_vx = 0;
	
	//�ж��Ƿ��п�������
	if(min_angle != 0)
	{
		//����������
		AX_BEEP_On();
		
		//��������
		ax_robot_vw = -3*follow_angle;
	}
	else
	{
		//�����������ر�
		AX_BEEP_Off();
		
		ax_robot_vw = 0;
	}
	
	//��ӡ������Ϣ
	//printf("@%d %d \r\n",follow_angle, ax_robot_vw);

	//��ʱ
	vTaskDelay(100); 
}



/********�״�������**************************/

#define LS_FOLLOW_DistanceMin  200   //��Ч��С����
#define LS_FOLLOW_DistanceMax  1500   //��Ч������
#define LS_FOLLOW_DistanceKeep 350   //�״������屣�־���

float ls_fangle_kp = 3.0f;   //����Ƕ�PID KP����
float ls_fangle_kd = 1.0f;   //����Ƕ�PID KD����
float ls_fdistance_kp = 1.0f;  //�������PID KP����
float ls_fdistance_kd = 0.5f;   //�������PID KD����

int16_t FOLLOW_Angle_PIDControl(float target, float current);  //����Ƕ�PID���ƺ���
int16_t FOLLOW_Distance_PIDControl(float target, float current);  //�������PID���ƺ���

/**
  * @��  ��  �����״����
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_FUN_List3(void)
{
	uint8_t i;
	
	uint16_t min_distance = 65000;   //�������	
	uint16_t min_angle = 0;          //����Ƕ�		
	
	static float follow_angle = 0;			//����Ƕ�
	static float follow_distance = 0;		//�������	
	
	//�ҳ���������Ƕ�
	for(i = 0; i < 250; i++)
	{
		//������Ч�Ƕ�270-90֮��
		if((ax_ls_point[i].angle>30000) || (ax_ls_point[i].angle<6000))  
		{	
			//�����Ч�����ڵ�����
			if((ax_ls_point[i].distance >200) && (ax_ls_point[i].distance < 1000))
			{	
				//Ѱ���������
				if(ax_ls_point[i].distance < min_distance)
				{
					min_distance = ax_ls_point[i].distance;
					min_angle = ax_ls_point[i].angle;
				}
			}
		}
	}
	
	//ת������180�㣬��λ��ÿ��
	if(min_angle > 18000)
		follow_angle = (min_angle - 36000)/100.0;	
	else
		follow_angle = min_angle/100.0;
		
	//��������Ϊ�������ͣ���λmm
	follow_distance = min_distance;	
	 
	//��ӡ������Ϣ
	//printf("@%d %d \r\n",follow_angle*100, follow_distance*100);
	
	//�ж��Ƿ�����Ч��������
	if(min_angle != 0)
	{
		//PID�������
		ax_robot_vx = FOLLOW_Distance_PIDControl(LS_FOLLOW_DistanceKeep, follow_distance);  //�������PID����
		ax_robot_vw = FOLLOW_Angle_PIDControl(0, follow_angle);  //����Ƕ�PID����
	}
	else
	{
		ax_robot_vx = 0;
		ax_robot_vw = 0;
	}

	//��ʱ
	vTaskDelay(100); 
}

/**
  * @��  ��  ת��Ƕȿ���PID����
  * @��  ��  target  Ŀ��Ƕȣ���λ��
  *          current ��ǰ�Ƕȣ���λ��
  * @����ֵ  ��
  */
int16_t FOLLOW_Angle_PIDControl(float target, float current)
{
	static float bias, output, last_bias;
	
	//����ƫ��	
	bias = target - current;
	
	//����PID���ת�������
	output = ls_fangle_kp*bias + ls_fangle_kd*(bias-last_bias);
	
	//��¼�ϴ�ƫ��
	last_bias = bias;
	
	//��ӡ������Ϣ
	//printf("%f %f \r\n",bias, output);
	
	return  (output);
}

/**
  * @��  ��  ����������PID����
  * @��  ��  target  Ŀ����룬��λmm
  *          current ��ǰ���룬��λmm
  * @����ֵ  ��
  */
int16_t FOLLOW_Distance_PIDControl(float target, float current)
{
	static float bias, output, last_bias;
	
	//����ƫ��	
	bias = target - current;
	
	//����PID���ת�������
	output = -ls_fdistance_kp*bias - ls_fdistance_kd*(bias-last_bias);
	
	//��¼�ϴ�ƫ��
	last_bias = bias;
	
	//��ӡ������Ϣ
	//printf("@%f %f \r\n",bias, output);
	
	return  (output);
	
}


/********�״���ǽֱ����ʻ���**************************/

#define LS_LINE_Speed          200     //��ʻ�ٶ�
float ls_line_kp = 0.2f;  //ֱ����ʻPID KP����
float ls_line_kd = 0.1f;   //ֱ����ʻPID KD����

int16_t LINE_PIDControl(float target, float current);  //�״�ֱ����ʻPID���ƺ���

/**
  * @��  ��  �����״�ֱ����ʻ
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_FUN_List4(void)
{
	uint8_t i;
	static uint8_t cnt=0;
	
	//����ֱ�����嵱ǰ����
	static uint16_t current_distance = 200; 
	static uint16_t last_distance = 200;
	
	//Ŀ��������
	static uint16_t target_distance = 200; 
	
	//�ҳ���������Ƕ�
	for(i = 0; i < 250; i++)
	{
		//ȡ��С����ǰ��75�����ҵ��״�̽������
		if((ax_ls_point[i].angle>7200) && (ax_ls_point[i].angle<7800))  
		{	
			//�����Ч�����ڵ�����
			if(ax_ls_point[i].distance < (target_distance + 300))
			{	
				//��Ч����
				current_distance = ax_ls_point[i].distance;			
				last_distance = current_distance;
			}
			else
			{   //��Ч�������ݣ�������һ�β������
				current_distance = last_distance;
			}
		}
	}
	
	if(cnt > 30)
	{
		//С��ǰ��
		ax_robot_vx = LS_LINE_Speed;
		
		//����С��PID��ֱ��
		ax_robot_vw = LINE_PIDControl(target_distance, current_distance);  
		 
		//��ӡ������Ϣ
		//printf("@%d %d \r\n",current_distance,ax_robot_vw*10);		
	}
	else
	{
		//��ǰ��������Ϊ�趨ǽ�ھ���
		target_distance = current_distance;
		
		cnt++;
	}
	
	//��ʱ
	vTaskDelay(100); 
}

/**
  * @��  ��  �״�ֱ����ʻPID����
  * @��  ��  target  Ŀ����룬��λmm
  *          current ��ǰ���룬��λmm
  * @����ֵ  ��
  */
int16_t LINE_PIDControl(float target, float current)
{
	static float bias, output, last_bias;
	
	//����ƫ��	
	bias = target - current;
	
	//����PID���ת�������
	output = ls_line_kp*bias + ls_line_kd*(bias-last_bias);
	
	//��¼�ϴ�ƫ��
	last_bias = bias;
	
	//��ӡ������Ϣ
	//printf("@%f %f \r\n",bias, output);
	
	return  (output);
	
}


/********CCDѲ����ʻ���**************************/
 
int16_t ax_ccd_speed = 300;  //CCDѲ���ٶȣ���λmm/s
int16_t ax_ccd_offset;   //CCDѲ�ߺ���λ��ƫ��
int16_t ax_ccd_kp = 50;  //CCDѲ��PID����kp
int16_t ax_ccd_kd = 20;  //CCDѲ��PID����kd


/**
  * @��  ��  ����-CCDѲ�߹���
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_FUN_List5(void)
{
	static float bias,bias_last;
	float move_w=0;   

	//����CCDѲ���ٶ�
	ax_robot_vx = ax_ccd_speed;
	
	//��ȡCCD�ɼ�������ĺ���ƫ��ֵ
	ax_ccd_offset = AX_CCD_GetOffset();
	bias = ax_ccd_offset;
	
	//����PID���ת�������
	move_w = -ax_ccd_kp*bias*0.01f - ax_ccd_kd*(bias-bias_last)*0.01f;
	
	//����ת���ٶȣ���ǰ���ٶ����
	ax_robot_vw  =  0.01f* move_w*ax_ccd_speed;	
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//��ӡ������Ϣ
	//printf("@%d %d \r\n",ax_ccd_offset, ax_robot_vw);
	
	//��ʱ
	vTaskDelay(20); 
}

/******************* (C) ��Ȩ 2023 XTARK **************************************/

