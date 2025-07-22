/**			                                                    
		   ____                    _____ _____  _____        XTARK@���˴���
		  / __ \                  / ____|  __ \|  __ \  
		 | |  | |_ __   ___ _ __ | |    | |__) | |__) |
		 | |  | | '_ \ / _ \ '_ \| |    |  _  /|  ___/ 
		 | |__| | |_) |  __/ | | | |____| | \ \| |     
		  \____/| .__/ \___|_| |_|\_____|_|  \_\_|     
		    		| |                                    
		    		|_|  OpenCRP ��ݮ�� ר��ROS�����˿�����                                   
									 
  ****************************************************************************** 
  *           
  * ��Ȩ���У� XTARK@���˴���  ��Ȩ���У�����ؾ�
  * ������վ�� www.xtark.cn
  * �Ա����̣� https://shop246676508.taobao.com  
  * ����ý�壺 www.cnblogs.com/xtark�����ͣ�
	* ����΢�ţ� ΢�Ź��ںţ����˴��£���ȡ������Ѷ��
  *      
  ******************************************************************************
  * @��  ��  Musk Han@XTARK
  * @��  ��  V1.0
  * @��  ��  2019-7-26
  * @��  ��  VIN�����ѹ���
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ROBOT_H
#define __ROBOT_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//C�⺯�������ͷ�ļ�
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h> 
#include <string.h>
#include <math.h>

//FreeRTOSͷ�ļ�
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

//�������ͷ�ļ�
#include "ax_sys.h"    //ϵͳ����
#include "ax_delay.h"  //�����ʱ
#include "ax_led.h"    //LED�ƿ���
#include "ax_beep.h"   //����������
#include "ax_vin.h"    //�����ѹ���
#include "ax_key.h"    //�������

#include "ax_uart1.h"    //���Դ���
#include "ax_uart2.h"    //��������
#include "ax_uart4.h"    //TTL����
#include "ax_uart5.h"    //Ԥ������

#include "ax_motor.h"    //ֱ��������ٿ���
#include "ax_encoder.h"  //����������
#include "ax_servo.h"    //�������

#include "ax_mpu6050.h"  //IMU���ٶ�������

#include "ax_rgb.h"      //RGB�ʵƿ���
#include "ax_sbus.h"     //SBUS��ģң��������
#include "ax_oled.h"     //OLED��ʾ


//����ٶȽṹ��
typedef struct  
{
	float  Wheel_RT;       //����ʵʱ�ٶȣ���λm/s
	float  Wheel_TG;       //����Ŀ���ٶȣ���λm/s
	short  Motor_Pwm;      //���PWM��ֵ
}MOTOR_Data;

//�������ٶȽṹ��
typedef struct  
{
	short  I_X;     //X���ٶȣ�16λ������
	short  I_Y;     //Y���ٶȣ�16λ������
	short  I_W;     //Yaw��ת���ٶȣ�16λ������
	
	float  F_X;     //X���ٶȣ�����
	float  F_Y;     //Y���ٶȣ�����
	float  F_W;     //Yaw��ת���ٶȣ�����
	
}ROBOT_Velocity;

//������ת����ƽṹ��
typedef struct  
{
	float  Radius;     //ת��뾶
	float  Angle;     //������ת��Ƕ�
	float  RAngle;     //ǰ����ת��Ƕ�
	float  SAngle;     //����Ƕ�
	
}ROBOT_Steering;

//������IMU����
typedef struct  
{
	short  ACC_X;     //X��
	short  ACC_Y;     //Y��
	short  ACC_Z;     //Z��
	
	short  GYRO_X;     //X��
	short  GYRO_Y;     //Y��
	short  GYRO_Z;     //Z��
	
}ROBOT_IMU;


#define  PI   3.1416     //Բ����PI

//���
#define  VBAT_40P    1065     //���40%��ѹ
#define  VBAT_20P    1012     //���20%��ѹ
#define  VBAT_10P    984      //���10%��ѹ

#define  PID_RATE        50        //PIDƵ��

//�����˲���
#define  R_WHEEL_BASE   0.165	  //�־࣬�����ֵľ���
#define  R_ACLE_BASE    0.160     //��࣬ǰ���ֵľ���


#define  R_TURN_R_MINI  0.35      //��Сת��뾶( L*cot30-W/2)

//���Ӳ��� 
#define  WHEEL_DIAMETER	     0.0725		//����ֱ��
#define  WHEEL_RESOLUTION    1440.0    //�������ֱ���

//�����ٶ�m/s�������ת��ϵ��
#define  WHEEL_SCALE  (PI*WHEEL_DIAMETER*PID_RATE/WHEEL_RESOLUTION) 

//�������ٶ�����
#define R_VX_LIMIT  1500   //X���ٶ���ֵ m/s*1000
#define R_VY_LIMIT  1200   //Y���ٶ���ֵ m/s*1000
#define R_VW_LIMIT  6280   //W��ת���ٶ���ֵ rad/s*1000

//������ͨ��֡ͷ����
//������ͨ��֡ͷ����
#define  ID_CPR2ROS_DATA    0x10    //��λ����ROS���͵��ۺ�����
#define  ID_ROS2CRP_VEL     0x50    //ROS����λ�����͵��ٶ�����
#define  ID_ROS2CRP_IMU     0x51    //ROS����λ�����͵�IMU��������ƫУ׼ָ��
#define  ID_ROS2CRP_AKM     0x5f    //ROS����λ�����͵�AKM�����˶����ƫ�����ǣ��������ͺ���

//ȫ�ֱ���
extern int16_t ax_imu_acc_data[3];  
extern int16_t ax_imu_gyro_data[3];
extern int16_t ax_imu_gyro_offset[3];   

extern int16_t ax_motor_kp;  
extern int16_t ax_motor_ki;    
extern int16_t ax_motor_kd; 

extern int16_t ax_servo_offset;

extern int8_t ax_imu_calibrate_flag;

//������ݽṹ��
extern MOTOR_Data MOTOR_A,MOTOR_B;

//�������ٶ����ݽṹ��
extern ROBOT_Velocity  RobotV_RT,RobotV_TG;


//������
extern TaskHandle_t Robot_Task_Handle;
extern TaskHandle_t Key_Task_Handle;
extern TaskHandle_t Bat_Task_Handle;


#endif

/******************* (C) ��Ȩ 2019 XTARK **************************************/
