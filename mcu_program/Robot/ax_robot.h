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
  * @��  ��  ������ȫ�ֱ���
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_ROBOT_H
#define __AX_ROBOT_H

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
#include "ax_sys.h"      //ϵͳ����
#include "ax_delay.h"    //�����ʱ
#include "ax_led.h"      //LED�ƿ���
#include "ax_beep.h"     //����������
#include "ax_vin.h"      //�����ѹ���
#include "ax_key.h"      //������� 
#include "ax_flash.h"    //FLASH��д����
#include "ax_servo.h"    //�������
#include "ax_motor.h"    //ֱ��������ٿ���
#include "ax_encoder.h"  //����������

#include "ax_uart1.h"    //���Դ���
#include "ax_uart2.h"    //��������
#include "ax_uart5.h"    //Ԥ������

#include "ax_mpu6050.h"  //MPU6050��������ͷ�ļ�
#include "ax_mpu6050_dmp.h" //MPU6050 DMP���ܺ���

#include "ax_rgb.h"      //RGB�ʵƿ���
#include "ax_sbus.h"     //SBUS��ģң��������
#include "ax_oled.h"     //OLED��ʾ
#include "ax_ps2.h"      //PS2�ֱ�

#include "ax_laser.h"    //�״ﴫ����
#include "ax_ccd.h"      //CCD������

#include "ax_function.h"  //���ܺ���ͷ�ļ�

#include "ax_balance.h"

//RGB��Ч����
typedef struct  
{
	uint8_t  M;    //��Ч��ģʽ
	uint8_t  S;    //��Ч��ģʽ
	uint8_t  T;    //��Чʱ�����
	
	uint8_t  R;     //��Ч��ɫ R
	uint8_t  G;     //��Ч��ɫ G
	uint8_t  B;     //��Ч��ɫ B
	
}ROBOT_Light;

//����
#define  PI           3.1416     //Բ����PI
#define  SQRT3        1.732      //3��ƽ��
#define  PID_RATE     100         //PIDƵ��

//���
#define  VBAT_40P    1065     //���40%��ѹ
#define  VBAT_20P    1012     //���20%��ѹ
#define  VBAT_10P    984      //���10%��ѹ

//����������汾
#define  ROBOT_FW_VER   "V1.01"

//ƽ��������ͺŶ���
#define ROBOT_B680

/******�����˲���*************************************/
#ifdef ROBOT_B680
#define  WHEEL_DIAMETER	      0.075	  //����ֱ��
#define  WHEEL_BASE           0.170	  //�־࣬�����ֵľ���
#define  WHEEL_RESOLUTION     1560.0  //�������ֱ���(13��),���ٱ�30,13x30x4=1560
#define  WHEEL_SCALE          (PI*WHEEL_DIAMETER*PID_RATE/WHEEL_RESOLUTION)  //�����ٶ�m/s�������ת��ϵ��
#endif

#ifdef ROBOT_B680_MAX
#define  WHEEL_DIAMETER	      0.080	  //����ֱ��
#define  WHEEL_BASE           0.178	  //�־࣬�����ֵľ���
#define  WHEEL_RESOLUTION     1560.0  //�������ֱ���(13��),���ٱ�30,13x30x4=1560
#define  WHEEL_SCALE          (PI*WHEEL_DIAMETER*PID_RATE/WHEEL_RESOLUTION)  //�����ٶ�m/s�������ת��ϵ��
#endif

/******������ͨ��Э��*************************************/
//USB����ͨ��֡ͷ����
#define  ID_UTX_ROS     0x10    //���͵�ROS�ۺ�����
#define  ID_UTX_DBG     0x11    //���͵�ƽ�⳵��������

#define  ID_URX_VEL     0x50    //���յ�ROS�ٶ�����
#define  ID_URX_BLC     0x02    //ֱ��PID���Ʋ���
#define  ID_URX_BLV     0x03    //�ٶ�PID���Ʋ���
#define  ID_URX_BLT     0x04    //ת��PID���Ʋ���



//����APPͨ��֡ͷ����
#define  ID_BLERX_CM    0x30    //APP�������� ��������ָ��
#define  ID_BLERX_YG    0x31    //APP�������� ҡ��ģʽ ����ָ��
#define  ID_BLERX_SB    0x32    //APP�������� �ֱ�ģʽ ����ָ��
#define  ID_BLERX_ZL    0x33    //APP�������� ����ģʽ ����ָ��
#define  ID_BLERX_TK    0x34    //APP�������� ̹��ģʽ ����ָ��
#define  ID_BLERX_AM    0x3A    //APP�������� ��е��ģʽ ����ָ��
#define  ID_BLERX_LG    0x41    //APP�������� �ƹ����ָ��
#define  ID_BLERX_LS    0x42    //APP�������� ����ƹ�Ч��ָ��

//�ƹ�ģʽ
#define  LEFFECT1    0x01    //��ɫģʽ  
#define  LEFFECT2    0x02    //����ģʽ  
#define  LEFFECT3    0x03    //��ɫģʽ  
#define  LEFFECT4    0x04    //����ģʽ 
#define  LEFFECT5    0x05    //����ģʽ  
#define  LEFFECT6    0x06    //����ģʽ

//�����˿��Ʒ�ʽ
#define  CTL_PS2    0x01    //PS2�ֱ�����
#define  CTL_APP    0x02    //APP����
#define  CTL_RMS    0x03    //��ģң��������
#define  CTL_ROS    0x04    //ROS����

//�����嵥
#define  CTL_FN1    0x11    //����-�״������ʻ
#define  CTL_FN2    0x12    //����-�״ﾯ��
#define  CTL_FN3    0x13    //����-�״����
#define  CTL_FN4    0x14    //����-�״���ֱ��
#define  CTL_FN5    0x15    //����-CCDѲ��
 
#define  CTL_FN_MAX    CTL_FN5    //ģʽ���ֵ

//������������
#define  BEEP_SHORT   0x01    //������������һ��(200ms)
#define  BEEP_LONG    0x02    //������������һ��(1000ms)

//��̨�ؽڽǶ����ƣ���λ�ǶȷŴ�10��
#define JOINTA_UP_LIMIT    900
#define JOINTA_LOW_LIMIT  -900
#define JOINTB_UP_LIMIT    900
#define JOINTB_LOW_LIMIT  -900

//�����̨��ƫ������ 
#define JOINTA_ANGLE_OFFSET   (0)     //S1�����ˮƽ�ؽ�
#define JOINTB_ANGLE_OFFSET   (0)     //S2�������ֱ�ؽ�

//ȫ�ֱ���
extern int16_t ax_robot_vx;
extern int16_t ax_robot_vw;
extern uint8_t ax_robot_move_enable;
extern uint8_t ax_function_enable;
extern uint8_t ax_control_mode;

extern int16_t ax_acc_data[3];
extern int16_t ax_gyro_data[3];
extern int16_t ax_angle_data[3];  

extern  ROBOT_Light  R_Light; 
extern uint8_t ax_light_enable;
extern uint8_t ax_beep_ring;  
extern  uint16_t ax_battery_vel; 

extern double ax_wheel_vel_l;
extern double ax_wheel_vel_r;

extern int16_t  ax_middle_angle;  

extern int16_t  ax_balance_angle; 
extern int16_t  ax_balance_gyro;  
extern int16_t  ax_balance_out;   
extern int16_t  ax_balance_kp;
extern int16_t  ax_balance_kd;	

extern int16_t  ax_velocity;  
extern int16_t  ax_velocity_out;	
extern int16_t  ax_velocity_kp;	
extern int16_t  ax_velocity_ki;	   

extern int16_t  ax_turn_gyro;  
extern int16_t  ax_turn_angle; 
extern int16_t  ax_turn_out;	 
extern int16_t  ax_turn_kp;	
extern int16_t  ax_turn_kd;	

extern JOYSTICK_TypeDef ax_joystick;  

extern int16_t ax_joint_angle[2];

//������
extern TaskHandle_t Control_Task_Handle;
extern TaskHandle_t Key_Task_Handle;
extern TaskHandle_t Disp_Task_Handle;
extern TaskHandle_t Trivia_Task_Handle;

extern StateVariable stX;

#endif

/******************* (C) ��Ȩ 2023 XTARK **************************************/
