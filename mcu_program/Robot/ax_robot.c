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

/* Includes ------------------------------------------------------------------*/
#include "ax_robot.h"

//������RGB����
ROBOT_Light  R_Light;

//IMU����
int16_t ax_acc_data[3];
int16_t ax_gyro_data[3];
int16_t ax_angle_data[3];

//�������˶�ʹ�ܿ���,Ĭ�ϴ�״̬
uint8_t ax_robot_move_enable = 1;

//�����˹���ʹ�ܿ���,Ĭ�Ϲر�״̬
uint8_t ax_function_enable = 0;

//�ƹ⿪��ʧ�ܣ�Ĭ�ϴ�״̬
uint8_t ax_light_enable = 1;

//�ƹ�Ч����Ч������
uint8_t ax_light_save_flag = 0;

//����������һ��
uint8_t ax_beep_ring = 0;

//PS2�ֱ���ֵ�ṹ��
JOYSTICK_TypeDef ax_joystick;  

//���Ʒ�ʽѡ��
uint8_t ax_control_mode = CTL_ROS;

//�����˵�ص�ѹ����
uint16_t ax_battery_vel; 

//�����̨�ؽڽǶ�
int16_t ax_joint_angle[2] = {0};  

//�������ٶ�
int16_t ax_robot_vx = 0;  //ǰ��Ϊ������λmm/s 
int16_t ax_robot_vw = 0;  //��ʱ��Ϊ������λ��/s

//������ƽ����ֵ
int16_t  ax_middle_angle = 0;   //С����е�м�Ƕȣ�Ĭ��Ϊ0

//������������ʵʱ�ٶ�
double ax_wheel_vel_l = 0;
double ax_wheel_vel_r = 0;
 
//������ƽ�����
int16_t  ax_balance_angle; //ƽ����� 
int16_t  ax_balance_gyro;  //ƽ�������ǽ��ٶ�
int16_t  ax_balance_out;   //ƽ��������
int16_t  ax_balance_kp = 2700;	//ƽ��kp   3600
int16_t  ax_balance_kd = 135;	//ƽ��Kd   180

//�������ٶȿ���
int16_t  ax_velocity = 0;  //�ٶ�ֵ
int16_t  ax_velocity_out;	//�ٶȿ������
int16_t  ax_velocity_kp = 8000;	  //�ٶ�kp
int16_t  ax_velocity_ki = 4000;	    //�ٶ�ki

//������ת�����
int16_t  ax_turn_gyro;    //ת�������ǽ��ٶ�
int16_t  ax_turn_angle;    //�����
int16_t  ax_turn_out;	  //ת��������
int16_t  ax_turn_kp = 1000;	//ת��kpϵ��
int16_t  ax_turn_kd = 5000;	//ת��kdϵ��

//lqr״̬
StateVariable stX = {0};

/******************* (C) ��Ȩ 2023 XTARK **************************************/

