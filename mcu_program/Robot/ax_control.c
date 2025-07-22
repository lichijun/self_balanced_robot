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
  * @��  ��  �����˿��ƴ����ļ�
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ax_control.h"
#include "ax_robot.h"

/**
  * @��  ��  ����PS2�ֱ���������
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_CTL_Ps2(void)
{
	//�������±��
	static uint8_t btn_select_flag = 0;
	static uint8_t btn_joyl_flag = 0;
	static uint8_t btn_joyr_flag = 0;
	static uint8_t btn_l1_flag = 0;
	static uint8_t btn_l2_flag = 0;
	
	static uint8_t  robot_amplitude = 5;   //�����ٶȣ�1~6
	static uint16_t joint_amplitude = 15;  //��̨�ؽ��ٶ�
	
	//���̵�ģʽ�£�ִ�п��Ʋ���
	if(ax_joystick.mode ==  0x73)
	{
		//�������ٶȿ���
		ax_robot_vx = (int16_t)(      robot_amplitude * (0x80 - ax_joystick.RJoy_UD));
		ax_robot_vw = (int16_t)(0.3 * robot_amplitude * (0x80 - ax_joystick.LJoy_LR));
		
		//SELECT�������л�RGB��Чģʽ
		if(ax_joystick.btn1 & PS2_BT1_SELECT)
		{
			btn_select_flag = 1;
		}
		else
		{
			if(btn_select_flag)
			{
				//�ƹ�Ч���л�
				if(R_Light.M < LEFFECT6)
					R_Light.M++;
				else
					R_Light.M = LEFFECT1;
				
				//��λ���
				btn_select_flag = 0;
			}
		}
		
		//��ҡ�˰���������
		if(ax_joystick.btn1 & PS2_BT1_JOY_L)
		{
			btn_joyl_flag = 1;
		}
		else
		{
			if(btn_joyl_flag)
			{
				
				//�ٶȼ�С
				if(robot_amplitude > 1)
				{
					robot_amplitude--;
				}
				else
				{
					robot_amplitude = 1;
					
					//������������ʾ
					ax_beep_ring = BEEP_SHORT;
				}
					
				//��λ���
				btn_joyl_flag = 0;
			}
		}
		
		//��ҡ�˰���������
		if(ax_joystick.btn1 & PS2_BT1_JOY_R)
		{
			btn_joyr_flag = 1;
		}
		else
		{
			if(btn_joyr_flag)
			{
				//�ٶ�����
				if(robot_amplitude < 6)
				{
					robot_amplitude++;
				}
				else
				{
					robot_amplitude = 6;
					
					//������������ʾ
					ax_beep_ring = BEEP_SHORT;					
				}
					
				//��λ���
				btn_joyr_flag = 0;
			}
		}
		
		
		
		//���Ұ������ƹؽ�1
		if(ax_joystick.btn1 & PS2_BT1_LEFT)
		{
			ax_joint_angle[0] += joint_amplitude; 
		}
		else if(ax_joystick.btn1 & PS2_BT1_RIGHT)
		{
			ax_joint_angle[0] -= joint_amplitude; 
		}
			
		//���°������ƹؽ�2
		if(ax_joystick.btn1 & PS2_BT1_DOWN)
		{
			ax_joint_angle[1] += joint_amplitude; 
		}
		else if(ax_joystick.btn1 & PS2_BT1_UP)
		{
			ax_joint_angle[1] -= joint_amplitude; 
		}
		
		//L1��������е�ۼ���
		if(ax_joystick.btn2 & PS2_BT2_L1)
		{
			btn_l1_flag = 1;
		}
		else
		{
			if(btn_l1_flag)
			{
				//ִ�ж�����
				if(joint_amplitude < 25)
				{
					joint_amplitude =  joint_amplitude + 5;
				}
				else
				{
					joint_amplitude = 25;
					
					//������������ʾ
					ax_beep_ring = BEEP_SHORT;					
				}
				
				//��λ���
				btn_l1_flag = 0;
			}
		}
		
		//L2��������е�ۼ���
		if(ax_joystick.btn2 & PS2_BT2_L2)
		{
			btn_l2_flag = 1;
		}
		else
		{
			if(btn_l2_flag)
			{
				//ִ�ж���
				if(joint_amplitude > 5)
				{
					joint_amplitude =  joint_amplitude -5;
				}
				else
				{
					joint_amplitude = 5;
					
					//������������ʾ
					ax_beep_ring = BEEP_SHORT;
				}
				
				//��λ���
				btn_l2_flag = 0;
			}
		}	
	}
}	

/**
  * @��  ��  �����ֻ�APP��������
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_CTL_App(void)
{
	static uint8_t comdata[16];
	
	//��������APP��������
	if(AX_UART2_GetData(comdata))
	{
		//ҡ��ģʽ�˶�����֡
		if((comdata[0] == ID_BLERX_YG))
		{
			ax_robot_vx = (int16_t)(  5*(int8_t)comdata[4] );
			ax_robot_vw = (int16_t)( -2*(int8_t)comdata[1] );
		}
		
		//�ֱ�ģʽ�˶�����֡
		if((comdata[0] == ID_BLERX_SB))
		{
			ax_robot_vx = (int16_t)(  5*(int8_t)comdata[4] );
			ax_robot_vw = (int16_t)( -2*(int8_t)comdata[1] );
		}
		
		//����ģʽ�˶�����֡
		if(comdata[0] == ID_BLERX_ZL)
		{
			ax_robot_vx = (int16_t)( -5*(int8_t)comdata[3] );
			ax_robot_vw = (int16_t)( -2*(int8_t)comdata[2] );
		}
		
		//�ƹ�Ч������֡
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
  * @��  ��  ����SBUS��ģң������������
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_CTL_Rms(void)
{
	static uint16_t sbusdata[8];
	static float amplitude;
	
	//��ȡSBUS��������
	if(AX_SBUS_GetRxData(sbusdata))
	{
		//SWD-8��ͨ��7�����˵��Ϸ���Ч
		if(sbusdata[7] < 500)
		{
			//SWA-5���������ٶ�
			if(sbusdata[4] < 500)          amplitude = 1.0;
			else if(sbusdata[4] < 1000)    amplitude = 0.7;
			else                           amplitude = 0.5;
			
			//�˶�����
			ax_robot_vx = (int16_t)(       amplitude * (sbusdata[1] - 992));
			ax_robot_vw = (int16_t)(-0.4 * amplitude * (sbusdata[3] - 992));
		}
		
		//�鿴����ͨ����ֵ
		//printf("c1=%04d c2=%04d c3=%04d ch4=%04d ch5=%04d ch6=%04d ch7=%04d ch8=%04d \r\n",sbusdata[0],
	    //       sbusdata[1],sbusdata[2],sbusdata[3],sbusdata[4],sbusdata[5],sbusdata[6],sbusdata[7]);
	}		
		
	
}

/******************* (C) ��Ȩ 2023 XTARK **************************************/

