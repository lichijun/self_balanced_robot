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
  * @��  ��  �������������ļ�
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ax_task.h"
#include "ax_robot.h"

#include "ax_light.h"
#include "ax_control.h"
#include "ax_function.h"

//��������
void JOINT_Control(void);       //�����̨����

/**
  * @��  ��  �����˿��ƺ͹��ܹ�������
  * @��  ��  ��
  * @����ֵ  ��
  */
void Control_Task(void* parameter)
{	
	while(1)
	{		
		//����ģʽ������Ƶ��Լ50HZ
		if(ax_control_mode < CTL_FN1)
		{
			//���Ʒ�ʽѡ��ROSģʽ��ִ���κβ���
			if(ax_control_mode == CTL_PS2)          AX_CTL_Ps2();    //PS2�ֱ�			
			else if (ax_control_mode == CTL_APP)    AX_CTL_App();    //APP����		
			else if (ax_control_mode == CTL_RMS)    AX_CTL_Rms();    //��ģң��������
			
			//�����̨����
			JOINT_Control();
		
			//��ʱ
			vTaskDelay(20); 
		}
		else  //����ģʽ
		{
			//�ж��Ƿ�������
			if(ax_function_enable != 0)
			{
				//��ͬģʽ���컯��ʾ
				switch(ax_control_mode)
				{
					case CTL_FN1://����-�״������ʻ
						AX_FUN_List1();
						break;
					case CTL_FN2://����-�״����
						AX_FUN_List2();
						break;
					case CTL_FN3://����-����������
						AX_FUN_List3();
						break;			
					case CTL_FN4://����-����������
						AX_FUN_List4();
						break;				
					case CTL_FN5://����-CCDѲ��
						AX_FUN_List5();
						break;
					default://��ѭ��
						vTaskDelay(100);   
						break;
				}			
			}
			else
			{
				//��ʱ
				vTaskDelay(100);   
			}			
		}                                
	}	
}

/**
  * @��  ��  �����̨���ƺ���
  * @��  ��  �� 
  * @����ֵ  ��
  */
void JOINT_Control(void)   
{
	//�������Ʊ���
	if(ax_joint_angle[0] > JOINTA_UP_LIMIT)	   ax_joint_angle[0] = JOINTA_UP_LIMIT;
	if(ax_joint_angle[0] < JOINTA_LOW_LIMIT)   ax_joint_angle[0] = JOINTA_LOW_LIMIT;
	
	if(ax_joint_angle[1] > JOINTB_UP_LIMIT)	   ax_joint_angle[1] = JOINTB_UP_LIMIT;
	if(ax_joint_angle[1] < JOINTB_LOW_LIMIT)   ax_joint_angle[1] = JOINTB_LOW_LIMIT;
	
	//���ö���Ƕ�
	AX_SERVO_S1_SetAngle( ax_joint_angle[0] + JOINTA_ANGLE_OFFSET);
	AX_SERVO_S2_SetAngle( -ax_joint_angle[1] - JOINTB_ANGLE_OFFSET);	
	
	//�鿴����Ƕ�
	//printf("@%d %d \r\n ", ax_joint_angle[0], ax_joint_angle[1]);		
}

/**
  * @��  ��  ���¹�������
  * @��  ��  ��
  * @����ֵ  ��
  */
void Trivia_Task(void* parameter)
{	
	//��������
	static uint16_t ax_bat_vol_cnt = 0; 
	

	while (1)
	{	
		
		/*****��ع���***********************************/
		
		//�ɼ���ص�ѹ
	    ax_battery_vel = AX_VIN_GetVol_X100();
		
		//���������ص�ѹ����
        //printf("@ %d  \r\n",ax_battery_vel);		
		
		//��������40%
		if(ax_battery_vel < VBAT_40P)  
		{
			//��ƿ�ʼ��˸��ʾ
			AX_LED_Red_Toggle();
			
			//��������20%
			if(ax_battery_vel < VBAT_20P)
			{
				//��Ƴ���
				AX_LED_Red_On();
				
				//��������10%���ر�ϵͳ���뱣��״̬
				if(ax_battery_vel < VBAT_10P)
				{
					//��ѹʱ�����
					ax_bat_vol_cnt++;
					
					//����10�Σ�����ر�״̬
					if(ax_bat_vol_cnt > 10 )
					{
						//�ر��̵ƣ���Ƴ���
						AX_LED_Green_Off();
						AX_LED_Red_On();
						
						//�������
						vTaskSuspend(Disp_Task_Handle);
						
						//�رյ������
						ax_robot_move_enable = 0; 
						
						//���OLED����������ʾ
						AX_OLED_ClearScreen();  
						
						//���������б���
						while(1)
						{	
							//��ʾ������ֹͣ��Ϣ
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
			//��ƹر�
			AX_LED_Red_Off();
		}				
		
		/*****���������й���***********************************/
		if(ax_beep_ring != 0)
		{
			if(ax_beep_ring == BEEP_SHORT)
			{
				//������һ��
				AX_BEEP_On();
				vTaskDelay(200); 
				AX_BEEP_Off();
				
				//����һ����־��λ
				ax_beep_ring = 0;
			}
			else //
			{
				//������һ��
				AX_BEEP_On();
				vTaskDelay(1000); 
				AX_BEEP_Off();
				
				//����һ����־��λ
				ax_beep_ring = 0;				
			}
		}
		
		//LEDϵͳ����ָʾ
		AX_LED_Green_Toggle();	
		
		
        //ѭ������500ms
		vTaskDelay(500); 
	}			
}

/**
  * @��  ��  ������������
  * @��  ��  ��
  * @����ֵ  ��
  */
void Key_Task(void* parameter)
{	
	uint8_t  i;
	//int16_t  temp;
	
	while (1)
	{		
		//����ɨ��
		if(AX_KEY_Scan() != 0)
		{
			//�����ʱ
			vTaskDelay(50);  
			
			//ȷ����������
			if(AX_KEY_Scan() != 0)
			{
				//�ȴ�����̧��
				for(i=0; i<200; i++)
				{

					vTaskDelay(50);
					
					if(AX_KEY_Scan() == 0)
					{
						break;
					}
					
					//����ʱ��3Sʱ��������ʾ��
					if(i == 60)
					{
						AX_BEEP_On();
						vTaskDelay(200);
						AX_BEEP_Off();
					}
				}

				//�̰����,С��1S
				if(i < 20)
				{
					//����ģʽ�ر�
					ax_function_enable = 0;
					AX_BEEP_Off();
					
					//�ٶ�����Ϊ��
					ax_robot_vx = 0;
					ax_robot_vw = 0;
					
					//�л�����ģʽ
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
						//���״�����ж�
						AX_LASER_SetEnable(LS_ENABLE);
					}
					else
					{
						//�ر��״�����ж�
						AX_LASER_SetEnable(LS_DISABLE);
					}
						
				}
				
				//�а����,����3S��С��10S
				if(i>60 && i<200)
				{
					//����ģʽ����
					ax_function_enable = 1;
				}				
				
					
				//������⣬����10S
				if(i == 200)
				{
					//�������					
				}
			}
		}
		
		//ѭ������
		vTaskDelay(50);    
	}			
}


/**
  * @��  ��  �ƹ⴦������
  * @��  ��  ��
  * @����ֵ  ��
  */
void Light_Task(void* parameter)
{	

	while (1)
	{	
        //��ʱ30ms��30HZ��ʾƵ��		
		vTaskDelay(30); 
		
		//�ж�ǰ���Ƿ��
		if(ax_light_enable != 0)
		{
			//������ʾ
			AX_LIGHT_Show();
		}
		else
		{
			//�ر�״̬
			AX_RGB_SetFullColor(0x00, 0x00, 0x00);
		}
	}			
}


/**
  * @��  ��  ������������
  * @��  ��  ��
  * @����ֵ  ��
  */
void Disp_Task(void* parameter)
{	

	while (1)
	{	
		//��ʱ	
		vTaskDelay(100); 
		
		//��ʾ��ص�ѹ��������Z������,���ƫ��,
		AX_OLED_DispValue(30, 2, (ax_battery_vel*0.1), 2, 1, 0);
		
		//��ʾ�ǶȺͽ��ٶ�
		AX_OLED_DispValue(30, 3, (ax_balance_angle), 2, 2, 0);
		AX_OLED_DispValue(90, 3, (ax_balance_gyro), 6, 0, 0);		
		
		//��ʾ����ʵʱ�ٶ�
		AX_OLED_DispValue(30, 4, (ax_wheel_vel_l*100), 2, 2, 0);
		AX_OLED_DispValue(90, 4, (ax_wheel_vel_r*100), 2, 2, 0);
		
		//������Ϣ
		AX_OLED_DispValue(30, 7, (stX.x*1000), 4, 0, 0);
		AX_OLED_DispValue(90, 7, (stX.theta/PI*180), 4, 0, 0);
		
		//��ʱ	
		vTaskDelay(100); 
		
		//��ͬģʽ���컯��ʾ
		switch(ax_control_mode)
		{
			case CTL_PS2://PS2����
				AX_OLED_DispStr(90, 2, "PS2", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);
				break;
			case CTL_APP://APP����
				AX_OLED_DispStr(90, 2, "APP", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;
			case CTL_RMS://��ģң��������
				AX_OLED_DispStr(90, 2, "RMS", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;
			case CTL_ROS://ROS����
				AX_OLED_DispStr(90, 2, "ROS", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;
			case CTL_FN1://����-�״������ʻ
				AX_OLED_DispStr(90, 2, "FN1", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;
			case CTL_FN2://����-�״����
				AX_OLED_DispStr(90, 2, "FN2", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;
			case CTL_FN3://����-
				AX_OLED_DispStr(90, 2, "FN3", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;			
			case CTL_FN4://����-
				AX_OLED_DispStr(90, 2, "FN4", 0);  
				AX_OLED_DispStr(0, 6, "                     ", 0);	
				break;				
			case CTL_FN5://����-CCDѲ��
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
  * @��  ��  PS2���ݻ�ȡ����
  * @��  ��  ��
  * @����ֵ  ��
  */
void Ps2_Task(void* parameter)
{	
	while(1)
	{
		//��ȡPS2�ֱ���ֵ
		AX_PS2_ScanKey(&ax_joystick);
		
		//�ж��Ƿ���PS2�ֱ�����
		//START���������º����ҡ�����ƣ�����PS2����ģʽ
		if((ax_joystick.btn1 == PS2_BT1_START) && (ax_joystick.LJoy_UD == 0x00))
		{
			//�л���PS2ģʽ
			ax_control_mode = CTL_PS2;	

			//ִ�з�����������ʾ
			ax_beep_ring = BEEP_SHORT;
		}

		
		//��ʱ	
		vTaskDelay(20); 
		
//		//��ӡ�ֱ���ֵ
//		printf("MODE:%2x BTN1:%2x BTN2:%2x RJOY_LR:%2x RJOY_UD:%2x LJOY_LR:%2x LJOY_UD:%2x\r\n",
//		ax_joystick.mode, ax_joystick.btn1, ax_joystick.btn2, 
//		ax_joystick.RJoy_LR, ax_joystick.RJoy_UD, ax_joystick.LJoy_LR, ax_joystick.LJoy_UD);
	}
}



/******************* (C) ��Ȩ 2023 XTARK **************************************/

