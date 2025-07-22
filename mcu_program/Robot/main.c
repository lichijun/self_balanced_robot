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
  * @��  ��  ��������
  * 
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include "ax_robot.h"

#include "ax_oled_chinese.h" //OLED���ֿ�
#include "ax_oled_picture.h" //OLED ͼƬ��

//������
//��������
#define START_TASK_PRIO		1
#define START_STK_SIZE 		256  
TaskHandle_t StartTask_Handler = NULL;
void Start_Task(void *pvParameters);

//���ƺ͹�������
#define CONTROL_TASK_PRIO		3     
#define CONTROL_STK_SIZE 		256 
TaskHandle_t Control_Task_Handle = NULL;
void Control_Task(void *pvParameters);

//������������
#define KEY_TASK_PRIO		4     
#define KEY_STK_SIZE 		128   
TaskHandle_t Key_Task_Handle = NULL;
void Key_Task(void *pvParameters);

//RGB��Ч����
#define LIGHT_TASK_PRIO		5     
#define LIGHT_STK_SIZE 		128   
TaskHandle_t Light_Task_Handle = NULL;
void Light_Task(void *pvParameters);

//OLED��ʾ����
#define DISP_TASK_PRIO		6     
#define DISP_STK_SIZE 		128   
TaskHandle_t Disp_Task_Handle = NULL;
void Disp_Task(void *pvParameters);

//���¹�������
#define TRIVIA_TASK_PRIO		10     
#define TRIVIA_STK_SIZE 		128   
TaskHandle_t Trivia_Task_Handle = NULL;
void Trivia_Task(void *pvParameters);

//PS2���ݻ�ȡ����
#define PS2_TASK_PRIO		11     
#define PS2_STK_SIZE 		128   
TaskHandle_t Ps2_Task_Handle = NULL;
void Ps2_Task(void *pvParameters);


/**
  * @��  ��  ����������
  * @��  ��  ��
  * @����ֵ  ��
  */
int main(void)
{	

	//�����ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);   
	
	//�����ʼ��
	AX_SERVO_S1234_Init();	

	//�����ʼ��
	AX_MOTOR_Init();
	
	//��ʱ������ʼ��
	AX_DELAY_Init();  
	
	//JTAG��ʼ��
	AX_JTAG_Set(JTAG_SWD_DISABLE);    	
	AX_JTAG_Set(SWD_ENABLE);      
    
	//LED��ʼ��
	AX_LED_Init();  
	
	//KEY��������ʼ��
	AX_KEY_Init();
	
	//��ص�ѹ����ʼ��
	AX_VIN_Init();
	
	//��������ʼ��
	AX_BEEP_Init();  
	
	//���Դ��ڳ�ʼ��
	AX_UART1_Init(230400);	
	
	//�����״ﴫ������ʼ��
	AX_LASER_Init();
	
	//�������ڳ�ʼ��
	AX_UART2_Init(115200);
	
	//��ģң����SBUS���ڳ�ʼ��
	AX_SBUS_Init();
	
	//PS2�ֱ���ʼ��
	AX_PS2_Init();
	
	//��������ʼ��
	AX_ENCODER_A_Init();  
	AX_ENCODER_B_Init(); 

	//RGB�ʵ�
	AX_RGB_Init();	
	AX_RGB_SetFullColor(0, 0, 0xff);
	
	//OLED��Ļ��ʼ��
	AX_OLED_Init();	
	AX_OLED_DispPicture(0, 0, 128, 8, PIC64X128_XTARK, 0); 
	
	//������ʾ��Ϣ
	AX_BEEP_On();
	AX_Delayms(100);	
	AX_BEEP_Off();
	AX_Delayms(1900);
	
	//MPU6050��ʼ��
	AX_MPU6050_Init();  //MPU6050��ʼ��  
	AX_MPU6050_DMP_Init();  //DMP��ʼ��
	
	//CCD��������ʼ��
	AX_CCD_Init();

	//����AppTaskCreate����
	xTaskCreate((TaskFunction_t )Start_Task,  /* ������ں��� */
								 (const char*    )"Start_Task",/* �������� */
								 (uint16_t       )START_STK_SIZE,  /* ����ջ��С */
								 (void*          )NULL,/* ������ں������� */
								 (UBaseType_t    )START_TASK_PRIO, /* ��������ȼ� */
								 (TaskHandle_t*  )&StartTask_Handler);/* ������ƿ�ָ�� */ 
							
	//�������񣬿�������						 
	vTaskStartScheduler(); 

	//ѭ��
	while (1);
}


/**
  * @��  ��  ����������
  * @��  ��  ��
  * @����ֵ  ��
  */
void Start_Task(void *pvParameters)
{
	/******��������������************************************************/
	
	//Ĭ�ϵ�Ч����
	R_Light.M  = LEFFECT2;  //����Ч��
	R_Light.S  = 0;
	R_Light.T  = 0;
	R_Light.R  = 0x00;
	R_Light.G  = 0xFF;
	R_Light.B  = 0xFF;	
	
	//���ö���Ƕȣ�������ƫ����
	AX_SERVO_S1_SetAngle(JOINTA_ANGLE_OFFSET);
	AX_SERVO_S2_SetAngle(JOINTB_ANGLE_OFFSET);		
	
	//����������ɣ��̵Ƶ�������������ʾ
	AX_LED_Green_On();	
	AX_BEEP_On();
	AX_Delayms(100);	
	AX_BEEP_Off();	
	
	//��ʾ�����ڽ���
	AX_OLED_ClearScreen();  //���OLED����������ʾ
	AX_OLED_DispStr(0, 0, "   * TARKBOT B680 *   ", 0);	
	AX_OLED_DispStr(0, 1, "---------------------", 0);	
	AX_OLED_DispStr(0, 2, " Vol:12.2V Mod:ROS   ", 0);	
	AX_OLED_DispStr(0, 3, " Agl:00.00 Gyo:00000 ", 0);
	AX_OLED_DispStr(0, 4, " MTA:00.00 MTB:-0.00 ", 0);	
	AX_OLED_DispStr(0, 5, "---------------------", 0);	
	AX_OLED_DispStr(0, 6, "                     ", 0);	

	//�����ٽ���
	taskENTER_CRITICAL();           
  
	//���������˿�������
	xTaskCreate((TaskFunction_t )Control_Task, /* ������ں��� */
			 (const char*    )"Control_Task",/* �������� */
			 (uint16_t       )CONTROL_STK_SIZE,   /* ����ջ��С */
			 (void*          )NULL,	/* ������ں������� */
			 (UBaseType_t    )CONTROL_TASK_PRIO,	    /* ��������ȼ� */
			 (TaskHandle_t*  )&Control_Task_Handle);/* ������ƿ�ָ�� */
			 	 								 
	//����������������
	xTaskCreate((TaskFunction_t )Key_Task, /* ������ں��� */
			 (const char*    )"Key_Task",/* �������� */
			 (uint16_t       )KEY_STK_SIZE,   /* ����ջ��С */
			 (void*          )NULL,	/* ������ں������� */
			 (UBaseType_t    )KEY_TASK_PRIO,	    /* ��������ȼ� */
			 (TaskHandle_t*  )&Key_Task_Handle);/* ������ƿ�ָ�� */
			 
	//RGB��Ч����
	xTaskCreate((TaskFunction_t )Light_Task, /* ������ں��� */
			 (const char*    )"Light_Task",/* �������� */
			 (uint16_t       )LIGHT_STK_SIZE,   /* ����ջ��С */
			 (void*          )NULL,	/* ������ں������� */
			 (UBaseType_t    )LIGHT_TASK_PRIO,	    /* ��������ȼ� */
			 (TaskHandle_t*  )&Light_Task_Handle);/* ������ƿ�ָ�� */			

	//OLED����ʾ����
	xTaskCreate((TaskFunction_t )Disp_Task, /* ������ں��� */
			 (const char*    )"Disp_Task",/* �������� */
			 (uint16_t       )DISP_STK_SIZE,   /* ����ջ��С */
			 (void*          )NULL,	/* ������ں������� */
			 (UBaseType_t    )DISP_TASK_PRIO,	    /* ��������ȼ� */
			 (TaskHandle_t*  )&Disp_Task_Handle);/* ������ƿ�ָ�� */	
			 
	//���¹�������
	xTaskCreate((TaskFunction_t )Trivia_Task, /* ������ں��� */
			 (const char*    )"Trivia_Task",/* �������� */
			 (uint16_t       )TRIVIA_STK_SIZE,   /* ����ջ��С */
			 (void*          )NULL,	/* ������ں������� */
			 (UBaseType_t    )TRIVIA_TASK_PRIO,	    /* ��������ȼ� */
			 (TaskHandle_t*  )&Trivia_Task_Handle);/* ������ƿ�ָ�� */

	//PS2�ֱ����ݶ�ȡ����
	xTaskCreate((TaskFunction_t )Ps2_Task, /* ������ں��� */
			 (const char*    )"Ps2_Task",/* �������� */
			 (uint16_t       )PS2_STK_SIZE,   /* ����ջ��С */
			 (void*          )NULL,	/* ������ں������� */
			 (UBaseType_t    )PS2_TASK_PRIO,	    /* ��������ȼ� */
			 (TaskHandle_t*  )&Ps2_Task_Handle);/* ������ƿ�ָ�� */				 
			 
						  
	//ɾ��AppTaskCreate����				
	vTaskDelete(StartTask_Handler); 

	//�˳��ٽ���
	taskEXIT_CRITICAL();  		 
						 							
}

/******************* (C) ��Ȩ 2023 XTARK **************************************/

