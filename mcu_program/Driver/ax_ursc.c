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
  * @作  者  Musk Han@
  * @版  本  V1.0
  * @日  期  2022-7-26
  * @内  容  超声波测距
  *
  ******************************************************************************
  * @说  明
  *
  ******************************************************************************
  */

#include "ax_ursc.h" 

#include "ax_delay.h" 

static uint32_t ultrasonic_distance;  //超声波距离值
static uint32_t ultrasonic_tim;

uint8_t  US_TIM_CAPTURE_STA = 0;	//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
uint16_t US_TIM_CAPTURE_UPVAL;
uint16_t US_TIM_CAPTURE_DOWNVAL;


/**
  * @简  述  超声波初始化	
  * @参  数  无
  * @返回值  无
  */
void AX_URSC_Init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	NVIC_InitTypeDef NVIC_InitStructure;	
	
	//GPIO功能时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	//设置超声波ECHO端口
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//设置超声波TRIG端口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;       //发送电平引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1 );
	
	//TIM时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
	//设置定时器
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period=0xffff; //(65535-1); 设置在下一个更新事件装入活动的自动重装载寄存器周期的值         计数到1000为1ms
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;  //设置用来作为TIMx时钟频率除数的预分频值  1M的计数频率 1US计数
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	//初始化TIM5输入捕获参数 通道1
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM5, &TIM_ICInitStructure);	
	
	//定时器中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;  //先占优先级1级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

    //不允许更新中断，允许CC1IE捕获中断	
	TIM_ITConfig(TIM5, TIM_IT_CC1 , ENABLE);   
	
	//使能定时器	
	TIM_Cmd(TIM5, ENABLE); 			
}

/**
  * @简  述  定时器中断服务程序,超声波接收通道
  * @参  数  无
  * @返回值  无
  */
void TIM5_IRQHandler(void)
{
	if ((US_TIM_CAPTURE_STA & 0X80) == 0) 		//还未成功捕获	
	{
		//捕获1发生捕获事件
		if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET) 		
		{   
			//清除中断标志位
			TIM_ClearITPendingBit(TIM5, TIM_IT_CC1); 	

			//捕获到一个下降沿
			if (US_TIM_CAPTURE_STA & 0X40)		
			{
				//记录下此时的定时器计数值
				US_TIM_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM5);
				
				
				//如果计数器初始值大于末尾值，说明计数器有溢出
				if (US_TIM_CAPTURE_DOWNVAL < US_TIM_CAPTURE_UPVAL)
				{
					ultrasonic_tim = 65535;
				}
				else
				{
					ultrasonic_tim = 0;
				}
				
				//得到总的高电平的时间
				ultrasonic_distance = US_TIM_CAPTURE_DOWNVAL - US_TIM_CAPTURE_UPVAL + ultrasonic_tim;	
				
				//计算距离&&UltrasonicWave_Distance<85
				ultrasonic_distance = ultrasonic_distance *17/1000;			
				
				//捕获标志位清零，这一步很重要！
				US_TIM_CAPTURE_STA = 0;	

				//设置为上升沿捕获		 
				TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Rising);  
				
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				//获取上升沿数据
				US_TIM_CAPTURE_UPVAL = TIM_GetCapture1(TIM5);	
				
				//标记已捕获到上升沿
				US_TIM_CAPTURE_STA |= 0X40;		
				
				//设置为下降沿捕获
				TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Falling);
			}
		}
	}
}

/**
  * @简  述  启动超声波测距
  * @参  数  无
  * @返回值  无
  */
void AX_URSC_StartUp(void)
{
	  //拉高信号，作为触发信号
	  GPIO_SetBits(GPIOA, GPIO_Pin_1);
	
	  //高电平信号超过10us
	  AX_Delayus(20);
	  GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

/**
  * @简  述  获取超声波测距数据
  * @参  数  无
  * @返回值  超声波测距值，单位cm
  */
uint32_t AX_URSC_GetDistance(void)
{
    //返回距离值	
	return ultrasonic_distance;
	
}

/******************* (C) 版权 2023 XTARK **************************************/
