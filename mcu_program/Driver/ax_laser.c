/**			                                                    
		   ____                    _____ _______ _____       XTARK@塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @内  容  乐动STL-19P雷达处理
  *
  ******************************************************************************
  * @说  明
  *
  * 
  ******************************************************************************
  */

#include "ax_laser.h"
#include <stdio.h>

#include "ax_robot.h"

static uint8_t uart4_rx_con=0;       //接收计数器
static uint8_t uart4_rx_buf[50];     //接收缓冲

//扫描一圈的雷达数据
LaserPointTypeDef ax_ls_point[250];

//用于crc校验的数组
static const uint8_t CrcTable[256] =
{
 0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
 0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
 0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
 0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
 0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
 0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};


//函数定义
uint8_t LS_CalCRC8(uint8_t *pbuf, uint8_t len);
void LS_DataHandle(void);


/**
  * @简  述  雷达串口初始化
  * @参  数  无
  * @返回值	 无
  */
void AX_LASER_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//**USART配置******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //打开串口GPIO的时钟

	//将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);  //打开串口外设的时钟

	//配置USART参数
	USART_InitStructure.USART_BaudRate = 230400;		//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);

	//配置USART为中断源
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢断优先级	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能中断
	NVIC_Init(&NVIC_InitStructure);//初始化配置NVIC

	//使能 USART， 配置完毕
	USART_Cmd(UART4, ENABLE);	
	
	//串口接收中断-关闭状态
	USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
}


/**
  * @简  述  设置雷达串口接收是否打开
  * @参  数  state  串口接收中断使能开关
	        参数可设置为LS_ENABLE 或 LS_DISABLE
  * @返回值  无
  */	
void AX_LASER_SetEnable(uint8_t state)
{
	
	if (state != LS_DISABLE)
	{
		//串口接收中断设置
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	}
	else
	{
		//串口接收中断设置
		USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
	}
}
	

void UART4_IRQHandler(void)
{
	uint8_t Res;
	
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //接收中断
	{
		Res =USART_ReceiveData(UART4);	
		
		//接收数据
		if (uart4_rx_con < 2)
		{
			//接收帧头
			if(uart4_rx_con == 0)  
			{
				//判断帧头
				if(Res == LS_HEADER)
				{
					uart4_rx_buf[uart4_rx_con] = Res;
					uart4_rx_con = 1;					
				}
			}
			else
			{
				//判断VerLen
				if(Res == LS_VERLEN)
				{
					uart4_rx_buf[uart4_rx_con] = Res;
					uart4_rx_con = 2;					
				}
			}
		}
		else  //接收数据
		{
			//判断是否接收完
			if(uart4_rx_con < LS_F_LEN)
			{
				uart4_rx_buf[uart4_rx_con] = Res;
				uart4_rx_con++;
			}
			else
			{
				//CRC校验
				if(LS_CalCRC8(uart4_rx_buf,(LS_F_LEN-1)) == uart4_rx_buf[(LS_F_LEN-1)])
				{
					//接收完毕，进行帧数据处理
					LS_DataHandle();
					
					//printf("IDs  \r\n" );
				}				
				
				//复位
				uart4_rx_con = 0;
			}
		}
	} 
}




/**
  * @简  述  计算CRC8
  * @参  数  *pbuf：数据指针
  *          len：数据长度
  * @返回值	 无
  */
uint8_t LS_CalCRC8(uint8_t *pbuf, uint8_t len)
{
	uint8_t crc = 0;	
    uint8_t i;
	
	for(i=0; i<len; i++)
	{
		crc = CrcTable[(crc^*pbuf++) & 0xFF];
		
	}
	
	return crc;
}


/**
  * @简  述  数据处理函数
  * @参  数  无
  * @返回值	 无
  */
void LS_DataHandle(void)
{
	uint8_t i;
	float temp;
	
	static uint16_t cnt = 0;
	
	//每秒采集5000次10HZ，转一圈采集500个点，方便单片机处理，2个点取一个点
	static LaserPointTypeDef point[250];
	
	
	//一帧数据起始和结束角度
	float angle_start = (((u16)uart4_rx_buf[5]<<8) + uart4_rx_buf[4])/100.0;
	float angle_end   = (((u16)uart4_rx_buf[43]<<8) + uart4_rx_buf[42])/100.0;
	float angle_area;
	
	//转动90度
	if(angle_start < 270) angle_start = angle_start + 90;
	else angle_start = angle_start -270;
	
	if(angle_end < 270) angle_end = angle_end + 90;
	else angle_end = angle_end -270;
	
	//起始角度大于结束角度，跨过360°
	if(angle_start > angle_end)
	{
		angle_area = (angle_end + 360 - angle_start)/6;
		
		for(i=0; i<6; i++)
		{
		
			temp = angle_start + angle_area*i;
			
			if(temp > 360)
			{
				point[cnt+i].angle = (temp - 360) * 100;
			}
			else
			{
				point[cnt+i].angle = (temp) * 100;
			}
			
			//计算距离
			point[cnt+i].distance =  ((u16)uart4_rx_buf[7+i*6]<<8) + uart4_rx_buf[6+i*6];
		}
	}
	else
	{
		angle_area  = (angle_end - angle_start)/6;
		
		for(i=0; i<6; i++)
		{
			//计算角度
			point[cnt+i].angle =  (angle_start + angle_area*i)*100;
			
			//计算距离
			point[cnt+i].distance =  ((u16)uart4_rx_buf[7+i*6]<<8) + uart4_rx_buf[6+i*6];
		}		
	}
	
//	//打印调试信息
//	printf("@%d %d \r\n@%d %d \r\n",point[cnt].angle,point[cnt].distance,point[cnt+3].angle,point[cnt+3].distance);
//	
//	for(i=0; i<6; i++)
//	{
//		if(point[cnt+i].distance == 0)
//			printf("@%d %d \r\n",point[cnt+i].angle,point[cnt+i].distance);		
//	}
//	
//	if(point[cnt].angle > 35000)
//	{
//		printf("@%d %d \r\n@%d %d \r\n",point[cnt].angle,point[cnt].distance,point[cnt+3].angle,point[cnt+3].distance);
//	}
//	cnt = 0;
	
	//一帧数据解析结束
	cnt = cnt+6;
	
	//判断是否转弯一圈（雷达转一圈大概有250个点）
	if(cnt > 249)
	{
		//将数组的数据转移到外部数组中，避免覆盖数据
		for(i=0; i<250; i++)
		{
			//计算角度
			ax_ls_point[i].angle = point[i].angle;
			ax_ls_point[i].distance = point[i].distance;
		}
		//复位
		cnt = 0;
	}
	
	
}

/******************* (C) 版权 2023 XTARK **************************************/
