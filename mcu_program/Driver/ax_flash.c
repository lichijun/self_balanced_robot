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
  * @��  ��  FLASH��д
  *
  ******************************************************************************
  */

#include "ax_flash.h"
#include <stdio.h>


//STM32F103RCT6�����ܹ���0~127��������ÿ��������СΪ2K
//����ѡ����󼸸�����������һ���ò�����λ��
#define TK_FLASH_DATA_SECTOR    126   
                  
//FLASH����ؼ���
#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)				  
				  
//STM32F103RCT6��FLASH������                 
#define TK_FLASH_SIZE      256              //��ѡSTM32��FLASH������С(��λΪK)
#define TK_FLASH_WREN      1                //ʹ��FLASHд��(0����ʹ��;1��ʹ��)
#define TK_SECTOR_SIZE     2048             //������ÿ����������Ϊ2K
#define TK_FLASH_BASE      0x08000000      //STM32 FLASH����ʼ��ַ
#define TK_FLASH_END       0x0803FFFF      //STM32F103RCT6 FLASH�Ľ�����ַ

//FLASH��д����ռ�
uint16_t flash_buf[TK_SECTOR_SIZE / 2]; //2K�ֽ�

//�ڲ���������
static void Flash_Unlock(void);
static void Flash_Lock(void);
static uint8_t Flash_GetStatus(void);
static uint8_t Flash_WaitDone(uint16_t time);
static uint8_t Flash_WriteHalfWord(uint32_t faddr, uint16_t dat);
static uint16_t Flash_ReadHalfWord(uint32_t faddr);
static void Flash_Write_NoCheck(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite);
static uint8_t Flash_ErasePage(uint32_t paddr);
static void Flash_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead);
static void Flash_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite);


/**
  * @��  ��  ��ָ����ַ��ʼ����ָ�����ȵ�����
  * @��  ��  addr:��ʼ��ַ���˵�ַ����Ϊ2�ı�������Χ0~2048
             pbuff:����ָ��
             num:����(16λ)��
  * @����ֵ	 ��
  */
void AX_FLASH_Read(uint16_t addr, uint16_t *pbuff, uint8_t num)
{
	uint32_t flash_addr;	  
	
	//����ʵ�ʵ�ַ
	flash_addr = TK_FLASH_BASE + TK_FLASH_DATA_SECTOR * TK_SECTOR_SIZE + addr;
	
	//��ȡ����
	Flash_Read(flash_addr, pbuff, num);
}

/**
  * @��  ��  ��ָ����ַ��ʼд��ָ�����ȵ�����
  * @��  ��  addr:��ʼ��ַ���˵�ַ����Ϊ2�ı�������Χ0~2048
             pbuff:����ָ��
             NumToWrite:����(16λ)��(����Ҫд���16λ���ݵĸ���.)
  * @����ֵ	 ��
  */
void AX_FLASH_Write(uint16_t addr, uint16_t *pbuff, uint8_t num)
{
	uint32_t flash_addr;

	
	
	//����ʵ�ʵ�ַ
	flash_addr = TK_FLASH_BASE + TK_FLASH_DATA_SECTOR * TK_SECTOR_SIZE + addr;
	
	//д������
	Flash_Write(flash_addr, pbuff, num);
}

/**
  * @��  ��  ��������ҳ
  * @��  ��  ��
  * @����ֵ	 ��
  */
void AX_FLASH_Erase(void)
{
	//����ѡ����ҳ
	Flash_ErasePage(TK_FLASH_DATA_SECTOR);
}


//�ڲ�����-------------------------------------------------------------------------------------

/**
  * @��  ��  ����STM32��FLASH
  * @��  ��  ��
  * @����ֵ	 ��
  */
static void Flash_Unlock(void)
{
	//д���������
	FLASH->KEYR = FLASH_KEY1; 
	FLASH->KEYR = FLASH_KEY2;
}


/**
  * @��  ��  ����STM32��FLASH
  * @��  ��  ��
  * @����ֵ	 ��
  */
static void Flash_Lock(void)
{
	 //����
	FLASH->CR |= 1 << 7;
}

/**
  * @��  ��  �õ�FLASH״̬
  * @��  ��  ��
  * @����ֵ	 ״̬
  */
static uint8_t Flash_GetStatus(void)
{
	uint32_t res;
	res = FLASH->SR;
	
	//�ж�״̬
	if (res & (1 << 0))
		return 1; //æ
	else if (res & (1 << 2))
		return 2; //��̴���
	else if (res & (1 << 4))
		return 3; //д��������
	return 0;	  //�������
}

/**
  * @��  ��  �ȴ�������ɽ�
  * @��  ��  time:Ҫ��ʱ�ĳ���
  * @����ֵ	 ״̬
  */
static uint8_t Flash_WaitDone(uint16_t time)
{
	uint8_t res, i;
	
	//ѭ���ж�
	do
	{
		res = Flash_GetStatus();
		if (res != 1)
			break; //��æ,����ȴ���,ֱ���˳�.
		for (i = 0; i < 10; i++); // �����ӳ�
		time--;
	} while (time);
	
	
	//TIMEOUT
	if (time == 0)
		res = 0xff; 
	
	return res;
}

/**
  * @��  ��  ��FLASHָ����ַд�����
  * @��  ��  dat:Ҫд�������
  * @����ֵ	 д������
  */
static uint8_t Flash_WriteHalfWord(uint32_t faddr, uint16_t dat)
{
	uint8_t res;
	
	res = Flash_WaitDone(0XFF);
	
	//OK
	if (res == 0) 
	{
		FLASH->CR |= 1 << 0;		   //���ʹ��
		*(vu16 *)faddr = dat;		   //д������
		res = Flash_WaitDone(0XFF); //�ȴ��������
		if (res != 1)				   //�����ɹ�
		{
			FLASH->CR &= ~(1 << 0); //���PGλ.
		}
	}
	return res;
}

/**
  * @��  ��  ��ȡָ����ַ�İ���(16λ����)
  * @��  ��  faddr:����ַ
  * @����ֵ	 ��Ӧ����
  */
static uint16_t Flash_ReadHalfWord(uint32_t faddr)
{
	return *(vu16 *)faddr;
}

/**
  * @��  ��  ������д��
  * @��  ��  WriteAddr:��ʼ��ַ
  *          pBuffer:����ָ��
  *          NumToWrite:����(16λ)��
  * @����ֵ	 ��
  */
static void Flash_Write_NoCheck(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
	uint16_t i;
	for (i = 0; i < NumToWrite; i++)
	{
		Flash_WriteHalfWord(WriteAddr, pBuffer[i]);
		WriteAddr += 2; //��ַ����2.
	}
}

/**
  * @��  ��  ����ҳ
  * @��  ��  paddr:ҳ��ַ
  * @����ֵ	 ִ�����
  */
static uint8_t Flash_ErasePage(uint32_t paddr)
{
	uint8_t res = 0;
	
	//�ȴ��ϴβ�������,>20ms
	res = Flash_WaitDone(0X5FFF); 
	
	
	if (res == 0)
	{
		FLASH->CR |= 1 << 1;			 //ҳ����
		FLASH->AR = paddr;				 //����ҳ��ַ
		FLASH->CR |= 1 << 6;			 //��ʼ����
		res = Flash_WaitDone(0X5FFF);    //�ȴ���������,>20ms
		if (res != 1)					 //��æ
		{
			FLASH->CR &= ~(1 << 1); //���ҳ������־.
		}
	}
	
	return res;	
}




/**
  * @��  ��  ��ָ����ַ��ʼ����ָ�����ȵ�����
  * @��  ��  ReadAddr:��ʼ��ַ
             pBuffer:����ָ��
             NumToWrite:����(16λ)��
  * @����ֵ	 ��
  */
static void Flash_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead)
{
	uint16_t i;
	for (i = 0; i < NumToRead; i++)
	{
		pBuffer[i] = Flash_ReadHalfWord(ReadAddr);   //��ȡ2���ֽ�.
		ReadAddr += 2;								 //ƫ��2���ֽ�.
	}
}

/**
  * @��  ��  ��ָ����ַ��ʼд��ָ�����ȵ�����
  * @��  ��  WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
             pBuffer:����ָ��
             NumToWrite:����(16λ)��(����Ҫд���16λ���ݵĸ���.)
  * @����ֵ	 ��
  */
static void Flash_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
	uint32_t secpos;	   //������ַ
	uint16_t secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
	uint16_t secremain;    //������ʣ���ַ(16λ�ּ���)
	uint16_t i;
	uint32_t offaddr;      //ȥ��0X08000000��ĵ�ַ
	
	 //�Ƿ���ַ
	if (WriteAddr < TK_FLASH_BASE || (WriteAddr >= (TK_FLASH_BASE + 1024 * TK_FLASH_SIZE)))
		return;								 
	
	 //����
	Flash_Unlock();						 
	
	//�����ַ
	offaddr = WriteAddr - TK_FLASH_BASE;	  //ʵ��ƫ�Ƶ�ַ.
	secpos = offaddr / TK_SECTOR_SIZE;		  //������ַ  0~127 for STM32F103RCT6
	secoff = (offaddr % TK_SECTOR_SIZE) / 2; //�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
	secremain = TK_SECTOR_SIZE / 2 - secoff; //����ʣ��ռ��С
	
	//�����ڸ�������Χ
	if (NumToWrite <= secremain)
		secremain = NumToWrite; 
	
	//д�����
	while (1)
	{
		//������������������
		Flash_Read(secpos * TK_SECTOR_SIZE + TK_FLASH_BASE, flash_buf, TK_SECTOR_SIZE / 2); 
		
		//У������
		for (i = 0; i < secremain; i++)															 
		{
			if (flash_buf[secoff + i] != 0XFFFF)
				break; //��Ҫ����
		}
		
		//��Ҫ����
		if (i < secremain) 
		{
			//�����������
			Flash_ErasePage(secpos * TK_SECTOR_SIZE + TK_FLASH_BASE); 
			
			//��������
			for (i = 0; i < secremain; i++)								 
			{
				flash_buf[i + secoff] = pBuffer[i];
			}
			
			//д����������
			Flash_Write_NoCheck(secpos * TK_SECTOR_SIZE + TK_FLASH_BASE, flash_buf, TK_SECTOR_SIZE / 2); 
		}
		else
			Flash_Write_NoCheck(WriteAddr, pBuffer, secremain);   //д�Ѿ������˵�,ֱ��д������ʣ������.
		
		//�ж��Ƿ�д�����
		if (NumToWrite == secremain)
			break; //д�������
		else	   //д��δ����
		{
			secpos++;					//������ַ��1
			secoff = 0;					//ƫ��λ��Ϊ0
			pBuffer += secremain;		//ָ��ƫ��
			WriteAddr += secremain * 2; //д��ַƫ��(16λ���ݵ�ַ,��Ҫ*2)
			NumToWrite -= secremain;	//�ֽ�(16λ)���ݼ�
			if (NumToWrite > (TK_SECTOR_SIZE / 2))
				secremain = TK_SECTOR_SIZE / 2; //��һ����������д����
			else
				secremain = NumToWrite; //��һ����������д����
		}
	};
	
	//����
	Flash_Lock(); 
}

/******************* (C) ��Ȩ 2023 XTARK **************************************/
