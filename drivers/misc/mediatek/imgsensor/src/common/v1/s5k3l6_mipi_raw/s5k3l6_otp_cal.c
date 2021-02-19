/**
Copyright(C) 2015 Transsion Inc
*/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/proc_fs.h>
#include <linux/dma-mapping.h>
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#define CALI_PFX "[S5K3L6_CALI]"
#define CALI_INF(fmt, args...)   pr_debug(CALI_PFX "[%s] " fmt, __FUNCTION__, ##args)
#define CALI_ERR(fmt, args...)   pr_err(CALI_PFX "[%s] " fmt, __FUNCTION__, ##args)

#define BYTE                unsigned char

#define EEPROM              BL24SA64
#define EEPROM_READ_ID    	(0xA1)
#define EEPROM_WRITE_ID    	(0xA0)

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

static bool selective_read_eeprom(u16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, EEPROM_READ_ID)<0)
		return false;

	return true;
}

static bool selective_write_eeprom(u16 addr, BYTE* data)
{
	char pu_send_cmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(*data & 0xFF)};

	if(iWriteRegI2C(pu_send_cmd, 3, EEPROM_WRITE_ID)<0)
		return false;
    mdelay(5);
	return true;
}

int write_eeprom_s5k3l6(u16 addr, BYTE* data, u32 size )
{
	int i = 0;
	BYTE flag = 0x00;
	int offset = addr;

	/*switched off write protect*/
	selective_write_eeprom(0x8000, &flag);
	mdelay(1);
	/*check again*/
	selective_read_eeprom(0x8000, &flag);
	if(flag != 0x00)
	{
		CALI_ERR("close write protect failed");
		return -1;
	}

	for(i = 0; i < size; i++)
	{
		if(!selective_write_eeprom(offset, &data[i]))
		{
			CALI_ERR("write failed, now/total:(%d)/(%d)", i, size);
			return -1;
		}
		//CALI_INF("write_eeprom 0x%x 0x%x\n",offset, data[i]);
		offset++;
		msleep(1);
	}
	return i;
}

int read_eeprom_s5k3l6(u16 addr, BYTE* data, u32 size )
{
	int i = 0;
	int offset = addr;

	for(i = 0; i < size; i++)
	{
		if(!selective_read_eeprom(offset, &data[i]))
		{
			CALI_ERR("read failed, now/total:(%d)/(%d)", i, size);
			return -1;
		}
		//CALI_INF("read_eeprom 0x%x 0x%x\n",offset, data[i]);
		offset++;
	}
	return i;
}

static int update_3l6_eepromInfo_flag = 0;

bool update_3l6_eepromInfo(void)
{
	int result = 1;
	int i = 0;
	BYTE buffer = 0;
	BYTE buffer4[4] = {0};
	
	if(update_3l6_eepromInfo_flag == 0)
		update_3l6_eepromInfo_flag = 1;
	else
		return 1;
	
	selective_read_eeprom(0x0000, &buffer);
	if(buffer != 0x1)
	{
		printk("update_3l6_eepromInfo read error 0x0000:0x%x\n",buffer);
		return 0;
	}
	
	selective_read_eeprom(0x0001, &buffer);
	printk("update_3l6_eepromInfo read module id:0x%x",buffer);
	
	selective_read_eeprom(0x000c, &buffer);
	if(buffer != 0x1)
	{
		printk("update_3l6_eepromInfo read awb flag error 0x000c:0x%x\n",buffer);
		return 0;
	}
	
	read_eeprom_s5k3l6(0x000d, buffer4, 4);
	printk("update_3l6_eepromInfo read awb r:0x%x,gr:0x%x,gb:0x%x,b:0x%x\n",buffer4[0],buffer4[1],buffer4[2],buffer4[3]);
	read_eeprom_s5k3l6(0x0011, buffer4, 4);
	printk("update_3l6_eepromInfo read golden awb r:0x%x,gr:0x%x,gb:0x%x,b:0x%x\n",buffer4[0],buffer4[1],buffer4[2],buffer4[3]);

	printk("update_3l6_eepromInfo read lsc data start\n");
	for(i=0x0017;i<=0x0762;i++)
	{
		selective_read_eeprom(i, &buffer);
		printk("0x%x ",buffer);
	}
	printk("update_3l6_eepromInfo read lsc data end\n");

	selective_read_eeprom(0x0764, &buffer);
	if(buffer != 0x1)
	{
		printk("update_3l6_eepromInfo read pdaf step1 flag error 0x0764:0x%x\n",buffer);
		return 0;
	}
	
	printk("update_3l6_eepromInfo read pdaf step1 data start\n");
	for(i=0x0765;i<=0x0954;i++)
	{
		selective_read_eeprom(i, &buffer);
		printk("0x%x ",buffer);
	}
	printk("update_3l6_eepromInfo read lsc data end\n");
	
	selective_read_eeprom(0x0956, &buffer);
	if(buffer != 0x1)
	{
		printk("update_3l6_eepromInfo read pdaf step2 flag error 0x0764:0x%x\n",buffer);
		return 0;
	}

	printk("update_3l6_eepromInfo read pdaf step2 data start\n");
	for(i=0x0957;i<=0x0ce2;i++)
	{
		selective_read_eeprom(i, &buffer);
		printk("0x%x,",buffer);
	}
	printk("update_3l6_eepromInfo read pdaf step2 data end\n");
	
	selective_read_eeprom(0x0ce6, &buffer);
	if(buffer != 0x1)
	{
		printk("update_3l6_eepromInfo read af flag error 0x0ce6:0x%x\n",buffer);
		return 0;
	}
	
	read_eeprom_s5k3l6(0x0ce7, buffer4, 4);
	printk("update_3l6_eepromInfo read af Macro positon H:0x%x,Macro positon L:0x%x,Infinty positon H:0x%x,Infinty positon L:0x%x\n",buffer4[0],buffer4[1],buffer4[2],buffer4[3]);
	
	return  result;
}