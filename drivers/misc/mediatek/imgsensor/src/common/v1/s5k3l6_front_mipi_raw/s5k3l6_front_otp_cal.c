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
#include "../../../../common/v1/dualcam_utils.h"

#define CALI_PFX "[S5K3L6_FRONT_CALI]"
#define CALI_INF(fmt, args...)   pr_debug(CALI_PFX "[%s] " fmt, __FUNCTION__, ##args)
#define CALI_ERR(fmt, args...)   pr_err(CALI_PFX "[%s] " fmt, __FUNCTION__, ##args)

#define BYTE                unsigned char

#define EEPROM              BL24SA64
#define EEPROM_READ_ID    	(0xA3)
#define EEPROM_WRITE_ID    	(0xA2)

#define MAX_OFFSET          (0xffff)

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

static bool selective_read_eeprom(u16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	if(addr > MAX_OFFSET)
		return false;

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, EEPROM_READ_ID)<0)
		return false;

	return true;
}

static bool selective_write_eeprom(u16 addr, BYTE* data)
{
	char pu_send_cmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(*data & 0xFF)};
	if(addr > MAX_OFFSET)
		return false;

	if(iWriteRegI2C(pu_send_cmd, 3, EEPROM_WRITE_ID)<0)
		return false;
    mdelay(5);
	return true;
}

static int write_eeprom_s5k3l6_front(u16 addr, BYTE* data, u32 size )
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

static int read_eeprom_s5k3l6_front(u16 addr, BYTE* data, u32 size )
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

