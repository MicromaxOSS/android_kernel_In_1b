/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2019, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_gestrue.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"
#if FTS_GESTURE_EN
/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define GESTURE_DOUBLECLICK                     0x24
#define GESTURE_V                               0x54
#define GESTURE_LEFT                            0x51
#define GESTURE_RIGHT                           0x52
#define GESTURE_O                               0x57
#define GESTURE_O_1                             0x30
#define GESTURE_M                               0x32
#define GESTURE_W                               0x31

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
/*
* gesture_id    - mean which gesture is recognised
* point_num     - points number of this gesture
* coordinate_x  - All gesture point x coordinate
* coordinate_y  - All gesture point y coordinate
* mode          - gesture enable/disable, need enable by host
*               - 1:enable gesture function(default)  0:disable
* active        - gesture work flag,
*                 always set 1 when suspend, set 0 when resume
*/
struct fts_gesture_st {
    u8 gesture_id;
    u8 point_num;
    u16 coordinate_x[FTS_GESTURE_POINTS_MAX];
    u16 coordinate_y[FTS_GESTURE_POINTS_MAX];
    u8 mode;
    u8 active;
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_gesture_st fts_gesture_data;

u16 x_max = 0x00;
u16 y_max = 0x00;
u16 x_min = 0xffff;
u16 y_min = 0xffff;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static ssize_t fts_gesture_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    fts_read_reg(FTS_REG_GESTURE_EN, &val);
    count = snprintf(buf, PAGE_SIZE, "Gesture Mode:%s\n",
                     fts_gesture_data.mode ? "On" : "Off");
    count += snprintf(buf + count, PAGE_SIZE, "Reg(0xD0)=%d\n", val);
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_gesture_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;
    mutex_lock(&input_dev->mutex);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        FTS_DEBUG("enable gesture");
        fts_gesture_data.mode = ENABLE;
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        FTS_DEBUG("disable gesture");
        fts_gesture_data.mode = DISABLE;
    }
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_gesture_buf_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    int i = 0;
    struct input_dev *input_dev = fts_data->input_dev;
    struct fts_gesture_st *gesture = &fts_gesture_data;

    mutex_lock(&input_dev->mutex);
    count = snprintf(buf, PAGE_SIZE, "Gesture ID:%d\n", gesture->gesture_id);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture PointNum:%d\n",
                      gesture->point_num);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture Points Buffer:\n");

    /* save point data,max:6 */
    for (i = 0; i < gesture->point_num; i++) {
        count += snprintf(buf + count, PAGE_SIZE, "%3d(%4d,%4d) ", i,
                          gesture->coordinate_x[i], gesture->coordinate_y[i]);
        if ((i + 1) % 4 == 0)
            count += snprintf(buf + count, PAGE_SIZE, "\n");
    }
    count += snprintf(buf + count, PAGE_SIZE, "\n");
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_gesture_buf_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}


/* sysfs gesture node
 *   read example: cat  fts_gesture_mode       ---read gesture mode
 *   write example:echo 1 > fts_gesture_mode   --- write gesture mode to 1
 *
 */
static DEVICE_ATTR(fts_gesture_mode, S_IRUGO | S_IWUSR, fts_gesture_show,
                   fts_gesture_store);
/*
 *   read example: cat fts_gesture_buf        --- read gesture buf
 */
static DEVICE_ATTR(fts_gesture_buf, S_IRUGO | S_IWUSR,
                   fts_gesture_buf_show, fts_gesture_buf_store);

static struct attribute *fts_gesture_mode_attrs[] = {
    &dev_attr_fts_gesture_mode.attr,
    &dev_attr_fts_gesture_buf.attr,
    NULL,
};

static struct attribute_group fts_gesture_group = {
    .attrs = fts_gesture_mode_attrs,
};

int fts_create_gesture_sysfs(struct device *dev)
{
    int ret = 0;

    ret = sysfs_create_group(&dev->kobj, &fts_gesture_group);
    if (ret) {
        FTS_ERROR("gesture sys node create fail");
        sysfs_remove_group(&dev->kobj, &fts_gesture_group);
        return ret;
    }

    return 0;
}

static void fts_gesture_report(struct input_dev *input_dev, int gesture_id)
{
	struct fts_gesture_st *gesture = &fts_gesture_data;

    FTS_DEBUG("gesture_id:0x%x", gesture_id);

    switch (gesture_id) {
	case GESTURE_DOUBLECLICK:
        gesture_report[0] = 0x01;

		gesture_report[1] = fts_gesture_data.coordinate_x[0];
		gesture_report[2] = fts_gesture_data.coordinate_y[0];

		gesture_report[13] = 0x00;
        break;
	case GESTURE_V:
        gesture_report[0] = 0x02;
		if (fts_gesture_data.coordinate_x[0] < fts_gesture_data.coordinate_x[gesture->point_num -1])
		{
			gesture_report[1] = fts_gesture_data.coordinate_x[0];
			gesture_report[2] = fts_gesture_data.coordinate_y[0];

			gesture_report[3] = fts_gesture_data.coordinate_x[gesture->point_num -1];
			gesture_report[4] = fts_gesture_data.coordinate_y[gesture->point_num -1];

			gesture_report[5] = (fts_gesture_data.coordinate_x[0] + fts_gesture_data.coordinate_x[gesture->point_num -1])/2;
			gesture_report[6] = y_max;

			gesture_report[13] = 0x00;
		}
		else
		{
			gesture_report[1] = fts_gesture_data.coordinate_x[0];
			gesture_report[2] = fts_gesture_data.coordinate_y[0];

			gesture_report[3] = fts_gesture_data.coordinate_x[gesture->point_num -1];
			gesture_report[4] = fts_gesture_data.coordinate_y[gesture->point_num -1];

			gesture_report[5] = (fts_gesture_data.coordinate_x[0] + fts_gesture_data.coordinate_x[gesture->point_num -1]) / 2;
			gesture_report[6] = y_max;

			gesture_report[13] = 0x01;
		}
        break;
    case GESTURE_RIGHT:
        gesture_report[0] = 0x04;
		if (fts_gesture_data.coordinate_y[0] < fts_gesture_data.coordinate_y[gesture->point_num -1])
		{
			gesture_report[1] = fts_gesture_data.coordinate_x[0];
			gesture_report[2] = fts_gesture_data.coordinate_y[0];

			gesture_report[3] = fts_gesture_data.coordinate_x[gesture->point_num -1];
			gesture_report[4] = fts_gesture_data.coordinate_y[gesture->point_num -1];

			gesture_report[5] = x_max;
			gesture_report[6] = (fts_gesture_data.coordinate_y[0] + fts_gesture_data.coordinate_y[gesture->point_num -1]) / 2;

			gesture_report[13] = 0x00;
		}
		else
		{
			gesture_report[1] = fts_gesture_data.coordinate_x[0];
			gesture_report[2] = fts_gesture_data.coordinate_y[0];

			gesture_report[3] = fts_gesture_data.coordinate_x[gesture->point_num -1];
			gesture_report[4] = fts_gesture_data.coordinate_y[gesture->point_num -1];

			gesture_report[5] = x_max;
			gesture_report[6] = (fts_gesture_data.coordinate_y[0] + fts_gesture_data.coordinate_y[gesture->point_num -1]) / 2;

			gesture_report[13] = 0x01;
		}
        break;
    case GESTURE_LEFT:
        gesture_report[0] = 0x05;
		if (fts_gesture_data.coordinate_y[0] < fts_gesture_data.coordinate_y[gesture->point_num -1])
		{
			gesture_report[1] = fts_gesture_data.coordinate_x[0];
			gesture_report[2] = fts_gesture_data.coordinate_y[0];

			gesture_report[3] = fts_gesture_data.coordinate_x[gesture->point_num -1];
			gesture_report[4] = fts_gesture_data.coordinate_y[gesture->point_num -1];

			gesture_report[5] = x_min;
			gesture_report[6] = (fts_gesture_data.coordinate_y[0] + fts_gesture_data.coordinate_y[gesture->point_num -1]) / 2;

			gesture_report[13] = 0x00;
		}
		else
		{
			gesture_report[1] = fts_gesture_data.coordinate_x[0];
			gesture_report[2] = fts_gesture_data.coordinate_y[0];

			gesture_report[3] = fts_gesture_data.coordinate_x[gesture->point_num -1];
			gesture_report[4] = fts_gesture_data.coordinate_y[gesture->point_num -1];

			gesture_report[5] = x_min;
			gesture_report[6] = (fts_gesture_data.coordinate_y[0] + fts_gesture_data.coordinate_y[gesture->point_num -1]) / 2;

			gesture_report[13] = 0x01;
		}
        break;
    case GESTURE_O:
        gesture_report[0] = 0x06;
		gesture_report[1] = fts_gesture_data.coordinate_x[0];
		gesture_report[2] = fts_gesture_data.coordinate_y[0];

		gesture_report[3] = x_max;
		gesture_report[4] = (y_min + y_max) / 2;

		gesture_report[5] = (x_min + x_max) / 2;
		gesture_report[6] = y_min;

		gesture_report[7] = x_min;
		gesture_report[8] = (y_min + y_max) / 2;

		gesture_report[9] = (x_min + x_max) / 2;
		gesture_report[10] = y_max;

		gesture_report[13] = 0x01;
        break;
	 case GESTURE_O_1:
        gesture_report[0] = 0x06;
		gesture_report[1] = fts_gesture_data.coordinate_x[0];
		gesture_report[2] = fts_gesture_data.coordinate_y[0];

		gesture_report[3] = x_max;
		gesture_report[4] = (y_min + y_max) / 2;

		gesture_report[5] = (x_min + x_max) / 2;
		gesture_report[6] = y_min;

		gesture_report[7] = x_min;
		gesture_report[8] = (y_min + y_max) / 2;

		gesture_report[9] = (x_min + x_max) / 2;
		gesture_report[10] = y_max;

		gesture_report[13] = 0x00;
        break;
    case GESTURE_W:
        gesture_report[0] = 0x0D;
		if (fts_gesture_data.coordinate_x[0] < fts_gesture_data.coordinate_x[gesture->point_num -1])
		{
			gesture_report[1] = fts_gesture_data.coordinate_x[0];
			gesture_report[2] = fts_gesture_data.coordinate_y[0];

			gesture_report[3] = fts_gesture_data.coordinate_x[gesture->point_num -1];
			gesture_report[4] = fts_gesture_data.coordinate_y[0];

			gesture_report[5] = (3 * fts_gesture_data.coordinate_x[0] + fts_gesture_data.coordinate_x[gesture->point_num -1]) / 4;
			gesture_report[6] = y_max;

			gesture_report[7] = (fts_gesture_data.coordinate_x[0] + fts_gesture_data.coordinate_x[gesture->point_num -1]) / 2;;
			gesture_report[8] = (fts_gesture_data.coordinate_y[0] +y_max) / 2;

			gesture_report[9] = (3 * fts_gesture_data.coordinate_x[gesture->point_num -1] + fts_gesture_data.coordinate_x[0]) / 4;
			gesture_report[10] = y_max;

			gesture_report[13] = 0x00;
		}
		else
		{
			gesture_report[1] = fts_gesture_data.coordinate_x[0];
			gesture_report[2] = fts_gesture_data.coordinate_y[0];

			gesture_report[3] = fts_gesture_data.coordinate_x[gesture->point_num -1];
			gesture_report[4] = fts_gesture_data.coordinate_y[0];

			gesture_report[5] = (3 * fts_gesture_data.coordinate_x[gesture->point_num -1] + fts_gesture_data.coordinate_x[0]) / 4;
			gesture_report[6] = y_max;

			gesture_report[7] = (fts_gesture_data.coordinate_x[0] + fts_gesture_data.coordinate_x[gesture->point_num -1]) / 2;;
			gesture_report[8] = (fts_gesture_data.coordinate_y[0] +y_max) / 2;

			gesture_report[9] = (3 * fts_gesture_data.coordinate_x[0] + fts_gesture_data.coordinate_x[gesture->point_num -1]) / 4;
			gesture_report[10] = y_max;

			gesture_report[13] = 0x01;
		}
        break;
    case GESTURE_M:
        gesture_report[0] = 0x0C;
		if (fts_gesture_data.coordinate_x[0] < fts_gesture_data.coordinate_x[gesture->point_num -1])
		{
			gesture_report[1] = fts_gesture_data.coordinate_x[0];
			gesture_report[2] = fts_gesture_data.coordinate_y[0];

			gesture_report[3] = fts_gesture_data.coordinate_x[gesture->point_num -1];
			gesture_report[4] = fts_gesture_data.coordinate_y[0];

			gesture_report[5] = (3 * fts_gesture_data.coordinate_x[0] + fts_gesture_data.coordinate_x[gesture->point_num -1]) / 4;
			gesture_report[6] = y_min;

			gesture_report[7] = (fts_gesture_data.coordinate_x[0] + fts_gesture_data.coordinate_x[gesture->point_num -1]) / 2;;
			gesture_report[8] = (fts_gesture_data.coordinate_y[0] +y_min) / 2;

			gesture_report[9] = (3 * fts_gesture_data.coordinate_x[gesture->point_num -1] + fts_gesture_data.coordinate_x[0]) / 4;
			gesture_report[10] = y_min;
			
			gesture_report[13] = 0x00;
		}
		else
		{
			gesture_report[1] = fts_gesture_data.coordinate_x[0];
			gesture_report[2] = fts_gesture_data.coordinate_y[0];

			gesture_report[3] = fts_gesture_data.coordinate_x[gesture->point_num -1];
			gesture_report[4] = fts_gesture_data.coordinate_y[0];

			gesture_report[5] = (3 * fts_gesture_data.coordinate_x[gesture->point_num -1] + fts_gesture_data.coordinate_x[0]) / 4;
			gesture_report[6] = y_min;

			gesture_report[7] = (fts_gesture_data.coordinate_x[0] + fts_gesture_data.coordinate_x[gesture->point_num -1]) / 2;;
			gesture_report[8] = (fts_gesture_data.coordinate_y[0] +y_min) / 2;

			gesture_report[9] = (3 * fts_gesture_data.coordinate_x[0] + fts_gesture_data.coordinate_x[gesture->point_num -1]) / 4;
			gesture_report[10] = y_min;

			gesture_report[13] = 0x01;
		}
        break;		
    default:
        gesture_report[0] = 0x00;
		gesture_report[1] = 0x00;
		gesture_report[2] = 0x00;
		gesture_report[3] = 0x00;
		gesture_report[4] = 0x00;
		gesture_report[5] = 0x00;
		gesture_report[6] = 0x00;
		gesture_report[7] = 0x00;
		gesture_report[8] = 0x00;
		gesture_report[9] = 0x00;
		gesture_report[10] = 0x00;
		gesture_report[11] = 0x00;
		gesture_report[12] = 0x00;
		gesture_report[13] = 0x00;
        break;
    }
    /* report event key */
    if (gesture_report[0] != 0x00) {
        FTS_DEBUG("Gesture Code=%d", gesture_report[0]);
		input_report_key(input_dev, KEY_F4, 1);
		input_sync(input_dev);
		input_report_key(input_dev, KEY_F4, 0);
		input_sync(input_dev);
    }
}

/*****************************************************************************
* Name: fts_gesture_readdata
* Brief: Read information about gesture: enable flag/gesture points..., if ges-
*        ture enable, save gesture points' information, and report to OS.
*        It will be called this function every intrrupt when FTS_GESTURE_EN = 1
*
*        gesture data length: 1(enable) + 1(reserve) + 2(header) + 6 * 4
* Input: ts_data - global struct data
*        data    - gesture data buffer if non-flash, else NULL
* Output:
* Return: 0 - read gesture data successfully, the report data is gesture data
*         1 - tp not in suspend/gesture not enable in TP FW
*         -Exx - error
*****************************************************************************/
int fts_gesture_readdata(struct fts_ts_data *ts_data, u8 *data)
{
    int ret = 0;
    int i = 0;
    int index = 0;
    u8 buf[FTS_GESTURE_DATA_LEN] = { 0 };
    struct input_dev *input_dev = ts_data->input_dev;
    struct fts_gesture_st *gesture = &fts_gesture_data;

    if (!ts_data->suspended || (DISABLE == gesture->mode)) {
        return 1;
    }


    ret = fts_read_reg(FTS_REG_GESTURE_EN, &buf[0]);
    if ((ret < 0) || (buf[0] != ENABLE)) {
        FTS_DEBUG("gesture not enable in fw, don't process gesture");
        return 1;
    }

    buf[2] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
    ret = fts_read(&buf[2], 1, &buf[2], FTS_GESTURE_DATA_LEN - 2);
    if (ret < 0) {
        FTS_ERROR("read gesture header data fail");
        return ret;
    }

	gesture->gesture_id = buf[2];
    gesture->point_num = buf[3];

	if (gesture->point_num > FTS_GESTURE_POINTS_MAX)
		gesture->point_num = FTS_GESTURE_POINTS_MAX;

    FTS_DEBUG("gesture_id=%d, point_num=%d",
              gesture->gesture_id, gesture->point_num);

    /* init variable before read gesture point */
    memset(gesture->coordinate_x, 0, gesture->point_num * sizeof(u16));
    memset(gesture->coordinate_y, 0, gesture->point_num * sizeof(u16));

    /* save point data,max:6 */
    for (i = 0; i < gesture->point_num; i++) {
        index = 4 * i + 4;
        gesture->coordinate_x[i] = (u16)(((buf[0 + index] & 0x0F) << 8)
                                         + buf[1 + index]);
        gesture->coordinate_y[i] = (u16)(((buf[2 + index] & 0x0F) << 8)
                                         + buf[3 + index]);

		if (gesture->coordinate_x[i] > x_max)
			x_max = gesture->coordinate_x[i];

		if (gesture->coordinate_x[i] < x_min)
			x_min = gesture->coordinate_x[i];

		if (gesture->coordinate_y[i] > y_max)
			y_max = gesture->coordinate_y[i];

		if (gesture->coordinate_y[i] < y_min)
			y_min = gesture->coordinate_y[i];

	//	FTS_DEBUG("%d, %d\n", gesture->coordinate_x[i], gesture->coordinate_y[i]);
    }

//	FTS_DEBUG("x_min = %d, x_max = %d\n", x_min, x_max);
//	FTS_DEBUG("y_min = %d, y_max = %d\n", y_min, y_max);

    /* report gesture to OS */
    fts_gesture_report(input_dev, gesture->gesture_id);

	x_max = 0x00;
	y_max = 0x00;
	x_min = 0xffff;
	y_min = 0xffff;

    return 0;
}

void fts_gesture_recovery(struct fts_ts_data *ts_data)
{
    if ((ENABLE == fts_gesture_data.mode) && (ENABLE == fts_gesture_data.active)) {
        FTS_DEBUG("gesture recovery...");
        fts_write_reg(0xD1, 0xFF);
        fts_write_reg(0xD2, 0xFF);
        fts_write_reg(0xD5, 0xFF);
        fts_write_reg(0xD6, 0xFF);
        fts_write_reg(0xD7, 0xFF);
        fts_write_reg(0xD8, 0xFF);
        fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
    }
}

int fts_gesture_suspend(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int i = 0;
    u8 state = 0xFF;

    FTS_INFO("gesture suspend...");
	FTS_INFO("double_tap_enable = %d", double_tap_enable);
    /* gesture not enable, return immediately */
    if (fts_gesture_data.mode == DISABLE) {
        FTS_DEBUG("gesture is disabled");
        return -EINVAL;
    }

	if (double_tap_enable == 0) {
        FTS_DEBUG("gesture is disabled");
        return -EINVAL;
    }
	
    for (i = 0; i < 5; i++) {
        fts_write_reg(0xD1, 0xFF);
        fts_write_reg(0xD2, 0xFF);
        fts_write_reg(0xD5, 0xFF);
        fts_write_reg(0xD6, 0xFF);
        fts_write_reg(0xD7, 0xFF);
        fts_write_reg(0xD8, 0xFF);
        fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
        msleep(1);
        fts_read_reg(FTS_REG_GESTURE_EN, &state);
        if (state == ENABLE)
            break;
    }

    if (i >= 5) {
        FTS_ERROR("Enter into gesture(suspend) fail");
        fts_gesture_data.active = DISABLE;
        return -EIO;
    }

    ret = enable_irq_wake(ts_data->irq);
    if (ret) {
        FTS_DEBUG("enable_irq_wake(irq:%d) fail", ts_data->irq);
    }

    fts_gesture_data.active = ENABLE;
    FTS_INFO("Enter into gesture(suspend) successfully!");
    return 0;
}

int fts_gesture_resume(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int i = 0;
    u8 state = 0xFF;

    FTS_INFO("gesture resume...");
    /* gesture not enable, return immediately */
    if (fts_gesture_data.mode == DISABLE) {
        FTS_DEBUG("gesture is disabled");
        return -EINVAL;
    }

    if (fts_gesture_data.active == DISABLE) {
        FTS_DEBUG("gesture active is disable, return immediately");
        return -EINVAL;
    }

	if (double_tap_enable == 0) {
        FTS_DEBUG("gesture is disabled");
        return -EINVAL;
    }
	
    fts_gesture_data.active = DISABLE;
    for (i = 0; i < 5; i++) {
        fts_write_reg(FTS_REG_GESTURE_EN, DISABLE);
        msleep(1);
        fts_read_reg(FTS_REG_GESTURE_EN, &state);
        if (state == DISABLE)
            break;
    }

    if (i >= 5) {
        FTS_ERROR("exit gesture(resume) fail");
        return -EIO;
    }

    ret = disable_irq_wake(ts_data->irq);
    if (ret) {
        FTS_DEBUG("disable_irq_wake(irq:%d) fail", ts_data->irq);
    }

    FTS_INFO("resume from gesture successfully");
    return 0;
}

int fts_gesture_init(struct fts_ts_data *ts_data)
{
	struct input_dev *input_dev = ts_data->input_dev;
	  
    FTS_FUNC_ENTER();
    
	input_set_capability(input_dev, EV_KEY, KEY_F4);
	
    fts_create_gesture_sysfs(ts_data->dev);

    memset(&fts_gesture_data, 0, sizeof(struct fts_gesture_st));
    fts_gesture_data.mode = ENABLE;
    fts_gesture_data.active = DISABLE;

    FTS_FUNC_EXIT();
    return 0;
}

int fts_gesture_exit(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    sysfs_remove_group(&ts_data->dev->kobj, &fts_gesture_group);
    FTS_FUNC_EXIT();
    return 0;
}
#endif
