/*
 * This file is part of the EMBTOM project
 * Copyright (c) 2018-2019 Thomas Willetal 
 * (https://github.com/tom3333)
 */
/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

/* *******************************************************************
 * Includes
 ********************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/tty.h>

#include <linux/kobject.h> 
#include <linux/sysfs.h>  
#include <linux/fs.h> 
#include <linux/string.h> 
#include <linux/init.h>

/* *******************************************************************
 * Defines
 ********************************************************************/
#define DRIVER_DESC "embtom serial touchscreen driver"
#define TOUCH_PROTO_VERSION 		"VERSION 0.2"
#define TOUCH_CALIBRATION_CONSTANTS 7
#define TOUCH_TMPBUF_LEN			(TOUCH_CALIBRATION_CONSTANTS * sizeof(u_int32_t) + TOUCH_CALIBRATION_CONSTANTS)

// MESSEAGE ENVIRONMENT
#define MESSAGE_MAX_LENGTH 	0x50
#define MESSAGE_SYNC_1 		0xFE
#define MESSAGE_SYNC_2 		0xFB

// TOUCH MOVE
#define TOUCH_MOVE_START 	0x01
#define TOUCH_MOVE_RELEASE 	0x02
#define TOUCH_MOVE_DRAG 	0x03
#define TOUCH_MOVE_LENGTH 	0x04

// TOUCH Commands
#define TOUCH_CMD_VERSION 	0xC0
#define TOUCH_CMD_SET_CALIB 0xC1
#define TOUCH_CMD_GET_CALIB 0xC2

// TOUCH Commands ACK
#define TOUCH_CMD_ACK_OK 	0x01
#define TOUCH_CMD_ACK_FAIL 	0x00

/* *******************************************************************
 * custom _data types (e.g. enumerations, structures, unions)
 * ******************************************************************/
enum touchCmd
{
	TOUCHCMD_getVersion,
	TOUCHCMD_getCalib,
	TOUCHCMD_setCalib
};

enum syncState
{
	SYNCSTATE_NONE,
	SYNCSTATE_BYTE_1,
	SYNCSTATE_BYTE_2,
	SYNCSTATE_CMD_MOVE,
	SYNCSTATE_CMD_CONTROL,
	SYNCSTATE_CMD_CONTROL_ACK_OK,
	SYNCSTATE_CMD_CONTROL_ACK_FAIL,
	SYNCSTATE_CMD_CONTROL_LEN,
	SYNCSTATE_CMD_INV
};

struct embtouch_data
{
	struct input_dev *input;
	char phys[32];
	struct serio *serio;
	struct completion cmdDone;
	int id;
	int idx;
	enum syncState rxSyncState;
	unsigned int rxDataIndex;
	char rxData[MESSAGE_MAX_LENGTH];
	char version[20];
	int versionValid;
	int calibrationData[TOUCH_CALIBRATION_CONSTANTS];
	int calibrationValid;
};

/* *******************************************************************
 * static inline
 * ******************************************************************/
inline static enum syncState evalMessageType(uint8_t _cmd)
{
	switch (_cmd)
	{
	case TOUCH_MOVE_START:
	case TOUCH_MOVE_DRAG:
	case TOUCH_MOVE_RELEASE:
		return SYNCSTATE_CMD_MOVE;

	case TOUCH_CMD_VERSION:
	case TOUCH_CMD_SET_CALIB:
	case TOUCH_CMD_GET_CALIB:
		return SYNCSTATE_CMD_CONTROL;
	}
	return SYNCSTATE_NONE;
}

inline static void processMoveCommands(struct embtouch_data *_embtouch, uint8_t _cmd)
{
	uint16_t x, y;
	x = _embtouch->rxData[1] << 8 | _embtouch->rxData[2];
	y = _embtouch->rxData[3] << 8 | _embtouch->rxData[4];

	switch (_cmd)
	{
		case TOUCH_MOVE_START:
        {
			input_report_abs(_embtouch->input, ABS_X, x);
			input_report_abs(_embtouch->input, ABS_Y, y);
			input_report_key(_embtouch->input, BTN_TOUCH, 1);
			input_sync(_embtouch->input);	
            break;
        }
        case TOUCH_MOVE_DRAG:
        {
			input_report_abs(_embtouch->input, ABS_X, x);
			input_report_abs(_embtouch->input, ABS_Y, y);
			input_report_key(_embtouch->input, BTN_TOUCH, 1);
			input_sync(_embtouch->input);
            break;
        }
        case TOUCH_MOVE_RELEASE:
        {
			input_report_key(_embtouch->input, BTN_TOUCH, 0);
			input_sync(_embtouch->input);
            break;
        }		
	}
}

static bool sendTouchCmd(struct embtouch_data *_embtouch, enum touchCmd _touchCmd, unsigned int _waitCompleted)
{
	char command[MESSAGE_MAX_LENGTH] = {0};
	unsigned int i, commandLen;

	switch (_touchCmd)
	{
		case TOUCHCMD_getVersion:
		{
			sprintf(command, "version\n");
			break;
		}
		case TOUCHCMD_getCalib:
		{
			sprintf(command, "get_calib\n");
			break;
		}
		case TOUCHCMD_setCalib:
		{
			sprintf(command, "save_calib: %d %d %d %d %d %d %d\n", _embtouch->calibrationData[0],
					_embtouch->calibrationData[1],
					_embtouch->calibrationData[2],
					_embtouch->calibrationData[3],
					_embtouch->calibrationData[4],
					_embtouch->calibrationData[5],
					_embtouch->calibrationData[6]);
			break;
		}
		default:
		{
			dev_err(&_embtouch->serio->dev, "Invalid command %u",_touchCmd);
			return false;
		}
	}

	if (_waitCompleted) {
		init_completion(&_embtouch->cmdDone);
	}
	
	commandLen = strlen(command);
	for (i = 0; i < commandLen; i++) {
		serio_write(_embtouch->serio, command[i]);
	}
	
	if (_waitCompleted) {
		if(wait_for_completion_timeout(&_embtouch->cmdDone, 2 * HZ) == 0) {
			dev_err(&_embtouch->serio->dev, "Request timed out %u",_touchCmd);
			return false;
		}
	}
	return true;
}

/* *******************************************************************
 * static sysfs device interface functions
 * ******************************************************************/
static ssize_t calibration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	struct serio *serio = to_serio_port(dev);
	struct embtouch_data *embtouch = serio_get_drvdata(serio);

	len = sprintf(buf, "%d %d %d %d %d %d %d", embtouch->calibrationData[0],
							  					 embtouch->calibrationData[1],
												 embtouch->calibrationData[2],
												 embtouch->calibrationData[3],
												 embtouch->calibrationData[4],
												 embtouch->calibrationData[5],
												 embtouch->calibrationData[6]);
	return len;
}

static ssize_t calibration_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int iter;
	char *key, *ips_str, *bufCpy;
	struct serio *serio = to_serio_port(dev);
	struct embtouch_data *embtouch = serio_get_drvdata(serio);

	bufCpy = kzalloc(count, GFP_KERNEL);
	ips_str = bufCpy;
	memcpy(bufCpy,buf,count);
	
	dev_info(&embtouch->serio->dev, "count %u",count);	
	iter=0;
	while ((key = strsep(&ips_str, " "))) 
	{	
		if((key-bufCpy+1) >= count) {
			break;
		}
		if (iter >= TOUCH_CALIBRATION_CONSTANTS) {
			break;
		}
		kstrtoint(key, 10, &embtouch->calibrationData[iter]);
	 	dev_info(&embtouch->serio->dev, "value:%u - %i",iter,embtouch->calibrationData[iter]);	
		iter++;
	}
	kfree(bufCpy);

	if (!sendTouchCmd(embtouch, TOUCHCMD_setCalib, 1)) {
		dev_err(&embtouch->serio->dev, "Failed to store the calibration data persistant");
		return 0;		
	}
    return count;
}

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	struct serio *serio = to_serio_port(dev);
	struct embtouch_data *embtouch = serio_get_drvdata(serio);
	
	len = sprintf(buf, "%s\n", embtouch->version);
	return len;
}

static const struct device_attribute calibration_attr = {
	.attr = { .name = "calibration", .mode = 0660},
	.show = calibration_show,
	.store = calibration_store
};

static const struct device_attribute version_attr = {
	.attr = { .name = "version", .mode = 0660},
	.show = version_show
};

/* *******************************************************************
 * static serio interface functions
 * ******************************************************************/
static irqreturn_t embtouch_interrupt(struct serio *_serio, unsigned char _data, unsigned int flags)
{
	unsigned int i;
	uint8_t touchCommand, frameCmdLen;
	struct embtouch_data *embtouch = serio_get_drvdata(_serio);

	switch (embtouch->rxSyncState)
	{
		case SYNCSTATE_NONE:
		{
			// Wait for start frame step 1
			if (_data == MESSAGE_SYNC_1) {
				embtouch->rxSyncState = SYNCSTATE_BYTE_1;
			}
			break;
		}
		case SYNCSTATE_BYTE_1:
		{
			// Wait for start frame step 2 -> frame valid
			if (_data == MESSAGE_SYNC_2) {
				embtouch->rxSyncState = SYNCSTATE_BYTE_2;
			}
			else {
				embtouch->rxSyncState = SYNCSTATE_NONE;
			}
			break;
		}

		case SYNCSTATE_BYTE_2:
		{
			// Valid frame, evaluate message type 
			//  -----------------------------
			// |  0  |    |    |    |    |    |
			//  ----------------------------- 
			// 0: is message type   
			embtouch->rxSyncState = evalMessageType(_data);
			embtouch->rxDataIndex = 0;
			embtouch->rxData[embtouch->rxDataIndex++] = _data;
			break;
		}

		case SYNCSTATE_CMD_MOVE:
		{
			embtouch->rxData[embtouch->rxDataIndex++] = _data;
			
			// Valid frame, evaluate message type 
			//  -----------------------------
			// |  0  |  1  |  2  |  3  |  4  |
			//  ----------------------------- 
			// 0: is message type   
			// 1-4: Touch coorinates
			if(embtouch->rxDataIndex == (TOUCH_MOVE_LENGTH +1)) {
				touchCommand = embtouch->rxData[0];
				processMoveCommands(embtouch, touchCommand);
				embtouch->rxSyncState = SYNCSTATE_NONE;
			}

			break;
		}

		case SYNCSTATE_CMD_CONTROL:
		{
			// Valid frame, evaluate message type 
			//  -----------------------------
			// |  0  |  1  |    |     |     |
			//  ----------------------------- 
			// 0: is message type   
			// 1: cmd ack
			embtouch->rxData[embtouch->rxDataIndex++] = _data;
			if (_data == TOUCH_CMD_ACK_OK) {
				embtouch->rxSyncState = SYNCSTATE_CMD_CONTROL_ACK_OK;
			}
			else {
				embtouch->rxSyncState = SYNCSTATE_CMD_CONTROL_ACK_FAIL;
			}
			break;
		}

		case SYNCSTATE_CMD_CONTROL_ACK_OK:
		{
			embtouch->rxData[embtouch->rxDataIndex++] = _data;
			touchCommand = embtouch->rxData[0];
			if ((touchCommand == TOUCH_CMD_VERSION) && (_data == '\n'))	{
				memcpy(&embtouch->version[0],&embtouch->rxData[2],embtouch->rxDataIndex-3);
				embtouch->versionValid = 1; 
				complete(&embtouch->cmdDone);
				embtouch->rxSyncState = SYNCSTATE_NONE;
			}
			else if ((touchCommand == TOUCH_CMD_SET_CALIB) && (_data == '\n'))	{
				embtouch->calibrationValid = 0;
				complete(&embtouch->cmdDone);
				embtouch->rxSyncState = SYNCSTATE_NONE;
			}
			else if (touchCommand == TOUCH_CMD_GET_CALIB) {
				embtouch->rxSyncState = SYNCSTATE_CMD_CONTROL_LEN;
			}
			break;
		}

		case SYNCSTATE_CMD_CONTROL_ACK_FAIL: {
			printk("CONTROL_ACK_FAIL");
			embtouch->rxData[embtouch->rxDataIndex++] = _data;
			embtouch->rxSyncState = SYNCSTATE_NONE;
			break;
		}

		case SYNCSTATE_CMD_CONTROL_LEN:
		{
			// Valid frame, evaluate message type 
			//  -----------------------------
			// |  0  |  1  |  2  |     |     |
			//  ----------------------------- 
			// 0: is message type   
			// 1: cmd ack
			// 2: cmd message len
			embtouch->rxData[embtouch->rxDataIndex++] = _data;
			touchCommand = embtouch->rxData[0];
			frameCmdLen = embtouch->rxData[2];
			if ((touchCommand == TOUCH_CMD_GET_CALIB) && (embtouch->rxDataIndex -3 == frameCmdLen))
			{
				for (i = 0; i < TOUCH_CALIBRATION_CONSTANTS; i++)
				{
					embtouch->calibrationData[i] =  embtouch->rxData[i * 4 + 3 +3] << 24 |
													embtouch->rxData[i * 4 + 2 +3] << 16 |
													embtouch->rxData[i * 4 + 1 +3] << 8 |
													embtouch->rxData[i * 4 + 0 +3];
				}
				embtouch->calibrationValid = 1;
				complete(&embtouch->cmdDone);
				embtouch->rxSyncState = SYNCSTATE_NONE;
			}
			break;
		}

		default:
			break;
	}
	return IRQ_HANDLED;
}

static int embtouch_connect(struct serio *_serio, struct serio_driver *drv)
{
	struct input_dev *input_dev;
	struct embtouch_data *embtouch;
	int err;

	embtouch = kzalloc(sizeof(struct embtouch_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if ((embtouch == NULL) || (input_dev == NULL))
	{
		dev_err(&_serio->dev, "Failed to allocate memory");
		err = -ENOMEM;
		goto fail1;
	}

	embtouch->serio = _serio;
	embtouch->id = _serio->id.id;
	embtouch->input = input_dev;
	embtouch->rxSyncState = SYNCSTATE_NONE;
	embtouch->rxDataIndex = 0;
	snprintf(embtouch->phys, sizeof(embtouch->phys), "%s/input0", _serio->phys);

	input_dev->name = "EmbTouch Controller";
	input_dev->phys = embtouch->phys;
	input_dev->id.bustype = BUS_RS232;
	input_dev->id.vendor = SERIO_EMBTOM;
	input_dev->id.product = embtouch->id;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &_serio->dev;
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	err = serio_open(_serio, drv);
	if (err)
	{
		dev_err(&_serio->dev, "Failed to open serial device");
		goto fail2;
	}
	serio_set_drvdata(_serio, embtouch);
	
	// Set input device to default values
	input_set_abs_params(input_dev, ABS_X, 0, 1000, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 1000, 0, 0);

	// request calibration _data from Touch Controller
	if (!sendTouchCmd(embtouch, TOUCHCMD_getVersion, 0)) {
		goto fail3;
	}

	if (!sendTouchCmd(embtouch, TOUCHCMD_getCalib, 1)) {
		goto fail3;
	}

	err = input_register_device(embtouch->input);
	if (err) {
		dev_err(&_serio->dev, "Failed to register touch device");
		goto fail3;
	}

	if( device_create_file(&_serio->dev, &calibration_attr)) {
		dev_err(&_serio->dev, "Failed to create calibration sysfs node");
		goto fail4;
	}
	
	if( device_create_file(&_serio->dev, &version_attr)) {
		dev_err(&_serio->dev, "Failed to create version sysfs node");
		goto fail5;
	}

	dev_info(&_serio->dev, "embtouch %s connected successfully", embtouch->version);
	return 0;

fail5:
	device_remove_file(&_serio->dev, &calibration_attr);
fail4:
	input_unregister_device(embtouch->input);
fail3:
	serio_close(_serio);
fail2:
fail1:
	input_free_device(input_dev);
	kfree(embtouch);
	return err;
}

static void embtouch_disconnect(struct serio *_serio)
{
	struct embtouch_data *embtouch = serio_get_drvdata(_serio);
	input_unregister_device(embtouch->input);
	serio_close(_serio);
	input_free_device(embtouch->input);
	device_remove_file(&_serio->dev, &calibration_attr);
	device_remove_file(&_serio->dev, &version_attr);
	dev_info(&embtouch->serio->dev, "touch driver disconnect successfully");
	kfree(embtouch);
}

static struct serio_device_id embtouch_serio_ids[] = {
	{
		.type  = SERIO_RS232,
		.proto = SERIO_EMBTOM,
		.id    = SERIO_ANY,
		.extra = SERIO_ANY,
	},
	{0}};

static struct serio_driver embtouch_serio_drv = {
	.driver = {
		.name = "embtouch",
	},
	.description = DRIVER_DESC,
	.id_table    = embtouch_serio_ids,
	.interrupt   = embtouch_interrupt,
	.connect     = embtouch_connect,
	.disconnect  = embtouch_disconnect,
};

module_serio_driver(embtouch_serio_drv);


MODULE_AUTHOR("Thomas Willetal");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
