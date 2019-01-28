/*
 * This is a driver for TEF665x familly of tuners from NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Author:Vahid Gharaee <vgharaee@gmail.com>
 * first release
 *  Date: 2019 Jan 7th
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/timekeeping.h>

#include "tef665x.h"

#define DRIVER_NAME "TEF6657"

#define CnvrtAndSwp(a) ((u16)(((u16)(*(a)) << 8 | ((u16)(*(a + 1))))))

typedef struct
{
    struct i2c_client *tef6657_client;
    struct mutex i2c_mutex;
    void (*pwrswch)(struct tef6657_t* dev, u8 pwr);
    void (*modswch)(struct tef6657_t* dev, u8 mode, TunFuncs_t tunfuncs, u16 freq);
} tef6657_t;


static tef6657_t *tef6657;

static int tef6657_write(struct i2c_client *client, u8 * buf, u8 len)
{
	int ret;

	ret = i2c_master_send(client, buf, len);

	return (ret == len) ? true : false;
}

static int tef6657_set_cmd(struct i2c_client *client, u8 module, u8 cmd, int len,...)
{
	u8 buf[CMD_MAXIMUM_LENGTH];
	u8 i;
	u16 temp;
    va_list     vArgs;

    va_start(vArgs, len);

	buf[0] = module;	//FM/AM/APP
	buf[1] = cmd;		//1,2,10,...
	buf[2] = 0x01;	    //always 1

	for(i = 3; i < len; i++)
	{
		temp = va_arg(vArgs,int);

		buf[i++] = (u8)(temp >> 8);
		buf[i] = (u8)temp;
	}

	va_end(vArgs);

	return tef6657_write(client, buf, len);
}

/*
module 64 APPL
cmd 1 Set_OperationMode mode

index
1 mode
	[ 15:0 ]
	device operation mode
	0 = normal operation
	1 = radio stand by mode (low-power mode without radio functionality)
	(default)
*/
static int tef6657_set_op_mode(struct i2c_client *client, u16 mode)
{
	return tef6657_set_cmd(client, MODULE_APPL, SET_OPERATION_MODE,
			5, mode);
}


/*
module 32 / 33 FM / AM
cmd 1 Tune_To mode, frequency

index
1 mode
	[ 15:0 ]
	tuning actions
	0 = no action (radio mode does not change as function of module band)
	1 = Preset Tune to new program with short mute time
	2 = Search Tune to new program and stay muted
	FM 3 = AF-Update Tune to alternative frequency, store quality
	and tune back with inaudible mute
	4 = Jump Tune to alternative frequency with short
	inaudible mute
	5 = Check Tune to alternative frequency and stay
	muted
	AM 3  5 = reserved
	6 = reserved
	7 = End Release the mute of a Search or Check action
	(frequency is not required and ignored)
2 frequency
[ 15:0 ]
	tuning frequency
	FM 6500  10800 65.00  108.00 MHz / 10 kHz step size
	AM LW 144  288 144  288 kHz / 1 kHz step size
	MW 522  1710 522  1710 kHz / 1 kHz step size
	SW 2300  27000 2.3  27 MHz / 1 kHz step size
*/
static int tune_tef6657 (struct i2c_client *client,u8 module, u16 tunfuncs, u16 freq)
{
	return tef6657_set_cmd(client, module,
			CMD_TUNE,
			(tunfuncs <= 5) ? 7 : 5, //length
			tunfuncs, freq);
}


void tuner_power(tef6657_t *dev, u8 pwr)
{
    tef6657_set_op_mode(dev->tef6657_client, pwr);
}


static void tuner_mode(tef6657_t* dev,bool mode, TunFuncs_t tunfuncs, u16 freq)
{
	if(mode) //FM
	{
        tune_tef6657(dev->tef6657_client, MODULE_FM, tunfuncs, freq);
	}
	else     //AM
	{
        tune_tef6657(dev->tef6657_client, MODULE_AM, tunfuncs, freq);
	}
}


static void tef6657dev_init(tef6657_t* dev)
{
    mutex_init(&dev->i2c_mutex);
    dev->pwrswch=tuner_power;
    dev->modswch=tuner_mode;
}


static int tef6657_cmd_read(struct i2c_client *client, u8 reg, u8 *buf, u32 len)
{
	int ret;

	ret = i2c_master_recv(client, buf, len);

	if(ret != len)
	{
		dev_err(&client->dev, "recv command error!! %i\n", ret);
		return false;
	}

	return true;
}

static int tef6657_get_cmd(struct i2c_client *client, u8 module, u8 cmd, u8 *receive, int len)
{
	u8 buf[3];

	buf[0]= module;			//FM/AM/APP
	buf[1]= cmd;		//2,10,...
	buf[2]= 1;	//always 1

	tef6657_write(client, buf, 3);

	return tef6657_cmd_read(client, buf[0], receive, len);
}
/*
module 64 APPL
cmd 128 Get_Operation_Status | status
index
1 status
	Device operation status
	0 = boot state; no command support
	1 = idle state
	2 = active state; radio standby
	3 = active state; FM
	4 = active state; AM
*/
static int tef6657_appl_get_operation_status(struct i2c_client *client,u8 *status)
{
	u8 buf[2];
	int ret;

	ret = tef6657_get_cmd(client,MODULE_APPL,
			GET_OPERATION_STATUS,
			buf,sizeof(buf));

	if(ret == true)
	{
//		dev_notice(&client->dev, "Status buf[0]=%d  buf[1]=%d\n",buf[0],buf[1]);
		*status = CnvrtAndSwp(buf);
	}
    else
    {
		dev_err(&client->dev, "get cmd error, buf[0]=%d buf[1]=%d\n",buf[0],buf[1]);
    }
	return ret;
}

static int appl_get_operation_status(struct i2c_client *client, TEF6657_STATE *status)
{
	u8 data;
	int ret;

    ret = tef6657_appl_get_operation_status(client, &data);
    if(ret == true)
	{
//dev_notice(&client->dev, "appl get operation status 1 data = %d \n",data);
		switch(data)
		{
			case 0:
//				dev_notice(&client->dev, "status = tef6657 boot state \n");
				*status = TEF6657_Boot_state;
				break;
			case 1:
//				dev_notice(&client->dev, "status = eDevTEF665x_Idle_state \n");
				*status = TEF6657_Idle_state;
				break;
			default:
//				dev_notice(&client->dev, "status = eDevTEF665x_Active_state \n");
				*status = TEF6657_Active_state;
				break;
		}
    }
    else
        dev_err(&client->dev, "Couldn't get appl status\n");

    return ret;
}

/*
module 64 APPL
cmd 4 Set_ReferenceClock frequency

index
1 frequency_high
	[ 15:0 ]
	MSB part of the reference clock frequency
	[ 31:16 ]
2 frequency_low
	[ 15:0 ]
	LSB part of the reference clock frequency
	[ 15:0 ]
	frequency [*1 Hz] (default = 9216000)
3 type
	[ 15:0 ]
	clock type
	0 = crystal oscillator operation (default)
	1 = external clock input operation
*/

static int appl_set_reference_clock(struct i2c_client *client,u32 frequency, u16 type)  //0x3d 0x900
{
	return tef6657_set_cmd(client, MODULE_APPL,
            SET_REFERENCE_CLK,
            9,
            (u16)(frequency >> 16),
            (u16)frequency, type);
}

/*
module 64 APPL
cmd 5 Activate mode

index
1 mode
	[ 15:0 ]
	1 = goto active state with operation mode of radio standby
*/

static int appl_activate(struct i2c_client *client)
{
	return tef6657_set_cmd(client, MODULE_APPL, CMD_ACTIVATE, 5, 1);
}

static int tef6657_idle_state(struct i2c_client *client)
{
    int ret;

	TEF6657_STATE status;

    udelay(50);

    ret = appl_get_operation_status(client, &status);

    if((ret == true) && (status != TEF6657_Boot_state))
	{
        if(true != appl_set_reference_clock(client,TEF6657_REF_CLK, TEF6657_IS_CRYSTAL_CLK)) //TEF665x_IS_EXT_CLK
        {
            dev_err(&client->dev, "appl set reference clock NOT succeed!\n");
            return false;
        }
        if(true != appl_activate(client))//Activate : APPL_Activate mode = 1.[ w 40 05 01 0001 ]
        {
            dev_err(&client->dev, "tuner not activated!\n");
            return false;
        }
    }
    else
        dev_err(&client->dev, "appl get operation status NOT succeed or boot is not passed!\n");

	dev_notice(&client->dev, "Tuner activated \n");
    return true;
}

static int chip_power_on(struct i2c_client *client)
{
	int ret;

	TEF6657_STATE status;

    ret = appl_get_operation_status(client, &status); //[ w 40 80 01 [ r 0000 ]

    if(ret == true)
		dev_notice(&client->dev, "TEF6657 Power ON \n");
    else
        dev_err(&client->dev, "TEF6657, Power ON error\n");

	return ret;
}

//Command start will bring the device into? idle state: [ w 14 0001 ]
static int tef6657_start_cmd(struct i2c_client *client)
{

	int err;
	u8  buf[3];

	buf[0] = 0x14;
	buf[1] = 0;
	buf[2] = 1;

	err = i2c_master_send(client, buf, 3);

	if(err != 3)
        dev_err(&client->dev, "tef6657 start error : %d \n", err);

	return err;
}

static int tef6657_boot_state(struct i2c_client *client)
{

	tef6657_patch(client);

	msleep(50);

	tef6657_start_cmd(client);

	msleep(50);

	return 0;
}

static int tef6657_write_array(struct i2c_client *client, const unsigned char *ary)
{
	unsigned char buf[32];
	int len;
	int i;
	int ret;

	len = ary[0];

	for(i = 0; i < len; i++)
		buf[i] = ary[i+1];

	ret = i2c_master_send(client, buf, len);
	if(ret != len)
	{
		dev_err(&client->dev, "sends command tef6657 write array error!!\n");
		return false;
	}

    return true;
}

static int tef6657_patch_load(struct i2c_client *client, const u8 *data, u16 size)
{
	u8 buf[25]; //the size which we break the data into, is 24 bytes.
	int ret, i;

    u16 num = size / 24;
	u16 rem = size % 24;

    buf[0] = 0x1b;

    msleep(10);

    for(i = 0; i < num; i++)
    {
		memcpy(buf + 1, data + (24 * i), 24);

		ret = i2c_master_send(client, buf, 25);

		if(ret != 25)
		{
			dev_err(&client->dev, "send patch error! in %dth pack\n", i);
			return false;
		}
		udelay(50);
	}

    memcpy(buf + 1, data + (num * 24), rem);

    ret = i2c_master_send(client, buf, rem);
		if(ret != rem)
		{
			dev_err(&client->dev, "send patch error! in  last pack\n");
			return false;
		}
		udelay(50);

	return true;
}

int tef6657_patch(struct i2c_client *client)
{
	int err;

    u8 cmd1[] = {3, 0x1C, 0x00, 0x00};
    u8 cmd2[] = {3, 0x1C, 0x00, 0x74};
    u8 cmd3[] = {3, 0x1C, 0x00, 0x75};

	err = tef6657_write_array(client, cmd1);  // w 1C 0000
	if(!err)
	{
		dev_err(&client->dev, "TEF6657, send cmd1 error !\n");
		goto sndfailed;
	}

	err = tef6657_write_array(client, cmd2);  // w 1C 0074
	if(!err)
	{
		dev_err(&client->dev, "TEF6657, send cmd2 error !\n");
		goto sndfailed;
	}

	err = tef6657_patch_load(client, PatchData, sizeof(PatchData));
	if(!err)
	{
		dev_err(&client->dev, "TEF6657, Patch Load failed!\n");
		goto sndfailed;
	}

	err = tef6657_write_array(client, cmd1); // w 1C 0000
	if(!err)
	{
		dev_err(&client->dev, "TEF6657, send cmd1 error!\n");
		goto sndfailed;
	}

	err = tef6657_write_array(client, cmd3); //[ w 1C 0075 ]
	if(!err)
	{
		dev_err(&client->dev, "TEF6657, send cmd3 error\n");
		goto sndfailed;
	}

	err = tef6657_patch_load(client, LutData, sizeof(LutData));
	if(!err)
	{
		dev_err(&client->dev, "TEF6657, second patch load failed!\n");
		goto sndfailed;
	}

	err = tef6657_write_array(client, cmd1); // w 1C 0000
	if(!err)
	{
		dev_err(&client->dev, "TEF6657 send cmd1 error!\n");
        goto sndfailed;
    }

sndfailed:
    return err;
}

int tef665x_load_config(struct i2c_client *client, u8 *count)
{
	int i;
	int ret;
	const u8 *ptr = ConfData;

	for(i = 0; i < sizeof(ConfData); i += (ptr[i] + 1))
	{
		ret = tef6657_write_array(client, ptr + i);
        if(ret != true)
			break;
	}

    *count = i;
	return ret;
}

/*
module 48 AUDIO
cmd 11 Set_Mute mode

index
1 mode
	[ 15:0 ]
	audio mute
	0 = mute disabled
	1 = mute active (default)
*/
static int tef6657_set_mute(struct i2c_client *client, u16 mode)
{
	return tef6657_set_cmd(client, MODULE_AUDIO,
			SET_MUTE,
			5,
			mode);
}

/*
module 48 AUDIO
cmd 10 Set_Volume volume

index
1 volume
	[ 15:0 ] (signed)
	audio volume
	-599  +240 = -60  +24 dB volume
	0 = 0 dB (default)
*/
int tef6657_set_volume(struct i2c_client *client,int16_t volume)
{
	return tef6657_set_cmd(client, MODULE_AUDIO,
			SET_VOLUME,
			5,
			volume*10);
}

static int tef6657_active(struct i2c_client *client)
{
    u8 ret;
	TEF6657_STATE status;

    int m_nstatus;
	fm_quality_t fm_quality;

	ret =  appl_get_operation_status(client,&status);
    if(ret == true)
    {
		if((status != TEF6657_Boot_state) && (status != TEF6657_Idle_state))
		{
            //if(true != tef665x_load_config(client, &j))
              //  dev_err(&client->dev, "Load config failed at %dth \n", j);

			ret = tune_tef6657(client, MODULE_FM, TuningFunc_Preset, 9350);// tune to 89.8MHz

			tef6657_set_mute(client, 1);//mute
			tef6657_set_volume(client, -50);//set to 0db

		}
        else
        {
            if(status == TEF6657_Boot_state)
                dev_notice(&client->dev, "ativation failed, still at boot state \n");
            else
                dev_err(&client->dev, "activation failed, still idle \n");
            return false;
        }
    }
    m_nstatus = fm_signal(client, &fm_quality);
    return true;
}

void tef6657_chip_init(struct i2c_client *client)
{
    chip_power_on(client);
	msleep(50);
	tef6657_boot_state(client);
	msleep(100);
	tef6657_idle_state(client);
	msleep(200);
	tef6657_active(client);
}

static int get_signal_status(struct i2c_client *client, bool channel, signal_status_t *pst)
{
	int ret;
    u8 d[2];
    u16 status;

    ret = tef6657_get_cmd(client, channel,
             GET_SIGNAL_STATUS,
              d, sizeof(d));

    status = CnvrtAndSwp(d);
    if(ret == true)
    {
        pst->stereo = (status & (1 << 15)) >> 15;
        pst->digital_signal = (status & (1 << 14)) >> 15;
    }

    return ret;
}

int fm_get_quality (struct i2c_client *client, fm_quality_t *pfmq )
{
    int ret;
    u8 d[14];

    ret = tef6657_get_cmd(client, MODULE_FM,
             GET_QUALITY_DATA,
              d, sizeof(d));

    if(ret == true)
    {
        pfmq->status = (CnvrtAndSwp(d) & 0x3FFF) / 10;
        pfmq->level = (CnvrtAndSwp(d + 2)) / 10;
        pfmq->ultra_sonic_noise = CnvrtAndSwp(d + 4) / 10;
        pfmq->wideband_am_multi_path = CnvrtAndSwp(d + 6) / 10;
        pfmq->freq_offset = CnvrtAndSwp(d + 8);
        pfmq->if_bandwidth = CnvrtAndSwp(d + 10);
        pfmq->modulation = CnvrtAndSwp(d + 12) / 10;

    }
	return ret;
}

int fm_signal(struct i2c_client *client, fm_quality_t *pfmq)
{
	u8 stereo = false;
	signal_status_t sig_sts;

    get_signal_status(client, 1, &sig_sts);
	stereo = sig_sts.stereo;

	fm_get_quality(client, pfmq);
/*
	printk("FM, FreqOffset=%d\n",pfmq->freq_offset);
	printk("FM, IFBandwidth=%d\n",pfmq->if_bandwidth);
	printk("FM, Level=%d\n",pfmq->level);
	printk("FM, Status=%d\n",pfmq->status);
	printk("FM, Modulation=%d\n",pfmq->modulation);
	printk("FM, UltraSonicNoise=%d\n",pfmq->ultra_sonic_noise);
	printk("FM, WidebandAMMultipath=%d\n",pfmq->wideband_am_multi_path);
	printk("sterio = %d\n", stereo);
*/
	return stereo;
}

int am_get_quality (struct i2c_client *client, am_quality_t *pamq )
{
        int ret;
    u8 d[14];

    ret = tef6657_get_cmd(client, MODULE_AM,
             GET_QUALITY_DATA,
              d, sizeof(d));

    if(ret == true)
    {
        pamq->status =(CnvrtAndSwp(d) & 0x3FFF) / 10;
        pamq->level = CnvrtAndSwp(d + 2) / 10;
        pamq->freq_offset = CnvrtAndSwp(d + 8);
        pamq->if_bandwidth = CnvrtAndSwp(d + 10);
    }
	return ret;
}

int am_signal(struct i2c_client *client, am_quality_t *pamq)
{
	bool stereo = false;
	signal_status_t sig_sts;

    get_signal_status(client, 1, &sig_sts);
	stereo = sig_sts.stereo;

	am_get_quality(client, pamq);

	printk("AM, FreqOffset = %d\n",pamq->freq_offset);
	printk("AM, IFBandwidth = %d\n",pamq->if_bandwidth);
	printk("AM, Level = %d\n",pamq->level);
	printk("AM, Status = %d\n",pamq->status);

	return stereo;
}


int tef6657_get_id(struct i2c_client *client)
{
	u8 buf[6];
	int ret;

	u16 device;
	u16 hw_version;
	u16 sw_version;

	ret = tef6657_get_cmd(client, MODULE_APPL,
			GET_ID,
			buf,sizeof(buf));

	if(ret == true){
		device = CnvrtAndSwp(buf);
		hw_version = CnvrtAndSwp(buf + 2);
		sw_version = CnvrtAndSwp(buf + 4);


		printk("Identification device=%d hw_version=%d sw_version=%d\n", device, hw_version, sw_version);
    } else {
        dev_err(&client->dev, "Identification failed, no HW/SW versions\n");
    }

	return ret;
}

 /*------------------- sysfs interface -------------------- */

static ssize_t show_reset (struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 *msg = "writing 'reset' will reset the device.";

    return sprintf(buf, "%s\n", msg);
}

static ssize_t set_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client* client = container_of(dev, struct i2c_client, dev);






    if(0 != strncmp(buf, "reset", 5))
    {
        printk(KERN_ALERT"value undefined! reset is defined\n");
        return -EINVAL;
    }

    mutex_lock(&tef6657->i2c_mutex);

    tef6657_chip_init(client);

    mutex_unlock(&tef6657->i2c_mutex);

    return count;
}

static DEVICE_ATTR(tef6657_reset, S_IWUSR | S_IRUGO,
        show_reset, set_reset);


static ssize_t show_tune(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 *msg = "writing FM-TunningAction-XXXX or AM-TuningAction-XXXX which XXXX is the frequency will tune the device, and the TunningAction is 1 to 7.";

    return sprintf(buf, "%s\n", msg);
}

static ssize_t set_tune(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client* client = container_of(dev,struct i2c_client, dev);

    int ret;
    long freq;
    int TuningAction = 1;
    u8 amfm;
    u8 str[4];

    if(0 != strncmp(buf, "AM", 2))
    {
        if(0 != strncmp(buf, "FM", 2))
        {
            printk(KERN_ALERT"Band undefined! AM or FM is defined\n");
            return -EINVAL;
        }
        else
        {
            amfm = MODULE_FM;
        }
    }
    else
    {
        amfm = MODULE_AM;
    }

    ret = kstrtoint((buf + 3), 10, &TuningAction);
    if(TuningAction > 7 || TuningAction < 1)
    {
            printk(KERN_ALERT "TuningAction undefined! 1 to 7 is defined\n");
            return -EINVAL;
    }
    strncpy(str, buf + 5, 4);

    ret = kstrtol(str, 10, &freq);

    mutex_lock(&tef6657->i2c_mutex);

    tune_tef6657(client, amfm, (u8)TuningAction, (uint16_t)freq);

    mutex_unlock(&tef6657->i2c_mutex);

    return count;
}

static DEVICE_ATTR(tef6657_tune, S_IWUSR | S_IRUGO,
        show_tune, set_tune);


static ssize_t show_mute(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 *msg = "writing mute/unmute will do mute/unmute the device.";

    return sprintf(buf, "%s\n", msg);
}

static ssize_t set_mute(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client* client = container_of(dev,struct i2c_client, dev);

    int k;

    if(0 != strncmp(buf, "mute", sizeof(buf) - 1))
    {
        if(0 != strncmp(buf, "unmute", sizeof(buf) - 1))
        {
            printk(KERN_ALERT"value unefined! mute or unmute is defined\n");
            return -EINVAL;
        }
        else
        {
            k = 0;
        }
    }
    else
    {
        k = 1;
    }

    mutex_lock(&tef6657->i2c_mutex);

	tef6657_set_mute(client, (int16_t)k);

    mutex_unlock(&tef6657->i2c_mutex);

    return count;
}

static DEVICE_ATTR(tef6657_mute, S_IWUSR | S_IRUGO,
        show_mute, set_mute);

static ssize_t show_volume(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 *msg = "writing -60 to +24 in this file will change the volume.";

    return sprintf(buf, "%s\n", msg);
}

static ssize_t set_volume(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client* client = container_of(dev,struct i2c_client, dev);

    int ret;

    long volume;

	ret = kstrtol(buf, 0, &volume);

    mutex_lock(&tef6657->i2c_mutex);

	tef6657_set_volume(client, (int16_t)volume);

    mutex_unlock(&tef6657->i2c_mutex);

    return count;
}

static DEVICE_ATTR(tef6657_volume, S_IWUSR | S_IRUGO,
		    show_volume, set_volume);

static ssize_t set_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return count;
}

static ssize_t show_status(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client* client = container_of(dev, struct i2c_client, dev);

    u8 data;

    /*
    Device operation status
	0 = boot state; no command support
	1 = idle state
	2 = active state; radio standby
	3 = active state; FM
	4 = active state; AM
    */

    mutex_lock(&tef6657->i2c_mutex);

    tef6657_appl_get_operation_status(client, &data);

    mutex_unlock(&tef6657->i2c_mutex);

    sprintf(buf,"%d\n", data);

	return sizeof(buf);
}

static DEVICE_ATTR(tef6657_status, S_IWUSR | S_IRUGO,
		   show_status, set_status);

static struct attribute *tef6657_attributes[] = {
	&dev_attr_tef6657_status.attr,
	&dev_attr_tef6657_volume.attr,
	&dev_attr_tef6657_mute.attr,
    &dev_attr_tef6657_tune.attr,
//	&dev_attr_tef6657_signal.attr,
//	&dev_attr_tef6657_search.attr,
//	&dev_attr_tef6657_power.attr,
    &dev_attr_tef6657_reset.attr,
//	&dev_attr_tef6657_ad.attr,
	NULL
};

static const struct attribute_group tef6657_attr_group = {
	.attrs = tef6657_attributes,
};

/*--------------------------------*/

static int tef6657_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;

    if(!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    {
        ret = -EIO;
        dev_err(&client->dev, "It did not probe, ret=%i\n",ret);
        goto exit_error;
    }

    tef6657 = (tef6657_t *)kzalloc(sizeof(tef6657_t), GFP_KERNEL);

    if (tef6657 == NULL)
    {
        ret = -EIO;
        dev_err(&client->dev, "tef6657 allocation error, ret=%i\n",ret);
        goto exit_error;
    }

    tef6657dev_init(tef6657);

    tef6657->tef6657_client = client;

    tef6657_chip_init(tef6657->tef6657_client);

	ret = sysfs_create_group(&client->dev.kobj, &tef6657_attr_group);

    return 0;

exit_error:
    return ret;
}

static int tef6657_remove(struct i2c_client *cliet)
{
    //kfree()
    return 0;
}

static const struct of_device_id tef6657_of_match[] = {
    {.compatible = "nxp,tef6657"},
    {},
};
MODULE_DEVICE_TABLE(of, tef6657_of_match);

static const struct i2c_device_id tef6657_id_table[] = {
    {DRIVER_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, tef6657_id_table);

static struct i2c_driver tef6657_dev = {
    .driver = {
        .owner = THIS_MODULE,
        .name = DRIVER_NAME,
        .of_match_table = of_match_ptr(tef6657_of_match),
    },
    .probe = tef6657_probe,
    .remove = tef6657_remove,
    .id_table = tef6657_id_table,
};

static int __init tef6657_init(void)
{
    int ret;
    ret = i2c_add_driver(&tef6657_dev);
    return ret;
}

static void __exit tef6657_exit(void)
{
    i2c_del_driver(&tef6657_dev);
}

module_init(tef6657_init);
module_exit(tef6657_exit);

MODULE_DESCRIPTION("TEF665x Radio Tunner");
MODULE_AUTHOR("Vahid Gharaee");
MODULE_LICENSE("GPL v2");


