/* BMA150 motion sensor driver
 *
 *
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
//#include <mach/mt_gpio.h>

//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <cust_acc.h>
#include <cust_gpio_usage.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>
/*----------------------------------------------------------------------------*/
#define DEBUG 0
/*----------------------------------------------------------------------------*/
#define BF1036_AXIS_X          0
#define BF1036_AXIS_Y          1
#define BF1036_AXIS_Z          2
#define BF1036_AXES_NUM        3

#define BF1036_BUFSIZE		   256
/*----------------------------------------------------------------------------*/
static struct platform_driver bf1036_driver;
static GSENSOR_VECTOR3D gsensor_gain;
static bool sensor_suspend = false;
/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[2Dsensor] "
#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR  GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_INFO GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
static void bf1036_power(struct acc_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("bf1036power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("bf1036 ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "BF1036"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "BF1036"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
	power_on = on;    
}
/*----------------------------------------------------------------------------*/
static int BF1036_ReadChipInfo( char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	sprintf(buf, "BF1036 Chip");
	return 0;
}

static int count = 0; 
/*----------------------------------------------------------------------------*/

static void bf1036_set_pin_gpio(int Pin)
{
	mt_set_gpio_mode(Pin,GPIO_MODE_00);
	mt_set_gpio_dir(Pin,GPIO_DIR_OUT);
	mt_set_gpio_out(Pin,GPIO_OUT_ZERO);	
	mt_set_gpio_dir(Pin,GPIO_DIR_IN);
	mdelay(5);
}

static void bf1036_get_gpio(int Pin, int *Data)
{
	bf1036_set_pin_gpio(Pin);
	*Data =mt_get_gpio_in(Pin);
}
/*----------------------------------------------------------------------------*/
static void bf1036_read_gpio(int Pin, int *Data)
{
	mt_set_gpio_mode(Pin,GPIO_MODE_00);
	mt_set_gpio_dir(Pin,GPIO_DIR_IN);
	mdelay(5);
	*Data =mt_get_gpio_in(Pin);
}
static int  bf1036_ReadSensorData(char *buff)
{
	int acc_raw[BF1036_AXES_NUM];
	int acc_data[BF1036_AXES_NUM];
	int res = 0;

	if(sensor_suspend)
	{
		return 0;
	}
	
	bf1036_read_gpio(GPIO_GSE_1_EINT_PIN, &acc_raw[0]);
	bf1036_read_gpio(GPIO_GSE_2_EINT_PIN, &acc_raw[1]);
	acc_raw[2] = 0;
	
	if((acc_raw[0]==1)&&(acc_raw[1]==1))
	{
		acc_data[0] =  0;
		acc_data[1] =  1;
	}
	else if((acc_raw[0]==1)&&(acc_raw[1]==0))
	{
		acc_data[0] = -1;
		acc_data[1] =  0;
	}
	else if((acc_raw[0]==0)&&(acc_raw[1]==1))
	{
		acc_data[0] =  1;
		acc_data[1] =  0;
	}
	else if((acc_raw[0]==0)&&(acc_raw[1]==0))
	{
		acc_data[0] =  0;
		acc_data[1] = -1;
	}
		acc_data[2] =  0;

	acc_data[BF1036_AXIS_X] = acc_data[BF1036_AXIS_X] * GRAVITY_EARTH_1000;
	acc_data[BF1036_AXIS_Y] = acc_data[BF1036_AXIS_Y] * GRAVITY_EARTH_1000;
	acc_data[BF1036_AXIS_Z] = acc_data[BF1036_AXIS_Z] * GRAVITY_EARTH_1000;	

	count++;
	if(count >= 300){
		count = 0;
	}
	res = count % 3;
	switch(res)
	{
		case 0:
			acc_data[BF1036_AXIS_X] += 50;
			acc_data[BF1036_AXIS_Y] += 50;
			acc_data[BF1036_AXIS_Z] += 50;
			break;
			
		case 1:
			break;
		
		case 2:
			acc_data[BF1036_AXIS_X] -= 50;
			acc_data[BF1036_AXIS_Y] -= 50;
			acc_data[BF1036_AXIS_Z] -= 50;
			break;
	}
	
	sprintf(buff, "%04x %04x %04x", acc_data[BF1036_AXIS_X], acc_data[BF1036_AXIS_Y], acc_data[BF1036_AXIS_Z]);

	return 0;

}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[BF1036_BUFSIZE];
	BF1036_ReadChipInfo( strbuf, BF1036_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{	
	char strbuf[BF1036_BUFSIZE];
	
	bf1036_ReadSensorData(strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}

/*----------------------------------------------------------------------------*/
static ssize_t show_pinstatus_value(struct device_driver *ddri, char *buf)
{
	int pin_raw[BF1036_AXES_NUM]={-1,-1,-1};
	
	bf1036_get_gpio(GPIO_GSE_1_EINT_PIN, &pin_raw[0]);
	bf1036_get_gpio(GPIO_GSE_2_EINT_PIN, &pin_raw[1]);

	return snprintf(buf, PAGE_SIZE, "%d %d\n",pin_raw[0],pin_raw[1]);
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,   0660, show_chipinfo_value,    NULL);
static DRIVER_ATTR(sensordata, 0660, show_sensordata_value,  NULL);
static DRIVER_ATTR(pinstatus,  0660, show_pinstatus_value,   NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *bf1036_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_pinstatus,
};
/*----------------------------------------------------------------------------*/
static int bf1036_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(bf1036_attr_list)/sizeof(bf1036_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, bf1036_attr_list[idx])))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", bf1036_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int bf1036_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(bf1036_attr_list)/sizeof(bf1036_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, bf1036_attr_list[idx]);
	}
	
	return err;
}

/*----------------------------------------------------------------------------*/
int bf1036_gsensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;	
	int value;
	hwm_sensor_data* gsensor_data;
	char buff[BF1036_BUFSIZE];
	
	switch (command)
	{
		case SENSOR_DELAY:	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{ 
				value = *(int *)buff_in;
				GSE_LOG("enable value=%d\n",value);
				if(value == 1)
				{
					bf1036_set_pin_gpio(GPIO_GSE_1_EINT_PIN);
					bf1036_set_pin_gpio(GPIO_GSE_2_EINT_PIN);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (hwm_sensor_data *)buff_out;
				bf1036_ReadSensorData(buff);					
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
					&gsensor_data->values[1], &gsensor_data->values[2]);				
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gsensor_data->value_divide = 1000;                 
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int bf1036_open(struct inode *inode, struct file *file)
{
	return 0; //nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int bf1036_release(struct inode *inode, struct file *file)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static long bf1036_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)

{
	char strbuf[BF1036_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:		
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			BF1036_ReadChipInfo(strbuf, BF1036_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;
			
		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			bf1036_ReadSensorData(strbuf);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;
			
		case GSENSOR_IOCTL_READ_RAW_DATA:
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;  

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations bf1036_fops = {
	.owner = THIS_MODULE,
	.open = bf1036_open,
	.release = bf1036_release,
	.unlocked_ioctl = bf1036_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice bf1036_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &bf1036_fops,
};

/*----------------------------------------------------------------------------*/
static int bf1036_suspend(struct platform_device *dev, pm_message_t state)
{
    struct acc_hw *hw = get_cust_acc_hw();
	
    GSE_FUN();
    bf1036_power(hw, 0);
    GSE_LOG("bf1036_suspend\n");
	sensor_suspend = true;
	
    return 0;
}

static int bf1036_resume(struct platform_device *dev)
{
	 struct acc_hw *hw = get_cust_acc_hw();
	 
     GSE_FUN();    
     bf1036_power(hw, 1); 
	 GSE_LOG("bf1036_resume\n");
	 sensor_suspend = false;
	 return 0;
}

/*----------------------------------------------------------------------------*/
static int bf1036_gpio_config(void)
{
   //because we donot use EINT to support low power
   // config to GPIO input mode + PD 
    
    //set to GPIO_GSE_1_EINT_PIN
    mt_set_gpio_mode(GPIO_GSE_1_EINT_PIN, GPIO_GSE_1_EINT_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_GSE_1_EINT_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_GSE_1_EINT_PIN, GPIO_OUT_ZERO); 
    mt_set_gpio_dir(GPIO_GSE_1_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_GSE_1_EINT_PIN, GPIO_PULL_DISABLE);
	
    //set to GPIO_GSE_2_EINT_PIN
	mt_set_gpio_mode(GPIO_GSE_2_EINT_PIN, GPIO_GSE_2_EINT_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_GSE_2_EINT_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_GSE_2_EINT_PIN, GPIO_OUT_ZERO); 
    mt_set_gpio_dir(GPIO_GSE_2_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_GSE_2_EINT_PIN, GPIO_PULL_DISABLE);
	
	mdelay(5);
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int bf1036_probe(struct platform_device *pdev) 
{
	struct hwmsen_object sobj;
	struct acc_hw *hw = get_cust_acc_hw();
	int err = 0;

	GSE_FUN();

	bf1036_gpio_config();
	
	bf1036_power(hw, 1);
	  
	if((err = misc_register(&bf1036_device)))
	{
		GSE_ERR("bf1036_device register failed\n");
		return -1;
	}
	if((err = bf1036_create_attr(&bf1036_driver.driver)))
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	sobj.self = NULL;
	sobj.polling = 1;
	sobj.sensor_operate = bf1036_gsensor_operate;
	if((err = hwmsen_attach(ID_ACCELEROMETER, &sobj)))
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}
	{
	    #include <linux/hardware_info.h>
		hardwareinfo_set_prop(HARDWARE_ACCELEROMETER, "bf1036");
	}

	return 0;
	
	exit_create_attr_failed:
		bf1036_delete_attr(&bf1036_driver.driver);
	exit_misc_device_register_failed:
		misc_deregister(&bf1036_device);
	exit_init_failed:
	exit_kfree:
	GSE_ERR("%s: err = %d\n", __func__, err);        
	return err;
	

}
/*----------------------------------------------------------------------------*/
static int bf1036_remove(struct platform_device *pdev)
{
   int err = 0;	
   struct acc_hw *hw = get_cust_acc_hw();

    GSE_FUN();    
    bf1036_power(hw, 0);  

	if((err = hwmsen_detach(ID_ACCELEROMETER)))
	{
		GSE_ERR("hwmsen_detach fail: %d\n", err);
	}
	
	if((err = bf1036_delete_attr(&bf1036_driver.driver)))
	{
		GSE_ERR("bma150_delete_attr fail: %d\n", err);
	}
	
	if((err = misc_deregister(&bf1036_device)))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}
 
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver bf1036_driver = {
	.probe      = bf1036_probe,
	.remove     = bf1036_remove, 
  	.suspend	= bf1036_suspend,
  	.resume	 	= bf1036_resume,
	.driver     = {
		.name  = "gsensor",
		.owner = THIS_MODULE,
	}
};

/*----------------------------------------------------------------------------*/
static int __init bf1036_init(void)
{
	if(platform_driver_register(&bf1036_driver))
	{
		GSE_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit bf1036_exit(void)
{
	platform_driver_unregister(&bf1036_driver);
}
/*----------------------------------------------------------------------------*/
module_init(bf1036_init);
module_exit(bf1036_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("bf1036 driver");
MODULE_AUTHOR("Xiaoli.li@mediatek.com");

