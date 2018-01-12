
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/kdev_t.h>
#include <linux/time.h>

#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_gpio.h>
#include <linux/earlysuspend.h>
#include <mtk_kpd.h>
#include <mach/upmu_common.h>
#include <mach/mt_boot.h>

#include "mhall_bu52051nvx.h"

/*----------------------------------------------------------------------*/
#define MHALL_DEBUG
#define MHALL_TAG	"[MHALL]"
#if defined(MHALL_DEBUG)
#define MHALL_FUN(f)			printk(KERN_INFO MHALL_TAG"%s\n", __FUNCTION__)
#define MHALL_ERR(fmt, args...)	printk(KERN_ERR  MHALL_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MHALL_LOG(fmt, args...)	printk(KERN_INFO MHALL_TAG"%s(%d):" fmt, __FUNCTION__, __LINE__, ##args)
#define MHALL_DBG(fmt, args...)	printk(KERN_INFO fmt, ##args)
#else
#define MHALL_FUN(f)
#define MHALL_ERR(fmt, args...)
#define MHALL_LOG(fmt, args...)
#define MHALL_DBG(fmt, args...)
#endif
/*----------------------------------------------------------------------*/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

/*----------------------------------------------------------------------
static variable defination
----------------------------------------------------------------------*/
static DEFINE_MUTEX(mhall_eint_mutex);

static struct input_dev *mhall_input_dev = NULL;
static struct work_struct mhall_eint_work;
static struct platform_driver mhall_driver;
static struct early_suspend  mhall_early_drv;
static int mhall_resume_flag = 1;

#define EINT_PIN_COVER        (1)
#define EINT_PIN_LEAVE        (0)

int cur_mhall_eint_state = EINT_PIN_LEAVE;
static int mhall_is_enable = 1; //0-> disable;  1->enable;  defaule enable;

/****************************************************************/
/*******static function defination                             **/
/****************************************************************/
static void mhall_eint_work_callback(struct work_struct *work)
{	
	mutex_lock(&mhall_eint_mutex);
	/* for SW_LID, 0: lid open => leave, 1: lid shut => cover */
	input_report_switch(mhall_input_dev, SW_LID, cur_mhall_eint_state);
	input_sync(mhall_input_dev);
	mutex_unlock(&mhall_eint_mutex);
	
	MHALL_LOG("report mhall state = %s\n", cur_mhall_eint_state ? "cover" : "leave");
}

void mhall_eint_func(void)
{
	MHALL_FUN();

	mt_eint_mask(CUST_EINT_MHALL_NUM);
		
	if(cur_mhall_eint_state == EINT_PIN_COVER){
		if (CUST_EINT_MHALL_TYPE == CUST_EINTF_TRIGGER_HIGH){
			mt_eint_set_polarity(CUST_EINT_MHALL_NUM, 1);
		 }else{
			mt_eint_set_polarity(CUST_EINT_MHALL_NUM, 0);
		 }
		 cur_mhall_eint_state = EINT_PIN_LEAVE;

	}else{
		if(CUST_EINT_MHALL_TYPE == CUST_EINTF_TRIGGER_HIGH){
			mt_eint_set_polarity(CUST_EINT_MHALL_NUM, 0);
		}else{
			mt_eint_set_polarity(CUST_EINT_MHALL_NUM, 1);
		}
		cur_mhall_eint_state = EINT_PIN_COVER; 
	}

    mt_eint_unmask(CUST_EINT_MHALL_NUM);

	if(NORMAL_BOOT == get_boot_mode())
	{
		if(mhall_is_enable)
		{
			schedule_work(&mhall_eint_work);
		}
	}
}

static int mhall_setup_eint(void)
{
	/*configure to GPIO function, external interrupt*/
	mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE); //To disable GPIO PULL.

	mt_eint_set_hw_debounce(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_TYPE, mhall_eint_func, 0);
	
	mt_eint_unmask(CUST_EINT_MHALL_NUM);  

    return 0;
}

/*----------------------------------------------------------------------------*/
static void mhall_early_suspend(struct early_suspend *h) 
{   
	MHALL_FUN();
	mhall_resume_flag = 0;
}
static void mhall_late_resume(struct early_suspend *h)
{   
	MHALL_FUN();
	mhall_resume_flag = 1;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_mhall_state(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", cur_mhall_eint_state);        
}

static ssize_t show_mhall_enable(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", mhall_is_enable);        
}
static ssize_t store_mhall_enable(struct device_driver *ddri, const char *buf, size_t count)
{
	int tmp;
	if(1 == sscanf(buf, "%d", &tmp))
	{
		mhall_is_enable = tmp;
	}
	return count; 
}

static DRIVER_ATTR(mhall_state, S_IWUSR | S_IRUGO, show_mhall_state, NULL);
static DRIVER_ATTR(mhall_enable, S_IWUSR | S_IRUGO, show_mhall_enable, store_mhall_enable);

static struct driver_attribute *mhall_attr_list[] = {
	&driver_attr_mhall_state,
	&driver_attr_mhall_enable,
};

/*----------------------------------------------------------------------------*/
static int mhall_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(mhall_attr_list)/sizeof(mhall_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, mhall_attr_list[idx]))
		{            
			MHALL_LOG("driver_create_file (%s) = %d\n", mhall_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int mhall_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(mhall_attr_list)/sizeof(mhall_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for(idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, mhall_attr_list[idx]);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/

static int mhall_probe(struct platform_device *dev)	
{
	int err = 0;
	
	MHALL_FUN();

	mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE);
	msleep(10);
	
	if(mt_get_gpio_in(GPIO_MHALL_EINT_PIN) == 0)
	{
		cur_mhall_eint_state = EINT_PIN_COVER;
	}
	else{
		cur_mhall_eint_state = EINT_PIN_LEAVE;
	}
	
	/*** creat mhall_input_dev ***/
	mhall_input_dev = input_allocate_device();
	if (!mhall_input_dev)
    {
	    MHALL_ERR(" failed to allocate input device\n");
    }

	mhall_input_dev->name = "mhall";
	__set_bit(EV_SW,  mhall_input_dev->evbit);
	__set_bit(SW_LID, mhall_input_dev->swbit);
	
	err = input_register_device(mhall_input_dev);
	if(err){
		MHALL_ERR("failed to register mhall input device\n");
        input_free_device(mhall_input_dev);
	}

	INIT_WORK(&mhall_eint_work, mhall_eint_work_callback);
	mhall_setup_eint();

	/* Create sysfs files. */
	if(err = mhall_create_attr(&mhall_driver.driver))
	{
		MHALL_LOG("create attr file fail\n");
	}
	
	mhall_early_drv.level	= EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1,
	mhall_early_drv.suspend	= mhall_early_suspend,
	mhall_early_drv.resume	= mhall_late_resume,	  
	register_early_suspend(&mhall_early_drv);

	MHALL_LOG("%s: OK\n", __func__);
	return 0;
}

static int mhall_remove(struct platform_device *dev)	
{	
	int err = 0;

	input_unregister_device(mhall_input_dev);
	cancel_work_sync(&mhall_eint_work);
	/* Destroy sysfs files. */
	if(err = mhall_delete_attr(&mhall_driver.driver))
	{
		MHALL_LOG("delete attr file fail\n");
	}
	
	return 0;
}

static struct platform_driver mhall_driver = {
	.probe		= mhall_probe,	
	.remove   	= mhall_remove,
	.driver     = {
		.name   = "mhall",
	},
};

static int mhall_init(void)
{
	platform_driver_register(&mhall_driver);

    return 0;
}

static void  mhall_exit(void)
{
	platform_driver_unregister(&mhall_driver);
}

module_init(mhall_init);
module_exit(mhall_exit);

MODULE_DESCRIPTION("Wingtech Mhall driver");
MODULE_AUTHOR("wing <xxx@wingtech.com>");
MODULE_LICENSE("GPL");

