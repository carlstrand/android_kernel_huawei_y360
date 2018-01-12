/* drivers/alsps/ltr558.c - LTR558 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "ltr558.h"
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define LTR558_DEV_NAME   "LTR_558ALS"

//may adjust later
#define PS_DETECTED_THRES	        80           // add for ps Calibration  Close
#define PS_UNDETECTED_THRES	        35           // add for ps Calibration  Far away

/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)                 
/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);	
	
/*----------------------------------------------------------------------------*/

static struct i2c_client *ltr558_i2c_client = NULL;

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr558_i2c_id[] = {{LTR558_DEV_NAME,0},{}};
/*the adapter id & i2c address will be available in customization*/
static struct i2c_board_info __initdata i2c_ltr558={ I2C_BOARD_INFO("LTR_558ALS", (0x4A>>1))};

//static unsigned short ltr558_force[] = {0x00, 0x46, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const ltr558_forces[] = { ltr558_force, NULL };
//static struct i2c_client_address_data ltr558_addr_data = { .forces = ltr558_forces,};
/*----------------------------------------------------------------------------*/
static int ltr558_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int ltr558_i2c_remove(struct i2c_client *client);
static int ltr558_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int ltr558_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int ltr558_i2c_resume(struct i2c_client *client);
static int ltr558_als_enable(int gainrange);
static int ltr558_als_disable(void);


static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;
static int final_lux_val;

static int LTR558_local_init(void);
static int LTR558_remove(void);
static int LTR558_init_flag = -1; // 0<== OK, -1 <==> fail

static struct sensor_init_info LTR558_init_info = {
		.name = LTR558_DEV_NAME,
		.init = LTR558_local_init,
		.uninit = LTR558_remove,
};

/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct ltr558_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/

struct ltr558_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;
    struct mutex lock;
	/*i2c address group*/
    struct ltr558_i2c_addr  addr;

     /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;
    atomic_t    als_suspend;

    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;

static int als_times = 0;
static int als_temp[3] = {0};

static int LTR558_ALS_REPEAT = 19;
static int luxdata_int_prev = 0;
static unsigned int sum_luxdata_prev = 0;

static struct ltr558_priv *ltr558_obj = NULL;
//static struct platform_driver ltr558_alsps_driver;
static int ltr558_get_als_value(struct ltr558_priv *obj, u16 als);
static int ltr558_get_ps_value(struct ltr558_priv *obj, u16 ps);

/*----------------------------------------------------------------------------*/
static struct i2c_driver ltr558_i2c_driver = {	
	.probe      = ltr558_i2c_probe,
	.remove     = ltr558_i2c_remove,
	.detect     = ltr558_i2c_detect,
	.suspend    = ltr558_i2c_suspend,
	.resume     = ltr558_i2c_resume,
	.id_table   = ltr558_i2c_id,
	//.address_data = &ltr558_addr_data,
	.driver = {
		//.owner          = THIS_MODULE,
		.name           = LTR558_DEV_NAME,
	},
};


/* 
 * #########
 * ## I2C ##
 * #########
 */

// I2C Read
static int ltr558_i2c_read_reg(u8 regnum)
{
    u8 buffer[1],reg_value[1];
	int res = 0;
	
	buffer[0]= regnum;
	res = i2c_master_send(ltr558_obj->client, buffer, 0x1);
	if(res <= 0)
	{
		return res;
	}
	res = i2c_master_recv(ltr558_obj->client, reg_value, 0x1);
	if(res <= 0)
	{
		return res;
	}
	return reg_value[0];
}

// I2C Write
static int ltr558_i2c_write_reg(u8 regnum, u8 value)
{
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = regnum;   
	databuf[1] = value;
	res = i2c_master_send(ltr558_obj->client, databuf, 0x2);

	if (res < 0)
		return res;
	else
		return 0;
}

/* 
 * ###############
 * ## PS CONFIG ##
 * ###############
 */
static int ltr558_ps_set_thres()
{
	APS_FUN();

	int res;
	u8 databuf[2];
	
		struct i2c_client *client = ltr558_obj->client;
		struct ltr558_priv *obj = ltr558_obj;
	
	if(1 == ps_cali.valid)
	{
		databuf[0] = LTR558_PS_THRES_LOW_0; 
		databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}
		databuf[0] = LTR558_PS_THRES_LOW_1; 
		databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}
		databuf[0] = LTR558_PS_THRES_UP_0;	
		databuf[1] = (u8)(ps_cali.close & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}
		databuf[0] = LTR558_PS_THRES_UP_1;	
		databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);
		//databuf[1] = (u8)((ps_cali.close & 0x00) >> 8);;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}
	}
	else
	{
		databuf[0] = LTR558_PS_THRES_LOW_0; 
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}
		databuf[0] = LTR558_PS_THRES_LOW_1; 
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low )>> 8) & 0x00FF);
		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}
		databuf[0] = LTR558_PS_THRES_UP_0;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}
		databuf[0] = LTR558_PS_THRES_UP_1;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) >> 8) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}
	
	}

	res = 0;
	return res;
	
	EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	return res;

}


static int ltr558_ps_enable(int gainrange)
{
	struct i2c_client *client = ltr558_obj->client;
	struct ltr558_priv *obj = ltr558_obj;
	u8 databuf[2];	
	int res;
	int error;
	int setgain;
    //APS_LOG("ltr558_ps_enable() ...start!\n");
	switch (gainrange) {
		case PS_RANGE1:
			setgain = MODE_PS_ON_Gain1;
			break;

		case PS_RANGE4:
			setgain = MODE_PS_ON_Gain4;
			break;

		case PS_RANGE8:
			setgain = MODE_PS_ON_Gain8;
			break;

		case PS_RANGE16:
			setgain = MODE_PS_ON_Gain16;
			break;

		default:
			setgain = MODE_PS_ON_Gain1;
			break;
	}

	//APS_LOG("LTR558_PS_CONTR==>%d!\n",setgain);

	error = ltr558_i2c_write_reg(LTR558_PS_N_PULSES, 15); // 2 
	
	if(15 != ltr558_i2c_read_reg(LTR558_PS_N_PULSES)){
			error = ltr558_i2c_write_reg(LTR558_PS_N_PULSES, 15);
			}
	 if(error<0)
	 {
		 APS_LOG("ltr558_ps_enable() error2\n");
		 return error;
	 } 

	error = ltr558_i2c_write_reg(LTR558_PS_CONTR, setgain); 

	if(setgain != ltr558_i2c_read_reg(LTR558_PS_CONTR)){
			error = ltr558_i2c_write_reg(LTR558_PS_CONTR, setgain);
			}
	if(error<0)
	{
	    APS_LOG("ltr558_ps_enable() error1\n");
	    return error;
	}
	
	mdelay(WAKEUP_DELAY);
    
	/* =============== 
	 * ** IMPORTANT **
	 * ===============
	 * Other settings like timing and threshold to be set here, if required.
 	 * Not set and kept as device default for now.
 	 */

	/*error = ltr558_i2c_write_reg(LTR558_PS_LED, 0x63); 
	if(error<0)
    {
        APS_LOG("ltr558_ps_enable() error3...\n");
	    return error;
	}*/
	
	/*for interrup work mode support --*/
	if(0 == obj->hw->polling_mode_ps)
	{		

		ltr558_ps_set_thres();

		databuf[0] = LTR558_PS_MEAS_RATE;	
		databuf[1] = 0x00;    //50ms    default 100ms
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}
		
		databuf[0] = LTR558_INTERRUPT;	
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}

		databuf[0] = LTR558_INTERRUPT_PERSIST;	
		databuf[1] = 0x10;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}
		mt_eint_unmask(CUST_EINT_ALS_NUM);

	}	
	
 	APS_LOG("ltr558_ps_enable ...OK!\n");
 	
	return error;

	EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	return res;
}

// Put PS into Standby mode
static int ltr558_ps_disable(void)
{
	int error;
	struct ltr558_priv *obj = ltr558_obj;
		
	error = ltr558_i2c_write_reg(LTR558_PS_CONTR, MODE_PS_StdBy); 
	
	if(MODE_PS_StdBy != ltr558_i2c_read_reg(LTR558_PS_CONTR)){
			error = ltr558_i2c_write_reg(LTR558_PS_CONTR, MODE_PS_StdBy);
			}
	
	if(error<0)
 	    APS_LOG("ltr558_ps_disable ...ERROR\n");
 	else
        APS_LOG("ltr558_ps_disable ...OK\n");

	if(0 == obj->hw->polling_mode_ps)
	{
		cancel_work_sync(&obj->eint_work);
		mt_eint_mask(CUST_EINT_ALS_NUM);
	}
	
	return error;
}


static int ltr558_ps_read(void)
{
	int psval_lo, psval_hi, psdata;

	psval_lo = ltr558_i2c_read_reg(LTR558_PS_DATA_0);
	if (psval_lo < 0){
	    
	    APS_DBG("psval_lo error\n");
		psdata = psval_lo;
		goto out;
	}
	//APS_DBG("psval_lo = %d\n", psval_lo);	
	psval_hi = ltr558_i2c_read_reg(LTR558_PS_DATA_1);
	if (psval_hi < 0){
	    APS_DBG("psval_hi error\n");
		psdata = psval_hi;
		goto out;
	}
	//APS_DBG("psval_hi = %d\n", psval_hi);	
	
	psdata = ((psval_hi & 7)* 256) + psval_lo;
    //psdata = ((psval_hi&0x7)<<8) + psval_lo;
    //APS_DBG("ps_value = %d\n", psdata);
    
	out:
	final_prox_val = psdata;
	
	return psdata;
}

/* 
 * ################
 * ## ALS CONFIG ##
 * ################
 */

static int ltr558_als_enable(int gainrange)
{
	int error;
	int i;

	if (gainrange == 1){
		error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_ON_Range1);
		
		if(MODE_ALS_ON_Range1 != ltr558_i2c_read_reg(LTR558_ALS_CONTR)){
			error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_ON_Range1);
			}
	}
	else if (gainrange == 2){
		error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_ON_Range2);
		
		if(MODE_ALS_ON_Range2 != ltr558_i2c_read_reg(LTR558_ALS_CONTR)){
			error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_ON_Range2);
			}
		}
	else
		error = -1;

	mdelay(WAKEUP_DELAY);

	/* =============== 
	 * ** IMPORTANT **
	 * ===============
	 * Other settings like timing and threshold to be set here, if required.
 	 * Not set and kept as device default for now.
 	 */
 	if(error<0)
 	    APS_LOG("ltr558_als_enable ...ERROR\n");
 	else
        APS_LOG("ltr558_als_enable ...OK\n");

	als_times = 0;
	for(i=0;i<3;i++){
		als_temp[i] = 0;
	}
	LTR558_ALS_REPEAT = 0;
	sum_luxdata_prev = 0;
	
	return error;
}


// Put ALS into Standby mode
static int ltr558_als_disable(void)
{
	int error;
	error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_StdBy);
	
	if(MODE_ALS_StdBy != ltr558_i2c_read_reg(LTR558_ALS_CONTR)){
			error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_StdBy);
			}
	
	if(error<0)
 	    APS_LOG("ltr558_als_disable ...ERROR\n");
 	else
        APS_LOG("ltr558_als_disable ...OK\n");
	return error;
}

static int ltr558_als_read(int gainrange)
{
	int alsval_ch0_lo, alsval_ch0_hi;
	int alsval_ch1_lo, alsval_ch1_hi;
	unsigned int alsval_ch0, alsval_ch1;
	int luxdata_int;
	int ratio;

	alsval_ch0_lo = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_0);
	alsval_ch0_hi = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_1);
	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;

	alsval_ch1_lo = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_0);
	alsval_ch1_hi = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_1);
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;
	
    //APS_DBG("alsval_ch0=%d,  alsval_ch1=%d\n", alsval_ch0, alsval_ch1);

	if (alsval_ch0==0 && alsval_ch1==0)
		{
			luxdata_int = 0;  //all 0, means dark lux=0, preventing dividing 0
			goto err;
		}

	ratio = (alsval_ch1*100) / (alsval_ch0 + alsval_ch1);

	if(alsval_ch0 < 60000)
	{
		if(ratio < 45)
		{
		  luxdata_int =((17743*alsval_ch0 + 11059*alsval_ch1)*3)/100000;
		}
		else if((ratio >= 45)&&(ratio < 64))
		{
		   luxdata_int = ((37725*alsval_ch0 - 13363*alsval_ch1)*3)/100000;
		}
		else if((ratio >= 64)&&(ratio < 90))
		{
		   luxdata_int = ((1690*alsval_ch0 - 169*alsval_ch1)*3)/10000;
		}
		else
		{
		   luxdata_int = 0;
		}
	}
	if(((alsval_ch0 <5 )&&(alsval_ch1 >= 2000))||(alsval_ch0 >= 60000) ||(alsval_ch1 >= 60000))
	{
		luxdata_int = 65534;
	}

	if (gainrange == ALS_RANGE2_64K)
		luxdata_int = luxdata_int * 150; //100;

err:
	final_lux_val = luxdata_int;

	LTR558_ALS_REPEAT ++;
	if(LTR558_ALS_REPEAT < 19){
		if(LTR558_ALS_REPEAT > 4){
			sum_luxdata_prev += luxdata_int;
		}	
		return luxdata_int_prev;
	}
	LTR558_ALS_REPEAT = 0;
	
	luxdata_int = (sum_luxdata_prev + luxdata_int) / 15;
	
	sum_luxdata_prev = 0;
		
	//------------------------------------
		if(als_times >= 2){
			als_temp[0] = als_temp[1];
		  	als_temp[1] = als_temp[2];
		  	als_temp[2] = luxdata_int;
		    luxdata_int = ((als_temp[0] + als_temp[1] + als_temp[2]) / 3);	
		}
		else if(als_times == 1){
		  	als_temp[1] = als_temp[2];
		  	als_temp[2] = luxdata_int;
			als_times++;
		    luxdata_int = ((als_temp[1] + als_temp[2]) / 2);	
		}
		else if(als_times == 0){
		  	als_temp[2] = luxdata_int;
			als_times++;
		}
	//-------------------------------------

	luxdata_int_prev = luxdata_int;
	//APS_DBG("ALS: als_data = %d\n", luxdata_int);

	return luxdata_int;
}



/*----------------------------------------------------------------------------*/
int ltr558_get_addr(struct alsps_hw *hw, struct ltr558_i2c_addr *addr)
{
	/***
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	***/
	return 0;
}

/*-----------------------------------------------------------------------------*/
void ltr558_eint_func(void)
{
	APS_FUN();

	struct ltr558_priv *obj = ltr558_obj;
	if(!obj)
	{
		return;
	}
	
	schedule_work(&obj->eint_work);
	//schedule_delayed_work(&obj->eint_work);
}

/*----------------------------------------------------------------------------*/
/*---for interrup work mode support ---*/
int ltr558_setup_eint(struct i2c_client *client)
{
	APS_FUN();
	struct ltr558_priv *obj = (struct ltr558_priv *)i2c_get_clientdata(client);        

	ltr558_obj = obj;
	
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, ltr558_eint_func, 0);
	
	mt_eint_unmask(CUST_EINT_ALS_NUM);  
    return 0;
}

/*----------------------------------------------------------------------------*/
static void ltr558_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "LTR558")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "LTR558")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}

/*----------------------------------------------------------------------------*/
/*---for interrup work mode support ---*/
static int ltr558_check_and_clear_intr(struct i2c_client *client) 
{
//***
	APS_FUN();

	int res,intp,intl;
	u8 buffer[2];	
	u8 temp;
		//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/	
		//	  return 0;
	
		buffer[0] = LTR558_ALS_PS_STATUS;
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		temp = buffer[0];

		res = 1;
		intp = 0;
		intl = 0;
		if(0 != (buffer[0] & 0x02))
		{
			res = 0;
			intp = 1;
		}
		if(0 != (buffer[0] & 0x08))
		{
			res = 0;
			intl = 1;		
		}
	
		if(0 == res)
		{
			if((1 == intp) && (0 == intl))
			{
				buffer[1] = buffer[0] & 0xfD;
				
			}
			else if((0 == intp) && (1 == intl))
			{
				buffer[1] = buffer[0] & 0xf7;
			}
			else
			{
				buffer[1] = buffer[0] & 0xf5;
			}
			buffer[0] = LTR558_ALS_PS_STATUS;
			res = i2c_master_send(client, buffer, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			else
			{
				res = 0;
			}
		}
	
		return res;
	
	EXIT_ERR:
		APS_ERR("tmd2772_check_and_clear_intr fail\n");
		return 1;

}
/*----------------------------------------------------------------------------*/


/*yucong add for interrupt mode support MTK inc 2012.3.7*/
static int ltr558_check_intr(struct i2c_client *client) 
{
//	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	APS_FUN();

	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

	buffer[0] = LTR558_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//APS_ERR("tmd2772_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x02))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x08))
	{
		res = 0;
		intl = 1;		
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2772_check_intr fail\n");
	return 1;
}

static int ltr558_clear_intr(struct i2c_client *client) 
{
//	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[2];

	APS_FUN();
	
	buffer[0] = LTR558_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	buffer[1] = buffer[0] & 0xf5;
	buffer[0] = LTR558_ALS_PS_STATUS;

	res = i2c_master_send(client, buffer, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2772_check_and_clear_intr fail\n");
	return 1;
}

/*----------------------------------------------------------------------------*/
static int ltr558_get_partid(void)
{
	int i = 0, regdata = 0;
	
	APS_LOG("ltr558_get_partid...\n");
	
	for(i=0; i<3; i++){
		regdata = ltr558_i2c_read_reg(LTR558_PART_ID_REG);
		if(regdata < 0){
		    APS_ERR("read i2c error!\n");
			return ltr558_ERR_I2C;
		}
		if(LTR558_PART_ID == regdata){
			APS_LOG("Get ltr558 partid OK!\n");
			return ltr558_SUCCESS;
		}
	}
	APS_ERR("This is not ltr558, partid = 0x%x!\n",regdata);
	return -1; 
}

/*----------------------------------------------------------------------------*/

static int ltr558_devinit(void)
{
	int res;
	int init_ps_gain;
	int init_als_gain;
	u8 databuf[2];	

	struct i2c_client *client = ltr558_obj->client;

	struct ltr558_priv *obj = ltr558_obj;   
	
	mdelay(PON_DELAY);

	// Enable PS to Gain1 at startup
	init_ps_gain = PS_RANGE8;   //PS_RANGE1 //by bing
	ps_gainrange = init_ps_gain;

	res = ltr558_ps_enable(init_ps_gain);
	if (res < 0)
		goto EXIT_ERR;


	// Enable ALS to Full Range at startup
	init_als_gain = ALS_RANGE2_64K;  //ALS_RANGE2_64K;  
	als_gainrange = init_als_gain;

	res = ltr558_als_enable(init_als_gain);
	if (res < 0)
		goto EXIT_ERR;

	/*---for interrup work mode support ---*/
	if(0 == obj->hw->polling_mode_ps)
	{	
		APS_LOG("eint enable");
		ltr558_ps_set_thres();

		databuf[0] = LTR558_PS_MEAS_RATE;	
		databuf[1] = 0x00;    //50ms    default 100ms
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}
		
		databuf[0] = LTR558_INTERRUPT;	
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}

		databuf[0] = LTR558_INTERRUPT_PERSIST;	
		databuf[1] = 0x10;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}
		
		databuf[0] = LTR558_ALS_MEAS_RATE;	
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr558_ERR_I2C;
		}

	}

	if((res = ltr558_setup_eint(client))!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	
	if((res = ltr558_check_and_clear_intr(client)))
	{
		APS_ERR("check/clear intr: %d\n", res);
		//    return res;
	}

	res = 0;

	EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;

}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_als(struct device_driver *ddri, char *buf)
{
	int als_data = 0;
	int als_enable, als_value;
	
	if(!ltr558_obj)
	{
		APS_ERR("ltr558_obj is null!!\n");
		return 0;
	}
	als_enable = test_bit(CMC_BIT_ALS, &ltr558_obj->enable)?(1):(0);
	if(als_enable){
		als_data = ltr558_als_read(als_gainrange);
	}else{
		return snprintf(buf, PAGE_SIZE, "ALS is disable\n");
	}
	if(als_data < 0){
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", als_data);
	}else{
		als_value = ltr558_get_als_value(ltr558_obj, als_data);
	}
	return snprintf(buf, PAGE_SIZE, "%d => %d lux\n",als_data,als_value);
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_ps(struct device_driver *ddri, char *buf)
{
	int	ps_data = 0;
	int ps_value, ps_enable;
	 
	if(!ltr558_obj)
	{
		APS_ERR("ltr558_obj is null!!\n");
		return 0;
	}
	ps_enable = test_bit(CMC_BIT_PS, &ltr558_obj->enable)?(1):(0);
	if(ps_enable){
		ps_data = ltr558_ps_read();
	}else{
		return snprintf(buf, PAGE_SIZE, "PS is disable\n");
	}		
	if(ps_data < 0){
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", ps_data);
	}else{
		ps_value = ltr558_get_ps_value(ltr558_obj, ps_data);
	}
	return snprintf(buf, PAGE_SIZE, "%d => 0x%04X\n",ps_data,ps_value);
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_ps_thres(struct device_driver *ddri, char *buf)
{
	int ps_thres_close, ps_thres_far, valid;

	struct alsps_hw *hw = ltr558_get_cust_alsps_hw();
	
		if(ps_cali.valid == 1)
		{
			ps_thres_close = ps_cali.close;
			ps_thres_far = ps_cali.far_away;
			valid = ps_cali.valid;
		}
		else if(ps_cali.valid == 0){
			ps_thres_close = hw->ps_threshold_high;
			ps_thres_far = hw->ps_threshold_low;
			valid = ps_cali.valid;
		}
	return snprintf(buf, PAGE_SIZE, "%d=>%d,%d\n",valid,ps_thres_close,ps_thres_far);
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int ps_enable, als_enable;
	if(!ltr558_obj)
	{
		APS_ERR("ltr558_obj is null!!\n");
		return 0;
	}
	
	als_enable = test_bit(CMC_BIT_ALS, &ltr558_obj->enable)?(1):(0);
	ps_enable = test_bit(CMC_BIT_PS, &ltr558_obj->enable)?(1):(0);
	
	if(ltr558_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			ltr558_obj->hw->i2c_num, ltr558_obj->hw->power_id, ltr558_obj->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "Enable: als:%d ps:%d\n",als_enable,ps_enable);

	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_reg(struct device_driver *ddri, char *buf)
{
	int i = 0, count = 0;
	int bufdata;
	
	if(!ltr558_obj)
	{
		APS_ERR("ltr558_obj is null!!\n");
		return 0;
	}

	for(i = 0; i < 31; i++)
	{
		bufdata = ltr558_i2c_read_reg(0x80+i);
		count+= sprintf(buf+count,"[%x] = (%x)\n",0x80+i,bufdata);
	}

	return count;
}
static ssize_t ltr558_store_reg(struct device_driver *ddri, char *buf, size_t count)
{
	int ret,value;
	u8 reg;
	if(!ltr558_obj)
	{
		APS_ERR("ltr558_obj is null!!\n");
		return 0;
	}
	
	if(2 == sscanf(buf, "%x %x ", &reg,&value))
	{ 
		APS_DBG("before write reg: %x, reg_value = %x  write value=%x\n", reg,ltr558_i2c_read_reg(reg),value);
	    ret=ltr558_i2c_write_reg(reg,value);
		APS_DBG("after write reg: %x, reg_value = %x\n", reg,ltr558_i2c_read_reg(reg));
	}
	else
	{
		APS_DBG("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,	 S_IWUSR | S_IRUGO, ltr558_show_als,	NULL);
static DRIVER_ATTR(ps,	 S_IWUSR | S_IRUGO, ltr558_show_ps,		NULL);
static DRIVER_ATTR(ps_thres, S_IWUSR | S_IRUGO, ltr558_show_ps_thres,	NULL);
static DRIVER_ATTR(status, S_IWUSR | S_IRUGO, ltr558_show_status, NULL);
static DRIVER_ATTR(reg,	 S_IWUSR | S_IRUGO, ltr558_show_reg, 	ltr558_store_reg);
/*----------------------------------------------------------------------------*/
static struct device_attribute *ltr558_attr_list[] = {
	&driver_attr_als,
	&driver_attr_ps,
	&driver_attr_ps_thres,
	&driver_attr_status,
	&driver_attr_reg,
};
/*----------------------------------------------------------------------------*/
static int ltr558_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(ltr558_attr_list)/sizeof(ltr558_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, ltr558_attr_list[idx]))
		{
			APS_ERR("driver_create_file (%s) = %d\n", ltr558_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
	static int ltr558_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(ltr558_attr_list)/sizeof(ltr558_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, ltr558_attr_list[idx]);
	}

	return err;
}
/*----------------------------------------------------------------------------*/

static int ltr558_get_als_value(struct ltr558_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	u8 databuf[2];
	struct i2c_client *client = ltr558_obj->client;
		
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
	//-------------------------------
		if((obj->hw->als_value[idx] < 350) && (als_gainrange != ALS_RANGE1_320)){
			
				als_gainrange = ALS_RANGE1_320;
							
				ltr558_als_disable();
				mdelay(20);
				
				ltr558_als_enable(als_gainrange);
				mdelay(20);
		}else if((obj->hw->als_value[idx] > 1000) && (als_gainrange != ALS_RANGE2_64K)){
			
				als_gainrange = ALS_RANGE2_64K;
				
				ltr558_als_disable();
				mdelay(20);
				
				ltr558_als_enable(als_gainrange);
				mdelay(20);
		}			
	//-------------------------------
		//APS_DBG("ALS: als_value: %d => %d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %d => %d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
static int ltr558_get_ps_value(struct ltr558_priv *obj, u16 ps)
{
	int val, mask = atomic_read(&obj->ps_mask);
	int invalid = 0;

	static int val_temp = 1;
	
	APS_LOG("%s, valid =%d, close = %d, far = %d\n",__func__,ps_cali.valid,ps_cali.close,ps_cali.far_away);
		
	if(ps_cali.valid == 1)
	{
			//APS_LOG("fangliang: valid =%d  %s\n", ps_cali.valid,__func__);
            if(ps > ps_cali.close)
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
			}
			else if(ps < ps_cali.far_away)
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
			}
			else
				val = val_temp;
	}
	else
	{
			//APS_LOG("fangliang: valid =%d  %s\n", ps_cali.valid,__func__);
			if((ps > atomic_read(&obj->ps_thd_val_high)))
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
			}
			//else if((ps < atomic_read(&obj->ps_thd_val_low))&&(temp_ps[0]  < atomic_read(&obj->ps_thd_val_low)))
			else if((ps < atomic_read(&obj->ps_thd_val_low)))
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
			}
			else
				val = val_temp;	
	}
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 50000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}


/*----------------------------------------------------------------------------*/
/*---for interrup work mode support ---*/
static void ltr558_eint_work(struct work_struct *work)
{
	struct ltr558_priv *obj = (struct ltr558_priv *)container_of(work, struct ltr558_priv, eint_work);
	int err;
	hwm_sensor_data sensor_data;
	u8 databuf[2];
	int res = 0;
	APS_FUN();
	if((err = ltr558_check_intr(obj->client)))
	{
		APS_ERR("ltr558_eint_work check intrs: %d\n", err);
	}
	else
	{
		//get raw data
		obj->ps = ltr558_ps_read();
    	if(obj->ps < 0)
    	{
    		err = -1;
    		return;
    	}
		
		sensor_data.values[0] = ltr558_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			
		APS_DBG("%s: ps_data=%d, ps_value=%d\n",__func__,obj->ps,sensor_data.values[0]);
		
/*singal interrupt function add*/
#if 1

	if(1 == ps_cali.valid){
		
		if(intr_flag_value){
				//printk("lidechun interrupt value ps will < 750, intr_flag_value = %d \n",intr_flag_value);

				databuf[0] = LTR558_PS_THRES_LOW_0; 
				databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR558_PS_THRES_LOW_1; 
				databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR558_PS_THRES_UP_0;	
				databuf[1] = (u8)(0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR558_PS_THRES_UP_1; 
				databuf[1] = (u8)((0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
		else{	
				//printk("lidechun interrupt value ps will > 900,intr_flag_value = %d\n",intr_flag_value);
				databuf[0] = LTR558_PS_THRES_LOW_0; 
				databuf[1] = (u8)(0 & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR558_PS_THRES_LOW_1; 
				databuf[1] = (u8)((0 & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);				
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR558_PS_THRES_UP_0;	
				databuf[1] = (u8)(ps_cali.close & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR558_PS_THRES_UP_1; 
				databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
			}
	}else{
	
		if(intr_flag_value){
				//printk("lidechun interrupt value ps will < 750, intr_flag_value = %d \n",intr_flag_value);

				databuf[0] = LTR558_PS_THRES_LOW_0;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR558_PS_THRES_LOW_1;	
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR558_PS_THRES_UP_0;	
				databuf[1] = (u8)(0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR558_PS_THRES_UP_1; 
				databuf[1] = (u8)((0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
		else{	
				//printk("lidechun interrupt value ps will > 900,intr_flag_value = %d\n",intr_flag_value);
				databuf[0] = LTR558_PS_THRES_LOW_0;	
				databuf[1] = (u8)(0 & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR558_PS_THRES_LOW_1;	
				databuf[1] = (u8)((0 & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR558_PS_THRES_UP_0;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR558_PS_THRES_UP_1; 
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
			}
	}
#endif
		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	ltr558_clear_intr(obj->client);
	mt_eint_unmask(CUST_EINT_ALS_NUM);      
}


/*----------------------------------------------------------------------------*/

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int ltr558_open(struct inode *inode, struct file *file)
{
	file->private_data = ltr558_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int ltr558_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/

/*begin: added by fangliang for ps nvram*/
static void ltr558_WriteCalibration(struct PS_CALI_DATA_STRUCT *data_cali)
{
	struct alsps_hw *hw = ltr558_get_cust_alsps_hw();

 	if(data_cali->valid == 1)
  	{
	  	ps_cali.close = data_cali->close;
		ps_cali.far_away = data_cali->far_away;
		ps_cali.valid = 1;

  	}
	else if(data_cali->valid == 0){
		ps_cali.close = hw->ps_threshold_high;
		ps_cali.far_away = hw->ps_threshold_low;
		ps_cali.valid = 0;
	}
}

static int ltr558_read_data_for_cali(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali)
{
     int i=0 ,err = 0,j = 0,sum=0;
	 u16 data[21],data_cali;

	 for(i = 0;i<20;i++)
	 	{
	 		mdelay(5);
			if(err = ltr558_ps_read())
			{
				data[i] = err;
				sum += data[i];

			}
			else
			{
				//APS_ERR("fangliang ltr558_read_data_for_cali fail: %d\n", i); 
				break;
			}
			mdelay(55);
	 	}
	 	 
	 if(i == 20)
	 	{
			data_cali = sum/20;
			//APS_LOG("fangliang:ltr558_read_data_for_cali data = %d  %s",data_cali,__func__);

			ps_data_cali->far_away = data_cali + PS_UNDETECTED_THRES;
			ps_data_cali->close = data_cali + PS_DETECTED_THRES;
			ps_data_cali->valid = 1;

			if(data_cali == 0)
			{
				ps_data_cali->far_away = 34 + PS_UNDETECTED_THRES;
				ps_data_cali->close = 34 + PS_DETECTED_THRES;
				ps_data_cali->valid = 1;
			}

			err= 0;
	 	}
	 else
	 	{
	 	ps_data_cali->valid = 0;
//		APS_LOG("fangliang:ltr558_read_data_for_cali data error  %s",__func__);
	 	err=  -1;
	 	}
	 
	 return err;
}
/*end: added by fangliang for ps nvram*/

//static int ltr558_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
  //     unsigned long arg)
static int ltr558_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)       
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct ltr558_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	struct PS_CALI_DATA_STRUCT ps_cali_temp;// added by fangliang for ps nvram

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
			    err = ltr558_ps_enable(ps_gainrange);
				if(err < 0)
				{
					APS_ERR("enable ps fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
			    err = ltr558_ps_disable();
				if(err < 0)
				{
					APS_ERR("disable ps fail: %d\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
		    obj->ps = ltr558_ps_read();
			if(obj->ps < 0)
			{
				goto err_out;
			}
			
			dat = ltr558_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			obj->ps = ltr558_ps_read();
			if(obj->ps < 0)
			{
				goto err_out;
			}
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
			    err = ltr558_als_enable(als_gainrange);
				if(err < 0)
				{
					APS_ERR("enable als fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
			    err = ltr558_als_disable();
				if(err < 0)
				{
					APS_ERR("disable als fail: %d\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
		    obj->als = ltr558_als_read(als_gainrange);
			if(obj->als < 0)
			{
				goto err_out;
			}

			dat = ltr558_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			obj->als = ltr558_als_read(als_gainrange);
			if(obj->als < 0)
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			ltr558_get_als_value(obj, obj->als);
			break;
/*begin: added by fangliang for ps nvram*/
		case ALSPS_SET_PS_CALI:
			dat = (void __user*)arg;
			if(dat == NULL)
			{
				APS_LOG("dat == NULL\n");
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&ps_cali_temp,dat, sizeof(ps_cali_temp)))
			{
				APS_LOG("copy_from_user\n");
				err = -EFAULT;
				break;	  
			}
			ltr558_WriteCalibration(&ps_cali_temp);//ps_cali 
			//APS_LOG(" fangliang: ALSPS_SET_PS_CALI %d,%d,%d   %s \tn",ps_cali_temp.close,ps_cali_temp.far_away,ps_cali_temp.valid,__func__);
			break;
		case ALSPS_GET_PS_RAW_DATA_FOR_CALI:
			ltr558_read_data_for_cali(obj->client,&ps_cali_temp);
			if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
/*end: added by fangliang for ps nvram*/			

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}

/*----------------------------------------------------------------------------*/
static struct file_operations ltr558_fops = {
	//.owner = THIS_MODULE,
	.open = ltr558_open,
	.release = ltr558_release,
	.unlocked_ioctl = ltr558_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice ltr558_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ltr558_fops,
};

static int ltr558_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	//struct ltr558_priv *obj = i2c_get_clientdata(client);    
	//int err;
	APS_FUN();    
	
#if 0 
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		err = ltr558_als_disable();
		if(err < 0)
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		err = ltr558_ps_disable();
		if(err < 0)
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		ltr558_power(obj->hw, 0);
	}
#endif	

	return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr558_i2c_resume(struct i2c_client *client)
{
	//struct ltr558_priv *obj = i2c_get_clientdata(client);        
	//int err;
	APS_FUN();
	
#if 0
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	ltr558_power(obj->hw, 1);
/*	err = ltr558_devinit();
	if(err < 0)
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}*/
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
	    err = ltr558_als_enable(als_gainrange);
	    if (err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		err = ltr558_ps_enable(ps_gainrange);
	    if (err < 0)
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}
#endif

	return 0;
}

static void ltr558_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct ltr558_priv *obj = container_of(h, struct ltr558_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	atomic_set(&obj->als_suspend, 1); 
	err = ltr558_als_disable();
	if(err < 0)
	{
		APS_ERR("disable als fail: %d\n", err); 
	}
}

static void ltr558_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct ltr558_priv *obj = container_of(h, struct ltr558_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
	    err = ltr558_als_enable(als_gainrange);
		if(err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
}

static int ltr558_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ltr558_priv *obj = (struct ltr558_priv *)self;
	
	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
				    err = ltr558_ps_enable(ps_gainrange);
					if(err < 0)
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
				    err = ltr558_ps_disable();
					if(err < 0)
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				obj->ps = ltr558_ps_read();
    			if(obj->ps < 0)
    			{
    				err = -1;
    				break;
    			}
				sensor_data->values[0] = ltr558_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

static int ltr558_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ltr558_priv *obj = (struct ltr558_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
				    err = ltr558_als_enable(als_gainrange);
					if(err < 0)
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
				    err = ltr558_als_disable();
					if(err < 0)
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				obj->als = ltr558_als_read(als_gainrange);
				#if defined(MTK_AAL_SUPPORT)
				sensor_data->values[0] = obj->als;
				#else				
				sensor_data->values[0] = ltr558_get_als_value(obj, obj->als);
				#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int ltr558_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, LTR558_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr558_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltr558_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	client->addr = 0x46>>1;
	
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ltr558_obj = obj;

	obj->hw = ltr558_get_cust_alsps_hw();
	ltr558_get_addr(obj->hw, &obj->addr);

	INIT_WORK(&obj->eint_work, ltr558_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	//atomic_set(&obj->als_cmd_val, 0xDF);
	//atomic_set(&obj->ps_cmd_val,  0xC1);
	//atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   
	obj->als_modulus = (400*100)/(16*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);

	APS_LOG("ltr558_devinit() start...!\n");
	ltr558_i2c_client = client;

	if(err = ltr558_get_partid())
	{
		goto exit_init_failed;
	}
	
	if(err = ltr558_devinit())
	{
		goto exit_init_failed;
	}
	APS_LOG("ltr558_devinit() ...OK!\n");

	//printk("@@@@@@ manufacturer value:%x\n",ltr558_i2c_read_reg(0x87));

	if(err = misc_register(&ltr558_device))
	{
		APS_ERR("ltr558_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if(err = ltr558_create_attr(&(LTR558_init_info.platform_diver_addr->driver)))
	{
		APS_ERR("ltr558 create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	obj_ps.self = ltr558_obj;
	/*---for interrup work mode support ---*/
	if(1 == obj->hw->polling_mode_ps)
	{
		obj_ps.polling = 1;
	}
	else
	{
		obj_ps.polling = 0;
	}

	obj_ps.sensor_operate = ltr558_ps_operate;
	if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = ltr558_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = ltr558_als_operate;
	if(err = hwmsen_attach(ID_LIGHT, &obj_als))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 2,
	obj->early_drv.suspend  = ltr558_early_suspend,
	obj->early_drv.resume   = ltr558_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif
	{
		//#include <linux/hardware_info.h>
		//hardwareinfo_set_prop(HARDWARE_ALSPS, "ltr558");
	}

	LTR558_init_flag = 0;
	
	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
		ltr558_delete_attr(&(LTR558_init_info.platform_diver_addr->driver));
	exit_misc_device_register_failed:
		misc_deregister(&ltr558_device);
	exit_init_failed:
	//i2c_detach_client(client);
	exit_kfree:
	kfree(obj);
	exit:
	ltr558_i2c_client = NULL;           
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	
	LTR558_init_flag = -1;
	
	return err;
}
/*----------------------------------------------------------------------------*/
static int ltr558_i2c_remove(struct i2c_client *client)
{
	int err;	

	if(err = ltr558_delete_attr(&(LTR558_init_info.platform_diver_addr->driver)))
	{
		APS_ERR("ltr558_delete_attr fail: %d\n", err);
	}
	
	if(err = misc_deregister(&ltr558_device))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	ltr558_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
#if 0
/*----------------------------------------------------------------------------*/
static int ltr558_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	ltr558_power(hw, 1);
	//ltr558_force[0] = hw->i2c_num;
	//ltr558_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",ltr558_force[0],ltr558_force[1]);
	if(i2c_add_driver(&ltr558_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr558_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	ltr558_power(hw, 0);    
	i2c_del_driver(&ltr558_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver ltr558_alsps_driver = {
	.probe      = ltr558_probe,
	.remove     = ltr558_remove,    
	.driver     = {
		.name  = "als_ps",
		//.owner = THIS_MODULE,
	}
};
#endif

static int LTR558_remove(void)
{
	struct alsps_hw *hw = ltr558_get_cust_alsps_hw();
	APS_FUN();
	ltr558_power(hw, 0);
	i2c_del_driver(&ltr558_i2c_driver);
	return 0;
}
static int LTR558_local_init(void)
{
	struct acc_hw *hw = ltr558_get_cust_alsps_hw();
	APS_FUN(); 

	ltr558_power(hw, 1);
	if(i2c_add_driver(&ltr558_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	if(-1 == LTR558_init_flag)
	{
		return -1;
	}
	return 0;	
}

/*----------------------------------------------------------------------------*/
static int __init ltr558_init(void)
{
	APS_FUN();
	struct alsps_hw *hw = ltr558_get_cust_alsps_hw();
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 
	i2c_register_board_info(hw->i2c_num, &i2c_ltr558, 1);
#if 0
	if(platform_driver_register(&ltr558_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
#endif
	hwmsen_alsps_sensor_add(&LTR558_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr558_exit(void)
{
	APS_FUN();
	//platform_driver_unregister(&ltr558_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(ltr558_init);
module_exit(ltr558_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Keqi Li");
MODULE_DESCRIPTION("LTR-558ALS Driver");
MODULE_LICENSE("GPL");

