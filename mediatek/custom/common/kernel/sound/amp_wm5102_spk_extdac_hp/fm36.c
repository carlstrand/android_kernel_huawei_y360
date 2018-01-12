/* drivers/misc/fm36.c - fm36 voice processor driver
 *
 *  Copyright (C) 2013 Fortemedia Inc.
 *  modified by qianhoulong in 2013/11/12
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <sound/core.h>
#include "fm36.h"
#include "fm36_psf.h"
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/delay.h>
#include <mach/mt_clkmgr.h>
#include <linux/timer.h>
#include <linux/xlog.h>
#define CONFIG_DEBUG_MSG

#ifdef CONFIG_DEBUG_MSG
#define PRINTK(format, args...) xlog_printk(ANDROID_LOG_DEBUG, "fm36", format, ##args);
#else
#define PRINTK(format, args...)
#endif

#define FM36_I2C_CHANNEL     (2)        //I2CL: SDA2,SCL2

#define FM36_SLAVE_ADDR_WRITE 0xC0
#define FM36_SLAVE_ADDR_READ  0xC1

#define FM36_I2C_DEVNAME "FM36"
#define hallsensor_DISABLE_TIMER  (2*HZ)
struct  timer_list hall_timer;
enum Set_Mode_Num g_fm36_effect_mode = fm36_STATE_NONE;

static FM36_CONTROL fm36_con;
// I2C variable
static struct i2c_client *fm36_dsp = NULL;

// new I2C register method
static const struct i2c_device_id fm36_i2c_id[] = {{FM36_I2C_DEVNAME, 0}, {}};
static struct i2c_board_info __initdata  fm36_dev = {I2C_BOARD_INFO(FM36_I2C_DEVNAME, (FM36_SLAVE_ADDR_WRITE >> 1))};

//function declration
static int fm36_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int fm36_i2c_remove(struct i2c_client *client);

//i2c driver
struct i2c_driver fm36_i2c_driver =
{
    .probe = fm36_i2c_probe,
    .remove = fm36_i2c_remove,
    .driver = {
        .name = FM36_I2C_DEVNAME,
    },
    .id_table = fm36_i2c_id,
};

 void fm36_reset(void)
{
    mt_set_gpio_mode(GPIO_FM36_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_FM36_RST_PIN, GPIO_OUT_ZERO);

    mdelay(10);
    mt_set_gpio_out(GPIO_FM36_RST_PIN, GPIO_OUT_ONE);
    mdelay(15);

    memset(&fm36_con, 0, sizeof(FM36_CONTROL));

    PRINTK("%s\n", __func__);
}

static int raw_i2c_read(struct i2c_client *client, char *buf, int count)
{
    if (count != i2c_master_recv(client, buf, count)){
        PRINTK("i2c_master_recv error\n");
        return -1;
    }

    return 0;
}

static int raw_i2c_write(struct i2c_client *client, char *buf, int count)
{
    if(count != i2c_master_send(client, buf, count)){
        PRINTK("i2c_master_send error\n");
        return -1;
    }

    return 0;
}

static int fm36_read(int regH, int regL)
{
    uint8_t wbuf[5];
    uint8_t rbuf[4];
    int dataH, dataL, dataA;
    struct i2c_client *client =fm36_dsp;

    wbuf[0]=0xFC;
    wbuf[1]=0xF3;
    wbuf[2]=0x37;
    wbuf[3]=regH;
    wbuf[4]=regL;
    raw_i2c_write(client, wbuf, 5);
    //msleep(1);

    /* Get high byte */
    rbuf[0]=0xfc;
    rbuf[1]=0xf3;
    rbuf[2]=0x60;
    rbuf[3]=0x26;
    raw_i2c_write(client, rbuf, 4);
    raw_i2c_read(client, rbuf, 1);
    dataH = rbuf[0];

    /* Get low byte */
    rbuf[0]=0xfc;
    rbuf[1]=0xf3;
    rbuf[2]=0x60;
    rbuf[3]=0x25;
    raw_i2c_write(client, rbuf, 4);
    raw_i2c_read(client, rbuf, 1);
    dataL = rbuf[0];

    dataA = dataH;
    dataA = dataA << 8;
    dataA = dataA | dataL;

    return dataA;
}

static int fm36_write(int regH, int regL, int dataH, int dataL)
{
    uint8_t wbuf[7];
    struct i2c_client *client = fm36_dsp;

    wbuf[0]=0xFC;
    wbuf[1]=0xF3;
    wbuf[2]=0x3B;
    wbuf[3]=regH;
    wbuf[4]=regL;
    wbuf[5]=dataH;
    wbuf[6]=dataL;
    raw_i2c_write(client, wbuf, 7);
    return 0;
}

int fm36_write_word(int reg, int data)
{
    int regH, regL;
    int dataH, dataL;

    regH = reg >> 8;
    regL = reg & 0xff;
    dataH = data >> 8;
    dataL = data & 0xff;

    //PRINTK("fm36 reg=0x%04x, data=0x%04x", reg, data);
    return fm36_write(regH, regL, dataH, dataL);	
}

static int fm36_read_word(int reg)
{
    int regH, regL;
    int data;

    regH = reg >> 8;
    regL = reg & 0xff;

    data = fm36_read(regH, regL);	
    //PRINTK("fm36 0x%04x=0x%04x", reg, data);
    return data;
}
static int fm36_Delay()
{
   int data=0;
PRINTK( "fm36_Delay enter \n" );  
#if 0
   data = fm36_read_word(0x2306);
	mdelay(50);
    PRINTK("fm36 0x2306=%x", data);
	
    data = fm36_read_word(0x22E0);
	
    PRINTK("fm36 in count 0x22E0=%x", data);
	mdelay(50);
    data = fm36_read_word(0x22E2);
	
    PRINTK("fm36 out count 0x22E2=%x", data);
//	mod_timer(&hall_timer, jiffies +(1000/HZ));

	PRINTK( "fm36 Starting timer to fire in 1000ms (%ld)\n", jiffies );  
  	int   ret = mod_timer( &hall_timer, jiffies + msecs_to_jiffies(1000) );  
  	  if (ret) PRINTK("fm36 Error in mod_timer\n");  
#endif
	
    return data;
}
static int fm36_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc, rc_mode, mode;
    fm36_dsp = client;

    PRINTK("%s %s\n", __func__, id->name);

    fm36_con.denoise_en = true;
    fm36_con.bypass_en = false;

    fm36_reset();

    fm36_write_word(0x2273, 0x0AAA);

    /*
    * read fm36 identity register
    * */
    rc = fm36_read_word(0x2273);
    if (rc == 0x0AAA)
    {
        PRINTK("detect fm36 okay \n");
    }
    else
    {
        PRINTK("detect fm36 failed\n");
    }

    fm36_voice_processor_Bypass();

    return 0;
}

static int fm36_i2c_remove(struct i2c_client *client)
{
    PRINTK("ecodec_i2c_remove");
    fm36_dsp = NULL;
    i2c_unregister_device(client);
    i2c_del_driver(&fm36_i2c_driver);
    return 0;
}

unsigned char FM36_Register(void)
{
    i2c_register_board_info(FM36_I2C_CHANNEL, &fm36_dev, 1);
    if (i2c_add_driver(&fm36_i2c_driver))
    {
        PRINTK("fail to add fm36 device into i2c");
        return -1;
    }
    return 0;
}

static int fm36_patch_code_download(void)
{
/*	struct i2c_client *client;
	unsigned char *buf;
	int length= 0;
	length = sizeof(fm36_patch_code);
	i2c_master_send(client, buf, length);*/
}

static void fm36_para_download(fm36_REG_struct *pPara, int length)
{
    int i = 0;
    
    for (i = 0; i < length; i++) {
        fm36_write_word(pPara[i].regaddr, pPara[i].regdata);
    }
}
static void fm36_para_read(fm36_REG_struct *pPara, int length)
{
    int i = 0;
    for (i = 0; i < length; i++) {
	pPara[i].regdata=0;
        pPara[i].regdata=fm36_read_word(pPara[i].regaddr);
	PRINTK("%s  regaddr=%x regdata=%x\n",__func__, pPara[i].regaddr,pPara[i].regdata);	
    }

}
static int  fm36_patch_cons_download(void)
{
//	fm36_para_download((fm36_REG_struct *)&fm36_patch_cons[0], sizeof(fm36_patch_cons)/sizeof(fm36_REG_struct));
}

static void fm36_sleep(void)
{
    fm36_write_word(0x22F9, 0x0001);//PWD by I2C
    mdelay(10);
}

 int fm36_set_init(struct i2c_client *client)
{
    fm36_dsp=client;
}
 
 int fm36_set_mode(enum Set_Mode_Num mode)
{
    int rc;
   PRINTK("fm36_set_mode_enter\n");
    PRINTK("fm36 old CEM=%d new CEM=%d\n", g_fm36_effect_mode, mode);

    switch (mode)
    {
        case fm36_STATE_8K_INIT:
            {
		  fm36_patch_code_download();
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_8K_INIT_para[0], sizeof(FM36_8K_INIT_para)/sizeof(fm36_REG_struct));
            }
            break;

        case fm36_STATE_SLEEP:
            {
                fm36_sleep();
            }
            break;

        case fm36_STATE_EFFECT_8K_HFNS_ENABLE:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_8K_HFNSon_para[0], sizeof(FM36_8K_HFNSon_para)/sizeof(fm36_REG_struct));
            }
            break;
	case fm36_STATE_EFFECT_8K_HFNS_DISABLE:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_8K_HFNSoff_para[0], sizeof(FM36_8K_HFNSoff_para)/sizeof(fm36_REG_struct));
            }
            break;
	case fm36_STATE_EFFECT_8K_HFFFP:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_8K_HFFFP_para[0], sizeof(FM36_8K_HFFFP_para)/sizeof(fm36_REG_struct));
            }
            break;
        case fm36_STATE_EFFECT_8K_HSNS_ENABLE:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_8K_HSNSon_para[0], sizeof(FM36_8K_HSNSon_para)/sizeof(fm36_REG_struct));
            }
            break;

        case fm36_STATE_EFFECT_8K_HSNS_DISABLE:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_8K_HSNSoff_para[0], sizeof(FM36_8K_HSNSoff_para)/sizeof(fm36_REG_struct));
            }
            break;
			
        case fm36_STATE_EFFECT_8K_BLUETOOTH_HEADSET:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_8K_BTHS_para[0], sizeof(FM36_8K_BTHS_para)/sizeof(fm36_REG_struct));
            }
            break;

        case fm36_STATE_EFFECT_8K_BLUETOOTH_HANDFREE:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_8K_BTHF_para[0], sizeof(FM36_8K_BTHF_para)/sizeof(fm36_REG_struct));
            }
            break;

        case fm36_STATE_EFFECT_8K_MEDIA_RECORD:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_8K_REC_para[0], sizeof(FM36_8K_REC_para)/sizeof(fm36_REG_struct));
            }
            break;
        case fm36_STATE_EFFECT_16K_HFNS_ENABLE:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_16K_HFNSon_para[0], sizeof(FM36_16K_HFNSon_para)/sizeof(fm36_REG_struct));
            }
            break;
	case fm36_STATE_EFFECT_16K_HFNS_DISABLE:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_16K_HFNSoff_para[0], sizeof(FM36_16K_HFNSoff_para)/sizeof(fm36_REG_struct));
            }
            break;
	case fm36_STATE_EFFECT_16K_HFFFP:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_16K_HFFFP_para[0], sizeof(FM36_16K_HFFFP_para)/sizeof(fm36_REG_struct));
            }
            break;
        case fm36_STATE_EFFECT_16K_HSNS_ENABLE:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_16K_HSNSon_para[0], sizeof(FM36_16K_HSNSon_para)/sizeof(fm36_REG_struct));
            }
            break;

        case fm36_STATE_EFFECT_16K_HSNS_DISABLE:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_16K_HSNSoff_para[0], sizeof(FM36_16K_HSNSoff_para)/sizeof(fm36_REG_struct));
            }
            break;

        case fm36_STATE_EFFECT_16K_MEDIA_RECORD:
            {
		  fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_16K_REC_para[0], sizeof(FM36_16K_REC_para)/sizeof(fm36_REG_struct));
            }
            break;

	case fm36_STATE_EFFECT_16K_HFVR:
            {
		  					fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_16K_HFVR_para[0], sizeof(FM36_16K_HFVR_para)/sizeof(fm36_REG_struct));
            }
            break;

	case fm36_STATE_EFFECT_16K_CARKITVR:
            {
		  					fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_16K_CARKIT_para[0], sizeof(FM36_16K_CARKIT_para)/sizeof(fm36_REG_struct));
            }
            break;
	case fm36_STATE_EFFECT_16K_VOICEINCALL_HEADPHONE:			//qhl 2013/11/12
            {
		  					fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_16K_VoiceInCall_HeadPhone_para[0], sizeof(FM36_16K_VoiceInCall_HeadPhone_para)/sizeof(fm36_REG_struct));
            }
            break;
	case fm36_STATE_EFFECT_32K_MEDIA_RECORD:
            {
		  					fm36_patch_cons_download();
                fm36_para_download((fm36_REG_struct *)&FM36_32K_REC_para[0], sizeof(FM36_32K_REC_para)/sizeof(fm36_REG_struct));
            }
            break;

        case fm36_STATE_FTM_8K_MIC0_TEST:
            {
		   				fm36_patch_cons_download();		
		   				fm36_para_download((fm36_REG_struct *)&FM36_8K_MIC0_para[0], sizeof(FM36_8K_MIC0_para)/sizeof(fm36_REG_struct));
            }
            break;

        case  fm36_STATE_FTM_8K_MIC1_TEST:
            {
	   	   				 fm36_patch_cons_download();	
                 fm36_para_download((fm36_REG_struct *)&FM36_8K_MIC1_para[0], sizeof(FM36_8K_MIC1_para)/sizeof(fm36_REG_struct));
            }
            break;
			
        case  fm36_STATE_FTM_8K_MIC2_TEST:
            {
	   	   				 fm36_patch_cons_download();	
                 fm36_para_download((fm36_REG_struct *)&FM36_8K_MIC2_para[0], sizeof(FM36_8K_MIC2_para)/sizeof(fm36_REG_struct));
            }
            break;
        case  fm36_STATE_FTM_PWD:
            {
		   PRINTK("fm36_STATE_FTM_PWD\n");
	   	   fm36_patch_cons_download();	
		     PRINTK("fm36 sizof = %d,%d,%d",sizeof(FM36_PWD_para), sizeof(fm36_REG_struct),sizeof(FM36_PWD_para)/sizeof(fm36_REG_struct));
                 fm36_para_download((fm36_REG_struct *)&FM36_PWD_para[0], sizeof(FM36_PWD_para)/sizeof(fm36_REG_struct));
            }
            break;
        case  fm36_STATE_FTM_NOPWD:
            {
	   	   				 fm36_patch_cons_download();	
                 fm36_para_download((fm36_REG_struct *)&FM36_NOPWD_para[0], sizeof(FM36_NOPWD_para)/sizeof(fm36_REG_struct));
            }
            break;
        default:
            PRINTK("not support effect mode %d\n", mode);
            return -1;
    }

    if (mode != fm36_STATE_SLEEP)
    {
        mdelay(50);
        rc = fm36_read_word(0x22fb);
        if (rc != 0x5a5a) {    
            PRINTK("fm36 set mode %d failed, result=0x%x\n", mode, rc);
            return -2;
        }
    }

    g_fm36_effect_mode = mode;
	PRINTK("fm36 set mode %d sucess, result=0x%x\n", mode, rc);
    return 0;
}

/*
 * fm36 command, FIXME 
 * retry two times, if fm36_set_mode failed 
 * */
 
int fm36_voice_processor_init(void)
{
    int rc;
    int retry = 2;

    do{
        rc = fm36_set_mode(fm36_STATE_8K_INIT);
        if (rc == 0) {
            break; 
        }
    }while(retry--);

    return rc;
}
int fm36_adb_read(char *para,int length)
{
    fm36_para_read((fm36_REG_struct *)para, length);
   return 0;
}
int fm36_voice_processor_8k_hf_noise_suppression(int on)
{
     int rc;
    int retry = 2;

    fm36_reset();
    
    do 
    {
        if (on)
        {
            rc = fm36_set_mode(fm36_STATE_EFFECT_8K_HFNS_ENABLE);
        }
        else
        {
            rc = fm36_set_mode(fm36_STATE_EFFECT_8K_HFNS_DISABLE);
        }

        if (rc == 0) {
            break; 
        }
    } while(retry--);

    return rc;
}

int fm36_voice_processor_8k_hf_ffp(void)
{
    int rc;
    int retry = 2;

    fm36_reset();
  do 
    {
    	rc = fm36_set_mode(fm36_STATE_EFFECT_8K_HFFFP);
	if (rc == 0) {
            break; 
        }
    } while(retry--);
    return rc;
}

int fm36_voice_processor_8k_hs_noise_suppression(int on)
{
    int rc;
    int retry = 2;

    fm36_reset();
    
    do 
    {
        if (on)
        {
            rc = fm36_set_mode(fm36_STATE_EFFECT_8K_HSNS_ENABLE);
        }
        else
        {
            rc = fm36_set_mode(fm36_STATE_EFFECT_8K_HSNS_DISABLE);
        }

        if (rc == 0) {
            break; 
        }
    } while(retry--);

    return rc;
}

int fm36_voice_processor_8k_bths(void)
{
    int rc;
    int retry = 2;

    fm36_reset();
  do 
    {
    	rc = fm36_set_mode(fm36_STATE_EFFECT_8K_BLUETOOTH_HEADSET);
	if (rc == 0) {
            break; 
        }
    } while(retry--);
    return rc;
}

int fm36_voice_processor_8k_bthf(void)
{
    int rc;
    int retry = 2;

    fm36_reset();
  do 
    {
    	rc = fm36_set_mode(fm36_STATE_EFFECT_8K_BLUETOOTH_HANDFREE);
	if (rc == 0) {
            break; 
        }
    } while(retry--);
    return rc;
}

int fm36_voice_processor_8k_mm_record(void)
{
    int rc;
    int retry = 2;

    fm36_reset();
    
    do
    {
        rc = fm36_set_mode(fm36_STATE_EFFECT_8K_MEDIA_RECORD);
        if (rc == 0) {
            break; 
        }
    }while (retry--);


    return rc;
}

int fm36_voice_processor_16k_hf_noise_suppression(int on)
{
    int rc = 0;
    int retry = 2;

    PRINTK("fm36_voice_processor_16k_hf_noise_suppression voicecall_en=%d \n",fm36_con.voicecall_en);   

    if(fm36_con.voicecall_en)
        return rc;

    fm36_reset();
    
    do
    {
        if (on)
        {
            rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HFNS_ENABLE);
        }
        else
        {
            rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HFNS_DISABLE);
        }

        if (rc == 0) {
            break; 
        }
        mdelay(10);
    } while(retry--);
    PRINTK("fm36 set mode fm36_STATE_EFFECT_16K_HFNS_ENABLE rc= %d \n",rc);

    fm36_con.voicecall_en = true;

    return rc;
}

int fm36_voice_processor_16k_hf_ffp(void)
{
    int rc;
    int retry = 2;

    fm36_reset();
    do
     {
    	   rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HFFFP);
	if(rc==0){
		break;
	 }
    	}while(retry--);
    return rc;
}

int fm36_voice_processor_16k_hs_music(void)
{
    int rc = 0;
    int retry = 2;

    PRINTK("fm36_voice_processor_16k_hs_music music_en=%d \n",fm36_con.music_en);   

    if(fm36_con.music_en)
        return rc;

    fm36_reset();
    
    do
    {
        rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HSNS_ENABLE);
        if (rc == 0) {
            break; 
        }
    } while(retry--);
    PRINTK("fm36 set mode fm36_STATE_EFFECT_16K_HSNS_ENABLE rc= %d \n",rc);

    fm36_con.music_en = true;

    return rc;
}

int fm36_voice_processor_16k_hs_noise_suppression(int on)
{
    int rc = 0;
    int retry = 2;

    PRINTK("fm36_voice_processor_16k_hs_noise_suppression voicecall_en=%d \n",fm36_con.voicecall_en);   

    if(fm36_con.voicecall_en)
        return rc;

    fm36_reset();
    
    do 
    {
        if (on)
        {
            rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HSNS_ENABLE);
        }
        else
        {
            rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HSNS_DISABLE);
        }

        if (rc == 0) {
            break; 
        }
        mdelay(10);
    } while(retry--);
    PRINTK("fm36 set mode fm36_STATE_EFFECT_16K_HSNS_ENABLE rc= %d \n",rc);

    fm36_con.voicecall_en = true;

    return rc;
}

int fm36_voice_processor_16k_mm_record(void)
{
    int rc;
    int retry = 2;

    fm36_reset();
    
    do
    {
        rc = fm36_set_mode(fm36_STATE_EFFECT_16K_MEDIA_RECORD);
        if (rc == 0) {
            break; 
        }
    }while (retry--);


    return rc;
}

int fm36_voice_processor_16k_hfvr(void)
{
    int rc;
    int retry = 2;

    fm36_reset();
    
    do
    {
        rc = fm36_set_mode(fm36_STATE_EFFECT_16K_HFVR);
        if (rc == 0) {
            break; 
        }
    }while (retry--);


    return rc;
}

int fm36_voice_processor_16k_carkit_vr(void)
{
    int rc;
    int retry = 2;

    fm36_reset();
    
    do
    {
        rc = fm36_set_mode(fm36_STATE_EFFECT_16K_CARKITVR);
        if (rc == 0) {
            break; 
        }
    }while (retry--);


    return rc;
}

int fm36_voice_processor_32K_mm_record(void)
{
    int rc;
    int retry = 2;

    fm36_reset();
    
    do
    {
        rc = fm36_set_mode(fm36_STATE_EFFECT_32K_MEDIA_RECORD);
        if (rc == 0) {
            break; 
        }
    }while (retry--);


    return rc;
}

int  fm36_voice_processor_sleep(void)
{
    int rc;

    rc = fm36_set_mode(fm36_STATE_SLEEP);

    return rc;
}

int  fm36_voice_processor_mic0_test(void)
{
    int rc;
    int retry = 2; 

    fm36_reset();
    
    do {
        rc = fm36_set_mode(fm36_STATE_FTM_8K_MIC0_TEST);
        if (rc == 0) {
            break;
        }
    }while(retry--);

    return rc;
}

int  fm36_voice_processor_mic1_test(void)
{
    int rc;
    int retry = 2;

    fm36_reset();

    do{
        rc = fm36_set_mode(fm36_STATE_FTM_8K_MIC1_TEST);
        if (rc == 0) {
            break; 
        }
    }while(retry--);

    return rc;
}

int  fm36_voice_processor_mic2_test(void)
{
    int rc;
    int retry = 2;

    fm36_reset();

    do{
        rc = fm36_set_mode(fm36_STATE_FTM_8K_MIC2_TEST);
        if (rc == 0) {
            break; 
        }
    }while(retry--);

    return rc;
}
int  fm36_voice_processor_Bypass(void)			//qhl 2013/11/12
{
    int rc = 0;
    int retry = 2;

    PRINTK("fm36_voice_processor_Bypass bypass_en=%d \n",fm36_con.bypass_en);   
    if(fm36_con.bypass_en)
        return rc;

    fm36_reset();

    do{
        rc = fm36_set_mode(fm36_STATE_FTM_PWD);
        if (rc == 0) {
            break;
        }
        mdelay(10);
    }while(retry--);
    PRINTK("fm36 set mode fm36_STATE_FTM_PWD rc= %d \n",rc);   
    rc= fm36_set_mode(fm36_STATE_SLEEP);
    PRINTK("fm36 set mode fm36_STATE_SLEEP rc= %d \n",rc);   

    fm36_con.bypass_en = true;

    return rc;
}
int  fm36_voice_processor_VoiceInCall_HeadPhone(void)	//qhl 2013/11/12
{
    int rc = 0;
    int retry = 2;

    PRINTK("fm36_voice_processor_VoiceInCall_HeadPhone voicecall_en=%d \n",fm36_con.voicecall_en);   

    if(fm36_con.voicecall_en)
        return rc;
    
    fm36_reset();

    do{
        rc = fm36_set_mode(fm36_STATE_EFFECT_16K_VOICEINCALL_HEADPHONE);
        if (rc == 0) {
            break; 
        }
        mdelay(10);
    }while(retry--);
    PRINTK("fm36 set mode fm36_STATE_EFFECT_16K_VOICEINCALL_HEADPHONE rc= %d \n",rc);   

    fm36_con.voicecall_en = true;

    return rc;
}

/*
 * Do fm36 susupend/resume in codec susupend/resume
 * */
static int fm36_voice_processor_suspend(struct i2c_client *client, pm_message_t mesg)
{	
    PRINTK("%s\n", __func__);
    return 0;
}

static int fm36_voice_processor_resume(struct i2c_client *client)
{
    PRINTK("%s\n", __func__);
    return 0;
}

static int fm36_voice_processo_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc;
    fm36_dsp = client;

    PRINTK("%s %s\n", __func__, id->name);

    fm36_reset();

    fm36_write_word(0x2273, 0x0AAA);

    /*
     * read fm36 identity register
     * */
    rc = fm36_read_word(0x2273);
    if (rc == 0x0AAA)
    {
        PRINTK("detect fm36 ok\n");
    }
    else
    {
        PRINTK("detect fm36 failed\n");
    }

    fm36_voice_processor_init();
    fm36_set_mode(fm36_STATE_SLEEP);

    return 0;
}


static int fm36_remove(struct i2c_client *client)
{
    return 0;
}


/*static const struct i2c_device_id fm36_id[] = {
    {"fm36-dsp", 0 },
    { }
};

static struct i2c_driver fm36_driver = {
    .probe          = fm36_voice_processo_probe,
    .remove         = fm36_remove,
    .suspend 		= fm36_voice_processor_suspend,
    .resume 		= fm36_voice_processor_resume,
    .id_table       = fm36_id,
    .driver         = {
        .name = "fm36-dsp",
    },
};

static void __exit fm36_exit(void)
{
    i2c_del_driver(&fm36_driver);
}

static int __init fm36_init(void)
{
    int ret = i2c_add_driver(&fm36_driver);
    if (ret){
        PRINTK("FAILED");
        return ret;
    }

    PRINTK("%s\n", __func__);
    return 0;
}

module_init(fm36_init);
module_exit(fm36_exit);
*/

void fm36_denoise_control(unsigned char mode)
{
    PRINTK("%s,mode=:%d\n", __func__,mode);

    if(mode)
        fm36_con.denoise_en = true;
    else
        fm36_con.denoise_en = false;
}

