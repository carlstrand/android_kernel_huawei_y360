/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)

#define LCM_ID_NT35590 (0x90)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq_V3(ppara, size, force_update)	        	lcm_util.dsi_set_cmdq_V3(ppara, size, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


#define   LCM_DSI_CMD_MODE							0

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static void lcm_poweron(bool onoff)
{
	if(onoff == true)
	{
		mt_set_gpio_out((GPIO82 | 0x80000000),1);
		MDELAY(15);
		mt_set_gpio_out((GPIO81 | 0x80000000),1);
		MDELAY(15);
	}
	else
	{
		mt_set_gpio_out((GPIO81 | 0x80000000),0);
                MDELAY(15);
		mt_set_gpio_out((GPIO82 | 0x80000000),0);		
                MDELAY(15);		
	}
}

static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
    // Sleep Out
	
	{0xB0, 1, {0x04}},
	{0xD6, 1, {0x01}},
	{0xB3, 6, {0x14,0x00,0x00,0x00,0x00,0x00}},
	{0xB6, 2, {0x3A,0xC3}},//590Mbps~740Mbps
	{0xC0, 1, {0x00}},

	{0xC1, 34, {0x84,0x60,0x10,0xEB,0xFF,0x6F,0xCE,0xFF,0xFF,0x17,0x12,0x58,0x73,0xAE,0x31,0x20,0xC6,0xFF,0xFF,0x1F,0xF3,0xFF,0x5F,0x10,0x10,0x10,0x10,0x00,0x62,0x01,0x22,0x22,0x00,0x01}},
	{0xC2, 7, {0x31,0xF7,0x80,0x06,0x08,0x80,0x00}},
	{0xC4, 22, {0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x06}},
	{0xC6, 40, {0xC8,0x08,0x67,0x08,0x67,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x16,0x18,0x08,0xC8,0x08,0x67,0x08,0x67,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x16,0x18,0x08}},
	//{0xC6, 40, {0xC8,0x08,0x67,0x08,0x67,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x16,0x18,0x08,0x00,0x08,0x67,0x08,0x67,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x16,0x18,0x08}},
	
	{0xCA, 32, {0x00,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0x14,0x14,0x80,0x80,0x0A,0x4A,0x37,0xA0,0x55,0xF8,0x0C,0x0C,0x20,0x10,0x3F,0x3F,0x00,0x00,0x10,0x10,0x3F,0x3F,0x3F,0x3F}},
	{0xCB, 8, {0x31,0xFC,0x3F,0x8C,0x00,0x00,0x00,0x00}},
	{0xCC, 1, {0x0B}},

	{0xD0, 10, {0x11,0x81,0xBB,0x19,0x99,0x4C,0x19,0x19,0x0C,0x00}},
	{0xD3, 25, {0x1B,0x33,0xBB,0xBB,0xB3,0x33,0x33,0x33,0x01,0x01,0x00,0xA0,0xD8,0xA0,0x0D,0x48,0x48,0x44,0x3B,0x22,0x72,0x07,0x3D,0xBF,0x33}},

	{0xD5, 7, {0x06,0x00,0x00,0x01,0x42,0x01,0x42}},
	{0xD5, 7, {0x06,0x00,0x00,0x01,0x42,0x01,0x42}},

	{0xC7, 30, {0x00,0x10,0x1F,0x20,0x2C,0x38,0x42,0x50,0x34,0x3C,0x4C,0x5D,0x68,0x6D,0x71,0x00,0x10,0x1F,0x20,0x2C,0x38,0x42,0x50,0x34,0x3C,0x4C,0x5D,0x68,0x6D,0x71}},
	{0xC8, 19, {0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0xFC,0x00}},
	{0xCE, 7, {0x00,0x01,0x00,0xC1,0x00,0x00,0x00}},

	{0x51, 1, {0xFF}},
	{0x53, 1, {0x2C}},
	{0x55, 1, {0x01}},

	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},

	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
	
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
	// Display off sequence
	{0xD5, 7, {0x06,0x00,0x00,0x00,0x48,0x00,0x48}},
	{0xD5, 7, {0x06,0x00,0x00,0x00,0x48,0x00,0x48}},
	{REGFLAG_DELAY, 25, {}},
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
				//dsi_set_cmdq_V3(lcm_deep_sleep_mode_in_setting_V3, sizeof(lcm_deep_sleep_mode_in_setting_V3) / sizeof(LCM_setting_table_V3), 1);
       	}
    }
	
}



static void init_lcm_registers(void)
{
	  //lcm_poweron(true);
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    //lcm_poweron(true);
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;
		params->ctrl = LCM_CTRL_GPIO;
		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=720*3;	

		
		params->dsi.vertical_sync_active				= 1;
		params->dsi.vertical_backporch					= 4;
		params->dsi.vertical_frontporch					= 2;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 50;
		params->dsi.horizontal_backporch				= 100;
		params->dsi.horizontal_frontporch				= 120;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		params->dsi.pll_select=1;	//0: MIPI_PLL; 1: LVDS_PLL
		// Bit rate calculation
		//1 Every lane speed
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
		params->dsi.fbk_div =0x11;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
}

static void lcm_init(void)
{
   // mt_set_gpio_mode(GPIO131, GPIO_MODE_07);    
   // mt_set_gpio_mode(GPIO_LCM_POWER_EN, GPIO_MODE_00);    
   // mt_set_gpio_dir(GPIO_LCM_POWER_EN, GPIO_DIR_OUT);
   // mt_set_gpio_out(GPIO_LCM_POWER_EN, GPIO_OUT_ZERO);
   lcm_poweron(true);
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);
//	SET_RESET_PIN(0);	
//	MDELAY(10);
	#ifdef BUILD_LK
		printf("%s r63315_in\n", __func__);
	#else
		printk("%s r63315_in\n", __func__);
	#endif
	init_lcm_registers();
}



static void lcm_suspend(void)
{
	push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	 lcm_poweron(false);
//	mt_set_gpio_out(GPIO_LCM_POWER_EN, GPIO_OUT_ZERO);
	MDELAY(60);		
}


static void lcm_resume(void)
{
#if 0
	int i = 0;
#ifdef BUILD_LK
	printf("%s in\n", __func__);
#else
	printk("%s in\n", __func__);
#endif
	SET_RESET_PIN(0);	
	MDELAY(1);	
  //  mt_set_gpio_out(GPIO_LCM_POWER_EN, GPIO_OUT_ONE);
	SET_RESET_PIN(1);		
	MDELAY(11);      
	for(i = 0; i < 11; ++i)
	{
  //			mt_set_gpio_out(GPIO_LCM_POWER_EN, GPIO_OUT_ZERO);
			UDELAY(5);
	//	    mt_set_gpio_out(GPIO_LCM_POWER_EN, GPIO_OUT_ONE);
			UDELAY(5);
	}
	MDELAY(1);
	for(i = 0; i < 27; ++i)
	{
//  			mt_set_gpio_out(GPIO_LCM_POWER_EN, GPIO_OUT_ZERO);
			UDELAY(5);
		//    mt_set_gpio_out(GPIO_LCM_POWER_EN, GPIO_OUT_ONE);
			UDELAY(5);
	}	
	MDELAY(8);  	
	init_lcm_registers();
	#ifdef BUILD_LK
		printf("%s out\n", __func__);
	#else
		printk("%s out\n", __func__);
	#endif	
#endif

    lcm_init();
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
#endif
#if 1
static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];  

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	
	SET_RESET_PIN(1);
	MDELAY(20); 
	
	 #if defined(BUILD_LK)
    lcm_poweron(true);
    #endif	

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; //we only need ID
    #ifdef BUILD_LK
		printf("%s, LK nt35590 debug: nt35590 id = 0x%08x\n", __func__, id);
    #else
		printk("%s, kernel nt35590 horse debug: nt35590 id = 0x%08x\n", __func__, id);
    #endif
    
    #if defined(BUILD_LK)
    lcm_poweron(false);
    #endif	
    
    return 1;
/*
    if(id == LCM_ID_NT35590)
    	return 1;
    else
        return 0;
*/

}
#endif

LCM_DRIVER r63315_fhd_dsi_vdo_lcm_drv = 
{
    .name			= "r63315_fhd_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
