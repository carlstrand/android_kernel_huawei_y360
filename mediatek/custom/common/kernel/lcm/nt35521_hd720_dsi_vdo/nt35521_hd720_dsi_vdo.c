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
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define LCM_ID_NT35521 (0x5521)

#define LCM_DSI_CMD_MODE	0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)  (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define REGFLAG_DELAY			0XFE
#define REGFLAG_END_OF_TABLE	0xFF   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
/*
	Note :

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	//must use 0x39 for init setting for all register.
*/

#if 0  //For AUO 5G  -> init code is in OTP
  //NT35521 + H466TAN02.2 
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
	{0xB1,2,{0x68,0x21}},
	{0xB6,1,{0x01}},
	{0xB8,4,{0x01,0x02,0x08,0x02}},
	{0xBB,2,{0x11,0x11}},
	{0xBC,2,{0x00,0x00}},
	{0xBD,5,{0x01,0xA3,0x20,0x10,0x01}},

	{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
	{0xB0,2,{0x0F,0x0F}},
	{0xB1,2,{0x0F,0x0F}},
	{0xB3,2,{0x1E,0x1E}},
	{0xB4,2,{0x18,0x18}},
	{0xB5,2,{0x05,0x05}},
	{0xB9,2,{0x34,0x34}},
	{0xBA,2,{0x24,0x24}},
	// VGMP VGSP = 4.5V
	{0xBC,2,{0x78,0x00}},
	{0xBD,2,{0x78,0x00}},

	{0xBE,1,{0x3E}},
	{0xC0,1,{0x0C}},
	{0xCA,1,{0x00}},

	{0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},
	{0xEE,1,{0x01}},
	{0xB0,16,{0x00,0xAF,0x00,0xB6,0x00,0xC5,0x00,0xD4,0x00,0xE2,0x00,0xF9,0x01,0x0F,0x01,0x33}},
	{0xB1,16,{0x01,0x52,0x01,0x85,0x01,0xB1,0x01,0xF6,0x02,0x2F,0x02,0x31,0x02,0x66,0x02,0x9F}},
	{0xB2,16,{0x02,0xC4,0x02,0xF5,0x03,0x15,0x03,0x40,0x03,0x5A,0x03,0x7B,0x03,0x94,0x03,0xAC}},
	{0xB3,4,{0x03,0xC8,0x03,0xFF}},

	// PAGE3 : GateEQ
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},
	{0xB0,2,{0x20,0x00}},
	{0xB1,2,{0x20,0x00}},

	//STV Settings
	{0xB2,5,{0x04,0x00,0x52,0x01,0x51}},
	{0xB3,5,{0x04,0x00,0x52,0x01,0x51}},

	//RST Settings
	{0xB6,5,{0x04,0x00,0x52,0x01,0x51}},
	{0xB7,5,{0x04,0x00,0x52,0x01,0x51}},

	//VDC Settings
	{0xBA,5,{0x44,0x00,0x60,0x01,0x72}},
	{0xBB,5,{0x44,0x00,0x60,0x01,0x72}},
	{0xBC,5,{0x53,0x00,0x03,0x00,0x48}},
	{0xBD,5,{0x53,0x00,0x03,0x00,0x48}},
	{0xC4,1,{0x40}},
	{0xC5,1,{0x40}},

	//PAGE5 : Initial & On/Off
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},
    {0xB0,2,{0x17,0x06}},
    {0xB8,1,{0x00}},
    {0xBD,5,{0x0F,0x03,0x03,0x00,0x03}},
    {0xB1,2,{0x17,0x06}},
    {0xB9,1,{0x00}},
    {0xB2,2,{0x17,0x06}},
    {0xBA,1,{0x00}},
    {0xB3,2,{0x17,0x06}},
    {0xBB,1,{0x00}},
    {0xB4,2,{0x17,0x06}},
    {0xB5,2,{0x17,0x06}},
    {0xB6,2,{0x17,0x06}},
    {0xB7,2,{0x17,0x06}},
    {0xBC,1,{0x00}},
    {0xE5,1,{0x06}},
    {0xE6,1,{0x06}},
    {0xE7,1,{0x06}},
    {0xE8,1,{0x06}},
    {0xE9,1,{0x0A}},
    {0xEA,1,{0x06}},
    {0xEB,1,{0x06}},
    {0xEC,1,{0x06}},

	// PAGE5 : dof_opt_en, clr_opt_en
	{0xED,1,{0x30}},

	// STV Settings
	{0xC0,1,{0x07}},
	{0xC1,1,{0x05}},

	// RST Settings
	{0xC4,1,{0x82}},
	{0xC5,1,{0x80}},

	// CLK Settings
	{0xC8,2,{0x03,0x20}},
	// modified v4
	{0xC9,2,{0x01,0x21}},
	{0xCA,2,{0x03,0x20}},
	{0xCB,2,{0x07,0x20}},

	// CLK Porch
	{0xD1,11,{0x03,0x05,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xD2,11,{0x03,0x05,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xD3,11,{0x00,0x05,0x04,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xD4,11,{0x00,0x05,0x04,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	// PAGE6 : GOUT Mapping, VGLO select
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x06}},
	{0xB0,2,{0x2E,0x2E}},
    {0xB1,2,{0x2E,0x2E}},
    {0xB2,2,{0x2E,0x2E}},
    {0xB3,2,{0x2E,0x09}},
    {0xB4,2,{0x0B,0x23}},
    {0xB5,2,{0x1D,0x1F}},
    {0xB6,2,{0x11,0x17}},
    {0xB7,2,{0x13,0x19}},
    {0xB8,2,{0x01,0x03}},
    {0xB9,2,{0x2E,0x2E}},

    {0xBA,2,{0x2E,0x2E}},
    {0xBB,2,{0x02,0x00}},
    {0xBC,2,{0x18,0x12}},
    {0xBD,2,{0x16,0x10}},
    {0xBE,2,{0x1E,0x1C}},
    {0xBF,2,{0x22,0x0A}},
    {0xC0,2,{0x08,0x2E}},
    {0xC1,2,{0x2E,0x2E}},
    {0xC2,2,{0x2E,0x2E}},
    {0xC3,2,{0x2E,0x2E}},
    {0xE5,2,{0x25,0x24}},

	// For X499 BW
	{0xC4,2,{0x2E,0x2E}},
    {0xC5,2,{0x2E,0x2E}},
    {0xC6,2,{0x2E,0x2E}},
    {0xC7,2,{0x2E,0x02}},
    {0xC8,2,{0x00,0x24}},
    {0xC9,2,{0x1E,0x1C}},
    {0xCA,2,{0x18,0x12}},
    {0xCB,2,{0x16,0x10}},
    {0xCC,2,{0x0A,0x08}},
    {0xCD,2,{0x2E,0x2E}},

    {0xCE,2,{0x2E,0x2E}},
    {0xCF,2,{0x09,0x0B}},
    {0xD0,2,{0x11,0x17}},
    {0xD1,2,{0x13,0x19}},
    {0xD2,2,{0x1D,0x1F}},
    {0xD3,2,{0x25,0x01}},
    {0xD4,2,{0x03,0x2E}},
    {0xD5,2,{0x2E,0x2E}},
    {0xD6,2,{0x2E,0x2E}},
    {0xD7,2,{0x2E,0x2E}},
    {0xE6,2,{0x22,0x23}},

    {0xD8,5,{0x00,0x00,0x00,0x00,0x00}},
    {0xD9,5,{0x00,0x00,0x00,0x00,0x00}},
    {0xE7,1,{0x00}},
#endif	

	{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
	{0xC8,1,{0x80}},
	
	{0x11,1,{0x00}},
	{REGFLAG_DELAY,120,{}}, 
	
	{0x29,1,{0x00}},
	{REGFLAG_DELAY,10,{}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
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
       	}
    }
	
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

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

	//params->active_width  = 62;    //physical size
	//params->active_height = 114;
	
    // enable tearing-free
	params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity	= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = BURST_VDO_MODE;
#endif

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_THREE_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size = 256;

    // Video mode setting
    //because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
    params->dsi.intermediat_buffer_num = 0;
	
    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active                = 8;
    params->dsi.vertical_backporch                  = 16;
    params->dsi.vertical_frontporch                 = 15;
    params->dsi.vertical_active_line                = FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active              = 5;
    params->dsi.horizontal_backporch                = 50;
    params->dsi.horizontal_frontporch               = 50;
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

	params->dsi.cont_clock = 0;  //nonContinuous mode

    // Bit rate calculation
    params->dsi.pll_div1 = 0;	// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps;  fvco=fref*(div1+1)
    params->dsi.pll_div2 = 1;	// div2=0,1,2,3;div1_real=1,2,4,4;  div2=0~15: fout=fvo/(2*div2)
    params->dsi.fbk_div  = 19;  // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	

}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    UDELAY(10); 
    SET_RESET_PIN(0);
    UDELAY(20); 
    SET_RESET_PIN(1);
    MDELAY(20); 

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
	
    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);

    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	/* Deep standby */
	data_array[0] = 0x014F3902;
    dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
	
	SET_RESET_PIN(0);
	MDELAY(50);
	
}

static void lcm_resume(void)
{
	lcm_init();
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(
    unsigned int x, 
    unsigned int y,
    unsigned int width, 
    unsigned int height)
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

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned char  id_high = 0;
	unsigned char  id_low  = 0;
	unsigned int   id = 0;
	unsigned char  buffer[2];
	unsigned int   array[16];  

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);

	SET_RESET_PIN(1);
	MDELAY(20); 

	//*************Enable CMD2 Page1  *******************//
	array[0]=0x00063902;
	array[1]=0x52AA55F0;
	array[2]=0x00000108;
	dsi_set_cmdq(array, 3, 1);

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xC5, buffer, 2);

	id_high = buffer[0];            //should be 0x55
	id_low  = buffer[1];            //should be 0x21
	id = (id_high << 8) | id_low;   //should be 0x5521

	#ifdef BUILD_LK
		printf("%s, LK debug: nt35521 id = 0x%08x\n", __func__, id);
	#else
		printk("%s, kernel horse debug: nt35521 id = 0x%08x\n", __func__, id);
	#endif

	if(id == LCM_ID_NT35521)
	 	return 1;
	else
		return 0;
	
}

LCM_DRIVER nt35521_hd720_dsi_vdo_lcm_drv =
{
    .name           = "nt35521_hd720_dsi_vdo",
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



