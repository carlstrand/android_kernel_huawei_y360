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

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0xfffe
#define REGFLAG_END_OF_TABLE      							0xffff   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID_OTM1283A                                      0x1283

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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size) 


static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[128];
};

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
				{0x00,1,{0x00}},
				{0xff,3,{0x12,0x83,0x01}},
					  
				{0x00,1,{0x80}},
				{0xff,2,{0x12,0x83}},
		
				{0x00,1,{0x92}},				
				{0xff,2,{0x30,0x02}},	//MIPI 4lane 
		
				{0x00,1,{0xa2}},
				{0xc1,1,{0x08}},
		
				{0x00,1,{0xa4}},
				{0xc1,1,{0xf0}},
		
				{0x00,1,{0x80}},
				{0xc0,9,{0x00,0x64,0x00,0x0f,0x11,0x00,0x64,0x0f,0x11}},
		
				{0x00,1,{0x90}},
				{0xc0,6,{0x00,0x55,0x00,0x01,0x00,0x04}},
				  
				{0x00,1,{0xb3}},
				{0xc0,2,{0x00,0x50}},
					  
				{0x00,1,{0x81}},
				{0xc1,1,{0x66}},
		
				{0x00,1,{0x81}},
				{0xc4,1,{0x83}},
							  
				{0x00,1,{0x82}},
				{0xc4,1,{0x02}},
					 
				{0x00,1,{0x90}},
				{0xc4,1,{0x49}}, 
		
				{0x00,1,{0xb9}},
				{0xb0,1,{0x51}}, 
							  
				{0x00,1,{0xc6}},
				{0xb0,1,{0x03}}, 
				{0x00,1,{0xa4}},
				{0xc0,1,{0x00}}, 
		
				{0x00,1,{0x87}},
				{0xc4,1,{0x18}}, 
		
				{0x00,1,{0xb0}},
				{0xc6,1,{0x03}}, 
					  
				{0x00,1,{0x90}},
				{0xf5,4,{0x02,0x11,0x02,0x11}},
					  
				{0x00,1,{0x90}},
				{0xc5,1,{0x50}},
					  
				{0x00,1,{0x94}},
				{0xc5,1,{0x66}},
					  
				{0x00,1,{0xb2}},
				{0xf5,2,{0x00,0x00}},
		
				{0x00,1,{0xb4}},
				{0xf5,2,{0x00,0x00}},
					  
				{0x00,1,{0xb6}},
				{0xf5,2,{0x00,0x00}},
					  
				{0x00,1,{0xb8}},
				{0xf5,2,{0x00,0x00}},
					  
				{0x00,1,{0x94}},//VCL on
				{0xF5,1,{0x02}},
					  
				{0x00,1,{0xBA}},//VSP on
				{0xF5,1,{0x03}},
					  
				{0x00,1,{0xC3}},//VSP on
				{0xF5,1,{0x81}},
		
				{0x00,1,{0xB4}},//VSP on
				{0xF5,1,{0xC0}},
				
				{0x00,1,{0xa0}},
				{0xC4,14,{0x05,0x10,0x04,0x02,0x05,0x15,0x11,0x05,0x10,0x07,0x02,0x05,0x15,0x11}},
			
				{0x00,1,{0xb0}},
				{0xC4,2,{0x66,0x66}},
					  
				{0x00,1,{0x91}},
				{0xC5,2,{0x4C,0x50}},
					 
				{0x00,1,{0xB0}},
				{0xC5,2,{0x04,0xB8}},
					 
				{0x00,1,{0xB5}},
				{0xc5,6,{0x03,0xe8,0x40,0x03,0xe8,0x40}},
				
				{0x00,1,{0xBB}},
				{0xC5,2,{0x80,0x00}},
				
				{0x00,1,{0x80}},
				{0xcb,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
				{0x00,1,{0x90}},
				{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0xff,0x00}},
				{0x00,1,{0xa0}},
				{0xcb,15,{0xff,0x00,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
				{0x00,1,{0xb0}},
				{0xcb,15,{0x00,0x00,0x00,0xff,0x00,0xff,0x00,0xff,0x00,0xff,0x00,0x00,0x00,0x00,0x00}},
				{0x00,1,{0xc0}},
				{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x05,0x05}},
				{0x00,1,{0xd0}},
				{0xcb,15,{0x05,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
				{0x00,1,{0xe0}},
				{0xcb,14,{0x00,0x00,0x00,0x05,0x00,0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00,0x00}},
				{0x00,1,{0xf0}},
				{0xcb,11,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},
					  
				{0x00,1,{0x80}},
				{0xcc,15,{0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x00,0x10,0x0E}},
				{0x00,1,{0x90}},
				{0xcc,15,{0x0C, 0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
				{0x00,1,{0xa0}},
				{0xcc,14,{0x00, 0x00,0x00,0x09,0x00,0x0F,0x0D,0x0B,0x01,0x03,0x00,0x00,0x00,0x00}},
				{0x00,1,{0xb0}},
				{0xcc,15,{0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x00,0x10,0x0E}},//
				{0x00,1,{0xc0}},
				{0xcc,15,{0x0C, 0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
				{0x00,1,{0xd0}},
				{0xcc,15,{0x00, 0x00,0x00,0x09,0x00,0x0F,0x0D,0x0B,0x01,0x03,0x00,0x00,0x00,0x00}},
					  
				{0x00,1,{0x80}},
				{0xce,12,{0x87, 0x03,0x06,0x86,0x03,0x06,0x85,0x03,0x06,0x84,0x03,0x06}},
				{0x00,1,{0x90}},
				{0xce,14,{0xF0, 0x00,0x00,0xF0,0x00,0x00,0xF0,0x00,0x00,0xF0,0x00,0x00,0x00,0x00}},
				{0x00,1,{0xa0}},
				{0xce,14,{0x38, 0x05,0x84,0xFE,0x00,0x06,0x00,0x38,0x04,0x84,0xFF,0x00,0x06,0x00}},
				{0x00,1,{0xb0}},
				{0xce,14,{0x38, 0x03,0x85,0x00,0x00,0x06,0x00,0x38,0x02,0x85,0x01,0x00,0x06,0x00}},
				{0x00,1,{0xc0}},
				{0xce,14,{0x38, 0x01,0x85,0x02,0x00,0x06,0x00,0x38,0x00,0x85,0x03,0x00,0x06,0x00}},
				{0x00,1,{0xd0}},
				{0xce,14,{0x30, 0x00,0x85,0x04,0x00,0x06,0x00,0x30,0x01,0x85,0x05,0x00,0x06,0x00}},
					  
				{0x00,1,{0x80}},
				{0xcf,14,{0x70, 0x00,0x00,0x10,0x00,0x00,0x00,0x70,0x00,0x00,0x10,0x00,0x00,0x00}},
				{0x00,1,{0x90}},
				{0xcf,14,{0x70, 0x00,0x00,0x10,0x00,0x00,0x00,0x70,0x00,0x00,0x10,0x00,0x00,0x00}},
				{0x00,1,{0xa0}},
				{0xcf,14,{0x70, 0x00,0x00,0x10,0x00,0x00,0x00,0x70,0x00,0x00,0x10,0x00,0x00,0x00}},
				{0x00,1,{0xb0}},
				{0xcf,14,{0x70, 0x00,0x00,0x10,0x00,0x00,0x00,0x70,0x00,0x00,0x10,0x00,0x00,0x00}},
				{0x00,1,{0xc0}},
				{0xcf,11,{0x01, 0x01,0x20,0x20,0x00,0x00,0x01,0x80,0x01,0x03,0x08}},
		   
				{0x00,1,{0x00}},
				{0xD8,2,{0xBC,0xBC}},
					  
				{0x00,1,{0x00}},
				{0xD9,1,{0xA3}},//5A, 6e
					 
		//W_COM(0x00, 0x00);
		//W_COM(0xE1, 0x04, 0x09, 0x0E, 0x0D, 0x06, 0x0F, 0x0A, 0x09, 0x04, 0x07, 0x0D, 0x07, 0x0E, 0x16, 0x10, 0x0A);
		//W_COM(0x00, 0x00);
		//W_COM(0xE2, 0x04, 0x09, 0x0F, 0x0D, 0x06, 0x0F, 0x0A, 0x09, 0x04, 0x07, 0x0D, 0x07, 0x0E, 0x16, 0x10, 0x0A);
				{0x00,1,{0x00}},
				{0xE1,16,{0x04, 0x0a,0x0E,0x0D,0x05,0x0e,0x0A,0x09,0x04,0x07,0x0D,0x07,0x0E,0x16,0x10,0x0A}},
				{0x00,1,{0x00}},
				{0xE2,16,{0x04, 0x0a,0x0F,0x0D,0x05,0x0e,0x0A,0x09,0x04,0x07,0x0D,0x07,0x0E,0x16,0x10,0x0A}},
		
				{0x00,1,{0xA0}},
				{0xD6,12,{0x01, 0x00,0x01,0x00,0x01,0xCD,0x01,0xCD,0x01,0xCD,0x01,0xCD}},
				{0x00,1,{0xB0}},
				{0xD6,12,{0x01, 0xCD,0x01,0xCD,0x01,0xCD,0x01,0xCD,0x01,0xCD,0x01,0xCD}},
				{0x00,1,{0xC0}},
				{0xD6,12,{0x77, 0x11,0x00,0x89,0x11,0x89,0x89,0x11,0x89,0x89,0x11,0x89}},
				{0x00,1,{0xD0}},
				{0xD6,6,{0x89, 0x11,0x89,0x89,0x11,0x89}},
				{0x00,1,{0xE0}},
				{0xD6,12,{0x3C, 0x11,0x44,0x44,0x11,0x44,0x44,0x11,0x44,0x44,0x11,0x44}},
				{0x00,1,{0xF0}},
				{0xD6,6,{0x44, 0x11,0x44,0x44,0x11,0x44}},
				{0x00,1,{0x90}},
				{0xD6,1,{0x80}},
				{0x00,1,{0x81}},			
				{0xD6,2,{0x03,0xFF}},
				
				{0x00,1,{0xB1}},
				{0xC6,1,{0x02}},
				{0x00,1,{0xB4}},
				{0xC6,1,{0x12}},
				
				
				{0x00,1,{0xA0}},
				{0xC1,1,{0x02}},
				{0x00,1,{0x80}},
				{0xC4,1,{0x30}},
				{0x00,1,{0x8A}},
				{0xC4,1,{0x40}},
				{0x00,1,{0x8B}},
				{0xC4,1,{0x00}},
					  
				{0x00,1,{0x00}},
				{0xff,3,{0xff,0xff,0xff}},
		
				{0x55,1,{0x90}},
				
				{0x11,0,{}},
				{REGFLAG_DELAY, 120, {}},
				{0x29,0,{}},
				{REGFLAG_DELAY, 20, {}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
  
  
	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x0, {}}	
 
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


static void init_lcm_registers(void)
{
   unsigned int data_array[16];

    data_array[0] = 0x00043902;                          
    data_array[1] = 0x9483FFB9;                 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x00113902;                          
    data_array[1] = 0x008211BA; //send one param for test
    data_array[2] = 0x1000c516;
    data_array[3] = 0x03240fff;
    data_array[4] = 0x20252421;
    data_array[5] = 0x00000008;
    dsi_set_cmdq(&data_array, 6, 1);
    MDELAY(1);


//0x12,0x82,0x00,0x16,0xC5,0x00,0x10,0xFF,0x0F,0x24,0x03,0x21,0x24,0x25,0x20,0x08



    data_array[0] = 0x00113902;                          
    data_array[1] = 0x040001B1; 
    data_array[2] = 0x11110187;
    data_array[3] = 0x3f3f372f;
    data_array[4] = 0xE6011247;
    data_array[5] = 0x000000E2;
    dsi_set_cmdq(&data_array, 6, 1); 
    MDELAY(1);
    
   // 0x01,0x00,0x04,0x87,0x01,0x11,0x11,0x2F,0x37,0x3F,0x3F,0x47,0x12,0x01,0xE6,0xE2
    
    data_array[0] = 0x00073902;                          
    data_array[1] = 0x08c800B2; 
    data_array[2] = 0x00220004;
    dsi_set_cmdq(&data_array, 3, 1); 
    MDELAY(1);
    
    //0x00,0xC8,0x08,0x04,0x00,0x22

    data_array[0] = 0x00173902;                          
    data_array[1] = 0x320680B4; 
    data_array[2] = 0x15320310; 
    data_array[3] = 0x08103208;
    data_array[4] = 0x05430433;
    data_array[5] = 0x06430437;
    data_array[6] = 0x00066161;
    dsi_set_cmdq(&data_array, 7, 1);
    MDELAY(1);

//0x80,0x06,0x32,0x10,0x03,0x32,0x15,0x08,0x32,0x10,0x08,0x33,0x04,0x43,0x05,0x37,0x04,0x43,0x06,0x61,0x61,0x06
    data_array[0] = 0x00053902;                          
    data_array[1] = 0x100006Bf; 
    data_array[2] = 0x00000004;
    dsi_set_cmdq(&data_array, 3, 1); 
    MDELAY(1);

	//{0xBF,4,{0x06,0x00,0x10,0x04}},
	
	  data_array[0] = 0x00033902;                          
    data_array[1] = 0x00170CC0;
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x00023902;                          
    data_array[1] = 0x00000bB6; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x00023902;                          
    data_array[1] = 0x000032D4; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x00213902;                          
    data_array[1] = 0x000000D5; 
    data_array[2] = 0x01000A00;
    data_array[3] = 0x0000cc00;
    data_array[4] = 0x88888800;
    data_array[5] = 0x88888888;
    data_array[6] = 0x01888888;
    data_array[7] = 0x01234567;
    data_array[8] = 0x88888823;
    data_array[9] = 0x00000088;

    dsi_set_cmdq(&data_array, 10, 1); 
    MDELAY(1);

//0x00,0x00,0x00,0x00,0x0A,0x00,0x01,0x00,0xCC,0x00,0x00,0x00,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x01,0x67,0x45,0x23,0x01,0x23,0x88,0x88,0x88,0x88}},
	
    data_array[0] = 0x00023902;                          
    data_array[1] = 0x000009CC; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x00053902;                          
    data_array[1] = 0x001000C7;
    data_array[2] = 0x00000010;  
    dsi_set_cmdq(&data_array, 3, 1);
    MDELAY(1);

   
//#if using cabc,please open it
/*  data_array[0] = 0x00023902;                          
    data_array[1] = 0x0000FF51; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(5);

    data_array[0] = 0x00023902;                          
    data_array[1] = 0x00002453; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(5);

    data_array[0] = 0x00023902;                          
    data_array[1] = 0x00000155; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(5);*/

//#if use TE function,please open it
/*
    data_array[0] = 0x00023902;                          
    data_array[1] = 0x00000035; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);*/


	
    data_array[0] = 0x002B3902;                          
    data_array[1] = 0x060400E0; 
    data_array[2] = 0x133F332b;                          
    data_array[3] = 0x0D0e0a34;
    data_array[4] = 0x13111311;                          
    data_array[5] = 0x04001710;
    data_array[6] = 0x3F332b06;                          
    data_array[7] = 0x0e0a3413;
    data_array[8] = 0x1113110d;                          
    data_array[9] = 0x0b171013;
    data_array[10] = 0x0b110717;
    data_array[11] = 0x00110717;
    dsi_set_cmdq(&data_array, 12, 1);
    MDELAY(5);

//0x00,0x04,0x06,0x2B,0x33,0x3F,0x13,0x34,0x0A,0x0E,0x0D,0x11,0x13,0x11,0x13,0x10,0x17,0x00,0x04,0x06,0x2B,0x33,0x3F,0x13,0x34,0x0A,0x0E,0x0D,0x11,0x13,0x11,0x13,0x10,0x17,0x0B,0x17,0x07,0x11,0x0B,0x17,0x07,0x11}

    data_array[0] = 0x00023902;                          
    data_array[1] = 0x000032d4; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);
    
    data_array[0] = 0x00110500; // Sleep Out
    dsi_set_cmdq(&data_array, 1, 1); 
    MDELAY(400);

    data_array[0] = 0x00290500; // Display On
    dsi_set_cmdq(&data_array, 1, 1); 
    MDELAY(10);
    
    data_array[0] = 0x00023902;                          
    data_array[1] = 0x000141e4; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);
    
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

    // enable tearing-free
    //params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity			= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;


    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size = 128;

    // Video mode setting		
    params->dsi.intermediat_buffer_num = 0;

    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;


    params->dsi.word_count = 720*3;	
/*
	params->dsi.vertical_sync_active				= 4; // 1
	params->dsi.vertical_backporch					= 16; // 16
	params->dsi.vertical_frontporch 				= 16; // 15
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 90; // 60
	params->dsi.horizontal_backporch				= 90; // 72
	params->dsi.horizontal_frontporch				= 129; // 108
	params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;


    // Bit rate calculation
    params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
    params->dsi.fbk_div =24;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	*/
/////QQQQQ
params->dsi.vertical_sync_active	=2;//
	params->dsi.vertical_backporch		= 8;//
	params->dsi.vertical_frontporch		= 6;//
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 30;//
	params->dsi.horizontal_backporch	= 80;//
	params->dsi.horizontal_frontporch	= 86;//
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;
        //params->dsi.HS_PRPR = 4;
    //    params->dsi.CLK_HS_PRPR = 6;
	// Bit rate calculation
	
	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4
	params->dsi.fbk_div =17;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);

    //init_lcm_registers();
  
  	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    unsigned int data_array[16];

#if 1
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);
#endif

#if 0
    // Display Off
	data_array[0]=0x00280500;
	dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(120);

    // Sleep In
    data_array[0] = 0x00100500;
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(50);
#endif
}


static void lcm_resume(void)
{
    lcm_init();
}


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
    dsi_set_cmdq(&data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(&data_array, 3, 1);

    data_array[0]= 0x00290508; //HW bug, so need send one HS packet
    dsi_set_cmdq(&data_array, 1, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(&data_array, 1, 0);
}

static unsigned int lcm_compare_id(void)
{	
#if 1	    
	unsigned int id = 0;
	unsigned int id1 = 0,id2 = 0,id3 = 0,id4 = 0,id5 = 0,id6 = 0;
	unsigned char buffer[6],buffer1[2];
	unsigned int array[16];

	SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);

	array[0]=0x00023902;
	array[1]=0x00000000;
	dsi_set_cmdq(&array, 2, 1);
	array[0]=0x00043902;
	array[1]=0x010980ff;
	dsi_set_cmdq(&array, 2, 1);
	array[0]=0x00023902;
	array[1]=0x00008000;
	dsi_set_cmdq(&array, 2, 1);
	array[0]=0x00033902;
	array[1]=0x000980ff;
	dsi_set_cmdq(&array, 2, 1);

	array[0] = 0x00053700;// set return byte number
	dsi_set_cmdq(&array, 1, 1);

	read_reg_v2(0xA1, &buffer, 5);
	array[0] = 0x00013700;// set return byte number
	dsi_set_cmdq(&array, 1, 1);
	array[0]=0x00023902;
	array[1]=0x00005000;
	dsi_set_cmdq(&array, 2, 1);
	MDELAY(1);
	read_reg_v2(0xF8, &buffer1, 1);

	id = ((buffer[2]&0xff) << 8) | ((buffer[3]&0xff)) ;
#if defined(BUILD_LK)
	printf("%s, buffer[0]=0x%x,buffer[1]=0x%x,buffer[2]=0x%x,buffer[3]=0x%x buffer[4]=0x%x buffer1[0]=0x%x id = 0x%x\n",
	__func__,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer1[0],id);
#endif
	return (LCM_ID_OTM1283A == id)?1:0;
     
#else
    return 1;
#endif
}
void lcm_debug_func(unsigned char *buffer, int len)
{
    int i = 0;
	unsigned int data_array[16];
	
	if(len/4 == 0 || len%4 != 0)
	{
	   //printk("lcm_debug:data format error\n");
	   return;
	}
	for(i = 0; i < len/4; i++)
	{
	   data_array[i] = buffer[4*i];
	   data_array[i] = (data_array[i]<<8)|buffer[4*i+1];
	   data_array[i] = (data_array[i]<<8)|buffer[4*i+2];
	   data_array[i] = (data_array[i]<<8)|buffer[4*i+3];
	   //printk("lcm_debug:data_array[%d] = %x\n", i, data_array[i]);
	}
	
	dsi_set_cmdq((unsigned int*)data_array, i, 1);
	MDELAY(10);
		
    return;
}

LCM_DRIVER otm1283_hd720_dsi_cmd_drv = 
{
	.name		    = "otm1283a_hd720_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	//.lcm_debug		= lcm_debug_func,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
};
