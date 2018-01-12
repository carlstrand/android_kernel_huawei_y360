/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2009
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


/*******************************************************************************
 *
 * Filename:
 * ---------
 * external_codec_driver.h
 *
 * Project:
 * --------
 *   MT6592_phone_v1
 *
 * Description:
 * ------------
 *   external codec control
 *
 * Author:
 * -------
 *   Stephen Chen
 *
 *
 *------------------------------------------------------------------------------
 * $Revision$
 * $Modtime:$
 * $Log:$
 * *
 *
 *******************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <mach/mt_typedefs.h>

#ifndef _EXTERNAL_CODEC_DRIVER_H_
#define _EXTERNAL_CODEC_DRIVER_H_

/* WM5102 registers addresses */
#define WM5102_CHIPID   0x01    /* Chip ID */
#define WM5102_MODE 0x02    /* Mode Control */
#define WM5102_MIXING   0x03
#define WM5102_MUTE 0x04    /* Mute Control */
#define WM5102_VOLA 0x05    /* DAC Channel A Volume Control */
#define WM5102_VOLB 0x06    /* DAC Channel B Volume Control */
#define WM5102_RAMP 0x07
#define WM5102_MISC1    0x08
#define WM5102_MISC2    0x09

#define WM5102_FIRSTREG 0x01
#define WM5102_LASTREG  0x09
#define WM5102_NUMREGS  (WM5102_LASTREG - WM5102_FIRSTREG + 1)

typedef enum
{
    DIF_LEFT_JUSTIFIED,
    DIF_I2S,
    DIF_RIGHT_JUSTIFIED_16BIT,
    DIF_RIGHT_JUSTIFIED_24BIT,
    DIF_RIGHT_JUSTIFIED_20BIT,
    DIF_RIGHT_JUSTIFIED_18BIT,
    NUM_OF_DIF
} DIGITAL_INTERFACE_FORMAT;
typedef enum
{
    DEVICE_OUT_EARPIECER = 0,
    DEVICE_OUT_EARPIECEL = 1,
    DEVICE_OUT_HEADSETR = 2,
    DEVICE_OUT_HEADSETL = 3,
    DEVICE_OUT_SPEAKERR = 4,
    DEVICE_OUT_SPEAKERL = 5,
    DEVICE_OUT_SPEAKER_HEADSET_R = 6,
    DEVICE_OUT_SPEAKER_HEADSET_L = 7,
    DEVICE_OUT_LINEOUTR = 8,
    DEVICE_OUT_LINEOUTL = 9,
    DEVICE_2IN1_SPK = 10,
    //DEVICE_IN_LINEINR = 11,
    //DEVICE_IN_LINEINL = 12,
    DEVICE_IN_ADC1 = 13,
    DEVICE_IN_ADC2 = 14,
    DEVICE_IN_ADC3 = 15,
    DEVICE_IN_PREAMP_L = 16,
    DEVICE_IN_PREAMP_R = 17,
    DEVICE_IN_DIGITAL_MIC = 18,
    DEVICE_MAX
}DEVICE_TYPE;


enum ECODEC_CONTROL_SUBCOMMAND
{
    ECODEC_GETREGISTER_VALUE,
    ECODEC_SETREGISTER_VALUE,
};

enum AUDIO_ECODEC_CONTROL_COMMAND
{
    NUM_AUD_ECODEC_COMMAND,
};

typedef struct
{
    u8 music_hp_out;
    u8 music_speaker_out;
    u8 music_dualmode_out;
    u8 music_earpiece_out;
    u8 record_mic1_in;
    u8 record_mic2_in;
    u8 record_hpmic_in;
    u8 voicecall_hp_out;
    u8 voicecall_speaker_out;
    u8 voicecall_earpiece_out;
    u8 codec_poweroff_state;
} ECODEC_Control;

typedef struct
{
    u8 main_mic;
    u8 ref_mic;
} ECODEC_Factory_Mic;

void ExtCodec_Init(void);
void ExtCodec_PowerOn(void);
void ExtCodec_PowerOff(u8 mode);
bool ExtCodec_Register(void);
void ExtCodec_Mute(void);
void ExtCodec_SetGain(u8 leftright, u8 gain);
u16 ExtCodec_ReadReg(u32 addr);
void ExtCodec_DumpReg(void);
void ExtCodecDevice_Suspend(void);
void ExtCodecDevice_Resume(void);
void cust_extcodec_gpio_on(void);
void cust_extcodec_gpio_off(void);
void cust_extHPAmp_gpio_on(void);
void cust_extHPAmp_gpio_off(void);
void cust_extPLL_gpio_config(void);
void ExtCodec_Set_Voicecall_Earpiece_Out(void);
void ExtCodec_Set_Voicecall_Headphone_Out(void);
void ExtCodec_Set_Voicecall_Speaker_Out(void);
void ExtCodec_Set_Music_Speaker_Out(void);
void ExtCodec_Set_Mic1_In(void);
void ExtCodec_Set_Mic2_In(void);
void ExtCodec_Set_HpMic_In(void);
void ExtCodec_Set_DualMode_Out(void);
void ExtCodec_Factory_MainMic_En(u8 enable);
void ExtCodec_Factory_RefMic_En(u8 enable);
bool ExtCodec_Get_Factory_MicState(void);
void ExtCodec_Set_Music_Earpiece_Out(void);
void ExtCodec_Set_Voicecall_Speaker_Out_Ftm(void);
#endif
