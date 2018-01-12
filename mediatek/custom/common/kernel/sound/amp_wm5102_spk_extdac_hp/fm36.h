
/* include/linux/i2c/fm36.h - fm36 voice processor driver
 *
 * Copyright (C) 2013 Fortemedia Inc.
 * modified by qianhoulong in 2013/11/12
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */



#ifndef __FM36_PDATA_H__
#define __FM36_PDATA_H__


typedef struct
{
    unsigned char denoise_en;
    unsigned char bypass_en;
    unsigned char voicecall_en;
    unsigned char music_en;
} FM36_CONTROL;

struct fm36_platform_data {
	void (*set_mclk) (bool, bool);
	int gpio_rst;
};
enum Set_Mode_Num {
    fm36_STATE_NONE,
    fm36_STATE_8K_INIT,
    fm36_STATE_SLEEP,
    fm36_STATE_EFFECT_8K_HFNS_ENABLE,
    fm36_STATE_EFFECT_8K_HFNS_DISABLE,
    fm36_STATE_EFFECT_8K_HFFFP,
    fm36_STATE_EFFECT_8K_HSNS_ENABLE,
    fm36_STATE_EFFECT_8K_HSNS_DISABLE,
    fm36_STATE_EFFECT_8K_BLUETOOTH_HANDSET,
    fm36_STATE_EFFECT_8K_BLUETOOTH_HANDFREE,    
    fm36_STATE_EFFECT_8K_MEDIA_RECORD,
    fm36_STATE_EFFECT_8K_BLUETOOTH_HEADSET,
    fm36_STATE_FTM_8K_MIC0TEST,
    fm36_STATE_EFFECT_16K_HFNS_ENABLE,
    fm36_STATE_EFFECT_16K_HFNS_DISABLE,
    fm36_STATE_EFFECT_16K_HFFFP,
    fm36_STATE_EFFECT_16K_HSNS_ENABLE,
    fm36_STATE_EFFECT_16K_HSNS_DISABLE,
    fm36_STATE_EFFECT_16K_MEDIA_RECORD,    
    fm36_STATE_EFFECT_16K_HFVR,
    fm36_STATE_EFFECT_16K_CARKITVR,
    fm36_STATE_EFFECT_16K_VOICEINCALL_HEADPHONE,
    fm36_STATE_EFFECT_32K_MEDIA_RECORD,
    fm36_STATE_FTM_8K_MIC0_TEST, 
    fm36_STATE_FTM_8K_MIC1_TEST, 
    fm36_STATE_FTM_8K_MIC2_TEST, 
    fm36_STATE_FTM_PWD,
    fm36_STATE_FTM_NOPWD
};
  void fm36_reset(void);
int fm36_voice_processor_init(void);
int fm36_voice_processor_8k_hf_noise_suppression(int on);
int fm36_voice_processor_8k_hf_ffp(void);
int fm36_voice_processor_8k_hs_noise_suppression(int on);
int fm36_voice_processor_8k_bths(void);
int fm36_voice_processor_8k_bthf(void);
int fm36_voice_processor_8k_mm_record(void);
int fm36_voice_processor_16k_hf_noise_suppression(int on);
int fm36_voice_processor_16k_hf_ffp(void);
int fm36_voice_processor_16k_hs_noise_suppression(int on);
int fm36_voice_processor_16k_mm_record(void);
int fm36_voice_processor_16k_hfvr(void);
int fm36_voice_processor_16k_carkit_vr(void);
int fm36_voice_processor_32K_mm_record(void);
int  fm36_voice_processor_sleep(void);
int  fm36_voice_processor_mic0_test(void);
int  fm36_voice_processor_mic1_test(void);
int  fm36_voice_processor_mic2_test(void);
int  fm36_voice_processor_Bypass(void);
int  fm36_voice_processor_VoiceInCall_HeadPhone(void);
//static void fm36_para_download(fm36_REG_struct *pPara, int length);
//static int  fm36_patch_cons_download(void);
int fm36_set_init(struct i2c_client *client);
int fm36_set_mode(enum Set_Mode_Num mode);
unsigned char FM36_Register(void);
void fm36_denoise_control(unsigned char mode);
int fm36_write_word(int reg, int data);
int fm36_adb_read(char *para,int length);
int fm36_voice_processor_16k_hs_music(void);
#endif