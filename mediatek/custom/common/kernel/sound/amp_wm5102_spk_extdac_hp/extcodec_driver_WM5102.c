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

/*****************************************************************************
*                E X T E R N A L      R E F E R E N C E S
******************************************************************************
*/
#include <asm/uaccess.h>
#include <linux/xlog.h>
#include <linux/i2c.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
//#include <platform/mt_reg_base.h>
#include <linux/delay.h>

#include "external_codec_driver.h"


#define GPIO_BASE (0x10005000)

/*****************************************************************************
*                          DEBUG INFO
******************************************************************************
*/

static bool ecodec_log_on = true;

#define ECODEC_PRINTK(fmt, arg...) \
    do { \
        if (ecodec_log_on) xlog_printk(ANDROID_LOG_INFO,"ECODEC", "%s() "fmt"\n", __func__,##arg); \
    }while (0)

/*****************************************************************************
*               For I2C definition
******************************************************************************
*/

#define ECODEC_I2C_CHANNEL     (2)        //I2CL: SDA2,SCL2

#define ECODEC_SLAVE_ADDR_WRITE 0x34
#define ECODEC_SLAVE_ADDR_READ  0x35

#define ECODEC_I2C_DEVNAME "WM5102"

static ECODEC_Control wm5102_con;
static ECODEC_Factory_Mic factory_mic;

// I2C variable
static struct i2c_client *new_client = NULL;

// new I2C register method
static const struct i2c_device_id ecodec_i2c_id[] = {{ECODEC_I2C_DEVNAME, 0}, {}};
static struct i2c_board_info __initdata  ecodec_dev = {I2C_BOARD_INFO(ECODEC_I2C_DEVNAME, (ECODEC_SLAVE_ADDR_WRITE >> 1))};

//function declration
static int ecodec_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ecodec_i2c_remove(struct i2c_client *client);
//i2c driver
struct i2c_driver ecodec_i2c_driver =
{
    .probe = ecodec_i2c_probe,
    .remove = ecodec_i2c_remove,
    .driver = {
        .name = ECODEC_I2C_DEVNAME,
    },
    .id_table = ecodec_i2c_id,
};

// function implementation
//read one register
static u16   I2CRead(u32 addr)
{
    u16 regvalue = 0;
    char cmd_buf[4] = {0x00};
    int ret = 0;

    if (!new_client)
    {
        ECODEC_PRINTK("ecodec_read_byte I2C client not initialized!!");
        return -1;
    }
    cmd_buf[0] = (addr >> 24);
    cmd_buf[1] = ((addr >> 16) & 0xFF);
    cmd_buf[2] = ((addr >> 8) & 0xFF);
    cmd_buf[3] = (addr & 0xFF);
    
    ret = i2c_master_send(new_client, cmd_buf, 4);
    if (ret < 0)
    {
        ECODEC_PRINTK("ecodec_read_byte read sends command error!!");
        return -1;
    }
    ret = i2c_master_recv(new_client, cmd_buf, 2);
    if (ret < 0)
    {
        ECODEC_PRINTK("ecodec_read_byte reads recv data error!!");
        return -1;
    }
    regvalue = (cmd_buf[0]<<8)|cmd_buf[1];
    ECODEC_PRINTK("addr 0x%x data 0x%x", addr, regvalue);
    return regvalue;
}

// write register
static ssize_t  I2CWrite(u32 addr, u16 writeData)
{
    char write_data[6] = {0};
    int ret = 0;

    if (!new_client)
    {
        ECODEC_PRINTK("I2C client not initialized!!");
        return -1;
    }
    write_data[0] = ((addr >> 24) & 0xFF);
    write_data[1] = ((addr >> 16) & 0xFF);
    write_data[2] = ((addr >> 8) & 0xFF);
    write_data[3] = (addr & 0xFF);

    write_data[4] = ((writeData >> 8) & 0xFF);
    write_data[5] = (writeData & 0xFF);
    
    ret = i2c_master_send(new_client, write_data, 6);
    if (ret < 0)
    {
        ECODEC_PRINTK("write sends command error!!");
        return -1;
    }
    //ECODEC_PRINTK("addr 0x%x data 0x%x", addr, writeData);
    return 0;
}

static void Wm5102RevBPatchFile08012013(void)
{
    //* ----- ------ -------------------- ------- --------- ------------------------------
    //*  REG   DATA         ACCESS        READ OR  DEVICE
    //* INDEX  VALUE         TYPE          WRITE   ADDRESS  COMMENT (for information only)
    //* ----- ------ -------------------- ------- --------- ------------------------------

    //* Test Keys
    I2CWrite(0x80, 0x0003); //SMbus_32inx_16dat     Write  0x34      * 

    //*Disable control interface IRQ
    I2CWrite(0x081, 0xE022); //SMbus_32inx_16dat     Write  0x34      * 

    //*Configure DAC VOSR
    I2CWrite(0x410, 0x4080); // SMbus_32inx_16dat     Write  0x34      * DAC1 OSR range config
    I2CWrite(0x418, 0x4080); // SMbus_32inx_16dat     Write  0x34      * DAC2 
    I2CWrite(0x420, 0x4080); // SMbus_32inx_16dat     Write  0x34      * DAC3 
    I2CWrite(0x428, 0xC000); // SMbus_32inx_16dat     Write  0x34      * DAC4 


    //* Configure Headphone amp bias  
    I2CWrite(0x4B0, 0x0066); // SMbus_32inx_16dat     Write  0x34      

    //* Noise gate threshold = -86dBFs input
    I2CWrite(0x458, 0x000b); // SMbus_32inx_16dat     Write  0x34

    //* Low power LDO mode
    I2CWrite(0x212, 0x0000); // SMbus_32inx_16dat     Write  0x34

    //* FLL freerun disabled
    I2CWrite(0x171, 0x0000); // SMbus_32inx_16dat     Write  0x34 

    //*Clear test key
    I2CWrite(0x80, 0x0000); // SMbus_32inx_16dat     Write  0x34      * 
}

static void  ecodec_set_hw_parameters(DIGITAL_INTERFACE_FORMAT dif)
{
    ECODEC_PRINTK("ecodec_set_hw_parameters\n");
}

static void  ecodec_set_control_port(u8 enable)
{
    //CPEN
    ECODEC_PRINTK("ecodec_set_control_port (+), enable=%d\n ", enable);
}

static void  ecodec_set_power(u8 enable)
{
    //PDN
    ECODEC_PRINTK("ecodec_set_power(+) enable=%d\n ", enable);
}

static void  ecodec_mute(u8 enable)
{
    //MUTE
    ECODEC_PRINTK("ecodec_mute WM5102_MUTE(+), enable=%d\n ", enable);
}

void ecodec_dump_register(void)
{
    ECODEC_PRINTK("ecodec_dump_register\n");
}

u16 ExtCodec_ReadReg(u32 addr)
{
    ECODEC_PRINTK("ExtCodec_ReadReg\n");
    return I2CRead(addr);
}

static int ecodec_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    volatile u16 reg;
    new_client = client;
    //new_client->timing = 400;
    ECODEC_PRINTK("client timing=%dK ", new_client->timing);

    //Open VGP3
    hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "Audio");
    memset(&wm5102_con, 0, sizeof(ECODEC_Control));
    memset(&factory_mic, 0, sizeof(ECODEC_Factory_Mic));

    cust_extcodec_gpio_on();

    return 0;
}

static int ecodec_i2c_remove(struct i2c_client *client)
{
    ECODEC_PRINTK("ecodec_i2c_remove\n");
    new_client = NULL;
    i2c_unregister_device(client);
    i2c_del_driver(&ecodec_i2c_driver);
    return 0;
}

static int ecodec_register(void)
{
    ECODEC_PRINTK("ecodec_register\n");
    i2c_register_board_info(ECODEC_I2C_CHANNEL, &ecodec_dev, 1);
    if (i2c_add_driver(&ecodec_i2c_driver))
    {
        ECODEC_PRINTK("fail to add device into i2c\n");
        return -1;
    }
    return 0;
}

/*****************************************************************************
*                  F U N C T I O N        D E F I N I T I O N
******************************************************************************
*/
void cust_extcodec_gpio_on()
{
    //external DAC, DAC_PWREN = 1, RST set to HIGH,
    ECODEC_PRINTK("Set GPIO for WM5102 ON\n ");
    mt_set_gpio_mode(GPIO_AUD_EXTDAC_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTDAC_RST_PIN, GPIO_OUT_ZERO); // RST tied lo
    mt_set_gpio_mode(GPIO_AUD_EXTDAC_PWREN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTDAC_PWREN_PIN, GPIO_OUT_ONE);  // power on
    mdelay(10);

    mt_set_gpio_mode(GPIO_AUD_EXTDAC_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTDAC_RST_PIN, GPIO_OUT_ONE); // RST tied high

    memset(&wm5102_con, 0, sizeof(ECODEC_Control));
    //memset(&factory_mic, 0, sizeof(ECODEC_Factory_Mic));

    Wm5102RevBPatchFile08012013();
}

void cust_extcodec_gpio_off()
{
    ECODEC_PRINTK("ExtCodecDevice_Suspend codec_poweroff_state:%d\n",wm5102_con.codec_poweroff_state);
    if(wm5102_con.codec_poweroff_state)
        return;

    I2CWrite(0x400, 0x0000);
    I2CWrite(0x210, 0x00D4);

    mt_set_gpio_mode(GPIO_AUD_EXTDAC_PWREN_PIN, GPIO_MODE_00);
    mt_set_gpio_out(GPIO_AUD_EXTDAC_PWREN_PIN, GPIO_OUT_ZERO);   // power off

    memset(&wm5102_con, 0, sizeof(ECODEC_Control));
    memset(&factory_mic, 0, sizeof(ECODEC_Factory_Mic));

    wm5102_con.codec_poweroff_state = 1;
}

void ExtCodec_Init()
{
    ECODEC_PRINTK("ExtCodec_Init\n ");
}

void ExtCodec_PowerOn(void)
{
    //cust_extcodec_gpio_on();
}

void ExtCodec_PowerOff(u8 mode)
{
    ECODEC_PRINTK("ExtCodec_PowerOff mode:%d\n", mode);

    switch(mode)
    {
        case DEVICE_OUT_EARPIECEL:
            cust_extcodec_gpio_off();
            break;

        case DEVICE_OUT_HEADSETL:
            ECODEC_PRINTK("DEVICE_OUT_HEADSETL music_hp_out:%d,record_hpmic_in:%d\n",
                wm5102_con.music_hp_out,wm5102_con.record_hpmic_in);

            if(wm5102_con.record_hpmic_in)
            {
                wm5102_con.music_hp_out = false;
            }
            else
            {
                cust_extcodec_gpio_off();
            }
            break;

        case DEVICE_OUT_SPEAKERL:
            ECODEC_PRINTK("DEVICE_OUT_SPEAKERL record_mic1_in:%d,music_speaker_out:%d\n",
                wm5102_con.record_mic1_in,wm5102_con.music_speaker_out);

            if(wm5102_con.record_mic1_in)
            {
                wm5102_con.music_speaker_out = false;
            }
            else
            {
                cust_extcodec_gpio_off();
            }
            break;

        case DEVICE_IN_ADC1:
            ECODEC_PRINTK("DEVICE_IN_ADC1 record_mic1_in:%d,music_speaker_out:%d,record_hpmic_in:%d,music_hp_out:%d,music_dualmode_out:%d\n",
                wm5102_con.record_mic1_in,wm5102_con.music_speaker_out,
                wm5102_con.record_hpmic_in,wm5102_con.music_hp_out,wm5102_con.music_dualmode_out);

            if(wm5102_con.music_speaker_out)
            {
                wm5102_con.record_mic1_in = false;
            }
            else if(wm5102_con.music_hp_out)
            {
                wm5102_con.record_hpmic_in = false;
            }
            else if(wm5102_con.music_dualmode_out)
            {
                wm5102_con.music_dualmode_out = false;
            }
            else
            {
                cust_extcodec_gpio_off();
            }
            break;

        default:
            cust_extcodec_gpio_off();
            break;
    }
    
}

bool ExtCodec_Register(void)
{
    ecodec_register();
    return true;
}

void ExtCodec_Mute(void)
{
    ecodec_mute(1);
}

void ExtCodec_SetGain(u8 leftright, u8 gain)
{
    ECODEC_PRINTK("ExtCodec_SetGain record_hpmic_in:%d,music_hp_out:%d\n",
        wm5102_con.record_hpmic_in,wm5102_con.music_hp_out);

    if((wm5102_con.record_hpmic_in)||(wm5102_con.music_hp_out))
    {
        wm5102_con.music_hp_out = true;
        return;
    }
    cust_extcodec_gpio_on();

    I2CWrite(0x102, 0x000b);// SMbus_32inx_16dat     Write  0x34      * 44.1 kHz sample rate set-up

    //FLL 13M to 45.xxxM
    I2CWrite(0x173, 0x1e12);//
    I2CWrite(0x174, 0x1fbd);//
    I2CWrite(0x176, 0x0000);//
    I2CWrite(0x175, 0x0000);//
    I2CWrite(0x179, 0x0010);//
    I2CWrite(0x172, 0x8006);//
    I2CWrite(0x187, 0x0001);//
    I2CWrite(0x171, 0x0001);//
    I2CWrite(0x101, 0x0344);// SMbus_32inx_16dat     Write  0x34      * 45.xxx MHz System Clock enable

    I2CWrite(0x210, 0x00D5);

    I2CWrite(0x500, 0x000B);
    I2CWrite(0x508, 0x1020);
    //***** AIF Enable *****************************************
    I2CWrite(0x504, 0x0002);// SMbus_32inx_16dat     Write  0x34      * AIF1 I2S set-up
    I2CWrite(0x51A, 0x0003);// SMbus_32inx_16dat     Write  0x34      * AIF1 input enables

    //***** Speaker mixer and output enable ********************
    I2CWrite(0x204, 0x8005);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer
    I2CWrite(0x680, 0x0020);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer
    I2CWrite(0x688, 0x0021);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer

    I2CWrite(0x412, 0x0082);// SMbus_32inx_16dat     Write  0x34      * HP left volume
    I2CWrite(0x416, 0x0082);// SMbus_32inx_16dat     Write  0x34      * HP right volume + volume update
    I2CWrite(0x411, 0x0281);// SMbus_32inx_16dat     Write  0x34      * HP left volume
    I2CWrite(0x415, 0x0281);// SMbus_32inx_16dat     Write  0x34      * HP right volume + volume update

    I2CWrite(0x400, 0x0003);// SMbus_32inx_16dat     Write  0x34      * HP enable


    I2CWrite(0x80 , 0x0003); //SMbus_32inx_16dat     Write  0x34      * Ctrl IF Debug 1(80H):    0003  USER_KEY=1, TEST_KEY=1
    I2CWrite(0x100, 0x0042); //SMbus_32inx_16dat     Write  0x34      * 32k clocking
    I2CWrite(0x211, 0x0010); //SMbus_32inx_16dat     Write  0x34      * LDO1 Test Control(211H): 0018  LDO1_MODE=0, LDO1_TST_MODE=00, LDO1_FB_TRIM=110
    I2CWrite(0x200, 0x0005); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x218, 0x01A5); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x219, 0x01A5); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x300, 0x0020); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0001  IN3L_ENA=0, IN3R_ENA=0, IN2L_ENA=0, IN2R_ENA=0, IN1L_ENA=0, IN1R_ENA=1
    I2CWrite(0x507, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 1(507H): 1820  AIF1TX_WL=01_1000, AIF1TX_SLOT_LEN=0010_0000
    //I2CWrite(0x310, 0x2080); //SMbus_32inx_16dat     Write  0x34      * 128, Diff 
    I2CWrite(0x519, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Tx Enables(519H):   0003  AIF1TX8_ENA=0, AIF1TX7_ENA=0, AIF1TX6_ENA=0, AIF1TX5_ENA=0, AIF1TX4_ENA=0, AIF1TX3_ENA=0, AIF1TX2_ENA=1, AIF1TX1_ENA=1
    I2CWrite(0x700, 0x0014); //SMbus_32inx_16dat     Write  0x34      * AIF1TX1MIX Input 1 Source(700H): 0011  AIF1TX1MIX_STS1=0, AIF1TX1MIX_SRC1=IN1R
    I2CWrite(0x708, 0x0014); //SMbus_32inx_16dat     Write  0x34      * AIF1TX2MIX Input 1 Source(708H): 0011  AIF1TX2MIX_STS1=0, AIF1TX2MIX_SRC1=IN1R
    I2CWrite(0x320, 0x20BC); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 1R(315H): 0380  IN_VU=1, IN1R_MUTE=1, IN1R_DIG_VOL=0dB
    I2CWrite(0x321, 0x0280); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 1R(315H): 0380  IN_VU=1, IN1R_MUTE=1, IN1R_DIG_VOL=0dB

    wm5102_con.music_hp_out = true;
}

void ExtCodec_DumpReg(void)
{
    ecodec_dump_register();
}

int ExternalCodec(void)
{
    return 1;
}

void ExtCodecDevice_Suspend(void)
{
    ECODEC_PRINTK("ExtCodecDevice_Suspend\n");

    cust_extcodec_gpio_off();
}

void ExtCodecDevice_Resume(void)
{
    ECODEC_PRINTK("ExtCodecDevice_Resume\n");
    //cust_extcodec_gpio_on();
}

void cust_extHPAmp_gpio_on()
{

}

void cust_extHPAmp_gpio_off()
{
    //external HPAmp, AMP set to disable, gain set to 0dB
}

void cust_extPLL_gpio_config()
{
    //external PLL, set S0/S1
}

void ExtCodec_Set_Music_Earpiece_Out(void)
{
    ECODEC_PRINTK("ExtCodec_Set_Music_Earpiece_Out music_earpiece_out:%d\n",
        wm5102_con.music_earpiece_out);

    if(wm5102_con.music_earpiece_out)
    {
        return;
    }
    cust_extcodec_gpio_on();

    //VoiceCallwithreceiverandIN1L_IN1Rmicin13MAIF116K();
    //* ----- ------ -------------------- ------- --------- ------------------------------
    //*  REG   DATA         ACCESS        READ OR  DEVICE
    //* INDEX  VALUE         TYPE          WRITE   ADDRESS  COMMENT (for information only)
    //* ----- ------ -------------------- ------- --------- ------------------------------

    //*Load psia1_2_to_cdc_aif1_2.txt
    //Load WM5102_RevB_PatchFile_08012013.txt

    //* FLL1 setting,13M MCLK1 in,24.576M out.
    I2CWrite(0x171, 0x0000); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 1(171H):    0000  FLL1_FREERUN=0, FLL1_ENA=0
    I2CWrite(0x172, 0x0007); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 2(172H):    0007  FLL1_CTRL_UPD=0, FLL1_N=00_0000_0111
    I2CWrite(0x173, 0x0391); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 3(173H):    0391  FLL1_THETA=0000_0001_0100_1001
    I2CWrite(0x174, 0x0659); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 4(174H):    0659  FLL1_LAMBDA=0000_0010_0111_0001
    I2CWrite(0x175, 0x0008); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 5(175H):    0006  FLL1_FRATIO=000, FLL1_OUTDIV=4
    I2CWrite(0x176, 0x0000); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 6(176H):    0040  FLL1_REFCLK_DIV=01, FLL1_REFCLK_SRC=0000
    I2CWrite(0x172, 0x8007); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 2(172H):    8007  FLL1_CTRL_UPD=1, FLL1_N=00_0000_0111
    I2CWrite(0x171, 0x0001); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 1(171H):    0001  FLL1_FREERUN=0, FLL1_ENA=1

    //*Configure clock
    I2CWrite(0x101, 0x0244); //SMbus_32inx_16dat     Write  0x34      * System Clock 1(101H):    0140  SYSCLK_FRAC=Integer Rate, SYSCLK_FREQ=12.288MHz (11.2896MHz), SYSCLK_ENA=1, SYSCLK_SRC=MCLK1

    //*set sample rate 
    I2CWrite(0x102, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Sample rate 1(102H):     0003  SAMPLE_RATE_1=48kHz
    I2CWrite(0x103, 0x0012); //SMbus_32inx_16dat     Write  0x34      * Sample rate 2(103H):     0011 SAMPLE_RATE_2=16kHz
    I2CWrite(0x100, 0x0042); //SMbus_32inx_16dat     Write  0x34      * 32k clocking

    //* Enable User & Test Keys
    I2CWrite(0x80, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Ctrl IF Debug 1(80H):    0003  USER_KEY=1, TEST_KEY=1

    //* Configure LDO & CP
    I2CWrite(0x210, 0x00D5); //SMbus_32inx_16dat     Write  0x34      * LDO1 Control 1(210H):    00D5  LDO1_VSEL=00_0110, LDO1_FAST=1, LDO1_DISCH=1, LDO1_BYPASS=0, LDO1_ENA=1 
    I2CWrite(0x200, 0x0007); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x218, 0x01A7); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x204, 0x8005); //SMbus_32inx_16dat     Write  0x34      * HP Charge Pump 1(204H):  8005  STARTUP_SERIES=1, CP_DISCH=1, CP_ENA=1
    I2CWrite(0x208, 0x0041); //SMbus_32inx_16dat     Write  0x34      * disable envelope

    //* Configure AIF1
    I2CWrite(0x500, 0x0008); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x503, 0x0800); //SMbus_32inx_16dat     Write  0x34      * AIF1 Rate Ctrl(503H):    0800  AIF1_RATE=Sample Rate 2, AIF1_TRI=0
    I2CWrite(0x504, 0x0002); //SMbus_32inx_16dat     Write  0x34      * AIF1 Format(504H):       0000  AIF1_FMT=DSP Mode A
    I2CWrite(0x507, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 1(507H): 1020  AIF1TX_WL=16bits, AIF1TX_SLOT_LEN=32cycles
    I2CWrite(0x508, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 2(508H): 1020  AIF1RX_WL=16bits, AIF1RX_SLOT_LEN=32cycles
    I2CWrite(0x519, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Tx Enables(519H):   0003  AIF1TX8_ENA=0, AIF1TX7_ENA=0, AIF1TX6_ENA=0, AIF1TX5_ENA=0, AIF1TX4_ENA=0, AIF1TX3_ENA=0, AIF1TX2_ENA=1, AIF1TX1_ENA=1
    I2CWrite(0x51A, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Rx Enables(51AH):   0003  AIF1RX8_ENA=0, AIF1RX7_ENA=0, AIF1RX6_ENA=0, AIF1RX5_ENA=0, AIF1RX4_ENA=0, AIF1RX3_ENA=0, AIF1RX2_ENA=1, AIF1RX1_ENA=1

    //* Configure mixer core
    I2CWrite(0x6A0, 0x0020); //SMbus_32inx_16dat     Write  0x34      * OUT3LMIX Input 1 Source(6A0H): 0020  OUT3MIX_STS1=0, OUT3MIX_SRC1=AIF1RX1
    I2CWrite(0x6A2, 0x0021); //SMbus_32inx_16dat     Write  0x34      * OUT3LMIX Input 2 Source(6A2H): 0021  OUT3MIX_STS2=0, OUT3MIX_SRC2=AIF1RX2
    I2CWrite(0x700, 0x0010); //SMbus_32inx_16dat     Write  0x34      * AIF1TX1MIX Input 1 Source(700H): 0010  AIF1TX1MIX_STS1=0, AIF1TX1MIX_SRC1=IN1L
    I2CWrite(0x708, 0x0011); //SMbus_32inx_16dat     Write  0x34      * AIF1TX2MIX Input 1 Source(708H): 0011  AIF1TX2MIX_STS1=0, AIF1TX2MIX_SRC1=IN1R

    //* Configure DRE & NG
    I2CWrite(0x458, 0x000B); //SMbus_32inx_16dat     Write  0x34     * NGATE update
    I2CWrite(0x488, 0x0000); //SMbus_32inx_16dat     Write  0x34     * Disable the variable OSR
    I2CWrite(0x440, 0x8030); //SMbus_32inx_16dat     Write  0x34     * DRE enable

    //* Enable channels
    I2CWrite(0x300, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
    I2CWrite(0x308, 0x0800); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
    I2CWrite(0x318, 0x2080); //SMbus_32inx_16dat     Write  0x34      * 128, Diff 
    //I2CWrite(0x22B, 0x0600); //SMbus_32inx_16dat     Write  0x34      * Enable OUT3L channel (DAC)    masked in the datasheet and register map .
    I2CWrite(0x400, 0x0020); //SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 
    I2CWrite(0x408, 0x0800); //SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 

    //* Confgigure Gain
    I2CWrite(0x420, 0xE080); //SMbus_32inx_16dat     Write  0x34      * DAC gain
    I2CWrite(0x421, 0x027C); //SMbus_32inx_16dat     Write  0x34      * DAC Digital Volume 1L(411H): 0280  OUT_VU=1, OUT1L_MUTE=0, OUT1L_VOL=1000_0000
    I2CWrite(0x311, 0x0280); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 2L(319H): 0280  IN_VU=1, IN2L_MUTE=0, IN2L_DIG_VOL=0dB
    I2CWrite(0x310, 0x009E); //SMbus_32inx_16dat     Write  0x34      * 
    I2CWrite(0x315, 0x0280); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x314, 0x009E); //SMbus_32inx_16dat     Write  0x34      * 

    wm5102_con.music_earpiece_out = true;
}

void ExtCodec_Set_Voicecall_Earpiece_Out(void)
{
    ECODEC_PRINTK("ExtCodec_Set_Voicecall_Earpiece_Out voicecall_earpiece_out:%d\n",
        wm5102_con.voicecall_earpiece_out);

    if(wm5102_con.voicecall_earpiece_out)
    {
        return;
    }
    cust_extcodec_gpio_on();

    //VoiceCallwithreceiverandIN1L_IN1Rmicin13MAIF116K();
    //* ----- ------ -------------------- ------- --------- ------------------------------
    //*  REG   DATA         ACCESS        READ OR  DEVICE
    //* INDEX  VALUE         TYPE          WRITE   ADDRESS  COMMENT (for information only)
    //* ----- ------ -------------------- ------- --------- ------------------------------

    //*Load psia1_2_to_cdc_aif1_2.txt
    //Load WM5102_RevB_PatchFile_08012013.txt

    //* FLL1 setting,13M MCLK1 in,24.576M out.
    I2CWrite(0x171, 0x0000); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 1(171H):    0000  FLL1_FREERUN=0, FLL1_ENA=0
    I2CWrite(0x172, 0x0007); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 2(172H):    0007  FLL1_CTRL_UPD=0, FLL1_N=00_0000_0111
    I2CWrite(0x173, 0x0391); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 3(173H):    0391  FLL1_THETA=0000_0001_0100_1001
    I2CWrite(0x174, 0x0659); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 4(174H):    0659  FLL1_LAMBDA=0000_0010_0111_0001
    I2CWrite(0x175, 0x0008); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 5(175H):    0006  FLL1_FRATIO=000, FLL1_OUTDIV=4
    I2CWrite(0x176, 0x0000); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 6(176H):    0040  FLL1_REFCLK_DIV=01, FLL1_REFCLK_SRC=0000
    I2CWrite(0x172, 0x8007); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 2(172H):    8007  FLL1_CTRL_UPD=1, FLL1_N=00_0000_0111
    I2CWrite(0x171, 0x0001); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 1(171H):    0001  FLL1_FREERUN=0, FLL1_ENA=1

    //*Configure clock
    I2CWrite(0x101, 0x0244); //SMbus_32inx_16dat     Write  0x34      * System Clock 1(101H):    0140  SYSCLK_FRAC=Integer Rate, SYSCLK_FREQ=12.288MHz (11.2896MHz), SYSCLK_ENA=1, SYSCLK_SRC=MCLK1

    //*set sample rate 
    I2CWrite(0x102, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Sample rate 1(102H):     0003  SAMPLE_RATE_1=48kHz
    I2CWrite(0x103, 0x0012); //SMbus_32inx_16dat     Write  0x34      * Sample rate 2(103H):     0011 SAMPLE_RATE_2=16kHz
    I2CWrite(0x100, 0x0042); //SMbus_32inx_16dat     Write  0x34      * 32k clocking

    //* Enable User & Test Keys
    I2CWrite(0x80, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Ctrl IF Debug 1(80H):    0003  USER_KEY=1, TEST_KEY=1

    //* Configure LDO & CP
    I2CWrite(0x210, 0x00D5); //SMbus_32inx_16dat     Write  0x34      * LDO1 Control 1(210H):    00D5  LDO1_VSEL=00_0110, LDO1_FAST=1, LDO1_DISCH=1, LDO1_BYPASS=0, LDO1_ENA=1 
    I2CWrite(0x200, 0x0007); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x218, 0x01A7); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x204, 0x8005); //SMbus_32inx_16dat     Write  0x34      * HP Charge Pump 1(204H):  8005  STARTUP_SERIES=1, CP_DISCH=1, CP_ENA=1
    I2CWrite(0x208, 0x0041); //SMbus_32inx_16dat     Write  0x34      * disable envelope

    //* Configure AIF1
    I2CWrite(0x500, 0x0008); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x503, 0x0800); //SMbus_32inx_16dat     Write  0x34      * AIF1 Rate Ctrl(503H):    0800  AIF1_RATE=Sample Rate 2, AIF1_TRI=0
    I2CWrite(0x504, 0x0002); //SMbus_32inx_16dat     Write  0x34      * AIF1 Format(504H):       0000  AIF1_FMT=DSP Mode A
    I2CWrite(0x507, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 1(507H): 1020  AIF1TX_WL=16bits, AIF1TX_SLOT_LEN=32cycles
    I2CWrite(0x508, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 2(508H): 1020  AIF1RX_WL=16bits, AIF1RX_SLOT_LEN=32cycles
    I2CWrite(0x519, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Tx Enables(519H):   0003  AIF1TX8_ENA=0, AIF1TX7_ENA=0, AIF1TX6_ENA=0, AIF1TX5_ENA=0, AIF1TX4_ENA=0, AIF1TX3_ENA=0, AIF1TX2_ENA=1, AIF1TX1_ENA=1
    I2CWrite(0x51A, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Rx Enables(51AH):   0003  AIF1RX8_ENA=0, AIF1RX7_ENA=0, AIF1RX6_ENA=0, AIF1RX5_ENA=0, AIF1RX4_ENA=0, AIF1RX3_ENA=0, AIF1RX2_ENA=1, AIF1RX1_ENA=1

    //* Configure mixer core
    I2CWrite(0x6A0, 0x0020); //SMbus_32inx_16dat     Write  0x34      * OUT3LMIX Input 1 Source(6A0H): 0020  OUT3MIX_STS1=0, OUT3MIX_SRC1=AIF1RX1
    I2CWrite(0x6A2, 0x0021); //SMbus_32inx_16dat     Write  0x34      * OUT3LMIX Input 2 Source(6A2H): 0021  OUT3MIX_STS2=0, OUT3MIX_SRC2=AIF1RX2
    I2CWrite(0x700, 0x0010); //SMbus_32inx_16dat     Write  0x34      * AIF1TX1MIX Input 1 Source(700H): 0010  AIF1TX1MIX_STS1=0, AIF1TX1MIX_SRC1=IN1L
    I2CWrite(0x708, 0x0011); //SMbus_32inx_16dat     Write  0x34      * AIF1TX2MIX Input 1 Source(708H): 0011  AIF1TX2MIX_STS1=0, AIF1TX2MIX_SRC1=IN1R

    //* Configure DRE & NG
    I2CWrite(0x458, 0x000B); //SMbus_32inx_16dat     Write  0x34     * NGATE update
    I2CWrite(0x488, 0x0000); //SMbus_32inx_16dat     Write  0x34     * Disable the variable OSR
    I2CWrite(0x440, 0x8030); //SMbus_32inx_16dat     Write  0x34     * DRE enable

    //* Enable channels
    I2CWrite(0x300, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
    I2CWrite(0x308, 0x0800); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
    I2CWrite(0x318, 0x2080); //SMbus_32inx_16dat     Write  0x34      * 128, Diff 
    //I2CWrite(0x22B, 0x0600); //SMbus_32inx_16dat     Write  0x34      * Enable OUT3L channel (DAC)    masked in the datasheet and register map .
    I2CWrite(0x400, 0x0020); //SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 
    I2CWrite(0x408, 0x0800); //SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 

    //* Confgigure Gain
    I2CWrite(0x420, 0xE080); //SMbus_32inx_16dat     Write  0x34      * DAC gain
    I2CWrite(0x421, 0x027C); //SMbus_32inx_16dat     Write  0x34      * DAC Digital Volume 1L(411H): 0280  OUT_VU=1, OUT1L_MUTE=0, OUT1L_VOL=1000_0000
    I2CWrite(0x311, 0x0280); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 2L(319H): 0280  IN_VU=1, IN2L_MUTE=0, IN2L_DIG_VOL=0dB
    I2CWrite(0x310, 0x009E); //SMbus_32inx_16dat     Write  0x34      * 
    I2CWrite(0x315, 0x0280); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x314, 0x009E); //SMbus_32inx_16dat     Write  0x34      * 

    wm5102_con.voicecall_earpiece_out = true;
}

void ExtCodec_Set_Voicecall_Headphone_Out(void)
{
    ECODEC_PRINTK("ExtCodec_Set_Voicecall_Headphone_Out voicecall_hp_out:%d\n",
        wm5102_con.voicecall_hp_out);

    if(wm5102_con.voicecall_hp_out)
    {
        return;
    }
    cust_extcodec_gpio_on();

    //VoiceCallwithHPandIN3Lmicin13MAIF116K();
    //* ----- ------ -------------------- ------- --------- ------------------------------
    //*  REG   DATA         ACCESS        READ OR  DEVICE
    //* INDEX  VALUE         TYPE          WRITE   ADDRESS  COMMENT (for information only)
    //* ----- ------ -------------------- ------- --------- ------------------------------

    //*Load psia1_2_to_cdc_aif1_2.txt
    //Load WM5102_RevB_PatchFile_08012013.txt

    //* FLL1 setting,13M MCLK1 in,24.576M out.
    I2CWrite(0x171, 0x0000); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 1(171H):    0000  FLL1_FREERUN=0, FLL1_ENA=0
    I2CWrite(0x172, 0x0007); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 2(172H):    0007  FLL1_CTRL_UPD=0, FLL1_N=00_0000_0111
    I2CWrite(0x173, 0x0391); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 3(173H):    0391  FLL1_THETA=0000_0001_0100_1001
    I2CWrite(0x174, 0x0659); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 4(174H):    0659  FLL1_LAMBDA=0000_0010_0111_0001
    I2CWrite(0x175, 0x0008); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 5(175H):    0006  FLL1_FRATIO=000, FLL1_OUTDIV=4
    I2CWrite(0x176, 0x0000); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 6(176H):    0040  FLL1_REFCLK_DIV=01, FLL1_REFCLK_SRC=0000
    I2CWrite(0x172, 0x8007); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 2(172H):    8007  FLL1_CTRL_UPD=1, FLL1_N=00_0000_0111
    I2CWrite(0x171, 0x0001); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 1(171H):    0001  FLL1_FREERUN=0, FLL1_ENA=1

    //* Configure clock
    I2CWrite(0x101, 0x0244); //SMbus_32inx_16dat     Write  0x34      * System Clock 1(101H):    0140  SYSCLK_FRAC=Integer Rate, SYSCLK_FREQ=12.288MHz (11.2896MHz), SYSCLK_ENA=1, SYSCLK_SRC=MCLK1

    //*set sample rate 
    I2CWrite(0x102, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Sample rate 1(102H):     0003  SAMPLE_RATE_1=48kHz
    I2CWrite(0x103, 0x0012); //SMbus_32inx_16dat     Write  0x34      * Sample rate 2(103H):     0011 SAMPLE_RATE_2=16kHz
    I2CWrite(0x100, 0x0042); //SMbus_32inx_16dat     Write  0x34      * 32k clocking

    //* Enable User & Test Keys
    I2CWrite(0x80, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Ctrl IF Debug 1(80H):    0003  USER_KEY=1, TEST_KEY=1

    //* Configure LDO & CP
    I2CWrite(0x210, 0x00D5); //SMbus_32inx_16dat     Write  0x34      * LDO1 Control 1(210H):    00D5  LDO1_VSEL=00_0110, LDO1_FAST=1, LDO1_DISCH=1, LDO1_BYPASS=0, LDO1_ENA=1 
    I2CWrite(0x200, 0x0007); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x219, 0x01A7); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x204, 0x8005); //SMbus_32inx_16dat     Write  0x34      * HP Charge Pump 1(204H):  8005  STARTUP_SERIES=1, CP_DISCH=1, CP_ENA=1
    I2CWrite(0x208, 0x0041); //SMbus_32inx_16dat     Write  0x34      * disable envelope

    //* Configure AIF1
    I2CWrite(0x500, 0x0008); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x503, 0x0800); //SMbus_32inx_16dat     Write  0x34      * AIF1 Rate Ctrl(503H):    0800  AIF1_RATE=Sample Rate 2, AIF1_TRI=0
    I2CWrite(0x504, 0x0002); //SMbus_32inx_16dat     Write  0x34      * AIF1 Format(504H):       0000  AIF1_FMT=DSP Mode A
    I2CWrite(0x507, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 1(507H): 1020  AIF1TX_WL=16bits, AIF1TX_SLOT_LEN=32cycles
    I2CWrite(0x508, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 2(508H): 1020  AIF1RX_WL=16bits, AIF1RX_SLOT_LEN=32cycles
    I2CWrite(0x519, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Tx Enables(519H):   0003  AIF1TX8_ENA=0, AIF1TX7_ENA=0, AIF1TX6_ENA=0, AIF1TX5_ENA=0, AIF1TX4_ENA=0, AIF1TX3_ENA=0, AIF1TX2_ENA=1, AIF1TX1_ENA=1
    I2CWrite(0x51A, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Rx Enables(51AH):   0003  AIF1RX8_ENA=0, AIF1RX7_ENA=0, AIF1RX6_ENA=0, AIF1RX5_ENA=0, AIF1RX4_ENA=0, AIF1RX3_ENA=0, AIF1RX2_ENA=1, AIF1RX1_ENA=1

    //* Configure mixer core
    I2CWrite(0x680, 0x0020); //SMbus_32inx_16dat     Write  0x34      * OUT3LMIX Input 1 Source(6A0H): 0028  AIF1RX1 to receiver (Out3)
    I2CWrite(0x688, 0x0021); //SMbus_32inx_16dat     Write  0x34      * OUT3LMIX Input 2 Source(6A2H): 0029  AIF1RX2 to receiver (Out3)
    I2CWrite(0x700, 0x0014); //SMbus_32inx_16dat     Write  0x34      * AIF2TX1MIX Input 1 Source(740H): 0012  IN2L to AIF2TX1 
    I2CWrite(0x708, 0x0014); //SMbus_32inx_16dat     Write  0x34      * AIF2TX2MIX Input 1 Source(748H): 0012  IN2L to AIF2TX2 

    //* Configure DRE & NG
    I2CWrite(0x458, 0x000B); //SMbus_32inx_16dat     Write  0x34     * NGATE update
    I2CWrite(0x488, 0x0000); //SMbus_32inx_16dat     Write  0x34     * Disable the variable OSR
    I2CWrite(0x440, 0x8030); //SMbus_32inx_16dat     Write  0x34     * DRE enable

    //* Enable channels
    I2CWrite(0x300, 0x0020); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN3L_ENA=1
    I2CWrite(0x308, 0x0800); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x320, 0x009E); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
    I2CWrite(0x321, 0x0280); //SMbus_32inx_16dat     Write  0x34      * 128, Diff 
    //I2CWrite(0x22B, 0x0600); //SMbus_32inx_16dat     Write  0x34      * Enable OUT3L channel (DAC)    masked in the datasheet and register map .
    I2CWrite(0x400, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 
    I2CWrite(0x408, 0x0800); //SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 

    //* Confgigure Gain
    I2CWrite(0x410, 0x6080); //SMbus_32inx_16dat     Write  0x34      * DAC gain
    I2CWrite(0x411, 0x027C); //SMbus_32inx_16dat     Write  0x34      * DAC Digital Volume 1L(411H): 0280  OUT_VU=1, OUT1L_MUTE=0, OUT1L_VOL=1000_0000
    I2CWrite(0x415, 0x027C); //SMbus_32inx_16dat     Write  0x34      * DAC Digital Volume 1L(411H): 0280  OUT_VU=1, OUT1L_MUTE=0, OUT1L_VOL=1000_0000
    //I2CWrite(0x315, 0x0380); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 1R(315H): 0380  IN_VU=1, IN1R_MUTE=1, IN1R_DIG_VOL=0dB

    wm5102_con.voicecall_hp_out = true;
}

void ExtCodec_Set_Voicecall_Speaker_Out(void)
{
    ECODEC_PRINTK("ExtCodec_Set_Voicecall_Speaker_Out, voicecall_speaker_out:%d\n",
        wm5102_con.voicecall_speaker_out);

    if(wm5102_con.voicecall_speaker_out)
    {
        return;
    }
    cust_extcodec_gpio_on();

    //VoiceCallwithSPKLandIN1L_IN1Rmicin13MAIF116K();
    //* ----- ------ -------------------- ------- --------- ------------------------------
    //*  REG   DATA         ACCESS        READ OR  DEVICE
    //* INDEX  VALUE         TYPE          WRITE   ADDRESS  COMMENT (for information only)
    //* ----- ------ -------------------- ------- --------- ------------------------------

    //*Load psia1_2_to_cdc_aif1_2.txt
    //Load WM5102_RevB_PatchFile_08012013.txt

    //* FLL1 setting,13M MCLK1 in,24.576M out.
    I2CWrite(0x171, 0x0000); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 1(171H):    0000  FLL1_FREERUN=0, FLL1_ENA=0
    I2CWrite(0x172, 0x0007); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 2(172H):    0007  FLL1_CTRL_UPD=0, FLL1_N=00_0000_0111
    I2CWrite(0x173, 0x0391); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 3(173H):    0391  FLL1_THETA=0000_0001_0100_1001
    I2CWrite(0x174, 0x0659); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 4(174H):    0659  FLL1_LAMBDA=0000_0010_0111_0001
    I2CWrite(0x175, 0x0008); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 5(175H):    0006  FLL1_FRATIO=000, FLL1_OUTDIV=4
    I2CWrite(0x176, 0x0000); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 6(176H):    0040  FLL1_REFCLK_DIV=01, FLL1_REFCLK_SRC=0000
    I2CWrite(0x172, 0x8007); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 2(172H):    8007  FLL1_CTRL_UPD=1, FLL1_N=00_0000_0111
    I2CWrite(0x171, 0x0001); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 1(171H):    0001  FLL1_FREERUN=0, FLL1_ENA=1

    //*Configure clock
    I2CWrite(0x101, 0x0244); //SMbus_32inx_16dat     Write  0x34      * System Clock 1(101H):    0140  SYSCLK_FRAC=Integer Rate, SYSCLK_FREQ=12.288MHz (11.2896MHz), SYSCLK_ENA=1, SYSCLK_SRC=MCLK1

    //*set sample rate 
    I2CWrite(0x102, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Sample rate 1(102H):     0003  SAMPLE_RATE_1=48kHz
    I2CWrite(0x103, 0x0012); //SMbus_32inx_16dat     Write  0x34      * Sample rate 2(103H):     0011 SAMPLE_RATE_2=16kHz
    I2CWrite(0x100, 0x0042); //SMbus_32inx_16dat     Write  0x34      * 32k clocking

    //* Enable User & Test Keys
    I2CWrite(0x80, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Ctrl IF Debug 1(80H):    0003  USER_KEY=1, TEST_KEY=1

    //* Configure LDO & CP
    I2CWrite(0x210, 0x00D5); //SMbus_32inx_16dat     Write  0x34      * LDO1 Control 1(210H):    00D5  LDO1_VSEL=00_0110, LDO1_FAST=1, LDO1_DISCH=1, LDO1_BYPASS=0, LDO1_ENA=1 
    I2CWrite(0x200, 0x0007); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x218, 0x01A7); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x204, 0x8005); //SMbus_32inx_16dat     Write  0x34      * HP Charge Pump 1(204H):  8005  STARTUP_SERIES=1, CP_DISCH=1, CP_ENA=1
    I2CWrite(0x208, 0x0041); //SMbus_32inx_16dat     Write  0x34      * disable envelope

    //* Configure AIF1
    I2CWrite(0x500, 0x0008); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x503, 0x0800); //SMbus_32inx_16dat     Write  0x34      * AIF1 Rate Ctrl(503H):    0800  AIF1_RATE=Sample Rate 2, AIF1_TRI=0
    I2CWrite(0x504, 0x0002); //SMbus_32inx_16dat     Write  0x34      * AIF1 Format(504H):       0000  AIF1_FMT=DSP Mode A
    I2CWrite(0x507, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 1(507H): 1020  AIF1TX_WL=16bits, AIF1TX_SLOT_LEN=32cycles
    I2CWrite(0x508, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 2(508H): 1020  AIF1RX_WL=16bits, AIF1RX_SLOT_LEN=32cycles
    I2CWrite(0x519, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Tx Enables(519H):   0003  AIF1TX8_ENA=0, AIF1TX7_ENA=0, AIF1TX6_ENA=0, AIF1TX5_ENA=0, AIF1TX4_ENA=0, AIF1TX3_ENA=0, AIF1TX2_ENA=1, AIF1TX1_ENA=1
    I2CWrite(0x51A, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Rx Enables(51AH):   0003  AIF1RX8_ENA=0, AIF1RX7_ENA=0, AIF1RX6_ENA=0, AIF1RX5_ENA=0, AIF1RX4_ENA=0, AIF1RX3_ENA=0, AIF1RX2_ENA=1, AIF1RX1_ENA=1

    //* Configure mixer core
    I2CWrite(0x6B0, 0x0020); //SMbus_32inx_16dat     Write  0x34      * OUT1LMIX Input 1 Source(680H): 0020  OUT1LMIX_STS1=0, OUT1LMIX_SRC1=0010_0000
    I2CWrite(0x6B1, 0x0074); //SMbus_32inx_16dat     Write  0x34      * OUT4LMIX Input 1 Volume(6B1H): 0074  OUT4LMIX_VOL1=-6dB
    I2CWrite(0x6B2, 0x0021); //SMbus_32inx_16dat     Write  0x34      * OUT4LMIX Input 2 Source(6B2H): 0021  OUT4LMIX_STS2=0, OUT4LMIX_SRC2=AIF1RX2
    I2CWrite(0x6B3, 0x0074); //SMbus_32inx_16dat     Write  0x34      * OUT4LMIX Input 2 Volume(6B3H): 0074  OUT4LMIX_VOL2=-6dB
    I2CWrite(0x700, 0x0010); //SMbus_32inx_16dat     Write  0x34      * AIF1TX1MIX Input 1 Source(700H): 0010  AIF1TX1MIX_STS1=0, AIF1TX1MIX_SRC1=IN1L
    I2CWrite(0x708, 0x0011); //SMbus_32inx_16dat     Write  0x34      * AIF1TX2MIX Input 1 Source(708H): 0011  AIF1TX2MIX_STS1=0, AIF1TX2MIX_SRC1=IN1R

    //* Configure DRE & NG
    I2CWrite(0x458, 0x000B); //SMbus_32inx_16dat     Write  0x34     * NGATE update
    I2CWrite(0x488, 0x0000); //SMbus_32inx_16dat     Write  0x34     * Disable the variable OSR
    I2CWrite(0x440, 0x8030); //SMbus_32inx_16dat     Write  0x34     * DRE enable

    //* Enable channels
    I2CWrite(0x300, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
    I2CWrite(0x308, 0x0800); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
    I2CWrite(0x318, 0x2080); //SMbus_32inx_16dat     Write  0x34      * 128, Diff 
    //I2CWrite(0x22B, 0x0600); //SMbus_32inx_16dat     Write  0x34      * Enable OUT3L channel (DAC)    masked in the datasheet and register map .
    I2CWrite(0x400, 0x0080); // SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 
    I2CWrite(0x408, 0x0800); // SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 
    //* Confgigure Gain
    I2CWrite(0x429, 0x0280); //SMbus_32inx_16dat     Write  0x34      * DAC Digital Volume 1L(411H): 0280  OUT_VU=1, OUT1L_MUTE=0, OUT1L_VOL=1000_0000
    I2CWrite(0x311, 0x0280); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 2L(319H): 0280  IN_VU=1, IN2L_MUTE=0, IN2L_DIG_VOL=0dB
    I2CWrite(0x310, 0x009E); //SMbus_32inx_16dat     Write  0x34      * 
    I2CWrite(0x315, 0x0280); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x314, 0x009E); //SMbus_32inx_16dat     Write  0x34      * 

    wm5102_con.voicecall_speaker_out = true;
}

void ExtCodec_Set_Voicecall_Speaker_Out_Ftm(void)
{
    ECODEC_PRINTK("ExtCodec_Set_Voicecall_Speaker_Out_Ftm, factory main_mic:%d,ref_mic:%d\n",
        factory_mic.main_mic,factory_mic.ref_mic);

    cust_extcodec_gpio_on();

    //VoiceCallwithSPKLandIN1L_IN1Rmicin13MAIF116K();
    //* ----- ------ -------------------- ------- --------- ------------------------------
    //*  REG   DATA         ACCESS        READ OR  DEVICE
    //* INDEX  VALUE         TYPE          WRITE   ADDRESS  COMMENT (for information only)
    //* ----- ------ -------------------- ------- --------- ------------------------------

    //*Load psia1_2_to_cdc_aif1_2.txt
    //Load WM5102_RevB_PatchFile_08012013.txt

    //* FLL1 setting,13M MCLK1 in,24.576M out.
    I2CWrite(0x171, 0x0000); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 1(171H):    0000  FLL1_FREERUN=0, FLL1_ENA=0
    I2CWrite(0x172, 0x0007); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 2(172H):    0007  FLL1_CTRL_UPD=0, FLL1_N=00_0000_0111
    I2CWrite(0x173, 0x0391); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 3(173H):    0391  FLL1_THETA=0000_0001_0100_1001
    I2CWrite(0x174, 0x0659); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 4(174H):    0659  FLL1_LAMBDA=0000_0010_0111_0001
    I2CWrite(0x175, 0x0008); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 5(175H):    0006  FLL1_FRATIO=000, FLL1_OUTDIV=4
    I2CWrite(0x176, 0x0000); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 6(176H):    0040  FLL1_REFCLK_DIV=01, FLL1_REFCLK_SRC=0000
    I2CWrite(0x172, 0x8007); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 2(172H):    8007  FLL1_CTRL_UPD=1, FLL1_N=00_0000_0111
    I2CWrite(0x171, 0x0001); //SMbus_32inx_16dat     Write  0x34      * FLL1 Control 1(171H):    0001  FLL1_FREERUN=0, FLL1_ENA=1

    //*Configure clock
    I2CWrite(0x101, 0x0244); //SMbus_32inx_16dat     Write  0x34      * System Clock 1(101H):    0140  SYSCLK_FRAC=Integer Rate, SYSCLK_FREQ=12.288MHz (11.2896MHz), SYSCLK_ENA=1, SYSCLK_SRC=MCLK1

    //*set sample rate 
    I2CWrite(0x102, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Sample rate 1(102H):     0003  SAMPLE_RATE_1=48kHz
    I2CWrite(0x103, 0x0012); //SMbus_32inx_16dat     Write  0x34      * Sample rate 2(103H):     0011 SAMPLE_RATE_2=16kHz
    I2CWrite(0x100, 0x0042); //SMbus_32inx_16dat     Write  0x34      * 32k clocking

    //* Enable User & Test Keys
    I2CWrite(0x80, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Ctrl IF Debug 1(80H):    0003  USER_KEY=1, TEST_KEY=1

    //* Configure LDO & CP
    I2CWrite(0x210, 0x00D5); //SMbus_32inx_16dat     Write  0x34      * LDO1 Control 1(210H):    00D5  LDO1_VSEL=00_0110, LDO1_FAST=1, LDO1_DISCH=1, LDO1_BYPASS=0, LDO1_ENA=1 
    I2CWrite(0x200, 0x0007); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x218, 0x01A7); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x204, 0x8005); //SMbus_32inx_16dat     Write  0x34      * HP Charge Pump 1(204H):  8005  STARTUP_SERIES=1, CP_DISCH=1, CP_ENA=1
    I2CWrite(0x208, 0x0041); //SMbus_32inx_16dat     Write  0x34      * disable envelope

    //* Configure AIF1
    I2CWrite(0x500, 0x0008); //SMbus_32inx_16dat     Write  0x34      *
    I2CWrite(0x503, 0x0800); //SMbus_32inx_16dat     Write  0x34      * AIF1 Rate Ctrl(503H):    0800  AIF1_RATE=Sample Rate 2, AIF1_TRI=0
    I2CWrite(0x504, 0x0002); //SMbus_32inx_16dat     Write  0x34      * AIF1 Format(504H):       0000  AIF1_FMT=DSP Mode A
    I2CWrite(0x507, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 1(507H): 1020  AIF1TX_WL=16bits, AIF1TX_SLOT_LEN=32cycles
    I2CWrite(0x508, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 2(508H): 1020  AIF1RX_WL=16bits, AIF1RX_SLOT_LEN=32cycles
    I2CWrite(0x519, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Tx Enables(519H):   0003  AIF1TX8_ENA=0, AIF1TX7_ENA=0, AIF1TX6_ENA=0, AIF1TX5_ENA=0, AIF1TX4_ENA=0, AIF1TX3_ENA=0, AIF1TX2_ENA=1, AIF1TX1_ENA=1
    I2CWrite(0x51A, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Rx Enables(51AH):   0003  AIF1RX8_ENA=0, AIF1RX7_ENA=0, AIF1RX6_ENA=0, AIF1RX5_ENA=0, AIF1RX4_ENA=0, AIF1RX3_ENA=0, AIF1RX2_ENA=1, AIF1RX1_ENA=1

    if(factory_mic.main_mic)
    {
        //* Configure mixer core
        I2CWrite(0x6B0, 0x0020); //SMbus_32inx_16dat     Write  0x34      * OUT1LMIX Input 1 Source(680H): 0020  OUT1LMIX_STS1=0, OUT1LMIX_SRC1=0010_0000
        I2CWrite(0x6B1, 0x0074); //SMbus_32inx_16dat     Write  0x34      * OUT4LMIX Input 1 Volume(6B1H): 0074  OUT4LMIX_VOL1=-6dB
        I2CWrite(0x6B2, 0x0021); //SMbus_32inx_16dat     Write  0x34      * OUT4LMIX Input 2 Source(6B2H): 0021  OUT4LMIX_STS2=0, OUT4LMIX_SRC2=AIF1RX2
        I2CWrite(0x6B3, 0x0074); //SMbus_32inx_16dat     Write  0x34      * OUT4LMIX Input 2 Volume(6B3H): 0074  OUT4LMIX_VOL2=-6dB
        I2CWrite(0x700, 0x0010); //SMbus_32inx_16dat     Write  0x34      * AIF1TX1MIX Input 1 Source(700H): 0010  AIF1TX1MIX_STS1=0, AIF1TX1MIX_SRC1=IN1L
        I2CWrite(0x708, 0x0010); //SMbus_32inx_16dat     Write  0x34      * AIF1TX2MIX Input 1 Source(708H): 0011  AIF1TX2MIX_STS1=0, AIF1TX2MIX_SRC1=IN1R

        //* Configure DRE & NG
        I2CWrite(0x458, 0x000B); //SMbus_32inx_16dat     Write  0x34     * NGATE update
        I2CWrite(0x488, 0x0000); //SMbus_32inx_16dat     Write  0x34     * Disable the variable OSR
        I2CWrite(0x440, 0x8030); //SMbus_32inx_16dat     Write  0x34     * DRE enable

        //* Enable channels
        I2CWrite(0x300, 0x0002); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
        I2CWrite(0x308, 0x0800); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
        I2CWrite(0x318, 0x2080); //SMbus_32inx_16dat     Write  0x34      * 128, Diff 
        //I2CWrite(0x22B, 0x0600); //SMbus_32inx_16dat     Write  0x34      * Enable OUT3L channel (DAC)    masked in the datasheet and register map .
        I2CWrite(0x400, 0x0080); // SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 
        I2CWrite(0x408, 0x0800); // SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 
        //* Confgigure Gain
        I2CWrite(0x429, 0x0280); //SMbus_32inx_16dat     Write  0x34      * DAC Digital Volume 1L(411H): 0280  OUT_VU=1, OUT1L_MUTE=0, OUT1L_VOL=1000_0000
        I2CWrite(0x311, 0x0280); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 2L(319H): 0280  IN_VU=1, IN2L_MUTE=0, IN2L_DIG_VOL=0dB
        I2CWrite(0x310, 0x009E); //SMbus_32inx_16dat     Write  0x34      * 
        I2CWrite(0x315, 0x0380); //SMbus_32inx_16dat     Write  0x34      *
        I2CWrite(0x314, 0x009E); //SMbus_32inx_16dat     Write  0x34      * 
    }
    else if(factory_mic.ref_mic)
    {
        //* Configure mixer core
        I2CWrite(0x6B0, 0x0020); //SMbus_32inx_16dat     Write  0x34      * OUT1LMIX Input 1 Source(680H): 0020  OUT1LMIX_STS1=0, OUT1LMIX_SRC1=0010_0000
        I2CWrite(0x6B1, 0x0074); //SMbus_32inx_16dat     Write  0x34      * OUT4LMIX Input 1 Volume(6B1H): 0074  OUT4LMIX_VOL1=-6dB
        I2CWrite(0x6B2, 0x0021); //SMbus_32inx_16dat     Write  0x34      * OUT4LMIX Input 2 Source(6B2H): 0021  OUT4LMIX_STS2=0, OUT4LMIX_SRC2=AIF1RX2
        I2CWrite(0x6B3, 0x0074); //SMbus_32inx_16dat     Write  0x34      * OUT4LMIX Input 2 Volume(6B3H): 0074  OUT4LMIX_VOL2=-6dB
        I2CWrite(0x700, 0x0011); //SMbus_32inx_16dat     Write  0x34      * AIF1TX1MIX Input 1 Source(700H): 0010  AIF1TX1MIX_STS1=0, AIF1TX1MIX_SRC1=IN1L
        I2CWrite(0x708, 0x0011); //SMbus_32inx_16dat     Write  0x34      * AIF1TX2MIX Input 1 Source(708H): 0011  AIF1TX2MIX_STS1=0, AIF1TX2MIX_SRC1=IN1R

        //* Configure DRE & NG
        I2CWrite(0x458, 0x000B); //SMbus_32inx_16dat     Write  0x34     * NGATE update
        I2CWrite(0x488, 0x0000); //SMbus_32inx_16dat     Write  0x34     * Disable the variable OSR
        I2CWrite(0x440, 0x8030); //SMbus_32inx_16dat     Write  0x34     * DRE enable

        //* Enable channels
        I2CWrite(0x300, 0x0001); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
        I2CWrite(0x308, 0x0800); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
        I2CWrite(0x318, 0x2080); //SMbus_32inx_16dat     Write  0x34      * 128, Diff 
        //I2CWrite(0x22B, 0x0600); //SMbus_32inx_16dat     Write  0x34      * Enable OUT3L channel (DAC)    masked in the datasheet and register map .
        I2CWrite(0x400, 0x0080); // SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 
        I2CWrite(0x408, 0x0800); // SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 
        //* Confgigure Gain
        I2CWrite(0x429, 0x0280); //SMbus_32inx_16dat     Write  0x34      * DAC Digital Volume 1L(411H): 0280  OUT_VU=1, OUT1L_MUTE=0, OUT1L_VOL=1000_0000
        I2CWrite(0x311, 0x0380); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 2L(319H): 0280  IN_VU=1, IN2L_MUTE=0, IN2L_DIG_VOL=0dB
        I2CWrite(0x310, 0x009E); //SMbus_32inx_16dat     Write  0x34      * 
        I2CWrite(0x315, 0x0280); //SMbus_32inx_16dat     Write  0x34      *
        I2CWrite(0x314, 0x009E); //SMbus_32inx_16dat     Write  0x34      * 
    }
    else
    {
        //* Configure mixer core
        I2CWrite(0x6B0, 0x0020); //SMbus_32inx_16dat     Write  0x34      * OUT1LMIX Input 1 Source(680H): 0020  OUT1LMIX_STS1=0, OUT1LMIX_SRC1=0010_0000
        I2CWrite(0x6B1, 0x0074); //SMbus_32inx_16dat     Write  0x34      * OUT4LMIX Input 1 Volume(6B1H): 0074  OUT4LMIX_VOL1=-6dB
        I2CWrite(0x6B2, 0x0021); //SMbus_32inx_16dat     Write  0x34      * OUT4LMIX Input 2 Source(6B2H): 0021  OUT4LMIX_STS2=0, OUT4LMIX_SRC2=AIF1RX2
        I2CWrite(0x6B3, 0x0074); //SMbus_32inx_16dat     Write  0x34      * OUT4LMIX Input 2 Volume(6B3H): 0074  OUT4LMIX_VOL2=-6dB
        I2CWrite(0x700, 0x0010); //SMbus_32inx_16dat     Write  0x34      * AIF1TX1MIX Input 1 Source(700H): 0010  AIF1TX1MIX_STS1=0, AIF1TX1MIX_SRC1=IN1L
        I2CWrite(0x708, 0x0011); //SMbus_32inx_16dat     Write  0x34      * AIF1TX2MIX Input 1 Source(708H): 0011  AIF1TX2MIX_STS1=0, AIF1TX2MIX_SRC1=IN1R

        //* Configure DRE & NG
        I2CWrite(0x458, 0x000B); //SMbus_32inx_16dat     Write  0x34     * NGATE update
        I2CWrite(0x488, 0x0000); //SMbus_32inx_16dat     Write  0x34     * Disable the variable OSR
        I2CWrite(0x440, 0x8030); //SMbus_32inx_16dat     Write  0x34     * DRE enable

        //* Enable channels
        I2CWrite(0x300, 0x0003); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
        I2CWrite(0x308, 0x0800); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0008   IN2L_ENA=1
        I2CWrite(0x318, 0x2080); //SMbus_32inx_16dat     Write  0x34      * 128, Diff 
        //I2CWrite(0x22B, 0x0600); //SMbus_32inx_16dat     Write  0x34      * Enable OUT3L channel (DAC)    masked in the datasheet and register map .
        I2CWrite(0x400, 0x0080); // SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 
        I2CWrite(0x408, 0x0800); // SMbus_32inx_16dat     Write  0x34      * Output Enables 1(400H):  0020  , enable Out3 
        //* Confgigure Gain
        I2CWrite(0x429, 0x0280); //SMbus_32inx_16dat     Write  0x34      * DAC Digital Volume 1L(411H): 0280  OUT_VU=1, OUT1L_MUTE=0, OUT1L_VOL=1000_0000
        I2CWrite(0x311, 0x0280); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 2L(319H): 0280  IN_VU=1, IN2L_MUTE=0, IN2L_DIG_VOL=0dB
        I2CWrite(0x310, 0x009E); //SMbus_32inx_16dat     Write  0x34      * 
        I2CWrite(0x315, 0x0280); //SMbus_32inx_16dat     Write  0x34      *
        I2CWrite(0x314, 0x009E); //SMbus_32inx_16dat     Write  0x34      * 
    }
}

void ExtCodec_Set_Music_Speaker_Out(void)
{
    ECODEC_PRINTK("ExtCodec_Set_Music_Speaker_Out music_speaker_out:%d,record_mic1_in:%d\n",
        wm5102_con.music_speaker_out,wm5102_con.record_mic1_in);

    if((wm5102_con.music_speaker_out)||(wm5102_con.record_mic1_in))
    {
        wm5102_con.music_speaker_out = true;
        return;
    }
    cust_extcodec_gpio_on();

    I2CWrite(0x102, 0x000b);// SMbus_32inx_16dat     Write  0x34      * 44.1 kHz sample rate set-up

    //FLL 13M to 45.xxxM
    I2CWrite(0x173, 0x1e12);//
    I2CWrite(0x174, 0x1fbd);//
    I2CWrite(0x176, 0x0000);//
    I2CWrite(0x175, 0x0000);//
    I2CWrite(0x179, 0x0010);//
    I2CWrite(0x172, 0x8006);//
    I2CWrite(0x187, 0x0001);//
    I2CWrite(0x171, 0x0001);//
    I2CWrite(0x101, 0x0344);// SMbus_32inx_16dat     Write  0x34      * 45.xxx MHz System Clock enable

    I2CWrite(0x500, 0x000B);
    I2CWrite(0x508, 0x1020);
    //***** AIF Enable *****************************************
    I2CWrite(0x504, 0x0002);// SMbus_32inx_16dat     Write  0x34      * AIF1 I2S set-up
    I2CWrite(0x51A, 0x0003);// SMbus_32inx_16dat     Write  0x34      * AIF1 input enables

    //***** Speaker mixer and output enable ********************
    I2CWrite(0x6B0, 0x0020);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer
    I2CWrite(0x6B8, 0x0021);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer
    I2CWrite(0x400, 0x00C0);// SMbus_32inx_16dat     Write  0x34      * Speaker enable
    I2CWrite(0x429, 0x0080);// SMbus_32inx_16dat     Write  0x34      * Speaker left volume
    I2CWrite(0x42D, 0x0280);// SMbus_32inx_16dat     Write  0x34      * Speaker right volume + volume update


    I2CWrite(0x80 , 0x0003); //SMbus_32inx_16dat     Write  0x34      * Ctrl IF Debug 1(80H):    0003  USER_KEY=1, TEST_KEY=1
    I2CWrite(0x100, 0x0042); //SMbus_32inx_16dat     Write  0x34      * 32k clocking
    I2CWrite(0x210, 0x00D5); //SMbus_32inx_16dat     Write  0x34      * LDO1 Control 1(210H):    00D5  LDO1_VSEL=00_0110, LDO1_FAST=1, LDO1_DISCH=1, LDO1_BYPASS=0, LDO1_ENA=1
    I2CWrite(0x211, 0x0010); //SMbus_32inx_16dat     Write  0x34      * LDO1 Test Control(211H): 0018  LDO1_MODE=0, LDO1_TST_MODE=00, LDO1_FB_TRIM=110
    I2CWrite(0x200, 0x0005); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x218, 0x01A5); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x219, 0x01A5); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x300, 0x0002); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0001  IN3L_ENA=0, IN3R_ENA=0, IN2L_ENA=0, IN2R_ENA=0, IN1L_ENA=0, IN1R_ENA=1
    I2CWrite(0x507, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 1(507H): 1820  AIF1TX_WL=01_1000, AIF1TX_SLOT_LEN=0010_0000
    I2CWrite(0x310, 0x2080); //SMbus_32inx_16dat     Write  0x34      * 128, Diff 
    I2CWrite(0x519, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Tx Enables(519H):   0003  AIF1TX8_ENA=0, AIF1TX7_ENA=0, AIF1TX6_ENA=0, AIF1TX5_ENA=0, AIF1TX4_ENA=0, AIF1TX3_ENA=0, AIF1TX2_ENA=1, AIF1TX1_ENA=1
    I2CWrite(0x700, 0x0010); //SMbus_32inx_16dat     Write  0x34      * AIF1TX1MIX Input 1 Source(700H): 0011  AIF1TX1MIX_STS1=0, AIF1TX1MIX_SRC1=IN1R
    I2CWrite(0x708, 0x0010); //SMbus_32inx_16dat     Write  0x34      * AIF1TX2MIX Input 1 Source(708H): 0011  AIF1TX2MIX_STS1=0, AIF1TX2MIX_SRC1=IN1R
    I2CWrite(0x310, 0x20BC); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 1R(315H): 0380  IN_VU=1, IN1R_MUTE=1, IN1R_DIG_VOL=0dB
    I2CWrite(0x311, 0x0280); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 1R(315H): 0380  IN_VU=1, IN1R_MUTE=1, IN1R_DIG_VOL=0dB

    wm5102_con.music_speaker_out = true;
}

void ExtCodec_Set_Mic1_In(void)
{
    ECODEC_PRINTK("ExtCodec_Set_Mic1_In music_speaker_out:%d,record_mic1_in:%d\n",
        wm5102_con.music_speaker_out, wm5102_con.record_mic1_in);

    if((wm5102_con.music_speaker_out)||(wm5102_con.record_mic1_in))
    {
        wm5102_con.record_mic1_in = true;
        return;
    }
    cust_extcodec_gpio_on();

    I2CWrite(0x102, 0x000b);// SMbus_32inx_16dat     Write  0x34      * 44.1 kHz sample rate set-up

    //FLL 13M to 45.xxxM
    I2CWrite(0x173, 0x1e12);//
    I2CWrite(0x174, 0x1fbd);//
    I2CWrite(0x176, 0x0000);//
    I2CWrite(0x175, 0x0000);//
    I2CWrite(0x179, 0x0010);//
    I2CWrite(0x172, 0x8006);//
    I2CWrite(0x187, 0x0001);//
    I2CWrite(0x171, 0x0001);//
    I2CWrite(0x101, 0x0344);// SMbus_32inx_16dat     Write  0x34      * 45.xxx MHz System Clock enable

    I2CWrite(0x500, 0x000B);
    I2CWrite(0x508, 0x1020);
    //***** AIF Enable *****************************************
    I2CWrite(0x504, 0x0002);// SMbus_32inx_16dat     Write  0x34      * AIF1 I2S set-up
    I2CWrite(0x51A, 0x0003);// SMbus_32inx_16dat     Write  0x34      * AIF1 input enables

    //***** Speaker mixer and output enable ********************
    I2CWrite(0x6B0, 0x0020);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer
    I2CWrite(0x6B8, 0x0021);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer
    I2CWrite(0x400, 0x00C0);// SMbus_32inx_16dat     Write  0x34      * Speaker enable
    I2CWrite(0x429, 0x0080);// SMbus_32inx_16dat     Write  0x34      * Speaker left volume
    I2CWrite(0x42D, 0x0280);// SMbus_32inx_16dat     Write  0x34      * Speaker right volume + volume update


    I2CWrite(0x80 , 0x0003); //SMbus_32inx_16dat     Write  0x34      * Ctrl IF Debug 1(80H):    0003  USER_KEY=1, TEST_KEY=1
    I2CWrite(0x100, 0x0042); //SMbus_32inx_16dat     Write  0x34      * 32k clocking
    I2CWrite(0x210, 0x00D5); //SMbus_32inx_16dat     Write  0x34      * LDO1 Control 1(210H):    00D5  LDO1_VSEL=00_0110, LDO1_FAST=1, LDO1_DISCH=1, LDO1_BYPASS=0, LDO1_ENA=1
    I2CWrite(0x211, 0x0010); //SMbus_32inx_16dat     Write  0x34      * LDO1 Test Control(211H): 0018  LDO1_MODE=0, LDO1_TST_MODE=00, LDO1_FB_TRIM=110
    I2CWrite(0x200, 0x0005); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x218, 0x01A5); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x219, 0x01A5); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x300, 0x0002); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0001  IN3L_ENA=0, IN3R_ENA=0, IN2L_ENA=0, IN2R_ENA=0, IN1L_ENA=0, IN1R_ENA=1
    I2CWrite(0x507, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 1(507H): 1820  AIF1TX_WL=01_1000, AIF1TX_SLOT_LEN=0010_0000
    I2CWrite(0x310, 0x2080); //SMbus_32inx_16dat     Write  0x34      * 128, Diff 
    I2CWrite(0x519, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Tx Enables(519H):   0003  AIF1TX8_ENA=0, AIF1TX7_ENA=0, AIF1TX6_ENA=0, AIF1TX5_ENA=0, AIF1TX4_ENA=0, AIF1TX3_ENA=0, AIF1TX2_ENA=1, AIF1TX1_ENA=1
    I2CWrite(0x700, 0x0010); //SMbus_32inx_16dat     Write  0x34      * AIF1TX1MIX Input 1 Source(700H): 0011  AIF1TX1MIX_STS1=0, AIF1TX1MIX_SRC1=IN1R
    I2CWrite(0x708, 0x0010); //SMbus_32inx_16dat     Write  0x34      * AIF1TX2MIX Input 1 Source(708H): 0011  AIF1TX2MIX_STS1=0, AIF1TX2MIX_SRC1=IN1R
    I2CWrite(0x310, 0x20BC); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 1R(315H): 0380  IN_VU=1, IN1R_MUTE=1, IN1R_DIG_VOL=0dB
    I2CWrite(0x311, 0x0280); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 1R(315H): 0380  IN_VU=1, IN1R_MUTE=1, IN1R_DIG_VOL=0dB

    wm5102_con.record_mic1_in = true;
}

void ExtCodec_Set_Mic2_In(void)
{
#if 0 //Qlw 2013/12/12 Not used now
    ECODEC_PRINTK("ExtCodec_Set_Mic2_In:%d\n",wm5102_con.record_mic2_in);

    if(wm5102_con.record_mic2_in)
        return;

    cust_extcodec_gpio_on();

    //* FLL1 setting,13M MCLK1 in,45.xxxM out.
    I2CWrite(0x161, 0x0001); //Add by Henry 2013/10/31

    I2CWrite(0x173, 0x1e12);//
    I2CWrite(0x174, 0x1fbd);//
    I2CWrite(0x176, 0x0000);//
    I2CWrite(0x175, 0x0000);//
    I2CWrite(0x179, 0x0010);//
    I2CWrite(0x172, 0x8006);//
    I2CWrite(0x187, 0x0001);//
    I2CWrite(0x171, 0x0001);//

    I2CWrite(0x80 , 0x0003); //SMbus_32inx_16dat     Write  0x34      * Ctrl IF Debug 1(80H):    0003  USER_KEY=1, TEST_KEY=1
    I2CWrite(0x101, 0x0344); //SMbus_32inx_16dat     Write  0x34      * System Clock 1(101H):    8140  SYSCLK_FRAC=Integer Rate, SYSCLK_FREQ=49.1552MHz (45.1585MHz), SYSCLK_ENA=1, SYSCLK_SRC=FLL1
    I2CWrite(0x102, 0x000B); //SMbus_32inx_16dat     Write  0x34      * Sample rate 1(102H):     0003  SAMPLE_RATE_1=44.1kHz
    I2CWrite(0x100, 0x0042); //SMbus_32inx_16dat     Write  0x34      * 32k clocking
    I2CWrite(0x210, 0x00D5); //SMbus_32inx_16dat     Write  0x34      * LDO1 Control 1(210H):    00D5  LDO1_VSEL=00_0110, LDO1_FAST=1, LDO1_DISCH=1, LDO1_BYPASS=0, LDO1_ENA=1
    I2CWrite(0x211, 0x0010); //SMbus_32inx_16dat     Write  0x34      * LDO1 Test Control(211H): 0018  LDO1_MODE=0, LDO1_TST_MODE=00, LDO1_FB_TRIM=110
    I2CWrite(0x200, 0x0005); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x218, 0x01A5); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x219, 0x01A5); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x300, 0x0001); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0001  IN3L_ENA=0, IN3R_ENA=0, IN2L_ENA=0, IN2R_ENA=0, IN1L_ENA=0, IN1R_ENA=1
    I2CWrite(0x500, 0x000B); //SMbus_32inx_16dat     Write  0x34      * BCLK=1.536M.
    I2CWrite(0x504, 0x0002); //SMbus_32inx_16dat     Write  0x34      * AIF1 Format(504H):       0002  AIF1_FMT=010
    I2CWrite(0x507, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 1(507H): 1820  AIF1TX_WL=01_1000, AIF1TX_SLOT_LEN=0010_0000
    I2CWrite(0x508, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 2(508H): 1820  AIF1RX_WL=01_1000, AIF1RX_SLOT_LEN=0010_0000
    I2CWrite(0x310, 0x2080); //SMbus_32inx_16dat     Write  0x34      * 128, Diff 
    I2CWrite(0x519, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Tx Enables(519H):   0003  AIF1TX8_ENA=0, AIF1TX7_ENA=0, AIF1TX6_ENA=0, AIF1TX5_ENA=0, AIF1TX4_ENA=0, AIF1TX3_ENA=0, AIF1TX2_ENA=1, AIF1TX1_ENA=1
    I2CWrite(0x700, 0x0011); //SMbus_32inx_16dat     Write  0x34      * AIF1TX1MIX Input 1 Source(700H): 0011  AIF1TX1MIX_STS1=0, AIF1TX1MIX_SRC1=IN1R
    I2CWrite(0x708, 0x0011); //SMbus_32inx_16dat     Write  0x34      * AIF1TX2MIX Input 1 Source(708H): 0011  AIF1TX2MIX_STS1=0, AIF1TX2MIX_SRC1=IN1R
    I2CWrite(0x314, 0x00BC); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 1R(315H): 0380  IN_VU=1, IN1R_MUTE=1, IN1R_DIG_VOL=0dB
    I2CWrite(0x315, 0x0280); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 1R(315H): 0380  IN_VU=1, IN1R_MUTE=1, IN1R_DIG_VOL=0dB

    wm5102_con.record_mic2_in = true;
#endif
}

void ExtCodec_Set_HpMic_In(void)
{
    ECODEC_PRINTK("ExtCodec_Set_HpMic_In record_hpmic_in:%d,music_hp_out:%d\n",
        wm5102_con.record_hpmic_in,wm5102_con.music_hp_out);

    if((wm5102_con.record_hpmic_in)||(wm5102_con.music_hp_out))
    {
        wm5102_con.record_hpmic_in = true;
        return;
    }
    cust_extcodec_gpio_on();

    I2CWrite(0x102, 0x000b);// SMbus_32inx_16dat     Write  0x34      * 44.1 kHz sample rate set-up

    //FLL 13M to 45.xxxM
    I2CWrite(0x173, 0x1e12);//
    I2CWrite(0x174, 0x1fbd);//
    I2CWrite(0x176, 0x0000);//
    I2CWrite(0x175, 0x0000);//
    I2CWrite(0x179, 0x0010);//
    I2CWrite(0x172, 0x8006);//
    I2CWrite(0x187, 0x0001);//
    I2CWrite(0x171, 0x0001);//
    I2CWrite(0x101, 0x0344);// SMbus_32inx_16dat     Write  0x34      * 45.xxx MHz System Clock enable

    I2CWrite(0x210, 0x00D5);

    I2CWrite(0x500, 0x000B);
    I2CWrite(0x508, 0x1020);
    //***** AIF Enable *****************************************
    I2CWrite(0x504, 0x0002);// SMbus_32inx_16dat     Write  0x34      * AIF1 I2S set-up
    I2CWrite(0x51A, 0x0003);// SMbus_32inx_16dat     Write  0x34      * AIF1 input enables

    //***** Speaker mixer and output enable ********************
    I2CWrite(0x204, 0x8005);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer
    I2CWrite(0x680, 0x0020);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer
    I2CWrite(0x688, 0x0021);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer

    I2CWrite(0x412, 0x0082);// SMbus_32inx_16dat     Write  0x34      * HP left volume
    I2CWrite(0x416, 0x0082);// SMbus_32inx_16dat     Write  0x34      * HP right volume + volume update
    I2CWrite(0x411, 0x0281);// SMbus_32inx_16dat     Write  0x34      * HP left volume
    I2CWrite(0x415, 0x0281);// SMbus_32inx_16dat     Write  0x34      * HP right volume + volume update

    I2CWrite(0x400, 0x0003);// SMbus_32inx_16dat     Write  0x34      * HP enable


    I2CWrite(0x80 , 0x0003); //SMbus_32inx_16dat     Write  0x34      * Ctrl IF Debug 1(80H):    0003  USER_KEY=1, TEST_KEY=1
    I2CWrite(0x100, 0x0042); //SMbus_32inx_16dat     Write  0x34      * 32k clocking
    I2CWrite(0x211, 0x0010); //SMbus_32inx_16dat     Write  0x34      * LDO1 Test Control(211H): 0018  LDO1_MODE=0, LDO1_TST_MODE=00, LDO1_FB_TRIM=110
    I2CWrite(0x200, 0x0005); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x218, 0x01A5); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x219, 0x01A5); //SMbus_32inx_16dat     Write  0x34      * Mic Charge Pump 1(200H): 0007  CPMIC_DISCH=1, CPMIC_BYPASS=1, CPMIC_ENA=1
    I2CWrite(0x300, 0x0020); //SMbus_32inx_16dat     Write  0x34      * Input Enables(300H):     0001  IN3L_ENA=0, IN3R_ENA=0, IN2L_ENA=0, IN2R_ENA=0, IN1L_ENA=0, IN1R_ENA=1
    I2CWrite(0x507, 0x1020); //SMbus_32inx_16dat     Write  0x34      * AIF1 Frame Ctrl 1(507H): 1820  AIF1TX_WL=01_1000, AIF1TX_SLOT_LEN=0010_0000
    //I2CWrite(0x310, 0x2080); //SMbus_32inx_16dat     Write  0x34      * 128, Diff 
    I2CWrite(0x519, 0x0003); //SMbus_32inx_16dat     Write  0x34      * AIF1 Tx Enables(519H):   0003  AIF1TX8_ENA=0, AIF1TX7_ENA=0, AIF1TX6_ENA=0, AIF1TX5_ENA=0, AIF1TX4_ENA=0, AIF1TX3_ENA=0, AIF1TX2_ENA=1, AIF1TX1_ENA=1
    I2CWrite(0x700, 0x0014); //SMbus_32inx_16dat     Write  0x34      * AIF1TX1MIX Input 1 Source(700H): 0011  AIF1TX1MIX_STS1=0, AIF1TX1MIX_SRC1=IN1R
    I2CWrite(0x708, 0x0014); //SMbus_32inx_16dat     Write  0x34      * AIF1TX2MIX Input 1 Source(708H): 0011  AIF1TX2MIX_STS1=0, AIF1TX2MIX_SRC1=IN1R
    I2CWrite(0x320, 0x20BC); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 1R(315H): 0380  IN_VU=1, IN1R_MUTE=1, IN1R_DIG_VOL=0dB
    I2CWrite(0x321, 0x0280); //SMbus_32inx_16dat     Write  0x34      * ADC Digital Volume 1R(315H): 0380  IN_VU=1, IN1R_MUTE=1, IN1R_DIG_VOL=0dB

    wm5102_con.record_hpmic_in = true;
}

void ExtCodec_Set_DualMode_Out(void)
{
    ECODEC_PRINTK("ExtCodec_Set_DualMode_Out:%d\n",wm5102_con.music_dualmode_out);
    cust_extcodec_gpio_on();

    I2CWrite(0x102, 0x000b);// SMbus_32inx_16dat     Write  0x34      * 44.1 kHz sample rate set-up

    //FLL 13M to 45.xxxM
    I2CWrite(0x173, 0x1e12);//
    I2CWrite(0x174, 0x1fbd);//
    I2CWrite(0x176, 0x0000);//
    I2CWrite(0x175, 0x0000);//
    I2CWrite(0x179, 0x0010);//
    I2CWrite(0x172, 0x8006);//
    I2CWrite(0x187, 0x0001);//
    I2CWrite(0x171, 0x0001);//
    I2CWrite(0x101, 0x0344);// SMbus_32inx_16dat     Write  0x34      * 45.xxx MHz System Clock enable

    I2CWrite(0x210, 0x00D5);

    I2CWrite(0x500, 0x000B);
    I2CWrite(0x508, 0x1020);
    //***** AIF Enable *****************************************
    I2CWrite(0x504, 0x0002);// SMbus_32inx_16dat     Write  0x34      * AIF1 I2S set-up
    I2CWrite(0x51A, 0x0003);// SMbus_32inx_16dat     Write  0x34      * AIF1 input enables

    //***** Speaker mixer and output enable ********************
    I2CWrite(0x204, 0x8005);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer
    I2CWrite(0x680, 0x0020);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer
    I2CWrite(0x688, 0x0021);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer

    I2CWrite(0x412, 0x0082);// SMbus_32inx_16dat     Write  0x34      * HP left volume
    I2CWrite(0x416, 0x0082);// SMbus_32inx_16dat     Write  0x34      * HP right volume + volume update
    I2CWrite(0x411, 0x0281);// SMbus_32inx_16dat     Write  0x34      * HP left volume
    I2CWrite(0x415, 0x0281);// SMbus_32inx_16dat     Write  0x34      * HP right volume + volume update

    I2CWrite(0x400, 0x0003);// SMbus_32inx_16dat     Write  0x34      * HP enable


    //***** Speaker mixer and output enable ********************
    I2CWrite(0x6B0, 0x0020);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer
    I2CWrite(0x6B8, 0x0021);// SMbus_32inx_16dat     Write  0x34      * AIF1 to speaker mixer
    I2CWrite(0x400, 0x00C3);// SMbus_32inx_16dat     Write  0x34      * Speaker enable
    I2CWrite(0x429, 0x0080);// SMbus_32inx_16dat     Write  0x34      * Speaker left volume
    I2CWrite(0x42D, 0x0280);// SMbus_32inx_16dat     Write  0x34      * Speaker right volume + volume update

    wm5102_con.music_dualmode_out = true;
}

void ExtCodec_Factory_MainMic_En(u8 enable)
{
    ECODEC_PRINTK("ExtCodec_Factory_MainMic_En:%d\n",enable);
    factory_mic.main_mic = enable;
}

void ExtCodec_Factory_RefMic_En(u8 enable)
{
    ECODEC_PRINTK("ExtCodec_Factory_RefMic_En:%d\n",enable);
    factory_mic.ref_mic = enable;
}

bool ExtCodec_Get_Factory_MicState(void)
{
    ECODEC_PRINTK("ExtCodec_Get_MicState main:%d,ref:%d\n",factory_mic.main_mic,factory_mic.ref_mic);

    if((factory_mic.main_mic)||(factory_mic.ref_mic))
        return true;
    else
        return false;
}

