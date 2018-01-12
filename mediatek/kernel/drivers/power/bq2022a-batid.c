#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
//====================
//#include <linux/init.h>        /* For init/exit macros */
//#include <linux/module.h>      /* For MODULE_ marcros  */
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/xlog.h>

//==================
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpt.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>
//#include "mt6320_battery.h"
//#include <mach/pmic_mt6320_sw.h>
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#include <cust_gpio_usage.h>
//========================
#include <linux/platform_device.h>
#include "bq2022a-batid.h"
#include <mach/charging.h>

/* BQ2022A. */
#define	ROM_COMMAND		(0xcc)		// Skip ROM Command
#define	CTL_COMMAND		(0xc3)		// Read Control Command    2022a
#define	WADDR_LB			(0x00)		// Write Address Low Byte
#define	WADDR_HB			(0x00)		// Write Address High Byte


static struct bq2022a_platform_data *g_bq2022a = NULL;
static struct bq2022a_platform_data  bq2022a_data={0};

static int bq2022a_bat_id = GPIO_BQ_BSQ_PIN;

//******************************************************************************
// Description : Creates the Reset signal to initiate SDQ communication
//******************************************************************************

void bq2022a_sdq_reset(void)
{
	/*SDQ_HIGH;
	SDQ_OUTPUT;
	SDQ_LOW;
	udelay(500);
	SDQ_HIGH;*/
    mt_set_gpio_mode(bq2022a_bat_id,GPIO_BQ_BSQ_PIN_M_GPIO);
    mt_set_gpio_dir(bq2022a_bat_id, GPIO_DIR_OUT);
	mt_set_gpio_out(bq2022a_bat_id, GPIO_OUT_ONE);
	mt_set_gpio_out(bq2022a_bat_id, GPIO_OUT_ZERO);
	/* Reset time should be > 480 usec */
	udelay(800);
	mt_set_gpio_out(bq2022a_bat_id, GPIO_OUT_ONE);
}

//******************************************************************************
// Description : Detects if a device responds to Reset signal
// Arguments : PresenceTimer - Sets timeout if no device present
//             	InputData - Actual state of GPIO
// 			GotPulse - States if a pulse was detected
// Returns : GotPulse
//******************************************************************************
unsigned char bq2022a_sdq_detect(void)
{
	unsigned int PresenceTimer = 300;
	static volatile unsigned char InputData;
	static volatile unsigned char GotPulse = 0;

	//SDQ_INPUT;
	mt_set_gpio_dir(bq2022a_bat_id,GPIO_DIR_IN);
	while ((PresenceTimer > 0) && (GotPulse == 0)) {
		InputData = mt_get_gpio_in(bq2022a_bat_id);		//Capture state of the SDQ bus
		if (InputData == 0) {                  	// If SDQ bus got pulled low
			GotPulse = 1;				// then the device responded
		} else {                                 		// If SDQ bus is still high
			GotPulse = 0;				// then the device did not respond
			--PresenceTimer;          	// Decrease timeout counter
		}
	}
	udelay(200);                				// Delay before attempting command

	return GotPulse;                          		// Return if device detected or not
}

//******************************************************************************
// unsigned char SDQ_readBit(void)
// Description : Receives the bit value returned by the SDQ slave
// Arguments : none
// Returns : State of the SDQ bus (0 or 1)
//******************************************************************************
static unsigned char bq2022a_sdq_readbit(int time)
{
	static unsigned char inbit;

	/*SDQ_HIGH;
	SDQ_OUTPUT;
	SDQ_LOW;
	SDQ_INPUT;
	udelay(12);*/

	//gpio_set_value(bq2022a_bat_id, 1);
	mt_set_gpio_out(bq2022a_bat_id, GPIO_OUT_ONE);	
    mt_set_gpio_dir(bq2022a_bat_id, GPIO_DIR_OUT);
	//gpio_set_value(bq2022a_bat_id, 0);
	mt_set_gpio_out(bq2022a_bat_id, GPIO_OUT_ZERO);	
	udelay(10);
	mt_set_gpio_dir(bq2022a_bat_id,GPIO_DIR_IN);
	udelay(time);

	/*InBit = SDQ_READ;           // Capture state of the SDQ bus
	udelay(65);                             // Wait for end of read bit cycle
	SDQ_HIGH;
	SDQ_OUTPUT;*/

	inbit = mt_get_gpio_in(bq2022a_bat_id);
	udelay(65);                             // Wait for end of read bit cycle
	//gpio_set_value(bq2022a_bat_id, 1);
	mt_set_gpio_out(bq2022a_bat_id, GPIO_OUT_ONE);	
    mt_set_gpio_dir(bq2022a_bat_id, GPIO_DIR_OUT);

	if (inbit)
		return 1;                               // Return SDQ HIGH
	else
		return 0;                               // Return SDQ LOW
}

//******************************************************************************
// unsigned char SDQ_readByte(void)
// Description : Reads 8 bits on the SDQ line and returns the byte value.
// Arguments : Databyte - Byte value returned by SDQ slave
//	       		MaskByte - Used to seperate each bit
//            	 i - Used for 8 time loop
// Returns : DataByte
//******************************************************************************
unsigned char bq2022a_sdq_readbyte(int time)
{
	unsigned char data = 0x00;
	unsigned char mask, i;
	unsigned long flags;

	//_DINT();		//disable the irq
	spin_lock_irqsave(&g_bq2022a->bqlock, flags);

	for (i = 0; i < 8; i++) {					// Select one bit at a time
		mask = bq2022a_sdq_readbit(time);		// Read One Bit
		mask <<= i;						// Determine bit position in byte
		data	 = (data | mask);				// Keep adding bits to form the byte
	}

	udelay(200);							// Delay before attempting command
	//_EINT();
	spin_unlock_irqrestore(&g_bq2022a->bqlock, flags);

  return data;                              // Return byte value read
}

void bq2022a_sdq_writebyte(u8 value)
{
	unsigned char mask = 1;
	int i;
	unsigned long flags;

	//_DINT();
	spin_lock_irqsave(&g_bq2022a->bqlock, flags);

	//SDQ_HIGH;
	//SDQ_OUTPUT;
	//SDQ_LOW;

	//gpio_set_value(bq2022a_bat_id, 1);
	mt_set_gpio_out(bq2022a_bat_id, GPIO_OUT_ONE);	
    mt_set_gpio_dir(bq2022a_bat_id, GPIO_DIR_OUT);
	//gpio_set_value(bq2022a_bat_id, 0);

	for (i = 0; i < 8; i++) {
		//gpio_set_value(bq2022a_bat_id, 0);
		mt_set_gpio_out(bq2022a_bat_id, GPIO_OUT_ZERO);
		
		if (mask & value) {
			udelay(10);
			//SDQ_HIGH;
			//gpio_set_value(bq2022a_bat_id, 1);
			mt_set_gpio_out(bq2022a_bat_id, GPIO_OUT_ONE);			
			udelay(100);
		} else {
			udelay(100);
			//SDQ_HIGH;
			//gpio_set_value(bq2022a_bat_id, 1);
			mt_set_gpio_out(bq2022a_bat_id, GPIO_OUT_ONE);			
			udelay(10);
		}

		udelay(7);
		mask <<= 1;
	}

	//_EINT();
	spin_unlock_irqrestore(&g_bq2022a->bqlock, flags);

}

static int bat_module_id = 0;
bool is_bat_id_byte = KAL_FALSE;
static const unsigned char *con_bat_id[] = {
	[0] = 0xed, [1] = 0x21, [2] = 0x4c, [3] = 0xe5,
	[4] = 0xed, [5] = 0xa9, [6] = 0x4b, [7] = 0x2e,
};

int bq2022a_read_bat_id(void)
{
	unsigned char bat_id;
	int i;
	static int delay_time = 12;

	bq2022a_sdq_reset();
	bat_id = bq2022a_sdq_detect();
	bq2022a_sdq_writebyte(ROM_COMMAND);	// Skip ROM Command
	bq2022a_sdq_writebyte(CTL_COMMAND);	// Read Control Command    2022a
	bq2022a_sdq_writebyte(WADDR_LB);		// Write Address Low Byte
	bq2022a_sdq_writebyte(WADDR_HB);		// Write Address High Byte
	bat_id = bq2022a_sdq_readbyte(delay_time);			// Read CRC, one page, one CRC, one page is 32 bytes
	for (i = 0; i < 9; i++) {
		bat_id = bq2022a_sdq_readbyte(delay_time);	// Read Data to Verify Write 8 time
		battery_xlog_printk(BAT_LOG_CRTI,"@hxq__bat_id = %x,con_bat_id = %x,delay_time = %d\n\r",bat_id,con_bat_id[i],delay_time);			
		if((bat_id != 0)&&(bat_id != 0xff)&&(is_bat_id_byte == KAL_FALSE))
		{
			is_bat_id_byte = KAL_TRUE;
		}
		if ((bat_id != con_bat_id[i])&&(i < 8))
		{
			mt_set_gpio_dir(bq2022a_bat_id, GPIO_DIR_OUT);
			mt_set_gpio_out(bq2022a_bat_id, GPIO_OUT_ONE);
			delay_time++;
			if(delay_time > 62)
			{
				delay_time = 12;
			}
			return -1;
		}
		if(i == 8)
		{
			bat_module_id = (bat_id & 0x0f);
		}
	}
	bat_id = bq2022a_sdq_readbyte(delay_time);		// Read CRC, one page, one CRC, one page is 32 bytes
	delay_time = 12;
	mt_set_gpio_dir(bq2022a_bat_id, GPIO_DIR_OUT);
	mt_set_gpio_out(bq2022a_bat_id, GPIO_OUT_ONE);	
	return 0;
}
EXPORT_SYMBOL_GPL(bq2022a_read_bat_id);

/*
	2:	Guangyu		+	Guangyu		
	1:	Sunwoda		+	Samsung
	3:	Sunwoda		+	Sony
	4:	Sunwoda		+	Samsung(customdown)
	5:	Desay		+	LG
	6:	Feimaotui		+	Sony
*/
int bq2022a_read_bat_module_id(void)
{
	return bat_module_id;	
}
EXPORT_SYMBOL_GPL(bq2022a_read_bat_module_id);

static void __init bq2022a_init(void)
{
	int ret;
	struct bq2022a_platform_data *data;
#if 1	
	data = (struct bq2022a_platform_data *)
				kzalloc(sizeof(struct bq2022a_platform_data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;
	g_bq2022a = data;
#else // 使用静态变量，避免MODULE init 的调用前后关系影响。
	g_bq2022a = bq2022a_data;
#endif 
	spin_lock_init(&g_bq2022a->bqlock);


}

static void __exit bq2022a_exit(void)
{
   battery_xlog_printk(BAT_LOG_CRTI,"bq2022a_exit!! \n\r");			
}

module_init(bq2022a_init);
module_exit(bq2022a_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("bq2022a-batid driver");
