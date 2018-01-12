/*
 * Driver for CAM_CAL
 *
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "s24cs64a.h"
#include <asm/system.h>  // for SMP

//#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define CAM_CALDB printk
#else
#define CAM_CALDB(x,...)
#endif


static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 1
// for init.rc static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO("CAM_CAL_S24CS64A", 0xAA>>1)};

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208

//#define CAM_CAL_TEST_LSC  //pengjinqiang for test
/*******************************************************************************
*
********************************************************************************/
// for init.rc #define CAM_CAL_DRVNAME "CAM_CAL_S24CS64A"
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0xA0>>1)};
static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
//static spinlock_t g_CAM_CALLock;
//spin_lock(&g_CAM_CALLock);
//spin_unlock(&g_CAM_CALLock);

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
//static DEFINE_SPINLOCK(kdcam_cal_drv_lock);
//spin_lock(&kdcam_cal_drv_lock);
//spin_unlock(&kdcam_cal_drv_lock);

/*******************************************************************************
*
********************************************************************************/
// maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt65xx.c which is 8 bytes
int iWriteCAM_CAL(u16 a_u2Addr  , u32 a_u4Bytes, u8 * puDataInBytes)
{

#if 0   //pengjinqiang modify
    int  i4RetValue = 0;
    u32 u4Index = 0;
    char puSendCmd[8] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF) ,
        0, 0, 0, 0, 0, 0};
    if(a_u4Bytes + 2 > 8)
    {
        CAM_CALDB("[S24CAM_CAL] exceed I2c-mt65xx.c 8 bytes limitation (include address 2 Byte)\n");
        return -1;
    }

    for(u4Index = 0 ; u4Index < a_u4Bytes ; u4Index += 1 )
    {
        puSendCmd[(u4Index + 2)] = puDataInBytes[u4Index];
    }
    
    i4RetValue = i2c_master_send(g_pstI2Cclient, puSendCmd, (a_u4Bytes + 2));
    if (i4RetValue != (a_u4Bytes + 2))
    {
        CAM_CALDB("[S24CAM_CAL] I2C write  failed!! \n");
        return -1;
    }
    mdelay(10); //for tWR singnal --> write data form buffer to memory.
    #endif
	
   //CAM_CALDB("[CAM_CAL] iWriteCAM_CAL done!! \n");
    return 0;
}


// maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt65xx.c which is 8 bytes
int iReadCAM_CAL_A0(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {(char)a_u2Addr };

    CAM_CALDB("[CAM_CAL] iReadCAM_CAL_A0 :%d,%d\n",a_u2Addr,ui4_length);


    if(ui4_length > 8)
    {
        CAM_CALDB("[S24CAM_CAL] exceed I2c-mt65xx.c 8 bytes limitation\n");
        return -1;
    }
	

	 g_pstI2Cclient->addr = S24CS64A_DEVICE_ID>>1;

    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient->addr = g_pstI2Cclient->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);	
    spin_unlock(&g_CAM_CALLock); // for SMP


    CAM_CALDB("[CAM_CAL] i2c_master_send \n");
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 1);
    if (i4RetValue < 0)
    {
        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
        return -1;
    }

    CAM_CALDB("[CAM_CAL] i2c_master_recv \n");
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
    if (i4RetValue != ui4_length)
    {
        CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
        return -1;
    }
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient->addr = g_pstI2Cclient->addr & I2C_MASK_FLAG;    
    spin_unlock(&g_CAM_CALLock); // for SMP    


    return 0;
}

// maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt65xx.c which is 8 bytes
int iReadCAM_CAL_A2(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {(char)a_u2Addr };

    CAM_CALDB("[CAM_CAL] iReadCAM_CAL_A2 :%d,%d\n",a_u2Addr,ui4_length);


    if(ui4_length > 8)
    {
        CAM_CALDB("[S24CAM_CAL] exceed I2c-mt65xx.c 8 bytes limitation\n");
        return -1;
    }
	

	 g_pstI2Cclient->addr = S24CS64A_DEVICE_ID1>>1;

    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient->addr = g_pstI2Cclient->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);	
    spin_unlock(&g_CAM_CALLock); // for SMP


    CAM_CALDB("[CAM_CAL] i2c_master_send \n");
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 1);
    if (i4RetValue < 0)
    {
        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
        return -1;
    }

    CAM_CALDB("[CAM_CAL] i2c_master_recv \n");
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
    if (i4RetValue != ui4_length)
    {
        CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
        return -1;
    }
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient->addr = g_pstI2Cclient->addr & I2C_MASK_FLAG;    
    spin_unlock(&g_CAM_CALLock); // for SMP    


    return 0;
}

int iReadCAM_CAL_A4(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {(char)a_u2Addr };

    CAM_CALDB("[CAM_CAL] iReadCAM_CAL_A4 :%d,%d\n",a_u2Addr,ui4_length);

	g_pstI2Cclient->addr = 0xA4>>1;

    CAM_CALDB("[CAM_CAL] i2c_master_send \n");
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 1);
    if (i4RetValue < 0)
    {
        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
        return -1;
    }

    CAM_CALDB("[CAM_CAL] i2c_master_recv \n");
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
    if (i4RetValue != ui4_length)
    {
        CAM_CALDB("[CAM_CAL] I2C read data failed!!size is %d\n", i4RetValue);
        return -1;
    }
	CAM_CALDB("[CAM_CAL] I2C read data size is %d!! \n", i4RetValue);
 
    return 0;
}

int iReadCAM_CAL_A6(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {(char)a_u2Addr };

    CAM_CALDB("[CAM_CAL] iReadCAM_CAL_A6 :%d,%d\n",a_u2Addr,ui4_length);

	g_pstI2Cclient->addr = 0xA6>>1;

    CAM_CALDB("[CAM_CAL] i2c_master_send \n");
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 1);
    if (i4RetValue < 0)
    {
        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
        return -1;
    }

    CAM_CALDB("[CAM_CAL] i2c_master_recv \n");
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
    if (i4RetValue != ui4_length)
    {
        CAM_CALDB("[CAM_CAL] I2C read data failed!!size is %d\n", i4RetValue);
        return -1;
    }
	CAM_CALDB("[CAM_CAL] I2C read data size is %d!! \n", i4RetValue);

    return 0;
}

int iReadCAM_CAL_A8(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {(char)a_u2Addr };

    CAM_CALDB("[CAM_CAL] iReadCAM_CAL_A8 :%d,%d\n",a_u2Addr,ui4_length);

	g_pstI2Cclient->addr = 0xA8>>1;

    CAM_CALDB("[CAM_CAL] i2c_master_send \n");
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 1);
    if (i4RetValue < 0)
    {
        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
        return -1;
    }

    CAM_CALDB("[CAM_CAL] i2c_master_recv \n");
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
    if (i4RetValue != ui4_length)
    {
        CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
        return -1;
    }
	CAM_CALDB("[CAM_CAL] I2C read data size is %d!! \n", i4RetValue);

    return 0;
}

int iReadCAM_CAL_AA(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {(char)a_u2Addr };

    CAM_CALDB("[CAM_CAL] iReadCAM_CAL_AA :%d,%d\n",a_u2Addr,ui4_length);

	g_pstI2Cclient->addr = 0xAA>>1;

    CAM_CALDB("[CAM_CAL] i2c_master_send \n");
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 1);
    if (i4RetValue < 0)
    {
        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
        return -1;
    }

    CAM_CALDB("[CAM_CAL] i2c_master_recv \n");
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
    if (i4RetValue != ui4_length)
    {
        CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
        return -1;
    }
	CAM_CALDB("[CAM_CAL] I2C read data size is %d!! \n", i4RetValue);

    return 0;
}

int iReadIMX135AWBData(unsigned char *pinputdata)
{
	int i;
	int  i4RetValue = 0;
	unsigned char * LSCDataA0=NULL;
	
	LSCDataA0 = (u8 *)kmalloc(6,GFP_KERNEL);
	if(NULL == LSCDataA0)
	{
	    CAM_CALDB("[S24CAM_CAL] iReadLSCDataA0 allocate mem failed\n");
	    return -ENOMEM;
	}

	i4RetValue = iReadCAM_CAL_A0(0xF, 6, LSCDataA0);
	if (i4RetValue != 0)
	{
	   CAM_CALDB("[CAM_CAL] I2C iReadData failed!! \n");
	   return -1;
	} 		  

	for(i=0; i<6; i++)
	{
		pinputdata[i] = LSCDataA0[i];
		CAM_CALDB("[iReadIMX135AWBData] pinputdata[%d] =0x%x\n",i,pinputdata[i]);
	}

	kfree(LSCDataA0);

	return 0;
}

int iReadIMX135LSCData(unsigned char *pinputdata)
{
	int i;
	unsigned char * LSCDataA4=NULL;
	unsigned char * LSCDataA6=NULL;
	unsigned char * pbuf=NULL;

	int  i4RetValue = 0;
	int  i4ResidueDataLength = 0;
	u32  u4CurrentOffset = 0;
	
	LSCDataA4 = (u8 *)kmalloc(253,GFP_KERNEL);
	if(NULL == LSCDataA4)
	{
	    CAM_CALDB("[S24CAM_CAL] iReadLSCDataA0 allocate mem failed\n");
	    return -ENOMEM;
	}

	LSCDataA6 = (u8 *)kmalloc(251,GFP_KERNEL);
	if(NULL == LSCDataA6)
	{
	    CAM_CALDB("[S24CAM_CAL] iReadLSCDataA2 allocate mem failed\n");
	    return -ENOMEM;
	}

#if 1
	pbuf = LSCDataA4;
	i4ResidueDataLength = 253;
	u4CurrentOffset = 0x03;
	do 
	{
	  if(i4ResidueDataLength >= 8)
	  {
		  i4RetValue = iReadCAM_CAL_A4((u16)u4CurrentOffset, 8, pbuf);
		  if (i4RetValue != 0)
		  {
			   CAM_CALDB("[CAM_CAL] I2C iReadData failed!! \n");
			   return -1;
		  } 		  
		  i4ResidueDataLength -= 8;
		  u4CurrentOffset = u4CurrentOffset + 8;
		  pbuf = pbuf + 8;
	  }
	  else
	  {
		  i4RetValue = iReadCAM_CAL_A4((u16)u4CurrentOffset, i4ResidueDataLength, pbuf);
		  if (i4RetValue != 0)
		  {
			   CAM_CALDB("[CAM_CAL] I2C iReadData failed!! \n");
			   return -1;
		  }
		  i4ResidueDataLength -= i4ResidueDataLength;
	  }
	}while (i4ResidueDataLength > 0);

	pbuf = LSCDataA6;
	i4ResidueDataLength = 251;
	u4CurrentOffset = 0x00;
	do 
	{
	  if(i4ResidueDataLength >= 8)
	  {
		  i4RetValue = iReadCAM_CAL_A6((u16)u4CurrentOffset, 8, pbuf);
		  if (i4RetValue != 0)
		  {
			   CAM_CALDB("[CAM_CAL] I2C iReadData failed!! \n");
			   return -1;
		  } 		  
		  i4ResidueDataLength -= 8;
		  u4CurrentOffset = u4CurrentOffset + 8;
		  pbuf = pbuf + 8;
	  }
	  else
	  {
		  i4RetValue = iReadCAM_CAL_A6((u16)u4CurrentOffset, i4ResidueDataLength, pbuf);
		  if (i4RetValue != 0)
		  {
			   CAM_CALDB("[CAM_CAL] I2C iReadData failed!! \n");
			   return -1;
		  }  
		  i4ResidueDataLength -= i4ResidueDataLength;
	  }
	}while (i4ResidueDataLength > 0);
#else
	i4RetValue = iReadCAM_CAL_A4((u16)0x03, 8, LSCDataA4);
	if (i4RetValue != 0)
	{
		CAM_CALDB("[CAM_CAL] I2C iReadData failed!! \n");
		return -1;
	}
	i4RetValue = iReadCAM_CAL_A6((u16)0x0, 8, LSCDataA6);
	if (i4RetValue != 0)
	{
		CAM_CALDB("[CAM_CAL] I2C iReadData failed!! \n");
		return -1;
	}
#endif

#if 1
	for(i=0; i<504; i++)
	{
		if(i < 253)
		{
			pinputdata[i] = LSCDataA4[i];
		}
		else
		{
			pinputdata[i] = LSCDataA6[i-253];
		}
		//CAM_CALDB("[iReadIMX135LSCData] pinputdata[%d] =0x%x\n",i,pinputdata[i]);
	}
#endif

	kfree(LSCDataA4);
	kfree(LSCDataA6);

	return 0;
}

static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{

#if 0  //pengjinqiang modify
   int  i4RetValue = 0;
   int  i4ResidueDataLength;
   u32 u4IncOffset = 0;
   u32 u4CurrentOffset;
   u8 * pBuff;

   CAM_CALDB("[S24CAM_CAL] iWriteData\n" );

   if (ui4_offset + ui4_length >= 0x2000)
   {
      CAM_CALDB("[S24CAM_CAL] Write Error!! S-24CS64A not supprt address >= 0x2000!! \n" );
      return -1;
   }

   i4ResidueDataLength = (int)ui4_length;
   u4CurrentOffset = ui4_offset;
   pBuff = pinputdata;   

   CAM_CALDB("[S24CAM_CAL] iWriteData u4CurrentOffset is %d \n",u4CurrentOffset);   

   do 
   {
       if(i4ResidueDataLength >= 6)
       {
           i4RetValue = iWriteCAM_CAL((u16)u4CurrentOffset, 6, pBuff);
           if (i4RetValue != 0)
           {
                CAM_CALDB("[CAM_CAL] I2C iWriteData failed!! \n");
                return -1;
           }           
           u4IncOffset += 6;
           i4ResidueDataLength -= 6;
           u4CurrentOffset = ui4_offset + u4IncOffset;
           pBuff = pinputdata + u4IncOffset;
       }
       else
       {
           i4RetValue = iWriteCAM_CAL((u16)u4CurrentOffset, i4ResidueDataLength, pBuff);
           if (i4RetValue != 0)
           {
                CAM_CALDB("[CAM_CAL] I2C iWriteData failed!! \n");
                return -1;
           }            
           u4IncOffset += 6;
           i4ResidueDataLength -= 6;
           u4CurrentOffset = ui4_offset + u4IncOffset;
           pBuff = pinputdata + u4IncOffset;
           //break;
       }
   }while (i4ResidueDataLength > 0);
   CAM_CALDB("[S24CAM_CAL] iWriteData done\n" );
 #endif
 
   return 0;
}



//this function can only read 0xA0 0~9 byte
/*Read R/G B/G Gb/Gr 6 byte data*/
static int iReadData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   int  i4RetValue = 0;
   int  i4ResidueDataLength;
   u32 u4IncOffset = 0;
   u32 u4CurrentOffset;
   u8 * pBuff;
   int i;
   CAM_CALDB("[S24EEPORM] iReadData \n" );
   
  /*if (ui4_offset  >8)
   {
      CAM_CALDB("[S24CAM_CAL] Read Error!! S-24CS64A not supprt address >= 0x2000!! \n" );
      return -1;
   }
   */

   i4ResidueDataLength = (int)ui4_length;
   u4CurrentOffset = ui4_offset;
   pBuff = pinputdata;
if (i4ResidueDataLength > 0)
   {
       
 
           i4RetValue = iReadCAM_CAL_A0((u16)u4CurrentOffset, i4ResidueDataLength, pBuff);

            for(i=0;i<i4ResidueDataLength;i++)
            {
            	 CAM_CALDB("[CAM_CAL] Vendor data :0x%x\n",pBuff[i]);
            }

	if (i4RetValue != 0)
           {
                CAM_CALDB("[CAM_CAL] I2C iReadData failed!! \n");
                return -1;
           }  
	    
			
      
   }
//   CAM_CALDB("[S24EEPORM] iReadData finial address is %d length is %d buffer address is 0x%x\n",u4CurrentOffset, i4ResidueDataLength, pBuff);   
//   CAM_CALDB("[S24EEPORM] iReadData done\n" );
   return 0;
}


//this function can only read 0xA0 LSC byte
/*10<=ui4_offset<=255 */
static int iReadLSCDataA0(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   int  i4RetValue = 0;
   int  i4ResidueDataLength;
   u32 u4IncOffset = 0;
   u32 u4CurrentOffset;
   u8 * pBuff;
//   CAM_CALDB("[S24EEPORM] iReadData \n" );
   
   if (ui4_offset <10||ui4_offset >255)
   {
      CAM_CALDB("[S24CAM_CAL] lsc position is not correct! \n" );
      return -1;
   }

   i4ResidueDataLength = (int)ui4_length;
   u4CurrentOffset = ui4_offset;
   pBuff = pinputdata;
   do 
   {
       if(i4ResidueDataLength >= 8)
       {
           i4RetValue = iReadCAM_CAL_A0((u16)u4CurrentOffset, 8, pBuff);
           if (i4RetValue != 0)
           {
                CAM_CALDB("[CAM_CAL] I2C iReadData failed!! \n");
                return -1;
           }           
           u4IncOffset += 8;
           i4ResidueDataLength -= 8;
           u4CurrentOffset = ui4_offset + u4IncOffset;
           pBuff = pinputdata + u4IncOffset;
       }
       else
       {
           i4RetValue = iReadCAM_CAL_A0((u16)u4CurrentOffset, i4ResidueDataLength, pBuff);
           if (i4RetValue != 0)
           {
                CAM_CALDB("[CAM_CAL] I2C iReadData failed!! \n");
                return -1;
           }  
           u4IncOffset += 8;
           i4ResidueDataLength -= 8;
           u4CurrentOffset = ui4_offset + u4IncOffset;
           pBuff = pinputdata + u4IncOffset;
           //break;
       }
   }while (i4ResidueDataLength > 0);
//   CAM_CALDB("[S24EEPORM] iReadData finial address is %d length is %d buffer address is 0x%x\n",u4CurrentOffset, i4ResidueDataLength, pBuff);   
//   CAM_CALDB("[S24EEPORM] iReadData done\n" );
   return 0;
}


//this function can only read 0xA2 LSC byte
/*0<=ui4_offset<=33 */
static int iReadLSCDataA2(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   int  i4RetValue = 0;
   int  i4ResidueDataLength;
   u32 u4IncOffset = 0;
   u32 u4CurrentOffset;
   u8 * pBuff;
//   CAM_CALDB("[S24EEPORM] iReadData \n" );
   
   if (ui4_offset <0||ui4_offset >33)
   {
      CAM_CALDB("[S24CAM_CAL] lsc position in A2  is not correct! \n" );
      return -1;
   }

   i4ResidueDataLength = (int)ui4_length;
   u4CurrentOffset = ui4_offset;
   pBuff = pinputdata;
   do 
   {
       if(i4ResidueDataLength >= 8)
       {
           i4RetValue = iReadCAM_CAL_A2((u16)u4CurrentOffset, 8, pBuff);
           if (i4RetValue != 0)
           {
                CAM_CALDB("[CAM_CAL] I2C iReadData failed!! \n");
                return -1;
           }           
           u4IncOffset += 8;
           i4ResidueDataLength -= 8;
           u4CurrentOffset = ui4_offset + u4IncOffset;
           pBuff = pinputdata + u4IncOffset;
       }
       else
       {
           i4RetValue = iReadCAM_CAL_A2((u16)u4CurrentOffset, i4ResidueDataLength, pBuff);
           if (i4RetValue != 0)
           {
                CAM_CALDB("[CAM_CAL] I2C iReadData failed!! \n");
                return -1;
           }  
           u4IncOffset += 8;
           i4ResidueDataLength -= 8;
           u4CurrentOffset = ui4_offset + u4IncOffset;
           pBuff = pinputdata + u4IncOffset;
           //break;
       }
   }while (i4ResidueDataLength > 0);
//   CAM_CALDB("[S24EEPORM] iReadData finial address is %d length is %d buffer address is 0x%x\n",u4CurrentOffset, i4ResidueDataLength, pBuff);   
//   CAM_CALDB("[S24EEPORM] iReadData done\n" );
   return 0;
}

 int iReadModuleID(unsigned char * pinputdata)
 {
	 return iReadCAM_CAL_A0(3,1,pinputdata);
 }
 int iReadLSCData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{

 int i;
  unsigned char * LSCDataA0=NULL;
    unsigned char * LSCDataA2=NULL;
  LSCDataA0 = (u8 *)kmalloc(246,GFP_KERNEL);

        if(NULL == LSCDataA0 )
        {

            CAM_CALDB("[S24CAM_CAL] iReadLSCDataA0 allocate mem failed\n");
            return -ENOMEM;
        }
  
  LSCDataA2 = (u8 *)kmalloc(34,GFP_KERNEL);


     if(NULL == LSCDataA2 )
        {

		     kfree(LSCDataA0);

            CAM_CALDB("[S24CAM_CAL] iReadLSCDataA2 allocate mem failed\n");
            return -ENOMEM;
        }
  
  
  
 iReadLSCDataA0(10, 246, LSCDataA0);
 iReadLSCDataA2(0, 34, LSCDataA2);
 
 for(i=0; i<280; i++)
 {
 		if(i < 246)
 			{
 					pinputdata[i] = LSCDataA0[i];
 			}
 			else
 				{
 					pinputdata[i] = LSCDataA2[i-246];
 				}

	 CAM_CALDB("[S24CAM_CAL] pinputdata[i] =0x%x\n",i,pinputdata[i]);		
	}

     kfree(LSCDataA0);
	     kfree(LSCDataA2);
 
 return 0;
}
/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long CAM_CAL_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;

#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
     CAM_CALDB("[S24CAM_CAL] init Working buffer address 0x%8x  command is 0x%8x\n", (u32)pWorkingBuff, (u32)a_u4Command);

 
    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:    
            CAM_CALDB("[S24CAM_CAL] Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif            
            i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[S24CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
#endif 
            CAM_CALDB("[CAM_CAL] offset %d \n", ptempbuf->u4Offset);
            CAM_CALDB("[CAM_CAL] length %d \n", ptempbuf->u4Length);
            CAM_CALDB("[CAM_CAL] Before read Working buffer address 0x%8x \n", (u32)pWorkingBuff);

            i4RetValue = iReadData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
            CAM_CALDB("[S24CAM_CAL] After read Working buffer data  0x%4x \n", *pWorkingBuff);


#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            

            break;
        default :
      	     CAM_CALDB("[S24CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        CAM_CALDB("[S24CAM_CAL] to user length %d \n", ptempbuf->u4Length);
        CAM_CALDB("[S24CAM_CAL] to user  Working buffer address 0x%8x \n", (u32)pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            CAM_CALDB("[S24CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
	CAM_CALDB("[S24CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);

#if 0
	unsigned char * pCaldata=NULL;
	pCaldata = (u8*)kmalloc(504,GFP_KERNEL); 
	if(NULL == pCaldata)
	{
		CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
		return -ENOMEM;
	}
	iReadIMX135AWBData(pCaldata);
	iReadIMX135LSCData(pCaldata);
	kfree(pCaldata);
#endif

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALDB("[S24CAM_CAL] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALDB("[S24CAM_CAL] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}


//////////////////////////////////////////////////////////////////////
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME,0},{}};   
#if 0 //test110314 Please use the same I2C Group ID as Sensor
static unsigned short force[] = {CAM_CAL_I2C_GROUP_ID, S24CS64A_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#else
//static unsigned short force[] = {IMG_SENSOR_I2C_GROUP_ID, S24CS64A_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#endif
//static const unsigned short * const forces[] = { force, NULL };              
//static struct i2c_client_address_data addr_data = { .forces = forces,}; 


static struct i2c_driver CAM_CAL_i2c_driver = {
    .probe = CAM_CAL_i2c_probe,                                   
    .remove = CAM_CAL_i2c_remove,                           
//   .detect = CAM_CAL_i2c_detect,                           
    .driver.name = CAM_CAL_DRVNAME,
    .id_table = CAM_CAL_i2c_id,                             
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, CAM_CAL_DRVNAME);
    return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {             
int i4RetValue = 0;

#ifdef CAM_CAL_TEST_LSC//pengjinqiang for test 
unsigned char * pCaldata=NULL;
#endif

    CAM_CALDB("[S24CAM_CAL] Attach I2C \n");
//    spin_lock_init(&g_CAM_CALLock);

#ifdef  CAM_CAL_TEST_LSC//pengjinqiang for test 
 pCaldata = (u8*)kmalloc(300,GFP_KERNEL); 

  if(NULL == pCaldata)
    {
        CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
#endif
    //get sensor i2c client
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = S24CS64A_DEVICE_ID>>1;
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    CAM_CALDB("[CAM_CAL] g_pstI2Cclient->addr = 0x%8x \n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterCAM_CALCharDrv();

    if(i4RetValue){
        CAM_CALDB("[S24CAM_CAL] register char device failed!\n");
        return i4RetValue;
    }

    CAM_CALDB("[S24CAM_CAL] Attached!! \n");

#ifdef CAM_CAL_TEST_LSC//pengjinqiang for test 
	   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800, "S24CS64A"))
    {
        CAM_CALDB("[S24CAM_CAL] Fail to enable analog gain\n");
        return -EIO;
    }
 // iReadData(4, 6, pCaldata);
iReadLSCData(0,280,pCaldata);
	      kfree(pCaldata);
#endif

    return 0;                                                                                       
} 

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_i2C_init(void)
{


    i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALDB("failed to register S24CAM_CAL driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALDB("failed to register S24CAM_CAL driver, 2nd time\n");
        return -ENODEV;
    }	


    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


