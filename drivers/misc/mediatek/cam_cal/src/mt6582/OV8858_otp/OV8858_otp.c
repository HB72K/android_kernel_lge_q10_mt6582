/*
 * Driver for EEPROM
 *
 *
 */

#if 0
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "eeprom.h"
#include "eeprom_define.h"
#include "GT24c32a.h"
#include <asm/system.h>  // for SMP
#else

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

#include "OV8858_otp.h"
#include <asm/system.h>  // for SMP
#endif
//#define EEPROMGETDLT_DEBUG
#define EEPROM_DEBUG
#ifdef EEPROM_DEBUG
#define EEPROMDB printk
#else
#define EEPROMDB(x,...)
#endif


static DEFINE_SPINLOCK(g_EEPROMLock); // for SMP
#define EEPROM_I2C_BUSNUM 1
static struct i2c_board_info __initdata kd_eeprom_dev={ I2C_BOARD_INFO("CAM_CAL_DRV", 0xB0>>1)};

/*******************************************************************************
*
********************************************************************************/
#define EEPROM_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define EEPROM_DRVNAME "CAM_CAL_DRV"
#define EEPROM_I2C_GROUP_ID 0

/*******************************************************************************
*
********************************************************************************/
//#define FM50AF_EEPROM_I2C_ID 0x28
#define FM50AF_EEPROM_I2C_ID 0x28


/*******************************************************************************
/* define LSC data for M24C08F EEPROM on L10 project */
/********************************************************************************/
#define SampleNum 221
#define Read_NUMofEEPROM 2
#define Boundary_Address 256
#define EEPROM_Address_Offset 0xC


/*******************************************************************************
*
********************************************************************************/
static unsigned short EEPROM_Address[2] = {0x0,0x0} ;
extern u16 gOTP_AWB_Data[3];

/*******************************************************************************
*
********************************************************************************/
#if defined( COMMON_CAM_CAL_DRV)
int OV8858_OTP_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#else //COMMON_CAM_CAL_DRV

#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int EEPROM_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else
static long EEPROM_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#endif


#endif//COMMON_CAM_CAL_DRV
{
    int i4RetValue = 0, ResetCheck;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
    ssize_t writeSize;
    u8 readTryagain=0;

#ifdef EEPROMGETDLT_DEBUG
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
            EEPROMDB("[OV8858OTP] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                EEPROMDB("[OV8858OTP] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
        ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
        pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    }

    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        EEPROMDB("OV8858OTP] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
     EEPROMDB("[OV8858OTP] init Working buffer address 0x%8x  command is 0x%8x\n", (u32)pWorkingBuff, (u32)a_u4Command);

    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        EEPROMDB("[OV8858OTP] ioctl copy from user failed\n");
        return -EFAULT;
    }

    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            EEPROMDB("[OV8858OTP] Write CMD, There isn't Write Case \n");
            break;
        case CAM_CALIOC_G_READ:
            EEPROMDB("[OV8858OTP] Read CMD \n");
#ifdef EEPROMGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            EEPROMDB("[OV8858OTP] offset %x \n", ptempbuf->u4Offset);
            EEPROMDB("[OV8858OTP] length %x \n", ptempbuf->u4Length);
            EEPROMDB("[OV8858OTP] Before read Working buffer address 0x%8x \n", (u32)pWorkingBuff);

            if(4 <= ptempbuf->u4Length )
			{
			    ResetCheck = *((u32 *)(ptempbuf->pu1Params)) ;
			}
            if(ptempbuf->u4Offset == 0x800)
            {
              *(u16 *)pWorkingBuff = 0x3;

            }
            else if( ((ptempbuf->u4Offset & 0x000FFFFF) == 0x00008858) && (EEPROM_Address[0]==0x0))
            {
                *(u32 *)pWorkingBuff = (ptempbuf->u4Offset | 0x10000000);
                EEPROM_Address[0] = (ptempbuf->u4Offset & 0xffff0000) >> 20;
                EEPROM_Address[1] = (ptempbuf->u4Offset & 0xffff0000) >> 20;
                EEPROMDB("[OV8858OTP] Init I2C address %x \n", *(u32 *)pWorkingBuff );
            }
            else if( ((ptempbuf->u4Offset & 0x000FFFFF) == 0x00008858) && (ResetCheck == 0xABCDFEDC) )
            {
                *(u32 *)pWorkingBuff = (ptempbuf->u4Offset | 0x10000000);
                EEPROMDB("[OV8858OTP] Reset I2C address %x \n", *(u32 *)pWorkingBuff );
            }
            else
            {
			   *(u16 *)pWorkingBuff = gOTP_AWB_Data[0];
			   *((u16 *)pWorkingBuff + 1) = gOTP_AWB_Data[1];
			   *((u16 *)pWorkingBuff + 2) = gOTP_AWB_Data[2];
            }
            EEPROMDB("[OV8858OTP] After read Working buffer data  0x%4x \n", *pWorkingBuff);



#ifdef EEPROMGETDLT_DEBUG
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
      	     EEPROMDB("[OV8858OTP] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        EEPROMDB("[OV8858OTP] to user length %d \n", ptempbuf->u4Length);
        EEPROMDB("[OV8858OTP] to user  Working buffer address 0x%8x \n", (u32)pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            EEPROMDB("[OV8858OTP] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}

#if defined( COMMON_CAM_CAL_DRV)
EXPORT_SYMBOL(OV8858_OTP_Ioctl);
#endif

MODULE_DESCRIPTION("OTP driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


