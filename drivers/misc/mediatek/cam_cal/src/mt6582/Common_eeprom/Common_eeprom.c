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

#include "Common_eeprom.h"
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
static struct i2c_board_info __initdata kd_eeprom_dev={ I2C_BOARD_INFO("CAM_CAL_DRV", 0xAA>>1)};

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
#define FM50AF_EEPROM_I2C_ID 0xA1


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
static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_EEPROMdevno = MKDEV(EEPROM_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pEEPROM_CharDrv = NULL;
//static spinlock_t g_EEPROMLock;
//spin_lock(&g_EEPROMLock);
//spin_unlock(&g_EEPROMLock);

static struct class *EEPROM_class = NULL;
static atomic_t g_EEPROMatomic;
//static DEFINE_SPINLOCK(kdeeprom_drv_lock);
//spin_lock(&kdeeprom_drv_lock);
//spin_unlock(&kdeeprom_drv_lock);

 

#if defined(GT24C32A)
extern int GT24C32A_EEPROM_Ioctl(    struct file *file, unsigned int a_u4Command, unsigned long a_u4Param);
#endif

#if defined(BRCB032GWZ_3)
extern int BRCB032GW_3_EEPROM_Ioctl(    struct file *file, unsigned int a_u4Command, unsigned long a_u4Param);
#endif

#if defined(BRCC064GWZ_3)
extern int BRCC064GWZ_3_EEPROM_Ioctl(    struct file *file, unsigned int a_u4Command, unsigned long a_u4Param);
#endif

#if defined(DW9716_EEPROM)
extern int DW9716_EEPROM_Ioctl(    struct file *file, unsigned int a_u4Command, unsigned long a_u4Param);
#endif

// LGE_CHANGE_S: [2015-10-05] yonghwan.lym@lge.com, HI842(ZC533) VCM,EEPROM BringUp
#if defined(ZC533)
extern int ZC533_EEPROM_Ioctl(    struct file *file, unsigned int a_u4Command, unsigned long a_u4Param);
/* [LGE_CHANGE_S] Add camera info for Component F/W version menu in mini OS, 2015-10-20, eungdoo.kim@lge.com */
extern int iReadDataFromZc533(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata);
#define MODULE_VENDOR_ID 0x7000C533
/* [LGE_CHANGE_E] Add camera info for Component F/W version menu in mini OS, 2015-10-20, eungdoo.kim@lge.com */
#endif
// LGE_CHANGE_E: [2015-10-05] yonghwan.lym@lge.com, HI842(ZC533) VCM,EEPROM BringUp

//LGE_CHANGE_S: [2015-12-15] yonghwan.lym@lge.com, OV8858 OTP Enable
#if defined(OV8858_OTP)
extern int OV8858_OTP_Ioctl(    struct file *file, unsigned int a_u4Command, unsigned long a_u4Param);
#endif
//LGE_CHANGE_E: [2015-12-15] yonghwan.lym@lge.com, OV8858 OTP Enable

/*******************************************************************************
*
********************************************************************************/
/* [LGE_CHANGE_S] Add camera info for Component F/W version menu in mini OS, 2015-10-20, eungdoo.kim@lge.com */
static struct class *camera_vendor_id_class = NULL;
static s8 main_cam_module_id = -1;
static uint16_t moduleId = 0;

static ssize_t show_LGCameraMainID(struct device *dev,struct device_attribute *attr, char *buf)	
{
    EEPROMDB("show_LGCameraMainID: main_camera_id [%d] \n", main_cam_module_id);
    switch (main_cam_module_id)
       {
        case 0x01:
        case 0x02:
        case 0x05:
        case 0x06:
        case 0x07:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id, "LGIT");
        case 0x03:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id, "Fujifilm");
        case 0x04:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id, "Minolta");
        case 0x10:
        case 0x11:
        case 0x12:
        case 0x13:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id, "Cowell");
        case 0x14:
        case 0x15:
        case 0x16:
        case 0x17:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id, "IM-tech");
        case 0x20:
        case 0x21:
        case 0x22:
        case 0x23:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id, "Sunny");
        default:
            return sprintf(buf, "id:0x%x, %s\n", main_cam_module_id&0xFF, "Reserved for future");
    }
}
static DEVICE_ATTR(vendor_id, S_IRUGO, show_LGCameraMainID, NULL);
/* [LGE_CHANGE_E] Add camera info for Component F/W version menu in mini OS, 2015-10-20, eungdoo.kim@lge.com */

/*******************************************************************************
*
********************************************************************************/
// maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt6516.c which is 8 bytes


/*******************************************************************************
*
********************************************************************************/
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
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
    ssize_t writeSize;
    u8 readTryagain=0;

#ifdef EEPROMGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    EEPROMDB("[COMMON_EEPROM]1 In to IOCTL %x %x\n",_IOC_DIR(a_u4Command),_IOC_WRITE);
    EEPROMDB("[COMMON_EEPROM]2 In to IOCTL %x %x\n",CAM_CALIOC_G_READ,CAM_CALIOC_S_WRITE);


    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            EEPROMDB("[S24EEPROM] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                EEPROMDB("[S24EEPROM] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;

    EEPROMDB("[COMMON_EEPROM] In to IOCTL %x (0x%08x)\n",ptempbuf->u4Offset,(ptempbuf->u4Offset & 0x000FFFFF));
    /* [LGE_CHANGE_S] Add camera info for Component F/W version menu in mini OS, 2015-10-20, eungdoo.kim@lge.com */
	if(!moduleId)
	{
		iReadDataFromZc533(MODULE_VENDOR_ID, 2, &moduleId);
        main_cam_module_id =  moduleId;
    }
    /* [LGE_CHANGE_E] Add camera info for Component F/W version menu in mini OS, 2015-10-20, eungdoo.kim@lge.com */
    if( (ptempbuf->u4Offset & 0x000FFFFF) == 0x00024C32)
    {
#if defined(GT24C32A)
        EEPROMDB("[GT24C32A] Jump to IOCTL \n");

        i4RetValue = GT24C32A_EEPROM_Ioctl( file,a_u4Command,a_u4Param);
#else
        EEPROMDB("[GT24C32A] Not defined in config \n");
#endif

    }
    else if( (ptempbuf->u4Offset & 0x000FFFFF) == 0x000BCB32)
    {
#if defined(BRCB032GWZ_3)
        EEPROMDB("[BRCB032GW] Jump to IOCTL \n");

        i4RetValue = BRCB032GW_3_EEPROM_Ioctl( file,a_u4Command,a_u4Param);
#else
        EEPROMDB("[BRCB032GW] Not defined in config \n");
#endif
        
    }
	else if( (ptempbuf->u4Offset & 0x000FFFFF) == 0x000BCC64)
    {
#if defined(BRCC064GWZ_3)
        EEPROMDB("[BRCC064GW] Jump to IOCTL \n");

        i4RetValue = BRCC064GWZ_3_EEPROM_Ioctl( file,a_u4Command,a_u4Param);
#else
        EEPROMDB("[BRCC064GW] Not defined in config \n");
#endif
        
    }    
    else if( (ptempbuf->u4Offset & 0x000FFFFF) == 0x000d9716)
    {
#if defined(DW9716_EEPROM)
        EEPROMDB("[DW9716EEPROM] Jump to IOCTL \n");
        i4RetValue = DW9716_EEPROM_Ioctl( file,a_u4Command,a_u4Param);
#else
        EEPROMDB("[DW9716EEPROM] Not defined in config \n");
#endif
    }
// LGE_CHANGE_S: [2015-10-05] yonghwan.lym@lge.com, HI842(ZC533) VCM,EEPROM BringUp
    else if( (ptempbuf->u4Offset & 0x000FFFFF) == 0x00C533)
    {
#if defined(ZC533)
        EEPROMDB("[ZC533] Jump to IOCTL \n");
        i4RetValue = ZC533_EEPROM_Ioctl( file,a_u4Command,a_u4Param);
#else
        EEPROMDB("[ZC533EEPROM] Not defined in config \n");
#endif
    }  
// LGE_CHANGE_S: [2015-10-05] yonghwan.lym@lge.com, HI842(ZC533) VCM,EEPROM BringUp
//LGE_CHANGE_S: [2015-12-15] yonghwan.lym@lge.com, OV8858 OTP Enable
    else if ( (ptempbuf->u4Offset & 0x000FFFFF) == 0x00008858)
    {
#if defined(OV8858_OTP)
        EEPROMDB("[OV8858OTP] Jump to IOCTL \n");
        i4RetValue = OV8858_OTP_Ioctl( file,a_u4Command,a_u4Param);
#else
        EEPROMDB("[OV8858OTP] Not defined in config \n");
#endif
    }
//LGE_CHANGE_E: [2015-12-15] yonghwan.lym@lge.com, OV8858 OTP Enable
    else
        {
            EEPROMDB("[COMMON_EEPROM] Masic number is wrong \n");
        }

    kfree(pBuff);

    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int EEPROM_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    EEPROMDB("[COMMON_EEPROM] EEPROM_Open Client %p\n",g_pstI2Cclient);
    spin_lock(&g_EEPROMLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_EEPROMLock);
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_EEPROMatomic,0);
    }
    spin_unlock(&g_EEPROMLock);

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int EEPROM_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_EEPROMLock);

    g_u4Opened = 0;

    atomic_set(&g_EEPROMatomic,0);

    spin_unlock(&g_EEPROMLock);

    return 0;
}

static const struct file_operations g_stEEPROM_fops =
{
    .owner = THIS_MODULE,
    .open = EEPROM_Open,
    .release = EEPROM_Release,
    //.ioctl = EEPROM_Ioctl
    .unlocked_ioctl = EEPROM_Ioctl
};

#define EEPROM_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterEEPROMCharDrv(void)
{
    struct device* EEPROM_device = NULL;

#if EEPROM_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_EEPROMdevno, 0, 1,EEPROM_DRVNAME) )
    {
        EEPROMDB("[S24EEPROM] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_EEPROMdevno , 1 , EEPROM_DRVNAME) )
    {
        EEPROMDB("[COMMON_EEPROM] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pEEPROM_CharDrv = cdev_alloc();

    if(NULL == g_pEEPROM_CharDrv)
    {
        unregister_chrdev_region(g_EEPROMdevno, 1);

        EEPROMDB("[COMMON_EEPROM] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pEEPROM_CharDrv, &g_stEEPROM_fops);

    g_pEEPROM_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pEEPROM_CharDrv, g_EEPROMdevno, 1))
    {
        EEPROMDB("[COMMON_EEPROM] Attatch file operation failed\n");

        unregister_chrdev_region(g_EEPROMdevno, 1);

        return -EAGAIN;
    }

    EEPROM_class = class_create(THIS_MODULE, "EEPROMdrv");
    if (IS_ERR(EEPROM_class)) {
        int ret = PTR_ERR(EEPROM_class);
        EEPROMDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    EEPROM_device = device_create(EEPROM_class, NULL, g_EEPROMdevno, NULL, EEPROM_DRVNAME);
    /* [LGE_CHANGE_S] Add camera info for Component F/W version menu in mini OS, 2015-10-20, eungdoo.kim@lge.com */
	{
		struct device* camera_vendor_id_dev;
	    camera_vendor_id_class = class_create(THIS_MODULE, "camera");
	    camera_vendor_id_dev = device_create(camera_vendor_id_class, NULL, 0, NULL, "vendor_id");
	    device_create_file(camera_vendor_id_dev, &dev_attr_vendor_id);
	}
    /* [LGE_CHANGE_E] Add camera info for Component F/W version menu in mini OS, 2015-10-20, eungdoo.kim@lge.com */
    return 0;
}

inline static void UnregisterEEPROMCharDrv(void)
{
    //Release char driver
    cdev_del(g_pEEPROM_CharDrv);

    unregister_chrdev_region(g_EEPROMdevno, 1);

    device_destroy(EEPROM_class, g_EEPROMdevno);
    class_destroy(EEPROM_class);
}


//////////////////////////////////////////////////////////////////////
#ifndef EEPROM_ICS_REVISION
static int EEPROM_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int EEPROM_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int EEPROM_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int EEPROM_i2c_remove(struct i2c_client *);

static const struct i2c_device_id EEPROM_i2c_id[] = {{EEPROM_DRVNAME,0},{}};   
#if 0 //test110314 Please use the same I2C Group ID as Sensor
static unsigned short force[] = {EEPROM_I2C_GROUP_ID, S24CS64A_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#else
//static unsigned short force[] = {IMG_SENSOR_I2C_GROUP_ID, S24CS64A_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#endif
//static const unsigned short * const forces[] = { force, NULL };              
//static struct i2c_client_address_data addr_data = { .forces = forces,}; 


static struct i2c_driver EEPROM_i2c_driver = {
    .probe = EEPROM_i2c_probe,                                   
    .remove = EEPROM_i2c_remove,                           
//   .detect = EEPROM_i2c_detect,                           
    .driver.name = EEPROM_DRVNAME,
    .id_table = EEPROM_i2c_id,                             
};

#ifndef EEPROM_ICS_REVISION
static int EEPROM_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, EEPROM_DRVNAME);
    return 0;
}
#endif
static int EEPROM_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {             
int i4RetValue = 0;
    EEPROMDB("[S24EEPROM] Attach I2C \n");
//    spin_lock_init(&g_EEPROMLock);

    //get sensor i2c client
    spin_lock(&g_EEPROMLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = COMMON_DEVICE_ID>>1;
    spin_unlock(&g_EEPROMLock); // for SMP    
    
    EEPROMDB("[COMMON_EEPROM] g_pstI2Cclient->addr = 0x%8x \n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterEEPROMCharDrv();

    if(i4RetValue){
        EEPROMDB("[COMMON_EEPROM] register char device failed!\n");
        return i4RetValue;
    }


    EEPROMDB("[COMMON_EEPROM] Attached!! \n");
    return 0;                                                                                       
} 

static int EEPROM_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int EEPROM_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&EEPROM_i2c_driver);
}

static int EEPROM_remove(struct platform_device *pdev)
{
    i2c_del_driver(&EEPROM_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stEEPROM_Driver = {
    .probe		= EEPROM_probe,
    .remove	= EEPROM_remove,
    .driver		= {
        .name	= EEPROM_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stEEPROM_Device = {
    .name = EEPROM_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init EEPROM_i2C_init(void)
{
    i2c_register_board_info(EEPROM_I2C_BUSNUM, &kd_eeprom_dev, 1);
    if(platform_driver_register(&g_stEEPROM_Driver)){
        EEPROMDB("failed to register S24EEPROM driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stEEPROM_Device))
    {
        EEPROMDB("failed to register S24EEPROM driver, 2nd time\n");
        return -ENODEV;
    }	

    return 0;
}

static void __exit EEPROM_i2C_exit(void)
{
	platform_driver_unregister(&g_stEEPROM_Driver);
}

module_init(EEPROM_i2C_init);
module_exit(EEPROM_i2C_exit);

MODULE_DESCRIPTION("EEPROM driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


