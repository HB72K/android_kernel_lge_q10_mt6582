/******************************************************************************
 * MODULE       : pas230.c
 * FUNCTION     : Driver source for PAS230,
 *              : Proximity Sensor(PS) and Ambient Light Sensor(ALS) IC.
 * AUTHOR       : Seo Ji Won < jiwon.seo@lge.com >
 * MODIFY       : Lee Kang Young < jude.lee@lge.com >
 * PROGRAMMED   : Sensing solution Group, PARTRON CO.,LTD.
 * MODIFICATION : Modified by PARTRON
 * REMARKS      : File : mediatek\custom\common\kernel\alsps\pas230.c
 * COPYRIGHT    : Copyright (C) 2015 PARTRON CO.,LTD.
 *              : This program is free software; you can redistribute it and/or
 *              : modify it under the terms of the GNU General Public License
 *              : as published by the Free Software Foundation; either version 2
 *              : of the License, or (at your option) any later version.
 *              :
 *              : This program is distributed in the hope that it will be useful,
 *              : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *              : GNU General Public License for more details.
 *              :
 *              : You should have received a copy of the GNU General Public License
 *              : along with this program; if not, write to the Free Software
 *              : Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *****************************************************************************/

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#define MT6582

#ifdef MT6582
#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#ifdef MT6582
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <cust_gpio_usage.h>
#include <mach/eint.h>

#if defined CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include "pas230.h"
#define PAS230_I2CADDR    (0x53)

#define CALB_SYSTEM_WAIT     (101)
#define CALB_IC_WAIT         (5)
#define CALB_TIMES           (2)
#define CALB_BOX_TIMES       (20)
#define CALB_REMOVAL_TIME    (5)

#define SET_IC_DISABLE       (0)
#define SET_IC_ENABLE        (1)

#define PAS230_PINT        (1 << 1)

#define PS_TH_VAL_MAX          (0x3FF) /* 10bit resolution */
#define PS_TH_VAL_MIN          (0)

enum {
	MAIN_CTRL=0x00,
	PS_LED,
	PS_PULSES,
	PS_MEAS_RATE,
	ALS_CS_MEAS_RATE,
	ALS_CS_GAIN,
	PART_ID,
	MAIN_STATUS,
	PS_DATA=0x08,
	CLEAR_DATA=0x0a,
	GREEN_DATA=0x0d,
	BLUE_DATA=0x10,	RED_DATA=0x13,	COMP_DATA=0x16,
	INT_CFG=0x19,
	INT_PST,
	PS_THRES_UP=0x1b,
	PS_THRES_LOW=0x1d,
	PS_CAN=0x1f,
	ALS_THRES_UP=0x21,
	ALS_THRES_LOW=0x24,
	ALS_THRES_VAR=0x27
};

static u8 reg_defaults[40] = {
#ifdef PAS230_ALS_SENSOR_ENABLE
	0x03, /* 0x00_0 : MAIN_CTRL */
#else
	0x01,
#endif
	0x36, /* 0x01_1 : PS_LED */
	0x32, /* 0x02_2 : PS_PULSES */
	0x55, /* 0x03_3 : PS_MEAS_RATE */
	0x22, /* 0x04_4 : ALS_CS_MEAS_RATE */
	0x01, /* 0x05_5 : ALS_CS_GAIN */
	0xb1, /* 0x06_6 : PART_ID */
	0x00, /* 0x07_7 : MAIN_STATUS */
	0x00, 0x00, /* 0x08_8 : PS_DATA */
	0x00, 0x00, 0x00, /* 0x0a_10 : CLEAR_DATA */
	0x00, 0x00, 0x00, /* 0x0d_13 : GREEN_DATA */
	0x00, 0x00, 0x00, /* 0x10_16 : BLUE_DATA */
	0x00, 0x00, 0x00, /* 0x13_19 : RED_DATA */
	0x00, 0x00, 0x00, /* 0x16_22 : COMP_DATA */	
#ifdef PAS230_ALS_SENSOR_INT
	0x15, /* 0x19_25 : INT_CFG ALS/PS INT ENABLE */
#else
	0x11, /* 0x19_25 : INT_CFG ONLY PS ENABLE*/
#endif
	0x11, /* 0x1a_26 : INT_PST */
	0x14, 0x00, /* 0x1b_27 : PS_THRES_UP, 2047_80 */
	0x0a, 0x00, /* 0x1d_29 : PS_THRES_LOW, 0_65 */
	0x00, 0x00, /* 0x1f_31 : PS_CAN, 2047_0 */
	0xff, 0xff, 0x0f, /* 0x21_33 : ALS_THRES_UP */
	0x00, 0x00, 0x00, /* 0x24_36 : ALS_THRES_LOW */
	0x00, /* 0x27_39 : ALS_THRES_VAR */
};

#define PS_ON		(reg_defaults[0]&0x01)
#define PS_OFF		(reg_defaults[0]&(0x01^0xff))
#define ALS_CS_ON	(reg_defaults[0]&0x02)
#define ALS_CS_OFF	(reg_defaults[0]&(0x02^0xff))
#define ALL_ON		(reg_defaults[0]&0x03)
#define ALL_OFF		(reg_defaults[0]&(0x03^0xff))

#define PAS230_PS_MEAS_RATE_6_25MS	0x01
#define PAS230_PS_MEAS_RATE_100MS	0x05
#define PAS230_PS_MEAS_RATE_200MS	0x06

#ifdef PAS230_ALS_SENSOR_INT
#define ALS_INT_MASK		(0x10)
#define ALS_THRESHOLD_MAX	0x07ffff
#define ALS_THRESHOLD_MIN	0
#define ALS_THRESHOLD_UP	19
#define ALS_THRESHOLD_LOW	19
#define ALS_LUX_THRESHOLD	4	/* Matching the Pocket Detection thresold  */
#define LUX_VAR_PERCENT		1
#define ALS_LUX_COEFF		12
static atomic_t irq_status = ATOMIC_INIT(-1);
static atomic_t driver_suspend_flag = ATOMIC_INIT(0);
static atomic_t ps_int_flag = ATOMIC_INIT(0);
#endif

enum {
#ifdef PAS230_ALS_SENSOR_ENABLE
	LIGHT_ENABLED = BIT(0),
#endif
	PROXIMITY_ENABLED = BIT(1),
};

static int pas230_set_ps_enable ( struct i2c_client *client, unsigned char enable );
static int pas230_set_ps_led ( struct i2c_client *client, unsigned char ps_led );
static int pas230_get_ps_led ( struct i2c_client *client, unsigned char *ps_led );
static int pas230_set_ps_pulse ( struct i2c_client *client, unsigned char ps_pulse );
static int pas230_get_ps_pulse ( struct i2c_client *client, unsigned char *ps_pulse );
static int pas230_set_ps_meas_rate ( struct i2c_client *client, unsigned char ps_meas_rate );
static int pas230_get_ps_meas_rate ( struct i2c_client *client, unsigned char *ps_meas_rate );
static int pas230_set_ps_int_cfg ( struct i2c_client *client, unsigned char ps_int_cfg );
static int pas230_get_ps_int_cfg ( struct i2c_client *client, unsigned char *ps_int_cfg );
static int pas230_set_ps_int_pst ( struct i2c_client *client, unsigned char int_pst );
static int pas230_get_ps_int_pst ( struct i2c_client *client, unsigned char *int_pst );
static int pas230_set_ps_can_0 ( struct i2c_client *client, unsigned char ps_can );
static int pas230_get_ps_can_0 ( struct i2c_client *client, unsigned char *ps_can );
static int pas230_set_ps_can_1 ( struct i2c_client *client, unsigned char ps_can );
static int pas230_get_ps_can_1 ( struct i2c_client *client, unsigned char *ps_can );
static int pas230_set_pilt ( struct i2c_client *client, unsigned short threshold );
static int pas230_set_piht ( struct i2c_client *client, unsigned short threshold );
static int pas230_get_status ( struct i2c_client *client, unsigned char *pData );
static int pas230_get_ctrl ( struct i2c_client *client, unsigned char *pData );
static int pas230_get_pdata ( struct i2c_client *client, unsigned short *pData );
static int pas230_get_deivceid( struct i2c_client *client, unsigned char *pData );
static int pas230_reduce_ps_int_rate( struct i2c_client *client, unsigned char enable );
#ifdef PAS230_ALS_SENSOR_ENABLE
static int pas230_set_als_meas_rate ( struct i2c_client *client, unsigned char ps_meas_rate );
static int pas230_get_als_meas_rate ( struct i2c_client *client, unsigned char *ps_meas_rate );
static int pas230_set_als_cs_gain ( struct i2c_client *client, unsigned char als_cs_gain );
static int pas230_get_als_cs_gain ( struct i2c_client *client, unsigned char *als_cs_gain );
static int pas230_get_alsdata( struct i2c_client *client );
#endif


static int pas230_proximity_rsp ( struct i2c_client *client, int val );
static long pas230_initialize ( struct i2c_client *client  );
static long pas230_ps_enable ( struct i2c_client *client  );
static long pas230_ps_disable ( struct i2c_client *client  );
static void pas230_swap(int *x, int *y);
static int pas230_do_calibration ( struct i2c_client *client, int *value );
static unsigned int pas230_calc_calibration ( struct i2c_client *client );

static long pas230_ps_activate ( struct i2c_client *client, int enable );

#ifdef PAS230_ALS_SENSOR_ENABLE
static long pas230_als_activate ( struct i2c_client *client, int enable );
#endif

static void pas230_eint_func ( void );
static void pas230_eint_work ( struct work_struct *work );

static ssize_t pas230_show_cali_value ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_cali_value ( struct device_driver *dev, char *buf, size_t count );
static ssize_t pas230_show_ps_led ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_ps_led ( struct device_driver *dev, char *buf, size_t count );
static ssize_t pas230_show_ps_pulse ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_ps_pulse ( struct device_driver *dev, char *buf, size_t count );
static ssize_t pas230_show_ps_meas_rate ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_ps_meas_rate ( struct device_driver *dev, char *buf, size_t count );
static ssize_t pas230_show_pilt ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_pilt ( struct device_driver *dev, char *buf, size_t count );
static ssize_t pas230_show_piht ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_piht ( struct device_driver *dev, char *buf, size_t count );
static ssize_t pas230_show_int_cfg ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_int_cfg ( struct device_driver *dev, char *buf, size_t count );
static ssize_t pas230_show_ps_can ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_ps_can ( struct device_driver *dev, char *buf, size_t count );

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define PAS230_DEV_NAME     "PAS230"
static DEFINE_MUTEX(pas230_access);

/****************************************************************************
 * Macros
 ****************************************************************************/
#ifdef PAS230_ALS_SENSOR_ENABLE
#define SENSOR_TAG					"[LGE_ALS/PS]"
#else
#define SENSOR_TAG                  "[LGE_Proximity]"
#endif

#ifdef CONFIG_MT_ENG_BUILD
#define DEBUG 1
#endif

#ifdef DEBUG
#define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
#define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt, __FUNCTION__,##args)
#define SENSOR_DBG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[D]""%s : "fmt, __FUNCTION__,##args)
#else
#define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
#define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt, __FUNCTION__,##args)
#define SENSOR_DBG(fmt, args...)    NULL
#endif

#ifdef CONFIG_OF_DT
static const struct of_device_id psensor_of_match[] = {
	{ .compatible = "mediatek,als_ps", },
	{},
};
#endif

/****************************************************************************
* Type Definitions
****************************************************************************/
typedef enum
{
	PS_NEAR = 0,
	PS_FAR = 1,
	PS_UNKNOWN = 2
} PS_STATUS;

struct pas230_priv
{
	struct i2c_client *client;
	struct work_struct eint_work;

	unsigned int activate; /* 1 = activate, 0 = deactivate */

	/* variables to store register value - begin */
	unsigned int enable;
	unsigned int pilt;
	unsigned int piht;
    unsigned int interrupt;

    unsigned int ps_led;
    unsigned int ps_pulse;
	unsigned int ps_meas_rate;
#ifdef PAS230_ALS_SENSOR_ENABLE
	unsigned int als_meas_rate;
	unsigned int als_cs_gain;
#endif
	unsigned int ps_int_cfg;
	unsigned int ps_int_pst;
	unsigned int ps_can_0;
	unsigned int ps_can_1;
	/* variables to store register value - end */

	u16 ps;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
#ifdef PAS230_ALS_SENSOR_ENABLE
	unsigned int enable_als_sensor;
#endif

	unsigned int ps_status; /* current status of poximity detection : 0 = near, 1 = far */

	/* threshold value to detect "near-to-far" event */
	unsigned int far_threshold;
	unsigned int near_threshold;

	unsigned int ps_cross_talk; /* a result value of calibration. it will be used to compensate threshold value. */
#ifdef PAS230_ALS_SENSOR_INT
	int als_data;
	unsigned int als_low_threshold;
	unsigned int als_up_threshold;
#endif

#if defined CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_drv;
#endif

#if defined CONFIG_FB
	struct notifier_block fb_notif;
#endif
};


/****************************************************************************
* Variables
****************************************************************************/
static struct i2c_client *pas230_i2c_client = NULL; /* for general file I/O service. will be init on pas230_i2c_probe() */
static struct pas230_priv *g_pas230_ptr = NULL; /* for interrupt service call. will be init on pas230_i2c_probe() */
static struct platform_driver pas230_alsps_driver;

static int near_offset = 200 ; /* parameter for taget pdata setting */
static int far_offset = 85 ;    /* parameter for far detection */
static int ps_cross_talk_max = 500;
static unsigned int ps_cross_talk_default=100;
static unsigned int ps_cross_talk_offset=50;
/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
extern void mt_eint_unmask ( unsigned int line );
extern void mt_eint_mask ( unsigned int line );
extern void mt_eint_set_polarity ( unsigned int eint_num, unsigned int pol );
extern void mt_eint_set_hw_debounce ( unsigned int eint_num, unsigned int ms );
extern unsigned int mt_eint_set_sens ( unsigned int eint_num, unsigned int sens );
void mt_eint_registration(unsigned int eint_num, unsigned int flag,
              void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);


/****************************************************************************
* Local Function Prototypes
****************************************************************************/
void pas230_eint_func ( void );

/****************************************************************************
* Local Functions
****************************************************************************/

//==========================================================
// Platform(AP) dependent functions
//==========================================================
static void pas230_setup_eint ( void )
{
	SENSOR_FUN ();

	mt_set_gpio_mode(GPIO_PROXIMITY_INT, GPIO_PROXIMITY_INT_M_EINT);
	mt_set_gpio_dir(GPIO_PROXIMITY_INT, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_PROXIMITY_INT, GPIO_PULL_DISABLE);

	/* Configure external interrupt settings for external interrupt pin */
	mt_eint_set_hw_debounce(CUST_EINT_PROXIMITY_NUM, CUST_EINT_PROXIMITY_DEBOUNCE_EN);
	mt_eint_registration(CUST_EINT_PROXIMITY_NUM, EINTF_TRIGGER_FALLING, pas230_eint_func, 0);

	/* Mask external interrupt to avoid un-wanted interrupt. Unmask it after initialization */
	mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );

}

//==========================================================
// PAS230 Register Read / Write Funtions
//==========================================================
static int pas230_write_byte ( struct i2c_client *client, u8 reg, u8 val )
{
	int res = 0;
	u8 pBuf[2] = { 0 };

    mutex_lock(&pas230_access);

	pBuf[0] = reg;
	pBuf[1] = val;

	res = i2c_master_send ( client, pBuf, 2 );
	if ( res == 2 )
	{
		mutex_unlock(&pas230_access);
		return PAS230_SUCCESS;
	}
	else
	{
		SENSOR_ERR ( "failed to write to PAS230 ( err=%d, reg=0x%02x, val=0x%02x )\n", res, reg, val );
        mutex_unlock(&pas230_access);
		return PAS230_ERR_I2C;
	}

}


static int pas230_write_word ( struct i2c_client *client, u8 reg, u16 val )
{
	int res = 0;
	u8 pBuf[3] = { 0 };

    mutex_lock(&pas230_access);

	pBuf[0] = reg ;
	pBuf[1] = val & 0xFF ;
	pBuf[2] = ( val >> 8 ) & 0xFF ;

	res = i2c_master_send ( client, pBuf, 3 );
	if ( res == 3 )
	{
		mutex_unlock(&pas230_access);
		return PAS230_SUCCESS;
	}
	else
	{
		SENSOR_ERR ( "failed to write to PAS230 ( err=%d, reg=0x%02x, val=0x%04x )\n", res, reg, val );
        mutex_unlock(&pas230_access);
		return PAS230_ERR_I2C;
	}

}


static int pas230_read_byte ( struct i2c_client *client, u8 reg, u8 *pVal )
{
	int res = 0;

    mutex_lock(&pas230_access);

	if ( pVal == NULL )
	{
		SENSOR_ERR ( "invalid input ( pVal=NULL )" );
		goto EXIT_ERR;
	}

	res = i2c_master_send ( client, &reg, 1 );
	if ( res != 1 )
	{
		SENSOR_ERR ( "pas230_read_byte error i2c_master_send (1)....\n" );
		goto EXIT_ERR;
	}

	res = i2c_master_recv ( client, pVal, 1 );
	if ( res != 1 )
	{
		SENSOR_ERR ( "pas230_read_byte error i2c_master_recv (2)....\n" );
		goto EXIT_ERR;
	}

    mutex_unlock(&pas230_access);
	return PAS230_SUCCESS;

	EXIT_ERR:
	SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, reg );
    mutex_unlock(&pas230_access);
	return PAS230_ERR_I2C;
}

static int pas230_read_word ( struct i2c_client *client, u8 reg, u16 *pVal )
{
	int res = 0;
	u8 pBuf[2] = { 0 };

    mutex_lock(&pas230_access);

	if ( pVal == NULL )
	{
		SENSOR_ERR ( "invalid input ( pVal=NULL )" );
		goto EXIT_ERR;
	}

	res = i2c_master_send ( client, &reg, 1 );
	if ( res != 1 )
	{
		goto EXIT_ERR;
	}

	res = i2c_master_recv ( client, pBuf, 2 );
	if ( res != 2 )
	{
		goto EXIT_ERR;
	}

	*pVal = ( ( u16 ) pBuf[1] << 8 ) | pBuf[0] ;

	SENSOR_DBG ( "I2C read ( reg=0x%02x, val=0x%04x )\n", reg, *pVal );
    mutex_unlock(&pas230_access);
	return PAS230_SUCCESS;

	EXIT_ERR:
	SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, reg );
    mutex_unlock(&pas230_access);
	return PAS230_ERR_I2C;
}

//==========================================================
// PAS230 Basic Read / Write Funtions
//==========================================================
static int pas230_set_ps_enable ( struct i2c_client *client, unsigned char enable )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res;
	u8 tmp;

	res = pas230_read_byte ( client, MAIN_CTRL, &tmp );
	if ( res != PAS230_SUCCESS )
		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, MAIN_CTRL );
	else {
		if( enable == SET_IC_ENABLE ) {
			res = pas230_write_byte(client, MAIN_CTRL, tmp | PS_ON);
			SENSOR_DBG("ENABLE PS MAIN_CTRL=0x%02x\n", tmp | PS_ON);
		} else {
			res = pas230_write_byte(client, MAIN_CTRL, tmp & PS_OFF);
			SENSOR_DBG("DISABLE PS MAIN_CTRL=0x%02x\n", tmp & PS_OFF);
		}

		if ( res == PAS230_SUCCESS ) {
				obj->enable_ps_sensor = (unsigned int)enable;
		} else {
			SENSOR_ERR ( "failed to write to PAS230 ( err=%d, reg=0x%02x )\n", res, MAIN_CTRL );
		}
	}

    return res;
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static int pas230_set_als_enable( struct i2c_client *client, unsigned char enable )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res;
	u8 tmp;

	res = pas230_read_byte ( client, MAIN_CTRL, &tmp );
	if ( res != PAS230_SUCCESS )
		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, MAIN_CTRL );
	else {
		if(enable == 1) {
			res = pas230_write_byte(client, MAIN_CTRL, tmp | ALS_CS_ON);
			SENSOR_DBG("ENABLE ALS MAIN_CTRL=0x%02x\n", tmp | ALS_CS_ON);
		} else {
			res = pas230_write_byte(client, MAIN_CTRL, tmp & ALS_CS_OFF);
			SENSOR_DBG("DISABLE ALS MAIN_CTRL=0x%02x\n", tmp & ALS_CS_OFF);
		}

		if ( res == PAS230_SUCCESS ) {
		   obj->enable_als_sensor = (unsigned int)enable;
		} else {
		   SENSOR_ERR ( "failed to write to PAS230 ( err=%d, reg=0x%02x )\n", res, MAIN_CTRL );
		}
	}

    return res;
}
#endif

static int pas230_set_ps_led ( struct i2c_client *client, unsigned char ps_led )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;
    
   	res = pas230_write_byte ( client, PS_LED, ( u8 )ps_led );
	if ( res == PAS230_SUCCESS ) {
	    obj->ps_led = (unsigned int)ps_led;
	} else {
	  SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_LED );
	}

	return res;
}

static int pas230_get_ps_led ( struct i2c_client *client, unsigned char *ps_led )
{
	int	res = 0;

	res = pas230_read_byte ( client, PS_LED, ( u8 * )ps_led );
	if ( res != PAS230_SUCCESS )
		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_LED );

	return res;
}


static int pas230_set_ps_pulse ( struct i2c_client *client, unsigned char ps_pulse )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = pas230_write_byte ( client, PS_PULSES, ( u8 )ps_pulse );
	if ( res == PAS230_SUCCESS ) {
		obj->ps_pulse = (unsigned int)ps_pulse;
	} else {
		SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_PULSES );
	}

	return res;
}

static int pas230_get_ps_pulse ( struct i2c_client *client, u8 *ps_pulse )
{
	int res = 0;

	res = pas230_read_byte ( client, PS_PULSES, ( u8 * )ps_pulse );
	if ( res != PAS230_SUCCESS )
		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_PULSES );

	return res;
}


static int pas230_set_ps_meas_rate ( struct i2c_client *client, unsigned char ps_meas_rate )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = pas230_write_byte ( client, PS_MEAS_RATE, ( u8 )ps_meas_rate );
	if ( res == PAS230_SUCCESS ) {
		obj->ps_meas_rate = (unsigned int)ps_meas_rate;
	} else {
		SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_MEAS_RATE );
	}

	return res;
}

static int pas230_get_ps_meas_rate ( struct i2c_client *client, u8 *ps_meas_rate )
{
	int res = 0;

	res = pas230_read_byte ( client, PS_MEAS_RATE, ( u8 * )ps_meas_rate );
	if ( res != PAS230_SUCCESS )
		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_MEAS_RATE );

	return res;
}

static int pas230_set_als_meas_rate ( struct i2c_client *client, unsigned char als_meas_rate )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = pas230_write_byte ( client, ALS_CS_MEAS_RATE, ( u8 )als_meas_rate );
	if ( res == PAS230_SUCCESS ) {
		obj->als_meas_rate = (unsigned int)als_meas_rate;
	} else {
		SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, ALS_CS_MEAS_RATE );
	}

	return res;
}

static int pas230_get_als_meas_rate ( struct i2c_client *client, u8 *als_meas_rate )
{
	int res = 0;

	res = pas230_read_byte ( client, ALS_CS_MEAS_RATE, ( u8 * )als_meas_rate );
	if ( res != PAS230_SUCCESS )
		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, ALS_CS_MEAS_RATE );

	return res;
}

static int pas230_set_als_cs_gain ( struct i2c_client *client, unsigned char als_cs_gain )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = pas230_write_byte ( client, ALS_CS_GAIN, ( u8 )als_cs_gain );
	if ( res == PAS230_SUCCESS ) {
		obj->als_cs_gain = (unsigned int)als_cs_gain;
	} else {
		SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, ALS_CS_GAIN );
	}

	return res;
}

static int pas230_get_als_cs_gain ( struct i2c_client *client, u8 *als_cs_gain )
{
	int res = 0;

	res = pas230_read_byte ( client, ALS_CS_GAIN, ( u8 * )als_cs_gain );
	if ( res != PAS230_SUCCESS )
		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, ALS_CS_GAIN );

	return res;
}

static int pas230_set_ps_int_cfg ( struct i2c_client *client, unsigned char ps_int_cfg )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = pas230_write_byte ( client, INT_CFG, ( u8 )ps_int_cfg );
	if ( res == PAS230_SUCCESS ) {
		obj->ps_int_cfg = (unsigned int)ps_int_cfg;
	} else {
		SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, INT_CFG );
	}

	return res;
}

static int pas230_get_ps_int_cfg ( struct i2c_client *client, unsigned char *ps_int_cfg )
{
	int res = 0;

	res = pas230_read_byte ( client, INT_CFG, ( u8 * )ps_int_cfg );
	if ( res != PAS230_SUCCESS )
		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, INT_CFG );

	return res;
}

static int pas230_set_ps_int_pst ( struct i2c_client *client, unsigned char int_pst )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int	res = 0;

	res = pas230_write_byte ( client, INT_PST, ( u8 )int_pst );
	if ( res == PAS230_SUCCESS ) {
		obj->ps_int_pst = (unsigned int)int_pst;
	} else {
		SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, INT_PST );
	}

	return res;
}

static int pas230_get_ps_int_pst ( struct i2c_client *client, unsigned char *int_pst )
{
	int	res = 0;

	res = pas230_read_byte ( client, INT_PST, ( u8 * )int_pst );
	if ( res != PAS230_SUCCESS )
		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, INT_PST );
    	
	return res;
}

static int pas230_set_ps_can_0 ( struct i2c_client *client, unsigned char ps_can )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = pas230_write_byte ( client, PS_CAN, ( u8 )ps_can );
	if ( res == PAS230_SUCCESS ) {
    	    obj->ps_can_0 = (unsigned int)ps_can;
    	}
    	else{
  	      SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_CAN );
    	}

	return res;
}

static int pas230_get_ps_can_0 ( struct i2c_client *client, unsigned char *ps_can )
{
	int	res = 0;

	res = pas230_read_byte ( client, PS_CAN, ( u8 * )ps_can );
	if ( res != PAS230_SUCCESS )
		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_CAN );

	return res;
}

static int pas230_set_ps_can_1 ( struct i2c_client *client, unsigned char ps_can )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int	res = 0;

	res = pas230_write_byte ( client, PS_CAN+1, ( u8 )ps_can );
	if ( res == PAS230_SUCCESS ) {
		obj->ps_can_1 = (unsigned int)ps_can;
	} else {
		SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_CAN+1 );
	}

	return res;
}

static int pas230_get_ps_can_1 ( struct i2c_client *client, unsigned char *ps_can )
{
	int res = 0;

	res = pas230_read_byte ( client, PS_CAN+1, ( u8 * )ps_can );
	if ( res != PAS230_SUCCESS )
		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_CAN+1 );

	return res;
}

static int pas230_get_pilt(struct i2c_client *client)
{
	u8 data[2] = {0,};
	int val = 0;
	int res = 0;

	res = pas230_read_byte(client, PS_THRES_LOW, data);
	if(res != PAS230_SUCCESS){
		SENSOR_ERR("failed read from PAS230( err=%d, reg=0x%02x )\n", res, PS_THRES_LOW);
	}

	res = pas230_read_byte(client, PS_THRES_LOW+1, data+1);
	if(res != PAS230_SUCCESS){
		SENSOR_ERR("failed read from PAS230( err=%d, reg=0x%02x )\n", res, PS_THRES_LOW+1);
	}

	SENSOR_DBG("PS_THRES_LOW = 0x%02x", data[0]);
	SENSOR_DBG("PS_THRES_LOW+1 = 0x%02x", data[1]);

	val = data[0] | (data[1] << 8);
	return val;
}

static int pas230_get_piht(struct i2c_client *client)
{
	u8 data[2] = {0,};
	int val = 0;
	int res = 0;

	res = pas230_read_byte(client, PS_THRES_UP, data);
	if(res != PAS230_SUCCESS){
		SENSOR_ERR("failed read from PAS230( err=%d, reg=0x%02x )\n", res, PS_THRES_UP);
	}

	res = pas230_read_byte(client, PS_THRES_UP+1, data+1);
	if(res != PAS230_SUCCESS){
		SENSOR_ERR("failed read from PAS230( err=%d, reg=0x%02x )\n", res, PS_THRES_UP+1);
	}

	SENSOR_DBG("PS_THRES_UP = 0x%02x", data[0]);
	SENSOR_DBG("PS_THRES_UP+1 = 0x%02x", data[1]);

	val = data[0] | (data[1] << 8);
	return val;
}

static int pas230_set_pilt ( struct i2c_client *client, unsigned short threshold )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;
	u8 LSB,MSB;

	LSB = threshold & 0x00FF ;
	MSB = (threshold & 0x0F00) >> 8;

	res = pas230_write_byte ( client, PS_THRES_LOW, LSB );
	res = pas230_write_byte ( client, PS_THRES_LOW+1, MSB );

	if ( res == PAS230_SUCCESS ) {
		obj->pilt = (unsigned int)threshold;
	}

	return res;
}

static int pas230_set_piht ( struct i2c_client *client, unsigned short threshold )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;
	u8 LSB,MSB;

	LSB = threshold & 0x00FF ;
	MSB = (threshold & 0x0F00) >> 8;

	res = pas230_write_byte ( client, PS_THRES_UP, LSB );
	res = pas230_write_byte ( client, PS_THRES_UP+1, MSB );

	if ( res == PAS230_SUCCESS ) {
		obj->piht = (unsigned int)threshold;
	}

	return res;
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static int pas230_set_als_thres_up( struct i2c_client *client, int threshold)
{
	struct  pas230_priv *obj = i2c_get_clientdata( client );
	int res = 0;
	u8 als_thres_up[3] = {0, };

	als_thres_up[0] = (u8)threshold; 				/* LSB */
	als_thres_up[1] = (u8)(threshold>>8); 			/* Intervening byte*/
	als_thres_up[2] = (u8)((threshold>>16) & 0x07); /* MSB */

	res = pas230_write_byte ( client, ALS_THRES_UP,	  als_thres_up[0] );
	res = pas230_write_byte ( client, ALS_THRES_UP+1, als_thres_up[1] );
	res = pas230_write_byte ( client, ALS_THRES_UP+2, als_thres_up[2] );

	if( res == PAS230_SUCCESS) {
		obj-> als_up_threshold = threshold;
	}

	return res;
}

static int pas230_set_als_thres_low( struct i2c_client *client, int threshold)
{
	struct  pas230_priv *obj = i2c_get_clientdata( client );
	int res = 0;
	u8 als_thres_low[3] = {0, };

	als_thres_low[0] = (u8)threshold; 				/* LSB */
	als_thres_low[1] = (u8)(threshold>>8); 			/* Intervening byte*/
	als_thres_low[2] = (u8)((threshold>>16) & 0x07);/* MSB */

	res = pas230_write_byte ( client, ALS_THRES_LOW,   als_thres_low[0] );
	res = pas230_write_byte ( client, ALS_THRES_LOW+1, als_thres_low[1] );
	res = pas230_write_byte ( client, ALS_THRES_LOW+2, als_thres_low[2] );

	if( res == PAS230_SUCCESS) {
		obj-> als_low_threshold = threshold;
	}

	return res;
}


#endif

static int pas230_get_status ( struct i2c_client *client, unsigned char *pData )
{
	int res = 0;

	res = pas230_read_byte ( client, MAIN_STATUS, ( u8 * ) pData );
	if ( res == PAS230_SUCCESS )
	{
		SENSOR_DBG ( "MAIN_STATUS=0x%02x\n", *pData );
	}

	return res;
}

static int pas230_get_ctrl ( struct i2c_client *client, unsigned char *pData )
{
	int res = 0;

	res = pas230_read_byte ( client, MAIN_CTRL, ( u8 * ) pData );
	if ( res == PAS230_SUCCESS )
	{
		SENSOR_LOG ( "MAIN_CTRL=0x%02x\n", *pData );
	}

	return res;
}

static int pas230_get_pdata ( struct i2c_client *client, unsigned short *pData )
{
	int res = 0;

	res = pas230_read_word ( client, PS_DATA, ( u16 * ) pData );
	*pData = *pData&0x3FF;

	if ( res == PAS230_SUCCESS )
	{
		SENSOR_DBG ( "PDATA=%d\n", *pData );
	}

	return res;
}

static int pas230_get_deivceid( struct i2c_client *client, unsigned char *pData )
{
	int res = 0;

	res = pas230_read_byte ( client, PART_ID, ( u8 * )pData );
	if ( res == PAS230_SUCCESS )
	{
		SENSOR_DBG ( "DEVICEID=0x%02x\n", *pData );
	}

	return res;
}

static int pas230_reduce_ps_int_rate( struct i2c_client *client, unsigned char enable )
{
	int res = 0;

	if( enable ) {
		reg_defaults[PS_MEAS_RATE] = (reg_defaults[PS_MEAS_RATE] & 0xf8) | PAS230_PS_MEAS_RATE_6_25MS;
		res = pas230_set_ps_meas_rate(client, reg_defaults[PS_MEAS_RATE]);
	} else {
		reg_defaults[PS_MEAS_RATE] = (reg_defaults[PS_MEAS_RATE] & 0xf8) | PAS230_PS_MEAS_RATE_100MS;
		res = pas230_set_ps_meas_rate(client, reg_defaults[PS_MEAS_RATE]);
	}

	return res;
}

//==========================================================
// pas230 Data Processign Funtions
//==========================================================
static int pas230_proximity_rsp ( struct i2c_client *client, int val)
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int ps_status = obj->ps_status;

	if( (ps_status == PS_FAR) && (val >= obj->near_threshold) ) {
		ps_status = PS_NEAR;
		pas230_set_pilt ( client, obj->far_threshold );
		pas230_set_piht ( client, PS_TH_VAL_MAX );
	} else if( (ps_status == PS_FAR) && (val < obj->near_threshold) ) {
		ps_status = PS_FAR;
		pas230_set_pilt ( client, PS_TH_VAL_MIN );
		pas230_set_piht ( client, obj->near_threshold );
	} else if( (ps_status == PS_NEAR) && (val >= obj->far_threshold) ) {
		ps_status = PS_NEAR;
		pas230_set_pilt ( client, obj->far_threshold );
		pas230_set_piht ( client, PS_TH_VAL_MAX );
	} else if( (ps_status == PS_NEAR) && (val < obj->far_threshold) ) {
		ps_status = PS_FAR;
		pas230_set_pilt ( client, PS_TH_VAL_MIN );
		pas230_set_piht ( client, obj->near_threshold );
	}

	obj->ps_status = ps_status;

	return ps_status;
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static int pas230_als_rsp( struct i2c_client *client )
{
	struct pas230_priv *obj = i2c_get_clientdata( client );
	unsigned int als_up_threshold = 0;
	unsigned int als_low_threshold = 0;

	if(atomic_read(&driver_suspend_flag) == PAS230_TRUE){
		if(obj->als_data > ALS_LUX_THRESHOLD) {
			SENSOR_LOG("als_data(%d)>THRESOLD(%d), als state is changed dark to bright", obj->als_data, ALS_LUX_THRESHOLD);
			als_up_threshold = ALS_THRESHOLD_MAX;
			als_low_threshold = ALS_THRESHOLD_LOW;
		} else if(obj->als_data <= ALS_LUX_THRESHOLD) {
			SENSOR_LOG("als_data(%d)<=THRESOLD(%d), als state is changed brigh to dark", obj->als_data, ALS_LUX_THRESHOLD);
			als_up_threshold = ALS_THRESHOLD_UP;
			als_low_threshold = ALS_THRESHOLD_MIN;
		} else{
			SENSOR_LOG("als state is not changed. als_data(%d)", obj->als_data);
		}
	} else {
		if( obj->als_data > ALS_THRESHOLD_MAX ) {
			als_up_threshold = ALS_THRESHOLD_MAX;
			als_low_threshold = (obj->als_data * (100-LUX_VAR_PERCENT)) / 100 ;
		} else {
			als_up_threshold = (obj->als_data * (100+LUX_VAR_PERCENT)) / 100 ;
			als_low_threshold = (obj->als_data * (100-LUX_VAR_PERCENT)) / 100 ;
		}
	}

	pas230_set_als_thres_up( client, als_up_threshold );
	pas230_set_als_thres_low( client, als_low_threshold);

	return 0;
}
#endif

static long pas230_initialize ( struct i2c_client *client  )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;
	unsigned char id = 0;
	SENSOR_FUN();
    res = pas230_get_deivceid(client, &id);
	if ( res != PAS230_SUCCESS )
	{
		SENSOR_ERR ( "failed to read Device ID and it means I2C error happened\n");
		return res;
	}
	 
	SENSOR_LOG ( "PAS230 Device ID = 0x%02x\n", id );

	obj->enable_ps_sensor = 0;

	/* disable proximity */
	pas230_set_ps_enable(client,SET_IC_DISABLE);
#ifdef PAS230_ALS_SENSOR_ENABLE
	/* disable ALS */
	obj->enable_als_sensor = 0;
	pas230_set_als_enable(client, SET_IC_DISABLE);
#endif

	/* initialize registers of proximity */
	pas230_set_ps_led ( client, reg_defaults[PS_LED]);
	pas230_set_ps_pulse ( client, reg_defaults[PS_PULSES] );
	pas230_set_ps_meas_rate ( client, reg_defaults[PS_MEAS_RATE]);
	pas230_set_ps_int_cfg ( client, reg_defaults[INT_CFG]);
	pas230_set_ps_int_pst ( client, reg_defaults[INT_PST]);
	pas230_set_ps_can_0 ( client, reg_defaults[PS_CAN] );
	pas230_set_ps_can_1 ( client, reg_defaults[PS_CAN+1] );

#if PAS230_ALS_SENSOR_ENABLE
	pas230_set_als_cs_gain( client, reg_defaults[ALS_CS_GAIN] );
	pas230_set_als_meas_rate( client, reg_defaults[ALS_CS_MEAS_RATE] );

	res = pas230_write_byte(client, ALS_THRES_UP, reg_defaults[ALS_THRES_UP]);
	res = pas230_write_byte(client, ALS_THRES_UP+1, reg_defaults[ALS_THRES_UP+1]);
	res = pas230_write_byte(client, ALS_THRES_UP+2, reg_defaults[ALS_THRES_UP+2]);
	if ( res != PAS230_SUCCESS ) {
		SENSOR_ERR ( "failed to write ( err=%d, reg=0x%02x )\n", res, ALS_THRES_UP );
		return res;
	}

	res = pas230_write_byte ( client, ALS_THRES_LOW,	reg_defaults[ALS_THRES_LOW] );
	res = pas230_write_byte ( client, ALS_THRES_LOW+1,	reg_defaults[ALS_THRES_LOW+1] );
	res = pas230_write_byte ( client, ALS_THRES_LOW+2,	reg_defaults[ALS_THRES_LOW+2] );
	if ( res != PAS230_SUCCESS ) {
		SENSOR_ERR ( "failed to write ( err=%d, reg=0x%02x )\n", res, ALS_THRES_LOW );
		return res;
	}

	res = pas230_write_byte(client, ALS_THRES_VAR, reg_defaults[ALS_THRES_VAR]);
	if ( res != PAS230_SUCCESS ) {
		SENSOR_ERR ( "failed to write ( err=%d, reg=0x%02x )\n", res, ALS_THRES_VAR );
		return res;
	}
#endif

	/* crosstalk value shall be set by LGP Server using I/O so init here to 150 */
	if(obj->ps_cross_talk > ps_cross_talk_max || obj->ps_cross_talk == 0 ) {
		obj->ps_cross_talk = ps_cross_talk_default + ps_cross_talk_offset;
	}

	if(obj->ps_cross_talk < 0) {
		obj->ps_cross_talk = 0;
	}

	obj->near_threshold = near_offset + obj->ps_cross_talk;
	obj->far_threshold = obj->near_threshold - far_offset;

	return res;

}

static long pas230_ps_enable ( struct i2c_client *client  )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	hwm_sensor_data sensor_data;
	int res = 0;
	unsigned char old_power_state;

    old_power_state = (unsigned char)obj->enable_ps_sensor;

	SENSOR_FUN();

	obj->ps_status = PS_FAR;
	pas230_set_pilt ( client, PS_TH_VAL_MAX );
	pas230_set_piht ( client, PS_TH_VAL_MIN );

	/* enable PAS230 */
	res = pas230_set_ps_enable ( client, SET_IC_ENABLE );
    if(res != PAS230_SUCCESS) {
	    goto enable_error;
    }

	/* inform to upper layer ( hwmsen ) */
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	sensor_data.value_divide = 1;
	sensor_data.values[0] = obj->ps_status;
	if ( ( res = hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data ) ) )
	{
		SENSOR_ERR ( "failed to send inform ( err = %d )\n", res );
	    goto enable_error;
	}
#ifdef PAS230_ALS_SENSOR_INT
	if( atomic_read(&irq_status) == -1 ) {
		/* unmask external interrupt */
		mt_eint_unmask ( CUST_EINT_PROXIMITY_NUM );
		atomic_set(&irq_status, +1);
	} else {
		atomic_set(&irq_status, +1);
	}
#else
	mt_eint_unmask ( CUST_EINT_PROXIMITY_NUM );
#endif

	SENSOR_LOG ( "PAS230 was enabled\n" );

    return (PAS230_SUCCESS);
    
enable_error :
    pas230_set_ps_enable(client, old_power_state);
	SENSOR_ERR ( "failed to enable PAS230\n" );

    return (res);
}

static long pas230_ps_disable ( struct i2c_client *client  )
{
	int res = 0;

	SENSOR_FUN();

	pas230_set_piht(client, PS_TH_VAL_MAX);
	pas230_set_pilt(client, PS_TH_VAL_MIN);
#ifdef PAS230_ALS_SENSOR_INT
	if(atomic_read(&irq_status) == 1) {
		atomic_set(&irq_status, -1);
	} else {
		mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );
		atomic_set(&irq_status, -1);
	}
#else
	mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );
#endif

	/* To reduce interrupt latency */
	if( atomic_read(&ps_int_flag) == 0 ) {
		pas230_reduce_ps_int_rate( client, 1 );
		atomic_set(&ps_int_flag, 1);
	}

	/* disable PAS230 */
	res = pas230_set_ps_enable ( client, SET_IC_DISABLE);
	if ( res == PAS230_SUCCESS )
	{
		SENSOR_LOG ( "PAS230 was disabled\n" );
	}
	else
	{
		SENSOR_ERR ( "failed to disable PAS230\n" );
	}

	return res;
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static long pas230_als_enable ( struct i2c_client *client  )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	hwm_sensor_data sensor_data;
	int res = 0;
	unsigned char old_power_state;

	old_power_state = (unsigned char)obj->enable_als_sensor;

	SENSOR_FUN();
#ifdef PAS230_ALS_SENSOR_INT
	if( atomic_read(&irq_status) == -1 ) {
		/* unmask external interrupt */
		mt_eint_unmask ( CUST_EINT_PROXIMITY_NUM );
		atomic_set(&irq_status, +1);
	} else {
		atomic_set(&irq_status, +1);
	}
#else
	mt_eint_unmask ( CUST_EINT_PROXIMITY_NUM );
#endif
	res = pas230_set_als_enable ( client, SET_IC_ENABLE );
    if(res != PAS230_SUCCESS) {
	    goto enable_error;
    }

    obj->als_data = pas230_get_alsdata( client );
    pas230_als_rsp(client);

	/* inform to upper layer ( hwmsen ) */
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	sensor_data.value_divide = 1;
	sensor_data.values[0] = obj->als_data;
	if ( ( res = hwmsen_get_interrupt_data ( ID_LIGHT, &sensor_data ) ) )
	{
		SENSOR_ERR ( "failed to send inform ( err = %d )\n", res );
	    goto enable_error;
	}

	SENSOR_LOG ( "PAS230 was enabled\n" );
	return (PAS230_SUCCESS);

enable_error :
    pas230_set_als_enable(client, old_power_state);
	SENSOR_ERR ( "failed to enable PAS230\n" );
    
    return (res);

}

static long pas230_als_disable ( struct i2c_client *client  )
{
	int res = 0;

	SENSOR_FUN();

	/* disable PAS230 */
	res = pas230_set_als_enable ( client, SET_IC_DISABLE );
	if ( res == PAS230_SUCCESS ) {
		SENSOR_LOG ( "PAS230 was disabled\n" );
	} else {
		SENSOR_ERR ( "failed to disable PAS230\n" );
	}

#ifdef PAS230_ALS_SENSOR_INT
	if(atomic_read(&irq_status) == 1) {
		atomic_set(&irq_status, -1);
	} else {
		mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );
		atomic_set(&irq_status, -1);
	}
#else
	mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );
#endif

	return res;
}

static int pas230_get_alsdata( struct i2c_client *client )
{
	int value = 0;
	u8 als_value[3] = {0, };

	pas230_read_byte( client, GREEN_DATA,   als_value   );
	pas230_read_byte( client, GREEN_DATA+1, als_value+1 );
	pas230_read_byte( client, GREEN_DATA+2, als_value+2 );

	value = ((als_value[2]<<16) | (als_value[1]<<8) | als_value[0]);

#if 1
	if ( ALS_LUX_COEFF < 10 ) {
		SENSOR_DBG("als value=%d", value);
	} else {
		value = (value * ALS_LUX_COEFF) / 10;
		SENSOR_DBG("als value=%d", value);
	}
#else
/* For pocket Detection scene */
	value -= 4;
	if (value <0) {
		value = 0;
	} else {
		value /= 3;
	}
#endif

	return value;
}

#endif

void pas230_swap(int *x, int *y)
{
     int temp = *x;
     *x = *y;
     *y = temp;
}

static int pas230_do_calibration ( struct i2c_client *client, int *value )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned int calib_data;
	unsigned int old_enable = 0;
	int result,i;

	old_enable  = obj->enable_ps_sensor;
	calib_data = 0;

	for (i=0; i < CALB_TIMES; i++) {
		calib_data = pas230_calc_calibration(client);
		if( calib_data <= ps_cross_talk_max ) {
			SENSOR_LOG("ps_cross_talk save : %d\n", calib_data);
			obj->ps_cross_talk = calib_data;
			break;
		}
	}

	result = pas230_set_ps_enable ( client, old_enable );
	if(result != PAS230_SUCCESS) {
		return (result);
	}

	*value = obj->ps_cross_talk;

	if(i >= CALB_TIMES){
		SENSOR_ERR ( "failed to calibrate cross talk/n" );
		return -1;
	} else {
		obj->near_threshold = near_offset + obj->ps_cross_talk;
		obj->far_threshold = obj->near_threshold - far_offset;
	}

	/* we should store it to storage ( it should be free from factory reset ) but ATCI Demon will store it through LGP Demon */
	return 0;
}

static unsigned int pas230_calc_calibration ( struct i2c_client *client )
{
    unsigned int value;
	int temp_pdata[CALB_BOX_TIMES] = {0,};
#if DEBUG
	int temp_state[CALB_BOX_TIMES] = {0,};
#endif
	unsigned int i                 = 0;
	unsigned int j                 = 0;
	unsigned int sum_of_pdata      = 0;
	int result ;


	/* Enable PS and Mask interrupt */
	result = pas230_set_ps_enable ( client, SET_IC_DISABLE );
	if(result != PAS230_SUCCESS) {
        return (result);
    }

	result = pas230_set_ps_enable ( client, SET_IC_ENABLE );
    if(result != PAS230_SUCCESS) {
        return (result);
    }

	mdelay ( CALB_SYSTEM_WAIT );

	/* Read pdata */
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		pas230_get_pdata ( client, (unsigned short *)&( temp_pdata[i] ) );
		mdelay ( CALB_IC_WAIT );
	}

#if DEBUG
	SENSOR_LOG ( "State Value = " );
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		SENSOR_LOG ( "%d ", temp_state[i] );
	}
	SENSOR_LOG ( "\n" );
	SENSOR_LOG ( "Read Value = " );
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		SENSOR_LOG ( "%d ", temp_pdata[i] );
	}
	SENSOR_LOG ( "\n" );
#endif

	/* sort pdata */
	for ( i = 0 ; i < CALB_BOX_TIMES - 1 ; i++ )
	{
		for ( j = i + 1 ; j < CALB_BOX_TIMES ; j++ )
		{
			if ( temp_pdata[i] > temp_pdata[j] )
			{
				pas230_swap ( temp_pdata+i, temp_pdata+j );
			}
		}
	}

#if DEBUG
	SENSOR_LOG (  );
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		SENSOR_LOG ( "count = %d Read Value = %d\n", i, temp_pdata[i] );
	}
#endif

	/* take ten middle data only */
	for ( i = CALB_REMOVAL_TIME ; i < (CALB_BOX_TIMES - CALB_REMOVAL_TIME) ; i++ )
	{
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	/* calculate average */
    value = sum_of_pdata / (CALB_BOX_TIMES - (CALB_REMOVAL_TIME * 2));
	SENSOR_LOG ( "New calibrated cross talk = %d\n", value );

    return value;
}

//==========================================================
// PAS230 General Control Funtions
//==========================================================
static long pas230_ps_activate ( struct i2c_client *client, int enable )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	long res = 0;
	SENSOR_FUN();
	if ( obj->enable_ps_sensor != enable )
	{
		if ( enable )
		{
			res = pas230_ps_enable ( client );
			if ( res == PAS230_SUCCESS )
			{
				SENSOR_LOG ( "PAS230 was enabled\n" );
			}
			else
			{
				SENSOR_ERR ( "failed to enable PAS230\n" );
			}
		}
		else
		{
			res = pas230_ps_disable ( client );
			if ( res == PAS230_SUCCESS )
			{
				SENSOR_LOG ( "PAS230 was disabled\n" );
			}
			else
			{
				SENSOR_ERR ( "failed to disable PAS230\n" );
			}
		}

		if ( res == PAS230_SUCCESS )
		{
			obj->enable_ps_sensor = enable;
		}
		
	}

	return res;
	
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static long pas230_als_activate ( struct i2c_client *client, int enable )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	long res = 0;

	SENSOR_FUN();

	if ( obj->enable_als_sensor != enable )
	{
		if ( enable )
		{
			res = pas230_als_enable ( client );
			if ( res == PAS230_SUCCESS )
			{
				SENSOR_LOG ( "PAS230 was enabled\n" );
			}
			else
			{
				SENSOR_ERR ( "failed to enable PAS230\n" );
			}
		}
		else
		{
			res = pas230_als_disable ( client );
			if ( res == PAS230_SUCCESS )
			{
				SENSOR_LOG ( "PAS230 was disabled\n" );
			}
			else
			{
				SENSOR_ERR ( "failed to disable PAS230\n" );
			}
		}

		if ( res == PAS230_SUCCESS )
		{
			obj->enable_als_sensor = enable;
		}
	}

	return res;

}
#endif

//==========================================================
// PAS230 Interrupt Service Routines
//==========================================================
void pas230_eint_func ( void )
{
	struct pas230_priv *obj = g_pas230_ptr;

	if ( !obj )
		return;

	schedule_work ( &obj->eint_work );
}

static void pas230_eint_work ( struct work_struct *work )
{
	struct pas230_priv *obj = ( struct pas230_priv * ) container_of ( work, struct pas230_priv, eint_work );
	struct i2c_client *client = obj->client;
	hwm_sensor_data sensor_data;

	int err;
	int new_ps_status = 0;
	unsigned char int_status = 0;
	unsigned short pdata = 0;

	/* read main status register */
	err = pas230_get_status ( client, &int_status );
	if ( err != PAS230_SUCCESS) {
		SENSOR_ERR("ADC value is not valid so just skip this interrupt");
		goto CLEAR_INTERRUPT;
	}

	if(int_status & PAS230_PINT) {
		if(obj->enable_ps_sensor == (SET_IC_ENABLE)) {
			SENSOR_LOG ( "PS interrupt happened\n" );

			if( atomic_read(&ps_int_flag) == 1 ) {
				pas230_reduce_ps_int_rate( client, 0 );
				atomic_set(&ps_int_flag, 0);
			}

			err = pas230_get_pdata ( client, &pdata );
			if ( err != PAS230_SUCCESS) {
				SENSOR_ERR("Can't access register of pdata.");
				goto CLEAR_INTERRUPT;
			}

			new_ps_status = pas230_proximity_rsp(client, pdata);
			if(new_ps_status) {
				SENSOR_LOG ( "PS=FAR, ps_logic=(%d)\n", (int_status & 0x04));
			} else {
				SENSOR_LOG ( "PS=NEAR, ps_logic=(%d)\n", (int_status & 0x04));
			}

			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			sensor_data.value_divide = 1;
			sensor_data.values[0] = new_ps_status;
			err = hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data );
			if ( err ) {
			    SENSOR_ERR ( "failed to send inform ( err = %d )\n", err );
			}
		}
	}

#ifdef PAS230_ALS_SENSOR_INT
	if(obj->enable_als_sensor == (SET_IC_ENABLE)) {
		if( int_status & ALS_INT_MASK ) {
			obj->als_data = pas230_get_alsdata( client );
			SENSOR_DBG("ALS Interrupt happened ALS_DATA = %d\n",obj->als_data);
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			sensor_data.value_divide = 1;
			sensor_data.values[0] =	obj->als_data;
			if (( err = hwmsen_get_interrupt_data ( ID_LIGHT, &sensor_data ))){
				SENSOR_ERR ( "failed to send inform ( err = %d )\n", err );
			}
			/*Reset ALS threshold*/
			pas230_als_rsp(client);
		}
	}
#endif

CLEAR_INTERRUPT:	
	mt_eint_unmask ( CUST_EINT_PROXIMITY_NUM );
}

//==========================================================
// PAS230 ADB Shell command function
//==========================================================
static ssize_t pas230_show_cali_value ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%u\n", data->ps_cross_talk );
}

static ssize_t pas230_store_cali_value ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	int ret;
	int data;

	ret = pas230_do_calibration ( client, &data );

	return count;
}

static ssize_t pas230_show_ps_led ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );
	unsigned char ps_led = 0;
	int err = 0;

	err = pas230_get_ps_led(client, &ps_led);
	if( err != PAS230_SUCCESS){
		SENSOR_ERR("failed to read ps_led\n");
	}

	SENSOR_LOG("ps_led=0x%02x\n", ps_led);

	return sprintf ( buf, "0x%02x\n", data->ps_led  );
}

static ssize_t pas230_store_ps_led ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_ps_led ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->ps_led  = (unsigned int)val;

	return count;
}

static ssize_t pas230_show_ps_pulse ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );
	unsigned char ps_pulse = 0;
	int err = 0;

	err = pas230_get_ps_pulse(client, &ps_pulse);
	if( err != PAS230_SUCCESS){
		SENSOR_ERR("failed to read ps_pulse\n");
	}

	SENSOR_LOG("ps_pulse=0x%02x\n", ps_pulse);

	return sprintf ( buf, "%d\n", data->ps_pulse  );
}

static ssize_t pas230_store_ps_pulse ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_ps_pulse ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->ps_pulse  = (unsigned int)val;

	return count;
}

static ssize_t pas230_show_ps_meas_rate ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );
	unsigned char ps_meas_rate = 0;
	int err = 0;

	err = pas230_get_ps_meas_rate(client, &ps_meas_rate);
	if( err != PAS230_SUCCESS){
		SENSOR_ERR("failed to read ps_meas_rate\n");
	}

	SENSOR_LOG("ps_meas_rate=0x%02x\n", ps_meas_rate);


	return sprintf ( buf, "0x%02x\n", data->ps_meas_rate );
}

static ssize_t pas230_store_ps_meas_rate ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_ps_meas_rate ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->ps_meas_rate  = (unsigned int)val;

	return count;
}

static ssize_t pas230_show_als_meas_rate ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );
	unsigned char als_meas_rate = 0;
	int err = 0;

	err = pas230_get_als_meas_rate(client, &als_meas_rate);
	if( err != PAS230_SUCCESS){
		SENSOR_ERR("failed to read als_meas_rate\n");
	}

	SENSOR_LOG("als_meas_rate=0x%02x\n", als_meas_rate);


	return sprintf ( buf, "0x%02x\n", data->als_meas_rate );
}

static ssize_t pas230_store_als_meas_rate ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_als_meas_rate ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

	obj->als_meas_rate  = (unsigned int)val;

	return count;
}

static ssize_t pas230_show_als_cs_gain ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );
	unsigned char als_cs_gain = 0;
	int err = 0;

	err = pas230_get_als_cs_gain(client, &als_cs_gain);
	if( err != PAS230_SUCCESS){
		SENSOR_ERR("failed to read als_cs_gain\n");
	}

	SENSOR_LOG("als_cs_gain=0x%02x\n", als_cs_gain);


	return sprintf ( buf, "0x%02x\n", data->als_cs_gain );
}

static ssize_t pas230_store_als_cs_gain ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_als_cs_gain ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

	obj->als_cs_gain  = (unsigned int)val;

	return count;
}

static ssize_t pas230_show_pilt ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );
	int value = 0;

	value = pas230_get_pilt(client);
	SENSOR_DBG("show pilt = %d\n", data->pilt);

	return sprintf ( buf, "%d\n", value );
}

static ssize_t pas230_store_pilt ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_pilt ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->far_threshold = (unsigned int)val;

	return count;
}

static ssize_t pas230_show_piht ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );
	int value = 0;

	value = pas230_get_piht(client);
	SENSOR_DBG("show piht %d\n", data->piht);

	return sprintf ( buf, "%d\n", value );
}

static ssize_t pas230_store_piht ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_piht ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->near_threshold = (unsigned int)val;

	return count;
}


static ssize_t pas230_show_status ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = pas230_i2c_client;
    unsigned char status = 0;

    pas230_get_status ( client, &status );

    return sprintf ( buf, "0x%02x\n", status );
}

static ssize_t pas230_show_ctrl ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = pas230_i2c_client;
    unsigned char status = 0;

    pas230_get_ctrl ( client, &status );

    return sprintf ( buf, "0x%02x\n", status );
}

static ssize_t pas230_show_pdata ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = pas230_i2c_client;
    unsigned short data = 0;

    pas230_get_pdata ( client, &data );

    return sprintf ( buf, "%d\n", data);
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static ssize_t pas230_show_alsdata ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	int data = 0;

    data = pas230_get_alsdata ( client );

    return sprintf ( buf, "%d\n", data );
}
#endif

static ssize_t pas230_show_deviceid ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = pas230_i2c_client;
    unsigned char data = 0;

    pas230_get_deivceid ( client, &data );

    return sprintf ( buf, "%02x\n", data );
}

static ssize_t pas230_show_near_offset ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );

	SENSOR_LOG("near_threshold = %d\n", obj->near_threshold);
	SENSOR_LOG("far_threshold = %d\n", obj->far_threshold);

	return sprintf ( buf, "%d\n", near_offset);
}

static ssize_t pas230_store_near_offset ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	near_offset = val;
	obj->near_threshold = near_offset + obj->ps_cross_talk;
	obj->far_threshold = obj->near_threshold - far_offset;

	ret = pas230_set_piht ( client, (unsigned short)obj->near_threshold );
	ret = pas230_set_pilt ( client, (unsigned short)obj->far_threshold );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t pas230_show_far_offset ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );

	SENSOR_LOG("near_threshold = %d\n", obj->near_threshold);
	SENSOR_LOG("far_threshold = %d\n", obj->far_threshold);

	return sprintf ( buf, "%d\n", far_offset);
}

static ssize_t pas230_store_target_far_offset ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	far_offset = val;
	obj->near_threshold = near_offset + obj->ps_cross_talk;
	obj->far_threshold = obj->near_threshold - far_offset;

	ret = pas230_set_piht ( client, (unsigned short)obj->near_threshold );
	ret = pas230_set_pilt ( client, (unsigned short)obj->far_threshold );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t pas230_show_ps_enable ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );

    switch(data->enable_ps_sensor) {//need for fix (enable_ps_sensor and enable_als_sensor)
          case 0:
         	   return sprintf ( buf, "%s\n", "Proximity Disabled");
          case 1:
               return sprintf ( buf, "%s\n", "Proximity Enabled" );

           default:
               return sprintf ( buf, "%s\n", "Proximity Error" );
     	}
    
}

static ssize_t pas230_store_ps_enable ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret=0;

    switch(val) {
          case 0:
            ret = pas230_ps_activate ( client, 0 );
            break;
          case 1:
            ret = pas230_ps_activate ( client, 1 );
            break;

           default:
           	break;
     	}

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t pas230_show_int_cfg(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );
	unsigned char int_cfg = 0;
	int err = 0;

	err = pas230_get_ps_int_cfg(client, &int_cfg);
	if( err != PAS230_SUCCESS){
		SENSOR_ERR("failed to read int_cfg");
	}

	SENSOR_LOG("int_cfg=0x%02x\n", int_cfg);


	return sprintf ( buf, "0x%02x\n", data->ps_int_cfg );

}

static ssize_t pas230_store_int_cfg(struct device_driver *dev, char *buf, size_t count)
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_ps_int_cfg ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->ps_int_cfg  = (unsigned int)val;

	return count;
}

static ssize_t pas230_show_ps_can(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );
	unsigned char ps_can[2] = {0,};
	int err = 0;
	int val = 0;

	pas230_get_ps_can_0(client, ps_can);
	pas230_get_ps_can_1(client, ps_can+1);

	val = ps_can[0] | (ps_can[1] << 8);
	SENSOR_LOG("ps_can = 0x%02x\n", val);

	return sprintf ( buf, "0x%02x\n", val);
}

static ssize_t pas230_store_ps_can(struct device_driver *dev, char *buf, size_t count)
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	unsigned char ps_can_0 = 0;
	unsigned char ps_can_1 = 0;
	int ret = 0;

	ps_can_0 = val & 0xFF;
	ps_can_1 = (val >> 8) & 0x07;

	ret = pas230_set_ps_can_0( client, ps_can_0 );
	if ( ret < 0 )
		return ret;

	ret = pas230_set_ps_can_0( client, ps_can_1 );
	if ( ret < 0 )
		return ret;

    obj->ps_can_0 = ps_can_0;
    obj->ps_can_1 = ps_can_1;

	return count;
}

static DRIVER_ATTR ( cali, S_IWUSR | S_IRUGO, pas230_show_cali_value, pas230_store_cali_value );
static DRIVER_ATTR ( ps_led, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_ps_led, pas230_store_ps_led );
static DRIVER_ATTR ( ps_pulse, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_ps_pulse, pas230_store_ps_pulse );
static DRIVER_ATTR ( ps_meas_rate, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_ps_meas_rate, pas230_store_ps_meas_rate );
static DRIVER_ATTR ( pilt, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_pilt, pas230_store_pilt );
static DRIVER_ATTR ( piht, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_piht, pas230_store_piht );
static DRIVER_ATTR ( status, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_status, NULL );
static DRIVER_ATTR ( ctrl, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_ctrl, NULL );
static DRIVER_ATTR ( pdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_pdata, NULL );
static DRIVER_ATTR ( deviceid, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_deviceid, NULL );  
static DRIVER_ATTR ( near_offset, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_near_offset, pas230_store_near_offset );
static DRIVER_ATTR ( far_offset, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_far_offset, pas230_store_target_far_offset );
static DRIVER_ATTR ( enable, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_ps_enable, pas230_store_ps_enable );
static DRIVER_ATTR ( int_cfg, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_int_cfg, pas230_store_int_cfg );
static DRIVER_ATTR ( ps_can, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_ps_can, pas230_store_ps_can );
#ifdef PAS230_ALS_SENSOR_ENABLE
static DRIVER_ATTR ( alsdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_alsdata, NULL );
static DRIVER_ATTR ( als_meas_rate, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_als_meas_rate, pas230_store_ps_meas_rate );
static DRIVER_ATTR ( als_cs_gain, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_als_cs_gain, pas230_store_als_cs_gain );
#endif

static struct driver_attribute *pas230_attr_list[] = {
	&driver_attr_cali,		   
	&driver_attr_ps_led,
	&driver_attr_ps_pulse,
	&driver_attr_ps_meas_rate,
	&driver_attr_pilt,
	&driver_attr_piht,
	&driver_attr_status,
	&driver_attr_ctrl,
	&driver_attr_pdata,
	&driver_attr_deviceid,
	&driver_attr_near_offset,
	&driver_attr_far_offset,
	&driver_attr_enable,
	&driver_attr_ps_can,
#ifdef PAS230_ALS_SENSOR_ENABLE
	&driver_attr_alsdata,
	&driver_attr_als_meas_rate,
	&driver_attr_als_cs_gain,
#endif
};

static int pas230_create_attr ( struct device_driver *driver )
{
	int idx;
	int err = 0;
	int num = ( int ) ( sizeof ( pas230_attr_list ) / sizeof ( pas230_attr_list[0] ) );

	if ( driver == NULL )
	{
		return -EINVAL;
	}

	for ( idx = 0 ; idx < num ; idx++ )
	{
		err = driver_create_file ( driver, pas230_attr_list[idx] );
		if ( err )
		{
			SENSOR_ERR ( "driver_create_file (%s) = %d\n", pas230_attr_list[idx]->attr.name, err );
			break;
		}
	}

	return err;
}

static int pas230_delete_attr ( struct device_driver *driver )
{
	int idx;
	int err = 0;
	int num = ( int ) ( sizeof ( pas230_attr_list ) / sizeof ( pas230_attr_list[0] ) );

	if ( driver == NULL )
	{
		return -EINVAL;
	}

	for ( idx = 0 ; idx < num ; idx++ )
	{
		driver_remove_file ( driver, pas230_attr_list[idx] );
	}

	return err;
}

//==========================================================
// PAS230 Service APIs ( based on File I/O )
//==========================================================
static int pas230_open ( struct inode *inode, struct file *file )
{
	SENSOR_FUN ();
	file->private_data = pas230_i2c_client;

	if ( !file->private_data )
	{
		SENSOR_ERR ( "Invalid input paramerter\n" );
		return -EINVAL;
	}

	return nonseekable_open ( inode, file );
}

static int pas230_release ( struct inode *inode, struct file *file )
{
	SENSOR_FUN ();
	file->private_data = NULL;
	return 0;
}

static long pas230_unlocked_ioctl ( struct file *file, unsigned int cmd, unsigned long arg )
{
	struct i2c_client *client = ( struct i2c_client * ) file->private_data;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	long err = 0;
	void __user *ptr = ( void __user * ) arg;
	int dat;
	uint32_t enable;
	uint32_t crosstalk = 0;

	SENSOR_FUN();

	switch ( cmd )
	{
		case ALSPS_SET_PS_MODE:
			SENSOR_LOG ( "CMD = ALSPS_SET_PS_MODE\n" );
			if ( copy_from_user ( &enable, ptr, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			if ( enable )
			{
				if ( ( err = pas230_ps_activate ( obj->client, 1 ) ) )
				{
					SENSOR_ERR ( "failed to activate PAS230 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			else
			{
				if ( ( err = pas230_ps_activate ( obj->client, 0 ) ) )
				{
					SENSOR_ERR ( "failed to deactivate PAS230 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			break;

		case ALSPS_GET_PS_MODE:
			SENSOR_LOG ( "CMD = ALSPS_GET_PS_MODE\n" );
			enable = obj->enable_ps_sensor;
			if ( copy_to_user ( ptr, &enable, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			SENSOR_LOG ( "CMD = ALSPS_GET_PS_DATA\n" );
			dat = obj->ps_status;
			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_RAW_DATA:
			SENSOR_LOG ( "CMD = ALSPS_GET_PS_RAW_DATA\n" );
			err = pas230_get_pdata ( obj->client, (unsigned short *)&dat );
			if ( err )
			{
				goto err_out;
			}

			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_CALI:
			SENSOR_LOG ( "CMD = ALSPS_GET_CALI\n" );
			err = pas230_do_calibration ( obj->client, &dat );
			if ( err == 0 )
			{
				if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
				{
					err = -EFAULT;
					goto err_out;
				}
			}
			break;


		case ALSPS_SET_CALI:
			SENSOR_LOG ( "CMD = ALSPS_SET_CALI\n" );
			if ( copy_from_user ( &crosstalk, ptr, sizeof ( crosstalk ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}

			if ( ( crosstalk == 0x0000FFFF ) || ( crosstalk == 0 ) ) {
				obj->ps_cross_talk = ps_cross_talk_default;
			} else {
				obj->ps_cross_talk = crosstalk;
			}

			if(obj->ps_cross_talk > 0) {
				obj->ps_cross_talk += ps_cross_talk_offset;
			}
			SENSOR_LOG ( "ps_cross_talk = %d\n", obj->ps_cross_talk );
			obj->near_threshold = near_offset + obj->ps_cross_talk;
			obj->far_threshold = obj->near_threshold - far_offset;
			break;

		case ALSPS_GET_DEVICEID:
			SENSOR_LOG ( "CMD = ALSPS_GET_DEVICEID\n" );
			err = pas230_get_deivceid ( obj->client, (unsigned char*) &dat );
			if ( err )
			{
				goto err_out;
			}

			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

#ifdef PAS230_ALS_SENSOR_ENABLE
		case ALSPS_SET_ALS_MODE:
			SENSOR_LOG( "CMD = ALSPS_SET_ALS_MODE\n" );
			if ( copy_from_user ( &enable, ptr, sizeof ( enable ) ) ) {
				err = -EFAULT;
				goto err_out;
			}

			if ( enable ) {
				if ( ( err = pas230_als_activate ( obj->client, 1 ) ) ) {
					SENSOR_ERR ( "failed to activate RPR0521 ( err = %d )\n", (int)err );
					goto err_out;
				}
			} else {
				if ( ( err = pas230_als_activate ( obj->client, 0 ) ) ) {
					SENSOR_ERR ( "failed to deactivate RPR0521 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			break;

		case ALSPS_GET_ALS_MODE:
			SENSOR_LOG ( "CMD = ALSPS_GET_ALS_MODE\n" );
			enable = obj->enable_als_sensor;
			if ( copy_to_user ( ptr, &enable, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA:
			SENSOR_LOG ( "CMD = ALSPS_GET_ALS_DATA\n" );
			dat = obj->als_data;
			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) ) {
				err = -EFAULT;
				goto err_out;
			} else {
				SENSOR_LOG ( "CMD = ALSPS_GET_ALS_DATA : Overlaped. Skip. \n" );
			}
			break;
#endif
		default:
			SENSOR_ERR ( "Invalid Command = 0x%04x\n", cmd );
			err = -ENOIOCTLCMD;
			break;
	}

	err_out : return err;
}

static struct file_operations pas230_fops = {
	.owner = THIS_MODULE,
	.open = pas230_open,
	.release = pas230_release,
	.unlocked_ioctl = pas230_unlocked_ioctl,
};

static struct miscdevice pas230_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &pas230_fops,
};

//==========================================================
// PAS230 Service APIs ( based on hwmsen Interface )
//==========================================================
static int pas230_ps_operate ( void *self, uint32_t command, void *buff_in, int size_in, void *buff_out, int size_out, int *actualout )
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data = NULL;
	struct pas230_priv *obj = ( struct pas230_priv * ) self;

	SENSOR_FUN();

	switch ( command )
	{
		case SENSOR_DELAY:
			SENSOR_LOG ( "CMD = SENSOR_DELAY\n" );
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			break;

		case SENSOR_ENABLE:
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				value = *( int * ) buff_in;
				if ( value )
				{
					SENSOR_LOG ( "CMD = SENSOR_ENABLE ( Enable )\n" );
					err = pas230_ps_activate ( obj->client, 1 );
					if ( err )
					{
						SENSOR_ERR ( "failed to activate PAS230 ( err = %d )\n", err );
						return -1;
					}
				}
				else
				{
					SENSOR_LOG ( "CMD = SENSOR_ENABLE ( Disable )\n" );
					err = pas230_ps_activate ( obj->client, 0 );
					if ( err )
					{
						SENSOR_ERR ( "failed to deactivate PAS230 ( err = %d )\n", err );
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			SENSOR_LOG ( "CMD = SENSOR_GET_DATA\n" );
			if ( ( buff_out == NULL ) || ( size_out < sizeof ( hwm_sensor_data ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				sensor_data->values[0] = obj->ps_status;
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;

		default:
			SENSOR_ERR ( "Invalid Command = %d\n", command );
			err = -1;
			break;
	}

	return err;
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static int pas230_als_operate ( void *self, uint32_t command, void *buff_in, int size_in, void *buff_out, int size_out, int *actualout )
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data = NULL;
	struct pas230_priv *obj = ( struct pas230_priv * ) self;

	SENSOR_FUN();

	switch ( command )
	{
		case SENSOR_DELAY:
			SENSOR_LOG ( "CMD = SENSOR_DELAY\n" );
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				value = *( int * ) buff_in;
				if ( value )
				{
					SENSOR_LOG ( "CMD = SENSOR_ENABLE ( Enable )\n" );
					err = pas230_als_activate ( obj->client, 1 );
					if ( err )
					{
						SENSOR_ERR ( "failed to activate PAS230 ( err = %d )\n", err );
						return -1;
					}
				}
				else
				{
					SENSOR_LOG ( "CMD = SENSOR_ENABLE ( Disable )\n" );
					err = pas230_als_activate ( obj->client, 0 );
					if ( err )
					{
						SENSOR_ERR ( "failed to deactivate PAS230 ( err = %d )\n", err );
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			SENSOR_LOG ( "CMD = SENSOR_GET_DATA\n" );
			if ( ( buff_out == NULL ) || ( size_out < sizeof ( hwm_sensor_data ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				sensor_data->values[0] = pas230_get_alsdata(obj->client);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;

		default:
			SENSOR_ERR ( "Invalid Command = %d\n", command );
			err = -1;
			break;
	}

	return err;
}

#if defined CONFIG_FB
static int pas230_als_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = (struct fb_event *)data;
	int *blank = NULL;
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );

	if (evdata && evdata->data)
		blank = (int *)evdata->data;
	else
		return PAS230_SUCCESS;

	if (event == FB_EVENT_BLANK) {
		if (*blank == FB_BLANK_POWERDOWN) {
			atomic_set(&driver_suspend_flag, 1);
			SENSOR_DBG("[IN] LCD Sleep\n");
		} else if (*blank == FB_BLANK_UNBLANK) {
			atomic_set(&driver_suspend_flag, 0);
			SENSOR_DBG("[OUT] LCD Sleep\n");
		}
	}
	return PAS230_SUCCESS;
}
#endif
#endif

//==========================================================
// PAS230 Initialization related Routines
//==========================================================
static int pas230_init_client ( struct i2c_client *client )
{
	int err = 0;

	SENSOR_FUN();

	err = pas230_initialize ( client );
	if ( err != PAS230_SUCCESS )
	{
		SENSOR_ERR ( "failed to init PAS230\n" );
	}

	return err;
}

/****************************************************************************
* I2C BUS Related Functions
****************************************************************************/

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void pas230_early_suspend ( struct early_suspend *h )
{
	SENSOR_FUN ();
}

static void pas230_late_resume ( struct early_suspend *h )
{
	SENSOR_FUN ();
}
#endif

static int pas230_i2c_probe ( struct i2c_client *client, const struct i2c_device_id *id )
{
	struct pas230_priv *obj;
	struct hwmsen_object obj_ps;
#ifdef PAS230_ALS_SENSOR_ENABLE
	struct hwmsen_object obj_als;
#endif
	struct alsps_hw *hw = get_cust_alsps_hw ();
	int err = 0;

	SENSOR_FUN();

	obj = devm_kzalloc(&client->dev, sizeof(struct pas230_priv), GFP_KERNEL);

	if ( !obj )
	{
		SENSOR_ERR("failed to allock memory for module data");
		err = -ENOMEM;
		goto exit;
	}
	memset ( obj, 0, sizeof ( *obj ) );

	obj->client = client;
	i2c_set_clientdata ( client, obj );

	g_pas230_ptr = obj;
	pas230_i2c_client = client;

	INIT_WORK ( &obj->eint_work, pas230_eint_work );

	#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend = pas230_early_suspend,
	obj->early_drv.resume = pas230_late_resume,
	register_early_suspend ( &obj->early_drv );
	#endif

	/* Initialize PAS230 */ 
	err = pas230_init_client ( client );
	if ( err )
	{
		SENSOR_ERR ( "failed to init PAS230 ( err = %d )\n", err );
		goto exit_init_failed;
	}

	/* Register PAS230 as a misc device for general I/O interface */
	err = misc_register ( &pas230_device );
	if ( err )
	{
		SENSOR_ERR ( "failed to register misc device ( err = %d )\n", err );
		goto exit_misc_device_register_failed;
	}

	err = pas230_create_attr ( &pas230_alsps_driver.driver );
	if ( err )
	{
		SENSOR_ERR ( "create attribute err = %d\n", err );
		goto exit_create_attr_failed;
	}

#if defined CONFIG_FB
	obj->fb_notif.notifier_call = pas230_als_fb_notifier_callback;
	fb_register_client(&obj->fb_notif);
#endif

	/* Register PAS230 as a member device of hwmsen */
	obj_ps.self = obj;
	obj_ps.polling = hw->polling_mode_ps;
	obj_ps.sensor_operate = pas230_ps_operate;
	err = hwmsen_attach ( ID_PROXIMITY, &obj_ps );
	if ( err )
	{
		SENSOR_ERR ( "failed to attach to hwmsen ( err = %d )\n", err );
		goto exit_hwsen_attach_failed;
	}
#ifdef PAS230_ALS_SENSOR_ENABLE
	/* Register PAS230 as a member device of hwmsen */
	obj_als.self = obj;
	obj_als.polling = hw->polling_mode_als;
	obj_als.sensor_operate = pas230_als_operate;
	err = hwmsen_attach ( ID_LIGHT, &obj_als );
	if ( err )
	{
		SENSOR_ERR ( "failed to attach to hwmsen ( err = %d )\n", err );
		goto exit_hwsen_attach_failed;
	}
#endif

	SENSOR_LOG("done\n");

	return 0;

exit_hwsen_attach_failed:
    pas230_delete_attr(&pas230_alsps_driver.driver);
exit_create_attr_failed:
	misc_deregister ( &pas230_device );
exit_misc_device_register_failed:
exit_init_failed:
	unregister_early_suspend ( &obj->early_drv );
exit:
	pas230_i2c_client = NULL;
	SENSOR_ERR ( "Err = %d\n", err );

	return err;
}

static int pas230_i2c_remove ( struct i2c_client *client )
{
	int err;
	struct pas230_priv *obj = i2c_get_clientdata ( client );

	SENSOR_FUN();

	mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );
	if( obj->enable_ps_sensor == SET_IC_ENABLE || obj->enable_als_sensor == SET_IC_ENABLE ) {
		err = pas230_write_byte(client, MAIN_CTRL, ALL_OFF);
		if ( err == PAS230_SUCCESS )
		{
			obj->enable_ps_sensor = SET_IC_DISABLE;
			obj->enable_als_sensor = SET_IC_DISABLE;
			SENSOR_LOG ( "PAS230 was disabled\n" );
		}
	}

	err = pas230_delete_attr ( &pas230_alsps_driver.driver );
	if ( err )
	{
		SENSOR_ERR ( "pas230_delete_attr fail: %d\n", err );
	}

	err = misc_deregister ( &pas230_device );
	if ( err )
	{
		SENSOR_ERR ( "failed to deregister misc driver : %d\n", err );
	}

	pas230_i2c_client = NULL;
	g_pas230_ptr = NULL;
	i2c_unregister_device ( client );

	return 0;
}

static int pas230_i2c_suspend ( struct i2c_client *client, pm_message_t msg )
{	
	SENSOR_FUN();
	return 0;
}

static int pas230_i2c_resume ( struct i2c_client *client )
{
	SENSOR_FUN();
	return 0;
}

static int pas230_i2c_detect ( struct i2c_client *client, struct i2c_board_info *info )
{
	SENSOR_FUN ();
	strcpy ( info->type, PAS230_DEV_NAME );
	return 0;
}

static const struct i2c_device_id pas230_i2c_id[] = { { PAS230_DEV_NAME, 0 }, {} };

static struct i2c_driver pas230_i2c_driver = {
	.probe = pas230_i2c_probe,
	.remove = pas230_i2c_remove,
	.suspend = pas230_i2c_suspend,
	.resume = pas230_i2c_resume,
	.detect = pas230_i2c_detect,
	.id_table = pas230_i2c_id,
	.driver = {
		.name = PAS230_DEV_NAME,
	},
};


/****************************************************************************
* Linux Device Driver Related Functions
****************************************************************************/
static int pas230_probe ( struct platform_device *pdev )
{
	SENSOR_FUN();

	/* Configure external ( GPIO ) interrupt */
	pas230_setup_eint ();

	/* Add PAS230 as I2C driver */
	if ( i2c_add_driver ( &pas230_i2c_driver ) )
	{
		SENSOR_ERR ( "failed to add i2c driver\n" );
		return -1;
	}

	return 0;
}

static int pas230_remove ( struct platform_device *pdev )
{
	SENSOR_FUN ();

	i2c_del_driver ( &pas230_i2c_driver );

	return 0;
}

static struct i2c_board_info __initdata i2c_PAS230 = { I2C_BOARD_INFO ( "PAS230", PAS230_I2CADDR ) };

static struct platform_driver pas230_alsps_driver = {
	.probe = pas230_probe,
	.remove = pas230_remove,
	.driver = {
		.name = "als_ps",
#ifdef CONFIG_OF_DT
		.of_match_table = psensor_of_match,
#endif

	},
};

static int __init pas230_init ( void )
{
	struct alsps_hw *hw = get_cust_alsps_hw ();
	i2c_register_board_info ( hw->i2c_num, &i2c_PAS230, 1 );

	SENSOR_FUN();

	if ( platform_driver_register ( &pas230_alsps_driver ) )
	{
		SENSOR_ERR ( "failed to register platform driver\n" );
		return -ENODEV;
	}
	return 0;
}

static void __exit pas230_exit ( void )
{
	SENSOR_FUN ();
	platform_driver_unregister ( &pas230_alsps_driver );
}


module_init ( pas230_init );
module_exit ( pas230_exit );

MODULE_AUTHOR ( "Seo Ji Won" );
MODULE_DESCRIPTION ( "pas230 driver" );
MODULE_LICENSE ( "GPL" );

/* End Of File */
