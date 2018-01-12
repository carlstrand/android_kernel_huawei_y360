

#ifndef __CUST_STK3X1X_H__
#define __CUST_STK3X1X_H__


/*******************************************/

/* define setting value */
#define ALS_AVR_COUNT 		  5		// als do average N times
#define CUST_ALS_SKIP_COUNT   5		// als skip N count after enable 

/*******************************************/

//#define WT_ALS_SWITCH_AUTO	// enable als auto switch 
//#define WT_ALS_ENABLE_SKIP  	// enable als skip first N count

/*******************************************/

/* For ps calibration */
#define CUST_STK3X1X_PS_THRES_CLOSE		91	 // Close 
#define CUST_STK3X1X_PS_THRES_FAR		42	 // Far away 

/****************configuration********************/
/*STK3X1X ps cust setting*/

#define CUST_STK3X1X_PSCTRL_VAL		0x31  // ps_persistance=1, BIT[5:4]=>ps_gain=64X, PS_IT=0.37ms 
#define CUST_STK3X1X_LEDCTRL_VAL	0xBF  // BIT[7:6]=>50mA IRDR, 64/64 LED duty 
#define CUST_STK3X1X_WAIT_VAL		0x7	  // 50 ms 

/*********************************************/
/*STK3X1X als cust setting*/

#define CUST_STK3X1X_ALSCTRL_VAL	0x38  // als_persistance=1, als_gain=64X, ALS_IT=50ms

#endif
