#ifndef _CUST_LEDS_DEF_H
#define _CUST_LEDS_DEF_H

#define WT_TORCH_CUST_LED_FLASHLIGHT  /*** torch from led flashlight ***/

//#define CUST_LEDS_BACKLIGHT_PMIC_PARA /* parallel */
//#define CUST_LEDS_BACKLIGHT_PMIC_SERI /* series */
#define NO_NEED_USB_LED
#define WT_ADJUST_BLS_PWM_MIN_DUTY_RATIO  //BLS_PWM背光驱动方式下，调整最低亮度的PWM占空比为4%

enum mt65xx_led_mode
{
	MT65XX_LED_MODE_NONE,
	MT65XX_LED_MODE_PWM,
	MT65XX_LED_MODE_GPIO,
	MT65XX_LED_MODE_PMIC,
	//MT65XX_LED_MODE_CUST,
	MT65XX_LED_MODE_CUST_LCM,	
	MT65XX_LED_MODE_CUST_BLS_PWM,
#ifdef WT_TORCH_CUST_LED_FLASHLIGHT
	MT65XX_LED_MODE_CUST_FLASHLIGHT
#endif
};

enum mt65xx_led_pmic
{
	MT65XX_LED_PMIC_LCD_ISINK=0,
	MT65XX_LED_PMIC_NLED_ISINK0,
	MT65XX_LED_PMIC_NLED_ISINK1,
	MT65XX_LED_PMIC_NLED_ISINK2,
	MT65XX_LED_PMIC_NLED_ISINK3
};
struct PWM_config
{
	int clock_source;
	int div;
	int low_duration;
	int High_duration;
	BOOL pmic_pad;
};
typedef int (*cust_brightness_set)(int level, int div);
typedef int (*cust_set_brightness)(int level);
#ifdef WT_TORCH_CUST_LED_FLASHLIGHT
	typedef int (*cust_flashlight_brightness_set)(char *led_name, int level);
#endif
/*
 * name : must the same as lights HAL
 * mode : control mode
 * data :
 *    PWM:  pwm number
 *    GPIO: gpio id
 *    PMIC: enum mt65xx_led_pmic
 *    CUST: custom set brightness function pointer
*/
struct cust_mt65xx_led {
	char                 *name;
	enum mt65xx_led_mode  mode;
	int                   data;
 struct PWM_config config_data;
};

extern struct cust_mt65xx_led *get_cust_led_list(void);
#endif /* _CUST_LEDS_DEF_H */
