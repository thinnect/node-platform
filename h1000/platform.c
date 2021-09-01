/**
 * Platform specific functions.
 *
 * Copyright Thinnect Inc. 2021
 * @license MIT
*/
#include "platform.h"

bool buttonstate = 0;

int platform_init()
{
	int ret = hal_gpio_pin_init(LED_1,OEN);
	if(PPlus_SUCCESS != ret)
	{
		return ret;
	}
	ret = hal_gpio_pin_init(LED_2,OEN);
	if(PPlus_SUCCESS != ret)
	{
		return ret;
	}
	ret = hal_gpio_pin_init(LED_3,OEN);
	if(PPlus_SUCCESS != ret)
	{
		return ret;
	}
	
	hal_gpio_write(LED_1, 0);
	if(PPlus_SUCCESS != ret)
	{
		return ret;
	}
	
	hal_gpio_write(LED_2, 0);
	if(PPlus_SUCCESS != ret)
	{
		return ret;
	}
	
	hal_gpio_write(LED_3, 0);
	if(PPlus_SUCCESS != ret)
	{
		return ret;
	}
	
	ret = hal_gpio_pin_init(PLATFORM_BUTTON,IE);
	if(PPlus_SUCCESS != ret)
	{
		return ret;
	}
	
	hal_gpio_pull_set(PLATFORM_BUTTON,PULL_DOWN);
	if(PPlus_SUCCESS != ret)
	{
		return ret;
	}
	
	ret = hal_gpio_pin_init(FAKE_VCC, OEN);
	if(PPlus_SUCCESS != ret)
	{
		return ret;
	}
	
	hal_gpio_write(FAKE_VCC, 1);
	if(PPlus_SUCCESS != ret)
	{
		return ret;
	}
	
	ret = hal_gpio_pin_init(PLATFORM_BUTTON,IE);
	if(PPlus_SUCCESS != ret)
	{
		return ret;
	}
	
	
	return PPlus_SUCCESS;
}
void platform_setleds(uint8_t leds)
{
	//LOG("Count is %d \r\n", leds);
	
	hal_gpio_write(LED_1, leds & LED_1_MASK);
	hal_gpio_write(LED_2, leds & LED_2_MASK);
	hal_gpio_write(LED_3, leds & LED_3_MASK);
}
bool platform_button()
{
	buttonstate = hal_gpio_read(PLATFORM_BUTTON);
	//WaitMs(200);
	return (buttonstate != hal_gpio_read(PLATFORM_BUTTON));
}
