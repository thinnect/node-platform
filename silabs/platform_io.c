/**
 * Generic Platform IO functions.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma, Madis Uusjärv
 */
#include "em_cmu.h"
#include "em_gpio.h"

#include "platform_io.h"

/******************************************************************************
 * Initialize platform LEDs. LEDs always go 0->1->2->...
 ******************************************************************************/
void PLATFORM_LedsInit ()
{
    CMU_ClockEnable(cmuClock_GPIO, true);
    #ifdef PLATFORM_LED0_PORT
    GPIO_PinModeSet(PLATFORM_LED0_PORT, PLATFORM_LED0_PIN, gpioModePushPull, 0);
    #endif
    #ifdef PLATFORM_LED1_PORT
    GPIO_PinModeSet(PLATFORM_LED1_PORT, PLATFORM_LED1_PIN, gpioModePushPull, 0);
    #endif
    #ifdef PLATFORM_LED2_PORT
    GPIO_PinModeSet(PLATFORM_LED2_PORT, PLATFORM_LED2_PIN, gpioModePushPull, 0);
    #endif
}

static void ledSet (GPIO_Port_TypeDef port, unsigned int pin, bool on)
{
    #ifdef PLATFORM_LEDS_INVERTED
        if (on)
        {
            GPIO_PinOutClear(port, pin);
        }
        else
        {
            GPIO_PinOutSet(port, pin);
        }
    #else
        if (on)
        {
            GPIO_PinOutSet(port, pin);
        }
        else
        {
            GPIO_PinOutClear(port, pin);
        }
    #endif
}

/******************************************************************************
 * Set LEDs to the specified value
 * @param leds - bitmap of LED values
 ******************************************************************************/
void PLATFORM_LedsSet (uint8_t leds)
{
    #ifdef PLATFORM_LED0_PORT
    if(leds & 1)
    {
        ledSet(PLATFORM_LED0_PORT, PLATFORM_LED0_PIN, true);
    }
    else
    {
        ledSet(PLATFORM_LED0_PORT, PLATFORM_LED0_PIN, false);
    }
    #endif
    #ifdef PLATFORM_LED1_PORT
    if(leds & 2)
    {
        ledSet(PLATFORM_LED1_PORT, PLATFORM_LED1_PIN, true);
    }
    else
    {
        ledSet(PLATFORM_LED1_PORT, PLATFORM_LED1_PIN, false);
    }
    #endif
    #ifdef PLATFORM_LED2_PORT
    if(leds & 4)
    {
        ledSet(PLATFORM_LED2_PORT, PLATFORM_LED2_PIN, true);
    }
    else
    {
        ledSet(PLATFORM_LED2_PORT, PLATFORM_LED2_PIN, false);
    }
    #endif
}

static uint8_t ledGet(GPIO_Port_TypeDef port, unsigned int pin)
{
    #ifdef PLATFORM_LEDS_INVERTED
        return 1 & ~GPIO_PinOutGet(port, pin);
    #else
        return GPIO_PinOutGet(port, pin);
    #endif
}

/******************************************************************************
 * Get current LEDs state
 ******************************************************************************/
uint8_t PLATFORM_LedsGet ()
{
    uint8_t leds = 0;

    #ifdef PLATFORM_LED0_PORT
    leds |= (ledGet(PLATFORM_LED0_PORT, PLATFORM_LED0_PIN) << 0);
    #endif
    #ifdef PLATFORM_LED1_PORT
    leds |= (ledGet(PLATFORM_LED1_PORT, PLATFORM_LED1_PIN) << 1);
    #endif
    #ifdef PLATFORM_LED2_PORT
    leds |= (ledGet(PLATFORM_LED2_PORT, PLATFORM_LED2_PIN) << 2);
    #endif

    return leds;
}

/******************************************************************************
 * Enable PIN1..PIN4
 ******************************************************************************/
void PLATFORM_GpioPinInit ()
{
    CMU_ClockEnable(cmuClock_GPIO, true);
    #ifdef PLATFORM_PIN1_PORT
    GPIO_PinModeSet(PLATFORM_PIN1_PORT, PLATFORM_PIN1_PIN, gpioModePushPull, 0);
    #endif
    #ifdef PLATFORM_PIN2_PORT
    GPIO_PinModeSet(PLATFORM_PIN2_PORT, PLATFORM_PIN2_PIN, gpioModePushPull, 0);
    #endif
    #ifdef PLATFORM_PIN3_PORT
    GPIO_PinModeSet(PLATFORM_PIN3_PORT, PLATFORM_PIN3_PIN, gpioModePushPull, 0);
    #endif
    #ifdef PLATFORM_PIN4_PORT
    GPIO_PinModeSet(PLATFORM_PIN4_PORT, PLATFORM_PIN4_PIN, gpioModePushPull, 0);
    #endif
}

/******************************************************************************
 * Clear GPIO pin
 * @param pin_nr - TSB pin number (1..4) in JP1 header
 ******************************************************************************/
void PLATFORM_ClearGpioPin (uint8_t pin_nr)
{
    switch (pin_nr)
    {
        case 1:
            #ifdef PLATFORM_PIN1_PORT
            GPIO_PinOutClear(PLATFORM_PIN1_PORT, PLATFORM_PIN1_PIN);
            #endif
        break;

        case 2:
            #ifdef PLATFORM_PIN2_PORT
            GPIO_PinOutClear(PLATFORM_PIN2_PORT, PLATFORM_PIN2_PIN);
            #endif
        break;

        case 3:
            #ifdef PLATFORM_PIN3_PORT
            GPIO_PinOutClear(PLATFORM_PIN3_PORT, PLATFORM_PIN3_PIN);
            #endif
        break;

        case 4:
            #ifdef PLATFORM_PIN4_PORT
            GPIO_PinOutClear(PLATFORM_PIN4_PORT, PLATFORM_PIN4_PIN);
            #endif
        break;
    }
}

/******************************************************************************
 * Set GPIO pin
 * @param pin_nr - TSB pin number (1..4) in JP1 header
 ******************************************************************************/
void PLATFORM_SetGpioPin (uint8_t pin_nr)
{
    switch (pin_nr)
    {
        case 1:
            #ifdef PLATFORM_PIN1_PORT
            GPIO_PinOutSet(PLATFORM_PIN1_PORT, PLATFORM_PIN1_PIN);
            #endif
        break;

        case 2:
            #ifdef PLATFORM_PIN2_PORT
            GPIO_PinOutSet(PLATFORM_PIN2_PORT, PLATFORM_PIN2_PIN);
            #endif
        break;

        case 3:
            #ifdef PLATFORM_PIN3_PORT
            GPIO_PinOutSet(PLATFORM_PIN3_PORT, PLATFORM_PIN3_PIN);
            #endif
        break;

        case 4:
            #ifdef PLATFORM_PIN4_PORT
            GPIO_PinOutSet(PLATFORM_PIN4_PORT, PLATFORM_PIN4_PIN);
            #endif
        break;
    }
}

/******************************************************************************
 * Get GPIO pin
 * @param pin_nr - TSB pin number (1..4) in JP1 header
 * @param return true if PIN is high.
 ******************************************************************************/
bool PLATFORM_GetGpioPin (uint8_t pin_nr)
{
    switch (pin_nr)
    {
        case 1:
            #ifdef PLATFORM_PIN1_PORT
            return 0 != GPIO_PinOutGet(PLATFORM_PIN1_PORT, PLATFORM_PIN1_PIN);
            #endif
        break;

        case 2:
            #ifdef PLATFORM_PIN2_PORT
            return 0 != GPIO_PinOutGet(PLATFORM_PIN2_PORT, PLATFORM_PIN2_PIN);
            #endif
        break;

        case 3:
            #ifdef PLATFORM_PIN3_PORT
            return 0 != GPIO_PinOutGet(PLATFORM_PIN3_PORT, PLATFORM_PIN3_PIN);
            #endif
        break;

        case 4:
            #ifdef PLATFORM_PIN4_PORT
            return 0 != GPIO_PinOutGet(PLATFORM_PIN4_PORT, PLATFORM_PIN4_PIN);
            #endif
        break;
    }
    return false;
}

/******************************************************************************
 * Toggle GPIO pin
 * @param pin_nr - TSB pin number (1..4) in JP1 header
 ******************************************************************************/
void PLATFORM_ToggleGpioPin (uint8_t pin_nr)
{
    switch (pin_nr)
    {
        case 1:
            #ifdef PLATFORM_PIN1_PORT
            GPIO_PinOutToggle(PLATFORM_PIN1_PORT, PLATFORM_PIN1_PIN);
            #endif
        break;

        case 2:
            #ifdef PLATFORM_PIN2_PORT
            GPIO_PinOutToggle(PLATFORM_PIN2_PORT, PLATFORM_PIN2_PIN);
            #endif
        break;

        case 3:
            #ifdef PLATFORM_PIN3_PORT
            GPIO_PinOutToggle(PLATFORM_PIN3_PORT, PLATFORM_PIN3_PIN);
            #endif
        break;

        case 4:
            #ifdef PLATFORM_PIN4_PORT
            GPIO_PinOutToggle(PLATFORM_PIN4_PORT, PLATFORM_PIN4_PIN);
            #endif
        break;
    }
}

/******************************************************************************
 * Enable TSB button
 ******************************************************************************/
void PLATFORM_ButtonPinInit ()
{
    CMU_ClockEnable(cmuClock_GPIO, true);
    #ifdef PLATFORM_BUTTON_PORT
    GPIO_PinModeSet(PLATFORM_BUTTON_PORT, PLATFORM_BUTTON_PIN, gpioModeInputPull, 1);
    #endif
}

bool PLATFORM_ButtonGet ()
{
    #ifdef PLATFORM_BUTTON_PORT
    return 0 == GPIO_PinInGet(PLATFORM_BUTTON_PORT, PLATFORM_BUTTON_PIN);
    #else
    return false;
    #endif
}
