/*
 * Voltage reader API.
 *
 * Copyright Thinnect Inc. 2021
 * @license MIT
 */
#ifndef VOLTAGE_READER_H_
#define VOLTAGE_READER_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * Read voltage from GPIO pin
 *
 * The function will try to acquire the ADC from platform_adc and
 * returns quickly with *false* if the ADC is not available.
 *
 * @param param port - GPIO port used for voltage measurement, e.g.: gpioPortB
 * @param pin - GPIO port used for voltage measurement, e.g.: 1
 * @param pos_inp_port_pin - IADC port & pin, e.g.: iadcPosInputPortBPin1
 * @param ref_vcc_voltage - reference voltage, e.g.: 5000
 * @param ref_gnd_voltage - reference voltage, e.g.: 5000
 * @param iadc_ref - IADC Reference, e.g.: iadcCfgReferenceInt1V2
 * @param ref_voltage - reference voltage, e.g.: 2420
 * @param analog_gain - IADC Analog Gain, e.g.: iadcCfgAnalogGain0P5x
 * @param voltage_mV - pointer for returning battery voltage in millivolts.
 * @return true if voltage read and returned.
 */
bool voltage_reader (GPIO_Port_TypeDef port, 
                     unsigned int pin,
                     IADC_PosInput_t pos_inp_port_pin,
                     uint32_t ref_vcc_voltage,
                     uint32_t ref_gnd_voltage,
                     IADC_CfgReference_t iadc_ref,
                     uint32_t ref_voltage,
                     IADC_CfgAnalogGain_t analog_gain,
                     uint16_t* voltage_mV);
#endif// VOLTAGE_READER_H_