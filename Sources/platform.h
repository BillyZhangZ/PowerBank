/*
 * platform.h
 *
 *  Created on: Sep 10, 2014
 *      Author: B51761
 */

#ifndef PLATFORM_H_
#define PLATFORM_H_
#include<stdint.h>
#include"MKL16Z4.h"

#define ADC_BATT_CHN 8
#define ADC_CURR_CHN 9

#define ADC_VALUE_4V (3620 - 1000)
#define ADC_VALUE_3P7V (3350 - 1000)
#define ADC_VALUE_3P4V (3000 - 1000)
#define ADC_VALUE_3V (2800 - 1000)
#define NO_LOAD_VALUE 50

enum system_state{IDLE,CHARGING,BOOSTING,CHARGE_BOOST};
enum system_event{NONE,SWITCH_PRESSED,LOW_POWER_TIMER,CHARGER_PLUGIN,DEVICE_PLUGIN};

void start_boost();
uint8_t get_battery_level();
uint8_t disp_batt_level_charge(void);
uint8_t disp_batt_level_discharge(void);
void disp_batt_level_switch(void);
void power_bank_state_machine(void);
#endif /* PLATFORM_H_ */
