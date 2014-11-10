/*
 * main implementation: use this 'C' sample to create your own application
 *
 */
#include<stdint.h>
#include<stdio.h>
#include"led.h"
#include"button.h"
#include"adc16.h"
#include"llwu.h"
#include"lptimer.h"
#include"charge.h"
#include "delay.h"
#include"platform.h"



#if defined(__GNUC__)

#elif defined(__ICCARM__)

#elif defined(__CWCC__)

#elif defined(__CC_ARM)

#endif


void init_hardware()
{
	/*48MHZ*/
	MCG_C4 |= MCG_C4_DRST_DRS(0x01);
	SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK\
				| SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
 
	adc_init();
	button_init();
	charge_pin_init();
	led_init();	
	llwu_init();
	lptimer_init(5000);
	statistic_init();
}


uint8_t gwakeup_source;
uint8_t power_output_flag = 0;



int main(void)
{
	uint16_t adc_value = 0, i = 0;
	 
	init_hardware();
	
	/*Power bank state machine*/
	power_bank_state_machine();
	
	return 0;
}
