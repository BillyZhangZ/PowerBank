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
}


uint8_t gwakeup_source;
uint8_t power_output_flag = 0;



int main(void)
{
	uint16_t adc_value = 0, i = 0;
	printf("build time %s\n",__TIME__);
	init_hardware();
#if 0
	while(1)
	{
		printf("%u\t",adc_read(ADC_CURR_CHN));
		delay_busy(100);
		if(i++ %10 == 0) printf("\n");
	}
#endif
	/*Power bank state machine*/
	power_bank_state_machine();
	
	return 0;
}
