/*
 * platform.c
 *
 *  Created on: Sep 10, 2014
 *      Author: B51761
 */
#include"platform.h"
#include"adc16.h"
#include"led.h"
#include"llwu.h"
#include"lptimer.h"
#include"watchdog.h"
void do_nothing(const char * _EWL_RESTRICT format, ...)
{
	//delay_busy(10);
}
//#define DEBUG
#ifdef DEBUG
#define debug_printf(s) printf s
#else
#define  debug_printf(s) do_nothing(s)
#endif

enum system_state gcur_state = IDLE;
enum system_state glast_state = IDLE;
enum system_event gevent = NONE;

#define BATT_VALUE_NUM 100
#define CURR_VALUE_NUM 4
static uint16_t batt_history_value[BATT_VALUE_NUM];
static uint16_t batt_index = 0;
static uint16_t curr_history_value[CURR_VALUE_NUM];
static uint16_t curr_index = 0;
void statistic_init()
{
	int i = 0;
	for(i = 0;i < BATT_VALUE_NUM; i++) batt_history_value[i] = ADC_VALUE_4V;
	for(i = 0;i < CURR_VALUE_NUM; i++) curr_history_value[i] = NO_LOAD_VALUE;
}
void statistic_curr_init()
{
	int i = 0;
	 
	for(i = 0;i < CURR_VALUE_NUM; i++) curr_history_value[i] = NO_LOAD_VALUE;
}

uint16_t get_value_history(uint8_t ch)
{
	uint32_t value = 0;
	int i = 0;
	if(ch == ADC_BATT_CHN)
	{
		for(i = 0;i < BATT_VALUE_NUM; i++) value += batt_history_value[i];
		return (value/BATT_VALUE_NUM);
	}
	else{
			for(i = 0;i < CURR_VALUE_NUM; i++) 
			{
				//value += curr_history_value[i];
				if(value < curr_history_value[i]) value = curr_history_value[i];
			}
			//return (value/CURR_VALUE_NUM);
			return value;
	}

}
void update_value_history(uint8_t ch, uint16_t value)
{
	if(ch == ADC_BATT_CHN)
	{
		if(batt_index == BATT_VALUE_NUM) batt_index = 0;
		batt_history_value[batt_index] = value;
		batt_index++;
	}
	else
	{
		if(curr_index == CURR_VALUE_NUM) curr_index = 0;
			curr_history_value[curr_index] = value;
			curr_index++;
	}
}

 
/*
 * check is power bank is available
 * */
unsigned char power_check()
{
#if 1
	uint16_t value = adc_multi_read(ADC_BATT_CHN, 20);
	if( value < ADC_VALUE_3V)
	{
		debug_printf((" battery low than 3V %u\n",value));
		return 0;
	}
	else 
#endif
		return 1;
}
void start_boost()
{
	 
#if 0
	/*EMU_CB high*/
	GPIOC_PDOR |= GPIO_PDOR_PDO(1<<1);
#endif
	/*set mode_sel(PTE0) high*/
	GPIOE_PDOR |= GPIO_PDOR_PDO(0x01);
	watchdog_reset(); 
	delay_busy(400);
	watchdog_reset(); 
	delay_busy(400);
	watchdog_reset(); 
	delay_busy(400);
	watchdog_reset(); 
	/*sleep high*/
	GPIOD_PDOR |= GPIO_PDOR_PDO(0x80);
	 
	
	/*detect 5v to USB host flag,nBOOST(PTC6), wait to low*/
	while((GPIOC_PDIR & GPIO_PDIR_PDI(1<<6)));
	
	 
	//delay_busy(1000);
	//__asm("bkpt");
}

void stop_boost()
{
	/*turn off 5v output*/
	/*set mode_sel(PTE0) low*/
	GPIOE_PDOR &= ~GPIO_PDOR_PDO(0x01);
	/*sleep low*/
	GPIOD_PDOR &= ~GPIO_PDOR_PDO(0x80);
}
 
/*
 * get battery level 
 * 1 0~3V
 * 2 3V~3.4V
 * 3 3.4V~3.7V
 * 4 3.7V~4.0V
 * 5 4.0 ~4.2V
 */
uint8_t get_battery_level()
{
	uint16_t battery_value = 0, sample_cnt = 10;
	battery_value = adc_multi_read(ADC_BATT_CHN, sample_cnt);
#if 0
	debug_printf(("batt %u\n",battery_value));
	 
#endif
	if(battery_value > ADC_VALUE_4V)
	{
		return 5;
	}
	else if(battery_value > ADC_VALUE_3P7V)
	{
		return 4;
	}
	else if(battery_value > ADC_VALUE_3P4V)
	{
		return 3;
	}
	else if(battery_value > ADC_VALUE_3V)
	{
		return 2;
	}
	else
	{
		return 1;
	}
}
uint8_t disp_batt_level_charge(void)
{
	static uint8_t cnt = 0;

	switch(get_battery_level())
	{
	case 1:
		if(cnt){
		led_off(1);
		led_off(2);
		led_off(3);
		led_off(4);
		}
	else{
		led_on(1);
		led_on(2);
		led_on(3);
		led_on(4);
	}
		break;
	case 2:
		if(cnt)
			led_off(1);
		else
			led_on(1);
		led_off(2);
		led_off(3);
		led_off(4);
		break;
	case 3:
		led_on(1);
		if(cnt)
			led_off(2);
		else
			led_on(2);
		led_off(3);
		led_off(4);
		break;
	case 4:
		led_on(1);
		led_on(2);
		if(cnt)
			led_off(3);
		else
			led_on(3);
	
		led_off(4);
		break;
	case 5:
		led_on(1);
		led_on(2);
		led_on(3);
		if(cnt)
			led_off(4);
		else
			led_on(4);
		break;
	default:
		led_on(1);led_on(2);
		led_on(3);led_on(4);
		return 1;
		break;
	}
	cnt = !cnt;
	return 0;
}

/*
 * indicate power bank status in discharge mode
 * return 0 indicate power bank is OK
 * 			1 low battery detected
 */
uint8_t disp_batt_level_discharge(void)
{
	static uint8_t cnt = 0;
	 
	int i = 4;
	switch(get_battery_level())
	{
	case 1:
		led_on(1);led_on(2);
		led_on(3);led_on(4);
		i = 100000; while(i--);
		led_off(1);led_off(2);
		led_off(3);led_off(4);
		i = 100000; while(i--);	
					
		led_on(1);led_on(2);
		led_on(3);led_on(4);
		i = 100000; while(i--);
									
		led_off(1);led_off(2);
		led_off(3);led_off(4);
	
		
		//low battery detected
		return 1;
		break;
	case 2:
		led_on(1);
		led_off(2);
		led_off(3);
		led_off(4);
		break;
	case 3:
		led_on(1);
		led_on(2);
		led_off(3);
		led_off(4);
		break;
	case 4:
		led_on(1);
		led_on(2);
		led_on(3);
		led_off(4);
		break;
	case 5:
		led_on(1);
		led_on(2);
		led_on(3);
		led_on(4);
		break;
	default:
		break;
	}
	
	return 0;
}
/*
 * indicate power bank status
 * when button/switch is pressed
 * 
 */
void disp_batt_level_switch(void)
{		
	switch(get_battery_level())
	{
	case 1:
		led_off(1);led_off(2);
		led_off(3);led_off(4);
		break;
	case 2:
		led_on(1);
		led_off(2);
		led_off(3);
		led_off(4);
		break;
	case 3:
		led_on(1);
		led_on(2);
		led_off(3);
		led_off(4);
		break;
	case 4:
		led_on(1);
		led_on(2);
		led_on(3);
		led_off(4);
		break;
	case 5:
		led_on(1);
		led_on(2);
		led_on(3);
		led_on(4);
		break;
	default:
		break;
	}
}
/*
 * detect if power bank if full 
 * 1 power bank is full
 * 0 not full
 */
uint8_t power_bank_full()
{
	//pin detect
	
	if((GPIOD_PDIR & GPIO_PDIR_PDI(1<<4)) == 0/*nACOK low*/ && (GPIOD_PDIR & GPIO_PDIR_PDI(1<<6))/*nCHG high*/)
	{
		//full
		return 1;
	}
	else	return 0;
}

/*
 * detect if charger is plugged out
 * 1 plugged out
 * 0 remain plugged
 */
uint8_t charger_plugout()
{
	//nACOK
	if(GPIOD_PDIR & GPIO_PDIR_PDI(1<<4))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*
 * detect if device is plugged out
 * 1 plugged out
 * 0 not
 * 
 */
uint8_t device_plug_out()
{
#if 1
	uint16_t value = adc_multi_read(ADC_CURR_CHN, 20);
	if( value < NO_LOAD_VALUE)
	{
		debug_printf((" low boost current %u\n",value));
		return 1;
	}
	else 
		return 0;
#elif 0
	int i = 0;
	while(1)
	{
		debug_printf(("cur %u\t",adc_read(ADC_CURR_CHN)));
		
		if(adc_read(ADC_CURR_CHN) < NO_LOAD_VALUE) i++;
		else return 0;
		if(i == 40) return 1;
		delay_busy(10);
	}
	
	return 0;
#else
	static uint16_t last_curr_value = 0;
	uint16_t curr_value = adc_read(ADC_CURR_CHN);
	
	if( (curr_value < NO_LOAD_VALUE) && last_curr_value < NO_LOAD_VALUE)
	{
		debug_printf((" low boost current %u\n",curr_value));
		return 1;
	}
	else 
		return 0;
#endif
	
}
void power_bank_state_machine(void)
{
	//watchdog_init();
	 
	debug_printf(("go into idle\n"));
	for(;;){
		/*Sleep and wait for events: buttons, plug-ins, low power timer,state updated in their ISR*/
		/*this time must less than watch dog timeout, 1024ms*/
		

		lptimer_init(500);
		
		enter_lls();

	//	watchdog_reset(); 
		 
		//debug_printf(("state %d\n", gcur_state));
		switch(gcur_state)
		{
		case IDLE:
			
			if(gevent == LOW_POWER_TIMER)
			{
				//debug_printf(("%d second\n", i++));
#if 0
				led_toggle(1);
				 
				 
#else
				led_off(1);
#endif
				led_off(2);
				led_off(3);
				led_off(4);
			}
			else if(gevent == SWITCH_PRESSED)
			{
				if(power_check()) start_boost();
				else
				{
					//blink and then sleep
					led_on(1);
					delay_busy(50);
					led_toggle(1);
					delay_busy(1000);
					led_toggle(1);
					delay_busy(50);
					led_toggle(1);
					delay_busy(1000);
					led_toggle(1);
					delay_busy(50);
					led_toggle(1);
					break;
				}
				//indicate power 
				disp_batt_level_switch();
			}
			else if(gevent == CHARGER_PLUGIN)
			{
				debug_printf(("go into charging\n"));
				glast_state = gcur_state;
				gcur_state = CHARGING;
				//automatically charging
				//mask switch wake up
				mask_switch();
			}
			else if(gevent == DEVICE_PLUGIN)
			{
				
				if(power_check()) start_boost();
				else
				{
					//blink and then sleep
					led_on(1);
					delay_busy(50);
					led_toggle(1);
					delay_busy(1000);
					led_toggle(1);
					delay_busy(50);
					led_toggle(1);
					delay_busy(1000);
					led_toggle(1);
					delay_busy(50);
					led_toggle(1);
					break;
				}
				mask_switch();
				debug_printf(("go into boosting\n"));
				glast_state = gcur_state;
				gcur_state = BOOSTING;
			}
			break;
		case CHARGING:
			if(gevent == LOW_POWER_TIMER)
			{	
				//debug_printf(("%d second\n", i++));
				/***********************************************************/
				//detect if power bank is full
				if(power_bank_full())
				{
					debug_printf(("power bank full\n"));
					debug_printf(("go into idle\n"));
					glast_state = gcur_state;
					gcur_state = IDLE;
					unmask_switch();
					//led will be turn off in IDLE
					break;
				}
				
				 if(charger_plugout())
				 {
					 debug_printf(("charger plugged out go into idle\n"));
					glast_state = gcur_state;
					gcur_state = IDLE;
					unmask_switch();
				 }
				disp_batt_level_charge();
			}
			else if((gevent == CHARGER_PLUGOUT))
			{
				debug_printf(("charger plugged out go into idle\n"));
				glast_state = gcur_state;
				gcur_state = IDLE;
				unmask_switch();
			}
			else if(gevent == DEVICE_PLUGIN)
			{
				//start boosting
				if(power_check()) start_boost();
				else
				{
					//blink and then sleep
					led_on(1);
					delay_busy(50);
					led_toggle(1);
					delay_busy(1000);
					led_toggle(1);
					delay_busy(50);
					led_toggle(1);
					delay_busy(1000);
					led_toggle(1);
					delay_busy(50);
					led_toggle(1);
					break;
				}
				debug_printf(("go into charge and boost\n"));
				//update system state,gevent = state_machine;
				glast_state = gcur_state;
				gcur_state = CHARGE_BOOST;
			}
			break;
		case BOOSTING:
			if(gevent == LOW_POWER_TIMER)
			{	
				//debug_printf(("%d second\n", i++));
				//detect if device is plugged out
				if(device_plug_out())
				{
					statistic_curr_init();
					stop_boost();
					debug_printf(("device plugged out\n"));
					debug_printf(("go into idle\n"));
					glast_state = gcur_state;
					gcur_state = IDLE;
					unmask_switch();
					break;
				}
			
				if(disp_batt_level_discharge() == 1)
				{
					//power alert
					stop_boost();
					debug_printf(("power bank battery low\n"));
					debug_printf(("go into idle\n"));
					led_on(1);
					delay_busy(100);
					led_toggle(1);
					delay_busy(100);
					led_toggle(1);
					delay_busy(100);
					led_toggle(1);
					delay_busy(100);
					led_toggle(1);
					delay_busy(100);
					led_toggle(1);
					glast_state = gcur_state;
					gcur_state = IDLE;
					unmask_switch();
					break;
				}
			
			}
			else if(gevent == CHARGER_PLUGIN)
			{
				debug_printf(("go into charge and boost\n"));
				//start_charge
				glast_state = gcur_state;
				gcur_state = CHARGE_BOOST;
			} 
			break;
		case CHARGE_BOOST:
			if(gevent == LOW_POWER_TIMER)
			{	
				//debug_printf(("%d second\n", i++));
						
				if(power_bank_full())
				{
					debug_printf(("power bank full\n"));
					debug_printf(("go into idle\n"));
					glast_state = gcur_state;
					gcur_state = IDLE;
					unmask_switch();
					//led will be turn off in IDLE
					break;
				}
				if(device_plug_out())
				{
					statistic_curr_init();
					stop_boost();
					debug_printf(("go into charging\n"));
					glast_state = gcur_state;
					gcur_state = CHARGING;
					unmask_switch();
					break;
				}
				disp_batt_level_charge();
			}
			else if(gevent == CHARGER_PLUGOUT)
			{
			 	debug_printf(("go into boost\n"));
				glast_state = gcur_state;
				gcur_state = BOOSTING;		 
			}
			break;
		default:
			break;
		}
		gevent = NONE;
		glast_state = gcur_state;
	}
	
}
