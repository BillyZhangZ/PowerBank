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
	for(i = 0;i < BATT_VALUE_NUM; i++) 
	{
		batt_history_value[i] = adc_read(ADC_BATT_CHN);
	}
	for(i = 0;i < CURR_VALUE_NUM; i++) curr_history_value[i] = adc_read(ADC_CURR_CHN);
}
void statistic_curr_init()
{
	int i = 0;
	 
	for(i = 0;i < CURR_VALUE_NUM; i++) curr_history_value[i] = adc_read(ADC_CURR_CHN);
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

	uint16_t value = adc_read_save(ADC_BATT_CHN);
	if( value < ADC_VALUE_3V)
	{
		debug_printf((" battery low than 3V %u\n",value));
		return 0;
	}
	else 

		return 1;
}
void start_boost()
{	
#if 0
	/*PTC1, EMU_CB*/
	GPIOC_PDOR |= GPIO_PDOR_PDO(1<<1);
#endif
	/*set mode_sel(PTE0) high*/
	GPIOE_PDOR |= GPIO_PDOR_PDO(0x01);
	/*about 1s~2s delay is a must here to let it fully output*/
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
	uint16_t battery_value = 0;
	battery_value = adc_read_save(ADC_BATT_CHN);

	debug_printf(("batt %u\n",battery_value));

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
	case 2:
		if(cnt)
			led_ctrl(0,0,0,0);
		else
			led_ctrl(1,0,0,0);
		break;
	case 3:
		if(cnt)
			led_ctrl(1,0,0,0);
		else
			led_ctrl(1,1,0,0);
		break;
	case 4:
		if(cnt)
			led_ctrl(1,1,0,0);
		else
			led_ctrl(1,1,1,0);
		break;
	case 5:
		if(cnt)
			led_ctrl(1,1,1,0);
		else
			led_ctrl(1,1,1,1);
		break;
	default:
			led_ctrl(1,1,1,1);	
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
	uint16_t battery_value = 0, level = 0, level_value[5] = {ADC_VALUE_3V,ADC_VALUE_3P4V,ADC_VALUE_3P7V,ADC_VALUE_4V};
	//set initialization of 5 to let it update last_level first
	static int last_level = 5;
	battery_value = adc_read_save(ADC_BATT_CHN);
	level = get_battery_level();

	if(level > last_level)  
	{
		if(battery_value < level_value[level-1] + CLAMP_VALUE)
			level--;
	}
	last_level = level;

	switch(level)
	{
	case 1:
		low_power_alert();
		//low battery detected
		return 1;
		break;
	case 2:
		led_ctrl(1,0,0,0);
		break;
	case 3:
		led_ctrl(1,1,0,0);
		break;
	case 4:
		led_ctrl(1,1,1,0);
		break;
	case 5:
		led_ctrl(1,1,1,1);
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
		led_ctrl(0,0,0,0);
		break;
	case 2:
		led_ctrl(1,0,0,0);
		break;
	case 3:
		led_ctrl(1,1,0,0);
		break;
	case 4:
		led_ctrl(1,1,1,0);
		break;
	case 5:
		led_ctrl(1,1,1,1);
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
 
	uint16_t value = adc_read_save(ADC_CURR_CHN);
	if( value < NO_LOAD_VALUE)
	{
		debug_printf((" low boost current %u\n",value));
		return 1;
	}
	else 
		return 0;
	
}

void low_power_alert()
{
	int i = 8;
	while(i--)
	{
		delay_busy(400);
		led_toggle(1);led_toggle(2);led_toggle(3);led_toggle(4);
	}
	led_ctrl(0,0,0,0);					
}

void start_indicate()
{
	led_ctrl(0,0,0,0);
	led_ctrl(1,0,0,0);
	delay_busy(1000);
	led_ctrl(1,1,0,0);
	delay_busy(1000);
	led_ctrl(1,1,1,0);
	delay_busy(1000);
	led_ctrl(1,1,1,1);
	delay_busy(1000);
	led_ctrl(0,0,0,0);
}

void device_plugout_indicate()
{
	led_ctrl(0,0,0,0);
	led_ctrl(1,0,0,1);
	delay_busy(500);
	led_ctrl(0,1,1,0);
	delay_busy(500);
	led_ctrl(1,0,0,1);
	delay_busy(500);
	led_ctrl(0,1,1,0);
	delay_busy(500);
	led_ctrl(0,0,0,0);
}
void power_bank_state_machine(void)
{
	char off_the_led = 0;
	start_indicate();
	//watchdog_init();
	debug_printf(("go into idle\n"));
	for(;;){
		/*Sleep and wait for events: buttons, plug-ins, low power timer,state updated in their ISR*/
		/*this time must less than watch dog timeout, 1024ms*/
		
		lptimer_init(500);
		
		enter_lls();

		//watchdog_reset(); 
		 
		//debug_printf(("state %d\n", gcur_state));
		switch(gcur_state)
		{
		case IDLE:
			
			if(gevent == LOW_POWER_TIMER)
			{
				//debug_printf(("%d second\n", i++));
				if((off_the_led && (--off_the_led) == 0))
				{
					led_off(1);
					led_off(2);
					led_off(3);
					led_off(4);
				}
				
			}
			else if(gevent == SWITCH_PRESSED)
			{
				//indicate power 
				disp_batt_level_switch();
				off_the_led = 4;
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
					led_ctrl(1,1,1,0);		
					//blink and then sleep
					low_power_alert();
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
					//do nothing here
					led_ctrl(1,1,1,1);
					break;
				}
				else disp_batt_level_charge();
					
				 if(charger_plugout())
				 {
					debug_printf(("charger plugged out go into idle\n"));
					glast_state = gcur_state;
					gcur_state = IDLE;
					unmask_switch();
					led_ctrl(0,0,0,0);
					break;
				 }
				
			}
			else if((gevent == CHARGER_PLUGOUT))
			{
				debug_printf(("charger plugged out go into idle\n"));
				glast_state = gcur_state;
				gcur_state = IDLE;
				unmask_switch();
				led_ctrl(0,0,0,0);
			}
			else if(gevent == DEVICE_PLUGIN)
			{
				//start boosting
				if(power_check()) start_boost();
				else
				{
					//blink and then sleep
					low_power_alert();
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
					device_plugout_indicate();
					break;
				}
			
				if(disp_batt_level_discharge() == 1)
				{
					
					//power alert
					stop_boost();
					debug_printf(("power bank battery low\n"));
					debug_printf(("go into idle\n"));
				 
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
					device_plugout_indicate();
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
