/*
 * Led.c
 *
 *  Created on: Sep 4, 2014
 *      Author: B51761
 */
#include<stdint.h>
#include"led.h"
/*
 * 
 */
void led_init(void)
{
  SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTC_MASK);
	//led1
	GPIOC_PDDR |= GPIO_PDDR_PDD(1<<3);
	GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<3);
	PORTC_PCR3 |= PORT_PCR_MUX(1);
	//led2
	GPIOC_PDDR |= GPIO_PDDR_PDD(1<<2);
	GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<2);
	PORTC_PCR2 |= PORT_PCR_MUX(1);

	//led3
	GPIOA_PDDR |= GPIO_PDDR_PDD(1<<1);
	GPIOA_PDOR &= ~GPIO_PDOR_PDO(1<<1);
	PORTA_PCR1 |= PORT_PCR_MUX(1);
	//led4
	GPIOA_PDDR |= GPIO_PDDR_PDD(1<<2);
	GPIOA_PDOR &= ~GPIO_PDOR_PDO(1<<2);
	PORTA_PCR2 |= PORT_PCR_MUX(1);

}
/*
 * 
 */
void led_deinit(void)
{

	//led1
		GPIOC_PDDR |= GPIO_PDDR_PDD(1<<3);
		GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<3);
		PORTC_PCR3 |= PORT_PCR_MUX(1);
		//led2
		GPIOC_PDDR |= GPIO_PDDR_PDD(1<<2);
		GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<2);
		PORTC_PCR2 |= PORT_PCR_MUX(1);

		//led3
		GPIOA_PDDR |= GPIO_PDDR_PDD(1<<1);
		GPIOA_PDOR &= ~GPIO_PDOR_PDO(1<<1);
		PORTA_PCR1 |= PORT_PCR_MUX(1);
		//led4
		GPIOA_PDDR |= GPIO_PDDR_PDD(1<<2);
		GPIOA_PDOR &= ~GPIO_PDOR_PDO(1<<2);
		PORTA_PCR2 |= PORT_PCR_MUX(1);
    SIM_SCGC5 &= ~(SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTC_MASK);
}
/*
 * 
 */
void led_on(uint8_t num)
{
	switch(num)
	{
	case 1:
		GPIOC_PDOR |= GPIO_PDOR_PDO(1<<3);
		break;
	case 2:
		GPIOC_PDOR |= GPIO_PDOR_PDO(1<<2);
		break;
	case 3:
		GPIOA_PDOR |= GPIO_PDOR_PDO(1<<1);
		break;
	case 4:
		GPIOA_PDOR |= GPIO_PDOR_PDO(1<<2);
		break;
	default:
		break;
	}	
	
}

/*
 */
void led_off(uint8_t num)
{
	switch(num)
	{
	case 1:
		GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<3);
		break;
	case 2:
		GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<2);
		break;
	case 3:
		GPIOA_PDOR &= ~GPIO_PDOR_PDO(1<<1);
		break;
	case 4:
		GPIOA_PDOR &= ~GPIO_PDOR_PDO(1<<2);
		break;
	default:
		break;
	}	

}

/*
 */
void led_toggle(uint8_t num)
{
	switch(num)
	{
		case 1:
			GPIOC_PTOR |= GPIO_PTOR_PTTO(1<<3);
			break;
		case 2:
			GPIOC_PTOR |= GPIO_PTOR_PTTO(1<<2);
			break;
		case 3:
			GPIOA_PTOR = GPIO_PTOR_PTTO(1<<1);
			break;
		case 4:
			GPIOA_PTOR = GPIO_PTOR_PTTO(1<<2);
			break;
		default:
			break;
	}	
	
}

void led_ctrl(int led1, int led2, int led3, int led4)
{
	led1?led_on(1):led_off(1);
	led2?led_on(2):led_off(2);
	led3?led_on(3):led_off(3);
	led4?led_on(4):led_off(4);
}
