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
	//led1
	GPIOC_PDDR |= GPIO_PDDR_PDD(1<<2);
	GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<2);
	PORTC_PCR2 |= PORT_PCR_MUX(1);
	//led2
	GPIOC_PDDR |= GPIO_PDDR_PDD(1<<3);
	GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<3);
	PORTC_PCR3 |= PORT_PCR_MUX(1);

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
		GPIOC_PDDR |= GPIO_PDDR_PDD(1<<2);
		GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<2);
		PORTC_PCR2 |= PORT_PCR_MUX(1);
		//led2
		GPIOC_PDDR |= GPIO_PDDR_PDD(1<<3);
		GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<3);
		PORTC_PCR3 |= PORT_PCR_MUX(1);

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
void led_on(uint8_t num)
{
	switch(num)
	{
	case 1:
		GPIOC_PDOR |= GPIO_PDOR_PDO(1<<2);
		break;
	case 2:
		GPIOC_PDOR |= GPIO_PDOR_PDO(1<<3);
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
		GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<2);
		break;
	case 2:
		GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<3);
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
			GPIOC_PTOR |= GPIO_PTOR_PTTO(1<<2);
			break;
		case 2:
			GPIOC_PTOR |= GPIO_PTOR_PTTO(1>>3);
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
