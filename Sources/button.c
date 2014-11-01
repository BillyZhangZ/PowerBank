/*
 * button.c
 *
 *  Created on: Sep 4, 2014
 *      Author: B51761
 */
#include"button.h"
void button_init()
{
	/*PTC5*/
	GPIOC_PDDR &= ~GPIO_PDDR_PDD(1<<5);
	PORTC_PCR5 |= PORT_PCR_MUX(1)/*|PORT_PCR_IRQC(0x0a) */| PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
}

void button_deinit()
{
	/*PTC5*/
	GPIOC_PDDR |= GPIO_PDDR_PDD(1<<5);
	GPIOC_PDOR |= GPIO_PDOR_PDO(1<<5);
	PORTC_PCR5 |= PORT_PCR_MUX(1);
}

/*
 * low asserted
 */
uint8_t button_get_value()
{
	return (GPIOC_PDIR & (1<<5));
}

