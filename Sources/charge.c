/*
 * charge.c
 *
 *  Created on: Sep 5, 2014
 *      Author: B51761
 */
#include"charge.h"
							
void charge_pin_init()
{
	
   /*PTA3, init SWD PTA0 */
   	   //default is swd 
   
   /*PTA4, init NMI */
   	   //default is NMI
	
	/*init uart1 pin*/
	/*PTA18, PORTA_PCR18: ISF=0,MUX=3 */
   PORTA_PCR18 = (uint32_t)((PORTA_PCR18 & (uint32_t)~(uint32_t)(
				  PORT_PCR_ISF_MASK |
				  PORT_PCR_MUX(0x03)
				 )) | (uint32_t)(
				  PORT_PCR_MUX(0x03)
				 ));                                  
   /*PTA19, PORTA_PCR19: ISF=0,MUX=3 */
   PORTA_PCR19 = (uint32_t)((PORTA_PCR19 & (uint32_t)~(uint32_t)(
				  PORT_PCR_ISF_MASK |
				  PORT_PCR_MUX(0x03)
				 )) | (uint32_t)(
				  PORT_PCR_MUX(0x03)
				 ));  
   
	
   /*PTB0, ADC_BATT*/
	GPIOB_PDDR &= ~GPIO_PDDR_PDD(0x01);
	PORTB_PCR0 = (PORTB_PCR0 & (~PORT_PCR_MUX_MASK)) | PORT_PCR_MUX(0);
		
	/*PTB1, ADC_Curr*/
	GPIOB_PDDR &= ~GPIO_PDDR_PDD(0x02);
	PORTB_PCR1 = (PORTB_PCR1 & (~PORT_PCR_MUX_MASK)) | PORT_PCR_MUX(0);

	/*PTC1, init EMU_CB*/
	GPIOC_PDDR |= GPIO_PDDR_PDD(1<<1);
	GPIOC_PDOR &= ~GPIO_PDOR_PDO(1<<1);
	PORTC_PCR1 = (PORTC_PCR4 & (~PORT_PCR_MUX_MASK)) | PORT_PCR_MUX(1);

	/*PTC4, PLUG_INT, USB load detect, input, pull up, falling edge when phone attached*/
	GPIOC_PDDR &= ~GPIO_PDDR_PDD(0x10);
	PORTC_PCR4 = (PORTC_PCR4 & (~PORT_PCR_MUX_MASK)) | PORT_PCR_MUX(1) /*|PORT_PCR_IRQC(0x0a) */| PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

	/*PTC5, KEY_PWR, control pin, input, pull up*/
	GPIOC_PDDR &= ~GPIO_PDDR_PDD(0x20);
	PORTC_PCR5 = (PORTC_PCR5 & (~PORT_PCR_MUX_MASK)) | PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	
	/*PTC6, nBOOST, 5v output indication, input, pull up, low when 5v output to USB host*/
	GPIOC_PDDR &= ~GPIO_PDDR_PDD(0x40);
	PORTC_PCR6 = (PORTC_PCR6 & (~PORT_PCR_MUX_MASK)) | PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	
	
	/*PTC7, CHG_EN, not used, input*/
	GPIOC_PDDR &= ~GPIO_PDDR_PDD(0x80);
	PORTC_PCR7 = (PORTC_PCR7 & (~PORT_PCR_MUX_MASK)) | PORT_PCR_MUX(1);
	
	/*PTD4, nACOK, micro USB plug-in indication, input, pull up, low when 5v micro-usb plugged in and high
	 * when plugged out*/
	GPIOD_PDDR &= ~GPIO_PDDR_PDD(0x10);
	PORTD_PCR4 = (PORTD_PCR4 & (~PORT_PCR_MUX_MASK)) | PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;	
		
    /*PTD5, init gpio_on/off */
	GPIOD_PDDR &= ~GPIO_PDDR_PDD(1<<5);
	GPIOD_PDOR |= GPIO_PDOR_PDO(1<<5);
	PORTD_PCR5 = (PORTD_PCR5 & (~PORT_PCR_MUX_MASK)) | PORT_PCR_MUX(1);	
			
	/*PTD6, nCHG,charge state pin, input, pull up. low during charging and high when done*/
	GPIOD_PDDR &= ~GPIO_PDDR_PDD(0x40);
	PORTD_PCR6 = (PORTD_PCR6 & (~PORT_PCR_MUX_MASK)) | PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;	

	/*PTD7,sleep pin, output, low,*/
	GPIOD_PDDR |= GPIO_PDDR_PDD(0x80);
	GPIOD_PDOR &= ~GPIO_PDOR_PDO(0x80);
	PORTD_PCR7 = (PORTD_PCR7 & (~PORT_PCR_MUX_MASK)) | PORT_PCR_MUX(1);
	
	/*PTE0, mode select pin, output, low*/
	GPIOE_PDDR |= GPIO_PDDR_PDD(0x01);
	GPIOE_PDOR &= ~GPIO_PDOR_PDO(0x01);
	PORTE_PCR0 |= PORT_PCR_MUX(1);
	
	/*PTE30, Init DAC OUT */
    PORTE_PCR30 = (uint32_t)((PORTE_PCR30 & (uint32_t)~(uint32_t)(
				  PORT_PCR_ISF_MASK |
				  PORT_PCR_MUX(0x03)
				 )) | (uint32_t)(
				  PORT_PCR_MUX(0x00)
				 ));  
}

void PORTCD_IRQHandler()
{

}

 
