/*
 * lptimer.c
 *
 *  Created on: Sep 5, 2014
 *      Author: B51761
 */
#include"lptimer.h"

void lptimer_init(uint16_t msec)
{
	 SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;                                   
	  /* LPTMR0_CSR: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,TCF=1,TIE=0,TPS=0,TPP=0,TFC=0,TMS=0,TEN=0 */
	  LPTMR0_CSR = (LPTMR_CSR_TCF_MASK | LPTMR_CSR_TPS(0x00)); /* Clear control register */
	  /* LPTMR0_CMR: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COMPARE=0x0CCC */
	  //LPTMR0_CMR = LPTMR_CMR_COMPARE(0x0CCC); /* Set up compare register */
	  /*the interupt period from 2s*/
	  LPTMR0_CMR = LPTMR_CMR_COMPARE(msec);
	  /* LPTMR0_PSR: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,PRESCALE=0,PBYP=1,PCS=0 */
	  LPTMR0_PSR = LPTMR_PSR_PRESCALE(0x00) |
	               LPTMR_PSR_PBYP_MASK |
	               LPTMR_PSR_PCS(0x01);    /*Source LPO 1kHZ*//* Set up prescaler register */
	  
	  /* NVIC_IPR7: PRI_28=0x80 */
	  NVIC_IPR7 = (uint32_t)((NVIC_IPR7 & (uint32_t)~(uint32_t)(
	               NVIC_IP_PRI_28(0x7F)
	              )) | (uint32_t)(
	               NVIC_IP_PRI_28(0x80)
	              ));            

	  /* NVIC_ISER: SETENA|=0x10000000 */
	  NVIC_ISER |= NVIC_ISER_SETENA(0x10000000);   


	  /* LPTMR0_CSR: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,TCF=0,TIE=1,TPS=0,TPP=0,TFC=0,TMS=0,TEN=1 */
	  LPTMR0_CSR = (LPTMR_CSR_TIE_MASK | LPTMR_CSR_TPS(0x00) | LPTMR_CSR_TEN_MASK); /* Set up control register */
}
#include"led.h"

void LPTimer_IRQHandler()
{
	 LPTMR0_CSR |= LPTMR_CSR_TCF_MASK;

}
