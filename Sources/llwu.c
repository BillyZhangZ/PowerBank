/*
 * llwu.c
 *
 *  Created on: Sep 5, 2014
 *      Author: B51761
 */

#include"llwu.h"
#include"platform.h"
extern uint8_t gwakeup_source;
void llwu_init(void)
{
	 SMC_PMPROT = SMC_PMPROT_ALLS_MASK | SMC_PMPROT_AVLLS_MASK;
#if 0
	  /* PTC5 */
	 LLWU_PE3 = (uint8_t)((LLWU_PE3 & (uint8_t)~(uint8_t)(
			 LLWU_PE3_WUPE10_MASK  |
			 LLWU_PE3_WUPE11_MASK  
	          )) 
			 /*PTC4(USB HOST PLUG-IN) and PTC5(BUTTON)*/
	         | (uint8_t)(LLWU_PE3_WUPE8(0x02) | LLWU_PE3_WUPE9(0x02)  )
			); 
#endif
 
	  /* LLWU_ME: WUME7=0,WUME5=0,WUME4=0,WUME1=0,WUME0=1 */
	  LLWU_ME = (uint8_t)((LLWU_ME & (uint8_t)~(uint8_t)(
	           LLWU_ME_WUME7_MASK |
	           LLWU_ME_WUME5_MASK |
	           LLWU_ME_WUME4_MASK |
	           LLWU_ME_WUME1_MASK
	          )) 
	         | (uint8_t)(LLWU_ME_WUME0_MASK)
			);
 
	  /*charger plugin*/
	  LLWU_PE4 = (uint8_t)((LLWU_PE4 & (uint8_t)~(uint8_t)(
			  LLWU_PE4_WUPE12_MASK |
			  LLWU_PE4_WUPE13_MASK |
			  LLWU_PE4_WUPE15_MASK
	          )) 
	         | (uint8_t)(LLWU_PE4_WUPE14_MASK)
			);
	    /* LLWU_FILT1: FILTF=1 device plugin*/
	    LLWU_FILT1 |= (LLWU_FILT1_FILTF_MASK | LLWU_FILT1_FILTSEL(0x08) | LLWU_FILT1_FILTE(0x02));                                   
	    /* LLWU_FILT2: FILTF=1 switch*/
	    LLWU_FILT2 |=(LLWU_FILT2_FILTF_MASK | LLWU_FILT2_FILTSEL(0x09) | LLWU_FILT2_FILTE(0x02));   

	  /* NVIC_IPR1: PRI_6=0  PMC and LLWU interupt priority, 11b low priority*/
	 NVIC_IPR1 |= (uint32_t)(NVIC_IP_PRI_6(0xff));       
	  /*NVIC_ISER Enable LLWU interrupt*/
	 NVIC_ISER |= NVIC_ISER_SETENA(0x80);   
}

void unmask_switch(void)
{
	  LLWU_FILT2 |=(LLWU_FILT2_FILTF_MASK | LLWU_FILT2_FILTSEL(0x09) | LLWU_FILT2_FILTE(0x02));
}
void mask_switch(void)
{
	  LLWU_FILT2 &= LLWU_FILT2_FILTE(0x00);
}
extern enum system_event gevent;
void LLW_IRQHandler()
{
	/*need to determine button wake-up or USB Host wake-up*/
	if(LLWU_FILT1 & LLWU_FILT1_FILTF_MASK)
	{
		//USB host wake-up
		LLWU_FILT1 |= LLWU_FILT1_FILTF_MASK;
		gevent = DEVICE_PLUGIN;
		return;
	}
	if(LLWU_FILT2 & LLWU_FILT2_FILTF_MASK)
	{
		//switch button
		LLWU_FILT2 |= LLWU_FILT2_FILTF_MASK;
		gevent = SWITCH_PRESSED;
		return;
	}
	if(LLWU_F3 & LLWU_F3_MWUF0_MASK)
	{
		 LPTMR0_CSR |= LPTMR_CSR_TCF_MASK;
		 LLWU_F3 |= LLWU_F3_MWUF0_MASK;
		 gevent = LOW_POWER_TIMER;
		return;
	}
	if(LLWU_F2 & LLWU_F2_WUF14_MASK)
	{
		 
		LLWU_F2 |= LLWU_F2_WUF14_MASK;
		gevent = CHARGER_PLUGIN;
		return;
	}	
}
void enter_lls(void)
{
    /* LLWU_F2: WUF15=1,WUF14=1,WUF13=1,WUF12=1,WUF11=1,WUF10=1,WUF9=1,WUF8=1 */
    LLWU_F2 = LLWU_F2_WUF15_MASK |
              LLWU_F2_WUF14_MASK |
              LLWU_F2_WUF13_MASK |
              LLWU_F2_WUF12_MASK |
              LLWU_F2_WUF11_MASK |
              LLWU_F2_WUF10_MASK |
              LLWU_F2_WUF9_MASK |
              LLWU_F2_WUF8_MASK;    

    /* LLWU_FILT1: FILTF=1 */
    LLWU_FILT1 |= (LLWU_FILT1_FILTF_MASK);                                   
    /* LLWU_FILT2: FILTF=1 */
    LLWU_FILT2 |=(LLWU_FILT2_FILTF_MASK);   

    /* SCB_SCR: SLEEPONEXIT=0 */
    SCB_SCR &= (uint32_t)~(uint32_t)(SCB_SCR_SLEEPONEXIT_MASK); 

    /* SCB_SCR: SLEEPDEEP=1 */
    SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK;                                   
    /* SMC_PMCTRL: STOPM=3 LLS mode*/
    SMC_PMCTRL = (uint8_t)((SMC_PMCTRL & (uint8_t)~(uint8_t)(
                  SMC_PMCTRL_STOPM(0x07)
                 )) | (uint8_t)(
                  SMC_PMCTRL_STOPM(0x03)
                 ));              

    (void)(SMC_PMCTRL == 0U);        /* Dummy read of SMC_PMCTRL to ensure the register is written before enterring low power mode */
    asm("wfi");
 
}
