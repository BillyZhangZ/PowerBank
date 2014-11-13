/*
** ###################################################################
**     Processors:          MKL25Z128FM4
**                          MKL25Z128FT4
**                          MKL25Z128LH4
**                          MKL25Z128VLK4
**
**     Compilers:           ARM Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    KL25P80M48SF0RM, Rev.3, Sep 2012
**     Version:             rev. 1.3, 2012-10-04
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright: 2012 Freescale, Inc. All Rights Reserved.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2012-06-13)
**         Initial version.
**     - rev. 1.1 (2012-06-21)
**         Update according to reference manual rev. 1.
**     - rev. 1.2 (2012-08-01)
**         Device type UARTLP changed to UART0.
**     - rev. 1.3 (2012-10-04)
**         Update according to reference manual rev. 3.
**
** ###################################################################
*/

/**
 * @file MKL25Z4
 * @version 1.3
 * @date 2012-10-04
 * @brief Device specific configuration file for MKL25Z4 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "device/fsl_device_registers.h"
#define BOOT_KL26_CUSTOM_BOARD  

#define DISABLE_WDOG    1
#ifdef BOOT_KL26_CUSTOM_BOARD 
#define CLOCK_SETUP     0
#else
#define CLOCK_SETUP     1
#endif
/* Predefined clock setups
   0 ... Multipurpose Clock Generator (MCG) in FLL Engaged Internal (FEI) mode
         Reference clock source for MCG module is the slow internal clock source 32.768kHz
         Core clock = 41.94MHz, BusClock = 13.98MHz
   1 ... Multipurpose Clock Generator (MCG) in PLL Engaged External (PEE) mode
         Reference clock source for MCG module is an external crystal 8MHz
         Core clock = 48MHz, BusClock = 24MHz
   2 ... Multipurpose Clock Generator (MCG) in Bypassed Low Power External (BLPE) mode
         Core clock/Bus clock derived directly from an external crystal 8MHz with no multiplication
         Core clock = 8MHz, BusClock = 8MHz
*/

/*----------------------------------------------------------------------------
  Define clock source values
 *----------------------------------------------------------------------------*/
#if (CLOCK_SETUP == 0)
    #define CPU_XTAL_CLK_HZ                 8000000u /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             32768u   /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             4000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            41943040u /* Default System clock value */
#elif (CLOCK_SETUP == 1)
    #define CPU_XTAL_CLK_HZ                 8000000u /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             32768u   /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             4000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            48000000u /* Default System clock value */
#elif (CLOCK_SETUP == 2)
    #define CPU_XTAL_CLK_HZ                 8000000u /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             32768u   /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             4000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            8000000u /* Default System clock value */
#endif /* (CLOCK_SETUP == 2) */


/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

#ifdef BOOT_KL26_CUSTOM_BOARD
#define BOOT_DELAY 300*5200

#define LOOP_CNT_1MS_24MHZ 2600
#define LOOP_CNT_1MS_48MHZ LOOP_CNT_1MS_24MHZ*2
#define LOOP_CNT_1MS_72MHZ LOOP_CNT_1MS_24MHZ*3
#define LOOP_CNT_1MS_96MHZ LOOP_CNT_1MS_24MHZ*4
void delay_busy(uint16_t msec)
{
	int volatile loop_cnt = 0;
	switch((MCG_C4 & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT)
	{
		/*24MHZ core*/
		case 0x00:
			loop_cnt = LOOP_CNT_1MS_24MHZ*msec;
			while(--loop_cnt);
			break;
		/*48MHZ core*/			
		case 0x01:
			loop_cnt = LOOP_CNT_1MS_48MHZ*msec;
			while(--loop_cnt);
			break;
		/*72MHZ core*/
		case 0x02:
			loop_cnt = LOOP_CNT_1MS_72MHZ*msec;
			while(--loop_cnt);
			break;
		/*96MHZ core*/
		case 0x03:
			loop_cnt = LOOP_CNT_1MS_96MHZ*msec;
			while(--loop_cnt);
			break;
		default:
			break;
	}
}
/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */
 
int flag = 0;
 
typedef void(*app_t)(void);

#endif


void SystemInit (void) {
#ifdef BOOT_KL26_CUSTOM_BOARD
   int i = 0;
   app_t app = (app_t)*((unsigned int *)(0x8000+4));
#endif
#if (DISABLE_WDOG)
  /* Disable the WDOG module */
  /* SIM_COPC: COPT=0,COPCLKS=0,COPW=0 */
  SIM->COPC = (uint32_t)0x00u;
#endif /* (DISABLE_WDOG) */


#ifdef BOOT_KL26_CUSTOM_BOARD

    /*PTC5*/
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    GPIOC_PDDR &= ~GPIO_PDDR_PDD(1<<5);
    PORTC_PCR5 |= PORT_PCR_MUX(1)| PORT_PCR_PE_MASK  | PORT_PCR_PS_MASK;
   
    while(++i < BOOT_DELAY)
    {
      if((GPIOC_PDIR &(1<<5)) == 0)  continue;
      else    break;
    }
    
    PORTC_PCR5 = PORT_PCR_MUX(1)| PORT_PCR_PE_MASK   | PORT_PCR_PS_MASK;
    SIM_SCGC5 &= ~SIM_SCGC5_PORTC_MASK;
   
    //never come back
    if(i != BOOT_DELAY) 
    {
       __asm("cpsie i");
      app();
    }

#endif

    
#if (CLOCK_SETUP == 0)
  /* SIM->CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=2,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = (uint32_t)0x00020000UL; /* Update system prescalers */
  /* Switch to FEI Mode */
  /* MCG->C1: CLKS=0,FRDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (uint8_t)0x06U;
  /* MCG_C2: LOCRE0=0,??=0,RANGE0=0,HGO0=0,EREFS0=0,LP=0,IRCS=0 */
  MCG->C2 = (uint8_t)0x00U;
#define FLL_48M
#ifdef FLL_48M
  /* MCG->C4: DMX32=1,DRST_DRS=0 */
  MCG->C4 = (uint8_t)((MCG->C4 | (uint8_t)0x80U) |(uint8_t)0x20U);
#else
    /* MCG->C4: DMX32=0,DRST_DRS=1 */
  MCG->C4 = (uint8_t)((MCG->C4 & (uint8_t)~(uint8_t)0xC0U) | (uint8_t)0x20U);
#endif
  /* OSC0->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = (uint8_t)0x80U;
  /* MCG->C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0 */
  MCG->C5 = (uint8_t)0x00U;
  /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
  MCG->C6 = (uint8_t)0x00U;
  while((MCG->S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
  }
  while((MCG->S & 0x0CU) != 0x00U) {    /* Wait until output of the FLL is selected */
  }
#elif (CLOCK_SETUP == 1)
  /* SIM->SCGC5: PORTA=1 */
  SIM->SCGC5 |= (uint32_t)0x0200UL;     /* Enable clock gate for ports to enable pin routing */
  /* SIM->CLKDIV1: OUTDIV1=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = (uint32_t)0x10010000UL; /* Update system prescalers */
  /* PORTA->PCR18: ISF=0,MUX=0 */
  PORTA->PCR[18] &= (uint32_t)~0x01000700UL;
  /* PORTA->PCR19: ISF=0,MUX=0 */
  PORTA->PCR[19] &= (uint32_t)~0x01000700UL;
  
  /* Switch to FBE Mode */
  /* OSC0->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=1,SC4P=0,SC8P=0,SC16P=1 */
  OSC0->CR = (uint8_t)0x89U;
  /* MCG->C2: LOCRE0=0,??=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0 */
  MCG->C2 = (uint8_t)0x24U;
  /* MCG->C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (uint8_t)0x9AU;
  /* MCG->C4: DMX32=0,DRST_DRS=0 */
  MCG->C4 &= (uint8_t)~(uint8_t)0xE0U;
  /* MCG->C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=1 */
  MCG->C5 = (uint8_t)0x01U;
  /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
  MCG->C6 = (uint8_t)0x00U;
  while((MCG->S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
  }
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  /* Switch to PBE Mode */
  /* MCG->C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=0 */
  MCG->C6 = (uint8_t)0x40U;
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  while((MCG->S & MCG_S_LOCK0_MASK) == 0x00U) { /* Wait until locked */
  }
  /* Switch to PEE Mode */
  /* MCG->C1: CLKS=0,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (uint8_t)0x1AU;
  while((MCG->S & 0x0CU) != 0x0CU) {    /* Wait until output of the PLL is selected */
  }
  
#elif (CLOCK_SETUP == 2)
  /* SIM->SCGC5: PORTA=1 */
  SIM->SCGC5 |= (uint32_t)0x0200UL;     /* Enable clock gate for ports to enable pin routing */
  /* SIM->CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = (uint32_t)0x00000000UL; /* Update system prescalers */
  /* PORTA->PCR18: ISF=0,MUX=0 */
  PORTA->PCR[18] &= (uint32_t)~0x01000700UL;
  /* PORTA->PCR19: ISF=0,MUX=0 */
  PORTA->PCR[19] &= (uint32_t)~0x01000700UL;
  /* Switch to FBE Mode */
  /* OSC0->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=1,SC4P=0,SC8P=0,SC16P=1 */
  OSC0->CR = (uint8_t)0x89U;
  /* MCG->C2: LOCRE0=0,??=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0 */
  MCG->C2 = (uint8_t)0x24U;
  /* MCG->C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (uint8_t)0x9AU;
  /* MCG->C4: DMX32=0,DRST_DRS=0 */
  MCG->C4 &= (uint8_t)~(uint8_t)0xE0U;
  /* MCG->C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0 */
  MCG->C5 = (uint8_t)0x00U;
  /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
  MCG->C6 = (uint8_t)0x00U;
  while((MCG->S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
  }
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  /* Switch to BLPE Mode */
  /* MCG->C2: LOCRE0=0,??=0,RANGE0=2,HGO0=0,EREFS0=1,LP=1,IRCS=0 */
  MCG->C2 = (uint8_t)0x26U;
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
#endif /* (CLOCK_SETUP == 2) */
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {
  uint32_t MCGOUTClock;                                                        /* Variable to store output clock frequency of the MCG module */
  uint8_t Divider;

  if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x0u) {
    /* Output of FLL or PLL is selected */
    if ((MCG->C6 & MCG_C6_PLLS_MASK) == 0x0u) {
      /* FLL is selected */
      if ((MCG->C1 & MCG_C1_IREFS_MASK) == 0x0u) {
        /* External reference clock is selected */
        MCGOUTClock = CPU_XTAL_CLK_HZ;                                       /* System oscillator drives MCG clock */
        Divider = (uint8_t)(1u << ((MCG->C1 & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT));
        MCGOUTClock = (MCGOUTClock / Divider);  /* Calculate the divided FLL reference clock */
        if ((MCG->C2 & MCG_C2_RANGE0_MASK) != 0x0u) {
          MCGOUTClock /= 32u;                                                  /* If high range is enabled, additional 32 divider is active */
        } /* ((MCG->C2 & MCG_C2_RANGE0_MASK) != 0x0u) */
      } else { /* (!((MCG->C1 & MCG_C1_IREFS_MASK) == 0x0u)) */
        MCGOUTClock = CPU_INT_SLOW_CLK_HZ;                                     /* The slow internal reference clock is selected */
      } /* (!((MCG->C1 & MCG_C1_IREFS_MASK) == 0x0u)) */
      /* Select correct multiplier to calculate the MCG output clock  */
      switch (MCG->C4 & (MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK)) {
        case 0x0u:
          MCGOUTClock *= 640u;
          break;
        case 0x20u:
          MCGOUTClock *= 1280u;
          break;
        case 0x40u:
          MCGOUTClock *= 1920u;
          break;
        case 0x60u:
          MCGOUTClock *= 2560u;
          break;
        case 0x80u:
          MCGOUTClock *= 732u;
          break;
        case 0xA0u:
          MCGOUTClock *= 1464u;
          break;
        case 0xC0u:
          MCGOUTClock *= 2197u;
          break;
        case 0xE0u:
          MCGOUTClock *= 2929u;
          break;
        default:
          break;
      }
    } else { /* (!((MCG->C6 & MCG_C6_PLLS_MASK) == 0x0u)) */
      /* PLL is selected */
      Divider = (1u + (MCG->C5 & MCG_C5_PRDIV0_MASK));
      MCGOUTClock = (uint32_t)(CPU_XTAL_CLK_HZ / Divider);                     /* Calculate the PLL reference clock */
      Divider = ((MCG->C6 & MCG_C6_VDIV0_MASK) + 24u);
      MCGOUTClock *= Divider;                       /* Calculate the MCG output clock */
    } /* (!((MCG->C6 & MCG_C6_PLLS_MASK) == 0x0u)) */
  } else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x40u) {
    /* Internal reference clock is selected */
    if ((MCG->C2 & MCG_C2_IRCS_MASK) == 0x0u) {
      MCGOUTClock = CPU_INT_SLOW_CLK_HZ;                                       /* Slow internal reference clock selected */
    } else { /* (!((MCG->C2 & MCG_C2_IRCS_MASK) == 0x0u)) */
      MCGOUTClock = CPU_INT_FAST_CLK_HZ / (1 << ((MCG->SC & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT));  /* Fast internal reference clock selected */
    } /* (!((MCG->C2 & MCG_C2_IRCS_MASK) == 0x0u)) */
  } else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80u) {
    /* External reference clock is selected */
    MCGOUTClock = CPU_XTAL_CLK_HZ;                                           /* System oscillator drives MCG clock */
  } else { /* (!((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80u)) */
    /* Reserved value */
    return;
  } /* (!((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80u)) */
  SystemCoreClock = (MCGOUTClock / (1u + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT)));
}

uint32_t GetSystemMCGPLLClock (void)
{
#if (CLOCK_SETUP == 0)
    // 0 ... Multipurpose Clock Generator (MCG) in FLL Engaged Internal (FEI) mode
    //       Reference clock source for MCG module is the slow internal clock source 32.768kHz
    //       Core clock = 41.94MHz, BusClock = 13.98MHz
    // FLL engaged internal (FEI) is the default mode of operation and is entered when all the following
    //     condtions occur:
    //     � C1[CLKS] bits are written to 00
    //     � C1[IREFS] bit is written to 1
    //     � C6[PLLS] bit is written to 0
    //     In FEI mode, MCGOUTCLK is derived from the FLL clock (DCOCLK) that is controlled by the 32
    //     kHz Internal Reference Clock (IRC). The FLL loop will lock the DCO frequency to the FLL factor, as
    //     selected by C4[DRST_DRS] and C4[DMX32] bits, times the internal reference frequency. See the
    //     C4[DMX32] bit description for more details. In FEI mode, the PLL is disabled in a low-power state
    //     unless C5[PLLCLKEN0] is set.
    // MCG->C1 = (uint8_t)0x06U; IREFS = 1 IRCLKEN = 1, slow interal clock selected
    // MCG->C2 = (uint8_t)0x00U;
    // MCG->C4 = (uint8_t)((MCG->C4 & (uint8_t)~(uint8_t)0xC0U) | (uint8_t)0x20U); DRST_DRS = 1, DMX32 = 0 FLL Factor = 1280
    // MCG->C5 = (uint8_t)0x00U; PLL Disabled
    // MCG->C6 = (uint8_t)0x00U; FLL Selected

    return 0;

#elif (CLOCK_SETUP == 1)
    // 1 ... Multipurpose Clock Generator (MCG) in PLL Engaged External (PEE) mode
    //       Reference clock source for MCG module is an external crystal 8MHz
    //       Core clock = 48MHz, BusClock = 24MHz
    // PLL Engaged External (PEE) mode is entered when all the following conditions occur:
    //     � C1[CLKS] bits are written to 00
    //     � C1[IREFS] bit is written to 0
    //     � C6[PLLS] bit is written to 1
    //     In PEE mode, the MCGOUTCLK is derived from the PLL clock, which is controlled by the external
    //     reference clock. The PLL clock frequency locks to a multiplication factor, as specified by C6[VDIV0],
    //               times the external reference frequency, as specified by C5[PRDIV0]. The PLL's programmable
    //                   reference divider must be configured to produce a valid PLL reference clock. The FLL is disabled in
    //                   a low-power state.
    // MCG->C2: LOCRE0=0,??=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0
    // MCG->C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0
    // MCG->C4: DMX32=0,DRST_DRS=0
    // MCG->C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=1
    // MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0
    // MCG->C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=0
    // MCG->C1: CLKS=0,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0
    // C6[VDIV0] = 24x
    // C5[PRDIV0] = /2
    // MCG PLL output then /2 again

    return (((CPU_XTAL_CLK_HZ * 24) / 2 ) / 2);

#elif (CLOCK_SETUP == 2)
    // 2 ... Multipurpose Clock Generator (MCG) in Bypassed Low Power External (BLPE) mode
    //       Core clock/Bus clock derived directly from an external crystal 8MHz with no multiplication
    //       Core clock = 8MHz, BusClock = 8MHz
    // Bypassed Low Power External (BLPE) mode is entered when all the following conditions occur:
    //     � C1[CLKS] bits are written to 10
    //     � C1[IREFS] bit is written to 0
    //     � C2[LP] bit is written to 1
    //     In BLPE mode, MCGOUTCLK is derived from the external reference clock. The FLL is disabled and
    //     PLL is disabled even if the C5[PLLCLKEN0] is set to 1.

    return 0;

#endif // (CLOCK_SETUP == 2)
}

uint32_t GetSystemMCGFLLClock (void)
{
#if (CLOCK_SETUP == 0)
    // 0 ... Multipurpose Clock Generator (MCG) in FLL Engaged Internal (FEI) mode
    //       Reference clock source for MCG module is the slow internal clock source 32.768kHz
    //       Core clock = 41.94MHz, BusClock = 13.98MHz
    // MCG->C1 = (uint8_t)0x06U; IREFS = 1 IRCLKEN = 1, slow interal clock selected
    // MCG->C2 = (uint8_t)0x00U;
    // MCG->C4 = (uint8_t)((MCG->C4 & (uint8_t)~(uint8_t)0xC0U) | (uint8_t)0x20U); DRST_DRS = 1, DMX32 = 0 FLL Factor = 1280
    // MCG->C5 = (uint8_t)0x00U; PLL Disabled
    // MCG->C6 = (uint8_t)0x00U; FLL Selected

    return (CPU_INT_SLOW_CLK_HZ * 1280);

#elif (CLOCK_SETUP == 1)
    // FLL Clock disabled

    return 0;

#elif (CLOCK_SETUP == 2)
    // 2 ... Multipurpose Clock Generator (MCG) in Bypassed Low Power External (BLPE) mode
    //       Core clock/Bus clock derived directly from an external crystal 8MHz with no multiplication
    //       Core clock = 8MHz, BusClock = 8MHz
    // Bypassed Low Power External (BLPE) mode is entered when all the following conditions occur:
    //     � C1[CLKS] bits are written to 10
    //     � C1[IREFS] bit is written to 0
    //     � C2[LP] bit is written to 1
    //     In BLPE mode, MCGOUTCLK is derived from the external reference clock. The FLL is disabled and
    //     PLL is disabled even if the C5[PLLCLKEN0] is set to 1.

    return 0;

#endif // (CLOCK_SETUP == 2)
}

