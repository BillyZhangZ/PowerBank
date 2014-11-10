/*
 * File:		a16dc.c
 * Purpose:		Simple Driver or API for AdC16/PGA
 * This file contains the following two functions:
 *  1. ADC_Cal: calibrates the ADC
 *  2. ADC_Config_Alt: Simply configures an ADC per a structure
 *
 * Typical usage: Fill the structure with the ADC and PGA register contents 
 * needed for the ADC usage.

 * a) Call the ADC_Config_Alt function to configure an ADC, (ADC0 or ADC1)
 * b) Call the ADC_Cal function to calibrate that ADC
 * c) Call the ADC_Config_Alt function again to restore desired configuation
 *    after a calibration
 *
 *
 */

 
#include "adc16.h"
// function prototypes:

static uint8_t ADC_Cal(ADC_MemMapPtr);

static void ADC_Config_Alt(ADC_MemMapPtr, tADC_ConfigPtr);

static void ADC_Read_Cal(ADC_MemMapPtr, tADC_Cal_Blk *);




tADC_Config Master_Adc_Config;  
tADC_Cal_Blk CalibrationStore[2];

// Prototypes
void adc_init(void);

/**   adc_init
 * \brief    initialize and calibrate ADC0 module
 * \brief    48MHz IPBus clock; ADC clock = 48M/2/8 = 3MHz
 * \brief    Total conversion time: 56N+4ADCK
 * \brief    given sampling rate Fs = 6.4K, 156us/sample,  156*3= 468 ADCK
 * \author   FSL
 * \param    none
 * \return   none
 * \warning  assumes 48MHz IPBus clock
 */ 
void adc_init(void)
{
   SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
   PMC_REGSC |= PMC_REGSC_BGBE_MASK;
#if 0
#ifdef CMSIS
   NVIC_EnableIRQ(ADC0_IRQn);
#else
   enable_irq(INT_ADC0 - 16);
#endif
#endif
  /* 48MHz IPBus clock
   * ADC clock = 48M/2/8 = 3MHz
   * Total conversion time: 56N+4ADCK
   * Given sampling rate Fs = 6.4K, 156us/sample,  156*3= 468 ADCK
   * the maximum h/w average number = 8
   * use h/w average number = 4
   * Total conversion time: 56*4+4 = 228 ADC clocks,76us
   * There are 468-228 = 240 ADC clocks (ie. 80us) free for post processing
   */
 

 // Initialize ADC0
  // Do calibration first with 32 h/w averages
  Master_Adc_Config.CONFIG1  = ADLPC_NORMAL | ADC_CFG1_ADIV(ADIV_8) | ADLSMP_LONG | ADC_CFG1_MODE(MODE_16)
                              | ADC_CFG1_ADICLK(ADICLK_BUS_2);  
  Master_Adc_Config.CONFIG2  = MUXSEL_ADCA | ADACKEN_ENABLED | ADHSC_HISPEED | ADC_CFG2_ADLSTS(ADLSTS_20) ;
  Master_Adc_Config.COMPARE1 = 0x1234u ; 
  Master_Adc_Config.COMPARE2 = 0x5678u ;
  Master_Adc_Config.STATUS2  = ADTRG_SW | ACFE_DISABLED | ACFGT_GREATER | ACREN_ENABLED | DMAEN_DISABLED | ADC_SC2_REFSEL(REFSEL_RES);
  Master_Adc_Config.STATUS3  = CAL_OFF | ADCO_SINGLE | AVGE_ENABLED | ADC_SC3_AVGS(AVGS_32);
 // Master_Adc_Config.PGA      = PGAEN_DISABLED | PGACHP_NOCHOP | PGALP_NORMAL | ADC_PGA_PGAG(PGAG_64);
 
  Master_Adc_Config.STATUS1A = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(8);       
  Master_Adc_Config.STATUS1B = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(8);  
  
  ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // config ADC
  ADC_Cal(ADC0_BASE_PTR);                    // do the calibration
 
  ADC_Read_Cal(ADC0_BASE_PTR,&CalibrationStore[1]);   // store the cal
  
  // Now do normal ADC configuration with 4 h/w averages and h/w trigger from PDB
  Master_Adc_Config.CONFIG1  = ADLPC_NORMAL | ADC_CFG1_ADIV(ADIV_8) | ADLSMP_LONG | ADC_CFG1_MODE(MODE_12)
                              | ADC_CFG1_ADICLK(ADICLK_BUS_2);  
  Master_Adc_Config.CONFIG2  = MUXSEL_ADCB | ADACKEN_ENABLED | ADHSC_HISPEED | ADC_CFG2_ADLSTS(ADLSTS_20) ;
  Master_Adc_Config.COMPARE1 = 0x1234u ; 
  Master_Adc_Config.COMPARE2 = 0x5678u ;
  Master_Adc_Config.STATUS2  = !ADTRG_HW | ACFE_DISABLED | ACFGT_GREATER | ACREN_DISABLED | DMAEN_DISABLED | ADC_SC2_REFSEL(REFSEL_RES);
  Master_Adc_Config.STATUS3  = CAL_OFF | ADCO_SINGLE  /*AVGE_ENABLED*/ | ADC_SC3_AVGS(AVGS_4);
//  Master_Adc_Config.PGA      = PGAEN_DISABLED | PGACHP_NOCHOP | PGALP_NORMAL | ADC_PGA_PGAG(PGAG_64);

  Master_Adc_Config.STATUS1A = !AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(8);                                                               
  Master_Adc_Config.STATUS1B = !AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(8);    

  ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // config the ADC again to default conditions    
  (volatile void)ADC0_RA;
}

/**
 * @}
 */ /* end of group ADC_Register_Masks */



/**   adc_start_conversion, result is stored in the ADC interrupt on a global variable
 * \brief    start adc conversion of channel n
 * \author   FSL
 * \param    channel
 * \return   none
 * \warning  assumes 48MHz IPBus clock
 */ 
void adc_start_conversion(uint8_t channel)
{
    ADC0_SC1A = !AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(channel);     
}


/*
  simple read of ADC
*/
/*channel 8 is ADC_BATT, channel 15 is ADC_CURR*/
uint16_t adc_read(uint8_t channel)
{
   ADC0_SC1A = !AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(channel) ;     // start conversion
   while((ADC0_SC1A & ADC_SC1_COCO_MASK)== 0){};
   return ADC0_RA;
}

uint16_t adc_read_save(uint8_t channel)
{
		uint16_t sample_value = adc_read(channel);
		 
		update_value_history(channel, sample_value);
		return get_value_history(channel);		 	
}
/**   adc0_isr,
 * \brief    reads the results of the Adc conversion
 * \brief    values is stored in a global variable  adc_sample0, adc_sample1
 * \author   FSL
 * \param    none
 * \return   none
 * \warning  
 */ 
#ifdef CMSIS
void ADC0_IRQHandler(void)
#else
void adc0_isrv(void)
#endif
{


  if((ADC0_SC1A & ADC_SC1_COCO_MASK)!= 0){
    (void)ADC0_RA;
  }
 
  if((ADC0_SC1B & ADC_SC1_COCO_MASK)!= 0){
    (void)ADC0_RB;
  }
     
}

/******************************************************************************
Function 1. Name	AUTO CAL ROUTINE   

Parameters		ADC module pointer points to adc0 or adc1 register map 
                         base address.
Returns			Zero indicates success.
Notes         		Calibrates the ADC16. Required to meet specifications 
                        after reset and before a conversion is initiated.
******************************************************************************/
static uint8_t ADC_Cal(ADC_MemMapPtr adcmap)
{

  unsigned short cal_var;
  
  ADC_SC2_REG(adcmap) &=  ~ADC_SC2_ADTRG_MASK ; // Enable Software Conversion Trigger for Calibration Process    - ADC0_SC2 = ADC0_SC2 | ADC_SC2_ADTRGW(0);   
  ADC_SC3_REG(adcmap) &= ( ~ADC_SC3_ADCO_MASK & ~ADC_SC3_AVGS_MASK ); // set single conversion, clear avgs bitfield for next writing
  ADC_SC3_REG(adcmap) |= ( ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(AVGS_32) );  // Turn averaging ON and set at max value ( 32 )
  
  
  ADC_SC3_REG(adcmap) |= ADC_SC3_CAL_MASK ;      // Start CAL
  while ( (ADC_SC1_REG(adcmap,A) & ADC_SC1_COCO_MASK ) == COCO_NOT ); // Wait calibration end
  	
  if ((ADC_SC3_REG(adcmap)& ADC_SC3_CALF_MASK) == CALF_FAIL )
  {  
   return(1);    // Check for Calibration fail error and return 
  }
  // Calculate plus-side calibration
  cal_var = 0x00;
  
  cal_var =  ADC_CLP0_REG(adcmap); 
  cal_var += ADC_CLP1_REG(adcmap);
  cal_var += ADC_CLP2_REG(adcmap);
  cal_var += ADC_CLP3_REG(adcmap);
  cal_var += ADC_CLP4_REG(adcmap);
  cal_var += ADC_CLPS_REG(adcmap);

  cal_var = cal_var/2;
  cal_var |= 0x8000; // Set MSB

  ADC_PG_REG(adcmap) = ADC_PG_PG(cal_var);
 

  // Calculate minus-side calibration
  cal_var = 0x00;

  cal_var =  ADC_CLM0_REG(adcmap); 
  cal_var += ADC_CLM1_REG(adcmap);
  cal_var += ADC_CLM2_REG(adcmap);
  cal_var += ADC_CLM3_REG(adcmap);
  cal_var += ADC_CLM4_REG(adcmap);
  cal_var += ADC_CLMS_REG(adcmap);

  cal_var = cal_var/2;

  cal_var |= 0x8000; // Set MSB

  ADC_MG_REG(adcmap) = ADC_MG_MG(cal_var); 
  
  ADC_SC3_REG(adcmap) &= ~ADC_SC3_CAL_MASK ; /* Clear CAL bit */

  return(0);
}




/******************************************************************************
Function 2 Name 	ADC_Config_Alt 
Parameters		the register values to be set in the adc in a structure
Returns			NONE
Notes         		Configures ADC0 or ADC1 depending on adcmap
                        Prior to calling this function populate the structure
                        elements with the desired ADC configuration.
******************************************************************************/


static void ADC_Config_Alt(ADC_MemMapPtr adcmap, tADC_ConfigPtr ADC_CfgPtr)
{
 ADC_CFG1_REG(adcmap) = ADC_CfgPtr->CONFIG1;
 ADC_CFG2_REG(adcmap) = ADC_CfgPtr->CONFIG2;
 ADC_CV1_REG(adcmap)  = ADC_CfgPtr->COMPARE1; 
 ADC_CV2_REG(adcmap)  = ADC_CfgPtr->COMPARE2;
 ADC_SC2_REG(adcmap)  = ADC_CfgPtr->STATUS2;
 ADC_SC3_REG(adcmap)  = ADC_CfgPtr->STATUS3;
 //ADC_PGA_REG(adcmap)  = ADC_CfgPtr->PGA;  pbd
 ADC_SC1_REG(adcmap,A)= ADC_CfgPtr->STATUS1A;       
 ADC_SC1_REG(adcmap,B)= ADC_CfgPtr->STATUS1B;
}


static void ADC_Read_Cal(ADC_MemMapPtr adcmap, tADC_Cal_Blk *blk)
{
  blk->OFS  = ADC_OFS_REG(adcmap);
  blk->PG   = ADC_PG_REG(adcmap); 
  blk->MG   = ADC_MG_REG(adcmap); 
  blk->CLPD = ADC_CLPD_REG(adcmap); 
  blk->CLPS = ADC_CLPS_REG(adcmap); 
  blk->CLP4 = ADC_CLP4_REG(adcmap);
  blk->CLP3 = ADC_CLP3_REG(adcmap); 
  blk->CLP2 = ADC_CLP2_REG(adcmap); 
  blk->CLP1 = ADC_CLP1_REG(adcmap);
  blk->CLP0 = ADC_CLP0_REG(adcmap);
  blk->CLMD = ADC_CLMD_REG(adcmap); 
  blk->CLMS = ADC_CLMS_REG(adcmap); 
  blk->CLM4 = ADC_CLM4_REG(adcmap);
  blk->CLM3 = ADC_CLM3_REG(adcmap); 
  blk->CLM2 = ADC_CLM2_REG(adcmap); 
  blk->CLM1 = ADC_CLM1_REG(adcmap);
  blk->CLM0 = ADC_CLM0_REG(adcmap);
  
}


