/* 
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#if !defined(__FSL_DSPI_TYPES_H__)
#define __FSL_DSPI_TYPES_H__

#include "fsl_platform_status.h"
#include "device/fsl_device_registers.h"
#include <stdint.h>
#include <stdbool.h>

/*! 
 * @addtogroup dspi_types
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Error codes for the DSPI driver.*/
enum _dspi_errors
{
    kStatus_DSPI_SlaveTxUnderrun = 1,        /*!< DSPI Slave TX Underrun error.*/
    kStatus_DSPI_SlaveRxOverrun,             /*!< DSPI Slave RX Overrun error.*/
    kStatus_DSPI_Timeout,                    /*!< DSPI transfer timed out.*/
    kStatus_DSPI_Busy,                       /*!< DSPI instance is already busy performing a 
                                                 transfer.*/
    kStatus_DSPI_NoTransferInProgress,       /*!< Attempt to abort a transfer when no transfer 
                                                  was in progress.*/
    kStatus_DSPI_InvalidBitCount,             /*!< bits-per-frame value not valid*/
    kStatus_DSPI_InvalidInstanceNumber      /*!< DSPI instance number does not match current count*/
};

/*! @brief DSPI master or slave configuration.*/
typedef enum _dspi_master_slave_mode {
    kDspiMaster = 1,     /*!< DSPI peripheral operates in master mode.*/
    kDspiSlave = 0       /*!< DSPI peripheral operates in slave mode.*/
} dspi_master_slave_mode_t;

/*! @brief DSPI clock polarity configuration for a given CTAR.*/
typedef enum _dspi_clock_polarity {
    kDspiClockPolarity_ActiveHigh = 0,   /*!< Active-high DSPI clock (idles low).*/
    kDspiClockPolarity_ActiveLow = 1     /*!< Active-low DSPI clock (idles high).*/
} dspi_clock_polarity_t;

/*! @brief DSPI clock phase configuration for a given CTAR.*/
typedef enum _dspi_clock_phase {
    kDspiClockPhase_FirstEdge = 0,       /*!< Data is captured on the leading edge of SCK and 
                                              changed on the following edge.*/
    kDspiClockPhase_SecondEdge = 1       /*!< Data is changed on the leading edge of SCK and 
                                              captured on the following edge.*/
} dspi_clock_phase_t;

/*! @brief DSPI data shifter direction options for a given CTAR.*/
typedef enum _dspi_shift_direction {
    kDspiMsbFirst = 0,   /*!< Data transfers start with most significant bit.*/
    kDspiLsbFirst = 1    /*!< Data transfers start with least significant bit.*/
} dspi_shift_direction_t;

/*! @brief DSPI Clock and Transfer Attributes Register (CTAR) selection.*/
typedef enum _dspi_ctar_selection {
    kDspiCtar0 = 0,   /*!< CTAR0 selection option for master or slave mode*/
    kDspiCtar1 = 1    /*!< CTAR1 selection option for master mode only*/
} dspi_ctar_selection_t;

/*! @brief DSPI Peripheral Chip Select (PCS) Polarity configuration.*/
typedef enum _dspi_pcs_polarity_config {
    kDspiPcs_ActiveHigh = 0, /*!< PCS Active High (idles low)*/
    kDspiPcs_ActiveLow  = 1 /*!< PCS Active Low (idles high)*/
} dspi_pcs_polarity_config_t;

/*! @brief DSPI Peripheral Chip Select (PCS) configuration (which PCS to configure).*/
typedef enum _dspi_which_pcs_config {
    kDspiPcs0 = 1 << 0, /*!< PCS[0] */
    kDspiPcs1 = 1 << 1, /*!< PCS[1] */
    kDspiPcs2 = 1 << 2, /*!< PCS[2] */
    kDspiPcs3 = 1 << 3, /*!< PCS[3] */
    kDspiPcs4 = 1 << 4, /*!< PCS[4] */
    kDspiPcs5 = 1 << 5  /*!< PCS[5] */
} dspi_which_pcs_config_t;

/*! 
 * @brief DSPI Sample Point: Controls when the DSPI master samples SIN in Modified Transfer 
 *  Format. This field is valid only when CPHA bit in CTAR register is 0.
 */
typedef enum _dspi_master_sample_point {
    kDspiSckToSin_0Clock = 0,  /*!< 0 system clocks between SCK edge and SIN sample*/
    kDspiSckToSin_1Clock = 1,  /*!< 1 system clock between SCK edge and SIN sample*/
    kDspiSckToSin_2Clock = 2   /*!< 2 system clocks between SCK edge and SIN sample*/
} dspi_master_sample_point_t;

/*! @brief DSPI fifo selects*/
typedef enum _dspi_fifo {
    kDspiTxFifo = 0,    /*!< DSPI tx fifo.*/
    kDspiRxFifo = 1     /*!< DSPI rx fifo.*/
} dspi_fifo_t;

/*! @brief DSPI status flags and interrupt request enable*/
typedef enum _dspi_status_and_interrupt_request {
    kDspiTxComplete = BP_SPI_RSER_TCF_RE,  /*!< TCF status/interrupt enable */
    kDspiTxAndRxStatus = BP_SPI_SR_TXRXS,  /*!< TXRXS status only, no interrupt*/
    kDspiEndOfQueue = BP_SPI_RSER_EOQF_RE, /*!< EOQF status/interrupt enable*/
    kDspiTxFifoUnderflow = BP_SPI_RSER_TFUF_RE, /*!< TFUF status/interrupt enable*/
    kDspiTxFifoFillRequest = BP_SPI_RSER_TFFF_RE, /*!< TFFF status/interrupt enable*/
    kDspiRxFifoOverflow = BP_SPI_RSER_RFOF_RE, /*!< RFOF status/interrupt enable*/
    kDspiRxFifoDrainRequest = BP_SPI_RSER_RFDF_RE /*!< RFDF status/interrupt enable*/

} dspi_status_and_interrupt_request_t;

/*! @brief DSPI FIFO counter or pointer defines based on bit positions*/
typedef enum _dspi_fifo_counter_pointer {
    kDspiRxFifoPointer = BP_SPI_SR_POPNXTPTR, /*!< RX fifo pointer*/
    kDspiRxFifoCounter = BP_SPI_SR_RXCTR,     /*!< RX fifo counter*/
    kDspiTxFifoPointer = BP_SPI_SR_TXNXTPTR,  /*!< TX fifo pointer*/
    kDspiTxFifoCounter = BP_SPI_SR_TXCTR      /*!< TX fifo counter*/
} dspi_fifo_counter_pointer_t;

/*! @}*/

#endif /* __FSL_DSPI_TYPES_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

