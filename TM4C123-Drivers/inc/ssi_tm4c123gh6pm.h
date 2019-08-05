/**
 ******************************************************************************
 * @file    ssi_tm4c123gh6pm.h, file name will change
 * @author  Aditya Mall,
 * @brief   TM4C123GH6PM Device Peripheral Access Layer Header File.
 *
 *  This file contains:
 *  TODO add details
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 Aditya Mall </center></h2>
 *
 * TODO Add license, add your name as you make changes
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


#ifndef SSI_TM4C123GH6PM_H_
#define SSI_TM4C123GH6PM_H_


/*
 * @brief Standard Headers and board specific header file
 */
#include "main_tm4c123gh6pm.h"
#include <stdint.h>


/******************************************************************************/
/*                                                                            */
/*                  Macros for SSI Peripheral Initialization                  */
/*                                                                            */
/******************************************************************************/


/*
 * @brief SSI Control 0 (CR0) Configuration Macros
 */
#define SSI_CR0_SPH_FIRST          ((uint8_t)0x00UL)  /*!< Serial Clock Phase (SPH), Data is captured on the First clock edge transition       */
#define SSI_CR0_SPH_SECOND         ((uint8_t)0x01UL)  /*!< Serial Clock Phase (SPH), Data is captured on the Second clock edge transition      */
#define SSI_CR0_SPO_LOW            ((uint8_t)0x00UL)  /*!< Serial Clock Polarity (SPO), A steady state Low value is placed on the SSInClk pin  */
#define SSI_CR0_SPO_HIGH           ((uint8_t)0x01UL)  /*!< Serial Clock Polarity (SPO), A steady state High value is placed on the SSInClk pin */
#define SSI_CR0_FRF_FREESCALE      ((uint8_t)0x00UL)  /*!< SSI Frame Format Select (FRF), Freescale SPI Frame Format                           */
#define SSI_CR0_FRF_TI             ((uint8_t)0x01UL)  /*!< SSI Frame Format Select (FRF), Texas Instruments Synchronous Serial Frame Format    */
#define SSI_CR0_FRF_MICROWIRE      ((uint8_t)0x02UL)  /*!< SSI Frame Format Select (FRF), MICROWIRE Frame Format                               */
#define SSI_CR0_DSS_4BIT           ((uint8_t)0x03UL)  /*!< SSI Data Size Select (DSS), 4-bit data                                              */
#define SSI_CR0_DSS_5BIT           ((uint8_t)0x04UL)  /*!< SSI Data Size Select (DSS), 5-bit data                                              */
#define SSI_CR0_DSS_6BIT           ((uint8_t)0x05UL)  /*!< SSI Data Size Select (DSS), 6-bit data                                              */
#define SSI_CR0_DSS_7BIT           ((uint8_t)0x06UL)  /*!< SSI Data Size Select (DSS), 7-bit data                                              */
#define SSI_CR0_DSS_8BIT           ((uint8_t)0x07UL)  /*!< SSI Data Size Select (DSS), 8-bit data                                              */
#define SSI_CR0_DSS_9BIT           ((uint8_t)0x08UL)  /*!< SSI Data Size Select (DSS), 9-bit data                                              */
#define SSI_CR0_DSS_10BIT          ((uint8_t)0x09UL)  /*!< SSI Data Size Select (DSS), 10-bit data                                             */
#define SSI_CR0_DSS_11BIT          ((uint8_t)0x0AUL)  /*!< SSI Data Size Select (DSS), 11-bit data                                             */
#define SSI_CR0_DSS_12BIT          ((uint8_t)0x0BUL)  /*!< SSI Data Size Select (DSS), 12-bit data                                             */
#define SSI_CR0_DSS_13BIT          ((uint8_t)0x0CUL)  /*!< SSI Data Size Select (DSS), 13-bit data                                             */
#define SSI_CR0_DSS_14BIT          ((uint8_t)0x0DUL)  /*!< SSI Data Size Select (DSS), 14-bit data                                             */
#define SSI_CR0_DSS_15BIT          ((uint8_t)0x0EUL)  /*!< SSI Data Size Select (DSS), 15-bit data                                             */
#define SSI_CR0_DSS_16BIT          ((uint8_t)0x0FUL)  /*!< SSI Data Size Select (DSS), 16-bit data                                             */



/*
 * @brief SSI Control 1 (CR1) Configuration Macros
 */
#define SSI_CR1_EOT_HALF           ((uint8_t)0x00UL) /*!< SSI End of Transmission (EOT), Transmit FIFO is half full or less         */
#define SSI_CR1_EOT_ENABLE         ((uint8_t)0x01UL) /*!< SSI End of Transmission (EOT), The End of Transmit interrupt mode enabled */
#define SSI_CR1_MS_MASTER          ((uint8_t)0x00UL) /*!< SSI Master/Slave Select (MS), The SSI is configured as a master           */
#define SSI_CR1_MS_SLAVE           ((uint8_t)0x01UL) /*!< SSI Master/Slave Select (MS), The SSI is configured as a slave            */
#define SSI_CR1_SSE_DISABLE        ((uint8_t)0x00UL) /*!< SSI Synchronous Serial Port Enable (SSE), SSI operation is disabled.      */
#define SSI_CR1_SSE_ENABLE         ((uint8_t)0x01UL) /*!< SSI Synchronous Serial Port Enable (SSE), SSI operation is enabled.       */
#define SSI_CR1_LBM_DISABLE        ((uint8_t)0x00UL) /*!< SSI Loop-back Mode (LBM), Normal serial port operation enabled.           */
#define SSI_CR1_LBM_ENABLE         ((uint8_t)0x01UL) /*!< SSI Loop-back Mode (LBM), Loop-back mode enabled.                         */



/*
 * @brief SSI Status Register (SR) FLAGS
 */
#define SSI_SR_BSY_IDLE_FLAG       ((uint8_t)0x00UL) /*!< SSI Busy Bit (BSY), The SSI is idle                                    */
#define SSI_SR_BSY_BUSY_FLAG       ((uint8_t)0x01UL) /*!< SSI Busy Bit (BSY), The SSI is currently TX/RXing or TX FIFO not empty */
#define SSI_SR_RFF_NOT_FULL_FLAG   ((uint8_t)0x00UL) /*!< SSI Receive FIFO Full (RFF), The receive FIFO is not full              */
#define SSI_SR_RFF_FULL_FLAG       ((uint8_t)0x01UL) /*!< SSI Receive FIFO Full (RFF), The receive FIFO is full                  */
#define SSI_SR_RNE_EMPTY_FLAG      ((uint8_t)0x00UL) /*!< SSI Receive FIFO Not Empty (RNE), The receive FIFO is not full         */
#define SSI_SR_RNE_NOT_EMPTY_FLAG  ((uint8_t)0x01UL) /*!< SSI Receive FIFO Not Empty (RNE), The receive FIFO is not full         */
#define SSI_SR_TNF_FULL_FLAG       ((uint8_t)0x00UL) /*!< SSI Transmit FIFO Not Full (TNF), The transmit FIFO is full            */
#define SSI_SR_TNF_NOT_FULL_FLAG   ((uint8_t)0x01UL) /*!< SSI Transmit FIFO Not Full (TNF), The transmit FIFO is  not full       */
#define SSI_SR_TFE_NOT_EMPTY_FLAG  ((uint8_t)0x00UL) /*!< SSI Transmit FIFO Empty (TFE), The transmit FIFO is not empty          */
#define SSI_SR_TFE_EMPTY_FLAG      ((uint8_t)0x01UL) /*!< SSI Transmit FIFO Empty (TFE), The transmit FIFO is empty              */



/*
 * @brief SSI Clock Configuration (SSICC)
 */
#define SSI_CC_CS_SYSCLK           ((uint8_t)0x00UL) /*!< SSI Baud Clock Source (CS), System clock                          */
#define SSI_CC_CS_PIOSC            ((uint8_t)0x05UL) /*!< SSI Baud Clock Source (CS), precision internal oscillator (PIOSC) */



/******************************************************************************/
/*                                                                            */
/*                  Data Structures for SSI Pin Initialization                */
/*                                                                            */
/******************************************************************************/


/*
 * @brief GPIO Configuration Structure
 */
typedef struct
{
    uint8_t device_mode;      /*!< SSI Device Mode Select, Master/Slave */
    uint8_t clock_source;     /*!< SSI Clock Source Select              */
    uint8_t clock_phase;      /*!< SSI Clock Phase Select               */
    uint8_t clock_polarity;   /*!< SSI Clock Polarity Select            */
    uint8_t clock_speed;      /*!< SSI Clock Speed Select               */
    uint8_t frame_format;     /*!< SSI Frame Format Select              */
    uint8_t data_size;        /*!< SSI Data Size Select                 */

    GPIO_PORT_T *clock_port;  /*!< SSI Clock Pin Port initialize       */

} ssi_config_t;



/*
 * @brief GPIO Handle Structure
 */
typedef struct
{
    SSI_PERIPH_T *p_ssi_x;     /*!< Pointer to the SSI Peripheral Address */
    ssi_config_t  ssi_periph;  /*!< SSI Peripheral structure variable     */

} ssi_handle_t;



/******************************************************************************/
/*                                                                            */
/*                SSI Peripheral Driver Functions Prototypes                  */
/*                                                                            */
/******************************************************************************/


int8_t ssi_init(ssi_handle_t *p_ssi_handle);


void ssi_send_data(ssi_handle_t *p_ssi_handle, uint8_t *p_txbuffer, uint32_t len);


int8_t ssi_receive_data(ssi_handle_t *p_ssi_handle, uint8_t *p_rxbuffer, uint32_t len);



#endif
