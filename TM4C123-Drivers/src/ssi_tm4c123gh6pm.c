/**
 ******************************************************************************
 * @file    ssi_tm4c123gh6pm.c, file name will change
 * @author  Aditya Mall,
 * @brief   TM4C123GH6PM Device Peripheral Access Layer Source File.
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



/*
 * @brief Standard Headers and board specific header file
 */
#include "gpio_tm4c123gh6pm.h"
#include "ssi_tm4c123gh6pm.h"
#include <string.h>


/******************************************************************************/
/*                                                                            */
/*                      SSI Peripheral Driver Functions                       */
/*                                                                            */
/******************************************************************************/



/*
 * @brief   helper function to enable SPI Peripheral Clock
 * @param   *p_ssi_x   : pointer to the SPI peripheral structure (SSI_PERIPH_T).
 * @retval  None.
 */
static void hf_ssi_clock_enable(SSI_PERIPH_T *p_ssi_x)
{

    SYSCTL_T *p_sys_clock = SYSCTL;  /*!< Pointer to System Control Peripheral Structure */

    /* @brief Enable Clock for SSI peripheral with re initialization check  */
    if( (p_ssi_x == SSI0) && !(p_sys_clock->RCGCSSI & SSI0_ENABLE) )
    {
        p_sys_clock->RCGCSSI |= SSI0_ENABLE;

        /* @brief check if SSI peripheral is ready */
        while( !(p_sys_clock->PRSSI & SSI0_ENABLE) );
    }
    else if( (p_ssi_x == SSI1) && !(p_sys_clock->RCGCSSI & SSI1_ENABLE) )
    {
        p_sys_clock->RCGCSSI |= SSI1_ENABLE;

        /* @brief check if SSI peripheral is ready */
        while( !(p_sys_clock->PRSSI & SSI1_ENABLE) );
    }
    else if( (p_ssi_x == SSI2) && !(p_sys_clock->RCGCSSI & SSI2_ENABLE) )
    {
        p_sys_clock->RCGCSSI |= SSI2_ENABLE;

        /* @brief check if SSI peripheral is ready */
        while( !(p_sys_clock->PRSSI & SSI2_ENABLE) );
    }
    else if( (p_ssi_x == SSI3) && !( p_sys_clock->RCGCSSI & SSI3_ENABLE) )
    {
        p_sys_clock->RCGCSSI |= SSI3_ENABLE;

        /* @brief check if SSI peripheral is ready */
        while( !(p_sys_clock->PRSSI & SSI3_ENABLE) );
    }

}


/*
 * @brief   helper function to enable SPI Peripheral Clock
 * @param   *p_ssi_x     : *p_ssi_handle : pointer to the SSI Handle structure (ssi_handle_t).
 * @retval  SSI_PERIPH_T : pointer to the SPI peripheral structure (SSI_PERIPH_T).
 */
static SSI_PERIPH_T* hf_ssi_pin_config(ssi_handle_t *p_ssi_handle)
{

    gpio_handle_t ssi_pins;

    GPIO_PORT_T *ssi_pins_port = p_ssi_handle->ssi_periph.clock_port;

    ssi_pins.pin_config.pin_mode           = GPIO_DEN_ENABLE;
    ssi_pins.pin_config.alternate_function = GPIO_AFSEL_ENABLE;

    if( ssi_pins_port == GPIOA )
    {
        if(p_ssi_handle->p_ssi_x != SSI0)
        {
            return (SSI_PERIPH_T *)0;
        }

        /* @brief Select SSI0 pin pack */
        ssi_pins.p_gpio_x                = GPIOA;
        ssi_pins.pin_config.port_control = PCTL_AF2;
        gpio_init(&ssi_pins);

        /* @brief SSI clock (clk) */
        if(p_ssi_handle->ssi_periph.clock_polarity == SSI_CR0_SPO_HIGH)
        {
            ssi_pins.pin_config.pullupdown = GPIO_PUR_ENABLE;
        }
        ssi_pins.pin_config.pin_number = 2;
        gpio_init(&ssi_pins);

        /* @brief SSI Slave Select (fss) */
        ssi_pins.pin_config.pin_number = 3;
        ssi_pins.pin_config.pullupdown = GPIO_PUR_ENABLE;
        gpio_init(&ssi_pins);

        /* @brief SSI Receive MISO (rx) */
        ssi_pins.pin_config.pin_number = 4;
        gpio_init(&ssi_pins);

        /* @brief SSI Receive MOSI (tx) */
        ssi_pins.pin_config.pin_number = 5;
        gpio_init(&ssi_pins);

        return SSI0;
    }
    else if( ssi_pins_port == GPIOD )
    {
        if(p_ssi_handle->p_ssi_x == SSI1 || p_ssi_handle->p_ssi_x == SSI3 )
        {

            /* @brief Select SSI1 or SSI3 pin pack */
            ssi_pins.p_gpio_x                = GPIOD;
            ssi_pins.pin_config.port_control = PCTL_AF2;
            gpio_init(&ssi_pins);

            /* @brief SSI clock (clk) */
            if(p_ssi_handle->ssi_periph.clock_polarity == SSI_CR0_SPO_HIGH)
            {
                ssi_pins.pin_config.pullupdown = GPIO_PUR_ENABLE;
            }
            ssi_pins.pin_config.pin_number = 0;
            gpio_init(&ssi_pins);

            /* @brief SSI Slave Select (fss) */
            ssi_pins.pin_config.pin_number = 1;
            ssi_pins.pin_config.pullupdown = GPIO_PUR_ENABLE;
            gpio_init(&ssi_pins);

            /* @brief SSI Receive MISO (rx) */
            ssi_pins.pin_config.pin_number = 2;
            gpio_init(&ssi_pins);

            /* @brief SSI Receive MOSI (tx) */
            ssi_pins.pin_config.pin_number = 3;
            gpio_init(&ssi_pins);
        }
        else
        {
            return (SSI_PERIPH_T *)0;
        }
        if(p_ssi_handle->p_ssi_x == SSI1)
        {
            return SSI1;
        }
        else if(p_ssi_handle->p_ssi_x == SSI3)
        {

            return SSI3;
        }

    }
    else if( ssi_pins_port == GPIOF )
    {
        if(p_ssi_handle->p_ssi_x != SSI1)
        {
            return (SSI_PERIPH_T *)0;
        }

        /* @brief Select SSI1 pin pack */
        ssi_pins.p_gpio_x                = GPIOF;
        ssi_pins.pin_config.port_control = PCTL_AF2;
        gpio_init(&ssi_pins);

        /* @brief SSI clock (clk) */
        if(p_ssi_handle->ssi_periph.clock_polarity == SSI_CR0_SPO_HIGH)
        {
            ssi_pins.pin_config.pullupdown = GPIO_PUR_ENABLE;
        }
        ssi_pins.pin_config.pin_number = 2;
        gpio_init(&ssi_pins);

        /* @brief SSI Slave Select (fss) */
        ssi_pins.pin_config.pin_number = 3;
        ssi_pins.pin_config.pullupdown = GPIO_PUR_ENABLE;
        gpio_init(&ssi_pins);

        /* @brief SSI Receive MISO (rx) */
        ssi_pins.pin_config.pin_number = 0;
        gpio_init(&ssi_pins);

        /* @brief SSI Receive MOSI (tx) */
        ssi_pins.pin_config.pin_number = 1;
        gpio_init(&ssi_pins);

        return SSI1;
    }
    else if( ssi_pins_port == GPIOB )
    {
        if(p_ssi_handle->p_ssi_x != SSI2)
        {
            return (SSI_PERIPH_T *)0;
        }

        /* @brief Select SSI1 pin pack */
        ssi_pins.p_gpio_x                = GPIOB;
        ssi_pins.pin_config.port_control = PCTL_AF2;
        gpio_init(&ssi_pins);

        /* @brief SSI clock (clk) */
        if(p_ssi_handle->ssi_periph.clock_polarity == SSI_CR0_SPO_HIGH)
        {
            ssi_pins.pin_config.pullupdown = GPIO_PUR_ENABLE;
        }
        ssi_pins.pin_config.pin_number = 4;
        gpio_init(&ssi_pins);

        /* @brief SSI Slave Select (fss) */
        ssi_pins.pin_config.pin_number = 5;
        ssi_pins.pin_config.pullupdown = GPIO_PUR_ENABLE;
        gpio_init(&ssi_pins);

        /* @brief SSI Receive MISO (rx) */
        ssi_pins.pin_config.pin_number = 6;
        gpio_init(&ssi_pins);

        /* @brief SSI Receive MOSI (tx) */
        ssi_pins.pin_config.pin_number = 7;
        gpio_init(&ssi_pins);

        return SSI2;
    }


    return (SSI_PERIPH_T *)0;
}



/*
 * @brief  Initializes SSI.
 * @param  *p_ssi_handle : pointer to the SSI Handle structure (ssi_handle_t).
 * @retval 0: Success, -1: Fail
 */
int8_t ssi_init(ssi_handle_t *p_ssi_handle)
{

    SSI_PERIPH_T *p_ssi;

    /* @brief Configure GPIO pins for SSI peripheral, return address of SSI peripheral */
    p_ssi = hf_ssi_pin_config(p_ssi_handle);
    if(p_ssi == NULL)
    {
        return -1;
    }

    /* @brief enable SSI clock */
    hf_ssi_clock_enable(p_ssi);


    /* @brief disable SSI peripheral prior configuration (safe programming) */
    p_ssi_handle->p_ssi_x->CR1 &= ~(1 << 1);

    /* @brief configure device mode */
    switch(p_ssi_handle->ssi_periph.device_mode)
    {

    case SSI_CR1_MS_MASTER:
        p_ssi_handle->p_ssi_x->CR1 &= ~(1 << 2);
        break;

    case SSI_CR1_MS_SLAVE:
        p_ssi_handle->p_ssi_x->CR1 |= (1 << 2);
        break;

    default:
        //p_ssi_handle->p_ssi_x->CR1 &= ~(1 << 2);
        break;

    }

    /* @brief Set SSI Clock Source */
    switch(p_ssi_handle->ssi_periph.clock_source)
    {

    case SSI_CC_CS_SYSCLK:
        p_ssi_handle->p_ssi_x->CC &= ~(0x00UL);
        break;

    case SSI_CC_CS_PIOSC_CLK:
        p_ssi_handle->p_ssi_x->CC |=  0x05UL;
        break;

    default:
        break;

    }

    /* @brief Configure clock phase */
    switch(p_ssi_handle->ssi_periph.clock_phase)
    {

    case SSI_CR0_SPH_FIRST:
        p_ssi_handle->p_ssi_x->CR0 &= ~(1 << 7);
        break;

    case SSI_CR0_SPH_SECOND:
        p_ssi_handle->p_ssi_x->CR0 |= (1 << 7);
        break;

    default:
        break;

    }


    /* @brief Configure clock polarity */
    switch(p_ssi_handle->ssi_periph.clock_polarity)
    {

    case SSI_CR0_SPO_LOW:
        p_ssi_handle->p_ssi_x->CR0 &= ~(1 << 6);
        break;

    case SSI_CR0_SPO_HIGH:
        p_ssi_handle->p_ssi_x->CR0 |= (1 << 6);
        break;

    default:
        break;

    }


    /* @brief Configure frame format */
    switch(p_ssi_handle->ssi_periph.frame_format)
    {

    case SSI_CR0_FRF_FREESCALE:
        p_ssi_handle->p_ssi_x->CR0 &= ~(0x03UL << 4);
        break;

    case SSI_CR0_FRF_TEXAS:
        p_ssi_handle->p_ssi_x->CR0 |= (SSI_CR0_FRF_TEXAS << 4);
        break;

    case SSI_CR0_FRF_MICROWIRE:
        p_ssi_handle->p_ssi_x->CR0 |= (SSI_CR0_FRF_MICROWIRE << 4);
        break;

    default:
        break;

    }

    /* @brief Set clock speed */
    p_ssi_handle->p_ssi_x->CPSR = p_ssi_handle->ssi_periph.clock_speed;

    /* @brief Configure Data Size */
    p_ssi_handle->p_ssi_x->CR0 |= p_ssi_handle->ssi_periph.data_size;


    /* @brief Enable SSI peripheral  */
    p_ssi_handle->p_ssi_x->CR1 |= (1 << 1);


    return 0;
}



/*
 * @brief Send Data Through MISO (Tx)
 * @param *p_ssi_x : pointer to the SSI peripheral structure.
 * @param ssi_data : data to be send
 */
void ssi_put_data(SSI_PERIPH_T *p_ssi_x, uint16_t ssi_data)
{
    /* @brief Hold till Transmit FIFO is not empty */
    while((p_ssi_x->SR & SSI_SR_TFE_EMPTY_FLAG) == 0){};

    if( (p_ssi_x->CR0 & 0x0F) > SSI_CR0_DSS_8BIT )
    {
        p_ssi_x->DR = ssi_data;
    }
    else
    {
        p_ssi_x->DR = (uint8_t)ssi_data;
    }

}



/*
 * @brief Receive Through MISO (rx)
 * @param *p_ssi_x : pointer to the SSI peripheral structure.
 * @param ssi_data : data to be send
 */
uint16_t ssi_get_data(SSI_PERIPH_T *p_ssi_x)
{
    uint16_t data = 0;

    /* @brief Hold till Receive FIFO is Empty */
    while( (p_ssi_x->SR & SSI_SR_RNE_NOT_EMPTY_FLAG) == 0 ) {};

    data = p_ssi_x->DR;

    if( (p_ssi_x->CR0 & 0x0F) > SSI_CR0_DSS_8BIT )
    {
        return data;
    }
    else
    {
        return (uint8_t)data;
    }

    return 0;
}



/*
 * @brief Send Data Through MISO (Tx)
 * @param *p_ssi_x    : pointer to the SSI peripheral structure.
 * @param *p_txbuffer :
 * @param data_length : data to be send
 */
void ssi_send_stream(SSI_PERIPH_T *p_ssi_x, uint16_t *p_txbuffer, uint32_t data_length)
{
    uint32_t ssi_data_count;

    while(data_length)
    {
        ssi_put_data(p_ssi_x, p_txbuffer[ssi_data_count]);

        ssi_data_count++;
        data_length--;
    }

}




