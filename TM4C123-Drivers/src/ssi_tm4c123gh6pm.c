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
    if( (p_ssi_x == SSI0) && !(p_sys_clock->RCGCSSI & 0x01UL) )
    {
        p_sys_clock->RCGCSSI |= (0x01UL);
    }
    else if( (p_ssi_x == SSI1) && !( p_sys_clock->RCGCSSI & (1 << 1) ) )
    {
        p_sys_clock->RCGCSSI |= (1 << 1);
    }
    else if( (p_ssi_x == SSI2) && !( p_sys_clock->RCGCSSI & (1 << 2) ) )
    {
        p_sys_clock->RCGCSSI |= (1 << 2);
    }
    else if( (p_ssi_x == SSI3) && !( p_sys_clock->RCGCSSI & (1 << 3) ) )
    {
        p_sys_clock->RCGCSSI |= (1 << 3);
    }

}


static SSI_PERIPH_T* hf_ssi_pin_config(ssi_handle_t *p_ssi_handle)
{

    gpio_handle_t ssi_pins, ssi_clk, ssi_fss, ssi_tx, ssi_rx;

    GPIO_PORT_T *ssi_clk_port = p_ssi_handle->ssi_periph.clock_port;

    ssi_pins.pin_config.pin_mode           = GPIO_DEN_ENABLE;
    ssi_pins.pin_config.alternate_function = GPIO_AFSEL_ENABLE;

    if( ssi_clk_port == GPIOA )
    {
        if(p_ssi_handle->p_ssi_x != SSI0)
        {
            return (SSI_PERIPH_T *)0;
        }

        /* @brief Select SSI0 pin pack */
        ssi_pins.p_gpio_x                = GPIOA;
        ssi_pins.pin_config.port_control = PCTL_AF2;
        gpio_init(&ssi_pins);

        ssi_clk.pin_config.pin_number = 2;
        gpio_init(&ssi_clk);

        ssi_fss.pin_config.pin_number = 3;
        gpio_init(&ssi_fss);

        ssi_fss.pin_config.pin_number = 4;
        gpio_init(&ssi_rx);

        ssi_fss.pin_config.pin_number = 5;
        gpio_init(&ssi_tx);

        return SSI0;
    }
    else if( ssi_clk_port == GPIOD )
    {
        /* @brief Select SSI1 or SSI3 pin pack */

        ssi_pins.p_gpio_x                = GPIOD;
        ssi_pins.pin_config.port_control = PCTL_AF2;
        gpio_init(&ssi_pins);

        ssi_clk.pin_config.pin_number = 0;
        gpio_init(&ssi_clk);

        ssi_fss.pin_config.pin_number = 1;
        gpio_init(&ssi_fss);

        ssi_fss.pin_config.pin_number = 2;
        gpio_init(&ssi_rx);

        ssi_fss.pin_config.pin_number = 3;
        gpio_init(&ssi_tx);

        if(p_ssi_handle->p_ssi_x == SSI1)
        {
            return SSI1;
        }
        else if(p_ssi_handle->p_ssi_x == SSI3)
        {

            return SSI3;
        }

    }
    else if( ssi_clk_port == GPIOF )
    {
        /* @brief Select SSI1 pin pack */
        ssi_pins.p_gpio_x                = GPIOF;
        ssi_pins.pin_config.port_control = PCTL_AF2;
        gpio_init(&ssi_pins);

        ssi_clk.pin_config.pin_number = 2;
        gpio_init(&ssi_clk);

        ssi_fss.pin_config.pin_number = 3;
        gpio_init(&ssi_fss);

        ssi_fss.pin_config.pin_number = 0;
        gpio_init(&ssi_rx);

        ssi_fss.pin_config.pin_number = 1;
        gpio_init(&ssi_tx);

        return SSI1;
    }
    else if( ssi_clk_port == GPIOB )
    {
        /* @brief Select SSI1 pin pack */
        ssi_pins.p_gpio_x                = GPIOB;
        ssi_pins.pin_config.port_control = PCTL_AF2;
        gpio_init(&ssi_pins);

        ssi_clk.pin_config.pin_number = 4;
        gpio_init(&ssi_clk);

        ssi_fss.pin_config.pin_number = 5;
        gpio_init(&ssi_fss);

        ssi_fss.pin_config.pin_number = 6;
        gpio_init(&ssi_rx);

        ssi_fss.pin_config.pin_number = 7;
        gpio_init(&ssi_tx);

        return SSI2;
    }


    return (SSI_PERIPH_T *)0;
}



/*
 * @brief   Initializes SSI.
 * @param   *p_ssi_handle : pointer to the SSI Handle structure (ssi_handle_t).
 * @retval  None.
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


    return 0;
}







