/**
 ******************************************************************************
 * @file    gpio_tm4c123gh6pm.c, file name will change
 * @author  Aditya Mall,
 * @brief   TM4C123GH6PM Device Peripheral Access Layer Header File.
 *
 *  This file contains:
 *              - Helper functions for Driver exposed API functions
 *              - Driver exposed APIs, for GPIO
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
 * @brief Standard and Driver header files
 */
#include "gpio_tm4c123gh6pm.h"


/******************************************************************************/
/*                                                                            */
/*                       Peripheral Driver Functions                          */
/*                                                                            */
/******************************************************************************/


/*
 * @brief   helper function to enable GPIO Peripheral Clock
 * @param   *p_gpio_x   : pointer to the GPIO port structure (GPIO_PORT_T).
 * @retval  None.
 */
static void hf_gpio_clock_enable(GPIO_PORT_T *p_gpio_x)
{

    SYSCTL_T *p_sys_clock = SYSCTL;  /*!< Pointer to System Control Peripheral Structure */

    /* @brief Enable Clock for GPIO ports with re initialization check  */

    if(p_gpio_x == GPIOA && !(p_sys_clock->RCGCGPIO & 0x01UL))
    {
        p_sys_clock->RCGCGPIO |= (0x01UL);
    }
    else if(p_gpio_x == GPIOB && !( p_sys_clock->RCGCGPIO & (1 << 1) ))
    {
        p_sys_clock->RCGCGPIO |= (1 << 1);
    }
    else if(p_gpio_x == GPIOC && !( p_sys_clock->RCGCGPIO & (1 << 2) ))
    {
        p_sys_clock->RCGCGPIO |= (1 << 2);
    }
    else if(p_gpio_x == GPIOD && !( p_sys_clock->RCGCGPIO & (1 << 3) ))
    {
        p_sys_clock->RCGCGPIO |= (1 << 3);
    }
    else if(p_gpio_x == GPIOE && !( p_sys_clock->RCGCGPIO & (1 << 4) ))
    {
        p_sys_clock->RCGCGPIO |= (1 << 4);
    }
    else if( (p_gpio_x == GPIOF) && !( p_sys_clock->RCGCGPIO & (1 << 5) ) )
    {
        p_sys_clock->RCGCGPIO |= (1 << 5);
    }

}



/*
 * @brief   Initializes GPIO pin.
 * @param   *p_gpio_handle : pointer to the GPIO Handle structure (gpio_handle_t).
 * @retval  None.
 */
void gpio_init(gpio_handle_t *p_gpio_handle)
{

    uint8_t  drive_select  = 0;  /*!< Variable for selecting drive current values                 */
    uint8_t  pupd_select   = 0;  /*!< Variable for selecting Pull-Up or Pull-Down configurations  */
    uint8_t  pin_mode      = 0;  /*!< Variable for selecting Digital or Analog configurations     */

    hf_gpio_clock_enable(p_gpio_handle->p_gpio_x);

    /*
     * @brief Configure GPIO Pin Direction
     * @note  All configurations are done by simply left shifting value set in the gpiopinConfig structure by PIN NUMBER (Set by User)
     * @note  See Value of Switch cases in header file gpio_tm4c123gh6pm.h
     */
    p_gpio_handle->p_gpio_x->DIR |= (p_gpio_handle->pin_config.direction << p_gpio_handle->pin_config.pin_number);

    /* @brief Configure GPIO Pin Open Drain Select */
    p_gpio_handle->p_gpio_x->ODR |= (p_gpio_handle->pin_config.opendrain << p_gpio_handle->pin_config.pin_number);

    /* @brief Configure GPIO Pin Drive Select */
    drive_select = p_gpio_handle->pin_config.drive;

    switch(drive_select)
    {

    case GPIO_DR2R:
        p_gpio_handle->p_gpio_x->DR2R |= (1 << p_gpio_handle->pin_config.pin_number);
        break;

    case GPIO_DR4R:
        p_gpio_handle->p_gpio_x->DR4R |= (1 << p_gpio_handle->pin_config.pin_number);
        break;

    case GPIO_DR8R:
        p_gpio_handle->p_gpio_x->DR8R |= (1 << p_gpio_handle->pin_config.pin_number);
        break;

    default:
        break;

    }

    /* @brief Configure GPIO Pin Pull-Up and Pull-Down Select */
    pupd_select = p_gpio_handle->pin_config.pullupdown;

    switch(pupd_select)
    {

    case GPIO_NOPUPD:
        break;

    case GPIO_PUR_ENABLE:
        p_gpio_handle->p_gpio_x->PUR |= (1 << p_gpio_handle->pin_config.pin_number);
        break;

    case GPIO_PDR_ENABLE:
        p_gpio_handle->p_gpio_x->PDR |= (1 << p_gpio_handle->pin_config.pin_number);
        break;

    default:
        break;

    }

    /* @brief GPIO Digital or Analog Register Mode Select  */
    pin_mode = p_gpio_handle->pin_config.pin_mode;

    switch(pin_mode)
    {

    case GPIO_DEN_ENABLE:
        p_gpio_handle->p_gpio_x->AMSEL &= ~(1 << p_gpio_handle->pin_config.pin_number);
        p_gpio_handle->p_gpio_x->DEN   &= ~(1 << p_gpio_handle->pin_config.pin_number);
        p_gpio_handle->p_gpio_x->DEN   |= (1 << p_gpio_handle->pin_config.pin_number);
        break;

    case GPIO_AMSEL_ENABLE:
        p_gpio_handle->p_gpio_x->DEN   &= ~(1 << p_gpio_handle->pin_config.pin_number);
        p_gpio_handle->p_gpio_x->AMSEL &= ~(1 << p_gpio_handle->pin_config.pin_number);
        p_gpio_handle->p_gpio_x->AMSEL |= (1 << p_gpio_handle->pin_config.pin_number);
        break;

    default:
        break;

    }

    /*
     * @brief GPIO Port Control Register and GPIO Pin Alternate Function
     * @note  Values of PCTLVAL, see header file gpio_tm4c123gh6pm.h
     */
    if(p_gpio_handle->pin_config.alternate_function == GPIO_AFSEL_ENABLE)
    {

        p_gpio_handle->p_gpio_x->AFSEL |= (p_gpio_handle->pin_config.alternate_function << p_gpio_handle->pin_config.pin_number);

        p_gpio_handle->p_gpio_x->PCTL  |= (p_gpio_handle->pin_config.port_control << (4 * p_gpio_handle->pin_config.pin_number));

    }

}


/*
 * @brief   Deinitialize GPIO pin.
 * @param   *p_gpio_handle : pointer to the GPIO Handle structure (gpio_handle_t).
 * @retval  None.
 */
void gpio_deinit(gpio_handle_t *p_gpio_handle)
{



}



/*
 * @brief   Read from GPIO pin (Blocking function)
 * @param   *p_gpio_x : pointer to the GPIO port structure (GPIO_PORT_T).
 * @param   pin       : GPIO Pin Number.
 * @retval  uint8_t   : Return value from the pin.
 */
uint8_t gpio_read_from_pin(GPIO_PORT_T *p_gpio_x, uint8_t pin)
{

    uint8_t retval = 0;                /*!< Variable to store the return value of the pin                       */

    retval = (p_gpio_x->DATA >> pin);  /*!< Shift value from the pin to LSB and mask it with 0xFF (Masking bit) */

    return retval;
}



/*
 * @brief   Write to GPIO pin
 * @param   *p_gpio_x : pointer to the GPIO port structure (GPIO_PORT_T).
 * @param   pin       : GPIO Pin Number
 * @bool    value     : Value to be written, 1 or 0.
 * @retval  None.
 */
void gpio_write_to_pin(GPIO_PORT_T *p_gpio_x, uint8_t pin, bool value)
{

    if (value == ENABLE)
        p_gpio_x->DATA |= (1 << pin);   /*!< Set bit of the corresponding Pin Number   */

    else if (value == DISABLE)
        p_gpio_x->DATA &= ~(1 << pin);  /*!< Clear bit of the corresponding Pin Number */

}



/*
 * @brief   Read from GPIO port (Blocking function)
 * @param   *p_gpio_x  : pointer to the GPIO port structure (GPIO_PORT_T).
 * @retval  uint8_t  : Data from the port
 */
uint8_t gpio_read_from_port(GPIO_PORT_T *p_gpio_x)
{

    uint8_t retVal = 0;              /*!< Variable to store the return value of the port */

    retVal = (p_gpio_x->DATA) & 0xFF;  /*!< Return Masked value of the data register       */

    return retVal;
}



/*
 * @brief   Write to GPIO port
 * @param   *p_gpio_x  : pointer to the GPIO port structure (GPIO_PORT_T).
 * @param   value  : Data to be written to the port
 * @retval  None.
 */
void gpio_write_to_port(GPIO_PORT_T *p_gpio_x, uint8_t value)
{
    p_gpio_x->DATA = value;  /*!< Write value directly to the Data Register */
}



/*
 * @brief   Configure GPIO Interrupt
 * @param   IRQNumber   : Interrupt Number
 * @param   IRQPriority : Interrupt Priority
 * @param   state       : Enable or Disable Interrupt
 * @retval  None.
 */
void GPIO_InterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, bool state)
{



}




/*
 * @brief   Configure GPIO Interrupt
 * @param   pinNumber: GPIO Pin Number triggering the interrupt
 * @retval  None.
 */
void GPIO_InterruptHandling(uint8_t pinNumber)
{



}


