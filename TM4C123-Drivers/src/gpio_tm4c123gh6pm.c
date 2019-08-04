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


/**
 * @brief   helper function to enable GPIO Peripheral Clock
 * @param   *p_gpio_x   : pointer to the GPIO port structure (GPIO_PORT_T).
 * @retval  None.
 */
static inline void GPIO_clockEnable(GPIO_PORT_T *p_gpio_x)
{

    GPIO_PORT_T *portAddress = p_gpio_x;  /*!< Pointer to the Address of the GPIO port            */
    SYSCTL_T    *sysClock    = SYSCTL;  /*!< Pointer to the Address of System Control Structure */


    /* @brief Enable Clock for GPIO ports with re initialization check  */

    if(portAddress == GPIOA && !(sysClock->RCGCGPIO & 0x01UL))
        sysClock->RCGCGPIO |= (0x01UL);

    else if(portAddress == GPIOB && !( sysClock->RCGCGPIO & (1 << 1) ))
        sysClock->RCGCGPIO |= (1 << 1);

    else if(portAddress == GPIOC && !( sysClock->RCGCGPIO & (1 << 2) ))
        sysClock->RCGCGPIO |= (1 << 2);

    else if(portAddress == GPIOD && !( sysClock->RCGCGPIO & (1 << 3) ))
        sysClock->RCGCGPIO |= (1 << 3);

    else if(portAddress == GPIOE && !( sysClock->RCGCGPIO & (1 << 4) ))
        sysClock->RCGCGPIO |= (1 << 4);

    else if(portAddress == GPIOF && !( sysClock->RCGCGPIO & (1 << 5) ))
        sysClock->RCGCGPIO |= (1 << 5);

}



/**
 * @brief   Intializes GPIO pin.
 * @param   *pGPIOHandle : pointer to the GPIO Handle structure (gpio_handle_t).
 * @retval  None.
 */
void GPIO_Init(gpio_handle_t *pGPIOHandle)
{

    uint8_t  driveSelect  = 0;  /*!< Variable for selecting drive current values                 */
    uint8_t  pupdSelect   = 0;  /*!< Variable for selecting Pull-Up or Pull-Down configurations  */
    uint8_t  pinMode      = 0;  /*!< Variable for selecting Digital or Analog configurations     */


    GPIO_clockEnable(pGPIOHandle->p_gpio_x);

    /*
     * @brief Configure GPIO Pin Direction
     * @note  All configurations are done by simply left shifting value set in the gpiopinConfig structure by PIN NUMBER (Set by User)
     * @note  See Value of Switch cases in header file gpio_tm4c123gh6pm.h
     */
    pGPIOHandle->p_gpio_x->DIR |= (pGPIOHandle->gpioPinConfig.direction << pGPIOHandle->gpioPinConfig.pin_number);


    /* @brief Configure GPIO Pin Open Drain Select */
    pGPIOHandle->p_gpio_x->ODR |= (pGPIOHandle->gpioPinConfig.opendrain << pGPIOHandle->gpioPinConfig.pin_number);



    /* @brief Configure GPIO Pin Drive Select */
    driveSelect = pGPIOHandle->gpioPinConfig.drive;

    switch(driveSelect)
    {

    case GPIO_DR2R:
        pGPIOHandle->p_gpio_x->DR2R |= (1 << pGPIOHandle->gpioPinConfig.pin_number);
        break;

    case GPIO_DR4R:
        pGPIOHandle->p_gpio_x->DR4R |= (1 << pGPIOHandle->gpioPinConfig.pin_number);
        break;

    case GPIO_DR8R:
        pGPIOHandle->p_gpio_x->DR8R |= (1 << pGPIOHandle->gpioPinConfig.pin_number);
        break;

    default:
        break;

    }


    /* @brief Configure GPIO Pin Pull-Up and Pull-Down Select */
    pupdSelect = pGPIOHandle->gpioPinConfig.pullupdown;

    switch(pupdSelect)
    {

    case GPIO_NOPUPD:
        break;

    case GPIO_PUR_ENABLE:
        pGPIOHandle->p_gpio_x->PUR |= (1 << pGPIOHandle->gpioPinConfig.pin_number);
        break;

    case GPIO_PDR_ENABLE:
        pGPIOHandle->p_gpio_x->PDR |= (1 << pGPIOHandle->gpioPinConfig.pin_number);
        break;

    default:
        break;

    }


    /* @brief GPIO Digital or Analog Register Mode Select  */
    pinMode = pGPIOHandle->gpioPinConfig.pin_mode;

    switch(pinMode)
    {

    case GPIO_DEN_ENABLE:
        pGPIOHandle->p_gpio_x->AMSEL &= ~(1 << pGPIOHandle->gpioPinConfig.pin_number);
        pGPIOHandle->p_gpio_x->DEN   &= ~(1 << pGPIOHandle->gpioPinConfig.pin_number);
        pGPIOHandle->p_gpio_x->DEN   |= (1 << pGPIOHandle->gpioPinConfig.pin_number);
        break;

    case GPIO_AMSEL_ENABLE:
        pGPIOHandle->p_gpio_x->DEN   &= ~(1 << pGPIOHandle->gpioPinConfig.pin_number);
        pGPIOHandle->p_gpio_x->AMSEL &= ~(1 << pGPIOHandle->gpioPinConfig.pin_number);
        pGPIOHandle->p_gpio_x->AMSEL |= (1 << pGPIOHandle->gpioPinConfig.pin_number);
        break;

    default:
        break;

    }


    /*
     * @brief GPIO Port Control Register and GPIO Pin Alternate Function
     * @note  Values of PCTLVAL, see header file gpio_tm4c123gh6pm.h
     */
    if(pGPIOHandle->gpioPinConfig.alternate_function == GPIO_AFSEL_ENABLE)
    {

        pGPIOHandle->p_gpio_x->AFSEL |= (pGPIOHandle->gpioPinConfig.alternate_function << pGPIOHandle->gpioPinConfig.pin_number);

        pGPIOHandle->p_gpio_x->PCTL  |= (pGPIOHandle->gpioPinConfig.pctl_value << (4 * pGPIOHandle->gpioPinConfig.pin_number));

    }

}



/*
 * @brief   Deinitialize GPIO pin.
 * @param   *pGPIOHandle : pointer to the GPIO Handle structure (gpio_handle_t).
 * @retval  None.
 */
void GPIO_DeInit(gpio_handle_t *pGPIOHandle)
{



}



/*
 * @brief   Read from GPIO pin (Blocking function)
 * @param   *p_gpio_x   : pointer to the GPIO port structure (GPIO_PORT_T).
 * @param   pinNumber : GPIO Pin Number.
 * @retval  uint8_t   : Return value from the pin.
 */
uint8_t GPIO_ReadFromPin(GPIO_PORT_T *p_gpio_x, uint8_t pinNumber)
{

    uint8_t retVal = 0;                     /*!< Variable to store the return value of the pin                       */

    retVal = (p_gpio_x->DATA >> pinNumber);   /*!< Shift value from the pin to LSB and mask it with 0xFF (Masking bit) */

    return retVal;
}



/*
 * @brief   Write to GPIO pin
 * @param   *p_gpio_x   : pointer to the GPIO port structure (GPIO_PORT_T).
 * @param   pinNumber : GPIO Pin Number
 * @bool    value     : Value to be written, 1 or 0.
 * @retval  None.
 */
void GPIO_WriteToPin(GPIO_PORT_T *p_gpio_x, uint8_t pinNumber, bool value)
{

    if (value == ENABLE)
        p_gpio_x->DATA |= (1 << pinNumber);   /*!< Set bit of the corresponding Pin Number   */

    else if (value == DISABLE)
        p_gpio_x->DATA &= ~(1 << pinNumber);  /*!< Clear bit of the corresponding Pin Number */

}



/*
 * @brief   Read from GPIO port (Blocking function)
 * @param   *p_gpio_x  : pointer to the GPIO port structure (GPIO_PORT_T).
 * @retval  uint8_t  : Data from the port
 */
uint8_t GPIO_ReadFromPort(GPIO_PORT_T *p_gpio_x)
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
void GPIO_WriteToPort(GPIO_PORT_T *p_gpio_x, uint8_t value)
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


