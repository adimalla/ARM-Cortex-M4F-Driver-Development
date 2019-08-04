/**
 ******************************************************************************
 * @file    gpio.c, file name will change
 * @author  Aditya Mall,
 * @brief   GPIO API Layer Source File.
 *
 *  TODO complete details
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
 * @brief Standard Header and driver specific header files
 */
#include "gpio_tm4c123gh6pm.h"
#include "gpio.h"
#include <stdarg.h>
#include <string.h>


/******************************************************************************/
/*                                                                            */
/*                             API Functions                                  */
/*                                                                            */
/******************************************************************************/



static void _hf_setPinDefaults(gpio_handle_t *gpioPin)
{
    gpioPin->pin_config.drive      = GPIO_DR2R;
    gpioPin->pin_config.pin_mode   = GPIO_DEN_ENABLE;
    gpioPin->pin_config.pullupdown = GPIO_NOPUPD;
    gpioPin->pin_config.opendrain  = GPIO_ODDR_DISABLE;
}



/*
 * @brief   Intializes GPIO pin.
 * @param   *port_pin : Name of the GPIO Pin
 * @param
 * @param
 * @retval  None.
 */
int8_t pinMode(char *port_pin, uint8_t pinDirection,...)
{

    va_list gpioArgs;

    gpio_handle_t gpioPin;

    uint8_t pinNumber = 0;
    uint8_t argValue  = 0;
    uint8_t numOfArgs = 3;
    uint8_t loopVar   = 0;

    uint8_t digitalSetFlag = 0;

    va_start(gpioArgs, pinDirection);


    if( (strncmp(port_pin,"pa",2 ) == 0) || (strncmp(port_pin,"PA",2 ) == 0) )
    {
        gpioPin.p_gpio_x = GPIOA;
    }
    else if( (strncmp(port_pin,"pb",2 ) == 0) || (strncmp(port_pin,"PB",2 ) == 0) )
    {
        gpioPin.p_gpio_x = GPIOB;
    }
    else if( (strncmp(port_pin,"pc",2 ) == 0) || (strncmp(port_pin,"PC",2 ) == 0) )
    {
        gpioPin.p_gpio_x = GPIOC;
    }
    else if( (strncmp(port_pin,"pd",2 ) == 0) || (strncmp(port_pin,"PD",2 ) == 0) )
    {
        gpioPin.p_gpio_x = GPIOD;
    }
    else if( (strncmp(port_pin,"pe",2 ) == 0) || (strncmp(port_pin,"PE",2 ) == 0) )
    {
        gpioPin.p_gpio_x = GPIOE;
    }
    else if( (strncmp(port_pin,"pf",2 ) == 0) || (strncmp(port_pin,"PF",2 ) == 0) )
    {
        gpioPin.p_gpio_x = GPIOF;
    }
    else
    {
        return -1;
    }


    pinNumber = port_pin[2] - 48;

    /* @brief Prevent user from accessing JTAG pins on GPIO Port C */
    gpioPin.pin_config.pin_number = pinNumber;
    if( (gpioPin.p_gpio_x == GPIOC) && (pinNumber <= 3) )
    {
        return -1;
    }


    gpioPin.pin_config.direction = pinDirection;


    _hf_setPinDefaults(&gpioPin);

    while(loopVar < numOfArgs)
    {
        argValue = (uint8_t)va_arg(gpioArgs, int);

        // This should take care of unexpected values
        if(argValue < 2 || argValue > 9)
        {
            break;
        }

        switch(argValue)
        {

        case DIGITAL:
            if(gpioPin.pin_config.pin_mode != GPIO_AMSEL_ENABLE)
            {
                gpioPin.pin_config.pin_mode = GPIO_DEN_ENABLE;

                digitalSetFlag = 1;
            }
            break;

        case ANALOG:
            if(digitalSetFlag == 0)
            {
                gpioPin.pin_config.pin_mode = GPIO_AMSEL_ENABLE;
            }
            break;

        case PULLUP:
            if(gpioPin.pin_config.pullupdown != GPIO_PDR_ENABLE)
            {
                gpioPin.pin_config.pullupdown = GPIO_PUR_ENABLE;
            }
            break;

        case PULLDOWN:
            if(gpioPin.pin_config.pullupdown != GPIO_PUR_ENABLE)
            {
                gpioPin.pin_config.pullupdown = GPIO_PDR_ENABLE;
            }
            break;

        case OPEN_DRAIN:
            gpioPin.pin_config.opendrain = GPIO_ODDR_ENABLE;
            break;

        case DRIVE_4MA:
            gpioPin.pin_config.drive = GPIO_DR4R;
            break;

        }

        loopVar++;
    }

    // TODO, this should be a function pointer
    gpio_init(&gpioPin);

    va_end(gpioArgs);

    return 0;
}


/*
 * @brief   Write to GPIO Pin
 * @param   *port_pin : Name of the GPIO pin/
 * @param   pinState  : Value of State to set on the pin.
 * @retval  int8_t    : Success = 0, Failure = -1.
 */
int8_t digitalWrite(char *port_pin, uint8_t pinState)
{

    uint8_t pinNumber = 0;

    pinNumber = port_pin[2] - 48;

    if( (strncmp(port_pin,"pa",2 ) == 0) || (strncmp(port_pin,"PA",2 ) == 0) )
    {
        gpio_write_to_pin(GPIOA, pinNumber, pinState);
    }
    else if( (strncmp(port_pin,"pb",2 ) == 0) || (strncmp(port_pin,"PB",2 ) == 0) )
    {
        gpio_write_to_pin(GPIOB, pinNumber, pinState);
    }
    else if( (strncmp(port_pin,"pc",2 ) == 0) || (strncmp(port_pin,"PC",2 ) == 0) )
    {
        gpio_write_to_pin(GPIOC, pinNumber, pinState);
    }
    else if( (strncmp(port_pin,"pd",2 ) == 0) || (strncmp(port_pin,"PD",2 ) == 0) )
    {
        gpio_write_to_pin(GPIOD, pinNumber, pinState);
    }
    else if( (strncmp(port_pin,"pe",2 ) == 0) || (strncmp(port_pin,"PE",2 ) == 0) )
    {
        gpio_write_to_pin(GPIOE, pinNumber, pinState);
    }
    else if( (strncmp(port_pin,"pf",2 ) == 0) || (strncmp(port_pin,"PF",2 ) == 0) )
    {
        gpio_write_to_pin(GPIOF, pinNumber, pinState);
    }
    else
    {
        return -1;
    }

    return 0;
}



/*
 * @brief   Read from GPIO pin (Blocking function)
 * @param   *port_pin : Name of the GPIO pin
 * @retval  int8_t    : Success = Return value from the pin, Failure = -1
 */
int8_t digitalRead(char *port_pin)
{
    uint8_t returnValue = 0;
    uint8_t pinNumber  = 0;

    pinNumber = port_pin[2] - 48;


    if( (strncmp(port_pin,"pa",2 ) == 0) || (strncmp(port_pin,"PA",2 ) == 0) )
    {
        returnValue = gpio_read_from_pin(GPIOA, pinNumber);
    }
    else if( (strncmp(port_pin,"pb",2 ) == 0) || (strncmp(port_pin,"PB",2 ) == 0) )
    {
        returnValue = gpio_read_from_pin(GPIOB, pinNumber);
    }
    else if( (strncmp(port_pin,"pc",2 ) == 0) || (strncmp(port_pin,"PC",2 ) == 0) )
    {
        returnValue = gpio_read_from_pin(GPIOC, pinNumber);
    }
    else if( (strncmp(port_pin,"pd",2 ) == 0) || (strncmp(port_pin,"PD",2 ) == 0) )
    {
        returnValue = gpio_read_from_pin(GPIOD, pinNumber);
    }
    else if( (strncmp(port_pin,"pe",2 ) == 0) || (strncmp(port_pin,"PE",2 ) == 0) )
    {
        returnValue = gpio_read_from_pin(GPIOE, pinNumber);
    }
    else if( (strncmp(port_pin,"pf",2 ) == 0) || (strncmp(port_pin,"PF",2 ) == 0) )
    {
        returnValue = gpio_read_from_pin(GPIOF, pinNumber);
    }
    else
    {
        return -1;
    }

    return returnValue;

}


