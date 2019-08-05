/**
 ******************************************************************************
 * @file    gpio_tm4c123gh6pm.h, file name will change
 * @author  Aditya Mall,
 * @brief   TM4C123GH6PM Device Peripheral Access Layer Header File.
 *
 *  This file contains:
 *              - Macros for GPIO pin Initialization
 *              - Macros for enabling & disabling Clock at GPIO Ports
 *              - Data Structures for GPIO Pin Initialization
 *              - API Function Prototypes
 *              - API Interrupt Prototypes
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



#ifndef GPIO_TM4C123GH6PM_H_
#define GPIO_TM4C123GH6PM_H_


/*
 * @brief Standard Headers and board specific header file
 */
#include "main_tm4c123gh6pm.h"
#include <stdint.h>
#include <stdbool.h>


/******************************************************************************/
/*                                                                            */
/*                  Macros for GPIO Pin Initialization                        */
/*                                                                            */
/******************************************************************************/


/*
 * @brief GPIO Pin Initialization Macros
 */
#define GPIO_DIR_INPUT           ((uint8_t)0x00UL)  /*!< Enable GPIO Direction register (GPIODIR) as Input                    */
#define GPIO_DIR_OUTPUT          ((uint8_t)0x01UL)  /*!< Enable GPIO Direction register (GPIODIR) as Output                   */
#define GPIO_IS_EDGE             ((uint8_t)0x00UL)  /*!< Enable GPIO Interrupt Sense Register (GPIOIS) to be Edge Sensitive   */
#define GPIO_IS_LEVEL            ((uint8_t)0x01UL)  /*!< Enable GPIO Interrupt Sense Register (GPIOIS) to be Level Sensitive  */
#define GPIO_BE_DISABLE          ((uint8_t)0x00UL)  /*!< Interrupt generation is controlled by GPIO Interrupt Event register  */
#define GPIO_BE_ENABLE           ((uint8_t)0x01UL)  /*!< Enable GPIO Interrupt Both Edges (GPIOIBE) to be Both Edge Sensitive */
#define GPIO_AFSEL_DISABLE       ((uint8_t)0x00UL)  /*!< Disable GPIO Alternate Function Select register (GPIOAFSEL)          */
#define GPIO_AFSEL_ENABLE        ((uint8_t)0x01UL)  /*!< Enable GPIO Alternate Function Select register (GPIOAFSEL)           */
#define GPIO_DR2R                ((uint8_t)0x00UL)  /*!< Enable GPIO Drive Select (GPIODR2R) to 2mA Drive                     */
#define GPIO_DR4R                ((uint8_t)0x01UL)  /*!< Enable GPIO Drive Select (GPIODR4R) to 4mA Drive                     */
#define GPIO_DR8R                ((uint8_t)0x02UL)  /*!< Enable GPIO Drive Select (GPIODR8R) to 8mA Drive                     */
#define GPIO_ODDR_DISABLE        ((uint8_t)0x00UL)  /*!< Disable GPIO Open Drain Select Register (GPIOODR)                    */
#define GPIO_ODDR_ENABLE         ((uint8_t)0x01UL)  /*!< Enable GPIO Open Drain Select Register (GPIOODR)                     */
#define GPIO_NOPUPD              ((uint8_t)0x00UL)  /*!< No Pull-Up ad Pull-Down is selected                                  */
#define GPIO_PUR_ENABLE          ((uint8_t)0x01UL)  /*!< Enable GPIO Pull-Up Select Register (GPIOPUR)                        */
#define GPIO_PDR_ENABLE          ((uint8_t)0x02UL)  /*!< Enable GPIO Pull-Down Select Register (GPIOPUR)                      */
#define GPIO_DEN_DISABLE         ((uint8_t)0x00UL)  /*!< Disable GPIO Digital Register (GPIOAMSEL)                            */
#define GPIO_DEN_ENABLE          ((uint8_t)0x01UL)  /*!< Enable GPIO Digital Register (GPIODEN)                               */
#define GPIO_AMSEL_DISABLE       ((uint8_t)0x00UL)  /*!< Disable GPIO Analog Mode Select Register (GPIOAMSEL)                 */
#define GPIO_AMSEL_ENABLE        ((uint8_t)0x02UL)  /*!< Enable GPIO Analog Mode Select Register (GPIOAMSEL)                  */


/*
 * @brief GPIO Slew Rate Control Select (GPIOSLR) Register
 * @note  Slew rate control is only available when using the 8-mA drive strength option
 */
#define GPIO_SLR_DISABLE         ((uint8_t)0x00UL)  /*!< */
#define GPIO_SLR_ENABLE          ((uint8_t)0x01UL)  /*!< */


/*
 * @brief GPIO Port Control (GPIOPCTL) Register Defines
 */
#define PCTL_AF0                 ((uint8_t)0x00UL)  /*!< Bit Field Encoding of Alternate Function 0  */
#define PCTL_AF1                 ((uint8_t)0x01UL)  /*!< Bit Field Encoding of Alternate Function 1  */
#define PCTL_AF2                 ((uint8_t)0x02UL)  /*!< Bit Field Encoding of Alternate Function 2  */
#define PCTL_AF3                 ((uint8_t)0x03UL)  /*!< Bit Field Encoding of Alternate Function 3  */
#define PCTL_AF4                 ((uint8_t)0x04UL)  /*!< Bit Field Encoding of Alternate Function 4  */
#define PCTL_AF5                 ((uint8_t)0x05UL)  /*!< Bit Field Encoding of Alternate Function 5  */
#define PCTL_AF6                 ((uint8_t)0x06UL)  /*!< Bit Field Encoding of Alternate Function 6  */
#define PCTL_AF7                 ((uint8_t)0x07UL)  /*!< Bit Field Encoding of Alternate Function 7  */
#define PCTL_AF8                 ((uint8_t)0x08UL)  /*!< Bit Field Encoding of Alternate Function 8  */
#define PCTL_AF9                 ((uint8_t)0x09UL)  /*!< Bit Field Encoding of Alternate Function 9  */
#define PCTL_AF10                ((uint8_t)0x0AUL)  /*!< Bit Field Encoding of Alternate Function 10 */
#define PCTL_AF11                ((uint8_t)0x0BUL)  /*!< Bit Field Encoding of Alternate Function 11 */
#define PCTL_AF12                ((uint8_t)0x0CUL)  /*!< Bit Field Encoding of Alternate Function 12 */
#define PCTL_AF13                ((uint8_t)0x0DUL)  /*!< Bit Field Encoding of Alternate Function 13 */
#define PCTL_AF14                ((uint8_t)0x0EUL)  /*!< Bit Field Encoding of Alternate Function 14 */
#define PCTL_AF15                ((uint8_t)0x0FUL)  /*!< Bit Field Encoding of Alternate Function 15 */


/*
 * @brief UART Module 0, 1, 2, 3, 4, 5, 6, and 7 Defines
 */
#define UART0RX_PA0              PCTL_AF1  /*!< UART 0 Rx on PA0  */
#define UART0TX_PA1              PCTL_AF1  /*!< UART 0 Tx on PA1  */
#define UART1RX_PB0              PCTL_AF1  /*!< UART 1 Rx on PB0  */
#define UART1TX_PB1              PCTL_AF1  /*!< UART 1 Tx on PB1  */
#define UART1RX_PC4              PCTL_AF2  /*!< UART 1 Rx on PC4  */
#define UART1TX_PC5              PCTL_AF2  /*!< UART 1 Tx on PC5  */
#define UART1RTS_PC4             PCTL_AF8  /*!< UART 1 RTS on PC4 */
#define UART1CTS_PC5             PCTL_AF8  /*!< UART 1 CTS on PC5 */
#define UART1RTS_PF0             PCTL_AF1  /*!< UART 1 RTS on PF0 */
#define UART1CTS_PF1             PCTL_AF1  /*!< UART 1 CTS on PF1 */
#define UART2RX_PD6              PCTL_AF1  /*!< UART 2 CTS on PD6 */
#define UART2TX_PD7              PCTL_AF1  /*!< UART 2 CTS on PD7 */
#define UART3RX_PC6              PCTL_AF1  /*!< UART 3 CTS on PC6 */
#define UART3TX_PC7              PCTL_AF1  /*!< UART 3 CTS on PC7 */
#define UART4RX_PC4              PCTL_AF1  /*!< UART 4 Rx on PC4  */
#define UART4TX_PC5              PCTL_AF1  /*!< UART 4 Tx on PC5  */
#define UART5RX_PE4              PCTL_AF1  /*!< UART 5 Rx on PE4  */
#define UART5TX_P45              PCTL_AF1  /*!< UART 5 Tx on PE5  */
#define UART6RX_PD4              PCTL_AF1  /*!< UART 6 Rx on PD4  */
#define UART6TX_PD5              PCTL_AF1  /*!< UART 6 Tx on PD5  */
#define UART7RX_PE0              PCTL_AF1  /*!< UART 7 Rx on PE0  */
#define UART7TX_PE1              PCTL_AF1  /*!< UART 7 Tx on PE1  */


/*
 * @brief SSI Module 0, 1, 2, and 3 defines
 */
#define SSI0CLK_PA2              PCTL_AF2  /*!< SSI 0 Clk on PA2 */
#define SSI0FSS_PA3              PCTL_AF2  /*!< SSI 0 Fss on PA3 */
#define SSI0RX_PA4               PCTL_AF2  /*!< SSI 0 Rs on PA4  */
#define SSI0TX_PA5               PCTL_AF2  /*!< SSI 0 Tx on PA5  */
#define SSI1CLK_PD0              PCTL_AF2  /*!< SSI 1 Clk on PD0 */
#define SSI1FSS_PD1              PCTL_AF2  /*!< SSI 1 Fss on PD1 */
#define SSI1RX_PD2               PCTL_AF2  /*!< SSI 1 Rs on PD2  */
#define SSI1TX_PD3               PCTL_AF2  /*!< SSI 1 Tx on PD3  */
#define SSI1CLK_PF2              PCTL_AF2  /*!< SSI 1 Clk on PF2 */
#define SSI1FSS_PF3              PCTL_AF2  /*!< SSI 1 Fss on PF3 */
#define SSI1RX_PF0               PCTL_AF2  /*!< SSI 1 Rs on PF0  */
#define SSI1TX_PF1               PCTL_AF2  /*!< SSI 1 Tx on PF1  */
#define SSI2CLK_PB4              PCTL_AF2  /*!< SSI 2 Clk on PB4 */
#define SSI2FSS_PB5              PCTL_AF2  /*!< SSI 2 Fss on PB5 */
#define SSI2RX_PB6               PCTL_AF2  /*!< SSI 2 Rs on PB6  */
#define SSI2TX_PB7               PCTL_AF2  /*!< SSI 2 Tx on PB7  */
#define SSI3CLK_PD0              PCTL_AF1  /*!< SSI 3 Clk on PD0 */
#define SSI3FSS_PD1              PCTL_AF1  /*!< SSI 3 Fss on PD1 */
#define SSI3RX_PD2               PCTL_AF1  /*!< SSI 3 Rs on PD2  */
#define SSI3TX_PD3               PCTL_AF1  /*!< SSI 3 Tx on PD3  */



/* TODO Complete defines for rest of the modules as you build module drivers */
/*
 * @brief I2 C Module defines
 */



/* @brief GPIO Interrupt Event (GPIOIEV) Register */
#define GPIO_EV_FALLING_EDGE     ((uint8_t)0x00UL)  /*!< */
#define GPIO_EV_RISING_EDGE      ((uint8_t)0x01UL)  /*!< */

/* @brief GPIO Interrupt Mask (GPIOIM) Register */
#define GPIO_IM_DISABLE          ((uint8_t)0x00UL)  /*!< */
#define GPIO_IM_ENABLE           ((uint8_t)0x01UL)  /*!< */

/* @brief GPIO Interrupt Clear (GPIOICR) Register */
#define GPIO_IC_CLEAR            ((uint8_t)0x01UL)  /*!< The corresponding interrupt is cleared by writing 1                        */


/*
 * @brief GPIO Status / Read only registers
 */
#define GPIO_RIS_FLAG            ((uint8_t)0x01UL)  /*!< GPIO Raw Interrupt Status (GPIORIS) Flag, Interrupt condition has occurred */
#define GPIO_MIS_FLAG            ((uint8_t)0x01UL)  /*!< GPIO Masked Interrupt Status Flag, Interrupt condition has occurred        */


/*
 * @brief GPIO Peripheral Enable Macros
 */
#define GPIOA_ENABLE             ((uint8_t)0x01UL)
#define GPIOB_ENABLE             ((uint8_t)0x02UL)
#define GPIOC_ENABLE             ((uint8_t)0x04UL)
#define GPIOD_ENABLE             ((uint8_t)0x08UL)
#define GPIOE_ENABLE             ((uint8_t)0x10UL)
#define GPIOF_ENABLE             ((uint8_t)0x20UL)


/******************************************************************************/
/*                                                                            */
/*                  Data Structures for GPIO Pin Initialization               */
/*                                                                            */
/******************************************************************************/


/*
 * @brief GPIO Configuration Structure
 */
/*TODO Add slew rate control */
typedef struct
{
    uint8_t pin_number;          /*!< GPIO Pin Number                                      */
    uint8_t direction;           /*!< GPIO Pin Direction                                   */
    uint8_t alternate_function;  /*!< GPIO Pin Alternate Function                          */
    uint8_t drive;               /*!< GPIO Pin Drive Select, GPIODR2R, GPIODR4R, GPIODR8R  */
    uint8_t opendrain;           /*!< GPIO Pin Open Drain Select                           */
    uint8_t pullupdown;          /*!< GPIO Pin Pull-Up and Pull-Down Select                */
    uint8_t pin_mode;            /*!< GPIO Digital or Analog Register Mode Select          */
    uint8_t port_control;        /*!< GPIO Port Control Register Value                     */

} gpio_config_t;


/*
 * @brief GPIO Handle Structure
 */
typedef struct
{
    GPIO_PORT_T    *p_gpio_x;  /*!< Pointer to GPIO Peripheral Address                         */
    gpio_config_t  pin_config; /*!< Variable Declaration GPIO pin configuration data structure */

} gpio_handle_t;




/******************************************************************************/
/*                                                                            */
/*                 GPIO Peripheral Driver Functions Prototypes                */
/*                                                                            */
/******************************************************************************/



/*
 * @brief   Intializes GPIO pin.
 * @param   *p_gpio_handle : pointer to the GPIO Handle structure (gpio_handle_t).
 * @retval  None.
 */
void gpio_init(gpio_handle_t *p_gpio_handle);



/*
 * @brief   Deinitialize GPIO pin.
 * @param   *p_gpio_handle : pointer to the GPIO Handle structure (gpio_handle_t).
 * @retval  None.
 */
void gpio_deinit(gpio_handle_t *p_gpio_handle);



/*
 * @brief   Read from GPIO pin (Blocking function)
 * @param   *p_gpio_x : pointer to the GPIO port structure (GPIO_PORT_T).
 * @param   pin       : GPIO Pin Number.
 * @retval  uint8_t   : Return value from the pin.
 */
uint8_t gpio_read_from_pin(GPIO_PORT_T *p_gpio_x, uint8_t pin);



/*
 * @brief   Write to GPIO pin
 * @param   *p_gpio_x : pointer to the GPIO port structure (GPIO_PORT_T).
 * @param   pin       : GPIO Pin Number
 * @bool    state     : Value to be written, 1 or 0.
 * @retval  None.
 */
void gpio_write_to_pin(GPIO_PORT_T *p_gpio_x, uint8_t pin, bool state);



/*
 * @brief   Read from GPIO port (Blocking function)
 * @param   *p_gpio_x  : pointer to the GPIO port structure (GPIO_PORT_T).
 * @retval  uint8_t  : Data from the port
 */
uint8_t gpio_read_from_port(GPIO_PORT_T *p_gpio_x);



/*
 * @brief   Write to GPIO port
 * @param   *p_gpio_x  : pointer to the GPIO port structure (GPIO_PORT_T).
 * @param   value  : Data to be written to the port
 * @retval  None.
 */
void gpio_write_to_port(GPIO_PORT_T *p_gpio_x, uint8_t value);




/******************************* TODO ***********************************************************/

/*
 * @brief   Configure GPIO Interrupt
 * @param   IRQNumber   : Interrupt Number
 * @param   IRQPriority : Interrupt Priority
 * @param   state       : Enable or Disable Interrupt
 * @retval  None.
 */
void GPIO_InterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, bool state);



/*
 * @brief   Configure GPIO Interrupt
 * @param   pinNumber: GPIO Pin Number triggering the interrupt
 * @retval  None.
 */
void GPIO_InterruptHandling(uint8_t pinNumber);




#endif /* GPIO_TM4C123GH6PM_H_ */
