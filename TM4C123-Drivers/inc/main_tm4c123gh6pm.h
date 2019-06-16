/**
 ******************************************************************************
 * @file    custom_tm4c123.hgh6pm, file name will change
 * @author  Aditya Mall, Hari Haran Krishnan
 * @brief   TM4C123GH6PM Device Peripheral Access Layer Header File.
 *
 *          This file contains:
 *           - Data structures and the address mapping for all peripherals
 *           - peripherals registers declarations and bits definition
 *           - Macros to access peripheral’s registers hardware
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 Aditya Mall, Hari Haran Krishnan </center></h2>
 *
 * TODO Add license.
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

#ifndef MAIN_TM4C123GH6PM_H_
#define MAIN_TM4C123GH6PM_H_


/*
 * @brief Standard C Headers
 */
#include <stdint.h>



/**
 * @brief IO definitions (access restrictions to peripheral registers), idea taken from ARM header file
 */
#define __I       volatile const       /*!< Defines 'read only' permissions    */
#define __O       volatile             /*!< Defines 'write only' permissions   */
#define __IO      volatile             /*!< Defines 'read / write' permissions */


/*
 * @brief Generic Bit Macro Definitions
 */
#define SET       1
#define RESET     0
#define ENABLE    SET
#define DISABLE   RESET
#define EN        ENABLE
#define DS        DISABLE



/***************************************************************************/
/*                                                                         */
/*                       Device Memory Map                                 */
/*                                                                         */
/***************************************************************************/


/**
 * @brief Base Addresses of Flash and SRAM memory
 */
#define FLASH_BASEADDR           0x00000000UL  /*!< FLASH (up to 256 KB) base address in the alias region */
#define SRAM_BASEADDR            0x20000000UL  /*!< SRAM (32 KB) base address in the alias region         */
#define SRAM_BB_BASEADDR         0x22000000UL  /*!< SRAM (32 KB) base address in the bit-band region      */


/**
 * @brief Peripheral Bus Base Addresses
 */
#define PERIPH_BASEADDR          0x40000000UL                      /*!< Peripheral base address in the alias region         */
#define PERIPH_BB_BASEADDR       (PERIPH_BASEADDR + 0x02000000UL)  /*!< Bit-banded alias of 0x4000.0000 through 0x400F.FFFF */
#define APBPERIPH_BASEADDR       PERIPH_BASEADDR                   /*!< Base address of Advanced Peripheral Bus             */
#define AHBPERIPH_BASEADDR       (PERIPH_BASEADDR + 0x00050000UL)  /*!< Base address of Advanced High Performance Bus       */


/**
 * @brief APB Peripherals Base Addresses
 * TODO add other peripherals
 */
#define WATCHDOG0_BASEADDR       (APBPERIPH_BASEADDR + 0x0000UL)      /*!< Base address of Watchdog timer 0                    */
#define WATCHDOG1_BASEADDR       (APBPERIPH_BASEADDR + 0x1000UL)      /*!< Base address of Watchdog timer 1                    */

#define GPIOA_BASEADDR           (APBPERIPH_BASEADDR + 0x4000UL)      /*!< Base address of GPIO Port A                         */
#define GPIOB_BASEADDR           (APBPERIPH_BASEADDR + 0x5000UL)      /*!< Base address of GPIO Port B                         */
#define GPIOC_BASEADDR           (APBPERIPH_BASEADDR + 0x6000UL)      /*!< Base address of GPIO Port C                         */
#define GPIOD_BASEADDR           (APBPERIPH_BASEADDR + 0x7000UL)      /*!< Base address of GPIO Port D                         */
#define GPIOE_BASEADDR           (APBPERIPH_BASEADDR + 0x00024000UL)  /*!< Base address of GPIO Port E                         */
#define GPIOF_BASEADDR           (APBPERIPH_BASEADDR + 0x00025000UL)  /*!< Base address of GPIO Port F                         */

#define SSI0_BASEADDR            (APBPERIPH_BASEADDR + 0x8000UL)      /*!< Base address of SSI0                                */
#define SSI1_BASEADDR            (APBPERIPH_BASEADDR + 0x9000UL)      /*!< Base address of SSI1                                */
#define SSI2_BASEADDR            (APBPERIPH_BASEADDR + 0xA000UL)      /*!< Base address of SSI2                                */
#define SSI3_BASEADDR            (APBPERIPH_BASEADDR + 0xB000UL)      /*!< Base address of SSI3                                */

#define UART0_BASEADDR           (APBPERIPH_BASEADDR + 0xC000UL)      /*!< Base address of UART0                               */
#define UART1_BASEADDR           (APBPERIPH_BASEADDR + 0xD000UL)      /*!< Base address of UART1                               */
#define UART2_BASEADDR           (APBPERIPH_BASEADDR + 0xE000UL)      /*!< Base address of UART2                               */
#define UART3_BASEADDR           (APBPERIPH_BASEADDR + 0xF000UL)      /*!< Base address of UART3                               */
#define UART4_BASEADDR           (APBPERIPH_BASEADDR + 0x00010000UL)  /*!< Base address of UART4                               */
#define UART5_BASEADDR           (APBPERIPH_BASEADDR + 0x00011000UL)  /*!< Base address of UART5                               */
#define UART6_BASEADDR           (APBPERIPH_BASEADDR + 0x00012000UL)  /*!< Base address of UART6                               */
#define UART7_BASEADDR           (APBPERIPH_BASEADDR + 0x00013000UL)  /*!< Base address of UART7                               */

#define I2C0_BASEADDR            (APBPERIPH_BASEADDR + 0x00020000UL)  /*!< Base address of I2C 0                               */
#define I2C1_BASEADDR            (APBPERIPH_BASEADDR + 0x00021000UL)  /*!< Base address of I2C 1                               */
#define I2C2_BASEADDR            (APBPERIPH_BASEADDR + 0x00022000UL)  /*!< Base address of I2C 2                               */
#define I2C3_BASEADDR            (APBPERIPH_BASEADDR + 0x00023000UL)  /*!< Base address of I2C 3                               */

#define SYSCTL_BASEADDR          (APBPERIPH_BASEADDR + 0x000FE000UL)  /*!< Base address of System Control Block                */



/**
 * @brief AHB Peripherals Base Addresses
 */
#define USB_BASEADDR             (AHBPERIPH_BASEADDR + 0x0000UL)  /*!< Base address of USB                        */
#define GPIOA_AHB_BASEADDR       (AHBPERIPH_BASEADDR + 0x8000UL)  /*!< Base address of GPIO Port A (AHB aperture) */
#define GPIOB_AHB_BASEADDR       (AHBPERIPH_BASEADDR + 0x9000UL)  /*!< Base address of GPIO Port B (AHB aperture) */
#define GPIOC_AHB_BASEADDR       (AHBPERIPH_BASEADDR + 0xA000UL)  /*!< Base address of GPIO Port C (AHB aperture) */
#define GPIOD_AHB_BASEADDR       (AHBPERIPH_BASEADDR + 0xB000UL)  /*!< Base address of GPIO Port D (AHB aperture) */
#define GPIOE_AHB_BASEADDR       (AHBPERIPH_BASEADDR + 0xC000UL)  /*!< Base address of GPIO Port E (AHB aperture) */
#define GPIOF_AHB_BASEADDR       (AHBPERIPH_BASEADDR + 0xD000UL)  /*!< Base address of GPIO Port F (AHB aperture) */





/***************************************************************************/
/*                                                                         */
/*                 Device Specific Data Structure Section                  */
/*                                                                         */
/***************************************************************************/



/**
 * @brief Peripheral Register Structures
 */



/**
 * @brief System Control Register Structure
 */
typedef struct
{
    __IO uint32_t DID0;             /*!< Device Identification 0,                                                           Address offset: 0x000          */
    __IO uint32_t DID1;             /*!< Device Identification 1,                                                           Address offset: 0x004          */
    __I  uint32_t LEGACY[10];       /*!< Legacy Registers,                                                                  Address offset: 0x008 to 0x02C */
    __IO uint32_t PBORCTL;          /*!< Brown-Out Reset Control,                                                           Address offset: 0x030          */
    __I  uint32_t LEGACY2[7];       /*!< Legacy Registers,                                                                  Address offset: 0x034 to 0x04C */
    __IO uint32_t RIS;              /*!< Raw Interrupt Status,                                                              Address offset: 0x050          */
    __IO uint32_t IMC;              /*!< Interrupt Mask Control,                                                            Address offset: 0x054          */
    __IO uint32_t MISC;             /*!< Masked Interrupt Status and Clear,                                                 Address offset: 0x058          */
    __IO uint32_t RESC;             /*!< Reset Cause,                                                                       Address offset: 0x05C          */
    __IO uint32_t RCC;              /*!< Run-Mode Clock Configuration,                                                      Address offset: 0x060          */
    __I  uint32_t RESERVED[2];      /*!< RESERVED,                                                                          Address offset: 0x064 to 0x068 */
    __IO uint32_t GPIOHBCTL;        /*!< GPIO High-Performance Bus Control,                                                 Address offset: 0x06C          */
    __IO uint32_t RCC2;             /*!< Run-Mode Clock Configuration 2,                                                    Address offset: 0x070          */
    __I  uint32_t RESERVED1[2];     /*!< RESERVED,                                                                          Address offset: 0x074 to 0x078 */
    __IO uint32_t MOSCCTL;          /*!< Main Oscillator Control,                                                           Address offset: 0x07C          */
    __I  uint32_t RESERVED2[32];    /*!< RESERVED,                                                                          Address offset: 0x080 to 0x0FC */
    __I  uint32_t LEGACY3[3];       /*!< Legacy Registers,                                                                  Address offset: 0x100 to 0x108 */
    __I  uint32_t RESERVED3;        /*!< RESERVED,                                                                          Address offset: 0x10C          */
    __I  uint32_t LEGACY4[3];       /*!< Legacy Registers,                                                                  Address offset: 0x110 to 0x118 */
    __I  uint32_t RESERVED4;        /*!< RESERVED,                                                                          Address offset: 0x11C          */
    __I  uint32_t LEGACY5[3];       /*!< Legacy Registers,                                                                  Address offset: 0x120 to 0x128 */
    __I  uint32_t RESERVED5[6];     /*!< RESERVED,                                                                          Address offset: 0x12C to 0x140 */
    __IO uint32_t DSLPCLKCFG;       /*!< Deep Sleep Clock Configuration,                                                    Address offset: 0x144          */
    __I  uint32_t RESERVED6;        /*!< RESERVED,                                                                          Address offset: 0x148          */
    __IO uint32_t SYSPROP;          /*!< System Properties,                                                                 Address offset: 0x14C          */
    __IO uint32_t PIOSCCAL;         /*!< Precision Internal Oscillator Calibration,                                         Address offset: 0x150          */
    __IO uint32_t PIOSCSTAT;        /*!< Precision Internal Oscillator Statistics,                                          Address offset: 0x154          */
    __I  uint32_t RESERVED7[2];     /*!< RESERVED,                                                                          Address offset: 0x158 to 0x15C */
    __IO uint32_t PLLFREQ0;         /*!< PLL Frequency 0,                                                                   Address offset: 0x160          */
    __IO uint32_t PLLFREQ1;         /*!< PLL Frequency 1,                                                                   Address offset: 0x164          */
    __IO uint32_t PLLSTAT;          /*!< PLL Status,                                                                        Address offset: 0x168          */
    __I  uint32_t RESERVED8[7];     /*!< RESERVED,                                                                          Address offset: 0x16C to 0x184 */
    __IO uint32_t SLPPWRCFG;        /*!< Sleep Power Configuration,                                                         Address offset: 0x188          */
    __IO uint32_t DSLPPWRCFG;       /*!< Deep-Sleep Power Configuration,                                                    Address offset: 0x18C          */
    __I  uint32_t LEGACY6;          /*!< Legacy Registers,                                                                  Address offset: 0x190          */
    __I  uint32_t RESERVED9[3];     /*!< RESERVED,                                                                          Address offset: 0x194 to 0x19C */
    __I  uint32_t LEGACY7;          /*!< Legacy Registers,                                                                  Address offset: 0x1A0          */
    __I  uint32_t RESERVED10[4];    /*!< RESERVED,                                                                          Address offset: 0x1A4 to 0x1B0 */
    __IO uint32_t LDOSPCTL;         /*!< LDO Sleep Power Control,                                                           Address offset: 0x1B4          */
    __I  uint32_t LDOSPCAL;         /*!< LDO Sleep Power Calibration,                                                       Address offset: 0x1B8          */
    __IO uint32_t LDODPCTL;         /*!< LDO Deep-Sleep Power Control,                                                      Address offset: 0x1BC          */
    __IO uint32_t LDODPCAL;         /*!< LDO Deep-Sleep Power Calibration,                                                  Address offset: 0x1C0          */
    __I  uint32_t RESERVED11[2];    /*!< RESERVED,                                                                          Address offset: 0x1C4 to 0x1C8 */
    __IO uint32_t SDPMST;           /*!< Sleep / Deep-Sleep Power Mode Status,                                              Address offset: 0x1CC          */
    __I  uint32_t RESERVED12[76];   /*!< RESERVED,                                                                          Address offset: 0x1D0 to 0x2FC */
    __IO uint32_t PPWD;             /*!< Watchdog Timer Peripheral Present,                                                 Address offset: 0x300          */
    __IO uint32_t PPTIMER;          /*!< 16/32-Bit General-Purpose Timer Peripheral Present,                                Address offset: 0x304          */
    __IO uint32_t PPGPIO;           /*!< General-Purpose Input/Output Peripheral Present,                                   Address offset: 0x308          */
    __IO uint32_t PPDMA;            /*!< Micro Direct Memory Access Peripheral Present,                                     Address offset: 0x30C          */
    __I  uint32_t RESERVED13;       /*!< RESERVED,                                                                          Address offset: 0x310          */
    __IO uint32_t PPHIB;            /*!< Hibernation Peripheral Present,                                                    Address offset: 0x314          */
    __IO uint32_t PPUART;           /*!< Universal Asynchronous Receiver/Transmitter Peripheral Present,                    Address offset: 0x318          */
    __IO uint32_t PPSSI;            /*!< Synchronous Serial Interface Peripheral Present,                                   Address offset: 0x31C          */
    __IO uint32_t PPI2C;            /*!< Inter-Integrated Circuit Peripheral Present,                                       Address offset: 0x320          */
    __I  uint32_t RESERVED14;       /*!< RESERVED,                                                                          Address offset: 0x324          */
    __IO uint32_t PPUSB;            /*!< Universal Serial Bus Peripheral Present,                                           Address offset: 0x328          */
    __I  uint32_t RESERVED15[2];    /*!< RESERVED,                                                                          Address offset: 0x32C to 0x330 */
    __IO uint32_t PPCAN;            /*!< Controller Area Network Peripheral Present,                                        Address offset: 0x334          */
    __IO uint32_t PPADC;            /*!< Analog-to-Digital Converter Peripheral Present,                                    Address offset: 0x338          */
    __IO uint32_t PPACMP;           /*!< Analog Comparator Peripheral Present,                                              Address offset: 0x33C          */
    __IO uint32_t PPPWM;            /*!< Pulse Width Modulator Peripheral Present,                                          Address offset: 0x340          */
    __IO uint32_t PPQEI;            /*!< Quadrature Encoder Interface Peripheral Present,                                   Address offset: 0x344          */
    __I  uint32_t RESERVED16[4];    /*!< RESERVED,                                                                          Address offset: 0x348 to 0x354 */
    __IO uint32_t PPEEPROM;         /*!< EEPROM Peripheral Present,                                                         Address offset: 0x358          */
    __IO uint32_t PPWTIMER;         /*!< 32/64-Bit Wide General-Purpose Timer Peripheral Present,                           Address offset: 0x35C          */
    __I  uint32_t RESERVED17[104];  /*!< RESERVED,                                                                          Address offset: 0x360 to 0x4FC */
    __IO uint32_t SRWD;             /*!< Watchdog Timer Software Reset,                                                     Address offset: 0x500          */
    __IO uint32_t SRTIMER;          /*!< 16/32-Bit General-Purpose Timer Software Reset,                                    Address offset: 0x504          */
    __IO uint32_t SRGPIO;           /*!< General-Purpose Input/Output Software Reset,                                       Address offset: 0x508          */
    __IO uint32_t SRDMA;            /*!< Micro Direct Memory Access Software Reset,                                         Address offset: 0x50C          */
    __I  uint32_t RESERVED18;       /*!< RESERVED,                                                                          Address offset: 0x510          */
    __IO uint32_t SRHIB;            /*!< Hibernation Software Reset,                                                        Address offset: 0x514          */
    __IO uint32_t SRUART;           /*!< Universal Asynchronous Receiver/Transmitter Software Reset,                        Address offset: 0x518          */
    __IO uint32_t SRSSI;            /*!< Synchronous Serial Interface Software Reset,                                       Address offset: 0x51C          */
    __IO uint32_t SRI2C;            /*!< Inter-Integrated Circuit Software Reset,                                           Address offset: 0x520          */
    __I  uint32_t RESERVED19;       /*!< RESERVED,                                                                          Address offset: 0x524          */
    __IO uint32_t SRUSB;            /*!< Universal Serial Bus Software Reset,                                               Address offset: 0x528          */
    __I  uint32_t RESERVED20[2];    /*!< RESERVED,                                                                          Address offset: 0x52C to 0x530 */
    __IO uint32_t SRCAN;            /*!< Controller Area Network Software Reset,                                            Address offset: 0x534          */
    __IO uint32_t SRADC;            /*!< Analog-to-Digital Converter Software Reset,                                        Address offset: 0x538          */
    __IO uint32_t SRACMP;           /*!< Analog Comparator Software Reset,                                                  Address offset: 0x53C          */
    __IO uint32_t SRPWM;            /*!< Pulse Width Modulator Software Reset,                                              Address offset: 0x540          */
    __IO uint32_t SRQEI;            /*!< Quadrature Encoder Interface Software Reset,                                       Address offset: 0x544          */
    __I  uint32_t RESERVED21[4];    /*!< RESERVED,                                                                          Address offset: 0x548 to 0x554 */
    __IO uint32_t SREEPROM;         /*!< EEPROM Software Reset,                                                             Address offset: 0x558          */
    __IO uint32_t SRWTIMER;         /*!< 32/64-Bit Wide General-Purpose Timer Software Reset,                               Address offset: 0x55C          */
    __I  uint32_t RESERVED22[40];   /*!< RESERVED,                                                                          Address offset: 0x560 to 0x5FC */
    __IO uint32_t RCGCWD;           /*!< Watchdog Timer Run Mode Clock Gating Control,                                      Address offset: 0x600          */
    __IO uint32_t RCGCTIMER;        /*!< 16/32-Bit General-Purpose Timer Run Mode Clock Gating Control,                     Address offset: 0x604          */
    __IO uint32_t RCGCGPIO;         /*!< General-Purpose Input/Output Run Mode Clock Gating Control,                        Address offset: 0x608          */
    __IO uint32_t RCGCDMA;          /*!< Micro Direct Memory Access Run Mode Clock Gating Control,                          Address offset: 0x60C          */
    __I  uint32_t RESERVED23;       /*!< RESERVED,                                                                          Address offset: 0x610          */
    __IO uint32_t RCGCHIB;          /*!< Hibernation Run Mode Clock Gating Control,                                         Address offset: 0x614          */
    __IO uint32_t RCGCUART;         /*!< Universal Asynchronous Receiver/Transmitter Run Mode Clock Gating Control,         Address offset: 0x618          */
    __IO uint32_t RCGCSSI;          /*!< Synchronous Serial Interface Run Mode Clock Gating Control,                        Address offset: 0x61C          */
    __IO uint32_t RCGCI2C;          /*!< Inter-Integrated Circuit Run Mode Clock Gating Control,                            Address offset: 0x620          */
    __I  uint32_t RESERVED24;       /*!< RESERVED,                                                                          Address offset: 0x624          */
    __IO uint32_t RCGCUSB;          /*!< Universal Serial Bus Run Mode Clock Gating Control,                                Address offset: 0x628          */
    __I  uint32_t RESERVED25[2];    /*!< RESERVED,                                                                          Address offset: 0x62C to 0x630 */
    __IO uint32_t RCGCCAN;          /*!< Controller Area Network Run Mode Clock Gating Control,                             Address offset: 0x634          */
    __IO uint32_t RCGCADC;          /*!< Analog-to-Digital Converter Run Mode Clock Gating Control,                         Address offset: 0x638          */
    __IO uint32_t RCGCACMP;         /*!< Analog Comparator Run Mode Clock Gating Control,                                   Address offset: 0x63C          */
    __IO uint32_t RCGCPWM;          /*!< Pulse Width Modulator Run Mode Clock Gating Control,                               Address offset: 0x640          */
    __IO uint32_t RCGCQEI;          /*!< Quadrature Encoder Interface Run Mode Clock Gating Control,                        Address offset: 0x644          */
    __I  uint32_t RESERVED26[4];    /*!< RESERVED,                                                                          Address offset: 0x648 to 0x654 */
    __IO uint32_t RCGCEEPROM;       /*!< EEPROM Run Mode Clock Gating Control,                                              Address offset: 0x658          */
    __IO uint32_t RCGCWTIMER;       /*!< 32/64-Bit Wide General-Purpose Timer Run Mode Clock Gating Control,                Address offset: 0x65C          */
    __I  uint32_t RESERVED27[40];   /*!< RESERVED,                                                                          Address offset: 0x660 to 0x6FC */
    __IO uint32_t SCGCWD;           /*!< Watchdog Timer Sleep Mode Clock Gating Control,                                    Address offset: 0x700          */
    __IO uint32_t SCGCTIMER;        /*!< 16/32-Bit General-Purpose Timer Sleep Mode Clock Gating Control,                   Address offset: 0x704          */
    __IO uint32_t SCGCGPIO;         /*!< General-Purpose Input/Output Sleep Mode Clock Gating Control,                      Address offset: 0x708          */
    __IO uint32_t SCGCDMA;          /*!< Micro Direct Memory Access Sleep Mode Clock Gating Control,                        Address offset: 0x70C          */
    __I  uint32_t RESERVED28;       /*!< RESERVED,                                                                          Address offset: 0x710          */
    __IO uint32_t SCGCHIB;          /*!< Hibernation Sleep Mode Clock Gating Control,                                       Address offset: 0x714          */
    __IO uint32_t SCGCUART;         /*!< Universal Asynchronous Receiver/Transmitter Sleep Mode Clock Gating Control,       Address offset: 0x718          */
    __IO uint32_t SCGCSSI;          /*!< Synchronous Serial Interface Sleep Mode Clock Gating Control,                      Address offset: 0x71C          */
    __IO uint32_t SCGCI2C;          /*!< Inter-Integrated Circuit Sleep Mode Clock Gating Control,                          Address offset: 0x720          */
    __I  uint32_t RESERVED29;       /*!< RESERVED,                                                                          Address offset: 0x724          */
    __IO uint32_t SCGCUSB;          /*!< Universal Serial Bus Sleep Mode Clock Gating Control,                              Address offset: 0x728          */
    __I  uint32_t RESERVED30[2];    /*!< RESERVED,                                                                          Address offset: 0x72C to 0x730 */
    __IO uint32_t SCGCCAN;          /*!< Controller Area Network Sleep Mode Clock Gating Control,                           Address offset: 0x734          */
    __IO uint32_t SCGCADC;          /*!< Analog-to-Digital Converter Sleep Mode Clock Gating Control,                       Address offset: 0x738          */
    __IO uint32_t SCGCACMP;         /*!< Analog Comparator Sleep Mode Clock Gating Control,                                 Address offset: 0x73C          */
    __IO uint32_t SCGCPWM;          /*!< Pulse Width Modulator Sleep Mode Clock Gating Control,                             Address offset: 0x740          */
    __IO uint32_t SCGCQEI;          /*!< Quadrature Encoder Interface Sleep Mode Clock Gating Control,                      Address offset: 0x744          */
    __I  uint32_t RESERVED31[4];    /*!< RESERVED,                                                                          Address offset: 0x748 to 0x754 */
    __IO uint32_t SCGCEEPROM;       /*!< EEPROM Sleep Mode Clock Gating Control,                                            Address offset: 0x758          */
    __IO uint32_t SCGCWTIMER;       /*!< 32/64-Bit Wide General-Purpose Timer Sleep Mode Clock Gating Control,              Address offset: 0x75C          */
    __I  uint32_t RESERVED32[40];   /*!< RESERVED,                                                                          Address offset: 0x760 to 0x7FC */
    __IO uint32_t DCGCWD;           /*!< Watchdog Timer Deep-Sleep Mode Clock Gating Control,                               Address offset: 0x800          */
    __IO uint32_t DCGCTIMER;        /*!< 16/32-Bit General-Purpose Timer Deep-Sleep Mode Clock Gating Control,              Address offset: 0x804          */
    __IO uint32_t DCGCGPIO;         /*!< General-Purpose Input/Output Deep-Sleep Mode Clock Gating Control,                 Address offset: 0x808          */
    __IO uint32_t DCGCDMA;          /*!< Micro Direct Memory Access Deep-Sleep Mode Clock Gating Control,                   Address offset: 0x80C          */
    __I  uint32_t RESERVED33;       /*!< RESERVED,                                                                          Address offset: 0x810          */
    __IO uint32_t DCGCHIB;          /*!< Hibernation Deep-Sleep Mode Clock Gating Control,                                  Address offset: 0x814          */
    __IO uint32_t DCGCUART;         /*!< Universal Asynchronous Receiver/Transmitter Deep-Sleep Mode Clock Gating Control,  Address offset: 0x818          */
    __IO uint32_t DCGCSSI;          /*!< Synchronous Serial Interface Deep-Sleep Mode Clock Gating Control,                 Address offset: 0x81C          */
    __IO uint32_t DCGCI2C;          /*!< Inter-Integrated Circuit Deep-Sleep Mode Clock Gating Control,                     Address offset: 0x820          */
    __I  uint32_t RESERVED34;       /*!< RESERVED,                                                                          Address offset: 0x824          */
    __IO uint32_t DCGCUSB;          /*!< Universal Serial Bus Deep-Sleep Mode Clock Gating Control,                         Address offset: 0x828          */
    __I  uint32_t RESERVED35[2];    /*!< RESERVED,                                                                          Address offset: 0x82C to 0x230 */
    __IO uint32_t DCGCCAN;          /*!< Controller Area Network Deep-Sleep Mode Clock Gating Control,                      Address offset: 0x834          */
    __IO uint32_t DCGCADC;          /*!< Analog-to-Digital Converter Deep-Sleep Mode Clock Gating Control,                  Address offset: 0x838          */
    __IO uint32_t DCGCACMP;         /*!< Analog Comparator Deep-Sleep Mode Clock Gating Control,                            Address offset: 0x83C          */
    __IO uint32_t DCGCPWM;          /*!< Pulse Width Modulator Deep-Sleep Mode Clock Gating Control,                        Address offset: 0x840          */
    __IO uint32_t DCGCQEI;          /*!< Quadrature Encoder Interface Deep-Sleep Mode Clock Gating Control,                 Address offset: 0x844          */
    __I  uint32_t RESERVED36[4];    /*!< RESERVED,                                                                          Address offset: 0x848 to 0x854 */
    __IO uint32_t DCGCEEPROM;       /*!< EEPROM Deep-Sleep Mode Clock Gating Control,                                       Address offset: 0x858          */
    __IO uint32_t DCGCWTIMER;       /*!< 32/64-Bit Wide General-Purpose Timer Deep-Sleep Mode Clock Gating Control          Address offset: 0x85C          */
    __I  uint32_t RESERVED37[104];  /*!< RESERVED,                                                                          Address offset: 0x860 to 0x9FC */
    __IO uint32_t PRWD;             /*!< Watchdog Timer Peripheral Ready,                                                   Address offset: 0xA00          */
    __IO uint32_t PRTIMER;          /*!< 16/32-Bit General-Purpose Timer Peripheral Ready,                                  Address offset: 0xA04          */
    __IO uint32_t PRGPIO;           /*!< General-Purpose Input/Output Peripheral Ready,                                     Address offset: 0xA08          */
    __IO uint32_t PRDMA;            /*!< Micro Direct Memory Access Peripheral Ready,                                       Address offset: 0xA0C          */
    __I  uint32_t RESERVED38;       /*!< RESERVED,                                                                          Address offset: 0xA10          */
    __IO uint32_t PRHIB;            /*!< Hibernation Peripheral Ready,                                                      Address offset: 0xA14          */
    __IO uint32_t PRUART;           /*!< Universal Asynchronous Receiver/Transmitter Peripheral Ready,                      Address offset: 0xA18          */
    __IO uint32_t PRSSI;            /*!< Synchronous Serial Interface Peripheral Ready,                                     Address offset: 0xA1C          */
    __IO uint32_t PRI2C;            /*!< Inter-Integrated Circuit Peripheral Ready,                                         Address offset: 0xA20          */
    __I  uint32_t RESERVED39;       /*!< RESERVED,                                                                          Address offset: 0xA24          */
    __IO uint32_t PRUSB;            /*!< Universal Serial Bus Peripheral Ready,                                             Address offset: 0xA28          */
    __I  uint32_t RESERVED40[2];    /*!< RESERVED,                                                                          Address offset: 0xA2C to 0x230 */
    __IO uint32_t PRCAN;            /*!< Controller Area Network Peripheral Ready,                                          Address offset: 0xA34          */
    __IO uint32_t PRADC;            /*!< Analog-to-Digital Converter Peripheral Ready,                                      Address offset: 0xA38          */
    __IO uint32_t PRACMP;           /*!< Analog Comparator Peripheral Ready,                                                Address offset: 0xA3C          */
    __IO uint32_t PRPWM;            /*!< Pulse Width Modulator Peripheral Ready,                                            Address offset: 0xA40          */
    __IO uint32_t PRQEI;            /*!< Quadrature Encoder Interface Peripheral Ready,                                     Address offset: 0xA44          */
    __I  uint32_t RESERVED41[4];    /*!< RESERVED,                                                                          Address offset: 0xA48 to 0xA54 */
    __IO uint32_t PREEPROM;         /*!< EEPROM Peripheral Ready,                                                           Address offset: 0xA58          */
    __IO uint32_t PRWTIMER;         /*!< 32/64-Bit Wide General-Purpose Timer Peripheral Ready,                             Address offset: 0xA5C          */

} SYSCTL_t;



/**
 * @brief System Control Legacy Register Structure
 */
typedef struct
{
    __I  uint32_t RESERVED[2];    /*!< RESERVED,                                         Address offset: 0x000 to 0x004 */
    __IO uint32_t DC0;            /*!< Device Capabilities 0,                            Address offset: 0x008          */
    __I  uint32_t RESERVED1;      /*!< RESERVED,                                         Address offset: 0x00C          */
    __IO uint32_t DC1;            /*!< Device Capabilities 1,                            Address offset: 0x010          */
    __IO uint32_t DC2;            /*!< Device Capabilities 2,                            Address offset: 0x014          */
    __IO uint32_t DC3;            /*!< Device Capabilities 3,                            Address offset: 0x018          */
    __IO uint32_t DC4;            /*!< Device Capabilities 4,                            Address offset: 0x01C          */
    __IO uint32_t DC5;            /*!< Device Capabilities 5,                            Address offset: 0x020          */
    __IO uint32_t DC6;            /*!< Device Capabilities 6,                            Address offset: 0x024          */
    __IO uint32_t DC7;            /*!< Device Capabilities 7,                            Address offset: 0x028          */
    __IO uint32_t DC8;            /*!< Device Capabilities 8                             Address offset: 0x02C          */
    __I  uint32_t RESERVED2[4];   /*!< RESERVED,                                         Address offset: 0x030 to 0x03C */
    __IO uint32_t SRCR0;          /*!< Software Reset Control 0,                         Address offset: 0x040          */
    __IO uint32_t SRCR1;          /*!< Software Reset Control 1,                         Address offset: 0x044          */
    __IO uint32_t SRCR2;          /*!< Software Reset Control 2,                         Address offset: 0x048          */
    __I  uint32_t RESERVED3[45];  /*!< RESERVED,                                         Address offset: 0x04C to 0x0FC */
    __IO uint32_t RCGC0;          /*!< Run Mode Clock Gating Control Register 0,         Address offset: 0x100          */
    __IO uint32_t RCGC1;          /*!< Run Mode Clock Gating Control Register 1,         Address offset: 0x104          */
    __IO uint32_t RCGC2;          /*!< Run Mode Clock Gating Control Register 2,         Address offset: 0x108          */
    __I  uint32_t RESERVED4;      /*!< RESERVED,                                         Address offset: 0x10C          */
    __IO uint32_t SCGC0;          /*!< Sleep Mode Clock Gating Control Register 0,       Address offset: 0x110          */
    __IO uint32_t SCGC1;          /*!< Sleep Mode Clock Gating Control Register 1,       Address offset: 0x114          */
    __IO uint32_t SCGC2;          /*!< Sleep Mode Clock Gating Control Register 2,       Address offset: 0x118          */
    __I  uint32_t RESERVED5;      /*!< RESERVED,                                         Address offset: 0x11C          */
    __IO uint32_t DCGC0;          /*!< Deep Sleep Mode Clock Gating Control Register 0,  Address offset: 0x120          */
    __IO uint32_t DCGC1;          /*!< Deep-Sleep Mode Clock Gating Control Register 1,  Address offset: 0x124          */
    __IO uint32_t DCGC2;          /*!< Deep Sleep Mode Clock Gating Control Register 2,  Address offset: 0x128          */
    __I  uint32_t RESERVED6[25];  /*!< RESERVED,                                         Address offset: 0x12C to 0x18C */
    __IO uint32_t DC9;            /*!< Device Capabilities 9,                            Address offset: 0x190          */
    __I  uint32_t RESERVED7[3];   /*!< RESERVED,                                         Address offset: 0x194 to 0x19C */
    __IO uint32_t NVMSTAT;        /*!< Non-Volatile Memory Information,                  Address offset: 0x1A0          */

} SYSCTL_LEGACY_t;




/**
 * @brief General-Purpose Input/Outputs
 * TODO complete Address offsets
 */
typedef struct
{
    __I  uint32_t BITMASK[255];  /*!< GPIO [9:2] register mask space, see Data Sheet,  Address offset: 0x000 */
    __IO uint32_t DATA;          /*!< GPIO Data,                                       Address offset: 0x000 */
    __IO uint32_t DIR;           /*!< GPIO Direction,                                  Address offset: 0x000 */
    __IO uint32_t IS;            /*!< GPIO Interrupt Sense,                            Address offset: 0x000 */
    __IO uint32_t IBE;           /*!< GPIO Interrupt Both Edges,                       Address offset: 0x000 */
    __IO uint32_t IEV;           /*!< GPIO Interrupt Event,                            Address offset: 0x000 */
    __IO uint32_t IM;            /*!< GPIO Interrupt Mask,                             Address offset: 0x000 */
    __IO uint32_t RIS;           /*!< GPIO Raw Interrupt Status,                       Address offset: 0x000 */
    __IO uint32_t MIS;           /*!< GPIO Masked Interrupt Status,                    Address offset: 0x000 */
    __O  uint32_t ICR;           /*!< GPIO Interrupt Clear,                            Address offset: 0x000 */
    __IO uint32_t AFSEL;         /*!< GPIO Alternate Function Select,                  Address offset: 0x000 */
    __I  uint32_t RESERVED[55];  /*!< RESERVED,                                        Address offset: 0x000 */
    __IO uint32_t DR2R;          /*!< GPIO 2-mA Drive Select,                          Address offset: 0x000 */
    __IO uint32_t DR4R;          /*!< GPIO 4-mA Drive Select,                          Address offset: 0x000 */
    __IO uint32_t DR8R;          /*!< GPIO 8-mA Drive Select,                          Address offset: 0x000 */
    __IO uint32_t ODR;           /*!< GPIO Open Drain Select,                          Address offset: 0x000 */
    __IO uint32_t PUR;           /*!< GPIO Pull-Up Select,                             Address offset: 0x000 */
    __IO uint32_t PDR;           /*!< GPIO Pull-Down Select,                           Address offset: 0x000 */
    __IO uint32_t SLR;           /*!< GPIO Slew Rate Control Select,                   Address offset: 0x000 */
    __IO uint32_t DEN;           /*!< GPIO Digital Enable,                             Address offset: 0x000 */
    __IO uint32_t LOCK;          /*!< GPIO Lock,                                       Address offset: 0x000 */
    __IO uint32_t CR;            /*!< GPIO Commit,                                     Address offset: 0x000 */
    __IO uint32_t AMSEL;         /*!< GPIO Analog Mode Select,                         Address offset: 0x000 */
    __IO uint32_t PCTL;          /*!< GPIO Port Control,                               Address offset: 0x000 */
    __IO uint32_t ADCCTL;        /*!< GPIO ADC Control,                                Address offset: 0x000 */
    __IO uint32_t DMACTL;        /*!< GPIO DMA Control,                                Address offset: 0x000 */

} GPIO_PORT_t;



/***************************************************************************/
/*                                                                         */
/*                       Device Peripheral Definitions                     */
/*                                                                         */
/***************************************************************************/


/**
 * @brief Peripheral Declarations
 */
#define GPIOA                    ((GPIO_PORT_t *) GPIOA_BASEADDR)
#define GPIOB                    ((GPIO_PORT_t *) GPIOB_BASEADDR)
#define GPIOC                    ((GPIO_PORT_t *) GPIOC_BASEADDR)
#define GPIOD                    ((GPIO_PORT_t *) GPIOD_BASEADDR)
#define GPIOE                    ((GPIO_PORT_t *) GPIOE_BASEADDR)
#define GPIOF                    ((GPIO_PORT_t *) GPIOF_BASEADDR)

#define GPIOA_AHB                ((GPIO_PORT_t *) GPIOA_AHB_BASEADDR)
#define GPIOB_AHB                ((GPIO_PORT_t *) GPIOB_AHB_BASEADDR)
#define GPIOC_AHB                ((GPIO_PORT_t *) GPIOC_AHB_BASEADDR)
#define GPIOD_AHB                ((GPIO_PORT_t *) GPIOD_AHB_BASEADDR)
#define GPIOE_AHB                ((GPIO_PORT_t *) GPIOE_AHB_BASEADDR)
#define GPIOF_AHB                ((GPIO_PORT_t *) GPIOF_AHB_BASEADDR)

#define SYSCTL                   ((SYSCTL_t *)SYSCTL_BASEADDR)
#define SYSCTL_LEGACY            ((SYSCTL_LEGACY_t*)SYSCTL_BASEADDR)





#endif /* MAIN_TM4C123GH6PM_H_ */