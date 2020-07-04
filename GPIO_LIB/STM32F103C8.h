/***
******************************************************************************************************************************************************
******************************************************************************************************************************************************

   * @file           STM32F103C8.h
   * @author         Sivaprakash Pasupathibalan
   * @Updated date   04/07/2020       
   * @brief          Device peripheral registers and Macros for Cortex-M3 MediumDensity Board
           * Register Bit declarations
		   * Structures for register access
		   * Last updated for RCC and GPIO
		   
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
	
*******************************************************************************************************************************************************
*******************************************************************************************************************************************************
***/
#ifndef STM32F103C8_H
#define STM32F103C8_H
#include <stdint.h>                        /**   MACRO Definition Library for Code Portability **/

/**   MACRO RCC in CORTEX-M3 base address to structure **/
#define RCC       ((volatile RCC_typedef*)0x40021000U)

/**   MACRO for available PORTs in CORTEX-M3 base address to structure **/
#define PORTA     ((volatile GPIO_typedef*)0x40010800U)                              
#define PORTB     ((volatile GPIO_typedef*)0x40010C00U)
#define PORTC     ((volatile GPIO_typedef*)0x40011000U)
#define PORTD     ((volatile GPIO_typedef*)0x40011400U)
#define PORTE     ((volatile GPIO_typedef*)0x40011800U)


/**   MACRO for RCC_CR register bits **/
#define PLLRDY 25
#define PLLON  24
#define CSSON  19
#define HSEBYP 18
#define HSERDY 17
#define HSEON  16
#define HSICAL 15           //15 - 8
#define HSITRIM 7           //7 - 3
#define HSIRDY  1
#define HSION   0

/**   MACRO for RCC_CFGR register bits **/
#define MCO  26 //26-24
#define USBPRE 22
#define PLLMUL 21 //21 -18
#define PLLXTPRE 17
#define PLLSRC 16
#define ADCPRE 15 //15 -14
#define PPRE2  13 //13 -11
#define PPRE1  10 //10 - 8
#define HPRE   7  //7 - 4
#define SWS    3  // 3 - 2
#define SW     1  // 1 - 0

/**   MACRO for RCC_CIR register bits **/
#define CSSC   23
#define PLLRDYC 20
#define HSERDYC 19
#define HSIRDYC 18
#define LSERDYC 17
#define LSIRDYC 16
#define PLLRDYIE 12
#define HSERDYIE 11
#define HSIRDYIE 10
#define LSERDYIE 9
#define LSIRDYIE 8
#define CSSF 7
#define PLLRDYF 4
#define HSERDYF 3
#define HSIRDYF 2
#define LSERDYF 1
#define LSIRDYF 0

/**   MACRO for RCC_APB2RSTR register bits **/
#define TIM11RST 21
#define TIM10RST 20
#define TIM9RST 19
#define ADC3RST 15
#define USART1RST 14
#define TIM8RST 13
#define SPI1RST 12
#define TIM1RST 11
#define ADC2RST 10
#define ADC1RST 9
#define IOPGRST 8
#define IOPFRST 7
#define IOPERST 6
#define IOPDRST 5
#define IOPCRST 4
#define IOPBRST 3
#define IOPARST 2
#define AFIORST 0

/**   MACRO for RCC_APB1RSTR register bits **/
#define DACRST 29
#define PWRRST 28
#define BKPRST 27
#define CANRST 25
#define USBRST 23
#define I2C2RST 22
#define I2C1RST 21
#define UART5RST 20
#define UART4RST 19
#define USART3RST 18
#define USART2RST 17
#define SPI3RST 15
#define SPI2RST 14
#define WWDGRST 11
#define TIM14RST 8
#define TIM13RST 7
#define TIM12RST 6
#define TIM7RST 5
#define TIM6RST 4
#define TIM5RST 3
#define TIM4RST 2
#define TIM3RST 1
#define TIM2RST 0

/**   MACRO for RCC_AHBENR register bits **/
#define SDIOEN 10
#define FSMCEN 8
#define CRCEN 6
#define FLITFEN 4
#define SRAMEN 2
#define DMA2EN 1
#define DMA1EN 0

/**   MACRO for RCC_APB2ENR register bits **/
#define TIM11EN 21
#define TIM10EN 20
#define TIM9EN 19
#define ADC3EN 15
#define USART1EN 14
#define TIM8EN 13
#define SPI1EN 12
#define TIM1EN 11
#define ADC2EN 10
#define ADC1EN 9
#define IOPGEN 8
#define IOPFEN 7
#define IOPEEN 6
#define IOPDEN 5
#define IOPCEN 4
#define IOPBEN 3
#define IOPAEN 2
#define AFIOEN 0

/**   MACRO for RCC_APB1ENR register bits **/
#define DACEN 29
#define PWREN 28
#define BKPEN 27
#define CANEN 25
#define USBEN 23
#define I2C2EN 22
#define I2C1EN 21
#define UART5EN 20
#define UART4EN 19
#define USART3EN 18
#define USART2EN 17
#define SPI3EN 15
#define SPI2EN 14
#define WWDGEN 11
#define TIM14EN 8
#define TIM13EN 7
#define TIM12EN 6
#define TIM7EN 5
#define TIM6EN 4
#define TIM5EN 3
#define TIM4EN 2
#define TIM3EN 1
#define TIM2EN 0

/**   MACRO for RCC_BDCR register bits **/
#define BDRST 16
#define RTCEN 15
#define RTCSEL 9  // 9 - 8
#define LSEBYP 2
#define LSERDY 1
#define LSEON 0

/**   MACRO for RCC_CSR register bits **/
#define LPWRRSTF 31
#define WWDGRSTF 30
#define IWDGRSTF 29
#define SFTRSTF 28
#define PORRSTF 27
#define PINRSTF 26
#define RMVF 24
#define LSIRDY 1
#define LSION 0

/**   MACRO for GPIO_PIN **/
#define PIN_0  0
#define PIN_1  1
#define PIN_2  2
#define PIN_3  3
#define PIN_4  4
#define PIN_5  5
#define PIN_6  6
#define PIN_7  7
#define PIN_8  8
#define PIN_9  9
#define PIN_10  10
#define PIN_11  11
#define PIN_12  12
#define PIN_13  13
#define PIN_14  14
#define PIN_15  15

/** Structure for RCC Control Registers  **/
typedef struct{
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
}RCC_typedef;

/**  Structure for GPIO Registers  **/
typedef struct{
	volatile uint32_t CRL;
	volatile uint32_t CRH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;	
}GPIO_typedef;

#endif