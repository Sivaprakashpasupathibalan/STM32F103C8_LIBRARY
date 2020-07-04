/***
******************************************************************************************************************************************************
******************************************************************************************************************************************************

   * @file           STM32F103C8_Startup.c
   * @author         Sivaprakash Pasupathibalan
   * @Updated date   04/07/2020       
   * @brief          Startup file for Cortex-M3 MediumDensity Board
           * Vector Table for Handlers and IRQ of STM32F103C8 Blue pill Board
		   * Protoype for Handlers and other function
		   * Allocate memory address from Linker Script
		   * Copy data from Flash to SRAM and initialization of the Address in .bss segment
		   
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
	
*******************************************************************************************************************************************************
*******************************************************************************************************************************************************
***/
#include<stdint.h>                                   /**   MACRO Definition Library for Code Portability **/

/**  SRAM Address and Size Definitions for STM32F103C8 with 20KB of SRAM  **/
#define SRAM_START  0x20000000U
#define SRAM_SIZE   (20 * 1024)   
#define SRAM_END    (SRAM_START + SRAM_SIZE)
#define STACK_START SRAM_END


/** extern declarations of Variables from the Linker Script to get segment address **/
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _ld_data;

/** main function prototype declaration to pass the exection from reset handler to main function in user program  **/
int main(void);

/** Protoype declaration for Standard C functions **/
void __libc_init_array(void);


/***
******************************************************************************************************************************************
******************************************************************************************************************************************
       @brief     * Vector table prototype declarations for Handlers and IRQ in CORTEX-M3 STM32F103C8 Medium density Board
	              * Reset_Handler Prototype and Default_Handler prototype is provided
				  * All the Handlers and IRQ except Reset_Handler is mapped to the default Handler but set to weak 
				    so user can override the prototype in user program.
******************************************************************************************************************************************
******************************************************************************************************************************************
***/
/** Protoype declaration for Handlers **/
void Reset_Handler(void);
void Default_Handler(void);
void NMI_Handler(void)          __attribute__(( weak , alias("Default_Handler")));
void HardFault_Handler(void)    __attribute__(( weak , alias("Default_Handler")));
void MemManage_Handler(void)    __attribute__(( weak , alias("Default_Handler")));
void BusFault_Handler(void)     __attribute__(( weak , alias("Default_Handler")));
void UsageFault_Handler(void)   __attribute__(( weak , alias("Default_Handler")));
void SVCall_Handler(void)       __attribute__(( weak , alias("Default_Handler")));
void DebugMonitor_Handler(void) __attribute__(( weak , alias("Default_Handler")));
void PendSV_Handler(void)       __attribute__(( weak , alias("Default_Handler")));
void SysTick_Handler(void)      __attribute__(( weak , alias("Default_Handler")));

/** Protoype declaration for IRQ Handlers **/
void WWDG_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void PVD_IRQ_Handler(void)               __attribute__(( weak , alias("Default_Handler")));
void TAMPER_IRQ_Handler(void)            __attribute__(( weak , alias("Default_Handler")));
void RTC_IRQ_Handler(void)               __attribute__(( weak , alias("Default_Handler")));
void FLASH_IRQ_Handler(void)             __attribute__(( weak , alias("Default_Handler")));
void RCC_IRQ_Handler(void)               __attribute__(( weak , alias("Default_Handler")));
void EXTI0_IRQ_Handler(void)             __attribute__(( weak , alias("Default_Handler")));
void EXTI1_IRQ_Handler(void)             __attribute__(( weak , alias("Default_Handler")));
void EXTI2_IRQ_Handler(void)             __attribute__(( weak , alias("Default_Handler")));
void EXTI3_IRQ_Handler(void)             __attribute__(( weak , alias("Default_Handler")));
void EXTI4_IRQ_Handler(void)             __attribute__(( weak , alias("Default_Handler")));
void DMA1_Channel1_IRQ_Handler(void)     __attribute__(( weak , alias("Default_Handler")));
void DMA1_Channel2_IRQ_Handler(void)     __attribute__(( weak , alias("Default_Handler")));
void DMA1_Channel3_IRQ_Handler(void)     __attribute__(( weak , alias("Default_Handler")));
void DMA1_Channel4_IRQ_Handler(void)     __attribute__(( weak , alias("Default_Handler")));
void DMA1_Channel5_IRQ_Handler(void)     __attribute__(( weak , alias("Default_Handler")));
void DMA1_Channel6_IRQ_Handler(void)     __attribute__(( weak , alias("Default_Handler")));
void DMA1_Channel7_IRQ_Handler(void)     __attribute__(( weak , alias("Default_Handler")));
void ADC1_2_IRQ_Handler(void)            __attribute__(( weak , alias("Default_Handler")));
void USB_HP_CAN_TX_IRQ_Handler(void)     __attribute__(( weak , alias("Default_Handler")));
void USB_LP_CAN_RX0_IRQ_Handler(void)    __attribute__(( weak , alias("Default_Handler")));
void CAN_RX1_IRQ_Handler(void)           __attribute__(( weak , alias("Default_Handler")));
void CAN_SCE_IRQ_Handler(void)           __attribute__(( weak , alias("Default_Handler")));
void EXTI9_5_IRQ_Handler(void)           __attribute__(( weak , alias("Default_Handler")));
void TIM1_BRK_IRQ_Handler(void)          __attribute__(( weak , alias("Default_Handler")));
void TIM1_UP_IRQ_Handler(void)           __attribute__(( weak , alias("Default_Handler")));
void TIM1_TRG_COM_IRQ_Handler(void)      __attribute__(( weak , alias("Default_Handler")));
void TIM1_CC_IRQ_Handler(void)           __attribute__(( weak , alias("Default_Handler")));
void TIM2_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void TIM3_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void TIM4_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void I2C1_EV_IRQ_Handler(void)           __attribute__(( weak , alias("Default_Handler")));
void I2C1_ER_IRQ_Handler(void)           __attribute__(( weak , alias("Default_Handler")));
void I2C2_EV_IRQ_Handler(void)           __attribute__(( weak , alias("Default_Handler")));
void I2C2_ER_IRQ_Handler(void)           __attribute__(( weak , alias("Default_Handler")));
void SPI1_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void SPI2_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void USART1_IRQ_Handler(void)            __attribute__(( weak , alias("Default_Handler")));
void USART2_IRQ_Handler(void)            __attribute__(( weak , alias("Default_Handler")));
void USART3_IRQ_Handler(void)            __attribute__(( weak , alias("Default_Handler")));
void EXTI15_10_IRQ_Handler(void)         __attribute__(( weak , alias("Default_Handler")));
void RTCAlarm_IRQ_Handler(void)          __attribute__(( weak , alias("Default_Handler")));
void USBWakeup_IRQ_Handler(void)         __attribute__(( weak , alias("Default_Handler")));
void TIM8_BRK_IRQ_Handler(void)          __attribute__(( weak , alias("Default_Handler")));
void TIM8_UP_IRQ_Handler(void)           __attribute__(( weak , alias("Default_Handler")));
void TIM8_TRG_COM_IRQ_Handler(void)      __attribute__(( weak , alias("Default_Handler")));
void TIM8_CC_IRQ_Handler(void)           __attribute__(( weak , alias("Default_Handler")));
void ADC3_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void FSMC_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void SDIO_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void TIM5_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void SPI3_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void UART4_IRQ_Handler(void)             __attribute__(( weak , alias("Default_Handler")));
void UART5_IRQ_Handler(void)             __attribute__(( weak , alias("Default_Handler")));
void TIM6_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void TIM7_IRQ_Handler(void)              __attribute__(( weak , alias("Default_Handler")));
void DMA2_Channel1_IRQ_Handler(void)     __attribute__(( weak , alias("Default_Handler")));
void DMA2_Channel2_IRQ_Handler(void)     __attribute__(( weak , alias("Default_Handler")));
void DMA2_Channel3_IRQ_Handler(void)     __attribute__(( weak , alias("Default_Handler")));
void DMA2_Channel4_5_IRQ_Handler(void)   __attribute__(( weak , alias("Default_Handler")));

/***
***********************************************************************************************************************************************
***********************************************************************************************************************************************
        @brief * Vector array of the Handlers and IRQ
		       * Address of the Handler functions to the array values reserved values are provided 0
			   * creating a seperate .isr_vector section in the object file
			   * placing the file before .text segment in the flash so the address will be mapped.
***********************************************************************************************************************************************
***********************************************************************************************************************************************
***/
uint32_t Vectors [] __attribute__((section(".isr_vector"))) = {
	STACK_START,
	(uint32_t)(&Reset_Handler),
	(uint32_t)(&NMI_Handler),
	(uint32_t)(&HardFault_Handler),
	(uint32_t)(&MemManage_Handler),
	(uint32_t)(&BusFault_Handler),
	(uint32_t)(&UsageFault_Handler),
	0,
	(uint32_t)(&SVCall_Handler),
	(uint32_t)(&DebugMonitor_Handler),
	0,
	(uint32_t)(&PendSV_Handler),
	(uint32_t)(&SysTick_Handler),
	(uint32_t)(&WWDG_IRQ_Handler),
	(uint32_t)(&PVD_IRQ_Handler),
	(uint32_t)(&TAMPER_IRQ_Handler),
	(uint32_t)(&RTC_IRQ_Handler),
	(uint32_t)(&FLASH_IRQ_Handler),
	(uint32_t)(&RCC_IRQ_Handler),
	(uint32_t)(&EXTI0_IRQ_Handler),
	(uint32_t)(&EXTI1_IRQ_Handler),
	(uint32_t)(&EXTI2_IRQ_Handler),
	(uint32_t)(&EXTI3_IRQ_Handler),
	(uint32_t)(&EXTI4_IRQ_Handler),
	(uint32_t)(&DMA1_Channel1_IRQ_Handler),
	(uint32_t)(&DMA1_Channel2_IRQ_Handler),
	(uint32_t)(&DMA1_Channel3_IRQ_Handler),
	(uint32_t)(&DMA1_Channel4_IRQ_Handler),
	(uint32_t)(&DMA1_Channel5_IRQ_Handler),
	(uint32_t)(&DMA1_Channel6_IRQ_Handler),
	(uint32_t)(&DMA1_Channel7_IRQ_Handler),
	(uint32_t)(&ADC1_2_IRQ_Handler),
	(uint32_t)(&USB_HP_CAN_TX_IRQ_Handler),  
	(uint32_t)(&USB_LP_CAN_RX0_IRQ_Handler),
	(uint32_t)(&CAN_RX1_IRQ_Handler),
	(uint32_t)(&CAN_SCE_IRQ_Handler),
	(uint32_t)(&EXTI9_5_IRQ_Handler),
	(uint32_t)(&TIM1_BRK_IRQ_Handler),
	(uint32_t)(&TIM1_UP_IRQ_Handler),
	(uint32_t)(&TIM1_TRG_COM_IRQ_Handler),
	(uint32_t)(&TIM1_CC_IRQ_Handler),
	(uint32_t)(&TIM2_IRQ_Handler),
	(uint32_t)(&TIM3_IRQ_Handler),
	(uint32_t)(&TIM4_IRQ_Handler),
	(uint32_t)(&I2C1_EV_IRQ_Handler),
	(uint32_t)(&I2C1_ER_IRQ_Handler),
	(uint32_t)(&I2C2_EV_IRQ_Handler),
	(uint32_t)(&I2C2_ER_IRQ_Handler),
	(uint32_t)(&SPI1_IRQ_Handler),
	(uint32_t)(&SPI2_IRQ_Handler),
	(uint32_t)(&USART1_IRQ_Handler),
	(uint32_t)(&USART2_IRQ_Handler),
	(uint32_t)(&USART3_IRQ_Handler),
	(uint32_t)(&EXTI15_10_IRQ_Handler),
	(uint32_t)(&RTCAlarm_IRQ_Handler),
	(uint32_t)(&USBWakeup_IRQ_Handler),
	(uint32_t)(&TIM8_BRK_IRQ_Handler),
	(uint32_t)(&TIM8_UP_IRQ_Handler),
	(uint32_t)(&TIM8_TRG_COM_IRQ_Handler),
	(uint32_t)(&TIM8_CC_IRQ_Handler),
	(uint32_t)(&ADC3_IRQ_Handler),
	(uint32_t)(&FSMC_IRQ_Handler),
	(uint32_t)(&SDIO_IRQ_Handler),
	(uint32_t)(&TIM5_IRQ_Handler),
	(uint32_t)(&SPI3_IRQ_Handler),
	(uint32_t)(&UART4_IRQ_Handler),
	(uint32_t)(&UART5_IRQ_Handler),
	(uint32_t)(&TIM6_IRQ_Handler),
	(uint32_t)(&TIM7_IRQ_Handler),
	(uint32_t)(&DMA2_Channel1_IRQ_Handler),
	(uint32_t)(&DMA2_Channel2_IRQ_Handler),
	(uint32_t)(&DMA2_Channel3_IRQ_Handler),
	(uint32_t)(&DMA2_Channel4_5_IRQ_Handler)	
};
/***
***********************************************************************************************************************************************
***********************************************************************************************************************************************
        @brief * Reset Handler function
		       * copy data segment from flash to SRAM
			   * initialization of bss address values to Zero
			   * initialization of standard C Functions
			   * Transfer execution to main function in user program
***********************************************************************************************************************************************
***********************************************************************************************************************************************
***/
void Reset_Handler(void)
{
	uint32_t Size = (uint32_t)(&_edata) - (uint32_t)(&_sdata);
	uint8_t *pDest = (uint8_t*)&_sdata;
	uint8_t *pSrc = (uint8_t*)&_ld_data; //Flash
	for(uint32_t i = 0; i < Size; i++)
	{
	   *pDest++ = *pSrc++;	
	}
	Size = (uint32_t)(&_ebss) - (uint32_t)(&_sbss);
	pDest = (uint8_t*)&_sbss;
	for(uint32_t i = 0; i < Size; i++) *pDest++ = 0;
	__libc_init_array();
	main();
}

/** default handler user can override in user program for the Handlers **/
void Default_Handler(void)
{
	while(1);           /** DO NOTHING **/
}