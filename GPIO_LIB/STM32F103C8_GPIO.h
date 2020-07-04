/***
******************************************************************************************************************************************************
******************************************************************************************************************************************************

   * @file           STM32F103C8_GPIO.h
   * @author         Sivaprakash Pasupathibalan
   * @Updated date   04/07/2020       
   * @brief          GPIO Library for Cortex-M3 MediumDensity Board
           * Functions to set Mode of GPIO and Configurations
		   * Macro definitions for Mode and configuration values
		   * Functions to Read , Set , Reset the registers
		   
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
	
*******************************************************************************************************************************************************
*******************************************************************************************************************************************************
***/
#ifndef STM32F103C8_GPIO_H
#define STM32F103C8_GPIO_H
#include <stdint.h>                                /**   MACRO Definition Library for Code Portability **/
#include "STM32F103C8.h"                           /** Include file for Device peripheral registers and Macros for Cortex-M3 MediumDensity Board  **/

/** GPIO MODES **/
#define INPUT_MODE 00
#define OUTPUT_MODE_10Mhz 01
#define OUTPUT_MODE_2Mhz  10
#define OUTPUT_MODE_50Mhz 11

/** GPIO Configuration in Input Mode **/
#define ANALOG_MODE 00
#define FLOATING_INPUT 01
#define PULLUP_PULLDOWN 10
#define Reserved 11

/** GPIO Configuration in Output Mode **/
#define PUSH_PULL 00
#define OPEN_DRAIN 01
#define AL_PUSH_PULL 10
#define AL_OPEN_DRAIN 11

/** Prototype for the Library Functions for GPIO    **/
void GPIO_MODE(volatile uint32_t* REG, uint8_t PIN , uint8_t MODE);
void GPIO_CNF(volatile uint32_t* REG, uint8_t PIN , uint8_t CNF);   
uint8_t GPIO_READ(volatile uint32_t* REG , uint8_t PIN );                      
void GPIO_DATA_SET(volatile uint32_t* REG, uint8_t PIN ,uint8_t Status);                              
void GPIO_RESET_BSRR(volatile uint32_t* REG, uint8_t PIN , uint8_t Status);                      
void GPIO_SET_BSRR(volatile uint32_t* REG, uint8_t PIN , uint8_t Status);                     
void GPIO_CNF_PIN_LCK(volatile uint32_t* REG , uint8_t PIN , uint8_t Status);                      
void GPIO_CNF_LCKK(volatile uint32_t* REG);                                                 

/**
**************************************************************************************************************
      @brief    * This function Locks the GPIO Modes and Configuration set
	            * While this bit is set it Locks the mode and config until next reset
				
	  @params   * It takes the port register as parameter 
	  
	  @example  *  GPIO_CNF_LCKK(&(PORTA->LCKR));
**************************************************************************************************************
**/
void GPIO_CNF_LCKK(volatile uint32_t* REG) {*REG |= (1 << 16);}   	

/**
**************************************************************************************************************
      @brief    * This function Locks the GPIO Modes and Configuration set for the pins
	            * While this bit is set in the register it Locks the mode and config until next reset when the 
				  void GPIO_CNF_LCKK(volatile uint32_t* REG) is also set
				
	  @params   * It takes the port register as parameter , PIN Macro , Status (0 - default ,1 - Lock) 
	  
	  @example  *  GPIO_CNF_PIN_LCK(&(PORTA->LCKR), PIN_0 , 1);   the particular bit will be set
**************************************************************************************************************
**/
void GPIO_CNF_PIN_LCK(volatile uint32_t* REG , uint8_t PIN , uint8_t Status)
{
    if(Status == 0 | Status == 1)
	{
	*REG &= ~(1 << PIN);
	*REG |= (Status << PIN);
	}  	
}

/**
**************************************************************************************************************
      @brief    * This function Resets the Output data Register
	            * While this bit is set for the pin it resets the corresponding ODR register
				
	  @params   * It takes the port register as parameter , PIN Macro , Status (0 - default ,1 - reset) 
	  
	  @example  *  GPIO_RESET_BSRR(&(PORTA->BSRR), PIN_0 , 1);   the particular bit will be set
**************************************************************************************************************
**/
void GPIO_RESET_BSRR(volatile uint32_t* REG, uint8_t PIN , uint8_t Status)
{
	PIN = PIN - 16;
    if(Status == 0 | Status == 1)
	{
	*REG &= ~(1 << PIN);
	*REG |= (Status << PIN);
	}
}

/**
**************************************************************************************************************
      @brief    * This function sets the Output data Register
	            * While this bit is set for the pin it sets the corresponding ODR register
				
	  @params   * It takes the port register as parameter , PIN Macro , Status (0 - default ,1 - set) 
	  
	  @example  *  GPIO_SET_BSRR(&(PORTA->BSRR), PIN_0 , 1);   the particular bit will be set
**************************************************************************************************************
**/
void GPIO_SET_BSRR(volatile uint32_t* REG, uint8_t PIN , uint8_t Status)
{
    if(Status == 0 | Status == 1)
	{
	*REG &= ~(1 << PIN);
	*REG |= (Status << PIN);
	}
}

/**
**************************************************************************************************************
      @brief    * This function sets the  bits in Output data Register
	            * While this bit is set for the pin it sets the corresponding ODR register bits
				
	  @params   * It takes the port register as parameter , PIN Macro , Status (0 - default ,1 - set) 
	  
	  @example  *  GPIO_DATA_SET(&(PORTA->ODR), PIN_0 , 1);   the particular bit will be set
**************************************************************************************************************
**/
void GPIO_DATA_SET(volatile uint32_t* REG, uint8_t PIN ,uint8_t Status)
{
	if(Status == 0 | Status == 1)
	{
	*REG &= ~(1 << PIN);
	*REG |= (Status << PIN);
	}
}

/**
**************************************************************************************************************
      @brief    * This reads the data from Input data register 
	            * While this bit is set it returns 1 else 0
				
	  @params   * It takes the port register as parameter , PIN Macro
	  
	  @example  *  GPIO_READ(&(PORTA->IDR), PIN_0);   the status of the bit will be returned
**************************************************************************************************************
**/
uint8_t GPIO_READ(volatile uint32_t* REG , uint8_t PIN )                                             
{
	if((*REG && (1 << PIN)) == 1) return 1;
	else return 0;
}

/**
**************************************************************************************************************
      @brief    * This set the mode of the pin in the register
				
	  @params   * It takes the port register as parameter , PIN Macro , Macro of modes 
	  
	  @example  *  GPIO_MODE(&(PORTA->CRL), PIN_0 , OUTPUT_MODE_50Mhz ); //sets the mode of pin
**************************************************************************************************************
**/
void GPIO_MODE(volatile uint32_t* REG, uint8_t PIN , uint8_t MODE)
{
	uint8_t MODE_0 = MODE / 10;
	uint8_t MODE_1 = MODE % 10;
    if(PIN <= 7 && PIN >= 0)
	{
		if(PIN == 0)
		{
			*REG &= ~( (1 << 0) | (1 << 1) );             
			*REG |= (MODE_1 << 0);
			*REG |= (MODE_0 << 1);
	    }
		else
		{
		    *REG &= ~( (1 << (PIN + (3 * PIN))) | (1 << ((PIN + (3 * PIN)) + 1)));              
			*REG |= (MODE_1 << (PIN + (3 * PIN))); 
			*REG |=  (MODE_0 << ((PIN + (3 * PIN)) + 1));
		}
	}
	else if(PIN >= 8 && PIN <= 15)
	{
		REG = REG + 1;
		PIN = PIN - 8;
		if(PIN == 0)
		{
			*REG &= ~( (1 << 0) | (1 << 1) );          
			*REG |= (MODE_1 << 0);
			*REG |= (MODE_0 << 1);
	    }
		else
		{
		    *REG &= ~( (1 << (PIN + (3 * PIN))) | (1 << ((PIN + (3 * PIN)) + 1)));      
			*REG |= (MODE_1 << (PIN + (3 * PIN))); 
			*REG |=  (MODE_0 << ((PIN + (3 * PIN)) + 1));
		}
	}
}

/**
**************************************************************************************************************
      @brief    * This set the configuration of input or output of the pin in the register
				
	  @params   * It takes the port register as parameter , PIN Macro , Macro of configuration
	  
	  @example  *  GPIO_MODE(&(PORTA->CRL), PIN_0 , OPEN_DRAIN ); //sets the configuration of pin
**************************************************************************************************************
**/
void GPIO_CNF(volatile uint32_t* REG, uint8_t PIN , uint8_t CNF)
{
	uint8_t CNF_0 = CNF / 10;
	uint8_t CNF_1 = CNF % 10;
    if(PIN <= 7 && PIN >= 0)
	{
		if(PIN == 0)
		{
			*REG &= ~( (1 << 2) | (1 << 3) );                                                   
			*REG |= (CNF_1 << 2);
			*REG |= (CNF_0 << 3);
		}
		else
		{
			*REG &= ~( (1 << ((PIN + (3 * PIN))  + 2)) | (1 << (((PIN + (3 * PIN)) + 2) + 1)) );
			*REG |= (CNF_1 << ((PIN + (3 * PIN))  + 2));
			*REG |=  (CNF_0 << (((PIN + (3 * PIN)) + 2) + 1));
		}
	}
	else if(PIN >= 8 && PIN <= 15)
	{
		REG = REG + 1;
		PIN = PIN - 8;
		if(PIN == 0)
		{
			*REG &= ~( (1 << 2) | (1 << 3) );                                                  
			*REG |= (CNF_1 << 2);
			*REG |= (CNF_0 << 3);
		}
		else
		{
			*REG &= ~( (1 << ((PIN + (3 * PIN))  + 2)) | (1 << (((PIN + (3 * PIN)) + 2) + 1)) ); 
			*REG |= (CNF_1 << ((PIN + (3 * PIN))  + 2));
			*REG |=  (CNF_0 << (((PIN + (3 * PIN)) + 2) + 1));
		}
	}
}

#endif