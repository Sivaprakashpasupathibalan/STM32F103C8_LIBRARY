/***
******************************************************************************************************************************************************
******************************************************************************************************************************************************

   * @file           Blink.c
   * @author         Sivaprakash Pasupathibalan
   * @Updated date   04/07/2020       
   * @brief          Sample code to access the GPIO Library. Blinks the on board led in Blue pill Board
		   
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
	
*******************************************************************************************************************************************************
*******************************************************************************************************************************************************
***/
#include<stdio.h>                                           /**      Standary Library Functions   **/
#include "STM32F103C8.h"                                    /**      Header files for GPIO and peripheral access registers  **/
#include "STM32F103C8_GPIO.h"

extern void initialise_monitor_handles(void);               /** Semi hosting feature intialisation prototype  **/


int main()                                                 
{
  initialise_monitor_handles();
  /** Enable Clock for PORTC Module in the RCC Clock Gating Control **/  
  RCC->APB2ENR |= (1 << IOPCEN);                           
  /** Setting GPIO Mode **/
  GPIO_MODE(&(PORTC->CRL) , PIN_13 , OUTPUT_MODE_2Mhz); 
  /** Setting Configuration to OPEN_DRAIN **/  
  GPIO_CNF(&(PORTC->CRL) , PIN_13 , OPEN_DRAIN);           
  while(1)                /** Forever **/
  {
	/** printf for semi hosting feature not necessary **/
	//printf("LED IS OFF\n");
	/** sets the ODR Register  **/
	GPIO_DATA_SET(&(PORTC->ODR), PIN_13 , 1);
	/** Delay **/
    for(int i = 0; i < 100000; i++);
	//printf("LED IS ON\n");
	/** Clears the ODR register **/
	GPIO_DATA_SET(&(PORTC->ODR), PIN_13 , 0);
	/** Delay **/
    for(int i = 0; i < 100000; i++);
  }  
}

