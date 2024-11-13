/*******************************************************************/
/* @file GPIO_prog.c
 * @author Ahmed Atef
 * @brief The GPIO main source file, including functions definitions
 */
#include <stdint.h>
#include <stdlib.h>
#include "STM32F446xx.h"
#include "ErrType.h"
#include "GPIO_private.h"
#include "GPIO_interface.h"

static GPIO_RegDef_t* GPIO_Port[GPIO_PERIPHERAL_NUM]={

		GPIOA,GPIOB,
		GPIOC,GPIOD,
		GPIOE,GPIOF,
		GPIOG,GPIOH
};




/*
 * @fn GPIO_u8PinInit
 * @brief Initializes a GPIO pin based on the provided configuration.
 * @param[in] PinConfig Pointer to a PinConfig_t structure that contains
 *                  the configuration information for the specified GPIO pin.
 * @return uint8_t Status of the initialization (0 = success, 1 = error)
 */
uint8_t GPIO_u8PinInit(const PinConfig_t* PinConfig)
{
	uint8_t Local_u8ErrorState = OK;

	    if (PinConfig != NULL)
	    {
	        if ((PinConfig->Port <= PORTH) && (PinConfig->PinNum <= PIN15))
	        {
	            // Configure Mode: Input, Output, Analog, or Alternate Function
	            (GPIO_Port[PinConfig->Port]->MODER) &= ~(MODER_MASK << (PinConfig->PinNum * 2));
	            (GPIO_Port[PinConfig->Port]->MODER) |= (PinConfig->Mode << (PinConfig->PinNum * 2));

	            // Configure Pull-up/Pull-down, for both Input and Output modes
	            (GPIO_Port[PinConfig->Port]->PUPDR) &= ~(PUPDR_MASK << (PinConfig->PinNum * 2));
	            (GPIO_Port[PinConfig->Port]->PUPDR) |= (PinConfig->PullType << (PinConfig->PinNum * 2));

	            // Configure Output Type and Speed (if Mode is Output or Alternate Function)
	            if (PinConfig->Mode == OUTPUT || PinConfig->Mode == ALTERNATE_FUNCTION)
	            {
	                // Set Output Type: PushPull or Open Drain
	                (GPIO_Port[PinConfig->Port]->OTYPER) &= ~(OTYPER_MASK << PinConfig->PinNum);
	                (GPIO_Port[PinConfig->Port]->OTYPER) |= (PinConfig->OutputType << PinConfig->PinNum);

	                // Set Output Speed: Low, Medium, High, or Very High
	                (GPIO_Port[PinConfig->Port]->OSPEEDR) &= ~(OSPEEDR_MASK << (PinConfig->PinNum * 2));
	                (GPIO_Port[PinConfig->Port]->OSPEEDR) |= (PinConfig->Speed << (PinConfig->PinNum * 2));

	                // If Mode is Alternate Function, configure Alternate Function Registers (AFR)
	                if (PinConfig->Mode == ALTERNATE_FUNCTION)
	                {
	                    uint8_t Local_RegNum = PinConfig->PinNum / 8;
	                    uint8_t Local_BitNum = PinConfig->PinNum % 8;
	                    (GPIO_Port[PinConfig->Port]->AFR[Local_RegNum]) &= ~(AFR_MASK << (Local_BitNum * 4));
	                    (GPIO_Port[PinConfig->Port]->AFR[Local_RegNum]) |= (PinConfig->AltFunc << (Local_BitNum * 4));
	                }
	            }
	        }
	        else
	        {
	            Local_u8ErrorState = NOK; // Invalid Port or PinNum
	        }
	    }
	    else
	    {
	        Local_u8ErrorState = NULL_PTR_ERR; // Null pointer passed
	    }

	    return Local_u8ErrorState;


}

/**
 * @brief Sets the value of a specific GPIO pin.
 * @param Port Port number to which the pin belongs (e.g., PORTA).
 * @param PinNum Pin number (e.g., PIN0, PIN1).
 * @param PinVal Value to set the pin (PIN_LOW or PIN_HIGH).
 * @return uint8_t Status of the operation (0 = success, 1 = error)
 */
uint8_t GPIO_u8SetPinValue(Port_t Port, Pin_t PinNum, PinVal_t PinVal)
{

		uint8_t Local_u8ErrorState;

		if((Port <=PORTH )&&(PinNum <=PIN15))
		{
			if(PinVal == PIN_LOW)
			{

				GPIO_Port[Port]->ODR &= ~(1<<PinNum); /* Read Modify Write Takes At Least 3 ClockCycles*/
				/*GPIO_Port[Port]->BSRR =1<<(16+PinNum)*/ /*Takes Only One ClockCycle*/

			}
			else if(PinVal == PIN_HIGH)
			{

				GPIO_Port[Port]->ODR |= (1<<PinNum);/* Read Modify Write Takes At Least 3 ClockCycles*/
				/*GPIO_Port[Port]->BSRR =1<<(PinNum) *//*Takes Only One ClockCycle*/
			}
			else
			{

				Local_u8ErrorState = NOK;

			}

		}
		else
		{

			Local_u8ErrorState=NOK;

		}





		return Local_u8ErrorState;



}

/**
 * @brief Toggles the value of a specific GPIO pin.
 * @param Port Port number to which the pin belongs.
 * @param PinNum Pin number to toggle.
 * @return uint8_t Status of the operation (0 = success, 1 = error)
 */
uint8_t GPIO_u8TogglePinValue(Port_t Port, Pin_t PinNum)
{

		uint8_t Local_u8ErrorState;

		if((Port <=PORTH )&&(PinNum <=PIN15))
		{

			GPIO_Port[Port]->ODR ^= (TOGGLE_BIT_MASK<<PinNum);

		}
		else
		{

			Local_u8ErrorState=NOK;

		}


		return Local_u8ErrorState;

}

/**
 * @brief Reads the value of a specific GPIO pin.
 * @param Port Port number to which the pin belongs.
 * @param PinNum Pin number to read.
 * @param PinVal Pointer to store the read pin value.
 * @return uint8_t Status of the operation (0 = success, 1 = error)
 */
uint8_t GPIO_u8ReadPinValue(Port_t Port, Pin_t PinNum, PinVal_t *PinVal)
{

		uint8_t Local_u8ErrorState=OK;


		if((Port <=PORTH )&&(PinNum <=PIN15))
		{


			*PinVal = ((GPIO_Port[Port]->IDR >> PinNum)&GET_BIT_MASK);


		}
		else
	    {

				Local_u8ErrorState=NOK;

		}


		return Local_u8ErrorState;
}
