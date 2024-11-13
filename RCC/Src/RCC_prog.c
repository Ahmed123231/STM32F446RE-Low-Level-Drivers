#include <stdint.h>
#include "RCC_private.h"
#include "STM32F446xx.h"

#define RCC     ((RCC_RegDef_t*)RCC_BASE_ADDRESS)

/**
 * @brief Sets the status of the specified clock.
 *
 * This function enables or disables a specific clock based on the provided status.
 *
 * @param Clk_Type The type of clock to set (HSI, HSE, PLL, etc.).
 * @param Status The status to set for the clock (ON, OFF).
 * @return uint8_t Returns 0 on success, 1 if the clock type is invalid.
 */
uint8_t RCC_SetClkStatus(CLK_t Clk_Type, STATUS_t Status) {
    // Check if the clock type is within a valid range (HSI, HSE, PLL, etc.)



	if (Status == ON) {
	    switch (Clk_Type) {
	        case HSI:
	            RCC->CR |= (1 << 0); // Enable HSI (bit 0)
	            break;
	        case HSE:
	            RCC->CR |= (1 << 16); // Enable HSE (bit 16)
	            break;
	        case PLL:
	            RCC->CR |= (1 << 24); // Enable PLL (bit 24)
	            break;
	        // Handle other cases (PLLI2S, PLLSAI)
	        default:
	            return 1; // Invalid clock type
	    }
	} else {
	    switch (Clk_Type) {
	        case HSI:
	            RCC->CR &= ~(1 << 0); // Disable HSI (bit 0)
	            break;
	        case HSE:
	            RCC->CR &= ~(1 << 16); // Disable HSE (bit 16)
	            break;
	        case PLL:
	            RCC->CR &= ~(1 << 24); // Disable PLL (bit 24)
	            break;
	        // Handle other cases
	        default:
	            return 1; // Invalid clock type
	    }
	}
    // Wait for the clock to be ready (bit is set in RCC->CR register)
    while (((RCC->CR >> (Clk_Type + 1)) & 1) == 0);

    return 0;  // Success
}

/**
 * @brief Configures the system clock source.
 *
 * This function selects the clock source for the system clock (HSI, HSE, or PLL).
 *
 * @param SYSClkType The type of clock source to use (HSI, HSE, PLL).
 * @return uint8_t Returns 0 on success, 1 if the clock source is invalid.
 */
uint8_t RCC_SetSysClk(SYS_CLK_t SYSClkType) {
	if (SYSClkType > SYSPLLR) {
	    return 1;  // Invalid system clock type
	}
    // Set the APB1 prescaler to divide by 2 (bits [12:10] = 0b100)
    RCC->CFGR &= ~(0x7 << 10);     // Clear the APB1 prescaler bits
    RCC->CFGR |= (0b100 << 10);    // Set APB1 prescaler to divide by 2

    // Set the APB2 prescaler to divide by 1 (bits [15:13] = 0b000)
    RCC->CFGR &= ~(0x7 << 13);     // Clear the APB2 prescaler bits
    RCC->CFGR |= (0b000 << 13);    // Set APB2 prescaler to divide by 1
    /*setting the AHB prescaler By 4*/
    RCC->CFGR &= ~(0b1111 << 4);
    RCC->CFGR |= (0b1001  << 4);
    // Clear the SW bits (bits [1:0]) in RCC->CFGR to prepare for the new clock source
    RCC->CFGR &= ~(0x3 << 0);

    // Set the SW bits according to the desired clock source
    switch (SYSClkType) {
        case SYSHSI:
            RCC->CFGR |= (0 << 0); // HSI as system clock
            break;
        case SYSHSE:
            RCC->CFGR |= (1 << 0); // HSE as system clock
            break;
        case SYSPLLP:
            RCC->CFGR |= (2 << 0); // PLLP as system clock
            break;
        default:
            return 1;  // Invalid system clock type
    }


    // Wait for the system clock to be switched and confirmed (SWS[1:0] bits)
    while (((RCC->CFGR >> 2) & 0b11) != SYSClkType);


    return 0;  // Success
}

/**
 * @brief Configures the HSE clock mode to either bypassed or not bypassed.
 *
 * This function sets the HSE clock mode by modifying the HSEBYP bit in the RCC_CR register.
 *
 * @param HSE_MODE HSE mode to be configured, defined by the HSE_t enum.
 * @return uint8_t Returns 0 if the mode is set successfully, 1 for an invalid input.
 */
uint8_t RCC_HSE_Mode(HSE_t HSE_MODE) {
    if (HSE_MODE == BYPASSED) {
        RCC->CR |= (1 << 18);  // Set HSEBYP bit to bypass HSE with external clock signal
        return 0;             // Success
    } else if (HSE_MODE == NOT_BYPASSED) {
        RCC->CR &= ~(1 << 18); // Clear HSEBYP bit to use the HSE oscillator directly
        return 0;             // Success
    } else {
        return 1;             // Invalid mode input
    }
}

/**
 * @brief Configures the Phase-Locked Loop (PLL) for the STM32F446RE microcontroller.
 *
 * This function sets the PLL configuration parameters, including the input divider (PLLM),
 * the multiplier (PLLN), and the output divider (PLLP). It also selects the PLL clock source
 * (either HSI or HSE) and ensures that the PLL is enabled and ready for use as the system clock.
 *
 * @param PLL_M The input clock divider (PLLM):
 *              This parameter sets the PLLM divider, which divides the input clock (HSI or HSE)
 *              before being multiplied by the PLLN multiplier. The valid range for PLL_M is
 *              between 2 and 63 (inclusive). Example: PLL_M = 4 means the input clock is divided by 4.
 *
 * @param PLL_P The output clock divider (PLLP):
 *              This parameter sets the PLLP divider, which divides the PLL output clock to generate the system clock.
 *              The valid values for PLL_P are 2, 4, 6, or 8. Example: PLL_P = 4 means the PLL output clock is divided by 4.
 *
 * @param PLL_N The PLL multiplier (PLLN):
 *              This parameter sets the PLLN multiplier, which multiplies the PLL input clock (after being divided by PLL_M).
 *              The valid range for PLL_N is between 192 and 432 (inclusive). Example: PLL_N = 360 means the PLL input clock
 *              is multiplied by 360.
 *
 * @param Src The clock source for the PLL:
 *            This parameter selects the source of the PLL input clock. It should be either:
 *            - HSI (High-Speed Internal) oscillator or
 *            - HSE (High-Speed External) oscillator.
 *            Invalid values for this parameter will result in an error. Example: Src = HSE means the PLL uses the external oscillator as the input clock.
 *
 * @return uint8_t Returns 0 on success, or 1 if an error occurs (invalid parameter or clock source).
 */
uint8_t RCC_PLL_Config(uint32_t PLL_M, uint8_t PLL_P, uint16_t PLL_N, CLK_t Src) {
    // Ensure valid clock source
    if (Src != HSI && Src != HSE && Src != PLL) {
        return 1;  // Invalid source
    }

    // 1. Configure PLL source based on the selected source
    switch (Src) {
        case HSI:
            RCC->PLLCFGR &= ~(0x1 << 22); // Select HSI as PLL input
            break;
        case HSE:
            RCC->PLLCFGR |= (0x1 << 22);  // Select HSE as PLL input
            break;
        default:
            return 1;  // Invalid clock source
    }

    // 2. Configure PLLM (PLLM divider)
    // Clear the PLLM bits (PLLM is in bits [5:0]) and set the desired value
    // PLL_M must be between 2 and 63.
    RCC->PLLCFGR &= ~(0x3F << 0);           // Clear PLLM (bits 5:0)
    RCC->PLLCFGR |= (PLL_M << 0);            // Set PLLM as input divider (bits 5:0)

    // 3. Configure PLLN (PLLN multiplier)
    // Clear the PLLN bits (PLLN is in bits [14:6]) and set the desired value
    // PLL_N must be between 192 and 432.
    RCC->PLLCFGR &= ~(0x1FF << 6);          // Clear PLLN (bits 14:6)
    RCC->PLLCFGR |= (PLL_N << 6);            // Set PLLN as multiplier (bits 14:6)

    // 4. Configure PLLP (PLLP output divider)
    // Clear the PLLP bits (PLLP is in bits [17:16]) and set the desired value
    // PLL_P must be 2, 4, 6, or 8.
    RCC->PLLCFGR &= ~(0b11 << 16);           // Clear PLLP (bits 17:16)
    RCC->PLLCFGR |= ((PLL_P / 2 - 1) << 16); // Set PLLP as output divider (bits 17:16)

    // 5. Enable PLL by setting the PLLON bit
    RCC->CR |= (1 << 24); // Set PLLON bit to turn on PLL

    // 6. Wait until PLL is ready by checking the PLLRDY flag
    while (!(RCC->CR & (1 << 25))) {
        // Wait for PLLRDY (PLL ready flag)
    }



    return 0;  // Success
}


/**
 * @brief Enables the clock for a specific AHB1 peripheral.
 *
 * @param PeripheralName The peripheral to enable (e.g., GPIOAEN, GPIOBEN).
 * @return uint8_t Returns 0 on success, 1 if the peripheral name is invalid.
 */
uint8_t RCC_AHB1_EnableClk(RCC_AHB1_PERIPHERAL_t PeripheralName) {
    if (PeripheralName > 31) {
        return 1;  // Return error if the peripheral name is out of range
    }

    RCC->AHB1ENR |= (1 << PeripheralName);  // Enable the peripheral clock
    return 0;  // Success
}

/**
 * @brief Disables the clock for a specific AHB1 peripheral.
 *
 * @param PeripheralName The peripheral to disable (e.g., GPIOAEN, GPIOBEN).
 * @return uint8_t Returns 0 on success, 1 if the peripheral name is invalid.
 */
uint8_t RCC_AHB1_DisableClk(RCC_AHB1_PERIPHERAL_t PeripheralName) {
    if (PeripheralName > 31) {
        return 1;  // Return error if the peripheral name is out of range
    }

    RCC->AHB1ENR &= ~(1 << PeripheralName);  // Disable the peripheral clock
    return 0;  // Success
}

/**
 * @brief Enables the clock for a specific AHB2 peripheral.
 *
 * @param PeripheralName The peripheral to enable (e.g., DCMIEN, OTGFSEN).
 * @return uint8_t Returns 0 on success, 1 if the peripheral name is invalid.
 */
uint8_t RCC_AHB2_EnableClk(RCC_AHB2_PERIPHERAL_t PeripheralName) {
    if (PeripheralName > 31) {
        return 1;  // Return error if the peripheral name is out of range
    }

    RCC->AHB2ENR |= (1 << PeripheralName);  // Enable the peripheral clock
    return 0;  // Success
}

/**
 * @brief Disables the clock for a specific AHB2 peripheral.
 *
 * @param PeripheralName The peripheral to disable (e.g., DCMIEN, OTGFSEN).
 * @return uint8_t Returns 0 on success, 1 if the peripheral name is invalid.
 */
uint8_t RCC_AHB2_DisableClk(RCC_AHB2_PERIPHERAL_t PeripheralName) {
    if (PeripheralName > 31) {
        return 1;  // Return error if the peripheral name is out of range
    }

    RCC->AHB2ENR &= ~(1 << PeripheralName);  // Disable the peripheral clock
    return 0;  // Success
}

/**
 * @brief Enables the clock for a specific AHB3 peripheral.
 *
 * @param PeripheralName The peripheral to enable (e.g., FMCEN, QSPIEN).
 * @return uint8_t Returns 0 on success, 1 if the peripheral name is invalid.
 */
uint8_t RCC_AHB3_EnableClk(RCC_AHB3_PERIPHERAL_t PeripheralName) {
    if (PeripheralName > 31) {
        return 1;  // Return error if the peripheral name is out of range
    }

    RCC->AHB3ENR |= (1 << PeripheralName);  // Enable the peripheral clock
    return 0;  // Success
}

/**
 * @brief Disables the clock for a specific AHB3 peripheral.
 *
 * @param PeripheralName The peripheral to disable (e.g., FMCEN, QSPIEN).
 * @return uint8_t Returns 0 on success, 1 if the peripheral name is invalid.
 */
uint8_t RCC_AHB3_DisableClk(RCC_AHB3_PERIPHERAL_t PeripheralName) {
    if (PeripheralName > 31) {
        return 1;  // Return error if the peripheral name is out of range
    }

    RCC->AHB3ENR &= ~(1 << PeripheralName);  // Disable the peripheral clock
    return 0;  // Success
}

/**
 * @brief Enables the clock for a specific APB1 peripheral.
 *
 * @param PeripheralName The peripheral to enable (e.g., TIM2EN, USART2EN).
 * @return uint8_t Returns 0 on success, 1 if the peripheral name is invalid.
 */
uint8_t RCC_APB1_EnableClk(RCC_APB1_PERIPHERAL_t PeripheralName) {
    if (PeripheralName > 31) {
        return 1;  // Return error if the peripheral name is out of range
    }

    RCC->APB1ENR |= (1 << PeripheralName);  // Enable the peripheral clock
    return 0;  // Success
}

/**
 * @brief Disables the clock for a specific APB1 peripheral.
 *
 * @param PeripheralName The peripheral to disable (e.g., TIM2EN, USART2EN).
 * @return uint8_t Returns 0 on success, 1 if the peripheral name is invalid.
 */
uint8_t RCC_APB1_DisableClk(RCC_APB1_PERIPHERAL_t PeripheralName) {
    if (PeripheralName > 31) {
        return 1;  // Return error if the peripheral name is out of range
    }

    RCC->APB1ENR &= ~(1 << PeripheralName);  // Disable the peripheral clock
    return 0;  // Success
}

/**
 * @brief Enables the clock for a specific APB2 peripheral.
 *
 * @param PeripheralName The peripheral to enable (e.g., TIM1EN, ADC1EN).
 * @return uint8_t Returns 0 on success, 1 if the peripheral name is invalid.
 */
uint8_t RCC_APB2_EnableClk(RCC_APB2_PERIPHERAL_t PeripheralName) {
    if (PeripheralName > 31) {
        return 1;  // Return error if the peripheral name is out of range
    }

    RCC->APB2ENR |= (1 << PeripheralName);  // Enable the peripheral clock
    return 0;  // Success
}

/**
 * @brief Disables the clock for a specific APB2 peripheral.
 *
 * @param PeripheralName The peripheral to disable (e.g., TIM1EN, ADC1EN).
 * @return uint8_t Returns 0 on success, 1 if the peripheral name is invalid.
 */
uint8_t RCC_APB2_DisableClk(RCC_APB2_PERIPHERAL_t PeripheralName) {
    if (PeripheralName > 31) {
        return 1;  // Return error if the peripheral name is out of range
    }

    RCC->APB2ENR &= ~(1 << PeripheralName);  // Disable the peripheral clock
    return 0;  // Success
}
