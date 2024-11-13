#ifndef RCC_INTERFACE_H
#define RCC_INTERFACE_H

#include "RCC_private.h"

/**
 * @brief Sets the status of the specified clock.
 * 
 * This function enables or disables a specific clock based on the provided status.
 *
 * @param Clk_Type The type of clock to set (HSI, HSE, PLL).
 * @param Status The status to set for the clock (ON, OFF).
 */
uint8_t RCC_SetClkStatus(CLK_t Clk_Type, STATUS_t Status);

/**
 * @brief Configures the system clock source.
 * 
 * This function selects the clock source for the system clock (HSI, HSE, or PLL).
 *
 * @param ClkType The type of clock source to use (HSI, HSE, PLL).
 */
uint8_t RCC_SetSysClk(CLK_t ClkType);

/**
 * @brief Configures the High-Speed External (HSE) mode.
 * 
 * This function sets the mode for the High-Speed External clock.
 *
 * @param HSE_MODE The mode to set for the HSE (BYPASSED, NOT_BYPASSED).
 */
uint8_t RCC_HSE_Mode(HSE_t HSE_MODE);

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
uint8_t RCC_PLL_Config(uint32_t PLL_M, uint8_t PLL_P, uint16_t PLL_N, CLK_t Src);

/**
 * @brief Enables the clock for a specific AHB1 peripheral.
 * 
 * This function enables the clock for an AHB1 peripheral identified by PeripheralName.
 *
 * @param PeripheralName The peripheral to enable (e.g., GPIOAEN, GPIOBEN).
 */
uint8_t RCC_AHB1_EnableClk(RCC_AHB1_PERIPHERAL_t PeripheralName);

/**
 * @brief Disables the clock for a specific AHB1 peripheral.
 * 
 * This function disables the clock for an AHB1 peripheral identified by PeripheralName.
 *
 * @param PeripheralName The peripheral to disable (e.g., GPIOAEN, GPIOBEN).
 */
uint8_t RCC_AHB1_DisableClk(RCC_AHB1_PERIPHERAL_t PeripheralName);

/**
 * @brief Enables the clock for a specific AHB2 peripheral.
 * 
 * This function enables the clock for an AHB2 peripheral identified by PeripheralName.
 *
 * @param PeripheralName The peripheral to enable (e.g., DCMIEN, OTGFSEN).
 */
uint8_t RCC_AHB2_EnableClk(RCC_AHB2_PERIPHERAL_t PeripheralName);

/**
 * @brief Disables the clock for a specific AHB2 peripheral.
 * 
 * This function disables the clock for an AHB2 peripheral identified by PeripheralName.
 *
 * @param PeripheralName The peripheral to disable (e.g., DCMIEN, OTGFSEN).
 */
uint8_t RCC_AHB2_DisableClk(RCC_AHB2_PERIPHERAL_t PeripheralName);

/**
 * @brief Enables the clock for a specific AHB3 peripheral.
 * 
 * This function enables the clock for an AHB3 peripheral identified by PeripheralName.
 *
 * @param PeripheralName The peripheral to enable (e.g., FMCEN, QSPIEN).
 */
uint8_t RCC_AHB3_EnableClk(RCC_AHB3_PERIPHERAL_t PeripheralName);

/**
 * @brief Disables the clock for a specific AHB3 peripheral.
 * 
 * This function disables the clock for an AHB3 peripheral identified by PeripheralName.
 *
 * @param PeripheralName The peripheral to disable (e.g., FMCEN, QSPIEN).
 */
uint8_t RCC_AHB3_DisableClk(RCC_AHB3_PERIPHERAL_t PeripheralName);

/**
 * @brief Enables the clock for a specific APB1 peripheral.
 * 
 * This function enables the clock for an APB1 peripheral identified by PeripheralName.
 *
 * @param PeripheralName The peripheral to enable (e.g., TIM2EN, USART2EN).
 */
uint8_t RCC_APB1_EnableClk(RCC_APB1_PERIPHERAL_t PeripheralName);

/**
 * @brief Disables the clock for a specific APB1 peripheral.
 * 
 * This function disables the clock for an APB1 peripheral identified by PeripheralName.
 *
 * @param PeripheralName The peripheral to disable (e.g., TIM2EN, USART2EN).
 */
uint8_t RCC_APB1_DisableClk(RCC_APB1_PERIPHERAL_t PeripheralName);

/**
 * @brief Enables the clock for a specific APB2 peripheral.
 * 
 * This function enables the clock for an APB2 peripheral identified by PeripheralName.
 *
 * @param PeripheralName The peripheral to enable (e.g., TIM1EN, ADC1EN).
 */
uint8_t RCC_APB2_EnableClk(RCC_APB2_PERIPHERAL_t PeripheralName);

/**
 * @brief Disables the clock for a specific APB2 peripheral.
 * 
 * This function disables the clock for an APB2 peripheral identified by PeripheralName.
 *
 * @param PeripheralName The peripheral to disable (e.g., TIM1EN, ADC1EN).
 */
uint8_t RCC_APB2_DisableClk(RCC_APB2_PERIPHERAL_t PeripheralName);

#endif // RCC_INTERFACE_H
