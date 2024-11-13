/**
 * @file    USART_Driver.c
 * @brief   USART Driver for STM32F446RE microcontroller, providing functions 
 *          for initialization, data transmission, and reception over USART.
 *
 * This driver enables communication over USART, supporting configuration options
 * such as mode selection, hardware flow control, baud rate, parity, stop bits,
 * and word length. It provides both single-byte and array transmission functions,
 * as well as a function for receiving data.
 *
 * @author  Gerardo
 * @date    2024-11-12
 * @version 1.0
 *
 * @note    This code is designed for the STM32F446RE and may require adjustments
 *          for other microcontroller models. Ensure that configuration values
 *          are set appropriately for the application requirements.
 *
 * @attention
 *          This driver assumes that the USART peripheral clock is already enabled 
 *          and configured. Verify clock setup and RCC configurations as needed.
 *
 * @copyright
 *          (c) 2024 Gerardo. All rights reserved.
 */



#include "USART_Interface.h"
#include "STM32F446xx.h"
#include "ErrType.h"
#include <stdlib.h>


/**
 * @brief  Initializes the specified USART peripheral with the provided configuration settings.
 * 
 * This function configures the USART peripheral according to the parameters specified in
 * the USART_Config_t structure. It handles enabling the transmitter, receiver, hardware 
 * flow control, parity control, stop bits, word length, and baud rate.
 *
 * @param  USARTx        Pointer to the USART peripheral base address.
 * @param  USARTConfig   Pointer to the configuration structure for USART settings.
 * 
 * @return uint8_t       Returns USART_SUCCESS on successful initialization, USART_ERROR 
 *                       if there is an invalid configuration setting.
 *
 * @note   Ensure that the USARTx pointer and configuration values are valid before 
 *         calling this function.
 */
uint8_t USART_Init(USART_RegDef_t *USARTx, const USART_Config_t* USARTConfig)
{
    /* Check for null pointers (MISRA C:2012 Rule 17.2) */
    if ((USARTx == NULL) || (USARTConfig == NULL))
    {
        return NULL_PTR_ERR;
    }

    /* 1. Configure USART Mode (Transmitter, Receiver, or Both) */
    switch(USARTConfig->Mode)
    {
        case 0U:
            USARTx->CR1 |= (1U << 3U); /**< Enable Transmit Only */
            break;
        case 1U:
            USARTx->CR1 |= (1U << 2U); /**< Enable Receive Only */
            break;
        case 2U:
            USARTx->CR1 |= (1U << 3U) | (1U << 2U); /**< Enable Both Transmit and Receive */
            break;
        default:
            return NOK; /**< Return error for invalid mode */
    }

    /* 2. Configure Hardware Flow Control */
    #if HARDWARE_FLOW_CONTROL == ENABLED
        USARTx->CR3 |= (1U << 9U); /**< Enable CTS */
        USARTx->CR3 |= (1U << 8U); /**< Enable RTS */
    #endif

    /* 3. Configure Oversampling Rate (OVER8 or OVER16) */
    #if OVER_SAMPLING_RATE == OVER8
        USARTx->CR1 |= (1U << 15U); /**< Select oversampling by 8 */
    #elif OVER_SAMPLING_RATE == 2
        USARTx->CR1 &= ~(1U << 15U); /**< Select oversampling by 16 */
    #else
        return NOK; /**< Return error for invalid oversampling rate */
    #endif

    /* 4. Configure Word Length */
    USARTx->CR1 |= (USARTConfig->WordLength << 12U); /**< Set word length (e.g., 8 or 9 bits) */

    /* 5. Configure Parity Control */
    #if PARITY_CONTROL == ENABLED
        USARTx->CR1 |= (1U << 10U); /**< Enable parity control */
        USARTx->CR1 |= (USARTConfig->parity << 9U); /**< Set parity type (even or odd) */
    #endif

    /* 6. Configure Stop Bits */
    USARTx->CR2 &= ~(0b11U << 12U); /**< Clear stop bits */
    USARTx->CR2 |= ((USARTConfig->StopBits & 0b11U) << 12U); /**< Set stop bits (0.5, 1, 1.5, or 2) */

    /* 7. Configure Baud Rate */
    #if OVER_SAMPLING_RATE == 2
    /*
    uint32_t Loc_u16USART_Div = ((USART_P_CLOCK) * 100U) / (16U * USARTConfig->BaudRate);
    uint32_t mantissa = Loc_u16USART_Div / 100U;
    uint32_t fraction = Loc_u16USART_Div - (mantissa * 100U);
    */

    float Loc_u16USART_Div =  (((float)(USART_P_CLOCK)  / (16 * USARTConfig->BaudRate))*1000);
    uint32_t mantissa = Loc_u16USART_Div / 1000U;
    uint32_t fraction =  (uint32_t)Loc_u16USART_Div %1000;
    uint32_t Loc_u16USART_Div_Fraction = ((fraction *16)+500)/1000;
    USARTx->BRR = (mantissa << 4U) | (Loc_u16USART_Div_Fraction);
    /*
    USARTx->BRR = (mantissa << 4U) | (fraction / 16U); // Set BRR with mantissa and fraction
    */
    #elif OVER_SAMPLING_RATE == OVER8
    uint32_t Loc_u16USART_Div = ((USART_P_CLOCK) * 100U) / (8U * USARTConfig->BaudRate);
    uint32_t mantissa = Loc_u16USART_Div / 100U;
    uint32_t fraction = Loc_u16USART_Div - (mantissa * 100U);
    #endif
    /*8. Enable USART*/
    USARTx->CR1 |= (1U << 13U);  // Enable USART (UE: USART Enable)


    return OK; /**< Indicate successful initialization */
}


/*
 * @brief  Transmits a single byte of data over the specified USART peripheral.
 *
 * This function waits until the transmit data register is empty before loading
 * the data to be transmitted. It ensures that data is not written to the register
 * until the USART is ready.
 *
 * @param  USARTx  Pointer to the USART peripheral base address.
 * @param  data    The byte of data to be transmitted.
 * 
 * @return int8_t  Returns USART_SUCCESS (0) on successful transmission, 
 *                 or USART_ERROR (-1) if there was an error.
 *
 * @note   Ensure that USART has been properly initialized and enabled 
 *         before calling this function.
 */
uint8_t USART_Transmit(USART_RegDef_t *USARTx, uint8_t data)
{
    /* Check for null pointer (MISRA C:2012 Rule 17.2) */
    if (USARTx == NULL)
    {
        return NULL_PTR_ERR; /**< Return error if USARTx pointer is NULL */
    }

    /* Wait until the transmit data register is empty */
    while ((USARTx->SR & (1U << 7U)) == 0U)
    {
        /* Busy wait until TXE (Transmit Data Register Empty) flag is set */
    }

    /* Load the data into the Data Register */
    USARTx->DR = data;

    /* Wait for transmission to complete */
    while ((USARTx->SR & (1U << 6U)) == 0U)
    {
        /* Busy wait until TC (Transmission Complete) flag is set */
    }

    return OK; /**< Return success after data transmission */
}


/**
 * @brief  Receives a single byte of data from the specified USART peripheral.
 *
 * This function waits until the receive data register (RXNE) is full, 
 * then reads the received data. It ensures data is only read when 
 * the USART has received a byte.
 *
 * @param  USARTx        Pointer to the USART peripheral base address.
 * @param  receivedData  Pointer to store the received byte of data.
 * 
 * @return uint8_t       Returns USART_SUCCESS (0) on successful reception,
 *                       or USART_ERROR (-1) if there was an error.
 *
 * @note   Ensure that the USART peripheral has been initialized and enabled
 *         before calling this function.
 */
uint8_t USART_Receive(USART_RegDef_t *USARTx, uint8_t* receivedData)
{
    /* Check for null pointers (MISRA C:2012 Rule 17.2) */
    if ((USARTx == NULL) || (receivedData == NULL))
    {
        return NULL_PTR_ERR; /**< Return error if USARTx or receivedData is NULL */
    }

    /* Wait until the receive data register (RXNE) is full */
    while ((USARTx->SR & (1U << 5U)) == 0U)
    {
        /* Busy wait until RXNE (Receive Data Register Not Empty) flag is set */
    }

    /* Read the received data from the Data Register */
    *receivedData = (uint8_t)(USARTx->DR & 0xFFU); /**< Mask data to 8 bits */

    return OK; /**< Return success after data reception */
}

/**
 * @brief  Transmits an array of bytes over the specified USART peripheral.
 *
 * This function sends each byte in the dataArray, waiting for the transmit
 * data register (TXE) to be empty between each byte. This ensures all bytes
 * in the array are transmitted sequentially.
 *
 * @param  USARTx     Pointer to the USART peripheral base address.
 * @param  dataArray  Pointer to the array of bytes to transmit.
 * @param  size       Number of bytes in the dataArray to be transmitted.
 * 
 * @return uint8_t    Returns USART_SUCCESS (0) on successful transmission,
 *                    or USART_ERROR (-1) if there was an error.
 *
 * @note   Ensure that the USART peripheral has been initialized and enabled
 *         before calling this function.
 */
uint8_t USART_TransmitArray(USART_RegDef_t *USARTx, const uint8_t* dataArray, uint16_t size)
{
    uint16_t i; /* Loop counter */

    /* Check for null pointers and zero size (MISRA C:2012 Rule 17.2) */
    if ((USARTx == NULL) || (dataArray == NULL) || (size == 0U))
    {
        return NULL_PTR_ERR; /**< Return error if USARTx, dataArray is NULL, or size is zero */
    }

    /* Loop through each byte in the dataArray */
    for (i = 0U; i < size; ++i)
    {
        /* Wait until the transmit data register (TXE) is empty */
        while ((USARTx->SR & (1U << 7U)) == 0U)
        {
            /* Busy wait until TXE (Transmit Data Register Empty) flag is set */
        }

        /* Load the data byte into the Data Register */
        USARTx->DR = dataArray[i];
    }

    /* Wait for the last transmission to complete */
    while ((USARTx->SR & (1U << 6U)) == 0U)
    {
        /* Busy wait until TC (Transmission Complete) flag is set */
    }

    return OK; /**< Return success after entire array is transmitted */
}
