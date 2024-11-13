/**
 * @file  USART_Interface.h
 * @brief Interface for the Universal Synchronous Asynchronous Receiver Transmitter (USART) driver.
 *
 * This file provides function prototypes, macros, and type definitions for configuring and using
 * the USART peripheral in a generic microcontroller. It includes basic functions for initializing
 * the USART, transmitting and receiving single bytes and arrays, and specifying configuration 
 * parameters such as baud rate, word length, stop bits, and parity.
 *
 * @note   The USART peripheral clock frequency is defined as USART_P_CLOCK, which may vary depending 
 *         on the specific hardware setup.
 *
 * @version 1.0
 * @date   2024-11-11
 * @author Ahmed Atef Eid 
 */

#ifndef USART_INTERFACE_H
#define USART_INTERFACE_H

#include <stdint.h>
#include "STM32F446xx.h"

#define OVER_8              1    /**< Oversampling by 8 */
#define OVER_16             2    /**< Oversampling by 16 */
#define OVER_SAMPLING_RATE  2 /**< Default oversampling rate */

#define ENABLED             1    /**< Enable flag */
#define DISABLED            0    /**< Disable flag */
#define HARDWARE_FLOW_CONTROL DISABLED /**< Default hardware flow control setting */

#define PARITY_CONTROL      DISABLED /**< Default parity control setting */

#define USART_P_CLOCK       8000000UL /**< USART Peripheral Clock frequency in Hz */

/**
 * @brief USART operating modes.
 *
 * Defines the available modes for USART operation, including transmit-only,
 * receive-only, and full-duplex modes.
 */
typedef enum
{
    Tx,     /**< Transmit-only mode */
    RX,     /**< Receive-only mode */
    TX_RX   /**< Full-duplex mode (transmit and receive) */
} UART_Mode_t;

/**
 * @brief Commonly used baud rates for USART communication.
 *
 * Supported baud rates for standard and high-speed USART communication.
 */
typedef enum
{
    BAUD_1200    = 1200,
    BAUD_2400    = 2400,
    BAUD_4800    = 4800,
    BAUD_9600    = 9600,
    BAUD_19200   = 19200,
    BAUD_38400   = 38400,
    BAUD_57600   = 57600,
    BAUD_115200  = 115200,
    BAUD_230400  = 230400,
    BAUD_460800  = 460800,
    BAUD_921600  = 921600,
    BAUD_1000000 = 1000000,  /**< 1 Mbps */
    BAUD_2000000 = 2000000,  /**< 2 Mbps */
    BAUD_3000000 = 3000000,  /**< 3 Mbps */
    BAUD_4000000 = 4000000   /**< 4 Mbps (max for STM32F4 series) */
} UART_BaudRate_t;

/**
 * @brief Word length configurations for USART data transmission.
 *
 * Defines options for data word length in bits, applicable to USART configurations
 * that support variable word lengths.
 */
typedef enum
{
    WORD_LENGTH_8B = 0, /**< 8-bit word length */
    WORD_LENGTH_9B = 1  /**< 9-bit word length */
} UART_WordLength_t;

/**
 * @brief Stop bit configurations for USART communication.
 *
 * Defines the number of stop bits to use, allowing for configurations suitable for
 * various data transmission requirements.
 */
typedef enum
{
    STOP_BITS_1     = 0, /**< 1 stop bit */
    STOP_BITS_0_5   = 1, /**< 0.5 stop bits */
    STOP_BITS_2     = 2, /**< 2 stop bits */
    STOP_BITS_1_5   = 3  /**< 1.5 stop bits */
} UART_StopBits_t;

/**
 * @brief Parity options for USART communication.
 *
 * Defines the types of parity control available, supporting even and odd parity for
 * error-checking mechanisms in data transmission.
 */
typedef enum
{
    EVEN = 0, /**< Even parity */
    ODD  = 1  /**< Odd parity */
} UART_Parity_t;

/**
 * @brief Configuration structure for USART peripheral.
 *
 * Holds all configurable parameters for initializing the USART, including mode, 
 * baud rate, word length, stop bits, and parity.
 */
typedef struct 
{
    UART_Mode_t           Mode;       /**< USART operation mode (Tx, Rx, or TxRx) */
    UART_BaudRate_t       BaudRate;   /**< Desired baud rate for communication */
    UART_WordLength_t     WordLength; /**< Data word length in bits */
    UART_StopBits_t       StopBits;   /**< Number of stop bits to use */
    UART_Parity_t         parity;     /**< Parity control (even or odd) */
} USART_Config_t;

/**
 * @brief Initializes the USART peripheral with specified configuration.
 *
 * This function configures the USART based on the parameters specified in the 
 * USART_Config_t structure. It must be called before any other USART function.
 *
 * @param[in] USARTx      Pointer to the USART peripheral base address.
 * @param[in] USARTConfig Pointer to a configuration structure specifying baud rate,
 *                        word length, stop bits, parity, and mode.
 * 
 * @return uint8_t        Returns USART_OK if initialization is successful, otherwise USART_ERROR.
 *
 * @note Make sure the peripheral clock for the USART is enabled before calling this function.
 */
uint8_t USART_Init(USART_RegDef_t *USARTx, const USART_Config_t* USARTConfig);

/**
 * @brief Transmits a single byte of data over USART.
 *
 * Sends a single byte of data via USART, blocking until the transmission is complete.
 *
 * @param[in] USARTx Pointer to the USART peripheral base address.
 * @param[in] data   The byte of data to be transmitted.
 * 
 * @return uint8_t   Returns USART_OK if the byte was successfully transmitted, otherwise USART_ERROR.
 */
uint8_t USART_Transmit(USART_RegDef_t *USARTx, uint8_t data);

/**
 * @brief Receives a single byte of data from USART.
 *
 * Waits until data is available in the receive register, then reads a single byte.
 *
 * @param[in]  USARTx       Pointer to the USART peripheral base address.
 * @param[out] receivedData Pointer to a variable where the received byte will be stored.
 * 
 * @return uint8_t          Returns USART_OK if a byte is successfully received, otherwise USART_ERROR.
 */
uint8_t USART_Receive(USART_RegDef_t *USARTx, uint8_t* receivedData);

/**
 * @brief Transmits an array of bytes over USART.
 *
 * Sends multiple bytes of data in sequence, blocking until each byte has been transmitted.
 *
 * @param[in] USARTx    Pointer to the USART peripheral base address.
 * @param[in] dataArray Pointer to the array of data bytes to transmit.
 * @param[in] size      The number of bytes in the data array to transmit.
 * 
 * @return uint8_t      Returns USART_OK if the array is successfully transmitted, otherwise USART_ERROR.
 */
uint8_t USART_TransmitArray(USART_RegDef_t *USARTx, const uint8_t* dataArray, uint16_t size);

#endif /* USART_INTERFACE_H */
