#ifndef GPIO_INTERFACE_H
#define GPIO_INTERFACE_H

#include <stdint.h>

// Enumeration for GPIO ports
typedef enum 
{
    PORTA = 0,
    PORTB,
    PORTC,
    PORTD,
    PORTE,
    PORTF,
    PORTG,
    PORTH
}Port_t;

// Enumeration for GPIO pins
typedef enum
{
    PIN0 = 0,
    PIN1,
    PIN2,
    PIN3,
    PIN4,
    PIN5,
    PIN6,
    PIN7,
    PIN8,
    PIN9,
    PIN10,
    PIN11,
    PIN12,
    PIN13,
    PIN14,
    PIN15
	
}Pin_t;

// Enumeration for GPIO pin modes
typedef enum 
{
    INPUT = 0,
    OUTPUT,
    ALTERNATE_FUNCTION,
    ANALOG
}Mode_t;

// Enumeration for GPIO output speed
typedef enum 
{
    LOW = 0,
    MEDIUM,
    FAST,
    HIGH
	
}OutputSpeed_t;

// Enumeration for GPIO output type
typedef enum 
{
    PUSH_PULL = 0,
    OPEN_DRAIN
	
}OutputType_t;

// Enumeration for GPIO pull-up/pull-down configurations
typedef enum 
{
    NOPULL = 0,
    PULLUP,
    PULLDOWN
	
}PullUpDown_t;

// Enumeration for GPIO pin value states
typedef enum 
{
    PIN_LOW = 0,
    PIN_HIGH
	
}PinVal_t;

// Enumeration for alternate function settings
typedef enum
{
    AF0 = 0,
    AF1,
    AF2,
    AF3,
    AF4,
    AF5,
    AF6,
    AF7,
    AF8,
    AF9,
    AF10,
    AF11,
    AF12,
    AF13,
    AF14,
    AF15
	
}AltFunc_t;

// Struct for pin configuration
typedef struct
{
    Port_t          Port;         /*!< Specifies the port (e.g., PORTA, PORTB) */
    Pin_t           PinNum;       /*!< Specifies the pin number (e.g., PIN0, PIN1) */
    Mode_t          Mode;         /*!< Specifies the mode (INPUT, OUTPUT, etc.) */
    OutputSpeed_t   Speed;        /*!< Specifies the speed (LOW, MEDIUM, FAST, HIGH) */
    OutputType_t    OutputType;   /*!< Specifies the output type (PUSH_PULL, OPEN_DRAIN) */
    PullUpDown_t    PullType;     /*!< Specifies the pull-up/pull-down configuration */
    AltFunc_t       AltFunc;      /*!< Specifies the alternate function (AF0 - AF15) */
	
	
}PinConfig_t;

// Function Prototypes

/**
 * @brief Initializes a GPIO pin based on the provided configuration.
 * @param PinConfig Pointer to a PinConfig_t structure that contains
 *                  the configuration information for the specified GPIO pin.
 * @return uint8_t Status of the initialization (0 = success, 1 = error)
 */
uint8_t GPIO_u8PinInit(const PinConfig_t* PinConfig);

/**
 * @brief Sets the value of a specific GPIO pin.
 * @param Port Port number to which the pin belongs (e.g., PORTA).
 * @param PinNum Pin number (e.g., PIN0, PIN1).
 * @param PinVal Value to set the pin (PIN_LOW or PIN_HIGH).
 * @return uint8_t Status of the operation (0 = success, 1 = error)
 */
uint8_t GPIO_u8SetPinValue(Port_t Port, Pin_t PinNum, PinVal_t PinVal);

/**
 * @brief Toggles the value of a specific GPIO pin.
 * @param Port Port number to which the pin belongs.
 * @param PinNum Pin number to toggle.
 * @return uint8_t Status of the operation (0 = success, 1 = error)
 */
uint8_t GPIO_u8TogglePinValue(Port_t Port, Pin_t PinNum);

/**
 * @brief Reads the value of a specific GPIO pin.
 * @param Port Port number to which the pin belongs.
 * @param PinNum Pin number to read.
 * @param PinVal Pointer to store the read pin value.
 * @return uint8_t Status of the operation (0 = success, 1 = error)
 */
uint8_t GPIO_u8ReadPinValue(Port_t Port, Pin_t PinNum, PinVal_t* PinVal);

#endif /* GPIO_INTERFACE_H */
