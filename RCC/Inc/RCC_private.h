#ifndef RCC_PRIVATE_H
#define RCC_PRIVATE_H



/********************* Enumeration for Clock Source Types *********************/
typedef enum
{
    HSI = 0,    // Internal High-Speed Clock
    HSE = 16,    // External High-Speed Clock
    PLL = 24,    // Phase-Locked Loop Clock
	PLLI2S = 26,
	PLLSAI=28,
	

}CLK_t;

typedef enum 
{
	
	NOT_BYPASSED=0,
	BYPASSED
	
}HSE_t;



/********************* Enumeration for Clock Status *********************/
typedef enum
{
    OFF = 0,    // Clock OFF
    ON = 1      // Clock ON
	
}STATUS_t;

/********************* Enumeration for System Clock Multiplexer (PLL source selection) *****/
typedef enum
{
	SYSHSI = 0,    // HSI as the system clock
	SYSHSE = 1,    // HSE as the system clock
    SYSPLLP = 2,   // PLLP as the system clock
    SYSPLLR        // PLLR as the system clock
	
}SYS_CLK_t;

/********************* PLL Configuration Structure *********************/
typedef struct
{
    uint8_t  PLL_R;   // PLL division factor for the main system clock
    uint8_t  PLL_Q;   // PLL division factor for USB OTG FS, SDIO, and RNG
    uint8_t  PLL_P;   // PLL division factor for main system clock
    uint16_t PLL_N;   // PLL multiplication factor for VCO
    uint8_t  PLL_M;   // PLL division factor for input clock
	
} PLL_CONFIG_t;

/********************* Enumeration for AHB1 Peripheral Clock Enable *********************/
typedef enum
{
    GPIOAEN = 0,   // GPIOA clock enable
    GPIOBEN,       // GPIOB clock enable
    GPIOCEN,       // GPIOC clock enable
    GPIODEN,       // GPIOD clock enable
    GPIOEEN,       // GPIOE clock enable
    GPIOFEN,       // GPIOF clock enable
    GPIOGEN,       // GPIOG clock enable
    GPIOHEN,       // GPIOH clock enable
    CRCEN = 12,    // CRC clock enable
	SRAM =18,
    DMA1EN = 21,   // DMA1 clock enable
    DMA2EN,        // DMA2 clock enable
    OTGHSEN = 29,  // USB OTG HS clock enable
	OTGHSULPIEN
	
}RCC_AHB1_PERIPHERAL_t;

/********************* Enumeration for AHB2 Peripheral Clock Enable *********************/
typedef enum
{
    DCMIEN = 0,  // DCMI clock enable
    OTGFSEN = 7  // USB OTG FS clock enable
	
	
}RCC_AHB2_PERIPHERAL_t;

/********************* Enumeration for AHB3 Peripheral Clock Enable *********************/
typedef enum
{
    FMCEN = 0,  // Flexible Memory Controller clock enable
    QSPIEN      // Quad SPI clock enable
	
}RCC_AHB3_PERIPHERAL_t;

/********************* Enumeration for APB1 Peripheral Clock Enable *********************/
typedef enum
{
    TIM2EN = 0,  // TIM2 timer clock enable
    TIM3EN,      // TIM3 timer clock enable
    TIM4EN,      // TIM4 timer clock enable
    TIM5EN,      // TIM5 timer clock enable
    TIM6EN,      // TIM6 timer clock enable
    TIM7EN,      // TIM7 timer clock enable
    TIM12EN,     // TIM12 timer clock enable
    TIM13EN,     // TIM13 timer clock enable
    TIM14EN,     // TIM14 timer clock enable
    WWDGEN = 11, // Window watchdog clock enable
    SPI2EN = 14, // SPI2 clock enable
    SPI3EN,      // SPI3 clock enable
    SPDIFRXEN,   // SPDIF-RX clock enable
    USART2EN,    // USART2 clock enable
    USART3EN,    // USART3 clock enable
    UART4EN,     // UART4 clock enable
    UART5EN,     // UART5 clock enable
    I2C1EN,      // I2C1 clock enable
    I2C2EN,      // I2C2 clock enable
    I2C3EN,      // I2C3 clock enable
    FMPI2C1EN,   // FMPI2C1 clock enable
    CAN1EN,      // CAN1 clock enable
    CAN2EN,      // CAN2 clock enable
    CECEN,       // CEC clock enable
    PWREN,       // Power interface clock enable
    DACEN        // DAC clock enable
	
}RCC_APB1_PERIPHERAL_t;

/********************* Enumeration for APB2 Peripheral Clock Enable *********************/
typedef enum
{
    TIM1EN = 0,  // TIM1 timer clock enable
    TIM8EN,      // TIM8 timer clock enable
    USART1EN = 4,// USART1 clock enable
    USART6EN,    // USART6 clock enable
    ADC1EN = 8,  // ADC1 clock enable
    ADC2EN,      // ADC2 clock enable
    ADC3EN,      // ADC3 clock enable
    SDIOEN,      // SDIO clock enable
    SPI1EN,      // SPI1 clock enable
    SPI4EN,      // SPI4 clock enable
    SYSCFGEN,    // SYSCFG clock enable
    TIM9EN = 16, // TIM9 timer clock enable
    TIM10EN,     // TIM10 timer clock enable
    TIM11EN,     // TIM11 timer clock enable
    SAI1EN = 22, // SAI1 clock enable
    SAI2EN       // SAI2 clock enable
	
}RCC_APB2_PERIPHERAL_t;

#endif // RCC_PRIVATE_H
