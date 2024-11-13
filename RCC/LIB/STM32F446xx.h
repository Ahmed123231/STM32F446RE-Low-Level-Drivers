#ifndef STM32F446xx_H
#define STM32F446xx_H

/******************* Various Memories Base Addresses *******************/
#define FLASH_BASE_ADDRESS           0x08000000UL
#define SRAM_BASE_ADDRESS			 0x20000000UL
#define ROM_BASE_ADDRESS			 0x1FFF0000UL

/******************* Core Preipherals Base Addresses *******************/

#define NVIC_BASE_ADDRESS			 0xE000E100UL

/******************* AHB1 Preipherals Base Addresses *******************/
#define GPIOA_BASE_ADDRESS			 0x40020000U
#define GPIOB_BASE_ADDRESS			 0x40020400U
#define GPIOC_BASE_ADDRESS			 0x40020800U
#define GPIOD_BASE_ADDRESS			 0x40020C00U
#define GPIOE_BASE_ADDRESS			 0x40021000U
#define GPIOF_BASE_ADDRESS			 0x40021400U
#define GPIOG_BASE_ADDRESS			 0x40021800U
#define GPIOH_BASE_ADDRESS			 0x40021C00U
	 
#define RCC_BASE_ADDRESS 			 0x40023800U

/******************* AHB2 Preipherals Base Addresses *******************/

/******************* AHB3 Preipherals Base Addresses *******************/

/******************* APB1 Preipherals Base Addresses *******************/

/******************* APB2 Preipherals Base Addresses *******************/


/******************* GPIO Register Definition Structure *******************/

typedef struct {
	
	volatile uint32_t MODER;        /*GPIO PORT Mode Register */
	volatile uint32_t OTYPER;       /*GPIO PORT Output type Register */
	volatile uint32_t OSPEEDR;      /*GPIO PORT Output speed Register */
	volatile uint32_t PUPDR;        /*GPIO PORT Pull up/down Register */
	volatile uint32_t IDR;  		/*GPIO PORT Input Data Register */
	volatile uint32_t ODR;  		/*GPIO PORT Output Data Register */
	volatile uint32_t BSRR;  		/*GPIO PORT Bit Set/Reset Register */
	volatile uint32_t LCKR;  		/*GPIO PORT Lock Register */
	volatile uint32_t AFR[2];       /*GPIO PORT Alternate function Register */
	
}GPIO_RegDef_t;

/******************* GPIO Preipheral Base Addresses *******************/

#define GPIOA                  ((GPIO_RegDef_t*)GPIOA_BASE_ADDRESS)
#define GPIOB                  ((GPIO_RegDef_t*)GPIOB_BASE_ADDRESS)
#define GPIOC                  ((GPIO_RegDef_t*)GPIOC_BASE_ADDRESS)
#define GPIOD                  ((GPIO_RegDef_t*)GPIOD_BASE_ADDRESS)
#define GPIOE                  ((GPIO_RegDef_t*)GPIOE_BASE_ADDRESS)
#define GPIOF                  ((GPIO_RegDef_t*)GPIOF_BASE_ADDRESS)
#define GPIOG                  ((GPIO_RegDef_t*)GPIOG_BASE_ADDRESS)
#define GPIOH                  ((GPIO_RegDef_t*)GPIOH_BASE_ADDRESS)

/******************* RCC Register Definition Structure *******************/

typedef struct
{
	
	volatile uint32_t CR;      			/*!<RCC Clock Control Register,                    									   */
	volatile uint32_t PLLCFGR; 			/*!<RCC PLL Configuration Register,                									   */
	volatile uint32_t CFGR;    			/*!<RCC Clock Configuration Register,              									   */
	volatile uint32_t CIR;     			/*!<RCC Clock Interrupt Register,                  									   */
	volatile uint32_t AHB1RSTR; 	    /*!<RCC AHB1 Peripheral Reset Register,            									   */
	volatile uint32_t AHB2RSTR; 	    /*!<RCC AHB2 Peripheral Reset Register,            									   */
	volatile uint32_t AHB3RSTR; 	    /*!<RCC AHB3 Peripheral Reset Register,            									   */
	uint32_t          RESERVED0;  		/*!<Reserved, 0x1C                                 									   */
	volatile uint32_t APB1RSTR; 	    /*!<RCC APB1 peripheral Reset Register,            									   */
	volatile uint32_t APB2RSTR; 	    /*!<RCC APB2 peripheral Reset Register,            									   */
	uint32_t          RESERVED1[2];     /*!<Reserved, 0x28-0x2C                            									   */
	volatile uint32_t AHB1ENR;			/*!<RCC AHB1 Enable Register,                      									   */
	volatile uint32_t AHB2ENR;			/*!<RCC AHB2 Enable Register,                      									   */
	volatile uint32_t AHB3ENR;			/*!<RCC AHB3 Enable Register,                      									   */
	uint32_t          RESERVED2;		/*!<Reserved, 0x3C                                 									   */
	volatile uint32_t APB1ENR;			/*!<RCC APB1 Enable Register,                      									   */
	volatile uint32_t APB2ENR;			/*!<RCC APB2 Enable Register,                      									   */
	uint32_t          RESERVED3[2];		/*!<Reserved, 0x48-0x4C                            									   */
	volatile uint32_t AHB1LPENR;		/*!<RCC AHB1 peripheral clock enable,              									   */
	volatile uint32_t AHB2LPENR;		/*!<RCC AHB2 peripheral clock enable,              									   */
	volatile uint32_t AHB3LPENR;		/*!<RCC AHB3 peripheral clock enable,              									   */
	uint32_t          RESERVED4;		/*!<Reserved, 0x5C                                 									   */
	volatile uint32_t APB1LPENR;		/*!<RCC APB1 peripheral clock enabled in low power mode register,                      */
	volatile uint32_t APB2LPENR;		/*!<RCC APB2 peripheral clock enabled in low power mode register,                      */
	uint32_t          RESERVED5[2];		/*!<Reserved, 0x68-0x6C                                                                */
	volatile uint32_t BDCR;				/*!<RCC Backup domain control register,                                                */
	volatile uint32_t CSR;				/*!<RCC clock control & status register,                                               */
	uint32_t          RESERVED6[2];		/*!<Reserved, 0x78-0x7C                                                                */
	volatile uint32_t SSCGR;			/*!<RCC spread spectrum clock generation register,                                     */
	volatile uint32_t PLLI2SCFGR;		/*!<RCC PLLI2S configuration register,                                                 */
	volatile uint32_t PLLSAICFGR;		/*!<RCC PLL configuration register,                                                    */
	volatile uint32_t DCKCFGR;			/*!<RCC dedicated clock configuration register,                                        */
	volatile uint32_t CKGATENR;			/*!<RCC clocks gated enable register,                                                  */
	volatile uint32_t DCKCFGR2;			/*!<RCC dedicated clocks configuration register 2,                                     */
	
}RCC_RegDef_t;


/******************* NVIC Register Definition Structure *******************/

typedef struct
{
	volatile uint32_t ISER[8];       	/*!< Interrupt Set-Enable Registers (ISER): Enables interrupts, 0xE000E100 - 0xE000E11C (8 registers) */
	uint32_t          RESERVED0[24]; 	/*!< Reserved space to align ICER to 0xE000E180 */

	volatile uint32_t ICER[8];       	/*!< Interrupt Clear-Enable Registers (ICER): Disables interrupts, 0xE000E180 - 0xE000E19C (8 registers) */
	uint32_t          RESERVED1[24]; 	/*!< Reserved space to align ISPR to 0xE000E200 */

	volatile uint32_t ISPR[8];       	/*!< Interrupt Set-Pending Registers (ISPR): Sets pending state of interrupts, 0xE000E200 - 0xE000E21C (8 registers) */
	uint32_t          RESERVED2[24]; 	/*!< Reserved space to align ICPR to 0xE000E280 */

	volatile uint32_t ICPR[8];       	/*!< Interrupt Clear-Pending Registers (ICPR): Clears pending state of interrupts, 0xE000E280 - 0xE000E29C (8 registers) */
	uint32_t          RESERVED3[24]; 	/*!< Reserved space to align IABR to 0xE000E300 */

	volatile uint32_t IABR[8];       	/*!< Interrupt Active Bit Registers (IABR): Indicates active state of interrupts, 0xE000E300 - 0xE000E31C (8 registers) */
	uint32_t          RESERVED4[56]; 	/*!< Reserved space to align IPR to 0xE000E400 */

	volatile uint32_t IPR[60];       	/*!< Interrupt Priority Registers (IPR): Sets priority levels of interrupts, 0xE000E400 - 0xE000E4EF (240 bytes total) */
	uint32_t          RESERVED5[644]; 	/*!< Reserved space to align STIR to 0xE000EF00 */

	volatile uint32_t STIR;          	/*!< Software Trigger Interrupt Register (STIR): Generates software interrupts, 0xE000EF00 */
} NVIC_RegDef_t;

/******************* NVIC Preipheral Base Addresses *******************/

#define NVIC                  ((NVIC_RegDef_t*)NVIC_BASE_ADDRESS)   /*!< Pointer to NVIC_RegDef Struct*/


#endif 