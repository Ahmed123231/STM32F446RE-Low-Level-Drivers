# NVIC Driver for STM32F446xx Microcontroller

## Overview

This driver provides an API to interact with the Nested Vectored Interrupt Controller (NVIC) of the STM32F446xx microcontroller. The NVIC is responsible for managing interrupts, including enabling, disabling, prioritizing, and controlling interrupt pending states. The functions in this driver allow you to configure and manage interrupts efficiently in your embedded application.

## Features

The NVIC driver includes functions for the following operations:
- Enabling/Disabling IRQs (Interrupt Requests)
- Setting/Clearing Pending IRQs
- Setting and Retrieving IRQ Priorities
- Checking IRQ Active and Pending States

## File Structure

- `NVIC_Interface.c`: Implementation of NVIC driver functions.
- `NVIC_Interface.h`: Header file containing function prototypes and necessary includes.
- `NVIC_Private.h`: Internal definitions and private data structures (if any).
- `STM32F446xx.h`: Contains the register definitions for the STM32F446xx microcontroller.

## Function Overview

### 1. `void NVIC_EnableIRQ(IRQn_Type IRQn);`

Enables the specified IRQ in the NVIC.

#### Parameters:
- `IRQn`: The IRQ number to enable, defined in `IRQn_Type`.

#### Example usage:
```c
NVIC_EnableIRQ(TIM2_IRQn);  // Enables the TIM2 interrupt
