/*
 * mcuSdkGpio.c
 *
 *  Created on: Nov 12, 2023
 *      Author: ludovic
 */


// Standard C definitions
#include <stdint.h>
#include <stdbool.h>

// STM32 low level definitions
#include "stm32l1xx.h"
// API def
#include "mcuGpio.h"

#define MAX_ISR_NB  16

static GpioIsrCb IsrCallbacksTable[MAX_ISR_NB];

static void dummyIsrCallback (void);
static IRQn_Type PinToNvicIRQn(uint8_t pin);

/*
 *
 */
void mcuSdkGpioInit(void)
{
  // Enable all GPIOs
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN+RCC_AHBENR_GPIOBEN+RCC_AHBENR_GPIOCEN +RCC_AHBENR_GPIODEN
                +RCC_AHBENR_GPIOEEN + RCC_AHBENR_GPIOFEN + RCC_AHBENR_GPIOGEN +RCC_AHBENR_GPIOHEN;

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  for(uint8_t i = 0u; i<MAX_ISR_NB; i++ )
  {
    IsrCallbacksTable[i] = dummyIsrCallback;
  }
}

/*
 *
 */
uint8_t mcuSdkGpioPortAdressToIndex( void * ptrGpioBaseAddress)
{
  uint8_t index;

  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress);
  switch( (uint32_t)ptrGpio )
  {
    case (uint32_t)GPIOA:
      index = 0;
      break;
    case (uint32_t)GPIOB:
      index = 1;
      break;
    case (uint32_t)GPIOC:
      index = 2;
      break;
    case (uint32_t)GPIOD:
      index = 3;
      break;
    case (uint32_t)GPIOE:
          index = 4;
    case (uint32_t)GPIOH:
      index = 5;
      break;
    case (uint32_t)GPIOF:
      index = 6;
      break;
    case (uint32_t)GPIOG:
      index = 7;
      break;
    default:
      //TO DO error
      break;
  }
  return index;
}

/*
 *
 */
void mcuSdkGpioSetAsOutput( void * ptrGpioBaseAddress, uint8_t pin)
{
  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress);

  ptrGpio->OTYPER &= ~(1ul<<pin);

  uint32_t modeRValue = ptrGpio->MODER;
  modeRValue &= ~(0x3uL<< 2*pin); // Clear bits
  modeRValue |= (0x1uL<< 2*pin);  // Set bits
  ptrGpio->MODER = modeRValue;
}

/*
 *
 */
void mcuSdkGpioSetAsOpenDrainOutput( void * ptrGpioBaseAddress, uint8_t pin)
{
  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress);

  ptrGpio->OTYPER |= (1ul<<pin);  // Set the open drain function

  uint32_t modeRValue = ptrGpio->MODER;
  modeRValue &= ~(0x3uL<< 2*pin); // Clear bits
  modeRValue |= (0x1uL<< 2*pin);  // Set bits
  ptrGpio->MODER = modeRValue;
}

/*
 *
 */
void mcuSdkGpioSetAsInput( void * ptrGpioBaseAddress, uint8_t pin)
{
  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress);

  ptrGpio->OTYPER &= ~(1ul<<pin); // Reset the open drain field
  ptrGpio->PUPDR &= ~(0x3uL<< 2*pin); // Reset the pull option
  ptrGpio->MODER &= ~(0x3uL<< 2*pin); // COnfigure as input
}

/*
 *
 */
void mcuSdkGpioSetAsInputPullUp( void * ptrGpioBaseAddress, uint8_t pin)
{
  mcuSdkGpioSetAsInput( ptrGpioBaseAddress, pin);
  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress);

  ptrGpio->PUPDR &= ~(0x1uL<< 2*pin); // Then pull up

}

/*
 *
 */
void mcuSdkGpioSetAsInputPullDown( void * ptrGpioBaseAddress, uint8_t pin)
{
  mcuSdkGpioSetAsInput( ptrGpioBaseAddress, pin);
  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress);

  ptrGpio->PUPDR &= ~(0x2uL<< 2*pin); // Then pull Down

}

/*
 *
 */
void mcuSdkGpioSetPinHigh( void * ptrGpioBaseAddress, uint8_t pin)
{
  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress);

  ptrGpio->BSRR = 1ul<<pin; // Use the Set propriety

}

/*
 *
 */
void mcuSdkGpioSetPinLow( void * ptrGpioBaseAddress, uint8_t pin)
{
  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress);

  ptrGpio->BSRR = (0x10000ul<<pin); // Use the clear propriety

}


/*
 *
 */
void mcuSdkGpioTogglePin( void * ptrGpioBaseAddress, uint8_t pin)
{
  if( mcuSdkGpioIsPinHigh( ptrGpioBaseAddress, pin ) == false)
  {
    mcuSdkGpioSetPinHigh(ptrGpioBaseAddress, pin);
  }
  else
  {
    mcuSdkGpioSetPinLow(ptrGpioBaseAddress, pin);
  }
}

/*
 *
 */
bool mcuSdkGpioIsPinLow( void *ptrGpioBaseAddress, uint8_t pin )
{
  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef*)( ptrGpioBaseAddress );
  bool pinState;
  if( ( ptrGpio->IDR & ( 1uL << pin ) ) == 0uL )
  {
    pinState = true;
  }
  else
  {
    pinState = false;
  }
  return pinState;
}

/*
 *
 */
bool mcuSdkGpioIsPinHigh( void *ptrGpioBaseAddress, uint8_t pin )
{
  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress);

  bool pinState;
  if( (ptrGpio->IDR & (1uL<<pin)) == 0uL)
  {
    pinState = false;
  }
  else
  {
    pinState = true;
  }
  return pinState;
}

/*
 *
 */
void mcuSdkGpioConfigureInterruptOnRisingEdge(void * ptrGpioBaseAddress, uint8_t pin, GpioIsrCb callBackRoutine)
{
  IsrCallbacksTable[pin] = callBackRoutine;
  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress);
  EXTI->RTSR |= 1uL<< pin;
  EXTI->FTSR |= ~(1uL<< pin);
  SYSCFG->EXTICR[ pin >> 2] &= ~(0x0FuL << (4*(  mcuSdkGpioPortAdressToIndex(ptrGpio) & 3uL)));

}

/*
 *
 */
void mcuSdkGpioConfigureInterruptOnFallingEdge(void * ptrGpioBaseAddress, uint8_t pin, GpioIsrCb callBackRoutine)
{
  IsrCallbacksTable[pin] = callBackRoutine;

  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress);
  EXTI->FTSR |= 1uL<< pin;
  EXTI->RTSR |= ~(1uL<< pin);
  SYSCFG->EXTICR[ pin >> 2] &= ~(0x0FuL << (4*( pin & 3uL)));
  SYSCFG->EXTICR[ pin >> 2] |= (  mcuSdkGpioPortAdressToIndex(ptrGpio) << (4*( pin & 3uL)));
}

/*
 *
 */
void mcuSdkGpioConfigureInterruptOnBothEdge(void * ptrGpioBaseAddress, uint8_t pin, GpioIsrCb callBackRoutine)
{
  IsrCallbacksTable[pin] = callBackRoutine;
  GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress);
  EXTI->FTSR |= 1uL<< pin;
  EXTI->RTSR |= 1uL<< pin;
  SYSCFG->EXTICR[ pin >> 2] &= ~(0x0FuL << (4*( mcuSdkGpioPortAdressToIndex(ptrGpio) & 3uL)));
}

/*
 *
 */
void mcuSdkGpioEnableInterrupt(void * ptrGpioBaseAddress, uint8_t pin)
{
 // GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress); // Unused but MISRA compliant
  EXTI->IMR |= 1uL<<pin;

  __NVIC_EnableIRQ( PinToNvicIRQn(pin));

}

/*
 *
 */
void mcuSdkGpioDisableInterrupt(void * ptrGpioBaseAddress, uint8_t pin)
{
  //GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress); // Unused but MISRA compliant
  EXTI->IMR &= ~(1uL<<pin);
  __NVIC_DisableIRQ( PinToNvicIRQn(pin));
}

/*
 *
 */
void mcuSdkGpioClearPendingInterrupt(void * ptrGpioBaseAddress, uint8_t pin)
{
  //GPIO_TypeDef *ptrGpio = (GPIO_TypeDef *)(ptrGpioBaseAddress); // Unused but MISRA compliant
  EXTI->PR |= 1uL<<pin; // Writes 1 clear the pending interrupt
  __NVIC_ClearPendingIRQ(PinToNvicIRQn(pin));
}

/*
 *
 */
void EXTI0_IRQHandler(void)
{
  IsrCallbacksTable[0]();
}

/*
 *
 */
void EXTI1_IRQHandler(void)
{
  IsrCallbacksTable[1]();
}

/*
 *
 */
void EXTI2_IRQHandler(void)
{
  IsrCallbacksTable[2]();
}

/*
 *
 */
void EXTI3_IRQHandler(void)
{
  IsrCallbacksTable[3]();
}

/*
 *
 */
void EXTI4_IRQHandler(void)
{
  IsrCallbacksTable[4]();
}

/*
 *
 */
void EXTI9_5_IRQHandler(void)
{
  uint8_t offset = 5u;

  // Call one by one all active sources
  uint32_t activeSources = EXTI->PR & EXTI->IMR;
  activeSources &= 0x03E0uL; // 9 to 5 mask
  activeSources >>= 5; // Right alignment

  while( activeSources != 0uL)
  {
    if( (activeSources & 0x01uL) != 0uL)
    {
      IsrCallbacksTable[offset]();
    }
    offset ++;
    activeSources >>= 1;
  }
}

/*
 *
 */
void EXTI15_10_IRQHandler(void)
{
  uint8_t offset = 10u;

  // Call one by one all active sources
  uint32_t activeSources = EXTI->PR & EXTI->IMR;
  activeSources &= 0xFC00uL; // 15 to 10 mask
  activeSources >>= 10; // Right alignment

  while( activeSources != 0uL )
  {
    if( ( activeSources & 0x01uL ) != 0uL )
    {
      IsrCallbacksTable[offset]();
    }
    offset++;
    activeSources >>= 1;
  }
}


/*
 *
 */
static void dummyIsrCallback (void)
{

}

/*
 *
 */
static IRQn_Type PinToNvicIRQn(uint8_t pin)
{
  IRQn_Type irqType;

  if( pin <= 4)
  {
    irqType = EXTI0_IRQn + pin;;
  }
  else if ( pin <= 9)
  {
    irqType = EXTI9_5_IRQn;
  }
  else
  {
    irqType = EXTI15_10_IRQn;
  }
 return irqType;
}
