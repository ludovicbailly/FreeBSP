/*
 * mcuSdkGpio.h
 *
 *  Created on: Nov 12, 2023
 *      Author: ludovic
 */

#ifndef MCU_SDK_GPIO_H_
#define MCU_SDK_GPIO_H_

#include <stdint.h>
#include <stdbool.h>

typedef void (*GpioIsrCb)(void);

void mcuSdkGpioInit(void);
uint8_t mcuSdkGpioPortAdressToIndex( void * ptrGpioBaseAddress);
void mcuSdkGpioSetAsOutput( void * ptrGpioBaseAddress, uint8_t pin);
void mcuSdkGpioSetAsOpenDrainOutput( void * ptrGpioBaseAddress, uint8_t pin);
void mcuSdkGpioSetAsInput( void * ptrGpioBaseAddress, uint8_t pin);
void mcuSdkGpioSetAsInputPullUp( void * ptrGpioBaseAddress, uint8_t pin);
void mcuSdkGpioSetAsInputPullDown( void * ptrGpioBaseAddress, uint8_t pin);
void mcuSdkGpioSetPinHigh( void * ptrGpioBaseAddress, uint8_t pin);
void mcuSdkGpioSetPinLow( void * ptrGpioBaseAddress, uint8_t pin);
void mcuSdkGpioTogglePin( void * ptrGpioBaseAddress, uint8_t pin);
bool mcuSdkGpioIsPinLow( void * ptrGpioBaseAddress, uint8_t pin);
bool mcuSdkGpioIsPinHigh( void * ptrGpioBaseAddress, uint8_t pin);

void mcuSdkGgpioConfigureInterruptOnRisingEdge(void * ptrGpioBaseAddress, uint8_t pin, GpioIsrCb callBackRoutine);
void mcuSdkGgpioConfigureInterruptOnFallingEdge(void * ptrGpioBaseAddress, uint8_t pin, GpioIsrCb callBackRoutine);
void mcuSdkGgpioConfigureInterruptOnBothEdge(void * ptrGpioBaseAddress, uint8_t pin, GpioIsrCb callBackRoutine);
void mcuSdkGgpioEnableInterrupt(void * ptrGpioBaseAddress, uint8_t pin);
void mcuSdkGgpioDisableInterrupt(void * ptrGpioBaseAddress, uint8_t pin);

#endif
