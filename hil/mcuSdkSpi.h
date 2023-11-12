#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
  MCU_SDK_SPI_MODE0 = 0,
  MCU_SDK_SPI_MODE1,
  MCU_SDK_SPI_MODE2,
  MCU_SDK_SPI_MODE3
}McuSdkSpiMode_e;



typedef struct
{
  uint32_t        baudrate;
  McuSdkSpiMode_e SpiMode;
  bool            IsLsdFirst;
}McuSdkSpiConfig_s;

void mcuSdkSpiInit(void * spiBaseAddress, McuSdkSpiConfig_s *ptrConfig);    // Init spi bus
uint8_t mcuSdkSpiWriteAndReadChar(void* spiBaseAddress, uint8_t data);      // Send a byte on bus and returned the byte received

#endif
