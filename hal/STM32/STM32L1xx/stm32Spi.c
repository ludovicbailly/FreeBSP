#include <stdint.h>

#include "stm32l1xx.h"

#include "mcuSpiPort.h"
#include "mcuSpi.h"

//-------------------------------------------------------------------
//
//------------------------------------------------------------------
void mcuSdkSpiInit(void * spiBaseAddress, const McuSdkSpiConfig_s * const ptrConfig)
{
  SPI_TypeDef *ptrSpi = (SPI_TypeDef*)spiBaseAddress;

  switch( (uint32_t) ptrSpi )
  {
    case (uint32_t)SPI1:
      // Feeds with clock
      RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
      break;
    case (uint32_t)SPI2:
      // Feeds with clock
      RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
      break;
    case (uint32_t)SPI3:
      // Feeds with clock
      RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
      break;
    default:
      // Your SPI is not yet supported by this driver
      break;
  }

    /*
     * 28.3.3 Configuring the SPI in master mode
     * Procedure
        1. Select the BR[2:0] bits to define the serial clock baud rate (see SPI_CR1 register).
        2. Select the CPOL and CPHA bits to define one of the four relationships between the
          data transfer and the serial clock (see Figure 242). This step is not required when the
          TI mode is selected.
        3. Set the DFF bit to define 8- or 16-bit data frame format
        4. Configure the LSBFIRST bit in the SPI_CR1 register to define the frame format. This
          step is not required when the TI mode is selected.
        5.If the NSS pin is required in input mode, in hardware mode, connect the NSS pin to a
          high-level signal during the complete byte transmit sequence. In NSS software mode,
          set the SSM and SSI bits in the SPI_CR1 register. If the NSS pin is required in output
          mode, the SSOE bit only should be set. This step is not required when the TI mode is
          selected.
        6. Set the FRF bit in SPI_CR2 to select the TI protocol for serial communications.
        7. The MSTR and SPE bits must be set (they remain set only if the NSS pin is connected
          to a high-level signal).
     */

  // MSB first -> default
  if( ptrConfig->IsLsdFirst == true)
  {
    ptrSpi->CR1 |= SPI_CR1_LSBFIRST;
  }
  else
  {
    ptrSpi->CR1 &= ~SPI_CR1_LSBFIRST;
  }
  // Mode 0 -> default
  switch(ptrConfig->SpiMode)
  {
    case MCU_SDK_SPI_MODE0:
      ptrSpi->CR1 &= ~(SPI_CR1_CPOL + SPI_CR1_CPHA);
      break;
    case MCU_SDK_SPI_MODE1:
      ptrSpi->CR1 &= ~SPI_CR1_CPOL;
      ptrSpi->CR1 |= SPI_CR1_CPHA;
      break;
    case MCU_SDK_SPI_MODE2:
      ptrSpi->CR1 |= SPI_CR1_CPOL;
      ptrSpi->CR1 &= ~SPI_CR1_CPHA;
      break;
    case MCU_SDK_SPI_MODE3:
      ptrSpi->CR1 |= (SPI_CR1_CPOL + SPI_CR1_CPHA);
      break;
  }

  // 8 bits -> default
  // Is master
  ptrSpi->CR1 |= SPI_CR1_MSTR;

  // Clock is @8MHz divide by 8 to get SPI Clock @ 1MHz

  //TODO once clock driver is done
  ptrSpi->CR1 |= SPI_CR1_BR_1;

  // Control CS by software
  ptrSpi->CR1 |= SPI_CR1_SSM;
  ptrSpi->CR1 |= SPI_CR1_SSI;

  McuSdkSpiPortRoutePin( ptrSpi );

  // Finaly enable the SPI peripheral
  ptrSpi->CR1 |= SPI_CR1_SPE;
}

//-------------------------------------------------------------------
//
//------------------------------------------------------------------
uint8_t mcuSdkSpiWriteAndReadChar(void * spiBaseAddress, uint8_t data)
{
  SPI_TypeDef *ptrSpi = (SPI_TypeDef*)spiBaseAddress;

  // Send data
  ptrSpi->DR = data;

  // Wait until the date is forwarded to the shift register
  while( (ptrSpi->SR & SPI_SR_TXE) == 0uL)
  {
  }

  // Data is going away
  // Wait the end of transmission
  while( (ptrSpi->SR & SPI_SR_BSY) != 0uL)
  {
  }


  return  (uint8_t)ptrSpi->DR;
}
