
#include "drv_spi.h"
#include <avr/io.h>

void DRV_SPI_Initialize() {
  /* Set MOSI and SCK output, all others input */
  DDRB = (1<<DDB2)|(1<<DDB3)|(1<<DDB5);
  /* Enable SPI, Master, set clock rate fck/16 */
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize) {
  for (int i=0; i<spiTransferSize; i++) {
    /* Start transmission */
    SPDR = *SpiTxData;
    *SpiRxData = SPDR;
    
    SpiTxData++;
    SpiRxData++;
    // reading SPDR should result in both way simultaneous transmission
    /* Wait for transmission complete */
    while(!(SPSR & (1<<SPIF)))
      ;
  }
  return 1;
}

uint8_t DRV_SPI_SendByte() {
  SPDR = (uint8_t)2;
  uint8_t rx = SPDR;
  while(!(SPSR & (1<<SPIF)))
      ;
  return rx;
}

