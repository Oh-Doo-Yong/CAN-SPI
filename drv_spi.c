
#include "drv_spi.h"
#include <avr/io.h>

#define pin_SS (1<<PORTB2)

void DRV_SPI_Initialize() {
  /* Set MOSI and SCK output, all others input */
  DDRB = (1<<DDB2)|(1<<DDB3)|(1<<DDB5);
  PORTB |= pin_SS;
  //SPI.begin();
  /* Enable SPI, Master, set clock rate fck/16 */
  //SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0); // 1 MHz
  //SPSR |= (1<<SPI2X); // 2MHz
}

volatile int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize) {
  //SPI.beginTransaction(SPISettings(10E6, MSBFIRST, SPI_MODE0)); // frequency, MSBFIRST, SPI_MODE0
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0); // 1 MHz
  PORTB &= ~pin_SS;
  for (int i=0; i<spiTransferSize; i++) {
    /* Start transmission */
    //*SpiRxData = SPI.transfer(*SpiTxData);
    SPDR = *SpiTxData;
    // reading SPDR should result in both way simultaneous transmission
    /* Wait for transmission complete */
    while(!(SPSR & _BV(SPIF))) ;
    *SpiRxData = SPDR;
    
    SpiTxData++;
    SpiRxData++;
    
  }
  PORTB |= pin_SS;
  //SPI.endTransaction();
  return 0;
}

