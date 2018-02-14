#ifndef _DRV_SPI_H
#define _DRV_SPI_H

#include <inttypes.h>
#include <avr/io.h>

//========================================================//
#ifdef  __cplusplus
extern "C" {
#endif

// Index to SPI channel
// Used when multiple MCP2517FD are connected to the same SPI interface, but with different CS    
#define DRV_CANFDSPI_INDEX_0         0
#define DRV_CANFDSPI_INDEX_1         1

//! SPI Initialization
    
void DRV_SPI_Initialize();

//! SPI Read/Write Transfer

int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t XferSize);
uint8_t DRV_SPI_SendByte();

//! SPI Chip Select assert/de-assert

//int8_t DRV_SPI_ChipSelectAssert(uint8_t spiSlaveDeviceIndex, bool assert);

//========================================================//
#ifdef  __cplusplus
}
#endif
//========================================================//
#endif  /* _DRV_SPI_H */
