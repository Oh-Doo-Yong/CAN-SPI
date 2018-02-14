//#include "Arduino.h"
//#define __cplusplus

#include "drv_canfdspi_api.h"
#include "drv_spi.h"
#include <SPI.h>

uint16_t txCounter;
bool flush = true;

void setup() {
  Serial.begin(115200);
  uint8_t error = 0;
  DRV_SPI_Initialize();
  
  // Reset device
  error = DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0); // TODO: make sure it resets the chip
  Serial.println(error); // 1 is OK
  
  // Oscillator Configuration: divide by 10
  CAN_OSC_CTRL oscCtrl;
  DRV_CANFDSPI_OscillatorControlObjectReset(&oscCtrl);
  oscCtrl.ClkOutDivide = OSC_CLKO_DIV10;
  DRV_CANFDSPI_OscillatorControlSet(DRV_CANFDSPI_INDEX_0, oscCtrl);
  
  // Input/Output Configuration: use nINT0 and nINT1
  DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
  
  // CAN Configuration: ISO_CRC, enable TEF, enable TXQ
  CAN_CONFIG config;
  DRV_CANFDSPI_ConfigureObjectReset(&config);
  config.IsoCrcEnable = 1;
  config.StoreInTEF = 1;
  config.TXQEnable = 1;
  DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_0, &config);
  
  // Bit Time Configuration: 500K/2M, 80% sample point
  DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_2M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
  
  // TEF Configuration: 12 messages, time stamping enabled
  CAN_TEF_CONFIG tefConfig;
  tefConfig.FifoSize = 11;
  tefConfig.TimeStampEnable = 1;
  DRV_CANFDSPI_TefConfigure(DRV_CANFDSPI_INDEX_0, &tefConfig);
  
  // TXQ Configuration: 8 messages, 32 byte maximum payload, high priority
  CAN_TX_QUEUE_CONFIG txqConfig;
  DRV_CANFDSPI_TransmitQueueConfigureObjectReset(&txqConfig);
  txqConfig.TxPriority = 1;
  txqConfig.FifoSize = 7;
  txqConfig.PayLoadSize = CAN_PLSIZE_32;
  DRV_CANFDSPI_TransmitQueueConfigure(DRV_CANFDSPI_INDEX_0, &txqConfig);
  
  // FIFO 1: Transmit FIFO; 5 messages, 64 byte maximum payload, low priority
  CAN_TX_FIFO_CONFIG txfConfig;
  DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txfConfig);
  txfConfig.FifoSize = 4;
  txfConfig.PayLoadSize = CAN_PLSIZE_64;
  txfConfig.TxPriority = 0;
  DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, &txfConfig);
  
  // FIFO 2: Receive FIFO; 16 messages, 64 byte maximum payload, time stamping enabled
  CAN_RX_FIFO_CONFIG rxfConfig;
  rxfConfig.FifoSize = 15;
  rxfConfig.PayLoadSize = CAN_PLSIZE_64;
  rxfConfig.RxTimeStampEnable = 1;
  DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2, &rxfConfig);
  
  // Double Check RAM Usage: 2040 Bytes out of a maximum of 2048 Bytes -> OK.
  // Enable ECC
  DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_0);
  
  // Initialize RAM
  DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_0, 0xff);
  
  // Configuration Done: Select Normal Mode
  DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);
  
}

void loop() {
  
  // Assemble transmit message: CAN FD Extended frame with BRS, 32 data bytes
  CAN_TX_MSGOBJ txObj;
  uint8_t txd[MAX_DATA_BYTES];
  
  // Initialize ID and Control bits
  txObj.word[0] = 0;
  txObj.word[1] = 0;
  
  txObj.bF.id.SID = txCounter; // Standard or Base ID
  txObj.bF.id.EID = txCounter; // Extended ID
  
  txObj.bF.ctrl.FDF = 1; // CAN FD frame
  txObj.bF.ctrl.BRS = 1; // Switch bit rate
  txObj.bF.ctrl.IDE = 1; // Extended frame
  txObj.bF.ctrl.RTR = 0; // Not a remote frame request
  txObj.bF.ctrl.DLC = CAN_DLC_32; // 32 data bytes
  // Sequence: doesn't get transmitted, but will be stored in TEF
  txObj.bF.ctrl.SEQ = 2;
  
  // Initialize transmit data
  uint8_t i;
  for (i = 0; i < DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC); i++) {
    txd[i] = 10;
    //Serial.println("debug");
  }
//  Serial.println("");
  
  // Check that FIFO is not full
  // APP_TX_INT queries the input pin of the MCU, which is connected to nINT0
  delay(1000);
  //if (APP_TX_INT()) { // check if we can transmit, interrupt not added yet
    bool flush = true;

    // Load message and transmit
    DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0, CAN_TXQUEUE_CH0, &txObj, txd,
      DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC), flush);
  //}
  
  //uint8_t rx = DRV_SPI_SendByte();
  //Serial.println("printed to SPI");
  delay(10);
}
