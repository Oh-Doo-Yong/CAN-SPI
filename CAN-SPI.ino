//#include "Arduino.h"

#include "drv_canfdspi_api.h"
#include "drv_spi.h"
#include "CanAPI.h"

uint8_t *tx = malloc(8*sizeof(uint8_t));
uint8_t rx[MAX_DATA_BYTES]; // same as  uint8_t *rx = malloc(8*sizeof(uint8_t));
CAN_RX_MSGOBJ *rxObj;

void setup() {
  Serial.begin(115200);
  DRV_SPI_Initialize();
  
  uint8_t error = 0;

  configure_can(CAN_1000K_8M);
  error = can_init_interrupts();
  can_init_filter(CAN_FILTER0, 0x400, 0xF00);
  can_init_interrupts();
  
  Serial.println(error);
}

void loop() {
//  if (can_test_ram_access())
//    Serial.println("error reading ram");
//  else
//    Serial.println("ram works fine\n");

//  if (can_test_register_access())
//    Serial.println("error reading registers");
//  else
//    Serial.println("register access OK");

  uint8_t error = 0;
  error = can_transmit(0x45A, tx, CAN_DLC_8);
  if(error)
    Serial.println("CAN send issue");
  else
    Serial.println("CAN send OK");

  delay(500);
  uint16_t rxSid;
  uint8_t rxLen;
  CAN_RX_MSGOBJ *rxObj = can_receive(&rxSid, rx, &rxLen);
  if(rxObj){
    if(rxObj->bF.id.SID == 0x45A) { // or simply rxSid == 0x45A
      for(int i=0; i<rxLen; i++)
      {Serial.print(rx[i]); Serial.print(" ");}
      Serial.println("UpShift");
    } else
    Serial.println("Message received but SID mismatch");
  }
  else
    Serial.println("No message received");

  delay(1000);
}
