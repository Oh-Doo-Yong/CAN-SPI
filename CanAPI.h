/*
 * Author: Piotr Strycharski
 * Organisation: EUFS
 * Date: 07/06/18
 * Description: Main API for CAN communication
 * Target device: Arduino
 *
 * The original software provided by Microchip can be found at: https://www.microchip.com/DevelopmentTools/ProductDetails/ADM00576
 *
 *    Copyright 2018 EUFS
 */

#ifndef _CANAPI_H
#define _CANAPI_H

#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif

/**
  * @brief  Configure CAN module
  * @param  CAN_BITTIME_SETUP CAN_1000K_8M sets CAN baudrate to 1M (8M in FD mode)
  * Other comon setting is CAN_500K_1M (0.5M in CAN 2.0 mode)
  */
void configure_can(CAN_BITTIME_SETUP);
bool can_test_ram_access(void);
bool can_test_register_access(void);
/**
  * @brief  Transmit CAN message. Usualy lower priority, bigger messages
  * @param  CAN_MSGOBJ_ID standard CAN ID e.g. 0x45A
  * @param  *txd pointer to the data
  * @param  dlc data size (CAN_DLC_(0..8) for CAN 2.0)
  * @return 0 if success
  */
uint8_t can_transmit_fifo(uint16_t sid, uint8_t *txd, CAN_DLC dlc);
void can_tef_read(void);
/**
  * @brief  Filter CAN messages before storing them to RXQ and RXFIFO
  * @param  CAN_FILTER filter instance CAN_FILTER(0..31)
  * @param  sid all bits to compare e.g. 0x400
  * @param  msid what bits of incoming message have to match sid e.g. 0xF00 with the above sid matches 0x4XX
  * @return 0 if success
  */
void can_init_filter(CAN_FILTER filter, uint16_t sid, uint16_t msid);
void can_init_timestamping(void);
int8_t can_init_interrupts(void);
/**
  * @brief  Receive CAN message
  * @param  *sid pointer to message ID (filled by the receive function from CAN_RX_MSGOBJ)
  * @param  *rxd pointer to data
  * @param  *len pointer to length (filled by the receive function from CAN_RX_MSGOBJ)
  * @return pointer to CAN_RX_MSGOBJ
  */
CAN_RX_MSGOBJ* can_receive_fifo(uint16_t *sid, uint8_t *rxd, uint8_t *len);
/**
  * @brief  Transmit CAN message
  * @param  CAN_MSGOBJ_ID standard CAN ID e.g. 0x45A
  * @param  *txd pointer to the data
  * @param  dlc data size (CAN_DLC_(0..8) for CAN 2.0)
  * @return 0 if success
  */
uint8_t can_transmit(uint16_t sid, uint8_t *txd, CAN_DLC dlc);
/**
  * @brief  Receive CAN message using interrupt (typical method)
  * @param  *sid pointer to message ID (filled by the receive function from CAN_RX_MSGOBJ)
  * @param  *rxd pointer to data
  * @param  *len pointer to length (filled by the receive function from CAN_RX_MSGOBJ)
  * @return pointer to CAN_RX_MSGOBJ
  */
CAN_RX_MSGOBJ* can_receive(uint16_t *sid, uint8_t *rxd, uint8_t *len);
void can_request_config(void);
bool can_wait_for_config(void);

#ifdef __cplusplus
}
#endif


#endif // _CANAPI_H
/*******************************************************************************
 End of File
*/

