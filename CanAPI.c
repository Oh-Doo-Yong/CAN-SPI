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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include <math.h>
#include <avr/io.h>
#include <util/delay.h>
#include <Arduino.h>

#include "drv_canfdspi_api.h"
#include "drv_canfdspi_register.h"
#include "drv_spi.h"
#include "CanAPI.h"


// *****************************************************************************
// *****************************************************************************
// Section: Config Definitions
// *****************************************************************************
// *****************************************************************************

// NOP
#define Nop() __asm__ __volatile__ ("nop");

// Interrupt Status (need to be negated as below)
#define APP_INT()    !(PIND>>PIND3)&1
#define APP_RX_INT() !(PIND>>PIND2)&1
#define APP_TX_INT() !(PIND>>PIND4)&1

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************


uint8_t error;

uint16_t txCounter;

CAN_OPERATION_MODE opMode;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void can_configure_interrupt_pins()
{
    DDRD &= ~((1<<DDD2) | (1<<DDD3) | (1<<DDD4));
}

// *****************************************************************************
// *****************************************************************************
// Section: Can API
// *****************************************************************************
// *****************************************************************************


    /* MCP2517FD Configuration */
void configure_can(CAN_BITTIME_SETUP bitTime)
{
    can_configure_interrupt_pins();
    
    // Reset device
    DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);
    //_delay_ms(1);
      
    // Oscillator Configuration: divide by 10
    CAN_OSC_CTRL oscCtrl;
    DRV_CANFDSPI_OscillatorControlObjectReset(&oscCtrl);
    oscCtrl.ClkOutDivide = OSC_CLKO_DIV4;
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

    // Bit Time Configuration: 500K/1M, 80% sample point
    DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, bitTime, CAN_SSP_MODE_AUTO, CAN_SYSCLK_20M);

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

    /* RAM access test */
bool can_test_ram_access()
{
    // Variables
    uint8_t txd[MAX_DATA_BYTES];
    uint8_t rxd[MAX_DATA_BYTES];
    uint8_t i;
    uint8_t length;

    // Verify read/write with different access length
    // Note: RAM can only be accessed in multiples of 4 bytes
    for (length = 4; length <= MAX_DATA_BYTES; length += 4) {
        for (i = 0; i < length; i++) {
            txd[i] = rand() & 0xff;
            rxd[i] = 0xff;
        }

        // Write data to RAM
        DRV_CANFDSPI_WriteByteArray(DRV_CANFDSPI_INDEX_0, cRAMADDR_START, txd, length);

        // Read data back from RAM
        DRV_CANFDSPI_ReadByteArray(DRV_CANFDSPI_INDEX_0, cRAMADDR_START, rxd, length);

        // Verify
        bool good = false;
        for (i = 0; i < length; i++) {
            good = txd[i] == rxd[i];

            if (!good) {
                Nop();
                Nop();
                return 1;
                // Data mismatch
            }
        }
        return 0;
    }

}

    /* Register access test */
bool can_test_register_access()
{
    // Variables
    uint8_t txd[MAX_DATA_BYTES];
    uint8_t rxd[MAX_DATA_BYTES];
    uint8_t i;
    uint8_t length;

    // Verify read/write with different access length
    // Note: registers can be accessed in multiples of bytes
    for (length = 1; length <= MAX_DATA_BYTES; length++) {
        for (i = 0; i < length; i++) {
            txd[i] = rand() & 0x7f; // Bit 31 of Filter objects is not implemented
            rxd[i] = 0xff;
        }
        // Write data to registers
        DRV_CANFDSPI_WriteByteArray(DRV_CANFDSPI_INDEX_0, cREGADDR_CiFLTOBJ, txd, length);

        // Read data back from registers
        DRV_CANFDSPI_ReadByteArray(DRV_CANFDSPI_INDEX_0, cREGADDR_CiFLTOBJ, rxd, length);

        // Verify
        bool good = false;
        for (i = 0; i < length; i++) {
            good = txd[i] == rxd[i];

            if (!good) {
                Nop();
                Nop();
                return 1;
                // Data mismatch
            }
        }
        return 0;
    }

    Nop();
    Nop();

}

    /* Transmit using FIFO status */
uint8_t can_transmit_fifo(uint16_t sid, uint8_t *txd, CAN_DLC dlc)
{
       // Configure transmit message
    CAN_TX_MSGOBJ txObj;

    // Initialize ID and Control bits
    txObj.word[0] = 0;
    txObj.word[1] = 0;

    txObj.bF.id.SID = sid; // Standard or Base ID
    txObj.bF.id.EID = 0; // Extended ID

    txObj.bF.ctrl.FDF = 0; // 1: CAN FD frame 0: CAN 2.0
    txObj.bF.ctrl.BRS = 1; // Switch bit rate
    txObj.bF.ctrl.IDE = 0; // Extended frame
    txObj.bF.ctrl.RTR = 0; // Not a remote frame request
    txObj.bF.ctrl.DLC = dlc; // # of data bytes, any from 1 to 8 for CAN 2.0
    // Sequence: doesn't get transmitted, but will be stored in TEF
    txObj.bF.ctrl.SEQ = 2;

    // Initialize transmit data
    // Remeber to commtent it out later
    uint8_t i;
    for (i = 0; i < DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC); i++) {
        txd[i] = i*2;
    }

    // Check that FIFO is not full
    CAN_TX_FIFO_EVENT txFlags;
    bool flush = true;
    int8_t error = 0;

    error = DRV_CANFDSPI_TransmitChannelEventGet(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, &txFlags);

    if (txFlags & CAN_TX_FIFO_NOT_FULL_EVENT) {
        // Load message and transmit
        DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, &txObj, txd,
                DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC), flush);

        txCounter++;

        if (txCounter > 0x7ff) {
            Nop();
            Nop();
            txCounter = 0;
        }
    }
    else
      return 1; // error

    Nop();
    Nop();

    return 0;
}

    /* Read TEF message */
void can_tef_read()
{
    // TEF Object
    CAN_TEF_MSGOBJ tefObj;
    uint32_t id;

    // Check that TEF is not empty
    CAN_TEF_FIFO_EVENT tefFlags;
    DRV_CANFDSPI_TefEventGet(DRV_CANFDSPI_INDEX_0, &tefFlags);

    if (tefFlags & CAN_TEF_FIFO_NOT_EMPTY_EVENT) {
        // Read message and UINC
        DRV_CANFDSPI_TefMessageGet(DRV_CANFDSPI_INDEX_0, &tefObj);

        // Process message
        Nop();
        Nop();
        id = tefObj.bF.id.EID;
    }


}

    /* Initialize filter */
void can_init_filter(CAN_FILTER filter, uint16_t sid, uint16_t msid)
{
    /* Configure Filter 0: match SID = 0x300-0x30F, Standard frames only */

    // Disable Filter 0
    DRV_CANFDSPI_FilterDisable(DRV_CANFDSPI_INDEX_0, filter);

    // Configure Filter Object 0
    CAN_FILTEROBJ_ID fObj;
    fObj.SID = sid; // set of bits, the match is forced by msid
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // Only except Standard frames

    DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, filter, &fObj);

    // Configure Mask Object 0 
    CAN_MASKOBJ_ID mObj;
    mObj.MSID = msid; // 0 means don't care // standard identifier mask bits
    mObj.MSID11 = 0;
    mObj.MEID = 0;
    mObj.MIDE = 1; // Match IDE bit
    /* Identifier Receive mode bit
     * 1 = Match only message types (standard or extended ID) that correspond to EXIDE bit in filter
     * 0 = Match both standard and extended messagge frames if IDs match
     */

    DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_0, filter, &mObj);

    // Link Filter to RX FIFO 2, and enable Filter
    bool filterEnable = true;
    DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_0, filter, CAN_FIFO_CH2, filterEnable);


}

    /* Initialize time stamping */
void can_init_timestamping()
{
    // Disable TBC
    DRV_CANFDSPI_TimeStampDisable(DRV_CANFDSPI_INDEX_0);

    // Configure pre-scaler so TBC increments every 1 us @ 40MHz clock: 40-1 = 39
    DRV_CANFDSPI_TimeStampPrescalerSet(DRV_CANFDSPI_INDEX_0, 39);

    // Set TBC to zero
    DRV_CANFDSPI_TimeStampSet(DRV_CANFDSPI_INDEX_0, 0);

    // Enable TBC
    DRV_CANFDSPI_TimeStampEnable(DRV_CANFDSPI_INDEX_0);

}

    /* Initialize RX and TX interrupt pins */
int8_t can_init_interrupts()
{
    int8_t error = 0;
    // Clear Main Interrupts
    error += DRV_CANFDSPI_ModuleEventClear(DRV_CANFDSPI_INDEX_0, CAN_ALL_EVENTS);

    // Configure transmit and receive interrupt for TXQ and FIFO 2
    error += DRV_CANFDSPI_TransmitChannelEventEnable(DRV_CANFDSPI_INDEX_0, CAN_TXQUEUE_CH0, CAN_TX_FIFO_NOT_FULL_EVENT);
    error += DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2, CAN_RX_FIFO_NOT_EMPTY_EVENT);
    error += DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0, CAN_TX_EVENT | CAN_RX_EVENT);

}

    /* Receive message using FIFO status */
CAN_RX_MSGOBJ* can_receive_fifo(uint16_t *sid, uint8_t *rxd, uint8_t *len)
{
    // Receive Message Object
    CAN_RX_MSGOBJ rxObj;

    // Check that FIFO is not empty
    CAN_RX_FIFO_EVENT rxFlags;

    DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2, &rxFlags);

    if (rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
        // Read message and UINC
        DRV_CANFDSPI_ReceiveMessageGet(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2, &rxObj, rxd, MAX_DATA_BYTES);
        *sid = rxObj.bF.id.SID;
        *len = rxObj.bF.ctrl.DLC;
        return &rxObj;
        // Example of message processing
        /*
        uint32_t ts; // time stamp
        if (rxObj.bF.id.SID == 0x300 && rxObj.bF.ctrl.IDE == 0) {
            Nop();
            Nop();
            ts = rxObj.bF.timeStamp;
        }
        */
    }
    return NULL;

}

    /* Transmit message using interrupt pin */ // Arduino PD4
uint8_t can_transmit(uint16_t sid, uint8_t *txd, CAN_DLC dlc)
{
    // Configure transmit message
    CAN_TX_MSGOBJ txObj;

    // Initialize ID and Control bits
    txObj.word[0] = 0;
    txObj.word[1] = 0;

    txObj.bF.id.SID = sid; // Standard or Base ID
    txObj.bF.id.EID = 0; // Extended ID

    txObj.bF.ctrl.FDF = 0; // 1: CAN FD frame 0: CAN 2.0
    txObj.bF.ctrl.BRS = 1; // Switch bit rate
    txObj.bF.ctrl.IDE = 0; // Extended frame
    txObj.bF.ctrl.RTR = 0; // Not a remote frame request
    txObj.bF.ctrl.DLC = dlc; // # of data bytes, any from 1 to 8 for CAN 2.0
    // Sequence: doesn't get transmitted, but will be stored in TEF
    txObj.bF.ctrl.SEQ = 2;

    // Initialize transmit data
    // Remeber to commtent it out later
    uint8_t i;
    for (i = 0; i < DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC); i++) {
        txd[i] = i;
    }

    // Check that FIFO is not full
    // APP_TX_INT queries the input pin of the MCU, which is connected to nINT0
    if (APP_TX_INT()) {
        bool flush = true;

        // Load message and transmit
        DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0, CAN_TXQUEUE_CH0, &txObj, txd,
                DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC), flush);
    }
    else
        return 1; // error
    return 0;
    
}

    /* Receive message using interrupt pin */ // Arduino PD2
CAN_RX_MSGOBJ* can_receive(uint16_t *sid, uint8_t *rxd, uint8_t *len)
{
    // Receive Message Object
    CAN_RX_MSGOBJ rxObj;

    // Check that FIFO is not empty
    // APP_RX_INT queries the input pin of the MCU, which is connected to nINT1
    if (APP_RX_INT()) {
        // Set buffer size
        //rxd = malloc(MAX_DATA_BYTES*sizeof(uint8_t));
        // Read message and UINC
        DRV_CANFDSPI_ReceiveMessageGet(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2, &rxObj, rxd, MAX_DATA_BYTES);
        *sid = rxObj.bF.id.SID;
        *len = rxObj.bF.ctrl.DLC;
        return &rxObj;

        // Example of message processing
        /*
        uint32_t ts; // time stamp
        if (rxObj.bF.id.SID == 0x45A && rxObj.bF.ctrl.IDE == 0) {
            Nop();
            Nop();
            ts = rxObj.bF.timeStamp;

            return rxd;
        }
        */

    }
    return NULL;

}

    /* Request Configuration mode */
void can_request_config()
{
    // Select Configuration Mode
    DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_CONFIGURATION_MODE);

}

    /* Wait for Configuration mode, so it is safe to reset */
bool can_wait_for_config()
{
    // MCP2517FD will finish transmit message first
    // We need to make sure transmit is done before we go to APP_STATE_INIT and reset the MCP2517FD
    // Otherwise, we will get error frames

    opMode = DRV_CANFDSPI_OperationModeGet(DRV_CANFDSPI_INDEX_0);

    if (opMode != CAN_CONFIGURATION_MODE) {
        return 1;
    } else {
        return 0;
    }

}


/*******************************************************************************
 End of File
 */
