#pragma once

#include <Arduino.h>
#include <assert.h>

/* Symbolic names for bit rate of CAN message                                */
typedef enum {CAN_50KBPS, CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS} BITRATE;

/* Real speed for bit rate of CAN message                                    */
// uint32_t SPEEDS;

/* Symbolic names for formats of CAN message                                 */
typedef enum {STANDARD_FORMAT = 0, EXTENDED_FORMAT} CAN_FORMAT;

/* Symbolic names for type of CAN message                                    */
typedef enum {DATA_FRAME = 0, REMOTE_FRAME}         CAN_FRAME;


typedef struct
{
  uint32_t id;        /* 29 bit identifier                               */
  uint8_t  data[8];   /* Data field                                      */
  uint8_t  len;       /* Length of data field in bytes                   */
  uint8_t  ch;        /* Object channel(Not use)                         */
  uint8_t  format;    /* 0 - STANDARD, 1- EXTENDED IDENTIFIER            */
  uint8_t  type;      /* 0 - DATA FRAME, 1 - REMOTE FRAME                */
} CAN_msg_t;

typedef struct
{
    uint16_t baud_rate_prescaler;                /// [1 to 1024]
    uint8_t time_segment_1;                      /// [1 to 16]
    uint8_t time_segment_2;                      /// [1 to 8]
    uint8_t resynchronization_jump_width;        /// [1 to 4] (recommended value is 1)
} CAN_bit_timing_config_t;

#define CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE     1000
#define CAN_STM32_ERROR_MSR_INAK_NOT_SET         1001
#define CAN_STM32_ERROR_MSR_INAK_NOT_CLEARED     1002
#define CAN_STM32_ERROR_UNSUPPORTED_FRAME_FORMAT 1003



#define DEBUG 0



int16_t ComputeCANTimings(const uint32_t peripheral_clock_rate,
                          const uint32_t target_bitrate,
                          CAN_bit_timing_config_t* const out_timings);

void printRegister(const char * buf, uint32_t reg);
void CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2);
bool CANInit(BITRATE bitrate, int remap);
void CANReceive(CAN_msg_t* CAN_rx_msg);
void CANSend(CAN_msg_t* CAN_tx_msg);
uint8_t CANMsgAvail(void);


