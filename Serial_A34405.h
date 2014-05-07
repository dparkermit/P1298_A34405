#ifndef __SERIAL_A34405_H
#define __SERIAL_A34405_H

#include "ETM_BUFFER_BYTE_64.h"
#include <uart.h>

// Command List
#define CMD_READ_RAM_VALUE                              0x20
#define CMD_SET_TARGET_POSITION                         0x30
#define CMD_MOVE_CLOCKWISE                              0x32
#define CMD_MOVE_COUNTER_CLOCKWISE                      0x34
#define CMD_READ_EEPROM_REGISTER                        0x40
#define CMD_WRITE_EEPROM_REGISTER                       0x42
#define CMD_OVERCURRENT_SHUTDOWN_TEST                   0xE0
#define CMD_READ_AFC_ERROR_DATA_HISTORY                 0x50
#define CMD_READ_MEM_LOCATION                           0x54
#define CMD_SET_ERROR_OFFSET                            0x60
#define CMD_SET_HOME_POSITION                           0x36


// RAM Locations
#define RAM_READ_STATE                                  0x01
#define RAM_READ_VERSION                                0x02

#define RAM_READ_CURRENT_POSITION                       0x10
#define RAM_READ_TARGET_POSITION                        0x12
#define RAM_READ_HOME_POSITION                          0x14
#define RAM_READ_MAX_POSITION                           0x16
#define RAM_READ_MIN_POSITION                           0x18

#define RAM_READ_ADCBUF0                                0x30
#define RAM_READ_ADCBUF1                                0x31
#define RAM_READ_ADC_MOTOR_CURRENT_A                    0x32
#define RAM_READ_ADC_MOTOR_CURRENT_B                    0x33
#define RAM_READ_ADC_PARAMETER_INPUT                    0x3B

#define RAM_READ_SIGMA_DATA                             0x40
#define RAM_READ_DELTA_DATA                             0x41
#define RAM_READ_FREQUENCY_ERROR_FILTERED               0x42
#define RAM_READ_FREQUENCY_ERROR_OFFSET                 0x43
#define RAM_READ_NUMBER_PULSES_ON                       0x44

#define RAM_READ_PRF                                    0x50



#define COMMAND_BUFFER_EMPTY  0x00
#define COMMAND_BUFFER_FULL   0x02

#define COMMAND_LENGTH        9
#define SYNC_BYTE_1           0xF1
#define SYNC_BYTE_2           0xF2
#define SYNC_BYTE_3_RECEIVE   0xF3
#define SYNC_BYTE_3_SEND      0xF4




struct CommandStringStruct {
  unsigned char command_byte;
  unsigned char register_byte;
  unsigned char data_high_byte;
  unsigned char data_low_byte;
  unsigned char data_state;
};

extern struct CommandStringStruct command_string;
extern BUFFERBYTE64 uart1_input_buffer;
extern BUFFERBYTE64 uart1_output_buffer;

void DoSerialCommand(void);



/*
  SIGNAL       USB-COM422-PLUS (MALE)  <-------->  A30956-000 (J1 FEMALE)   SIGNAL
  TXD-               1                                   9                  RXD-
  TXD+               2                                   2                  RXD+
  RXD+               3                                   4                  TXD+
  RXD-               4                                   6                  TXD-
  GND                5                                   5                  GND
*/


#endif
