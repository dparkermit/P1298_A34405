#ifndef __SERIAL_A34405_H
#define __SERIAL_A34405_H
#include "ETM_BUFFER_BYTE_64.h"
#include <uart.h>

// Command List
#define CMD_READ_RAM_VALUE                              0x20
#define CMD_SET_TARGET_POSITION                         0x30
#define CMD_READ_EEPROM_REGISTER                        0x40
#define CMD_WRITE_EEPROM_REGISTER                       0x42






// RAM Locations
#define RAM_READ_STATE                                  0x01
#define RAM_READ_VERSION                                0x02








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
