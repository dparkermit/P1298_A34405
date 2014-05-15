#include <p30fxxxx.h>
#include "Serial_A34405.h"
#include "Main.h"
#include "Stepper.h"
#include "Version_A34405.h"


/*
  Serial Commands

  A single command is stored in command_string
  If there is a valid command stored in command_string, then the command_string.data_state = COMMAND_BUFFER_FULL
  If there is NOT a volid command stored in command_string, then command_string.data_state = COMMAND_BUFFER_EMPTY
  
  
  When a byte is received on the UART it is transfered to the "uart1_input_buffer" by the UART receive interrupt - the input buffer is a circular buffer that is 64 Bytes deep
  (see buffer64.h for more infor on the buffer)
  
  Every time through the command loop (200us to 1ms) DoSerialCommand() will be called
  If the command_string is empty, then the input buffer is searched for a valid command (the oldest valid command will be moved to command_string)

  If a command was found OR the command_string was already full, then the command is executed.

  Assume an average execution cycle of 1mS and 9 bytes per command.  A command rate of 72 K Baund can be sustained. (57.6 K Baud Standard will work)
  
  Assume an average execution cycle of 500uS and 9 bytes per command, A command rate of 144 K Baud can be sustained (115.2 K Baud Standard should be safe) 

*/

void LookForCommand(void);
void ExecuteCommand(void);
unsigned char CheckCRC(unsigned int crc);
unsigned int MakeCRC(unsigned char command_byte, unsigned char register_byte, unsigned int data_word);
unsigned int ReadFromRam(unsigned int ram_location);
void SendCommand(unsigned char command_byte, unsigned char register_byte, unsigned int data_word);

struct CommandStringStruct command_string;
BUFFERBYTE64 uart1_input_buffer;
BUFFERBYTE64 uart1_output_buffer;

unsigned int *ram_pointer;

unsigned char data_logging_to_uart;

void DoSerialCommand(void) {
  /* 
     Look for a command and execute it.
  */

  if (command_string.data_state != COMMAND_BUFFER_FULL) {
    LookForCommand();
  }
  
  if (command_string.data_state == COMMAND_BUFFER_FULL) {
    ExecuteCommand();
  }
  
}



void LookForCommand(void) {
  unsigned char read_byte;
  unsigned int crc;
  /*
    If the state is "waitng for command" then it looks for a command in the buffer, if the state is "executing command" it does nothing
    
    To look for a command in the buffer.
    1) See if there are enough bytes in the buffer to contain a command.
    2) If there are look for command sync
       2b) If there are less bytes in the buffer than it takes to make a command, exit
    3) If command Syncs, check the checksum ^ increment the read_position as each byte is read
       3b) If command does not sync, increment the the read positon and return to step 1    
    4) If the checksum checks out, move the command data into the command data structure
    4b) If the checksum fails, return to step 1     
  */
  
  while ((command_string.data_state == COMMAND_BUFFER_EMPTY) && (BufferByte64BytesInBuffer(&uart1_input_buffer) >= COMMAND_LENGTH)) {
    // Look for a command
    read_byte = BufferByte64ReadByte(&uart1_input_buffer);
    if (read_byte == SYNC_BYTE_1) {
      read_byte = BufferByte64ReadByte(&uart1_input_buffer);
      if (read_byte == SYNC_BYTE_2) {
	read_byte = BufferByte64ReadByte(&uart1_input_buffer);
	if (read_byte == SYNC_BYTE_3_RECEIVE) {
	  // All of the sync bytes matched, this should be a valid command
	  command_string.command_byte   = BufferByte64ReadByte(&uart1_input_buffer);
	  command_string.data_high_byte = BufferByte64ReadByte(&uart1_input_buffer);
	  command_string.data_low_byte  = BufferByte64ReadByte(&uart1_input_buffer);
	  command_string.register_byte  = BufferByte64ReadByte(&uart1_input_buffer);
	  crc                           = BufferByte64ReadByte(&uart1_input_buffer);
	  crc                           = (crc << 8) + BufferByte64ReadByte(&uart1_input_buffer);
	  if (CheckCRC(crc)) {
	    command_string.data_state = COMMAND_BUFFER_FULL;
	  }
	}
      }
    }
  }
}

void SendLoggingDataToUart() {
  unsigned char byte;
  if (data_logging_to_uart) {
    BufferByte64WriteByte(&uart1_output_buffer, 0xFE);
    BufferByte64WriteByte(&uart1_output_buffer, (afc_motor.current_position >> 8));
    BufferByte64WriteByte(&uart1_output_buffer, (afc_motor.current_position & 0x00FF));
    BufferByte64WriteByte(&uart1_output_buffer, (afc_motor.target_position >> 8));
    BufferByte64WriteByte(&uart1_output_buffer, (afc_motor.target_position & 0x00FF));

    if (afc_data.sigma_data > 0xFF) {
      byte = 0xFF;
    } else {
      byte = afc_data.sigma_data;
    }
    BufferByte64WriteByte(&uart1_output_buffer, (byte & 0x00FF));

    if (afc_data.delta_data > 0xFF) {
      byte = 0xFF;
    } else {
      byte = afc_data.delta_data;
    }
    BufferByte64WriteByte(&uart1_output_buffer, (byte & 0x00FF));

    BufferByte64WriteByte(&uart1_output_buffer, afc_data.frequency_error_filtered);
    BufferByte64WriteByte(&uart1_output_buffer, (afc_data.pulses_on >> 8));
    BufferByte64WriteByte(&uart1_output_buffer, (afc_data.pulses_on & 0x00FF));
    if ((!U1STAbits.UTXBF) && (BufferByte64IsNotEmpty(&uart1_output_buffer))) {
      /*
	There is at least one byte available for writing in the outputbuffer and the transmit buffer is not full.
	Move a byte from the output buffer into the transmit buffer
	All subsequent bytes will be moved from the output buffer to the transmit buffer by the U1 TX Interrupt
      */
      U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
    }
  }
}

void SendCommand(unsigned char command_byte, unsigned char register_byte, unsigned int data_word) {
  unsigned int crc;
  if (!data_logging_to_uart) {
    crc = MakeCRC(command_byte, register_byte, data_word);
    BufferByte64WriteByte(&uart1_output_buffer, SYNC_BYTE_1);
    BufferByte64WriteByte(&uart1_output_buffer, SYNC_BYTE_2);
    BufferByte64WriteByte(&uart1_output_buffer, SYNC_BYTE_3_SEND);
    BufferByte64WriteByte(&uart1_output_buffer, command_byte);
    BufferByte64WriteByte(&uart1_output_buffer, (data_word >> 8));
    BufferByte64WriteByte(&uart1_output_buffer, (data_word & 0x00FF));
    BufferByte64WriteByte(&uart1_output_buffer, register_byte);
    BufferByte64WriteByte(&uart1_output_buffer, (crc >> 8));
    BufferByte64WriteByte(&uart1_output_buffer, (crc & 0x00FF));
  
    if ((!U1STAbits.UTXBF) && (BufferByte64IsNotEmpty(&uart1_output_buffer))) {
      /*
	There is at least one byte available for writing in the outputbuffer and the transmit buffer is not full.
	Move a byte from the output buffer into the transmit buffer
	All subsequent bytes will be moved from the output buffer to the transmit buffer by the U1 TX Interrupt
      */
      U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
    }
  }
}


void ExecuteCommand(void) {
  unsigned int data_word;
  unsigned int return_data_word;
  unsigned int return_command_byte;
  
  data_word = command_string.data_high_byte;
  data_word = data_word << 8;
  data_word = data_word + command_string.data_low_byte;
  
  return_data_word = data_word;
  return_command_byte = command_string.command_byte;
  switch (command_string.command_byte) 
    {

    case CMD_READ_RAM_VALUE:
      return_data_word = ReadFromRam(command_string.register_byte);
      break;

    case CMD_SET_TARGET_POSITION:
      // DPARKER only activate these commands if we are in software control mode
      // For the moment they are always active
      SetMotorTarget(POSITION_TYPE_ABSOLUTE_POSITION, data_word); 
      break;

    case CMD_MOVE_CLOCKWISE:
      // DPARKER only activate these commands if we are in software control mode
      // For the moment they are always active
      SetMotorTarget(POSITION_TYPE_RELATIVE_CLOCKWISE, data_word);
      break;

    case CMD_MOVE_COUNTER_CLOCKWISE:
      // DPARKER only activate these commands if we are in software control mode
      // For the moment they are always active
      SetMotorTarget(POSITION_TYPE_RELATIVE_COUNTER_CLOCKWISE, data_word);
      break;

    case CMD_DO_POSITION_AUTO_ZERO:
      auto_zero_requested = 1;
      break;

    case CMD_READ_EEPROM_REGISTER:
      break;
    
    case CMD_WRITE_EEPROM_REGISTER:
      break;

    case CMD_READ_AFC_ERROR_DATA_HISTORY:
      return_data_word = afc_data.frequency_error_history[command_string.register_byte];
      break;
      
    case CMD_READ_MEM_LOCATION:
      if (data_word >= 0x09FF) {
	data_word = 0x09FF;
      }
      data_word &= 0b1111111111111110;
      ram_pointer = data_word;
      // NOTE!!!! This will generate a complier warning (as it should for direct memory access)
      return_data_word = *ram_pointer;
      break;

    case CMD_SET_ERROR_OFFSET:
      if (command_string.register_byte == 0) {
	afc_data.frequency_error_offset = data_word;
      } else if (command_string.register_byte == 1) {
	afc_data.frequency_error_offset = 0;
	afc_data.frequency_error_offset -= data_word;
      }
      break;

    case CMD_SET_HOME_POSITION:
      M24LC64FWriteWord(&U23_M24LC64F, EEPROM_REGISTER_HOME_POSITION, data_word);
      afc_motor.home_position = data_word;
      break;

    case CMD_DATA_LOGGING:
      if (command_string.register_byte == 1) {
	data_logging_to_uart = 1;
      } else {
      	data_logging_to_uart = 0;
      }
      break;

    case CMD_OVERCURRENT_SHUTDOWN_TEST:
      /*
	// DO NOTHING
	IOCON1 = 0b0000001100000000;
	IOCON2 = 0b0000001100000000;
	IOCON3 = 0b0000001100000000;
	IOCON4 = 0b0000001100000000;
      */
      break;

    }

  // Echo the command that was recieved back to the controller
  SendCommand(return_command_byte, command_string.register_byte, return_data_word);
  command_string.data_state = COMMAND_BUFFER_EMPTY;
}



unsigned int ReadFromRam(unsigned int ram_location) {
  unsigned int data_return;

  switch (ram_location) 
    {

    case RAM_READ_STATE:
      data_return = control_state;
      break;

    case RAM_READ_VERSION:
      data_return = A34405_SOFTWARE_VERSION;
      break;

    case RAM_READ_CURRENT_POSITION:
      data_return = afc_motor.current_position;
      break;
      
    case RAM_READ_TARGET_POSITION:
      data_return = afc_motor.target_position;
      break;

    case RAM_READ_HOME_POSITION:
      data_return = afc_motor.home_position;
      break;

    case RAM_READ_MAX_POSITION:
      data_return = afc_motor.max_position;
      break;

    case RAM_READ_MIN_POSITION:
      data_return = afc_motor.min_position;
      break;

    case RAM_READ_ADCBUF0:
      data_return = ADCBUF0;
      break;

    case RAM_READ_ADCBUF1:
      data_return = ADCBUF1;
      break;
      
    case RAM_READ_ADC_MOTOR_CURRENT_A:
      data_return = afc_motor.adc_motor_current_a;
      break;

    case RAM_READ_ADC_MOTOR_CURRENT_B:
      data_return = afc_motor.adc_motor_current_b;
      break;

    case RAM_READ_ADC_PARAMETER_INPUT:
      data_return = adc_parameter_input;
      break;

    case RAM_READ_SIGMA_DATA:
      data_return = afc_data.sigma_data;
      break;

    case RAM_READ_DELTA_DATA:
      data_return = afc_data.delta_data;
      break;

    case RAM_READ_FREQUENCY_ERROR_FILTERED:
      data_return = afc_data.frequency_error_filtered;
      break;
      
    case RAM_READ_FREQUENCY_ERROR_OFFSET:
      data_return = afc_data.frequency_error_offset;
      break;

    case RAM_READ_NUMBER_PULSES_ON:
      data_return = afc_data.pulses_on;
      break;

    case RAM_READ_PRF:
      data_return = pulse_frequency;
      break;


    }  
  
  return data_return;
}


unsigned int MakeCRC(unsigned char command_byte, unsigned char register_byte, unsigned int data_word) {
  unsigned int crc;
  crc = SYNC_BYTE_1 + SYNC_BYTE_2 + SYNC_BYTE_3_SEND;
  crc += command_byte + register_byte;
  crc += (data_word >> 8);
  crc += (data_word & 0x00FF);
  
  return crc;
  // DPAKRER Make real CRC
}


unsigned char CheckCRC(unsigned int crc) {
  unsigned int crcCheck;
  // At the moment the CRC is just a checksum
  crcCheck = SYNC_BYTE_1 + SYNC_BYTE_2 + SYNC_BYTE_3_RECEIVE; 
  crcCheck += command_string.command_byte + command_string.register_byte;
  crcCheck += command_string.data_high_byte + command_string.data_low_byte;
  if (crcCheck == crc) {
    return 1;
  } else {
    return 0;
  }
  // DPARKER make Real CRC

}



//void _ISRNOPSV _U1RXInterrupt(void) {
void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) _U1RXInterrupt(void) {
  _U1RXIF = 0;
  while (U1STAbits.URXDA) {
    BufferByte64WriteByte(&uart1_input_buffer, U1RXREG);
  }
}



//void _ISRNOPSV _U1TXInterrupt(void) {
void __attribute__((interrupt(__save__(CORCON,SR)),no_auto_psv)) _U1TXInterrupt(void) {
  _U1TXIF = 0;
  while ((!U1STAbits.UTXBF) && (BufferByte64BytesInBuffer(&uart1_output_buffer))) {
    /*
      There is at least one byte available for writing in the outputbuffer and the transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
    */
    U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
  }
}

