#include <libpic30.h>
#include "M24LC64F.h"
#include "ETM_I2C.h"

void ByteWriteI2C(unsigned char HighAdd, unsigned char LowAdd, unsigned char data,  unsigned char address, unsigned char i2c_port);

unsigned char ByteReadI2C(unsigned char HighAdd, unsigned char LowAdd, unsigned char address, unsigned char i2c_port);



void M24LC64FWriteWord(M24LC64F* ptr_M24LC64F, unsigned int register_location, unsigned int data) {
  unsigned int temp;
  unsigned char adr_high_byte;
  unsigned char adr_low_byte;
  unsigned char data_low_byte;
  unsigned char data_high_byte;

  if (register_location <= M24LC64F_MAX_16BIT_REGISTERS) {
    // The register_location is in "word" locations and if greater than M24LC64F_MAX_16BIT_REGISTERS it will extended beyond the device
    
    data_high_byte = (data >> 8);
    data_low_byte = (data & 0x00FF);
    
    temp = (register_location*2);
    adr_high_byte = (temp >> 8);
    adr_low_byte = (temp & 0x00FF);
    ByteWriteI2C(adr_high_byte, adr_low_byte, data_high_byte, ptr_M24LC64F->address, ptr_M24LC64F->i2c_port);
    
    temp = (register_location*2+1);
    adr_high_byte = (temp >> 8);
    adr_low_byte = (temp & 0x00FF);
    ByteWriteI2C(adr_high_byte, adr_low_byte, data_low_byte, ptr_M24LC64F->address, ptr_M24LC64F->i2c_port);
    
  }
}



unsigned int M24LC64FReadWord(M24LC64F* ptr_M24LC64F, unsigned int register_location) {
  unsigned int temp;
  unsigned char adr_high_byte;
  unsigned char adr_low_byte;
  unsigned char data_low_byte;
  unsigned char data_high_byte;
  
  if (register_location <= M24LC64F_MAX_16BIT_REGISTERS) {
    // The register_location is in "word" locations and if greater than M24LC64F_MAX_16BIT_REGISTERS it will extended beyond the device
    
    temp = (register_location*2);
    adr_high_byte = (temp >> 8);
    adr_low_byte = (temp & 0x00FF);
    data_high_byte = ByteReadI2C(adr_high_byte, adr_low_byte, ptr_M24LC64F->address, ptr_M24LC64F->i2c_port); 
    
    temp = (register_location*2+1);
    adr_high_byte = (temp >> 8);
    adr_low_byte = (temp & 0x00FF);
    data_low_byte = ByteReadI2C(adr_high_byte, adr_low_byte, ptr_M24LC64F->address, ptr_M24LC64F->i2c_port); 
    
  } else {
    data_low_byte = 0;
    data_high_byte = 0;
  }
  
  temp = data_high_byte*256 + data_low_byte;
  return temp;
}




void ByteWriteI2C(unsigned char HighAdd, unsigned char LowAdd, unsigned char data, unsigned char address, unsigned char i2c_port) {
  unsigned int error_check;

  error_check = WaitForI2CBusIdle(i2c_port);
  error_check |= GenerateI2CStart(i2c_port);
  error_check |= WriteByteI2C(address | M24LC64F_WRITE_CONTROL_BIT, i2c_port);
  error_check |= WriteByteI2C(HighAdd, i2c_port);
  error_check |= WriteByteI2C(LowAdd, i2c_port);
  error_check |= WriteByteI2C(data, i2c_port);
  error_check |= GenerateI2CStop(i2c_port);
}



unsigned char ByteReadI2C(unsigned char HighAdd, unsigned char LowAdd, unsigned char address, unsigned char i2c_port) {
  unsigned char data;
  unsigned int error_check;
  
  
  error_check = WaitForI2CBusIdle(i2c_port);
  error_check |= GenerateI2CStart(i2c_port);

  error_check |= WriteByteI2C(address | M24LC64F_WRITE_CONTROL_BIT, i2c_port);
  error_check |= WriteByteI2C(HighAdd, i2c_port);
  error_check |= WriteByteI2C(LowAdd, i2c_port);


  error_check |= GenerateI2CRestart(i2c_port);
  error_check |= WriteByteI2C(address | M24LC64F_READ_CONTROL_BIT, i2c_port);
  error_check |= ReadByteI2C(i2c_port);

  error_check |= GenerateI2CStop(i2c_port);
  
  
  data = error_check;
  return data;
}




