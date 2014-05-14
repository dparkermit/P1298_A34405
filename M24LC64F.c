#include <libpic30.h>
#include <p30f2023.h>
#include "M24LC64F.h"
#include "ETM_I2C.h"


#define M24LC64F_READ_CONTROL_BIT          0b00000001
#define M24LC64F_WRITE_CONTROL_BIT         0b00000000

#define M24LC64F_MAX_16BIT_REGISTERS       0x1F3F
#define M24LC64F_WP_PIN_WRITE_ENABLE       0





void M24LC64FWriteWord(M24LC64F* ptr_M24LC64F, unsigned int register_location, unsigned int data) {
  unsigned int temp;
  unsigned char adr_high_byte;
  unsigned char adr_low_byte;
  unsigned char data_low_byte;
  unsigned char data_high_byte;
   unsigned int error_check;

  if (register_location <= M24LC64F_MAX_16BIT_REGISTERS) {
    // The register_location is in "word" locations and if greater than M24LC64F_MAX_16BIT_REGISTERS it will extended beyond the device
    
    data_high_byte = (data >> 8);
    data_low_byte = (data & 0x00FF);
    
    temp = (register_location*2);
    adr_high_byte = (temp >> 8);
    adr_low_byte = (temp & 0x00FF);
    
    error_check = WaitForI2CBusIdle(ptr_M24LC64F->i2c_port);
    error_check |= GenerateI2CStart(ptr_M24LC64F->i2c_port);
    error_check |= WriteByteI2C(ptr_M24LC64F->address | M24LC64F_WRITE_CONTROL_BIT, ptr_M24LC64F->i2c_port);
    error_check |= WriteByteI2C(adr_high_byte, ptr_M24LC64F->i2c_port);
    error_check |= WriteByteI2C(adr_low_byte, ptr_M24LC64F->i2c_port);
    error_check |= WriteByteI2C(data_low_byte, ptr_M24LC64F->i2c_port);
    error_check |= WriteByteI2C(data_high_byte, ptr_M24LC64F->i2c_port);
    error_check |= GenerateI2CStop(ptr_M24LC64F->i2c_port);
    
  }
}



unsigned int M24LC64FReadWord(M24LC64F* ptr_M24LC64F, unsigned int register_location) {
  unsigned int error_check;
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
 
    error_check = WaitForI2CBusIdle(ptr_M24LC64F->i2c_port);
    error_check |= GenerateI2CStart(ptr_M24LC64F->i2c_port);
    
    error_check |= WriteByteI2C(ptr_M24LC64F->address | M24LC64F_WRITE_CONTROL_BIT, ptr_M24LC64F->i2c_port);
    error_check |= WriteByteI2C(adr_high_byte, ptr_M24LC64F->i2c_port);
    error_check |= WriteByteI2C(adr_low_byte, ptr_M24LC64F->i2c_port);
    
    error_check |= GenerateI2CRestart(ptr_M24LC64F->i2c_port);
    error_check |= WriteByteI2C(ptr_M24LC64F->address | M24LC64F_READ_CONTROL_BIT, ptr_M24LC64F->i2c_port);
    data_low_byte = ReadByteI2C(ptr_M24LC64F->i2c_port);
    
    error_check |= GenerateI2CRestart(ptr_M24LC64F->i2c_port);
    error_check |= WriteByteI2C(ptr_M24LC64F->address | M24LC64F_READ_CONTROL_BIT, ptr_M24LC64F->i2c_port);
    data_high_byte = ReadByteI2C(ptr_M24LC64F->i2c_port);
     
    error_check |= GenerateI2CStop(ptr_M24LC64F->i2c_port);
  
  } else {
    data_low_byte = 0;
    data_high_byte = 0;
  }

  temp = data_high_byte;
  temp *= 256; 
  temp += data_low_byte;
  return temp;

}


