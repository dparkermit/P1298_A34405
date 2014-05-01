/*
  This is an extension of the built in I2C functions that provides more robust operation and easier use.
  This has been written work on devices with 1 or 2 I2C ports.

  
  Dan Parker
  2013_04_02
*/

/*
  This module has been validated on A34760 using MCP23017 functions (pic30F6014A).
  All other applications are suspect
*/


#ifndef __ETM_I2C_H
#define __ETM_I2C_H

#define I2CCON_DEFAULT_SETUP_30F6014A         0b1011000000100000
#define I2CCON_DEFAULT_SETUP_PIC30F           0b1011000000100000


#define I2C_WRITE_CONTROL_BIT   0b00000000
#define I2C_READ_CONTROL_BIT    0b00000001


void ConfigureI2C(unsigned char i2c_port, unsigned int configuration, unsigned long baud_rate, unsigned long fcy_clk, unsigned long pulse_gobbler_delay_fcy);
/*
  This function sets up the selected i2c_port with the configuation parameter that is passed in.
  It also calculates the baud rate register based on the selected baud rate and the system clock.
  pulse_gobbler_delay_fcy is the inverse of the pulse_gobbler_delay.  Good luck finding this on the data sheet
  Typical values range from 100ns - 900ns
*/


unsigned int WaitForI2CBusIdle(unsigned char i2c_port);
/*
  This will wait for bus idle on the specified I2C Bus
  
  The result will be 0x0000 if everything works properly
  The result will be 0xFA00 if there is an I2C bus fault
*/

unsigned int GenerateI2CStart(unsigned char i2c_port);
/*
  This will generate a start condition on the specified I2C bus
  
  The result will be 0x0000 if everything works properly
  The result will be 0xFA00 if there is an I2C bus fault
*/

unsigned int GenerateI2CRestart(unsigned char i2c_port);
/*
  This will generate a re-start condition on the specified I2C bus
  
  The result will be 0x0000 if everything works properly
  The result will be 0xFA00 if there is an I2C bus fault
*/

unsigned int WriteByteI2C(unsigned char data, unsigned char i2c_port);
/*
  This will write a byte to the specified I2C bus
  
  The result will be 0x0000 if everything works properly
  The result will be 0xFA00 if there is an I2C bus fault
*/

unsigned int ReadByteI2C(unsigned char i2c_port);
/*
  This will read a byte from the specified I2C bus
  
  The result will be 0x00dd if everything works properly
  The result will be 0xFA00 if there is an I2C bus fault
*/


unsigned int GenerateI2CStop(unsigned char i2c_port);
/*
  This will generate a stop condition on the specified I2C bus
  
  The result will be 0x0000 if everything works properly
  The result will be 0xFA00 if there is an I2C bus fault
*/


#define I2C_PORT                 0
#define I2C_PORT_1               1
#define I2C_PORT_2               2

extern unsigned int etm_i2c1_error_count; // This global variable counts the number of i2c_errors.  It may be useful for code bebugging and validation

#if defined(_I2C2MD)
extern unsigned int etm_i2c2_error_count; // This global variable counts the number of i2c_errors.  It may be useful for code bebugging and validation
#endif

#endif
