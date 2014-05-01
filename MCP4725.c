#include "MCP4725.h"
#include "ETM_I2C.h"
#include "Main.h"


MCP4725 U24_MCP4725;



#define MCP4725_POWER_DOWN_OFF  0b00000000
#define MCP4725_POWER_DOWN_1K   0b00010000
#define MCP4725_POWER_DOWN_10K  0b00100000
#define MCP4725_POWER_DOWN_100K 0b00110000
    


void SetupMCP4725(MCP4725* ptr_MCP4725) {
  ConfigureI2C(ptr_MCP4725->i2c_port, I2CCON_DEFAULT_SETUP_PIC30F, I2C_CLK, FCY, I2C_PGD_CONST_FCY);
  MCP4725UpdateFast(ptr_MCP4725);
}


void MCP4725UpdateFast(MCP4725* ptr_MCP4725) {
  unsigned int error_check;
  unsigned char dataHB;
  unsigned char dataLB;

  dataLB = (ptr_MCP4725->data_12_bit & 0x00FF);
  dataHB = ((ptr_MCP4725->data_12_bit >> 8) & 0x00FF);
  dataHB |= MCP4725_POWER_DOWN_OFF;
  error_check = WaitForI2CBusIdle(ptr_MCP4725->i2c_port); 
  error_check |= GenerateI2CStart(ptr_MCP4725->i2c_port); 
  error_check |= WriteByteI2C((ptr_MCP4725->address | I2C_WRITE_CONTROL_BIT),ptr_MCP4725->i2c_port);
  error_check |= WriteByteI2C(dataHB,ptr_MCP4725->i2c_port);
  error_check |= WriteByteI2C(dataLB,ptr_MCP4725->i2c_port);
  error_check |= GenerateI2CStop(ptr_MCP4725->i2c_port);
  if (error_check != 0) {
    ptr_MCP4725->write_error_count = ptr_MCP4725->write_error_count+1;
  }
}
