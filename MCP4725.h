#ifndef __MCP4725_H
#define __MCP4725_H

typedef struct {
  unsigned char address;
  unsigned char i2c_port;
  unsigned int  data_12_bit;
  unsigned int  write_error_count;
} MCP4725;

#define MCP4725_ADDRESS_A0_0    0b11000000
#define MCP4725_ADDRESS_A0_1    0b11000010

extern MCP4725 U24_MCP4725;

void SetupMCP4725(MCP4725* ptr_MCP4725);

void MCP4725UpdateFast(MCP4725* ptr_MCP4725);

#endif
