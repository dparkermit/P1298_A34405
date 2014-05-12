#ifndef __MAIN_H
#define __MAIN_H



/********************** dsPIC parameters *******************************/
#define FCY                         29100000    // FRC = 14.55MHz , PLL= X32 , FCY = 14.55e6*32/8/2 = 29.1e6
#define I2C_CLK                     100000      // I2C running at 100KHz
#define I2C_PGD_CONST_FCY           26          // This is the Pulse Gobler Delay in FCY Cycles
//#define I2C_BAUD_RATE_GENERATOR	    0x11B       // (unsigned int)((1/I2C_CLK - PGD_CONST)*FCY - 1)



// STATE DEFINITIONS
#define STATE_STARTUP            0x10
#define STATE_WAIT_FOR_AUTO_ZERO 0x18
#define STATE_MOTOR_ZERO         0x20
#define STATE_MOTOR_STARTUP_HOME 0x24
#define STATE_RESET              0x30
#define STATE_AFC_NOT_PULSING    0x40
#define STATE_AFC_PULSING        0x44
#define STATE_MANUAL_MODE        0x50
#define STATE_FAULT              0xF0


// ------------ TRIS CONFIGURATION ----------------------
#define TRISA_VALUE                   0b1111001011111111
#define TRISB_VALUE                   0b1111101111111111
#define TRISD_VALUE                   0b1111111111111111
#define TRISE_VALUE                   0b1111111111111111
#define TRISF_VALUE                   0b0011111111111111


// ------------ DIGITAL INPUT PINS ----------------------
#define PIN_MODE_SELECT               _RD0
#define ILL_AFC_MODE                  1

#define PIN_STEP_COUNTER_CLOCKWISE    _RB4
#define PIN_STEP_CLOCKWISE            _RB5
#define ILL_STEP_PIN_ACTIVE           1


// ------------ DIGITAL OUTPUT PINS ---------------------

#define PIN_BRIDGE_POWER_RELAY        _LATA8
#define OLL_BRIDGE_POWER_RELAY_CLOSED 1

#define PIN_FAULT_OUT_2               _LATA10

#define PIN_FAULT_OUT_3               _LATA11

#define PIN_SAMPLE_TRIGGER            _LATB10
#define OLL_TRIGGER_SH                1

#define PIN_SPARE_OUT                 _LATF14

#define PIN_FAULT_OUT_1               _LATF15


// GLOBAL DATA TYPES
typedef struct {
  unsigned int sigma_data;
  unsigned int delta_data;
  signed char frequency_error_filtered;
  signed char frequency_error_history[16];
  signed char frequency_error_offset;
  unsigned char data_pointer; 
  unsigned char trigger_complete;
  unsigned char fast_afc_done;
  signed char slow_response_error_counter;
  unsigned int pulses_on;
  unsigned int time_off_100ms_units;
} TYPE_AFC_DATA;

// GLOBAL VARIABLES
extern unsigned char control_state;
extern unsigned int prf_counter;
extern unsigned int pulse_frequency;
extern unsigned char software_auto_zero;
extern TYPE_AFC_DATA afc_data;



#endif
