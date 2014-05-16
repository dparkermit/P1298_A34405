#ifndef __MAIN_H
#define __MAIN_H

/* 
   Resource Usage
  
   Interrupts
   INT0 - Trigger Input
   INT2 - NOT ACTIVE -- Motor Overcurrent
   PWM_SPECIAL_EVENT_TRIGGER - Used to generate current waveforms


   Hardware Resources

   TMR1 - Used for general timing functions - Set to .1 second
   TMR2 - Used to time the motor movement.  
   ADC Module - Used to Sample AFC feedback, Motor Currents, and Interact with the PLC
   UART1 - Used for Digital Serial Link
   I2C - Connected to ADC and possibly EEPROM
   

*/



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

//#define PIN_SAMPLE_ANALOG_INPUT       _RB8
#define PIN_SAMPLE_ANALOG_INPUT       _RB4
#define ILL_SAMPLE_NOW                1


// ------------ DIGITAL OUTPUT PINS ---------------------

#define PIN_BRIDGE_POWER_RELAY        _LATA8
#define OLL_BRIDGE_POWER_RELAY_CLOSED 1

#define PIN_FAULT_OUT_2               _LATA10

#define PIN_FAULT_OUT_3               _LATA11

#define PIN_SAMPLE_TRIGGER            _LATB10
#define OLL_TRIGGER_SH                1

#define PIN_SPARE_OUT                 _LATF14

#define PIN_FAULT_OUT_1               _LATF15



// GLOBAL VARIABLES
extern unsigned char control_state;
extern unsigned int pulse_frequency;
extern unsigned char auto_zero_requested;

extern M24LC64F U23_M24LC64F;


//extern _prog_addressT FLASH_address_afc_config;
//extern int afc_config_ram_copy[16];

#define EEPROM_REGISTER_HOME_POSITION     1
#define EEPROM_REGISTER_ERROR_OFFSET      2




#define PARAMETER_MOTOR_POSITION          0
#define PARAMETER_HOME_POSITION           1
#define PARAMETER_AFC_OFFSET              2
#define PARAMETER_PRF                     3
#define PARAMETER_UNUSED                  4
#define PARAMETER_AUTO_ZERO               5


// ----------------- CONFIGURE ADC ---------------- //

#define ADCON_DEFAULT (ADC_MOD_DIS | ADC_IDLE_CONT | ADC_SOFT_TRIG_DIS | ADC_DATA_INT | ADC_INT_EN_2CONV | ADC_ORDER_EVEN_FST | ADC_SAMP_SEQ | ADC_PLL_EN_FADC_14)
  
#define ADPCFG_DEFAULT    0b1111010111110000  // AN0,AN1,AN2,AN3,AN9,AN11
    
#define ADCPC0_DEFAULT (ADC_AN1_0_IR_GEN_DIS | ADC_AN1_0_TRIG_INDV_SW |  ADC_AN3_2_IR_GEN_DIS   | ADC_AN3_2_TRIG_INDV_SW)
#define ADCPC1_DEFAULT (ADC_AN5_4_IR_GEN_DIS | ADC_AN5_4_NOCONV       |  ADC_AN7_6_IR_GEN_DIS   | ADC_AN7_6_NOCONV)
#define ADCPC2_DEFAULT (ADC_AN9_8_IR_GEN_DIS | ADC_AN9_8_TRIG_INDV_SW |  ADC_AN11_10_IR_GEN_DIS | ADC_AN11_10_TRIG_INDV_SW)

// ---------------- CONFIGURE T1 --------------//
// Setupt T1 for .1 second Interrupt
#define T1_PERIOD_VALUE           (unsigned int)(FCY/256/10)
#define T1_CONFIG_VALUE           0b1000000000110000   // Timer On and 8 Prescale


#endif
