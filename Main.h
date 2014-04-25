#ifndef __MAIN_H
#define __MAIN_H

/********************** dsPIC parameters *******************************/

#define FCY                         29100000    /* FRC = 14.55MHz , PLL= X32 , FCY = 14.55e6*32/8/2 = 29.1e6 */
#define MOTOR_PWM_FCY               40000       /* desired PWM frequency 40kHz */



#define I2C_CLK                     100000      /* I2C running at 100KHz */
#define PGD_CONST                   0.000000250
#define I2C_BAUD_RATE_GENERATOR	    0x11B       /*(unsigned int)((1/I2C_CLK - PGD_CONST)*FCY - 1)*/




/******************* MOTOR_PARAMETERS *******************************/

//#define DEADTIME                (unsigned int)(0.000005*FCY)
// DPARKER The deadtime is a function of 4x FCY so it should be multiplied by 4

#define DEADTIME                (unsigned int)(0.000005*FCY*4)
#define PTPER_VALUE             (unsigned int)(FCY*32/MOTOR_PWM_FCY)
#define FULL_DC                 PTPER_VALUE
#define HALF_DC                 (unsigned int)(PTPER_VALUE/2)
#define ZERO_DC                 0

//motor and winding direction - do not change - these values are used in different mathematical formulae
#define COUNTER_CLOCKWISE         0               //winding forward direction; used by setDir variables
#define CLOCKWISE                 1               //winding reverse direction; used by setDir variables
#define STOP                      2


// STATE DEFENITIONS

#define STATE_STARTUP            0x10
#define STATE_MOTOR_ZERO         0x20
#define STATE_MOTOR_STARTUP_HOME 0x24
#define STATE_RESET              0x30
#define STATE_AFC_NOT_PULSING    0x40
#define STATE_AFC_START_UP       0x44
#define STATE_AFC_STEADY_STATE   0x48
#define STATE_MANUAL_MODE        0x50
#define STATE_SOFTWARE_CONTROL   0x60
#define STATE_FAULT              0xF0



typedef struct {
  unsigned int current_position;
  unsigned int target_position;
  unsigned int home_position;
  unsigned int max_position;
  unsigned int min_position;
  unsigned int speed_rpm;
  unsigned int high_low_torque_select;
  
  
} STEPPER_MOTOR;






// INPUT PIN DEFINITIONS
#define PIN_MODE_SELECT               _RD0
#define ILL_AFC_MODE                  1

#define PIN_STEP_COUNTER_CLOCKWISE    _RB4
#define PIN_STEP_CLOCKWISE            _RB5
#define ILL_STEP_PIN_ACTIVE           1


// OUTPUT PIN DEFINITIONS
#define TRISA_VALUE                   0b1111001011111111
#define TRISB_VALUE                   0b1111101111111111
#define TRISD_VALUE                   0b1111111111111111
#define TRISE_VALUE                   0b1111111111111111
#define TRISF_VALUE                   0b0011111111111111

#define PIN_BRIDGE_POWER_RELAY        _LATA8
#define OLL_BRIDGE_POWER_RELAY_CLOSED 1

#define PIN_SAMPLE_TRIGGER            _LATB10
#define OLL_TRIGGER_SH                1

#define PIN

#define PIN_FAULT_OUT_1               _LATF15
#define PIN_FAULT_OUT_2               _LATA10
#define PIN_FAULT_OUT_3               _LATA11
#define PIN_SPARE_OUT                 _LATF14



#endif
