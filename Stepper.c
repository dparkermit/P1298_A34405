#include <p30f2023.h>
#include "Stepper.h"
#include "Tables.h"
#include "Main.h"


// Global Variables
unsigned int motor_motion;
STEPPER_MOTOR afc_motor;

// LOCAL Variables

const unsigned int PWMHighPowerTable[128] = {FULL_POWER_TABLE_VALUES};
const unsigned int PWMLowPowerTable[128] = {LOW_POWER_TABLE_VALUES};



unsigned int counterTablePWM;
unsigned int counterPWM;
unsigned int motor_stopped_counter; 

unsigned char motor_slowdown;

unsigned char adc_average_counter;
unsigned int adc_parameter_input_accumulator;
unsigned int adc_motor_current_a_accumulator;
unsigned int adc_motor_current_b_accumulator;
unsigned int adc_home_position_accumulator;

unsigned int adc_parameter_input;
unsigned int adc_motor_current_a;
unsigned int adc_motor_current_b;


#define PWM_PWMCON_VALUE        0b0000000000000000
/* 
   Clear Fault Interrupt flag 
   Clear Current Limit Interrupt flag 
   Clear PWM Trigger Interrupt flag 
   Disable Fault Interrupt 
   Disable Current Limit Interrupt 
   Disable Trigger Interrupt 
   Time base is read from PTMR 
   Duty cycle is read from PDC 
   No extenal reset for PTMR
   PWM updates synchronized to PWM time base
*/

#define PWM_IOCON_VALUE         0b1100000000000000
/* 
   PWM Module controls High output
   PWM Module controls Low output 
   Output Polarity is active High 
   Low Output Polarity is active High 
   Comp. output mode 
   PWM Generator Provides data for High output 
   PWM Generator Provides data for Low output 
   If overide is enabled the outputs will be low 
   Overide occurs on the next clock cycle, not the next PWM cycle 
*/


#define PWM_IOCON_VALUE_OVERRIDE  0b0000001100000000
/* 
   This value is loaded to shutdown the PWM pair by setting both gate drive outputs to zero.

*/
 
void InitPWM(void) {

  counterTablePWM = 0;
  adc_average_counter = 0;
  adc_parameter_input_accumulator = 0;
  adc_motor_current_a_accumulator = 0;
  adc_motor_current_b_accumulator = 0;
  
  PTPER = PTPER_VALUE;                      /* PTPER = FCY*32(PLL)/(Desired PWM Freq.) Refer to PWM section for more details   */

  /* Initialize PWM Generator 1 */
  IOCON1   = PWM_IOCON_VALUE;
  PWMCON1  = PWM_PWMCON_VALUE;

  DTR1     = DEADTIME;
  ALTDTR1  = DEADTIME;
  PDC1     = (unsigned int)(PTPER_VALUE/2);
  PHASE1   = 0;        

  /* Initialize PWM Generator 2 */
  IOCON2   = PWM_IOCON_VALUE;
  PWMCON2  = PWM_PWMCON_VALUE;

  DTR2     = DEADTIME;
  ALTDTR2  = DEADTIME;
  PDC2     = (unsigned int)(PTPER_VALUE/2);
  PHASE2   = 0;        


  /* Initialize PWM Generator 3 */
  IOCON3   = PWM_IOCON_VALUE;
  PWMCON3  = PWM_PWMCON_VALUE;

  DTR3     = DEADTIME;
  ALTDTR3  = DEADTIME;
  PDC3     = (unsigned int)(PTPER_VALUE/2);
  PHASE3   = 0;        


  /* Initialize PWM Generator 4 */
  IOCON4   = PWM_IOCON_VALUE;
  PWMCON4  = PWM_PWMCON_VALUE;

  DTR4     = DEADTIME;
  ALTDTR4  = DEADTIME;
  PDC4     = (unsigned int)(PTPER_VALUE/2);
  PHASE4   = 0;        


  SEVTCMP	                = 10;
  PTCONbits.SEIEN               = 1;
  PTCONbits.PTEN                = 1;        // Enable the PWM Module  
  
  afc_motor.max_position = MOTOR_MAXIMUM_POSITION;
  afc_motor.min_position = MOTOR_MINIMUM_POSITION;
  afc_motor.home_position = MOTOR_DEFAULT_HOME_POSITION;
  afc_motor.target_position = afc_motor.home_position;
  afc_motor.current_position = afc_motor.home_position;
  
}



void SetMotorTarget(unsigned int position_type, unsigned int value) {
  unsigned int current_target;
  unsigned int new_target;
  
  current_target = afc_motor.target_position;
  
  if (position_type == POSITION_TYPE_ABSOLUTE_POSITION) {
    new_target = value;
  } else if (position_type == POSITION_TYPE_RELATIVE_CLOCKWISE) {
    if ((0xFFFF-value) > current_target) {
      new_target = current_target + value;
    } else {
      new_target = 0xFFFF;
    }
  } else if (position_type == POSITION_TYPE_RELATIVE_COUNTER_CLOCKWISE) {
    if (current_target >= value) {
      new_target = current_target - value;
    } else {
      new_target = 0;
    }    
  }
  if (new_target >= afc_motor.max_position) {
    new_target = afc_motor.max_position;
  } 
  if (new_target <= afc_motor.min_position) {
    new_target = afc_motor.min_position;
  }
  
  afc_motor.target_position = new_target;
}



/******************************************************************************
* Interrupt:     _PWMSpEventMatchInterrupt()
*
* Output:		None
*
* Overview:		ISR for PWM special event, executes duty cycle updates
*
* Note:			None
*******************************************************************************/
void __attribute__((interrupt(__save__(CORCON,SR)),auto_psv)) _PWMSpEventMatchInterrupt(void) {
  //void __attribute__((interrupt, no_auto_psv)) _PWMSpEventMatchInterrupt(void) {
  unsigned int counterTablePWM_32;
  unsigned int counterTablePWM_64;
  unsigned int counterTablePWM_96;
  _PSEMIF = 0;  


  // We average the adc_motor_currents and the parameter input over 64 motor PWM cycles (1.6mS)
  adc_average_counter++;
  if (adc_average_counter >= 64) {
    adc_average_counter = 0;
    adc_parameter_input = adc_parameter_input_accumulator;
    adc_motor_current_a = adc_motor_current_a_accumulator;
    adc_motor_current_b = adc_motor_current_b_accumulator;
    //afc_motor.home_position = adc_home_position_accumulator >> 5;  DPARKER - Set HOme position in software for now
    adc_parameter_input_accumulator = 0;
    adc_motor_current_a_accumulator = 0;
    adc_motor_current_b_accumulator = 0;
    adc_home_position_accumulator = 0;
  }
  adc_home_position_accumulator += ADCBUF9;
  adc_parameter_input_accumulator += ADCBUF11;
  adc_motor_current_a_accumulator += ADCBUF2;
  adc_motor_current_b_accumulator += ADCBUF3;
  _SWTRG1 = 1;  // Trigger conversion on motor currents
  _SWTRG5 = 1;  // Trigger conversion on parameter input
  _SWTRG4 = 1;  // Trigger conversion on Home input
  
  if (motor_motion != MOTOR_MOTION_STOPPED) {
    motor_stopped_counter = 0;
    motor_slowdown++;
    if (motor_slowdown >= MOTOR_SLOWDOWN_FACTOR) {
      motor_slowdown = 0;
      if (motor_motion == MOTOR_MOTION_CLOCKWISE) {
	  counterTablePWM--;
	} else {
	  counterTablePWM++;
	}
      counterTablePWM &= 0x007F;
      counterTablePWM_32 = counterTablePWM + 32;
      counterTablePWM_32 &= 0x007F;
      counterTablePWM_64 = counterTablePWM + 64;
      counterTablePWM_64 &= 0x007F;
      counterTablePWM_96 = counterTablePWM + 96;
      counterTablePWM_96 &= 0x007F;

      
      PDC1 = PWMHighPowerTable[counterTablePWM];
      PDC2 = PWMHighPowerTable[counterTablePWM_64];
      PDC3 = PWMHighPowerTable[counterTablePWM_32];
      PDC4 = PWMHighPowerTable[counterTablePWM_96];
      
      counterPWM++;
      if (counterPWM >= 32) {
	counterPWM = 0;
	if (motor_motion == MOTOR_MOTION_CLOCKWISE) {
	  if (afc_motor.current_position < 0xFFFF) {
	    afc_motor.current_position++;
	  }
	} else {
	  if (afc_motor.current_position > 0) {
	    afc_motor.current_position--;
	  }
	}
	motor_motion = MOTOR_MOTION_STOPPED;
      }
    }
  } else {
    motor_stopped_counter++;
    if (motor_stopped_counter >= MOTOR_DECREASE_CURRENT_PWM_CYCLES) {
      motor_stopped_counter = MOTOR_DECREASE_CURRENT_PWM_CYCLES;
      counterTablePWM &= 0x007F;
      counterTablePWM_32 = counterTablePWM + 32;
      counterTablePWM_32 &= 0x007F;
      counterTablePWM_64 = counterTablePWM + 64;
      counterTablePWM_64 &= 0x007F;
      counterTablePWM_96 = counterTablePWM + 96;
      counterTablePWM_96 &= 0x007F;

      PDC1 = PWMLowPowerTable[counterTablePWM];
      PDC2 = PWMLowPowerTable[counterTablePWM_64];
      PDC3 = PWMLowPowerTable[counterTablePWM_32];
      PDC4 = PWMLowPowerTable[counterTablePWM_96];
    }
  }

}




/******************************************************************************
* Interrupt:     _T2Interrupt
*
* Output:		None
*
* Overview:		T2 Interrupt
*
* Note:			None
*******************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
  _T2IF = 0;

  // Update the motor position
  if (motor_motion == MOTOR_MOTION_STOPPED) {
    if (afc_motor.target_position > afc_motor.max_position) {
      afc_motor.target_position = afc_motor.max_position;
    }
    if (afc_motor.target_position < afc_motor.min_position) {
      afc_motor.target_position = afc_motor.min_position;
    }
    if (afc_motor.current_position > afc_motor.target_position) {
      motor_motion = MOTOR_MOTION_COUNTER_CLOCKWISE;
    } else if (afc_motor.current_position < afc_motor.target_position) {
      motor_motion = MOTOR_MOTION_CLOCKWISE;
    } 
  }
  
}


/******************************************************************************
* Interrupt:     _INT2Interrupt()
*
* Output:	None
*
* Overview:	ISR for external interrupt INT2, triggers on falling edge for Over Current fault
*
* Note:		None
*******************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void) {
  // Shutdown the PWM and save the fault
  // Active the override regisiters (set to zero in the configuration)
  _INT2IF = 0;
  
  IOCON1 = PWM_IOCON_VALUE_OVERRIDE;
  IOCON2 = PWM_IOCON_VALUE_OVERRIDE;
  IOCON3 = PWM_IOCON_VALUE_OVERRIDE;
  IOCON4 = PWM_IOCON_VALUE_OVERRIDE;
 
  // DPARKER need to save that there was a fault somehow so that we can move to fault state
}
