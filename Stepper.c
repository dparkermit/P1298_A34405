#include <p30f2023.h>
#include "Stepper.h"
#include "Tables.h"
#include "Main.h"


// Global Variables
STEPPER_MOTOR afc_motor;                         // Data structure that holds the motor parameters
unsigned int adc_parameter_input;                // This stores the "parameter select" value


// LOCAL Variables
const unsigned int PWMHighPowerTable[128] = {FULL_POWER_TABLE_VALUES};
const unsigned int PWMLowPowerTable[128] = {LOW_POWER_TABLE_VALUES};

unsigned int pwm_table_index;                    // This stores the location in the microstepping table that we will use next
unsigned int counter_1_32_micro_steps;           // This stores who many steps through the 1/32 step microstep table we are
unsigned int motor_stopped_counter_pwm_cycles;   // This counts how many PWM cycles the motor has been stationary for, this is used to figure out when to switch to low power mode
unsigned char pwm_microstep_counter;             // We hold each position in the microstep table for PWM_TO_MICROSTEP_RATIO pwm cycles.  This counter is used for that

unsigned char adc_average_counter;               // The "low speed" adc inputs (everything not sigma or delta), are samples every PWM cycle (40 KHz) and then averaged.
unsigned int adc_parameter_input_accumulator;    // This stores the data for the "parameter" input that we average
unsigned int adc_motor_current_a_accumulator;    // This stores the data for the "motor current a" input that we average
unsigned int adc_motor_current_b_accumulator;    // This stores the data for the "motor current b" input that we average
unsigned int adc_home_position_accumulator;      // This stores the data for the "home position" input that we average


#define MOTOR_MOTION_COUNTER_CLOCKWISE  0
#define MOTOR_MOTION_CLOCKWISE          1
#define MOTOR_MOTION_STOPPED            2

#define DEADTIME                             (unsigned int)(DEADTIME_NANO_SECONDS/1.07)
#define PTPER_VALUE                          (unsigned int)(FCY*32/MOTOR_PWM_FCY)
#define MOTOR_DECREASE_CURRENT_PWM_CYCLES    (unsigned int) (MOTOR_PWM_FCY * MOTOR_LOW_POWER_DELAY)

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

// See H File for documentation 
void InitPWM(void) {

  pwm_table_index = 0;
  adc_average_counter = 0;
  adc_parameter_input_accumulator = 0;
  adc_motor_current_a_accumulator = 0;
  adc_motor_current_b_accumulator = 0;
  
  PTPER = PTPER_VALUE;

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


// See H file for documentation
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




/*
  This interrupt handels the PWM special event trigger
  This will happen once per PWM cycle near the start of the PWM cycle timer
  
  This function does a couple of things.
  1) It is the timer for the ADC sampeling and filtering - This could get moved to another timer interrupt if we want
  2) It manages the PWM function.
     2a) If the motor is moving, it steps through the microstep table to generate one smooth full step
     2b) If the motor is not moving for a long enough period of time, it reduces the motor holding current

*/
void __attribute__((interrupt(__save__(CORCON,SR)),auto_psv)) _PWMSpEventMatchInterrupt(void) {
  unsigned int pwm_slow_down_value;
  unsigned int pwm_table_index_32;
  unsigned int pwm_table_index_64;
  unsigned int pwm_table_index_96;
  _PSEMIF = 0;  


    // --------------  BEGIN ADC AVERAGING -------------- //
  // We average the adc_motor_currents and the parameter input over 64 motor PWM cycles (1.6mS at 40KHz)
  adc_average_counter++;
  if (adc_average_counter >= 64) {
    adc_average_counter = 0;
    adc_parameter_input = adc_parameter_input_accumulator;
    afc_motor.adc_motor_current_a = adc_motor_current_a_accumulator;
    afc_motor.adc_motor_current_b = adc_motor_current_b_accumulator;
    //afc_motor.home_position = adc_home_position_accumulator >> 5;  DPARKER - Set HOme position in software for now
    adc_parameter_input_accumulator = 0;
    adc_motor_current_a_accumulator = 0;
    adc_motor_current_b_accumulator = 0;
    adc_home_position_accumulator = 0;
  }
  _SWTRG1 = 0;  // Trigger conversion on motor currents
  _SWTRG5 = 0;  // Trigger conversion on parameter input
  _SWTRG4 = 0;  // Trigger conversion on Home input
  adc_home_position_accumulator += ADCBUF9;
  adc_parameter_input_accumulator += ADCBUF11;
  adc_motor_current_a_accumulator += ADCBUF2;
  adc_motor_current_b_accumulator += ADCBUF3;
  _SWTRG1 = 1;  // Trigger conversion on motor currents
  _SWTRG5 = 1;  // Trigger conversion on parameter input
  _SWTRG4 = 1;  // Trigger conversion on Home input
  // --------------  END ADC AVERAGING CODE -------------- //



  // -------------- BEGIN MOTOR PWM CONTROL CODE ------- //
  if (afc_motor.motor_motion != MOTOR_MOTION_STOPPED) {
  // Move through the micro step table if the motor is moving
    motor_stopped_counter_pwm_cycles = 0;
    pwm_microstep_counter++;
    if ((control_state == STATE_AFC_PULSING) && (afc_data.fast_afc_done)) {
      // We know that we are moving the motor slowly, so slow down the motor step transitions to damp out osciallations
      // DPARKER, it would probably be safest to limit the motor movement interrupt here as well (or do both in the same location)
      pwm_slow_down_value = PWM_TO_MICROSTEP_RATIO_SLOW_MODE;
      PR2 = T2_PERIOD_VALUE_SLOW;
    } else {
      pwm_slow_down_value = PWM_TO_MICROSTEP_RATIO;
      PR2 = T2_PERIOD_VALUE;
    }
    if (pwm_microstep_counter >= pwm_slow_down_value) {
      pwm_microstep_counter = 0;
      if (afc_motor.motor_motion == MOTOR_MOTION_CLOCKWISE) {
	  pwm_table_index--;
	} else {
	  pwm_table_index++;
	}
      pwm_table_index &= 0x007F;
      pwm_table_index_32 = pwm_table_index + 32;
      pwm_table_index_32 &= 0x007F;
      pwm_table_index_64 = pwm_table_index + 64;
      pwm_table_index_64 &= 0x007F;
      pwm_table_index_96 = pwm_table_index + 96;
      pwm_table_index_96 &= 0x007F;

      
      PDC1 = PWMHighPowerTable[pwm_table_index];
      PDC2 = PWMHighPowerTable[pwm_table_index_64];
      PDC3 = PWMHighPowerTable[pwm_table_index_32];
      PDC4 = PWMHighPowerTable[pwm_table_index_96];
      
      counter_1_32_micro_steps++;
      if (counter_1_32_micro_steps >= 32) {
	// We have moved a full Step
	counter_1_32_micro_steps = 0;
	if (afc_motor.motor_motion == MOTOR_MOTION_CLOCKWISE) {
	  if (afc_motor.current_position < 0xFFFF) {
	    afc_motor.current_position++;
	  }
	} else {
	  if (afc_motor.current_position > 0) {
	    afc_motor.current_position--;
	  }
	}
	afc_motor.motor_motion = MOTOR_MOTION_STOPPED;
      }
    }
  } else {
    // If the motor is not moving increment the timer that tells us how long the motor has been stopped for
    // If enough time has passed, reduce the holding current by selected the low power pwm table
    motor_stopped_counter_pwm_cycles++;
    if (motor_stopped_counter_pwm_cycles >= MOTOR_DECREASE_CURRENT_PWM_CYCLES) {
      motor_stopped_counter_pwm_cycles = MOTOR_DECREASE_CURRENT_PWM_CYCLES;
      pwm_table_index &= 0x007F;
      pwm_table_index_32 = pwm_table_index + 32;
      pwm_table_index_32 &= 0x007F;
      pwm_table_index_64 = pwm_table_index + 64;
      pwm_table_index_64 &= 0x007F;
      pwm_table_index_96 = pwm_table_index + 96;
      pwm_table_index_96 &= 0x007F;

      PDC1 = PWMLowPowerTable[pwm_table_index];
      PDC2 = PWMLowPowerTable[pwm_table_index_64];
      PDC3 = PWMLowPowerTable[pwm_table_index_32];
      PDC4 = PWMLowPowerTable[pwm_table_index_96];
    }
  }
  // -------------- END MOTOR PWM CONTROL CODE ------- //
  
}




/*
  The T2 interrupt controls the motor movent
  The maximum speed of the motor is one step per _T2 interrupt
  The maximum speed of the motor is set by setting the time of the _T2 interrupt 
*/
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
  _T2IF = 0;

  // Update the motor position
  // If the motor is already moving, do not adjust it's target
  // If the motor is not moving and is not at it's target position, move a step to the target position
  if (afc_motor.motor_motion == MOTOR_MOTION_STOPPED) {
    if (afc_motor.target_position > afc_motor.max_position) {
      afc_motor.target_position = afc_motor.max_position;
    }
    if (afc_motor.target_position < afc_motor.min_position) {
      afc_motor.target_position = afc_motor.min_position;
    }
    if (afc_motor.current_position > afc_motor.target_position) {
      afc_motor.motor_motion = MOTOR_MOTION_COUNTER_CLOCKWISE;
    } else if (afc_motor.current_position < afc_motor.target_position) {
      afc_motor.motor_motion = MOTOR_MOTION_CLOCKWISE;
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
