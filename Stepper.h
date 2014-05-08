#ifndef __STEPPER_H
#define __STEPPER_H

// ---------------- Motor Configuration Values -------------- //
#define MOTOR_PWM_FCY              40000       // The motor Drive PWM Frequency in 40KHz
#define DEADTIME_NANO_SECONDS      600         // This is the deadtime between the motor drive mosfets

#define STEPS_PER_SECOND           100         // The motor will move at 100 steps per second or 1/2 Revolution per Second

#define PWM_TO_MICROSTEP_RATIO      6          // This sets the speed of the AC waveform when changing positions.  It will deppend on the motor circuit
#define MOTOR_LOW_POWER_DELAY      .1          // This is the time (in seconds) that the system wait with no motor movement to switch to low power mode
                                               // This value must be less than or equal to 2^16-1 times the PWM period (about 1.6 seconds at 40KHz)

#define MOTOR_MINIMUM_POSITION      100        // Under normal operation (not Zero Find) the motor position can not go below this value
#define MOTOR_MAXIMUM_POSITION      800        // Under normal operation (not Zero Find) the motor position can not go above this value
#define MOTOR_DEFAULT_HOME_POSITION 670        // This is the default home position that is load at boot-up.  It should be overwritten by the PLC



void InitPWM(void);
/*
  This function shold be called to initilize the PWM at startup.
  If the PWM module is shutdown due to fault (or some other reason), this function can be called to restart it.
  After this function is called, the PWM is running, the H bridge is running, motor is energized.
*/




void SetMotorTarget(unsigned int position_type, unsigned int value);
/*
  This function should be called to set the motor target position
  
  The target can be specified as an absolution position or steps (clockwise/counterclockwise) from the current position.
*/

#define POSITION_TYPE_RELATIVE_COUNTER_CLOCKWISE 0
#define POSITION_TYPE_RELATIVE_CLOCKWISE         1
#define POSITION_TYPE_ABSOLUTE_POSITION          2


typedef struct {
  unsigned int current_position;
  unsigned int target_position;
  unsigned int home_position;
  unsigned int max_position;
  unsigned int min_position;
  unsigned char motor_motion;
  unsigned int adc_motor_current_a;
  unsigned int adc_motor_current_b;
} STEPPER_MOTOR;


extern STEPPER_MOTOR afc_motor;

extern unsigned int adc_parameter_input;
extern unsigned int adc_motor_current_a;
extern unsigned int adc_motor_current_b;

#endif
