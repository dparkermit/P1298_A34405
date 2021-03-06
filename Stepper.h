#ifndef __STEPPER_H
#define __STEPPER_H

// ---------------- Motor Configuration Values -------------- //
#define MOTOR_PWM_FCY              40000       // The motor Drive PWM Frequency in 40KHz
#define DEADTIME_NANO_SECONDS      400         // This is the deadtime between the motor drive mosfets

#define STEPS_PER_SECOND           100         // The motor will move at 100 steps per second or 1/2 Revolution per Second
#define STEPS_PER_SECOND_SLOW      25          //

#define MICRO_STEPPING_RESOLUTION   4          // This will break each "step" into this many smaller units

#define PWM_TO_MICROSTEP_RATIO      6          // This sets the speed of the AC waveform when changing positions.  It will deppend on the motor circuit
#define PWM_TO_MICROSTEP_RATIO_SLOW_MODE      36         // This sets the speed of the AC waveform when changing positions.  It will deppend on the motor circuit
#define MOTOR_LOW_POWER_DELAY      .1          // This is the time (in seconds) that the system wait with no motor movement to switch to low power mode
                                               // This value must be less than or equal to 2^16-1 times the PWM period (about 1.6 seconds at 40KHz)
#define MOTOR_MINIMUM_POSITION      100        // Under normal operation (not Zero Find) the motor position can not go below this value
#define MOTOR_MAXIMUM_POSITION      800        // Under normal operation (not Zero Find) the motor position can not go above this value

#define T2_CONFIG_VALUE                     0b1000000000110000   // Timer On and 256 Prescale
#define AFC_MOTOR_T2_PERIOD_VALUE           (unsigned int)(FCY/256/STEPS_PER_SECOND/MICRO_STEPPING_RESOLUTION)
#define AFC_MOTOR_T2_PERIOD_VALUE_SLOW      (unsigned int)(FCY/256/STEPS_PER_SECOND_SLOW/MICRO_STEPPING_RESOLUTION)







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

#define MOTOR_MOTION_COUNTER_CLOCKWISE  0
#define MOTOR_MOTION_CLOCKWISE          1
#define MOTOR_MOTION_STOPPED            2



extern STEPPER_MOTOR afc_motor;

extern unsigned int adc_parameter_input;
extern unsigned int adc_motor_current_a;
extern unsigned int adc_motor_current_b;
extern unsigned int adc_analog_value_input;

#endif
