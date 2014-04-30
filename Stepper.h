#ifndef __STEPPER_H
#define __STEPPER_H

// ---------------- Motor Configuration Values -------------- //
#define MOTOR_PWM_FCY              40000       // The motor Drive PWM Frequency in 40KHz
#define STEPS_PER_SECOND           100         // The motor will move at 100 steps per second or 1/2 Revolution per Second
#define DEADTIME_NANO_SECONDS      600         // This is the deadtime between the motor drive mosfets
#define MOTOR_SLOWDOWN_FACTOR      6           // This sets the speed of the AC waveform when changing positions.  It will deppend on the motor circuit
#define MOTOR_LOW_POWER_DELAY      .1          // This is the time (in seconds) that the system wait with no motor movement to switch to low power mode


#define DEADTIME                (unsigned int)(DEADTIME_NANO_SECONDS/1.07)
#define PTPER_VALUE             (unsigned int)(FCY*32/MOTOR_PWM_FCY)
#define MOTOR_DECREASE_CURRENT_PWM_CYCLES    (unsigned long) (MOTOR_PWM_FCY * MOTOR_LOW_POWER_DELAY)


void InitPWM(void);


void SetMotorTarget(unsigned int position_type, unsigned int value);

#define POSITION_TYPE_RELATIVE_COUNTER_CLOCKWISE 0
#define POSITION_TYPE_RELATIVE_CLOCKWISE         1
#define POSITION_TYPE_ABSOLUTE_POSITION          2



extern unsigned int motor_motion;

#define MOTOR_MOTION_COUNTER_CLOCKWISE  0
#define MOTOR_MOTION_CLOCKWISE          1
#define MOTOR_MOTION_STOPPED            2



typedef struct {
  unsigned int current_position;
  unsigned int target_position;
  unsigned int home_position;
  unsigned int max_position;
  unsigned int min_position;
  unsigned int speed_rpm;
  unsigned int high_low_torque_select;
} STEPPER_MOTOR;


extern STEPPER_MOTOR afc_motor;

extern unsigned int adc_parameter_input;
extern unsigned int adc_motor_current_a;
extern unsigned int adc_motor_current_b;


#endif
