#ifndef __AFC_H
#define __AFC_H


// PARAMETERS TO CONFIGURE THE AFC RESPONSE

#define MOTOR_MOVE_FULL_STEP_THRESHOLD         1   // This is error signal that will cause the motor to move a full step.
                                                   // Make this number larger to provide more stable action (at the expense of accuracy)
                                                   // This number is divided by the microstep ratio so it should be an integer multiple of the micro step ratio

#define SIGMA_DELTA_MINIMUM_ADC_READING        5   // If the sigma & delta readings are both less than we will assume there was no pulse and the error will be set to Zero

#define LINAC_COOLDOWN_OFF_TIME                5   // 1 seconds





// GLOBAL DATA TYPES
typedef struct {
  unsigned int sigma_data;
  unsigned int delta_data;
  signed char frequency_error_filtered;
  signed char frequency_error_history[16];
  signed char frequency_error_offset;
  unsigned char valid_data_history_count;
  unsigned char data_pointer; 
  unsigned char trigger_complete;
  unsigned char fast_afc_done;
  unsigned int pulses_on;
  unsigned int time_off_100ms_units;
  signed int distance_from_home_at_stop;
} TYPE_AFC_DATA;


extern TYPE_AFC_DATA afc_data;

void DoSystemCooldown(void);
// DPARKER - Description

void DoAFC(void);
// DPARKER - Description

void FilterFrequencyErrorData(void);
// DPARKER - Description

#endif
