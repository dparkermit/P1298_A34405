#ifndef __AFC_H
#define __AFC_H



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
