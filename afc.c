#include "afc.h"
#include "Stepper.h"
#include "tables.h"
#include "Serial_A34405.h"



#define MAX_NUMBER_OF_PULSES_FOR_STARTUP_RESPONSE  256
#define MAX_NUMBER_OF_PULSES_FOR_SECOND_RESPONSE 800

#define FREQUENCY_ERROR_FAST_MOVE_4_STEPS       (MOTOR_MOVE_FULL_STEP_THRESHOLD * 9)
#define FREQUENCY_ERROR_FAST_MOVE_3_STEPS       (MOTOR_MOVE_FULL_STEP_THRESHOLD * 6)
#define FREQUENCY_ERROR_FAST_MOVE_2_STEPS       (MOTOR_MOVE_FULL_STEP_THRESHOLD * 3)
#define FREQUENCY_ERROR_FAST_MOVE_1_STEPS       (MOTOR_MOVE_FULL_STEP_THRESHOLD * 1)

#define FREQUENCY_ERROR_SLOW_THRESHOLD          3
#define FREQUENCY_ERROR_SLOW_THRESHOLD_POSITIVE (MOTOR_MOVE_FULL_STEP_THRESHOLD * 10)              //So the AFC does not oscillate during slow mode





const unsigned int CooldownTable[2][24] = {{COOLDOWN_TABLE_ROW_1_VALUES}, {COOLDOWN_TABLE_ROW_2_VALUES}};


TYPE_AFC_DATA afc_data;


signed char FrequencyErrorFilterSlowResponse();
signed char FrequencyErrorFilterFastResponse();

void ClearAFCErrorHistory(void);



void FilterFrequencyErrorData(void) {
  if (!afc_data.fast_afc_done) {
    afc_data.frequency_error_filtered = FrequencyErrorFilterFastResponse();
  } else {
    afc_data.frequency_error_filtered = FrequencyErrorFilterSlowResponse();
  }
}




void DoAFC(void) {
  unsigned int new_target_position;
  // Sigma/delta data is 10 bit unsigned so we should have no problem with the conversion to signed and overflow even with the frequency error offset.
  new_target_position = afc_motor.current_position;

  if (afc_data.pulses_on >= MAX_NUMBER_OF_PULSES_FOR_STARTUP_RESPONSE) {
    afc_data.fast_afc_done = 1;
  }

  if (!afc_data.fast_afc_done) {
    /*
      The magnetron has just turned on after being off for a period of time.
      The tuner *could* be wildly out of position.
      We need to react to the incoming data from AFC very quickly
      This means less filtering and and high gain integral response - Max 4 Steps per sample
    */
    if (afc_data.frequency_error_filtered > FREQUENCY_ERROR_FAST_MOVE_4_STEPS) {
      new_target_position += 4 * MICRO_STEPPING_RESOLUTION;
    } else if (afc_data.frequency_error_filtered > FREQUENCY_ERROR_FAST_MOVE_3_STEPS) {
      new_target_position += 3 * MICRO_STEPPING_RESOLUTION;
    } else if (afc_data.frequency_error_filtered > FREQUENCY_ERROR_FAST_MOVE_2_STEPS) {
      new_target_position += 2 * MICRO_STEPPING_RESOLUTION;
    } else if (afc_data.frequency_error_filtered > FREQUENCY_ERROR_FAST_MOVE_1_STEPS) {
      new_target_position += 1 * MICRO_STEPPING_RESOLUTION;
    } else if (afc_data.frequency_error_filtered > -FREQUENCY_ERROR_FAST_MOVE_1_STEPS) {
      if (afc_data.pulses_on >= 4) {
	afc_data.fast_afc_done = 1;
	// DPARKER - I think we need to clear the previous data here
	// We don't want to be in the case where the data from fast samples is still in the history and we use that for slow evaluation
      }
    } else if (afc_data.frequency_error_filtered > -FREQUENCY_ERROR_FAST_MOVE_2_STEPS) {
      if (new_target_position >= 1) {
	new_target_position -= 1 * MICRO_STEPPING_RESOLUTION;
      } else {
	new_target_position = 0;
      }
    } else if (afc_data.frequency_error_filtered > -FREQUENCY_ERROR_FAST_MOVE_3_STEPS) {
      if (new_target_position >= 2) {
	new_target_position -= 2 * MICRO_STEPPING_RESOLUTION;
      } else {
	new_target_position = 0;
      }
    } else if (afc_data.frequency_error_filtered > -FREQUENCY_ERROR_FAST_MOVE_4_STEPS) {
      if (new_target_position >= 3) {
	new_target_position -= 3 * MICRO_STEPPING_RESOLUTION;
      } else {
	new_target_position = 0;
      }
    } else {
      if (new_target_position >= 4) {
	new_target_position -= 4 * MICRO_STEPPING_RESOLUTION;
      } else {
	new_target_position = 0;
      }
    }
    SetMotorTarget(POSITION_TYPE_ABSOLUTE_POSITION, new_target_position);
  } else {
    //afc_data.fast_afc_done = 1;
    /*
      The magnetron has pulsing for Number Of Startup Pulses (or the error has reached zero) 
      The tuner is very close to where it should be so we just need to correct for minor drifts
      We can filter/average the incoming data over a longer timeframe
    */

    /* 
       When pulsing at 400Hz, it can take a lot of pulses to move the motor.
       We don't want to be calculating a new target with data that is taken while the motor is moving
       Therefore we only evaluate error position data while the motor is not moving
       
       If we are pulsing at lower frequencies, we still need to use the data that we get to update the motor position
       frequency_error_filtered contains an average of the previous 8 data points (note this will take 8 samples to ramp up after the motor has stopped moving)
       If that average value is above the error threshold, then the target positon of the motor is moved.
    */

    /*
      Assuming, PWM_TO_MICROSTEP_RATIO_SLOW_MODE = 36, and STEPS_PER_SECOND_SLOW = 25
      At 50Hz pulse rate, what is the fastest the motor will move?
      Well it takes 20.8ms To move the motor 1 step
      It takes 160ms to generate fill the table with 8 data points
      Therefore if the error is above the threshold the entire time, the logest it can take to request a turn is 180mS.
      If the error is right at the error threshold it will take 5 motor cycle interrupts to move one step.
      .5 steps per second.
      Obviously if the error is more severe it will take less time.

      At 400Hz pulse rate, what is the fastest the motor will move?
      Well it takes 20.8ms To move the motor 1 step
      It takes 20ms to generate fill the table with 8 data points
      Therefore if the error is above the threshold the entire time, the logest it can take to request a turn is 40.8mS.
      If the error is right at the error threshold it will take 2 motor cycle interrupts to move one step. 
      12.5 steps per second


      Is this fast enough?  We will need to look at test data and see if our error increases faster than the AFC can track
    */
    
    if (afc_motor.current_position == afc_motor.target_position) {
      /* 
	 If the target position != to the current_position then we have already instructed to motor to move.  
	 We should wait for that process to complete before telling it to move again
      */
      new_target_position = afc_motor.current_position;
      if (afc_data.frequency_error_filtered > FREQUENCY_ERROR_SLOW_THRESHOLD_POSITIVE) {
	if (new_target_position <= 0xFFFE) {
	  new_target_position++;
	  SetMotorTarget(POSITION_TYPE_ABSOLUTE_POSITION, new_target_position);
	}	
      } else if (afc_data.frequency_error_filtered < -FREQUENCY_ERROR_SLOW_THRESHOLD) {
	if (new_target_position >= 1) {
	  new_target_position--;
	  SetMotorTarget(POSITION_TYPE_ABSOLUTE_POSITION, new_target_position);
	}
      }
    }
  }
  SendLoggingDataToUart();
}




void DoSystemCooldown(void) {
  unsigned char index;
  unsigned int off_time;
  unsigned int remainder;
  unsigned long temp;
  

  off_time = afc_data.time_off_100ms_units;
  
  if (off_time >= CooldownTable[0][23]) {
    temp = 0;
  } else {
    index = 0;
    while ((index < 22) && (off_time >= CooldownTable[0][index])) {
      index++;
    }
    // calculate the fractional multiplier
    temp = CooldownTable[1][index] - CooldownTable[1][index+1];
    temp *= (CooldownTable[0][index+1]- off_time);
    temp /= (CooldownTable[0][index+1]- CooldownTable[0][index]);
    temp += CooldownTable[1][index+1];
    
    // temp is now our fractional multiplier
    // DPARKER - the rounding will not work properly if Finish Position > Target Position.  I am ok because this will never happen in real life.
    temp *= afc_data.distance_from_home_at_stop;
    remainder = temp & 0x0000FFFF;
    temp >>= 16;
    if (remainder >= 0x8000) {
      temp++;
    }
  }
  SetMotorTarget(POSITION_TYPE_ABSOLUTE_POSITION, (afc_motor.home_position - temp));

}




signed char FrequencyErrorFilterSlowResponse() {
  // average the previous 8 data points
  // if the motor is moving, error data is set to zero
  // DPARKER we only need 8 data points so there is no need to store 16 points (unless we think we might need 16 in the future)

  signed int average;
  unsigned char location;
  unsigned int pulsesToFilter;
  unsigned int x;

  if (afc_data.pulses_on < MAX_NUMBER_OF_PULSES_FOR_SECOND_RESPONSE) {
      pulsesToFilter = 4;
  }
  else {
      pulsesToFilter = 8;
  }

  if ((afc_data.pulses_on < pulsesToFilter) || (afc_data.valid_data_history_count < pulsesToFilter)) {
    // There are not at least 4 valid data points for the current motor position.  Return Zero
    return 0;
  } else {
    location = afc_data.data_pointer;
    average = 0;
    // Remember that data_pointer points to the NEXT place to put data, so most recent data is n-1
    // If number of pulses is greater than or equal to 8, Average the prevous 8 data points
    for (x = 0; x < pulsesToFilter; x++){
        location--;
        location &= 0x0F;
        average += afc_data.frequency_error_history[location];
    }
    
    if (pulsesToFilter == 8){
        average >>= 3;
    } else if (pulsesToFilter == 4) {
        average >>= 2;
    }

    return average;
  }
}



signed char FrequencyErrorFilterFastResponse() {
  signed int average;
  unsigned char location;

  if (afc_data.pulses_on < 2) {
    return 0;
  } else {
    
    location = afc_data.data_pointer;
    // Remember that data_pointer points to the NEXT place to put data, so most recent data is n-1
    // If number of pulses is less than 2 do nothing
    // If number of pulses is greater than or equal to 2, Average the prevous 2 data points
    location--;
    location &= 0x0F;
    average = afc_data.frequency_error_history[location];
    location--;
    location &= 0x0F;
    average += afc_data.frequency_error_history[location];
    average >>= 1; 
    
    return average;
  }
}

















// DPARKER clear error history is not needed, delete
void ClearAFCErrorHistory(void) {
  afc_data.frequency_error_history[0] = 0;
  afc_data.frequency_error_history[1] = 0;
  afc_data.frequency_error_history[2] = 0;
  afc_data.frequency_error_history[3] = 0;
  afc_data.frequency_error_history[4] = 0;
  afc_data.frequency_error_history[5] = 0;
  afc_data.frequency_error_history[6] = 0;
  afc_data.frequency_error_history[7] = 0;
  afc_data.frequency_error_history[8] = 0;
  afc_data.frequency_error_history[9] = 0;
  afc_data.frequency_error_history[10] = 0;
  afc_data.frequency_error_history[11] = 0;
  afc_data.frequency_error_history[12] = 0;
  afc_data.frequency_error_history[13] = 0;
  afc_data.frequency_error_history[14] = 0;
  afc_data.frequency_error_history[15] = 0;
}


