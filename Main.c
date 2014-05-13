#include <p30f2023.h>
#include <smpsadc.h>
#include "Main.h"
#include "Stepper.h"
#include "Serial_A34405.h"
#include "MCP4725.h"
#include "tables.h"

 
#define NUMBER_OF_PULSES_FOR_STARTUP_RESPONSE  128

#define FREQUENCY_ERROR_FAST_MOVE_4_STEPS      108
#define FREQUENCY_ERROR_FAST_MOVE_3_STEPS      78
#define FREQUENCY_ERROR_FAST_MOVE_2_STEPS      48
#define FREQUENCY_ERROR_FAST_MOVE_1_STEPS      14

#define FREQUENCY_ERROR_SLOW_THRESHOLD         20



#define LINAC_COOLDOWN_OFF_TIME                10  // 1 seconds



#define COOL_DOWN_TABLE_MAX_INDEX              511 // .8 second units to 408 seconds 

const unsigned int CoolDownTable[512] = {COOL_DOWN_TABLE_VALUES};


_FOSCSEL(FRC_PLL);                                      /* Internal FRC oscillator with PLL */
_FICD(ICS_PGD);                                         /* Enable Primary ICP pins */
_FPOR(PWRT_128);
_FBS(BWRP_OFF & NO_BOOT_CODE);
_FGS(CODE_PROT_OFF);                                    /* Disable Code Protection */
_FWDT(FWDTEN_ON & WDTPOST_PS2048 & WDTPRE_PR32);        /* Enable Watch Dog */ // DPARKER CALCULATE WDT TIMEOUT
_FOSC(CSW_ON_FSCM_OFF & FRC_HI_RANGE & OSC2_CLKO);      /* Set up for internal fast RC 14.55MHz clock multiplied by X32 PLL  FOSC = 14.55e6*32/8 = 58.2MHz FCY = FOSC/2 = 29.1MHz*/


_prog_addressT FLASH_address_afc_config;
unsigned int __attribute__((space(psv),aligned(_FLASH_ROW))) afc_config_in_FLASH[] = {0,1,2,3, 4,5,6,7, 8,9,10,11, 12,13,14,15};  // Create 16 word structure in FLASH and load default configuration values
unsigned int afc_config_ram_copy[16];




/* 
   Resource Usage
  
   INT0 - Trigger Input
   INT2 - Motor Overcurrent

   PWM_SPECIAL_EVENT_TRIGGER - Used to generate current waveforms

   TMR1 - Used for general timing functions - Set to .1 S
   TMR2 - Used to time the motor movement.  

   UART1 - Used for Digital Serial Link
   I2C - Connected to ADC and (possibly EEPROM)
   

*/


// GLOBAL Vairables

TYPE_AFC_DATA afc_data;
unsigned char control_state;
unsigned int prf_counter;
unsigned int pulse_frequency;
unsigned char software_auto_zero;


// Local Variables and Function Prototypes
unsigned char manual_control_ccw_pulse_input_ready;
unsigned char manual_control_cw_pulse_input_ready;
unsigned char four_second_counter = 0;

#define LINAC_COOLDOWN_FACTIONAL_MULTIPLIER  55000

void DoSystemCooldown(void);


void DoAFC(void);

void DoStateMachine(void);

void InitPeripherals(void);

void ResetI2C(void);

void ClearAFCErrorHistory(void);

void ResetI2C(void) {
  unsigned char n;
  _I2CEN = 0; // turn off I2C module
  _TRISG2 = 0; // Set I2C Clock Pin to output
  n=0;
  while (n++ <= 20) {
    _LATG2 = 1 ;
    __delay32(100);
    _LATG2 = 0 ; 
    __delay32(100);
  }
  _TRISG2 = 1; // Set I2C Clock Pin to input
  _I2CEN = 1; // turn on I2C module
}


unsigned int FaultCheck(void);
unsigned int FaultCheck(void) {
  return 0;
}

void ResetAllFaults(void);
void ResetAllFaults(void) {
}

signed char FrequencyErrorFilterSlowResponse();
signed char FrequencyErrorFilterFastResponse();


void FilterFrequencyErrorData(void);


int main (void) {
  while(OSCCONbits.LOCK!=1);          /* Wait for PLL to lock */
  //__delay32(EEPROM_DELAY*10);         /* Wait for EEPROMs to settle */
  __delay32(30000000);
  
  
  _init_prog_address(FLASH_address_afc_config, afc_config_in_FLASH);
  _memcpy_p2d16(afc_config_ram_copy, FLASH_address_afc_config, _FLASH_ROW);
  ClrWdt();
  

  control_state = STATE_STARTUP;
  
  while(1) {
    DoStateMachine();
  }
}


void DoStateMachine(void) {

  switch (control_state) {
    

  case STATE_STARTUP:
    ClrWdt();
    InitPeripherals();
    InitPWM();
    control_state = STATE_WAIT_FOR_AUTO_ZERO;
    break;


  case STATE_WAIT_FOR_AUTO_ZERO:
    software_auto_zero = 0;
    while (control_state == STATE_WAIT_FOR_AUTO_ZERO) {
      DoSerialCommand();
      ClrWdt();
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (software_auto_zero) {
	control_state = STATE_MOTOR_ZERO;
      }
    }
    break;


  case STATE_MOTOR_ZERO:
    afc_motor.max_position = 1000;
    afc_motor.min_position = 0;
    afc_motor.current_position = afc_motor.max_position;
    afc_motor.target_position = afc_motor.min_position;
    software_auto_zero = 0;
    while (control_state == STATE_MOTOR_ZERO) {
      DoSerialCommand();
      ClrWdt();
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (afc_motor.current_position == afc_motor.target_position) {
	control_state = STATE_MOTOR_STARTUP_HOME;
      }
    }
    break;

  case STATE_MOTOR_STARTUP_HOME:
    afc_motor.current_position = 0;
    afc_motor.target_position = afc_motor.home_position;
    while (control_state == STATE_MOTOR_STARTUP_HOME) {
      ClrWdt();  
      DoSerialCommand();
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (afc_motor.current_position == afc_motor.target_position) {
	control_state = STATE_RESET;
      }
    }
    break;
    
  case STATE_RESET:
    afc_motor.min_position = MOTOR_MINIMUM_POSITION;
    afc_motor.max_position = MOTOR_MAXIMUM_POSITION;
    ClrWdt();
    ResetAllFaults();
    DoSerialCommand();
    if (FaultCheck()) {
      control_state = STATE_FAULT;
    } else if (PIN_MODE_SELECT == ILL_AFC_MODE) {
      control_state = STATE_AFC_PULSING;
    } else {
      control_state = STATE_MANUAL_MODE;
    }
    break;
    
    
  case STATE_MANUAL_MODE:
    software_auto_zero = 0;
    while (control_state == STATE_MANUAL_MODE) {
      ClrWdt();
      DoSerialCommand();
      
      if (afc_data.trigger_complete) {
	afc_data.trigger_complete = 0;
	FilterFrequencyErrorData();
      }

      // Look for a pulse on the counter clockwise pin
      if (PIN_STEP_COUNTER_CLOCKWISE != ILL_STEP_PIN_ACTIVE) {
	manual_control_ccw_pulse_input_ready = 1;
      }
      if (manual_control_ccw_pulse_input_ready && (PIN_STEP_COUNTER_CLOCKWISE == ILL_STEP_PIN_ACTIVE)) {
	manual_control_ccw_pulse_input_ready = 0;
	SetMotorTarget(POSITION_TYPE_RELATIVE_COUNTER_CLOCKWISE, 1);
      }

      // Look for a pulse on the clockwise pin
      if (PIN_STEP_CLOCKWISE != ILL_STEP_PIN_ACTIVE) {
	manual_control_cw_pulse_input_ready = 1;
      }
      if (manual_control_cw_pulse_input_ready && (PIN_STEP_CLOCKWISE == ILL_STEP_PIN_ACTIVE)) {
	manual_control_cw_pulse_input_ready = 0;
	SetMotorTarget(POSITION_TYPE_RELATIVE_CLOCKWISE, 1);
      }

      // Change state if needed
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (software_auto_zero) {
	control_state = STATE_MOTOR_ZERO;
      } else if (PIN_MODE_SELECT == ILL_AFC_MODE) {
	control_state = STATE_RESET;
      }  
    }
    break;

    /*
  case STATE_SOFTWARE_CONTROL:
    while (control_state == STATE_SOFTWARE_CONTROL) {
      DoSerialCommand();
      ClrWdt();
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (PIN_MODE_SELECT == ILL_AFC_MODE) {
	control_state = STATE_RESET;
      } 
    }
    break;
    */

  case STATE_AFC_PULSING:
    afc_data.pulses_on = 0;
    afc_data.fast_afc_done = 0;
    while (control_state == STATE_AFC_PULSING) {
      ClrWdt();
      DoSerialCommand();
      if (afc_data.trigger_complete) {
	afc_data.trigger_complete = 0;
	FilterFrequencyErrorData();
	DoAFC();
      }
      
      // Look for change of state
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (PIN_MODE_SELECT != ILL_AFC_MODE) {
	control_state = STATE_RESET;
      } else if (afc_data.time_off_100ms_units >= LINAC_COOLDOWN_OFF_TIME) {
	control_state = STATE_AFC_NOT_PULSING;
      } 
    }
    break;

  case STATE_AFC_NOT_PULSING:
    afc_data.distance_from_home_at_stop = afc_motor.home_position - afc_motor.current_position;
    while (control_state == STATE_AFC_NOT_PULSING) {
      ClrWdt();
      DoSerialCommand();
      DoSystemCooldown();
      // Look for change of state
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (PIN_MODE_SELECT != ILL_AFC_MODE) {
	control_state = STATE_RESET;
      } else if (afc_data.trigger_complete) {
	control_state = STATE_AFC_PULSING;
      } 
    }
    break;


  case STATE_FAULT:
    while (control_state == STATE_FAULT) {
      ClrWdt();
      DoSerialCommand();
    }
    break;
    
    
  }
}

  
void InitPeripherals(void){
  PIN_SAMPLE_TRIGGER = !OLL_TRIGGER_SH;
  PIN_BRIDGE_POWER_RELAY = !OLL_BRIDGE_POWER_RELAY_CLOSED;
  
  TRISA = TRISA_VALUE;
  TRISB = TRISB_VALUE;
  TRISD = TRISD_VALUE;
  TRISE = TRISE_VALUE;
  TRISF = TRISF_VALUE;


  /* Init I2C */
  /*
  I2CCON = I2C_ON & I2C_IDLE_CON &  I2C_CLK_REL & I2C_IPMI_DIS & I2C_7BIT_ADD & I2C_SLW_DIS & I2C_SM_DIS; 
  I2CCON &= I2C_GCALL_DIS & I2C_STR_DIS & I2C_ACK;
  I2CBRG = I2C_BAUD_RATE_GENERATOR;
  */



  // Set up Interrupts
  // Set up external INT0 */
  // This is the trigger interrupt
  _INT0IF = 0;		// Clear Interrupt flag
  _INT0IE = 1;		// Disable INT0 Interrupt
  _INT0EP = 1; 	        // Interrupt on falling edge
  _INT0IP = 7;		// Set interrupt to highest priority
  // triggerINT = 0;  // DPARKER, I have no clue WTF this was
	
  /* Set up external INT2 */
  // This is the overcurrent interrupt
  _INT2IF = 0;		// Clear Interrupt flag
  _INT2IE = 1;		// Enable INT2 Interrupt
  _INT2EP = 1; 	        // Interrupt on falling edge
  _INT2IP = 6;		// Set interrupt to second highest priority
	

  // Configure T2 Interrupt.  This is used to time motor steps
  _T2IF = 0;
  _T2IE = 1;
  _T2IP = 3;

  // Configure T1 Interrupt.  This is used for general purpose timing
  _T1IF = 0;
  _T1IE = 1;
  _T1IP = 3;

  
  // PWM Special event trigger
  _PSEMIF = 0;
  _PSEMIE = 1;
  _PSEMIP = 5;


  // Configure UART Interrupts
  _U1RXIE = 0; // Disable RX Interrupt
  _U1RXIP = 4; // Priority Level 3
  
  _U1TXIE = 0; // Disable TX Interrupt
  _U1RXIP = 4; // Priority Level 3



  // Configure T2
  /* With 29 MHz Clock and 256 pre-scale the minimum pulses per second is 2*/



#define T2_CONFIG_VALUE           0b1000000000110000   // Timer On and 256 Prescale


#define T1_PERIOD_VALUE           (unsigned int)(FCY/256/10)
#define T1_CONFIG_VALUE           0b1000000000110000   // Timer On and 8 Prescale
  
  PR1 = T1_PERIOD_VALUE;  
  T1CON = T1_CONFIG_VALUE;
  
  
  PR2 = T2_PERIOD_VALUE;  
  T2CON = T2_CONFIG_VALUE;

  
  /* 
     --- UART 1 setup ---
     See uart.h and Microchip documentation for more information about the condfiguration
     
  */
  #define UART1_BAUDRATE             303000        // U1 Baud Rate
  //#define UART1_BAUDRATE             9600
#define A35997_U1MODE_VALUE        (UART_DIS & UART_IDLE_STOP & UART_RX_TX & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT)
  //#define A35997_U1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A35997_U1STA_VALUE         (UART_INT_TX & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A35997_U1BRG_VALUE         (unsigned int)(((FCY/UART1_BAUDRATE)/16)-1)
  

  
  // Initialize the UART
  
  // ----------------- UART #1 Setup and Data Buffer -------------------------//
  // Setup the UART input and output buffers
  BufferByte64Initialize(&uart1_input_buffer);
  BufferByte64Initialize(&uart1_output_buffer);
  


  //U1MODE = A35997_U1MODE_VALUE;
  // U1STA = A35997_U1STA_VALUE;  
  U1BRG = A35997_U1BRG_VALUE;
  U1MODE = 0b0000000000000000;
  U1STA = 0b0000010000000000; 
  //U1BRG = 188;




  // Begin UART operation
  command_string.data_state = COMMAND_BUFFER_EMPTY;  // The command buffer is empty

  _U1TXIF = 0;	// Clear the Transmit Interrupt Flag
  _U1RXIF = 0;	// Clear the Recieve Interrupt Flag
  _U1TXIE = 1;	// Enable Transmit Interrupts
  _U1RXIE = 1;	// Enable Recieve Interrupts  
  U1MODEbits.UARTEN = 1;	// And turn the peripheral on
  U1STA = 0b0000010000000000;   // The U1STA register must be set AFTER the module is enabled for some reason





  // ----------------- CONFIGURE ADC ---------------- //
  //#define ADCON_DEFAULT     0b1000000000100101 
#define ADCON_DEFAULT (ADC_MOD_DIS | ADC_IDLE_CONT | ADC_SOFT_TRIG_DIS | ADC_DATA_INT | ADC_INT_EN_2CONV | ADC_ORDER_EVEN_FST | ADC_SAMP_SEQ | ADC_PLL_EN_FADC_14)
  
#define ADPCFG_DEFAULT    0b1111010111110000  // AN0,AN1,AN2,AN3,AN9,AN11
    
#define ADCPC0_DEFAULT (ADC_AN1_0_IR_GEN_DIS | ADC_AN1_0_TRIG_INDV_SW |  ADC_AN3_2_IR_GEN_DIS   | ADC_AN3_2_TRIG_INDV_SW)
#define ADCPC1_DEFAULT (ADC_AN5_4_IR_GEN_DIS | ADC_AN5_4_NOCONV       |  ADC_AN7_6_IR_GEN_DIS   | ADC_AN7_6_NOCONV)
#define ADCPC2_DEFAULT (ADC_AN9_8_IR_GEN_DIS | ADC_AN9_8_TRIG_INDV_SW |  ADC_AN11_10_IR_GEN_DIS | ADC_AN11_10_TRIG_INDV_SW)

  ADSTAT = 0;               // Clear the status register
  ADCON = ADCON_DEFAULT;
  ADPCFG = ADPCFG_DEFAULT;
  ADCPC0 = ADCPC0_DEFAULT;
  ADCPC1 = ADCPC1_DEFAULT;
  ADCPC2 = ADCPC2_DEFAULT;
  _ADON = 1;

  U24_MCP4725.address = MCP4725_ADDRESS_A0_0;
  U24_MCP4725.i2c_port = 0;
  U24_MCP4725.data_12_bit = 0x0000;
  U24_MCP4725.write_error_count = 0;

  SetupMCP4725(&U24_MCP4725);
  
  ResetI2C();

  software_auto_zero = 0;
  prf_counter = 0;


  //afc_data.frequency_error_offset = afc_config_ram_copy[4]; // DPARKER add defined locations  
  afc_data.frequency_error_offset = 0;
  
}    


/*
  This function will run once every LINAC_COOLDOWN_OFF_TIME period
  When it runs, it compares the current position to the home position
  It will then move (1 - LINAC_COOLDOWN_FACTIONAL_MULTIPLIER) percent of the way home (minimum of one step)

  // DPARKER it may be nessesary to modify so that the momevement is smoother (and does not have the one step minimum movement) - Especially if we are always working with small step sizes.
*/
void DoSystemCooldown(void) {
  // DPARKER figure out how to make this work with negative number for "distance from home at stop" 
  unsigned int time_index;
  unsigned int multiplier;
  unsigned long result;
  unsigned int remainder;
  if (afc_data.time_off_100ms_units >= LINAC_COOLDOWN_OFF_TIME) {
    time_index = (afc_data.time_off_100ms_units >> 3);  // Time index will be in .8 second units
    if (time_index >= COOL_DOWN_TABLE_MAX_INDEX) {
      time_index = COOL_DOWN_TABLE_MAX_INDEX;
    } 
    multiplier = CoolDownTable[time_index];
    result = multiplier;
    result *= afc_data.distance_from_home_at_stop;
    remainder = result & 0x0000FFFF;
    result >>= 16;
    if (remainder >= 0x8000) {
      result++;
    }
    SetMotorTarget(POSITION_TYPE_ABSOLUTE_POSITION, (afc_motor.home_position - result));
  }
  


  /*
  signed long temperature_offset;
  if (afc_data.time_off_100ms_units >= LINAC_COOLDOWN_OFF_TIME) {
    afc_data.time_off_100ms_units = 0;
      if (afc_motor.current_position > afc_motor.home_position) {
      temperature_offset = afc_motor.current_position;
      temperature_offset -= afc_motor.home_position;
      temperature_offset *= LINAC_COOLDOWN_FACTIONAL_MULTIPLIER;
      temperature_offset >>= 16;
      afc_motor.target_position = afc_motor.home_position + temperature_offset;
    } else if (afc_motor.current_position < afc_motor.home_position) {
      temperature_offset = afc_motor.home_position;
      temperature_offset -= afc_motor.current_position;
      temperature_offset *= LINAC_COOLDOWN_FACTIONAL_MULTIPLIER;
      temperature_offset >>= 16;
      afc_motor.target_position = afc_motor.home_position - temperature_offset;
    }
  }
  */
}


// This function sets all of the data in the Error History array to zero
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


void DoAFC(void) {
  unsigned int new_target_position;
  // Sigma/delta data is 10 bit unsigned so we should have no problem with the conversion to signed and overflow even with the frequency error offset.
  new_target_position = afc_motor.current_position;


  if ((afc_data.pulses_on <= NUMBER_OF_PULSES_FOR_STARTUP_RESPONSE) && (!afc_data.fast_afc_done)) {
    /*
      The magnetron has just turned on after being off for a period of time.
      The tuner *could* be wildly out of position.
      We need to react to the incoming data from AFC very quickly
      This means less filtering and and high gain integral response - Max 4 Steps per sample
    */
    if (afc_data.frequency_error_filtered > FREQUENCY_ERROR_FAST_MOVE_4_STEPS) {
      new_target_position += 4;
    } else if (afc_data.frequency_error_filtered > FREQUENCY_ERROR_FAST_MOVE_3_STEPS) {
      new_target_position += 3;
    } else if (afc_data.frequency_error_filtered > FREQUENCY_ERROR_FAST_MOVE_2_STEPS) {
      new_target_position += 2;
    } else if (afc_data.frequency_error_filtered > FREQUENCY_ERROR_FAST_MOVE_1_STEPS) {
      new_target_position += 1;
    } else if (afc_data.frequency_error_filtered > -FREQUENCY_ERROR_FAST_MOVE_1_STEPS) {
      if (afc_data.pulses_on >= 4) {
	afc_data.fast_afc_done = 1;
	//ClearAFCErrorHistory(); // DPARKER no need to clear AFC data anymore
      }
    } else if (afc_data.frequency_error_filtered > -FREQUENCY_ERROR_FAST_MOVE_2_STEPS) {
      if (new_target_position >= 1) {
	new_target_position -= 1;
      } else {
	new_target_position = 0;
      }
    } else if (afc_data.frequency_error_filtered > -FREQUENCY_ERROR_FAST_MOVE_3_STEPS) {
      if (new_target_position >= 2) {
	new_target_position -= 2;
      } else {
	new_target_position = 0;
      }
    } else if (afc_data.frequency_error_filtered > -FREQUENCY_ERROR_FAST_MOVE_4_STEPS) {
      if (new_target_position >= 3) {
	new_target_position -= 3;
      } else {
	new_target_position = 0;
      }
    } else {
      if (new_target_position >= 4) {
	new_target_position -= 4;
      } else {
	new_target_position = 0;
      }
    }
    SetMotorTarget(POSITION_TYPE_ABSOLUTE_POSITION, new_target_position);
  } else {
    afc_data.fast_afc_done = 1;
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
      Assuming, PWM_TO_MICROSTEP_RATIO_SLOW_MODE = 36, and STEPS_PER_SECOND_SLOW = 26
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
      if (afc_data.frequency_error_filtered > FREQUENCY_ERROR_SLOW_THRESHOLD) {
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

  


void FilterFrequencyErrorData(void) {
  if ((afc_data.pulses_on <= NUMBER_OF_PULSES_FOR_STARTUP_RESPONSE) && (!afc_data.fast_afc_done)) {
    afc_data.frequency_error_filtered = FrequencyErrorFilterFastResponse();
  } else {
    afc_data.frequency_error_filtered = FrequencyErrorFilterSlowResponse();
  }
}



signed char FrequencyErrorFilterSlowResponse() {
  // average the previous 8 data points
  // if the motor is moving, error data is set to zero

  signed int average;
  unsigned char location;
  
  if (afc_data.pulses_on < 8) {
    return 0;
  } else {
    
    location = afc_data.data_pointer;
    // Remember that data_pointer points to the NEXT place to put data, so most recent data is n-1
    // If number of pulses is greater than or equal to 8, Average the prevous 8 data points
    location--;
    location &= 0x0F;
    average = afc_data.frequency_error_history[location];
    location--;
    location &= 0x0F;
    average += afc_data.frequency_error_history[location];
    location--;
    location &= 0x0F;
    average += afc_data.frequency_error_history[location];
    location--;
    location &= 0x0F;
    average += afc_data.frequency_error_history[location];
    location--;
    location &= 0x0F;
    average += afc_data.frequency_error_history[location];
    location--;
    location &= 0x0F;
    average += afc_data.frequency_error_history[location];
    location--;
    location &= 0x0F;
    average += afc_data.frequency_error_history[location];
    location--;
    location &= 0x0F;
    average += afc_data.frequency_error_history[location];


    average >>= 3; 
    
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




void __attribute__((interrupt, shadow, no_auto_psv)) _INT0Interrupt(void) {
  /* 
     This is the signal that pulse has been sent to the linac
     Need to trigger the sample and hold after correct delay and schedule the ADC to read those sampled values.
  */ 
  signed int error;  
  unsigned int n;
  
  
  // DPARKER DELAY UNTILL WE HAVE A GOOD SIGNAL ON SIGMA/DELTA
  //__delay32(12); // 3uS
  
  PIN_SAMPLE_TRIGGER = OLL_TRIGGER_SH;
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  // DPARKER DELAY SAMPLE TIME - At least 300ns, but longer would be better

  PIN_SAMPLE_TRIGGER = !OLL_TRIGGER_SH;
  // DPARKER DELAY OUTPUT TIME - 2us According to spec sheet
  // DPARKER DELAY Until the output pulse has died down a bit so system is less noise
  // 20us Total
  __delay32(600);
    
  n = 0;
  afc_data.sigma_data = 0;
  afc_data.delta_data = 0;

  _P0RDY = 0;
  _SWTRG0 = 1;             // Trigger Conversion on AN0/AN1
  while (n<14) {
    n++;
    while(!_P0RDY);           // Wait for the conversion on AN0/AN1 to complete
    afc_data.sigma_data += ADCBUF0;
    afc_data.delta_data += ADCBUF1;
    _P0RDY = 0;
    _SWTRG0 = 1;             // Trigger Conversion on AN0/AN1
  } 
  // while(_P0RDY);           // Wait for the conversion on AN0/AN1 to complete
  while(!_P0RDY);
  afc_data.sigma_data += ADCBUF0;
  afc_data.delta_data += ADCBUF1;
  afc_data.sigma_data >>= 4;
  afc_data.delta_data >>= 4;

  //error = afc_data.sigma_data - afc_data.delta_data;
  error = afc_data.delta_data - afc_data.sigma_data;
  error += afc_data.frequency_error_offset;
  if (error > 127) {
    error = 127;
  } else if ( error < -128) {
    error = -128;
  }
  if ((afc_data.fast_afc_done) && (afc_motor.motor_motion != MOTOR_MOTION_STOPPED)) {
    /* 
       When pulsing at 400Hz, it can take a lot of pulses to move the motor.
       We don't want to be calculating a new target with data that is taken while the motor is moving
       Therefore we only evaluate error position data while the motor is not moving
       
       The easiest way to this is to just set the error data to zero if the motor is moving and we are in "slow mode"
       Remember that in "fast mode" we do adjust the motor position while it is still moving, so we have to store data if fast mode is not done
    */
    error = 0;
  }
  afc_data.frequency_error_history[afc_data.data_pointer] = error;
  afc_data.data_pointer++;
  afc_data.data_pointer &= 0x0F;
  afc_data.pulses_on++;
  if (afc_data.pulses_on > 0xFF00) {
    afc_data.pulses_on = 0xFF00;
  }
  afc_data.time_off_100ms_units = 0;
  prf_counter++;
  afc_data.trigger_complete = 1;
  _INT0IF = 0;
}


// This is set up for .1 Second Timer
void  __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {

  _T1IF = 0;

  // Update the feedback to the control board.
  /*
    Based on the value of the spare analog input (adc_parameter_input) we return different data on the DAC
    
    adc_parameter_input = 0,  0,      0     -> 6553   = Return Motor Position        (voltage scaling = ???)
    adc_parameter_input = 1V, 13107,  6554  -> 13107  = Return Simga Reading         (voltage scaling = ???)
    adc_parameter_input = 2V, 26214,  13108 -> 32767  = Return Delta Reading
    adc_parameter_input = 3V, 39321,  32768 -> 45874  = Return Pulse Rate Reading    (voltage scaling = ???)
    adc_parameter_input = 4V, 53428,  45785 -> 58981  = N/A Returns 0x0000
    adc_parameter_input = 5V, 65535,  58982 -> 65535  = N/A Returns 0x0000
  */
  
  if (adc_parameter_input <= 6553) {
    U24_MCP4725.data_12_bit = (afc_motor.current_position << 2);
  } else if (adc_parameter_input <= 13107) {
    U24_MCP4725.data_12_bit = afc_data.sigma_data << 2;
  } else if (adc_parameter_input <= 32767) {
    U24_MCP4725.data_12_bit = afc_data.delta_data << 2;
  } else if (adc_parameter_input <= 45874) {
    U24_MCP4725.data_12_bit = (pulse_frequency << 2);
  } else if (adc_parameter_input <= 58981) {
    U24_MCP4725.data_12_bit = 0x0000;
  } else {
    U24_MCP4725.data_12_bit = 0x0000;
  }

  MCP4725UpdateFast(&U24_MCP4725);


  // Calculate PRF
  four_second_counter++;
  if (four_second_counter >= 40) {
    PIN_FAULT_OUT_1 = !PIN_FAULT_OUT_1;
    four_second_counter = 0;
    pulse_frequency = prf_counter >> 2;
    prf_counter = 0;
  }

  // Update the not pulsing counter
  afc_data.time_off_100ms_units++;
  if (afc_data.time_off_100ms_units >= 0xFF00) {
    afc_data.time_off_100ms_units = 0xFF00;
  }
}





// DPARKER THIS FUNCTION FOR DEBUGGING ONLY
void  __attribute__((interrupt, no_auto_psv)) _StackError(void) {
  Nop();
  Nop();
  __asm__ ("Reset");
}


// DPARKER THIS FUNCTION FOR DEBUGGING ONLY
void  __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  Nop();
  Nop();
  __asm__ ("Reset");
}




