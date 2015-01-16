#include <p30f2023.h>
#include <libpic30.h>
#include <smpsadc.h>
#include "MCP4725.h"
#include "M24LC64F.h"
#include "Main.h"
#include "Stepper.h"
#include "afc.h"
#include "Serial_A34405.h"




// GLOBAL Vairables
unsigned char control_state;
unsigned int pulse_frequency      = 0;                // This is the prf
unsigned char auto_zero_requested = 0;                // An auto zero has been request (over PLC or serial interface).  This does not mean that an auto zero will happen though.  It must be in a state where auto zero is valid.
MCP4725 U24_MCP4725;
M24LC64F U23_M24LC64F;


// Local Variables and Function Prototypes
unsigned char manual_control_ccw_pulse_input_ready  = 0;   // This stores the state of the digital input so that we can detect transitions 
unsigned char manual_control_cw_pulse_input_ready   = 0;   // This stores the state of the digital input so that we can detect transitions 
unsigned char pin_sample_analog_input_ready         = 0;   // This stores the state of the digital input so that we can detect transitions 
unsigned int  prf_counter                           = 0;   // We count the number of pulses over 4 seconds and then divide by 4 to get PRF - This counts the number of pulses
unsigned char four_second_counter                   = 0;   // We count the number of pulses over 4 seconds and then divide by 4 to get PRF - This counts the time


void DoStateMachine(void);
void InitPeripherals(void);



unsigned char ConvertParameterInput(void);
void DoAnalogInputSample(void);
void UpdateAnalogOutput();
void ResetI2C(void);

unsigned char FaultCheck(void);
unsigned char FaultCheck(void) {
  return 0;
}


// -------------- SET THE PIC CONFIGURATION BITS ----------------//
_FOSCSEL(FRC_PLL);                                      /* Internal FRC oscillator with PLL */
_FICD(ICS_PGD);                                         /* Enable Primary ICP pins */
_FPOR(PWRT_128);
_FBS(BWRP_OFF & NO_BOOT_CODE);
_FGS(CODE_PROT_OFF);                                    /* Disable Code Protection */
_FWDT(FWDTEN_ON & WDTPOST_PS2048 & WDTPRE_PR32);        /* Enable Watch Dog */ // DPARKER CALCULATE WDT TIMEOUT
_FOSC(CSW_ON_FSCM_OFF & FRC_HI_RANGE & OSC2_CLKO);      /* Set up for internal fast RC 14.55MHz clock multiplied by X32 PLL  FOSC = 14.55e6*32/8 = 58.2MHz FCY = FOSC/2 = 29.1MHz*/



int main (void) {
  while(OSCCONbits.LOCK!=1);          /* Wait for PLL to lock */
  __delay32(30000000);
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
    auto_zero_requested = 1;
    while (control_state == STATE_WAIT_FOR_AUTO_ZERO) {
      DoSerialCommand();
      ClrWdt();
      // Look for a pulse on the sample pin
      if (PIN_SAMPLE_ANALOG_INPUT != ILL_SAMPLE_NOW) {
	pin_sample_analog_input_ready = 1;
      }
      if (pin_sample_analog_input_ready && (PIN_SAMPLE_ANALOG_INPUT == ILL_SAMPLE_NOW)) {
	pin_sample_analog_input_ready = 0;
	DoAnalogInputSample();
      }

      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (auto_zero_requested) {
	control_state = STATE_MOTOR_ZERO;
      }
    }
    break;


  case STATE_MOTOR_ZERO:
    afc_motor.max_position = 1000*MICRO_STEPPING_RESOLUTION;
    afc_motor.min_position = 0;
    afc_motor.current_position = afc_motor.max_position;
    afc_motor.target_position = afc_motor.min_position;
    auto_zero_requested = 0;
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
    afc_motor.min_position = MOTOR_MINIMUM_POSITION * MICRO_STEPPING_RESOLUTION;
    afc_motor.max_position = MOTOR_MAXIMUM_POSITION * MICRO_STEPPING_RESOLUTION;
    ClrWdt();
    // ResetAllFaults();  DPARKER no faults so no need to reset them
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
    auto_zero_requested = 0;
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

      // Look for a pulse on the sample pin
      if (PIN_SAMPLE_ANALOG_INPUT != ILL_SAMPLE_NOW) {
	pin_sample_analog_input_ready = 1;
      }
      if (pin_sample_analog_input_ready && (PIN_SAMPLE_ANALOG_INPUT == ILL_SAMPLE_NOW)) {
	pin_sample_analog_input_ready = 0;
	DoAnalogInputSample();
      }
      
      // Change state if needed
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (auto_zero_requested) {
	control_state = STATE_MOTOR_ZERO;
      } else if (PIN_MODE_SELECT == ILL_AFC_MODE) {
	control_state = STATE_RESET;
      }  
    }
    break;

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
	

  // Configure T2 Interrupt.  This is used to time motor steps
  _T2IF = 0;
  _T2IE = 1;
  _T2IP = 5;

  // Configure T1 Interrupt.  This is used for general purpose timing
  _T1IF = 0;
  _T1IE = 1;
  _T1IP = 3;

  
  // PWM Special event trigger
  _PSEMIF = 0;
  _PSEMIE = 1;
  _PSEMIP = 6;


  // Configure UART Interrupts
  _U1RXIE = 0; // Disable RX Interrupt
  _U1RXIP = 4; // Priority Level 3
  
  _U1TXIE = 0; // Disable TX Interrupt
  _U1RXIP = 4; // Priority Level 3


  // Configure Timers
  PR1 = T1_PERIOD_VALUE;  
  T1CON = T1_CONFIG_VALUE;
  
  PR2 = AFC_MOTOR_T2_PERIOD_VALUE;  
  T2CON = T2_CONFIG_VALUE;
  
  
  /* 
     --- UART Setup ---
     See uart.h and Microchip documentation for more information about the condfiguration
     // DPARKER cleanup this uart configuration
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
   
  // Begin UART operation
  command_string.data_state = COMMAND_BUFFER_EMPTY;  // The command buffer is empty

  _U1TXIF = 0;	// Clear the Transmit Interrupt Flag
  _U1RXIF = 0;	// Clear the Recieve Interrupt Flag
  _U1TXIE = 1;	// Enable Transmit Interrupts
  _U1RXIE = 1;	// Enable Recieve Interrupts  
  U1MODEbits.UARTEN = 1;	// And turn the peripheral on
  U1STA = 0b0000010000000000;   // The U1STA register must be set AFTER the module is enabled for some reason


  // ----------------- CONFIGURE ADC ---------------- //
  ADSTAT = 0;               // Clear the status register
  ADCON = ADCON_DEFAULT;
  ADPCFG = ADPCFG_DEFAULT;
  ADCPC0 = ADCPC0_DEFAULT;
  ADCPC1 = ADCPC1_DEFAULT;
  ADCPC2 = ADCPC2_DEFAULT;
  _ADON = 1;


  // Configure external ADC
  U24_MCP4725.address = MCP4725_ADDRESS_A0_0;
  U24_MCP4725.i2c_port = 0;
  U24_MCP4725.data_12_bit = 0x0000;
  U24_MCP4725.write_error_count = 0;

  SetupMCP4725(&U24_MCP4725);
  
  // Configure external EEPROM
  U23_M24LC64F.address = M24LC64F_ADDRESS_0;
  U23_M24LC64F.i2c_port = 0;
  


  afc_data.frequency_error_offset = M24LC64FReadWord(&U23_M24LC64F, EEPROM_REGISTER_ERROR_OFFSET);

  

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
  // DPARKER DELAY SAMPLE TIME - At least 300ns, but longer would be better
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
  PIN_SAMPLE_TRIGGER = !OLL_TRIGGER_SH;

  // DPARKER DELAY OUTPUT TIME - 2us According to spec sheet
  // DPARKER DELAY Until the output pulse has died down a bit so system is less noise
  // 20us Total
  __delay32(600);
    
  if (!_RA9){
  // Have the Pic sample the output of the S&H 16 times and average those results so that we have hopefully cleaner data
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


  error = afc_data.sigma_data - afc_data.delta_data;
  //error = afc_data.delta_data - afc_data.sigma_data;
  error += afc_data.frequency_error_offset;
  if (error > 127) {
    error = 127;
  } else if ( error < -128) {
    error = -128;
  }
  if ((afc_data.sigma_data <= SIGMA_DELTA_MINIMUM_ADC_READING) || (afc_data.delta_data <= SIGMA_DELTA_MINIMUM_ADC_READING)) {
    // There wasn't acutaully a pulse, record an error signal of zero.
    // We don't want to ignore the pulse because we want to log what happened
    error = 0;
  }
  
  /* 
     afc_data.valid_data_history_count is also zero in the PWM interrupt if the motor is moving
     It is nessessary to check both places to work properly across all range of rep rates
  */
  if (afc_motor.target_position != afc_motor.current_position) {
    afc_data.valid_data_history_count = 0;
  }

  afc_data.valid_data_history_count++;
  if (afc_data.valid_data_history_count >= 0x0F) {
    afc_data.valid_data_history_count = 0x0F;
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
  }
  _INT0IF = 0;
}




unsigned char ConvertParameterInput(void) {
  /*
    Based on the value of the spare analog input (adc_parameter_input) we select a different parameter
    
    adc_parameter_input = 0 (0->.5)    ,  0,      0     -> 6553   = Select Motor Position        0V = Position 0, 5V = Position 1024, 4 LSB Per Position
    adc_parameter_input = 1V (.5->1.5) , 13170,  6554  -> 19660  = Select Home Poisiton         0V = Position 0, 5V = Position 1024, 4 LSB Per Position
    adc_parameter_input = 2V (1.5->2.5), 26214,  19661 -> 32767  = Select AFC Offset            0V = -128, 2.5V = 0, 5V = 127, 16 LSB Per Position        
    adc_parameter_input = 3V (2.5->3.5), 39321,  32768 -> 45874  = Select Pulse Rate Reading    0V = 0Hz, 5V = 512Hz, 8 LSB per Hz
    adc_parameter_input = 4V (3.5->4.5), 53428,  45785 -> 58981  = Unused                       Return Zero
    adc_parameter_input = 5V (4.5->5)  , 65535,  58982 -> 65535  = Select Auto Zero             0V = Not in (Do Not)  Auto Zero, 5V = In (Do)Auto Zero
    
    adc_analog_value_input is a 16 bit number
  */
  
  if (adc_parameter_input <= 6553) {
    return PARAMETER_MOTOR_POSITION;
  } else if (adc_parameter_input <= 19660) {
    return PARAMETER_HOME_POSITION;
  } else if (adc_parameter_input <= 32767) {
    return PARAMETER_AFC_OFFSET;
  } else if (adc_parameter_input <= 45874) {
    return PARAMETER_PRF;
  } else if (adc_parameter_input <= 58981) {
    return PARAMETER_UNUSED;
  } else {
    return PARAMETER_AUTO_ZERO;
  }
}



void DoAnalogInputSample(void) {
  unsigned char parameter;
  unsigned int error;
  
  parameter = ConvertParameterInput();

  switch (parameter) {
    
  case PARAMETER_MOTOR_POSITION:
    // Move the motor to that position
    afc_motor.target_position = (adc_analog_value_input >> 6) * MICRO_STEPPING_RESOLUTION;
    break;
    
  case PARAMETER_HOME_POSITION:
    // Set the home position and store to EEPRORM
    afc_motor.home_position = (adc_analog_value_input >> 6) * MICRO_STEPPING_RESOLUTION;
    if (afc_motor.min_position > afc_motor.home_position)
        afc_motor.home_position = DEFAULT_HOME_POSITION * MICRO_STEPPING_RESOLUTION;
    M24LC64FWriteWord(&U23_M24LC64F, EEPROM_REGISTER_HOME_POSITION, afc_motor.home_position);
    break;

  case PARAMETER_AFC_OFFSET:
    // Set the Frequency Error offset and store to EEPROM
    //error = adc_analog_value_input >> 8;
    //if (error >= 128) {
    //  afc_data.frequency_error_offset = (error-128);
    //} else {
    //  error = 128 - error;
    //  afc_data.frequency_error_offset = 0;
    //  afc_data.frequency_error_offset -= error;
    //}
    //M24LC64FWriteWord(&U23_M24LC64F, EEPROM_REGISTER_ERROR_OFFSET, afc_data.frequency_error_offset);
    break;

  case PARAMETER_PRF:
    // Do nothing
    break;

  case PARAMETER_UNUSED:
    // Do nothing
    break;
    
  case PARAMETER_AUTO_ZERO:
    // Auto zero if appropriate
    if (adc_analog_value_input >= 0x8000) {
      auto_zero_requested = 1;
    }
    break;
  }
}


void UpdateAnalogOutput(void) {
  unsigned char parameter;

  parameter = ConvertParameterInput();
  
  switch (parameter) {
    
  case PARAMETER_MOTOR_POSITION:
    U24_MCP4725.data_12_bit = (afc_motor.current_position << 2) / MICRO_STEPPING_RESOLUTION;
    break;

  case PARAMETER_HOME_POSITION:
    U24_MCP4725.data_12_bit = (afc_motor.home_position << 2) / MICRO_STEPPING_RESOLUTION;
    break;

  case PARAMETER_AFC_OFFSET:
    if (afc_data.frequency_error_offset >= 0) {
      U24_MCP4725.data_12_bit = afc_data.frequency_error_offset;
      U24_MCP4725.data_12_bit = afc_data.frequency_error_offset + 128;
      U24_MCP4725.data_12_bit <<= 4;
    } else {
      U24_MCP4725.data_12_bit = (afc_data.frequency_error_offset + 128);
      U24_MCP4725.data_12_bit <<= 4;
    }
    break;

  case PARAMETER_PRF:
    U24_MCP4725.data_12_bit = (pulse_frequency << 2);
    break;

  case PARAMETER_UNUSED:
    U24_MCP4725.data_12_bit = 0x0000;
    break;
    
  case PARAMETER_AUTO_ZERO:
    if ((control_state == STATE_MOTOR_ZERO) || (control_state == STATE_MOTOR_STARTUP_HOME)) {
      U24_MCP4725.data_12_bit = 0x0FFF;
    } else {
      U24_MCP4725.data_12_bit = 0;
    }
    break;
  }
  
  MCP4725UpdateFast(&U24_MCP4725);
} 


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



// This is set up for .1 Second Timer
void  __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
  _T1IF = 0;

  // Schedule an Update the feedback to the control board.
  //do_analog_update = 1;  // don't write to the I2C bus durring an interrupt.  That can perform indetermintly if something else is writting to the I2C bus at the time the interrupt is called
  // DPARKER do not do I2c stuff in interrupt
  UpdateAnalogOutput();

  // Check for Bus collision on the I2C bus
  if (_BCL) {
    ResetI2C();
    _BCL = 0;
  }

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




