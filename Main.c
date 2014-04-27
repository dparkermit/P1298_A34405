#include <p30f2023.h>
#include <libpic30.h>
#include "Main.h"
#include "tables.h"
#include "Serial_A34405.h"

_FOSCSEL(FRC_PLL);                          /* Internal FRC oscillator with PLL */
//_FOSC(CSW_FSCM_OFF & FRC_HI_RANGE);      /* Set up for internal fast RC 14.55MHz clock multiplied by X32 PLL  FOSC = 14.55e6*32/8 = 58.2MHz FCY = FOSC/2 = 29.1MHz*/
//_FGS(CODE_PROT_OFF & GWRP_OFF);                        /* Disable Code Protection */
_FICD(ICS_PGD);                             /* Enable Primary ICP pins */
//_FWDT(WDTPOST_PS2048 & WDTPRE_PR32 & FWDTEN_ON & WINDIS_OFF);                           /* Enable Watch Dog */
_FPOR(PWRT_128);
_FBS(BWRP_OFF & NO_BOOT_CODE);

_FGS(CODE_PROT_OFF);                        /* Disable Code Protection */
_FWDT(FWDTEN_ON);                           /* Enable Watch Dog */
_FOSC(CSW_ON_FSCM_OFF & FRC_HI_RANGE & OSC2_CLKO);      /* Set up for internal fast RC 14.55MHz clock multiplied by X32 PLL  FOSC = 14.55e6*32/8 = 58.2MHz FCY = FOSC/2 = 29.1MHz*/


/* 
   Resource Usage
  
   INT0 - Trigger Input
   INT2 - Motor Overcurrent

   TMR3(Dan) - Use to time the motor position


*/



#define MOTOR_DECREASE_CURRENT_PWM_CYCLES    4000 // .1 second


#define FULL_STEP                 32

#define MOVING_COUNTER_CLOCKWISE  0
#define MOVING_CLOCKWISE          1
#define MOTOR_STOPPED             2



#define MOTOR_FREQUENCY_DIVISOR   1



unsigned int motor_motion;
unsigned int frequency_divisor_counter;
unsigned int counterTablePWM;
unsigned int counterPWM;
unsigned long motor_stopped_counter; 

unsigned int software_target_position;


unsigned int manual_control_ccw_pulse_input_ready;
unsigned int manual_control_cw_pulse_input_ready;
      



#define EEPROM_DELAY 1000

STEPPER_MOTOR afc_motor;
unsigned int control_state;
void DoStateMachine(void);

void InitPeripherals(void);


unsigned int FaultCheck(void);
unsigned int FaultCheck(void) {
  return 0;
}

void ResetAllFaults(void);
void ResetAllFaults(void) {
}


unsigned int home_position;
unsigned int software_control_mode_selected;


int main (void) {
  while(OSCCONbits.LOCK!=1);          /* Wait for PLL to lock */
  __delay32(EEPROM_DELAY*10);         /* Wait for EEPROMs to settle */
  __delay32(30000000);
  control_state = STATE_STARTUP;
  
  while(1) {
    DoStateMachine();
  }
}


void DoStateMachine(void) {

  switch (control_state) {
    

  case STATE_STARTUP:
    InitPeripherals();
    control_state = STATE_MOTOR_ZERO;
    break;

  case STATE_MOTOR_ZERO:
    afc_motor.current_position = afc_motor.max_position;
    afc_motor.target_position = afc_motor.min_position;
    while (control_state == STATE_MOTOR_ZERO) {
      DoSerialCommand();
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (afc_motor.current_position == 10) {
	control_state = STATE_MOTOR_STARTUP_HOME;
      }
    }
    break;

  case STATE_MOTOR_STARTUP_HOME:
    afc_motor.current_position = 0;
    afc_motor.target_position = home_position;
    while (control_state == STATE_MOTOR_STARTUP_HOME) {
      DoSerialCommand();
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (afc_motor.current_position == afc_motor.target_position) {
	control_state = STATE_RESET;
      }
    }
    break;
    
  case STATE_RESET:
    ResetAllFaults();
    DoSerialCommand();
    if (FaultCheck()) {
      control_state = STATE_FAULT;
    } else if (PIN_MODE_SELECT == ILL_AFC_MODE) {
      control_state = STATE_AFC_NOT_PULSING;
    } else {
      if (software_control_mode_selected) {
	control_state = STATE_SOFTWARE_CONTROL;
      } else {
	control_state = STATE_MANUAL_MODE;
      }
    }
    break;

    
  case STATE_MANUAL_MODE:
    if (PIN_STEP_COUNTER_CLOCKWISE != ILL_STEP_PIN_ACTIVE) {
      manual_control_ccw_pulse_input_ready = 1;
    }
    if (PIN_STEP_CLOCKWISE != ILL_STEP_PIN_ACTIVE) {
      manual_control_cw_pulse_input_ready = 1;
    }
    
    while (control_state == STATE_MANUAL_MODE) {
      DoSerialCommand();

      if (manual_control_ccw_pulse_input_ready && (PIN_STEP_COUNTER_CLOCKWISE == ILL_STEP_PIN_ACTIVE)) {
	manual_control_ccw_pulse_input_ready = 0;
	if (afc_motor.target_position <= (afc_motor.min_position + 1)) {
	  afc_motor.target_position = afc_motor.min_position;
	} else {
	  afc_motor.target_position = afc_motor.target_position - 1;
	}
	__delay32(30000); // Delay 30K Clock cycles so that bounces on the step signal will not cause multiple motor steps
      }

      if (manual_control_cw_pulse_input_ready && (PIN_STEP_CLOCKWISE == ILL_STEP_PIN_ACTIVE)) {
	manual_control_cw_pulse_input_ready = 0;
	if (afc_motor.target_position >= (afc_motor.max_position - 1)) {
	  afc_motor.target_position = afc_motor.max_position;
	} else {
	  afc_motor.target_position = afc_motor.target_position + 1;
	}
	__delay32(30000); // Delay 30K Clock Cycles so that bounces on the step signal will not cause multiple motor steps
      }
    
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (PIN_MODE_SELECT == ILL_AFC_MODE) {
	control_state = STATE_RESET;
      } 
      
    }
    break;

  case STATE_SOFTWARE_CONTROL:
    while (control_state == STATE_SOFTWARE_CONTROL) {
      DoSerialCommand();
      //SetMotorTarget(&afc_motor, software_target_position);
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (PIN_MODE_SELECT == ILL_AFC_MODE) {
	control_state = STATE_RESET;
      } 
    }
    break;

  case STATE_AFC_START_UP:
    while (control_state == STATE_AFC_START_UP) {
      DoSerialCommand();
      //SetMotorTarget(&afc_motor, home_position);
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (PIN_MODE_SELECT != ILL_AFC_MODE) {
	control_state = STATE_RESET;
      } 
    }
    break;

  case STATE_AFC_STEADY_STATE:
    control_state = STATE_RESET;
    break;
    
  case STATE_AFC_NOT_PULSING:
    control_state = STATE_RESET;
    break;


  case STATE_FAULT:
    while (control_state == STATE_FAULT) {
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

  counterTablePWM = 0;


  // Set up Interrupts
  /* Set up external INT0 */
  // This is the trigger interrupt
  IEC0bits.INT0IE = 0;		/* Disable INT0 Interrupt */
  IFS0bits.INT0IF = 0;		/* Clear Interrupt flag */
  INTCON2bits.INT0EP = 1; 	/* Interrupt on falling edge */ 
  IPC0bits.INT0IP = 7;		/* Set interrupt to highest priority */
  // triggerINT = 0;  // DPARKER, I have no clue WTF this was
	
  /* Set up external INT2 */
  // This is the overcurrent interrupt
  IEC1bits.INT2IE = 1;		/* Enable INT2 Interrupt */
  IFS1bits.INT2IF = 0;		/* Clear Interrupt flag */
  INTCON2bits.INT2EP = 1; 	/* Interrupt on falling edge */ 
  IPC4bits.INT2IP = 6;		/* Set interrupt to second highest priority */
	
  // DPARKER what are these for
  /*
  IEC1bits.PSEMIE = 1;
  IFS1bits.PSEMIF = 0;
  IPC4bits.PSEMIP = 5;
  IPC1bits.T2IP = 4;
  IPC1bits.T3IP = 2;
  */

  // Configure UART Interrupts
  _U1RXIE = 0; // Disable RX Interrupt
  _U1RXIP = 3; // Priority Level 3
  
  _U1TXIE = 0; // Disable TX Interrupt
  _U1RXIP = 3; // Priority Level 3




  /* 
     --- UART 1 setup ---
     See uart.h and Microchip documentation for more information about the condfiguration
     
  */
#define UART1_BAUDRATE             9600        // U1 Baud Rate
#define A35997_U1MODE_VALUE        (UART_DIS & UART_IDLE_STOP & UART_RX_TX & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT)
  //#define A35997_U1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A35997_U1STA_VALUE         (UART_INT_TX & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A35997_U1BRG_VALUE         (((FCY/UART1_BAUDRATE)/16)-1)
  

  
  // Initialize the UART
  
  // ----------------- UART #1 Setup and Data Buffer -------------------------//
  // Setup the UART input and output buffers
  BufferByte64Initialize(&uart1_input_buffer);
  BufferByte64Initialize(&uart1_output_buffer);
  
  //U1MODE = A35997_U1MODE_VALUE;
  // U1STA = A35997_U1STA_VALUE;  
  //U1BRG = A35997_U1BRG_VALUE;  
  U1MODE = 0b0000000000000000;
  U1STA =  0b0100010000000000;
  U1BRG = 14;

  




  // Begin UART operation
  command_string.data_state = COMMAND_BUFFER_EMPTY;  // The command buffer is empty

  _U1TXIF = 0;	// Clear the Transmit Interrupt Flag
  _U1RXIF = 0;	// Clear the Recieve Interrupt Flag
  _U1TXIE = 1;	// Enable Transmit Interrupts
  _U1RXIE = 1;	// Enable Recieve Interrupts  
  U1MODEbits.UARTEN = 1;	// And turn the peripheral on
}    


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


#define PWM_PWMCON_VALUE        0b0000000000000001

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
   Immediate update to PDC  --> DPARKER - I need jason to explain to me why this is set and not synced to PWM TIME BASE
*/

#define PWM_IOCON_OVERRIDE_OFF_VALUE  0b0000001100000001
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
   Immediate update to PDC  --> DPARKER - I need jason to explain to me why this is set and not synced to PWM TIME BASE
*/



 
void initPWM(void) {
  
  PTPER = PTPER_VALUE;                      /* PTPER = FCY*32(PLL)/(Desired PWM Freq.) Refer to PWM section for more details   */

  /* Initialize PWM Generator 1 */
  IOCON1   = PWM_IOCON_VALUE;
  PWMCON1  = PWM_PWMCON_VALUE;

  DTR1     = DEADTIME;
  ALTDTR1  = DEADTIME;
  PDC1     = HALF_DC;                     
  PHASE1   = 0;                     /* No staggering */

  /* Initialize PWM Generator 2 */
  IOCON2   = PWM_IOCON_VALUE;
  PWMCON2  = PWM_PWMCON_VALUE;

  DTR2     = DEADTIME;
  ALTDTR2  = DEADTIME;
  PDC2     = HALF_DC;                     
  PHASE2   = 0;                     /* No staggering */


  /* Initialize PWM Generator 3 */
  IOCON3   = PWM_IOCON_VALUE;
  PWMCON3  = PWM_PWMCON_VALUE;

  DTR3     = DEADTIME;
  ALTDTR3  = DEADTIME;
  PDC3     = HALF_DC;                     
  PHASE3   = 0;        /* No staggering */


  /* Initialize PWM Generator 4 */
  IOCON3   = PWM_IOCON_VALUE;
  PWMCON3  = PWM_PWMCON_VALUE;

  DTR4     = DEADTIME;
  ALTDTR4  = DEADTIME;
  PDC4     = HALF_DC;                     
  PHASE4   = 0;        /* No staggering */	


  SEVTCMP	                = 10;
  PTCONbits.SEIEN               = 1;
  PTCONbits.PTEN                = 1;        /* Enable PWM Module */
  
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
void __attribute__((interrupt, no_auto_psv)) _PWMSpEventMatchInterrupt(void) {
  IFS1bits.PSEMIF = 0;
  
  if (motor_motion == MOTOR_STOPPED) {
    if (afc_motor.target_position > afc_motor.max_position) {
      afc_motor.target_position = afc_motor.max_position;
    }
    if (afc_motor.target_position < afc_motor.min_position) {
      afc_motor.target_position = afc_motor.min_position;
    }
    if (afc_motor.current_position > afc_motor.target_position) {
      motor_motion = MOVING_COUNTER_CLOCKWISE;
      frequency_divisor_counter = 0;
    } else if (afc_motor.current_position < afc_motor.target_position) {
      motor_motion = MOVING_CLOCKWISE;
      frequency_divisor_counter = 0;
    } else if (motor_stopped_counter >= MOTOR_DECREASE_CURRENT_PWM_CYCLES) {
      motor_stopped_counter = 0;
      // DPARKER set PWM duty cycle so that current draw will be reduced.
      // Obviously at a loss of holding torque
    }
  } else {
    motor_stopped_counter = 0;
    frequency_divisor_counter++;
    if (frequency_divisor_counter == MOTOR_FREQUENCY_DIVISOR) {
      frequency_divisor_counter = 0;
      if (motor_motion == MOVING_CLOCKWISE) {
	counterTablePWM--;
      } else {
	counterTablePWM++;
      }
    }
    counterTablePWM %= (TABLE_SIZE);
    PDC1 = winding1ATable[counterTablePWM];
    PDC2 = winding1BTable[counterTablePWM];
    PDC3 = winding1ATable[(counterTablePWM + 32)];
    PDC4 = winding1BTable[(counterTablePWM + 32)];

    counterPWM++;
    if (counterPWM >= FULL_STEP) {
      counterPWM = 0;
      if (motor_motion == MOVING_CLOCKWISE) {
	if (afc_motor.current_position <= 0xFFFF) {
	  afc_motor.current_position++;
	}
      } else {
	if (afc_motor.current_position >= 1) {
	  afc_motor.current_position--;
	}
      }
      motor_motion = MOTOR_STOPPED;
    }
  }
}


void __attribute__((interrupt, shadow, no_auto_psv)) _INT0Interrupt(void) {
  _INT0IF = 0;  
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
  
  IOCON1 = PWM_IOCON_OVERRIDE_OFF_VALUE;
  IOCON2 = PWM_IOCON_OVERRIDE_OFF_VALUE;
  IOCON3 = PWM_IOCON_OVERRIDE_OFF_VALUE;
  IOCON4 = PWM_IOCON_OVERRIDE_OFF_VALUE;

  // DPARKER need to save that there was a fault somehow so that we can move to fault state
}


// DPARKER THIS FUNCTION FOR DEBUGGING ONLY
void  __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  Nop();
  Nop();
  __asm__ ("Reset");
}
