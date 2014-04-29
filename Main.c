#include <p30f2023.h>
#include <libpic30.h>
#include "Main.h"
#include "Stepper.h"
#include "Serial_A34405.h"

_FOSCSEL(FRC_PLL);                                      /* Internal FRC oscillator with PLL */
_FICD(ICS_PGD);                                         /* Enable Primary ICP pins */
_FPOR(PWRT_128);
_FBS(BWRP_OFF & NO_BOOT_CODE);
_FGS(CODE_PROT_OFF);                                    /* Disable Code Protection */
_FWDT(FWDTEN_ON & WDTPOST_PS2048 & WDTPRE_PR32);        /* Enable Watch Dog */ // DPARKER CALCULATE WDT TIMEOUT
_FOSC(CSW_ON_FSCM_OFF & FRC_HI_RANGE & OSC2_CLKO);      /* Set up for internal fast RC 14.55MHz clock multiplied by X32 PLL  FOSC = 14.55e6*32/8 = 58.2MHz FCY = FOSC/2 = 29.1MHz*/


/* 
   Resource Usage
  
   INT0 - Trigger Input
   INT2 - Motor Overcurrent

   TMR2(Dan) - Used to time the motor position

*/






unsigned int manual_control_ccw_pulse_input_ready;
unsigned int manual_control_cw_pulse_input_ready;
      




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



unsigned int software_control_mode_selected;


int main (void) {
  while(OSCCONbits.LOCK!=1);          /* Wait for PLL to lock */
  //__delay32(EEPROM_DELAY*10);         /* Wait for EEPROMs to settle */
  __delay32(30000000);
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
    control_state = STATE_MOTOR_ZERO;
    break;

  case STATE_MOTOR_ZERO:
    afc_motor.current_position = 300;
    afc_motor.target_position = 100;
    while (control_state == STATE_MOTOR_ZERO) {
      DoSerialCommand();
      ClrWdt();
      if (FaultCheck()) {
	control_state = STATE_FAULT;
      } else if (afc_motor.current_position == 10) {
	control_state = STATE_MOTOR_STARTUP_HOME;
      }
    }
    break;

  case STATE_MOTOR_STARTUP_HOME:
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
    ClrWdt();
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
      ClrWdt();
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
      ClrWdt();
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
      ClrWdt();
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
  _INT0IE = 0;		// Disable INT0 Interrupt
  _INT0EP = 1; 	        // Interrupt on falling edge
  _INT0IP = 7;		// Set interrupt to highest priority
  // triggerINT = 0;  // DPARKER, I have no clue WTF this was
	
  /* Set up external INT2 */
  // This is the overcurrent interrupt
  _INT2IF = 0;		// Clear Interrupt flag
  _INT2IE = 1;		// Enable INT2 Interrupt
  _INT2EP = 1; 	        // Interrupt on falling edge
  _INT2IP = 6;		// Set interrupt to second highest priority
	
  // DPARKER what are these for
  /*
  _T2IP = 4;
  _T3IP = 2;
  */

  // Configure T2 Interrupt.  This is used to time motor steps
  _T2IF = 0;
  _T2IE = 1;
  _T2IP = 4;

  
  // PWM Special event trigger
  _PSEMIF = 0;
  _PSEMIE = 1;
  _PSEMIP = 5;


  // Configure UART Interrupts
  _U1RXIE = 0; // Disable RX Interrupt
  _U1RXIP = 3; // Priority Level 3
  
  _U1TXIE = 0; // Disable TX Interrupt
  _U1RXIP = 3; // Priority Level 3



  // Configure T2
  /* With 29 MHz Clock and 256 pre-scale the minimum pulses per second is 2*/


#define T2_PERIOD_VALUE           (unsigned int)(FCY/256/STEPS_PER_SECOND)
#define T2_CONFIG_VALUE           0b1000000000110000   // Timer On and 256 Prescale

  
  T2CON = T2_CONFIG_VALUE;
  PR2 = T2_PERIOD_VALUE;
  
  /* 
     --- UART 1 setup ---
     See uart.h and Microchip documentation for more information about the condfiguration
     
  */
#define UART1_BAUDRATE             120000        // U1 Baud Rate
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
}    




void __attribute__((interrupt, shadow, no_auto_psv)) _INT0Interrupt(void) {
  _INT0IF = 0;  
}




// DPARKER THIS FUNCTION FOR DEBUGGING ONLY
void  __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  Nop();
  Nop();
  __asm__ ("Reset");
}




