#include "ETM_I2C.h"


#if defined(__dsPIC30F__)
#include <p30fxxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__PIC24H__)
#include <p24Hxxxx.h>
#endif


unsigned int etm_i2c1_error_count = 0;

#if defined(_I2C1MD)  
unsigned int etm_i2c2_error_count = 0;
#endif

unsigned int etm_i2c_loop_timeout;

void ConfigureI2C(unsigned char i2c_port, unsigned int configuration, unsigned long baud_rate, unsigned long fcy_clk, unsigned long pulse_gobbler_delay_fcy) {

  unsigned long baud_rate_register;
  
  baud_rate_register = (fcy_clk*8)/baud_rate;


  etm_i2c_loop_timeout = baud_rate_register;
  /*
    The loop timeout is not ~equal to the number of clock cycles it takes to clock out a byte.
    Each loop takes at least 10 clock cycles to loop.
    So dived the number above by 2 should give plenty of margin for error
  */
  etm_i2c_loop_timeout >>= 1;

  baud_rate_register -= 8*pulse_gobbler_delay_fcy;
  baud_rate_register >>= 3;
  baud_rate_register -= 1;

#if defined(__dsPIC33F__)
  baud_rate_register -= 1;
#endif 
  
  if (baud_rate_register < 2) {
    baud_rate_register = 2;
  }
  if (baud_rate_register > 0xFFFF) {
    baud_rate_register = 2;
  }

#if defined(_I2CMD)
  if ((i2c_port == 0) || (i2c_port == 1)) {
    I2CCON = configuration;
    I2CBRG = baud_rate_register;
  } 
#endif
  
#if defined(_I2C1MD)  
  if (i2c_port = 1) {
    I2C1CON = configuration;
    I2C1BRG = baud_rate_register;
  }
#endif    
  
#if defined(_I2C2MD) 
  if (i2c_port = 2) {
    I2C2CON = configuration;
    I2C2BRG = baud_rate_register;
  }
#endif
}

unsigned int WaitForI2CBusIdle(unsigned char i2c_port) {
  
  unsigned int loop_counter;
  unsigned int i2c_result;

  i2c_result = 0;

#if defined(_I2CMD)
  if ((i2c_port == 0) || (i2c_port == 1)) {
    //Wait for bus Idle or Timeout
    loop_counter = 0;
    while (I2CSTATbits.TRSTAT) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;	
      }
    }
  }
#endif

#if defined(_I2C1MD)
  if (i2c_port == 1) {
    //Wait for bus Idle or Timeout
    loop_counter = 0;
    while (I2C1STATbits.TRSTAT) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;	
      }
    }
  }
#endif

#if defined(_I2C2MD)
  if (i2c_port == 2) {
    //Wait for bus Idle or Timeout
    loop_counter = 0;
    while (I2C2STATbits.TRSTAT) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c2_error_count++;
	break;
      }
    }
  }
#endif
  
  return i2c_result;
}
  




unsigned int GenerateI2CStart(unsigned char i2c_port) {

  unsigned int loop_counter;
  unsigned int i2c_result;

  i2c_result = 0;
  
#if defined(_I2CMD)
  if ((i2c_port == 0) || (i2c_port == 1)) {
    I2CCONbits.SEN = 1;		                         //Generate Start COndition
    //Wait for Start COndition or Timeout
    loop_counter = 0;
    while (I2CCONbits.SEN) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
  }
#endif
  
#if defined(_I2C1MD)
  if (i2c_port == 1) {
    I2C1CONbits.SEN = 1;	                         //Generate Start COndition
    //Wait for Start COndition or Timeout
    loop_counter = 0;
    while (I2C1CONbits.SEN) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
  }
#endif
  
#if defined(_I2C2MD)
  if (i2c_port == 2) {
    I2C2CONbits.SEN = 1;	                         //Generate Start COndition
    //Wait for Start COndition or Timeout
    loop_counter = 0;
    while (I2C2CONbits.SEN) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c2_error_count++;
	break;
      }
    }
  }
#endif
  
  return i2c_result;
}

  

unsigned int GenerateI2CRestart(unsigned char i2c_port) {
  unsigned int loop_counter;
  unsigned int i2c_result;
  
  i2c_result = 0;
  
#if defined(_I2CMD)
  if ((i2c_port == 0) || (i2c_port == 1)) {
    I2CCONbits.RSEN = 1;	                         //Generate Re-Start COndition
    //Wait for Re-Start COndition
    loop_counter = 0;
    while (I2CCONbits.RSEN) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
  }
#endif

#if defined(_I2C1MD)
  if (i2c_port == 1) {
    I2C1CONbits.RSEN = 1;		                 //Generate Re-Start COndition
    //Wait for Re-Start COndition
    loop_counter = 0;
    while (I2C1CONbits.RSEN) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
  }
#endif

#if defined(_I2C2MD)
  if (i2c_port == 2) {
    I2C2CONbits.RSEN = 1;		                 //Generate Re-Start COndition
    //Wait for Re-Start COndition
    loop_counter = 0;
    while (I2C2CONbits.RSEN) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c2_error_count++;
	break;
      }
    }
  }
#endif
    
  return i2c_result;
}



unsigned int WriteByteI2C(unsigned char data, unsigned char i2c_port) {
  unsigned int loop_counter;
  unsigned int i2c_result;
  
  i2c_result = 0;
  
#if defined(_I2CMD)
  if ((i2c_port == 0) || (i2c_port == 1)) {
    I2CTRN = (data);                                            //Load data to the transmit buffer
    // Wait for the transmit process to start
    loop_counter = 0;
    while (!I2CSTATbits.TRSTAT) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
    // Wait for the transmit process to complete (cleared at end of Slave ACK)
    while (I2CSTATbits.TRSTAT) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
  }
#endif

#if defined(_I2C1MD)
  if (i2c_port == 1) {
    I2C1TRN = (data);                                           //Load data to the transmit buffer
    // Wait for the transmit process to start
    loop_counter = 0;
    while (!I2C1STATbits.TRSTAT) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
    // Wait for the transmit process to complete (cleared at end of Slave ACK)
    while (I2C1STATbits.TRSTAT) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
  }
#endif

#if defined(_I2C2MD)
  if (i2c_port == 2) {
    I2C2TRN = (data);                                           //Load data to the transmit buffer
    // Wait for the transmit process to start
    loop_counter = 0;
    while (!I2C2STATbits.TRSTAT) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c2_error_count++;
	break;
      }
    }
    // Wait for the transmit process to complete (cleared at end of Slave ACK)
    while (I2C2STATbits.TRSTAT) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c2_error_count++;
	break;
      }
    }
  }
#endif
    
  return i2c_result;
}

unsigned int ReadByteI2C(unsigned char i2c_port) {
  unsigned int loop_counter;
  unsigned int i2c_result;
  
  i2c_result = 0;

#if defined(_I2CMD)
  if ((i2c_port == 0) || (i2c_port == 1)) {
    I2CCONbits.RCEN = 1;			                 //Start Master receive
    //Wait for data transfer
    loop_counter = 0;
    while (I2CCONbits.RCEN) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
    //Wait for receive bufer to be full
    while (!I2CSTATbits.RBF) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
    if (i2c_result == 0) {
      i2c_result = (I2CRCV & 0x00FF);
    }
  }
#endif

#if defined(_I2C1MD)
  if (i2c_port == 1) {
    I2C1CONbits.RCEN = 1;			                 //Start Master receive
    //Wait for data transfer
    loop_counter = 0;
    while (I2C1CONbits.RCEN) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
    //Wait for receive bufer to be full
    while (!I2C1STATbits.RBF) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
    if (i2c_result == 0) {
      i2c_result = (I2C1RCV & 0x00FF);
    }
  }
#endif

#if defined(_I2C2MD)
  if (i2c_port == 2) {
    I2C2CONbits.RCEN = 1;			                 //Start Master receive
    //Wait for data transfer
    loop_counter = 0;
    while (I2C2CONbits.RCEN) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c2_error_count++;
	break;
      }
    }
    //Wait for receive bufer to be full
    while (!I2C2STATbits.RBF) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c2_error_count++;
	break;
      }
    }
    if (i2c_result == 0) {
      i2c_result = (I2C2RCV & 0x00FF);
    }
  }
#endif
    
  return i2c_result;
}



unsigned int GenerateI2CStop(unsigned char i2c_port) {
  unsigned int loop_counter;
  unsigned int i2c_result;
  
  i2c_result = 0;

#if defined(_I2CMD)
  if ((i2c_port == 0) || (i2c_port == 1)) {
    I2CCONbits.PEN = 1;	                                 //Generate Stop COndition
    //Wait for Stop COndition or Timeout
    loop_counter = 0;
    while (I2CCONbits.PEN) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
  }
#endif

#if defined(_I2C1MD)
  if (i2c_port == 1) {
    I2C1CONbits.PEN = 1;		                 //Generate stop COndition
    //Wait for Stop COndition or Timeout
    loop_counter = 0;
    while (I2C1CONbits.PEN) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c1_error_count++;
	break;
      }
    }
  }
#endif

#if defined(_I2C2MD)
  if (i2c_port == 2) {
    I2C2CONbits.PEN = 1;		                 //Generate stop COndition
    //Wait for Stop COndition or Timeout
    loop_counter = 0;
    while (I2C1CONbits.PEN) {
      loop_counter++;
      if (loop_counter > etm_i2c_loop_timeout) {
	i2c_result = 0xFA00;
	etm_i2c2_error_count++;
	break;
      }
    }
  }
#endif

  return i2c_result;
}


