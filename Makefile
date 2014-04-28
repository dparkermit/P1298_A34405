# MPLAB IDE generated this makefile for use with GNU make.
# Project: P1298_A34405.mcp
# Date: Mon Apr 28 14:56:44 2014

AS = pic30-as.exe
CC = pic30-gcc.exe
LD = pic30-ld.exe
AR = pic30-ar.exe
HX = pic30-bin2hex.exe
RM = rm

P1298_A34405.hex : P1298_A34405.cof
	$(HX) "P1298_A34405.cof"

P1298_A34405.cof : Main.o Serial_A34405.o ETM_BUFFER_BYTE_64.o
	$(CC) -mcpu=30F2023 "Main.o" "Serial_A34405.o" "ETM_BUFFER_BYTE_64.o" -o"P1298_A34405.cof" -Wl,-Tp30F2023.gld,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,--defsym=__ICD2RAM=1,-Map="P1298_A34405.map",--report-mem

Main.o : ../../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/support/dsPIC30F/h/p30F2023.h ../../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/support/dsPIC30F/h/p30fxxxx.h ../../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/support/peripheral_30F_24H_33F/uart.h ETM_BUFFER_BYTE_64.h Serial_A34405.h tables.h Main.h ../../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/support/generic/h/libpic30.h ../../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/support/dsPIC30F/h/p30f2023.h Main.c
	$(CC) -mcpu=30F2023 -x c -c "Main.c" -o"Main.o" -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -g -Wall

Serial_A34405.o : Version_A34405.h Main.h ../../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/support/dsPIC30F/h/p30F2023.h ../../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/support/dsPIC30F/h/p30fxxxx.h ../../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/support/peripheral_30F_24H_33F/uart.h ETM_BUFFER_BYTE_64.h serial_A34405.h ../../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/support/dsPIC30F/h/p30F2023.h ../../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/support/dsPIC30F/h/p30fxxxx.h Serial_A34405.c
	$(CC) -mcpu=30F2023 -x c -c "Serial_A34405.c" -o"Serial_A34405.o" -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -g -Wall

ETM_BUFFER_BYTE_64.o : ETM_BUFFER_BYTE_64.h ETM_BUFFER_BYTE_64.c
	$(CC) -mcpu=30F2023 -x c -c "ETM_BUFFER_BYTE_64.c" -o"ETM_BUFFER_BYTE_64.o" -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -g -Wall

clean : 
	$(RM) "Main.o" "Serial_A34405.o" "ETM_BUFFER_BYTE_64.o" "P1298_A34405.cof" "P1298_A34405.hex"

