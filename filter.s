


	;; W8 is pointer to data array
	;; W10 is pointer to filter array
	CLR	A, [W8]+=2, W4,	[W10]+=2, W6       ; Clear Accumulator A, Move Data[0] to W4, Move Filter[0] to W6, Increment Data and Filter Pointer

	MAC	W4*W6, A, [W8], W5, [W10]+=2, W6   ; Multiply W4 * W6  (Data_0*Filter_0) and store in accumlator A, Load Data[1] into W5, Load Filter[1] into W6, Increment Filter Pointer
	MOV	W4, [W8++]                         ; Move Data_0 to Data[1], Increment Data Pointer
	MAC     W5*W6, A, [W8], W4, [W10]+=2, W6   ; Multiply W5 * W6  (Data_1*Filter_1) and store in accumlator A, Load Data[2] into W4, Load Filter[2] into W6, Increment Filter Pointer
	MOV	W5, [W8++]			   ; Move Data_1 to Data[2], Increment Data Pointer
	
	MAC	W4*W6, A, [W8], W5, [W10]+=2, W6   
	MOV	W4, [W8++]                         
	MAC     W5*W6, A, [W8], W4, [W10]+=2, W6   
	MOV	W5, [W8++]
	
	MAC	W4*W6, A, [W8], W5, [W10]+=2, W6   
	MOV	W4, [W8++]                         
	MAC     W5*W6, A, [W8], W4, [W10]+=2, W6   
	MOV	W5, [W8++]
	
	MAC	W4*W6, A, [W8], W5, [W10]+=2, W6   
	MOV	W4, [W8++]                         
	MAC     W5*W6, A	                   ; All 8 Data points have been multiplied and added into accumlator.  Data Array has been shifted one position
	                                           ; New Data Should be loaded into Data[0]  

	
