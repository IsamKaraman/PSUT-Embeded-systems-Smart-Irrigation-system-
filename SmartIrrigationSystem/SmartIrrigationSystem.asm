
_configurePWM:

;SmartIrrigationSystem.c,24 :: 		void configurePWM() {
;SmartIrrigationSystem.c,26 :: 		CCP1CON = 0b00001100; // Set CCP1 module for PWM
	MOVLW      12
	MOVWF      CCP1CON+0
;SmartIrrigationSystem.c,29 :: 		PR2 = 249;            // Set PWM period for 20ms (assuming Fosc/4)
	MOVLW      249
	MOVWF      PR2+0
;SmartIrrigationSystem.c,30 :: 		T2CON = 0b00000111;   // Enable TMR2 with prescaler 1:16
	MOVLW      7
	MOVWF      T2CON+0
;SmartIrrigationSystem.c,33 :: 		CCPR1L = 0x00;        // Set initial duty cycle to 0
	CLRF       CCPR1L+0
;SmartIrrigationSystem.c,34 :: 		CCP1CON |= 0b00000011; // Initialize CCP1CON for PWM
	MOVLW      3
	IORWF      CCP1CON+0, 1
;SmartIrrigationSystem.c,35 :: 		}
L_end_configurePWM:
	RETURN
; end of _configurePWM

_ADT_Init:

;SmartIrrigationSystem.c,36 :: 		void ADT_Init(void){
;SmartIrrigationSystem.c,37 :: 		ADCON0 = 0x49;// A/D converter ON, Don't GO, Channel 1, Fosc/16, equivalent to B'01001001'
	MOVLW      73
	MOVWF      ADCON0+0
;SmartIrrigationSystem.c,38 :: 		ADCON1 = 0xC0;// All channels Analog, 500 KHz, right justified, equivalent to B'11000000'
	MOVLW      192
	MOVWF      ADCON1+0
;SmartIrrigationSystem.c,39 :: 		TRISA = 0xFF; // PortA is my input
	MOVLW      255
	MOVWF      TRISA+0
;SmartIrrigationSystem.c,40 :: 		}
L_end_ADT_Init:
	RETURN
; end of _ADT_Init

_ADC_read_1:

;SmartIrrigationSystem.c,41 :: 		unsigned int ADC_read_1(void){
;SmartIrrigationSystem.c,42 :: 		ADCON0 = ADCON0 | 0x04;              // GO
	BSF        ADCON0+0, 2
;SmartIrrigationSystem.c,43 :: 		while(ADCON0 & 0x04);                // stay in GO until DONE
L_ADC_read_10:
	BTFSS      ADCON0+0, 2
	GOTO       L_ADC_read_11
	GOTO       L_ADC_read_10
L_ADC_read_11:
;SmartIrrigationSystem.c,44 :: 		return((ADRESH<<8) | ADRESL);        // read/return the result
	MOVF       ADRESH+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       ADRESL+0, 0
	IORWF      R0+0, 1
	MOVLW      0
	IORWF      R0+1, 1
;SmartIrrigationSystem.c,45 :: 		}
L_end_ADC_read_1:
	RETURN
; end of _ADC_read_1

_ADC_Read_0:

;SmartIrrigationSystem.c,46 :: 		unsigned int ADC_Read_0(void ) {
;SmartIrrigationSystem.c,47 :: 		ADCON0 = 0x41;  // Select the specified ADC channel
	MOVLW      65
	MOVWF      ADCON0+0
;SmartIrrigationSystem.c,48 :: 		ADCON0 = ADCON0 | 0x04;
	BSF        ADCON0+0, 2
;SmartIrrigationSystem.c,50 :: 		while (ADCON0 & 0x04);  // Wait for conversion to complete
L_ADC_Read_02:
	BTFSS      ADCON0+0, 2
	GOTO       L_ADC_Read_03
	GOTO       L_ADC_Read_02
L_ADC_Read_03:
;SmartIrrigationSystem.c,52 :: 		return ((ADRESH << 8) | ADRESL);  // Combine high and low byte results and return
	MOVF       ADRESH+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       ADRESL+0, 0
	IORWF      R0+0, 1
	MOVLW      0
	IORWF      R0+1, 1
;SmartIrrigationSystem.c,53 :: 		}
L_end_ADC_Read_0:
	RETURN
; end of _ADC_Read_0

_setPWM:

;SmartIrrigationSystem.c,54 :: 		void setPWM(unsigned int dutyCycle) {
;SmartIrrigationSystem.c,56 :: 		CCPR1L = dutyCycle >> 2;            // Set the upper 8 bits of duty cycle
	MOVF       FARG_setPWM_dutyCycle+0, 0
	MOVWF      R0+0
	MOVF       FARG_setPWM_dutyCycle+1, 0
	MOVWF      R0+1
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	MOVF       R0+0, 0
	MOVWF      CCPR1L+0
;SmartIrrigationSystem.c,57 :: 		CCP1CON &= 0b11001111;              // Clear the lower 2 bits of CCP1CON
	MOVLW      207
	ANDWF      CCP1CON+0, 1
;SmartIrrigationSystem.c,58 :: 		CCP1CON |= ((dutyCycle & 0x03) << 4); // Set the lower 2 bits of CCP1CON for LSB of duty cycle
	MOVLW      3
	ANDWF      FARG_setPWM_dutyCycle+0, 0
	MOVWF      R2+0
	MOVF       R2+0, 0
	MOVWF      R0+0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	MOVF       R0+0, 0
	IORWF      CCP1CON+0, 1
;SmartIrrigationSystem.c,59 :: 		}
L_end_setPWM:
	RETURN
; end of _setPWM

_customDelay:

;SmartIrrigationSystem.c,61 :: 		void customDelay(unsigned int milliseconds) {
;SmartIrrigationSystem.c,63 :: 		for (i = 0; i < milliseconds; i++)
	CLRF       R1+0
	CLRF       R1+1
L_customDelay4:
	MOVF       FARG_customDelay_milliseconds+1, 0
	SUBWF      R1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__customDelay40
	MOVF       FARG_customDelay_milliseconds+0, 0
	SUBWF      R1+0, 0
L__customDelay40:
	BTFSC      STATUS+0, 0
	GOTO       L_customDelay5
;SmartIrrigationSystem.c,64 :: 		for (j = 0; j < 165; j++); // Adjust this value based on your clock frequency
	CLRF       R3+0
	CLRF       R3+1
L_customDelay7:
	MOVLW      0
	SUBWF      R3+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__customDelay41
	MOVLW      165
	SUBWF      R3+0, 0
L__customDelay41:
	BTFSC      STATUS+0, 0
	GOTO       L_customDelay8
	INCF       R3+0, 1
	BTFSC      STATUS+0, 2
	INCF       R3+1, 1
	GOTO       L_customDelay7
L_customDelay8:
;SmartIrrigationSystem.c,63 :: 		for (i = 0; i < milliseconds; i++)
	INCF       R1+0, 1
	BTFSC      STATUS+0, 2
	INCF       R1+1, 1
;SmartIrrigationSystem.c,64 :: 		for (j = 0; j < 165; j++); // Adjust this value based on your clock frequency
	GOTO       L_customDelay4
L_customDelay5:
;SmartIrrigationSystem.c,65 :: 		}
L_end_customDelay:
	RETURN
; end of _customDelay

_interrupt:
	MOVWF      R15+0
	SWAPF      STATUS+0, 0
	CLRF       STATUS+0
	MOVWF      ___saveSTATUS+0
	MOVF       PCLATH+0, 0
	MOVWF      ___savePCLATH+0
	CLRF       PCLATH+0

;SmartIrrigationSystem.c,67 :: 		void interrupt(void){
;SmartIrrigationSystem.c,68 :: 		if ( INTCON & 0x02) {  // EXTERNAL INTERRUPT
	BTFSS      INTCON+0, 1
	GOTO       L_interrupt10
;SmartIrrigationSystem.c,69 :: 		unsigned char tempC=TRISC;
	MOVF       TRISC+0, 0
	MOVWF      interrupt_tempC_L1+0
;SmartIrrigationSystem.c,70 :: 		TRISC=0XFF; //11111 1111 Turns off all pumps
	MOVLW      255
	MOVWF      TRISC+0
;SmartIrrigationSystem.c,71 :: 		cccc = 5;
	MOVLW      5
	MOVWF      _cccc+0
	MOVLW      0
	MOVWF      _cccc+1
;SmartIrrigationSystem.c,72 :: 		for (cccc;cccc>0;cccc--){
L_interrupt11:
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      _cccc+1, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__interrupt44
	MOVF       _cccc+0, 0
	SUBLW      0
L__interrupt44:
	BTFSC      STATUS+0, 0
	GOTO       L_interrupt12
;SmartIrrigationSystem.c,73 :: 		PORTD=PORTD & 0XF8;  // TURN OFF LIGHTS
	MOVLW      248
	ANDWF      PORTD+0, 1
;SmartIrrigationSystem.c,75 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,76 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,77 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,78 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,79 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,80 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,81 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,82 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,83 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,84 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,85 :: 		PORTD=PORTD | 0x07;   // TURN ON LIGHTS
	MOVLW      7
	IORWF      PORTD+0, 1
;SmartIrrigationSystem.c,87 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,88 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,89 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,90 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,91 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,92 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,93 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,94 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,95 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,96 :: 		delay_100ms();
	CALL       _Delay_100ms+0
;SmartIrrigationSystem.c,72 :: 		for (cccc;cccc>0;cccc--){
	MOVLW      1
	SUBWF      _cccc+0, 1
	BTFSS      STATUS+0, 0
	DECF       _cccc+1, 1
;SmartIrrigationSystem.c,97 :: 		}
	GOTO       L_interrupt11
L_interrupt12:
;SmartIrrigationSystem.c,98 :: 		TRISC=tempC;
	MOVF       interrupt_tempC_L1+0, 0
	MOVWF      TRISC+0
;SmartIrrigationSystem.c,99 :: 		INTCON=INTCON & 0XFD; //1111 1101 clear INTF flag
	MOVLW      253
	ANDWF      INTCON+0, 1
;SmartIrrigationSystem.c,100 :: 		}
L_interrupt10:
;SmartIrrigationSystem.c,101 :: 		if ( INTCON & 0X04){ // TIMER OVERFLOW
	BTFSS      INTCON+0, 2
	GOTO       L_interrupt14
;SmartIrrigationSystem.c,102 :: 		TMR0 = 6;
	MOVLW      6
	MOVWF      TMR0+0
;SmartIrrigationSystem.c,103 :: 		DelayCntr++;
	INCF       _DelayCntr+0, 1
	BTFSC      STATUS+0, 2
	INCF       _DelayCntr+1, 1
;SmartIrrigationSystem.c,104 :: 		INTCON = INTCON & 0xFB;
	MOVLW      251
	ANDWF      INTCON+0, 1
;SmartIrrigationSystem.c,105 :: 		}
L_interrupt14:
;SmartIrrigationSystem.c,106 :: 		}
L_end_interrupt:
L__interrupt43:
	MOVF       ___savePCLATH+0, 0
	MOVWF      PCLATH+0
	SWAPF      ___saveSTATUS+0, 0
	MOVWF      STATUS+0
	SWAPF      R15+0, 1
	SWAPF      R15+0, 0
	RETFIE
; end of _interrupt

_initialize:

;SmartIrrigationSystem.c,109 :: 		void initialize() {
;SmartIrrigationSystem.c,111 :: 		ADCON1 = 0x80; // Configure PORTA pins as analog
	MOVLW      128
	MOVWF      ADCON1+0
;SmartIrrigationSystem.c,112 :: 		ADC_Init();
	CALL       _ADC_Init+0
;SmartIrrigationSystem.c,115 :: 		configurePWM();
	CALL       _configurePWM+0
;SmartIrrigationSystem.c,118 :: 		TRISA = 0b00000111; // Set RA0, RA1, and RA2 as input
	MOVLW      7
	MOVWF      TRISA+0
;SmartIrrigationSystem.c,119 :: 		TRISC = 0b11111100; // Set RC0 and RC1 as output   , RC3 input
	MOVLW      252
	MOVWF      TRISC+0
;SmartIrrigationSystem.c,120 :: 		TRISB = 0b11000101; // Set RB4 and RB5 as output green and red led
	MOVLW      197
	MOVWF      TRISB+0
;SmartIrrigationSystem.c,123 :: 		RC1_bit = 0; // pump off
	BCF        RC1_bit+0, BitPos(RC1_bit+0)
;SmartIrrigationSystem.c,124 :: 		RB4_bit = 0; // green led off
	BCF        RB4_bit+0, BitPos(RB4_bit+0)
;SmartIrrigationSystem.c,125 :: 		RB5_bit = 0; //red led off
	BCF        RB5_bit+0, BitPos(RB5_bit+0)
;SmartIrrigationSystem.c,126 :: 		RB3_bit = 0; // led initially off
	BCF        RB3_bit+0, BitPos(RB3_bit+0)
;SmartIrrigationSystem.c,129 :: 		PR2 = 249;        // Set PWM period
	MOVLW      249
	MOVWF      PR2+0
;SmartIrrigationSystem.c,130 :: 		T2CON = 0b00000111; // Enable TMR2, prescaler 1:16
	MOVLW      7
	MOVWF      T2CON+0
;SmartIrrigationSystem.c,131 :: 		}
L_end_initialize:
	RETURN
; end of _initialize

_readLightSensor:

;SmartIrrigationSystem.c,133 :: 		int readLightSensor() {
;SmartIrrigationSystem.c,138 :: 		return ADC_Read(0);
	CLRF       FARG_ADC_Read_channel+0
	CALL       _ADC_Read+0
;SmartIrrigationSystem.c,139 :: 		}
L_end_readLightSensor:
	RETURN
; end of _readLightSensor

_readTemperatureSensor:

;SmartIrrigationSystem.c,141 :: 		int readTemperatureSensor() {
;SmartIrrigationSystem.c,142 :: 		ADCON0 = 0b00000001; // Select channel 1 (RA1)
	MOVLW      1
	MOVWF      ADCON0+0
;SmartIrrigationSystem.c,143 :: 		customDelay(1000);   // Custom delay for 1 second
	MOVLW      232
	MOVWF      FARG_customDelay_milliseconds+0
	MOVLW      3
	MOVWF      FARG_customDelay_milliseconds+1
	CALL       _customDelay+0
;SmartIrrigationSystem.c,144 :: 		ADCON0 &= 0b11000011; // Clear the channel selection bits
	MOVLW      195
	ANDWF      ADCON0+0, 1
;SmartIrrigationSystem.c,145 :: 		ADCON0 |= 0b00001000; // Set the channel selection bits for channel 1 (binary 00001 shifted left by 3 bits)
	BSF        ADCON0+0, 3
;SmartIrrigationSystem.c,146 :: 		return ADC_Read(1);
	MOVLW      1
	MOVWF      FARG_ADC_Read_channel+0
	CALL       _ADC_Read+0
;SmartIrrigationSystem.c,147 :: 		}
L_end_readTemperatureSensor:
	RETURN
; end of _readTemperatureSensor

_readHumiditySensor:

;SmartIrrigationSystem.c,149 :: 		int readHumiditySensor() {
;SmartIrrigationSystem.c,150 :: 		return (PORTC & (1 << 3)) != 0;
	MOVLW      8
	ANDWF      PORTC+0, 0
	MOVWF      R1+0
	MOVLW      0
	ANDLW      0
	MOVWF      R1+1
	MOVLW      0
	XORWF      R1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__readHumiditySensor49
	MOVLW      0
	XORWF      R1+0, 0
L__readHumiditySensor49:
	MOVLW      1
	BTFSC      STATUS+0, 2
	MOVLW      0
	MOVWF      R0+0
	MOVLW      0
	MOVWF      R0+1
;SmartIrrigationSystem.c,151 :: 		}
L_end_readHumiditySensor:
	RETURN
; end of _readHumiditySensor

_initPWM_RC1_RC2:

;SmartIrrigationSystem.c,153 :: 		void initPWM_RC1_RC2() {
;SmartIrrigationSystem.c,156 :: 		T2CON=T2CON | 0X07;
	MOVLW      7
	IORWF      T2CON+0, 1
;SmartIrrigationSystem.c,158 :: 		PR2 = 199;  // For a 4MHz oscillator, PR2 value for 500Hz PWM frequency
	MOVLW      199
	MOVWF      PR2+0
;SmartIrrigationSystem.c,161 :: 		CCPR1L = 0;  //RC2
	CLRF       CCPR1L+0
;SmartIrrigationSystem.c,162 :: 		CCPR2L=0;   //RC1
	CLRF       CCPR2L+0
;SmartIrrigationSystem.c,165 :: 		CCP1CON=CCP1CON|0x0C;  //RC2
	MOVLW      12
	IORWF      CCP1CON+0, 1
;SmartIrrigationSystem.c,166 :: 		CCP2CON=CCP2CON|0x0C;  //RC1
	MOVLW      12
	IORWF      CCP2CON+0, 1
;SmartIrrigationSystem.c,167 :: 		}
L_end_initPWM_RC1_RC2:
	RETURN
; end of _initPWM_RC1_RC2

_updatePWM:

;SmartIrrigationSystem.c,169 :: 		void updatePWM(unsigned char dutyCycle, unsigned char pwmChannel) {
;SmartIrrigationSystem.c,170 :: 		if (pwmChannel == 1) {
	MOVF       FARG_updatePWM_pwmChannel+0, 0
	XORLW      1
	BTFSS      STATUS+0, 2
	GOTO       L_updatePWM15
;SmartIrrigationSystem.c,171 :: 		CCPR2L = dutyCycle;  // Update duty cycle value for RC1
	MOVF       FARG_updatePWM_dutyCycle+0, 0
	MOVWF      CCPR2L+0
;SmartIrrigationSystem.c,172 :: 		} else if (pwmChannel == 2) {
	GOTO       L_updatePWM16
L_updatePWM15:
	MOVF       FARG_updatePWM_pwmChannel+0, 0
	XORLW      2
	BTFSS      STATUS+0, 2
	GOTO       L_updatePWM17
;SmartIrrigationSystem.c,173 :: 		CCPR1L = dutyCycle;  // Update duty cycle value for RC2
	MOVF       FARG_updatePWM_dutyCycle+0, 0
	MOVWF      CCPR1L+0
;SmartIrrigationSystem.c,174 :: 		}else{
	GOTO       L_updatePWM18
L_updatePWM17:
;SmartIrrigationSystem.c,175 :: 		CCPR2L = dutyCycle;  // Update duty cycle value for RC1
	MOVF       FARG_updatePWM_dutyCycle+0, 0
	MOVWF      CCPR2L+0
;SmartIrrigationSystem.c,176 :: 		CCPR1L = dutyCycle;  // Update duty cycle value for RC2
	MOVF       FARG_updatePWM_dutyCycle+0, 0
	MOVWF      CCPR1L+0
;SmartIrrigationSystem.c,177 :: 		}
L_updatePWM18:
L_updatePWM16:
;SmartIrrigationSystem.c,178 :: 		}
L_end_updatePWM:
	RETURN
; end of _updatePWM

_main:

;SmartIrrigationSystem.c,180 :: 		void main() {
;SmartIrrigationSystem.c,183 :: 		initialize();
	CALL       _initialize+0
;SmartIrrigationSystem.c,184 :: 		TRISC = 0XFD;
	MOVLW      253
	MOVWF      TRISC+0
;SmartIrrigationSystem.c,186 :: 		Lcd_Init();
	CALL       _Lcd_Init+0
;SmartIrrigationSystem.c,187 :: 		Lcd_Cmd(_LCD_CLEAR);
	MOVLW      1
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,188 :: 		Lcd_Cmd(_LCD_CURSOR_OFF);
	MOVLW      12
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,189 :: 		initPWM_RC1_RC2();
	CALL       _initPWM_RC1_RC2+0
;SmartIrrigationSystem.c,192 :: 		Lcd_Out(1, 1, "Irrigation");
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr1_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,193 :: 		Lcd_Out(2,1,"System");
	MOVLW      2
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr2_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,194 :: 		delay_ms(5000);
	MOVLW      51
	MOVWF      R11+0
	MOVLW      187
	MOVWF      R12+0
	MOVLW      223
	MOVWF      R13+0
L_main19:
	DECFSZ     R13+0, 1
	GOTO       L_main19
	DECFSZ     R12+0, 1
	GOTO       L_main19
	DECFSZ     R11+0, 1
	GOTO       L_main19
	NOP
	NOP
;SmartIrrigationSystem.c,195 :: 		Lcd_Cmd(_LCD_CLEAR);
	MOVLW      1
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,196 :: 		Lcd_Cmd(_LCD_CURSOR_OFF);
	MOVLW      12
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,197 :: 		Lcd_Out(1, 1, "Checking");
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr3_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,198 :: 		delay_ms(5000);
	MOVLW      51
	MOVWF      R11+0
	MOVLW      187
	MOVWF      R12+0
	MOVLW      223
	MOVWF      R13+0
L_main20:
	DECFSZ     R13+0, 1
	GOTO       L_main20
	DECFSZ     R12+0, 1
	GOTO       L_main20
	DECFSZ     R11+0, 1
	GOTO       L_main20
	NOP
	NOP
;SmartIrrigationSystem.c,199 :: 		while(1) {
L_main21:
;SmartIrrigationSystem.c,202 :: 		int tempValue = readTemperatureSensor();
	CALL       _readTemperatureSensor+0
	MOVF       R0+0, 0
	MOVWF      main_tempValue_L1+0
	MOVF       R0+1, 0
	MOVWF      main_tempValue_L1+1
;SmartIrrigationSystem.c,204 :: 		int humidityValue = readHumiditySensor();
	CALL       _readHumiditySensor+0
	MOVF       R0+0, 0
	MOVWF      main_humidityValue_L1+0
	MOVF       R0+1, 0
	MOVWF      main_humidityValue_L1+1
;SmartIrrigationSystem.c,205 :: 		int lightValue = readLightSensor();
	CALL       _readLightSensor+0
	MOVF       R0+0, 0
	MOVWF      main_lightValue_L1+0
	MOVF       R0+1, 0
	MOVWF      main_lightValue_L1+1
;SmartIrrigationSystem.c,209 :: 		float celcius = (tempValue *500)/1023; // Convert sensor value to voltage (assuming a 5V system)
	MOVF       main_tempValue_L1+0, 0
	MOVWF      R0+0
	MOVF       main_tempValue_L1+1, 0
	MOVWF      R0+1
	MOVLW      244
	MOVWF      R4+0
	MOVLW      1
	MOVWF      R4+1
	CALL       _Mul_16X16_U+0
	MOVLW      255
	MOVWF      R4+0
	MOVLW      3
	MOVWF      R4+1
	CALL       _Div_16x16_S+0
	CALL       _int2double+0
	MOVF       R0+0, 0
	MOVWF      main_celcius_L1+0
	MOVF       R0+1, 0
	MOVWF      main_celcius_L1+1
	MOVF       R0+2, 0
	MOVWF      main_celcius_L1+2
	MOVF       R0+3, 0
	MOVWF      main_celcius_L1+3
;SmartIrrigationSystem.c,213 :: 		int tempThresholdHigh = 29; // Corresponding to 26°C
	MOVLW      29
	MOVWF      main_tempThresholdHigh_L1+0
	MOVLW      0
	MOVWF      main_tempThresholdHigh_L1+1
	MOVLW      37
	MOVWF      main_lightThreshold_L1+0
	MOVLW      2
	MOVWF      main_lightThreshold_L1+1
;SmartIrrigationSystem.c,216 :: 		celcius = fabs(celcius);
	MOVF       main_celcius_L1+0, 0
	MOVWF      FARG_fabs_d+0
	MOVF       main_celcius_L1+1, 0
	MOVWF      FARG_fabs_d+1
	MOVF       main_celcius_L1+2, 0
	MOVWF      FARG_fabs_d+2
	MOVF       main_celcius_L1+3, 0
	MOVWF      FARG_fabs_d+3
	CALL       _fabs+0
	MOVF       R0+0, 0
	MOVWF      main_celcius_L1+0
	MOVF       R0+1, 0
	MOVWF      main_celcius_L1+1
	MOVF       R0+2, 0
	MOVWF      main_celcius_L1+2
	MOVF       R0+3, 0
	MOVWF      main_celcius_L1+3
;SmartIrrigationSystem.c,217 :: 		intToStr(celcius,text1);
	CALL       _double2int+0
	MOVF       R0+0, 0
	MOVWF      FARG_IntToStr_input+0
	MOVF       R0+1, 0
	MOVWF      FARG_IntToStr_input+1
	MOVLW      main_text1_L0+0
	MOVWF      FARG_IntToStr_output+0
	CALL       _IntToStr+0
;SmartIrrigationSystem.c,218 :: 		Ltrim(text1);
	MOVLW      main_text1_L0+0
	MOVWF      FARG_Ltrim_string+0
	CALL       _Ltrim+0
;SmartIrrigationSystem.c,219 :: 		Lcd_Cmd(_LCD_CLEAR);
	MOVLW      1
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,220 :: 		Lcd_Cmd(_LCD_CURSOR_OFF);
	MOVLW      12
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,222 :: 		Lcd_Out(1, 8, text1);
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      8
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      main_text1_L0+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,223 :: 		delay_ms(500);
	MOVLW      6
	MOVWF      R11+0
	MOVLW      19
	MOVWF      R12+0
	MOVLW      173
	MOVWF      R13+0
L_main23:
	DECFSZ     R13+0, 1
	GOTO       L_main23
	DECFSZ     R12+0, 1
	GOTO       L_main23
	DECFSZ     R11+0, 1
	GOTO       L_main23
	NOP
	NOP
;SmartIrrigationSystem.c,225 :: 		celcius = fabs(celcius);
	MOVF       main_celcius_L1+0, 0
	MOVWF      FARG_fabs_d+0
	MOVF       main_celcius_L1+1, 0
	MOVWF      FARG_fabs_d+1
	MOVF       main_celcius_L1+2, 0
	MOVWF      FARG_fabs_d+2
	MOVF       main_celcius_L1+3, 0
	MOVWF      FARG_fabs_d+3
	CALL       _fabs+0
	MOVF       R0+0, 0
	MOVWF      FLOC__main+0
	MOVF       R0+1, 0
	MOVWF      FLOC__main+1
	MOVF       R0+2, 0
	MOVWF      FLOC__main+2
	MOVF       R0+3, 0
	MOVWF      FLOC__main+3
	MOVF       FLOC__main+0, 0
	MOVWF      main_celcius_L1+0
	MOVF       FLOC__main+1, 0
	MOVWF      main_celcius_L1+1
	MOVF       FLOC__main+2, 0
	MOVWF      main_celcius_L1+2
	MOVF       FLOC__main+3, 0
	MOVWF      main_celcius_L1+3
;SmartIrrigationSystem.c,226 :: 		if (celcius > tempThresholdHigh) {
	MOVF       main_tempThresholdHigh_L1+0, 0
	MOVWF      R0+0
	MOVF       main_tempThresholdHigh_L1+1, 0
	MOVWF      R0+1
	CALL       _int2double+0
	MOVF       FLOC__main+0, 0
	MOVWF      R4+0
	MOVF       FLOC__main+1, 0
	MOVWF      R4+1
	MOVF       FLOC__main+2, 0
	MOVWF      R4+2
	MOVF       FLOC__main+3, 0
	MOVWF      R4+3
	CALL       _Compare_Double+0
	MOVLW      1
	BTFSC      STATUS+0, 0
	MOVLW      0
	MOVWF      R0+0
	MOVF       R0+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main24
;SmartIrrigationSystem.c,233 :: 		intToStr(celcius,text1);
	MOVF       main_celcius_L1+0, 0
	MOVWF      R0+0
	MOVF       main_celcius_L1+1, 0
	MOVWF      R0+1
	MOVF       main_celcius_L1+2, 0
	MOVWF      R0+2
	MOVF       main_celcius_L1+3, 0
	MOVWF      R0+3
	CALL       _double2int+0
	MOVF       R0+0, 0
	MOVWF      FARG_IntToStr_input+0
	MOVF       R0+1, 0
	MOVWF      FARG_IntToStr_input+1
	MOVLW      main_text1_L0+0
	MOVWF      FARG_IntToStr_output+0
	CALL       _IntToStr+0
;SmartIrrigationSystem.c,234 :: 		Ltrim(text1);
	MOVLW      main_text1_L0+0
	MOVWF      FARG_Ltrim_string+0
	CALL       _Ltrim+0
;SmartIrrigationSystem.c,235 :: 		Lcd_Cmd(_LCD_CLEAR);
	MOVLW      1
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,236 :: 		Lcd_Cmd(_LCD_CURSOR_OFF);
	MOVLW      12
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,237 :: 		Lcd_Out(1, 1, "temp = ");
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr4_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,238 :: 		Lcd_Out(1, 8, text1);
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      8
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      main_text1_L0+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,239 :: 		PORTB = 0b00010000;      //turn on green led
	MOVLW      16
	MOVWF      PORTB+0
;SmartIrrigationSystem.c,240 :: 		updatePWM(20, 1);
	MOVLW      20
	MOVWF      FARG_updatePWM_dutyCycle+0
	MOVLW      1
	MOVWF      FARG_updatePWM_pwmChannel+0
	CALL       _updatePWM+0
;SmartIrrigationSystem.c,241 :: 		delay_ms(5000);   // 5000 milliseconds = 5 seconds
	MOVLW      51
	MOVWF      R11+0
	MOVLW      187
	MOVWF      R12+0
	MOVLW      223
	MOVWF      R13+0
L_main25:
	DECFSZ     R13+0, 1
	GOTO       L_main25
	DECFSZ     R12+0, 1
	GOTO       L_main25
	DECFSZ     R11+0, 1
	GOTO       L_main25
	NOP
	NOP
;SmartIrrigationSystem.c,242 :: 		Lcd_Cmd(_LCD_CLEAR);
	MOVLW      1
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,243 :: 		Lcd_Cmd(_LCD_CURSOR_OFF);
	MOVLW      12
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,244 :: 		Lcd_Out(1, 1, "wATERING");
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr5_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,245 :: 		Lcd_Out(2, 1, "Pump ON FS");
	MOVLW      2
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr6_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,247 :: 		updatePWM(255, 1); // 255 is the maximum duty cycle
	MOVLW      255
	MOVWF      FARG_updatePWM_dutyCycle+0
	MOVLW      1
	MOVWF      FARG_updatePWM_pwmChannel+0
	CALL       _updatePWM+0
;SmartIrrigationSystem.c,248 :: 		delay_ms(5000);   // 5000 milliseconds = 5 seconds
	MOVLW      51
	MOVWF      R11+0
	MOVLW      187
	MOVWF      R12+0
	MOVLW      223
	MOVWF      R13+0
L_main26:
	DECFSZ     R13+0, 1
	GOTO       L_main26
	DECFSZ     R12+0, 1
	GOTO       L_main26
	DECFSZ     R11+0, 1
	GOTO       L_main26
	NOP
	NOP
;SmartIrrigationSystem.c,253 :: 		updatePWM(0, 1);
	CLRF       FARG_updatePWM_dutyCycle+0
	MOVLW      1
	MOVWF      FARG_updatePWM_pwmChannel+0
	CALL       _updatePWM+0
;SmartIrrigationSystem.c,255 :: 		PORTB = 0b00100000; // Turn on red LED
	MOVLW      32
	MOVWF      PORTB+0
;SmartIrrigationSystem.c,256 :: 		Lcd_Cmd(_LCD_CLEAR);
	MOVLW      1
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,257 :: 		Lcd_Cmd(_LCD_CURSOR_OFF);
	MOVLW      12
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,258 :: 		Lcd_Out(1, 1, "Complete");
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr7_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,259 :: 		Lcd_Out(2, 1, "Pump OFF");
	MOVLW      2
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr8_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,260 :: 		}
	GOTO       L_main27
L_main24:
;SmartIrrigationSystem.c,262 :: 		else if (humidityValue == 1) {
	MOVLW      0
	XORWF      main_humidityValue_L1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main53
	MOVLW      1
	XORWF      main_humidityValue_L1+0, 0
L__main53:
	BTFSS      STATUS+0, 2
	GOTO       L_main28
;SmartIrrigationSystem.c,264 :: 		PORTB = 0b00010000; // turn on green led
	MOVLW      16
	MOVWF      PORTB+0
;SmartIrrigationSystem.c,265 :: 		Lcd_Cmd(_LCD_CLEAR);
	MOVLW      1
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,266 :: 		Lcd_Cmd(_LCD_CURSOR_OFF);
	MOVLW      12
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,267 :: 		Lcd_Out(1, 1, "Humidity Low");
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr9_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,268 :: 		Lcd_Out(2,1,"Pump ON");
	MOVLW      2
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr10_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,270 :: 		updatePWM(255, 1); // Full speed
	MOVLW      255
	MOVWF      FARG_updatePWM_dutyCycle+0
	MOVLW      1
	MOVWF      FARG_updatePWM_pwmChannel+0
	CALL       _updatePWM+0
;SmartIrrigationSystem.c,271 :: 		delay_ms(15000);
	MOVLW      153
	MOVWF      R11+0
	MOVLW      49
	MOVWF      R12+0
	MOVLW      162
	MOVWF      R13+0
L_main29:
	DECFSZ     R13+0, 1
	GOTO       L_main29
	DECFSZ     R12+0, 1
	GOTO       L_main29
	DECFSZ     R11+0, 1
	GOTO       L_main29
	NOP
;SmartIrrigationSystem.c,272 :: 		updatePWM(0, 1);
	CLRF       FARG_updatePWM_dutyCycle+0
	MOVLW      1
	MOVWF      FARG_updatePWM_pwmChannel+0
	CALL       _updatePWM+0
;SmartIrrigationSystem.c,273 :: 		PORTB = 0b00100000; // Turn on red LED
	MOVLW      32
	MOVWF      PORTB+0
;SmartIrrigationSystem.c,274 :: 		Lcd_Cmd(_LCD_CLEAR);
	MOVLW      1
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,275 :: 		Lcd_Cmd(_LCD_CURSOR_OFF);
	MOVLW      12
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,276 :: 		Lcd_Out(1, 1, "Complete");
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr11_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,277 :: 		Lcd_Out(2, 1, "Pump OFF");
	MOVLW      2
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr12_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,278 :: 		}
	GOTO       L_main30
L_main28:
;SmartIrrigationSystem.c,280 :: 		else if (lightValue > lightThreshold) {
	MOVLW      128
	XORWF      main_lightThreshold_L1+1, 0
	MOVWF      R0+0
	MOVLW      128
	XORWF      main_lightValue_L1+1, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main54
	MOVF       main_lightValue_L1+0, 0
	SUBWF      main_lightThreshold_L1+0, 0
L__main54:
	BTFSC      STATUS+0, 0
	GOTO       L_main31
;SmartIrrigationSystem.c,281 :: 		intToStr(lightValue,text2);
	MOVF       main_lightValue_L1+0, 0
	MOVWF      FARG_IntToStr_input+0
	MOVF       main_lightValue_L1+1, 0
	MOVWF      FARG_IntToStr_input+1
	MOVLW      main_text2_L0+0
	MOVWF      FARG_IntToStr_output+0
	CALL       _IntToStr+0
;SmartIrrigationSystem.c,282 :: 		Ltrim(text2);
	MOVLW      main_text2_L0+0
	MOVWF      FARG_Ltrim_string+0
	CALL       _Ltrim+0
;SmartIrrigationSystem.c,283 :: 		Lcd_Cmd(_LCD_CLEAR);
	MOVLW      1
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,284 :: 		Lcd_Cmd(_LCD_CURSOR_OFF);
	MOVLW      12
	MOVWF      FARG_Lcd_Cmd_out_char+0
	CALL       _Lcd_Cmd+0
;SmartIrrigationSystem.c,285 :: 		Lcd_Out(1, 1, "Dark");
	MOVLW      1
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr13_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,286 :: 		Lcd_Out(2,1,"Led ON");
	MOVLW      2
	MOVWF      FARG_Lcd_Out_row+0
	MOVLW      1
	MOVWF      FARG_Lcd_Out_column+0
	MOVLW      ?lstr14_SmartIrrigationSystem+0
	MOVWF      FARG_Lcd_Out_text+0
	CALL       _Lcd_Out+0
;SmartIrrigationSystem.c,287 :: 		PORTB = 0b00101000; // Assuming RB3 is connected to the LED
	MOVLW      40
	MOVWF      PORTB+0
;SmartIrrigationSystem.c,288 :: 		delay_ms(10000);
	MOVLW      102
	MOVWF      R11+0
	MOVLW      118
	MOVWF      R12+0
	MOVLW      193
	MOVWF      R13+0
L_main32:
	DECFSZ     R13+0, 1
	GOTO       L_main32
	DECFSZ     R12+0, 1
	GOTO       L_main32
	DECFSZ     R11+0, 1
	GOTO       L_main32
;SmartIrrigationSystem.c,289 :: 		PORTB = 0b00100000; // turn off led
	MOVLW      32
	MOVWF      PORTB+0
;SmartIrrigationSystem.c,290 :: 		}
	GOTO       L_main33
L_main31:
;SmartIrrigationSystem.c,292 :: 		PORTC = 0b00000000;
	CLRF       PORTC+0
;SmartIrrigationSystem.c,293 :: 		PORTB = 0b00100000;
	MOVLW      32
	MOVWF      PORTB+0
;SmartIrrigationSystem.c,294 :: 		}
L_main33:
L_main30:
L_main27:
;SmartIrrigationSystem.c,299 :: 		}
	GOTO       L_main21
;SmartIrrigationSystem.c,300 :: 		}
L_end_main:
	GOTO       $+0
; end of _main
