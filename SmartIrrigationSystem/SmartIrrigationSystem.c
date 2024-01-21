// LCD module connections
sbit LCD_RS at RD4_bit;
sbit LCD_EN at RD5_bit;
sbit LCD_D4 at RD0_bit;
sbit LCD_D5 at RD1_bit;
sbit LCD_D6 at RD2_bit;
sbit LCD_D7 at RD3_bit;

sbit LCD_RS_Direction at TRISD4_bit;
sbit LCD_EN_Direction at TRISD5_bit;
sbit LCD_D4_Direction at TRISD0_bit;
sbit LCD_D5_Direction at TRISD1_bit;
sbit LCD_D6_Direction at TRISD2_bit;
sbit LCD_D7_Direction at TRISD3_bit;
// End LCD module connections
int cccc = 10;
int count;
unsigned int DelayCntr;
unsigned int dutyCycle;



void setPWM(unsigned int dutyCycle);
void configurePWM() {
    // Set CCP1 module for PWM mode
    CCP1CON = 0b00001100; // Set CCP1 module for PWM

    // Configure Timer2 for PWM operation
    PR2 = 249;            // Set PWM period for 20ms (assuming Fosc/4)
    T2CON = 0b00000111;   // Enable TMR2 with prescaler 1:16

    // Initial PWM duty cycle
    CCPR1L = 0x00;        // Set initial duty cycle to 0
    CCP1CON |= 0b00000011; // Initialize CCP1CON for PWM
}
void ADT_Init(void){
     ADCON0 = 0x49;// A/D converter ON, Don't GO, Channel 1, Fosc/16, equivalent to B'01001001'
     ADCON1 = 0xC0;// All channels Analog, 500 KHz, right justified, equivalent to B'11000000'
     TRISA = 0xFF; // PortA is my input
}
unsigned int ADC_read_1(void){
         ADCON0 = ADCON0 | 0x04;              // GO
         while(ADCON0 & 0x04);                // stay in GO until DONE
         return((ADRESH<<8) | ADRESL);        // read/return the result
}
  unsigned int ADC_Read_0(void ) {
    ADCON0 = 0x41;  // Select the specified ADC channel
    ADCON0 = ADCON0 | 0x04;

           while (ADCON0 & 0x04);  // Wait for conversion to complete

    return ((ADRESH << 8) | ADRESL);  // Combine high and low byte results and return
}
void setPWM(unsigned int dutyCycle) {
    // Set PWM duty cycle
    CCPR1L = dutyCycle >> 2;            // Set the upper 8 bits of duty cycle
    CCP1CON &= 0b11001111;              // Clear the lower 2 bits of CCP1CON
    CCP1CON |= ((dutyCycle & 0x03) << 4); // Set the lower 2 bits of CCP1CON for LSB of duty cycle
}

void customDelay(unsigned int milliseconds) {
    unsigned int i, j;
    for (i = 0; i < milliseconds; i++)
        for (j = 0; j < 165; j++); 
}

void interrupt(void){
     if ( INTCON & 0x02) {  // EXTERNAL INTERRUPT
      unsigned char tempC=TRISC;
      TRISC=0XFF; //11111 1111 Turns off all pumps
      cccc = 5;
      for (cccc;cccc>0;cccc--){
      PORTD=PORTD & 0XF8;  
      //MymsDelay1(1000);
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      PORTD=PORTD | 0x07;   
      //MymsDelay1(1000);
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      delay_100ms();
      }
       TRISC=tempC;
      INTCON=INTCON & 0XFD; //1111 1101 clear INTF flag
      }
       if ( INTCON & 0X04){ // TIMER OVERFLOW
      TMR0 = 6;
      DelayCntr++;
      INTCON = INTCON & 0xFB;
      }
      }


void initialize() {
    // Configure ADC
    ADCON1 = 0x80; // Configure PORTA pins as analog
    ADC_Init();

    // Configure PWM
    configurePWM();

    // Configure port directions
    TRISA = 0b00000111; // Set RA0, RA1, and RA2 as input
    TRISC = 0b11111100; // Set RC0 and RC1 as output   , RC3 input
    TRISB = 0b11000101; // Set RB4 and RB5 as output green and red led

    // Initialize outputs
    PORTC = 0b00000000;
    PORTB = 0b00100000;
    // Initialize PWM
    PR2 = 249;        // Set PWM period
    T2CON = 0b00000111; // Enable TMR2, prescaler 1:16
}

int readLightSensor() {
    //ADCON0 = 0b00000000; // Select channel 0 (RA0)
    //customDelay(1000);   // Custom delay for 1 second
    //ADCON0 &= 0b11000011; // Clear the channel selection bits
    //ADCON0 |= 0b00000000; // Set the channel selection bits for channel 0 (binary 00000 shifted left by 3 bits)
    return ADC_Read(0);
}

int readTemperatureSensor() {
    ADCON0 = 0b00000001; // Select channel 1 (RA1)
    customDelay(1000);   // Custom delay for 1 second
    ADCON0 &= 0b11000011; // Clear the channel selection bits
    ADCON0 |= 0b00001000; // Set the channel selection bits for channel 1 (binary 00001 shifted left by 3 bits)
    return ADC_Read(1);
}

int readHumiditySensor() {
    return (PORTC & (1 << 3)) != 0;
}

void initPWM_RC1_RC2() {

    // Configure Timer2 for PWM
    T2CON=T2CON | 0X07;
    // Set PR2 to achieve the desired PWM frequency
    PR2 = 199;  // For a 4MHz oscillator, PR2 value for 500Hz PWM frequency

    // Set the initial duty cycle to 0%
    CCPR1L = 0;  //RC2
    CCPR2L=0;   //RC1

    // Configure CCP1 & CCP2 for PWM mode
   CCP1CON=CCP1CON|0x0C;  //RC2
   CCP2CON=CCP2CON|0x0C;  //RC1
}
// Function to update PWM duty cycle
void updatePWM(unsigned char dutyCycle, unsigned char pwmChannel) {
    if (pwmChannel == 1) {
        CCPR2L = dutyCycle;  // Update duty cycle value for RC1
    } else if (pwmChannel == 2) {
        CCPR1L = dutyCycle;  // Update duty cycle value for RC2
    }else{
       CCPR2L = dutyCycle;  // Update duty cycle value for RC1
       CCPR1L = dutyCycle;  // Update duty cycle value for RC2
    }
}

void main() {
    char text1[7];
    char text2[7];
    initialize();
    TRISC = 0XFD;
    // Initialize LCD
    Lcd_Init();
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);
    initPWM_RC1_RC2();

    // Display "Irrigation System!"
    Lcd_Out(1, 1, "Irrigation");
    Lcd_Out(2,1,"System");
    delay_ms(5000);
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);
    Lcd_Out(1, 1, "Checking");
    delay_ms(5000);
    while(1) {


        int tempValue = readTemperatureSensor();

        int humidityValue = readHumiditySensor();
        int lightValue = readLightSensor();
        int dutyCycle = (tempValue - 240) / 2; 
        float celcius = (tempValue *500)/1023; // Convert sensor value to voltage (assuming a 5V system)
        int tempThresholdHigh = 29; 

        int lightThreshold = 549; 
        celcius = fabs(celcius);
        intToStr(celcius,text1);
        Ltrim(text1);
        Lcd_Cmd(_LCD_CLEAR);
        Lcd_Cmd(_LCD_CURSOR_OFF);

        Lcd_Out(1, 8, text1);
        delay_ms(500);
        celcius = fabs(celcius);
        if (celcius > tempThresholdHigh) {
            intToStr(celcius,text1);
            Ltrim(text1);
            Lcd_Cmd(_LCD_CLEAR);
            Lcd_Cmd(_LCD_CURSOR_OFF);
            Lcd_Out(1, 1, "temp = ");
            Lcd_Out(1, 8, text1);
            PORTB = 0b00010000;      //turn on green led
            updatePWM(20, 1);
            delay_ms(5000);   // 5000 milliseconds = 5 seconds
             Lcd_Cmd(_LCD_CLEAR);
             Lcd_Cmd(_LCD_CURSOR_OFF);
             Lcd_Out(1, 1, "wATERING");
             Lcd_Out(2, 1, "Pump ON FS");
           updatePWM(255, 1); // 255 is the maximum duty cycle
           delay_ms(5000);   // 5000 milliseconds = 5 seconds
            // Set PWM duty cycle to full for 2 minutes


             // Turn off the pump
            updatePWM(0, 1);

            PORTB = 0b00100000; // Turn on red LED
            Lcd_Cmd(_LCD_CLEAR);
             Lcd_Cmd(_LCD_CURSOR_OFF);
             Lcd_Out(1, 1, "Complete");
             Lcd_Out(2, 1, "Pump OFF");
        }

         else if (humidityValue == 1) {
            //PORTC = 0b00000010; // Turn on pump
            PORTB = 0b00010000; // turn on green led
            Lcd_Cmd(_LCD_CLEAR);
            Lcd_Cmd(_LCD_CURSOR_OFF);
            Lcd_Out(1, 1, "Humidity Low");
            Lcd_Out(2,1,"Pump ON");
            // Turn on the pump and green LED at full speed
            updatePWM(255, 1); // Full speed
            delay_ms(15000);
            updatePWM(0, 1);
            PORTB = 0b00100000; // Turn on red LED
            Lcd_Cmd(_LCD_CLEAR);
             Lcd_Cmd(_LCD_CURSOR_OFF);
             Lcd_Out(1, 1, "Complete");
             Lcd_Out(2, 1, "Pump OFF");
              }

        else if (lightValue > lightThreshold) {
             intToStr(lightValue,text2);
            Ltrim(text2);
            Lcd_Cmd(_LCD_CLEAR);
            Lcd_Cmd(_LCD_CURSOR_OFF);
            Lcd_Out(1, 1, "Dark");
            Lcd_Out(2,1,"Led ON");
            PORTB = 0b00101000; // Assuming RB3 is connected to the LED
            delay_ms(10000);
            PORTB = 0b00100000; // turn off led
               }
         else {
            PORTC = 0b00000000;
            PORTB = 0b00100000;
        }


        //delay_ms(2000);

    }
}