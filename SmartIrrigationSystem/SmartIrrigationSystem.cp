#line 1 "C:/Users/20200365/Desktop/SmartIrrigationSystem/SmartIrrigationSystem.c"

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

int cccc = 10;
int count;
unsigned int DelayCntr;
unsigned int dutyCycle;



void setPWM(unsigned int dutyCycle);
void configurePWM() {

 CCP1CON = 0b00001100;


 PR2 = 249;
 T2CON = 0b00000111;


 CCPR1L = 0x00;
 CCP1CON |= 0b00000011;
}
void ADT_Init(void){
 ADCON0 = 0x49;
 ADCON1 = 0xC0;
 TRISA = 0xFF;
}
unsigned int ADC_read_1(void){
 ADCON0 = ADCON0 | 0x04;
 while(ADCON0 & 0x04);
 return((ADRESH<<8) | ADRESL);
}
 unsigned int ADC_Read_0(void ) {
 ADCON0 = 0x41;
 ADCON0 = ADCON0 | 0x04;

 while (ADCON0 & 0x04);

 return ((ADRESH << 8) | ADRESL);
}
void setPWM(unsigned int dutyCycle) {

 CCPR1L = dutyCycle >> 2;
 CCP1CON &= 0b11001111;
 CCP1CON |= ((dutyCycle & 0x03) << 4);
}

void customDelay(unsigned int milliseconds) {
 unsigned int i, j;
 for (i = 0; i < milliseconds; i++)
 for (j = 0; j < 165; j++);
}

void interrupt(void){
 if ( INTCON & 0x02) {
 unsigned char tempC=TRISC;
 TRISC=0XFF;
 cccc = 5;
 for (cccc;cccc>0;cccc--){
 PORTD=PORTD & 0XF8;

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
 INTCON=INTCON & 0XFD;
 }
 if ( INTCON & 0X04){
 TMR0 = 6;
 DelayCntr++;
 INTCON = INTCON & 0xFB;
 }
 }


void initialize() {

 ADCON1 = 0x80;
 ADC_Init();


 configurePWM();


 TRISA = 0b00000111;
 TRISC = 0b11111100;
 TRISB = 0b11000101;


 RC1_bit = 0;
 RB4_bit = 0;
 RB5_bit = 0;
 RB3_bit = 0;


 PR2 = 249;
 T2CON = 0b00000111;
}

int readLightSensor() {




 return ADC_Read(0);
}

int readTemperatureSensor() {
 ADCON0 = 0b00000001;
 customDelay(1000);
 ADCON0 &= 0b11000011;
 ADCON0 |= 0b00001000;
 return ADC_Read(1);
}

int readHumiditySensor() {
 return (PORTC & (1 << 3)) != 0;
}

void initPWM_RC1_RC2() {


 T2CON=T2CON | 0X07;

 PR2 = 199;


 CCPR1L = 0;
 CCPR2L=0;


 CCP1CON=CCP1CON|0x0C;
 CCP2CON=CCP2CON|0x0C;
}

void updatePWM(unsigned char dutyCycle, unsigned char pwmChannel) {
 if (pwmChannel == 1) {
 CCPR2L = dutyCycle;
 } else if (pwmChannel == 2) {
 CCPR1L = dutyCycle;
 }else{
 CCPR2L = dutyCycle;
 CCPR1L = dutyCycle;
 }
}

void main() {
 char text1[7];
 char text2[7];
 initialize();
 TRISC = 0XFD;

 Lcd_Init();
 Lcd_Cmd(_LCD_CLEAR);
 Lcd_Cmd(_LCD_CURSOR_OFF);
 initPWM_RC1_RC2();


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
 float celcius = (tempValue *500)/1023;



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
 PORTB = 0b00010000;
 updatePWM(20, 1);
 delay_ms(5000);
 Lcd_Cmd(_LCD_CLEAR);
 Lcd_Cmd(_LCD_CURSOR_OFF);
 Lcd_Out(1, 1, "wATERING");
 Lcd_Out(2, 1, "Pump ON FS");

 updatePWM(255, 1);
 delay_ms(5000);




 updatePWM(0, 1);

 PORTB = 0b00100000;
 Lcd_Cmd(_LCD_CLEAR);
 Lcd_Cmd(_LCD_CURSOR_OFF);
 Lcd_Out(1, 1, "Complete");
 Lcd_Out(2, 1, "Pump OFF");
 }

 else if (humidityValue == 1) {

 PORTB = 0b00010000;
 Lcd_Cmd(_LCD_CLEAR);
 Lcd_Cmd(_LCD_CURSOR_OFF);
 Lcd_Out(1, 1, "Humidity Low");
 Lcd_Out(2,1,"Pump ON");

 updatePWM(255, 1);
 delay_ms(15000);
 updatePWM(0, 1);
 PORTB = 0b00100000;
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
 PORTB = 0b00101000;
 delay_ms(10000);
 PORTB = 0b00100000;
 }
 else {
 PORTC = 0b00000000;
 PORTB = 0b00100000;
 }




 }
}
