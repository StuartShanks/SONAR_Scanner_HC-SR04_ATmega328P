#define F_CPU 16000000UL
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

//the following are definitions for pins for lcd.h
#define D0 eS_PORTC5
#define D1 eS_PORTC4
#define D2 eS_PORTB5
#define D3 eS_PORTC2
#define D4 eS_PORTD4
#define D5 eS_PORTC1
#define D6 eS_PORTC0
#define D7 eS_PORTD7
#define RS eS_PORTB0
#define EN eS_PORTC3

//the following are definitions for the HC-SR04 sensor
#define trigPin 3
#define echoPin 2

//header files
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "lcd.h"	//custom library, lcd.h, provided by the university

//volatile global variables,
//must be volatile as can now change in ISR 
static volatile unsigned int echoFlag = 0;	//volatile int , echoFlag = 0.
static volatile unsigned int pulseEnd = 0;	//volatile int , pulseEnd = 0.

//function to initialise USART
void initUSART()
{
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8); //this sets the BAUD rate, must shift UBRROH 8 bits to the right
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);	//sets lower 8 bits of UBRR0
	UCSR0B = (1<<TXEN0); //enables send of data
	UCSR0C = (3<<UCSZ00); //it is set to 8 bit
}

//initialise values
void init()
{
	DDRB = 0xFF;	//set pins on PortB to output
	PORTB = 0x00;	//set the pins to low initially, prevent floating
	DDRC = 0xFF;	//set pins as output, for LCD screen
	DDRD = 0xFF;	//set as output pins for LCD screen
	DDRD &= ~(1<<echoPin);	//echoPin is input
	PORTD = 0x00;	//set pins to low, prevent floating
	EIMSK |= (1<<INT0);		//interrupt INT0 enabled
	TIMSK2 |= (1<<TOIE2);	//enable overflow interrupt on Timer 2
	EICRA |= (1<<ISC00); // interrupt on rising edge and falling edge	
}

//initialise servo
void initServo()
{
	DDRB |= (1<<PINB1); // Set pin 9 to output PWM
	/* 1. Set Fast PWM mode 14: WGM11, WGM12, WGM13 to 1*/
	/* 2. Set pre-scale of 8 */
	/* 3. Set Fast PWM non-inverting mode */
	TCCR1A |= (1 << WGM11) | (1 << COM1A1);
	TCCR1B |= (1 << WGM13) | (1 << WGM12) |(1 << CS11);
	/* 4. Set ICR1: ICR1 is the top defining PWM period */
	ICR1 = 40000;
	/* 5. Set duty cycle, may need offset to correct */
}

//delay function using timer0
void delayFunction_Timer0(int a)	//8 bit timer
{
	OCR0A = a;	//counter limit
	TCNT0 = 0x00;		//initialise Timer0 bits
	TCCR0A |= (1<<WGM01);		//Timer0, set to CTC mode
	TCCR0B |= (1<<CS00);		//Timer0, CS00 = no pre-scaler
	while ((TIFR0 & (1<<OCF0A)) == 0) {	//count up to OCF0 value
	}
	TCCR0B = 0x00;		//stop clock when OCF1 value is reached
	TIFR0 = (1<<OCF0A);		//set flag to 1
}

//pulse 10uSecs to trigger input, this starts the ranging process
void signalPulse()
{
	PORTB |= (1<<trigPin);	//set trigPin high
	delayFunction_Timer0(0xA4);	//0x9F = 159 = 15 uSecs
	PORTB &= ~(1<<trigPin);		//set trigPin low again	
}

//function to iterate over the message string
//cannot directly iterate over the string, so must use char pointer
void putStringUSART(char* stringPtr)	//take pointer to char as parameter
{
	while (*stringPtr != 0) {	//while the data the pointer points to is not null
		while (!(UCSR0A & (1<<UDRE0)));	//wait to receive data
		UDR0 = *stringPtr;	//UDR0 = dereferenced stringPtr characters
		stringPtr++;	//advance stringPtr by 1
	}
}

//this function prints the distance bar to the LCD screen
void distanceBar()
{
	//this sets how many cm is represented by each char on LCD screen
	//changing this value is possible up to a max value of 17. 
	//(17x16 = 272cm range, max is 275cm thus cannot go beyond 17)
	int cmPerChar = 2;	
	
	if (pulseEnd > 0 && pulseEnd <= cmPerChar) {
		Lcd8_Write_String("+");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > cmPerChar && pulseEnd <= 2*cmPerChar) {
		Lcd8_Write_String("++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 2*cmPerChar && pulseEnd <= 3*cmPerChar) {
		Lcd8_Write_String("+++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 3*cmPerChar && pulseEnd <= 4*cmPerChar) {
		Lcd8_Write_String("++++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 4*cmPerChar && pulseEnd <= 5*cmPerChar) {
		Lcd8_Write_String("+++++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 5*cmPerChar && pulseEnd <= 6*cmPerChar) {
		Lcd8_Write_String("++++++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 6*cmPerChar && pulseEnd <= 7*cmPerChar) {
		Lcd8_Write_String("+++++++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 7*cmPerChar && pulseEnd <= 8*cmPerChar) {
		Lcd8_Write_String("++++++++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 8*cmPerChar && pulseEnd <= 9*cmPerChar) {
		Lcd8_Write_String("+++++++++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 9*cmPerChar && pulseEnd <= 10*cmPerChar) {
		Lcd8_Write_String("++++++++++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 10*cmPerChar && pulseEnd <= 11*cmPerChar) {
		Lcd8_Write_String("+++++++++++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 11*cmPerChar && pulseEnd <= 12*cmPerChar) {
		Lcd8_Write_String("++++++++++++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 12*cmPerChar && pulseEnd <= 13*cmPerChar) {
		Lcd8_Write_String("+++++++++++++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 14*cmPerChar && pulseEnd <= 15*cmPerChar) {
		Lcd8_Write_String("++++++++++++++");
		Lcd8_Write_String("                     ");
	}
	
	else if (pulseEnd > 16*cmPerChar && pulseEnd <= 17*cmPerChar) {
		Lcd8_Write_String("+++++++++++++++");
		Lcd8_Write_String("                     ");
	}	
	
	else {
		Lcd8_Write_String("                     ");
	}	
}

//for measuring angle, according to datasheet: inherent measurement angle is 15 degrees.
//servo motor  can turn 180 degrees.
//so we only need to measure angle from (0+15) <= angle <= (180-15)
//However, servo is unreliable at either extreme (close to 0 and 180 degrees)
//So, this servo will loop between 50 and 165 degrees
void angleLoop()
{
	int16_t pulseRange = 0;	//counter is 0 initially
	char pulseString[5];		//char array, used for sending string to USART
	char angleString[5];		//char array, for sending angle to USART
	
	//measure from 50 to 165 degrees
	for (int angle = 50; angle <= 165; angle++) {
		
		OCR1A = angle*25;	//OCR1A scales with the angle, multiplied by a constant
		_delay_ms(30);	//30ms delay, prevents de-bouncing 
		signalPulse();		//call signal pulse
		pulseRange = pulseEnd;	//pulseRange takes the value of TCNT2
		putStringUSART("Distance: ");	//print the distance to data visualizer
		itoa(pulseRange, pulseString, 10);		//convert int to string
		putStringUSART(pulseString);		//print string to data visualizer
		putStringUSART(" cm, ");	//print to data visualizer, new line after
		putStringUSART("Angle: ");	//print the distance to data visualizer
		itoa(angle, angleString, 10);		//convert int to string
		putStringUSART(angleString);	//print the angle to data visualiser
		putStringUSART(".\n");	//then do new line
		_delay_ms(1);	//1ms delay to prevent de-bouncing
		Lcd8_Set_Cursor(2,0);	//set new position of cursor
		distanceBar();	//go to distanceBar()
		_delay_ms(1);	//1ms delay to prevent de-bouncing
	}
	
	//measure from 165 back to 50 degrees
	for (int angle = 165; angle >= 50 ; angle--) {
		OCR1A = angle*25;		//OCR1A scales with the angle, multiplied by a constant
		_delay_ms(30);		//30ms delay, prevents de-bouncing 
		signalPulse();		//call signal pulse
		pulseRange = pulseEnd;		//pulseRange takes the value of TCNT2
		putStringUSART("Distance: ");	//print the distance to data visualizer
		itoa(pulseRange, pulseString, 10);		//convert int to string
		putStringUSART(pulseString);		//print string to data visualizer
		putStringUSART(" cm, ");	//print to data visualizer, new line after
		putStringUSART("Angle: ");	//print the distance to data visualizer
		itoa(angle, angleString, 10);		//convert int to string
		putStringUSART(angleString);	//print the angle to data visualiser
		putStringUSART(".\n");		//then do new line
		_delay_ms(1);			//1ms delay to prevent de-bouncing
		Lcd8_Set_Cursor(2,0);	//set new position of cursor
		distanceBar();		//go to distanceBar()
		_delay_ms(1);		//1ms delay to prevent de-bouncing
	}	
}

int main()
{
	initUSART(); //USART initialization
	init();	//output initialization
	initServo();
	Lcd8_Init();	//lcd8 initialisation
	_delay_ms(50);
	sei();		//set external interrupt
	while(1) {
		Lcd8_Set_Cursor(1,0);	//set location of LCD cursor
		Lcd8_Write_String("Distance:");		//write this message to LCD screen
		angleLoop();	//go to angleLoop
	}	
	return 0;
}

ISR(INT0_vect)
{
	//if the flag is 0, start timer2 with 1024 pre-scaler
	if(echoFlag == 0) {
		TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20);	//1024 pre-scaler
		echoFlag = 1;	//set flag to 1
	}
	else {
		TCCR2B = 0;	//stop timer
		pulseEnd = TCNT2;	//pulse takes values of counter
		TCNT2 = 0;		//reset counter
		echoFlag = 0;		//reset flag
	}
}

//this interrupt will reset Timer 2 if overflow occurs. 
//However, because the timer is 8 bit, it is limited to showing only values up to 275cm.
//Therefore, the Timer will never count beyond the range of the sensor, 400cm.
//Thus, overflow will never occur. This is simply here to increase robustness of the system.
ISR(TIMER2_OVF_vect){
	TCCR2B = 0;	//stop timer
	TCNT2 = 0;	//reset counter
	echoFlag = 0;	//reset flag
}
