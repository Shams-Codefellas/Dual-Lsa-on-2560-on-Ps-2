/* author - SHAMS HASAN RIZVI

 
PORTA - INDICATOR 1
PORTB - MOTOR PWM'S
PORTC - PS2 COMMUNICATION
PORTD - USART 1 and trigger
PORTE - USART 0 AND TRIGGER
PORTF - INDICATOR 2
PORTG - MOTOR DIRECTIONS
PORTH - USED FOR READING DATA[0] ON PS2
PORTJ - USED FOR READING DATA[1] ON PS2
PORTI - NOT USED
PORTK - READING USART0
PORTL - READING USART1


Bit Map based on controller above on the shelf.

Data[0]=0b11111110  default when nothing is pressed.

Right=11011110
Left=01111110
Down=10111110
Up=11001110
Left_Joystick_press=11111000
Right_Joystick_press=11110010

Data[1]=0xff  default when nothing is presses.

X=10111111;
Square=01111111
Triangle=11001111
O=11011111
R1=11100111
R2=11111001
L1=11110011
L2=11111100


*/


#ifndef F_CPU
#define F_CPU 8000000UL // or whatever may be your frequency
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define baudrate 38400
#define BRX ((F_CPU/(16UL*baudrate))-1)	//baudrate prescaler
#define BRX2 BRX

void initialize()
{

	//USART0  PE0 AND PE1
	UBRR0H = (unsigned char)(BRX >> 8);   //initialising baud rate of usart0 	
	UBRR0L = (unsigned char)BRX;

	//USART1 =PD2 AND PD3
	UBRR1H = (unsigned char)(BRX2 >> 8);   //initialising baud rate of usart1 
	UBRR1L = (unsigned char)BRX2;

	UCSR0C =  (1 << UCSZ01) | (1 << UCSZ00) ;  //asyn mode, 8 bit data size
	UCSR0B = (1 << TXEN0)  | (1 << RXEN0)  ;   //enable recieve,transmit

	UCSR1C =  (1 << UCSZ11) | (1 << UCSZ10) ;  //asyn mode, 8 bit data size
	UCSR1B = (1 << TXEN1)  | (1 << RXEN1)  ;   //enable receive,transmit

	// ports and pins
	
	DDRE = 0x04;		//PE2 - LSA TRIGGER for usart0  PE0 AND PE1 ARE RX AN TX
	PORTE = 0x04;		//declared as on and output
	
	DDRD = 0x01;		//PD0 - LSA TRIGGER for usart1  PD2 AND PD3 ARE RX AND TX
	PORTD = 0x01;		//declared as on and output

	DDRK = 0xff;       //output of usart0
	DDRL = 0xff;       //output of usart1
	
	DDRA = 0xff;        //indicative of usart0
	DDRF = 0xff;        //indicative of usart1
	DDRH = 0xff;        //reading data[0]
	DDRJ = 0xff;	    //reading data[1]

	DDRF = 0xff;        //indicative of usart1

	
}


uint8_t lsa_receive2()
{
	/*PORTF = 0xFF;
	_delay_ms(200);   //indicator 1
	PORTF = 0x00;
	_delay_ms(200);*/
	while((UCSR1A & (1<<RXC1)) == 0);   //this receive function is still fucked , last known working status = 10/12/2016
	
	return UDR1;
}

uint8_t lsa_receive()
{
	/*PORTA = 0xFF;
	_delay_ms(200);    //indicator 2
	PORTA = 0x00;
	_delay_ms(200);*/
	while((UCSR0A & (1<<RXC0)) == 0);   //this receive function is still fucked , last known working status = 10/12/20
	return UDR0;
}

void lsa_transmit(uint8_t data)
{
	while((UCSR0A & (1<<UDRE0)) == 0);   //not used 
	UDR0 = data;
}



void lsa_transmit2(uint8_t data)
{
	while((UCSR1A & (1<<UDRE1)) == 0);  //not used
	UDR1 = data;
}


short int data[2];
#define PSdata             1 //PC1
#define PScommand          2 //PC2
#define PSclock            7 //PC7
#define PSattention        6 //PC6
int i;

#define sbi(x,y)  x|=(1<<y)  // sets the y(name of bit is y), in the register x.
#define cbi(x,y)  x&=~(1<<y) //clears the y(name of bit is y), in the register x.

void pwm_init()
{
    // initialize TCCR0A as per requirement, say as follows
    TCCR0A = 0x83;  //Fast pwm, no prescaling, clear oco on match ,non-inverted counting.
    TCCR2A = 0x83;  //Fast pwm, no prescaling, clear oco on match ,non-inverted counting.
    TCCR1A = 0xA1;  //enable ocr1a and ocr1b to clear oco on match and non-inverted counting , fast pwm mode
    TCCR1B = 0x09;  //no prescaling and fast pwm .
  
    // make sure to make OC0 pin (pin PB3 for atmega32) as output pin
    DDRB |= (1<<PB7); //ocr0a
    DDRB |= (1<<PB4); //ocr2a
    DDRB |= (1<<PB5); //ocr1a
    DDRB |= (1<<PB6); //ocr1b

    PORTB = 0xff;   //switched on as all motor related stuff happens here.
    
}

void init_PS2() {  //basically initalising to set pin no,s for interaction and the procedure of getting the calls.
	
	sbi(DDRC, PC7);
	cbi(DDRC, PC1);   //all say something a , so probably they are analog pins.
	sbi(PORTC, PC1);
	cbi(DDRC, PC3);
	sbi(PORTC, PC3);
	sbi(DDRC, PC2);
	sbi(DDRC, PC6);
	int_PS2inanalougemode();
}

int gameByte(short int command) {
	short int i ;
	_delay_us(1);
	short int data = 0x00;
	for(i = 0;i < 8;i++) {
		if(command & _BV(i))
			sbi(PORTC, PScommand);
		else
			cbi(PORTC, PScommand);          //gameByte is the function that returns the other side of the duplex transmission.
		cbi(PORTC, PSclock);
		_delay_us(1);
		if((PINC & _BV(PSdata)))
			sbi(data, i);
		else
			cbi(data, i);
		sbi(PORTC, PSclock);
	}
	sbi(PORTC, PScommand);
	_delay_us(20);
	return(data);
}

void  int_PS2inanalougemode() {
	unsigned char chk_ana = 0, cnt = 0;
	while(cnt < 25) {
		sbi(PORTC, PScommand);  //pscommand is 2.
		sbi(PORTC, PSclock);    //ps clock is 7
		cbi(PORTC, PSattention);  //ps attention is 6
		gameByte(0x01);
		gameByte(0x43);          //go into config mode.
		gameByte(0x00);
		gameByte(0x01);
		gameByte(0x00);
		sbi(PORTC, PScommand);       
		_delay_ms(1);
		sbi(PORTC, PSattention);
		_delay_ms(10);
		sbi(PORTC, PScommand);   //why 2nd time set bit ??
		sbi(PORTC, PSclock);
		cbi(PORTC, PSattention);
		gameByte(0x01);
		gameByte(0x44);   	//turn on analog mode.
		gameByte(0x00);
		gameByte(0x01);
		gameByte(0x03);
		gameByte(0x00);
		gameByte(0x00);
		gameByte(0x00);
		gameByte(0x00);
		sbi(PORTC, PScommand);
		_delay_ms(1);
		sbi(PORTC, PSattention);
		_delay_ms(10);
		sbi(PORTC, PScommand);
		sbi(PORTC, PSclock);
		cbi(PORTC, PSattention);
		gameByte(0x01);
		gameByte(0x43);
		gameByte(0x00);
		gameByte(0x00);
		gameByte(0x5A);
		gameByte(0x5A);
		gameByte(0x5A);      //exit config mode.
		gameByte(0x5A);
		gameByte(0x5A);
		sbi(PORTC, PScommand);
		_delay_ms(1);
		sbi(PORTC, PSattention);
		_delay_ms(10);
		sbi(PORTC, PScommand);
		sbi(PORTC, PSclock);
		cbi(PORTC, PSattention);
		gameByte(0x01);
		chk_ana = gameByte(0x42);
		gameByte(0x00);
		gameByte(0x00);
		gameByte(0x00);     //set of values that we have to send to get the ouput back in full duplex transmission.
		gameByte(0x00);
		gameByte(0x00);
		gameByte(0x00);
		gameByte(0x00);
		sbi(PORTC, PScommand);
		_delay_ms(1);
		sbi(PORTC, PSattention);
		_delay_ms(10);
		cnt++;
	}
}

short int PS2_commn() {
	short int temp, data0, data1, data2, data3, data4, data5;
	sbi(PORTC, PScommand);
	cbi(PORTC, PSattention);
	gameByte(0x01);
	temp = gameByte(0x42);
	gameByte(0x00);			//all data's are corresponding to somevalue in the controller
	data0 = gameByte(0x00);         //l,r,u,d,select,start,lj,rj
	data1 = gameByte(0x00); 	//square,triangle,circle,x,l1,r1,l2,r2.
	data2 = gameByte(0x00);         //had square thoda
	data3 = gameByte(0x00);         //has left joystick and right down action.
	data4 = gameByte(0x00);         //has right joystick nly
	data5 = gameByte(0x00);         //probably something else.
	_delay_us(1); 
	sbi(PORTC, PScommand);
	_delay_us(1);
	sbi(PORTC, PSattention);
	data[0] = data0;
	data[1] = data1;
	_delay_ms(30);
	return 1;
}

int main(void) 
{
	initialize();    //initialising all uart stuff
	/*DDRA=0xff;	//display data[0]
	DDRK=0xff;*/	//display data[1]
	DDRG=0xff;	//motor control so all are declared as output.
	
	
	
	
	
	pwm_init();  //sets b as output and all other motor related stuff
	uint8_t pwm;
	volatile unsigned int curr,curr2,prev2=35,prev=35;
	pwm = 255;
	
	init_PS2();
	while(1) {
		PS2_commn();
		PORTH=data[0];		//data display
		PORTJ=data[1];		//data display
		
//*************conditions*********************    //conditions based on the bit mapping.

	if (data[0]==0b11111110)// no button pressed                    
	{
		PORTG=0b00000000;    //motor switched off
		
					
					OCR0A=pwm;
					OCR2A=pwm;
					OCR1A=pwm;
				    	OCR1B=pwm;
					
	}
	else if (data[0]==0b01111110)	//left button
	{
		PORTG=0b10010110;	//bkwd motion for right motor and entire bot.  
					OCR0A=pwm;
					OCR2A=pwm;
					OCR1A=pwm;
				    	OCR1B=pwm;

		PORTA = 0x0f;
		_delay_ms(2000);
		PORTE &= ~(1<<PE2);       //UART0 TRIGGER
		curr = lsa_receive();
		PORTE |= (1<<PE2);
		PORTA = 0xF0;
		_delay_ms(2000);
	    	PORTK = curr;  //READING THE VALUE OF USART0
		prev = curr;
		_delay_ms(5);
		
			
	}
	else if(data[0] == 0b11011110)  //right button
	{
		PORTG = 0b01101001;
		
		PORTF = 0x0f;
		_delay_ms(2000);
		PORTD &= ~(1<<PD0);  	           //UART1 TRIGGER
		curr2 = lsa_receive2();
		PORTD |= (1<<PD0);
		PORTF = 0xF0;
		_delay_ms(2000);
	    	PORTL = curr2;       //READING USART1
		prev2 = curr2;
		_delay_ms(5);

					OCR0A=pwm;
					OCR2A=pwm;
					OCR1A=pwm;
				    OCR1B=pwm;

	}
	else if(data[0] == 0b11001110)   //up button
	{
		PORTG = 0b10101010;

					OCR0A=pwm;
					OCR2A=pwm;
					OCR1A=pwm;
				    OCR1B=pwm;


	}
	else if(data[0] == 0b10111110)  //down button
	{
		PORTG = 0b01010101;

					OCR0A=pwm;
					OCR2A=pwm;
					OCR1A=pwm;
				    OCR1B=pwm;


	}
	else if(data[0] == 0b11111000)  //left joystick press to 45 degree left
	{
		PORTG = 0b10000010;
					
					OCR0A=pwm;
					OCR2A=pwm;
					OCR1A=pwm;
				    OCR1B=pwm;


	}
	else if(data[0] == 0b11110010)	  //right joytsick press  to 45degree right
	{
		PORTG = 0b00101000;

					OCR0A=pwm;
					OCR2A=pwm;
					OCR1A=pwm;
				    OCR1B=pwm;


	}
	else if(data[1] == 0b11100111) //r1 press increment pwm
	{
		pwm = pwm + 10;
	}
	else if(data[1] == 0b11110011)  //l1 press decrement pwm
	{
		pwm = pwm - 10;
	}
 } // end of while


} // end of main

