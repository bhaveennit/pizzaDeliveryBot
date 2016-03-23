
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>



void init_devicesc(void);
void print_seven_segment_1(int timerp1);
void print_seven_segment_2(int timerp2);
void print_seven_segment_3(int timerp3);
void sevensegment();



//char toggel = 0; //used as a variable for buzzer state toggel action
int timer=0;
int timerp1,timerp2,timerap2,timerp3;





//TIMER4 initialize - prescale:1024
// WGM: 0) Normal, TOP=0xFFFF
// desired value: 1Hz
// actual value:  1.000Hz (0.0%)
void timer4_init(void)
{
 TCCR4B = 0x00; //stop
 TCNT4H = 0x1F; //Counter higher 8 bit value
 TCNT4L = 0x01; //Counter lower 8 bit value
 OCR4AH = 0x00; //Output Compair Register (OCR)- Not used
 OCR4AL = 0x00; //Output Compair Register (OCR)- Not used
 OCR4BH = 0x00; //Output Compair Register (OCR)- Not used
 OCR4BL = 0x00; //Output Compair Register (OCR)- Not used
 OCR4CH = 0x00; //Output Compair Register (OCR)- Not used
 OCR4CL = 0x00; //Output Compair Register (OCR)- Not used
 ICR4H  = 0x00; //Input Capture Register (ICR)- Not used
 ICR4L  = 0x00; //Input Capture Register (ICR)- Not used
 TCCR4A = 0x00; 
 TCCR4C = 0x00;
 TCCR4B = 0x04; //start Timer
}

//This ISR can be used to schedule events like refreshing ADC data, LCD data
ISR(TIMER4_OVF_vect)
{
 //TIMER4 has overflowed
 TCNT4H = 0x1F; //reload counter high value
 TCNT4L = 0x01; //reload counter low value




timer++;

DDRH=0XFF;
DDRJ=0XFF;

//do something

timerp1=timer%10;
timerap2=timer/10;
timerp2=timerap2%10;
timerp3=timer/100;




} 


void print_seven_segment_1(int timerp1)
{
PORTH=0B00010000;

int valuepa1=timerp1;

if(valuepa1==0)
{PORTJ=0B10000000;}

else if(valuepa1==1)
{PORTJ=0B11110110;}

else if(valuepa1==2)
{PORTJ=0B00011000;}

else if(valuepa1==3)
{PORTJ=0B00110000;}

else if(valuepa1==4)
{PORTJ=0B00100110;}

else if(valuepa1==5)
{PORTJ=0B00100001;}

else if(valuepa1==6)
{PORTJ=0B00000001;}

else if(valuepa1==7)
{PORTJ=0B11110100;}

else if(valuepa1==8)
{PORTJ=0B00000000;}

else if(valuepa1==9)
{PORTJ=0B00100000;}

}

void print_seven_segment_2(int timerp2)
{
PORTH=0B00100000;


int valuepa2=timerp2;

if(valuepa2==1)
{PORTJ=0B11110110;}

else if(valuepa2==0)
{PORTJ=0B10000000;}


else if(valuepa2==2)
{PORTJ=0B00011000;}

else if(valuepa2==3)
{PORTJ=0B00110000;}

else if(valuepa2==4)
{PORTJ=0B00100110;}

else if(valuepa2==5)
{PORTJ=0B00100001;}

else if(valuepa2==6)
{PORTJ=0B00000001;}

else if(valuepa2==7)
{PORTJ=0B11110100;}

else if(valuepa2==8)
{PORTJ=0B00000000;}

else if(valuepa2==9)
{PORTJ=0B00100000;}

}

void print_seven_segment_3(int timerp3)
{
PORTH=0B01000000;


int valuepa3=timerp3;

if(valuepa3==1)
{PORTJ=0B11110110;}

else if(valuepa3==0)
{PORTJ=0B10000000;}


else if(valuepa3==2)
{PORTJ=0B00011000;}

else if(valuepa3==3)
{PORTJ=0B00110000;}

else if(valuepa3==4)
{PORTJ=0B00100110;}

else if(valuepa3==5)
{PORTJ=0B00100001;}

else if(valuepa3==6)
{PORTJ=0B00000001;}

else if(valuepa3==7)
{PORTJ=0B11110100;}

else if(valuepa3==8)
{PORTJ=0B00000000;}

else if(valuepa3==9)
{PORTJ=0B00100000;}

}


void init_devicesc(void)
{
 cli(); //Clears the global interrupts
// port_init(); //Initializes all ports 
 timer4_init();
 TIMSK4 = 0x01; //timer4 overflow interrupt enable
 sei();   //Enables the global interrupts
}



void sevensegment()
{
	DDRC=0XFF;
	init_devicesc();
	

	while(1)
	{print_seven_segment_1(timerp1);
_delay_ms(5);


print_seven_segment_2(timerp2);
_delay_ms(5);


print_seven_segment_3(timerp3);
_delay_ms(5);
}
}






