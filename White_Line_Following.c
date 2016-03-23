
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.c"
#include "Servo_Motor_Control_using_PWM.c"
#include "Buzzer_Beep.c"

unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

void linefollow(unsigned char valuel,unsigned char valuec,unsigned char valuer);
void linefollowp(unsigned char valuel,unsigned char valuec,unsigned char valuer);



int nodecorrect(unsigned char valuel,unsigned char valuec,unsigned char valuer);
int nodecorrectp(unsigned char valuel,unsigned char valuec,unsigned char valuer);
int nodecorrecto(unsigned char valuel,unsigned char valuec,unsigned char valuer);


void homedeliver(int hn);
void topizzashop(int hns);

void pizzashopstart(int sstart);
void gotopizzapoint(int shopno);

void turnright90();
void turnleft90();
void turnrighta90();
void turnlefta90();

void back40();
void backpickup();		
void forwardpickup();
void uturn();
void backstart();
void forwardshop();
void turnright120();
void turnleft120();
void backarm();
void backbar();
void bartop5ini();
void linealign();
void forwardh8h11();

void goh2fromh1();
void goh11fromh8();
void goh12();
void comeh11start();
void comestarth12();


void pickup();
void simplyplace();
void storepizzai3();
void pickpizzai3andplaceflex();




//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}


unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;



//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}





//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
 

}


void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}

//ISR for right position encoder
ISR(INT5_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
 	
}


//ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
 

}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}


void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortARestore = PORTA; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore; 			// setting the command to the port
}


//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();	
	left_encoder_pin_config(); //left encoder pin config
 	right_encoder_pin_config(); //right encoder pin config	
 
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}




//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}



void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	left_position_encoder_interrupt_init();
 	right_position_encoder_interrupt_init();
 
	sei();   //Enables the global interrupts
}





//Function used for setting motor's direction


void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop (void) //hard stop
{
  motion_set(0x00);
}



		int node=0;
		int nodep=0;	


int main()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	
	

	//to go till our start node
	bartop5ini();
	//linealign();

	//go to get the 115 pizza
	//gotopizzapoint(5);
	back();
	velocity(235,250);
	_delay_ms(2200);
	stop();
	storepizzai3();
	
stop();
_delay_ms(5000);
stop();
_delay_ms(5000);
stop();
_delay_ms(5000);
stop();
_delay_ms(5000);

	//directly from p5 to p7
	//p5p7();
	//linealign();

	forward();
	velocity(250,245);
	_delay_ms(3200);
	stop();

	pickup();

	//linealign();
	back();
	velocity(245,250);
	_delay_ms(1300);
	stop();
	turnleft90();
	ShaftCountLeft=0;
	ShaftCountRight=0;

	//pizzashopstart(7);

	//deliever the 40 pizza at h1
	homedeliver(1);
	simplyplace();
	
	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
	
	
	goh2fromh1();
	
	pickpizzai3andplaceflex();
	
	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
	
	
	
	topizzashop(2);


	turnlefta90();
	forward();
	velocity(250,245);
	_delay_ms(2500);
	stop();
	storepizzai3();

	back();
	velocity(245,250);
	
	_delay_ms(5000);
	
	_delay_ms(3500);
	stop();
	
_delay_ms(300);
	pickup();

	forward();
	velocity(250,245);
	_delay_ms(5500);
	stop();

	turnleft90();

	//take the  200 one pizza
//	gotopizzapoint(8);
	
	//pizzashopstart(8);
	
	//we directly go from p8 to p1
//	p8p1();
	
	
	//pizzashopstart(1);
	
	// deliever the 100 wala pizza at H8
	ShaftCountLeft=0;
	ShaftCountRight=0;
	
	homedeliver(8);
	
	simplyplace();
	
	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
	
	// go from h8 to h11
	
	goh11fromh8();
	
	back();
	velocity(250,250);
	_delay_ms(350);
	stop();
	pickpizzai3andplaceflex();
	
	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
	
	//go to start from h11
	
	comeh11start();
	

	turnlefta90();
	back();
	velocity(245,250);
	_delay_ms(4400);
	stop();
	_delay_ms(300);
	storepizzai3();

	//we will go directly from p3 to p7
//	p3p7();
	forward();
	velocity(250,245);
	_delay_ms(5400);
	stop();

	pickup();
	//pizzashopstart(7);
	back();
	velocity(245,250);
	_delay_ms(1600);
	stop();

	turnlefta90();
	
	ShaftCountLeft=0;
	ShaftCountRight=0;

	// go to deliever the 250 wala pizza
	goh12();
	
	back();
	velocity(250,250);
	_delay_ms(400);
	stop();
	simplyplace();
	
	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
	
	comestarth12();

	//go to take the 350 one pizza
	//gotopizzapoint(3);
	// take the other 350 wala pizza
//	gotopizzapoint(4);
	turnlefta90();
	back();
	velocity(245,250);
	_delay_ms(3200);
	stop();
	pickup();

	forward();
	velocity(250,245);
	_delay_ms(2850);
	stop();

	turnlefta90();


//	pizzashopstart(4);
	//go to deliever the final set of pizza
	
	ShaftCountLeft=0;
	ShaftCountRight=0;

	homedeliver(5);
	simplyplace();
	
	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
	
	uturn();
	backstart();
	backstart();
	pickpizzai3andplaceflex();
	
	buzzer_on();
	_delay_ms(5000);
	buzzer_off();
	
}	






// since our start must be marvelous for the intial pizza the function will be written spcially to avoid any errors at the start
void bartop5ini()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	node=0;
	nodep=0;
	int z;
	unsigned char valuel,valuec,valuer;
	

	

while(1)
{
	
	
	valuel=ADC_Conversion(3);
	valuec=ADC_Conversion(2);
	valuer=ADC_Conversion(1);
	z=nodecorrecto(valuel,valuec,valuer);

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3	

		lcd_print(2,1,z,2);
		lcd_print(2,5,ShaftCountLeft,3);
		lcd_print(2,9,ShaftCountRight,3);

if ((z==1)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{
stop();
turnright120();

break;
}



else
linefollow(valuel,valuec,valuer);

}
}
	



//now lets write functions which are simple and nat for picking an placing the blocks 
//small is 1 medium is 2 and large is 3 

void pickup()
{init_devicesa();
servo_3(50);
_delay_ms(500);
servo_1(72);
_delay_ms(1000);

servo_2(25);
_delay_ms(1300);

servo_3(120);
_delay_ms(500);

servo_1(0);
_delay_ms(1000);

servo_2(100);
_delay_ms(1000);
servo_1_free();
servo_2_free();
servo_3_free();

}


void simplyplace()
{
init_devicesa();
servo_1(72);
_delay_ms(1200);
servo_2(25);
_delay_ms(1200);

servo_3(50);
_delay_ms(300);

servo_1(10);
_delay_ms(1000);

servo_2(120);
_delay_ms(1000);
servo_1_free();
servo_2_free();
servo_3_free();

}

void storepizzai3()
{init_devicesa();
servo_3(50);
_delay_ms(300);

servo_1(72);
_delay_ms(100);

servo_2(25);
_delay_ms(1000);

servo_3(120);
_delay_ms(600);

servo_1(0);
_delay_ms(1200);

servo_1_free ();
_delay_ms(500);

servo_2(155);
_delay_ms(1200);

servo_2_free ();
_delay_ms(500);

servo_3(35);
_delay_ms(300);

servo_1_free();
servo_2_free();
servo_3_free();

}






void pickpizzai3andplaceflex()
{init_devicesa();


servo_3(05);
_delay_ms(300);

servo_1(0);
_delay_ms(800);

servo_2(155);
_delay_ms(800);

servo_3(120);
_delay_ms(1200);

servo_1(72);
_delay_ms(800);


servo_2(25);
_delay_ms(1500);

servo_3(35);
_delay_ms(300);

servo_1(10);
_delay_ms(1000);

servo_2(120);
_delay_ms(1000);

servo_1_free();
servo_2_free();
servo_3_free();

}







//now we write to go from h1 to h2
void goh2fromh1()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	node=0;
	int g;
	unsigned char valuel,valuec,valuer;
	
	back40();
	turnright120();
	ShaftCountLeft=0;
	ShaftCountRight=0;

while(1)
{
	
	
	valuel=ADC_Conversion(3);
	valuec=ADC_Conversion(2);
	valuer=ADC_Conversion(1);
	g=nodecorrect(valuel,valuec,valuer);

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3	

		lcd_print(2,1,g,2);
		lcd_print(2,5,ShaftCountLeft,3);
		lcd_print(2,9,ShaftCountRight,3);

if ((g==1)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{
turnleft90();
forwardshop();
break;
}


else
linefollow(valuel,valuec,valuer);

}
}



//now we write the program for h8 to h11
void goh11fromh8()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	node=0;
	int e;
	unsigned char valuel,valuec,valuer;
	
	back40();
	turnright90();
	ShaftCountLeft=0;
	ShaftCountRight=0;

while(1)
{
	
	
	valuel=ADC_Conversion(3);
	valuec=ADC_Conversion(2);
	valuer=ADC_Conversion(1);
	e=nodecorrect(valuel,valuec,valuer);

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3	

		lcd_print(2,1,e,2);
		lcd_print(2,5,ShaftCountLeft,3);
		lcd_print(2,9,ShaftCountRight,3);

if ((e==1)&&((valuec>=15&&valuel>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuec>=15&&valuer>=15)||(valuel>=70&&valuec>=70&&valuer>=70)))
{forwardh8h11();
turnright90();}

if ((e==2)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{forwardh8h11();
turnright90();}

if ((e==3)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{forwardh8h11();
turnleft90();
}

if ((e==5)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{stop();
break;}

else
linefollow(valuel,valuec,valuer);

}
}

//now lets write a program to got to h12
void goh12()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	node=0;
	int i;
	unsigned char valuel,valuec,valuer;

while(1)
{
	
	
	valuel=ADC_Conversion(3);
	valuec=ADC_Conversion(2);
	valuer=ADC_Conversion(1);
	i=nodecorrect(valuel,valuec,valuer);

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3	

		lcd_print(2,1,i,2);
		lcd_print(2,5,ShaftCountLeft,3);
		lcd_print(2,9,ShaftCountRight,3);

if ((i==2)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{forwardh8h11();
turnright90();}

if ((i==3)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{stop();
break;}


else
linefollow(valuel,valuec,valuer);

}
}

//now lets write a program to go to start from h12
void comestarth12()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	node=0;
	int h;
	unsigned char valuel,valuec,valuer;

	uturn();
	ShaftCountLeft=0;
	ShaftCountRight=0;


while(1)
{
	
	
	valuel=ADC_Conversion(3);
	valuec=ADC_Conversion(2);
	valuer=ADC_Conversion(1);
	h=nodecorrect(valuel,valuec,valuer);

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3	

		lcd_print(2,1,h,2);
		lcd_print(2,5,ShaftCountLeft,3);
		lcd_print(2,9,ShaftCountRight,3);


if ((h==1)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{forwardh8h11();
turnleft90();}

if ((h==3)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{
stop();
break;}

else
linefollow(valuel,valuec,valuer);

}
}


//now lets write a function to come back to the start from h11
void comeh11start()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	node=0;
	int f;
	unsigned char valuel,valuec,valuer;
	uturn();
	ShaftCountLeft=0;
	ShaftCountRight=0;

while(1)
{
	
	
	valuel=ADC_Conversion(3);
	valuec=ADC_Conversion(2);
	valuer=ADC_Conversion(1);
	f=nodecorrect(valuel,valuec,valuer);

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3	

		lcd_print(2,1,f,2);
		lcd_print(2,5,ShaftCountLeft,3);
		lcd_print(2,9,ShaftCountRight,3);

if ((f==2)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{forwardh8h11();
turnright90();}

if ((f==3)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{forwardh8h11();
turnleft90();}

if ((f==5)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{stop();
break;}

else
linefollow(valuel,valuec,valuer);

}
}

//now lets make the three other direct functions since,we will write it here so that it in accordance with nodecorrectp


//lets now make a function to come back from a house
//there would be three types of node categories 1 node,penultimate and ultimate
void topizzashop(int hns)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	node=0;
	int penultimate,ultimate;
	unsigned char valuel,valuec,valuer;
	int a;

if (hns==8)
penultimate=1;
else if (hns==1)
penultimate=1;

else if (hns==7)
penultimate=2;
else if (hns==2)
penultimate=2;

else if (hns==6)
penultimate=4;
else if (hns==3)
penultimate=4;

else if (hns==5)
penultimate=5;
else if (hns==4)
penultimate=5;


if (hns==8)
ultimate=2;
else if (hns==1)
ultimate=2;

else if (hns==7)
ultimate=3;
else if (hns==2)
ultimate=3;

else if (hns==6)
ultimate=5;
else if (hns==3)
ultimate=5;

else if (hns==5)
ultimate=6;
else if (hns==4)
ultimate=6;

back40();

if(hns==8||hns==6)
turnright90();
else if(hns==1||hns==3)
turnleft90();

else if(hns==7||hns==5)
turnright120();
else
turnleft120();

while(1)
{
	
	
	valuel=ADC_Conversion(3);
	valuec=ADC_Conversion(2);
	valuer=ADC_Conversion(1);
	a=nodecorrect(valuel,valuec,valuer);

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3	

		lcd_print(2,1,a,2);
		lcd_print(2,5,ShaftCountLeft,3);
		lcd_print(2,9,ShaftCountRight,3);
	
	

if ((a==penultimate)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{

if(hns>=5)
turnleft90();
else
turnright90();

}

else if ((a==ultimate)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{
stop();
_delay_ms(1000);
break;
}

else
linefollow(valuel,valuec,valuer);


}
}



// this function is only to deliver in the houses named from 1 to 8

void homedeliver(int hn)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	
	node=0;
	int correctnode,cf;
	unsigned char valuel,valuec,valuer;
	int b;

if (hn==8)
{correctnode=2;
cf=3;}
else if (hn==1)
{correctnode=2;
cf=3;}
else if (hn==7)
{correctnode=3;
cf=4;}
else if (hn==2)
{correctnode=3;
cf=4;}

else if (hn==6)
{correctnode=5;
cf=6;}
else if (hn==3)
{correctnode=5;
cf=6;}
else if (hn==5)
{correctnode=6;
cf=7;}
else if (hn==4)
{correctnode=6;
cf=8;}

	

while(1)
{
	
	
	valuel=ADC_Conversion(3);
	valuec=ADC_Conversion(2);
	valuer=ADC_Conversion(1);
	b=nodecorrect(valuel,valuec,valuer);

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3	

		lcd_print(2,1,b,2);
		lcd_print(2,5,ShaftCountLeft,3);
		lcd_print(2,9,ShaftCountRight,3);
	
	



if ((b==1)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{
if(hn>=5)
{
turnright90();
}

else
{
forwardshop();
turnleft90();
}

}
else if ((b==correctnode)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))

{
if (hn>=5)
{
turnright90();}

else
{
turnleft90();
}

}

else if ((b==cf)&&((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuec>110)||(valuer>110)||(valuel>110)||(valuel>=70&&valuec>=70&&valuer>=70)))
{
backstart();
break;}

else
linefollow(valuel,valuec,valuer);


}
}


void backarm()
{
stop();
back();
velocity(250,250);

_delay_ms(700);
stop();
}



void forwardshop()
{
stop();
forward();
velocity(250,250);
_delay_ms(700);
stop();
}

void forwardpickup()
{
stop();
forward();
velocity(250,250);
_delay_ms(300);
stop();
}

void forwardh8h11()
{
stop();
forward();
velocity(250,250);
_delay_ms(250);
stop();
}


void backstart()
{
stop();
back();
velocity(250,250);

_delay_ms(500);
stop();
}

void backbar()
{
stop();
back();
velocity(250,250);

_delay_ms(700);
stop();
}




void backpickup()
{
stop();
back();
velocity(250,250);

_delay_ms(300);
stop();
}


void back40()
{
stop();
back();
velocity(250,250);

_delay_ms(1100);

stop();
}


void uturn()
{
stop();
right();
velocity(250,250);
_delay_ms(1700);
stop();
}




void turnlefta90()
{
stop();
left();
velocity(150,250);
_delay_ms(1100);
stop();
}


void turnright90()
{
stop();
right();
velocity(250,150);
_delay_ms(900);
stop();
}


void turnleft90()
{
stop();
left();
velocity(150,250);
_delay_ms(900);
stop();
}


void turnright120()
{
stop();
right();
velocity(250,150);
_delay_ms(1200);
stop();
}

void turnleft120()
{
stop();
left();
velocity(150,250);
_delay_ms(1200);
stop();
}



int nodecorrect(unsigned char valuel,unsigned char valuec,unsigned char valuer)
{
if((((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuel>=70&&valuec>=70&&valuer>=70)||(valuec>110)||(valuer>110)||(valuel>110))&&(ShaftCountRight>75))||(ShaftCountRight>200))
{
node++;
ShaftCountLeft=0;
ShaftCountRight=0;
}
return node;
}

int nodecorrecto(unsigned char valuel,unsigned char valuec,unsigned char valuer)
{
if((((valuec>=15&&valuel>=15)||(valuec>=15&&valuer>=15)||(valuel>=70&&valuec>=70&&valuer>=70)||(valuec>110)||(valuer>110)||(valuel>110))&&(ShaftCountRight>40))||(ShaftCountRight>200))
{
node++;
ShaftCountLeft=0;
ShaftCountRight=0;
}
return node;
}


void linefollow(unsigned char valuel,unsigned char valuec,unsigned char valuer)
{

	
		if(valuel<=15&&valuer<=15&&valuec<=90)
		{
		forward();
		velocity(250,250);
		}
        
		else if(valuel>=10&&valuer<=10 )
		{
		forward();
		velocity(170,250);
		}
	
	    else if(valuer>=10&&valuel<=10)
		{
		forward();
		velocity(250,170);
		
		} 
		
		else if((valuec>=90&&valuel>=90)||(valuec>=90&&valuer>=90)||(valuec>=70&&valuel>=70&&valuer>=70))
		{forward();
		velocity(200,200);
		}

			
		if(valuec<=10&&valuer<=10&&valuel<=10)
		
		{back();
		velocity(120,120);
		 }

}



