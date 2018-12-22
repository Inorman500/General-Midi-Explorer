#define F_CPU 4000000
#include<io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<avr/io.h>
#define USART_BAUDRATE 31250

#define BAUD_PRESCALE 7

/*
/
/ Lab 2
/ ECE 353 Computer Systems Lab 1
/ Team Members: 1. Ivan Norman
/				2. Vincent Nguyen
/				3. Alessy LeBlanc
/				4. Aleck Chen
/
*/
unsigned int TM16_ReadTCNT1(void) // This funcion reads the 16bit timer 
{
	unsigned char sreg;
	unsigned int i;

	sreg = SREG;	// save global interrupt flag
	cli();			// disable interrupts
	i = TCNT1;		
	SREG = sreg;	// restore global interrupt flag
	return i;
}

void TM16_WriteTCNT1(unsigned int i)// This values changes the Value of the 16bit timer
{
	unsigned char sreg;

	sreg = SREG;	// save global interrupt flag
	cli();			// disable interrupts
	TCNT1 = i;		
	SREG = sreg;	// restore global interrupt flag
}

ISR(TIMER1_COMPB_vect)// This is an inturrupt to turn all the LED's on port B off after 800ms
{
	PORTB = 0;

}

void timeInit()// This function initializes the timer
{	TCNT1=0;
	sei();// Enable global interrupts 
	OCR1B = 12500;// This is quivilant to 800ms
	TIMSK|=1<<OCIE1B;// Enables timer 1 B inturrupts
	TCCR1B |= (1<<CS12);// SET PRESCALER TO 256, start timer 
	
}

void USART_Init()// This function initializes USART
{
	UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
	UBRRL = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
	UCSRB = (1 << RXEN) | (1 << TXEN);   // Turn on the transmission and reception circuitry
	UCSRC = (1 << URSEL)| (3 << UCSZ0); // Use 8-bit character sizes
}


void USART_Transmit( unsigned char data )
{	
	while( !(UCSRA & (1<<UDRE)) ); // wait for empty transmit buffer
	UDR = data;
}

unsigned char USART_Receive( void )
{
	while(!(UCSRA & (1<<RXC))) // wait for data to be received
	{
		if (!recBool())
		{
			return 0x000;
		}
	
	
	}
	return UDR;
}


void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
	while(EECR & (1<<EEWE)); 	// completion of previous write
	EEAR = uiAddress; 			// sets up address and data registers
	EEDR = ucData;
	EECR |= (1<<EEMWE); 		// logical one to EEMWE
	EECR |= (1<<EEWE); 			// start eeprom write by setting EEWE

}

unsigned char EEPROM_read(unsigned int uiAddress)
{
	
	while(EECR & (1<<EEWE)); 	// completion of previous write
	EEAR = uiAddress; 			// sets up address register
	EECR |= (1<<EERE); 			// logical one to EEMWE
	return EEDR; 				// return data from register
}
void InitADC()// initialize the adc
{
 //Rrescalar div factor =128
ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
ADMUX|=(1<<REFS0);
///ADMUX&=~(1<<ADLAR);

}

int ReadADC(uint8_t ch)// read the adc
{
   //Select ADC Channel ch must be 0-7
   ch=ch&0b00000111;
   ADMUX|=ch;
 	ADCSRA |= (1 << ADEN);  // Enable ADC
   ADCSRA |= (1 << ADSC);  // Start A2D Conversions
   while(ADCSRA & (1 << ADSC)); // wait for the conversion to be done
   
   int val=ADCL;// read from adl first
   val= ADCH<<8|val;

   ADCSRA|=(1<<ADIF);
	//ADMUX&=~(1<<ADLAR);
   return val;
}

int record (void)// THis function records notes played by the user and stores is in EEPROM
{

	int notespace = -1;//Address that the note will go into
	int delayspace= 3;// the position in EEPROM that the note will go in
	int noteCount=0;// number of ntoes counted so far 
	unsigned int time=0; // timer variable
	unsigned int noteLength=0;
	unsigned int noteDelay=0;
	double tickScale = 4194/65535;// scales down the value of the timer so that it fits in EEPROM
	int nextNote=0;// tells us when the next note comes in 
	// I only need to recieve the note on , and artificially create the note off
	
	
	// each note is stored in EEPROM like the following
	// Notebit 1 | Notebit 2| delaybit1 | delaybit 2 | delaybit3 | delaybit 4|
	
	//notebit 1 and 2 represrnt the bits of the note 
	//delaybit 1 and 2 represrnt the delay between the current note and note off
	
	//delaybit 3 and 4 represrnt the delay between note off and the next note 
	
	
	while( recBool() && (noteCount < 204))
	{
	uint8_t status = USART_Receive();// status
	uint8_t note = USART_Receive();// Data 1
	uint8_t data2 = USART_Receive();// data
		if (data2==100)
		{
			if(nextNote == 1)
			{
				unsigned int offTime=TM16_ReadTCNT1(); // this is fine 
				double difference =(offTime - time); /// this is fine
				double scaled=(difference*4194)/65535; // this works fine
				noteDelay = scaled;
				char delaybit1 = noteDelay>>8; // Shift of the right 2 chars
				char delaybit2 = noteDelay & 0x0FF; // keep the right 2 chars
				
				// this will write the bits for the delay between the the note off and the current note
				EEPROM_write(++notespace, delaybit1);
				EEPROM_write(++notespace, delaybit2);
			}

			PORTB = note; // turn the keys on
			TCNT1=0;// reset timer
			time = TM16_ReadTCNT1();
		
			EEPROM_write(++notespace, note);

			nextNote=0;
			noteCount++;
		}

		else if (data2==64)
		{	
			unsigned int offTime=TM16_ReadTCNT1(); // this is fine. read the current counter
			double difference =(offTime - time); /// Calculate the differnce between note on and off

			double scaled=(difference*4194)/65535; // scale down the time
			noteLength = scaled;
			// split the note into two differnt bytes to store into EEPROM 
			
			char delaybit1 = noteLength>>8; // Shift of the right 2 chars
			char delaybit2 = noteLength & 0x0FF; // keep the right 2 chars

			
			//write both Bits into EEPROM. THes bits reprenet the time between note on and note off 
			EEPROM_write(++notespace, delaybit1);
			EEPROM_write(++notespace, delaybit2);

			time = TM16_ReadTCNT1();// update time with current time
			nextNote=1;
		}

		}
		return notespace;// This lets us know where the last note is
		
}

void playback(){// This function playsback notes stored in EEPROM
		int x = 0;
		while(!modifyBool()&&playbackBool()&&(EEPROM_read(x) != 0x0)){ // This checks if we reached the end of a recording session or if Modify is switched on
			// Transmit note on
     			USART_Transmit(0x90);
			USART_Transmit(EEPROM_read(x));// Transmit the note stored by user
			USART_Transmit(0x64);
			PORTB=EEPROM_read(x);// This will light up the LEDs in acordance to the note played
			TCNT1=0;
			////////
			_delay_ms((EEPROM_read(x+1)<<8) | (EEPROM_read(x+2))); // This will mimic the same delay the user had between note on  and note off
			//////// transmit note off 
			USART_Transmit(0x80);
			USART_Transmit(EEPROM_read(x));
			USART_Transmit(0x40);
			///////
			_delay_ms((EEPROM_read(x+3)<<8) | (EEPROM_read(x+4)));// delay between note off and next note
			x = x + 5;
		}
		
	}
int photoCell(unsigned int delay){// This function will return a value from the photo cell. The more light there is, the higher the value returned
		uint16_t r;
		r = ReadADC(7);
		// max value 1023
		if((r > 1000)){// max
			return delay * 4;
		}
		if((r >= 750) && (r < 1000)){ // normal condition
			return delay * 3;
		}
		if((r >= 400) && (r < 750)){
			return delay * 2;
		}
		if((r <400)){ // this is covered
			return delay * 1;
		}
	}

void modify() // When the modify switch is tunred on, this will use values from the photo cell to either slow or speed up the playback
{
	int x = 0;
		while(modifyBool()&&(EEPROM_read(x) != 0x0)){// check if modify is on and if we haven't reached the end of a recoring session
			USART_Transmit(0x90);
			USART_Transmit(EEPROM_read(x));// tranmit note from note on
			USART_Transmit(0x64);
			PORTB=EEPROM_read(x); // display note on LED bard
			TCNT1=0;// restart TImer count
			////////
			_delay_ms(photoCell((EEPROM_read(x+1)<<8) | (EEPROM_read(x+2)))); // delay till notee off
			////////
			USART_Transmit(0x80);
			USART_Transmit(EEPROM_read(x));
			USART_Transmit(0x40);
			///////
			_delay_ms(photoCell((EEPROM_read(x+3)<<8) | (EEPROM_read(x+4)))); // delay till next note on
			x = x + 5;// Move to next note 
		}

}
int recBool()// This function checks if record is on
{
	return PINA & (1<<PIN0);
}


int playbackBool()// This checks if playback is on
{
return PINA & (1<<PIN2);
}
int modifyBool()// This checks if modify is on
{
return PINA & (1<<PIN1);
}


int main(void)
{
	DDRB = 0xFF; //initially turn LED's off
	PORTB = 0;
	InitADC();// initialise ADC, USART, and TIMER
	USART_Init();
	timeInit();	

	while(1)
	{	
		

		if (PINA & (1<<PIN2))// check playback 
		{
			if (PINA & (1<<PIN1)) // check modify
			{
			modify();
			}
			playback();
			_delay_ms(200);
				
		}
		if (PINA & (1<<PIN0)) //check record
		{
			int lastNote =  record(); // get the position of the last bit in EEPROM
			// We write an empty note at the end of the last note of the recording session to indicate the end of a session.
			EEPROM_write(lastNote+1, 0x00); // delay to till nbext note is 0
			EEPROM_write(lastNote+2, 0x00);// delay to next note is 0
			EEPROM_write(lastNote+3, 0x00); 
		}

	
	}

}	
