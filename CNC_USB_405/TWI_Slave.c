//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright Ruedi Heimlicher 2007. All rights reserved.
//



#include <string.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

#include "lcd.c"
#include "adc.c"
#include "version.c"
#include "usb_rawhid.c"
//#include "ringbuffer.c"

// USB
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))


volatile uint8_t do_output=0;
static volatile uint8_t buffer[32]={};
static volatile uint8_t sendbuffer[32]={};

// begin Ringbuffer
#define RINGBUFFERTIEFE 4
#define READYBIT   0       // buffer kann Daten aufnehmen
#define FULLBIT   1        // Buffer ist voll
#define STARTBIT   2       // Buffer ist geladen
#define RINGBUFFERBIT 3    // Ringbuffer wird verwendet
#define LASTBIT   4         // Letzter Abschnitt  ist geladen
#define ENDBIT   5          // Letzter Abschnitt  ist abgearbeitet
#define STOPBIT   6        // Ablauf stoppen
#define FIRSTBIT   7

static volatile uint8_t CNCDaten[RINGBUFFERTIEFE][33];

//volatile uint8_t inposition= 0;
//volatile uint8_t outposition= 0;

static volatile uint16_t    abschnittnummer=0;
static volatile uint16_t    endposition= 0xFFFF;
static volatile uint8_t     ladeposition=0;

static volatile uint8_t     ringbufferstatus=0x00;   

static volatile uint16_t    AbschnittCounter=0;
volatile uint8_t           liniencounter= 0;
// end Ringbuffer
// end USB 

//#define SDAPIN		4
//#define SCLPIN		5

// SPI
#define OSZIPORT	PORTA
#define OSZIPORTDDR	DDRA
#define OSZIPORTPIN	PINA
#define OSZI_PULS_A	0
#define OSZI_PULS_B	1

#define OSZI_A_LO    OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZI_A_HI    OSZIPORT |= (1<<OSZI_PULS_A)


#define OSZI_B_LO    OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI    OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG  OSZIPORT ^= (1<<OSZI_PULS_B)
// SPI



#define TIMER0_STARTWERT	0x40

#define LOOPLEDDDR          DDRF
#define LOOPLEDPORT         PORTF
#define LOOPLED             4 
#define DC                  7    // DC ON: LO
#define STROM               4    // Stepperstrom ON: LO

#define TASTENDDR           DDRF
#define TASTENPORT          PORTF
#define TASTENPIN           PINF

#define TASTE0             0   // HALT-PIN Motor A
#define TASTE1             1


#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define WDTBIT              7

// CNC12
#define CMD_PORT            PORTD
#define CMD_DDR             DDRD
#define CMD_PIN             PIND


// Auf Stepperport 1
#define END_A0_PIN          6       //  PIN fuer Endanschlag A0 
#define END_B0_PIN          7 		//           Endanschlag B0 


// Auf Stepperport 2
#define END_C0_PIN          6       //  PIN fuer Endanschlag C0 
#define END_D0_PIN          7 		//           Endanschlag D0 





#define RICHTUNG_A	0  // Bit auf richtung
#define RICHTUNG_B	1
#define RICHTUNG_C	2
#define RICHTUNG_D	3

#define GO_HOME            3 // Bit fuer befehl beginn home auf cncstatus
#define At_HOME            4 // Bit fuer befehl beginn home auf cncstatus

#define STEPPERPORT_1	PORTC
#define STEPPERDDR_1    DDRC
#define STEPPERPIN_1    PINC

// Seite 1
#define MA_STEP         0           // PINs auf Stepperport 1
#define MA_RI           1  
#define MA_EN           2

#define MB_STEP         3           // PINs auf Stepperport 1
#define MB_RI           4
#define MB_EN           5

#define END_A0          4           // Anschlagstatus:  Bit fuer Endanschlag bei A0 (NICHT PIN)
#define END_B0          5           // Anschlagstatus:  Bit fuer Endanschlag bei A0

// Seite 2

#define STEPPERPORT_2	PORTB
#define STEPPERDDR_2    DDRB
#define STEPPERPIN_2    PINB

#define MC_STEP         0           // PIN auf Stepperport 2
#define MC_RI           1
#define MC_EN           2

#define MD_STEP         3           // PIN auf Stepperport 2
#define MD_RI           4
#define MD_EN           5

#define END_C0          6           // Anschlagstatus: Bit fuer Endanschlag bei C0
#define END_D0          7           // Anschlagstatus:  Bit fuer Endanschlag bei D0

#define HALT_PIN 0

#define COUNT_A				0        // Motorstatus:   Schritte von Motor A zaehlen
#define COUNT_B				1        // Motorstatus:   Schritte von Motor B zaehlen

#define COUNT_C				2        // Motorstatus:   Schritte von Motor C zaehlen
#define COUNT_D				3        // Motorstatus:   Schritte von Motor D zaehlen



#define USB_DATENBREITE    32
#define USB_SEND           0

volatile uint8_t timer0startwert=TIMER0_STARTWERT;

void delay_ms(unsigned int ms);

static volatile uint8_t    anschlagstatus=0x00;
static volatile uint8_t    cncstatus=0x00;

static volatile uint8_t    motorstatus=0x00;

volatile uint8_t           usbstatus=0x00;

static volatile uint8_t richtung=0;
 
volatile uint8_t           status=0;

volatile uint8_t           PWM=0;
static volatile uint8_t    pwmposition=0;


// CNC

volatile uint16_t CounterA=0;			// Zaehler fuer Delay von Motor A 
volatile uint16_t CounterB=0;			// Zaehler fuer Delay von Motor B
volatile uint16_t CounterC=0;			// Zaehler fuer Delay von Motor C 
volatile uint16_t CounterD=0;			// Zaehler fuer Delay von Motor D

volatile uint16_t DelayA=24;			// Delay von Motor A 
volatile uint16_t DelayB=24;			// Delay von Motor B 
volatile uint16_t DelayC=24;			// Delay von Motor C 
volatile uint16_t DelayD=24;			// Delay von Motor D 

volatile uint16_t StepCounterA=0;	// Zaehler fuer Schritte von Motor A 
volatile uint16_t StepCounterB=0;	// Zaehler fuer Schritte von Motor B
volatile uint16_t StepCounterC=0;	// Zaehler fuer Schritte von Motor C 
volatile uint16_t StepCounterD=0;	// Zaehler fuer Schritte von Motor D



void slaveinit(void)
{
	//OSZIPORTDDR |= (1<<PULS);	// Output
	//OSZIPORT |= (1<<PULS);		// HI
	

	STEPPERDDR_1 |= (1<<MA_STEP);
	STEPPERPORT_1 |= (1<<MA_STEP);	// HI
	
	STEPPERDDR_1 |= (1 << MA_RI);
	STEPPERPORT_1 |= (1 << MA_RI);	// HI
   
	STEPPERDDR_1 |= (1 << MA_EN);
//	STEPPERPORT_1 &= ~(1 << MA_EN);   // LO
   STEPPERPORT_1 |= (1 << MA_EN);	// HI
	
	STEPPERDDR_1 |= (1 << MB_STEP);
	STEPPERPORT_1 |= (1 << MB_STEP); // HI
	
	STEPPERDDR_1 |= (1 << MB_RI);
	STEPPERPORT_1 |= (1 << MB_RI);	// HI
	
	STEPPERDDR_1 |= (1 << MB_EN);
//	STEPPERPORT_1 &= ~(1 << MB_EN); // LO
   STEPPERPORT_1 |= (1 << MB_EN);	// HI
   
   //Seite 2
	STEPPERDDR_2 |= (1<<MC_STEP);
	STEPPERPORT_2 |= (1<<MC_STEP);	// HI
	
	STEPPERDDR_2 |= (1 << MC_RI);
	STEPPERPORT_1 |= (1 << MC_RI);	// HI
   
	STEPPERDDR_2 |= (1 << MC_EN);
	//STEPPERPORT_2 &= ~(1 << MC_EN);   // LO
	STEPPERPORT_2 |= (1 << MC_EN);	// HI
	
	STEPPERDDR_2 |= (1 << MD_STEP);
	STEPPERPORT_2 |= (1 << MD_STEP); // HI
	
	STEPPERDDR_2 |= (1 << MD_RI);
	STEPPERPORT_2 |= (1 << MD_RI);	// HI
	
	STEPPERDDR_2 |= (1 << MD_EN);
	//STEPPERPORT_2 &= ~(1 << MD_EN); // LO
   STEPPERPORT_2 |= (1 << MD_EN);	// HI
   
   
	
	//Pin 0 von   als Ausgang fuer OSZI
   /*
	OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
    OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
    OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
    OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
	
    */
	TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang fŸr Taste 0
	TASTENPORT |= (1<<TASTE0);	//Pull-up

//	DDRB &= ~(1<<PORTB1);	//Bit 1 von PORT B als Eingang fŸr Taste 1
//	PORTB |= (1<<PORTB1);	//Pull-up
	

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin 4 von PORT D als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT D als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD

	
   // Anschlaege
   
   STEPPERPORT_1 &= ~(1<<END_A0_PIN);			//	Eingang fŸr Endanschlag A0
	STEPPERPORT_1 |= (1<<END_A0_PIN);			// Pull-up

	STEPPERPORT_1 &= ~(1<<END_B0_PIN);			//	Eingang fŸr Endanschlag B0
	STEPPERPORT_1 |= (1<<END_B0_PIN);			// Pull-up

   
   STEPPERDDR_2 &= ~(1<<END_C0_PIN);			//	Eingang fŸr Endanschlag C0
	STEPPERPORT_2 |= (1<<END_C0_PIN);			// Pull-up

   STEPPERDDR_2 &= ~(1<<END_D0_PIN);			//	Eingang fŸr Endanschlag D0
	STEPPERPORT_2 |= (1<<END_D0_PIN);			// Pull-up

	
   LOOPLEDDDR |= (1<<LOOPLED);                // Blink-LED
   LOOPLEDPORT |= (1<<LOOPLED);

   CMD_DDR |= (1<<DC);                       // DC-PWM-Ausgang
   CMD_PORT &= ~(1<<DC);                      // LO

   CMD_DDR |= (1<<STROM);                    // Stepperstrom-Ausgang, Active HI
   CMD_PORT &= ~(1<<STROM);                  // LO
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

void timer0 (void) 
{ 
// Timer fuer Exp
	//TCCR0 |= (1<<CS01);						// clock	/8
	//TCCR0 |= (1<<CS01)|(1<<CS02);			// clock	/64
	TCCR0B |= (1<<CS02)| (1<<CS02);			// clock	/256
	//TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
	TCCR0B |= (1 << CS10); // Set up timer 
	OCR0A = 0x2;
	
	//TIFR |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);							//Overflow Interrupt aktivieren
	TCNT0 = TIMER0_STARTWERT;					//RŸcksetzen des Timers

}

/*
void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);							//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
	TCCR2 |= (1<<CS20)|(1<<CS21);				//Takt /64	Intervall 64 us

	TCCR2 |= (1<<WGM21);							//	ClearTimerOnCompareMatch CTC

	//OC2 akt
//	TCCR2 |= (1<<COM20);						//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2);							//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<OCIE2);							//CTC Interrupt aktivieren

	TCNT2 = 0x00;									//Zaehler zuruecksetzen
	
	OCR2 = wert;									//Setzen des Compare Registers auf Servoimpulsdauer
} 
*/

volatile uint16_t timer2Counter=0; 

ISR (TIMER2_OVF_vect) 
{ 
	timer2Counter +=1;
	
	if (timer2Counter >= 14) 
	{
		CounterA+=1;
		CounterB+=1;
      CounterC+=1;
      CounterD+=1;
      
      if (PWM)
      {
         pwmposition ++;
      }
      else
      {
         pwmposition =0;
      }
      
		timer2Counter = 0; 
        //OSZI_B_TOGG ;
	
   } 
	TCNT2 = 10;							// ergibt 2 kHz fuer Timertakt
}

/*
ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
//		lcd_clr_line(1);
//		lcd_puts("Timer2 Comp\0");
		TCCR20=0;
}
*/



uint8_t  AbschnittLaden(const uint8_t* AbschnittDaten)
{
	uint8_t returnwert=0;
	/*			
	 Reihenfolge der Daten:
    0    schritteax lb
    1    schritteax hb
    2    schritteay lb
    3    schritteay hb
    
    4    delayax lb
    5    delayax hb
    6    delayay lb
    7    delayay hb

    8    schrittebx lb
    9    schrittebx hb
    10    schritteby lb
    11    schritteby hb
    
    12    delaybx lb
    13    delaybx hb
    14    delayby lb
    15    delayby hb

    
    16   (8)    code
    17   (9)    position // Beschreibung der Lage im Schnittpolygon:first, last, ...
    18   (10)   indexh     // Nummer des Abschnitts
    19   (11)   indexl   
    
	 */			
	int lage = 0;
//   lage = AbschnittDaten[9]; // Start: 1, innerhalb: 0, Ende: 2
   lage = AbschnittDaten[17]; // Start: 1, innerhalb: 0, Ende: 2
	if (lage & 0x01)
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   richtung=0;
   
	// Motor A
	STEPPERPORT_1 &= ~(1<<MA_EN); // Pololu ON
	CounterA=0;
	uint8_t dataL=0;
	uint8_t dataH=0;
	
	uint8_t delayL=0;
	uint8_t delayH=0;
	
	dataL=AbschnittDaten[0];
	dataH=AbschnittDaten[1];
	
	//lcd_gotoxy(17,0);
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_A); // Rueckwarts
		STEPPERPORT_1 &= ~(1<< MA_RI); // PIN fuer Treiber stellen
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_A);
		STEPPERPORT_1 |= (1<< MA_RI);
		//lcd_putc('v');	// Vorwaerts
	}
	
	dataH &= (0x7F);
	StepCounterA= dataH;		// HByte
	StepCounterA <<= 8;		// shift 8
	StepCounterA += dataL;	// +LByte
	
	delayL=AbschnittDaten[4];
	delayH=AbschnittDaten[5];
	
	
	DelayA = delayH;
	DelayA <<=8;
	DelayA += delayL;
	
	// Motor B
	CounterB=0;
	STEPPERPORT_1 &= ~(1<<MB_EN);	// Pololu ON
	dataL=AbschnittDaten[2];
	dataH=AbschnittDaten[3];
	//lcd_gotoxy(19,1);
   
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_B); // Rueckwarts
		STEPPERPORT_1 &= ~(1<< MB_RI);
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_B);
		STEPPERPORT_1 |= (1<< MB_RI);
		//lcd_putc('v');
	}
	
	dataH &= (0x7F);
	StepCounterB= dataH;		// HByte
	StepCounterB <<= 8;		// shift 8
	StepCounterB += dataL;	// +LByte
	
	DelayB = (AbschnittDaten[7]<<8)+ AbschnittDaten[6];
   
   
	// Motor C
	STEPPERPORT_2 &= ~(1<<MC_EN); // Pololu ON
	CounterC=0;
	dataL=0;
	dataH=0;
	
	delayL=0;
	delayH=0;
	
	dataL=AbschnittDaten[8];
	dataH=AbschnittDaten[9];
   /*
   lcd_gotoxy(0,1);
   lcd_puthex(dataL);
   lcd_puthex(dataH);
    */

	//richtung=0;
	//lcd_gotoxy(18,0);
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_C); // Rueckwarts
		STEPPERPORT_2 &= ~(1<< MC_RI);
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_C);
		STEPPERPORT_2 |= (1<< MC_RI);
		//lcd_putc('v');	// Vorwaerts
	}
	
	dataH &= (0x7F);
	StepCounterC= dataH;		// HByte
	StepCounterC <<= 8;		// shift 8
	StepCounterC += dataL;	// +LByte
   
	
	delayL=AbschnittDaten[12];
	delayH=AbschnittDaten[13];
   /*
   lcd_gotoxy(5,1);
   lcd_puthex(delayL);
   lcd_puthex(delayH);
   */
	DelayC = delayH;
	DelayC <<=8;
	DelayC += delayL;
   
   
   // Motor D
	STEPPERPORT_2 &= ~(1<<MD_EN); // Pololu ON
	CounterD=0;
	dataL=0;
	dataH=0;
	
	delayL=0;
	delayH=0;
	
	dataL=AbschnittDaten[10];
	dataH=AbschnittDaten[11];
   /*
   lcd_gotoxy(10,1);
   lcd_puthex(dataL);
   lcd_puthex(dataH);
    */
	//lcd_gotoxy(18,0);
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_D); // Rueckwarts
		STEPPERPORT_2 &= ~(1<< MD_RI);
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_D);
		STEPPERPORT_2 |= (1<< MD_RI);
		//lcd_putc('v');	// Vorwaerts
	}
	
	dataH &= (0x7F);
	StepCounterD= dataH;		// HByte
	StepCounterD <<= 8;		// shift 8
	StepCounterD += dataL;	// +LByte
	
	delayL=AbschnittDaten[14];
	delayH=AbschnittDaten[15];
   /*
   lcd_gotoxy(15,1);
   lcd_puthex(delayL);
   lcd_puthex(delayH);
   */
	DelayD = delayH;
	DelayD <<=8;
	DelayD += delayL;
   
   
   //lcd_gotoxy(19,0);
   //lcd_putc(' ');
   //lcd_gotoxy(19,0);
    
   
   
   motorstatus=0;
   
   /*
   if (StepCounterA > StepCounterB) 
   {
      motorstatus |= (1<<COUNT_A);
      //lcd_putc('A');
   }
   else 
   {
      motorstatus |= (1<<COUNT_B);
      //lcd_putc('B');
   }
    */
   
   // Test 16.12.11
   //motorstatus |= (1<<COUNT_A);
   
   //return returnwert;
   
   if (StepCounterA > StepCounterB) 
   {
      if (StepCounterA > StepCounterC)
      {
         if (StepCounterA > StepCounterD) // A max
         {
            motorstatus |= (1<<COUNT_A);
            //lcd_putc('A');
         }
         else //A>B A>C D>A
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
         
      }//A>C
      else // A>B A<C: A weg, B weg
      {
         if (StepCounterC > StepCounterD)
         {
            motorstatus |= (1<<COUNT_C);
            //lcd_putc('C');
         }
         else // A>B A<C D>C B weg, 
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
         
         
      }
   }// A>B
   
   else // B>A A weg
   {
      if (StepCounterB > StepCounterC) // C weg
      {
         if (StepCounterB > StepCounterD) // D weg
         {
            motorstatus |= (1<<COUNT_B);
            //lcd_putc('B');
         }
         else
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
      }
      else // B<C B weg
      {  
         if (StepCounterC > StepCounterD) // D weg
         {
            motorstatus |= (1<<COUNT_C);
            //lcd_putc('C');
         }
         else // D>C C weg
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
         
      }
   }
   
   return returnwert;
}

void AnschlagVonMotor(const uint8_t motor)
{
   
   if (richtung & (1<<(RICHTUNG_A + motor))) // Richtung ist auf Anschlag A0 zu         
   {
      
      if (!(anschlagstatus &(1<< (END_A0 + motor))))
      {
         cli();
         
         anschlagstatus |= (1<< (END_A0 + motor));      // Bit fuer Anschlag A0+motor setzen
         //lcd_putc('A');
         

         if (cncstatus & (1<<GO_HOME)) // nur eigene Seite abstellen
         {
            
            
            
            
            // Paralleler Schlitten gleichzeitig am Anschlag?
            switch (motor) // Stepperport 1
            {
               case 0:
               {
                  
               }
            
            
            }//switch motor

            //lcd_putc('B');
            sendbuffer[0]=0xB5 + motor;
            
           cncstatus |= (1<<motor);
                
            if (motor<2) // Stepperport 1
            {
               STEPPERPORT_1 |= (1<<(MA_EN + motor));     // Motor 0,1 OFF
               //STEPPERPORT_2 |= (1<<(MA_EN + motor + 2)); // Paralleler Motor 2,3 OFF
               StepCounterA=0;
               StepCounterB=0;
               CounterA=0;
               CounterB=0;
               
           }
            else // Stepperport 2
            {
               STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
               //STEPPERPORT_1 |= (1<<(MA_EN + motor - 2)); // Paralleler Motor 0,1 OFF
               StepCounterC=0;
               StepCounterD=0;
               CounterC=0;
               CounterD=0;
          }
            //cncstatus &= ~(1<<GO_HOME);
         
         }
         else           // beide Seiten abstellen
         {    
            cncstatus=0;
            sendbuffer[0]=0xA5 + motor;
            
            if (motor<2) // Stepperport 1
            {
               STEPPERPORT_1 |= (1<<(MA_EN + motor));     // Motor 0,1 OFF
               STEPPERPORT_2 |= (1<<(MA_EN + motor + 2)); // Paralleler Motor 2,3 OFF
            }
            else // Stepperport 2
            {
               STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
               STEPPERPORT_1 |= (1<<(MA_EN + motor - 2)); // Paralleler Motor 0,1 OFF
            }
            
            // Alles abstellen
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            
            CounterA=0;
            CounterB=0;
            CounterC=0;
            CounterD=0;
            
            ladeposition=0;
            motorstatus=0;
         
         }
         
         sendbuffer[5]=abschnittnummer;
         sendbuffer[6]=ladeposition;
         sendbuffer[7]=cncstatus;
         usb_rawhid_send((void*)sendbuffer, 50);
         sendbuffer[0]=0x00;
         sendbuffer[5]=0x00;
         sendbuffer[6]=0x00;
         sendbuffer[7]=0x00;

         
         //ladeposition=0;
        // motorstatus=0;
         
         richtung &= ~(1<<(RICHTUNG_A + motor));
        
         
         sei();
      } // NOT END_A0
      else
      {
         
      }
      
   }
   else 
   {
      if (!(anschlagstatus &(1<< (END_A0 + motor))))
      
      {
         anschlagstatus &= ~(1<< (END_A0 + motor)); // Bit fuer Anschlag B0 zuruecksetzen
      }
      
   }
   
}


void StepEndVonMotor(const uint8_t motor) // 0 - 3 fuer A  D
{
   STEPPERPORT_1 |= (1<<(MA_EN + motor));					// Motor A... OFF
   
   if (cncstatus & (1<<GO_HOME)) 
   {
      
   }
   else
   {
      
   }
   
   if (motor < 2)
   {
   STEPPERPORT_1 |= (1<<(MA_EN + motor));
   StepCounterA=0;
   StepCounterB=0;
   }
   else
   {
      STEPPERPORT_2 |= (1<<(MA_EN + motor -2)); // MC_EN ist = MA_EN, aber motor ist 3
      StepCounterC=0;
      StepCounterD=0;
      
   }
   
   STEPPERPORT_1 |= (1<<(MA_EN + motor));
   StepCounterA=0;
   StepCounterB=0;
   CounterA=0;
   CounterB=0;


   STEPPERPORT_2 |= (1<<(MA_EN + motor -2));
   StepCounterC=0;
   StepCounterD=0;
   CounterC=0;
   CounterD=0;

   
   if (abschnittnummer==endposition)
   {  
      ringbufferstatus = 0;
      //         anschlagstatus=0;
      anschlagstatus &= ~(1<< (END_A0 + motor)); // Bit fuer Anschlag A0 zuruecksetzen
      motorstatus=0;
      sendbuffer[0]=0xAA + motor;
      sendbuffer[1]=abschnittnummer;
      sendbuffer[5]=abschnittnummer;
      sendbuffer[6]=ladeposition;
      sendbuffer[7]=cncstatus;
      usb_rawhid_send((void*)sendbuffer, 50);
      sendbuffer[0]=0x00;
      sendbuffer[1]=0x00;
      sendbuffer[5]=0x00;
      sendbuffer[6]=0x00;
      sendbuffer[7]=0;
      ladeposition=0;
      
   }
   else 
   { 
      uint8_t aktuellelage=0;
      {
         uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
         aktuelleladeposition &= 0x03;
         // aktuellen Abschnitt laden
         aktuellelage = AbschnittLaden(CNCDaten[aktuelleladeposition]);
         if (aktuellelage==2) // war letzter Abschnitt
         {
            endposition=abschnittnummer; // letzter Abschnitt zu fahren
         }  
         else
         {
            // neuen Abschnitt abrufen
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            sendbuffer[0]=0xA0 + motor;
            usb_rawhid_send((void*)sendbuffer, 50);
            sendbuffer[0]=0x00;
            sendbuffer[5]=0x00;
            sendbuffer[6]=0x00;
            
         }
         
         ladeposition++;
         
      }
      if (aktuellelage==2)
      {
      }
      AbschnittCounter++;
      
   }
   
}


int main (void) 
{
    int8_t r;

   uint16_t count=0;
    
	// set for 16 MHz clock
	CPU_PRESCALE(0);
    
	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;
    
	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	sei();
		
	slaveinit();
		
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	delay_ms(100);
	lcd_cls();
	//lcd_puts("READY\0");
	lcd_puts("V: \0");
	lcd_puts(VERSION);
   lcd_clr_line(1);

	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	//timer0();
	
	//initADC(TASTATURPIN);
	//wdt_enable(WDTO_2S);
	
	uint16_t loopcount0=0;
	uint8_t loopcount1=0;

	
	//delay_ms(800);
	//eeprom_write_byte(&WDT_ErrCount0,0);
	
	uint8_t i=0;
   
	//timer2
	TCNT2   = 0; 
//	TCCR2A |= (1 << WGM21); // Configure timer 2 for CTC mode 
	TCCR2B |= (1 << CS20); // Start timer at Fcpu/8 
//	TIMSK2 |= (1 << OCIE2A); // Enable CTC interrupt 
	TIMSK2 |= (1 << TOIE2); // Enable OV interrupt 
	//OCR2A   = 5; // Set CTC compare value with a prescaler of 64 
    TCCR2A = 0x00;

	  sei();
	  
#pragma mark while	  
	while (1)
	{	
      //OSZI_B_LO;
		//Blinkanzeige
		loopcount0+=1;
		if (loopcount0==0xAFFF)
		{
			loopcount0=0;
			loopcount1+=1;
			LOOPLEDPORT ^=(1<<LOOPLED);
         //LOOPLEDPORT ^=(1<<DC);
         
		}
		
       /**	Begin USB-routinen	***********************/
      
      // Start USB
      //lcd_putc('u');
      r = usb_rawhid_recv((void*)buffer, 0);
		if (r > 0) 
      {
         //OSZI_B_HI;
         cli(); 
         // code abfragen
         uint8_t code = 0x00;
         code = buffer[16];
         
         // Empfang quittieren
         sendbuffer[5]=code;
         //sendbuffer[6]=code;
         sendbuffer[0]=0x33;
         usb_rawhid_send((void*)sendbuffer, 50);
         sendbuffer[0]=0x00;
         sendbuffer[5]=0x00;
         sendbuffer[6]=0x00;
         
         switch (code)
         {
            case 0xE0: // Man: Alles stoppen
            {
               ringbufferstatus = 0;
               motorstatus=0;
               sendbuffer[0]=0xE1;
               sendbuffer[5]=abschnittnummer;
               sendbuffer[6]=ladeposition;
               usb_rawhid_send((void*)sendbuffer, 50);
               sendbuffer[0]=0x00;
               sendbuffer[5]=0x00;
               sendbuffer[6]=0x00;
               ladeposition=0;
               AbschnittCounter=0;
               
               StepCounterA=0;
               StepCounterB=0;
               StepCounterC=0;
               StepCounterD=0;
               
               CounterA=0;
               CounterB=0;
               CounterC=0;
               CounterD=0;
               
            }break;
            
               
            case 0xE2: // DC ON_OFF
            {
               PWM = buffer[20];
               if (PWM==0)
               {
                  CMD_PORT &= ~(1<<DC);
               }
               
               //sendbuffer[0]=0xE3;
               //usb_rawhid_send((void*)sendbuffer, 50);
               //sendbuffer[0]=0x00;
               //sendbuffer[5]=0x00;
               //sendbuffer[6]=0x00;
               
            }break;
            
               
            case 0xE4: // Stepperstrom ON_OFF
            {
               if (buffer[8])
               {
                  CMD_PORT |= (1<<STROM); // ON
               }
               else
               {
                  CMD_PORT &= ~(1<<STROM); // OFF
               }
               sendbuffer[0]=0xE5;
               //usb_rawhid_send((void*)sendbuffer, 50);
               sendbuffer[0]=0x00;
               sendbuffer[5]=0x00;
               sendbuffer[6]=0x00;
               
            }break;
              
            default:
            {  
               
               uint8_t indexh=buffer[18];
               uint8_t indexl=buffer[19];
               abschnittnummer= indexh<<8;
               abschnittnummer += indexl;
               PWM = buffer[20];
               
               
               if (abschnittnummer==0) // neue Datenreihe
               {
                  uint8_t i=0, k=0;
                  for (k=0;k<RINGBUFFERTIEFE;k++)
                  {
                     for(i=0;i<USB_DATENBREITE;i++)
                     {
                        CNCDaten[k][i]=0;  
                     }
                  }
                  
                  ladeposition=0;
                  endposition=0xFFFF;
                  anschlagstatus=0;
                  motorstatus=0;
                  ringbufferstatus=0x00;
                  ringbufferstatus |= (1<<FIRSTBIT);
                  AbschnittCounter=0;
                  lcd_clr_line(1);
                  sendbuffer[5]=0x00;
                  //sendbuffer[6]=code;
                  
                  
                  if (code == 0xF0) // cncstatus fuer go_home setzen
                  {
                     sendbuffer[5]=0xF0;
                     sendbuffer[0]=0x45;
                     cncstatus |= (1<<GO_HOME); // Bit fuer go_home setzen
                  }
                  else
                  {
                     sendbuffer[0]=0x44;
                     cncstatus &= ~(1<<GO_HOME); // Bit fuer go_home zuruecksetzen
                  }
                  
                  usb_rawhid_send((void*)sendbuffer, 50);
                  sendbuffer[0]=0x00;
                  sendbuffer[5]=0x00;
                  sendbuffer[6]=0x00;
                  
               }
               else
               {
                  
               }
               
               
               
               
               
               
               if (buffer[17]& 0x02)// letzter Abschnitt, Bit 1
               {
                  ringbufferstatus |= (1<<LASTBIT);
                  if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
                  {
                     endposition=abschnittnummer; // erster ist auch letzter Abschnitt
                  }
               }
               
               // Daten vom buffer in CNCDaten laden
               {
                  uint8_t pos=(abschnittnummer);
                  pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe, wert 0,1 
                  if (abschnittnummer>8)
                  {
                     //lcd_putint1(pos);
                  }
                  uint8_t i=0;
                  for(i=0;i<USB_DATENBREITE;i++)
                  {
                     CNCDaten[pos][i]=buffer[i];  
                  }
                  
               }
               
               
               // Erster Abschnitt, mehr als ein Abschnitt, naechsten Abschnitt laden
               if ((abschnittnummer == 0)&&(endposition))
               {
                  {
                     //lcd_putc('*');
                     sendbuffer[5]=abschnittnummer;
                     sendbuffer[6]=ladeposition;
                     sendbuffer[0]=0xAF;
                     usb_rawhid_send((void*)sendbuffer, 50);
                     sendbuffer[0]=0x00;
                     sendbuffer[5]=0x00;
                     sendbuffer[6]=0x00;
                     
                     
                  }  
               }
               
               ringbufferstatus &= ~(1<<FIRSTBIT);
               
               // Ringbuffer ist voll oder  letzter Abschnitt schon erreicht
               if ((abschnittnummer ==1 )||((abschnittnummer ==0 )&&(ringbufferstatus & (1<<LASTBIT)))) 
               {
                  {
                     ringbufferstatus &= ~(1<<LASTBIT);
                     //Beginnen
                     //lcd_putc('s');
                     ringbufferstatus |= (1<<STARTBIT);
                     
                  }
               }
            }  
         } // switch code
         
         
         code=0;
         sei();
		} // end r>0, neue Daten
      
      
      
      /**	End USB-routinen	***********************/
     
      /**	HOT	***********************/
      
      if (PWM) // Draht soll heiss sein
      {
         if (pwmposition > PWM) // > DC OFF, PIN ist LO
         {
            CMD_PORT &= ~(1<<DC);
            OSZI_A_HI ;
         }
         else                    // > DC ON, PIN ist HI
         {
            CMD_PORT |= (1<<DC);
            OSZI_A_LO ;
            
         }
      }

      /**	Start CNC-routinen	***********************/
      
      if (ringbufferstatus & (1<<STARTBIT)) // Buffer ist geladen, Abschnitt 0 laden
      {
         cli();
         //lcd_putc('g'); // los
         ringbufferstatus &= ~(1<<STARTBIT);         
         ladeposition=0;
         AbschnittCounter=0;
         
         // Ersten Abschnitt laden
         uint8_t pos=AbschnittLaden((void*)CNCDaten[0]); 
         ladeposition++;
         if (pos==2) // nur ein Abschnitt
         {
            ringbufferstatus |=(1<<ENDBIT);
         }
         
         AbschnittCounter+=1;
         sei();
      } // end ringbufferstatus & (1<<STARTBIT)

        
       // ********************
      // * Anschlag Motor A *
      // ********************

      if ((STEPPERPIN_1 & (1<< END_A0_PIN)) ) // Eingang ist HI, Schlitten nicht am Anschlag A0
      {
         if (anschlagstatus &(1<< END_A0))
         {
            anschlagstatus &= ~(1<< END_A0); // Bit fuer Anschlag A0 zuruecksetzen
         }
      }
      else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A0
      {
         AnschlagVonMotor(0);
       // 
   //
      }
      
      // **************************************
      // * Motor A *
      // **************************************
      // Es hat noch Steps, CounterA ist abgezaehlt (Ende des aktuellen Impulses, DelayA bestimmt Impulsabstand fuer Steps)
 		
      if (StepCounterA && (CounterA >= DelayA) &&(!((anschlagstatus & (1<< END_A0)))))
		{
         cli();
         //lcd_putc('A');
         
			STEPPERPORT_1 &= ~(1<<MA_STEP);					// Impuls an Motor A LO ON
			CounterA=0;
			StepCounterA--;
         
			if (StepCounterA ==0 && (motorstatus & (1<< COUNT_A))) // Motor A ist relevant fuer Stepcount 
			{
				
            StepEndVonMotor(0);
         }
      
			sei();
		}
		else// if (CounterA)
		{
			STEPPERPORT_1 |= (1<<MA_STEP);
			if (StepCounterA ==0)							// Keine Steps mehr fuer Motor A
			{
 				STEPPERPORT_1 |= (1<<MA_EN);					// Motor A OFF
            
			}
			
		}
    
      /*     
       // Halt-Pin
       else if (!(richtung & (1<<RICHTUNG_A)))
       {
       DelayA = 0;
       StepCounterA = 0;
       STEPPERPORT_1 |= (1<<MA_STEP);     // StepCounterA beenden
       STEPPERPORT_1 |= (1<<MA_EN);
       CounterA=0;
       
       }
       */
      // ***************
      // * End Motor A *
      // ***************

      // **************************************
      // * Anschlag Motor B *
      // **************************************
      // Anschlag B0
      if ((STEPPERPIN_1 & (1<< END_B0_PIN)) ) // Schlitten nicht am Anschlag B0
      {
         if (anschlagstatus &(1<< END_B0))
         {
            anschlagstatus &= ~(1<< END_B0); // Bit fuer Anschlag B0 zuruecksetzen
         }
      }
      else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag B0
      {
         AnschlagVonMotor(1);
      //
         
      } // end Anschlag B0

      // End Anschlag B
      
      // **************************************
      // * Motor B *
      // **************************************

		if (StepCounterB && (CounterB >= DelayB) &&(!((anschlagstatus & (1<< END_B0)))))
		{
         cli();
         //lcd_putc('B');
         
			STEPPERPORT_1 &= ~(1<<MB_STEP);                         // Impuls an Motor B LO ON
			CounterB=0;
			StepCounterB--;
         
			if (StepCounterB ==0 && (motorstatus & (1<< COUNT_B))) // Motor B ist relevant fuer Stepcount 
			{
				StepEndVonMotor(1);
         }
			
			
			sei();
		}
		else  // if (CounterB)
		{
			STEPPERPORT_1 |= (1<<MB_STEP);
			if (StepCounterB ==0)							// Keine Steps mehr fuer Motor B
			{
				STEPPERPORT_1 |= (1<<MB_EN);					// Motor B OFF
            
			}
			
		}
      
      // ***************
      // * End Motor B *
      // ***************

      
      // ********************
      // * Anschlag Motor C *
      // ********************
      
      // Anschlag C0
      if ((STEPPERPIN_2 & (1<< END_C0_PIN)) ) // Eingang ist HI, Schlitten nicht am Anschlag C0
      {
         if (anschlagstatus &(1<< END_C0))
         {
            anschlagstatus &= ~(1<< END_C0); // Bit fuer Anschlag C0 zuruecksetzen
         }
      
      }
      else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag C0
      {
       AnschlagVonMotor(2);
      }
     
      // **************************************
      // * Motor C *
      // **************************************
		
      if (StepCounterC && (CounterC >= DelayC) &&(!((anschlagstatus & (1<< END_C0)))))
		{
         cli();
         //lcd_putc('C');
         
			STEPPERPORT_2 &= ~(1<<MC_STEP);					// Impuls an Motor C LO ON
			CounterC=0;
			StepCounterC--;
         
			if (StepCounterC ==0 && (motorstatus & (1<< COUNT_C))) // Motor C ist relevant fuer Stepcount 
			{
            
            StepEndVonMotor(2);
         }
			sei();
		}
		else// if (CounterC)
		{
			STEPPERPORT_2 |= (1<<MC_STEP);
			if (StepCounterC ==0)							// Keine Steps mehr fuer Motor C
			{
 				STEPPERPORT_2 |= (1<<MC_EN);					// Motor C OFF
            
			}
			
		}
      
      // ***************
      // * End Motor C *
      // ***************
      
      
      // ***************
      // * Anschlag Motor D *
      // ***************
      
      // Anschlag D0
      if ((STEPPERPIN_2 & (1<< END_D0_PIN)) ) // Schlitten nicht am Anschlag D0
      {
         if (anschlagstatus &(1<< END_D0))
         {
            anschlagstatus &= ~(1<< END_D0); // Bit fuer Anschlag D0 zuruecksetzen
         }
      }
      else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag D0
      {
         AnschlagVonMotor(3);
      }
      

      // **************************************
      // * Motor D *
      // **************************************
      if (StepCounterD && (CounterD >= DelayD) &&(!((anschlagstatus & (1<< END_D0)))))
		{
         cli();
         //lcd_putc('D');
         
			STEPPERPORT_2 &= ~(1<<MD_STEP);					// Impuls an Motor D LO ON
			CounterD=0;
			StepCounterD--;
         
			if (StepCounterD ==0 && (motorstatus & (1<< COUNT_D))) // Motor D ist relevant fuer Stepcount 
			{
				StepEndVonMotor(3);
          }
			sei();
		}
		else// if (CounterB)
		{
			STEPPERPORT_2 |= (1<<MD_STEP);
			if (StepCounterD ==0)							// Keine Steps mehr fuer Motor D
			{
				STEPPERPORT_2 |= (1<<MD_EN);					// Motor D OFF
            
			}
			
		}

      
      // ***************
      // * End Motor D *
      // ***************
      
		sei(); 
      
		
      
		/**	Ende CNC-routinen	***********************/
		
		
		/* **** rx_buffer abfragen **************** */
		//rxdata=0;
		
#pragma mark rxdata		
		//	Daten von USB vorhanden
      // rxdata
		
		//lcd_gotoxy(16,0);
      //lcd_putint(StepCounterA & 0x00FF);
		
		if (!(TASTENPIN & (1<<TASTE0))) // Taste 0
		{
			//lcd_gotoxy(8,1);
			//lcd_puts("T0 Down\0");
			
			if (!(TastenStatus & (1<<TASTE0))) //Taste 0 war noch nicht gedrueckt
			{
				//RingD2(5);
				TastenStatus |= (1<<TASTE0);
				
				Tastencount=0;
				//lcd_gotoxy(0,1);
				//lcd_puts("P0 \0");
				//lcd_putint(TastenStatus);
				//delay_ms(800);
			}
			else
			{
				
				
				Tastencount +=1;
				//lcd_gotoxy(7,1);
				//lcd_puts("TC \0");
				//lcd_putint(Tastencount);
				
				if (Tastencount >= Tastenprellen)
				{
               
					Tastencount=0;
               if (TastenStatus & (1<<TASTE0))
               {
                  //sendbuffer[0]=loopcount1;
                  //sendbuffer[1]=0xAB;
                  //usbstatus |= (1<<USB_SEND);
                  //lcd_gotoxy(2,1);
                  //lcd_putc('1');
                  
                  //usb_rawhid_send((void*)sendbuffer, 50);
               }
					TastenStatus &= ~(1<<TASTE0);
               //lcd_gotoxy(3,1);
               //lcd_puts("ON \0");
               //delay_ms(400);
               //lcd_gotoxy(3,1);
               // lcd_puts("  \0");
               //lcd_putint(TastenStatus);
               
               
 //					anschlagstatus |= (1<<TASTE0); // anschlagstatus &= ~(1<<TASTE0); ??
				}
			}//else
			
		}	// Taste 0
		
      
		
		if (!(TASTENPIN & (1<<TASTE1))) // Taste 1
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("T1 Down\0");
			
			if (! (TastenStatus & (1<<TASTE1))) //Taste 1 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<TASTE1);
				Tastencount=0;
				//lcd_gotoxy(3,1);
				//lcd_puts("P1 \0");
				//lcd_putint(Servoimpulsdauer);
				//delay_ms(800);
				
			}
			else
			{
				//lcd_gotoxy(3,1);
				//lcd_puts("       \0");
				
				Tastencount +=1;
				if (Tastencount >= Tastenprellen)
				{
					
					
					Tastencount=0;
					TastenStatus &= ~(1<<TASTE1);
					
				}
			}//	else
			
		} // Taste 1
		
		/* ******************** */
		//		initADC(TASTATURPIN);
		//		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
		
		Tastenwert=0;
		
		//lcd_gotoxy(3,1);
		//lcd_putint(Tastenwert);
      
		//OSZI_B_HI;
      if (usbstatus & (1<< USB_SEND))
      {
         //lcd_gotoxy(10,1);
         //lcd_puthex(AbschnittCounter);
         //sendbuffer[3]= AbschnittCounter;
         //usb_rawhid_send((void*)sendbuffer, 50);
         //sendbuffer[0]=0;
         //sendbuffer[5]=0;
         //sendbuffer[6]=0;
         //usbstatus &= ~(1<< USB_SEND);
         
      }
      
	}//while
   //free (sendbuffer);

// return 0;
}
