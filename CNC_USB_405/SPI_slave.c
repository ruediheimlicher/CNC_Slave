//***************************************************************************
//**************************************************************************
#include <avr/io.h>
//#include "inavr.h"
#include "lcd.h"

#define  Error   0x01
#define  Success 0x02
#define SLAVE_COMMAND 2
char     TransmitState = 0x00;
//char*    TextString    = "AVR communicating via the SPI"+0x00;
#define SPI_BUFSIZE 32

#define spi_buffer_size	16
volatile uint8_t								spi_slavebuffer[SPI_BUFSIZE];
volatile uint8_t								spi_slavestartdaten;

extern volatile uint8_t timer0startwert;

extern  volatile uint8_t spistatus;
static volatile uint8_t						ByteCounter=0xFF;

extern volatile uint8_t						SPI_Daten[];
extern volatile uint8_t						SPI_StartDaten;
extern volatile uint8_t						inbuffer[];

volatile short int received=0;

static volatile uint8_t						bitpos=0xFF;
static volatile uint8_t						startbitpos=0;
#define OSZIPORT	PORTC
#define OSZIPORTDDR	DDRC
#define OSZIPORTPIN	PINC
#define PULS	1

#define OSZILO OSZIPORT &= ~(1<<PULS)
#define OSZIHI OSZIPORT |= (1<<PULS)

/*
#define SPI_CONTROL_DDR			DDRD
#define SPI_CONTROL_PORT		PORTD
#define SPI_CONTROL_PORTPIN	PIND
#define SPI_CONTROL_CS_HC		PORTD1	// CS fuer HomeCentral Master

#define SPI_CONTROL_MOSI		PORTD3	// Eingang fuer Daten vom Master
#define SPI_CONTROL_MISO		PORTD4	// Ausgang fuer Daten zum Master

#define SPI_INT0_DDR				DDRD
#define SPI_INT0_PORT			PORTD

#define SPI_CONTROL_SCK			PORTD2		// INT0 als Eingang fuer CLK
*/
// MAkros:
#define IS_CS_HC_ACTIVE				SPI_CONTROL_PORTPIN & ~(1<< SPI_CONTROL_CS_HC)	// ist CS fuer HC  LO?



#define SPI_PULSE_DELAY			50 // delay in ns
//#define DATENBREITE				8	// Anzahl Bytes in einer Serie


/*

void InitSPI_SLAVE(void) 
{ 

	SPI_CONTROL_DDR   |= (1<<SPI_CONTROL_MISO);		// MISO als Output
	SPI_CONTROL_PORT	|= (1<<SPI_CONTROL_MISO);		// HI
	SPI_CONTROL_DDR	&= ~(1<<SPI_CONTROL_CS_HC);	// Chip Select als Eingang
	SPI_CONTROL_PORT	|= (1<<SPI_CONTROL_CS_HC);		// HI

	SPI_CONTROL_DDR	&= ~(1<<SPI_CONTROL_MOSI);		// MOSI als Eingang
	SPI_CONTROL_PORT	|= (1<<SPI_CONTROL_MOSI);		// HI
	
	SPI_INT0_DDR&= ~(1<<SPI_CONTROL_SCK);				// INT0 als SCK Eingang
	SPI_INT0_PORT |=(1<<SPI_CONTROL_SCK);				// HI
		
	// interrupt on INT0 pin falling edge (sensor triggered) 
	MCUCR = (1<<ISC01) ;

	// turn on interrupts!
	GICR  |= (1<<INT0);
	
	lcd_gotoxy(0,0);
	lcd_puts("SPI_S Init\0");

	sei(); // Enable global interrupts

} 
*/


// Interrupt Routine Slave Mode (interrupt controlled)
// Aufgerufen bei fallender Flanke an INT0

/*
ISR( INT0_vect )
{
	//OSZILO;
	
	if (spistatus & (1<<0))				// CS ist LO, Interrupt ist OK
	{
		//	OSZILO;
		//	_delay_us(100);
		
		TCNT0=timer0startwert; // Timer0 zuruecksetzen
		// PIN lesen:
		
		if (spistatus & (1<<1))	// StartDaten
		{
		
			if (SPI_CONTROL_PORTPIN & (1<<SPI_CONTROL_MOSI)) // bit ist H
			{
				SPI_StartDaten |= (1<<(7-bitpos));
				
				// Output laden
				//SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
			}
			else
			{
				SPI_StartDaten |= (0<<(7-bitpos));
				//SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
			}
			
			
			// Output laden
			if (spi_slavestartdaten & (1<<(7-bitpos)))
			{
				SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
			}
			else 
			{
				SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
			}

			bitpos++;


			if (bitpos>=8) // Byte fertig
			{
				spistatus &= ~(1<<1); // Bit fuer Startdaten zuruecksetzen
				bitpos=0;
			}
			
		}
		
		else
		{
			if (SPI_CONTROL_PORTPIN & (1<<SPI_CONTROL_MOSI)) // bit ist H
			{
				
				SPI_Daten[ByteCounter] |= (1<<(7-bitpos));
				inbuffer[ByteCounter] |= (1<<(7-bitpos));
				SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
				
			}
			else
			{
				SPI_Daten[ByteCounter] |= (0<<(7-bitpos));
				inbuffer[ByteCounter] |= (0<<(7-bitpos));
				SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
			}
			
			// Output laden
			if (spi_slavebuffer[ByteCounter] & (1<<(7-bitpos)))
			{
				SPI_CONTROL_PORT |= (1<<SPI_CONTROL_MISO);
			}
			else 
			{
				SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_MISO);
			}

			
			bitpos++;
			
			if (bitpos>=8) // Byte fertig
			{
				ByteCounter++;
				bitpos=0;
			}
			
		}
		
		
	}
	
//	OSZIHI;
	
}

*/

