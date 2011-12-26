/*
 *  web_SPI.c
 *  WebServer
 *
 *  Created by Sysadmin on 11.Oktober.09.
 *  Copyright 2009 Ruedi Heimlicher. All rights reserved.
 *
 */

#include <avr/io.h>

/*
#define SPI_CONTROL_DDR		DDRB
#define SPI_CONTROL_PORT	PORTB
#define SPI_CONTROL_HC		PORTB1	//CS fuer HomeCentral Master
#define SPI_CONTROL_SO		PORTB4
#define SPI_CONTROL_SI		PORTB3
#define SPI_CONTROL_SCK		PORTB5
*/
#define CS_HC_PASSIVE		SPI_CONTROL_PORT |= (1<< SPI_CONTROL_HC)	// CS fuer HC ist HI
#define CS_HC_ACTIVE			SPI_CONTROL_PORT &= ~(1<< SPI_CONTROL_HC)	// CS fuer HC ist LO 

void InitSPI(void) 
{ 
	DDRB = (1<<PORTB4)|(1<<PORTB5) | (1<<PORTB7);    // Set MOSI , SCK , and SS output 
	SPCR = ((1<<SPE)|(1<<MSTR) | (1<<SPR1) |(1<<SPR0));   // Enable SPI, Master, set clock rate fck/128  
} 




void WriteByteSPI(unsigned char byte) 
{ 
	CS_HC_ACTIVE; 
	SPDR = byte;						//Load byte to Data register 
	while(!(SPSR & (1<<SPIF)));	// Wait for transmission complete 
	CS_HC_PASSIVE;
} 

char ReadByteSPI(char addr) 
{ 
	CS_HC_ACTIVE;
   SPDR = addr;               //Load byte to Data register 
	//waitspi();
   while(!(SPSR & (1<<SPIF)));    // Wait for transmission complete 
   addr=SPDR; 
	CS_HC_PASSIVE;
   return addr; 
} 
