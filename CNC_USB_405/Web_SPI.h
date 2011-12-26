/*
 *  Web_SPI.h
 *  WebServer
 *
 *  Created by Sysadmin on 11.Oktober.09.
 *  Copyright 2009 Ruedi Heimlicher. All rights reserved.
 *
 */
 

 
 
void InitSPI(void);
void WriteByteSPI(unsigned char byte);
char ReadByteSPI(char addr);