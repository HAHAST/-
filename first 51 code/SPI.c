#include "STC8.h"

#define SS P00

unsigned char SPI_RW(unsigned char byte){
	SPCTL  = 0x40;
	SPSTAT = 0xC0;
	
	SPDAT = byte;
	while (!(SPSTAT & 0x80));
	SPSTAT = 0xC0;
	
	return SPDAT;
}