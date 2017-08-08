/*
 * UART.c
 *
 * Created: 24.08.2016 14:48:59
 *  Author: Mazinov
 */ 
 #include "UART.h"
 #include <util/delay.h>		//	дл€ _delay_ms()
 #include <avr/io.h>
 #include <avr/pgmspace.h>
 #include <stdio.h>

 int init_UART(void)
{
	//	Speed is 9600
	UBRRH=0;	//	UBRR=f/(16*band)-1 f=8000000√ц band=9600,
	UBRRL=51;	//	Normal asynchronous bi-directional operation;
	
	//			RXC			-	recieve end;
	//			|TXC		-	transmit end
	//			||UDRE 		-	lack of data to send
	//			|||FE		-	frame error
	//			||||DOR		-	buffer overflow error
	//			|||||PE		-	parity error
	//			||||||U2X	-	double speed
	//			|||||||MPCM	-	multiprocessor mode
	//			76543210
	UCSRA=0b00000000;

	//			RXCIE		-	Interrupt when data is received
	//			|TXCIE		-	Interrupt at the end of transmission
	//			||UDRIE		-	interruption of the absence of data to send
	//			|||RXEN		-	receiving permission
	//			||||TXEN	-	transmit permission
	//			|||||UCSZ2	-	UCSZ0:2 size of data frames
	//			||||||RXB8	-	9 bits received data 
	//			|||||||TXB8	-	9 bits of the transmitted data
	//			76543210
	UCSRB=0b00011000;	// Allowed reception and transmission via UART;

	//			URSEL		-	always 1
	//			|UMSEL		-	mode:1-synchronous 0-asynchronous
	//			||UPM1		-	UPM0:1 parity
	//			|||UPM0		-	UPM0:1 parity
	//			||||USBS	-	top bits: 0-1, 1-2
	//			|||||UCSZ1	-	UCSZ0:2 data frame size
	//			||||||UCSZ0	-	UCSZ0:2 data frame size
	//			|||||||UCPOL-	in synchronous mode - clocking
	//			76543210
	UCSRC=0b10000110;	//	8-bit package;
}
//	UART
void send_Uart(unsigned char uart_data)      //	Send a byte;
{
	while(!(UCSRA&(DATA_REGISTER_EMPTY)))	 // Set when the register is free;
	{}
	UDR = uart_data;
}

void send_Uart_str(unsigned char *s) //	Send string;
{
	while (*s != 0) send_Uart(*s++);
}

void send_int_Uart(unsigned int c)   //	Send number from 0000 to 9999;
{
	unsigned char temp;
	c=c%10000;
	temp=c/100;
	send_Uart(temp/10+'0');
	send_Uart(temp%10+'0');
	temp=c%100;
	send_Uart(temp/10+'0');
	send_Uart(temp%10+'0');
}

unsigned char getch_Uart(void)		 //	Byte recieve;
{
	while(!(UCSRA&(RX_COMPLETE)))	 // Set when the register is free
	{}
	return UDR;
}
unsigned char get_Uart_str(void)    //	Send a string;
{
	unsigned char *str;
	while (*str != 0) { *str = getch_Uart(); }
	return str;
}
/*
unsigned char get_String (void)
{
	const char SIZE = 16;
	char str[SIZE];
	int i=0;
	char tmpCh;
	while((tmpCh=getch_Uart())!=NEW_LINE)
	{
		str[i++]=tmpCh;
		str[i]=0;
		send_Uart(getch_Uart());
	}
	return str;
}
*/