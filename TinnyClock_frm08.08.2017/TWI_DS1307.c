/*
 * TWI_DS1307.c
 *
 * Created: 26.08.2016 12:58:37
 *  Author: Mazinov
 */ 
 #include <avr/io.h>
 #include <util/delay.h>
 #include "TWI_DS1307.h"

 // Функция инициализация шины TWI

 void I2CInit(void)
 {
	 // настройка TWI модуля
	 TWBR = 2;
	 TWSR = (1 << TWPS1)|(1 << TWPS0); // Prescaller 64;
	 TWCR |= (1 << TWEN);			   // Switch on TWI;
 }

 void I2CStart(void)
 {
	 // Transfer START condition;
	 TWCR = (1 << TWINT)|(1 << TWEN)|(1 << TWSTA);
	 // Flag installation waiting TWINT;
	 while(!(TWCR & (1 << TWINT)));
 }

 void I2CStop(void)
 {
	 TWCR = (1 << TWINT)|(1 << TWEN)|(1 << TWSTO); // Transfer STOP condition;
	 while(TWCR & (1 << TWSTO)); // Waiting for completion of transmission of the STOP condition;
 }

 // Data recording function on the bus;
 uint8_t I2CWriteByte(uint8_t data)
 {
	 TWDR = data; // Load data in TWDR;
	 TWCR = (1 << TWEN)|(1 << TWINT); // Reset TWINT flag to start data;
	 while(!(TWCR & (1 << TWINT))); // Waiting for the completion of transfer;
	 // Status check;
	 if((TWSR & 0xF8) == 0x18 || (TWSR & 0xF8) == 0x28 || (TWSR & 0xF8) == 0x40)
	 {
		// If the address of the DS1307, the bits R / W and data transferred
		// And received confirmation;
		 return 1;
	 }
	 else
	 return 0; // Error;
 }

 // Read data bus function;
 uint8_t I2CReadByte(uint8_t *data,uint8_t ack)
 {
	 if(ack) // Set confirmation;
	 {
		// Return a confirmation after receiving;
		 TWCR |= (1 << TWEA);
	 }
	 else
	 {
		// Return the non-confirmation after receiving;
		// The slave doesn't receive more data;
		// Recognizers typically used for the last byte;
		 TWCR &= ~(1 << TWEA);
	 }
	// Receiving data after a reset TWINT;
	 TWCR |= (1 << TWINT);
	 while(!(TWCR & (1 << TWINT))); // Wait for the installation TWINT flag;
	 // Status check;
	 if((TWSR & 0xF8) == 0x58 || (TWSR & 0xF8) == 0x50)
	 {		 
		 // Receive data, and return confirmation
		 // or
		 // Receive data, and return not confirmed
		 *data = TWDR; // Read data;
		 return 1;
	 }
	 else
	 return 0; // Error;
 }
 // Read data from DS1307
 uint8_t DS1307Read(uint8_t address,uint8_t *data)
 {
	 uint8_t res; // Result;
	 I2CStart();  // Start;
	 res = I2CWriteByte(0b11010000);	// Address DS1307 + bit W;
	 if(!res)	return 0; // Error
	 // Transfer the required register address;
	 res = I2CWriteByte(address);
	 if(!res)	return 0; // Error;
	 I2CStart(); // Start repeat;
	 res = I2CWriteByte(0b11010001);	// Address DS1307 + bit R;
	 if(!res)	return 0; // Error
	 // Reading data from non-confirmation
	 res = I2CReadByte(data,0);
	 if(!res)	return 0; // Error;
	 I2CStop(); // Stop;
	 return 1;
 }
 // Write data function on DS1307;
 uint8_t DS1307Write(uint8_t address,uint8_t data)
 {
	 uint8_t res;   // Result;
	 I2CStart();	// Start;
	 res = I2CWriteByte(0b11010000);	// Address DS1307 + bit W;
	 if(!res)	return 0; // Error;
	 // Transfer the required register address;
	 res = I2CWriteByte(address);
	 if(!res)	return 0; // Error;
	 res = I2CWriteByte(data); // Write data;
	 if(!res)	return 0; // Error;
	 I2CStop(); // Stop;
	 return 1;
 }

 // DS1307 Real Time Clock initialization
 // Square wave output on pin SQW/OUT: On
 // Square wave frequency: 1Hz
 void rtc_init(unsigned char rs,unsigned char sqwe,unsigned char out)
 {
	 rs&=3;
	 if (sqwe) rs|=0x10;
	 if (out) rs|=0x80;
	 I2CStart();
	 I2CWriteByte(0xd0);
	 I2CWriteByte(7);
	 I2CWriteByte(rs);
	 I2CStop();
 }