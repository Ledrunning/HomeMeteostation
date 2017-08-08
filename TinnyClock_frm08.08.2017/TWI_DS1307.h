/*
 * TWI_DS1307.h
 *
 * Created: 26.08.2016 12:58:09
 *  Author: Mazinov
 */ 


#ifndef TWI_DS1307_H_
#define TWI_DS1307_H_

// Initialization function TWI bus;
void I2CInit(void);
void I2CStart(void);
void I2CStop(void);
// Data recording function on the bus;
uint8_t I2CWriteByte(uint8_t data);
// Read the data function on the bus;
uint8_t I2CReadByte(uint8_t *data,uint8_t ack);
uint8_t DS1307Read(uint8_t address,uint8_t *data);
// Data recording function in the DS1307;
uint8_t DS1307Write(uint8_t address,uint8_t data);

#endif /* TWI_DS1307_H_ */