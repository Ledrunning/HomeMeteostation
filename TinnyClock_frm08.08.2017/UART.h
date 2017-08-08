/*
 * UART.h
 *
 * Created: 24.08.2016 14:48:42
 *  Author: Mazinov
 */ 


#ifndef UART_H_
#define UART_H_

#define NEW_LINE 13
#define RX_BUFFER   128    // Length of our buffer; (String length);
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)
#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<DOR)

int init_UART(void);
void send_Uart(unsigned char c);
void send_Uart_str(unsigned char *s);
void send_int_Uart(unsigned int c);
unsigned char getch_Uart(void);
void get_String(void);

#endif /* UART_H_ */