#include "main.h"
#include <avr/io.h>
#include <avr/interrupt.h>  
#include <avr/wdt.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdbool.h> // Bool data type;
#include "delay.h"
#include "onewire.h"
#include "ds18x20.h"
#include "UART.h"
#include "lcd_lib.h"
#define FIRST_ADC_INPUT 1
#define LAST_ADC_INPUT 2
#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (0<<ADLAR))
#define UARTLED      (1<<PINA0)
#define UP_LED    (1<<PINA3)
#define DOWN_LED  (1<<PINA4)
#define LEFT_LED  (1<<PINA5)
#define RIGHT_LED (1<<PINA6)
#define OK_LED    (1<<PIND7)
#define UP		  (1<<PIND3)
#define DOWN	  (1<<PIND4)
#define LEFT      (1<<PIND5)
#define RIGHT     (1<<PIND6)
#define BUZZER    (1<<PINC2)

// For printf function in UART;
FILE usart_str = FDEV_SETUP_STREAM(send_Uart, NULL, _FDEV_SETUP_WRITE); 

typedef unsigned char byte;

// Global vars;
unsigned char	nDevices;	// Number of sensors;
unsigned char	ow_devices_IDs[MAXDEVICES][8];	// Devices ID;
unsigned char	themperature[3]; // Will write temperature data in this array;
volatile unsigned int	adcData[LAST_ADC_INPUT-FIRST_ADC_INPUT+1];
volatile bool time_flag=0;
unsigned int	humidity, lightSensor;
char lcd_buf_tSenseOne[16], lcd_buf_tSenseTwo[16];
char humidity_buf[16], light_sensor_buf[16];
char uart_tSenseOne[16], uart_tSenseTwo[16];
float humidity_to_voltage, light_to_voltage;
unsigned char hour=0, minutes=0, seconds=0, day=0;
unsigned char month=0, year=0, week_day=0, a_hour=0, a_min=0;
unsigned char time_buffer, date_buffer;
char menu=0, subProgram=0;
bool ps=0, sub_alarm=0, sub_time=0, ps_set=0, alarm_on=0, sub_date=0;

// Bell symbol;
byte bellChar[8] = { 
	0b10000000,
	0b10000100,
	0b10001110,
	0b10001110,
	0b10001110,
	0b10011111,
	0b10100100,
	0b11000000};

// ADC interrupt service routine;
// with auto input scanning;
ISR(ADC_vect)
{
	static unsigned char input_index = 0;
	// Read the AD conversion result;
	adcData[input_index]=ADCW;
	// Select next ADC input;
	if (++input_index > (LAST_ADC_INPUT-FIRST_ADC_INPUT))
	input_index=0;
	ADMUX=(FIRST_ADC_INPUT | ADC_VREF_TYPE)+input_index;
	// Delay needed for the stabilization of the ADC input voltage;
	_delay_us(10);
	// Start the AD conversion;
	ADCSRA|=(1<<ADSC);
}

// Interrupt from external clock;
ISR(INT0_vect)
{
	 time_flag=1;    
}

// Prototype of functions;
// Atmega16 initialisation;
void mcu_init();
void print_address(unsigned char* address);
// Search all devices on 1Wire bus;
unsigned char search_ow_devices(void);
// Print temperature data; 
void temp_data_print(char* lcd_buf_tSenseOne, char* lcd_buf_tSenseTwo);
// Print humidity and light data on LCD;
void adc_data_toLCD(char* humidity_buf, char* light_sensor_buf);
// Adc inputs reading;
void adc_data_read();
// Temperature point menu;
void show_temperature();
// Humidity and light point menu;
void show_percents();
// Time poit menu;
void show_time();
// Set time point menu;
void set_time();
// Set date point menu;
void set_date();
// Set alarm point menu;
void set_alarm();
// Alarm buzzer sound;
void alarm_sound();
// Sending temperature, light and humidity to USART;
void usart_data_sender();
// Temperature initialization;
void temperature_init();
// Chart print;
void progress(void); 


int main(void)
{
	stdout = &usart_str;		// Point to stdout; 

	// Switch on LCD, delay, I2C and UART and initialize mcu;
	mcu_init();
	lcd_init();
	init_UART(); 
	I2CInit(); 
	timerDelayInit();
	// RTC configuring registers - 1HZ output;
	rtc_init(0,1,0);
	// Download bellChar to LCD memory;
//	lcd_definechar(bellChar, 0x00);

	lcd_gotoxy(4,0);
	lcd_string("-Tinny-",8);
	lcd_gotoxy(4,1);
	lcd_string("-clock-",8);
	PORTA = 0x00;
	timerDelayMs(1000);
	//_delay_ms(3000);
	PORTA = 0xFF;

	lcd_clr(); 
	//progress();
	

	nDevices = search_ow_devices(); // Find all devices;
	
	printf("+--------- Tinny clock version 0.1 ---------+\r");	// Print to Usart;
	printf("+------------ Found %d devices --------------+", nDevices);
	printf("\r");

	temperature_init();
	
	// Start the clock;
	uint8_t temp;
	DS1307Read(0x00,&temp);
	temp &= ~(1 << 7); // Set to zero bit - 7; 
	DS1307Write(0x00,temp);
	
	sei();

while(1)
{	
	
		adc_data_read(); 
		usart_data_sender();
	
		if(!(PIND &UP))
		{
			PORTA &= ~UP_LED;
		}
		else PORTA |= UP_LED;

		if(!(PIND &DOWN))
		{
			PORTA &= ~DOWN_LED;
		}
		else PORTA |= DOWN_LED;

		if(!(PIND &LEFT))
		{
			PORTA &= ~LEFT_LED;
		}
		else PORTA |= LEFT_LED;


		if(!(PIND &RIGHT))
		{
			PORTA &= ~RIGHT_LED;
		}
		
		else PORTA |= RIGHT_LED;	
			
		// UART read and write
		if(UCSRA&(RX_COMPLETE))			//	If take a byte from USART;
		{
			send_Uart(getch_Uart());	
			PORTA &= ~(1<<UARTLED);
			// LED on					//	Blink;
			timerDelayMs(50);			
			// LED off
			PORTA |= (1<<UARTLED);
			if(getch_Uart()==NEW_LINE)
			send_Uart(NEW_LINE);
		 } 
	   
		
		    switch(menu)
		    {
				case 0:
					show_temperature(); 
					alarm_sound();
				break;
				
				case 1:
					show_percents();
					alarm_sound();
			    break; 
				
				case 2:
					show_time(); 
					alarm_sound();
				break;
				
		        case 3:
					set_time();
					alarm_sound();
			    break;
			    
			    case 4:
					set_date();
					alarm_sound();
			    break;
			    
			    case 5:
					set_alarm();
					alarm_sound();
			    break; 
		    } 
			  if((PIND &UP)&&(PIND &RIGHT)&&(PIND &LEFT)&&(PIND &DOWN))
			  {
				  ps=0;
			  }
			 
			// printf ("Loh\r");
			 timerDelayMs(1000);
	}
}


void mcu_init()
{

	DDRC |= (1<<2);				// PC2 - is out;
	PORTC |= (0<<2);
	
	DDRD=(0<<DDD7) |(0<<DDD6) |(0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
	PORTD=(1<<PIND7) | (1<<PIND6) |(1<<PIND5) | (1<<PIND4) | (1<<PIND3) | (0<<PIND2) | (0<<PIND1) | (0<<PIND0);

	// Port A init;

	DDRA=(1<<DDA7) |(1<<DDA6) |(1<<DDA5) | (1<<DDA4) | (1<<DDA3) | (0<<DDA2) | (0<<DDA1) | (1<<DDA0);
	PORTA=(1<<PINA7) |(1<<PINA6) |(1<<PINA5) | (1<<PINA4) | (1<<PINA3) | (0<<PINA2) | (0<<PINA1) | (1<<PINA0);
	
	// ADC initialization
	// ADC Clock frequency: 125,000 kHz
	// ADC Voltage Reference: AREF pin
	// ADC Auto Trigger Source: Free Running
	ADMUX=FIRST_ADC_INPUT | (ADC_VREF_TYPE & 0xff);
	ADCSRA=(1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (0<<ADIF) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0);
	SFIOR=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

	// External Interrupt(s) initialization
	// INT0: On
	// INT0 Mode: Low level
	// INT1: Off
	// INT2: Off
	GICR|=(0<<INT1) | (1<<INT0) | (0<<INT2);
	MCUCR=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
	MCUCSR=(0<<ISC2);
	GIFR=(0<<INTF1) | (1<<INTF0) | (0<<INTF2);
}

void print_address(unsigned char* address)
{
	printf("%.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X", address[0],address[1],address[2],address[3],address[4],address[5],address[6],address[7]);
}


unsigned char search_ow_devices(void) 
{
	unsigned char	i;
	unsigned char	id[OW_ROMCODE_SIZE];
	unsigned char	diff, sensors_count;

	sensors_count = 0;

	for( diff = OW_SEARCH_FIRST; diff != OW_LAST_DEVICE && sensors_count < MAXDEVICES ; )
	{
		OW_FindROM( &diff, &id[0] );

		if( diff == OW_PRESENCE_ERR ) break;

		if( diff == OW_DATA_ERR )	break;

		for (i=0;i<OW_ROMCODE_SIZE;i++)
		ow_devices_IDs[sensors_count][i] = id[i];
		
		sensors_count++;
	}
	return sensors_count;

}

void adc_data_toLCD(char* humidity_buf, char* light_sensor_buf)
{
	lcd_gotoxy(0,0);
	lcd_string(humidity_buf,16);
	lcd_gotoxy(0,1);
	lcd_string(light_sensor_buf,16);
}

void temp_data_print(char* lcd_buf_tSenseOne, char* lcd_buf_tSenseTwo)
{
/*
	DS18x20_StartMeasure(ow_devices_IDs[0]);						
	DS18x20_StartMeasure(ow_devices_IDs[1]);						
	timerDelayMs(800);											
	unsigned char	data_array[2];								
	char temp = DS18x20_ReadData(ow_devices_IDs[0], data_array);	
	float first_sensor = DS18x20_ConvertToThemperatureFl(data_array);
	temp = DS18x20_ReadData(ow_devices_IDs[1], data_array);		
	float second_sensor = DS18x20_ConvertToThemperatureFl(data_array);
	sprintf(lcd_buf_tSenseOne,"Indoor: %.1f\C\xdf",first_sensor); */
	lcd_gotoxy(0,0);
	lcd_string(lcd_buf_tSenseOne,16);
	lcd_gotoxy(0,1);
	lcd_string(lcd_buf_tSenseTwo,16);
	timerDelayMs(30);
	
}

void adc_data_read()
{
	humidity = adcData[1];
	lightSensor = adcData[0];
}

void show_temperature() // Case 0;
{ 
	if((!(PIND &DOWN)) && (ps==0))      // Percents;
	{
		menu=1;
		lcd_clr();
		ps=1;
	}
	
		temp_data_print(lcd_buf_tSenseOne, lcd_buf_tSenseTwo);
	
}

void show_percents() // Case 1;
{
	if((!(PIND &UP)) && (ps==0))       // Step to temperature;
	{
		menu=0;
		lcd_clr();
		ps=1;
	}
	
	if((!(PIND &DOWN)) && (ps==0))      // Step to time showing;
	{
		menu=2;
		lcd_clr();
		ps=1;
	}
		adc_data_toLCD(humidity_buf, light_sensor_buf);
}

void show_time()  // Case 2;
{
	if((!(PIND &UP)) && (ps==0))       // Step to humidity & light;
	{
		menu=1;
		lcd_clr();
		ps=1;
	}
	if((!(PIND &DOWN)) && (ps==0))      // Step to the time settings;
	{
		menu=3;
		lcd_clr();
		ps=1;
	}

	// Allow to refresh information;
	lcd_gotoxy(0,0);
	lcd_string("Time:  ", 8);
	lcd_gotoxy(0,1);
	lcd_string("Date:  ", 8);
	if(time_flag==1)
	{
			// Read & convert time from BCD to BIN;
			DS1307Read(0x00,&time_buffer);
			seconds = (((time_buffer & 0xF0) >> 4)*10)+(time_buffer & 0x0F);
			DS1307Read(0x01,&time_buffer);
			minutes = (((time_buffer & 0xF0) >> 4)*10)+(time_buffer & 0x0F);
			DS1307Read(0x02,&time_buffer);
			hour = (((time_buffer & 0xF0) >> 4)*10)+(time_buffer & 0x0F);
			// Read & convert Date from BCD to BIN;
			DS1307Read(0x04,&date_buffer);
			day = (((date_buffer & 0xF0) >> 4)*10)+(date_buffer & 0x0F);
			DS1307Read(0x05,&date_buffer);
			month = (((date_buffer & 0xF0) >> 4)*10)+(date_buffer & 0x0F);
			DS1307Read(0x06,&date_buffer);
			year = (((date_buffer & 0xF0) >> 4)*10)+(date_buffer & 0x0F);

			// Print time to LCD;
			lcd_gotoxy(8,0);
			lcd_dat(hour/10+0x30);
			lcd_dat(hour%10+0x30);
			lcd_dat(':');
			lcd_dat(minutes/10+0x30);
			lcd_dat(minutes%10+0x30);
			lcd_dat(':');
			lcd_dat(seconds/10+0x30);
			lcd_dat(seconds%10+0x30);
			//lcd_gotoxy(9,0);
			//lcd_string("        ",8);
			// Print date to LCD;
			lcd_gotoxy(8,1);
			lcd_dat(day/10+0x30);
			lcd_dat(day%10+0x30);
			lcd_dat('/');
			lcd_dat(month/10+0x30);
			lcd_dat(month%10+0x30);
			lcd_dat('/');
			lcd_dat(year/10+0x30);
			lcd_dat(year%10+0x30);
			//lcd_gotoxy(9,1);
			//lcd_string("        ",8);
	
		if(alarm_on==1)
		{
			lcd_definechar(bellChar, 0x00);
			lcd_gotoxy(7,1);
			lcd_dat(0);               // Print bell symbol;
		}
		time_flag=0;                  // Set time flag to zero;
	}
}

void set_time() // Case 3;
{
	// Review time settings;
	if(sub_time==0)
	{
		lcd_gotoxy(0,0);
		lcd_string("Set time", 8);
		lcd_gotoxy(0,1);
		lcd_dat(hour/10+0x30);
		lcd_dat(hour%10+0x30);
		lcd_dat(':');
		lcd_dat(minutes/10+0x30);
		lcd_dat(minutes%10+0x30);
		lcd_string("    ", 4);		 // Fill strinf with Space;
		
		if((!(PIND &UP))&&(ps==0))   // Step to time screen;
		{
			menu=2;
			ps=1;
			lcd_clr();
		}
		if((!(PIND &RIGHT))&&(ps==0))   // Step to time settings;
		{
			if(subProgram==0)			// Step from main to sub menu;
			{
				sub_time=1;
				ps_set=1;			    // Checking that button is not pressed;
				ps=1;
				lcd_cursor_on();		// Show cursor;
				lcd_gotoxy(1,1);
				subProgram=8;			// Subprogramm 8- changed clock;
			}
		}
		if((!(PIND &DOWN))&&(ps==0))    // Step to date settings;
		{
			menu=4;
			ps=1;
		}
	}
	
	// Set time;
	if(sub_time==1)
	{
		if((!(PIND &RIGHT))&&(ps_set==0))   // Step to settings;
		{
			if(subProgram==9)
			{
				ps_set=1;  // Checking that button is not pressed;
				ps=1;
				lcd_cursor_off();     // Hide cursor;
				subProgram=0;		  // Sub-programm 0 - main menu;
				sub_time=0;
				DS1307Write(0x01, minutes);
				DS1307Write(0x02, hour);
			}
			if(subProgram==8)
			{
				ps_set=1;			  // Checking that button is not pressed;
				lcd_gotoxy(4,1);
				subProgram=9;		  //9 - change minutes;
			}

		}
		if((!(PIND &LEFT))&&(ps_set==0))   // Step to hours settings;
		{
			if(subProgram==9)
			{
				ps_set=1;
				lcd_gotoxy(1,1);
				subProgram=8;
			}

		}
		if((!(PIND &UP))&&(ps_set==0))		// Increase value;
		{
			if(subProgram==8)				// Increase hours;
			{
				hour++;
				if(hour>23)
				{
					hour=0;
				}
				lcd_gotoxy(0,1);
				lcd_dat(hour/10+0x30);
				lcd_dat(hour%10+0x30);
				lcd_gotoxy(1,1);
			}
			if(subProgram==9)      // Increase hours;
			{
				minutes++;
				if(minutes>59)
				{
					minutes=0;
				}
				lcd_gotoxy(3,1);
				lcd_dat(minutes/10+0x30);
				lcd_dat(minutes%10+0x30);
				lcd_gotoxy(4,1);
			}
			ps_set=1;
		}
		
		if((!(PIND &DOWN))&&(ps_set==0))  // Increase value;
		{
			if(subProgram==8)			  // Increase hours;
			{
				if(hour<1)
				{
					hour=24;
				}
				hour--;
				lcd_gotoxy(0,1);
				lcd_dat(hour/10+0x30);
				lcd_dat(hour%10+0x30);
				lcd_gotoxy(1,1);
			}
			if(subProgram==9)			 // Increase hours;
			{
				if(minutes==0)
				{
					minutes=60;
				}
				minutes--;
				lcd_gotoxy(3,1);
				lcd_dat(minutes/10+0x30);
				lcd_dat(minutes%10+0x30);
				lcd_gotoxy(4,1);
			}
			ps_set=1;
		}
		
		 if((PIND &UP)&&(PIND &RIGHT)&&(PIND &LEFT)&&(PIND &DOWN))
		 {
			 ps_set=0;
		 }
	}
	
}

void set_date() // Case 4;
{
	// Date settings Menu review;
	if(sub_date==0)
	{
		
		lcd_gotoxy(0,0);
		lcd_string("Set date", 8);
		lcd_gotoxy(0,1);
		lcd_dat(day/10+0x30);
		lcd_dat(day%10+0x30);
		lcd_dat('-');
		lcd_dat(month/10+0x30);
		lcd_dat(month%10+0x30);
		lcd_dat('-');
		lcd_dat(year/10+0x30);
		lcd_dat(year%10+0x30);
		
		if((!(PIND &UP))&&(ps==0))  // Step to hours settings;
		{
			menu=3;
			ps=1;
			lcd_clr();
		}

		if((!(PIND &DOWN))&&(ps==0))
		{
			if(subProgram==0)     // Go o the time setting menu;
			{
				menu=5; 
				ps=1;
				lcd_clr();
			}
		}
		if((!(PIND &RIGHT))&&(ps==0))   // Enter to settings submenu;
		{
			sub_date=1;
			ps_set=1;					// Check wrung buttons;
			lcd_cursor_on();			// Show cursor;
			lcd_gotoxy(1,1);
			subProgram=4;				// Change current day;
		}
	}
	// Date settings;
	if(sub_date==1)
	{
		if((!(PIND &UP))&&(ps_set==0))  // Increase value;
		{
			if(subProgram==4)			// Increase day;
			{
				day++;
				if(day>31)
				{
					day=1;
				}
				lcd_gotoxy(0,1);
				lcd_dat(day/10+0x30);
				lcd_dat(day%10+0x30);
				lcd_gotoxy(1,1);
			}
			if(subProgram==5)			// Increase month;
			{
				month++;
				if(month>12)
				{
					month=1;
				}
				
				lcd_gotoxy(3,1);
				lcd_dat(month/10+0x30);
				lcd_dat(month%10+0x30);
				lcd_gotoxy(4,1);
			}
			if(subProgram==6)			// Increase year;
			{
				year++;
				if(year>99)
				{
					year=0;
				}
				
				lcd_gotoxy(6,1);
				lcd_dat(year/10+0x30);
				lcd_dat(year%10+0x30);
				lcd_gotoxy(7,1);
			}
			ps_set=1;
		}
		if((!(PIND &DOWN))&&(ps_set==0))  // Decrease value;
		{
			if(subProgram==4)			  // Decrease days;
			{
				day--;
				if(day<1)
				{
					day=31;
				}
				
				lcd_gotoxy(0,1);
				lcd_dat(day/10+0x30);
				lcd_dat(day%10+0x30);
				lcd_gotoxy(1,1);
			}
			if(subProgram==5)			 // Decrease month;
			{
				month--;
				if(month<1)
				{
					month=12;
				}
				
				lcd_gotoxy(3,1);
				lcd_dat(month/10+0x30);
				lcd_dat(month%10+0x30);
				lcd_gotoxy(4,1);
			}
			if(subProgram==6)			 // Decrease year;
			{
				if(year==0)
				{
					year=100;
				}
				year--;
				lcd_gotoxy(6,1);
				lcd_dat(year/10+0x30);
				lcd_dat(year%10+0x30);
				lcd_gotoxy(7,1);
			}
			
			ps_set=1;
		}
		if((!(PIND &RIGHT))&&(ps_set==0))
		{
			ps_set=1;
			if(subProgram==6)			// Back to main menu;
			{
				subProgram=0;
				sub_date=0;
				lcd_cursor_off();		// Hide cursor;
				DS1307Write(0x03, day);
				DS1307Write(0x04, week_day);
				DS1307Write(0x05, month);
				DS1307Write(0x06, year);
				
				ps=1;
			}
			if(subProgram==5)			// Back to clock settings;
			{
				lcd_gotoxy(7,1);
				subProgram=6;
			}
			if(subProgram==4)			// Go to the settings menu;
			{
				lcd_gotoxy(4,1);
				subProgram=5;
			}
			
		}
		if((!(PIND &LEFT))&&(ps_set==0))
		{
			ps_set=1;
			if(subProgram==5)			// Go to the settings menu;
			{
				lcd_gotoxy(1,1);
				subProgram=4;
			}
			if(subProgram==6)			// Back to clock settings;
			{
				lcd_gotoxy(4,1);
				subProgram=5;
			}
			
			
		}
		
		 if((PIND &UP)&&(PIND &RIGHT)&&(PIND &LEFT)&&(PIND &DOWN)) // CHECK this condition!!!!!!!
		 {
			 ps_set=0;
		 }
	}
}

void set_alarm() // Case 5;
{
	// Alarm settings menu show;

	if(sub_alarm==0)              // If we are out of settings mode;
	{
		lcd_gotoxy(0,0);          // Print current alarm settings; 
		lcd_string("Set alarm       ",16);
		lcd_gotoxy(0,1);
		lcd_dat(a_hour/10+0x30);
		lcd_dat(a_hour%10+0x30);
		lcd_dat(':');
		lcd_dat(a_min/10+0x30);
		lcd_dat(a_min%10+0x30);
				
		if(alarm_on==0)            // If alarm off;
		{
			lcd_string(" off       ", 11);
		}
		if(alarm_on==1)
		{
			lcd_string(" on        ", 11);
		}
		// Buttons handler;
		if((!(PIND &UP))&&(ps==0))  // Go to the main screen;
		{                         
			menu=4;                 
			ps=1;                   // Button switch-on;
			lcd_clr();            
		}                       
		/*
		if((!(PIND &DOWN))&&(ps==0))  // Go to the date settings;
		{                         
			menu--;                 
			ps=1;                   // Button switch-on;
			lcd_clr();            
		}   
		*/                   
		if((!(PIND &RIGHT))&&(ps==0))   // Enter to submenu settings;
		{                        
			sub_alarm=1;           
			ps_set=1;					// Button switch-on;
			lcd_cursor_on();			// Show cursor;
			lcd_gotoxy(1,1);       
			subProgram=1;               // Increase alarms hours;
		}                         
		// End of button handlerr; 
	}

	// Alarm settings mode; 

	if(sub_alarm==1)						// If we are in alarm submenu settings;
	{
		//кнопка вверх
		if((!(PIND &UP))&&(ps_set==0))		// Increase value;
		{                             
			if(subProgram==1)				// If subProgram=1 - cursor under hours;
			{                           
				a_hour++;					// Increase hours;
				if(a_hour>23)				// 24 hour format, cause reset when hours more than 23;
				{                         
					a_hour=0;             
				}                         
				lcd_gotoxy(0,1);			 // Change hours only;
				lcd_dat(a_hour/10+0x30);	 
				lcd_dat(a_hour%10+0x30);
				lcd_gotoxy(1,1);          
			}                           
			if(subProgram==2)				// If subProgram=2 - cursor under minutes;
			{                           
				a_min++;					// Increase minutes;
				if(a_min>59)                // Set to zero when value is excess;
				{                         
					a_min=0;               
				}                         
				lcd_gotoxy(3,1);            // Update minutes only;
				lcd_dat(a_min/10+0x30);
				lcd_dat(a_min%10+0x30);
				lcd_gotoxy(4,1);          
			}                           
			if(subProgram==3)				// subProgram=3 cursor under alarm flag;
			{                           
				lcd_gotoxy(5,1);         
				if(alarm_on==0)				// Print "on" if alarm on;
				{                         
					lcd_string(" on  ", 5);     
					alarm_on=1;            
				}                         
				else						// Print "off";
				{                         
					lcd_string(" off", 4);       
					alarm_on=0;             
				}                         
				lcd_gotoxy(6,1);          
			}                           
			ps_set=1;						// Button switch on;
		}   
		                          
		// UP button handler ending;
		
		// Down button;
		if((!(PIND &DOWN))&&(ps_set==0))	// Decrease value;
		{                             
			if(subProgram==1)				// Cursor under hours;
			{                           
				if(a_hour==0)            
				{                        
					a_hour=24;             
				}                         
				a_hour--;                 
				lcd_gotoxy(0,1);          
				lcd_dat(a_hour/10+0x30);
				lcd_dat(a_hour%10+0x30);
				lcd_gotoxy(1,1);          
			}                           
			if(subProgram==3)				// Cursor under alarm;
			{                           
				lcd_gotoxy(5,1);          
				if(alarm_on==0)				// Change to "On" state;
				{                         
					lcd_string(" on  ", 5);     
					alarm_on=1;            
				}                         
				else                      
				{                         
					lcd_string(" off", 4);  // Off;
					alarm_on=0;             
				}                         
				lcd_gotoxy(6,1);          
			}                           
			if(subProgram==2)				// Cursor under minutes, decrease minutes;
			{                           
				if(a_min==0)              
				{                         
					a_min=60;              
				}                         
				a_min--;                  
				lcd_gotoxy(3,1);          
				lcd_dat(a_min/10+0x30);
				lcd_dat(a_min%10+0x30);
				lcd_gotoxy(4,1);          
			}                           
			ps_set=1;                   
		}  
		                           
		// Down button handler ending;		
		
		// Right button; 
		if((!(PIND &RIGHT))&&(ps_set==0))	 // Right button; 
		{                             
			ps_set=1;						 // Button switch on;
			//
			if(subProgram==3)				 // If cursor under alarm - out from settings submenu;
			{								
				lcd_cursor_off();			 // Hide cursor;
				subProgram=0;				 // Cursor to home;
				sub_alarm=0;				 // Return into main menu;
				ps=1;						 // Button switch-on;
			}                           
			if(subProgram==2)				 // Jump to alarm settings on;
			{                           
				subProgram=3;             
				lcd_gotoxy(6,1);          
			}                           
			if(subProgram==1)				 // Jump to minutes settings;
			{                           
				lcd_gotoxy(4,1);            
				subProgram=2;               
			}                           
		}   
		                          
		// Right button handler ending;
		
		// Left button handler;
		if((!(PIND &LEFT))&&(ps_set==0))	 // Left button;
		{                             
			ps_set=1;                   
			if(subProgram==2)				 // Return to hours settings;
			{                           
				lcd_gotoxy(1,1);          
				subProgram=1;             
			}                           
			if(subProgram==3)				 // Return to minute settings;
			{                           
				lcd_gotoxy(4,1);          
				subProgram=2;             
			}                           
		}                            
		//конец обработки влево
		
		
		 if((PIND &UP)&&(PIND &RIGHT)&&(PIND &LEFT)&&(PIND &DOWN)) // CHECK!!!!! ;
		 {
			 ps_set=0;
		 }
	}
}
	
void alarm_sound()
{
		if(alarm_on==1)						// Ring the alarm!
		{
			if((hour==a_hour)&&(minutes==a_min))
			{
				PORTC |=BUZZER;
			}
			else
			{
				PORTC &= ~BUZZER;
			}
		}
}

void usart_data_sender()
{
	unsigned char	data_array[2];
	char temp;
	float first_sensor, second_sensor;

	// Convert to the percents;
	humidity_to_voltage = 100.0 * humidity / 1024;
	light_to_voltage = 100.0 * lightSensor / 1024;
	sprintf(humidity_buf, "Humid.:  %1.2f%%", humidity_to_voltage);
	sprintf(light_sensor_buf, "Light:   %1.2f%%", light_to_voltage);
	
	// Read Temperature;
	DS18x20_StartMeasure(ow_devices_IDs[0]);
	DS18x20_StartMeasure(ow_devices_IDs[1]);
	timerDelayMs(800);

	// Converting temperature;
	temp = DS18x20_ReadData(ow_devices_IDs[0], data_array);
	first_sensor = DS18x20_ConvertToThemperatureFl(data_array);
	temp = DS18x20_ReadData(ow_devices_IDs[1], data_array);
	second_sensor = DS18x20_ConvertToThemperatureFl(data_array);
	sprintf(lcd_buf_tSenseOne, "Indoor: %.1fC\xdf",first_sensor);
	sprintf(lcd_buf_tSenseTwo, "Outdoor:%.1fC\xdf",second_sensor);


//CHECK THIS!!!!!!!!!!!!


	// This buffer I send to USART;
	sprintf(uart_tSenseOne,"Indoor:   %.1f",first_sensor);
	sprintf(uart_tSenseTwo,"Outdoor:  %.1f",second_sensor);

	// Send data to USART;
	timerDelayMs(1000);
	send_Uart(NEW_LINE);

	send_Uart_str(humidity_buf);
	send_Uart(' ');
	send_Uart_str(light_sensor_buf);
	send_Uart(NEW_LINE);
	send_Uart_str(uart_tSenseOne);
	send_Uart(' ');
	send_Uart_str(uart_tSenseTwo);
}

void temperature_init()
{
	for (unsigned char i = 0; i < nDevices; i++) // Sort devices and take a data;
	{
		// Device may be found in its code group, which is located in the first byte address;
		switch (ow_devices_IDs[i][0])
		{
			case OW_DS18B20_FAMILY_CODE:
			{
				printf("\r"); print_address(ow_devices_IDs[i]); // Print carriage, then address;
				printf(" - Thermometer DS18B20"); // Print type of the devicel;
				DS18x20_StartMeasure(ow_devices_IDs[i]); // Start measurement;
				timerDelayMs(800);
				unsigned char	data[2]; // Variable to store the high and low bytes of data;
				DS18x20_ReadData(ow_devices_IDs[i], data);
				
				float real_temperature = DS18x20_ConvertToThemperatureFl(data); // Convert data;
				printf(": %3.2f C", real_temperature);
				
			}
			break;
			case OW_DS18S20_FAMILY_CODE:
			{
				printf("\r"); print_address(ow_devices_IDs[i]);
				printf(" - Thermometer DS18S20");
			}
			break;

			case OW_DS1990_FAMILY_CODE:
			{
				printf("\r"); print_address(ow_devices_IDs[i]);
				printf(" - Serial button DS1990");
			}
			break;
			case OW_DS2430_FAMILY_CODE:
			{
				printf("\r"); print_address(ow_devices_IDs[i]);
				printf(" - EEPROM DS2430");
			}
			break;
			case OW_DS2413_FAMILY_CODE:
			{
				printf("\r"); print_address(ow_devices_IDs[i]);
				printf(" - Switch 2413");
			}
			break;
		}
		
	}
}
/*
void progress(void) 
{
	char i;
	for (i=0; i<255; i++)
	{
		lcd_gotoxy(0, 0);
		f(i, 255, 16);
		lcd_gotoxy(0, 1);
		f(i, 255, 16);
	}
	
}
*/