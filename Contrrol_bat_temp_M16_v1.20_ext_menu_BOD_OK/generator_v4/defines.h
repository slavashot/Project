/*
defines
 */

/* CPU frequency */
#define F_CPU 16000000UL

/* UART baud rate */
#define UART_BAUD  9600

/* HD44780 LCD port connections */
#define HD44780_PORT B
#define HD44780_RS PORT6
#define HD44780_RW PORT4
#define HD44780_E  PORT5
/* The data bits have to be in ascending order. */
#define HD44780_D4 PORT0
#define HD44780_D5 PORT1
#define HD44780_D6 PORT2
#define HD44780_D7 PORT3
//LED
#define OFF_LCD_BACKLIGHT			PORTB &= ~_BV(PB7);
#define ON_LCD_BACKLIGHT			PORTB|=_BV(PB7);

#define OFF_RELAY_1				PORTD &= ~_BV(PD5);
#define ON_RELAY_1				PORTD|=_BV(PD5);

#define OFF_RELAY_2				PORTD &= ~_BV(PD6);
#define ON_RELAY_2				PORTD|=_BV(PD6);

#define OFF_BUZER				PORTD &= ~_BV(PD4);
#define ON_BUZER				PORTD|=_BV(PD4);

