#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>


/************************************************************************/
/*          LCD Pin Definitions                                         */
/************************************************************************/
#define LCD_D4      PD4
#define LCD_D5      PD5
#define LCD_D6      PD6
#define LCD_D7      PD7

#define LCD_RS      PB0
#define LCD_EN      PB1
#define LCD_LIGHT   PB2
#define STEP_PIN    PB3
#define DIR_PIN     PB4
#define EN_PIN      PB5

#define KEY_ADC     PC0

// LCD Command Macros
#define LCD_CLEAR         0x01
#define LCD_RETURN_HOME   0x02
#define LCD_DISPLAY_ON    0x0C
#define LCD_ENTRY_MODE    0x06
#define LCD_FUNCTION_SET  0x28

#define UART_BUFFER_SIZE 32
volatile char uart_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_index = 0;


/************************************************************************/
/*          Editable Parameters                                        */
/************************************************************************/
<<<<<<< HEAD
volatile uint16_t step_count = 1000;   //  оличество шагов
volatile uint16_t step_speed = 200;    // —корость в шагах/сек
volatile uint8_t  editing_mode = 0;
volatile uint16_t steps_remaining = 0;
volatile uint8_t  step_dir = 0; // 1 - вправо, 0 - влево
volatile uint8_t  bSend_cmd = 0;
char buffer[16];


=======
// Initial values
volatile uint16_t step_count = 1000;   // Number of steps
volatile uint16_t step_delay = 1000;   // Delay between steps (in microseconds)
// Edit mode: 0 = normal, 1 = edit steps, 2 = edit speed
volatile uint8_t editing_mode = 0;

char buffer[16];

// Movement variables
volatile uint16_t steps_remaining = 0;
volatile uint8_t step_dir = 0; // 1 - right, 0 - left
>>>>>>> main

/************************************************************************/
/*          Function Prototypes                                         */
/************************************************************************/
void adc_init();
uint16_t adc_read(uint8_t channel);
uint8_t read_key();
uint8_t debounced_read_key();
void lcd_strobe();
void lcd_send_nibble(uint8_t data);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_init();
void lcd_print(const char *str);
char* int_to_str(int32_t num);
void lcd_backlight(uint8_t on);
void update_lcd_display();

void stepper_init();
void stepper_enable(uint8_t enable);
void stepper_start_move(uint16_t steps, uint8_t dir);
void timer1_init();

void uart_init();
void uart_send_char(char c);
void uart_send_string(const char* str);
void parse_uart_command(const char* cmd);
uint8_t uart_available();

void uart_init()
{
	UBRR0H = 0;
	UBRR0L = 103; // 9600 baud @16MHz (F_CPU/16/baud - 1)
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

void uart_send_char(char c)
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

char uart_receive_char()
{
	while (!(UCSR0A & (1 << RXC0)));  // ∆дЄм, пока данные придут
	return UDR0;
}

void uart_send_string(const char* str)
{
	while (*str)
	uart_send_char(*str++);
}

void uart_read_line(char *buf, uint8_t max_len)
{
	uint8_t i = 0;
	char c;

	while (i < max_len - 1)
	{
		c = uart_receive_char();
		if (c == '\n' || c == '\r')
		break;
		buf[i++] = c;
	}
	buf[i] = '\0';  // Null-terminate
}

void parse_uart_command(const char* cmd)
{
	if (cmd[0] == 'S')  // Set step count
	{
		//uart_send_string("\n\rCMD: S\n\r");
		uint16_t val = 0;
		if (sscanf(&cmd[1], "%u", &val) == 1)
		{
			uart_send_string(printf("Val: %s",val));
			step_count = val;
		}
	}
	else if (cmd[0] == 'V')  // Set delay
	{
		uint16_t val = 0;
		if (sscanf(&cmd[1], "%u", &val) == 1)
		{
			//step_speed = val;
			set_speed(val);
		}
	}
	else if (strcmp(cmd, "R") == 0)  // Move right
	{
		stepper_start_move(step_count, 1);
	}
	else if (strcmp(cmd, "L") == 0)  // Move left
	{
		stepper_start_move(step_count, 0);
	}

	update_lcd_display();
}

uint8_t uart_available()
{
	return (UCSR0A & (1 << RXC0));
}


/************************************************************************/
/*          ADC (Analog to Digital Converter)                           */
/************************************************************************/
void adc_init()
{
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Prescaler 64 (~125 kHz)
}

uint16_t adc_read(uint8_t channel)
{
	ADMUX = (1 << REFS0) | (channel & 0x07);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

uint8_t read_key()
{
	uint16_t adc_val = adc_read(KEY_ADC);
	// Calibration values for your button matrix:
	if (adc_val < 50)         return 1;  // Right
	else if (adc_val < 150)   return 2;  // Up
	else if (adc_val < 300)   return 3;  // Down
	else if (adc_val < 500)   return 4;  // Left
	else if (adc_val < 700)   return 5;  // Select
	else                      return 0;  // No press
}

uint8_t debounced_read_key()
{
	static uint8_t last_key = 0;
	uint8_t key = read_key();
	if (key == last_key)
	{
		return key;
	}
	else
	{
		_delay_ms(50);
		last_key = read_key();
		return last_key;
	}
}

/************************************************************************/
/*          LCD Functions                                               */
/************************************************************************/
void lcd_strobe()
{
	PORTB |= (1 << LCD_EN);
	_delay_us(1);
	PORTB &= ~(1 << LCD_EN);
	_delay_us(100);
}

void lcd_send_nibble(uint8_t data)
{
	PORTD = (PORTD & 0x0F) | ((data & 0x0F) << 4);
	lcd_strobe();
}

void lcd_command(uint8_t cmd)
{
	PORTB &= ~(1 << LCD_RS);
	lcd_send_nibble(cmd >> 4);
	lcd_send_nibble(cmd & 0x0F);
	if (cmd == LCD_CLEAR || cmd == LCD_RETURN_HOME)
	_delay_ms(2);
}

void lcd_data(uint8_t data)
{
	PORTB |= (1 << LCD_RS);
	lcd_send_nibble(data >> 4);
	lcd_send_nibble(data & 0x0F);
	_delay_us(100);
}

void lcd_backlight(uint8_t on)
{
	if (on)
	PORTB |= (1 << LCD_LIGHT);
	else
	PORTB &= ~(1 << LCD_LIGHT);
}

void lcd_init()
{
	DDRB |= (1 << LCD_RS) | (1 << LCD_EN) | (1 << LCD_LIGHT);
	DDRD |= (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);

	lcd_backlight(1);
	_delay_ms(50);

	PORTB &= ~(1 << LCD_RS);
	lcd_send_nibble(0x03);
	_delay_ms(5);
	lcd_send_nibble(0x03);
	_delay_us(100);
	lcd_send_nibble(0x03);
	_delay_us(100);
	lcd_send_nibble(0x02);
	_delay_us(100);

	lcd_command(LCD_FUNCTION_SET);
	lcd_command(LCD_DISPLAY_ON);
	lcd_command(LCD_ENTRY_MODE);
	lcd_command(LCD_CLEAR);
	_delay_ms(2);
}

void lcd_print(const char *str)
{
	while (*str)
	{
		lcd_data(*str++);
	}
}

char* int_to_str(int32_t num)
{
	static char buf[12];
	sprintf(buf, "%ld", num);
	return buf;
}

void update_lcd_display()
{
	lcd_command(LCD_CLEAR);
	if (editing_mode == 1)
	{
		lcd_print("Edit Steps:");
	}
<<<<<<< HEAD
	else
	if (editing_mode == 2)
=======
	else if (editing_mode == 2)
>>>>>>> main
	{
		lcd_print("Edit Speed:");
	}
	else
	{
		lcd_print("Ready");
	}

	lcd_command(0xC0);  // Second row
	sprintf(buffer, "S:%u  D:%u", step_count, step_delay);
	lcd_print(buffer);
}

/************************************************************************/
/*          Stepper Motor Functions                                     */
/************************************************************************/
void stepper_init()
{
	DDRB |= (1 << STEP_PIN) | (1 << DIR_PIN) | (1 << EN_PIN);
	// Disable driver at start (if required), we will enable it later
	PORTB &= ~(1 << EN_PIN);
}

<<<<<<< HEAD
void stepper_enable(uint8_t enable) 
{
	if (enable) 
	{
		PORTB |= (1 << EN_PIN);
		} 
		else 
		{
=======
void stepper_enable(uint8_t enable)
{
	if (enable)
	{
		PORTB |= (1 << EN_PIN);  // Enable driver (active high)
	}
	else
	{
>>>>>>> main
		PORTB &= ~(1 << EN_PIN);
	}
}

<<<<<<< HEAD
void stepper_start_move(uint16_t steps, uint8_t dir) 
=======
// Start moving the motor with specified steps and direction
void stepper_start_move(uint16_t steps, uint8_t dir)
>>>>>>> main
{
	step_dir = dir;
	steps_remaining = steps;
}

<<<<<<< HEAD
void set_speed(uint16_t steps_per_sec) 
{
	if (steps_per_sec == 0) steps_per_sec = 1;
	step_speed = steps_per_sec;
	OCR1A = (F_CPU / 8 / step_speed) - 1;
}

void timer1_init() 
{
=======
/************************************************************************/
/*          Timer 1 (CTC)                                              */
/************************************************************************/
void timer1_init()
{
	// Setup Timer1 in CTC mode, TOP = OCR1A
>>>>>>> main
	TCCR1B |= (1 << WGM12);
	TIMSK1 |= (1 << OCIE1A);  // Enable interrupt on compare match
	// Calculate OCR1A value based on step_delay
	OCR1A = ((F_CPU / 8) / (1000000 / step_delay)) - 1;
	// Prescaler 8:
	TCCR1B |= (1 << CS11);
}

<<<<<<< HEAD
ISR(TIMER1_COMPA_vect) 
{
	if (steps_remaining > 0) 
=======
// Generate step pulses for the stepper motor in the interrupt
ISR(TIMER1_COMPA_vect)
{
	if (steps_remaining > 0)
>>>>>>> main
	{
		if (step_dir)
		PORTB |= (1 << DIR_PIN);
		else
		PORTB &= ~(1 << DIR_PIN);

		PORTB |= (1 << STEP_PIN);
<<<<<<< HEAD
		for (volatile uint8_t i = 0; i < 20; i++) 
=======
		for (volatile uint8_t i = 0; i < 20; i++)
>>>>>>> main
		{
			__asm__ __volatile__("nop");
		}
		PORTB &= ~(1 << STEP_PIN);

		steps_remaining--;
	}
}

<<<<<<< HEAD
ISR(USART_RX_vect)
{
	char c = UDR0;

	if (c == '\n' || c == '\r'|| c == 'e')
	{
		//uart_send_string("end string");
		uart_buffer[uart_index] = '\0';
		//uart_send_string(uart_buffer);
		parse_uart_command((const char*)uart_buffer);
		uart_index = 0;
	}
	else if (uart_index < UART_BUFFER_SIZE - 1)
	{
		uart_buffer[uart_index++] = c;
		
		//uart_send_string(uart_buffer);
	}
}


=======
/************************************************************************/
/*          MAIN Function                                               */
/************************************************************************/
>>>>>>> main
int main(void)
{
	lcd_init();
	adc_init();
	stepper_init();
	stepper_enable(1);
	timer1_init();
<<<<<<< HEAD
	uart_init();
	sei();
	uart_send_string("Stepper ready\n");
=======
	sei();  // Enable global interrupts

	// Display initial message
>>>>>>> main
	lcd_print("Press any button");
	_delay_ms(1000);
	update_lcd_display();

<<<<<<< HEAD
	//uint8_t prev_key = 0;
=======
	uint8_t prev_key = 0;
	// Track the state of SELECT button (code 5)
>>>>>>> main
	uint8_t last_select = 0;

	while (1)
	{
<<<<<<< HEAD
		if (bSend_cmd)
		{
			stepper_start_move(step_count, step_dir);
		}
	
		uint8_t key = debounced_read_key();

		if (key == 5 && last_select == 0)
		{
=======
		uint8_t key = debounced_read_key();

		// If SELECT button is pressed (code 5)
		if (key == 5 && last_select == 0)
		{
			// Toggle edit mode: 0 -> 1 -> 2 -> 0
>>>>>>> main
			editing_mode = (editing_mode + 1) % 3;
			update_lcd_display();
		}
		last_select = (key == 5);

<<<<<<< HEAD

		if (editing_mode == 0)
		{
			if (key == 1)
=======
		// If not in edit mode, start movement
		if (editing_mode == 0)
		{
			if (key == 1)  // Right Ц move right
>>>>>>> main
			{
				lcd_command(LCD_CLEAR);
				lcd_print("Move Right");
				stepper_start_move(step_count, 1);
			}
<<<<<<< HEAD
			else if (key == 4)
=======
			else if (key == 4)  // Left Ц move left
>>>>>>> main
			{
				lcd_command(LCD_CLEAR);
				lcd_print("Move Left");
				stepper_start_move(step_count, 0);
			}
		}
		else
		{
<<<<<<< HEAD
			if (key == 2)
=======
			// In edit mode:
			if (key == 2)  // UP
>>>>>>> main
			{
				if (editing_mode == 1)
				{
					step_count += 100;
				}
				else if (editing_mode == 2)
				{
<<<<<<< HEAD
					step_speed += 50;
					set_speed(step_speed);
				}
				update_lcd_display();
			}
			if (key == 3)
=======
					// If speed is too high, reduce delay
					if (step_delay > 50)
					step_delay -= 50;
				}
				update_lcd_display();
			}
			if (key == 3)  // DOWN
>>>>>>> main
			{
				if (editing_mode == 1 && step_count >= 100)
				{
					step_count -= 100;
<<<<<<< HEAD
				} 
				else if (editing_mode == 2 && step_speed > 50)
				{
					step_speed -= 50;
					set_speed(step_speed);
=======
				}
				else if (editing_mode == 2)
				{
					step_delay += 50;
>>>>>>> main
				}
				update_lcd_display();
			}
		}

		// Recalculate timer based on the current step_delay value
		OCR1A = ((F_CPU / 8) / (1000000 / step_delay)) - 1;

		_delay_ms(100);
		//prev_key = key;
	}
	return 0;
}
