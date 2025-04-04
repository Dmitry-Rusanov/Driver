#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

/************************************************************************/
/*          Настройка пинов LCD                                         */
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

// Макросы команд LCD
#define LCD_CLEAR         0x01
#define LCD_RETURN_HOME   0x02
#define LCD_DISPLAY_ON    0x0C
#define LCD_ENTRY_MODE    0x06
#define LCD_FUNCTION_SET  0x28

#define STEPS_TO_MOVE 1000
#define MIN_DELAY 100
#define MAX_DELAY 800

char buffer[10];
volatile uint16_t steps_remaining = 0;
volatile uint8_t step_dir = 0;
volatile uint16_t step_delay = MAX_DELAY;

/************************************************************************/
/*          Прототипы функций                                           */
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

void timer1_init() {
	TCCR1B |= (1 << WGM12);
	TIMSK1 |= (1 << OCIE1A);
	OCR1A = ((F_CPU / 8) / (1000000 / step_delay)) - 1;
	TCCR1B |= (1 << CS11);
}

ISR(TIMER1_COMPA_vect) {
	if (steps_remaining > 0) {
		if (step_dir)
		PORTB |= (1 << DIR_PIN);
		else
		PORTB &= ~(1 << DIR_PIN);

		PORTB |= (1 << STEP_PIN);
		for (volatile uint8_t i = 0; i < 20; i++) __asm__ __volatile__("nop");
		PORTB &= ~(1 << STEP_PIN);

		steps_remaining--;

		if (steps_remaining > 0) {
			if (step_delay > MIN_DELAY && steps_remaining > 50) {
				step_delay -= 5;
				} else if (steps_remaining < 50 && step_delay < MAX_DELAY) {
				step_delay += 5;
			}
			OCR1A = ((F_CPU / 8) / (1000000 / step_delay)) - 1;
		}
	}
}

void stepper_start_move(uint16_t steps, uint8_t dir) {
	step_dir = dir;
	steps_remaining = steps;
	step_delay = MAX_DELAY;
	OCR1A = ((F_CPU / 8) / (1000000 / step_delay)) - 1;
}

void stepper_init() {
	DDRB |= (1 << STEP_PIN) | (1 << DIR_PIN) | (1 << EN_PIN);
	PORTB &= ~(1 << EN_PIN);
}

void stepper_enable(uint8_t enable) {
	if (enable) {
		PORTB |= (1 << EN_PIN);
		} else {
		PORTB &= ~(1 << EN_PIN);
	}
}

/************************************************************************/
/*          АЦП                                                         */
/************************************************************************/
void adc_init() {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

uint16_t adc_read(uint8_t channel) {
	ADMUX = (1 << REFS0) | (channel & 0x07);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

uint8_t read_key() {
	uint16_t adc_val = adc_read(KEY_ADC);

	if (adc_val < 50)        return 1;
	else if (adc_val < 150)  return 2;
	else if (adc_val < 300)  return 3;
	else if (adc_val < 500)  return 4;
	else if (adc_val < 700)  return 5;
	else                    return 0;
}

uint8_t debounced_read_key() {
	static uint8_t last_key = 0;
	uint8_t key = read_key();

	if (key == last_key) {
		return key;
		} else {
		_delay_ms(50);
		last_key = read_key();
		return last_key;
	}
}

/************************************************************************/
/*          LCD функции                                                 */
/************************************************************************/
void lcd_strobe() {
	PORTB |= (1 << LCD_EN);
	_delay_us(1);
	PORTB &= ~(1 << LCD_EN);
	_delay_us(100);
}

void lcd_send_nibble(uint8_t data) {
	PORTD = (PORTD & 0x0F) | ((data & 0x0F) << 4);
	lcd_strobe();
}

void lcd_command(uint8_t cmd) {
	PORTB &= ~(1 << LCD_RS);
	lcd_send_nibble(cmd >> 4);
	lcd_send_nibble(cmd & 0x0F);
	if (cmd == LCD_CLEAR || cmd == LCD_RETURN_HOME) _delay_ms(2);
}

void lcd_data(uint8_t data) {
	PORTB |= (1 << LCD_RS);
	lcd_send_nibble(data >> 4);
	lcd_send_nibble(data & 0x0F);
	_delay_us(100);
}

void lcd_backlight(uint8_t on) {
	if (on) PORTB |= (1 << LCD_LIGHT);
	else    PORTB &= ~(1 << LCD_LIGHT);
}

void lcd_init() {
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

void lcd_print(const char *str) {
	while (*str) {
		lcd_data(*str++);
	}
}

char* int_to_str(int32_t num) {
	static char buf[12];
	sprintf(buf, "%ld", num);
	return buf;
}

/************************************************************************/
/*          MAIN                                                        */
/************************************************************************/
int main() {
	lcd_init();
	adc_init();
	stepper_init();
	stepper_enable(1);
	timer1_init();
	sei();

	lcd_print("Press any button");
	_delay_ms(1000);

	uint8_t prev_key = 0;

	while (1) {
		uint8_t key = debounced_read_key();

		if (key != prev_key) {
			lcd_command(LCD_CLEAR);

			switch (key) {
				case 1:
				lcd_print("Right");
				stepper_start_move(STEPS_TO_MOVE, 1);
				break;
				case 4:
				lcd_print("Left");
				stepper_start_move(STEPS_TO_MOVE, 0);
				break;
				case 2: lcd_print("Up"); break;
				case 3: lcd_print("Down"); break;
				case 5: lcd_print("Select"); break;
				default: lcd_print("No key"); break;
			}

			prev_key = key;
		}
	}
}
