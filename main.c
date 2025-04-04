

#define F_CPU 16000000UL  // ��������, ��� 16 ��� (���������� ���� �������!)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/************************************************************************/
/*          ��������� �����                                             */
/************************************************************************/
#define LCD_D4      PD4  // {LCD}	D4  (Data 4)
#define LCD_D5      PD5  // {LCD}	D5  (Data 5)
#define LCD_D6      PD6  // {LCD}	D6  (Data 6)
#define LCD_D7      PD7  // {LCD}	D7  (Data 7)

#define LCD_RS      PB0  // {LCD}	D8  (Register Select)
#define LCD_EN      PB1  // {LCD}	D9  (Enable)
#define LCD_LIGHT   PB2  // {LCD}	D10 (Enable) 
#define DIR_PIN     PB3  // {MOTOR} D11
#define STEP_PIN    PB4  // {MOTOR}	D12
#define ENA_PIN     PB5  // {MOTOR} D13

#define KEY_ADC     PC0  // ���������� ���� A0


void adc_init();//������������� ADC
uint16_t adc_read(uint8_t channel);//������ ADC, ��� ����������� ������� ������
uint8_t read_key();//������������� ������� ������
uint8_t debounced_read_key();//����������� � ��������� ������� �������
void lcd_strobe();//������� ������� �� EN
void lcd_send_nibble(uint8_t data);//�������� 4 ��� (������)
void lcd_command(uint8_t cmd);//�������� ������� (RS = 0)
void lcd_data(uint8_t data);//�������� ������ (RS = 1)
void lcd_init();//������������� LCD
void lcd_print(const char *str);//����� ������
char* int_to_str(int32_t num);//




// ������������ ����� (Arduino Nano)


// ��������� ���������
#define STEPS_PER_REV      200
#define MICROSTEPS         16
#define MIN_DELAY          20    // ~5000 �����/��� (����. ��������)
#define MAX_DELAY          1000  // ~100 �����/��� (��������� ��������)

// ���������� ����������
volatile int32_t current_position = 0;
volatile int32_t target_steps = 0;
volatile uint16_t step_delay = MAX_DELAY;
volatile uint8_t movement_flags = 0;

/************************************************************************/
/*			������������� ADC                                           */
/************************************************************************/
void adc_init()
{
	ADMUX = (1 << REFS0);               // ������� ���������� AVcc
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // �������� 64 (~125 ���)
}

/************************************************************************/
/*			������ ADC, ��� ����������� ������� ������					*/
/************************************************************************/
uint16_t adc_read(uint8_t channel)
{
	ADMUX = (1 << REFS0) | (channel & 0x07);  // ����� ������
	ADCSRA |= (1 << ADSC);                    // ������ ��������������
	while (ADCSRA & (1 << ADSC));             // �������� ����������
	return ADC;
}

/************************************************************************/
/*			������������� ������� ������                                */
/************************************************************************/
uint8_t read_key()
{
	uint16_t adc_val = adc_read(KEY_ADC);
	
	if (adc_val < 50)      return 1;  // Right
	else if (adc_val < 150) return 2;  // Up
	else if (adc_val < 300) return 3;  // Down
	else if (adc_val < 500) return 4;  // Left
	else if (adc_val < 700) return 5;  // Select
	else                   return 0;  // ��� �������
}

/************************************************************************/
/*          ����������� � ��������� ������� �������                     */
/************************************************************************/
uint8_t debounced_read_key()
{
	static uint16_t last_key = 0;
	uint8_t key = read_key();
	
	if (key == last_key) {
		return key;  // ������ ������������
		} else {
		_delay_ms(50);  // �������� ��� ���������� ��������
		last_key = read_key();
		return last_key;
	}
}
/************************************************************************/
/*          ������� ������� �� EN                                       */
/************************************************************************/
void lcd_strobe()
{
	PORTB |= (1 << LCD_EN);
	_delay_us(1);          // ������� >450 ��
	PORTB &= ~(1 << LCD_EN);
	_delay_us(100);        // ����� ����� ���������
}

/************************************************************************/
/*          �������� 4 ��� (������)                                     */
/************************************************************************/
void lcd_send_nibble(uint8_t data)
{
	// ���������� � PORTD (D4-D7)
	PORTD = (PORTD & 0x0F) | (data << 4);
	lcd_strobe();
}

/************************************************************************/
/*          �������� ������� (RS = 0)                                   */
/************************************************************************/
void lcd_command(uint8_t cmd)
{
	PORTB &= ~(1 << LCD_RS);  // RS = 0 (�������)
	lcd_send_nibble(cmd >> 4);  // ������� �����
	lcd_send_nibble(cmd & 0x0F); // ������� �����
	if (cmd == 0x01 || cmd == 0x02) _delay_ms(2);  // �������/������� ����
}

/************************************************************************/
/*          �������� ������ (RS = 1)                                    */
/************************************************************************/
void lcd_data(uint8_t data)
{
	PORTB |= (1 << LCD_RS);  // RS = 1 (������)
	lcd_send_nibble(data >> 4);
	lcd_send_nibble(data & 0x0F);
	_delay_us(100);
}

/************************************************************************/
/*          ������������� LCD                                           */
/************************************************************************/
void lcd_init()
{
	// 1. ��������� ����� �� �����
	DDRB |= (1 << LCD_RS) | (1 << LCD_EN) | (1 << LCD_LIGHT);
	DDRD |= (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);
	
	PORTB |= (1 << LCD_D7);
	
	// 2. ��� ������������ ������� (>15 ��)
	_delay_ms(50);

	// 3. ������������������ ������ (�� �������� HD44780)
	PORTB &= ~(1 << LCD_RS);  // RS = 0 (�������)
	
	// 3.1 ������ ������� (8-������ �����)
	lcd_send_nibble(0x03);
	_delay_ms(5);  // ��� 5 ��

	// 3.2 ������ �������
	lcd_send_nibble(0x03);
	_delay_us(100);  // ��� 100 ���

	// 3.3 ������ �������
	lcd_send_nibble(0x03);
	_delay_us(100);

	// 4. ������� � 4-������ �����
	lcd_send_nibble(0x02);
	_delay_us(100);

	// 5. �������������� ���������
	lcd_command(0x28);  // 4-���, 2 ������, ����� 5x8
	lcd_command(0x0C);  // ������� ON, ������ OFF
	lcd_command(0x06);  // ������������� �������
	lcd_command(0x01);  // ������� �������
	_delay_ms(2);       // ��� ����������
}

/************************************************************************/
/*           ����� ������                                               */
/************************************************************************/
void lcd_print(const char *str)
{
	while (*str)
	{
		lcd_data(*str++);
	}
}

/************************************************************************/
/*      ����������� ����� � ������ ��� ������ �� LCD                    */
/************************************************************************/
char* int_to_str(int32_t num)
{
	static char buf[12];
	itoa(num, buf, 10);
	return buf;
}

/************************************************************************/
/*       ������������� ������� (Timer1)                                 */
/************************************************************************/
// 
void timer1_init()
{
	TCCR1A = 0;                 // Normal port operation
	TCCR1B = (1 << WGM12)       // CTC mode
	| (1 << CS11);       // Prescaler 8 (0.5us ��� 16MHz)
	TIMSK1 = (1 << OCIE1A);     // Enable compare match interrupt
}

/************************************************************************/
/*       ������������� ��������                                         */
/************************************************************************/
void motor_init()
{
	DDRB |= (1 << ENA_PIN) | (1 << DIR_PIN) | (1 << STEP_PIN);
	PORTB |= (1 << ENA_PIN);    // ��������� �������� �� ���������
	timer1_init();
}

/************************************************************************/
/*       ���������� ��� ��������� STEP ���������                        */
/************************************************************************/
ISR(TIMER1_COMPA_vect)
{
	static uint16_t accel_counter = 0;
	static uint8_t step_phase = 0;
	
	if (target_steps != 0)
	{
		if (step_phase == 0)
		{
			// ��������� �������� STEP
			PORTB |= (1 << STEP_PIN);
			step_phase = 1;
			
			// ���������� �������
			if (target_steps > 0)
			{
				current_position++;
				target_steps--;
			}
			else
			{
				current_position--;
				target_steps++;
			}
		}
		else
		{
			PORTB &= ~(1 << STEP_PIN);
			step_phase = 0;
			
			// ���������� ��������� ��������
			accel_counter++;
			if (accel_counter > 50)
			{  // ������ 50 �����
				accel_counter = 0;
				
				// ������ � ������ ��������
				if (abs(target_steps) > (abs(target_steps) + current_position)/3)
				{
					if (step_delay > MIN_DELAY) step_delay -= 2;
				}
				// ���������� � ����� ��������
				else
				{
					if (step_delay < MAX_DELAY) step_delay += 5;
				}
			}
		}
		
		OCR1A = step_delay;  // ��������� ����� ��������
	}
}

/************************************************************************/
/*       ������� �����������                                            */
/************************************************************************/
void move_to_position(int32_t position)
{
	// ������ ����������� �����
	target_steps = (position * MICROSTEPS) - current_position;
	
	// ��������� �����������
	if (target_steps >= 0)
	{
		PORTB |= (1 << DIR_PIN);
	}
	else
	{
		PORTB &= ~(1 << DIR_PIN);
	}
	
	// ��������� ��������
	//PORTB &= ~(1 << ENA_PIN);
	PORTB |= (1 << ENA_PIN);
	// ����� ���������� ��������
	step_delay = MAX_DELAY;
	movement_flags = 0;
	
	// ���� ���������� ��������
	while (target_steps != 0);
	
	// ���������� �������� (�����������)
	// PORTB |= (1 << ENA_PIN);
}

/************************************************************************/
/*      ������� ����������� ��� �������� �������                        */
/************************************************************************/
void move_immediate(int32_t steps)
{
	target_steps = steps * MICROSTEPS;
	step_delay = MIN_DELAY;
	
	if (steps >= 0)
	{
		PORTB |= (1 << DIR_PIN);
	}
	else
	{
		PORTB &= ~(1 << DIR_PIN);
	}
	
	PORTB &= ~(1 << ENA_PIN);
	while (target_steps != 0);
}

/************************************************************************/
/*      ������ �������������                                            */
/************************************************************************/
int main()
{
	lcd_init();
	 lcd_print("Hello, HD44780!");
	 lcd_command(0xC0);  // ������� �� 2-� ������
	 lcd_print("RS=PB0, EN=PB1");
	 
	motor_init();
	sei();  // ��������� ����������
	
	// ������������ ����� ������ (������ ���� � ��������� �����)
	// buttons_init();
	
	while(1)
	{
		// ������ ��������:
		move_to_position(1600);  // 1 ������ (200*8 ����������)
		_delay_ms(500);
		move_to_position(0);     // ������� � 0
		_delay_ms(500);
		
		// ����� ����� �������� ��������� ������ �� ���������� �����
		// uint8_t btn = read_button();
		// if (btn == RIGHT_BTN) move_to_position(100);
	}
}