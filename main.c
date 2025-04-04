

#define F_CPU 16000000UL  // Например, для 16 МГц (указывайте свою частоту!)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/************************************************************************/
/*          Настройка пинов                                             */
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

#define KEY_ADC     PC0  // Аналоговый вход A0


void adc_init();//Инициализация ADC
uint16_t adc_read(uint8_t channel);//Чтение ADC, для определения нажатой кнопки
uint8_t read_key();//Распознование нажатой кнопки
uint8_t debounced_read_key();//Антидребезг и обработка длинных нажатий
void lcd_strobe();//Быстрый импульс на EN
void lcd_send_nibble(uint8_t data);//Отправка 4 бит (ниббла)
void lcd_command(uint8_t cmd);//Отправка команды (RS = 0)
void lcd_data(uint8_t data);//Отправка данных (RS = 1)
void lcd_init();//Инициализация LCD
void lcd_print(const char *str);//Вывод строки
char* int_to_str(int32_t num);//




// Конфигурация пинов (Arduino Nano)


// Параметры двигателя
#define STEPS_PER_REV      200
#define MICROSTEPS         16
#define MIN_DELAY          20    // ~5000 шагов/сек (макс. скорость)
#define MAX_DELAY          1000  // ~100 шагов/сек (начальная скорость)

// Глобальные переменные
volatile int32_t current_position = 0;
volatile int32_t target_steps = 0;
volatile uint16_t step_delay = MAX_DELAY;
volatile uint8_t movement_flags = 0;

/************************************************************************/
/*			Инициализация ADC                                           */
/************************************************************************/
void adc_init()
{
	ADMUX = (1 << REFS0);               // Опорное напряжение AVcc
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Делитель 64 (~125 кГц)
}

/************************************************************************/
/*			Чтение ADC, для определения нажатой кнопки					*/
/************************************************************************/
uint16_t adc_read(uint8_t channel)
{
	ADMUX = (1 << REFS0) | (channel & 0x07);  // Выбор канала
	ADCSRA |= (1 << ADSC);                    // Запуск преобразования
	while (ADCSRA & (1 << ADSC));             // Ожидание завершения
	return ADC;
}

/************************************************************************/
/*			Распознование нажатой кнопки                                */
/************************************************************************/
uint8_t read_key()
{
	uint16_t adc_val = adc_read(KEY_ADC);
	
	if (adc_val < 50)      return 1;  // Right
	else if (adc_val < 150) return 2;  // Up
	else if (adc_val < 300) return 3;  // Down
	else if (adc_val < 500) return 4;  // Left
	else if (adc_val < 700) return 5;  // Select
	else                   return 0;  // Нет нажатия
}

/************************************************************************/
/*          Антидребезг и обработка длинных нажатий                     */
/************************************************************************/
uint8_t debounced_read_key()
{
	static uint16_t last_key = 0;
	uint8_t key = read_key();
	
	if (key == last_key) {
		return key;  // Кнопка удерживается
		} else {
		_delay_ms(50);  // Задержка для подавления дребезга
		last_key = read_key();
		return last_key;
	}
}
/************************************************************************/
/*          Быстрый импульс на EN                                       */
/************************************************************************/
void lcd_strobe()
{
	PORTB |= (1 << LCD_EN);
	_delay_us(1);          // Импульс >450 нс
	PORTB &= ~(1 << LCD_EN);
	_delay_us(100);        // Пауза между командами
}

/************************************************************************/
/*          Отправка 4 бит (ниббла)                                     */
/************************************************************************/
void lcd_send_nibble(uint8_t data)
{
	// Записываем в PORTD (D4-D7)
	PORTD = (PORTD & 0x0F) | (data << 4);
	lcd_strobe();
}

/************************************************************************/
/*          Отправка команды (RS = 0)                                   */
/************************************************************************/
void lcd_command(uint8_t cmd)
{
	PORTB &= ~(1 << LCD_RS);  // RS = 0 (команда)
	lcd_send_nibble(cmd >> 4);  // Старший ниббл
	lcd_send_nibble(cmd & 0x0F); // Младший ниббл
	if (cmd == 0x01 || cmd == 0x02) _delay_ms(2);  // Очистка/возврат дома
}

/************************************************************************/
/*          Отправка данных (RS = 1)                                    */
/************************************************************************/
void lcd_data(uint8_t data)
{
	PORTB |= (1 << LCD_RS);  // RS = 1 (данные)
	lcd_send_nibble(data >> 4);
	lcd_send_nibble(data & 0x0F);
	_delay_us(100);
}

/************************************************************************/
/*          Инициализация LCD                                           */
/************************************************************************/
void lcd_init()
{
	// 1. Настройка пинов на выход
	DDRB |= (1 << LCD_RS) | (1 << LCD_EN) | (1 << LCD_LIGHT);
	DDRD |= (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);
	
	PORTB |= (1 << LCD_D7);
	
	// 2. Ждём стабилизации питания (>15 мс)
	_delay_ms(50);

	// 3. Последовательность сброса (по даташиту HD44780)
	PORTB &= ~(1 << LCD_RS);  // RS = 0 (команда)
	
	// 3.1 Первая попытка (8-битный режим)
	lcd_send_nibble(0x03);
	_delay_ms(5);  // Ждём 5 мс

	// 3.2 Вторая попытка
	lcd_send_nibble(0x03);
	_delay_us(100);  // Ждём 100 мкс

	// 3.3 Третья попытка
	lcd_send_nibble(0x03);
	_delay_us(100);

	// 4. Переход в 4-битный режим
	lcd_send_nibble(0x02);
	_delay_us(100);

	// 5. Функциональная настройка
	lcd_command(0x28);  // 4-бит, 2 строки, шрифт 5x8
	lcd_command(0x0C);  // Дисплей ON, курсор OFF
	lcd_command(0x06);  // Автоинкремент курсора
	lcd_command(0x01);  // Очистка дисплея
	_delay_ms(2);       // Ждём завершения
}

/************************************************************************/
/*           Вывод строки                                               */
/************************************************************************/
void lcd_print(const char *str)
{
	while (*str)
	{
		lcd_data(*str++);
	}
}

/************************************************************************/
/*      Конвертация числа в строку для вывода на LCD                    */
/************************************************************************/
char* int_to_str(int32_t num)
{
	static char buf[12];
	itoa(num, buf, 10);
	return buf;
}

/************************************************************************/
/*       Инициализация таймера (Timer1)                                 */
/************************************************************************/
// 
void timer1_init()
{
	TCCR1A = 0;                 // Normal port operation
	TCCR1B = (1 << WGM12)       // CTC mode
	| (1 << CS11);       // Prescaler 8 (0.5us при 16MHz)
	TIMSK1 = (1 << OCIE1A);     // Enable compare match interrupt
}

/************************************************************************/
/*       Инициализация драйвера                                         */
/************************************************************************/
void motor_init()
{
	DDRB |= (1 << ENA_PIN) | (1 << DIR_PIN) | (1 << STEP_PIN);
	PORTB |= (1 << ENA_PIN);    // Двигатель выключен по умолчанию
	timer1_init();
}

/************************************************************************/
/*       Прерывание для генерации STEP импульсов                        */
/************************************************************************/
ISR(TIMER1_COMPA_vect)
{
	static uint16_t accel_counter = 0;
	static uint8_t step_phase = 0;
	
	if (target_steps != 0)
	{
		if (step_phase == 0)
		{
			// Генерация импульса STEP
			PORTB |= (1 << STEP_PIN);
			step_phase = 1;
			
			// Обновление позиции
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
			
			// Адаптивное изменение скорости
			accel_counter++;
			if (accel_counter > 50)
			{  // Каждые 50 шагов
				accel_counter = 0;
				
				// Разгон в начале движения
				if (abs(target_steps) > (abs(target_steps) + current_position)/3)
				{
					if (step_delay > MIN_DELAY) step_delay -= 2;
				}
				// Торможение в конце движения
				else
				{
					if (step_delay < MAX_DELAY) step_delay += 5;
				}
			}
		}
		
		OCR1A = step_delay;  // Установка новой скорости
	}
}

/************************************************************************/
/*       Функция перемещения                                            */
/************************************************************************/
void move_to_position(int32_t position)
{
	// Расчет необходимых шагов
	target_steps = (position * MICROSTEPS) - current_position;
	
	// Установка направления
	if (target_steps >= 0)
	{
		PORTB |= (1 << DIR_PIN);
	}
	else
	{
		PORTB &= ~(1 << DIR_PIN);
	}
	
	// Включение драйвера
	//PORTB &= ~(1 << ENA_PIN);
	PORTB |= (1 << ENA_PIN);
	// Сброс параметров движения
	step_delay = MAX_DELAY;
	movement_flags = 0;
	
	// Ждем завершения движения
	while (target_steps != 0);
	
	// Выключение драйвера (опционально)
	// PORTB |= (1 << ENA_PIN);
}

/************************************************************************/
/*      Быстрое перемещение без плавного разгона                        */
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
/*      Пример использования                                            */
/************************************************************************/
int main()
{
	lcd_init();
	 lcd_print("Hello, HD44780!");
	 lcd_command(0xC0);  // Переход на 2-ю строку
	 lcd_print("RS=PB0, EN=PB1");
	 
	motor_init();
	sei();  // Разрешить прерывания
	
	// Конфигурация пинов кнопок (должна быть в отдельном файле)
	// buttons_init();
	
	while(1)
	{
		// Пример движения:
		move_to_position(1600);  // 1 оборот (200*8 микрошагов)
		_delay_ms(500);
		move_to_position(0);     // Возврат в 0
		_delay_ms(500);
		
		// Здесь можно добавить обработку кнопок из отдельного файла
		// uint8_t btn = read_button();
		// if (btn == RIGHT_BTN) move_to_position(100);
	}
}