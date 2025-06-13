#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <LCD.h>
#include <aes.h>

#define LM35_PIN 0 // ADC0
#define PASS_LENGTH 20
#define PASSWORD "2222\0"

// AES encryption key
const uint8_t aes_key[16] = "0123456789abcdef"; // 16-byte key for AES-128

void init_ports(void);
void init_pwm(void);
uint16_t read_temperature(void);
uint8_t set_motor_speed(uint8_t motor, uint8_t speed);
int password_verification(void);
char *receive_data(void);
void send_data(char data[]);
void receive_pass(uint8_t data[], uint8_t length);
void send_pass(uint8_t data[], uint8_t length);
void ADC_init(void);
uint8_t ADC_read(uint8_t channel);
uint8_t handle_motors(uint8_t speed);
void handle_critical_error(void);
void lockout_system(void);
void LCD_display(char display[]);
void LCD_display_char(char display);
void LCD_clear(void);
void init_usart(void);
void send_motor_status(uint8_t motor_status, uint8_t speed);
void set_frequency(uint16_t frequency);
void aes_encrypt(uint8_t *input, const uint8_t *key);
void aes_decrypt(uint8_t *input, const uint8_t *key);

int main(void)
{
  ADC_init();
  init_LCD();
  sei(); // Enable global interrupts

  while (1)
  {
    char verified[PASS_LENGTH];
    int verify = password_verification();
    verified[0] = verify + '0';
    verified[1] = '\0';

    // Display the verified status on the LCD
    LCD_clear();

    // Encrypt the verified status
    uint8_t encrypted_verified[16] = {0};
    strncpy((char *)encrypted_verified, verified, 16);
    aes_encrypt(encrypted_verified, aes_key);

    send_pass(encrypted_verified, 16);

    // If password is correct
    while (verify == 1)
    {
      init_pwm();
      uint16_t temperature = read_temperature();
      uint8_t speed = (temperature / 10) * 25;

      // Limit motor speed to 255 (max PWM value)
      if (speed > 255)
        speed = 255;

      uint8_t motor_status = handle_motors(speed);

      // Send motor status to Master
      char *send_motor = receive_data();
      if (strcmp(send_motor, "Motor\0") == 0)
      {
        uint8_t duty_cycle = (speed * 100) / 255;
        send_motor_status(motor_status, duty_cycle);
      }

      // Send motor status to Master
      if (strcmp(send_motor, "Temp\0") == 0)
      {
        char temp_str[4];
        snprintf(temp_str, sizeof(temp_str), "%d", temperature);
        send_data(temp_str);
      }

      free(send_motor);
    }
  }
}

// Initialize PWM for motor control
void init_pwm(void)
{
  // Set up Timer0, Timer1, Timer2 for PWM
  DDRD = 0xAF;
  DDRB |= (1 << PB3);

  TCCR0 |= (1 << WGM00) | (1 << WGM01) | (1 << COM01) | (1 << CS01); // Fast PWM, prescaler 8
  TCCR1A |= (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B |= (1 << WGM12) | (1 << CS11);                              // Fast PWM, prescaler 8
  TCCR2 |= (1 << WGM20) | (1 << WGM21) | (1 << COM21) | (1 << CS21); // Fast PWM, prescaler 8
}

void ADC_init(void)
{
  DDRA = 0x00; // Set PORTA as input for temperature sensor
  // To read the higher 8 bits
  ADMUX = (1 << REFS0) | (0 << REFS1) | (1 << ADLAR);

  // Enable the ADC by setting ADEN
  // Set the ADC prescaler to 8 by setting ADPS2 and ADPS1
  // Do not start the conversion here
  ADCSRA = (1 << ADEN) | (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Read temperature from sensor
uint16_t read_temperature(void)
{
  uint8_t adc_value;
  adc_value = ADC_read(LM35_PIN);
  return (adc_value * 500) / 256; // Convert the ADC value to temperature in Celsius
}

uint8_t ADC_read(uint8_t channel)
{
  // Clear lower bits of ADMUX and select the ADC channel
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);

  // Start the conversion by setting ADSC
  ADCSRA |= (1 << ADSC);

  // Wait for the conversion to complete until ADSC bit in ADCSRA is 0
  while (ADCSRA & (1 << ADSC))
    ;

  // Return the 8-bit result from ADCH
  return ADCH;
}

// Set motor speed using PWM
uint8_t set_motor_speed(uint8_t motor, uint8_t speed)
{
  switch (motor)
  {
  case 1:
    OCR0 = speed;
    return OCR0;

  case 2:
    OCR1A = speed;
    return OCR1A;

  case 3:
    OCR2 = speed;
    return OCR2;

  default:
    // Invalid motor number
    return 0;
  }
}

void init_usart(void)
{
  // Set baud rate
  unsigned int baud = 9600;
  unsigned int ubrr = F_CPU / 16 / baud - 1;
  UBRRH = (unsigned char)(ubrr >> 8);
  UBRRL = (unsigned char)ubrr;

  // Enable transmitter and receiver
  UCSRB = (1 << RXEN) | (1 << TXEN);

  // Set frame format: 8 data bits, 1 stop bit
  UCSRC = (1 << UCSZ1) | (1 << UCSZ0);
}

char *receive_data(void)
{
  char *data = (char *)malloc(PASS_LENGTH * sizeof(char));
  if (data == NULL)
  {
    return NULL; // Return NULL if memory allocation fails
  }

  for (int i = 0; i < PASS_LENGTH; i++)
  {
    while (!(UCSRA & (1 << RXC)))
      ;
    // Wait for data received
    data[i] = UDR;
    if (data[i] == '\0')
    {
      break;
    }
  }

  return data;
}

void receive_pass(uint8_t data[], uint8_t length)
{
  for (uint8_t i = 0; i < length; i++)
  {
    while (!(UCSRA & (1 << RXC)))
      ;
    data[i] = UDR;
  }
}

void send_pass(uint8_t data[], uint8_t length)
{
  for (uint8_t i = 0; i < length; i++)
  {
    while (!(UCSRA & (1 << UDRE)))
      ;
    UDR = data[i]; // data register => Send Data
  }
}

int password_verification(void)
{
  static int incorrect_attempts = 0;
  init_usart();
  // Buffer to hold received password
  uint8_t received_password[16] = {0};
  receive_pass(received_password, 16);

  // Decrypt the received password
  aes_decrypt(received_password, aes_key);

  // Example correct password: "1234"
  if (strncmp((char *)received_password, PASSWORD, PASS_LENGTH) != 0)
  {
    incorrect_attempts++;
    if (incorrect_attempts >= 3)
    {
      lockout_system();
      incorrect_attempts = 0;
    }
    return 0; // Password incorrect
  }

  incorrect_attempts = 0;
  return 1; // Password correct
}

void send_data(char data[])
{
  for (int i = 0; i < PASS_LENGTH; i++)
  {
    while (!(UCSRA & (1 << UDRE)))
      ;
    UDR = data[i]; // data register => Send Data
    if (data[i] == '\0')
    {
      break;
    }
  }
}

void lockout_system(void)
{
  send_data("Lock");
}

uint8_t handle_motors(uint8_t speed)
{
  // Check motor statuses and adjust duty cycles if needed
  uint8_t motor_status = 0x07; // Assuming all motors are working initially

  // Example failure simulation: let's say motor 2, 3 fails
  motor_status = 0x01;
  uint8_t speed1, speed2, speed3;

  // Adjust duty cycles if motors fail
  if (motor_status == 0x07)
  {
    speed1 = set_motor_speed(1, speed);
    speed2 = set_motor_speed(2, speed);
    speed3 = set_motor_speed(3, speed);
  }
  else if (motor_status == 0x06) // Motor 1 and 2 working
  {
    speed1 = set_motor_speed(1, speed + speed / 2);
    speed2 = set_motor_speed(2, speed + speed / 2);
  }
  else if (motor_status == 0x05) // Motor 1 and 3 working
  {
    speed1 = set_motor_speed(1, speed + speed / 2);
    speed3 = set_motor_speed(3, speed + speed / 2);
  }
  else if (motor_status == 0x03) // Motor 2 and 3 working
  {
    speed2 = set_motor_speed(2, speed + speed / 2);
    speed3 = set_motor_speed(3, speed + speed / 2);
  }
  else if (motor_status == 0x04) // Only motor 3 working
  {
    speed3 = set_motor_speed(3, speed * 3);
  }
  else if (motor_status == 0x02) // Only motor 2 working
  {
    speed2 = set_motor_speed(2, speed * 3);
  }
  else if (motor_status == 0x01) // Only motor 1 working
  {
    speed1 = set_motor_speed(1, speed * 3);
  }
  else
  {
    handle_critical_error(); // All motors failed
  }

  int dutyCycle1 = (speed1 * 100) / 255;
  int dutyCycle2 = (speed2 * 100) / 255;
  int dutyCycle3 = (speed3 * 100) / 255;

  if (dutyCycle1 > 100 || dutyCycle2 > 100 || dutyCycle3 > 100)
  {
    handle_critical_error(); // Duty cycles exceeded the limit
    motor_status = 0x08;
  }

  return motor_status;
}

void handle_critical_error(void)
{
  // Speaker
  DDRD |= (1 << PD4);
  PORTD = 0x10;
  set_frequency(500);
  _delay_ms(400);
  DDRD &= (0 << PD4);
}

void send_motor_status(uint8_t motor_status, uint8_t duty_cycle)
{
  char motor_str[2];
  motor_str[0] = motor_status + '0'; // Convert to char
  motor_str[1] = '\0';

  char duty_str[4];
  snprintf(duty_str, sizeof(duty_str), "%d", duty_cycle);

  send_data(motor_str);
  send_data(duty_str);
}

void LCD_display(char display[])
{
  for (int i = 0; i < strlen(display); i++)
  {
    LCD_write(display[i]);
    _delay_ms(5);
  }
}

void LCD_display_char(char display)
{
  LCD_write(display);
  _delay_ms(5);
}

void LCD_clear(void)
{
  LCD_cmd(0x01); // Clear LCD screen
}

// Speaker
void set_frequency(uint16_t frequency)
{
  // Calculate the OCR value for the given frequency
  uint16_t ocr_value = (F_CPU / (2 * 64 * frequency)) - 1;
  OCR1B = ocr_value;
}

void aes_encrypt(uint8_t *input, const uint8_t *key)
{
  struct AES_ctx ctx;
  AES_init_ctx(&ctx, key);
  AES_ECB_encrypt(&ctx, input);
}

void aes_decrypt(uint8_t *input, const uint8_t *key)
{
  struct AES_ctx ctx;
  AES_init_ctx(&ctx, key);
  AES_ECB_decrypt(&ctx, input);
}