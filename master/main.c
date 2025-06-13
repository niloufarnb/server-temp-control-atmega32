#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <LCD.h>
#include <aes.h>

#define PASS_LENGTH 20

// Keypad mapping
const char keypad[4][3] = {
    {'7', '8', '9'},
    {'4', '5', '6'},
    {'1', '2', '3'},
    {'*', '0', 'E'} // 'E' for Enter
};

// AES encryption key
const uint8_t aes_key[16] = "0123456789abcdef"; // 16-byte key for AES-128

// Function prototypes
void init_ports(void);
void init_keypad(void);
void send_data(char data[]);
char *receive_data(void);
void send_pass(uint8_t data[], uint8_t length);
void receive_pass(uint8_t data[], uint8_t length);
char key_pressed(void);
void lockout_system(void);
void write_temperature(void);
void write_motor_status(void);
void init_usart(void);
void motor_check(char *motors_status, char *speed);
void aes_encrypt(uint8_t *input, const uint8_t *key);
void aes_decrypt(uint8_t *input, const uint8_t *key);

int main(void)
{
    //  Initialization
    init_ports();
    init_LCD();

    // System Locked
    LCD_clear();
    LCD_display("Password");
    _delay_ms(1);
    LCD_clear();

    char pass[PASS_LENGTH + 1]; // Array to store password + null terminator
    int choose = 0;             // for menue

    sei();

    while (1)
    {
        char key = 0;
        int pass_length = 0;
        init_keypad();

        // Get password from keypad
        while (key != 'E')
        {
            key = key_pressed();

            if ((key != 0) && (key != 'E'))
            {
                LCD_display_char(key);
                if (pass_length < PASS_LENGTH)
                {
                    pass[pass_length++] = key;
                    pass[pass_length] = '\0';
                }
            }
        }
        LCD_clear();

        init_usart();

        // Encrypt the password
        uint8_t encrypted_pass[16] = {0};
        strncpy((char *)encrypted_pass, pass, 16);
        aes_encrypt(encrypted_pass, aes_key);

        send_pass(encrypted_pass, 16);

        uint8_t verify[16] = {0};
        receive_pass(verify, 16);

        // Decrypt the response
        aes_decrypt(verify, aes_key);

        // Password's correct
        if (verify[0] == '1')
        {
            LCD_clear();
            LCD_display("Unlocked!");
            _delay_ms(5);
            while (1)
            {
                LCD_clear();
                LCD_display("Main Menu:");
                _delay_ms(5);
                LCD_clear();
                LCD_display("1. Motor Status");
                _delay_ms(5);
                LCD_clear();
                LCD_display("2. Temperature");
                _delay_ms(5);
                LCD_clear();

                do
                {
                    choose = key_pressed() - '0'; // Convert char to int
                } while (choose != 1 && choose != 2);

                if (choose == 1)
                {
                    LCD_display("Motor Status");
                    _delay_ms(5);
                    LCD_clear();
                    write_motor_status();
                }
                else
                {
                    LCD_display("Temperature");
                    _delay_ms(5);
                    LCD_clear();
                    write_temperature();
                }
            }
        }
        else if (verify[0] == '0')
        {
            LCD_clear();
            LCD_display("Password's wrong");
            _delay_ms(5);
            LCD_clear();
        }
        else if (strcmp(verify, "Lock") == 0)
        {
            LCD_clear();
            LCD_display("30 seconds Lock");
            _delay_ms(30000);
            LCD_clear();
        }
    }
}

void init_ports(void)
{
    // Configure ports for LCD, keypad, and LEDs
    DDRA = 0xFF; // Set PORTA as output for LCD
    DDRD = 0x1C; // 0001-1100
}

void init_keypad(void)
{
    // Set rows as outputs
    DDRC |= 0x0F; // PC0-PC3 as outputs
    // Set columns as inputs
    DDRC &= ~0x70; // PC4-PC6 as inputs
    // Enable pull-up resistors on columns
    PORTC |= 0x70;
}

char key_pressed(void)
{
    for (uint8_t row = 0; row < 4; row++)
    {
        PORTC = ~(1 << row); // Set current row low
        _delay_ms(1);        // Small delay for stabilization

        for (uint8_t col = 0; col < 3; col++)
        {
            if (!(PINC & (1 << (col + 4))))
            {
                // Wait for key release (debouncing)
                while (!(PINC & (1 << (col + 4))))
                    ;
                return keypad[row][col]; // Return pressed key
            }
        }
    }
    return 0; // No key pressed
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

void send_data(char data[])
{
    for (int i = 0; i < PASS_LENGTH; i++)
    {
        UDR = data[i]; // data register
        if (data[i] == '\0')
        {
            break;
        }

        while (!(UCSRA & (1 << UDRE)))
            ; // interrupt flag
    }
}

char *receive_data(void)
{
    char *data = (char *)malloc(PASS_LENGTH * sizeof(char)); // Dynamically allocate memory
    if (data == NULL)
    {
        return NULL; // Return NULL if memory allocation fails
    }

    for (int i = 0; i < PASS_LENGTH; i++)
    {
        while (!(UCSRA & (1 << RXC)))
            ;
        data[i] = UDR;
        if (data[i] == '\0')
        {
            break;
        }
    }
    return data;
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

void receive_pass(uint8_t data[], uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        while (!(UCSRA & (1 << RXC)))
            ;
        data[i] = UDR;
    }
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

// Function to display temperature
void write_temperature(void)
{
    char temp[5] = "Temp";
    send_data(temp);
    char *tempreture_str = receive_data();
    uint8_t tempreture = atoi(tempreture_str);
    LCD_write_number(tempreture);
}

// Function to display motor status
void write_motor_status(void)
{
    char motor[6] = "Motor";
    send_data(motor);
    char *motors_status = receive_data();
    char *duty_cycle = receive_data();
    motor_check(motors_status, duty_cycle);
}

void motor_check(char *motors_status, char *duty_cycle)
{

    uint8_t duty = atoi(duty_cycle);
    DDRB = 0x07;
    if (strcmp(motors_status, "8") == 0)
    {
        PORTB = 0x04;
        LCD_display("Duty cycles exceeded the limit");
        _delay_ms(2);
        LCD_clear();
    }
    else if (strcmp(motors_status, "7") == 0) // All of the motors are working fine
    {
        PORTB = 0x01;
        LCD_display("Motor1: Good ");
        LCD_write_number(duty);
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor2: Good ");
        LCD_write_number(duty);
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor3: Good ");
        LCD_write_number(duty);
        _delay_ms(2);
        LCD_clear();
    }

    else if (strcmp(motors_status, "6") == 0) // Motor 1 and 2 working
    {
        PORTB = 0x02;
        LCD_display("Motor1: Good ");
        LCD_write(duty + duty / 2);
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor2: Good ");
        LCD_write(duty + duty / 2);
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor3: Failed");
        _delay_ms(2);
        LCD_clear();
    }
    else if (strcmp(motors_status, "5") == 0) // Motor 1 and 3 working
    {
        PORTB = 0x02;
        LCD_display("Motor1: Good ");
        LCD_write(duty + duty / 2);
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor2: Failed");
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor3: Good ");
        LCD_write(duty + duty / 2);
        _delay_ms(2);
        LCD_clear();
    }
    else if (strcmp(motors_status, "3") == 0) // Motor 2 and 3 working
    {
        PORTB = 0x02;
        LCD_display("Motor1: Failed");
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor2: Good ");
        LCD_write(duty + duty / 2);
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor3: Good ");
        LCD_write(duty + duty / 2);
        _delay_ms(2);
        LCD_clear();
    }
    else if (strcmp(motors_status, "4") == 0) // Only motor 3 working
    {
        PORTB = 0x02;
        LCD_display("Motor1: Failed");
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor2: Failed");
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor3: Good ");
        LCD_write(duty * 3);
        _delay_ms(2);
        LCD_clear();
    }
    else if (strcmp(motors_status, "2") == 0) // Only motor 2 working
    {
        PORTB = 0x02;
        LCD_display("Motor1: Failed");
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor2: Good ");
        LCD_write(duty * 3);
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor3: Failed");
        _delay_ms(2);
        LCD_clear();
    }
    else if (strcmp(motors_status, "1") == 0) // Only motor 1 working
    {
        PORTB = 0x02;
        LCD_display("Motor1: Good ");
        LCD_write(duty * 3);
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor2: Failed");
        _delay_ms(2);
        LCD_clear();

        LCD_display("Motor3: Failed");
        _delay_ms(2);
        LCD_clear();
    }
    else
    {
        PORTB = 0x04;
        LCD_display("All failed");
        _delay_ms(2);
        LCD_clear();
    }
}
