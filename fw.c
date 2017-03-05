
/* some includes */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>

#define BAUD 115200
#include <util/setbaud.h>

void uart_init(void) {
   UBRR0H = UBRRH_VALUE;
   UBRR0L = UBRRL_VALUE;

#if USE_2X
   UCSR0A |= _BV(U2X0);
#else
   UCSR0A &= ~(_BV(U2X0));
#endif

   UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
   UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

void uart_putchar(char c) {
   loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
   UDR0 = c;
}

char uart_getchar(void) {
   loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
   return UDR0;
}

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);


int main(void)
{

   /* Setup serial port */
   uart_init();
   stdout = &uart_output;
   stdin  = &uart_input;

   char input;

   // Setup ports
   DDRB |= (1<<1) | (1<<0);
   PORTB |= (1<<0);
   PORTB &= ~(1<<1);

   /* Print hello and then echo serial
   ** port data while blinking LED */
   printf("Hello world!\r\n");
   while(1) {
      ch = getchar();
      if (mode == SENSOR_MODE) {
         /*float temp_val = getTemp();
         int float_size = 48;
         char[float_size] temp_val_string;
         snprintf(temp_val_string, float_size, "%f",temp_val);
         for (i = 0; i < float_size; i++) putch(temp_val_string[i]);
         putch('\n');*/
      } else if (mode == ACTUATOR_MODE) {
         putchar('a');
         putchar('\n');
         if (ch == '5') {
            color = 255<<16;
         } else if (ch == '6') {
            color = 255<<8;
         } else if (ch == '7') {
            color = 255;
         } else if (ch == '8') {
            color = 0;
         }
         PORTD = color;

      } else {
         
      }

      /* A bunch of if...else if... gives smaller code than switch...case ! */
      if (ch=='2') {
         mode = SENSOR_MODE;
      } else if (ch=='3') {
         mode = ACTUATOR_MODE;
      } else if (ch == '4') {
         mode = BOTH_MODE;
      }
   }
}
