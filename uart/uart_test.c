
/* some includes */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>

#include "Adafruit_NeoPixel.h"

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

void pwm_init(void){
  DDRD |= (1 << DDD6);
  // PD6 is now an output

  //Reset
  PORTD &= ~(1<<PD6);
  _delay_us(50);

  //PWM setup
  //TCCR0A |= 0x7; //Set PWM fast

  //8000000/(1/.8e-6) = 6.4
  OCR0A = 0x6; //Set TOP
  PORTD |= (1<<PD6); //Set output to 1
  TCCR0A |= 0x40; //Set toggle mode

  TCNT0 = 0;
  TIMSK0 |= (1<<OCIE0A); //Enable interrupts
}

ISR(TIM0_COMPA_vect , ISR_NAKED) {
  if(OCR0A == 0x6)
  {
    OCR0A = 0x4;
  }
  else
  {
    OCR0A = 0x6;
  }
}

/*
ISR(TIMER0_OVF_vect, ISR_NAKED) {
   TIMER_COUNTER = !TIMER_COUNTER;
   if (TIMER_COUNTER && (COLOR_COUNTER > 0)) {
      PORTD = COLOR%2;
      COLOR >>= 1;
      COLOR_COUNTER -= 1;
   }
}
*/

/*

void interrupt22 adcInterrupt(void) {
   printf("Reached the ADC interrupt...");
}
*/

void c_function(Adafruit_NeoPixel* Adafruit_NeoPixel)
{
  cplusplus_callback_function(Adafruit_NeoPixel);
}

int main(void)
{

  /* Setup serial port */
  uart_init();
  stdout = &uart_output;
  stdin  = &uart_input;

  uint8_t SENSOR_MODE = 0;
  uint8_t ACTUATOR_MODE = 1;
  uint8_t BOTH_MODE = 2;
  uint8_t mode = ACTUATOR_MODE;


  char ch;

  // Setup ports
  DDRB |= (1<<1) | (1<<0);
  PORTB |= (1<<0);
  PORTB &= ~(1<<1);

  // enable interrupts
  //TIMSK1 |= 1;

  // ADC interrupt
  //ADATE |= 1;

  // Trigger select bits for ADC
  /*
  ADTS2 |= 0;
  ADTS1 |= 0;
  ADTS0 |= 0;
  */


  /* Print hello and then echo serial
   ** port data while blinking LED */
  printf("Hello world!\r\n");

  pwm_init();

  adafruit_NeoPixel led = adafruit_NeoPixel(LED_COUNT, PIN, NEO_GRB + NEO_KHZ800);
  while(1) {
    ch = getchar();
    PORTD |= 1<<PD6; //just see if we get light
    if (mode == SENSOR_MODE) {
      putchar('s');
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
        neoPixel_setPixelColor(led,0,RED);
        neoPixel_show(led);
      } else if (ch == '6') {
        neoPixel_setPixelColor(led,0,GREEN);
        neoPixel_show(led);
      } else if (ch == '7') {
        neoPixel_setPixelColor(led,0,BLUE);
        neoPixel_show(led);
      } else if (ch == '8') {
        neoPixel_setPixelColor(led,0,0);
        neoPixel_show(led);
      }

    } else {
      putchar('b');
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
