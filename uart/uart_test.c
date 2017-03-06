
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

int led;
//8000000/(1/.8e-6) = 6.4
#define HIGH1 0x6
#define LOW1 0x4

void pwm_init(void){
  DDRD |= (1 << DDD6);// && (1 << DD;
  // PD6 is now an output

  //Reset
  PORTD &= ~(1<<PD6);
  _delay_us(50);

  //PWM setup
  //TCCR0A |= 0x7; //Set PWM fast

  OCR0A = HIGH1; //Set TOP
  TCCR0A |= (1 << COM0A0);
  TCCR0A |= (1 << WGM01);
  TCCR0A &= ~(1 << WGM00);
  TCCR0A &= ~(1 << WGM02);
  TCCR0A |= (1 << FOC0A);

  TCCR0A |= (1 << CS00);
  TCCR0A &= ~(1 << CS01);
  TCCR0A &= ~(1 << CS02);

  PRR &= ~(1 << PRTIM0);
  TIMSK0 |= (1 << OCIE0A); //Enable interrupts
}

ISR(TIMER0_COMPA_vect , ISR_NAKED) {
  if(OCR0A == HIGH1)
  {
    PORTB &= ~(1<<0);
    OCR0A = LOW1;
  }
  else
  {
    OCR0A = HIGH1;
  }
  PORTB |= (1<<0);
  //TIMSK0 |= (1 << OCIE0A); //Enable interrupts
  //TIFR0 |=  0x2;
  reti();
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
  PORTB &= ~(1<<0);

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

  sei();
  int counter = 0;
  while(1) {
    if(TCNT0 < 0x7F)
    {
      TCNT0 = 0;
      counter++;
    }
    if(counter % 0xF000 < 0x7000)
      PORTB &= ~(1<<1);
    else
      PORTB |= (1<<1);
  }
}
