/*
|| @author         Paul Stoffregen (paul at pjrc dot com)
|| @contribution   Brett Hagman <bhagman@wiring.org.co>
|| @url            http://wiring.org.co/
||
|| @description
|| | A Software PWM Library
|| |
|| | Simple timer abstractions.
|| #
||
|| @license Please see the accompanying LICENSE.txt file for this project.
||
|| @name Software PWM Library support
|| @type Library support
|| @target Atmel AVR 8 Bit
||
|| @version 1.0.1
||
*/

#include <avr/io.h>
#include <avr/interrupt.h>

// allow certain chips to use different timers
#if defined(__AVR_ATmega32U4__)
#define USE_TIMER4_HS // Teensy 2.0 lacks timer2, but has high speed timer4 :-)
#elif defined(__arm__) && defined(TEENSYDUINO)
#define USE_INTERVALTIMER  // Teensy 3.x has special interval timers :-)
#else
// default value is TIMER2, but we are using 0 because IRremote and RF69 use timer 1 and timer 2
#define USE_TIMER0
#endif

// for each timer, these macros define how to actually use it
#if defined(USE_TIMER2)
#define SOFTPWM_TIMER_INTERRUPT    TIMER2_COMPA_vect
#define SOFTPWM_TIMER_SET(val)     (TCNT2 = (val))
#define SOFTPWM_TIMER_INIT(ocr) ({\
  TIFR2 = (1 << TOV2);    /* clear interrupt flag */ \
  TCCR2B = (1 << CS21);   /* start timer (ck/8 prescalar) */ \
  TCCR2A = (1 << WGM21);  /* CTC mode */ \
  OCR2A = (ocr);          /* We want to have at least 30Hz or else it gets choppy */ \
  TIMSK2 = (1 << OCIE2A); /* enable timer2 output compare match interrupt */ \
})
#elif defined(USE_TIMER0)
#define SOFTPWM_TIMER_INTERRUPT    TIMER0_COMPA_vect
#define SOFTPWM_TIMER_INTERRUPTB   TIMER0_COMPB_vect
#define SOFTPWM_TIMER_SET(val)     (TCNT0 = (val))
#define SOFTPWM_TIMER_INIT(ocrA, ocrB) ({\
  TIFR0 = (1 << TOV2);    /* clear interrupt flag */ \
  TCCR0B = (1 << CS01);   /* start timer (ck/8 prescalar) */ \
  TCCR0A = (1 << WGM01);  /* CTC mode */ \
  OCR0A = (ocrA);          /* We want to have at least 30Hz or else it gets choppy */ \
  OCR0B = (ocrB);          /* We want to have at least 30Hz or else it gets choppy */ \
  TIMSK0 = (1 << OCIE0A); /* enable timer2 output compare match interrupt */ \
  TIMSK0 |= (1 << OCIE0B); /* enable timer2 output compare match interrupt */ \
})
#elif defined(USE_TIMER4_HS)
#define SOFTPWM_TIMER_INTERRUPT    TIMER4_COMPA_vect
#define SOFTPWM_TIMER_SET(val)     (TCNT4 = (val))
#define SOFTPWM_TIMER_INIT(ocr) ({\
  TCCR4A = 0; \
  TCCR4B = 0x04; /* CTC Mode */\
  TCCR4C = 0; \
  TCCR4D  = 0; \
  TCCR4E  = 0; \
  OCR4C  = 0; \
  OCR4C  = (ocr); \
  TIMSK4  = (1 << OCIE4A); \
})
#elif defined(USE_INTERVALTIMER)
#define SOFTPWM_TIMER_INTERRUPT    softpwm_interval_timer
#ifdef ISR
#undef ISR
#endif
#define ISR(f) void f(void)
#define SOFTPWM_TIMER_SET(val)
#define SOFTPWM_TIMER_INIT(ocr) ({\
  IntervalTimer *t = new IntervalTimer(); \
  t->begin(softpwm_interval_timer, 1000000.0 / (float)(SOFTPWM_FREQ * 256)); \
})
#endif

