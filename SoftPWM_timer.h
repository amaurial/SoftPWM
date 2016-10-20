/* $Id: SoftPWM_timer.h 116 2010-06-28 23:31:02Z bhagman@roguerobotics.com $

  A Software PWM Library

  Simple timer abstractions by Paul Stoffregen (paul at pjrc dot com)

    This library is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>

// allow certain chips to use different timers
#define USE_TIMER2

// for each timer, these macros define how to actually use it
#define SOFTPWM_TIMER_INTERRUPT    TIMER2_COMPA_vect
#define SOFTPWM_TIMER_INTERRUPTB    TIMER2_COMPB_vect
#define SOFTPWM_TIMER_SET(val)     (TCNT2 = (val))
#define SOFTPWM_TIMER_INIT(ocrA, ocrB) ({\
  TIFR2 = (1 << TOV2);    /* clear interrupt flag */ \
  TCCR2B = (1 << CS21);   /* start timer (ck/256 prescalar) */ \
  TCCR2A = (1 << WGM21);  /* CTC mode */ \
  OCR2A = (ocrA);          /* We want to have at least 30Hz or else it gets choppy */ \
  OCR2B = (ocrB);          /* We want to have at least 30Hz or else it gets choppy */ \
  TIMSK2 = (1 << OCIE2A); /* enable timer2 output compare match interrupt */ \
  TIMSK2 |= (1 << OCIE2B); /* enable timer2 output compare match interrupt */ \
})

