/*
|| @author         Brett Hagman <bhagman@wiring.org.co>
|| @contribution   Paul Stoffregen (paul at pjrc dot com)
|| @url            http://wiring.org.co/
|| @url            http://roguerobotics.com/
||
|| @description
|| | A Software PWM Library
|| | 
|| | Written by Brett Hagman
|| | http://www.roguerobotics.com/
|| | bhagman@roguerobotics.com, bhagman@wiring.org.co
|| |
|| | A Wiring (and Arduino) Library, for Atmel AVR8 bit series microcontrollers,
|| | to produce PWM signals on any arbitrary pin.
|| | 
|| | It was originally designed for controlling the brightness of LEDs, but
|| | could be adapted to control servos and other low frequency PWM controlled
|| | devices as well.
|| | 
|| | It uses a single hardware timer (Timer 2) on the Atmel microcontroller to
|| | generate up to 20 PWM channels (your mileage may vary).
|| | 
|| #
||
|| @license Please see the accompanying LICENSE.txt file for this project.
||
|| @notes
|| | Minor modification by Paul Stoffregen to support different timers.
|| |
|| #
||
|| @name Software PWM Library
|| @type Library
|| @target Atmel AVR 8 Bit
||
|| @version 1.0.1
||
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "SoftPWM.h"
#include "SoftPWM_timer.h"
#include <Arduino.h>

// see http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
// or https://eleccelerator.com/avr-timer-calculator/
//#if F_CPU
//#define SOFTPWM_FREQ 1000000UL // 1Mhertz
//#define SOFTPWM_OCR (F_CPU/(64UL*SOFTPWM_FREQ) - 1)
//#define SOFTPWM_OCRB (F_CPU/(64UL*SOFTPWM_FREQ))//100 hertz - overflow count of 9, total ticks 2500, remainder ticks 196
//#else

#define SOFTPWM_OCR 130
#define SOFTPWM_OCRB 130
//#endif

volatile uint8_t _isr_softcount = 0xff;
volatile uint16_t _isrb_count = 0; // we want 2500 ticks
volatile uint16_t _isr_milli = 0; // we want 2500 ticks
uint8_t _softpwm_defaultPolarity = SOFTPWM_NORMAL;
unsigned long custom_millis = 0;
unsigned long custom_micros = 0;

typedef struct
{
  // hardware I/O port and pin for this channel
  int8_t pin;
  uint8_t polarity;
  volatile uint8_t *outport;
  uint8_t pinmask;
  uint8_t pwmvalue;
  uint8_t checkval;
  uint8_t fadeuprate;
  uint8_t fadedownrate;
} softPWMChannel;

softPWMChannel _softpwm_channels[SOFTPWM_MAXCHANNELS];

unsigned long getCustomMillis(){        
    return custom_millis/10;
}

unsigned long getCustomMicros(){
    return custom_millis*10000;
}

fpointer func = 0;
ISR(SOFTPWM_TIMER_INTERRUPTB){    
    if (++_isrb_count == 50000){
        _isrb_count = 0;
        if (func == 0) return;
        else (func(1));
    }
}

// Here is the meat and gravy
/*
* this function will be called every millisecond, frequency  of 1 kHz
*/
ISR(SOFTPWM_TIMER_INTERRUPT)
{
    uint8_t i;
    int16_t newvalue;
    int16_t direction;
    //custom_micros++;
    //custom_millis++;    
    if(++_isr_softcount == 0) // this logic runs every 255ms
    {    
        // set all channels high - let's start again
        // and accept new checkvals
        for (i = 0; i < SOFTPWM_MAXCHANNELS; i++)
        {
        if (_softpwm_channels[i].fadeuprate > 0 || _softpwm_channels[i].fadedownrate > 0)
        {
            // we want to fade to the new value
            direction = _softpwm_channels[i].pwmvalue - _softpwm_channels[i].checkval;

            // we will default to jumping to the new value
            newvalue = _softpwm_channels[i].pwmvalue;

            if (direction > 0 && _softpwm_channels[i].fadeuprate > 0)
            {
            newvalue = _softpwm_channels[i].checkval + _softpwm_channels[i].fadeuprate;
            if (newvalue > _softpwm_channels[i].pwmvalue)
                newvalue = _softpwm_channels[i].pwmvalue;
            }
            else if (direction < 0 && _softpwm_channels[i].fadedownrate > 0)
            {
            newvalue = _softpwm_channels[i].checkval - _softpwm_channels[i].fadedownrate;
            if (newvalue < _softpwm_channels[i].pwmvalue)
                newvalue = _softpwm_channels[i].pwmvalue;
            }

            _softpwm_channels[i].checkval = newvalue;
        }
        else  // just set the channel to the new value
            _softpwm_channels[i].checkval = _softpwm_channels[i].pwmvalue;

        // now set the pin high (if not 0)
        if (_softpwm_channels[i].checkval > 0)  // don't set if checkval == 0
        {
            if (_softpwm_channels[i].polarity == SOFTPWM_NORMAL)
            *_softpwm_channels[i].outport |= _softpwm_channels[i].pinmask;
            else
            *_softpwm_channels[i].outport &= ~(_softpwm_channels[i].pinmask);
        }

        }
    }

    for (i = 0; i < SOFTPWM_MAXCHANNELS; i++)
    {
        if (_softpwm_channels[i].pin >= 0)  // if it's a valid pin
        {
        if (_softpwm_channels[i].checkval == _isr_softcount)  // if we have hit the width
        {
            // turn off the channel
            if (_softpwm_channels[i].polarity == SOFTPWM_NORMAL)
            *_softpwm_channels[i].outport &= ~(_softpwm_channels[i].pinmask);
            else
            *_softpwm_channels[i].outport |= _softpwm_channels[i].pinmask;
        }
        }
    }  
}



//void SoftPWMBegin(uint8_t defaultPolarity, fpointer f)
void SoftPWMBegin(uint8_t defaultPolarity)
{
  // We can tweak the number of PWM period by changing the prescalar
  // and the OCR - we'll default to ck/8 (CS21 set) and OCR=128.
  // This gives 1024 cycles between interrupts.  And the ISR consumes ~200 cycles, so
  // we are looking at about 20 - 30% of CPU time spent in the ISR.
  // At these settings on a 16 MHz part, we will get a PWM period of
  // approximately 60Hz (~16ms).

  uint8_t i;

  SOFTPWM_TIMER_INIT(SOFTPWM_OCR);
  //SOFTPWM_TIMER_INIT(SOFTPWM_OCR, SOFTPWM_OCRB);

  //func = f;

  for (i = 0; i < SOFTPWM_MAXCHANNELS; i++)
  {
    _softpwm_channels[i].pin = -1;
    _softpwm_channels[i].polarity = SOFTPWM_NORMAL;
    _softpwm_channels[i].outport = 0;
    _softpwm_channels[i].fadeuprate = 0;
    _softpwm_channels[i].fadedownrate = 0;
  }

  _softpwm_defaultPolarity = defaultPolarity;
}


void SoftPWMSetPolarity(int8_t pin, uint8_t polarity)
{
  uint8_t i;

  if (polarity != SOFTPWM_NORMAL)
    polarity = SOFTPWM_INVERTED;

  for (i = 0; i < SOFTPWM_MAXCHANNELS; i++)
  {
    if ((pin < 0 && _softpwm_channels[i].pin >= 0) ||  // ALL pins
       (pin >= 0 && _softpwm_channels[i].pin == pin))  // individual pin
    {
      _softpwm_channels[i].polarity = polarity;
    }
  }
}


void SoftPWMSetPercent(int8_t pin, uint8_t percent, uint8_t hardset)
{
  SoftPWMSet(pin, ((uint16_t)percent * 255) / 100, hardset);
}


void SoftPWMSet(int8_t pin, uint8_t value, uint8_t hardset)
{
  int8_t firstfree = -1;  // first free index
  uint8_t i;

  if (hardset)
  {
    SOFTPWM_TIMER_SET(0);
    _isr_softcount = 0xff;
  }

  // If the pin isn't already set, add it
  for (i = 0; i < SOFTPWM_MAXCHANNELS; i++)
  {
    if ((pin < 0 && _softpwm_channels[i].pin >= 0) ||  // ALL pins
       (pin >= 0 && _softpwm_channels[i].pin == pin))  // individual pin
    {
      // set the pin (and exit, if individual pin)
      _softpwm_channels[i].pwmvalue = value;

      if (pin >= 0) // we've set the individual pin
        return;
    }

    // get the first free pin if available
    if (firstfree < 0 && _softpwm_channels[i].pin < 0)
      firstfree = i;
  }

  if (pin >= 0 && firstfree >= 0)
  {
    // we have a free pin we can use
    _softpwm_channels[firstfree].pin = pin;
    _softpwm_channels[firstfree].polarity = _softpwm_defaultPolarity;
    _softpwm_channels[firstfree].outport = portOutputRegister(digitalPinToPort(pin));
    _softpwm_channels[firstfree].pinmask = digitalPinToBitMask(pin);
    _softpwm_channels[firstfree].pwmvalue = value;
//    _softpwm_channels[firstfree].checkval = 0;
    
    // now prepare the pin for output
    // turn it off to start (no glitch)
    if (_softpwm_defaultPolarity == SOFTPWM_NORMAL)
      digitalWrite(pin, LOW);
    else
      digitalWrite(pin, HIGH);
    pinMode(pin, OUTPUT);
  }
}

void SoftPWMEnd(int8_t pin)
{
  uint8_t i;

  for (i = 0; i < SOFTPWM_MAXCHANNELS; i++)
  {
    if ((pin < 0 && _softpwm_channels[i].pin >= 0) ||  // ALL pins
       (pin >= 0 && _softpwm_channels[i].pin == pin))  // individual pin
    {
      // now disable the pin (put it into INPUT mode)
      digitalWrite(_softpwm_channels[i].pin, 1);
      pinMode(_softpwm_channels[i].pin, INPUT);

      // remove the pin
      _softpwm_channels[i].pin = -1;
    }
  }
}


void SoftPWMSetFadeTime(int8_t pin, uint16_t fadeUpTime, uint16_t fadeDownTime)
{
  int16_t fadeAmount;
  uint8_t i;

  for (i = 0; i < SOFTPWM_MAXCHANNELS; i++)
  {
    if ((pin < 0 && _softpwm_channels[i].pin >= 0) ||  // ALL pins
       (pin >= 0 && _softpwm_channels[i].pin == pin))  // individual pin
    {

      fadeAmount = 0;
      if (fadeUpTime > 0)
        fadeAmount = 255UL * (SOFTPWM_OCR * 256UL / (F_CPU / 8000UL)) / fadeUpTime;

      _softpwm_channels[i].fadeuprate = fadeAmount;

      fadeAmount = 0;
      if (fadeDownTime > 0)
        fadeAmount = 255UL * (SOFTPWM_OCR * 256UL / (F_CPU / 8000UL)) / fadeDownTime;

      _softpwm_channels[i].fadedownrate = fadeAmount;

      if (pin >= 0)  // we've set individual pin
        break;
    }
  }
}
