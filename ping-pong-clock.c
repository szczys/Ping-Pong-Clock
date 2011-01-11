#define F_CPU 8000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#define SHIFTDDR DDRC
#define SHIFTPORT PORTC
#define SCK (1<<PC0)
#define RCK (1<<PC1)
#define SI (1<<PC2)
#define SHIFTMASK (SCK | RCK | SI)

#define COLDDR DDRD
#define COLPORT PORTD
#define COL0 (1<<PD4)
#define COL1 (1<<PD5)
#define COL2 (1<<PD6)
#define COL3 (1<<PD7)
#define COLMASK (COL0 | COL1 | COL2 | COL3)

//Definitions for lighting up numbers
#define digit0 0b01
#define digit1 0b1111111100000000
#define digit2 0b0000000011111111
#define digit3 0b0101010101010101

const unsigned int getDigit[] = {
  0b0111111001111110,	//0
  0b0101001000010100,	//1
  0b0110111001110111,	//2
  0b0101111001110101,	//3
  0b0101001001011111,	//4
  0b0101111001111011,	//5
  0b0111111001111011,	//6
  0b0101001001110100,	//7
  0b0111111001111111,	//8
  0b0101111001111111,	//9

  //Used for Hours Tens with dividing colon data
  0b0000000000000000,	//all LEDs off
  0b0010000000001000,	//Colon illuminated
  0b0101001000010100,	//One without colon
  0b0111001000011100	//One with colon
};

unsigned char col_list[4] = { COL0, COL1, COL2, COL3 };

//Display variables
unsigned char col_tracker = 0;	//Used for column scanning
volatile unsigned char hour_tens = 10;
volatile unsigned char hour_ones = 3;
volatile unsigned char minute_tens = 0;
volatile unsigned char minute_ones = 4;

volatile unsigned char *time_digits[] = {
  &minute_ones,
  &minute_tens,
  &hour_ones,
  &hour_tens
};

void init(void) {
  SHIFTDDR |= SHIFTMASK;	//Set pins as outputs
  COLDDR |= COLMASK;	//Set pins as outputs

  SHIFTPORT &= ~SHIFTMASK;	//Set all pins low
  COLPORT &= ~COLMASK;	//Set all pins low
}

void initTimers(void) {
  cli();

  //Timer0 for display scanning
  TIMSK0 |= (1<<TOIE0);			//Enable overflow interrupt
  TCCR0B |= (1<<CS01) | (1<<CS00);	//Start timer, prescale 64

  //Timer1 for 1 second timer

  
  sei();
}

void delay_ms(int c) {
  while(c--) {
    _delay_ms(1);
  }
}

void shift(unsigned int data) {
  SHIFTPORT &= ~SHIFTMASK; //All pins low

  for (unsigned char i=0; i<16; i++) {
    SHIFTPORT &= ~SCK; //Clock low

    //Set data pin
    if (data & (1<<i)) SHIFTPORT |= SI; 
    else SHIFTPORT &= ~SI;

    SHIFTPORT |= SCK; //Clock high
  }
  //SHIFTPORT |= RCK; //RCK Low
  //NOTE: latching will be handled by interrupt to keep LEDs lit as long as possible
}

void incMinute(void) {
  //NOTE: hours tens will be 10, 11, 12, or 13 because the flashing colon
  //      is driven by the same digit. See getDigit[] above.
  if (++minute_ones > 9) {
    minute_ones = 0;
    if (++minute_tens > 5) {
      minute_tens = 0;
      if ((hour_tens == 12) || (hour_tens == 13)) { 
        if (++hour_ones > 2) {
          hour_ones = 1;
          hour_tens = 10;	//10 represents all LEDs off
        }
      }
      else {
        if (++hour_ones > 9) {
          hour_ones = 0;
          hour_tens = 12;
        }
      }
    }
  }
}

int main(void) {
  init();
  initTimers();

  unsigned char seconds = 0;
  
  while(1) {
    delay_ms(1000);

    //blink the colon
    if ((hour_tens == 12) || (hour_tens == 10)) ++hour_tens;
    else --hour_tens;

    //count seconds
    if (++seconds > 59) {
      seconds = 0;
      incMinute();
    }

    
/*
    for (unsigned char j=0; j<3; j++) {
      COLPORT &= ~COLMASK;
      COLPORT |= col_list[j];

      for (unsigned char i=0; i<10; i++) {
        shift(getDigit[i]);
        delay_ms(1000);
      }
   }
    
    COLPORT &= ~COLMASK;
    shift(getDigit[1]);
    COLPORT |= col_list[3];
    delay_ms(1000);
    shift((1<<13) | (1<<3));
    delay_ms(1000);
*/

  /*
  COLPORT &= ~ COLMASK; //All transistors low
  COLPORT |= col_list[col_tracker];
  if (++col_tracker > 3) col_tracker = 0;
  delay_ms(1000);
  */

  }  
}

ISR(TIMER0_OVF_vect) {
  shift(getDigit[*(time_digits[col_tracker])]);
  COLPORT &= ~COLMASK;			//Turn off columns
  SHIFTPORT |= RCK;			//Latch data
  COLPORT |= col_list[col_tracker];	//turn on next column
  if (++col_tracker > 3) col_tracker = 0;	//preload column for next interrupt
}

