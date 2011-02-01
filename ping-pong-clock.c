/*--------------------------------------------------------------------------
Ping-Pong Clock
Copyright (c) 2010 Mike Szczys

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------
  This project uses an ATmega168 and a DS3232 real time clock to display
  time. A multiplexed LED display is used with ping-pong balls as diffusers.
  There are four buttons that facilitate selecting between time and
  temperature being displayed, and setting the time and date.
    http://hackaday.com/2011/01/31/how-to-build-a-ping-pong-ball-display/
--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------
  Planned Improvements:
    I would like this clock to adjust for daylight savings time
    automatically. Future improvements will include a function to calculate
    the day of the week (for storage in the DS3232 RTC) and then to use a
    lookup table for DST adjustments to the displayed time.
--------------------------------------------------------------------------*/

#define F_CPU 8000000

#include <avr/io.h>
#include <avr/interrupt.h>

/***********************************************
* Conditional defines for different processors *
***********************************************/

//The microprocessor is selected by setting the
//  MCU definitions at the beginning of the makefile
//  (eg: MCU = atmega168)

#if defined(__AVR_ATmega8__)			//ATmega8

  //INT0 register definitions
  #define EXT_INT_CONTROL	MCUCR
  #define EXT_INT_SELECT 	GICR

  //Timer0 register definitions
  #define TIMER0_CTRL_REG	TCCR0
  #define TIMER0_INT_MASK	TIMSK

  //Timer2 register definitions
  #define TIMER2_CTRL_REG	TCCR2
  #define TIMER2_INT_MASK	TIMSK

#elif defined(__AVR_ATmega168__)		//ATmega168

  //INT0 register definitions
  #define EXT_INT_CONTROL	EICRA
  #define EXT_INT_SELECT 	EIMSK

  //Timer0 register definitions
  #define TIMER0_CTRL_REG	TCCR0B
  #define TIMER0_INT_MASK	TIMSK0

  //Timer2 register definitions
  #define TIMER2_CTRL_REG	TCCR2B
  #define TIMER2_INT_MASK	TIMSK2

#endif

/***********************************************
* End conditonal defines                       *
***********************************************/

//13 pixels address by two 595 shift registers
#define SHIFTDDR DDRC
#define SHIFTPORT PORTC
#define SCK (1<<PC0)
#define RCK (1<<PC1)
#define SI (1<<PC2)
#define SHIFTMASK (SCK | RCK | SI)

//Digits pulled low by NPN transistors
#define COLDDR DDRD
#define COLPORT PORTD
#define COL0 (1<<PD4)
#define COL1 (1<<PD5)
#define COL2 (1<<PD6)
#define COL3 (1<<PD7)
#define COLMASK (COL0 | COL1 | COL2 | COL3)

//i2c definitions
#define i2c_slave_address 0xD0 	//Address for DS3232
#define i2c_read (i2c_slave_address + 1)
#define i2c_write i2c_slave_address

//Register addresses for time and date settings
#define seconds_address	0x00
#define minutes_address	0x01
#define hours_address	0x02
#define date_address	0x04
#define month_address	0x05
#define year_address	0x06

//button definitions
#define KEY_DDR		DDRB
#define KEY_PORT	PORTB
#define KEY_PIN		PINB
#define KEY0		0	//Mode button
#define KEY1		3	//Next
#define KEY2		1	//+
#define KEY3		2       //-

//Prototypes
void init(void);
void initTimers(void);
unsigned char get_key_press(unsigned char key_mask);
void shift(unsigned int data);
void twi_start(unsigned char SlvAddr);
void twi_send_byte(unsigned char data);
void twi_stop(void);
char twi_read_ack(void);
char twi_read_nack(void);
unsigned char get_dec(unsigned char hex_encoded);
void syncTime(void);
void disp_temperature(void);
void increment_state(void);
void showSettings(void);
void saveSettings(void);
unsigned char ds3232_read_setting(unsigned char address);
void ds3232_write_setting(unsigned char address, unsigned char value);
void increment_settings_value(void);
void decrement_settings_value(void);

/*******************************

 Character set for 13-led digit.
 Numbers represent bit in an int:

 14,13,12,
 11,  ,10,
  9, 8, 6,
  5,    4,
  3, 2, 1,

 Bits 0,7,15 are unused

 Colon used bits 11, 5, of the
 tens digit for top and bottom dot.

*******************************/

const unsigned int getDigit[] = {

  0b0111111001111110,	//0
  0b0001010001010010,	//1
  0b0111011101101110,	//2
  0b0111010101011110,	//3
  0b0101111101010010,	//4
  0b0111101101011110,	//5
  0b0111101101111110,	//6
  0b0111010001010010,	//7
  0b0111111101111110,	//8
  0b0111111101011110,	//9

  //Used for Hours Tens with dividing colon data
  0b0000000000000000,	//all LEDs off
  0b0000100000100000,	//Colon illuminated
  0b0001010001010010,	//One without colon
  0b0001110001110010,	//One with colon

  0b0111111101000000,	//degree symbol 		(index: 14)
  0b0100101101111010,	// h 				(index: 15)
  0b0000010001010010,	// m (left leg for hour ten	(index: 16)
  0b0000111001110010,	// m (others for hour ones)	(index: 17)
  0b0001010001010010,	// M (right leg for hour tens	(index: 18)
  0b0101111001010010,	// M (others for hour ones)	(index: 19)
  0b0110111001111100,	// D 				(index: 20)
  0b0101111101011110	// Y				(index: 21)
};

//Lookup table for column I/O definitions
unsigned char col_list[4] = { COL0, COL1, COL2, COL3 };

//Debounce
unsigned char debounce_cnt = 0;
volatile unsigned char key_press;
unsigned char key_state;

//Display variables
unsigned char col_tracker = 0;	//Used for column scanning
volatile unsigned char seconds = 0;
volatile unsigned char hour_tens = 10;
volatile unsigned char hour_ones = 3;
volatile unsigned char minute_tens = 0;
volatile unsigned char minute_ones = 4;

//Used as a flag by the INT0 ISR
volatile unsigned char one_hz = 0;

//State definitions and variable
#define state_time 0
#define state_temperature 1
#define state_set 2
unsigned char state = state_time;
unsigned char set_tracker;

//Array of pointers to currently displayed digit values
volatile unsigned char *time_digits[] = {
  &minute_ones,
  &minute_tens,
  &hour_ones,
  &hour_tens
};

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Initialize I/O and communications
  PARAMS: None
  RETURNS: None
--------------------------------------------------------------------------*/
void init(void) {
  SHIFTDDR |= SHIFTMASK;	//Set pins as outputs
  COLDDR |= COLMASK;	//Set pins as outputs

  SHIFTPORT &= ~SHIFTMASK;	//Set all pins low
  COLPORT &= ~COLMASK;	//Set all pins low

  //i2c
  TWBR = 0x02;	//Set 400khz I2C clock without prescaler
		//  equation: f_cpu/(16 + (2 * TWBR))*prescaler

  //Enable 1 Hz output from DS3232
  twi_start(i2c_write);
  twi_send_byte(0x0e);	//Address for Control Register
  twi_send_byte(0x00);	//Set 1 Hz square wave output
  twi_stop();

  //Enable INT0 for tracking square wave
  EXT_INT_CONTROL |= (1<<ISC01);	// INT0 as input on falling edge
  EXT_INT_SELECT |= (1<<INT0);	// Enable interrupt

  //Setup Button
  KEY_DDR &= ~((1<<KEY0) | (1<<KEY1) | (1<<KEY2) | (1<<KEY3));
  KEY_PORT |= (1<<KEY0) | (1<<KEY1) | (1<<KEY2) | (1<<KEY3);	//enable pull-up resistor
  
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - 
  PARAMS: None
  RETURNS: None
--------------------------------------------------------------------------*/
void initTimers(void) {
  cli();

  //Timer0 for display scanning
  TIMER0_INT_MASK |= (1<<TOIE0);			//Enable overflow interrupt
  TIMER0_CTRL_REG |= (1<<CS01) | (1<<CS00);	//Start timer, prescale 64

  //Timer2 for buttons
  TIMER2_CTRL_REG |= 1<<CS22 | 1<<CS21 | 1<<CS20;	//Divide by 1024
  TIMER2_INT_MASK |= 1<<TOIE2;		//enable timer overflow interrupt
  
  sei();
}

/*--------------------------------------------------------------------------
  FUNC: 7/23/10 - Used to read debounced button presses
  PARAMS: A keymask corresponding to the pin for the button you with to poll
  RETURNS: A keymask where any high bits represent a button press
--------------------------------------------------------------------------*/
unsigned char get_key_press( unsigned char key_mask )
{
  cli();			// read and clear atomic !
  key_mask &= key_press;	// read key(s)
  key_press ^= key_mask;	// clear key(s)
  sei();
  return key_mask;
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Send data to shift register
  PARAMS: sixteen bits of data to be shifted into the register
  RETURNS: None
--------------------------------------------------------------------------*/
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

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - TWI (or i2c) start condition
  PARAMS: The address of the slave device
  RETURNS: None
--------------------------------------------------------------------------*/
void twi_start(unsigned char SlvAddr)
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA);   //Send start command
  while(!(TWCR & (1<<TWINT)));                  //Wait for bus to become ready
  TWDR = SlvAddr;                     //Write device address to data register
  TWCR = (1<<TWINT) | (1<<TWEN);                //Send device address
  while(!(TWCR & (1<<TWINT)));                  //Wait for bus to become ready
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Send one byte of data over TWI (or i2c)
  PARAMS: eight bits of data
  RETURNS: None
--------------------------------------------------------------------------*/
void twi_send_byte(unsigned char data)
{
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while(!(TWCR & (1<<TWINT))); 
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - TWI (or i2c) stop condition
  PARAMS: None
  RETURNS: None
--------------------------------------------------------------------------*/
void twi_stop(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - TWI (or i2c) read one byte and send ack bit
  PARAMS: None
  RETURNS: one byte of data
--------------------------------------------------------------------------*/
char twi_read_ack(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  while(!(TWCR & (1<<TWINT)));
  return TWDR;
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - TWI (or i2c) read one byte and send nack bit
  PARAMS: None
  RETURNS: one byte of data
--------------------------------------------------------------------------*/
char twi_read_nack(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN);
  while(!(TWCR & (1<<TWINT)));
  return TWDR;  
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Convert hex-encoded decimal to decimal data
  PARAMS: Hex encoded decimal number
  RETURNS: plain decimal number
  NOTES: DS3232 uses hexicdecimal numbers but they represent decimal
    numbers. EG: 0x12 = 12; when the first hex digit reaches 10, it
    carries over to the next digit instead of displaying A as normal
--------------------------------------------------------------------------*/
unsigned char get_dec(unsigned char hex_encoded) {
  return ((hex_encoded>>4)*10) + (hex_encoded & 0x0F);
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Synchronizes save time values with the DS3232 RTC
  PARAMS: None
  RETURNS: None
  NOTES: This sets four global variables: hour_tens, hour_ones, minute_tens,
    and minute_ones.
--------------------------------------------------------------------------*/
void syncTime(void) {
  //Get time from DS3232
  twi_start(i2c_write);
  twi_send_byte(0x00);
  twi_start(i2c_read);
  unsigned char temp_secs = twi_read_ack();
  unsigned char temp_mins = twi_read_ack();
  unsigned char temp_hours = twi_read_nack();
  twi_stop();

  seconds = get_dec(temp_secs);
  
  temp_hours = get_dec(temp_hours);		//Convert hours to decimal
  if (temp_hours > 12) temp_hours -= 12;	//Compensate for 24 hour time
  hour_tens = ((temp_hours/10)*2) + 10;  	//Adjust tens for our colon indexing
  hour_ones = temp_hours%10;
  minute_tens = temp_mins>>4;
  minute_ones = temp_mins & 0x0F;
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Reads DS3232 temperature and displays it
  PARAMS: None
  RETURNS: None
  NOTES: This sets four global variables: hour_tens, hour_ones, minute_tens,
    and minute_ones.
--------------------------------------------------------------------------*/
void disp_temperature(void) {
  //Get temperature from DS3232
  twi_start(i2c_write);
  twi_send_byte(0x11);	//Address of upper temperature byte
  twi_start(i2c_read);
  unsigned char temperature_MSB = twi_read_ack();
  unsigned char temperature_LSB = twi_read_nack();
  twi_stop();

  //Combine temperature Bytes into a value 16 times larger than the actual reading
  //  in order to avoid floating point math while still maintaining precision.
  unsigned int curr_temperature = ((unsigned int)temperature_MSB << 4)+(temperature_LSB >> 4);
  

  //unsigned char curr_temperature = ;	//Get temperature data (ignore 2 least significant bits)

  //Convert from C to F
  curr_temperature = ((curr_temperature*9)/5)+(32*16);	//Adjusting for precision

  //Take care of dividing by sixteen and rounding accordingly
  if ((curr_temperature%16) >= 8) curr_temperature = (curr_temperature >> 4) + 1;	//Round up
  else curr_temperature >>= 4;	//Round down

  if (curr_temperature > 99) hour_tens = 1;
  else hour_tens = 10;
  hour_ones = curr_temperature/10;
  minute_tens = curr_temperature%10;
  minute_ones = 14;	//Display degree symbol
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Increments the current state of the state machine
  PARAMS: None
  RETURNS: None
  NOTES: This changes the global variable 'state'
--------------------------------------------------------------------------*/
void increment_state(void) {
  if (++state > state_set) state = state_time;
  one_hz = 0;

  switch(state) {
    case state_time:
      saveSettings();
      syncTime();
      break;
    case state_temperature:
      disp_temperature();
      break;
    case state_set:
      set_tracker = 0;
      showSettings();
      break;
  }
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Displays current settings mode
  PARAMS: None
  RETURNS: None
  NOTES: This is based on the global variable 'set_tracker'
--------------------------------------------------------------------------*/
void showSettings(void) {
  unsigned char data;
  switch(set_tracker) {
    case 0:
      //hours
      hour_tens = 10;	//off
      hour_ones = 15;	// h
      data = get_dec(ds3232_read_setting(hours_address));
      minute_tens = data/10;
      minute_ones = data%10;
      break;
    case 1:
      //minutes
      hour_tens = 16;	//first part of m
      hour_ones = 17;	//second part of m
      data = get_dec(ds3232_read_setting(minutes_address));
      minute_tens = data/10;
      minute_ones = data%10;
      break;
    case 2:
      //Month
      hour_tens = 18;	//first part of M
      hour_ones = 19;	//second part of M
      data = get_dec(ds3232_read_setting(month_address));
      minute_tens = data/10;
      minute_ones = data%10;
      break;
    case 3:
      //Date
      hour_tens = 10;	//off
      hour_ones = 20;	// D
      data = get_dec(ds3232_read_setting(date_address));
      minute_tens = data/10;
      minute_ones = data%10;
      break;
    case 4:
      //Year
      hour_tens = 10;	//off
      hour_ones = 21;	// Y
      data = get_dec(ds3232_read_setting(year_address));
      minute_tens = data/10;
      minute_ones = data%10;
      break;
  }
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Saves current time and date settings to the DS3232
  PARAMS: None
  RETURNS: None
  NOTES:
    -Uses global variable 'set_tracker' to determine which time data
     is to be set. Records the currently displayed data.
    -When minutes are recorded seconds are also zeroed out.    
--------------------------------------------------------------------------*/
void saveSettings(void) {
  //Roll data into decimal tens on upper nibble and decimal ones on lower nibble
  unsigned char data = ((minute_tens<<4) + minute_ones);
  switch(set_tracker) {
    case 0:
      //hours
      ds3232_write_setting(hours_address, data);
      break;
    case 1:
      //minutes (also reset seconds to zero)
      ds3232_write_setting(minutes_address, data);
      ds3232_write_setting(seconds_address, 0x00);
      break;
    case 2:
      //Month
      ds3232_write_setting(month_address, data);
      break;
    case 3:
      //Date
      ds3232_write_setting(date_address, data);
      break;
    case 4:
      //Year
      ds3232_write_setting(year_address, data);
      break;
  }
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Read one byte from DS3232 RTC over i2c bus
  PARAMS: DS3232 register address to read from
  RETURNS: One byte of data read from give DS3232 address
--------------------------------------------------------------------------*/
unsigned char ds3232_read_setting(unsigned char address) {
  //Read one byte from DS3232
  twi_start(i2c_write);
  twi_send_byte(address);	//Send register address
  twi_start(i2c_read);
  unsigned char read_byte = twi_read_nack();
  twi_stop();

  return read_byte;
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Write one byte to DS3232 RTC over i2c bus
  PARAMS:
    1: Register address to write to
    2: Byte of data to store at that address
  RETURNS: None
--------------------------------------------------------------------------*/
void ds3232_write_setting(unsigned char address, unsigned char value) {
  //Write one byte from DS3232
  twi_start(i2c_write);
  twi_send_byte(address);	//Send register address
  twi_send_byte(value);
  twi_stop(); 
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Increments current settings value and updates display.
  PARAMS: None
  RETURNS: None
  NOTES: Uses global variable 'set_tracker'
--------------------------------------------------------------------------*/
void increment_settings_value(void) {
  //Convert separated digits to decimal
  unsigned char data = (minute_tens * 10) + minute_ones;

  switch(set_tracker) {
    case 0:
      //hours
      if (++data > 23) data = 0; 
      break;
    case 1:
      //minutes (also reset seconds to zero)
      if (++data > 59) data = 0;
      break;
    case 2:
      //Month
      if (++data > 12) data = 1;
      break;
    case 3:
      //Date	NOTE: There is no checking for correct days in a month
      if (++data > 31) data = 1;
      break;
    case 4:
      //Year
      if (++data > 99) data = 0;
      break;
  }

  //Store value as separated digits
  if (data < 10) minute_tens = 0;
  else minute_tens = data/10;
  minute_ones = data%10;
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Decrements current settings value and updates display
  PARAMS: None
  RETURNS: None
--------------------------------------------------------------------------*/
void decrement_settings_value(void) {
  //Convert separated digits to decimal
  unsigned char data = (minute_tens * 10) + minute_ones;

  switch(set_tracker) {
    case 0:
      //hours
      if (data-- == 0) data = 23; 
      break;
    case 1:
      //minutes (also reset seconds to zero)
      if (data-- == 0) data = 59;
      break;
    case 2:
      //Month
      if (data-- == 0) data = 12;
      break;
    case 3:
      //Date	NOTE: There is no checking for correct days in a month
      if (data-- == 0) data = 31;
      break;
    case 4:
      //Year
      if (data-- == 0) data = 99;
      break;
  }

  //Store value as separated digits
  if (data < 10) minute_tens = 0;
  else minute_tens = data/10;
  minute_ones = data%10;
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Main
  PARAMS: None
  RETURNS: None
--------------------------------------------------------------------------*/
int main(void) {
  init();
  initTimers();

  syncTime();
  
  while(1) {
    
    if( get_key_press( 1<<KEY0 )) {	//Mode button
      increment_state();
    }

    switch(state) {
      //Time State
      case state_time:
        if (one_hz) {
          one_hz = 0;
    
          //blink the colon
          if ((hour_tens == 12) || (hour_tens == 10)) ++hour_tens;
          else --hour_tens;

          if (++seconds > 59) { syncTime(); }
        }
        break;

      //Temperature State
      case state_temperature:
        if (one_hz > 63) {	//The DS3232 only reads temperature
                                //once every 64 seconds so that's
                                //how often we'll update.
          one_hz = 0;
          disp_temperature();
        }
        break;

      //Settings state
      case state_set:
        if( get_key_press( 1<<KEY1 )) {	//Next button
          //Save current setting
          saveSettings();

          //Move to next setting
	  if (++set_tracker > 4) set_tracker = 0;
          showSettings();
        }
        if( get_key_press( 1<<KEY2 )) {	//Plus button
	  increment_settings_value();
        }
        if( get_key_press( 1<<KEY3 )) {	//Minus button
	  decrement_settings_value();
        }
        break;
    }
  }  
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - Timer0 overflow interrupt service routine
  PARAMS: None
  RETURNS: None
  NOTES: Used to multiplex the display
--------------------------------------------------------------------------*/
ISR(TIMER0_OVF_vect) {
  shift(getDigit[*(time_digits[col_tracker])]);
  COLPORT &= ~COLMASK;			//Turn off columns
  SHIFTPORT |= RCK;			//Latch data
  COLPORT |= col_list[col_tracker];	//turn on next column
  if (++col_tracker > 3) col_tracker = 0;	//preload column for next interrupt
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - INT0 interrupt service routine
  PARAMS: None
  RETURNS: None
  NOTES: Triggers on falling edge; used to blink the colon in the display
    and track one minute periods for syncronization with the DS3232. The
    one hertz squarewave is generated by the RTC.
--------------------------------------------------------------------------*/
ISR(INT0_vect) {
  ++one_hz;
}

/*--------------------------------------------------------------------------
  FUNC: 1/30/11 - TIMER2 overflow interrupt service routine
  PARAMS: None
  RETURNS: None
  NOTES: Used to debounce buttons.
--------------------------------------------------------------------------*/
ISR(TIMER2_OVF_vect)           // every 10ms
{
  static unsigned char ct0, ct1;
  unsigned char i;

  TCNT2 = (unsigned char)(signed short)-(((F_CPU / 1024) * .01) + 0.5);   // preload for 10ms

  i = key_state ^ ~KEY_PIN;    // key changed ?
  ct0 = ~( ct0 & i );          // reset or count ct0
  ct1 = ct0 ^ (ct1 & i);       // reset or count ct1
  i &= ct0 & ct1;              // count until roll over ?
  key_state ^= i;              // then toggle debounced state
  key_press |= key_state & i;  // 0->1: key press detect
}
