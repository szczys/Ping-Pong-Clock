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

//i2c definitions
#define i2c_slave_address 0xD0 	//Address for DS3232
#define i2c_read (i2c_slave_address + 1)
#define i2c_write i2c_slave_address

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
#define KEY1		2	//Next
#define KEY2		1	//+
#define KEY3		3       //-

//Prototypes
void init(void);
void initTimers(void);
unsigned char get_key_press(unsigned char key_mask);
void delay_ms(int c);
void shift(unsigned int data);
void incMinute(void);
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
  0b0111001000011100,	//One with colon

  0b0100000001111111,	//degree symbol 		(index: 14)
  0b0111101001001011,	// h 				(index: 15)
  0b0101001000000100,	// m (left leg for hour ten	(index: 16)
  0b0111001000001110,	// m (others for hour ones)	(index: 17)
  0b0101001000010100,	// M (right leg for hour tens	(index: 18)
  0b0101001001011110,	// M (others for hour ones)	(index: 19)
  0b0111110001101110,	// D 				(index: 20)
  0b1101111001011111	// Y				(index: 21)
};

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

volatile unsigned char one_hz = 0;

//State definitions and variable
#define state_time 0
#define state_temperature 1
#define state_set 2
unsigned char state = state_time;
unsigned char set_tracker;

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

  //i2c
  TWBR = 0x02;	//Set 400khz I2C clock without prescaler
		//  equation: f_cpu/(16 + (2 * TWBR))*prescaler

  //Enable 1 Hz output from DS3232
  twi_start(i2c_write);
  twi_send_byte(0x0e);	//Address for Control Register
  twi_send_byte(0x00);	//Set 1 Hz square wave output
  twi_stop();

  //Enable INT0 for tracking square wave
  EICRA |= (1<<ISC01);	// INT0 as input
  EIMSK |= (1<<INT0);	// Enable interrupt

  //Setup Button
  KEY_DDR &= ~((1<<KEY0) | (1<<KEY1) | (1<<KEY2) | (1<<KEY3));
  KEY_PORT |= (1<<KEY0) | (1<<KEY1) | (1<<KEY2) | (1<<KEY3);	//enable pull-up resistor
  
}

void initTimers(void) {
  cli();

  //Timer0 for display scanning
  TIMSK0 |= (1<<TOIE0);			//Enable overflow interrupt
  TCCR0B |= (1<<CS01) | (1<<CS00);	//Start timer, prescale 64

  //Timer2 for buttons
  TCCR2B |= 1<<CS22 | 1<<CS21 | 1<<CS20;	//Divide by 1024
  TIMSK2 |= 1<<TOIE2;		//enable timer overflow interrupt
  
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

void twi_start(unsigned char SlvAddr)
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA);   //Send start command
  while(!(TWCR & (1<<TWINT)));                  //Wait for bus to become ready
  TWDR = SlvAddr;                     //Write device address to data register
  TWCR = (1<<TWINT) | (1<<TWEN);                //Send device address
  while(!(TWCR & (1<<TWINT)));                  //Wait for bus to become ready
}

void twi_send_byte(unsigned char data)
{
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while(!(TWCR & (1<<TWINT))); 
}


void twi_stop(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

char twi_read_ack(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  while(!(TWCR & (1<<TWINT)));
  return TWDR;
}

char twi_read_nack(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN);
  while(!(TWCR & (1<<TWINT)));
  return TWDR;  
}

unsigned char get_dec(unsigned char hex_encoded) {
  return ((hex_encoded>>4)*10) + (hex_encoded & 0x0F);
}

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

unsigned char ds3232_read_setting(unsigned char address) {
  //Read one byte from DS3232
  twi_start(i2c_write);
  twi_send_byte(address);	//Send register address
  twi_start(i2c_read);
  unsigned char read_byte = twi_read_nack();
  twi_stop();

  return read_byte;
}

void ds3232_write_setting(unsigned char address, unsigned char value) {
  //Write one byte from DS3232
  twi_start(i2c_write);
  twi_send_byte(address);	//Send register address
  twi_send_byte(value);
  twi_stop(); 
}

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

ISR(TIMER0_OVF_vect) {
  shift(getDigit[*(time_digits[col_tracker])]);
  COLPORT &= ~COLMASK;			//Turn off columns
  SHIFTPORT |= RCK;			//Latch data
  COLPORT |= col_list[col_tracker];	//turn on next column
  if (++col_tracker > 3) col_tracker = 0;	//preload column for next interrupt
}

ISR(INT0_vect) {
  ++one_hz;
}

ISR(TIMER2_OVF_vect)           // every 10ms
{
  static unsigned char ct0, ct1;
  unsigned char i;

  TCNT0 = (unsigned char)(signed short)-(((F_CPU / 1024) * .01) + 0.5);   // preload for 10ms

  i = key_state ^ ~KEY_PIN;    // key changed ?
  ct0 = ~( ct0 & i );          // reset or count ct0
  ct1 = ct0 ^ (ct1 & i);       // reset or count ct1
  i &= ct0 & ct1;              // count until roll over ?
  key_state ^= i;              // then toggle debounced state
  key_press |= key_state & i;  // 0->1: key press detect
}
