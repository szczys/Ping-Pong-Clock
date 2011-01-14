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

//Prototypes
void init(void);
void initTimers(void);
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
char get_temperature(void);
unsigned char disp_temperature(void);


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

  0b0100000011111111	//degree symbol (index: 14)
};

unsigned char col_list[4] = { COL0, COL1, COL2, COL3 };

//Display variables
unsigned char col_tracker = 0;	//Used for column scanning
volatile unsigned char seconds = 0;
volatile unsigned char hour_tens = 10;
volatile unsigned char hour_ones = 3;
volatile unsigned char minute_tens = 0;
volatile unsigned char minute_ones = 4;

volatile unsigned char one_hz = 0;

//State definitions and variable
#define disp_time 0
#define disp_temp 1
#define disp_set 2
unsigned char state = disp_temp;

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
  
}

void initTimers(void) {
  cli();

  //Timer0 for display scanning
  TIMSK0 |= (1<<TOIE0);			//Enable overflow interrupt
  TCCR0B |= (1<<CS01) | (1<<CS00);	//Start timer, prescale 64
  
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

char get_temperature(void) {
  //Get temperature from DS3232
  twi_start(i2c_write);
  twi_send_byte(0x11);	//Address of upper temperature byte
  twi_start(i2c_read);
  unsigned char curr_temperature = twi_read_nack();
  twi_stop();

  return curr_temperature;
}

unsigned char disp_temperature(void) {

  //TODO: Improve code to use full temperature precision

  signed char curr_temperature = get_temperature();	//Get temperature data (ignore 2 least significant bits)

  //Convert from C to F
  curr_temperature = ((curr_temperature*9)/5)+32;

  if (curr_temperature > 99) hour_tens = 1;
  else hour_tens = 10;
  hour_ones = curr_temperature/10;
  minute_tens = curr_temperature%10;
  minute_ones = 14;	//Display degree symbol
}

int main(void) {
  init();
  initTimers();

  syncTime();
  
  while(1) {
    switch(state) {
    
      case disp_time:
        if (one_hz) {
          one_hz = 0;
    
          //blink the colon
          if ((hour_tens == 12) || (hour_tens == 10)) ++hour_tens;
          else --hour_tens;

          if (++seconds > 59) { syncTime(); }
        }
        break;

      case disp_temp:
        if (one_hz > 5) {	//Only update every 5 seconds
          one_hz = 0;
          disp_temperature();
        }
        break;

      case disp_set:
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

