/*
 * ATTINY_2313_I2C_SLAVE_TEST.c
 *
 * Created: 11/28/2011 9:07:52 AM
 *  Author: Owner
 */ 

/***********************************************************************+/
/| ATTiny2313 setup diagram	                                         |/
 |																								|
 |					---------	--																|
 |		Vcc ----|1		  20|----Vcc														|
 |	   HADDR0--|2		  19|----SCL														|
 |		HADDR1--|3		  18|----NC														|
 |	   NC------|4		  17|----SDA														|
 |		NC------|5		  16|----NC														|
 |		HADDR2--|6		  15|----NC														|
 |		1s sig--|7		  14|----NC														|
 |		NC------|8		  13|----NC														|
 |		NC------|9		  12|----BURN												   |
 |		GND-----|10		  11|----NC														|
 |				  ------------																|
 |																								|
 | Where HADDR0-2 is the hardware address that is read by the code at	   |
 |	startup.																					|
 |	BURN is the burn trigger.  This pin is set to logic level 1 to begin |
 |	the burn process, typically triggering a relay to complete the burn	|
 |	circuit.
 |	1s sig is a 1 second pulse meant only for debugging.  It will be		|
 |	disabled in production code.													   |
 ************************************************************************/

//	Default clock speed
/*
 *	The internal oscillator of the ATTiny2313 runs at 8 MHz.  If the CKDIV8
 *	fuse is set, the system clock is prescaled by 8; therefore, we are setting
 *	the F_CPU at 1 MHz
 */
#define F_CPU 20000000UL
#define DEBUG 1

#include <util/delay.h>
#include <avr/io.h>	
#include <avr/sfr_defs.h>
#include <compat/twi.h>
#include <avr/interrupt.h>
#include "usiTwiSlave.h"

#define NOP asm("nop");				//	skip one clock cycle

#define SDA     (1<<PB5)    //  I2C SDA
#define SCL     (1<<PB7)    //  I2C SCL

#define YD0     (1<<PD0)
#define YD1     (1<<PD1)
#define YD2     (1<<PD2)
#define YD3     (1<<PD3)
#define YD4     (1<<PD4)
#define YD5     (1<<PD5)
#define YD6     (1<<PD6)
#define YD7     (1<<PB2)	// Must bit2

#define YA0     (1<<PB0)	// Must bit0
#define YA1     (1<<PB1)	// Must bit1

#define YWR     (1<<PB3)
#define YRD     (1<<PB4)
#define YCS     (1<<PB6)

#define YIC     (1<<PA1)

//
//	OPCODES FOR OUR VIRTUAL DEVICE
//

#define I2C_INITIATE_BURN	0x10	//	stage 1 of the sequence
#define I2C_CONFIRM_BURN	0x20	//	stage 2 of the sequence; followed by secret code
#define I2C_CANCEL_BURN		0x30	//	cancel the process
#define I2C_SET_BURN_DURATION 0x40	//	followed by duration in seconds
#define I2C_CONFIRM_BURN_SECRET_CODE 0xCC	// this is the code that must be passed after I2C_CONFIRM_BURN
#define I2C_ACKNOWLEDGE	0x7F	//	acknowledgment sent back to host after successfully confirming burn

//
//	FUNCTION PROTOTYPES
//
void initTimer();
uint8_t hardwareAddress();
void beginBurn();

enum {
	MODE_DEFAULT,
	MODE_INITIATED,
	MODE_BURN
};
typedef uint8_t AKDBurnMode;


static void out_data(uint8_t address, uint8_t data)
{
	// Step1: Output addresses and assert CS.
	PORTB |= address & 0x03;
	// _delay_us(0.001);	// Setup >= 10ns.
	PORTB &= ~YCS;

	// Step2: Set direction to out and output data.
	DDRD = YD0 | YD1 | YD2 | YD3 | YD4 | YD5 | YD6;
	PORTD = data & 0x7f;
	DDRB |= YD7;
	PORTB = (PORTB & ~YD7) | (data >> (7 - 2));	// Must PB2

	// Step3: Assert WR.
	PORTB &= ~YWR;

	// Step4: Finish.
	_delay_us(0.1);		// Trigger width from assert CS >= 100ns.
	PORTB |= YWR | YCS;
	PORTD = 0;
	DDRD = 0;
	PORTB &= ~(YD7 | YA0 | YA1);
	DDRB &= ~YD7;
}

//
//	Initiate the timer/counter
//
//	We are using TIMER/COUNTER 1 in CTC mode
//	Target Timer Count = (Input Frequency / Prescale) / Target Frequency - 1
// or, (10^6/64/1)-1 = 15624
//
void initTimer()
{
	// Configure timer 1 for CTC mode
	TCCR1B |= (1 << WGM12);
	// Enable CTC interrupt
	TIMSK |= (1 << OCIE1A);

	//  Enable global interrupts
	sei();

	// Set CTC compare value to 1Hz at 1MHz AVR clock, with a prescaler of 64
	OCR1A   = 15624;
	// Start timer at Fcpu/64
	TCCR1B |= ((1 << CS10) | (1 << CS11));
}

//
//	Reads the hardware address of the device
//
//	The hardware address is set at PD0-2
//
uint8_t hardwareAddress() {
	return 0x20;
	//return PIND & ~0b11111000;
}

ISR(TIMER1_COMPA_vect)
{
	//	pulse the PD3 pin every second for testing purposes
	if( DEBUG ) {
		PORTD ^= (1<<PD3);
	}
	//	don't increment second count if we're not burning
}

//
//	GLOBALS
//
int main(void)
{
	//	setup PORTD data direction (PIND0-2 are the hardware address)
	DDRD = 0;
	PORTD = 0;

	//	PB0 is the burn trigger; so set the data direction register
	DDRB |= YA0 | YA1 | YWR | YRD | YCS;
	PORTB = YWR | YRD | YCS;

	// PA
	DDRA |= YIC;

	// Do reset YM2151
	PORTA = 0;
	_delay_us(100);
	PORTA = YIC;
	_delay_us(100);
	
	//	obtain I2C address at PIND0-2
	uint8_t slave_address = hardwareAddress();
  	
	// initialize as slave with our hardware address
	usiTwiSlaveInit( slave_address );
	
  	// enable interrupts (must be there, i2c needs them!)
  	sei();

  	// handle commands via I2C bus
  	while (1)
  	{
		  //	check if data is in the i2c receive buffer
		  if( usiTwiDataInReceiveBuffer() )
		  {
			  //	the first byte in the stream is our opcode
			  uint8_t address;
			  uint8_t data = usiTwiReceiveByte(&address);
			  address &= 0x01;

			  out_data(address, data);

			  //_delay_ms(25);
			  //if( b == I2C_INITIATE_BURN )
			  //{
				  	//if request is to initiate burn, only initiate if we are in default mode
				  	//otherwise, for safety, we drop back to default mode
				  //_burn_mode = (_burn_mode == MODE_DEFAULT)?MODE_INITIATED:MODE_DEFAULT;				  
			  //}	
			  //else if( b == I2C_CANCEL_BURN )
			  //{
				  	//if the request is to cancel, always drop back to default mode
				  //_burn_mode = MODE_DEFAULT;
				  //PORTD &= ~BURN_TRIGGER;
			  //}
			  //else if( b == I2C_CONFIRM_BURN )
			  //{
				  	//if the request is to confirm, look for a second byte that has the 
				  	//confirmation code.
				  //uint8_t confirm_byte = usiTwiReceiveByte();
				  //if( confirm_byte == I2C_CONFIRM_BURN_SECRET_CODE )
				  //{
					  //usiTwiTransmitByte(I2C_ACKNOWLEDGE);
					  //_delay_ms(10);
					  //_burn_mode = MODE_BURN;
					  //beginBurn();
				  //}					  					  
			  //}	
			  //else if( b == I2C_SET_BURN_DURATION )
			  //{
				  	//if the request if to set the burn duration, then look for the duration
				  	//in seconds in the next byte
				  //uint8_t duration_byte = usiTwiReceiveByte();
				  //_burn_duration = duration_byte;
			  //}			
		 }  
		 //	waste a cycle  	 
		 NOP
  	}
  	return 0;
}
