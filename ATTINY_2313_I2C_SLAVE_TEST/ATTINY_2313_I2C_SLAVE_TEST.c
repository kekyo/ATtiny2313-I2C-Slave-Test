/*
 * ATTINY_2313_I2C_SLAVE_TEST.c
 *
 * Created: 11/28/2011 9:07:52 AM
 *  Author: Owner
 */ 

/*
 *	The internal oscillator of the ATTiny2313 runs at 8 MHz.  If the CKDIV8
 *	fuse is set, the system clock is prescaled by 8; therefore, we are setting
 *	the F_CPU at 1 MHz
 */
#define F_CPU 20000000UL

#include <util/delay.h>
#include <avr/io.h>	
#include <avr/sfr_defs.h>
#include <compat/twi.h>
#include <avr/interrupt.h>
#include "usiTwiSlave.h"

#define NOP asm("nop");				//	skip one clock cycle

#define SDA (1<<PB5)    //  I2C SDA
#define SCL (1<<PB7)    //  I2C SCL

#define YD0 (1<<PD0)
#define YD1 (1<<PD1)
#define YD2 (1<<PD2)
#define YD3 (1<<PD3)
#define YD4 (1<<PD4)
#define YD5 (1<<PD5)
#define YD6 (1<<PD6)
#define YD7 (1<<PB0)

#define YA0 (1<<PB1)
#define YA1 (1<<PB2)

#define YWR (1<<PB3)
#define YRD (1<<PB4)
#define YCS (1<<PB6)

//#define YIC (1<<PA1)
#define YIC (1<<PB4)

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
	//if( DEBUG ) {
		PORTD ^= (1<<PD3);
	//}
	//	don't increment second count if we're not burning
}

static void out_addressbus(bool ya0, bool ya1)
{
	uint8_t value = PORTB;
	if (ya0)
	{
		value |= YA0;
	}
	else
	{
		value &= ~YA0;
	}
	if (ya1)
	{
		value |= YA1;
	}
	else
	{
		value &= ~YA1;
	}
	PORTB = value;

	_delay_us(0.1);	// 10ns
	PORTB &= ~YCS;
}

static void out_databus(uint8_t data)
{
	DDRD = YD0 | YD1 | YD2 | YD3 | YD4 | YD5 | YD6;
	DDRB |= YD7;

	PORTD = data & 0x7f;

	if (data & 0x80)
	{
		PORTB |= YD7;
	}
	else
	{
		PORTB &= ~YD7;
	}

	_delay_us(0.1);	// 10ns
}

static void trigger_wr_and_finish()
{
	PORTB &= ~YWR;
	_delay_us(0.1);		// Trigger width from assert CS >= 100ns.

	PORTB |= YWR;
	_delay_us(0.1);	// Trigger width from assert CS >= 100ns.

	PORTB |= YCS;
	_delay_us(0.1);	// 10ns
}

static void cleanup_bus()
{
	PORTD = 0;
	PORTB &= ~(YD7 | YA0 | YA1);

	DDRD = 0;
	DDRB &= ~YD7;
}

static uint8_t read_databus()
{
	// Read sequence ignores address, so assert both.
	PORTB &= ~(YRD | YCS);
	_delay_us(0.1);		// Trigger width from assert CS >= 100ns.

	uint8_t data = PIND;
	if (PINB & YD7)
	{
		data |= 0x80;
	}
	else
	{
		data &= ~0x80;
	}

	PORTB |= YRD | YCS;

	return data;
}

static void out_ym2151(uint8_t address, uint8_t data)
{
	out_addressbus(false, false);	// /A0 /A1
	out_databus(address);
	trigger_wr_and_finish();

	out_addressbus(true, false);	// A0 /A1
	out_databus(data);
	trigger_wr_and_finish();

	cleanup_bus();
}

//
//	GLOBALS
//
int main(void)
{
	// Set high impedance for input mode.
	MCUCR |= 0x80; 

	//	setup PORTD data direction (PIND0-2 are the hardware address)
	DDRD = 0;
	PORTD = 0;

	//	PB0 is the burn trigger; so set the data direction register
	DDRB |= YA0 | YA1 | YWR | YRD | YCS;
	PORTB = YWR | YRD | YCS;

	// PA
	//DDRA |= YIC;

	// Do reset YM2151
	//PORTA &= ~YIC;
	PORTB &= ~YIC;
	_delay_us(1000);
	//PORTA |= YIC;
	PORTB |= YIC;
	_delay_us(1000);
	
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
		if (!usiTwiDataInReceiveBuffer() )
		{
			NOP;
			continue;
		}

		//	the first byte in the stream is our opcode
		uint8_t address = usiTwiReceiveByte();

		uint16_t timeout = 1000;	// 1ms
		while ((timeout > 0) && (!usiTwiDataInReceiveBuffer()))
		{
			_delay_us(1);
			timeout--;
		}

		if (timeout == 0)
		{
			continue;
		}

		uint8_t data = usiTwiReceiveByte();
		out_ym2151(address, data);
  	}

  	return 0;
}
