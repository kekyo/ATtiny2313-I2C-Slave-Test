/*
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
#define YIC (1<<PA1)

static void out_addressbus(bool ya0, bool ya1)
{
	// Output addresses.

	if (ya0)
	{
		PORTB |= YA0;
	}
	else
	{
		PORTB &= ~YA0;
	}

	if (ya1)
	{
		PORTB |= YA1;
	}
	else
	{
		PORTB &= ~YA1;
	}
}

static void wait_for_ready_last_written()
{
	// Waiting for last written data (trigger_wr_and_finish) busy bit.
	// This place wants to settle last addresses YA0 and YA1.

	// Assert YCS (10ns)
	PORTB &= ~YCS;
	_delay_us(0.01);

	while (1)
	{
		// Assert YRD
		PORTB &= ~YRD;
		_delay_us(0.18);	// 180ns (TACC)

		// Strict load all 8bits data from databus.
		uint8_t status = PIND;
		if (PINB & YD7)
		{
			status |= 0x80;
		}
		else
		{
			status &= ~0x80;
		}

		PORTB |= YRD;

		_delay_us(0.01);	// 10ns (TDHW,TAH)

		// Is busy? (bit7)
		if (!(status & 0x80))
		{
			break;
		}
	}
	
	_delay_us(0.01);	// 10ns (TAS)

	// Databus (YD) settle to output.
	DDRD = YD0 | YD1 | YD2 | YD3 | YD4 | YD5 | YD6;
	DDRB |= YD7;
}

static void out_databus(uint8_t data)
{
	// Implicit delay 10ns (TAS)

	// 0..6bit
	PORTD = data & 0x7f;

	// 7bit
	if (data & 0x80)
	{
		PORTB |= YD7;
	}
	else
	{
		PORTB &= ~YD7;
	}
}

static void trigger_wr_and_finish()
{
	// Assert YWR (This place already asserted YCS)
	PORTB &= ~YWR;
	_delay_us(0.10);	// 100ns (TCW/TWW)

	// Pulse sent to YWR
	PORTB |= YWR;
	_delay_us(0.01);	// 10ns (TDHW,TAH)

	// Pulse sent to YCS
	PORTB |= YCS;

	// YD reset (For stablility databus)
	PORTD = 0;
	PORTB &= ~YD7;

	// YD to high impedance.
	DDRD = 0;
	DDRB &= ~YD7;
}

static void out_ym2151(uint8_t address, uint8_t data)
{
	wait_for_ready_last_written();
	out_addressbus(false, false);	// /A0 /A1
	out_databus(address);
	trigger_wr_and_finish();

	wait_for_ready_last_written();
	out_addressbus(true, false);	// A0 /A1
	out_databus(data);
	trigger_wr_and_finish();
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
	DDRB = YD7 | YA0 | YA1 | YWR | YRD | YCS;

	PORTB |= YWR;
	PORTB |= YRD;
	PORTB |= YCS;
	PORTB &= ~YD7;
	PORTB &= ~YA0;
	PORTB &= ~YA1;

	DDRA |= YIC;
	PORTA &= ~YIC;

	_delay_us(10000);

	PORTA |= YIC;
  	
	// initialize as slave with our hardware address
	usiTwiSlaveInit( 0x20 );
	
  	// enable interrupts (must be there, i2c needs them!)
  	sei();

  	// handle commands via I2C bus
  	while (1)
  	{
		//	check if data is in the i2c receive buffer
		while (!usiTwiDataInReceiveBuffer())
		{
			NOP;
		}

		//	the first byte in the stream is our opcode
		uint8_t address = usiTwiReceiveByte();

		while (!usiTwiDataInReceiveBuffer())
		{
			NOP;
		}

		uint8_t data = usiTwiReceiveByte();

		out_ym2151(address, data);
  	}

  	return 0;
}
