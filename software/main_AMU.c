/*
 * QUTMS_Charger.c
 *
 * Created: 22/11/2016 11:35:08 PM
 * Author : julius
 */ 
#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "MCP2515.h"
#include "AtmelCAN.h"


#define YELLOW_LED 0					//***
#define RED_LED 1						//***
#define CHARGER_BROADCAST	0x18FF50E5


void IO_init()
{
	
	DDRB  = 0b11000110;
	DDRD  = 0b10001011;
	DDRC  = 0b10100111;
	PORTB = 0b00000000;

	PORTD |= (1<<PIND1)|(1<<PIND7)|(1<<PIND3);		//the SS pin needs to be configured as an output, otherwise we get issues on removal of the programmer.
	PORTC |= 1<<PINC7;					//set CS high
	PORTC |= (1<<MCP2515_PIN_RESET);	//set reset to high
	PORTB &= ~(1<<PINB7);		//make sure it's set up for input.
}



void LED_on( uint8_t selection)
{
	switch(selection)
	{
		case YELLOW_LED:
		PORTD &= ~(1<<PIND7);
		break;
		case RED_LED:
		PORTD &= ~(1<<PIND1);
		break;
		default:
		break;
	}
}
void LED_off( uint8_t selection)
{
	switch(selection)
	{
		case YELLOW_LED:
		PORTD |= (1<<PIND7);
		break;
		case RED_LED:
		PORTD |= (1<<PIND1);
		break;
		default:
		break;
	}
}
void flash_LED(uint8_t number, uint8_t selection, uint8_t duration)
{
	duration = duration/2;
	if(selection == RED_LED)
	{
		for(uint8_t count = 0; count < number ;count++)
		{
			PORTD &= ~(1<<PIND1);
			for(uint8_t counter = 0; counter < duration; counter++)
			{
				_delay_us(995);
			}
			PORTD |= (1<<PIND1);
			for(uint8_t counter = 0; counter < duration; counter++)
			{
				_delay_us(995);
			}
		}
	}
	else
	{
		for(uint8_t count = 0;count<number;count++)
		{
			PORTD &= ~(1<<PIND7);
			for(uint8_t counter = 0; counter < duration; counter++)
			{
				_delay_us(995);
			}
			PORTD |= (1<<PIND7);
			for(uint8_t counter = 0; counter < duration; counter++)
			{
				_delay_us(995);
			}
		}
	}

}

int main(void)
{
	IO_init();
	CAN_init();
	_delay_ms(10);	//wait for voltages to settle
	sei();
	//CAN_RXInit(5,5,0b00011111111111111111111111111111,CHARGER_BROADCAST);		//setup to receive the broadcast message from the charger
	CAN_RXInit(5,1,0,0);
	uint8_t on_instruction[8]={3, 132, 0, 50, 0, 0, 0, 0};
	CAN_TXMOB(CAN_findFreeTXMOB(),8,on_instruction,0x1806E5F4,0);
	_delay_ms(1000);

    while (1) 
    {
		//CAN_TXMOB(CAN_findFreeTXMOB(),8,on_instruction,0x1806E5F4,0);
		_delay_ms(1000);
		
    }
}

ISR(CAN_INT_vect)
{

	if(CANSIT2 & (1 << SIT5))	//we received a CAN message on mob 5, which is set up to receive exclusively from the Chassis controller.
	{
		CANPAGE = (5 << 4);			//set the canpage to the receiver MOB
		CANSTMOB &= ~(1 << RXOK);	//unset the RXOK bit to clear the interrupt.

		LED_on(YELLOW_LED);
		_delay_ms(10);
		LED_off(YELLOW_LED);
		
	}
	CAN_RXInit(5,1,0,0);
}