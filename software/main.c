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
#include "ChargeController_V1.h"
#include "MCP2515_CC.h"
#include "uart.h"
#include "SPI.h"

void LED_flash(unsigned char times)
{
	for(uint8_t i = 0; i < times; i++)
	{
		PORTK ^= 0b00100000;
		_delay_ms(5);
		PORTK ^= 0b00100000;
		_delay_ms(5);
	}
	PORTK &= ~(0b00100000);
}
void LED_toggle()
{
	PORTK ^= 0b00100000;
}
void LED_off()
{
	PORTK &= ~(1<<PINK5);
}
void LED_on()
{
	PORTK |= (1<<PINK5);
}

void ExternalInterrupt_init()
{
	//INT1 for CAN1, INT0 for CAN2, PCINT7 for CAN3
	EICRA |= (2<<ISC00)|(2<<ISC10);
	EIMSK = (1<<INT0) | (1<<INT1);
	//enable interrupts fot PCINT7:0
	PCICR = (1<<PCIE0)|(1<<PCIE1);
	PCMSK0 = (1<<PCINT7);
	PCMSK1 = (1<<PCINT15);
}

void IO_init()
{
	//pins 8, 9 and 14 for MCP2515_STB high so the things respond (PE6, PE7, PH2)
	DDRE = 0b11000010;		//PE0 = RS232 RX1; PE1 = RS232 TX1;  PE6 = STB_CAN1; PE7 = STB_CAN2;
	PORTE = 0b00000000;		
	DDRH = 0b00000111;		//PH0 = CS_CAN1; PH1 = CS_CAN2; PH2 = STB_CAN3
	PORTH = 0b00000011;		//CS_CAN1 high; CS_CAN2 high;
	//pins 12, 13, 19 for the CS for each MCP2515 PH0, PH1, PB0)

	//pin 21 for MOSI, pin 20  for SCK (PB2, PB1)
	DDRB = 0b01100111;		//PB0 = CS_CAN3; PB1 = SCK; PB2 = MOSI; PB3 = MISO; PB5 = High drive A; PB6 = Low drive A; PB7 = CAN3_INT; 
	PORTB = 0b00000001;		//set CS_CAN3 high;
	
	DDRL = 0b00011000;		//PB3 = High drive B; PB4 = Low Drive B;
	PORTL = 0b00000000;
	
	DDRD = 0b11001000;		//PD0 = CAN2_INT; PD1 = CAN1_INT; PD2 = RS232 RX2; PD3 = RS232 TX2; PD6 = CAN1_TXPIN; PD7 = CAN2_TXPIN;
	DDRD = 0b00000000;
	
	DDRC = 0b00001000;		//PC3 = CAN3_TXPIN;
	PORTC= 0b00000000;
	
	DDRJ = 0b00000000;		//PORTJ is used for digital input;
	
	DDRA = 0b00011000;		//PA3 = ENABLE_B; PA4 = ENABLE_A; PA1 = dig input; PA2 = dig input;
	PORTA = 0b00011000;
	
	DDRK = 0b00100000;		//PK5 = debugging LED;
	PORTK = 0b00100000;
	
	ExternalInterrupt_init();
	//pin 26 is INT for CAN3
	
	//ping 43 and 44 for CAN2 and CAN1, PG2 and PA7
	/*DDRA  = 0b00110000;		//PA5 = LED2; PA4 = LED1
	DDRB  = 0b11110111;		//PB7 = INJ3; PB6 = INJ2; PB5 = INJ1; PB4 = AUX1; PB3 = MISO; PB2 = MOSI; PB1 = SCK; PB0 = SS
	DDRC  = 0b01000001;		//PC7 = SF_INJ12; PC6 = EN_INJ; PC1 = ST_AUX12; PC0 = CAN_SS
	DDRD  = 0b10001000;		//PD7 = CAN_TX0; PD5 = RTI2; PD4 = RTI1; PD3 = TX1; PD2 = RX1; PD1 = CAN_INT; PD0 = CAS2
	DDRE  = 0b00111010;		//PE7 = CAS1; PE6 = RTI3; PE5 = INJ4; PE4 = AUX4; PE3 = AUX3; PE2 = ST_AUX34; PE1 = TX0; PE0 = RX0
	DDRF  = 0b00000000;		//PORTF ALL ADCS
	DDRG  = 0b00111010;		//PG5 = AUX2; PG4 = Pulldown2; PG3 = Pulldown1; PG2 = ST_INJ34; PG1 = EN_AUX
		
	PORTA |= 0b11100000;	//turn on digital input pull-ups, turn LED2 on;
	PORTB |= 0b00011000;
	PORTC |= 0b00000001;	//set CAN_SS high

	PORTC |= 1;										//set CAN_CS high
		
	//interrupts
	EICRA = 0b00001000;		//enable INT1 on falling edges
	EIMSK = 0b00000010;		//enable INT1
	}*/
}

void Timer_init()
{
	TCCR0A = 0b00000000;			//normal mode 0xff is top
	TCCR0B = 0b00000101;			//prescale clock by 1024
	TIMSK0 = 0b00000001;			//turn on compare interrupt for OCR1A	
	
	TCCR1A = 0b00000000;			//CTC mode
	TCCR1B = 0b00001101;			//prescale clock by 1024
	OCR1A =  312;					//312 gives 50Hz main comms speed
	TIMSK1 = 0b00000010;			//turn on compare interrupt for OCR1A
}

void Relays_on()
{
	PORTB |= (1<<PB6);
	PORTB |= (1<<PB5);
}

void Relays_off()
{
	PORTB &= ~(1<<PB6);
	PORTB &= ~(1<<PB5);
}

void Charger_deactivate()
{
	//shutdown the relays
	Relays_off();
	//turn the charger off
	//charger.inputStatus = 1;
	//sendHeartbeat(CHARGER_H, CHARGER_KEEP_ALIVE, 0);
	accumulators[0].flags[0] &= ~(1);
	//sendHeartbeat(AMU_H, NORMAL, ACCUMULATOR_FRONT);
}

void Charger_activate()
{
	//shutdown the relays
	Relays_on();
	//turn the charger off
	charger.inputStatus = 0;
	sendHeartbeat(CHARGER_H, CHARGER_KEEP_ALIVE, 0);
	sendHeartbeat(AMU_H, NORMAL, ACCUMULATOR_FRONT);
}

void sendHeartbeat(unsigned char destination, unsigned char type, unsigned char address)
{
	uint8_t mob;
	uint32_t ID = 0;
	switch(destination)
	{
		case CHARGER_H:

			//obtain a mob that is free
			mob = MCP2515_findFreeTxBuffer(MCP2515_CAN1);
			if(mob == 0)LED_on();
			uint8_t on_instruction[8] = {(charger.inputVoltage>>8), 
										 (charger.inputVoltage & 0xff), 
										 (charger.inputCurrent>>8), 
										 (charger.inputCurrent & 0xff), 
										 charger.inputStatus, 0, 0, 0};
			ID = HEARTBEAT_CHARGER_ID;
			//type = what sort of command. address is which address of CHARGER, normal KEEP_ALIVE heartbeat packet;
			MCP2515_TX(MCP2515_CAN1, mob,8,on_instruction,ID);
			break;
		case AMU_H:
			
			//obtain a mob that is free
			mob = MCP2515_findFreeTxBuffer(MCP2515_CAN1);
			//if(mob == 0)LED_on();
			//type = what sort of command. address is which address of AMU, normal heartbeat packet;
			ID = (HEARTBEAT_AMU_ID|((uint32_t)type<<18)|((uint32_t)address<<13)|1);
			//transmit the packet.
			MCP2515_TX(MCP2515_CAN2,mob, 4, accumulators[0].flags, ID);
			break;
		default:
			break;
		
	}

}

void UART_processByte(char data)
{
	//PORTA ^= 32;
	
	static unsigned char count = 0;
	static unsigned char incomingString[48];							//made bigger to accommodate PT packets
	static unsigned char stringActive = 0;
	
	if(data == 'D')
	{
		stringActive = 1;
	}
	
	else if(stringActive == 1)
	{
		incomingString[count++] = data;
	}
	
	if(count > 7)
	{
		
		incomingString[count - 1] = '\0';
		count = 0;
		stringActive = 0;
		UART_parseInput(incomingString);
		
	}
	
}

void UART_parseInput(unsigned char* s)
{
	
	UART_processPacket(s);
	//uart1_putc('D');		//reply with the header byte (preserved - 33 bytes should follow)
	UART_sendRealTimeData();

	
	s[0] = '\0';				// clear the header byte
	
}

void UART_processPacket(unsigned char* s)
{
	
	switch(s[1])
	{
		case 1:
			
			charger.inputVoltage = (s[2]<<8) + s[3];
			charger.inputCurrent = (s[4]<<8) + s[5];
			uint8_t status = s[6];
			//LED_flash(4);
			//if(status)LED_flash(4);
			if((status & 128)==128)
			{
				//LED_flash(4);
				Relays_on();
				if((status & 64)==64)
				{
					STATUS_REG01 |= CHARGING;
					charger.inputStatus = 0;
				}
				else 
				{
					STATUS_REG01 &= ~(CHARGING);
					//Relays_off();
					charger.inputStatus = 1;
				}
				//if the balancing flag is on
				if(status & 32)
				{
					STATUS_REG01 |= BALANCING;
					accumulators[0].flags[0] |= 1;
				}
				else
				{
					STATUS_REG01 &= ~(BALANCING);
					accumulators[0].flags[0] &= ~(1);
				}
			}
			else 
			{
				STATUS_REG01 &= ~(CHARGING);
				accumulators[0].flags[0] &= ~(1);
				charger.inputStatus = 1;
				Relays_off();
			}
			break;
		default:
			
			break;
	}
	//checksum here
	//if the packet is valid, the master is alive
	STATUS_REG01 |= MASTER_ALIVE;
	
	
	/*uint16_t addr = ((uint16_t)(s[1]) << 8);
	addr += (uint16_t)(s[2]);
	uint32_t data = ((uint32_t)s[3]) << 24;
	data = ((uint32_t)s[4]) << 16;
	data += ((uint32_t)s[5]) << 8;
	data += ((uint32_t)s[6]);
	
	if (addr == 20)
	{
		if (data == 128)
		{
			
		}
		else
		{
			
		}
	}
	
	else if (addr == 21)
	{
		if (data == 128)
		{
			
		}
		else
		{
			
		}
	}
	
	else if (addr == 200)		//balancing command
	{
		PORTA ^= 32;
		if (data == 1)
		{
			//turn LED on
			cellBalancing = 1;
		}
		else
		{
			//turn LED off
			cellBalancing = 0;
		}
	}*/
}

void UART_sendRealTimeData (void)
{
	/*
	char uartString[20];
	itoa(charger.outputVoltage, uartString, 10);
	uart_puts("Voltage: ");
	uart_puts(uartString);
	uart_puts("\t");
	itoa(charger.outputCurrent, uartString, 10);
	uart_puts("Current: ");
	uart_puts(uartString);
	uart_puts("\t");
	//itoa(charger.status, uartString, 2);	//bits 2 and 3. input and starting state respectively
	uart_puts("Input: ");
	uart_puts((charger.outputStatus & (1<<2)) ? "Wrong":"Normal");
	uart_puts("\t");
	uart_puts("Starting: ");
	uart_puts((charger.outputStatus & (1<<3)) ? "Bad":"Good");
	uart_puts("\r\n");
	*/
	static uint8_t sequence;
	sequence++;
	uint8_t outgoingString[8];
	//preamble section
	outgoingString[0] = 68;					//always start with capital D
	outgoingString[1] = 6;					//payload length (locked at 1 for now)
	outgoingString[2] = 2;					//packet type 
	//charger.outputVoltage = 20;
	outgoingString[3] = sequence;
	outgoingString[4] = charger.outputVoltage>>8;		//charger voltage
	outgoingString[5] = charger.outputVoltage & 0xff;	
	//outgoingString[3] = 0;		//charger voltage
	//outgoingString[4] = 1;
	
	outgoingString[6] = charger.outputCurrent>>8;		//charger current
	outgoingString[7] = charger.outputCurrent & 0xff;
	//outgoingString[5] = 0;		//charger current
	//outgoingString[6] = 20;

	outgoingString[8] = charger.outputStatus;
	
	
	for (int i = 0; i < 8; i++) uart_putc(outgoingString[i]);
}

uint8_t CAN1_Process(uint8_t numBytes, uint8_t * data, uint32_t ID)
{

	uint8_t status = (MCP2515_receive_status(MCP2515_CAN1) & 3);		//poll to see if we have a message waiting.
	if(ID & REPLY_CHARGER_ID)
	{
		charger.outputVoltage = (data[0]<<8) + data[1];
		charger.outputCurrent = (data[2]<<8) + data[3];
		charger.outputStatus = data[4];
		//UART_sendRealTimeData();
		MCP2515_bit_modify(MCP2515_CAN1, MCP2515_CANINTF,0x00, status);				//e.g if status is 0b00000001, it will only modify the 0th pin.
		STATUS_REG01 |= CHARGER_ALIVE;
		return 1;
	}
	else if(ID & REPLY_AMU_ID)
	{
		STATUS_REG01 |= AMU_ALIVE;
		return 1;
	}
	return 0;
}

uint8_t CAN2_Process(uint8_t numBytes, uint8_t * data, uint32_t ID)
{

	return 0;
}

void CAN3_Process()
{
	
}

void error_state(uint16_t error)
{
	switch(error)
	{
		case ERROR_MASTER_TIMEOUT:
		//if the master is not responding
			Charger_deactivate();
			LED_flash(4);			
			break;
		case ERROR_CAN1_RESPONSE:
		//if there is an error with the mcp2515 can processing
			break;
		case ERROR_CHARGER_TIMEOUT:
		//if the charger is not responding
			Charger_deactivate();
			LED_flash(4);
			break;
		case ERROR_AMU_TIMEOUT:
		//if the amu is not responding
			Charger_deactivate();
			LED_flash(4);
			break;
		default:
			break;
	}
}

int main(void)
{
	_delay_ms(10);	//wait for voltages to settle
	IO_init();
	LED_off();
	uart_init(19200);
	SPI_init();
	
	MCP2515_init(MCP2515_CAN1);
	sei();
	//MCP2515_RXInit(MCP2515_CAN1, 0, 0);	//setup the buffer to match to the packet type bits
	MCP2515_reg_write(MCP2515_CAN1, MCP2515_CANINTF, 0b00000000);	//reset the interrupt registers as interrupts won't trigger if it is already low.
	//MCP2515_init(MCP2515_CAN2);
	//MCP2515_init(MCP2515_CAN3);
	
	charger.inputStatus = 1;		//ensure the charger isn't charging
	accumulators[0].flags[0] = 0;	//ensure no balancing is happening
	
	Timer_init();	//begin heartbeat timer and keep alive timeouts
	
	//PORTB |= (1<<PB6);
    while (1) 
    {
		//cli();
		//PROBLEM BUG:
		//When
		//LED_on();
		if(isCharAvailable() > 0)
		{
			
			
			UART_processByte(receiveChar());
		}
		//LED_off();
		if(charger.inputStatus==0)LED_off();
		else
		{
			LED_on();
		}
		if(STATUS_REG01 & CAN1_DataWaiting)
		{
			
			uint8_t status = MCP2515_check_receive_status(MCP2515_CAN1);
			uint8_t data[8];
			uint32_t ID;
			uint8_t numBytes;
			switch(status>>6)
			{
				case 1:
					
					MCP2515_PullCanPacket(MCP2515_CAN1, MCP2515_RXB0SIDH, &numBytes, data, &ID);
					
					//if(CAN1_Process(numBytes, data, ID) == 0)
					CAN1_Process(numBytes, data, ID);
					//MCP2515_bit_modify(MCP2515_CAN1, MCP2515_CANINTF,0x00, status);				//e.g if status is 0b00000001, it will only modify the 0th pin.
					
					break;
				case 2:
					MCP2515_PullCanPacket(MCP2515_CAN1, MCP2515_RXB1SIDH, &numBytes, data, &ID);
					if(CAN1_Process(numBytes, data, ID) == 0)error_state(ERROR_CAN1_RESPONSE);
					break;
				case 3:
					MCP2515_PullCanPacket(MCP2515_CAN1, MCP2515_RXB0SIDH, &numBytes, data, &ID);
					if(CAN1_Process(numBytes, data, ID) == 0)error_state(ERROR_CAN1_RESPONSE);
					MCP2515_PullCanPacket(MCP2515_CAN1, MCP2515_RXB1SIDH, &numBytes, data, &ID);
					if(CAN1_Process(numBytes, data, ID) == 0)error_state(ERROR_CAN1_RESPONSE);
					break;
			}
			
			STATUS_REG01 &= ~CAN1_DataWaiting;
		}
		
		
    }
}

ISR(INT1_vect)	//CAN 1
{
	STATUS_REG01 |= CAN1_DataWaiting;
}

/*ISR(INT0_vect)	//CAN 2
{
	STATUS_REG01 |= CAN2_DataWaiting;
}

ISR(PCINT0_vect)//CAN 3
{
	STATUS_REG01 |= CAN3_DataWaiting;
}*/

ISR(TIMER1_COMPA_vect)
{
	heartbeatTimer++;
	//sendHeartbeat(CHARGER_H, CHARGER_KEEP_ALIVE, 0);
	switch(heartbeatTimer)
	{
		case 12:
			//the charger does not need a specific address, so leave as 0
			sendHeartbeat(CHARGER_H, CHARGER_KEEP_ALIVE, 0);
			break;
		
		case 24:
			sendHeartbeat(AMU_H, NORMAL, ACCUMULATOR_FRONT);
			break;

		case 48:
			break;
		default:
		
			break;
	}
	
	if(heartbeatTimer > 48)heartbeatTimer = 0;		//500ms round robin for comms updates
	
	
	//	sendHeartbeat();			//send can packets to devices at the prescribed intervals
}

ISR(TIMER0_OVF_vect)
{
	
	//if everything is ok, turn the relays on. Otherwise turn them off
	if((STATUS_REG01 & CHARGING) && (STATUS_REG01 & AMU_ALIVE) && (STATUS_REG01 & CHARGER_ALIVE) && (STATUS_REG01 & MASTER_ALIVE))
	{
		Relays_on();
	}
	else
	{
		//Charger_deactivate();
	}
	
	
	if(KEEP_ALIVE_COUNTER >= 92)	// ((16 MHz) / (1024 prescaler) / (255 timer count))*1.5 = 1.5 seconds
	{

		if(STATUS_REG01 & CHARGER_ALIVE)
		{
			STATUS_REG01 &= ~CHARGER_ALIVE;
		}
		else
		{
			error_state(ERROR_CHARGER_TIMEOUT);
		}	
		if (STATUS_REG01 & AMU_ALIVE)
		{
			STATUS_REG01 &= ~AMU_ALIVE;
		}
		else
		{
			error_state(ERROR_AMU_TIMEOUT);
		}
		if (STATUS_REG01 & MASTER_ALIVE)
		{
			STATUS_REG01 &= ~MASTER_ALIVE;
		}
		else
		{
			error_state(ERROR_MASTER_TIMEOUT);
		}
		KEEP_ALIVE_COUNTER = 0;
	}
}