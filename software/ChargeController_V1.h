/*
 * ChassisController_V1.h
 *
 * Created: 5/12/2016 1:55:19 AM
 * Author : julius
 */ 

#ifndef CHARGECONTROLLER_H
#define CHARGECONTROLLER_H

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "MCP2515_CC.h"
#include <stdlib.h>



#define NUM_CHARGER 1
#define NUM_ACCUMULATORS	1
#define NUM_CMU_PER_AMU	6
#define CMU_CELL_COUNT	8

#define AMU_H	3
#define CHARGER_H 1

#define STATUS_REG01 GPIOR0
#define CAN1_DataWaiting	1
#define CAN2_DataWaiting	2
#define CAN3_DataWaiting	4
#define CHARGER_ALIVE		8
#define AMU_ALIVE			16
#define CHARGING			32
#define MASTER_ALIVE		64
#define BALANCING			128

#define STATUS_REG02 GPIOR1


#define KEEP_ALIVE_COUNTER GPIOR2

#define	HEARTBEAT_AMU_ID	0x0A000000		//Charge Controller to AMU
#define REPLY_AMU_ID		0x02000000		//AMU to Charge Controller
#define	HEARTBEAT_CHARGER_ID	0x1806E5F4	//Charge Controller to charger
#define REPLY_CHARGER_ID	0x18FF50E5		//Charger to Charge Controller


//ADC channel allocation

//single sensor, no error checking

#define ADC_SAMPLES	10

#define ERROR_MASTER_TIMEOUT	2
#define ERROR_CAN1_RESPONSE		3
#define ERROR_CHARGER_TIMEOUT	4
#define ERROR_AMU_TIMEOUT		5



//shutdown circuits
//seg 4 pin 63 dig 1 pinj0	//inertia switch
//seg 5 pin 64 dig 2 pinj1	//left front upright
//seg 6 pin 66 dig 3 pinj3	//brake overtravel
//seg 7 pin 68 dig 4 pinj5	//right front upright
//seg 8 pin 77 dig 5 pina1	//driver E-Stop

//digital inputs
//dig 6 pin 90	//driver switch

//digital outputs
//pin 25	//driver lamp a
//pin 24	//driver lamp b




unsigned int avgVolts = 0;
unsigned int avgTemp = 0;
unsigned int minVolts = 0;
unsigned int minTemp = 0;
unsigned int maxVolts = 0;
unsigned int maxTemp = 0;

int testValue1;
int testValue2;

unsigned int temps[0];
unsigned int voltages[8];


unsigned char cellBalancing = 0;
unsigned int AN1_voltage = 0;
unsigned int AN2_voltage = 0;


unsigned char operatingMode = 0;

unsigned int T1_temp = 0;
typedef struct Charger
{
	uint16_t inputVoltage;
	uint16_t outputVoltage;
	uint16_t inputCurrent;
	uint16_t outputCurrent;
	uint8_t inputStatus;
	uint8_t outputStatus;
}Charger;
typedef struct CMU
{
	uint8_t CMU_num, CMU_ID;
	uint16_t voltages[CMU_CELL_COUNT];	//will change with change of parameter. Unlikely to.
	uint16_t temperatures[CMU_CELL_COUNT];
}CMU;

typedef struct Accumulator
{
	uint8_t flags[4];
	uint8_t ID;
	uint16_t MinV;
	uint16_t MaxV;
	uint16_t AvgV;
	uint16_t AvgT;
	uint16_t MinT;
	uint16_t MaxT;
	CMU cmus[NUM_CMU_PER_AMU];
}Accumulator;

struct Accumulator	accumulators[NUM_ACCUMULATORS];
struct Charger charger;

unsigned int testTimer = 0;
unsigned char CANreceiver = 0;

volatile int heartbeatTimer = 0;
char CANdiagnostics[10][20];
uint8_t tempBuffer[10];

#define CANselect PORTC &= ~1
#define CANdeselect	  PORTC |= 1

void LED_flash(unsigned char times);

#define NORMAL 1
#define CHARGER_KEEP_ALIVE 2
#define CHARGER_SET_PARAMETERS 3
#define ACCUMULATOR_FRONT 1
#define ACCUMULATOR_REAR 2


void shutdown_state(uint16_t shutdownFlag);
void UART_sendRealTimeData (void);
void UART_parseInput(unsigned char* s);
void UART_processByte(char data);
void UART_processPacket(unsigned char* s);
void CAN2ascii(uint8_t CANdata,	char * CANresult);
void sendHeartbeat(unsigned char destination, unsigned char type, unsigned char address);

#endif