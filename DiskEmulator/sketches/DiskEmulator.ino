#include <RPi_Pico_TimerInterrupt.h>
#include <RPi_Pico_ISR_Timer.h>
#include <RPi_Pico_ISR_Timer.hpp>
#include <SD.h>

#define DEBUG

// Command Bus
#define CMD_RW		2
#define CMD_SEL1	3
#define CMD_SEL0	4

#define BUS_0		5
#define BUS_1		6
#define BUS_2		7
#define BUS_3		8
#define BUS_4		9
#define BUS_5		10
#define BUS_6		11
#define BUS_7		12

#define CMD_STROBE	28

// Internal Control Bins
#define READ_TRIG	0
#define DRV_ACK		1
#define WRITE_TRIG	15

// Physical Drive Signals
#define SECTOR 27
#define SEEK_COMPLETE 25
#define INDEX 22
#define FAULT 21

// Differential Signals
#define RW_OUT 13
#define RW_IN 20
#define CLK 14

// HD Device ID
#define ID 0

// Sector/Index Pulse Variables
RPI_PICO_Timer ITimer(0);
volatile unsigned int currentSector;

bool isSelected = false;
volatile short cylinderAddressRegister = 0;
volatile byte headAddressRegister = 0;

volatile unsigned short incomingCylinderAddress = 0;
volatile byte incomingHeadAddress = 0;

volatile byte statusRegister1 = 0;
#define REG1_READY				0x01
#define REG1_ON_CYLINDER		0x02
#define REG1_SEEKING			0x04
#define REG1_REZEROING			0x08
#define REG1_SERVO_ERROR		0x10
#define REG1_RW_FAULT			0x20
#define REG1_ILLEGAL_ADDRESS	0x40
#define REG1_SPEED_ERROR		0x80

volatile byte statusRegister2 = 0;
#define REG2_RW_UNSAFE		0x01
#define REG2_POR			0x04
#define REG2_PLO_ERROR		0x08
#define REG2_WRITE_PROT		0x10
#define REG2_GUARD_BAND		0x80

byte currentTrackData[10800];

bool SectorIndexPulseISR(__unused struct repeating_timer *t)
{
	currentSector = (currentSector + 1) % 20;

	unsigned int mask = 0;
	
	if (currentSector == 0)
		mask |= ((1 << SECTOR) | (1 << INDEX));
	else
		mask |= (1 << SECTOR);
	
	gpio_set_mask(mask);
	delayMicroseconds(3);
	gpio_clr_mask(mask);
	
	return true;
}

unsigned long powerOnTime;

void setup()
{
	statusRegister1 |= REG1_REZEROING | REG1_SEEKING;
	statusRegister2 |= REG2_POR;
	powerOnTime = millis();
	
	// Setup Command Bus
	pinMode(CMD_RW, INPUT);
	pinMode(CMD_SEL1, INPUT);
	pinMode(CMD_SEL0, INPUT);
	
	for (int i = 0; i < 8; ++i)
	{
		pinMode(BUS_0 + i, INPUT);
	}
	
	pinMode(CMD_STROBE, INPUT);
	
	// Setup internal control signals
	pinMode(DRV_ACK, OUTPUT);
	digitalWrite(DRV_ACK, LOW);
	pinMode(READ_TRIG, INPUT);
	pinMode(WRITE_TRIG, INPUT);
	
	// Setup differential signals
	pinMode(RW_IN, INPUT);
	pinMode(RW_OUT, OUTPUT);
	digitalWrite(RW_OUT, LOW);
	pinMode(CLK, INPUT);
	
}

bool loadTrack()
{
	File f = SD.open("hd0.img", FILE_READ);
	if (f)
	{
		statusRegister2 |= REG1_RW_FAULT;
		digitalWrite(FAULT, HIGH);
		return false;
	}
	
	if (!f.seek((cylinderAddressRegister * 32400) + (headAddressRegister*10800)))
	{
		f.close();
		statusRegister2 |= REG1_RW_FAULT;
		digitalWrite(FAULT, HIGH);
		return false;
	}
	
	if (!f.read(currentTrackData, 10800))
	{
		f.close();
		statusRegister2 |= REG1_RW_FAULT;
		digitalWrite(FAULT, HIGH);
		return false;		
	}
	f.close();
	
	return true;
}

bool saveTrack()
{
	File f = SD.open("hd0.img", FILE_WRITE);
	if (f)
	{
		statusRegister2 |= REG1_RW_FAULT;
		digitalWrite(FAULT, HIGH);
		return false;
	}
	
	if (!f.seek((cylinderAddressRegister * 32400) + (headAddressRegister * 10800)))
	{
		f.close();
		statusRegister2 |= REG1_RW_FAULT;
		digitalWrite(FAULT, HIGH);
		return false;
	}
	
	if (!f.write(currentTrackData, 10800))
	{
		f.close();
		statusRegister2 |= REG1_RW_FAULT;
		digitalWrite(FAULT, HIGH);
		return false;		
	}
	f.close();
	
	return true;
}

void setup1()
{
	// Setup Drive Signals
	pinMode(SECTOR, OUTPUT);
	digitalWrite(SECTOR, LOW);
	pinMode(SEEK_COMPLETE, OUTPUT);
	digitalWrite(SEEK_COMPLETE, LOW);
	pinMode(INDEX, OUTPUT);
	digitalWrite(INDEX, LOW);
	pinMode(FAULT, OUTPUT);
	digitalWrite(FAULT, LOW);
	
	// Start Index and Sector Pulse
	ITimer.attachInterruptInterval(833500, SectorIndexPulseISR);
	
	if (!SD.exists("hd0.img"))
	{
		File f = SD.open("hd0.img", FILE_WRITE);
		for (int i = 0; i < 1140; ++i)
			f.write(currentTrackData, 10800);
		f.close();
	}
	
	cylinderAddressRegister = 0;
	headAddressRegister = 0;
	loadTrack();
	
	statusRegister1 |= REG1_ON_CYLINDER;
	statusRegister1 &= ~REG1_REZEROING & ~REG1_SEEKING;
}

void setDataBusToOutput()
{
	gpio_set_dir_out_masked(0x1FE0);
}

void setDataBusToInput()
{
	gpio_set_dir_in_masked(0x1FE0);
}

void writeDataBus(byte data)
{
	gpio_put_masked(0x1FE0, (int)data << 5);
}

byte readDataBus(int data)
{
	return ((data & 0x1FE0) >> 5);
}

byte readCommand(int data)
{
	return ((data & 0x1C) >> 2);
}

byte extractUnitId(int data)
{
	return (readDataBus(data) >> 4);
}

void setIncomingCylinderAddressHigh(int data)
{
	incomingCylinderAddress = (incomingCylinderAddress & ~0x300) | (readDataBus(data) << 8);
}

void setIncomingCylinderAddressLow(int data)
{
	incomingCylinderAddress = (incomingCylinderAddress & ~0xFF) | readDataBus(data);
}

void setIncomingHeadAddress(int data)
{
	incomingHeadAddress = ((readDataBus(data) >> 2) & 0x03);
}

void loop()
{
	if ((statusRegister2 & REG2_POR) != 0 && millis() - powerOnTime > 12000)
	{
		statusRegister2 &= ~REG2_POR;
		statusRegister1 |= REG1_READY;
	}
	
	while (!digitalRead(CMD_STROBE)) ;
#ifdef DEBUG
	Serial.println("CMD_STROBE went HIGH");
#endif
	
	int data = gpio_get_all();
	
	byte command = readCommand(data);
	
#ifdef DEBUG
	Serial.print("Received command byte: ");
	Serial.print(command);
	Serial.print(" for ");
	Serial.print(extractUnitId(data));
	Serial.print(" with ");
	Serial.println(readDataBus(data), BIN);
#endif
	
	switch (command)
	{
		byte unitSelect;
		
	case 0:
		isSelected = extractUnitId(data) == ID;
		
		if (isSelected)
		{
			setIncomingHeadAddress(data);
			setIncomingCylinderAddressHigh(data);
#ifdef DEBUG
			Serial.println("Setting Head and High bits of CylinderRegister");
#endif
			if (incomingCylinderAddress > 353)
			{
				statusRegister1 |= REG1_ILLEGAL_ADDRESS;
			}
			else
			{
				statusRegister1 &= ~REG1_ILLEGAL_ADDRESS & ~REG1_ON_CYLINDER;
				statusRegister1 |= REG1_SEEKING;
			}
		}
		else
		{
			isSelected = false;
		}
		break;
	case 4:  // CMD BYTE 1
		if (isSelected)
		{
			setIncomingCylinderAddressLow(data);
#ifdef DEBUG
			Serial.println("Setting Low bits of CylinderRegister");
#endif
			if (incomingCylinderAddress > 353)
			{
				statusRegister1 |= REG1_ILLEGAL_ADDRESS;
			}
			else
			{
				statusRegister1 &= ~REG1_ILLEGAL_ADDRESS & ~REG1_ON_CYLINDER;
				statusRegister1 |= REG1_SEEKING;
			}
		}
		break;
	case 2:  // CMD BYTE 2
		if (isSelected)
		{
			// Start Read or Write
		}
		break;
	case 6:  // CMD BYTE 3
		if (isSelected)
		{
			if ((data & 0x02) != 0)  // REZERO
			{
#ifdef DEBUG
				Serial.println("Rezero-ing Drive");
#endif
				incomingCylinderAddress = 0;
				incomingHeadAddress = 0;
				digitalWrite(FAULT, LOW);
				statusRegister1 &= ~REG1_ILLEGAL_ADDRESS & ~REG1_ON_CYLINDER & ~REG1_RW_FAULT;
				statusRegister1 |= REG1_SEEKING | REG1_REZEROING;
			}
			else if ((data & 0x01) != 0) // FAULT CLEAR
			{
#ifdef DEBUG
				Serial.println("Clearing Fault");
#endif
				statusRegister1 &= ~REG1_RW_FAULT;
				digitalWrite(FAULT, LOW);
			}
		}
		break;
	case 1:  // CMD BYTE 4
		if (isSelected)
		{
#ifdef DEBUG
			Serial.println("Outputting Register1");
#endif
			setDataBusToOutput();
			writeDataBus(statusRegister1);
		}
		break;
	case 5:  // CMD BYTE 5
		if (isSelected)
		{
#ifdef DEBUG
			Serial.println("Outputting Register2");
#endif
			setDataBusToOutput();
			writeDataBus(statusRegister2);
		}
		break;
	case 3:  // CMD BYTE 6
		if (isSelected)
		{
#ifdef DEBUG
			Serial.println("Outputting low bits of CylinderAddress Register");
#endif
			setDataBusToOutput();
			writeDataBus(cylinderAddressRegister & 0xFF);
		}
		break;
	case 7:  // CMD BYTE 7
		if (isSelected)
		{
# ifdef DEBUG
			Serial.println("Outputting ID, HeadAddress Register and high bits of CylinderAddress Register");
#endif
			setDataBusToOutput();
			byte returnValue = ((ID & 0x0F) << 4) | ((headAddressRegister & 0x03)) << 2 | ((cylinderAddressRegister & 0x300) >> 8);
			writeDataBus(returnValue);
		}
		break;
	}
	
#ifdef DEBUG
	Serial.println("Setting DRV_ACK HIGH");
#endif
	digitalWrite(DRV_ACK, HIGH);	
	
	while (digitalRead(CMD_STROBE)) ;
#ifdef DEBUG
	Serial.println("CMD_STROBE went LOW");
#endif
	
#ifdef DEBUG
	Serial.println("Setting bus pins back to INPUT");
#endif
	setDataBusToInput();
#ifdef DEBUG
	Serial.println("Setting DRV_ACK LOW");
#endif
	digitalWrite(DRV_ACK, LOW);
}

void loop1()
{
	
}