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
#define WRITE_TRIG	1
#define DRV_ACK	15

// Physical Drive Signals
#define SECTOR 27
#define SEEK_COMPLETE 26
#define INDEX 22
#define FAULT 21

// Differential Signals
#define RW_OUT 13
#define RW_IN 20
#define CLK 14

#define LED 25

#define SDPIN 17

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

volatile byte currentTrackData[10800];
volatile bool flushTrack = false;

PIO rw_pio = pio0;

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

volatile bool setupHandshake = false;

void setup()
{	
	// Setup internal control signal
	gpio_init_mask(1 << DRV_ACK);
	gpio_set_dir_out_masked(1 << DRV_ACK);
	gpio_clr_mask(1 << DRV_ACK);

#ifdef DEBUG
	Serial.begin(115200);
	while (!Serial) ;
#endif
	
	statusRegister1 |= REG1_REZEROING & REG1_SEEKING;
	
	setupHandshake = true;
	
#ifdef DEBUG
	Serial.print("Waiting for disk to \"spin up\"...");
#endif
	
	while (setupHandshake) ;
	
#ifdef DEBUG
	Serial.println("...Disk is finished \"spinning up\".");
#endif
	
	// Setup Command Bus
	gpio_init_mask(1 << CMD_RW | 1 << CMD_SEL1 | 1 << CMD_SEL0 | 0xFF << BUS_0 | 1 << CMD_STROBE);
	
	gpio_set_dir_in_masked(1 << CMD_RW | 1 << CMD_SEL1 | 1 << CMD_SEL0 | 0xFF << BUS_0 | 1 << CMD_STROBE);
	
	//TODO: These would be PIO signals
	//gpio_set_dir_in_masked(1 << READ_TRIG | 1 << WRITE_TRIG);
	
	// Setup differential signals
	// TODO: These would be set up as PIO signals
	//gpio_set_dir_in_masked(1 << RW_IN);
	//gpio_set_dir_out_masked(1 << RW_OUT);
	//gpio_clr_mask(1 << RW_OUT);
	//gpio_set_dir_in_masked(1 << CLK);
	
#ifdef DEBUG
	Serial.printf("Disk is spun up.  Starting Command Processor...\r\n");
#endif
	
	gpio_set_mask(1 << DRV_ACK);
	delay(500);
	gpio_clr_mask(1 << DRV_ACK);
	
	// Mark setup finished, by lighting LED
	gpio_init_mask(1 << LED);
	gpio_set_dir_out_masked(1 << LED);
	gpio_set_mask(1 << LED);
	
	
}

bool loadTrack()
{
	File f = SD.open("hd0.img", FILE_READ);
	if (!f)
	{
		statusRegister2 |= REG1_RW_FAULT | REG2_RW_UNSAFE;
		gpio_set_mask(1 << FAULT);
		return false;
	}
	
	if (!f.seek((cylinderAddressRegister * 32400) + (headAddressRegister*10800)))
	{
		f.close();
		statusRegister2 |= REG1_RW_FAULT;
		gpio_set_mask(1 << FAULT);
		return false;
	}
	
	if (!f.read((uint8_t*)currentTrackData, 10800))
	{
		f.close();
		statusRegister2 |= REG1_RW_FAULT;
		gpio_set_mask(1 << FAULT);
		return false;		
	}
	f.close();
	
	return false;
}

bool saveTrack()
{
	File f = SD.open("hd0.img", FILE_WRITE);
	if (!f)
	{
		statusRegister2 |= REG1_RW_FAULT;
		gpio_set_mask(1 << FAULT);
		return false;
	}
	
	if (!f.seek((cylinderAddressRegister * 32400) + (headAddressRegister * 10800)))
	{
		f.close();
		statusRegister2 |= REG1_RW_FAULT;
		gpio_set_mask(1 << FAULT);
		return false;
	}
	
	if (!f.write((const uint8_t*)currentTrackData, 10800))
	{
		f.close();
		statusRegister2 |= REG1_RW_FAULT;
		gpio_set_mask(1 << FAULT);
		return false;		
	}
	f.close();
	
	return true;
}

void setup1()
{
	bool rwFailure = false;
	while (!setupHandshake) ;
	
	// Setup Drive Signals
	gpio_init_mask(1 << SECTOR | 1 << SEEK_COMPLETE | 1 << INDEX | 1 << FAULT);
	gpio_set_dir_out_masked(1 << SECTOR | 1 << SEEK_COMPLETE | 1 << INDEX | 1 << FAULT);
	gpio_clr_mask(1 << SECTOR | 1 << SEEK_COMPLETE | 1 << INDEX | 1 << FAULT);
	
#ifdef DEBUG
	Serial.println("Start Index and Sector Pulse");
#endif
	
	// Start Index and Sector Pulse
	ITimer.attachInterruptInterval(833500, SectorIndexPulseISR);
	
	for (int i = 0; i < 10800; ++i)
		currentTrackData[i] = 0;
	
	if (!SD.begin(SDPIN))
	{
#ifdef DEBUG
		Serial.println("Error intializing SD card.");
#endif
		statusRegister2 |= REG1_RW_FAULT | REG2_RW_UNSAFE;
		gpio_set_mask(1 << FAULT);
	}
	else
	{
#ifdef DEBUG
		Serial.println("SD card intialized.");
#endif	
		if (!SD.exists("hd0.img"))
		{
#ifdef DEBUG
			Serial.println("hd0.img not found, creating hd0.img.");
#endif
			File f = SD.open("hd0.img", FILE_WRITE);
			if (!f)
			{
				rwFailure = true;
				statusRegister2 |= REG1_RW_FAULT | REG2_RW_UNSAFE;
				gpio_set_mask(1 << FAULT);
			}
			else
			{
				for (int i = 0; i < 1140; ++i)
				{
					int wrote = f.write((const uint8_t*)currentTrackData, 10800);
					if (wrote < 10800)
					{
						rwFailure = true;
						statusRegister2 |= REG1_RW_FAULT | REG2_RW_UNSAFE;
						gpio_set_mask(1 << FAULT);
						break;
					}
					
				}
				
				if (!rwFailure)
				{
#ifdef DEBUG
					Serial.println("Sized hd0.img to 12312000 bytes.");
#endif	
					// Place the sync bit in the first 20 sectors
					for (int i = 0; i < 20; ++i)
					{
						bool didSeek = f.seek((i * 540) + 25);
						if (!didSeek)
						{
							rwFailure = true;
							statusRegister2 |= REG1_RW_FAULT | REG2_RW_UNSAFE;
							gpio_set_mask(1 << FAULT);
							break;
						}
						else
						{
							int wrote = f.write((uint8_t)1);
							if (wrote != 1)
							{
								rwFailure = true;
								statusRegister2 |= REG1_RW_FAULT | REG2_RW_UNSAFE;
								gpio_set_mask(1 << FAULT);
								break;
							}
							else
							{
#ifdef DEBUG
								Serial.println("Added initial sync bits.");
#endif
								f.close();
							}
						}

					}
				}
			}
		}
		else
		{
#ifdef DEBUG
			Serial.println("hd0.img found.");
#endif
		}
		
		if (!rwFailure)
		{
			
			cylinderAddressRegister = 0;
			headAddressRegister = 0;
			loadTrack();
	
			statusRegister1 |= REG1_ON_CYLINDER;
			statusRegister1 &= ~REG1_REZEROING & ~REG1_SEEKING;
		}
	}

	
	setupHandshake = false;
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

	while (!gpio_get(CMD_STROBE)) ;
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
			//			
			//			while (!readGate) ;
			//			
			//			// Short Read
			//			int sectorNum = currentSector;
			//			for (int i = 0; i < 25; ++i)
			//			{
			//				// put_byte_block(currentTrackData[sectorNum * 540])
			//			}
			//			
			//			while (!readGate && !writeGate && cmdStrobe) ;
			//			
			//			if (readGate)
			//			{
			//				for (int i = 0; i < 540; ++i)
			//				{
			//					//put_byte_block(currentTrackData[sectorNum * 540])
			//				}
			//			}
			//			else if (writeGate)
			//			{
			//				int sectorNum = currentSector;
			//				for (int i = 0; i < 540; ++i)
			//				{
			//					//currentTrackData[sectorNum * 540] = get_byte_block;
			//				}				
			//			}
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
				gpio_clr_mask(1 << FAULT);
				statusRegister1 &= ~REG1_ILLEGAL_ADDRESS & ~REG1_ON_CYLINDER & ~REG1_RW_FAULT;
				statusRegister1 |= REG1_SEEKING | REG1_REZEROING;
			}
			else if ((data & 0x01) != 0) // FAULT CLEAR
			{
#ifdef DEBUG
				Serial.println("Clearing Fault");
#endif
				statusRegister1 &= ~REG1_RW_FAULT;
				statusRegister2 &= ~REG2_RW_UNSAFE;
				gpio_clr_mask(1 << FAULT);
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
	
	if (command != 2)
	{
#ifdef DEBUG
		Serial.println("Setting DRV_ACK HIGH");
#endif
		gpio_set_mask(1 << DRV_ACK);	
	}

	while (gpio_get(CMD_STROBE)) ;
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
	gpio_clr_mask(1 << DRV_ACK);
}

void loop1()
{	
	if (flushTrack)
	{
		saveTrack();
		flushTrack = false;
	}
	
	if ((statusRegister1 & REG1_SEEKING) != 0)
	{
		if (headAddressRegister != incomingHeadAddress || cylinderAddressRegister != incomingCylinderAddress)
		{
			headAddressRegister = incomingHeadAddress;
			cylinderAddressRegister = incomingCylinderAddress;
	
			if (loadTrack())
			{
				statusRegister1 |= REG1_ON_CYLINDER;
			}
		}
		else
		{
			statusRegister1 |= REG1_ON_CYLINDER;
		}
		
		statusRegister1 &= ~REG1_SEEKING & ~REG1_REZEROING;
		
		gpio_set_mask(1 << SEEK_COMPLETE);
		delayMicroseconds(3);
		gpio_clr_mask(1 << SEEK_COMPLETE);                                                                                                                                                                                           
	}
}