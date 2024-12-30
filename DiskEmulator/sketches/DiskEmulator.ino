#include <RPi_Pico_TimerInterrupt.h>
#include <RPi_Pico_ISR_Timer.h>
#include <RPi_Pico_ISR_Timer.hpp>
#include <SD.h>

#include "read_data.h"
#include "write_data.h"
#include "read_trig.h"
#include "write_trig.h"

#define DEBUG

// Command Bus
#define CMD_SEL0	2
#define CMD_SEL1	3
#define CMD_RW		4


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
#define SYS_CLOCK 14

#define SDPIN 17

// HD Device ID
#define ID 0

#define IS_PIN_HIGH(x) (gpio_get_all() & (1 << x)) != 0
#define IS_PIN_LOW(x) (gpio_get_all() & (1 << x)) == 0
#define SET_PIN_HIGH(x) gpio_set_mask(1 << x)
#define SET_PIN_LOW(x) gpio_clr_mask(1 << x)

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

#define  read_pio pio0
#define  write_pio pio1

bool __not_in_flash_func(SectorIndexPulseISR)(__unused struct repeating_timer *t)
{
	currentSector = (currentSector + 1) % 20;

	unsigned int mask = 0;
	
	if (currentSector == 0)
		mask |= ((1 << SECTOR) | (1 << INDEX));
	else
		mask |= (1 << SECTOR);
	
	gpio_set_mask(mask);
	delayMicroseconds(4);
	gpio_clr_mask(mask);
	
	return true;
}

volatile bool setupHandshake = false;

volatile bool readTrigger = false;

void __not_in_flash_func(readTrigRaiseISR)()
{
	readTrigger = true;
	pio_interrupt_clear(read_pio, 0);
}

void __not_in_flash_func(readTrigLowerISR)()
{
	readTrigger = false;
	pio_interrupt_clear(read_pio, 1);
}

volatile bool writeTrigger = false;

void __not_in_flash_func(writeTrigRaiseISR)()
{
	writeTrigger = true;
	pio_interrupt_clear(write_pio, 0);
}

void __not_in_flash_func(writeTrigLowerISR)()
{
	writeTrigger = false;
	pio_interrupt_clear(write_pio, 1);
}

void setup()
{	
	// Setup internal control signals
	gpio_init_mask(1 << DRV_ACK | 1 << READ_TRIG | 1 << WRITE_TRIG | 1 << SYS_CLOCK);
	gpio_set_dir_in_masked(1 << READ_TRIG | 1 << WRITE_TRIG | 1 << SYS_CLOCK);
	
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
	
	// Setup PIO 0 state machines
	uint offset = pio_add_program(read_pio, &read_data_program);
	read_data_program_init(read_pio, 0, offset);

	offset = pio_add_program(read_pio, &read_trig_program);
	read_trig_program_init(read_pio, 1, offset);
	
	pio_set_irq0_source_enabled(read_pio, pis_interrupt0, true);
	irq_set_exclusive_handler(PIO0_IRQ_0, readTrigRaiseISR);
	irq_set_enabled(PIO0_IRQ_0, true);
	
	pio_set_irq1_source_enabled(read_pio, pis_interrupt1, true);
	irq_set_exclusive_handler(PIO0_IRQ_1, readTrigLowerISR);
	irq_set_enabled(PIO0_IRQ_1, true);

	// Setup PIO 1 state machines
	offset = pio_add_program(write_pio, &write_data_program);
	write_data_program_init(write_pio, 0, offset);
	
	offset = pio_add_program(write_pio, &write_trig_program);
	write_trig_program_init(write_pio, 1, offset);

	pio_set_irq0_source_enabled(write_pio, pis_interrupt0, true);
	irq_set_exclusive_handler(PIO1_IRQ_0, writeTrigRaiseISR);
	irq_set_enabled(PIO1_IRQ_0, true);
	
	pio_set_irq0_source_enabled(write_pio, pis_interrupt1, true);
	irq_set_exclusive_handler(PIO1_IRQ_1, writeTrigLowerISR);
	irq_set_enabled(PIO1_IRQ_1, true);
	
#ifdef DEBUG
	Serial.printf("Disk is spun up.  Initializing Command Processor...\r\n");
#endif
	
	gpio_set_mask(1 << DRV_ACK);
	delay(500);
	gpio_clr_mask(1 << DRV_ACK);
	
	while ((statusRegister1 & REG1_SEEKING) != 0)
	
	// Mark setup finished, by lighting LED
	gpio_init_mask(1 << LED_BUILTIN);
	gpio_set_dir_out_masked(1 << LED_BUILTIN);
	SET_PIN_HIGH(LED_BUILTIN);
	
	statusRegister1 |= REG1_READY;
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
		statusRegister1 |= REG1_RW_FAULT;
		statusRegister2 |= REG2_RW_UNSAFE;
		gpio_set_mask(1 << FAULT);
		return false;
	}
	
	if (!f.seek((cylinderAddressRegister * 32400) + (headAddressRegister * 10800)))
	{
		f.close();
		statusRegister1 |= REG1_RW_FAULT;
		statusRegister2 |= REG2_RW_UNSAFE;
		gpio_set_mask(1 << FAULT);
		return false;
	}
	
	if (!f.write((const uint8_t*)currentTrackData, 10800))
	{
		f.close();
		statusRegister1 |= REG1_RW_FAULT;
		statusRegister2 |= REG2_RW_UNSAFE;
		gpio_set_mask(1 << FAULT);
		return false;		
	}
	f.close();
	
	return true;
}

bool ConfigureSDCard()
{
	bool rwFailure = false;
	
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
						f.close();
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
						}

					}
					
					if (!rwFailure)
					{
#ifdef DEBUG
						Serial.println("Added initial sync bits.");
#endif
					}
					f.close();
				}
			}
		}
		else
		{
#ifdef DEBUG
			Serial.println("hd0.img found.");
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
				if (!f.seek(12312000 - 1)) 
				{
#ifdef DEBUG
					Serial.println("hd0.img is the wrong size.  Most likely it is corrupt.");
#endif
					rwFailure = true;
					statusRegister2 |= REG1_RW_FAULT | REG2_RW_UNSAFE;
					gpio_set_mask(1 << FAULT);
				}

				f.close();
			}	

		}
	}
	
	return !rwFailure;
}

void setup1()
{

	while (!setupHandshake) ;
	
	// Setup Drive Signals
	gpio_init_mask(1 << SECTOR | 1 << SEEK_COMPLETE | 1 << INDEX | 1 << FAULT);
	gpio_set_dir_out_masked(1 << SECTOR | 1 << SEEK_COMPLETE | 1 << INDEX | 1 << FAULT);
	gpio_clr_mask(1 << SECTOR | 1 << SEEK_COMPLETE | 1 << INDEX | 1 << FAULT);
	
#ifdef DEBUG
	Serial.println("Start Index and Sector Pulse");
#endif
	
	// Start Index and Sector Pulse
	ITimer.attachInterruptInterval(833.5, SectorIndexPulseISR);
	
	// Setup SD card access
	if (ConfigureSDCard())
	{	
		cylinderAddressRegister = 0;
		headAddressRegister = 0;
		if (loadTrack())
		{
			statusRegister1 |= REG1_ON_CYLINDER;
			statusRegister1 &= ~REG1_REZEROING & ~REG1_SEEKING;			
		}
	}
	
	setupHandshake = false;
}

void setDataBusToOutput()
{
	gpio_set_dir_out_masked(0xFF << BUS_0);
}

void setDataBusToInput()
{
	gpio_set_dir_in_masked(0xFF << BUS_0);
}

void writeDataBus(byte data)
{
	gpio_put_masked(0xFF << BUS_0, ~((int)data << BUS_0));
}

byte readDataBus(int pins)
{
	return ~(pins >> BUS_0);
}

byte getCommand(int pins)
{
	return (pins >> CMD_SEL0) & 0x03;
}

byte getUnitID(int pins)
{
	return (readDataBus(pins) & 0xF0) >> 0x04;
}

void setIncomingCylinderAddressHigh(int pins)
{
	incomingCylinderAddress = (incomingCylinderAddress & ~(0x03 << 0x08)) | (readDataBus(pins) << 8) ;
}

void setIncomingCylinderAddressLow(int pins)
{
	incomingCylinderAddress = (incomingCylinderAddress & ~0xFF) | readDataBus(pins);
}

void setIncomingHeadAddress(int pins)
{
	incomingHeadAddress = ((readDataBus(pins) >> 2) & 0x03);
}

void __not_in_flash_func(DoReadWrite)()
{
	SET_PIN_HIGH(DRV_ACK);
	
	while (IS_PIN_HIGH(CMD_STROBE))
	{
		if (readTrigger)
		{
			int sectorNum = currentSector;
	
			// Always do a short read
			for (int i = 0; i < 25; ++i)
			{
				read_data_putc(read_pio, 0, currentTrackData[(sectorNum * 540) + i]);
			}
			
			if (readTrigger) // If readTrigger is still HIGH, continue with full read
			{
				for (int i = 0; i < 515 /* 540 Total */; ++i)
				{
					read_data_putc(read_pio, 0, currentTrackData[(sectorNum * 540) + i]);
				}
			}
		}
		else if (writeTrigger)
		{
			int sectorNum = currentSector;
			int i = 0;
			while (writeTrigger || write_data_has_data(write_pio, 0))  // Need to purge the OSR even if writeTrigger has gone LOW
			{
				currentTrackData[(sectorNum * 540) + i] = write_data_getc(write_pio, 0);
				++i;
			}
		}
	}
}

void loop()
{
	while (IS_PIN_LOW(CMD_STROBE)) ;
	
	int pins = gpio_get_all();
	
	byte command = getCommand(pins);
	
#ifdef DEBUG
	Serial.print("Received command ");
	Serial.print(command);
	Serial.print(" with data: ");
	Serial.println(readDataBus(pins), HEX);
#endif
	
	switch (command)
	{
		byte unitSelect;
		
	case 0:
		isSelected = getUnitID(pins) == ID;
	
		if (isSelected)
		{
#ifdef DEBUG
			Serial.printf("Activating drive with address %d.\r\n", ID);
#endif
			setIncomingHeadAddress(pins);
			setIncomingCylinderAddressHigh(pins);
		}
		else
		{
#ifdef DEBUG
			Serial.println("Deactivating drive.");
#endif
			isSelected = false;
		}
		break;
	case 1: 
		if (isSelected)
		{
			setIncomingCylinderAddressLow(pins);

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
			DoReadWrite();
		}
		break;
	case 3:  // CMD BYTE 3
		if (isSelected)
		{
			if ((pins & 0x02) != 0)  // REZERO
			{
#ifdef DEBUG
				Serial.println("Rezero-ing Drive");
#endif
				if (ConfigureSDCard())
				{
					incomingCylinderAddress = 0;
					incomingHeadAddress = 0;
				
					statusRegister1 &= ~REG1_ILLEGAL_ADDRESS & ~REG1_ON_CYLINDER & ~REG1_RW_FAULT;
					statusRegister2 &= ~REG2_RW_UNSAFE;
					statusRegister1 |= REG1_SEEKING | REG1_REZEROING;
					SET_PIN_LOW(FAULT);
				}
			}
			else if ((pins & 0x01) != 0) // FAULT CLEAR
			{
#ifdef DEBUG
				Serial.println("Clearing Fault");
#endif
				if (ConfigureSDCard())
				{
					statusRegister1 &= ~REG1_RW_FAULT;
					statusRegister2 &= ~REG2_RW_UNSAFE;
					SET_PIN_LOW(FAULT);
				}
			}
		}
		break;
	case 4:  // CMD BYTE 4
		if (isSelected)
		{
#ifdef DEBUG
			Serial.println("Outputting Register 1");
#endif
			setDataBusToOutput();
			writeDataBus(statusRegister1);
		}
		break;
	case 5:  // CMD BYTE 5
		if (isSelected)
		{
#ifdef DEBUG
			Serial.println("Outputting Register 2");
#endif
			setDataBusToOutput();
			writeDataBus(statusRegister2);
		}
		break;
	case 6:  // CMD BYTE 6
		if (isSelected)
		{
#ifdef DEBUG
			Serial.println("Outputting Current CMD BYTE 2 Settings");
#endif
			setDataBusToOutput();
			writeDataBus(cylinderAddressRegister & 0xFF);
		}
		break;
	case 7:  // CMD BYTE 7
		if (isSelected)
		{
# ifdef DEBUG
			Serial.println("Outputting Current CMD BYTE 2 Settings");
#endif
			setDataBusToOutput();
			byte returnValue = ((ID & 0x0F) << 4) | ((headAddressRegister & 0x03)) << 2 | ((cylinderAddressRegister & 0x300) >> 8);
			writeDataBus(returnValue);
		}
		break;
	}
	
	SET_PIN_HIGH(DRV_ACK);	

	while (IS_PIN_HIGH(CMD_STROBE)) ;
	
	setDataBusToInput();
	
	SET_PIN_LOW(DRV_ACK);
}

void loop1()
{	
	bool success;
	
	if (flushTrack && (statusRegister2 & REG2_RW_UNSAFE) == 0)
	{
		if (saveTrack())
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
		delayMicroseconds(4);
		gpio_clr_mask(1 << SEEK_COMPLETE);                                                                                                                                                                                           
		
	}
}