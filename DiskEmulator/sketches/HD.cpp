#include "HD.h"

#include <SD.h>
#include <RPi_Pico_TimerInterrupt.h>
#include <RPi_Pico_ISR_Timer.h>
#include <RPi_Pico_ISR_Timer.hpp>

#include "rw_clock.h"
#include "read_data.h"
#include "read_trig.h"
#include "write_data.h"
#include "write_trig.h"


// Sector/Index Pulse Variables
static RPI_PICO_Timer ITimer(0);
static volatile unsigned int currentSector;	

static volatile byte currentTrackData[8][10800];
static volatile bool flushTrack[8] = { false };
static volatile int lastSeekTime[8];

static volatile short cylinderAddressRegister[8] = { 0 };
static volatile byte headAddressRegister[8] = { 0 };
static volatile unsigned short incomingCylinderAddress[8] = { 0 };
static volatile byte incomingHeadAddress[8] = { 0 };
	
static volatile byte statusRegisters[8][2] = { { 0 } };

static volatile bool setupHandshake = false;

static volatile bool isSelected = false;
static volatile byte activeDrive = 0;
static volatile byte existingDrives = 0;

static volatile bool writeTrigger = false;
static volatile bool readTrigger = false;


static void __not_in_flash_func(readTrigRaiseISR)()
{
	readTrigger = true;
	pio_interrupt_clear(read_pio, 0);
}

static void __not_in_flash_func(readTrigLowerISR)()
{
	readTrigger = false;
	pio_interrupt_clear(read_pio, 1);
}

static void __not_in_flash_func(writeTrigRaiseISR)()
{
	writeTrigger = true;
	pio_interrupt_clear(write_pio, 0);
}

static void __not_in_flash_func(writeTrigLowerISR)()
{
	writeTrigger = false;
	pio_interrupt_clear(write_pio, 1);
}



static bool __not_in_flash_func(SectorIndexPulseISR)(__unused struct repeating_timer *t)
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

static void __not_in_flash_func(DoReadWrite)()
{
	bool escape = false;
	
	while (IS_PIN_HIGH(CMD_STROBE))
	{	
		while (!escape && readTrigger == 0 && writeTrigger == 0) ;
		
		bool longRead = false;
		
		if (readTrigger)
		{
#ifdef DEBUG
			Serial.println("DEBUG: Starting Read.");
#endif
			int sectorNum = currentSector;
	
			// Always do a short read
			for (int i = 0; i < 25; ++i)
			{
				byte c = currentTrackData[activeDrive][(sectorNum * 540) + i];
				read_data_putc(read_pio, 0, c);	
			}
			
			if (readTrigger) // If readTrigger is still HIGH, continue with full read
			{
				for (int i = 0; i < 515 /* 540 Total */; ++i)
				{
					read_data_putc(read_pio, 0, currentTrackData[activeDrive][(sectorNum * 540) + i]);
				}
				longRead = true;
			}
			escape = true;
#ifdef DEBUG
			Serial.println("DEBUG: " + String(longRead ? "Long" : "Short") + " Read Finished");
#endif
		}
		
		if (writeTrigger)
		{
#ifdef DEBUG
			Serial.println("DEBUG: Starting Write.");
			int t = micros();
#endif
			int sectorNum = currentSector;
			int i = 0;
			do   
			{
				currentTrackData[activeDrive][(sectorNum * 540) + i] = write_data_getc(write_pio, 0);
				i++;
			} while (writeTrigger);
			
#ifdef DEBUG
			Serial.println("DEBUG: Write Finished, Bytes Written: " + String(i) + " in " + String(micros()-t) + "us.");
#endif
			pio_sm_clear_fifos(write_pio, 0);
			flushTrack[activeDrive] = true;
			escape = true;
		}
	}
}

static bool loadTrack()
{
	File f = SD.open("hd0.img", FILE_READ);
	if (!f)
	{
		statusRegisters[activeDrive][0] |= REG1_RW_FAULT;
		statusRegisters[activeDrive][1] |= REG2_RW_UNSAFE;
		gpio_set_mask(1 << FAULT);
		return false;
	}
	
	if (!f.seek((cylinderAddressRegister[activeDrive] * 32400) + (headAddressRegister[activeDrive] * 10800)))
	{
		f.close();
		statusRegisters[activeDrive][0] |= REG1_RW_FAULT;
		gpio_set_mask(1 << FAULT);
		return false;
	}
	
	if (!f.read((uint8_t*)currentTrackData, 10800))
	{
		f.close();
		statusRegisters[activeDrive][0] |= REG1_RW_FAULT;
		gpio_set_mask(1 << FAULT);
		return false;		
	}
	f.close();
	
	return true;
}

static bool saveTrack()
{
	File f = SD.open("hd0.img", FILE_WRITE);
	if (!f)
	{
		statusRegisters[activeDrive][0] |= REG1_RW_FAULT;
		statusRegisters[activeDrive][1] |= REG2_RW_UNSAFE;
		gpio_set_mask(1 << FAULT);
		return false;
	}
	
	if (!f.seek((cylinderAddressRegister[activeDrive] * 32400) + (headAddressRegister[activeDrive] * 10800)))
	{
		f.close();
		statusRegisters[activeDrive][0] |= REG1_RW_FAULT;
		statusRegisters[activeDrive][1] |= REG2_RW_UNSAFE;
		gpio_set_mask(1 << FAULT);
		return false;
	}
	
	if (!f.write((const uint8_t*)currentTrackData[activeDrive], 10800))
	{
		f.close();
		statusRegisters[activeDrive][0] |= REG1_RW_FAULT;
		statusRegisters[activeDrive][1] |= REG2_RW_UNSAFE;
		gpio_set_mask(1 << FAULT);
		return false;		
	}
	f.close();
	
	return true;
}

static byte ConfigureSDCard()
{
	int currentDrive = 0;
	bool rwFailure = false;
	
	for (int i = 0; i < 10800; ++i)
		currentTrackData[currentDrive][i] = 0;
	
	if (!SD.begin(SDPIN))
	{
#ifdef DEBUG
		Serial.println("Error intializing SD card.");
#endif
		statusRegisters[currentDrive][0] |= REG1_RW_FAULT;
		statusRegisters[currentDrive][1] |= REG2_RW_UNSAFE;
		gpio_set_mask(1 << FAULT);
	}
	else
	{
#ifdef DEBUG
		Serial.println("DEBUG: SD card intialized.");
#endif	
		if (!SD.exists("hd0.img"))
		{
#ifdef DEBUG
			Serial.println("DEBUG: hd0.img not found, creating hd0.img.");
#endif
			File f = SD.open("hd0.img", FILE_WRITE);
			if (!f)
			{
				rwFailure = true;
				statusRegisters[currentDrive][0] |= REG1_RW_FAULT;
				statusRegisters[currentDrive][1] |= REG2_RW_UNSAFE;
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
						statusRegisters[currentDrive][0] |= REG1_RW_FAULT;
						statusRegisters[currentDrive][1] |= REG2_RW_UNSAFE;
						gpio_set_mask(1 << FAULT);
						f.close();
						break;
					}
					
				}
				
				if (!rwFailure)
				{
#ifdef DEBUG
					Serial.println("DEBUG: Sized hd0.img to 12312000 bytes.");
#endif	
					// Place the sync bit in the first 20 sectors
					for (int i = 0; i < 20; ++i)
					{
						bool didSeek = f.seek((i * 540) + 25);
						if (!didSeek)
						{
							rwFailure = true;
							statusRegisters[currentDrive][0] |= REG1_RW_FAULT;
							statusRegisters[currentDrive][1] |= REG2_RW_UNSAFE;
							gpio_set_mask(1 << FAULT);
							break;
						}
						else
						{
							int wrote = f.write((uint8_t)1);
							if (wrote != 1)
							{
								rwFailure = true;
								statusRegisters[currentDrive][0] |= REG1_RW_FAULT;
								statusRegisters[currentDrive][1] |= REG2_RW_UNSAFE;
								gpio_set_mask(1 << FAULT);
								break;
							}
						}

					}
					
					if (!rwFailure)
					{
#ifdef DEBUG
						Serial.println("DEBUG: Added initial sync bits.");
#endif
					}
					f.close();
				}
			}
		}
		else
		{
#ifdef DEBUG
			Serial.println("DEBUG: hd0.img found.");
#endif
			File f = SD.open("hd0.img", FILE_WRITE);
			if (!f)
			{
				rwFailure = true;
				statusRegisters[currentDrive][0] |= REG1_RW_FAULT;
				statusRegisters[currentDrive][1] |= REG2_RW_UNSAFE;
				gpio_set_mask(1 << FAULT);
			}
			else
			{
				if (!f.seek(12312000 - 1)) 
				{
#ifdef DEBUG
					Serial.println("DEBUG: hd0.img is the wrong size.  Most likely it is corrupt.");
#endif
					rwFailure = true;
					statusRegisters[currentDrive][0] |= REG1_RW_FAULT;
					statusRegisters[currentDrive][1] |= REG2_RW_UNSAFE;
					gpio_set_mask(1 << FAULT);
				}

				f.close();
			}	

		}
	}
	
	return rwFailure ? 0 : 1;
}


HD::HD()
{
}

void HD::Setup()
{
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
	
	offset = pio_add_program(read_pio, &rw_clock_program);
	rw_clock_program_init(read_pio, 2, offset);

	// Setup PIO 1 state machines
	offset = pio_add_program(write_pio, &write_data_program);
	write_data_program_init(write_pio, 0, offset);
	
	offset = pio_add_program(write_pio, &write_trig_program);
	write_trig_program_init(write_pio, 1, offset);

	pio_set_irq0_source_enabled(write_pio, pis_interrupt0, true);
	irq_set_exclusive_handler(PIO1_IRQ_0, writeTrigRaiseISR);
	irq_set_enabled(PIO1_IRQ_0, true);
	
	pio_set_irq1_source_enabled(write_pio, pis_interrupt1, true);
	irq_set_exclusive_handler(PIO1_IRQ_1, writeTrigLowerISR);
	irq_set_enabled(PIO1_IRQ_1, true);
	
	statusRegisters[activeDrive][0] |= REG1_REZEROING & REG1_SEEKING;
	
#ifdef DEBUG
	Serial.println("DEBUG: Waiting for disk to \"spin up\".");
#endif
	
	setupHandshake = true;
	while (setupHandshake) ;
	
	lastSeekTime[activeDrive] = millis();
	
	while ((statusRegisters[activeDrive][0] & REG1_SEEKING) != 0) ;
	statusRegisters[activeDrive][0] |= REG1_READY;
	
#ifdef DEBUG
	Serial.println("DEBUG: Disk is finished \"spinning up\".");
#endif

}

byte HD::Setup1()
{
	while (!setupHandshake) ;
	
	// Setup Drive Signals
	gpio_init_mask(1 << SECTOR | 1 << SEEK_COMPLETE | 1 << INDEX | 1 << FAULT);
	gpio_set_dir_out_masked(1 << SECTOR | 1 << SEEK_COMPLETE | 1 << INDEX | 1 << FAULT);
	gpio_clr_mask(1 << SECTOR | 1 << SEEK_COMPLETE | 1 << INDEX | 1 << FAULT);
	
#ifdef DEBUG
	Serial.println("DEBUG: Start Index and Sector Pulse");
#endif
	
	// Start Index and Sector Pulse
	ITimer.attachInterruptInterval(833.5, SectorIndexPulseISR);
	
	// Setup SD card access
	existingDrives = 1; // ConfigureSDCard();
#ifdef DEBUG
	Serial.println("DEBUG: Drive Identification Bitmap = " + String(existingDrives, HEX));
#endif
	if (existingDrives)
	{	
		for (activeDrive = 0; activeDrive < 8; ++activeDrive)
		{
			if ((existingDrives & (1 << activeDrive)) != 0)
			{
#ifdef DEBUG
				Serial.println("DEBUG: Initializing Drive " + String(activeDrive) + ".") ;
#endif
				cylinderAddressRegister[activeDrive] = 0;
				headAddressRegister[activeDrive] = 0;

				if (loadTrack())
				{
					statusRegisters[activeDrive][0] |= REG1_ON_CYLINDER;
					statusRegisters[activeDrive][0] &= ~REG1_REZEROING & ~REG1_SEEKING;			
				}
			}
		}
		activeDrive = 0;
	}
	
	setupHandshake = false;
	
	return existingDrives;
}

static void setIncomingCylinderAddressHigh(byte data)
{
	incomingCylinderAddress[activeDrive] = (incomingCylinderAddress[activeDrive] & ~(0x03 << 8)) | ((data & 0x03) << 8);
}

static void setIncomingCylinderAddressLow(byte data)
{
	incomingCylinderAddress[activeDrive] = (incomingCylinderAddress[activeDrive] & ~0xFF) | data;
}

static void setIncomingHeadAddress(byte data)
{
	incomingHeadAddress[activeDrive] = ((data >> 2) & 0x03);
}

void HD::loop1()
{
	bool success;
	
	if (flushTrack[activeDrive] && (millis() - lastSeekTime[activeDrive] >= 2000) && (statusRegisters[activeDrive][1] & REG2_RW_UNSAFE) == 0)
	{
		if (saveTrack())
		{
			flushTrack[activeDrive] = false;
			lastSeekTime[activeDrive] = millis();
			Serial.println("DEBUG: Current track flushed to disk.");
		}
		else
		{
			Serial.println("ERROR: saveTrack Failed");
		}
	}
	

	if ((statusRegisters[activeDrive][0] & REG1_SEEKING) != 0)
	{
		if (headAddressRegister[activeDrive] != incomingHeadAddress[activeDrive] || cylinderAddressRegister[activeDrive] != incomingCylinderAddress[activeDrive])
		{
			headAddressRegister[activeDrive] = incomingHeadAddress[activeDrive];
			cylinderAddressRegister[activeDrive] = incomingCylinderAddress[activeDrive];
	
#ifdef DEBUG
			Serial.println("DEBUG: Seeking to cylinder " + String(cylinderAddressRegister[activeDrive], HEX) + " and head " + String(headAddressRegister[activeDrive], HEX));
#endif
			if (flushTrack[activeDrive] && (statusRegisters[activeDrive][1] & REG2_RW_UNSAFE) == 0)
			{
				if (saveTrack())
				{
					flushTrack[activeDrive] = false;
					Serial.println("DEBUG: Current track flushed to disk.");
				}
				else
				{
					Serial.println("ERROR: saveTrack Failed");
				}
			}

			if (loadTrack())
			{
				lastSeekTime[activeDrive] = millis();
				statusRegisters[activeDrive][0] |= REG1_ON_CYLINDER;
			}
		}
		else
		{
			statusRegisters[activeDrive][0] |= REG1_ON_CYLINDER;
		}
		
		statusRegisters[activeDrive][0] &= ~REG1_SEEKING & ~REG1_REZEROING;
		
		gpio_set_mask(1 << SEEK_COMPLETE);
		delayMicroseconds(4);
		gpio_clr_mask(1 << SEEK_COMPLETE);                                                                                                                                                                                           
	}
}

static byte getUnitID(byte data)
{
	return (data & 0xF0) >> 0x04;
}


void HD::sendCommand(byte command, byte data)
{
	switch (command)
	{

	case 0:
		if ((existingDrives & (1 << getUnitID(data))) != 0)
		{
			if (activeDrive != getUnitID(data) || isSelected == false)
			{	
#ifdef DEBUG
				if (isSelected)
				{
					Serial.println("DEBUG: Deactivating drive " + String(activeDrive) + ".");
				}
#endif
				if (flushTrack[activeDrive] == true)
				{
					// TODO: Need to handle flushing the current drives track, if necessary.
				}
				else
				{
					// TODO: Need to handle flushing the current drives track, if necessary.
				}
#ifdef DEBUG
				Serial.printf("DEBUG: Activating drive %d.\r\n", getUnitID(data));
#endif
			}
			
			activeDrive = getUnitID(data);
			activeDrive = getUnitID(data);
			setIncomingHeadAddress(data);
			setIncomingCylinderAddressHigh(data);
			isSelected = true; 
		}
		else
		{
#ifdef DEBUG
			if (isSelected)
			{
				Serial.println("DEBUG: Deactivating drive " + String(activeDrive) + ".");
			}
#endif
			isSelected = false;
		}
		break;
	case 1: 
		if (isSelected)
		{
			setIncomingCylinderAddressLow(data);

			if (incomingCylinderAddress[activeDrive] > 353)
			{
#ifdef DEBUG
				Serial.println("ERROR: Illegal Address Requested, Head=" + String(incomingHeadAddress[activeDrive]) + " Cylinder=" + String(incomingCylinderAddress[activeDrive]));
#endif
				statusRegisters[activeDrive][0] |= REG1_ILLEGAL_ADDRESS;
			}
			else
			{
				statusRegisters[activeDrive][0] &= ~REG1_ILLEGAL_ADDRESS & ~REG1_ON_CYLINDER;
				statusRegisters[activeDrive][0] |= REG1_SEEKING;
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
			if ((data & 0x02) != 0)  // REZERO
			{
#ifdef DEBUG
				Serial.println("DEBUG: Rezero-ing Drive");
#endif
				statusRegisters[activeDrive][0] &= ~REG1_ILLEGAL_ADDRESS & ~REG1_ON_CYLINDER & ~REG1_RW_FAULT;
				statusRegisters[activeDrive][1] &= ~REG2_RW_UNSAFE;
				SET_PIN_LOW(FAULT);
				incomingCylinderAddress[activeDrive] = 0;
				incomingHeadAddress[activeDrive] = 0;
				statusRegisters[activeDrive][0] |= REG1_SEEKING | REG1_REZEROING;
			}
			else if ((data & 0x01) != 0) // FAULT CLEAR
			{
#ifdef DEBUG
				Serial.println("DEBUG: Clearing Fault");
#endif
				statusRegisters[activeDrive][0] &= ~REG1_RW_FAULT;
				statusRegisters[activeDrive][1] &= ~REG2_RW_UNSAFE;
				SET_PIN_LOW(FAULT);
			}
		}
		break;
	}
}

bool HD::getStatus(byte command, byte& result)
{
	if (isSelected)
	{
		switch (command)
		{
		case 4:
#ifdef DEBUG
			Serial.println("DEBUG: Outputting Register 1: " + String(statusRegisters[activeDrive][0], BIN));
#endif
			result = statusRegisters[activeDrive][0];
			break;
		case 5:
#ifdef DEBUG
			Serial.println("DEBUG: Outputting Register 2: " + String(statusRegisters[activeDrive][1], BIN));
#endif
			result = statusRegisters[activeDrive][1];
			break;
		case 6:
			result = cylinderAddressRegister[activeDrive] & 0xFF;
#ifdef DEBUG
			Serial.println("DEBUG: Outputting Current CMD BYTE 1 Settings: " + String(result, BIN));
#endif
			break;
		case 7:
			result = ((ID & 0x0F) << 4) | ((headAddressRegister[activeDrive] & 0x03)) << 2 | ((cylinderAddressRegister[activeDrive] & 0x300) >> 8);
#ifdef DEBUG
			Serial.println("DEBUG: Outputting Current CMD BYTE 0 Settings: " + String(result, BIN));
#endif
			break;
		default:
			return false;
		}
		return true;
	}
	else
	{
		return false;
	}
}