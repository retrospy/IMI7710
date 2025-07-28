#include "common.h"
#include "HD.h"

HD hd;

volatile byte existingDrives = 0;

void setup()
{	
	// Setup internal control signals
	gpio_init_mask(1 << DRV_ACK | 1 << READ_TRIG | 1 << WRITE_TRIG);
	gpio_set_dir_in_masked(1 << READ_TRIG | 1 << WRITE_TRIG);
	
	gpio_set_dir_out_masked(1 << DRV_ACK);
	gpio_clr_mask(1 << DRV_ACK);
	
	// Setup Command Bus
	gpio_init_mask(1 << CMD_RW | 1 << CMD_SEL1 | 1 << CMD_SEL0 | 0xFF << BUS_0 | 1 << CMD_STROBE);
	gpio_set_dir_in_masked(1 << CMD_RW | 1 << CMD_SEL1 | 1 << CMD_SEL0 | 0xFF << BUS_0 | 1 << CMD_STROBE);

#ifdef DEBUG
	Serial.begin(115200);
	while (!Serial) ;
#endif
	
#ifdef DEBUG
	Serial.println("DEBUG: Waiting for disk to \"spin up\".");
#endif
	
	hd.Setup();
	//setDataBusToOutput();
	//writeDataBus(existingDrives);
	
#ifdef DEBUG
	Serial.printf("DEBUG: Disk is \"spun up\".  Starting Command Processor.\r\n");
#endif
	
	gpio_set_mask(1 << DRV_ACK);
	delay(500);
	gpio_clr_mask(1 << DRV_ACK);
	
	//setDataBusToInput();
	
	// Mark setup finished, by lighting LED
	gpio_init_mask(1 << LED_BUILTIN);
	gpio_set_dir_out_masked(1 << LED_BUILTIN);
	SET_PIN_HIGH(LED_BUILTIN);
}


void setup1()
{
	existingDrives = hd.Setup1();
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
	return (pins >> CMD_SEL0) & 0x07;
}

void loop()
{
	while (IS_PIN_LOW(CMD_STROBE)) ;
	
	int pins = gpio_get_all();
	
	byte command = getCommand(pins);
	
#ifdef DEBUG
	Serial.println("DEBUG: Received command " + String(command) + " with data: " + String(readDataBus(pins), HEX));
#endif
	
	if (command < 4)
	{
		hd.sendCommand(command, readDataBus(pins));
	}
	else
	{
		byte result;
		auto active = hd.getStatus(command, result);	
		if (active)
		{
			setDataBusToOutput();
			writeDataBus(result);
		}
	}
#ifdef DEBUG
	Serial.println("DEBUG: Raising DRIVE ACK.");
#endif
	
	SET_PIN_HIGH(DRV_ACK) ;	

#ifdef DEBUG
	Serial.println("DEBUG: Waiting for Strobe to lower.");
#endif
	
	while(IS_PIN_HIGH(CMD_STROBE)) ;
	
	setDataBusToInput() ;
	
#ifdef DEBUG
	Serial.println("DEBUG: Lowering DRIVE ACK.");
#endif
	
	SET_PIN_LOW(DRV_ACK) ;
}

void loop1()
{	
	hd.loop1();
}