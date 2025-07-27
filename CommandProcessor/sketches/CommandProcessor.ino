
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

#define CMD_STROBE	13

#define CMD_ACK		16

// HD Device ID Lines
#define ADDR0 17
#define ADDR1 18
#define ADDR2 19
#define ADDR3 20

// Internal Control Pins
#define OUTPUT_ENABLE 14

#define BUS_DIRECTION 15
#define BUS_ENABLE 28
#define READ LOW
#define WRITE HIGH

#define DRV_ACK		21
#define ENABLE_READ_TRIG	0
#define ENABLE_WRITE_TRIG	1

#define FAULT	22

// HD Device ID
#define ID 0

// LEDs
#define BUSY_LED 26
#define READY_LED 27

#define IS_PIN_HIGH(x) (gpio_get_all() & (1 << x)) != 0
#define IS_PIN_LOW(x) (gpio_get_all() & (1 << x)) == 0
#define SET_PIN_HIGH(x) gpio_set_mask(1 << x)
#define SET_PIN_LOW(x) gpio_clr_mask(1 << x)

bool isSelected = false;
byte activeDrive = 0;
byte existingDrives = 0;

byte readDataBus(int pins)
{
	return ~(pins >> BUS_0);
}

byte getCommand(int pins)
{
	return (pins >> CMD_SEL0) & 0x07;
}

byte getUnitID(int pins)
{
	return (readDataBus(pins) & 0xF0) >> 0x04;
}

void setDataBusToRead()
{
	SET_PIN_HIGH(BUS_DIRECTION);
}

void setDataBusToWrite()
{
	SET_PIN_LOW(BUS_DIRECTION);
}

void setup()
{
	
#ifdef DEBUG
	Serial.begin(115200);
	while (!Serial) ;
#endif
	
#ifdef DEBUG
	Serial.print("DEBUG: Waiting for disk to \"spin up\".");
#endif
	
	// Setup internal control signals
	gpio_init_mask(1 << DRV_ACK | 1 << ENABLE_READ_TRIG | 1 << ENABLE_WRITE_TRIG);
	
	gpio_set_dir_in_masked(1 << DRV_ACK);
	
	gpio_set_dir_out_masked(1 << ENABLE_READ_TRIG | 1 << ENABLE_WRITE_TRIG);
	gpio_clr_mask(1 << ENABLE_READ_TRIG | 1 << ENABLE_WRITE_TRIG);
	
	// Wait for disk to "spin up"
	while (!gpio_get(DRV_ACK)) ;
	
	existingDrives = readDataBus(gpio_get_all());
	
#ifdef DEBUG
	Serial.println("DEBUG: Drive \"spin up\" complete.");
#endif
	
	// Set Output Enable
	gpio_init_mask(1 << OUTPUT_ENABLE);
	gpio_set_dir_out_masked(1 << OUTPUT_ENABLE); 
	gpio_clr_mask(1 << OUTPUT_ENABLE); 

	// Setup Command Bus
	gpio_init_mask(1 << CMD_RW | 1 << CMD_SEL1 | 1 << CMD_SEL0 | 1 << BUS_DIRECTION | 0xFF << BUS_0 | 1 << CMD_STROBE | 1 << CMD_ACK);
	
	gpio_set_dir_in_masked(1 << CMD_RW | 1 << CMD_SEL1 | 1 << CMD_SEL0);

	gpio_set_dir_out_masked(1 << BUS_DIRECTION | 1 << BUS_ENABLE);
	gpio_set_mask(1 << BUS_DIRECTION | 1 << BUS_ENABLE);
	
	gpio_set_dir_in_masked(0xFF << BUS_0);
	
	gpio_set_dir_in_masked(1 << CMD_STROBE);
	
	gpio_set_dir_out_masked(1 << CMD_ACK);
	gpio_clr_mask(1 << CMD_ACK);
	
	// Set Address
	gpio_init_mask(0x0F << ADDR0);
	gpio_set_dir_out_masked(0x0F << ADDR0);

	// Mark setup finished, by lighting LED
	gpio_init_mask(1 << LED_BUILTIN | 1 << BUSY_LED | 1 << READY_LED);
	gpio_set_dir_out_masked(1 << LED_BUILTIN | 1 << BUSY_LED | 1 << READY_LED);
	gpio_set_mask(1 << LED_BUILTIN);
	gpio_clr_mask(1 << BUSY_LED);
	gpio_put(READY_LED, !gpio_get(FAULT));

	
#ifdef DEBUG
	Serial.println("DEBUG: Starting Command Processor.");
#endif
}

void loop()
{
	while (IS_PIN_LOW(CMD_STROBE)) ;
	
	SET_PIN_HIGH(BUSY_LED);
	
	int pins = gpio_get_all();
	
	byte command = getCommand(pins);

#ifdef DEBUG
	Serial.println("DEBUG: Received command " + String(command) + " with data: " + String(pins, HEX));
#endif

	switch (command)
	{
		byte selectedUnit;
		
	case 0:
		if ((existingDrives & (1 << getUnitID(pins))) != 0)
		{
			if (activeDrive != getUnitID(pins))
			{	
#ifdef DEBUG
				if (isSelected)
				{
					Serial.println("DEBUG: Deactivating drive " + String(activeDrive) + ".");
				}
				Serial.printf("DEBUG: Activating drive %d.\r\n", getUnitID(pins));
#endif
			}
			
			gpio_put_masked(0x0F << ADDR0, getUnitID(pins) << ADDR0);	

			SET_PIN_HIGH(OUTPUT_ENABLE);
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
			SET_PIN_LOW(OUTPUT_ENABLE);
		}
		break;
	case 1:
		// Do Nothing
		break;
	case 2:
		if (isSelected)
		{
			gpio_set_mask(1 << ENABLE_READ_TRIG | 1 << ENABLE_WRITE_TRIG);
			SET_PIN_HIGH(CMD_ACK);
		}
		break;
	case 3:
		// Do Nothing
		break;
	default:  // CMD Bytes 4-7
		setDataBusToWrite();
		break;
	}
	
	while (IS_PIN_LOW(DRV_ACK)) ;
	
#ifdef DEBUG
	if (command > 3)
	{
		Serial.print("Disk is reporting: ");
		pins = gpio_get_all();
		Serial.println(readDataBus(pins), HEX);
	}
#endif
	
	SET_PIN_HIGH(CMD_ACK);
	
	while (IS_PIN_HIGH(CMD_STROBE)) ;
	
	setDataBusToRead();
	
	SET_PIN_LOW(CMD_ACK);
	
	gpio_clr_mask(1 << ENABLE_READ_TRIG | 1 << ENABLE_WRITE_TRIG);
	
	SET_PIN_LOW(1 << BUSY_LED);
}
