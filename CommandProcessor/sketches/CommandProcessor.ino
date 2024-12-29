
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
#define READ LOW
#define WRITE HIGH

#define DRV_ACK		21
#define ENABLE_READ_TRIG	0
#define ENABLE_WRITE_TRIG	1

#define FAULT	22

// HD Device ID
#define ID 0

// Built-in LED
#define LED 25

// LEDs
#define BUSY_LED 26
#define READY_LED 27

bool isSelected = false;

byte readDataBus(int data)
{
	return ((data & 0x1FE0) >> 5);
}

byte extractUnitId(int data)
{
	return (readDataBus(data) >> 4);
}

byte readCommand(int data)
{
	return ((data & 0x1C) >> 2);
}

void setup()
{
	
#ifdef DEBUG
	Serial.begin(115200);
	while (!Serial) ;
#endif
	
#ifdef DEBUG
	Serial.print("Waiting for disk to \"spin up\"...");
#endif
	
	// Setup internal control signals
	gpio_init_mask(1 << DRV_ACK | 1 << ENABLE_READ_TRIG | 1 << ENABLE_WRITE_TRIG);
	
	gpio_set_dir_in_masked(1 << DRV_ACK);
	
	gpio_set_dir_out_masked(1 << ENABLE_READ_TRIG | 1 << ENABLE_WRITE_TRIG);
	gpio_clr_mask(1 << ENABLE_READ_TRIG | 1 << ENABLE_WRITE_TRIG);
	
	// Wait for disk to "spin up"
	while (!gpio_get(DRV_ACK)) ;
	
#ifdef DEBUG
	Serial.println("Done.");
#endif
	
	// Set Output Enable
	gpio_init_mask(1 << OUTPUT_ENABLE);
	gpio_set_dir_out_masked(1 << OUTPUT_ENABLE); 
	gpio_set_mask(1 << OUTPUT_ENABLE);  // TODO:  Change this to clr once command processor is working.

	// Setup Command Bus
	gpio_init_mask(1 << CMD_RW | 1 << CMD_SEL1 | 1 << CMD_SEL0 | 1 << BUS_DIRECTION | 0xFF << BUS_0 | 1 << CMD_STROBE | 1 << CMD_ACK);
	
	gpio_set_dir_in_masked(1 << CMD_RW | 1 << CMD_SEL1 | 1 << CMD_SEL0);

	gpio_set_dir_out_masked(1 << BUS_DIRECTION);
	gpio_clr_mask(1 << BUS_DIRECTION);
	
	gpio_set_dir_in_masked(0xFF << BUS_0);
	
	gpio_set_dir_in_masked(1 << CMD_STROBE);
	
	gpio_set_dir_out_masked(1 << CMD_ACK);
	gpio_clr_mask(1 << CMD_ACK);
	
	// Set Address
	gpio_init_mask(0x0F << ADDR0);
	gpio_set_dir_out_masked(0x0F << ADDR0);

	// Mark setup finished, by lighting LED
	gpio_init_mask(1 << LED | 1 << BUSY_LED | 1 << READY_LED );
	gpio_set_dir_out_masked(1 << LED | 1 << BUSY_LED | 1 << READY_LED);
	gpio_set_mask(1 << LED);
	gpio_clr_mask(1 << BUSY_LED);
	gpio_put(READY_LED, !gpio_get(FAULT));

	
#ifdef DEBUG
	Serial.printf("Drive is fully initialized.  Starting Command Processor...\r\n");
#endif
}

void loop()
{
	while (!digitalRead(CMD_STROBE)) ;
	
#ifdef DEBUG
	Serial.println("CMD_STROBE went HIGH");
#endif
	
	gpio_set_mask(1 << BUSY_LED);
	
	int data = gpio_get_all();
	
	byte command = readCommand(data);

#ifdef DEBUG
	Serial.print("Received command byte: ");
	Serial.print(command);
	Serial.print(" for ");
	Serial.println(extractUnitId(data));
#endif

	switch (command)
	{
		byte unitSelect;
		
	case 0: // CMD 0
		unitSelect = extractUnitId(data);
		isSelected = unitSelect == ID;
		if (isSelected)
		{
#ifdef DEBUG
			Serial.printf("Activating drive with address %d.\r\n", ID);
#endif
			gpio_put_masked(0x0F << ADDR0, (unitSelect & 0x0F) << ADDR0);	
#ifdef DEBUG
			Serial.println("Setting OUTPUT_ENABLE HIGH");
#endif
			gpio_set_mask(1 << OUTPUT_ENABLE);
		}
		else
		{
#ifdef DEBUG
			Serial.printf("Deactivating drive.\r\n", ID);
#endif
#ifdef DEBUG
			Serial.println("Setting OUTPUT_ENABLE LOW");
#endif
			gpio_set_mask(1 << OUTPUT_ENABLE);
		}
		break;
	case 4:  // CMD BYTE 1
		// Do Nothing
		break;
	case 2:  // CMD BYTE 2
		if (isSelected)
		{
			gpio_set_mask(1 << ENABLE_READ_TRIG | 1 << ENABLE_WRITE_TRIG);
		}
		break;
	case 6: // CMD BYTE 3
		// Do Nothing
		break;
	default:
#ifdef DEBUG
		Serial.println("Setting BUS_DIRECTION to WRITE");
#endif
		gpio_set_mask(1 << BUS_DIRECTION);
		break;
	}
	
	while (!digitalRead(DRV_ACK)) ;
#ifdef DEBUG
	Serial.println("DRV_ACK went HIGH");
#endif
	
#ifdef DEBUG
	Serial.println("Setting CMD_ACK HIGH");
#endif
	gpio_set_mask(1 << CMD_ACK);
	
	
	while(digitalRead(CMD_STROBE));
#ifdef DEBUG
	Serial.println("CMD_STROBE went LOW");
#endif
	
#ifdef DEBUG
	Serial.println("Setting BUS_DIRECTION to READ");
#endif
	gpio_clr_mask(1 << BUS_DIRECTION);
	
#ifdef DEBUG
	Serial.println("Setting CMD_ACK LOW");
#endif
	gpio_clr_mask(CMD_ACK);
	
	gpio_clr_mask(1 << ENABLE_READ_TRIG | 1 << ENABLE_WRITE_TRIG);
	
	gpio_clr_mask(1 << BUSY_LED);
}
