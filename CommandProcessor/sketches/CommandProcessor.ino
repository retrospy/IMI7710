
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

// Internal Control Bins
#define OUTPUT_ENABLE 14

#define BUS_DIRECTION 15
#define READ LOW
#define WRITE HIGH

#define READ_TRIG	0
#define DRV_ACK		1
#define WRITE_TRIG	21

// HD Device ID
#define ID 0

bool isSelected = false;

static inline byte readDataBus(int data)
{
	return ((data & 0x1FE0) >> 5);
}

static inline byte extractUnitId(int data)
{
	return (readDataBus(data) >> 4);
}

static inline byte readCommand(int data)
{
	return ((data & 0x1C) >> 2);
}

void setup()
{
	// Set Output Enable
	pinMode(OUTPUT_ENABLE, OUTPUT);
	digitalWrite(OUTPUT_ENABLE, LOW);

	// Setup Command Bus
	pinMode(CMD_RW, INPUT);
	pinMode(CMD_SEL1, INPUT);
	pinMode(CMD_SEL0, INPUT);

	pinMode(BUS_DIRECTION, OUTPUT);
	digitalWrite(BUS_DIRECTION, READ);
	
	for (int i = 0; i < 8; ++i)
	{
		pinMode(BUS_0 + i, INPUT);
	}
	
	pinMode(CMD_STROBE, INPUT);
	pinMode(CMD_ACK, OUTPUT);
	digitalWrite(CMD_ACK, LOW);
		
	// Set Address
	for (int i = 0; i <= (ADDR3 - ADDR0); ++i)
	{
		pinMode(ADDR0 + i, OUTPUT);
		digitalWrite(ADDR0 + i, (ID & (1 << i)) == 0 ? LOW : HIGH);	
	}
	
	// Setup internal control signals
	pinMode(DRV_ACK, INPUT);
	pinMode(READ_TRIG, OUTPUT);
	digitalWrite(READ_TRIG, LOW);
	pinMode(WRITE_TRIG, OUTPUT);
	digitalWrite(WRITE_TRIG, LOW);
	
	Serial.begin(115200);
}

void loop()
{
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
			Serial.println("Setting OUTPUT_ENABLE HIGH");
#endif
			digitalWrite(OUTPUT_ENABLE, HIGH);
		}
		else
		{
#ifdef DEBUG
			Serial.println("Setting OUTPUT_ENABLE LOW");
#endif
			digitalWrite(OUTPUT_ENABLE, LOW);
		}
		break;
	case 4:  // CMD BYTE 1
		// Do Nothing
		break;
	case 2:  // CMD BYTE 2
		if (isSelected)
		{
			// Read or Write
		}
		break;
	case 6: // CMD BYTE 3
		// Do Nothing
		break;
	default:
#ifdef DEBUG
		Serial.println("Setting BUS_DIRECTION to WRITE");
#endif
		digitalWrite(BUS_DIRECTION, WRITE);
		break;
	}
	
	while (!digitalRead(DRV_ACK)) ;
#ifdef DEBUG
	Serial.println("DRV_ACK went HIGH");
#endif
	
#ifdef DEBUG
	Serial.println("Setting CMD_ACK HIGH");
#endif
	digitalWrite(CMD_ACK, HIGH);
	
	
	while(digitalRead(CMD_STROBE));
#ifdef DEBUG
	Serial.println("CMD_STROBE went LOW");
#endif
	
#ifdef DEBUG
	Serial.println("Setting BUS_DIRECTION to READ");
#endif
	digitalWrite(BUS_DIRECTION, READ);
	
#ifdef DEBUG
	Serial.println("Setting CMD_ACK LOW");
#endif
	digitalWrite(CMD_ACK, LOW);
}
