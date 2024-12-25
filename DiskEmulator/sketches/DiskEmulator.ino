#include <RPi_Pico_TimerInterrupt.h>
#include <RPi_Pico_ISR_Timer.h>
#include <RPi_Pico_ISR_Timer.hpp>

#define SECTOR 27
#define INDEX 22

// Sector/Index Pulse Variables
RPI_PICO_Timer ITimer(0);
volatile unsigned int currentSector;


bool SectorIndexPulseISR(__unused struct repeating_timer *t)
{
	currentSector++;
	currentSector %= 20;

	if (currentSector == 0)
	{
		gpio_set_mask((1 << SECTOR) | (1 << INDEX));
	}
	else
	{
		gpio_put(SECTOR, HIGH);
	}
	
	delayMicroseconds(3);
	
	gpio_clr_mask((1 << SECTOR) | (1 << INDEX));
	
	return true;
}

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
}

void setup1()
{
	pinMode(SECTOR, OUTPUT);
	pinMode(INDEX, OUTPUT);
	
	// Start Index and Sector Pulse
	ITimer.attachInterruptInterval(833500, SectorIndexPulseISR);
}

void loop()
{
	digitalWrite(LED_BUILTIN, HIGH);
	delay(1000);
	digitalWrite(LED_BUILTIN, LOW);
	delay(1000);
}

void loop1()
{
	
}