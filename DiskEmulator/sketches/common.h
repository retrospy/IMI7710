#pragma once

#include <Arduino.h>

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

// Internal Control Pins
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

// SD Card Pins
#define SDPIN 17

// HD Device ID
#define ID 0

#define IS_PIN_HIGH(x) (gpio_get_all() & (1 << x)) != 0
#define IS_PIN_LOW(x) (gpio_get_all() & (1 << x)) == 0
#define SET_PIN_HIGH(x) gpio_set_mask(1 << x)
#define SET_PIN_LOW(x) gpio_clr_mask(1 << x)

#define  read_pio pio0
#define  write_pio pio1

#define REG1_READY				0x01
#define REG1_ON_CYLINDER		0x02
#define REG1_SEEKING			0x04
#define REG1_REZEROING			0x08
#define REG1_SERVO_ERROR		0x10
#define REG1_RW_FAULT			0x20
#define REG1_ILLEGAL_ADDRESS	0x40
#define REG1_SPEED_ERROR		0x80

#define REG2_RW_UNSAFE		0x01
#define REG2_POR			0x04
#define REG2_PLO_ERROR		0x08
#define REG2_WRITE_PROT		0x10
#define REG2_GUARD_BAND		0x80
