#pragma once

#include "common.h"

class HD
{
public:
	HD();
	void Setup();
	byte Setup1();
	
	void sendCommand(byte command, byte data);
	bool getStatus(byte command, byte& result);
	
	void loop1();
	
private:

	

	
	
};