#pragma once
#include <cstdint>


class io
{
public: 
	io();
	~io();
	

	//Bus *bus;
	void writeIO(uint16_t addr, uint8_t value);
	uint8_t readIO(uint16_t addr);
private:
	char serialData[2];
};

