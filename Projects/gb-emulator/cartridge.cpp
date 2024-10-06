#include "cartridge.h"

#include <stdio.h>

Cartridge::Cartridge(const std::string& ROM)
{
	loadRom(ROM);


}

Cartridge::~Cartridge()
{
}

void Cartridge::loadRom(std::string location)
{
	

	std::ifstream ifs;
	ifs.open(location, std::ios::binary);
	
	ifs.seekg(0, std::ios::end);
	romSize = ifs.tellg();

	romData = new uint8_t[romSize];
	ifs.seekg(std::ios::beg);
	ifs.read((char*)romData, romSize);

	//copy bits from 0x0134-143 into title
	std::copy(romData + 0x0134, romData + 0x0143, sHeader.romTitle);
	//std::cout << +romData[0x0145];
	
	//if value > 0 then rom is a gbc rom
	if (romData[0x0143] > 0)
		gbcFlag = true;
	//set the license code
	sHeader.licCode = romData[0x014B];
	

	//set cartridge type code
	sHeader.cartCode = romData[0x0147];

	//check ROM size (number of ROM banks)
	sHeader.romSize = romData[0x0148];

	//check RAM size 
	sHeader.ramSize = romData[0x0149];

	//check ROM version
	sHeader.romVersion = romData[0x014C];

	//check sum for valid rom
	uint8_t x = 0;
	for (uint16_t i = 0x0134; i <= 0x014C; i++) {
		x = x - romData[i] - 1;
	}

	if (x == romData[0x014D])
		sHeader.checkSum = true;
	else
		sHeader.checkSum = false;
	std::cout << "Title: " << sHeader.romTitle << std::endl;
	std::cout << "Type: " << cartMap[sHeader.cartCode] << std::endl;
	std::cout << "ROM Size: " << (32 << sHeader.romSize) << std::endl;
	std::cout << "RAM Size: " << +sHeader.ramSize << std::endl;
	std::cout << "Liscence: " << licMap[sHeader.licCode] << std::endl;
	std::cout << "ROM Version: " << +sHeader.romVersion << std::endl;
	std::cout << "Checksum: " << sHeader.checkSum << std::endl;
	
}
