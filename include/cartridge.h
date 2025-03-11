#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <map>
class Cartridge
{
public:
	Cartridge(const std::string& file);
	~Cartridge();

	uint8_t cartRead(uint16_t addr);
	void cartWrite(uint16_t addr, uint8_t data);

	void loadRom(std::string location);

	//misc information about ROM header
	struct header {
		char romTitle[16]; //char of size 16
		uint8_t licCode;
		uint8_t cartCode;
		uint8_t romSize;
		uint8_t ramSize;
		uint8_t romVersion;
		bool checkSum = false;
	};
	

	bool gbcFlag = false;

	bool MBC1;
	bool MBC2;

	header sHeader;
	
	uint32_t romSize;
	uint8_t* romData;
	
private:
	std::map<uint8_t, std::string> licMap{
		{0x00, "None" }, {0x01, "Nintendo"}, {0x08, "Capcom"},
		{0x09, "HOT-B"}, {0x0A, "Jaleco"}, {0x0B, "Coconuts Japan"},
		{0x0C, "Elite Systems"}, {0x13, "EA"}, {0x18, "Hudson Soft"},
		{0x19, "ITC Entertainment"}, {0x1A, "Yanoman"}, {0x1D, "Japan Clary"},
		{0x1F, "Virgin Games"}, {0x24, "PCM Complete"}, {0x25, "San-X"},
		{0x28, "Kemco"}, {0x29, "SETA Corporation"}, {0x30, "Infogrames"},
		{0x34, "Konami"}, {0x34, "HectorSoft"}, {0x38, "Capcom"},
		{0x39, "Banpresto"}, {0x3C, "Entertaiment Interactive"}, {0x3E, "Gremlin"},
		{0x41, "Ubi Soft"}, {0x42, "Atlus"}, {0x44, "Malibu Interactive"},
		{0x46, "Angel"}, {0x47, "Spectrum HoloByte"}, {0x49, "Irem"},
		{0x4A, "Virgin Games"}, {0x4D, "Malibu Interactive"}, {0x4F, "U.S. Gold"},

	};

	std::map<uint8_t, std::string> cartMap{
		{0x00, "ROM ONLY"}, {0x01, "MBC1"}, {0x02, "MBC1+RAM"},
		{0x03, "MBC1+RAM+BATTERY"}, {0x05, "MBC2"}, {0x06, "MBC2+BATTERY"},
		{0x08, "ROM+RAM"}, {0x09, "ROM+RAM+BATTERY"}, {0x0B, "MMM01"},
		{0x0C, "MMM01_RAM"}, {0x0D, "MMM01+RAM+BATTERY"}, {0x0F, "MBC3+TIMER+BATTERY"},
		{0x10, "MBC3+TIMER+RAM+BATTERY"}, {0x11, "MBC3"}, {0x12, "MBC3+RAM"},
		{0x13, "MBC3+RAM+BATTERY"}, {0x19, "MBC5"}, {0x1A, "MBC5+RAM"},
		{0x1B, "MBC5+RAM+BATTERY"}
	};
	
};

