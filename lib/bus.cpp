#include "bus.h"


Bus::Bus() {
    cpu = new z80();
   
    memory = new Ram();
    timer = new Timer(cpu);
    inout = new io(timer);
    cpu->ConnectBus(this);
    
}

Bus::~Bus() {

}

void Bus::busWrite(uint16_t addr, uint8_t data) {
    //std::cout << std::hex << "writting to " << addr << std::endl;
    if (addr < 0x8000) {
        //write to cartridge
        cart->cartWrite(addr, data);
    }
    else if (addr < 0xA000) {
        std::cout << std::hex << "unsupported write to " << addr << std::endl;
    }
    else if (addr < 0xC000) {
        cart->cartWrite(addr, data);
    }
    else if (addr < 0xE000) {
        //wram
        memory->write_wRam(addr, data);
    }
    else if (addr < 0xFE00) {
        //echo ram
        std::cout << std::hex << "unsupported write to " << addr << std::endl;
    }
    else if (addr < 0xFEA0) {
        //OAM memory
        std::cout << std::hex << "unsupported write to " << addr << std::endl;
    }
    else if (addr < 0xFF00) {
        std::cout << std::hex << "unsupported write to " << addr << std::endl;
    }
    else if (addr < 0xFF80) {
        //std::cout << std::hex << "writing to IO " << addr << std::endl;
        //std::cout << "writing value " << data << std::endl;
        inout->writeIO(addr, data);
    }
    
    else if (addr == 0xFFFF) {}
    else {
        memory->write_hRam(addr, data);
    }
}

void Bus::busWrite16(uint16_t addr, uint16_t data)
{
    busWrite(addr + 1, (data >> 8) & 0xFF); //write hibyte
    busWrite(addr, (data & 0xFF));
}

void Bus::insertCartridge(const std::shared_ptr<Cartridge>& cartridge)
{
    this->cart = cartridge;
    std::cout << "Cartridge Connected to bus..." << std::endl;
}

uint8_t Bus::busRead(uint16_t addr, bool bReadOnly) {
    //std::cout << std::hex << "reading from " << addr << std::endl;
    //rom bank
    if (addr < 0x8000) {
        //read cartridge
        return cart->cartRead(addr);
    }
    else if (addr < 0xA000) {
        //VRAM
        //std::cout << "Invalid read/write to 0x" << std::hex << addr;
        std::cout << "unsupported read" << std::endl;
        return 0x00;
    }
    else if (addr < 0xC000) {
        //std::cout << "Invalid read/write to 0x" << std::hex << addr;
        //Cartridge RAM
        return cart->cartRead(addr);
    }
    else if (addr < 0xE000) {
        //ram banks/ working ram

        return memory->read_wRam(addr);
    }
    else if (addr < 0xFE00) {
        //echo ram (not used)
        //std::cout << "Invalid read/write to 0x" << std::hex << addr;
        return 0;
    }
    else if (addr < 0xFEA0) {
        //Object attribute memory
        //std::cout << "Invalid read/write to 0x" << std::hex << addr;
        std::cout << "unsupported read" << std::endl;
        return 0x00;
    }
    else if (addr < 0xFF00) {
        //not usable
        //std::cout << "Invalid read/write to 0x" << std::hex << addr;
        return 0x00;
    }
    else if (addr < 0xFF80) {
        //IO registers
        //std::cout << "Invalid read/write to 0x" << std::hex << addr;
        std::cout << std::hex << "reading IO " << addr  << std::endl;
        return inout->readIO(addr);
    }
 
    else if (addr == 0xFFFF) {
        //CPU enable interrupt register
        //std::cout << "Invalid read/write to 0x" << std::hex << addr;
        return cpu->ie_register;
    }

    return memory->read_hRam(addr);
}

uint16_t Bus::busRead16(uint16_t addr)
{
    uint16_t lo = busRead(addr);
    uint16_t hi = busRead(addr+1);
    return (lo | (hi << 8));
}
