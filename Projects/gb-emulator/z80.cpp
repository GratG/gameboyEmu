#include "z80.h"
#include "bus.h"
#include <iostream>


z80::z80() {

}

z80::~z80() {

}

void z80::ConnectBus(Bus* n) {
    bus = n;
}

void z80::clock()
{
    if (cycles == 0) {
        opcode = read(pc);
        pc++;
        cycles = executeOP(opcode);
    }
    //std::cout << +cycles << std::endl;
    clock_cycles--;
    cycles--;
}

uint8_t z80::getFlag(FLAGSz80 f)
{
    //extract f from lo reg, if f > 0 return 1 else return 0
    return ((registerAF.lo & f) > 0) ? 1 : 0;
}

bool z80::checkBit(uint8_t reg, int pos)
{
    uint8_t mask = 1 << pos;
    return(reg & mask) ? true : false;
}

bool z80::checkBit(uint16_t reg, int pos)
{
    uint16_t mask = 1 << pos;
    return(reg & mask) ? true : false;
}

void z80::setFlag(FLAGSz80 f, bool v) {
    if (v)
        registerAF.lo |= f; //set the f flag (1)
    else
        registerAF.lo &= ~f; //clear flag (0)
}


uint8_t z80::read(uint16_t a) {
    return bus->busRead(a, false);

}

uint16_t z80::read16()
{
    uint16_t v = read(pc + 1);
    v = v << 8;
    v += read(pc);
    return v;
}

void z80::write(uint16_t a, uint8_t d) {
    bus->busWrite(a, d);
}

uint8_t z80::resetBit(uint8_t b, uint8_t x)
{
    b &= x;
    return b;
}

uint8_t z80::setBit(uint8_t b, uint8_t x)
{
    b |= x;
    return b;
}

void z80::BIT8_LOAD(uint8_t& reg) {
    //load immediate value of reg
    uint8_t n = read(reg);
    pc++;
    reg = n;
}
void z80::BIT8_LOAD(uint16_t& reg) {
    uint8_t n = read(reg);
    pc++;
    reg = n;
}
void z80::BIT8_LOAD(uint8_t& reg1, uint8_t& reg2) {
    reg1 = reg2;
    pc++;
}

void z80::BIT8_LOAD(uint8_t& reg1, uint16_t& reg2) {
    reg1 &= reg2;
    pc++;
}

void z80::BIT8_LOAD(uint16_t& reg1, uint8_t& reg2) {
    reg1 = reg2;
    pc++;
}

void z80::BIT8_LOAD(uint16_t& reg1, uint16_t& reg2) {
    reg1 = reg2;
    pc++;
}
void z80::BIT16_DEC(uint16_t& reg)
{
    reg--;
}
void z80::BIT16_INC(uint16_t& reg)
{
    reg++;
}
void z80::PUSH16(uint16_t b)
{
    //extract 8bit pairs from 16bit reg
    uint8_t hi = b >> 8;
    uint8_t lo = b & 0xFF;
    write(sp, hi);
    write(sp, lo);
    sp--;
    sp--;
}
void z80::POP16(uint16_t reg)
{
    nn = read(sp);
    write(reg, nn);
    sp--;
    sp--;
}
void z80::ADD_8BIT(uint8_t& reg1, uint8_t reg2, bool c)
{
    //keep the original value
    uint8_t temp = reg1;
    uint8_t addVal = 0;

    addVal = reg2;
    if (c) {
        if (getFlag(C) > 0)
            addVal++;
    }

    reg1 += addVal;

    //reset all flags to 0
    registerAF.lo = 0;

    //if sum == 0, set Z flag
    if (reg1 == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

    //half carry test
    uint16_t htest = (temp & 0xF); //set original 0xF bit of reg1
    htest += (addVal & 0xF); //add original 0xF bit of reg2 to reg1

    //test if carry occured
    if (htest > 0xF)
        registerAF.lo = setBit(registerAF.lo, H);
    if ((temp + addVal) > 0xFF)
        registerAF.lo = setBit(registerAF.lo, C);
}
void z80::ADD_16BIT(uint16_t& reg1, uint16_t reg2)
{
    //keep the original value
    uint16_t temp = reg1;
    uint16_t addVal = reg2;

    reg1 += addVal;

    registerAF.lo = resetBit(registerAF.lo, N);


    //half carry test
    if (((temp & 0xFF00) & 0xF) + ((addVal >> 8) & 0xF))
        registerAF.lo = setBit(registerAF.lo, H);
    else
        registerAF.lo = resetBit(registerAF.lo, H);

    //carry test
    if ((temp + addVal) > 0xFFFF)
        registerAF.lo = setBit(registerAF.lo, C);
    else
        registerAF.lo = resetBit(registerAF.lo, C);




}
void z80::SUB_8BIT(uint8_t& reg1, uint8_t reg2, bool c)
{
    uint8_t temp = reg1;

    uint8_t subVal = reg2;

    if (c) {
        if (getFlag(C) > 0)
            subVal++;
    }
    reg1 -= reg2;

    //reset flags
    registerAF.lo = 0;
    //set 0 flag
    if (reg1 == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    //set substitution flag
    registerAF.lo = setBit(registerAF.lo, N);
    //no borrow
    if (temp < subVal)
        registerAF.lo = setBit(registerAF.lo, C);

    int8_t htest = (temp & 0xF);
    htest -= (subVal & 0xF);

    if (htest < 0)
        registerAF.lo = setBit(registerAF.lo, H);


}
void z80::AND_8BIT(uint8_t& reg1, uint8_t reg2)
{
    uint8_t temp = reg2;

    reg1 &= temp;

    //reset flags
    registerAF.lo = 0;

    if (reg1 == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    //set H flag
    registerAF.lo = setBit(registerAF.lo, H);


}
void z80::OR_8BIT(uint8_t& reg1, uint8_t reg2)
{
    uint8_t temp = reg2;

    reg1 |= temp;

    //reset flags
    registerAF.lo = 0;

    if (reg1 == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

}
void z80::XOR_8BIT(uint8_t& reg1, uint8_t reg2)
{
    uint8_t temp = reg2;

    reg1 ^= temp;

    //reset flags
    registerAF.lo = 0;

    if (reg1 == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}
void z80::CP_8BIT(uint8_t reg1, uint8_t reg2)
{

    uint8_t temp = reg1;

    uint8_t subVal = reg2;

    reg1 -= reg2;

    //reset flags
    registerAF.lo = 0;
    //set 0 flag
    if (reg1 == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    //set substitution flag
    registerAF.lo = setBit(registerAF.lo, N);
    //no borrow
    if (temp < subVal)
        registerAF.lo = setBit(registerAF.lo, C);

    int8_t htest = (temp & 0xF);
    htest -= (subVal & 0xF);

    if (htest < 0)
        registerAF.lo = setBit(registerAF.lo, H);

}
void z80::INC_8BIT(uint8_t& reg)
{
    uint8_t temp = reg;
    reg++;

    //no flag reset
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    else
        registerAF.lo = resetBit(registerAF.lo, Z);

    registerAF.lo = resetBit(registerAF.lo, N);

    if ((temp & 0xF) == 0xF)
        registerAF.lo = setBit(registerAF.lo, H);
    else
        registerAF.lo = resetBit(registerAF.lo, H);

}
void z80::INC_16BIT(uint16_t& reg)
{
    uint16_t temp = reg;
    reg++;

    //no flag reset
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    else
        registerAF.lo = resetBit(registerAF.lo, Z);

    registerAF.lo = resetBit(registerAF.lo, N);

    if ((temp & 0xF) == 0xF)
        registerAF.lo = setBit(registerAF.lo, H);
    else
        registerAF.lo = resetBit(registerAF.lo, H);

}
void z80::DEC_8BIT(uint8_t& reg)
{
    uint8_t temp = reg;
    reg--;

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    else
        registerAF.lo = resetBit(registerAF.lo, Z);

    //set substitution flag
    registerAF.lo = resetBit(registerAF.lo, N);

    if ((temp & 0xF) == 0xF)
        registerAF.lo = setBit(registerAF.lo, H);
    else
        registerAF.lo = resetBit(registerAF.lo, H);
}

void z80::DEC_16BIT(uint16_t& reg)
{
    uint16_t temp = reg;
    reg--;

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    else
        registerAF.lo = resetBit(registerAF.lo, Z);

    //set substitution flag
    registerAF.lo = resetBit(registerAF.lo, N);

    if ((temp & 0xF) == 0xF)
        registerAF.lo = setBit(registerAF.lo, H);
    else
        registerAF.lo = resetBit(registerAF.lo, H);
}

void z80::SWAP_NIBBLES(uint8_t& reg)
{
    uint8_t temp = reg;
    uint8_t reglo = reg & 0x0F;
    uint8_t reghi = reg & 0xF0;
    reg = (reglo << 4) | (reghi >> 4);

    registerAF.lo = 0;
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

}

void z80::SWAP_NIB_16(uint16_t& reg)
{
    uint16_t temp = reg;
    uint16_t temphi = temp & 0xFF00;
    uint16_t templo = temp & 0x00FF;
    uint8_t reglo = templo & 0x0F;
    uint8_t reghi = templo & 0xF0;
    reg = temphi | ((reglo << 4) | (reghi >> 4));


    registerAF.lo = 0;
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

}



void z80::DAA()
{//obtained from another emulator's code

    if (getFlag(N))
    {
        if ((registerAF.hi & 0x0F) > 0x09 || registerAF.lo & 0x20)
        {
            registerAF.hi -= 0x06; //Half borrow: (0-1) = (0xF-0x6) = 9
            if ((registerAF.hi & 0xF0) == 0xF0) registerAF.lo |= 0x10; else registerAF.lo &= ~0x10;
        }

        if ((registerAF.hi & 0xF0) > 0x90 || registerAF.lo & 0x10) registerAF.hi -= 0x60;
    }
    else
    {
        if ((registerAF.hi & 0x0F) > 9 || registerAF.lo & 0x20)
        {
            registerAF.hi += 0x06; //Half carry: (9+1) = (0xA+0x6) = 10
            if ((registerAF.hi & 0xF0) == 0) registerAF.lo |= 0x10; else registerAF.lo &= ~0x10;
        }

        if ((registerAF.hi & 0xF0) > 0x90 || registerAF.lo & 0x10) registerAF.hi += 0x60;
    }

    if (registerAF.hi == 0) registerAF.lo |= 0x80; else registerAF.lo &= ~0x80;
}

void z80::CPL()
{
    registerAF.hi = ~registerAF.hi;
    registerAF.lo = setBit(registerAF.hi, N);
    registerAF.lo = setBit(registerAF.lo, H);
}

void z80::CCF()
{
    if (getFlag(C))
        registerAF.lo = setBit(registerAF.lo, C);
    else
        registerAF.lo = setBit(registerAF.lo, C);

    registerAF.lo = resetBit(registerAF.lo, N);
    registerAF.lo = resetBit(registerAF.lo, H);

}

void z80::SCF()
{
    registerAF.lo = setBit(registerAF.lo, C);
    registerAF.lo = resetBit(registerAF.lo, H);
    registerAF.lo = resetBit(registerAF.lo, N);
}

void z80::RLC(uint8_t& reg)
{

    bool isSet = checkBit(reg, 7);

    registerAF.lo = 0;
    reg <<= 1;

    if (isSet) {
        registerAF.lo = setBit(registerAF.lo, C);
        reg = setBit(reg, 0);
    }

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::RLCMem(uint16_t addr)
{
    uint8_t reg = read(addr);

    bool isSet = checkBit(reg, 7);

    registerAF.lo = 0;
    reg <<= 1;

    if (isSet) {
        registerAF.lo = setBit(registerAF.lo, C);
        reg = setBit(reg, 0);
    }

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    write(addr, reg);
}


void z80::RL(uint8_t& reg)
{
    bool isCarrySet = getFlag(C);
    bool isSet = checkBit(reg, 7);

    registerAF.lo = 0;
    reg <<= 1;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);

    if (isCarrySet)
        reg = setBit(reg, 0);

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::RLMem(uint16_t addr)
{
    uint8_t reg = read(addr);
    bool isCarrySet = getFlag(C);
    bool isSet = checkBit(reg, 7);

    registerAF.lo = 0;
    reg <<= 1;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);

    if (isCarrySet)
        reg = setBit(reg, 0);

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

    write(addr, reg);
}

void z80::RLCMem(uint16_t addr) {
    uint8_t reg = read(addr);

    bool isCarrySet = getFlag(C);
    bool isSet = checkBit(reg, 7);

    registerAF.lo = 0;
    reg <<= 1;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);

    if (isCarrySet)
        reg = setBit(reg, 0);

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

    write(addr, reg);

}

void z80::RRC(uint8_t& reg)
{
    bool isSet = checkBit(reg, 0);

    registerAF.lo = 0;
    reg >>= 1;

    if (isSet) {
        registerAF.lo = setBit(registerAF.lo, C);
        reg = setBit(reg, 7);
    }

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::RRCMem(uint16_t addr)
{
    uint8_t reg = read(addr);
    bool isSet = checkBit(reg, 0);

    registerAF.lo = 0;
    reg >>= 1;

    if (isSet) {
        registerAF.lo = setBit(registerAF.lo, C);
        reg = setBit(reg, 7);
    }

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    
    write(addr, reg);
}

void z80::RR(uint8_t& reg)
{

    bool isCarrySet = checkBit(registerAF.lo, C);
    bool isSet = checkBit(reg, 0);

    registerAF.lo = 0;
    reg >>= 1;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);

    if (isCarrySet)
        reg = setBit(reg, 7);

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::RRMem(uint16_t addr)
{
    uint8_t reg = read(addr);
    bool isCarrySet = checkBit(registerAF.lo, C);
    bool isSet = checkBit(reg, 0);

    registerAF.lo = 0;
    reg >>= 1;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);

    if (isCarrySet)
        reg = setBit(reg, 7);

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    write(addr, reg);
}

void z80::SLA(uint8_t& reg)
{
    bool isSet = checkBit(reg, 7);

    reg <<= 1;

    registerAF.lo = 0;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::SLAMem(uint16_t addr)
{
    uint8_t reg = read(addr);
    bool isSet = checkBit(reg, 7);

    reg <<= 1;

    registerAF.lo = 0;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    
    write(addr, reg);
}

void z80::SRA(uint8_t& reg)
{
    bool isSet = checkBit(reg, 0);
    bool isHSet = checkBit(reg, 7);

    reg >>= 1;

    registerAF.lo = 0;

    if (isHSet)
        reg = setBit(reg, 7);
    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::SRAMem(uint16_t addr)
{
    uint8_t reg = read(addr);
    bool isSet = checkBit(reg, 0);
    bool isHSet = checkBit(reg, 7);

    reg >>= 1;

    registerAF.lo = 0;

    if (isHSet)
        reg = setBit(reg, 7);
    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

    write(addr, reg);
}

void z80::SRL(uint8_t& reg)
{

    bool isLSet = checkBit(reg, 0);

    registerAF.lo = 0;

    reg >>= 1;

    if (isLSet)
        registerAF.lo = setBit(registerAF.lo, C);
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::SRLMem(uint16_t addr)
{
    uint8_t reg = read(addr);

    bool isLSet = checkBit(reg, 0);

    registerAF.lo = 0;

    reg >>= 1;

    if (isLSet)
        registerAF.lo = setBit(registerAF.lo, C);
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

    write(addr, reg);
}

void z80::testBit(uint8_t reg, int bit)
{

    bool isSet = checkBit(reg, bit);

    if (isSet)
        registerAF.lo = resetBit(registerAF.lo, Z);
    else
        registerAF.lo = setBit(registerAF.lo, Z);
    
    registerAF.lo = setBit(registerAF.lo, H);
    registerAF.hi = resetBit(registerAF.lo, N);
}

void z80::testBitMem(uint16_t addr, int bit)
{
    uint8_t reg = read(addr);
    bool isSet = checkBit(reg, bit);

    if (isSet)
        registerAF.lo = resetBit(registerAF.lo, Z);
    else
        registerAF.lo = setBit(registerAF.lo, Z);

    registerAF.lo = setBit(registerAF.lo, H);
    registerAF.hi = resetBit(registerAF.lo, N);

  
}

void z80::SET(uint8_t& reg, int bit)
{

    reg = setBit(reg, bit);

}

void z80::SETMem(uint16_t addr, int bit)
{
    uint8_t reg = read(addr);
    reg = setBit(reg, bit);
    write(addr, reg);
}

void z80::RES(uint8_t& reg, int bit)
{

    reg = resetBit(reg, bit);

}

void z80::RESMem(uint16_t addr, int bit)
{
    uint8_t reg = read(addr);
    reg = resetBit(reg, bit);
    write(addr, reg);
}

// opcodes from http://www.codeslinger.co.uk/pages/projects/gameboy/files/GB.pdf
// switch statement for all opcodes...
// returns the cycle number which decrements in clock()
// instructions with parameters will be above this
// each opcode uses instructions in different static
int z80::executeOP(uint8_t opcode)
{
    std::cout << std::hex << +opcode << std::endl;
    switch (opcode) {
        //all 8bit load opcodes:
    case 0x06:
        BIT8_LOAD(registerBC.hi);
        return 8;
    case 0x0E:
        BIT8_LOAD(registerBC.lo);
        return 8;
    case 0x16:
        BIT8_LOAD(registerDE.hi);
        return 8;
    case 0x1E:
        BIT8_LOAD(registerDE.lo);
        return 8;
    case 0x26:
        BIT8_LOAD(registerHL.hi);
        return 8;
    case 0x2E:
        BIT8_LOAD(registerHL.lo);
        return 8;
        //load r2 into r1
    case 0x7F:
        BIT8_LOAD(registerAF.hi, registerAF.hi);
        return 4;
    case 0x78:
        BIT8_LOAD(registerAF.hi, registerBC.hi);
        return 4;
    case 0x79:
        BIT8_LOAD(registerAF.hi, registerBC.lo);
        return 4;
    case 0x7A:
        BIT8_LOAD(registerAF.hi, registerDE.hi);
        return 4;
    case 0x7B:
        BIT8_LOAD(registerAF.hi, registerDE.lo);
        return 4;
    case 0x7C:
        BIT8_LOAD(registerAF.hi, registerHL.hi);
        return 4;
    case 0x7D:
        BIT8_LOAD(registerAF.hi, registerHL.lo);
        return 4;
    case 0x7E:
        BIT8_LOAD(registerAF.hi, registerHL.reg);
        return 8;
    case 0x40:
        BIT8_LOAD(registerBC.hi, registerBC.hi);
        return 4;
    case 0x41:
        BIT8_LOAD(registerBC.hi, registerBC.lo);
        return 4;
    case 0x42:
        BIT8_LOAD(registerBC.hi, registerDE.hi);
        return 4;
    case 0x43:
        BIT8_LOAD(registerBC.hi, registerDE.lo);
        return 4;
    case 0x44:
        BIT8_LOAD(registerBC.hi, registerHL.hi);
        return 4;
    case 0x45:
        BIT8_LOAD(registerBC.hi, registerHL.lo);
        return 4;
    case 0x46:
        BIT8_LOAD(registerBC.hi, registerHL.reg);
        return 8;
    case 0x48:
        BIT8_LOAD(registerBC.lo, registerBC.hi);
        return 4;
    case 0x49:
        BIT8_LOAD(registerBC.lo, registerBC.lo);
        return 4;
    case 0x4A:
        BIT8_LOAD(registerBC.lo, registerDE.hi);
        return 4;
    case 0x4B:
        BIT8_LOAD(registerBC.lo, registerDE.lo);
        return 4;
    case 0x4C:
        BIT8_LOAD(registerBC.lo, registerHL.hi);
        return 4;
    case 0x4D:
        BIT8_LOAD(registerBC.lo, registerHL.lo);
        return 4;
    case 0x4E:
        BIT8_LOAD(registerBC.lo, registerHL.reg);
        return 8;
    case 0x50:
        BIT8_LOAD(registerDE.hi, registerBC.hi);
        return 4;
    case 0x51:
        BIT8_LOAD(registerDE.hi, registerBC.lo);
        return 4;
    case 0x52:
        BIT8_LOAD(registerDE.hi, registerDE.hi);
        return 4;
    case 0x53:
        BIT8_LOAD(registerDE.hi, registerDE.lo);
        return 4;
    case 0x54:
        BIT8_LOAD(registerDE.hi, registerHL.hi);
        return 4;
    case 0x55:
        BIT8_LOAD(registerDE.hi, registerHL.lo);
        return 4;
    case 0x56:
        BIT8_LOAD(registerDE.hi, registerHL.reg);
        return 8;
    case 0x58:
        BIT8_LOAD(registerDE.lo, registerBC.hi);
        return 4;
    case 0x59:
        BIT8_LOAD(registerDE.lo, registerBC.lo);
        return 4;
    case 0x5A:
        BIT8_LOAD(registerDE.lo, registerDE.hi);
        return 4;
    case 0x5B:
        BIT8_LOAD(registerDE.lo, registerDE.lo);
        return 4;
    case 0x5C:
        BIT8_LOAD(registerDE.lo, registerHL.hi);
        return 4;
    case 0x5D:
        BIT8_LOAD(registerDE.lo, registerHL.lo);
        return 4;
    case 0x5E:
        BIT8_LOAD(registerDE.lo, registerHL.reg);
        return 8;
    case 0x60:
        BIT8_LOAD(registerHL.hi, registerBC.hi);
        return 4;
    case 0x61:
        BIT8_LOAD(registerHL.hi, registerBC.lo);
        return 4;
    case 0x62:
        BIT8_LOAD(registerHL.hi, registerDE.hi);
        return 4;
    case 0x63:
        BIT8_LOAD(registerHL.hi, registerDE.lo);
        return 4;
    case 0x64:
        BIT8_LOAD(registerHL.hi, registerHL.hi);
        return 4;
    case 0x65:
        BIT8_LOAD(registerHL.hi, registerHL.lo);
        return 4;
    case 0x66:
        BIT8_LOAD(registerHL.hi, registerHL.reg);
        return 8;
    case 0x68:
        BIT8_LOAD(registerHL.lo, registerBC.hi);
        return 4;
    case 0x69:
        BIT8_LOAD(registerHL.lo, registerBC.lo);
        return 4;
    case 0x6A:
        BIT8_LOAD(registerHL.lo, registerDE.hi);
        return 4;
    case 0x6B:
        BIT8_LOAD(registerHL.lo, registerDE.lo);
        return 4;
    case 0x6C:
        BIT8_LOAD(registerHL.lo, registerHL.hi);
        return 4;
    case 0x6D:
        BIT8_LOAD(registerHL.lo, registerHL.lo);
        return 4;
    case 0x6E:
        BIT8_LOAD(registerHL.lo, registerHL.reg);
        return 8;
    case 0x70:
        BIT8_LOAD(registerHL.reg, registerBC.hi);
        return 8;
    case 0x71:
        BIT8_LOAD(registerHL.reg, registerBC.lo);
        return 8;
    case 0x72:
        BIT8_LOAD(registerHL.reg, registerDE.hi);
        return 8;
    case 0x73:
        BIT8_LOAD(registerHL.reg, registerDE.lo);
        return 8;
    case 0x74:
        BIT8_LOAD(registerHL.reg, registerHL.hi);
        return 8;
    case 0x75:
        BIT8_LOAD(registerHL.reg, registerHL.lo);
        return 8;
    case 0x36:
        BIT8_LOAD(registerHL.reg);
        return 12;
    case 0x0A:
        BIT8_LOAD(registerAF.hi, registerBC.reg);
        return 8;
    case 0x1A:
        BIT8_LOAD(registerAF.hi, registerDE.reg);
        return 8;
    case 0xFA:
        nn = read16();
        n = read(nn);
        registerAF.hi = n;
        pc++; pc++;
        return 16;
    case 0x03E:
        BIT8_LOAD(registerAF.hi);
        return 8;
    case 0x47:
        BIT8_LOAD(registerBC.hi, registerAF.hi);
        return 4;
    case 0x4F:
        BIT8_LOAD(registerBC.lo, registerAF.hi);
        return 4;
    case 0x57:
        BIT8_LOAD(registerDE.hi, registerAF.hi);
        return 4;
    case 0x5F:
        BIT8_LOAD(registerDE.lo, registerAF.hi);
        return 4;
    case 0x67:
        BIT8_LOAD(registerHL.hi, registerAF.hi);
        return 4;
    case 0x6F:
        BIT8_LOAD(registerBC.hi, registerAF.hi);
        return 4;
    case 0x02:
        BIT8_LOAD(registerBC.reg, registerAF.hi);
        return 8;
    case 0x12:
        BIT8_LOAD(registerDE.reg, registerAF.hi);
        return 8;
    case 0x77:
        BIT8_LOAD(registerHL.reg, registerAF.hi);
        return 8;
    case 0xEA:
        nn = read16();
        n = read(nn);
        write(n, registerAF.hi);
        return 16;
    case 0xF2:
        n = 0xFF00;
        n |= registerBC.lo;
        BIT8_LOAD(registerAF.hi, n);
        return 8;
    case 0xE2:
        n = 0xFF00;
        n |= registerBC.lo;
        BIT8_LOAD(n, registerAF.hi);
        return 8;
        //load A into memory, dec/inc HL
    case 0x3A:
        BIT8_LOAD(registerAF.hi, registerHL.reg);
        BIT16_DEC(registerHL.reg);
        return 8;
    case 0x2A:
        BIT8_LOAD(registerAF.hi, registerHL.reg);
        BIT16_INC(registerHL.reg);
        return 8;
    case 0x22:
        BIT8_LOAD(registerHL.reg, registerAF.hi);
        BIT16_INC(registerHL.reg);
        return 8;
        //load n + 0xFF00 to A
    case 0xE0:
        n = read(pc);
        n |= 0xFF00;
        BIT8_LOAD(n, registerAF.hi);
        return 12;
    case 0xF0:
        n = read(pc);
        n |= 0xFF00;
        BIT8_LOAD(registerAF.hi, n);
        return 12;
    case 0x01:
        BIT8_LOAD(registerBC.reg);
        return 12;
    case 0x11:
        BIT8_LOAD(registerDE.reg);
        return 12;
    case 0x21:
        BIT8_LOAD(registerHL.reg);
        return 12;
    case 0x31:
        BIT8_LOAD(sp);
        return 12;
    case 0xF9:
        BIT8_LOAD(sp, registerHL.reg);
        return 8;
    case 0xF8: {
        n = read(pc);
        pc++;
        registerAF.lo = resetBit(registerAF.lo, Z);
        registerAF.lo = resetBit(registerAF.lo, N);

        uint16_t value = (sp + n) & 0xFFFF;
        registerHL.reg = value;

        uint32_t v = sp + n;
        if (v > 0xFFFF)
            registerAF.lo = setBit(registerAF.lo, C);
        else
            registerAF.lo = resetBit(registerAF.lo, C);
        if (((registerAF.lo & 0xF) + (n & 0xF)) > 0xF)
            registerAF.lo = setBit(registerAF.lo, H);
        else
            registerAF.lo = resetBit(registerAF.lo, H);
        return 12;
    }
    case 0x08:
        nn = read16();
        pc += 2;
        BIT8_LOAD(nn, sp);
        return 20;
    case 0xF5:
        PUSH16(registerAF.reg);
        return 16;
    case 0xC5:
        PUSH16(registerBC.reg);
        return 16;
    case 0xD5:
        PUSH16(registerDE.reg);
        return 16;
    case 0xE5:
        PUSH16(registerHL.reg);
        return 16;
    case 0xF1:
        POP16(registerAF.reg);
        return 12;
    case 0xC1:
        POP16(registerBC.reg);
        return 12;
    case 0xD1:
        POP16(registerDE.reg);
        return 12;
    case 0xE1:
        POP16(registerHL.reg);
        return 12;
    case 0x87:
        ADD_8BIT(registerAF.hi, registerAF.hi, false);
        return 4;
    case 0x80:
        ADD_8BIT(registerAF.hi, registerBC.hi, false);
        return 4;
    case 0x81:
        ADD_8BIT(registerAF.hi, registerBC.lo, false);
        return 4;
    case 0x82:
        ADD_8BIT(registerAF.hi, registerDE.hi, false);
        return 4;
    case 0x83:
        ADD_8BIT(registerAF.hi, registerDE.lo, false);
        return 4;
    case 0x84:
        ADD_8BIT(registerAF.hi, registerHL.hi, false);
        return 4;
    case 0x85:
        ADD_8BIT(registerAF.hi, registerHL.lo, false);
        return 4;
    case 0x86:
        ADD_8BIT(registerAF.hi, registerHL.reg, false);
        return 8;
    case 0xC6:
        n = read(pc);
        pc++;
        ADD_8BIT(registerAF.hi, n, false);
        return 8;
    case 0x8F:
        ADD_8BIT(registerAF.hi, registerHL.reg, true);
        return 4;
    case 0x88:
        ADD_8BIT(registerAF.hi, registerBC.hi, true);
        return 4;
    case 0x89:
        ADD_8BIT(registerAF.hi, registerBC.lo, true);
        return 4;
    case 0x8A:
        ADD_8BIT(registerAF.hi, registerDE.hi, true);
        return 4;
    case 0x8B:
        ADD_8BIT(registerAF.hi, registerDE.lo, true);
        return 4;
    case 0x8C:
        ADD_8BIT(registerAF.hi, registerHL.hi, true);
        return 4;
    case 0x8D:
        ADD_8BIT(registerAF.hi, registerHL.lo, true);
        return 4;
    case 0x8E:
        ADD_8BIT(registerAF.hi, registerHL.reg, true);
        return 8;
    case 0xCE:
        n = read(pc);
        pc++;
        ADD_8BIT(registerAF.hi, n, true);
        return 8;
    case 0x97:
        SUB_8BIT(registerAF.hi, registerAF.hi, false);
        return 4;
    case 0x90:
        SUB_8BIT(registerAF.hi, registerBC.hi, false);
        return 4;
    case 0x91:
        SUB_8BIT(registerAF.hi, registerBC.lo, false);
        return 4;
    case 0x92:
        SUB_8BIT(registerAF.hi, registerDE.hi, false);
        return 4;
    case 0x93:
        SUB_8BIT(registerAF.hi, registerDE.lo, false);
        return 4;
    case 0x94:
        SUB_8BIT(registerAF.hi, registerHL.hi, false);
        return 4;
    case 0x95:
        SUB_8BIT(registerAF.hi, registerHL.lo, false);
        return 4;
    case 0x96:
        SUB_8BIT(registerAF.hi, registerHL.reg, false);
        return 8;
    case 0xD6:
        n = read(pc);
        pc++;
        SUB_8BIT(registerAF.hi, n, false);
        return 8;
    case 0x9F:
        SUB_8BIT(registerAF.hi, registerAF.hi, true);
        return 4;
    case 0x98:
        SUB_8BIT(registerAF.hi, registerBC.hi, true);
        return 4;
    case 0x99:
        SUB_8BIT(registerAF.hi, registerBC.lo, true);
        return 4;
    case 0x9A:
        SUB_8BIT(registerAF.hi, registerDE.hi, true);
        return 4;
    case 0x9B:
        SUB_8BIT(registerAF.hi, registerDE.lo, true);
        return 4;
    case 0x9C:
        SUB_8BIT(registerAF.hi, registerHL.hi, true);
        return 4;
    case 0x9D:
        SUB_8BIT(registerAF.hi, registerHL.lo, true);
        return 4;
    case 0x9E:
        SUB_8BIT(registerAF.hi, registerHL.reg, true);
        return 8;
        /*case ??:
            n = read(pc);
            pc++;
            SUB_8BIT(registerAF.hi, n, false);
            return ?;*/
    case 0xDE:
        n = read(pc);
        pc++;
        SUB_8BIT(registerAF.hi, n, true);
        return 8;
    case 0xA7:
        AND_8BIT(registerAF.hi, registerAF.hi);
        return 4;
    case 0xA0:
        AND_8BIT(registerAF.hi, registerBC.hi);
        return 4;
    case 0xA1:
        AND_8BIT(registerAF.hi, registerBC.lo);
        return 4;
    case 0xA2:
        AND_8BIT(registerAF.hi, registerDE.hi);
        return 4;
    case 0xA3:
        AND_8BIT(registerAF.hi, registerDE.lo);
        return 4;
    case 0xA4:
        AND_8BIT(registerAF.hi, registerHL.hi);
        return 4;
    case 0xA5:
        AND_8BIT(registerAF.hi, registerHL.lo);
        return 4;
    case 0xA6:
        AND_8BIT(registerAF.hi, registerHL.reg);
        return 8;
    case 0xE6:
        n = read(pc);
        pc++;
        AND_8BIT(registerAF.hi, n);
        return 8;
    case 0xB7:
        OR_8BIT(registerAF.hi, registerAF.hi);
        return 4;
    case 0xB0:
        OR_8BIT(registerAF.hi, registerBC.hi);
        return 4;
    case 0xB1:
        OR_8BIT(registerAF.hi, registerBC.lo);
        return 4;
    case 0xB2:
        OR_8BIT(registerAF.hi, registerDE.hi);
        return 4;
    case 0xB3:
        OR_8BIT(registerAF.hi, registerDE.lo);
        return 4;
    case 0xB4:
        OR_8BIT(registerAF.hi, registerHL.hi);
        return 4;
    case 0xB5:
        OR_8BIT(registerAF.hi, registerHL.lo);
        return 4;
    case 0xB6:
        OR_8BIT(registerAF.hi, registerHL.reg);
        return 8;
    case 0xF6:
        n = read(pc);
        pc++;
        OR_8BIT(registerAF.hi, n);
        return 8;
    case 0xAF:
        XOR_8BIT(registerAF.hi, registerAF.hi);
        return 4;
    case 0xA8:
        XOR_8BIT(registerAF.hi, registerBC.hi);
        return 4;
    case 0xA9:
        XOR_8BIT(registerAF.hi, registerBC.lo);
        return 4;
    case 0xAA:
        XOR_8BIT(registerAF.hi, registerDE.hi);
        return 4;
    case 0xAB:
        XOR_8BIT(registerAF.hi, registerDE.lo);
        return 4;
    case 0xAC:
        XOR_8BIT(registerAF.hi, registerHL.hi);
        return 4;
    case 0xAD:
        XOR_8BIT(registerAF.hi, registerHL.lo);
        return 4;
    case 0xAE:
        XOR_8BIT(registerAF.hi, registerHL.reg);
        return 8;
    case 0xEE:
        n = read(pc);
        pc++;
        XOR_8BIT(registerAF.hi, n);
        return 8;
    case 0xBF:
        CP_8BIT(registerAF.hi, registerAF.hi);
        return 4;
    case 0xB8:
        CP_8BIT(registerAF.hi, registerBC.hi);
        return 4;
    case 0xB9:
        CP_8BIT(registerAF.hi, registerBC.lo);
        return 4;
    case 0xBA:
        CP_8BIT(registerAF.hi, registerDE.hi);
        return 4;
    case 0xBB:
        CP_8BIT(registerAF.hi, registerDE.lo);
        return 4;
    case 0xBC:
        CP_8BIT(registerAF.hi, registerHL.hi);
        return 4;
    case 0xBD:
        CP_8BIT(registerAF.hi, registerHL.lo);
        return 4;
    case 0xBE:
        CP_8BIT(registerAF.hi, registerHL.reg);
        return 8;
    case 0xFE:
        n = read(pc);
        pc++;
        CP_8BIT(registerAF.hi, n);
        return 8;
    case 0x3C:
        INC_8BIT(registerAF.hi);
        return 4;
    case 0x04:
        INC_8BIT(registerBC.hi);
        return 4;
    case 0x0C:
        INC_8BIT(registerBC.lo);
        return 4;
    case 0x14:
        INC_8BIT(registerDE.hi);
        return 4;
    case 0x1C:
        INC_8BIT(registerDE.lo);
        return 4;
    case 0x24:
        INC_8BIT(registerHL.hi);
        return 4;
    case 0x2C:
        INC_8BIT(registerHL.lo);
        return 4;
    case 0x34:
        INC_16BIT(registerHL.reg);
        return 12;
    case 0x3D:
        DEC_8BIT(registerAF.hi);
        return 4;
    case 0x05:
        DEC_8BIT(registerBC.hi);
        return 4;
    case 0x0D:
        DEC_8BIT(registerBC.lo);
        return 4;
    case 0x15:
        DEC_8BIT(registerDE.hi);
        return 4;
    case 0x1D:
        DEC_8BIT(registerDE.lo);
        return 4;
    case 0x25:
        DEC_8BIT(registerHL.hi);
        return 4;
    case 0x2D:
        DEC_8BIT(registerHL.lo);
        return 4;
    case 0x35:
        DEC_16BIT(registerHL.reg);
        return 12;
    case 0x09:
        ADD_16BIT(registerHL.reg, registerBC.reg);
        return 8;
    case 0x19:
        ADD_16BIT(registerHL.reg, registerDE.reg);
        return 8;
    case 0x29:
        ADD_16BIT(registerHL.reg, registerHL.reg);
        return 8;
    case 0x39:
        ADD_16BIT(registerHL.reg, sp);
        return 8;
    case 0xE8:
        n = read(pc);
        pc++;
        ADD_16BIT(sp, n);
        return 16;
    case 0x03:
        registerBC.reg++;
        return 8;
    case 0x13:
        registerDE.reg++;
        return 8;
    case 0x23:
        registerHL.reg++;
        return 8;
    case 0x33:
        sp++;
        return 8;
    case 0x0B:
        registerBC.reg--;
        return 8;
    case 0x1B:
        registerDE.reg--;
        return 8;
    case 0x2B:
        registerHL.reg--;
        return 8;
    case 0x3B:
        sp--;
        return 8;
    case 0x27:
        DAA();
        return 4;
    case 0x2F:
        CPL();
        return 4;
    case 0x3F:
        CCF();
        return 4;
    case 0x37:
        SCF();
        return 4;
    case 0xF3://interrupt variable might need to change AFTER instruction
        enableInterrupts = false;
        return 4;
    case 0xFB:
        enableInterrupts = true;
        return 4;
    case 0x00:
        return 4;
    case 0x76:
        halted = true;
        return 4;
    case 0x10:
        return 4;
    case 0x07:
        RLC(registerAF.hi);
        return 4;
    case 0x17:
        RL(registerAF.hi);
        return 4;
    case 0x0F:
        RRC(registerAF.hi);
        return 4;
    case 0x1F:
        RR(registerAF.hi);
        return 4;
    case 0xCB: {
        pc++;
        uint8_t exOpcode = read(pc);
        return executeExOP(exOpcode);
    }
    default:
        std::cout << "Unknown opcode" << std::endl;
        return 0;
    }//end of switch
}

int z80::executeExOP(uint8_t opcode) {

    switch (opcode) {
    case 0x37:
        SWAP_NIBBLES(registerAF.hi);
        return 8;
    case 0x30:
        SWAP_NIBBLES(registerBC.hi);
        return 8;
    case 0x31:
        SWAP_NIBBLES(registerBC.lo);
        return 8;
    case 0x32:
        SWAP_NIBBLES(registerDE.hi);
        return 8;
    case 0x33:
        SWAP_NIBBLES(registerDE.lo);
        return 8;
    case 0x34:
        SWAP_NIBBLES(registerHL.hi);
        return 8;
    case 0x35:
        SWAP_NIBBLES(registerHL.lo);
        return 8;
    case 0x36:
        SWAP_NIB_16(registerHL.reg);
        return 16;
    case 0x07:
        RLC(registerAF.hi);
        return 8;
    case 0x00:
        RLC(registerBC.hi);
        return 8;
    case 0x01:
        RLC(registerBC.lo);
        return 8;
    case 0x02:
        RLC(registerDE.hi);
        return 8;
    case 0x03:
        RLC(registerDE.lo);
        return 8;
    case 0x04:
        RLC(registerHL.hi);
        return 8;
    case 0x05:
        RLC(registerHL.lo);
        return 8;
    case 0x06:
        RLCMem(registerHL.reg);
        return 16;
    case 0x17:
        RL(registerAF.hi);
        return 8;
    case 0x10:
        RL(registerBC.hi);
        return 8;
    case 0x11:
        RL(registerBC.lo);
        return 8;
    case 0x12:
        RL(registerDE.hi);
        return 8;
    case 0x13:
        RL(registerDE.lo);
        return 8;
    case 0x14:
        RL(registerHL.hi);
        return 8;
    case 0x15:
        RL(registerHL.lo);
        return 8;
    case 0x16:
        RLMem(registerHL.reg);
        return 16;
    case 0x0F:
        RRC(registerAF.hi);
        return 8;
    case 0x08:
        RRC(registerBC.hi);
        return 8;
    case 0x09:
        RRC(registerBC.lo);
        return 8;
    case 0x0A:
        RRC(registerDE.hi);
        return 8;
    case 0x0B:
        RRC(registerDE.lo);
        return 8;
    case 0x0C:
        RRC(registerHL.hi);
        return 8;
    case 0x0D:
        RRC(registerHL.lo);
        return 8;
    case 0x0E:
        RRCMem(registerHL.reg);
        return 16;
    case 0x1F:
        RR(registerAF.hi);
        return 8;
    case 0x18:
        RR(registerBC.hi);
        return 8;
    case 0x19:
        RR(registerBC.lo);
        return 8;
    case 0x1A:
        RR(registerDE.hi);
        return 8;
    case 0x1B:
        RR(registerDE.lo);
        return 8;
    case 0x1C:
        RR(registerHL.hi);
        return 8;
    case 0x1D:
        RR(registerHL.lo);
        return 8;
    case 0x1E:
        RRMem(registerHL.reg);
        return 16;
    case 0x27:
        SLA(registerAF.hi);
        return 8;
    case 0x20:
        SLA(registerBC.hi);
        return 8;
    case 0x21:
        SLA(registerBC.lo);
        return 8;
    case 0x22:
        SLA(registerDE.hi);
        return 8;
    case 0x23:
        SLA(registerDE.lo);
        return 8;
    case 0x24:
        SLA(registerHL.hi);
        return 8;
    case 0x25:
        SLA(registerHL.lo);
        return 8;
    case 0x26:
        SLAMem(registerHL.reg);
        return 16;
    case 0x2F:
        SRA(registerAF.hi);
        return 8;
    case 0x28:
        SRA(registerBC.hi);
        return 8;
    case 0x29:
        SRA(registerBC.lo);
        return 8;
    case 0x2A:
        SRA(registerDE.hi);
        return 8;
    case 0x2B:
        SRA(registerDE.lo);
        return 8;
    case 0x2C:
        SRA(registerHL.hi);
        return 8;
    case 0x2D:
        SRA(registerHL.lo);
        return 8;
    case 0x2E:
        SRAMem(registerHL.reg);
        return 16;
    case 0x3F:
        SRL(registerAF.hi);
        return 8;
    case 0x38:
        SRL(registerBC.hi);
        return 8;
    case 0x39:
        SRL(registerBC.lo);
        return 8;
    case 0x3A:
        SRL(registerDE.hi);
        return 8;
    case 0x3B:
        SRL(registerDE.lo);
        return 8;
    case 0x3C:
        SRL(registerHL.hi);
        return 8;
    case 0x3D:
        SRL(registerHL.lo);
        return 8;
    case 0x3E:
        SRLMem(registerHL.reg);
        return 8;


    }//end of switch
}