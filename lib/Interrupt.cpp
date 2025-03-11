#include "Interrupt.h"

Interrupt::Interrupt(z80* c)
{
	cpu = c;
}





Interrupt::~Interrupt()
{
}

void Interrupt::intHandle(uint16_t addr)
{
	cpu->pushShortToStack(cpu->pc);
	cpu->pc = addr;
}

bool Interrupt::intCheck(uint16_t addr, InterruptFlags f)
{
	if ((cpu->int_flags & f) && (cpu->ie_register & f)) {
		intHandle(addr);
		cpu->int_flags &= f;
		cpu->halted = false;
		cpu->interruptsEnabled = false;
	}

	return false;
}

void Interrupt::cpuRequestInterrupt(InterruptFlags f)
{

}



void Interrupt::cpuHandleInterrupts()
{
	if (intCheck(0x40, INT_VBLANK)) {

	}
	else if (intCheck(0x48, INT_LCD)) {

	}
	else if (intCheck(0x50, INT_TIMER)) {

	}
	else if (intCheck(0x58, INT_SERIAL)) {

	}
	else if (intCheck(0x60, INT_JOYPAD)) {

	}
}
