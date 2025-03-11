#pragma once
#include "bus.h"

class Bus;
class dbg
{
public:
	dbg(Bus* b);
	Bus *bus = nullptr;
	char dbg_msg[1024] = { 0 };
	int msg_size = 0;
	void dbgUpdate();
	void dbgPrint();
};

