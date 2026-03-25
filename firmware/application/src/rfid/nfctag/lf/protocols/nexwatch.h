#pragma once

#include "protocols.h"

extern const protocol nexwatch;

uint8_t nexwatch_t55xx_writer(uint8_t* uid, uint32_t* blks);
