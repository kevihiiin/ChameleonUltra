#pragma once

#include "protocols.h"

extern const protocol motorola;

uint8_t motorola_t55xx_writer(uint8_t* uid, uint32_t* blks);
