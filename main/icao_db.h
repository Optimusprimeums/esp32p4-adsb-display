#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Country of registration from the ICAO 24-bit address block, plus a
 * "probably military" flag from known military sub-ranges. */
void icao_lookup(uint32_t addr, const char **country, bool *military);
