#pragma once
#include <stdint.h>

typedef struct __attribute__((packed)) {
	uint8_t mode;
	uint8_t id_lo;
	uint8_t id_hi;
	float   x_le;
	float   y_le;
	float   z_le;
} lighthouse_pkt_t;