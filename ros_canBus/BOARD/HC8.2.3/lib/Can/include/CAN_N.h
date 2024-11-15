#ifndef __CAN_N_H__
#define __CAN_N_H__

#include <stdint.h>
#include "CAN_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef union {
	uint16_t Identifier;
	bool RTR : 1;
	bool IDE : 1;
	bool R0 : 0;
	uint8_t DLC : 4;
	union {
		uint8_t u8[8];   /**< \brief Payload byte access*/
		uint32_t u32[2]; /**< \brief Payload u32 access*/
		uint16_t u16[4];
	} Data;
	uint16_t CRC;
	uint8_t ACK;
} CAN_all_t;

#ifdef __cplusplus
}
#endif

#endif