#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
//#include <netinet/in.h>
#include "epb_MmBp.h"
#include "crc32.h"
#include "aes_crypt.h"
#include "ble_gap.h"

#define BigLittleSwap16(A)  ((((uint16_t)(A) & 0xff00) >> 8) | \
                            (((uint16_t)(A) & 0x00ff) << 8))


#define BigLittleSwap32(A)  ((((uint32_t)(A) & 0xff000000) >> 24) | \
                            (((uint32_t)(A) & 0x00ff0000) >> 8) | \
                            (((uint32_t)(A) & 0x0000ff00) << 8) | \
                            (((uint32_t)(A) & 0x000000ff) << 24))

typedef struct
{
	uint8_t bMagicNumber;
	uint8_t bVer;
	uint16_t nLength;
	uint16_t nCmdId;
	uint16_t nSeq;
}BpFixHead;


void wechat_pack_request(uint16_t cmdid, uint8_t *buf ,uint8_t *data, uint16_t len);
int wechat_unpack_response(uint8_t *buf);

