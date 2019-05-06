/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _IO_CROSSFIRE_H_
#define _IO_CROSSFIRE_H_

#include <stdint.h>
#include "dataconstants.h"
#include "definitions.h"
#include "fifo.h"
#include "ff.h"
#include "sdcard.h"
#include "crsf.h"


typedef enum
{
  DEVICE_INTERNAL = 0,
  USB_HID,
  CRSF_SHARED_FIFO,
  CRSF_ESP,
  LAST_CRSF_PORT
} _CRSF_PORT_NAME;


#define TELEMETRY_BUFFER_SIZE           128
#define CROSSFIRE_FIFO_SIZE             256
#define CROSSFIRE_CHANNELS_COUNT        16

struct CrossfireKeyData {
  union {
    struct {
      uint8_t left;
      uint8_t right;
      uint8_t up;
      uint8_t down;
      uint8_t enter;
      uint8_t enter_long;
      uint8_t enter_double_click;
      uint8_t exit;
      uint8_t exit_long;
      uint8_t exit_double_click;
    };
    uint8_t values[12];
  };
};

struct CrossfireSharedData {
  struct CrossfireKeyData keys;
  int16_t sticks[NUM_STICKS];
  uint8_t stick_state;
  int32_t channels[CROSSFIRE_CHANNELS_COUNT];
  Fifo<uint8_t, CROSSFIRE_FIFO_SIZE> crsf_tx;   //from XF to OpenTX
  Fifo<uint8_t, CROSSFIRE_FIFO_SIZE> crsf_rx;   //from OpenTX to XF
#if defined(CRSF_OPENTX) && defined(CRSF_SD)
  uint32_t crsfHandlerAddress;
#endif
};

#if defined(CRSF_OPENTX) && defined(CRSF_SD)
//#define DEBUG_CRSF_SD_READ_COMPARE
//#define DEBUG_CRSF_SD_WRITE_COMPARE
//#define DEBUG_CRSF_SD_READ
//#define DEBUG_CRSF_SD_WRITE
//#define DEBUG_CRSF_SD_ERASE
#define CRSF_SD_DELAY					50
#define CRSF_SD_WRITE_DELAY				50
#define CRSF_SD_SUBCMD_ADD				5
#define CRSF_SD_DATA_START_ADD			15
#define CRSF_SD_DATA_MAX_BUFFER_SIZE	(LIBCRSF_MAX_BUFFER_SIZE - CRSF_SD_DATA_START_ADD - LIBCRSF_CRC_SIZE)

typedef enum {
	CRSF_SD_START = 0,
	CRSF_SD_FINISH,
	CRSF_SD_WRITE,
	CRSF_SD_READ,
	CRSF_SD_ACK,
	CRSF_SD_ERASE,
	CRSF_SD_BUSY,
	CRSF_SD_ERROR,
	CRSF_SD_IDLE,
	CRSF_SD_DATA_READY,
	CRSF_SD_DATA_TX_COMPLETED,
	CRSF_SD_INVALID_FILENAME
} CRSF_SD_STATE;

typedef enum {
	CRSF_SD_SUBCMD_READ = 0x01,
	CRSF_SD_SUBCMD_WRITE = 0x02,
	CRSF_SD_SUBCMD_ERASE = 0x03
} CRSF_SD_SUBCMD;

typedef enum {
	CRSF_SD_FLAG_OK = 0,
	CRSF_SD_FLAG_START,
	CRSF_SD_FLAG_FINISH,
	CRSF_SD_FLAG_ACK,
	CRSF_SD_FLAG_ERROR,
	CRSF_SD_FLAG_RETRY,
	CRSF_SD_FLAG_RETRY_START
} CRSF_SD_FLAG;

typedef struct CRSF_SD_STRUCT{
	FIL file;
	FILINFO fileInfo;
	FRESULT result;
	uint32_t bytesReceived = 0;
	uint32_t bytesPendingToSend = 0;
	uint8_t sync;
	uint8_t length;
	uint8_t cmd;
	uint8_t dst;
	uint8_t org;
	uint8_t subcmd;
	uint8_t flag;
	uint32_t chunk;
	uint32_t requestDataLength;
	uint8_t crc;
	uint8_t calCrc;
	uint32_t numOfBytesRead;
	uint32_t numOfBytesWritten;
	BYTE txBuf[LIBCRSF_MAX_BUFFER_SIZE];
	BYTE rxBuf[LIBCRSF_MAX_BUFFER_SIZE];
	BYTE payload[CRSF_SD_DATA_MAX_BUFFER_SIZE];
} CrsfSd_t;

extern uint8_t enableOpentxSdWriteHandler;
extern uint8_t enableOpentxSdReadHandler;
extern uint8_t enableOpentxSdEraseHandler;

void crsfSdHandler();

#endif

typedef struct CrossfireSharedData CrossfireSharedData_t;
#define crossfireSharedData (*((CrossfireSharedData_t*)SHARED_MEMORY_ADDRESS))

void CRSF_Init( void );
void crsfSharedFifoHandler( void );
void crsfEspHandler( void );
void CRSF_to_Shared_FIFO( uint8_t *p_arr );
void CRSF_to_ESP( uint8_t *p_arr );
void CRSF_This_Device( uint8_t *p_arr );
void AgentLegacyCalls( uint8_t *arr );

#endif // _CROSSFIRE_H_
