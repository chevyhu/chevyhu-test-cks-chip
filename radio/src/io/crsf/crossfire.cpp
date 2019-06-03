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


#include <string.h>
#include "crsf.h"
#include "crsf_write.h"
#include "crsf_utilities.h"
#include "usb_driver.h"
#include "board.h"
#include "debug.h"
#include "crossfire.h"

extern void CRSF_To_USB_HID( uint8_t *p_arr );
extern void usbAgentWrite( uint8_t *p_arr );

#define ARRAY_SIZE(array) (sizeof(array)/sizeof(array[0]))

_libCrsf_CRSF_Port CRSF_Ports[] = {
         { DEVICE_INTERNAL,   &CRSF_This_Device }
        ,{ USB_HID,           &CRSF_To_USB_HID }
        ,{ CRSF_SHARED_FIFO,  &CRSF_to_Shared_FIFO }
        ,{ CRSF_ESP,  &CRSF_to_ESP }
};

// TODO: perna link to correct data
static uint8_t libCrsf_MySlaveAddress = LIBCRSF_REMOTE_ADD;
static char *libCrsf_MyDeviceName = "TBS TANGO II";
static uint32_t libCrsf_MySerialNo = 1;
static uint32_t libCrsf_MyHwID = 0x040001;
static uint32_t libCrsf_MyFwID = 0x0100;

Fifo<uint8_t, TELEMETRY_BUFFER_SIZE> crsf_telemetry_buffer;

extern Fifo<uint8_t, ESP_TX_BUFFER_SIZE> espTxFifo;
extern Fifo<uint8_t, ESP_RX_BUFFER_SIZE> espRxFifo;

#if defined(CRSF_OPENTX) && defined(CRSF_SD)

static uint8_t OpentxBuf[LIBCRSF_MAX_BUFFER_SIZE];
static uint8_t isOpentxBufSet = 0;

uint8_t enableOpentxSdWriteHandler = 0;
uint8_t enableOpentxSdReadHandler = 0;
uint8_t enableOpentxSdEraseHandler = 0;

static void SetOpentxBuf(uint8_t* p_arr){
	memcpy(OpentxBuf, p_arr, LIBCRSF_MAX_BUFFER_SIZE);
	isOpentxBufSet = 1;
}

#endif

void CRSF_Init( void )
{
  /* init crsf library */

  memset( &crossfireSharedData, 0, sizeof(CrossfireSharedData) );

  libCrsf_Init( libCrsf_MySlaveAddress, libCrsf_MyDeviceName, libCrsf_MySerialNo, libCrsf_MyHwID, libCrsf_MyFwID );

  libCrsf_CRSF_Add_Device_Function_List( &CRSF_Ports[0], ARRAY_SIZE(CRSF_Ports));

  crossfireSharedData.crsfHandlerAddress = (uint32_t)crsfSdHandler;

}

void CRSF_This_Device( uint8_t *p_arr )
{

  uint8_t arr[LIBCRSF_MAX_BUFFER_SIZE];
  uint32_t i = 0;

  /* handle parameter and command frames */
  switch ( *(p_arr + LIBCRSF_TYPE_ADD) )
  {
//    TODO: perna handle CRSF parameter frames once agent is ready
    case LIBCRSF_EX_PARAM_PING_DEVICE:
      if ( libCrsf_checkif_devicecalled( p_arr, true )) {
//        TRACE("LIBCRSF_EX_PARAM_PING_DEVICE");
        // Parameter_Pack_Device_Information( &arr[LIBCRSF_LENGTH_ADD] );
        libCrsf_crsfwrite( LIBCRSF_EX_PARAM_DEVICE_INFO, &arr[ LIBCRSF_LENGTH_ADD ] );
        libCrsf_CRSF_Routing( DEVICE_INTERNAL, &arr[0] );
      }
      break;

#if defined(CRSF_OPENTX)
    case LIBCRSF_OPENTX_RELATED:
#if defined(CRSF_SD)
      if( *(p_arr + CRSF_SD_SUBCMD_ADD) == CRSF_SD_SUBCMD_READ ){
    	  SetOpentxBuf(p_arr);
		  enableOpentxSdWriteHandler = 1;
      }
      else if( *(p_arr + CRSF_SD_SUBCMD_ADD) == CRSF_SD_SUBCMD_WRITE ){
    	  SetOpentxBuf(p_arr);
    	  enableOpentxSdReadHandler = 1;
      }
      else if( *(p_arr + CRSF_SD_SUBCMD_ADD) == CRSF_SD_SUBCMD_ERASE ){
    	  SetOpentxBuf(p_arr);
		  enableOpentxSdEraseHandler = 1;
      }
#endif
      break;
#endif
    case LIBCRSF_CMD_FRAME:
      if( *(p_arr + CRSF_SD_SUBCMD_ADD) == 0x06 ){
        TRACE("got 06 command\r\n");
      }
      else if ( *(p_arr + CRSF_SD_SUBCMD_ADD) == 0x07 ){
        TRACE("got 07 command\r\n");
      }
      break;

    default:
      // Buffer telemetry data inside a FIFO to let telemetryWakeup read from it and keep the
      // compatibility with the existing telemetry infrastructure.
      // TODO: perna clean debug printf
//      debugPrintf("\r\n");
      for(i = 0; i < *(p_arr + LIBCRSF_LENGTH_ADD) + 2; i++) {
//        debugPrintf("%x,", *(p_arr + i) );
        crsf_telemetry_buffer.push(*(p_arr + i));
      }
//      debugPrintf("\r\n");
      break;
  }
}

uint8_t telemetryGetByte(uint8_t * byte)
{
  bool res = crsf_telemetry_buffer.pop(*byte);
#if defined(LUA)
  if (telemetryProtocol == PROTOCOL_TELEMETRY_CROSSFIRE) //henry: what is the protocol?
  {
    static uint8_t prevdata;
    if (prevdata == 0x7E && outputTelemetryBufferSize > 0 && *byte == outputTelemetryBufferTrigger) {
      sportSendBuffer(outputTelemetryBuffer, outputTelemetryBufferSize);
    }
    prevdata = *byte;
  }
#endif
  return res;
}

void CRSF_to_Shared_FIFO( uint8_t *p_arr )
{
  for( uint8_t i = 0; i < (*(p_arr + LIBCRSF_LENGTH_ADD) + LIBCRSF_HEADER_OFFSET + LIBCRSF_CRC_SIZE); i++ ) {
    crossfireSharedData.crsf_rx.push(*(p_arr + i));
  }
}

void CRSF_to_ESP( uint8_t *p_arr )
{
  for( uint8_t i = 0; i < (*(p_arr + LIBCRSF_LENGTH_ADD) + LIBCRSF_HEADER_OFFSET + LIBCRSF_CRC_SIZE); i++ ) {
	  espTxFifo.push(*(p_arr + i));
  }
}

void crsfSharedFifoHandler( void )
{
  uint8_t byte;
  static _libCrsf_CRSF_PARSE_DATA CRSF_Data;
  if ( crossfireSharedData.crsf_tx.pop(byte) ){
    if ( libCrsf_CRSF_Parse( &CRSF_Data, byte )) {
      libCrsf_CRSF_Routing( CRSF_SHARED_FIFO, CRSF_Data.Payload );
    }
  }
}

void crsfSetModelID(void)
{
  uint32_t count = 0;
  BYTE txBuf[LIBCRSF_MAX_BUFFER_SIZE];

  libUtil_Write8(txBuf, &count, LIBCRSF_UART_SYNC); /* device address */
  libUtil_Write8(txBuf, &count, 0);                 /* frame length */
  libUtil_Write8(txBuf, &count, LIBCRSF_CMD_FRAME); /* cmd type */
  libUtil_Write8(txBuf, &count, LIBCRSF_RC_TX);     /* Destination Address */
  libUtil_Write8(txBuf, &count, LIBCRSF_REMOTE_ADD);/* Origin Address */
  libUtil_Write8(txBuf, &count, 0x10);              /* sub command */
  libUtil_Write8(txBuf, &count, 0x06);              /* command of set model/receiver id */
  libUtil_Write8(txBuf, &count, modelHeaders[g_eeGeneral.currModel].modelId[EXTERNAL_MODULE]);  /* model ID */

  uint8_t crc = libCRC8_Get_CRC_Arr(&txBuf[2], count-2, POLYNOM_1);
  libUtil_Write8(txBuf, &count, crc);
  txBuf[LIBCRSF_LENGTH_ADD] = count - 2;

  CRSF_to_Shared_FIFO(txBuf);

  TRACE("set model id command\r\n");
}


void crsfEspHandler( void )
{
  uint8_t byte;
  static _libCrsf_CRSF_PARSE_DATA CRSF_Data;
  if ( espRxFifo.pop(byte) ) {
    if ( libCrsf_CRSF_Parse( &CRSF_Data, byte )) {
	  libCrsf_CRSF_Routing( CRSF_ESP, CRSF_Data.Payload );
    }
  }
  ESP_WriteHandler();
}

#if defined(CRSF_OPENTX) && defined(CRSF_SD)


#if defined(DEBUG_CRSF_SD_READ) || defined(DEBUG_CRSF_SD_WRITE) || defined(DEBUG_CRSF_SD_ERASE)

static void TraceState(char* prefix, uint8_t state){
	switch(state){
		case CRSF_SD_START:
			TRACE("%sCRSF_SD_START", prefix);
			break;
		case CRSF_SD_FINISH:
			TRACE("%sCRSF_SD_FINISH", prefix);
			break;
		case CRSF_SD_WRITE:
			TRACE("%sCRSF_SD_WRITE", prefix);
			break;
		case CRSF_SD_READ:
			TRACE("%sCRSF_SD_READ", prefix);
			break;
		case CRSF_SD_ACK:
			TRACE("%sCRSF_SD_ACK", prefix);
			break;
		case CRSF_SD_ERASE:
			TRACE("%sCRSF_SD_ERASE", prefix);
			break;
		case CRSF_SD_BUSY:
			TRACE("%sCRSF_SD_BUSY", prefix);
			break;
		case CRSF_SD_ERROR:
			TRACE("%sCRSF_SD_ERROR", prefix);
			break;
		case CRSF_SD_IDLE:
			TRACE("%sCRSF_SD_IDLE", prefix);
			break;
		case CRSF_SD_DATA_READY:
			TRACE("%sCRSF_SD_DATA_READY", prefix);
			break;
		case CRSF_SD_DATA_TX_COMPLETED:
			TRACE("%sCRSF_SD_DATA_TX_COMPLETED", prefix);
			break;
		case CRSF_SD_INVALID_FILENAME:
			TRACE("%sCRSF_SD_INVALID_FILENAME", prefix);
			break;

	}
}

static void TraceFlag(char* prefix, uint8_t flag){
	switch(flag){
		case CRSF_SD_FLAG_OK:
			TRACE("%sCRSF_SD_FLAG_OK", prefix);
			break;
		case CRSF_SD_FLAG_START:
			TRACE("%sCRSF_SD_FLAG_START", prefix);
			break;
		case CRSF_SD_FLAG_FINISH:
			TRACE("%sCRSF_SD_FLAG_FINISH", prefix);
			break;
		case CRSF_SD_FLAG_ACK:
			TRACE("%sCRSF_SD_FLAG_ACK", prefix);
			break;
		case CRSF_SD_FLAG_ERROR:
			TRACE("%sCRSF_SD_FLAG_ERROR", prefix);
			break;
		case CRSF_SD_FLAG_RETRY:
			TRACE("%sCRSF_SD_FLAG_RETRY", prefix);
			break;
		case CRSF_SD_FLAG_RETRY_START:
			TRACE("%sCRSF_SD_FLAG_RETRY_START", prefix);
			break;

	}
}

#endif // defined(DEBUG_CRSF_SD_READ) || defined(DEBUG_CRSF_SD_WRITE) || defined(DEBUG_CRSF_SD_ERASE)

static void crsfSdPackFrame(CrsfSd_t* crsfSd){
	uint32_t count = 0;
	libUtil_Write8(crsfSd->txBuf, &count, LIBCRSF_UART_SYNC);
	libUtil_Write8(crsfSd->txBuf, &count, 0);
	libUtil_Write8(crsfSd->txBuf, &count, LIBCRSF_OPENTX_RELATED);
	libUtil_Write8(crsfSd->txBuf, &count, crsfSd->dst);
	libUtil_Write8(crsfSd->txBuf, &count, crsfSd->org);
	libUtil_Write8(crsfSd->txBuf, &count, crsfSd->subcmd);
	libUtil_Write8(crsfSd->txBuf, &count, crsfSd->flag);
	libUtil_Write32(crsfSd->txBuf, &count, crsfSd->chunk);
	libUtil_Write32(crsfSd->txBuf, &count, crsfSd->requestDataLength);
	libUtil_WriteBytes(crsfSd->txBuf, &count, crsfSd->payload, crsfSd->numOfBytesRead);
	crsfSd->crc = libCRC8_Get_CRC_Arr(&crsfSd->txBuf[2], count-2, POLYNOM_1);
	libUtil_Write8(crsfSd->txBuf, &count, crsfSd->crc);
	crsfSd->txBuf[LIBCRSF_LENGTH_ADD] = count - 2;
}

static void crsfSdUnpackFrame(CrsfSd_t* crsfSd){
	uint32_t count = 0;
	crsfSd->sync = libUtil_Read8(crsfSd->rxBuf, &count);
	crsfSd->length = libUtil_Read8(crsfSd->rxBuf, &count);
	crsfSd->cmd = libUtil_Read8(crsfSd->rxBuf, &count);
	crsfSd->dst = libUtil_Read8(crsfSd->rxBuf, &count);
	crsfSd->org = libUtil_Read8(crsfSd->rxBuf, &count);
	crsfSd->subcmd = libUtil_Read8(crsfSd->rxBuf, &count);
	crsfSd->flag = libUtil_Read8(crsfSd->rxBuf, &count);
	crsfSd->chunk = libUtil_Read32(crsfSd->rxBuf, &count);
	crsfSd->requestDataLength = libUtil_Read32(crsfSd->rxBuf, &count);
	for(uint8_t i = 0; i < crsfSd->length - CRSF_SD_DATA_START_ADD + 1; i++){
		crsfSd->payload[i] = libUtil_Read8(crsfSd->rxBuf, &count);
	}
	crsfSd->crc = libUtil_Read8(crsfSd->rxBuf, &count);
	crsfSd->calCrc = libCRC8_Get_CRC_Arr(&crsfSd->rxBuf[2], crsfSd->length-1, POLYNOM_1);
}

static void crsfSdEraseHandler(){
	static CrsfSd_t CrsfSdOpentx;
	static uint8_t state = CRSF_SD_START;
	static uint8_t targetDst = 0;
	static uint16_t stateRepeatCount = 0;
	static uint8_t prevState = CRSF_SD_START;

	switch(state){
		case CRSF_SD_START:
		{
			if(isOpentxBufSet){
				isOpentxBufSet = 0;
				memcpy(CrsfSdOpentx.rxBuf, OpentxBuf, LIBCRSF_MAX_BUFFER_SIZE);
				crsfSdUnpackFrame(&CrsfSdOpentx);
				#ifdef DEBUG_CRSF_SD_ERASE
				TRACE("crsfSdEraseHandler:CRSF_SD_START:sync:%x:length:%d:cmd:%x:subcmd:%d:flag:%d:chunk:%ld:crc:%d:calCrc:%d", CrsfSdOpentx.sync, CrsfSdOpentx.length, CrsfSdOpentx.cmd, CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.chunk, CrsfSdOpentx.crc, CrsfSdOpentx.calCrc);
				#endif

				if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == LIBCRSF_OPENTX_RELATED && CrsfSdOpentx.flag == CRSF_SD_FLAG_START){
					state = CRSF_SD_WRITE;
					targetDst = CrsfSdOpentx.org;
					CrsfSdOpentx.result = f_unlink((char*)CrsfSdOpentx.payload);
					delay_us(CRSF_SD_DELAY);
					if( CrsfSdOpentx.result == FR_OK ){
						#if defined(DEBUG_CRSF_SD_ERASE)
						TRACE("crsfSdEraseHandler:CRSF_SD_START:result:%d", CrsfSdOpentx.result);
						TRACE("crsfSdEraseHandler:CRSF_SD_START:del %s successful", CrsfSdOpentx.payload);
						#endif
					}
					else{
						#ifdef DEBUG_CRSF_SD_ERASE
						TRACE("crsfSdEraseHandler:CRSF_SD_START:result:%x", CrsfSdOpentx.result);
						TRACE("crsfSdEraseHandler:CRSF_SD_START:del failed");
						#endif
						state = CRSF_SD_IDLE;
					}
				}
				else{
					CrsfSdOpentx.dst = CrsfSdOpentx.org;
					CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
					CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_ERASE;
					CrsfSdOpentx.flag = CRSF_SD_FLAG_RETRY_START;
					CrsfSdOpentx.requestDataLength = 0;
					CrsfSdOpentx.numOfBytesRead = 0;
					crsfSdPackFrame(&CrsfSdOpentx);
					CRSF_to_Shared_FIFO(CrsfSdOpentx.txBuf);
					state = CRSF_SD_START;
					#ifdef DEBUG_CRSF_SD_ERASE
					TRACE("crsfSdEraseHandler:CRSF_SD_START:retry to write start");
					#endif
				}
			}
			break;
		}

		case CRSF_SD_WRITE:
		{
			if(crossfireSharedData.crsf_rx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdOpentx.dst = targetDst;
				CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
				CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_ERASE;
				CrsfSdOpentx.flag = CRSF_SD_FLAG_OK;
				crsfSdPackFrame(&CrsfSdOpentx);
				CRSF_to_Shared_FIFO(CrsfSdOpentx.txBuf);
				state = CRSF_SD_ACK;
				#ifdef DEBUG_CRSF_SD_ERASE
				TRACE("crsfSdEraseHandler:CRSF_SD_WRITE:subcmd:%d:flag:%d:bytesPendingToSend:%d:chunk:%ld", CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk);
				#endif
			}
			break;
		}

		case CRSF_SD_ACK:
		{
			if(isOpentxBufSet){
				isOpentxBufSet = 0;
				memcpy(CrsfSdOpentx.rxBuf, OpentxBuf, LIBCRSF_MAX_BUFFER_SIZE);
				crsfSdUnpackFrame(&CrsfSdOpentx);
				#ifdef DEBUG_CRSF_SD_ERASE
				TRACE("crsfSdEraseHandler:CRSF_SD_ACK:subcmd:%d:flag:%d:bytesPendingToSend:%d:chunk:%ld:numOfBytesRead:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk, CrsfSdOpentx.numOfBytesRead);
				#endif
				if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == LIBCRSF_OPENTX_RELATED && CrsfSdOpentx.flag == CRSF_SD_FLAG_FINISH){
					state = CRSF_SD_FLAG_FINISH;
					#if defined(DEBUG_CRSF_SD_ERASE)
					TRACE("crsfSdEraseHandler:CRSF_SD_ACK:subcmd:%d:flag:%d:bytesPendingToSend:%d:chunk:%ld", CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk);
					#endif
				}
				else{
					CrsfSdOpentx.flag = CRSF_SD_FLAG_RETRY;
					state = CRSF_SD_WRITE;
					#ifdef DEBUG_CRSF_SD_ERASE
					TRACE("crsfSdEraseHandler:CRSF_SD_ACK:retry to write");
					#endif
				}
			}
			break;
		}

		case CRSF_SD_FINISH:
		{
			#if defined(DEBUG_CRSF_SD_ERASE)
			TRACE("crsfSdEraseHandler:CRSF_SD_FINISH");
			#endif
			state = CRSF_SD_START;
			enableOpentxSdEraseHandler = 0;
			break;
		}

		case CRSF_SD_ERROR:
		{
			#ifdef DEBUG_CRSF_SD_ERASE
			TRACE("crsfSdEraseHandler:CRSF_SD_ERROR:%d", CrsfSdOpentx.result);
			#endif
			f_close(&CrsfSdOpentx.file);
			state = CRSF_SD_IDLE;

			break;
		}

		case CRSF_SD_IDLE:
		{
			state = CRSF_SD_START;
			enableOpentxSdEraseHandler = 0;
			break;
		}

		default:
		{
			break;
		}
	}

    if(state == prevState){
        if(stateRepeatCount++ > 1000){
        	enableOpentxSdEraseHandler = 0;
            stateRepeatCount = 0;
            state = CRSF_SD_START;
            TRACE("OPENTX CRSF_SD_ERASE KILL\r\n");
        }
    }
    else{
        stateRepeatCount = 0;
    }
    prevState = state;
}

static void crsfSdWriteHandler(){
	static CrsfSd_t CrsfSdOpentx;
	static uint8_t state = CRSF_SD_START;
	static uint8_t targetDst = 0;
	static uint16_t stateRepeatCount = 0;
	static uint8_t prevState = CRSF_SD_START;

	switch(state){
		case CRSF_SD_START:
		{
			if(isOpentxBufSet){
				isOpentxBufSet = 0;
				memcpy(CrsfSdOpentx.rxBuf, OpentxBuf, LIBCRSF_MAX_BUFFER_SIZE);
				crsfSdUnpackFrame(&CrsfSdOpentx);
				#ifdef DEBUG_CRSF_SD_WRITE
				TRACE("crsfSdWriteHandler:CRSF_SD_START:sync:%x:length:%d:cmd:%x:subcmd:%d:flag:%d:chunk:%ld:requestDataLength:%ld:crc:%d:calCrc:%d", CrsfSdOpentx.sync, CrsfSdOpentx.length, CrsfSdOpentx.cmd, CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.chunk, CrsfSdOpentx.requestDataLength, CrsfSdOpentx.crc, CrsfSdOpentx.calCrc);
				#endif

				if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == LIBCRSF_OPENTX_RELATED && CrsfSdOpentx.flag == CRSF_SD_FLAG_START){
					targetDst = CrsfSdOpentx.org;
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_START:start to write");
					TRACE("crsfSdWriteHandler:CRSF_SD_START:file:%s", CrsfSdOpentx.payload);
					#endif

					CrsfSdOpentx.result = f_open(&CrsfSdOpentx.file, (char*)CrsfSdOpentx.payload, FA_READ);
					delay_us(CRSF_SD_DELAY);
					CrsfSdOpentx.result = f_lseek(&CrsfSdOpentx.file, (uint32_t)CrsfSdOpentx.chunk);
					delay_us(CRSF_SD_DELAY);
					if( CrsfSdOpentx.result == FR_OK ){
						#if defined(DEBUG_CRSF_SD_WRITE) || defined(DEBUG_CRSF_SD_WRITE_COMPARE)
						TRACE("crsfSdWriteHandler:CRSF_SD_START:result:%d", CrsfSdOpentx.result);
						TRACE("crsfSdWriteHandler:CRSF_SD_START:open %s successful", CrsfSdOpentx.payload);
						#endif
						CrsfSdOpentx.result = f_stat((char*)CrsfSdOpentx.payload, &CrsfSdOpentx.fileInfo);
						delay_us(CRSF_SD_DELAY);
						if( CrsfSdOpentx.result == FR_OK ){
							#if defined(DEBUG_CRSF_SD_WRITE) || defined(DEBUG_CRSF_SD_WRITE_COMPARE)
							TRACE("crsfSdWriteHandler:CRSF_SD_START:get info successful:fsize:%d", CrsfSdOpentx.fileInfo.fsize);
							#endif
							CrsfSdOpentx.bytesPendingToSend = CrsfSdOpentx.requestDataLength;
							state = CRSF_SD_WRITE;
						}
						else{
							#ifdef DEBUG_CRSF_SD_WRITE
							TRACE("crsfSdWriteHandler:CRSF_SD_START:get file state failed");
							#endif
							state = CRSF_SD_IDLE;
						}
					}
					else{
						#ifdef DEBUG_CRSF_SD_WRITE
						TRACE("crsfSdWriteHandler:CRSF_SD_START:result:%x", CrsfSdOpentx.result);
						TRACE("crsfSdWriteHandler:CRSF_SD_START:open failed");
						#endif
						state = CRSF_SD_IDLE;
					}
				}
				else{
					CrsfSdOpentx.dst = CrsfSdOpentx.org;
					CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
					CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_READ;
					CrsfSdOpentx.flag = CRSF_SD_FLAG_RETRY_START;
					CrsfSdOpentx.requestDataLength = 0;
					CrsfSdOpentx.numOfBytesRead = 0;
					crsfSdPackFrame(&CrsfSdOpentx);
					CRSF_to_Shared_FIFO(CrsfSdOpentx.txBuf);
					state = CRSF_SD_START;
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_START:retry to write start");
					#endif
				}
			}
			break;
		}

		case CRSF_SD_WRITE:
		{
			#ifdef DEBUG_CRSF_SD_WRITE
			TRACE("crsfSdWriteHandler:CRSF_SD_WRITE:subcmd:%d:flag:%d:bytesPendingToSend:%d:chunk:%ld:numOfBytesRead:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk, CrsfSdOpentx.numOfBytesRead);
			#endif
			if(CrsfSdOpentx.bytesPendingToSend <= 0 && CrsfSdOpentx.flag == CRSF_SD_FLAG_FINISH){
				state = CRSF_SD_FINISH;
				CrsfSdOpentx.result = f_close(&CrsfSdOpentx.file);
				delay_us(CRSF_SD_DELAY);
			}
			else if((CrsfSdOpentx.bytesPendingToSend > 0  && (CrsfSdOpentx.flag == CRSF_SD_FLAG_START || CrsfSdOpentx.flag == CRSF_SD_FLAG_ACK) && crossfireSharedData.crsf_rx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE))){
				CrsfSdOpentx.result = f_read(&CrsfSdOpentx.file, CrsfSdOpentx.payload, CrsfSdOpentx.requestDataLength, (UINT*)&CrsfSdOpentx.numOfBytesRead);
				delay_us(CRSF_SD_DELAY);
				if( CrsfSdOpentx.result == FR_OK ){
					CrsfSdOpentx.chunk = CrsfSdOpentx.bytesPendingToSend / CrsfSdOpentx.numOfBytesRead;
					if(CrsfSdOpentx.chunk == 1 && (CrsfSdOpentx.bytesPendingToSend - CrsfSdOpentx.numOfBytesRead == 0)){
						CrsfSdOpentx.chunk = 0;
					}
					CrsfSdOpentx.dst = targetDst;
					CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
					CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_READ;
					CrsfSdOpentx.flag = CRSF_SD_FLAG_OK;
					crsfSdPackFrame(&CrsfSdOpentx);
					CRSF_to_Shared_FIFO(CrsfSdOpentx.txBuf);
					state = CRSF_SD_ACK;
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_WRITE:subcmd:%d:flag:%d:bytesPendingToSend:%d:chunk:%ld:numOfBytesRead:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk, CrsfSdOpentx.numOfBytesRead);
					#endif
				}
			}
			else if(CrsfSdOpentx.flag == CRSF_SD_FLAG_RETRY && crossfireSharedData.crsf_rx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdOpentx.dst = targetDst;
				CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
				CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_READ;
				CrsfSdOpentx.flag = CRSF_SD_FLAG_RETRY;
				crsfSdPackFrame(&CrsfSdOpentx);
				CRSF_to_Shared_FIFO(CrsfSdOpentx.txBuf);
				state = CRSF_SD_ACK;
				#ifdef DEBUG_CRSF_SD_WRITE
				TRACE("crsfSdWriteHandler:CRSF_SD_WRITE:subcmd:%d:flag:%d:bytesPendingToSend:%d:chunk:%ld", CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk);
				#endif
			}
			break;
		}

		case CRSF_SD_ACK:
		{
			if(isOpentxBufSet){
				isOpentxBufSet = 0;
				memcpy(CrsfSdOpentx.rxBuf, OpentxBuf, LIBCRSF_MAX_BUFFER_SIZE);
				crsfSdUnpackFrame(&CrsfSdOpentx);
				#ifdef DEBUG_CRSF_SD_WRITE
				TRACE("crsfSdWriteHandler:CRSF_SD_ACK:subcmd:%d:flag:%d:bytesPendingToSend:%d:chunk:%ld:numOfBytesRead:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk, CrsfSdOpentx.numOfBytesRead);
				#endif
				if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == LIBCRSF_OPENTX_RELATED && (CrsfSdOpentx.flag == CRSF_SD_FLAG_ACK || CrsfSdOpentx.flag == CRSF_SD_FLAG_FINISH)){
					CrsfSdOpentx.bytesPendingToSend -= CrsfSdOpentx.numOfBytesRead;
					state = CRSF_SD_WRITE;
					#if defined(DEBUG_CRSF_SD_WRITE)
					TRACE("crsfSdWriteHandler:CRSF_SD_ACK:subcmd:%d:flag:%d:bytesPendingToSend:%d:chunk:%ld", CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk);
					#endif
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_ACK:success to write");
					#endif
				}
				else{
					CrsfSdOpentx.flag = CRSF_SD_FLAG_RETRY;
					state = CRSF_SD_WRITE;
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_ACK:retry to write");
					#endif
				}
			}
			break;
		}

		case CRSF_SD_FINISH:
		{
			#if defined(DEBUG_CRSF_SD_WRITE) || defined(DEBUG_CRSF_SD_WRITE_COMPARE)
			TRACE("crsfSdWriteHandler:CRSF_SD_FINISH");
			#endif
			state = CRSF_SD_START;
			enableOpentxSdWriteHandler = 0;
			break;
		}

		case CRSF_SD_ERROR:
		{
			#ifdef DEBUG_CRSF_SD_WRITE
			TRACE("crsfSdWriteHandler:CRSF_SD_ERROR:%d", CrsfSdOpentx.result);
			#endif
			f_close(&CrsfSdOpentx.file);
			state = CRSF_SD_IDLE;

			break;
		}

		case CRSF_SD_IDLE:
		{
			state = CRSF_SD_START;
			enableOpentxSdWriteHandler = 0;
			break;
		}

		default:
		{
			break;
		}
	}

    if(state == prevState){
        if(stateRepeatCount++ > 1000){
        	enableOpentxSdWriteHandler = 0;
            stateRepeatCount = 0;
            state = CRSF_SD_START;
            TRACE("OPENTX CRSF_SD_WRITE KILL\r\n");
        }
    }
    else{
        stateRepeatCount = 0;
    }
    prevState = state;
}

static void crsfSdReadHandler(){
	static CrsfSd_t CrsfSdOpentx;
	static uint8_t state = CRSF_SD_START;
	static uint8_t targetDst = 0;
	static uint16_t stateRepeatCount = 0;
	static uint8_t prevState = CRSF_SD_START;

	switch(state){
		case CRSF_SD_START:
		{
			if(isOpentxBufSet){
				isOpentxBufSet = 0;
				memcpy(CrsfSdOpentx.rxBuf, OpentxBuf, LIBCRSF_MAX_BUFFER_SIZE);
				crsfSdUnpackFrame(&CrsfSdOpentx);
				#ifdef DEBUG_CRSF_SD_READ
				TRACE("crsfSdReadHandler:CRSF_SD_START:sync:%x:length:%d:cmd:%x:subcmd:%d:flag:%d:chunk:%ld:requestDataLength:%d:crc:%d:calCrc:%d", CrsfSdOpentx.sync, CrsfSdOpentx.length, CrsfSdOpentx.cmd, CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.chunk, CrsfSdOpentx.requestDataLength, CrsfSdOpentx.crc, CrsfSdOpentx.calCrc);
				#endif
				if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == LIBCRSF_OPENTX_RELATED && (CrsfSdOpentx.flag == CRSF_SD_FLAG_START || CrsfSdOpentx.flag == CRSF_SD_FLAG_RETRY_START)){
					CrsfSdOpentx.result = f_open(&CrsfSdOpentx.file, (const char*)CrsfSdOpentx.payload, FA_OPEN_ALWAYS | FA_WRITE);
					delay_us(CRSF_SD_DELAY);
					#if defined(DEBUG_CRSF_SD_READ) || defined(DEBUG_CRSF_SD_READ_COMPARE)
					TRACE("crsfSdReadHandler:CRSF_SD_START:open:%s", CrsfSdOpentx.payload);
					TRACE("crsfSdReadHandler:CRSF_SD_START:result:%d", CrsfSdOpentx.result);
					#endif
					if(CrsfSdOpentx.result == FR_OK){
						CrsfSdOpentx.result = f_lseek(&CrsfSdOpentx.file, (uint32_t)CrsfSdOpentx.requestDataLength);
						delay_us(CRSF_SD_DELAY);
						#if defined(DEBUG_CRSF_SD_READ) || defined(DEBUG_CRSF_SD_READ_COMPARE)
						TRACE("crsfSdReadHandler:CRSF_SD_START:offset:%d", (uint32_t)CrsfSdOpentx.requestDataLength);
						TRACE("crsfSdReadHandler:CRSF_SD_START:result:%d", CrsfSdOpentx.result);
						#endif
						if(CrsfSdOpentx.result == FR_OK){
							targetDst = CrsfSdOpentx.org;
							state = CRSF_SD_ACK;
						}
						else{
							state = CRSF_SD_IDLE;
						}
					}
					else{
						state = CRSF_SD_IDLE;
					}
				}
				else {
					if(crossfireSharedData.crsf_rx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
						CrsfSdOpentx.dst = CrsfSdOpentx.org;
						CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
						CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_WRITE;
						CrsfSdOpentx.flag = CRSF_SD_FLAG_RETRY_START;
						CrsfSdOpentx.numOfBytesRead = 0;
						crsfSdPackFrame(&CrsfSdOpentx);
						CRSF_to_Shared_FIFO(CrsfSdOpentx.txBuf);
						#ifdef DEBUG_CRSF_SD_READ
						TRACE("crsfSdReadHandler:CRSF_SD_START:crossfireSharedData.crsf_rx:%d", crossfireSharedData.crsf_rx.size());
						TraceState("crsfSdReadHandler:CRSF_SD_START:state:", state);
						#endif
					}
				}
			}
			break;
		}
		case CRSF_SD_READ:
		{
			if(isOpentxBufSet){
				isOpentxBufSet = 0;
				uint32_t prevChunk = CrsfSdOpentx.chunk;
				uint8_t prevLength = CrsfSdOpentx.length;
				memcpy(CrsfSdOpentx.rxBuf, OpentxBuf, LIBCRSF_MAX_BUFFER_SIZE);
				crsfSdUnpackFrame(&CrsfSdOpentx);
				#ifdef DEBUG_CRSF_SD_READ
				TRACE("crsfSdReadHandler:CRSF_SD_READ:sync:%x:length:%d:cmd:%x:subcmd:%d:flag:%d:chunk:%ld:crc:%d:calCrc:%d", CrsfSdOpentx.sync, CrsfSdOpentx.length, CrsfSdOpentx.cmd, CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.chunk, CrsfSdOpentx.crc, CrsfSdOpentx.calCrc);
				#endif
				if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == LIBCRSF_OPENTX_RELATED && (CrsfSdOpentx.flag == CRSF_SD_FLAG_OK || CrsfSdOpentx.flag == CRSF_SD_FLAG_RETRY)){// && CrsfSdOpentx.dst == LIBCRSF_RC_TX){
					if(prevChunk != CrsfSdOpentx.chunk || (prevChunk == CrsfSdOpentx.chunk && prevLength != CrsfSdOpentx.length)){
						uint16_t actualReceviedLength = CrsfSdOpentx.length - CRSF_SD_DATA_START_ADD + 1;
						CrsfSdOpentx.bytesReceived += actualReceviedLength;
						CrsfSdOpentx.result = f_write(&CrsfSdOpentx.file, CrsfSdOpentx.payload, actualReceviedLength, (UINT*)&CrsfSdOpentx.numOfBytesWritten);
						delay_us(CRSF_SD_WRITE_DELAY);
						#ifdef DEBUG_CRSF_SD_READ
						TRACE("crsfSdReadHandler:CRSF_SD_READ:result:%d:numOfBytesWritten:%d", CrsfSdOpentx.result, CrsfSdOpentx.numOfBytesWritten);
						TRACE("crsfSdReadHandler:CRSF_SD_READ:written in buffer");
						#endif
					}
					#if defined(DEBUG_CRSF_SD_READ)
					TRACE("crsfSdReadHandler:CRSF_SD_READ:subcmd:%d:flag:%d:bytesReceived:%d:chunk:%ld:numOfBytesWritten:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.bytesReceived, CrsfSdOpentx.chunk, CrsfSdOpentx.numOfBytesWritten);
					#endif
					state = CRSF_SD_DATA_READY;
				}
				else if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == LIBCRSF_OPENTX_RELATED && CrsfSdOpentx.flag == CRSF_SD_FLAG_RETRY_START){// && CrsfSdOpentx.dst == LIBCRSF_RC_TX){
					state = CRSF_SD_START;
				}
				else{
					state = CRSF_SD_ERROR;
				}
			}
			break;
		}

		case CRSF_SD_ACK:
		{
			if(crossfireSharedData.crsf_rx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdOpentx.dst = targetDst;
				CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
				CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_WRITE;
				if(CrsfSdOpentx.chunk > 0){
					CrsfSdOpentx.flag = CRSF_SD_FLAG_ACK;
					state = CRSF_SD_READ;
				}
				else{
					CrsfSdOpentx.flag = CRSF_SD_FLAG_FINISH;
					state = CRSF_SD_FINISH;
				}
				CrsfSdOpentx.requestDataLength = 0;
				CrsfSdOpentx.numOfBytesRead = 0;
				crsfSdPackFrame(&CrsfSdOpentx);
				CRSF_to_Shared_FIFO(CrsfSdOpentx.txBuf);
				#if defined(DEBUG_CRSF_SD_READ)
				TRACE("crsfSdReadHandler:CRSF_SD_ACK:subcmd:%d:flag:%d:bytesReceived:%d:chunk:%ld", CrsfSdOpentx.subcmd, CrsfSdOpentx.flag, CrsfSdOpentx.bytesReceived, CrsfSdOpentx.chunk);
				TRACE("crsfSdReadHandler:CRSF_SD_ACK:ack sent");
				#endif
			}
			break;
		}

		case CRSF_SD_FINISH:
		{
			CrsfSdOpentx.result = f_sync(&CrsfSdOpentx.file);
			delay_us(CRSF_SD_DELAY);
			CrsfSdOpentx.result = f_close(&CrsfSdOpentx.file);
			delay_us(CRSF_SD_DELAY);
			#if defined(DEBUG_CRSF_SD_READ) || defined(DEBUG_CRSF_SD_READ_COMPARE)
			TRACE("crsfSdReadHandler:CRSF_SD_FINISH:result:%d", CrsfSdOpentx.result);
			TRACE("crsfSdReadHandler:CRSF_SD_FINISH");
			#endif
			state = CRSF_SD_START;
			enableOpentxSdReadHandler = 0;
			break;
		}

		case CRSF_SD_DATA_READY:
		{
			#ifdef DEBUG_CRSF_SD_READ
			TRACE("crsfSdReadHandler:CRSF_SD_DATA_READY");
			#endif
			state = CRSF_SD_ACK;
			break;
		}

		case CRSF_SD_ERROR:
		{
			#ifdef DEBUG_CRSF_SD_READ
			TRACE("crsfSdReadHandler:CRSF_SD_ERROR");
			#endif
			if(crossfireSharedData.crsf_rx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdOpentx.dst = targetDst;
				CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
				CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_WRITE;
				CrsfSdOpentx.flag = CRSF_SD_FLAG_RETRY;
				state = CRSF_SD_READ;
				CrsfSdOpentx.numOfBytesRead = 0;
				crsfSdPackFrame(&CrsfSdOpentx);
				CRSF_to_Shared_FIFO(CrsfSdOpentx.txBuf);
				#ifdef DEBUG_CRSF_SD_READ
				TraceState("crsfSdReadHandler:CRSF_SD_ERROR:state:", state);
				#endif
			}
			break;
		}

		case CRSF_SD_INVALID_FILENAME:
		{
			state = CRSF_SD_IDLE;

			break;
		}

		case CRSF_SD_IDLE:
		{
			state = CRSF_SD_START;
			enableOpentxSdReadHandler = 0;
			break;
		}

		default:
		{
			break;
		}
	}

    if(state == prevState){
        if(stateRepeatCount++ > 1000){
        	enableOpentxSdReadHandler = 0;
            stateRepeatCount = 0;
            state = CRSF_SD_START;
            TRACE("OPENTX CRSF_SD_READ KILL\r\n");
        }
    }
    else{
        stateRepeatCount = 0;
    }
    prevState = state;
}

void crsfSdHandler() {

	crsfSharedFifoHandler();

    if(enableOpentxSdWriteHandler){
//          TRACE("OPENTX CRSF_SD_WRITE\r\n");
	  crsfSdWriteHandler();
    }

    if(enableOpentxSdReadHandler){
//          TRACE("OPENTX CRSF_SD_READ\r\n");
	  crsfSdReadHandler();
    }

    if(enableOpentxSdEraseHandler){
//          TRACE("OPENTX CRSF_SD_ERASE\r\n");
	  crsfSdEraseHandler();
    }
}

#endif // CRSF_OPENTX && CRSF_SD

/* ****************************************************************************************************************** */
/* TODO: perna remove the code below since we will strictly use CRSF  */
static uint8_t ToSendDataBuffer[64];
static uint8_t AgentCrc;


void WR_TO_USB_Writebuff ( uint8_t Position, uint8_t Data_In )
{
  ToSendDataBuffer[Position] = Data_In;
  libCRC8_Calc(Data_In, &AgentCrc, POLYNOM_1);
}


void USB_WRITE_SIMPLE_CMD( uint8_t COMMAND )
{
  libCRC8_Reset( &AgentCrc );
  WR_TO_USB_Writebuff(0, '\n');
  WR_TO_USB_Writebuff(1, COMMAND);
  ToSendDataBuffer[2] = Get_libCRC8( &AgentCrc, POLYNOM_1 );
  usbAgentWrite(ToSendDataBuffer);
}


void USB_Send_Serial_Number()
{
  libCRC8_Reset( &AgentCrc );
  WR_TO_USB_Writebuff(0, '\n');
  WR_TO_USB_Writebuff(1, 30);

  WR_TO_USB_Writebuff(2, 0);
  WR_TO_USB_Writebuff(3, 0);
  WR_TO_USB_Writebuff(4, 0);
  WR_TO_USB_Writebuff(5, 1);

  ToSendDataBuffer[2] = Get_libCRC8( &AgentCrc, POLYNOM_1 );
  usbAgentWrite(ToSendDataBuffer);
}


void AgentLegacyCalls( uint8_t *pArr )
{
  pArr++;
  switch( *pArr )
  {
    case 2: //USB_DEVICE_INFO_REQUEST:$
      libCRC8_Reset( &AgentCrc );
      WR_TO_USB_Writebuff(0, '\n');
      WR_TO_USB_Writebuff(1, 3);
      WR_TO_USB_Writebuff(2, 2);

      WR_TO_USB_Writebuff(3, 0x00);
      WR_TO_USB_Writebuff(4, 0x04);
      WR_TO_USB_Writebuff(5, 0x00);
      WR_TO_USB_Writebuff(6, 0x00);
      WR_TO_USB_Writebuff(7, 0x01);
      WR_TO_USB_Writebuff(8, 0x00);

      WR_TO_USB_Writebuff(9, 8);
      WR_TO_USB_Writebuff(10, 'T');
      WR_TO_USB_Writebuff(11, 'a');
      WR_TO_USB_Writebuff(12, 'n');
      WR_TO_USB_Writebuff(13, 'g');
      WR_TO_USB_Writebuff(14, 'o');
      WR_TO_USB_Writebuff(15, ' ');
      WR_TO_USB_Writebuff(16, 'I');
      WR_TO_USB_Writebuff(17, 'I');

      ToSendDataBuffer[2] = Get_libCRC8( &AgentCrc, POLYNOM_1 );
      usbAgentWrite(ToSendDataBuffer);
      break;

    case 6:  // USB_MUX_CMDS_TO_SUB_CPU:
      USB_WRITE_SIMPLE_CMD( 7 );
      break;

    case 18: //USB_SERIAL_NUMBER_REQUEST:
      USB_Send_Serial_Number();
      break;

  }
}
