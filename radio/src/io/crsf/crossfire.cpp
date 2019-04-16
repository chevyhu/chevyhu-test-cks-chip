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
static uint8_t libCrsf_MySlaveAddress = 0xEA;
static char *libCrsf_MyDeviceName = "TBS TANGO II";
static uint32_t libCrsf_MySerialNo = 6;
static uint32_t libCrsf_MyHwID = 0x040001;
static uint32_t libCrsf_MyFwID = 0x0100;

Fifo<uint8_t, TELEMETRY_BUFFER_SIZE> crsf_telemetry_buffer;

extern Fifo<uint8_t, ESP_TX_BUFFER_SIZE> espTxFifo;
extern Fifo<uint8_t, ESP_RX_BUFFER_SIZE> espRxFifo;

void CRSF_Init( void )
{
  /* init crsf library */

  memset( &crossfireSharedData, 0, sizeof(CrossfireSharedData) );

  libCrsf_Init( libCrsf_MySlaveAddress, libCrsf_MyDeviceName, libCrsf_MySerialNo, libCrsf_MyHwID, libCrsf_MyFwID );

  libCrsf_CRSF_Add_Device_Function_List( &CRSF_Ports[0], ARRAY_SIZE(CRSF_Ports));

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
  if (telemetryProtocol == PROTOCOL_PULSES_CROSSFIRE) //henry: what is the protocol?
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
//	  	TRACE("sharedtx:%x", *(p_arr + i));
    crossfireSharedData.crsf_rx.push(*(p_arr + i));
//    TRACE("%X", *(p_arr + i));
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
  if ( crossfireSharedData.crsf_tx.pop(byte) ) {
    if ( libCrsf_CRSF_Parse( &CRSF_Data, byte )) {
      libCrsf_CRSF_Routing( CRSF_SHARED_FIFO, CRSF_Data.Payload );
    }
  }
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

#ifdef CRSF_SD
/*
static void crsfSdPackFrame(CrsfSd_t* crsfSd){
	uint32_t count = 0;
	libUtil_Write8(crsfSd->txBuf, &count, LIBCRSF_UART_SYNC);
	libUtil_Write8(crsfSd->txBuf, &count, 0);
	libUtil_Write8(crsfSd->txBuf, &count, 0x3a);
	libUtil_Write8(crsfSd->txBuf, &count, crsfSd->dst);
	libUtil_Write8(crsfSd->txBuf, &count, crsfSd->org);
	libUtil_Write8(crsfSd->txBuf, &count, crsfSd->subcmd);
	libUtil_Write16(crsfSd->txBuf, &count, crsfSd->chunk);
	libUtil_Write16(crsfSd->txBuf, &count, crsfSd->requestDataLength);
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
	crsfSd->chunk = libUtil_Read16(crsfSd->rxBuf, &count);
	crsfSd->requestDataLength = libUtil_Read16(crsfSd->rxBuf, &count);
	for(uint8_t i = 0; i < crsfSd->length - 9; i++){
		crsfSd->payload[i] = libUtil_Read8(crsfSd->rxBuf, &count);
	}
	crsfSd->crc = libUtil_Read8(crsfSd->rxBuf, &count);
	crsfSd->calCrc = libCRC8_Get_CRC_Arr(&crsfSd->rxBuf[2], crsfSd->length-1, POLYNOM_1);
}

static void crsfSdReadHandler( char* filename, BYTE* pData, uint16_t Length, uint8_t& state ){
	static CrsfSd_t CrsfSdXf;
	switch(state){
		case CRSF_SD_START:
		{
			if(Length > 0 && crossfireSharedData.crsf_tx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdXf.dst = LIBCRSF_REMOTE_ADD;
				CrsfSdXf.org = LIBCRSF_RC_TX;
				CrsfSdXf.subcmd = CRSF_SD_SUBCMD_START;
				CrsfSdXf.requestDataLength = Length;
				uint8_t filenameLength = 0;
				for(int i = 0; i < 128; i++){
					filenameLength++;
					if(filename[i] == 0){
						break;
					}
					if(i == 127){
						state = CRSF_SD_INVALID_FILENAME;
						return;
					}
				}
				memcpy(CrsfSdXf.payload, filename, filenameLength);
				CrsfSdXf.numOfBytesRead = filenameLength;
				crsfSdPackFrame(&CrsfSdXf);
				for(uint8_t i = 0; i < CrsfSdXf.txBuf[LIBCRSF_LENGTH_ADD] + 2; i++){
					crossfireSharedData.crsf_tx.push(CrsfSdXf.txBuf[i]);
					#ifdef DEBUG_CRSF_SD_READ
					TRACE("crsfSdReadHandler:CRSF_SD_START:txBuf[%d]:%x", i, CrsfSdXf.txBuf[i]);
					#endif
				}
				#ifdef DEBUG_CRSF_SD_READ
				TRACE("crsfSdReadHandler:CRSF_SD_START:start sent");
				#endif
				state = CRSF_SD_READ;
				#if defined(DEBUG_CRSF_SD_READ) || defined(DEBUG_CRSF_SD_READ_COMPARE)
				CrsfSdXf.result = f_open(&CrsfSdXf.file, "/comparison.bin", FA_CREATE_ALWAYS | FA_WRITE);
				TRACE("crsfSdReadHandler:CRSF_SD_START:result:%d", CrsfSdXf.result);
				#endif
			}
			break;
		}
		case CRSF_SD_READ:
		{
			if(crossfireSharedData.crsf_rx.size() > 0){
				for(uint8_t i = 0; i < LIBCRSF_MAX_BUFFER_SIZE; i++){
					if(!crossfireSharedData.crsf_rx.pop(CrsfSdXf.rxBuf[i])){
						break;
					}
					#ifdef DEBUG_CRSF_SD_READ
					TRACE("crsfSdReadHandler:CRSF_SD_READ:rxbuf[%d]:%x", i, CrsfSdXf.rxBuf[i]);
					#endif
				}
				uint16_t prevChunk = CrsfSdXf.chunk;
				uint8_t prevLength = CrsfSdXf.length;
				crsfSdUnpackFrame(&CrsfSdXf);
				#ifdef DEBUG_CRSF_SD_READ
				TRACE("crsfSdReadHandler:CRSF_SD_READ:sync:%x:length:%d:cmd:%x:subcmd:%d:chunk:%d:crc:%d:calCrc:%d", CrsfSdXf.sync, CrsfSdXf.length, CrsfSdXf.cmd, CrsfSdXf.subcmd, CrsfSdXf.chunk, CrsfSdXf.crc, CrsfSdXf.calCrc);
				#endif
				if(CrsfSdXf.sync == LIBCRSF_UART_SYNC && CrsfSdXf.crc == CrsfSdXf.calCrc && CrsfSdXf.cmd == 0x3a && (CrsfSdXf.subcmd == CRSF_SD_SUBCMD_OK || CrsfSdXf.subcmd == CRSF_SD_SUBCMD_RETRY) && CrsfSdXf.dst == LIBCRSF_RC_TX){
					if(prevChunk != CrsfSdXf.chunk || (prevChunk == CrsfSdXf.chunk && prevLength != CrsfSdXf.length)){
						uint16_t actualReceviedLength = CrsfSdXf.length - CRSF_SD_DATA_START_ADD + 1;
						memcpy(pData, CrsfSdXf.payload, actualReceviedLength);
						CrsfSdXf.bytesReceived += actualReceviedLength;
						#if defined(DEBUG_CRSF_SD_READ) || defined(DEBUG_CRSF_SD_READ_COMPARE)
						CrsfSdXf.result = f_write(&CrsfSdXf.file, CrsfSdXf.payload, actualReceviedLength, (UINT*)&CrsfSdXf.numOfBytesWritten);
						#endif
						#ifdef DEBUG_CRSF_SD_READ
						TRACE("crsfSdReadHandler:CRSF_SD_READ:result:%d:numOfBytesWritten:%d", CrsfSdXf.result, CrsfSdXf.numOfBytesWritten);
						TRACE("crsfSdReadHandler:CRSF_SD_READ:written in buffer");
						#endif
					}
					#if defined(DEBUG_CRSF_SD_READ) // || defined(DEBUG_CRSF_SD_READ_COMPARE)
					TRACE("crsfSdReadHandler:CRSF_SD_READ:subcmd:%d:bytesReceived:%d:chunk:%d:numOfBytesWritten:%d", CrsfSdXf.subcmd, CrsfSdXf.bytesReceived, CrsfSdXf.chunk, CrsfSdXf.numOfBytesWritten);
					#endif
					state = CRSF_SD_DATA_READY;

				}
				else if(CrsfSdXf.sync == LIBCRSF_UART_SYNC && CrsfSdXf.crc == CrsfSdXf.calCrc && CrsfSdXf.cmd == 0x3a && CrsfSdXf.subcmd == CRSF_SD_SUBCMD_RETRY_START && CrsfSdXf.dst == LIBCRSF_RC_TX){
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
			if(crossfireSharedData.crsf_tx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdXf.dst = LIBCRSF_REMOTE_ADD;
				CrsfSdXf.org = LIBCRSF_RC_TX;
				if(CrsfSdXf.chunk > 0){
					CrsfSdXf.subcmd = CRSF_SD_SUBCMD_ACK;
					state = CRSF_SD_READ;
				}
				else if(CrsfSdXf.chunk == 0){
					state = CRSF_SD_FINISH;
					CrsfSdXf.subcmd = CRSF_SD_SUBCMD_FINISH;
				}
				CrsfSdXf.requestDataLength = Length;
				CrsfSdXf.numOfBytesRead = 0;
				crsfSdPackFrame(&CrsfSdXf);
				for(uint8_t i = 0; i < CrsfSdXf.txBuf[LIBCRSF_LENGTH_ADD] + 2; i++){
					crossfireSharedData.crsf_tx.push(CrsfSdXf.txBuf[i]);
					#ifdef DEBUG_CRSF_SD_READ
					TRACE("crsfSdReadHandler:CRSF_SD_ACK:txBuf[%d]:%x", i, CrsfSdXf.txBuf[i]);
					#endif
				}
				#if defined(DEBUG_CRSF_SD_READ) // || defined(DEBUG_CRSF_SD_READ_COMPARE)
				TRACE("crsfSdReadHandler:CRSF_SD_ACK:subcmd:%d:bytesReceived:%d:chunk:%d", CrsfSdXf.subcmd, CrsfSdXf.bytesReceived, CrsfSdXf.chunk);
				TRACE("crsfSdReadHandler:CRSF_SD_ACK:ack sent");
				#endif
			}
			break;
		}

		case CRSF_SD_FINISH:
		{
			#if defined(DEBUG_CRSF_SD_READ) || defined(DEBUG_CRSF_SD_READ_COMPARE)
			CrsfSdXf.result = f_sync(&CrsfSdXf.file);
			TRACE("crsfSdReadHandler:CRSF_SD_FINISH:result:%d", CrsfSdXf.result);
			CrsfSdXf.result = f_close(&CrsfSdXf.file);
			TRACE("crsfSdReadHandler:CRSF_SD_FINISH:result:%d", CrsfSdXf.result);
			TRACE("crsfSdReadHandler:CRSF_SD_FINISH");
			#endif
			state = CRSF_SD_IDLE;
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
			if(crossfireSharedData.crsf_tx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdXf.dst = LIBCRSF_REMOTE_ADD;
				CrsfSdXf.org = LIBCRSF_RC_TX;
				CrsfSdXf.subcmd = CRSF_SD_SUBCMD_RETRY;
				CrsfSdXf.numOfBytesRead = 0;
				crsfSdPackFrame(&CrsfSdXf);
				for(uint8_t i = 0; i < CrsfSdXf.txBuf[LIBCRSF_LENGTH_ADD] + 2; i++){
					crossfireSharedData.crsf_tx.push(CrsfSdXf.txBuf[i]);
					#ifdef DEBUG_CRSF_SD_READ
					TRACE("crsfSdReadHandler:CRSF_SD_ACK:txBuf[%d]:%x", i, CrsfSdXf.txBuf[i]);
					#endif
				}
				state = CRSF_SD_ACK;
			}
			break;
		}

		case CRSF_SD_INVALID_FILENAME:
		{
			state = CRSF_SD_IDLE;
			break;
		}

		default:
		{
			break;
		}
	}
}

void crsfSdWriteHandler(){
	static CrsfSd_t CrsfSdOpentx;
	static uint8_t state = CRSF_SD_START;
	switch(state){
		case CRSF_SD_START:
		{
			if(crossfireSharedData.crsf_tx.size() > 0){
				for(uint8_t i = 0; i < LIBCRSF_MAX_BUFFER_SIZE; i++){
					if(!crossfireSharedData.crsf_tx.pop(CrsfSdOpentx.rxBuf[i])){
						break;
					}
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_START:rxbuf[%d]:%x", i, CrsfSdOpentx.rxBuf[i]);
					#endif
				}
				crsfSdUnpackFrame(&CrsfSdOpentx);

				#ifdef DEBUG_CRSF_SD_WRITE
				TRACE("crsfSdWriteHandler:CRSF_SD_START:sync:%x:length:%d:cmd:%x:subcmd:%d:chunk:%d:crc:%d:calCrc:%d", CrsfSdOpentx.sync, CrsfSdOpentx.length, CrsfSdOpentx.cmd, CrsfSdOpentx.subcmd, CrsfSdOpentx.chunk, CrsfSdOpentx.crc, CrsfSdOpentx.calCrc);
				#endif

				if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == 0X3a && CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_START){
					state = CRSF_SD_WRITE;
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_START:start to write");
					#endif

					CrsfSdOpentx.result = f_open(&CrsfSdOpentx.file, (char*)CrsfSdOpentx.payload, FA_READ);
					if( CrsfSdOpentx.result == FR_OK ){
						#if defined(DEBUG_CRSF_SD_WRITE) || defined(DEBUG_CRSF_SD_WRITE_COMPARE)
						TRACE("crsfSdWriteHandler:CRSF_SD_START:result:%d", CrsfSdOpentx.result);
						TRACE("crsfSdWriteHandler:CRSF_SD_START:open %s successful", CrsfSdOpentx.payload);
						#endif
						CrsfSdOpentx.result = f_stat((char*)CrsfSdOpentx.payload, &CrsfSdOpentx.fileInfo);
						if( CrsfSdOpentx.result == FR_OK ){
							#if defined(DEBUG_CRSF_SD_WRITE) || defined(DEBUG_CRSF_SD_WRITE_COMPARE)
							TRACE("crsfSdWriteHandler:CRSF_SD_START:get info successful:fsize:%d", CrsfSdOpentx.fileInfo.fsize);
							#endif
							CrsfSdOpentx.bytesPendingToSend = CrsfSdOpentx.fileInfo.fsize;
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
						TRACE("crsfSdWriteHandler:CRSF_SD_START:open failed");
						#endif
						state = CRSF_SD_IDLE;
					}
				}
				else{
					CrsfSdOpentx.dst = LIBCRSF_RC_TX;
					CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
					CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_RETRY_START;
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
			if(CrsfSdOpentx.bytesPendingToSend <= 0 && CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_FINISH){
				state = CRSF_SD_FINISH;
				CrsfSdOpentx.result = f_close(&CrsfSdOpentx.file);
			}
			else if((CrsfSdOpentx.bytesPendingToSend > 0  && CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_ACK && crossfireSharedData.crsf_rx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)) ||
					CrsfSdOpentx.bytesPendingToSend == CrsfSdOpentx.fileInfo.fsize){
				CrsfSdOpentx.result = f_read(&CrsfSdOpentx.file, CrsfSdOpentx.payload, CrsfSdOpentx.requestDataLength, (UINT*)&CrsfSdOpentx.numOfBytesRead);
				if( CrsfSdOpentx.result == FR_OK ){
					CrsfSdOpentx.chunk = CrsfSdOpentx.bytesPendingToSend / CrsfSdOpentx.numOfBytesRead;
					if(CrsfSdOpentx.chunk == 1 && (CrsfSdOpentx.bytesPendingToSend - CrsfSdOpentx.numOfBytesRead == 0)){
						CrsfSdOpentx.chunk = 0;
					}
					CrsfSdOpentx.dst = LIBCRSF_RC_TX;
					CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
					CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_OK;
					crsfSdPackFrame(&CrsfSdOpentx);
					CRSF_to_Shared_FIFO(CrsfSdOpentx.txBuf);
					state = CRSF_SD_ACK;
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_WRITE:subcmd:%d:bytesPendingToSend:%d:chunk:%d:numOfBytesRead:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk, CrsfSdOpentx.numOfBytesRead);
					#endif
				}
			}
			else if(CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_RETRY && crossfireSharedData.crsf_rx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdOpentx.dst = LIBCRSF_RC_TX;
				CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
				CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_RETRY;
				crsfSdPackFrame(&CrsfSdOpentx);
				CRSF_to_Shared_FIFO(CrsfSdOpentx.txBuf);
				state = CRSF_SD_ACK;
				#ifdef DEBUG_CRSF_SD_WRITE
				TRACE("crsfSdWriteHandler:CRSF_SD_WRITE:subcmd:%d:bytesPendingToSend:%d:chunk:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk);
				#endif
			}
			break;
		}

		case CRSF_SD_ACK:
		{
			if(crossfireSharedData.crsf_tx.size() > 0){
				for(uint8_t i = 0; i < LIBCRSF_MAX_BUFFER_SIZE; i++){
					if(!crossfireSharedData.crsf_tx.pop(CrsfSdOpentx.rxBuf[i])){
						break;
					}
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_ACK:rxbuf[%d]:%x", i, CrsfSdOpentx.rxBuf[i]);
					#endif
				}
				crsfSdUnpackFrame(&CrsfSdOpentx);
				if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == 0X3a && (CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_ACK || CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_FINISH)){
					CrsfSdOpentx.bytesPendingToSend -= CrsfSdOpentx.numOfBytesRead;
					state = CRSF_SD_WRITE;
					#if defined(DEBUG_CRSF_SD_WRITE) // || defined(DEBUG_CRSF_SD_WRITE_COMPARE)
					TRACE("crsfSdWriteHandler:CRSF_SD_ACK:subcmd:%d:bytesPendingToSend:%d:chunk:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk);
					#endif
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_ACK:success to write");
					#endif
				}
				else{
					CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_RETRY;
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
		default:
		{
			break;
		}
	}
}

uint8_t crsfSdRead( char* filename, BYTE *pData, uint16_t Length ){
	static uint8_t rxState = CRSF_SD_IDLE;
	static uint32_t bytesSent = 0;
	static uint16_t i = 0;
	static uint16_t count = 0;
	static uint16_t len = 0;
	if(rxState == CRSF_SD_IDLE){
		rxState = CRSF_SD_START;
	}
	if(bytesSent == 0 || bytesSent == Length){
		bytesSent = 0;
		i = 0;
		count = Length / CRSF_SD_DATA_MAX_BUFFER_SIZE;
		if(Length - count * CRSF_SD_DATA_MAX_BUFFER_SIZE > 0){
			count++;
		}
		len = Length / CRSF_SD_DATA_MAX_BUFFER_SIZE > 0 ? CRSF_SD_DATA_MAX_BUFFER_SIZE : Length;
	}
	if(i < count){
		if(rxState == CRSF_SD_FINISH || rxState == CRSF_SD_DATA_READY){
			if(Length - (count - 1) * CRSF_SD_DATA_MAX_BUFFER_SIZE > 0 && (i == count - 1)){
				len = Length - (count - 1) * CRSF_SD_DATA_MAX_BUFFER_SIZE;
			}
		}
		crsfSdReadHandler( filename, pData, len, rxState);
		if(rxState == CRSF_SD_FINISH || rxState == CRSF_SD_DATA_READY){
			pData += len;
			bytesSent += len;
			i++;
		}
	}

	if(i == count){
		return rxState;
	}
	else if(rxState == CRSF_SD_IDLE){
		return CRSF_SD_FINISH;
	}
	else{
		return CRSF_SD_BUSY;
	}
}*/



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

static void TraceSubCmd(char* prefix, uint8_t subcmd){
	switch(subcmd){
		case CRSF_SD_SUBCMD_OK:
			TRACE("%sCRSF_SD_SUBCMD_OK", prefix);
			break;
		case CRSF_SD_SUBCMD_START:
			TRACE("%sCRSF_SD_SUBCMD_START", prefix);
			break;
		case CRSF_SD_SUBCMD_FINISH:
			TRACE("%sCRSF_SD_SUBCMD_FINISH", prefix);
			break;
		case CRSF_SD_SUBCMD_ACK:
			TRACE("%sCRSF_SD_SUBCMD_ACK", prefix);
			break;
		case CRSF_SD_SUBCMD_ERROR:
			TRACE("%sCRSF_SD_SUBCMD_ERROR", prefix);
			break;
		case CRSF_SD_SUBCMD_RETRY:
			TRACE("%sCRSF_SD_SUBCMD_RETRY", prefix);
			break;
		case CRSF_SD_SUBCMD_RETRY_START:
			TRACE("%sCRSF_SD_SUBCMD_RETRY_START", prefix);
			break;

	}
}

static void crsfSdPackFrame(CrsfSd_t* crsfSd){
	uint32_t count = 0;
	libUtil_Write8(crsfSd->txBuf, &count, LIBCRSF_UART_SYNC);
	libUtil_Write8(crsfSd->txBuf, &count, 0);
	libUtil_Write8(crsfSd->txBuf, &count, 0x3a);
	libUtil_Write8(crsfSd->txBuf, &count, crsfSd->dst);
	libUtil_Write8(crsfSd->txBuf, &count, crsfSd->org);
	libUtil_Write8(crsfSd->txBuf, &count, crsfSd->subcmd);
	libUtil_Write16(crsfSd->txBuf, &count, crsfSd->chunk);
	libUtil_Write16(crsfSd->txBuf, &count, crsfSd->requestDataLength);
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
	crsfSd->chunk = libUtil_Read16(crsfSd->rxBuf, &count);
	crsfSd->requestDataLength = libUtil_Read16(crsfSd->rxBuf, &count);
	for(uint8_t i = 0; i < crsfSd->length - 9; i++){
		crsfSd->payload[i] = libUtil_Read8(crsfSd->rxBuf, &count);
	}
	crsfSd->crc = libUtil_Read8(crsfSd->rxBuf, &count);
	crsfSd->calCrc = libCRC8_Get_CRC_Arr(&crsfSd->rxBuf[2], crsfSd->length-1, POLYNOM_1);
}

static void crsfSdXfReadHandler( char* filename, BYTE* pData, uint16_t Length, uint8_t& state ){
	static CrsfSd_t CrsfSdXf;
	switch(state){
		case CRSF_SD_START:
		{
			if(Length > 0 && crossfireSharedData.crsf_tx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdXf.dst = LIBCRSF_REMOTE_ADD;
				CrsfSdXf.org = LIBCRSF_RC_TX;
				CrsfSdXf.subcmd = CRSF_SD_SUBCMD_START;
				CrsfSdXf.requestDataLength = Length;
				uint8_t filenameLength = 0;
				for(int i = 0; i < 128; i++){
					filenameLength++;
					if(filename[i] == 0){
						break;
					}
					if(i == 127){
						state = CRSF_SD_INVALID_FILENAME;
						return;
					}
				}
				memcpy(CrsfSdXf.payload, filename, filenameLength);
				CrsfSdXf.numOfBytesRead = filenameLength;
				crsfSdPackFrame(&CrsfSdXf);
				for(uint8_t i = 0; i < CrsfSdXf.txBuf[LIBCRSF_LENGTH_ADD] + 2; i++){
					crossfireSharedData.crsf_tx.push(CrsfSdXf.txBuf[i]);
					#ifdef DEBUG_CRSF_SD_XF_READ
					TRACE("crsfSdXfReadHandler:CRSF_SD_START:txBuf[%d]:%x", i, CrsfSdXf.txBuf[i]);
					#endif
				}
				#ifdef DEBUG_CRSF_SD_XF_READ
				TRACE("crsfSdXfReadHandler:CRSF_SD_START:start sent");
				#endif
				state = CRSF_SD_READ;
				#if defined(DEBUG_CRSF_SD_XF_READ) || defined(DEBUG_CRSF_SD_XF_READ_COMPARE)
				CrsfSdXf.result = f_open(&CrsfSdXf.file, "/comparison.bin", FA_CREATE_ALWAYS | FA_WRITE);
				TRACE("crsfSdXfReadHandler:CRSF_SD_START:result:%d", CrsfSdXf.result);
				#endif
			}
			break;
		}
		case CRSF_SD_READ:
		{
			if(crossfireSharedData.crsf_rx.size() > 0){
				for(uint8_t i = 0; i < LIBCRSF_MAX_BUFFER_SIZE; i++){
					if(!crossfireSharedData.crsf_rx.pop(CrsfSdXf.rxBuf[i])){
						break;
					}
					#ifdef DEBUG_CRSF_SD_XF_READ
					TRACE("crsfSdXfReadHandler:CRSF_SD_READ:rxbuf[%d]:%x", i, CrsfSdXf.rxBuf[i]);
					#endif
				}
				uint16_t prevChunk = CrsfSdXf.chunk;
				uint8_t prevLength = CrsfSdXf.length;
				crsfSdUnpackFrame(&CrsfSdXf);
				#ifdef DEBUG_CRSF_SD_XF_READ
				TRACE("crsfSdXfReadHandler:CRSF_SD_READ:sync:%x:length:%d:cmd:%x:subcmd:%d:chunk:%d:crc:%d:calCrc:%d", CrsfSdXf.sync, CrsfSdXf.length, CrsfSdXf.cmd, CrsfSdXf.subcmd, CrsfSdXf.chunk, CrsfSdXf.crc, CrsfSdXf.calCrc);
				TraceSubCmd("crsfSdXfReadHandler:CRSF_SD_READ:subcmd:", CrsfSdXf.subcmd);
				#endif
				if(CrsfSdXf.sync == LIBCRSF_UART_SYNC && CrsfSdXf.crc == CrsfSdXf.calCrc && CrsfSdXf.cmd == 0x3a && (CrsfSdXf.subcmd == CRSF_SD_SUBCMD_OK || CrsfSdXf.subcmd == CRSF_SD_SUBCMD_RETRY) && CrsfSdXf.dst == LIBCRSF_RC_TX){
					if(prevChunk != CrsfSdXf.chunk || (prevChunk == CrsfSdXf.chunk && prevLength != CrsfSdXf.length)){
						uint16_t actualReceviedLength = CrsfSdXf.length - CRSF_SD_DATA_START_ADD + 1;
						memcpy(pData, CrsfSdXf.payload, actualReceviedLength);
						CrsfSdXf.bytesReceived += actualReceviedLength;
						#if defined(DEBUG_CRSF_SD_XF_READ) || defined(DEBUG_CRSF_SD_XF_READ_COMPARE)
						CrsfSdXf.result = f_write(&CrsfSdXf.file, CrsfSdXf.payload, actualReceviedLength, (UINT*)&CrsfSdXf.numOfBytesWritten);
						#endif
						#ifdef DEBUG_CRSF_SD_XF_READ
						TRACE("crsfSdXfReadHandler:CRSF_SD_READ:result:%d:numOfBytesWritten:%d", CrsfSdXf.result, CrsfSdXf.numOfBytesWritten);
						TRACE("crsfSdXfReadHandler:CRSF_SD_READ:written in buffer");
						#endif
					}
					#if defined(DEBUG_CRSF_SD_XF_READ) // || defined(DEBUG_CRSF_SD_XF_READ_COMPARE)
					TRACE("crsfSdXfReadHandler:CRSF_SD_READ:subcmd:%d:bytesReceived:%d:chunk:%d:numOfBytesWritten:%d", CrsfSdXf.subcmd, CrsfSdXf.bytesReceived, CrsfSdXf.chunk, CrsfSdXf.numOfBytesWritten);
					TraceSubCmd("crsfSdXfReadHandler:CRSF_SD_READ:subcmd:", CrsfSdXf.subcmd);
					#endif
					state = CRSF_SD_DATA_READY;

				}
				else if(CrsfSdXf.sync == LIBCRSF_UART_SYNC && CrsfSdXf.crc == CrsfSdXf.calCrc && CrsfSdXf.cmd == 0x3a && CrsfSdXf.subcmd == CRSF_SD_SUBCMD_RETRY_START && CrsfSdXf.dst == LIBCRSF_RC_TX){
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
			if(crossfireSharedData.crsf_tx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdXf.dst = LIBCRSF_REMOTE_ADD;
				CrsfSdXf.org = LIBCRSF_RC_TX;
				if(CrsfSdXf.chunk > 0){
					CrsfSdXf.subcmd = CRSF_SD_SUBCMD_ACK;
					state = CRSF_SD_READ;
				}
				else if(CrsfSdXf.chunk == 0){
					state = CRSF_SD_FINISH;
					CrsfSdXf.subcmd = CRSF_SD_SUBCMD_FINISH;
				}
				CrsfSdXf.requestDataLength = Length;
				CrsfSdXf.numOfBytesRead = 0;
				crsfSdPackFrame(&CrsfSdXf);
				for(uint8_t i = 0; i < CrsfSdXf.txBuf[LIBCRSF_LENGTH_ADD] + 2; i++){
					crossfireSharedData.crsf_tx.push(CrsfSdXf.txBuf[i]);
					#ifdef DEBUG_CRSF_SD_XF_READ
					TRACE("crsfSdXfReadHandler:CRSF_SD_ACK:txBuf[%d]:%x", i, CrsfSdXf.txBuf[i]);
					#endif
				}
				#if defined(DEBUG_CRSF_SD_XF_READ) // || defined(DEBUG_CRSF_SD_XF_READ_COMPARE)
				TRACE("crsfSdXfReadHandler:CRSF_SD_ACK:subcmd:%d:bytesReceived:%d:chunk:%d", CrsfSdXf.subcmd, CrsfSdXf.bytesReceived, CrsfSdXf.chunk);
				TraceSubCmd("crsfSdXfReadHandler:CRSF_SD_ACK:subcmd:", CrsfSdXf.subcmd);
				TRACE("crsfSdXfReadHandler:CRSF_SD_ACK:ack sent");
				#endif
			}
			break;
		}

		case CRSF_SD_FINISH:
		{
			#if defined(DEBUG_CRSF_SD_XF_READ) || defined(DEBUG_CRSF_SD_XF_READ_COMPARE)
			CrsfSdXf.result = f_sync(&CrsfSdXf.file);
			TRACE("crsfSdXfReadHandler:CRSF_SD_FINISH:result:%d", CrsfSdXf.result);
			CrsfSdXf.result = f_close(&CrsfSdXf.file);
			TRACE("crsfSdXfReadHandler:CRSF_SD_FINISH:result:%d", CrsfSdXf.result);
			TRACE("crsfSdXfReadHandler:CRSF_SD_FINISH");
			#endif
			state = CRSF_SD_IDLE;
			break;
		}

		case CRSF_SD_DATA_READY:
		{
			#ifdef DEBUG_CRSF_SD_XF_READ
			TRACE("crsfSdXfReadHandler:CRSF_SD_DATA_READY");
			#endif
			state = CRSF_SD_ACK;
			break;
		}

		case CRSF_SD_ERROR:
		{
			#ifdef DEBUG_CRSF_SD_XF_READ
			TRACE("crsfSdXfReadHandler:CRSF_SD_ERROR");
			#endif
			if(crossfireSharedData.crsf_tx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdXf.dst = LIBCRSF_REMOTE_ADD;
				CrsfSdXf.org = LIBCRSF_RC_TX;
				CrsfSdXf.subcmd = CRSF_SD_SUBCMD_RETRY;
				CrsfSdXf.numOfBytesRead = 0;
				crsfSdPackFrame(&CrsfSdXf);
				for(uint8_t i = 0; i < CrsfSdXf.txBuf[LIBCRSF_LENGTH_ADD] + 2; i++){
					crossfireSharedData.crsf_tx.push(CrsfSdXf.txBuf[i]);
					#ifdef DEBUG_CRSF_SD_READ
					TRACE("crsfSdXfReadHandler:CRSF_SD_ACK:txBuf[%d]:%x", i, CrsfSdXf.txBuf[i]);
					#endif
				}
				state = CRSF_SD_ACK;
			}
			break;
		}

		case CRSF_SD_INVALID_FILENAME:
		{
			state = CRSF_SD_IDLE;
			break;
		}

		default:
		{
			break;
		}
	}
}

void crsfSdWriteHandler(){
	static CrsfSd_t CrsfSdOpentx;
	static uint8_t state = CRSF_SD_START;
	switch(state){
		case CRSF_SD_START:
		{
			if(crossfireSharedData.crsf_tx.size() > 0){
				for(uint8_t i = 0; i < LIBCRSF_MAX_BUFFER_SIZE; i++){
					if(!crossfireSharedData.crsf_tx.pop(CrsfSdOpentx.rxBuf[i])){
						break;
					}
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_START:rxbuf[%d]:%x", i, CrsfSdOpentx.rxBuf[i]);
					#endif
				}
				crsfSdUnpackFrame(&CrsfSdOpentx);

				#ifdef DEBUG_CRSF_SD_WRITE
				TRACE("crsfSdWriteHandler:CRSF_SD_START:sync:%x:length:%d:cmd:%x:subcmd:%d:chunk:%d:crc:%d:calCrc:%d", CrsfSdOpentx.sync, CrsfSdOpentx.length, CrsfSdOpentx.cmd, CrsfSdOpentx.subcmd, CrsfSdOpentx.chunk, CrsfSdOpentx.crc, CrsfSdOpentx.calCrc);
				#endif

				if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == 0X3a && CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_START){
					state = CRSF_SD_WRITE;
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_START:start to write");
					#endif

					CrsfSdOpentx.result = f_open(&CrsfSdOpentx.file, (char*)CrsfSdOpentx.payload, FA_READ);
					if( CrsfSdOpentx.result == FR_OK ){
						#if defined(DEBUG_CRSF_SD_WRITE) || defined(DEBUG_CRSF_SD_WRITE_COMPARE)
						TRACE("crsfSdWriteHandler:CRSF_SD_START:result:%d", CrsfSdOpentx.result);
						TRACE("crsfSdWriteHandler:CRSF_SD_START:open %s successful", CrsfSdOpentx.payload);
						#endif
						CrsfSdOpentx.result = f_stat((char*)CrsfSdOpentx.payload, &CrsfSdOpentx.fileInfo);
						if( CrsfSdOpentx.result == FR_OK ){
							#if defined(DEBUG_CRSF_SD_WRITE) || defined(DEBUG_CRSF_SD_WRITE_COMPARE)
							TRACE("crsfSdWriteHandler:CRSF_SD_START:get info successful:fsize:%d", CrsfSdOpentx.fileInfo.fsize);
							#endif
							CrsfSdOpentx.bytesPendingToSend = CrsfSdOpentx.fileInfo.fsize;
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
						TRACE("crsfSdWriteHandler:CRSF_SD_START:open failed");
						#endif
						state = CRSF_SD_IDLE;
					}
				}
				else{
					CrsfSdOpentx.dst = LIBCRSF_RC_TX;
					CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
					CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_RETRY_START;
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
			if(CrsfSdOpentx.bytesPendingToSend <= 0 && CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_FINISH){
				state = CRSF_SD_FINISH;
				CrsfSdOpentx.result = f_close(&CrsfSdOpentx.file);
			}
			else if((CrsfSdOpentx.bytesPendingToSend > 0  && CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_ACK && crossfireSharedData.crsf_rx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)) ||
					CrsfSdOpentx.bytesPendingToSend == CrsfSdOpentx.fileInfo.fsize){
				CrsfSdOpentx.result = f_read(&CrsfSdOpentx.file, CrsfSdOpentx.payload, CrsfSdOpentx.requestDataLength, (UINT*)&CrsfSdOpentx.numOfBytesRead);
				if( CrsfSdOpentx.result == FR_OK ){
					CrsfSdOpentx.chunk = CrsfSdOpentx.bytesPendingToSend / CrsfSdOpentx.numOfBytesRead;
					if(CrsfSdOpentx.chunk == 1 && (CrsfSdOpentx.bytesPendingToSend - CrsfSdOpentx.numOfBytesRead == 0)){
						CrsfSdOpentx.chunk = 0;
					}
					CrsfSdOpentx.dst = LIBCRSF_RC_TX;
					CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
					CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_OK;
					crsfSdPackFrame(&CrsfSdOpentx);
					CRSF_to_Shared_FIFO(CrsfSdOpentx.txBuf);
					state = CRSF_SD_ACK;
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_WRITE:subcmd:%d:bytesPendingToSend:%d:chunk:%d:numOfBytesRead:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk, CrsfSdOpentx.numOfBytesRead);
					#endif
				}
			}
			else if(CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_RETRY && crossfireSharedData.crsf_rx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdOpentx.dst = LIBCRSF_RC_TX;
				CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
				CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_RETRY;
				crsfSdPackFrame(&CrsfSdOpentx);
				CRSF_to_Shared_FIFO(CrsfSdOpentx.txBuf);
				state = CRSF_SD_ACK;
				#ifdef DEBUG_CRSF_SD_WRITE
				TRACE("crsfSdWriteHandler:CRSF_SD_WRITE:subcmd:%d:bytesPendingToSend:%d:chunk:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk);
				#endif
			}
			break;
		}

		case CRSF_SD_ACK:
		{
			if(crossfireSharedData.crsf_tx.size() > 0){
				for(uint8_t i = 0; i < LIBCRSF_MAX_BUFFER_SIZE; i++){
					if(!crossfireSharedData.crsf_tx.pop(CrsfSdOpentx.rxBuf[i])){
						break;
					}
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_ACK:rxbuf[%d]:%x", i, CrsfSdOpentx.rxBuf[i]);
					#endif
				}
				crsfSdUnpackFrame(&CrsfSdOpentx);
				if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == 0X3a && (CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_ACK || CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_FINISH)){
					CrsfSdOpentx.bytesPendingToSend -= CrsfSdOpentx.numOfBytesRead;
					state = CRSF_SD_WRITE;
					#if defined(DEBUG_CRSF_SD_WRITE) // || defined(DEBUG_CRSF_SD_WRITE_COMPARE)
					TRACE("crsfSdWriteHandler:CRSF_SD_ACK:subcmd:%d:bytesPendingToSend:%d:chunk:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.bytesPendingToSend, CrsfSdOpentx.chunk);
					#endif
					#ifdef DEBUG_CRSF_SD_WRITE
					TRACE("crsfSdWriteHandler:CRSF_SD_ACK:success to write");
					#endif
				}
				else{
					CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_RETRY;
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
		default:
		{
			break;
		}
	}
}

void crsfSdReadHandler(){
	static CrsfSd_t CrsfSdOpentx;
	static uint8_t state = CRSF_SD_START;
	static uint8_t prevState = CRSF_SD_START;
	switch(state){
		case CRSF_SD_START:
		{
//			uint8_t size = crossfireSharedData.crsf_tx.size();
//			TRACE("crsfSdReadHandler:crossfireSharedData.crsf_tx.size:%d", size);
			if(crossfireSharedData.crsf_tx.size() > 0){
				for(uint8_t i = 0; i < LIBCRSF_MAX_BUFFER_SIZE; i++){
					if(!crossfireSharedData.crsf_tx.pop(CrsfSdOpentx.rxBuf[i])){
						break;
					}
					#ifdef DEBUG_CRSF_SD_READ
					TRACE("crsfSdReadHandler:CRSF_SD_START:rxbuf[%d]:%x", i, CrsfSdOpentx.rxBuf[i]);
					#endif
				}
				uint8_t prev_subcmd = CrsfSdOpentx.subcmd;
				crsfSdUnpackFrame(&CrsfSdOpentx);

				#ifdef DEBUG_CRSF_SD_READ
				TRACE("crsfSdReadHandler:CRSF_SD_START:sync:%x:length:%d:cmd:%x:subcmd:%d:chunk:%d:crc:%d:calCrc:%d", CrsfSdOpentx.sync, CrsfSdOpentx.length, CrsfSdOpentx.cmd, CrsfSdOpentx.subcmd, CrsfSdOpentx.chunk, CrsfSdOpentx.crc, CrsfSdOpentx.calCrc);
				#endif
				if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == 0x3a && (CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_START || CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_RETRY_START) && CrsfSdOpentx.dst == LIBCRSF_REMOTE_ADD){
					CrsfSdOpentx.result = f_open(&CrsfSdOpentx.file, (const char*)CrsfSdOpentx.payload, FA_CREATE_ALWAYS | FA_WRITE);
					#if defined(DEBUG_CRSF_SD_READ) || defined(DEBUG_CRSF_SD_READ_COMPARE)
					TRACE("crsfSdReadHandler:CRSF_SD_START:result:%d", CrsfSdOpentx.result);
					#endif
					prevState == state;
					state = CRSF_SD_ACK;
				}
				else {
//					if(prev_subcmd == CRSF_SD_SUBCMD_START || prev_subcmd == CRSF_SD_SUBCMD_RETRY_START ){
//						CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_RETRY_START;
//					}
//					prevState = state;
//					state = CRSF_SD_ERROR;


					if(crossfireSharedData.crsf_rx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
						CrsfSdOpentx.dst = LIBCRSF_RC_TX;
						CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
						CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_RETRY_START;
						CrsfSdOpentx.numOfBytesRead = 0;
						crsfSdPackFrame(&CrsfSdOpentx);
						for(uint8_t i = 0; i < CrsfSdOpentx.txBuf[LIBCRSF_LENGTH_ADD] + 2; i++){
							crossfireSharedData.crsf_rx.push(CrsfSdOpentx.txBuf[i]);
							#ifdef DEBUG_CRSF_SD_READ
							TRACE("crsfSdReadHandler:CRSF_SD_START:txBuf[%d]:%x", i, CrsfSdOpentx.txBuf[i]);
							#endif
						}
						#ifdef DEBUG_CRSF_SD_READ
						TRACE("crsfSdReadHandler:CRSF_SD_START:crossfireSharedData.crsf_rx:%d", crossfireSharedData.crsf_rx.size());
						TraceState("crsfSdReadHandler:CRSF_SD_START:state:", state);
						#endif
					}
				}
//				TraceState("crsfSdReadHandler:CRSF_SD_START:state:", state);
			}
			break;
		}
		case CRSF_SD_READ:
		{
			if(crossfireSharedData.crsf_tx.size() > 0){
				for(uint8_t i = 0; i < LIBCRSF_MAX_BUFFER_SIZE; i++){
					if(!crossfireSharedData.crsf_tx.pop(CrsfSdOpentx.rxBuf[i])){
						break;
					}
					#ifdef DEBUG_CRSF_SD_READ
					TRACE("crsfSdReadHandler:CRSF_SD_READ:rxbuf[%d]:%x", i, CrsfSdOpentx.rxBuf[i]);
					#endif
				}
				uint16_t prevChunk = CrsfSdOpentx.chunk;
				uint8_t prevLength = CrsfSdOpentx.length;
				crsfSdUnpackFrame(&CrsfSdOpentx);
				#ifdef DEBUG_CRSF_SD_READ
				TRACE("crsfSdReadHandler:CRSF_SD_READ:sync:%x:length:%d:cmd:%x:subcmd:%d:chunk:%d:crc:%d:calCrc:%d", CrsfSdOpentx.sync, CrsfSdOpentx.length, CrsfSdOpentx.cmd, CrsfSdOpentx.subcmd, CrsfSdOpentx.chunk, CrsfSdOpentx.crc, CrsfSdOpentx.calCrc);
				#endif
				if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == 0x3a && (CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_OK || CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_RETRY) && CrsfSdOpentx.dst == LIBCRSF_RC_TX){
					if(prevChunk != CrsfSdOpentx.chunk || (prevChunk == CrsfSdOpentx.chunk && prevLength != CrsfSdOpentx.length)){
						uint16_t actualReceviedLength = CrsfSdOpentx.length - CRSF_SD_DATA_START_ADD + 1;
//						memcpy(pData, CrsfSdOpentx.payload, actualReceviedLength);
						CrsfSdOpentx.bytesReceived += actualReceviedLength;
						CrsfSdOpentx.result = f_write(&CrsfSdOpentx.file, CrsfSdOpentx.payload, actualReceviedLength, (UINT*)&CrsfSdOpentx.numOfBytesWritten);
						TRACE("crsfSdReadHandler:CRSF_SD_READ:result:%d:bytesReceived:%d", CrsfSdOpentx.result, CrsfSdOpentx.bytesReceived);
						#ifdef DEBUG_CRSF_SD_READ
						TRACE("crsfSdReadHandler:CRSF_SD_READ:result:%d:numOfBytesWritten:%d", CrsfSdOpentx.result, CrsfSdOpentx.numOfBytesWritten);
						TRACE("crsfSdReadHandler:CRSF_SD_READ:written in buffer");
						#endif
					}
					#if defined(DEBUG_CRSF_SD_READ) // || defined(DEBUG_CRSF_SD_READ_COMPARE)
					TRACE("crsfSdReadHandler:CRSF_SD_READ:subcmd:%d:bytesReceived:%d:chunk:%d:numOfBytesWritten:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.bytesReceived, CrsfSdOpentx.chunk, CrsfSdOpentx.numOfBytesWritten);
					#endif
					state = CRSF_SD_DATA_READY;
				}
				else if(CrsfSdOpentx.sync == LIBCRSF_UART_SYNC && CrsfSdOpentx.crc == CrsfSdOpentx.calCrc && CrsfSdOpentx.cmd == 0x3a && CrsfSdOpentx.subcmd == CRSF_SD_SUBCMD_RETRY_START && CrsfSdOpentx.dst == LIBCRSF_RC_TX){
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
//			uint8_t size = crossfireSharedData.crsf_rx.size();
//			TRACE("crsfSdReadHandler:crossfireSharedData.crsf_rx.size:%d", size);
			if(crossfireSharedData.crsf_rx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdOpentx.dst = LIBCRSF_RC_TX;
				CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
//				if(CrsfSdOpentx.chunk > 0){
				if(CrsfSdOpentx.chunk > 0){
					CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_ACK;
					state = CRSF_SD_READ;
				}
				else{
					CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_FINISH;
					state = CRSF_SD_FINISH;
				}
//				}
//				else if(CrsfSdOpentx.chunk == 0){
//					state = CRSF_SD_FINISH;
//					CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_FINISH;
//				}
				CrsfSdOpentx.requestDataLength = 0;
				CrsfSdOpentx.numOfBytesRead = 0;
				crsfSdPackFrame(&CrsfSdOpentx);
				for(uint8_t i = 0; i < CrsfSdOpentx.txBuf[LIBCRSF_LENGTH_ADD] + 2; i++){
					crossfireSharedData.crsf_rx.push(CrsfSdOpentx.txBuf[i]);
					#ifdef DEBUG_CRSF_SD_READ
					TRACE("crsfSdReadHandler:CRSF_SD_ACK:txBuf[%d]:%x", i, CrsfSdOpentx.txBuf[i]);
					#endif
				}
				#if defined(DEBUG_CRSF_SD_READ) // || defined(DEBUG_CRSF_SD_READ_COMPARE)
				TRACE("crsfSdReadHandler:CRSF_SD_ACK:subcmd:%d:bytesReceived:%d:chunk:%d", CrsfSdOpentx.subcmd, CrsfSdOpentx.bytesReceived, CrsfSdOpentx.chunk);
				TRACE("crsfSdReadHandler:CRSF_SD_ACK:ack sent");
				#endif
			}
			break;
		}

		case CRSF_SD_FINISH:
		{
			CrsfSdOpentx.result = f_sync(&CrsfSdOpentx.file);
			CrsfSdOpentx.result = f_close(&CrsfSdOpentx.file);
			TRACE("crsfSdReadHandler:CRSF_SD_FINISH:result:%d", CrsfSdOpentx.result);
			TRACE("crsfSdReadHandler:CRSF_SD_FINISH");
			#if defined(DEBUG_CRSF_SD_READ) || defined(DEBUG_CRSF_SD_READ_COMPARE)
			TRACE("crsfSdReadHandler:CRSF_SD_FINISH:result:%d", CrsfSdOpentx.result);
			TRACE("crsfSdReadHandler:CRSF_SD_FINISH");
			#endif
			state = CRSF_SD_IDLE;
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
				CrsfSdOpentx.dst = LIBCRSF_RC_TX;
				CrsfSdOpentx.org = LIBCRSF_REMOTE_ADD;
				CrsfSdOpentx.subcmd = CRSF_SD_SUBCMD_RETRY;
				state = CRSF_SD_READ;
				CrsfSdOpentx.numOfBytesRead = 0;
				crsfSdPackFrame(&CrsfSdOpentx);
				for(uint8_t i = 0; i < CrsfSdOpentx.txBuf[LIBCRSF_LENGTH_ADD] + 2; i++){
					crossfireSharedData.crsf_rx.push(CrsfSdOpentx.txBuf[i]);
					#ifdef DEBUG_CRSF_SD_READ
					TRACE("crsfSdReadHandler:CRSF_SD_ERROR:txBuf[%d]:%x", i, CrsfSdOpentx.txBuf[i]);
					#endif
				}
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

		default:
		{
			break;
		}
	}
//	TraceState("crsfSdReadHandler:state:", state);
//	TraceSubCmd("crsfSdReadHandler:subcmd:", CrsfSdOpentx.subcmd);
}

void crsfSdXfWriteHandler( char* filename, BYTE* pData, uint16_t Length, uint8_t& state ){
	static CrsfSd_t CrsfSdXf;
	static uint8_t prevState = state;
	switch(state){
		case CRSF_SD_START:
		{
//			uint8_t size = crossfireSharedData.crsf_tx.size();
//			TRACE("crsfSdXfWriteHandler:CRSF_SD_START:crossfireSharedData.crsf_tx.size:%d", size);
			if(Length > 0 && crossfireSharedData.crsf_tx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdXf.dst = LIBCRSF_REMOTE_ADD;
				CrsfSdXf.org = LIBCRSF_RC_TX;
				CrsfSdXf.subcmd = CRSF_SD_SUBCMD_START;
				CrsfSdXf.requestDataLength = Length;
				uint8_t filenameLength = 0;
				for(int i = 0; i < 128; i++){
					filenameLength++;
					if(filename[i] == 0){
						break;
					}
					if(i == 127){
						state = CRSF_SD_INVALID_FILENAME;
						return;
					}
				}
				CrsfSdXf.bytesPendingToSend = CrsfSdXf.fileInfo.fsize = fwLength;
				memcpy(CrsfSdXf.payload, filename, filenameLength);
				CrsfSdXf.numOfBytesRead = filenameLength;
				CrsfSdXf.chunk = 65535;
				crsfSdPackFrame(&CrsfSdXf);
				for(uint8_t i = 0; i < CrsfSdXf.txBuf[LIBCRSF_LENGTH_ADD] + 2; i++){
					crossfireSharedData.crsf_tx.push(CrsfSdXf.txBuf[i]);
					#ifdef DEBUG_CRSF_SD_XF_WRITE
					TRACE("crsfSdXfWriteHandler:CRSF_SD_START:txBuf[%d]:%x", i, CrsfSdXf.txBuf[i]);
					#endif
				}
				#ifdef DEBUG_CRSF_SD_XF_WRITE
				TRACE("crsfSdXfWriteHandler:CRSF_SD_START:start sent");
				#endif
				prevState = state;
				state = CRSF_SD_ACK;
			}
			break;
		}

		case CRSF_SD_WRITE:
		{

			#ifdef DEBUG_CRSF_SD_XF_WRITE
			TRACE("crsfSdXfWriteHandler:CRSF_SD_WRITE:subcmd:%d:bytesPendingToSend:%d:chunk:%d:numOfBytesRead:%d", CrsfSdXf.subcmd, CrsfSdXf.bytesPendingToSend, CrsfSdXf.chunk, CrsfSdXf.numOfBytesRead);
			TraceSubCmd("crsfSdXfWriteHandler:CRSF_SD_WRITE:subcmd:", CrsfSdXf.subcmd);
			#endif

//			if(CrsfSdXf.chunk == 0 && CrsfSdXf.bytesPendingToSend == 0){
//				state = CRSF_SD_FINISH;
//				for(int j = 0; j < 53; j++){
//					TRACE("CRSF_SD_WRITE:%d:%lx", j, pData[j]);
//				}
////				CrsfSdXf.result = f_close(&CrsfSdXf.file);
//			}
			if((CrsfSdXf.bytesPendingToSend > 0  && CrsfSdXf.subcmd == CRSF_SD_SUBCMD_ACK && crossfireSharedData.crsf_tx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)) ||
					CrsfSdXf.bytesPendingToSend == CrsfSdXf.fileInfo.fsize){
//				TRACE("crsfSdXfWriteHandler:CRSF_SD_WRITE:subcmd:%d:bytesPendingToSend:%d:chunk:%d:numOfBytesRead:%d", CrsfSdXf.subcmd, CrsfSdXf.bytesPendingToSend, CrsfSdXf.chunk, CrsfSdXf.numOfBytesRead);
//				TRACE("GOOD GOOD");
//				if(CrsfSdXf.bytesPendingToSend > CRSF_SD_DATA_MAX_BUFFER_SIZE){
//					CrsfSdXf.numOfBytesRead = CrsfSdXf.requestDataLength = Length;
//				}
//				else{
//					CrsfSdXf.numOfBytesRead = CrsfSdXf.requestDataLength = CrsfSdXf.bytesPendingToSend;
//				}
				CrsfSdXf.numOfBytesRead = CrsfSdXf.requestDataLength = Length;
				CrsfSdXf.chunk = CrsfSdXf.bytesPendingToSend / CrsfSdXf.numOfBytesRead;
				if(CrsfSdXf.chunk == 1 && (CrsfSdXf.bytesPendingToSend - CrsfSdXf.numOfBytesRead == 0)){
					CrsfSdXf.chunk = 0;
				}
//				TRACE("CrsfSdXf.chunk:%d", CrsfSdXf.chunk);

//				for(int j = 0; j < 53; j++){
//					TRACE("CRSF_SD_WRITE:%d:%lx", j, pData[j]);
//				}
				memcpy(CrsfSdXf.payload, pData, CrsfSdXf.numOfBytesRead);
				CrsfSdXf.dst = LIBCRSF_RC_TX;
				CrsfSdXf.org = LIBCRSF_REMOTE_ADD;
				CrsfSdXf.subcmd = CRSF_SD_SUBCMD_OK;
				crsfSdPackFrame(&CrsfSdXf);
				for(uint8_t i = 0; i < CrsfSdXf.txBuf[LIBCRSF_LENGTH_ADD] + 2; i++){
					crossfireSharedData.crsf_tx.push(CrsfSdXf.txBuf[i]);
					#ifdef DEBUG_CRSF_SD_XF_WRITE
					TRACE("crsfSdXfWriteHandler:CRSF_SD_WRITE:txBuf[%d]:%x", i, CrsfSdXf.txBuf[i]);
					#endif
				}
				state = CRSF_SD_ACK;
//				TRACE("crsfSdXfWriteHandler:CRSF_SD_WRITE:subcmd:%d:bytesPendingToSend:%d:chunk:%d:numOfBytesRead:%d", CrsfSdXf.subcmd, CrsfSdXf.bytesPendingToSend, CrsfSdXf.chunk, CrsfSdXf.numOfBytesRead);

				#ifdef DEBUG_CRSF_SD_XF_WRITE
				TRACE("crsfSdXfWriteHandler:CRSF_SD_WRITE:subcmd:%d:bytesPendingToSend:%d:chunk:%d:numOfBytesRead:%d", CrsfSdXf.subcmd, CrsfSdXf.bytesPendingToSend, CrsfSdXf.chunk, CrsfSdXf.numOfBytesRead);
				TraceSubCmd("crsfSdXfWriteHandler:CRSF_SD_WRITE:subcmd:", CrsfSdXf.subcmd);
				#endif
			}
			else if(CrsfSdXf.subcmd == CRSF_SD_SUBCMD_RETRY && crossfireSharedData.crsf_tx.hasSpace(LIBCRSF_MAX_BUFFER_SIZE)){
				CrsfSdXf.dst = LIBCRSF_RC_TX;
				CrsfSdXf.org = LIBCRSF_REMOTE_ADD;
				CrsfSdXf.subcmd = CRSF_SD_SUBCMD_RETRY;
				crsfSdPackFrame(&CrsfSdXf);
				CRSF_to_Shared_FIFO(CrsfSdXf.txBuf);
				state = CRSF_SD_ACK;
				#ifdef DEBUG_CRSF_SD_XF_WRITE
				TRACE("crsfSdXfWriteHandler:CRSF_SD_WRITE:subcmd:%d:bytesPendingToSend:%d:chunk:%d", CrsfSdXf.subcmd, CrsfSdXf.bytesPendingToSend, CrsfSdXf.chunk);
				TraceSubCmd("crsfSdXfWriteHandler:CRSF_SD_WRITE:subcmd:", CrsfSdXf.subcmd);
				#endif
			}
			break;
		}

		case CRSF_SD_ACK:
		{
//			uint8_t size = crossfireSharedData.crsf_rx.size();
//			TRACE("crsfSdXfWriteHandler:crossfireSharedData.crsf_rx.size:%d", size);
			if(crossfireSharedData.crsf_rx.size() > 0){
				for(uint8_t i = 0; i < LIBCRSF_MAX_BUFFER_SIZE; i++){
					if(!crossfireSharedData.crsf_rx.pop(CrsfSdXf.rxBuf[i])){
						break;
					}
					#ifdef DEBUG_CRSF_SD_XF_WRITE
					TRACE("crsfSdXfWriteHandler:CRSF_SD_ACK:rxbuf[%d]:%x", i, CrsfSdXf.rxBuf[i]);
					#endif
				}
				uint8_t prev_subcmd = CrsfSdXf.subcmd;
//				TraceSubCmd("crsfSdXfWriteHandler:CRSF_SD_ACK:prev_subcmd:", prev_subcmd);
				uint16_t prevChunk = CrsfSdXf.chunk;
				uint8_t prevLength = CrsfSdXf.length;
				crsfSdUnpackFrame(&CrsfSdXf);
				#if defined(DEBUG_CRSF_SD_XF_WRITE) // || defined(DEBUG_CRSF_SD_XF_WRITE_COMPARE)
				TRACE("crsfSdXfWriteHandler:CRSF_SD_ACK:sync:%x:length:%d:cmd:%x:subcmd:%d:chunk:%d:crc:%d:calCrc:%d", CrsfSdXf.sync, CrsfSdXf.length, CrsfSdXf.cmd, CrsfSdXf.subcmd, CrsfSdXf.chunk, CrsfSdXf.crc, CrsfSdXf.calCrc);
				TraceSubCmd("crsfSdXfWriteHandler:CRSF_SD_ACK:subcmd:", CrsfSdXf.subcmd);
				#endif
				if(CrsfSdXf.sync == LIBCRSF_UART_SYNC && CrsfSdXf.crc == CrsfSdXf.calCrc && CrsfSdXf.cmd == 0X3a && (CrsfSdXf.subcmd == CRSF_SD_SUBCMD_OK || CrsfSdXf.subcmd == CRSF_SD_SUBCMD_ACK || CrsfSdXf.subcmd == CRSF_SD_SUBCMD_FINISH)){
					if(!(prev_subcmd == CRSF_SD_SUBCMD_START && prev_subcmd == CRSF_SD_SUBCMD_RETRY_START) && prevChunk != 65535){
						CrsfSdXf.bytesPendingToSend -= CrsfSdXf.numOfBytesRead;
						if(CrsfSdXf.chunk == 0 && CrsfSdXf.bytesPendingToSend == 0){
							state = CRSF_SD_FINISH;
							for(int j = 0; j < 53; j++){
								TRACE("CRSF_SD_WRITE:%d:%lx", j, pData[j]);
							}
			//				CrsfSdXf.result = f_close(&CrsfSdXf.file);
						}
						else{
							state = CRSF_SD_DATA_TX_COMPLETED;
						}
					}
					else{
						state = CRSF_SD_WRITE;
					}
					#if defined(DEBUG_CRSF_SD_XF_WRITE) // || defined(DEBUG_CRSF_SD_XF_WRITE_COMPARE)
					TRACE("crsfSdXfWriteHandler:CRSF_SD_ACK:subcmd:%d:bytesPendingToSend:%d:chunk:%d", CrsfSdXf.subcmd, CrsfSdXf.bytesPendingToSend, CrsfSdXf.chunk);
					TraceSubCmd("crsfSdXfWriteHandler:CRSF_SD_ACK:subcmd:", CrsfSdXf.subcmd);
					#endif
					#ifdef DEBUG_CRSF_SD_XF_WRITE
					TRACE("crsfSdXfWriteHandler:CRSF_SD_ACK:success to write");
					#endif
				}
				else if(CrsfSdXf.sync == LIBCRSF_UART_SYNC && CrsfSdXf.crc == CrsfSdXf.calCrc && CrsfSdXf.cmd == 0X3a && CrsfSdXf.subcmd == CRSF_SD_SUBCMD_RETRY_START){
					if(prev_subcmd == CRSF_SD_SUBCMD_START || prev_subcmd == CRSF_SD_SUBCMD_RETRY_START){
						CrsfSdXf.subcmd = CRSF_SD_SUBCMD_RETRY_START;
						state = CRSF_SD_START;
						#ifdef DEBUG_CRSF_SD_XF_WRITE
						TRACE("crsfSdXfWriteHandler:CRSF_SD_ACK:retry to start");
						#endif
					}
				}
				else{
					if(prevState != CRSF_SD_START){
						CrsfSdXf.subcmd = CRSF_SD_SUBCMD_RETRY;
						state = CRSF_SD_WRITE;
					}
					else{
						CrsfSdXf.subcmd = CRSF_SD_SUBCMD_RETRY_START;
						state = CRSF_SD_START;
					}
					#ifdef DEBUG_CRSF_SD_XF_WRITE
					TRACE("crsfSdXfWriteHandler:CRSF_SD_ACK:retry to write");
					#endif
				}
			}
			break;
		}

		case CRSF_SD_DATA_TX_COMPLETED:
		{
			#if defined(DEBUG_CRSF_SD_XF_WRITE) || defined(DEBUG_CRSF_SD_XF_WRITE_COMPARE)
			TRACE("crsfSdXfWriteHandler:CRSF_SD_DATA_TX_COMPLETED");
			#endif
			state = CRSF_SD_WRITE;
			break;
		}

		case CRSF_SD_FINISH:
		{
			TRACE("crsfSdXfWriteHandler:CRSF_SD_FINISH");
			#if defined(DEBUG_CRSF_SD_XF_WRITE) || defined(DEBUG_CRSF_SD_XF_WRITE_COMPARE)
			TRACE("crsfSdXfWriteHandler:CRSF_SD_FINISH");
			#endif
			state = CRSF_SD_IDLE;
			break;
		}

		case CRSF_SD_ERROR:
		{
			#ifdef DEBUG_CRSF_SD_XF_WRITE
			TRACE("crsfSdXfWriteHandler:CRSF_SD_ERROR:%d", CrsfSdXf.result);
			#endif
			f_close(&CrsfSdXf.file);
			state = CRSF_SD_IDLE;

			break;
		}
		default:
		{
			break;
		}
	}
//	TraceState("crsfSdXfWriteHandler:state:", state);
//	TraceSubCmd("crsfSdXfWriteHandler:subcmd:", CrsfSdXf.subcmd);
}

char fwFilename[CRSF_SD_FW_NAME_LENGTH];
uint32_t fwLength = 0;

void crsfSdWriteHeader( uint32_t HW_ID, uint16_t FW_ID, uint32_t FW_Length ){
	fwFilename[0] = '0';
	fwFilename[1] = 'x';
	sprintf(&fwFilename[2], "%06lx", HW_ID);
	fwFilename[8] = 'x';
	fwFilename[9] = 'x';
	fwFilename[10] = '_';
	fwFilename[11] = '0';
	fwFilename[12] = 'x';
	sprintf(&fwFilename[13], "%04lx", FW_ID);
	fwFilename[17] = '.';
	fwFilename[18] = 'b';
	fwFilename[19] = 'i';
	fwFilename[20] = 'n';
	fwFilename[21] = 0;
	fwLength = FW_Length;
}

uint8_t crsfSdWrite( char* filename, BYTE *pData, uint16_t Length ){
	static uint32_t totalSent = 0;
	static BYTE *addr = pData;
	static uint8_t txState = CRSF_SD_IDLE;
	static uint32_t bytesSent = 0;
	static uint16_t i = 0;
	static uint16_t count = 0;
	static uint16_t len = 0;
	if(txState == CRSF_SD_IDLE){
		txState = CRSF_SD_START;
	}
	if(bytesSent == 0 || bytesSent == Length || i == count){
		bytesSent = 0;
		i = 0;
		count = Length / CRSF_SD_DATA_MAX_BUFFER_SIZE;
		if(Length - count * CRSF_SD_DATA_MAX_BUFFER_SIZE > 0){
			count++;
		}
		len = Length / CRSF_SD_DATA_MAX_BUFFER_SIZE > 0 ? CRSF_SD_DATA_MAX_BUFFER_SIZE : Length;
		addr = pData;
	}
	if(i < count){
		if(txState == CRSF_SD_FINISH || txState == CRSF_SD_DATA_TX_COMPLETED){
			if(Length - (count - 1) * CRSF_SD_DATA_MAX_BUFFER_SIZE > 0 && (i == count - 1)){
				len = Length - (count - 1) * CRSF_SD_DATA_MAX_BUFFER_SIZE;
			}
		}
		crsfSdXfWriteHandler( filename, addr, len, txState);
		crsfSdReadHandler();
		if(txState == CRSF_SD_FINISH || txState == CRSF_SD_DATA_TX_COMPLETED){
			totalSent += len;
			addr += len;
			bytesSent += len;
			i++;
			TRACE("totalSent:%ld:bytesSent:%d:pData:%lx:addr:%lx", totalSent, bytesSent, (uint32_t)pData, (uint32_t)addr - bytesSent);
		}
	}

	TraceState("crsfSdWrite:", txState);
	if(i == count){
		TRACE("CRSF_SD_DATA_TX_COMPLETED_HIHIHI");
		return txState;
	}
	else if(txState == CRSF_SD_IDLE){
		TRACE("CRSF_SD_FINISH_LALALA");
		return CRSF_SD_FINISH;
	}
	else{
		return CRSF_SD_BUSY;
	}
}

uint8_t crsfSdRead( char* filename, BYTE *pData, uint16_t Length ){
	static uint8_t rxState = CRSF_SD_IDLE;
	static uint32_t bytesSent = 0;
	static uint16_t i = 0;
	static uint16_t count = 0;
	static uint16_t len = 0;
	if(rxState == CRSF_SD_IDLE){
		rxState = CRSF_SD_START;
	}
	if(bytesSent == 0 || bytesSent == Length){
		bytesSent = 0;
		i = 0;
		count = Length / CRSF_SD_DATA_MAX_BUFFER_SIZE;
		if(Length - count * CRSF_SD_DATA_MAX_BUFFER_SIZE > 0){
			count++;
		}
		len = Length / CRSF_SD_DATA_MAX_BUFFER_SIZE > 0 ? CRSF_SD_DATA_MAX_BUFFER_SIZE : Length;
	}
	if(i < count){
		if(rxState == CRSF_SD_FINISH || rxState == CRSF_SD_DATA_READY){
			if(Length - (count - 1) * CRSF_SD_DATA_MAX_BUFFER_SIZE > 0 && (i == count - 1)){
				len = Length - (count - 1) * CRSF_SD_DATA_MAX_BUFFER_SIZE;
			}
		}
		crsfSdXfReadHandler( filename, pData, len, rxState);
		if(rxState == CRSF_SD_FINISH || rxState == CRSF_SD_DATA_READY){
			pData += len;
			bytesSent += len;
			i++;
		}
	}

	if(i == count){
		return rxState;
	}
	else if(rxState == CRSF_SD_IDLE){
		return CRSF_SD_FINISH;
	}
	else{
		return CRSF_SD_BUSY;
	}
}

#endif // CRSF_SD

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
