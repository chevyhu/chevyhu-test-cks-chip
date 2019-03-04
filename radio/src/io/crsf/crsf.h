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

/* USAGE ******************************
    Please Only include crsf.h
    This library will need to include libCrc, libUtil (No need to include crc8.h, utilities.h in your project)

    You may define the following MACROs in your project if needed.
	LIBCRSF_ENABLE_PARAMETER
        - LIBRARY_ENABLE_64BITS
	LIBCRSF_ENABLE_TELEMETRY
	LIBCRSF_ENABLE_COMMAND
	LIBCRSF_ENABLE_RC
	LIBCRSF_ENABLE_PARAMETER
	LIBCRSF_ENABLE_LOGGING
    LIBCRSF_ENABLE_FW_UPDATE_RESPONES or LIBCRSF_ENABLE_FW_UPDATE_REQUEST

    LIBCRSF_SYNC_PASS_ONLY
    LIBCRSF_ENABLE_PARSE_LESS_TYPE <-- This can stop use and redirect non-use type
    
    LIBCRSF_TEST_PRINT_ENABLE <-- For debug use only
**************************************/

#ifndef _CRSF_H    /* Guard against multiple inclusion */
#define _CRSF_H

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>



#include "crc8.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

/* ************************************************************************** */
#define LIBCRSF_MAX_BUFFER_SIZE                 64

#define LIBCRSF_DEVICE_LIST_SIZE                30
#ifdef LIBCRSF_ENABLE_PARAMETER
#define LIBCRSF_PARAMETER_LIST_SIZE             50
#endif

#define LIBCRSF_PARAM_VERSION_NUMBER            0x01

/* ************************************************************************** */
#define LIBCRSF_HEADER_OFFSET                   1
#define LIBCRSF_ADDRESS_ADD                     0
#define LIBCRSF_LENGTH_ADD                      1
#define LIBCRSF_TYPE_ADD                        2
#define LIBCRSF_PAYLOAD_START_ADD               3

#define LIBCRSF_EXT_HEAD_DST_ADD                3    /* extended header destination address address */
#define LIBCRSF_EXT_HEAD_ORG_ADD                4    /* extended header origin address address */
#define LIBCRSF_EXT_PAYLOAD_START_ADD           5
#define LIBCRSF_DEVICE_NAME_START_ADD           LIBCRSF_EXT_PAYLOAD_START_ADD
#define LIBCRSF_PARAMETER_NUMBER_ADD            LIBCRSF_EXT_PAYLOAD_START_ADD
#define LIBCRSF_REMAINING_CHUNK_ADD             6
#define LIBCRSF_PARAMETER_DATA_START_ADD        7
#define LIBCRSF_PARENT_FOLDER_ADD               LIBCRSF_PARAMETER_DATA_START_ADD
#define LIBCRSF_DATA_TYPE_ADD                   8
#define LIBCRSF_NAME_START_ADD                  9

#define LIBCRSF_CRC_SIZE                        1
#define LIBCRSF_PAYLOAD_SIZE                    LIBCRSF_MAX_BUFFER_SIZE


#define LIBCRSF_UART_SYNC                       0xC8
#define LIBCRSF_AGENT_LEGACY_SYNC               0x0A

/* ************************************************************************** */
#define LIBCRSF_PPM_CENTER                      1500

#define LIBCRSF_RC_MAX_NUMBER_OF_CHANNEL        16
#define LIBCRSF_RC_RESOLUTION                   11    /* Resolution in bits */
#define LIBCRSF_RC_CENTER                       992
#define LIBCRSF_RC_MAX                          ( 2 * LIBCRSF_RC_CENTER )
#define LIBCRSF_RC_MIN                          0

#define LIBCRSF_PPM_TO_CRSF_RC(x)               ( ( x - LIBCRSF_PPM_CENTER ) * 8/5 + LIBCRSF_RC_CENTER )
#define LIBCRSF_CRSF_TO_PPM_RC(x)               ( ( x - LIBCRSF_RC_CENTER ) * 5/8 + LIBCRSF_PPM_CENTER )

/* ************************************************************************** */
typedef enum {
    LIBCRSF_BROADCAST_ADD                       = 0x00,
    LIBCRSF_USB_HOST_ADD                        = 0x10,
    LIBCRSF_WIFI_ADD                            = 0x12,
    LIBCRSF_OSD_ADD                             = 0x80,
    LIBCRSF_CURR_SENS_ADD                       = 0x8A,
    LIBCRSF_TBS_CUR_SENS_ADD                    = 0xC0,
    LIBCRSF_GPS_ADD                             = 0xC2,
    LIBCRSF_BLACKBOX_ADD                        = 0xC4,
    LIBCRSF_FC_ADD                              = LIBCRSF_UART_SYNC,
    LIBCRSF_RACE_TAG_ADD                        = 0xCC,
    LIBCRSF_VTX_ADD                             = 0xCE,
    LIBCRSF_REMOTE_ADD                          = 0xEA,
    LIBCRSF_RC_RX                               = 0xEC,
    LIBCRSF_RC_TX                               = 0xEE
} _libCrsf_DEVICE_ADDRESS;

/* ************************************************************************** */
typedef enum {
#ifdef LIBCRSF_ENABLE_TELEMETRY
    LIBCRSF_BF_GPS                              = 0x02,
    LIBCRSF_BF_GPS_TIME                         = 0x03,
    LIBCRSF_BF_GPS_EXTENDED                     = 0x06,
    LIBCRSF_BF_BATTERY_SENSOR                   = 0x08,
    LIBCRSF_BF_HEARTBEAT                        = 0x0B,
    LIBCRSF_BF_VIDEO_TRANSMITTER                = 0x0F,
    LIBCRSF_BF_LINK_STATISTICS                  = 0x14,
    LIBCRSF_BF_ATTITUDE                         = 0x1E,
    LIBCRSF_BF_MAVLINK_FC_ADD_DATA              = 0x1F,
    LIBCRSF_BF_FLIGHT_MODE_TEXT_BASED           = 0x21,
#endif
#ifdef LIBCRSF_ENABLE_RC
    LIBCRSF_BF_RC_CHANNELS_PACK                 = 0x16,
#endif
    LIBCRSF_EX_PARAM_PING_DEVICE                = 0x28,
    LIBCRSF_EX_PARAM_DEVICE_INFO                = 0x29,
#ifdef LIBCRSF_ENABLE_PARAMETER
    LIBCRSF_EX_PARAM_SETTING_ENTRY              = 0x2B,
    LIBCRSF_EX_PARAM_SETTING_READ               = 0x2C,
    LIBCRSF_EX_PARAM_VALUE_WRITE                = 0x2D,
    LIBCRSF_EX_PARAM_GET_CHILD_LIST             = 0x2E,
    LIBCRSF_EX_PARAM_RETURN_CHILD_LIST          = 0x2F,
#endif
#ifdef LIBCRSF_ENABLE_COMMAND
    LIBCRSF_CMD_FRAME                           = 0x32,
#endif
#ifdef LIBCRSF_ENABLE_LOGGING
    LIBCRSF_LOGGING_FRAME                       = 0x34,
#endif
#if defined(LIBCRSF_ENABLE_FW_UPDATE_RESPONES) || defined(LIBCRSF_ENABLE_FW_UPDATE_REQUEST)
    LIBCRSF_FW_UPDATE                           = 0x38,
#endif

    LIBCRSF_KISS_FC_1                           = 0x78,
    LIBCRSF_KISS_FC_2                           = 0x79,

    LIBCRSF_BETAFLIGHT_FC_1                     = 0x7A,
    LIBCRSF_BETAFLIGHT_FC_2                     = 0x7B,
    LIBCRSF_BETAFLIGHT_FC_3                     = 0x7C,

    LIBCRSF_EX_PARAM_LAST_ADDRESS               = 0x96
} libCrsf_FrameTypesNum;

#define LIBCRSF_EXT_HEADER_RANGE_START          LIBCRSF_EX_PARAM_PING_DEVICE
#define LIBCRSF_EXT_HEADER_RANGE_STOP           LIBCRSF_EX_PARAM_LAST_ADDRESS

/* ************************************************************************** */
typedef enum {
    CRSF_PARSE_SYNC,
    CRSF_PARSE_RD_LENGTH,
    CRSF_PARSE_RD_FRAME
} _libCrsf_CRSF_PARSER_STATUS;

typedef struct {
    _libCrsf_CRSF_PARSER_STATUS Status;
    uint8_t Cnt;
    uint32_t Tmr;
    uint8_t Payload[ LIBCRSF_PAYLOAD_SIZE ];
} _libCrsf_CRSF_PARSE_DATA;

typedef struct {
    uint8_t Port_Name;
    void ( *Gateway )( uint8_t *pArr );
    uint8_t Device_List[ LIBCRSF_DEVICE_LIST_SIZE ];
}_libCrsf_CRSF_Port;

    void libCrsf_Init( uint8_t ThisDeviceAddress, char *ThisDeviceName, uint32_t serial_no, uint32_t hw_id, uint32_t fw_id );
    //void libCrsf_Get_Define_List( uint32_t *list, uint8_t listSize );
    void libCrsf_CRSF_Add_Device_Function_List( _libCrsf_CRSF_Port *functionList, uint8_t listSize );

    void libCrsf_add_router_filter( bool (*p_filter_function)( uint8_t Input_Port, uint8_t *pArr ));
    
    void libCrsf_enable_sync_on_boradcast_address( void ); 

    /* ***** Notes ***** */
    /* uint8_t *pArr for CRSF_Routing need to point to <Device address or Sync Byte>
     * of the CRSF frame
     * <Device address or Sync Byte> <Frame length> <Type> <Payload> <CRC> */
    void libCrsf_CRSF_Routing( uint8_t Input_Port, uint8_t *pArr );

    bool libCrsf_CRSF_Parse( _libCrsf_CRSF_PARSE_DATA *pParse_Data, uint8_t New_Data );

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _CRSF_H */

/* *****************************************************************************
 End of File
 */
