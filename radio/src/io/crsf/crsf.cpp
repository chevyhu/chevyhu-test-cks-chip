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


#include "crsf.h"

#define LIBCRSF_DEVICE_LIST_ENTRY_EMPTY  0

uint8_t libCrsf_MySlaveAddress = 0;
char *libCrsf_MyDeviceName;
uint32_t libCrsf_MySerialNo = 0;
uint32_t libCrsf_MyHwID = 0;
uint32_t libCrsf_MyFwID = 0;

_libCrsf_CRSF_Port *libCrsf_CRSF_Ports;
unsigned int libCrsf_CRSF_Ports_Size = 0;
bool (*libCrsf_filter_function)( uint8_t Input_Port, uint8_t *pArr );
static bool libCrsf_sync_on_boradcast_address = false;


void libCrsf_Init( uint8_t ThisDeviceAddress, char *ThisDeviceName, uint32_t serial_no, uint32_t hw_id, uint32_t fw_id ) {
    libCrsf_MySlaveAddress = ThisDeviceAddress;
    libCrsf_MyDeviceName = ThisDeviceName;
    libCrsf_MySerialNo = serial_no;
    libCrsf_MyHwID = hw_id;
    libCrsf_MyFwID = fw_id;
}

void libCrsf_CRSF_Add_Device_Function_List( _libCrsf_CRSF_Port *functionList, uint8_t listSize ) {
    libCrsf_CRSF_Ports = functionList;
    libCrsf_CRSF_Ports_Size = listSize;
#ifdef LIBCRSF_TEST_PRINT_ENABLE
    uint8_t Port_Cnt;
    for( Port_Cnt = 0; Port_Cnt < libCrsf_CRSF_Ports_Size; Port_Cnt++ ) {
        LIBUTIL_PRINTF("Port Name: %d %d \r\n"
                , Port_Cnt
                , libCrsf_CRSF_Ports[Port_Cnt].Port_Name );
    }
#endif
}

void libCrsf_add_router_filter( bool (*p_filter_function)( uint8_t Input_Port, uint8_t *pArr )) {

    libCrsf_filter_function = p_filter_function;
}

void libCrsf_enable_sync_on_boradcast_address( void ) {

    libCrsf_sync_on_boradcast_address = true;
}

void libCrsf_CRSF_Routing( uint8_t Input_Port, uint8_t *pArr ) {
    uint8_t Port_Cnt;
    uint8_t Device_Buff_Cnt;
    uint8_t Route_To_Port = 0;
    bool Device_Found = false;

    /* call filter function and return if filter function triggers */
    if( libCrsf_filter_function != NULL && libCrsf_filter_function( Input_Port, pArr ) ) {
        return;
    }

    if( *( pArr + LIBCRSF_TYPE_ADD ) >= LIBCRSF_EXT_HEADER_RANGE_START
            && *( pArr + LIBCRSF_TYPE_ADD ) < LIBCRSF_EXT_HEADER_RANGE_STOP ) {
        for( Port_Cnt = 0; Port_Cnt < libCrsf_CRSF_Ports_Size; Port_Cnt++ ) {
            if( Input_Port == libCrsf_CRSF_Ports[ Port_Cnt ].Port_Name ) {
                /* store source device address for future routing */
                for( Device_Buff_Cnt = 0; Device_Buff_Cnt < LIBCRSF_DEVICE_LIST_SIZE; Device_Buff_Cnt++ ) {
                    if( libCrsf_CRSF_Ports[ Port_Cnt ].Device_List[ Device_Buff_Cnt ] == *( pArr + LIBCRSF_EXT_HEAD_ORG_ADD ) ) {
                        break;     /* device already listed */
                    }

                    if( libCrsf_CRSF_Ports[ Port_Cnt ].Device_List[ Device_Buff_Cnt ] == LIBCRSF_DEVICE_LIST_ENTRY_EMPTY ) {
                        libCrsf_CRSF_Ports[ Port_Cnt ].Device_List[ Device_Buff_Cnt ] = *( pArr + LIBCRSF_EXT_HEAD_ORG_ADD ); /* new device found */
                        break;
                    }
                }
            } else if( *( pArr + LIBCRSF_EXT_HEAD_DST_ADD ) != LIBCRSF_BROADCAST_ADD ) {
                /* if frame is not sent to broadcast address try to route it */
                for( Device_Buff_Cnt = 0; Device_Buff_Cnt < LIBCRSF_DEVICE_LIST_SIZE; Device_Buff_Cnt++ ) {
                    if( libCrsf_CRSF_Ports[ Port_Cnt ].Device_List[ Device_Buff_Cnt ] == LIBCRSF_BROADCAST_ADD ) {
                        /* area we are searching never got written so we abort searching */
                        break;
                    }
                    if( libCrsf_CRSF_Ports[ Port_Cnt ].Device_List[ Device_Buff_Cnt ] == *( pArr + LIBCRSF_EXT_HEAD_DST_ADD )) {
                        Device_Found = true;
                        Route_To_Port = Port_Cnt;
                    }
                }
            }
        }
    }

    /* distribute frame */
    if( Device_Found ) {
        if( libCrsf_CRSF_Ports[ Route_To_Port ].Gateway != NULL ) {
            libCrsf_CRSF_Ports[ Route_To_Port ].Gateway( pArr );
        }
    } else {
#ifdef LIBCRSF_TEST_PRINT_ENABLE
        if( *( pArr + LIBCRSF_TYPE_ADD ) >= LIBCRSF_EXT_HEADER_RANGE_START
                && *( pArr + LIBCRSF_TYPE_ADD ) < LIBCRSF_EXT_HEADER_RANGE_STOP ) {
            LIBUTIL_PRINTF("Route to all. Type: 0x%X\r\n", *( pArr + LIBCRSF_TYPE_ADD ) );
        }
#endif

        for( Port_Cnt = 0; Port_Cnt < libCrsf_CRSF_Ports_Size; Port_Cnt++ ) {
            if( Input_Port != libCrsf_CRSF_Ports[ Port_Cnt ].Port_Name
                    && libCrsf_CRSF_Ports[ Port_Cnt ].Gateway != NULL ) {
                libCrsf_CRSF_Ports[ Port_Cnt ].Gateway( pArr );
            }
        }
    }
}

bool libCrsf_CRSF_Parse( _libCrsf_CRSF_PARSE_DATA *pParse_Data, uint8_t New_Data ) {
// TODO: perna connect system time with library
//    if( libUtil_systemtime != NULL ) {
//        if( libUtil_systemtime() - pParse_Data->Tmr > LIBUTIL_MS2TICKS( 1 ) ) {
//            pParse_Data->Status = CRSF_PARSE_SYNC;
//        }
//        pParse_Data->Tmr = libUtil_systemtime();
//    }
    switch( pParse_Data->Status ) {
        default:
        case CRSF_PARSE_SYNC:
            pParse_Data->Payload[ LIBCRSF_ADDRESS_ADD ] = New_Data;
#ifdef LIBCRSF_SYNC_PASS_ONLY
            if( pParse_Data->Payload[ LIBCRSF_ADDRESS_ADD ] == LIBCRSF_UART_SYNC ) {
#else
            if( ( pParse_Data->Payload[ LIBCRSF_ADDRESS_ADD ] == LIBCRSF_BROADCAST_ADD
                        && libCrsf_sync_on_boradcast_address )
               || pParse_Data->Payload[ LIBCRSF_ADDRESS_ADD ] == libCrsf_MySlaveAddress
               || pParse_Data->Payload[ LIBCRSF_ADDRESS_ADD ] == LIBCRSF_UART_SYNC ) {
#endif
                pParse_Data->Cnt = LIBCRSF_HEADER_OFFSET + LIBCRSF_HEADER_OFFSET;
                pParse_Data->Status = CRSF_PARSE_RD_LENGTH;
            }
            break;

        case CRSF_PARSE_RD_LENGTH:
            pParse_Data->Payload[ LIBCRSF_LENGTH_ADD ] = New_Data;
            if( pParse_Data->Payload[ LIBCRSF_LENGTH_ADD ] < LIBCRSF_PAYLOAD_SIZE
                    && pParse_Data->Payload[ LIBCRSF_LENGTH_ADD ] != 0 ) {
                pParse_Data->Status = CRSF_PARSE_RD_FRAME;
            } else {
                pParse_Data->Status = CRSF_PARSE_SYNC;
            }
            break;

        case CRSF_PARSE_RD_FRAME:
            if( pParse_Data->Cnt < LIBCRSF_PAYLOAD_SIZE ) {
                pParse_Data->Payload[ pParse_Data->Cnt++ ] = New_Data;
#ifdef LIBCRSF_ENABLE_PARSE_LESS_TYPE
                if( pParse_Data->Cnt >= LIBCRSF_TYPE_ADD ) {
                    if( pParse_Data->Payload[ LIBCRSF_TYPE_ADD ] == LIBCRSF_EX_PARAM_PING_DEVICE
                        || pParse_Data->Payload[ LIBCRSF_TYPE_ADD ] == LIBCRSF_EX_PARAM_DEVICE_INFO
//TODO: Add all the TYPE here, this can stop forwarding unuse frame to other device.
#ifdef LIBCRSF_ENABLE_COMMAND
                        || pParse_Data->Payload[ LIBCRSF_TYPE_ADD ] == LIBCRSF_CMD_FRAME
#endif
#if defined(LIBCRSF_ENABLE_FW_UPDATE_RESPONES) || defined(LIBCRSF_ENABLE_FW_UPDATE_REQUEST)
                        || pParse_Data->Payload[ LIBCRSF_TYPE_ADD ] == LIBCRSF_FW_UPDATE
#endif
                        ) {
                    } else {
                        pParse_Data->Status = CRSF_PARSE_SYNC;
                    }
                }
#endif
                if( pParse_Data->Cnt > pParse_Data->Payload[ LIBCRSF_LENGTH_ADD ] + LIBCRSF_HEADER_OFFSET ) {
                    if( pParse_Data->Payload[ pParse_Data->Payload[ LIBCRSF_LENGTH_ADD ] + LIBCRSF_HEADER_OFFSET ]
                            == libCRC8_Get_CRC_Arr( &pParse_Data->Payload[ LIBCRSF_TYPE_ADD ]
                            , pParse_Data->Payload[ LIBCRSF_LENGTH_ADD ] - LIBCRSF_CRC_SIZE, POLYNOM_1 ) ) {
                        pParse_Data->Status = CRSF_PARSE_SYNC;
                        return true;
                    }
                    pParse_Data->Status = CRSF_PARSE_SYNC;
                }
            } else {
                pParse_Data->Status = CRSF_PARSE_SYNC;
            }
            break;
    }
    return  false;
}

/* *****************************************************************************
 End of File
 */
