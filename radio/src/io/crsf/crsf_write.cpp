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
#include "crsf_write.h"
#include "crsf_utilities.h"

/* EXTERNAL USE************************************************************** */
extern uint8_t libCrsf_MySlaveAddress;
extern char *libCrsf_MyDeviceName;
extern uint32_t libCrsf_MySerialNo;
extern uint32_t libCrsf_MyHwID;
extern uint32_t libCrsf_MyFwID;

uint8_t libCrsf_MyparamListSize = 0;
uint8_t libCrsf_senderaddress;

/* Checking and Setup Function*********************************************** */
bool libCrsf_checkif_devicecalled( uint8_t *p_arr, bool General_Call ) {
    bool Feedback = false;

    if ( ( General_Call
         && *( p_arr + LIBCRSF_EXT_HEAD_DST_ADD ) == LIBCRSF_BROADCAST_ADD )
       || *( p_arr + LIBCRSF_EXT_HEAD_DST_ADD ) == libCrsf_MySlaveAddress ) {
        Feedback = true;
        libCrsf_senderaddress = *( p_arr + LIBCRSF_EXT_HEAD_ORG_ADD );
    }
    return Feedback;
}

/* Write Command************************************************************* */
void libCrsf_crsfwrite( uint8_t frameType, uint8_t *p_arr, ... ) {
    uint32_t i = 1; /* one byte space for length */

    va_list argp;
    va_start( argp, p_arr );

    libUtil_Write8( p_arr, &i, frameType );
    switch ( frameType ) {
#ifdef LIBCRSF_ENABLE_TELEMETRY
        case LIBCRSF_BF_GPS:
            libCrsf_packgps( p_arr, &i
                , ( libCrsf_gps_s *) va_arg( argp, int * ) );
            break;
        case LIBCRSF_BF_GPS_TIME:
            libCrsf_packgpstime( p_arr, &i
                , ( libCrsf_gps_time_s *) va_arg( argp, int * ) );
            break;
        case LIBCRSF_BF_GPS_EXTENDED:
            libCrsf_packgpsextended( p_arr, &i
                , ( libCrsf_gps_extended_s *) va_arg( argp, int * ) );
            break;
        case LIBCRSF_BF_BATTERY_SENSOR:
            libCrsf_packbattery( p_arr, &i
                , ( libCrsf_battery_s *) va_arg( argp, int * ) );
            break;
        case LIBCRSF_BF_HEARTBEAT:
            libCrsf_packheartbeat( p_arr, &i
                , ( libCrsf_heartbeat_s *) va_arg( argp, int * ) );
            break;
        case LIBCRSF_BF_VIDEO_TRANSMITTER:
            break;
        case LIBCRSF_BF_LINK_STATISTICS:
            libCrsf_packlink_statistics( p_arr, &i
                , ( libCrsf_link_statistics_s *) va_arg( argp, int * ) );
            break;
        case LIBCRSF_BF_ATTITUDE:
            libCrsf_packattitude( p_arr, &i
                , ( libCrsf_attitude_s *) va_arg( argp, int * ) );
            break;
        case LIBCRSF_BF_FLIGHT_MODE_TEXT_BASED:
            break;
        case LIBCRSF_BF_MAVLINK_FC_ADD_DATA:
            libCrsf_packmavlink_fc( p_arr, &i
                , ( libCrsf_mavlink_fc_s *) va_arg( argp, int * ) );
            break;
#endif
#ifdef LIBCRSF_ENABLE_RC
        case LIBCRSF_BF_RC_CHANNELS_PACK:
            libCrsf_packrxdata( p_arr, &i
                , ( uint8_t ) va_arg( argp, int ), ( uint16_t *) va_arg( argp, int * ) );
            break;
#endif
        case LIBCRSF_EX_PARAM_PING_DEVICE:
            libCrsf_packpingcommand( p_arr, &i );
            break;
        case LIBCRSF_EX_PARAM_DEVICE_INFO:
            libCrsf_packdeviceinfo( p_arr, &i );
            break;
#ifdef LIBCRSF_ENABLE_PARAMETER
        case LIBCRSF_EX_PARAM_SETTING_ENTRY:
            libCrsf_packparamsettingentry( p_arr, &i
                , ( uint8_t ) va_arg( argp, int ), ( uint8_t ) va_arg( argp, int ) );
            break;
        case LIBCRSF_EX_PARAM_SETTING_READ:
            libCrsf_packreadsettings( p_arr, &i
                , ( uint8_t ) va_arg( argp, int ), ( uint8_t ) va_arg( argp, int ), ( uint8_t ) va_arg( argp, int ) );
            break;
        case LIBCRSF_EX_PARAM_VALUE_WRITE:
            //TODO....
            break;
        case LIBCRSF_EX_PARAM_GET_CHILD_LIST:
            libCrsf_packgetchildcommand( p_arr, &i
                    , ( uint8_t ) va_arg( argp, int ), ( uint8_t ) va_arg( argp, int ) );
            break;
        case LIBCRSF_EX_PARAM_RETURN_CHILD_LIST:
            libCrsf_packreturnchildlist( p_arr, &i
                    , ( uint8_t ) va_arg( argp, int ) );
            break;
#endif
#ifdef LIBCRSF_ENABLE_COMMAND
        case LIBCRSF_CMD_FRAME:
            libCrsf_packcommandframe( p_arr, &i
                    , ( uint8_t ) va_arg( argp, int ), ( libCrsf_command_s *) va_arg( argp, int * ) );
            break;
#endif
#ifdef LIBCRSF_ENABLE_LOGGING
        case LIBCRSF_LOGGING_FRAME:
            break;
#endif
#if defined(LIBCRSF_ENABLE_FW_UPDATE_RESPONES) || defined(LIBCRSF_ENABLE_FW_UPDATE_REQUEST)
        case LIBCRSF_FW_UPDATE:
            libCrsf_packfwupdate( p_arr, &i
                    , ( uint8_t ) va_arg( argp, int ), ( uint8_t ) va_arg( argp, int ), ( libCrsf_FwUpdate_Data_u *) va_arg( argp, int * ) );
            break;
#endif
        case LIBCRSF_KISS_FC_1:
        case LIBCRSF_KISS_FC_2:

        case LIBCRSF_BETAFLIGHT_FC_1:
        case LIBCRSF_BETAFLIGHT_FC_2:
        case LIBCRSF_BETAFLIGHT_FC_3:
            break;
        default:
            break;
    }
    va_end( argp );
#ifdef LIBCRSF_TEST_PRINT_ENABLE
    LIBUTIL_PRINTF("CRSF Write Bytes Count: %d \r\n", i);
#endif
    libUtil_WriteEnd_8( p_arr, 0, i, POLYNOM_1 );
}

/* Broadcast Header Telemetry************************************************ */
#ifdef LIBCRSF_ENABLE_TELEMETRY
void libCrsf_packgps( uint8_t *p_arr, uint32_t *i
        , libCrsf_gps_s *p_struct ) {
    libUtil_Write32( p_arr, i, p_struct->latitude );
    libUtil_Write32( p_arr, i, p_struct->longitude );
    libUtil_Write16( p_arr, i, p_struct->groundspeed );
    libUtil_Write16( p_arr, i, p_struct->heading );
    libUtil_Write16( p_arr, i, p_struct->altitude );
    libUtil_Write8( p_arr, i, p_struct->satellites );
}

void libCrsf_packgpstime( uint8_t *p_arr, uint32_t *i
        , libCrsf_gps_time_s *p_struct ) {
    libUtil_Write16( p_arr, i, p_struct->year );
    libUtil_Write8( p_arr, i, p_struct->month );
    libUtil_Write8( p_arr, i, p_struct->day );
    libUtil_Write8( p_arr, i, p_struct->hour );
    libUtil_Write8( p_arr, i, p_struct->minute );
    libUtil_Write8( p_arr, i, p_struct->second );
    libUtil_Write16( p_arr, i, p_struct->millisecond );
}

void libCrsf_packgpsextended( uint8_t *p_arr, uint32_t *i
        , libCrsf_gps_extended_s *p_struct ) {
    libUtil_Write8(p_arr, i, p_struct->fix_type );
    libUtil_Write16(p_arr, i, (uint16_t) p_struct->n_speed );
    libUtil_Write16(p_arr, i, (uint16_t) p_struct->e_speed );
    libUtil_Write16(p_arr, i, (uint16_t) p_struct->v_speed );
    libUtil_Write16(p_arr, i, (uint16_t) p_struct->h_speed_acc );
    libUtil_Write16(p_arr, i, (uint16_t) p_struct->track_acc );
    libUtil_Write16(p_arr, i, (uint16_t) p_struct->alt_ellipsoid );
    libUtil_Write16(p_arr, i, (uint16_t) p_struct->h_acc );
    libUtil_Write16(p_arr, i, (uint16_t) p_struct->v_acc );
    libUtil_Write8(p_arr, i, 0x00 );  /* fill reserved byte */
    libUtil_Write8(p_arr, i, p_struct->hDOP );
    libUtil_Write8(p_arr, i, p_struct->vDOP );
}

void libCrsf_packbattery( uint8_t *p_arr, uint32_t *i
        , libCrsf_battery_s *p_struct ) {
    libUtil_Write16(p_arr, i, p_struct->voltage );
    libUtil_Write16(p_arr, i, p_struct->current );
    libUtil_Write24(p_arr, i, p_struct->capacity_used );
    libUtil_Write8(p_arr, i, p_struct->remaining );
}

void libCrsf_packheartbeat( uint8_t *p_arr, uint32_t *i
        , libCrsf_heartbeat_s *p_struct ) {
    libUtil_Write8(p_arr, i, p_struct->origin_add );
}

void libCrsf_packlink_statistics( uint8_t *p_arr, uint32_t *i
        , libCrsf_link_statistics_s *p_struct ) {
    libUtil_Write8( p_arr, i, p_struct->up_rssi_ant1);
    libUtil_Write8( p_arr, i, p_struct->up_rssi_ant2);
    libUtil_Write8( p_arr, i, p_struct->up_link_quality);
    libUtil_Write8( p_arr, i, (uint8_t) p_struct->up_snr);
    libUtil_Write8( p_arr, i, p_struct->active_antenna);
    libUtil_Write8( p_arr, i, p_struct->rf_profile);
    libUtil_Write8( p_arr, i, p_struct->up_rf_power);
    libUtil_Write8( p_arr, i, p_struct->down_rssi);
    libUtil_Write8( p_arr, i, p_struct->down_link_quality);
    libUtil_Write8( p_arr, i, (uint8_t) p_struct->down_snr);
}

void libCrsf_packattitude( uint8_t *p_arr, uint32_t *i
        , libCrsf_attitude_s *p_struct ) {
    libUtil_Write16(p_arr, i, (uint16_t)p_struct->pitch);
    libUtil_Write16(p_arr, i, (uint16_t)p_struct->roll);
    libUtil_Write16(p_arr, i, (uint16_t)p_struct->yaw);
}

void libCrsf_packmavlink_fc( uint8_t *p_arr, uint32_t *i
        , libCrsf_mavlink_fc_s *p_struct ) {
    libUtil_Write16(p_arr, i, (uint16_t)p_struct->airspeed);
    libUtil_Write8(p_arr, i, p_struct->base_mode);
    libUtil_Write32(p_arr, i, p_struct->custom_mode);
    libUtil_Write8(p_arr, i, p_struct->autopilot_type);
    libUtil_Write8(p_arr, i, p_struct->firmware_type);
}
#endif

/* Extended Header Frames**************************************************** */
void libCrsf_packpingcommand( uint8_t *p_arr, uint32_t *i ) {
    libUtil_Write8( p_arr, i, LIBCRSF_BROADCAST_ADD );
    libUtil_Write8( p_arr, i, libCrsf_MySlaveAddress );
}

void libCrsf_packdeviceinfo( uint8_t *p_arr, uint32_t *i ) {
    libUtil_Write8( p_arr, i, libCrsf_senderaddress );
    libUtil_Write8( p_arr, i, libCrsf_MySlaveAddress );
    libUtil_WriteString( p_arr, i, libCrsf_MyDeviceName, true );
    libUtil_Write32( p_arr, i, libCrsf_MySerialNo );
    libUtil_Write32( p_arr, i, libCrsf_MyHwID );
    libUtil_Write32( p_arr, i, libCrsf_MyFwID );
    if( libCrsf_MyparamListSize <= 2 ) {
        libUtil_Write8( p_arr, i, 0x00 );
    } else {
        libUtil_Write8( p_arr, i, libCrsf_MyparamListSize - 2 );
    }
    libUtil_Write8( p_arr, i, LIBCRSF_PARAM_VERSION_NUMBER );
}

#ifdef LIBCRSF_ENABLE_PARAMETER
void libCrsf_packreadsettings( uint8_t *p_arr, uint32_t *i
        , uint8_t device, uint8_t parameter, uint8_t chunk ) {
    libUtil_Write8( p_arr, i, device );
    libUtil_Write8( p_arr, i, libCrsf_MySlaveAddress );
    libUtil_Write8( p_arr, i, parameter );
    libUtil_Write8( p_arr, i, chunk );
}

void libCrsf_packparamsettingentry( uint8_t *p_arr, uint32_t *i
        , uint8_t param_num, uint8_t chunk_num ) {
    libUtil_Write8( p_arr, i, libCrsf_senderaddress );
    libUtil_Write8( p_arr, i, libCrsf_MySlaveAddress );

    libCrsf_write_packedparam( param_num, chunk_num, p_arr, i );
}

void libCrsf_packgetchildcommand( uint8_t *p_arr, uint32_t *i
        , uint8_t device, uint8_t parent_id ) {
    libUtil_Write8( p_arr, i, device );
    libUtil_Write8( p_arr, i, libCrsf_MySlaveAddress );
    libUtil_Write8( p_arr, i, parent_id );
}

void libCrsf_packreturnchildlist( uint8_t *p_arr, uint32_t *i
        , uint8_t parent_id ) {
    uint8_t child_id_list[LIBCRSF_PARAMETER_LIST_SIZE] = { 0 };
    libUtil_Write8( p_arr, i, libCrsf_senderaddress );
    libUtil_Write8( p_arr, i, libCrsf_MySlaveAddress );
    libUtil_Write8( p_arr, i, parent_id );
    libUtil_Write8( p_arr, i, libCrsf_parameterparent_id( parent_id ) );
    uint8_t count = libCrsf_getchild_infolist( parent_id, child_id_list, NULL, NULL );
    libUtil_WriteBytes( p_arr, i, child_id_list, count );
}
#endif

#if defined(LIBCRSF_ENABLE_FW_UPDATE_RESPONES) || defined(LIBCRSF_ENABLE_FW_UPDATE_REQUEST)
void libCrsf_packfwupdate( uint8_t *p_arr, uint32_t *i
        , uint8_t target_device, uint8_t fwupdate_command_id, libCrsf_FwUpdate_Data_u *fw_info ) {
    libUtil_Write8( p_arr, i, target_device );
    libUtil_Write8( p_arr, i, libCrsf_MySlaveAddress );
    libUtil_Write8( p_arr, i, fwupdate_command_id );
    switch( fwupdate_command_id ) {
#ifdef LIBCRSF_ENABLE_FW_UPDATE_RESPONES
        case LIBCRSF_FW_UPDATE_HEADER:
            libUtil_Write8( p_arr, i, fw_info->info.config );
            libUtil_Write32( p_arr, i, fw_info->info.hardware_id );
            libUtil_Write16( p_arr, i, fw_info->info.firmware_id );
            libUtil_Write32( p_arr, i, fw_info->info.firmware_length );
            libUtil_Write32( p_arr, i, fw_info->info.CRC32 );
            break;
        case LIBCRSF_FW_UPDATE_FW_DATA:
            libUtil_Write8( p_arr, i, fw_info->fw_data.chunk_id );
            libUtil_Write8( p_arr, i, fw_info->fw_data.chunk_total );
            libUtil_WriteBytes( p_arr, i, fw_info->fw_data.data, fw_info->fw_data.length );
            break;
        case LIBCRSF_FW_UPDATE_RANGE_DONE_WAITING:
            break;
        case LIBCRSF_FW_UPDATE_START_APPLICATION:
            break;
#endif
#ifdef LIBCRSF_ENABLE_FW_UPDATE_REQUEST
        case LIBCRSF_FW_UPDATE_ASK_FOR_BYTES:
            libUtil_Write32( p_arr, i, fw_info->info.start_add );
            libUtil_Write32( p_arr, i, fw_info->info.end_add );
            libUtil_Write32( p_arr, i, fw_info->info.CRC32 ); //maybe use sha or md5 laster
            break;
        case LIBCRSF_FW_UPDATE_RANGE_REPORT:
            libUtil_WriteBytes( p_arr, i, fw_info->fw_data.data, fw_info->fw_data.length );
            libUtil_Write32( p_arr, i, fw_info->fw_data.start_add );
            libUtil_Write32( p_arr, i, fw_info->fw_data.end_add );
            break;
        case LIBCRSF_FW_UPDATE_ERASE_FLASH_PROGRESS:
            libUtil_Write8( p_arr, i, fw_info->info.percentage );
            break;
#endif
        default:
            break;
    }
}
#endif

#ifdef LIBCRSF_ENABLE_RC
/* RX Data Frames************************************************************ */
void libCrsf_packrxdata( uint8_t *p_arr, uint32_t *i
        , uint8_t total_ppmchannels, uint16_t ppmdata[] ) {
    uint8_t channel;
    int32_t values;
    uint8_t currentByte = 0;
    uint8_t bits = 8; /* 8 bit space in one byte */
    uint8_t bitsleft;

    if( total_ppmchannels > LIBCRSF_RC_MAX_NUMBER_OF_CHANNEL ) {
        total_ppmchannels = LIBCRSF_RC_MAX_NUMBER_OF_CHANNEL;
    }
    libUtil_Write8( p_arr, i, LIBCRSF_BF_RC_CHANNELS_PACK );
    for( channel = 0; channel < total_ppmchannels; channel++ ) {
        values = LIBCRSF_PPM_TO_CRSF_RC( ppmdata[ channel ] );

        if( values > LIBCRSF_RC_MAX ) {
            values = LIBCRSF_RC_MAX;
        }
        if( values < LIBCRSF_RC_MIN ) {
            values = LIBCRSF_RC_MIN;
        }

        bitsleft = LIBCRSF_RC_RESOLUTION;
        while( bitsleft > 0 ) {
            currentByte |= (uint8_t) ( values << ( 8 - bits ));
            values >>= bits;
            if( bitsleft  > bits ) {
                bitsleft -= bits;
                libUtil_Write8( p_arr, i, currentByte );
                currentByte = 0;
                bits = 8;     /* 8 bit space in one byte */
            } else {
                bits -= bitsleft;
                bitsleft = 0;
            }
        }
    }
    if(currentByte != 0)
        libUtil_Write8( p_arr, i, currentByte );
}
#endif

#ifdef LIBCRSF_ENABLE_COMMAND
/* CRSF Command************************************************************** */
void libCrsf_packcommandframe( uint8_t *p_arr, uint32_t *i
      , uint8_t target_address, libCrsf_command_s *command ) {
    libUtil_Write8( p_arr, i, target_address );
    libUtil_Write8( p_arr, i, libCrsf_MySlaveAddress );
    libUtil_Write8( p_arr, i, command->command_id );
    libUtil_Write8( p_arr, i, command->sub_command_id );
    if( command->payload != NULL ) {
        switch( command->command_id ) {
            case LIBCRSF_FC_CMD:
                libCrsf_pack_fc_sub_command( p_arr, i, ( libCrsf_FC_Subcommands ) command->sub_command_id, command->payload );
                break;
            case LIBCRSF_BT_CMD:
                libCrsf_pack_bt_sub_command( p_arr, i, ( libCrsf_BT_Subcommands ) command->sub_command_id, command->payload );
                break;
            case LIBCRSF_OSD_CMD:
                libCrsf_pack_osd_sub_command( p_arr, i, ( libCrsf_OSD_Subcommands ) command->sub_command_id, command->payload );
                break;
            case LIBCRSF_VTX_CMD:
                libCrsf_pack_vtx_sub_command( p_arr, i, ( libCrsf_VTX_Subcommands ) command->sub_command_id, command->payload );
                break;
            case LIBCRSF_LED_CMD:
                libCrsf_pack_led_sub_command( p_arr, i, ( libCrsf_LED_Subcommands ) command->sub_command_id, command->payload );
                break;
            case LIBCRSF_GENERAL_CMD:
                libCrsf_pack_general_sub_command( p_arr, i, ( libCrsf_GENERAL_Subcommands ) command->sub_command_id, command->payload );
                break;
            case LIBCRSF_RC_RX_CMD:
                libCrsf_pack_rc_rx_sub_command( p_arr, i, ( libCrsf_RC_RX_Subcommands ) command->sub_command_id, command->payload );
                break;
            case LIBCRSF_WIFI_MODULE:
                libCrsf_pack_wifi_sub_command( p_arr, i, ( libCrsf_WIFI_Subcommands ) command->sub_command_id, command->payload );
                break;
            case LIBCRSF_ACK:
                libCrsf_pack_ack_sub_command( p_arr, i, command->payload );
                break;
            default:
                break;
        }
    }
    libUtil_WriteEnd_8( p_arr, 0, *i, POLYNOM_2 );
    ( *i )++;
}

void libCrsf_pack_fc_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_FC_Subcommands sub_command_id, uint8_t *payload ) {

}

void libCrsf_pack_bt_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_BT_Subcommands sub_command_id, uint8_t *payload ) {

}

void libCrsf_pack_osd_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_OSD_Subcommands sub_command_id, uint8_t *payload ) {

}

void libCrsf_pack_vtx_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_VTX_Subcommands sub_command_id, uint8_t *payload ) {

}

void libCrsf_pack_led_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_LED_Subcommands sub_command_id, uint8_t *payload ) {

}

void libCrsf_pack_general_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_GENERAL_Subcommands sub_command_id, uint8_t *payload ) {
    switch( sub_command_id ) {
        case LIBCRSF_GENERAL_START_BOOTLOADER_SUBCMD:
            break;
        case LIBCRSF_GENERAL_ERASE_MEMORY_SUBCMD:
            break;
        case LIBCRSF_GENERALSOFTWARE_PRODUCT_KEY_SUBCMD:
            break;
        case LIBCRSF_GENERALPRODUCT_FEEDBACK_SUBCMD:
            break;
        default:
            break;
    }
}

void libCrsf_pack_rc_rx_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_RC_RX_Subcommands sub_command_id, uint8_t *payload ) {
    switch( sub_command_id ) {
        case LIBCRSF_RC_RX_SET_TO_BIND_MODE_SUBCMD:
            break;
        case LIBCRSF_RC_RX_CANCEL_BIND_MODE_SUBCMD:
            break;
        case LIBCRSF_RC_RX_MODEL_SELECTION_SUBCMD:
            break;
        case LIBCRSF_RC_RX_CURRENT_MODEL_SELECTION_SUBCMD:
            break;
        default:
            break;
    }
}

void libCrsf_pack_wifi_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_WIFI_Subcommands sub_command_id, uint8_t *payload ) {
    switch( sub_command_id ) {
        case LIBCRSF_WIFI_FIRMWARE_FILE_URL_SUBCMD:
            libUtil_WriteString( p_arr, i, libCrsf_MyDeviceName, true );
            break;
    }

}

void libCrsf_pack_ack_sub_command( uint8_t *p_arr, uint32_t *i, uint8_t *payload ) {

}

/* ************************************************************************** */
#endif

/* *****************************************************************************
 End of File
 */
