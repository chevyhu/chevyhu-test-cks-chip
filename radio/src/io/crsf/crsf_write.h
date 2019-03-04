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

#ifndef _CRSF_WRITE_H    /* Guard against multiple inclusion */
#define _CRSF_WRITE_H

#include "crsf.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef LIBCRSF_ENABLE_COMMAND
/* LIBCRSF_CMD_FRAME********************************************************* */
typedef enum {
    LIBCRSF_FC_CMD                              = 0x01,
    LIBCRSF_BT_CMD                              = 0x03,
    LIBCRSF_OSD_CMD                             = 0x05,
    LIBCRSF_VTX_CMD                             = 0x08,
    LIBCRSF_LED_CMD                             = 0x09,
    LIBCRSF_GENERAL_CMD                         = 0x0A,
    LIBCRSF_RC_RX_CMD                           = 0x10,
    LIBCRSF_WIFI_MODULE                         = 0x12,
    LIBCRSF_ACK                                 = 0xFF,
} libCrsf_Commands;

typedef enum {
    LIBCRSF_FC_FORCE_DISARM_SUBCMD              = 0x01,
    LIBCRSF_FC_SCALE_CHANNEL_SUBCMD             = 0x02,
} libCrsf_FC_Subcommands;

typedef enum {
    LIBCRSF_BT_RESET_SUBCMD                     = 0x01,
    LIBCRSF_BT_ENABLE_SUBCMD                    = 0x02,
    LIBCRSF_BT_ECHO_SUBCMD                      = 0x64,
} libCrsf_BT_Subcommands;

typedef enum {
    LIBCRSF_OSD_SEND_BUTTON_SUBCMD              = 0x01,
} libCrsf_OSD_Subcommands;

typedef enum {
    LIBCRSF_VTX_CHANGE_CHANNEL_SUBCMD           = 0x01,
    LIBCRSF_VTX_CHANGE_FREQ_SUBCMD              = 0x02,
    LIBCRSF_VTX_CHANGE_POWER_SUBCMD             = 0x03,
    LIBCRSF_VTX_CHANGE_PITMODE_SUBCMD           = 0x04,
    LIBCRSF_VTX_POWER_UP_FROM_PITMODE_SUBCMD    = 0x05,
} libCrsf_VTX_Subcommands;

typedef enum {
    LIBCRSF_LED_SET_DEFAULT_SUBCMD              = 0x01,
    LIBCRSF_LED_OVERRIDE_COLOR_SUBCMD           = 0x02,
    LIBCRSF_LED_OVERRIDE_PULSE_SUBCMD           = 0x03,
    LIBCRSF_LED_OVERRIDE_BLINK_SUBCMD           = 0x04,
    LIBCRSF_LED_OVERRIDE_SHIFT_SUBCMD           = 0x05,
} libCrsf_LED_Subcommands;

typedef enum {
    LIBCRSF_RC_RX_SET_TO_BIND_MODE_SUBCMD       = 0x01,
    LIBCRSF_RC_RX_CANCEL_BIND_MODE_SUBCMD       = 0x02,
    LIBCRSF_RC_RX_MODEL_SELECTION_SUBCMD        = 0x05,
    LIBCRSF_RC_RX_CURRENT_MODEL_SELECTION_SUBCMD= 0x06,
} libCrsf_RC_RX_Subcommands;

typedef enum {
    LIBCRSF_GENERAL_START_BOOTLOADER_SUBCMD     = 0x0A,
    LIBCRSF_GENERAL_ERASE_MEMORY_SUBCMD         = 0x0B,
    LIBCRSF_GENERALSOFTWARE_PRODUCT_KEY_SUBCMD  = 0x60,
    LIBCRSF_GENERALPRODUCT_FEEDBACK_SUBCMD      = 0x61,
} libCrsf_GENERAL_Subcommands;

typedef enum {
    LIBCRSF_WIFI_FIRMWARE_FILE_URL_SUBCMD       = 0x01,
} libCrsf_WIFI_Subcommands;

typedef struct {
    libCrsf_Commands command_id;
    uint8_t sub_command_id;
    uint8_t *payload;
} libCrsf_command_s;
#endif

#ifdef LIBCRSF_ENABLE_LOGGING
#endif

#if defined(LIBCRSF_ENABLE_FW_UPDATE_RESPONES) || defined(LIBCRSF_ENABLE_FW_UPDATE_REQUEST)
#define LIBCRSF_MAX_FWUPDATE_BUFFER_SIZE        48
#define LIBCRSF_MAX_FWUPDATE_CHUNK              128
typedef union {
    struct {
        uint8_t config;
        uint32_t hardware_id;
        uint16_t firmware_id;
        uint32_t firmware_length;
        uint32_t start_add;
        uint32_t end_add;
        uint32_t CRC32;
        uint8_t percentage;
    } info;
    struct {
        uint32_t start_add;
        uint32_t end_add;
        uint8_t chunk_id;
        uint8_t chunk_total;
        uint8_t length;
        uint8_t data[ LIBCRSF_MAX_FWUPDATE_BUFFER_SIZE ];
    } fw_data;
} libCrsf_FwUpdate_Data_u;

typedef enum {
    LIBCRSF_FW_UPDATE_HEADER                    = 0x01,
    LIBCRSF_FW_UPDATE_ASK_FOR_BYTES             = 0x02,
    LIBCRSF_FW_UPDATE_FW_DATA                   = 0x03,
    LIBCRSF_FW_UPDATE_RANGE_DONE_WAITING        = 0x04,
    LIBCRSF_FW_UPDATE_RANGE_REPORT              = 0x05,
    LIBCRSF_FW_UPDATE_DOWNLOAD_FINISHED         = 0x06,
    LIBCRSF_FW_UPDATE_ERASE_FLASH_PROGRESS      = 0x07,
    LIBCRSF_FW_UPDATE_START_APPLICATION         = 0x08,
    LIBCRSF_FW_UPDATE_TEST                      = 0x10,
} libCrsf_Fw_Update;
#endif

/* Checking and Setup Function*********************************************** */
bool libCrsf_checkif_devicecalled( uint8_t *p_arr, bool General_Call );

/* Write Command************************************************************* */
void libCrsf_crsfwrite( uint8_t frameType, uint8_t *p_arr, ... );

#ifdef LIBCRSF_ENABLE_TELEMETRY
/* Broadcast Header Telemetry************************************************ */
void libCrsf_packgps( uint8_t *p_arr, uint32_t *i
        , libCrsf_gps_s *p_struct );
void libCrsf_packgpstime( uint8_t *p_arr, uint32_t *i
        , libCrsf_gps_time_s *p_struct );
void libCrsf_packgpsextended( uint8_t *p_arr, uint32_t *i
        , libCrsf_gps_extended_s *p_struct );
void libCrsf_packbattery( uint8_t *p_arr, uint32_t *i
        , libCrsf_battery_s *p_struct );
void libCrsf_packheartbeat( uint8_t *p_arr, uint32_t *i
        , libCrsf_heartbeat_s *p_struct );
void libCrsf_packlink_statistics( uint8_t *p_arr, uint32_t *i
        , libCrsf_link_statistics_s *p_struct );
void libCrsf_packattitude( uint8_t *p_arr, uint32_t *i
        , libCrsf_attitude_s *p_struct );
void libCrsf_packmavlink_fc( uint8_t *p_arr, uint32_t *i
        , libCrsf_mavlink_fc_s *p_struct );
#endif

/* Extended Header Frames**************************************************** */
void libCrsf_packpingcommand( uint8_t *p_arr, uint32_t *i );
void libCrsf_packdeviceinfo( uint8_t *p_arr, uint32_t *i );

#ifdef LIBCRSF_ENABLE_PARAMETER
void libCrsf_packreadsettings( uint8_t *p_arr, uint32_t *i
        , uint8_t device, uint8_t parameter, uint8_t chunk );
void libCrsf_packparamsettingentry( uint8_t *p_arr, uint32_t *i
        , uint8_t param_num, uint8_t chunk_num );
void libCrsf_packgetchildcommand( uint8_t *p_arr, uint32_t *i
        , uint8_t device, uint8_t parent_id );
void libCrsf_packreturnchildlist( uint8_t *p_arr, uint32_t *i
        , uint8_t parent_id );
#endif

#ifdef LIBCRSF_ENABLE_LOGGING
#endif

#if defined(LIBCRSF_ENABLE_FW_UPDATE_RESPONES) || defined(LIBCRSF_ENABLE_FW_UPDATE_REQUEST)
void libCrsf_packfwupdate( uint8_t *p_arr, uint32_t *i
        , uint8_t target_device, uint8_t fwupdate_command_id, libCrsf_FwUpdate_Data_u *fw_info );
#endif

#ifdef LIBCRSF_ENABLE_RC
/* RX Data Frames************************************************************ */
void libCrsf_packrxdata( uint8_t *p_arr, uint32_t *i
        , uint8_t total_ppmchannels, uint16_t ppmdata[] );
#endif

#ifdef LIBCRSF_ENABLE_COMMAND
/* CRSF Command************************************************************** */
void libCrsf_packcommandframe( uint8_t *p_arr, uint32_t *i
        , uint8_t target_address, libCrsf_command_s *command );

void libCrsf_pack_fc_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_FC_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_bt_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_BT_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_osd_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_OSD_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_vtx_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_VTX_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_led_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_LED_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_general_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_GENERAL_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_rc_rx_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_RC_RX_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_wifi_sub_command( uint8_t *p_arr, uint32_t *i, libCrsf_WIFI_Subcommands sub_command_id, uint8_t *payload );
void libCrsf_pack_ack_sub_command( uint8_t *p_arr, uint32_t *i, uint8_t *payload );
#endif

/* ************************************************************************** */

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _CRSF_WRITE_H */

/* *****************************************************************************
 End of File
 */
