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


#include "crc8.h"


#define CRC8_FINISH_VALUE                   ( uint8_t ) 0x00        /* CRC8_Calc musst be called with this value to finish the calculation */
#define BYTE_SIZE                           ( uint8_t ) 0x08        /* Bit Size of one Byte */
#define MSB_SET                             ( uint8_t ) 0x80        /* MSB Bit is set in this byte */
#define MSB_FLAG_CLEAR                      ( uint8_t ) 0x00        /* MSB Bit flag is not set */
#define MSB_FLAG_SET                        ( uint8_t ) 0x01        /* MSB Bit flag is set*/


uint8_t EXP_libCRC8 = CRC8_RESET_VALUE;


void libCRC8_Reset( uint8_t *CRC8 ) {
    ( *CRC8 ) = CRC8_RESET_VALUE;                    
}

void libCRC8_Calc( uint8_t Data_In, uint8_t *CRC8, uint8_t Polynom ) {
    uint8_t Bit_Cnt;
    uint8_t MSB_Flag;

    for ( Bit_Cnt = 0; Bit_Cnt < BYTE_SIZE; Bit_Cnt++ ) {
        MSB_Flag = MSB_FLAG_CLEAR;
        if ( ( *CRC8 ) & MSB_SET ) MSB_Flag = MSB_FLAG_SET;
            ( *CRC8 ) <<= 1;
        if ( Data_In & MSB_SET )
            ( *CRC8 )++;
        Data_In <<= 1;
        if ( MSB_Flag == MSB_FLAG_SET )
            ( *CRC8 ) ^= Polynom;
    }
}

uint8_t Get_libCRC8( uint8_t *CRC8, uint8_t Polynom ) {
    /* Finisch CRC calculation */
    libCRC8_Calc( CRC8_FINISH_VALUE, CRC8, Polynom );   
    return ( *CRC8 );
}

/* *pArr needs to point to lenght byte of array */
void libCRC8_Add_MBUS_CRC( uint8_t *pArr, uint8_t polynom ) {
    uint8_t Length;

    Length = *pArr++ - 1;
    *( pArr + Length ) = libCRC8_Get_CRC_Arr( pArr, Length, polynom );
    //printf( "CRC at: %03d; CRC: %02x\r\n", Length, *( pArr + Length ) );
}

uint8_t libCRC8_Get_CRC_Arr( uint8_t *pArr, uint8_t Length, uint8_t polynom ) {
    uint8_t i;

    libCRC8_Reset( &EXP_libCRC8 );
    for( i = 0; i < Length; i++ ) {
        libCRC8_Calc( *( pArr + i ), &EXP_libCRC8, polynom );
    }
    return Get_libCRC8( &EXP_libCRC8, polynom );
}

