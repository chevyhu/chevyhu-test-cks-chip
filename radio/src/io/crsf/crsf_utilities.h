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

#ifndef __UTILITIES_H__    /* Guard against multiple inclusion */
#define __UTILITIES_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <limits.h>
#include <stdbool.h>
#include "crc8.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

#define BITMASK( b )                                        ( 1 << ( ( b ) % CHAR_BIT ) )
#define BITSLOT( b )                                        ( ( b ) / CHAR_BIT )
#define BITSET( a, b )                                      ( ( a )[ BITSLOT( b ) ] |= BITMASK( b ) )
#define BITCLEAR( a, b )                                    ( ( a )[ BITSLOT( b ) ] &= ~BITMASK( b ) )
#define BITTEST( a, b )                                     ( ( a )[ BITSLOT( b ) ] & BITMASK( b ) )
#define BITNSLOTS( nb )                                     ( ( nb + CHAR_BIT -1 ) / CHAR_BIT )

#define LIBUTIL_ARRAY_SIZE( array )   ( sizeof( array ) / sizeof( array[ 0 ] ) )

/* ******************** Public Functions and Procedures ******************** */
    extern uint32_t ( *libUtil_systemtime )( void );
#ifdef LIBCRSF_TEST_PRINT_ENABLE
    extern int ( *libUtil_printf )( const char *pArr, ... );

    #define LIBUTIL_PRINTF( fmt, ... )                                                          \
    if( libUtil_printf != NULL ) {                                                              \
        if( libUtil_systemtime != NULL ) {                                                      \
            libUtil_printf( "%9u ms %s(%u):" fmt                                                \
                , libUtil_systemtime(), __FUNCTION__ , __LINE__ , ##__VA_ARGS__ );              \
        } else {                                                                                \
            libUtil_printf( "%s(%u):" fmt                                                       \
                , __FUNCTION__ , __LINE__ , ##__VA_ARGS__ );                                    \
        }                                                                                       \
    }

    #define LIBUTIL_PRINTF_ARR( arr )                                                           \
    if( libUtil_printf != NULL ) {                                                              \
        if( libUtil_systemtime != NULL ) {                                                      \
            libUtil_printf( "%9u ms %s(%u) Lenght: %03d; BYTES:"                                \
                , libUtil_systemtime(), __FUNCTION__ , __LINE__, len );                         \
        } else {                                                                                \
            libUtil_printf( "%s(%u) Lenght: %03d; BYTES:"                                       \
                , __FUNCTION__ , __LINE__, len );                                               \
        }                                                                                       \
        uint32_t libutil_pointer = 0;                                                           \
        for( libutil_pointer = 0; libutil_pointer < len; libutil_pointer++ ) {                  \
            libUtil_printf( " %02x", arr[ libutil_pointer ] );                                  \
        }                                                                                       \
        libUtil_printf( "\r\n" );                                                               \
    }

    void LIBUTIL_DEFINE_LIBPRINT( int ( *printfunction )( const char *pArr, ... ) );
#endif

    #define LIBUTIL_MS2TICKS( ms )                          ( ms )
    void LIBUTIL_GET_SYS_TIME_POINTER( uint32_t ( *sysTime )( void ) );

/* *********************** Write Byte Array Functions ********************** */
void libUtil_ClearBuffer( uint8_t *arrayPointer, uint32_t *increment, uint32_t arraySize );
void libUtil_Write8( uint8_t *arrayPointer, uint32_t *increment, uint8_t inputData );
void libUtil_Write16( uint8_t *arrayPointer, uint32_t *increment, uint16_t inputData );
void libUtil_Write24( uint8_t *arrayPointer, uint32_t *increment, uint32_t inputData );
void libUtil_Write32( uint8_t *arrayPointer, uint32_t *increment, uint32_t inputData );
#if LIBRARY_ENABLE_64BITS
void libUtil_Write64( uint8_t *arrayPointer, uint32_t *increment, uint64_t inputData );
#endif
uint32_t libUtil_WriteString( uint8_t *arrayPointer, uint32_t *increment, char *inputStr, bool nullendbyte );
void libUtil_WriteBytes( uint8_t *out_array, uint32_t *increment, uint8_t *in_array, uint32_t needBytes );
void libUtil_WriteEnd_8( uint8_t *arrayPointer, uint32_t frameLengthAt, uint8_t cmdSize, uint8_t polynom );

/* *********************** Shift Byte Array Functions ********************** */
void libUtil_RightShift( uint8_t *arrayPointer, uint16_t start, uint16_t end, uint16_t right_shift );

/* *********************** Read Byte Array Functions *********************** */
uint8_t libUtil_Read8( uint8_t *arrayPointer, uint32_t *increment );
uint16_t libUtil_Read16( uint8_t *arrayPointer, uint32_t *increment );
uint32_t libUtil_Read24( uint8_t *arrayPointer, uint32_t *increment );
uint32_t libUtil_Read32( uint8_t *arrayPointer, uint32_t *increment );
#if LIBRARY_ENABLE_64BITS
uint64_t libUtil_Read64( uint8_t *arrayPointer, uint32_t *increment );
#endif
int8_t libUtil_ReadInt8( uint8_t *arrayPointer, uint32_t *increment );
int16_t libUtil_ReadInt16( uint8_t *arrayPointer, uint32_t *increment );
int32_t libUtil_ReadInt24( uint8_t *arrayPointer, uint32_t *increment );
int32_t libUtil_ReadInt32( uint8_t *arrayPointer, uint32_t *increment );
#if LIBRARY_ENABLE_64BITS
int64_t libUtil_ReadInt64( uint8_t *arrayPointer, uint32_t *increment );
#endif
float libUtil_ReadFloat( uint8_t *arrayPointer, uint32_t *increment, uint8_t decimalPoint );
uint32_t libUtil_ReadString( uint8_t *arrayPointer, uint32_t *increment, char *outputStr, bool skipNullChar );
void libUtil_ReadTextSelection( uint8_t *arrayPointer, uint32_t *increment, char *outArray, uint8_t sizeOfChar, uint8_t numberOfSlot );

/* ************************************************************************* */
void libUtil_hex_to_string( uint8_t *hex, uint8_t sizeof_hex, char *separator, bool capital, char *string );
uint8_t libUtil_ReverseUint8( uint8_t data );
uint16_t libUtil_ReverseUint16( uint16_t data );
uint32_t libUtil_ReverseUint32( uint32_t data );
#if LIBRARY_ENABLE_64BITS
uint64_t libUtil_ReverseUint64( uint64_t data );
#endif
/* ************************************************************************* */

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* __UTILITIES_H__ */

/* ****************************************************************************
 End of File
 */
