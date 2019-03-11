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

/* ******************************** Includes ******************************* */
#include "crsf_utilities.h"

/* ************************************************************************* */

uint32_t ( *libUtil_systemtime )( void );
#ifdef LIBCRSF_TEST_PRINT_ENABLE
int ( *libUtil_printf )( const char *pArr, ... );

void LIBUTIL_DEFINE_LIBPRINT( int (*printfunction)(const char *pArr, ... ) ) {
    libUtil_printf = printfunction;
}
#endif

void LIBUTIL_GET_SYS_TIME_POINTER( uint32_t ( *sysTime )( void) ) {
    libUtil_systemtime = sysTime;
}

/* *********************** Write Byte Array Functions ********************** */
void libUtil_ClearBuffer( uint8_t *arrayPointer, uint32_t *increment, uint32_t arraySize ) {
    memset( arrayPointer, 0, arraySize );
    *increment = 0;
}

void libUtil_Write8( uint8_t *arrayPointer, uint32_t *increment, uint8_t inputData ) {
    *( arrayPointer + ( *increment )++ ) = inputData;
}

void libUtil_Write16( uint8_t *arrayPointer, uint32_t *increment, uint16_t inputData ) {
    libUtil_Write8( arrayPointer, increment, ( inputData >> 8 ) & 0xFF );
    libUtil_Write8( arrayPointer, increment, inputData & 0xFF );
}

void libUtil_Write24( uint8_t *arrayPointer, uint32_t *increment, uint32_t inputData ) {
    libUtil_Write8( arrayPointer, increment, ( inputData >> 16 ) & 0xFF );
    libUtil_Write8( arrayPointer, increment, ( inputData >> 8 ) & 0xFF );
    libUtil_Write8( arrayPointer, increment, inputData & 0xFF );
}

void libUtil_Write32( uint8_t *arrayPointer, uint32_t *increment, uint32_t inputData ) {
    libUtil_Write16( arrayPointer, increment, ( inputData >> 16 ) & 0xFFFF );
    libUtil_Write16( arrayPointer, increment, inputData & 0xFFFF );
}

#if LIBRARY_ENABLE_64BITS
void libUtil_Write64( uint8_t *arrayPointer, uint32_t *increment, uint64_t inputData ) {
    libUtil_Write32( arrayPointer, increment, ( inputData >> 32 ) & 0xFFFFFFFF );
    libUtil_Write32( arrayPointer, increment, inputData & 0xFFFFFFFF );
}
#endif

uint32_t libUtil_WriteString( uint8_t *arrayPointer, uint32_t *increment, char *inputStr, bool nullendbyte ) {
    uint32_t i = 0;
    while( inputStr[ i ] != '\0' ) {
        libUtil_Write8( arrayPointer, increment, inputStr[ i++ ] );
    }
    if( nullendbyte ) {
        libUtil_Write8( arrayPointer, increment, 0 );
    }
    return i;
}

void libUtil_WriteBytes( uint8_t *out_array, uint32_t *increment, uint8_t *in_array, uint32_t needBytes ) {
    uint32_t i = 0;
    for( i = 0; i < needBytes; i++ ) {
        libUtil_Write8( out_array, increment, in_array[i] );
    }
}

void libUtil_WriteEnd_8( uint8_t *arrayPointer, uint32_t frameLengthAt, uint8_t cmdSize, uint8_t polynom ) {
    libUtil_Write8( arrayPointer, &frameLengthAt, cmdSize );
    libCRC8_Add_MBUS_CRC( arrayPointer, polynom );
}

/* *********************** Shift Byte Array Functions ********************** */
void libUtil_RightShift( uint8_t *arrayPointer, uint16_t start, uint16_t end, uint16_t right_shift ) {
    uint16_t i;
    for( i = end; i <= start; i-- ) {
        *( arrayPointer + ( i +  right_shift ) ) = *( arrayPointer + i );
    }
}

/* *********************** Read Byte Array Functions *********************** */
uint8_t libUtil_Read8( uint8_t *arrayPointer, uint32_t *increment ) {
    return arrayPointer[ (*increment)++ ];
}

uint16_t libUtil_Read16( uint8_t *arrayPointer, uint32_t *increment ) {
    return ( ( ( ( uint16_t ) libUtil_Read8( arrayPointer, increment ) ) << 8 ) | libUtil_Read8( arrayPointer, increment ) );
}

uint32_t libUtil_Read24( uint8_t *arrayPointer, uint32_t *increment ) {
    return ( ( ( ( uint32_t ) libUtil_Read8( arrayPointer, increment ) ) << 16 ) | ( ( (uint32_t) libUtil_Read8( arrayPointer, increment ) ) << 8 ) | libUtil_Read8( arrayPointer, increment ) );
}

uint32_t libUtil_Read32( uint8_t *arrayPointer, uint32_t *increment ) {
    return ( ( ( ( uint32_t ) libUtil_Read16( arrayPointer, increment ) ) << 16 ) | libUtil_Read16( arrayPointer, increment ) );
}

#if LIBRARY_ENABLE_64BITS
uint64_t libUtil_Read64( uint8_t *arrayPointer, uint32_t *increment ) {
    return ( ( ( ( uint64_t ) libUtil_Read32( arrayPointer, increment ) ) << 32 ) | libUtil_Read32( arrayPointer, increment ) );
}
#endif

int8_t libUtil_ReadInt8( uint8_t *arrayPointer, uint32_t *increment ) {
    return ( int8_t ) libUtil_Read8( arrayPointer, increment );
}

int16_t libUtil_ReadInt16( uint8_t *arrayPointer, uint32_t *increment ) {
    return ( int16_t ) libUtil_Read16( arrayPointer, increment );
}

int32_t libUtil_ReadInt24( uint8_t *arrayPointer, uint32_t *increment ) {
    return ( int32_t ) libUtil_Read32( arrayPointer, increment );
}

int32_t libUtil_ReadInt32( uint8_t *arrayPointer, uint32_t *increment ) {
    return ( int32_t ) libUtil_Read32( arrayPointer, increment );
}

#if LIBRARY_ENABLE_64BITS
int64_t libUtil_ReadInt64( uint8_t *arrayPointer, uint32_t *increment ) {
    return ( int64_t ) libUtil_Read32( arrayPointer, increment );
}
#endif

float libUtil_ReadFloat( uint8_t *arrayPointer, uint32_t *increment, uint8_t decimalPoint ) {
    return ( ( ( float ) libUtil_ReadInt32( arrayPointer, increment ) ) / pow( 10, decimalPoint ) );
}

uint32_t libUtil_ReadString( uint8_t *arrayPointer, uint32_t *increment, char *outputStr, bool skipNullChar ) {
    uint32_t i = 0;
    while( arrayPointer[ ( *increment ) ] != '\0' )     {
        outputStr[ i++ ] = libUtil_Read8( arrayPointer, increment );
    }
    if( skipNullChar ) {
        ( *increment )++;
    }
    return i;
}

void libUtil_ReadTextSelection( uint8_t *arrayPointer, uint32_t *increment, char *outArray, uint8_t sizeOfChar, uint8_t numberOfSlot ) {
    uint32_t i = 0;
    while( arrayPointer[ ( *increment ) ] != '\0' ) {
        while( arrayPointer[ ( *increment ) ] != ';' && arrayPointer[ ( *increment ) ] != '\0' ) {
            if( i < sizeOfChar ) {
                outArray[ i++ ] = libUtil_Read8( arrayPointer, increment );
                outArray[ i + 1 ] = 0;
            } else {
                ( *increment )++;
            }
        }
        if( arrayPointer[ ( *increment ) ] == ';' ) {
            ( *increment )++;
            outArray += sizeOfChar;
            i = 0;
            numberOfSlot--;
            if( numberOfSlot <= 0 ) {
                break;
            }
        }
    }
}

/* ************************************************************************* */
void libUtil_hex_to_string( uint8_t *hex, uint8_t sizeof_hex, char *separator, bool capital, char *string ) {
    uint8_t i = 0;
    for( i = 0; i < sizeof_hex; i++ ) {
      if( capital ) {
          sprintf( string, "%s%s%02X", string, separator, hex[ i ] );
      } else {
          sprintf( string, "%s%s%02x", string, separator, hex[ i ] );
      }
    }
}

uint8_t libUtil_ReverseUint8( uint8_t data ) {
    uint8_t rval = 0;
    uint8_t i = 0;
    for( i = 0; i < 8; i++ ) {
        if( ( data & ( 1 << i ) ) != 0 ) {
            rval |= ( 0x80 >> i );
        }
    }
    return rval;
}

uint16_t libUtil_ReverseUint16( uint16_t data ) {
    return ( ( ( uint16_t ) libUtil_ReverseUint8( data & 0xFF ) << 8 ) | libUtil_ReverseUint8( ( data>>8 ) & 0xFF ) );
}

uint32_t libUtil_ReverseUint32( uint32_t data ) {
    return ( ( ( uint32_t ) libUtil_ReverseUint16( data & 0xFFFF ) << 16 ) | libUtil_ReverseUint16( ( data>>16 ) & 0xFFFF ) );
}

#if LIBRARY_ENABLE_64BITS
uint64_t libUtil_ReverseUint64( uint64_t data ) {
    return ( ( ( uint64_t ) libUtil_ReverseUint32( data & 0xFFFFFFFF ) << 32 ) | libUtil_ReverseUint32( ( data>>32 ) & 0xFFFFFFFF ) );
}
#endif

/* ************************************************************************* */


/* ****************************************************************************
 End of File
 */
