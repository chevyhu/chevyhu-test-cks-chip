/* 
 * File:   rtos_api.h
 * Author: Marcel
 *
 * Created on 22. November 2018, 14:21
 */

    
#ifdef DEF_API_CMD
    //          id,   function                          ,  return_type, params  
    DEF_API_CMD( 1, RTOS_WAIT_TICKS                     , const void  , const TickType_t xTicksToDelay)
    DEF_API_CMD( 2, ulTaskNotifyTake                    , uint32_t    , BaseType_t xClearCountOnExit, TickType_t xTicksToWait)
    DEF_API_CMD( 3, vTaskNotifyGiveFromISR              , void        , TaskHandle_t xTaskToNotify, BaseType_t *pxHigherPriorityTaskWoken)
    DEF_API_CMD( 4, Crossfire_Get_Firmware_Task_Handle  , TaskHandle_t, void)
    DEF_API_CMD( 5, vTaskGetRunTimeStats                , void        , char *pcWriteBuffer)
            
#undef DEF_API_CMD
#else
            
#define RTOS_API_VERSION 0x100

#ifndef RTOS_API_H
#define	RTOS_API_H

/* from ProjDefs.h */
typedef void * TaskHandle_t;
#define pdFALSE			( ( BaseType_t ) 0 )
#define pdTRUE			( ( BaseType_t ) 1 )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )    
#define pdMS_TO_TICKS( xTimeInMs ) ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInMs ) * ( TickType_t ) configTICK_RATE_HZ ) / ( TickType_t ) 1000 ) )
    
/* see https://www.microchip.com/forums/m988669.aspx for discussion
 * this creates a set of trampolines an 0x9D004000 following to RTOS functions
 */
#define KERNEL_API(number)  (KERNEL_API_ADDRESS+(4*number))
    
#define DEF_API_CMD(_id, _function, _return_type, ...) extern _return_type (*_function) (__VA_ARGS__)__attribute__((address(KERNEL_API(_id)),common));
#include "rtos_api.h"
    
#endif	/* RTOS_API_H */
#endif // #ifdef DEF_API_CMD




