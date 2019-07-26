/* 
 * File:   rtos_api.h
 * Author: Marcel
 *
 * Created on 22. November 2018, 14:21
 */

    
#ifdef DEF_API_CMD

#if !defined(SIMU)
    //          id,   function                          ,  return_type, 	params
    DEF_API_CMD( 1, RTOS_WAIT_TICKS                     , uint8_t       	, const TickType_t xTicksToDelay)
    DEF_API_CMD( 2, CoWaitForSingleFlag                 , StatusType  		, OS_FlagID id, U32 timeout)
    DEF_API_CMD( 3, isr_SetFlag              			, StatusType    	, OS_FlagID id)
    DEF_API_CMD( 4, CoCreateFlag                        , BOOL bAutoReset	, BOOL bInitialState)
    DEF_API_CMD( 5, CoClearFlag             			, StatusType  		, OS_FlagID id)
    DEF_API_CMD( 6, Crossfire_Sync_Func_Addr            , RTOS_TASK_HANDLE	, uint32_t *ptr)
#endif
#undef DEF_API_CMD
#else
            
#define RTOS_API_VERSION 0x100

#ifndef RTOS_API_H
#define	RTOS_API_H

/* from ProjDefs.h */
//typedef void * TaskHandle_t;
#define pdFALSE			( ( BaseType_t ) 0 )
#define pdTRUE			( ( BaseType_t ) 1 )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )    
#define pdMS_TO_TICKS( xTimeInMs ) ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInMs ) * ( TickType_t ) configTICK_RATE_HZ ) / ( TickType_t ) 1000 ) )
    
    
#endif	/* RTOS_API_H */
#endif // #ifdef DEF_API_CMD




