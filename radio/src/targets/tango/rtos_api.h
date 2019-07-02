/* 
 * File:   rtos_api.h
 * Author: Marcel
 *
 * Created on 22. November 2018, 14:21
 */

    
#ifdef DEF_API_CMD
    //          id,   function                          ,  return_type, 	params
    DEF_API_CMD( 1, RTOS_WAIT_TICKS                     , uint8_t       	, const TickType_t xTicksToDelay)
    DEF_API_CMD( 2, CoWaitForSingleFlag                 , StatusType  		, OS_FlagID id, U32 timeout)
    DEF_API_CMD( 3, isr_SetFlag              			, StatusType    	, OS_FlagID id)
    DEF_API_CMD( 4, Crossfire_Get_Firmware_Task_Handle  , RTOS_TASK_HANDLE	, void)
//    DEF_API_CMD( 5, vTaskGetRunTimeStats                , void        		, char *pcWriteBuffer)
    DEF_API_CMD( 6, ulPortSetTickCB                     , uint32_t      	, void* cb)
    DEF_API_CMD( 7, CoCreateFlag                        , BOOL bAutoReset	, BOOL bInitialState)
    DEF_API_CMD( 8, CoClearFlag             			, StatusType  		, OS_FlagID id)
    DEF_API_CMD( 9, Crossfire_Get_Func_Addr             , void        		, uint8_t type, uint32_t addr)
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
    
    
#endif	/* RTOS_API_H */
#endif // #ifdef DEF_API_CMD




