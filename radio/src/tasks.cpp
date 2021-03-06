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

#include "opentx.h"
#if defined(PCBTANGO) && defined(CRSF_OPENTX) && defined(CRSF_SD)
#include "io/crsf/crossfire.h"
#endif

RTOS_TASK_HANDLE menusTaskId;
RTOS_DEFINE_STACK(menusStack, MENUS_STACK_SIZE);

RTOS_TASK_HANDLE mixerTaskId;
RTOS_DEFINE_STACK(mixerStack, MIXER_STACK_SIZE);

RTOS_TASK_HANDLE audioTaskId;
RTOS_DEFINE_STACK(audioStack, AUDIO_STACK_SIZE);

RTOS_MUTEX_HANDLE audioMutex;
RTOS_MUTEX_HANDLE mixerMutex;

enum TaskIndex {
  MENU_TASK_INDEX,
  MIXER_TASK_INDEX,
  AUDIO_TASK_INDEX,
  CLI_TASK_INDEX,
  BLUETOOTH_TASK_INDEX,
  TASK_INDEX_COUNT,
  MAIN_TASK_INDEX = 255
};

void stackPaint()
{
#if 0
  menusStack.paint();
  mixerStack.paint();
  audioStack.paint();
#if defined(CLI)
  cliStack.paint();
#endif
#endif
}

#ifdef RTOS_FREERTOS
#ifdef __cplusplus
extern "C" {
#endif
void vApplicationTickHook(void) {
}

/* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created.  It is also called by various parts of the
   demo application.  If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook(void) {
  taskDISABLE_INTERRUPTS();
  for(;;);
}

/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
   task.  It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()).  If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
void vApplicationIdleHook(void) {
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
  (void) pcTaskName;
  (void) pxTask;
  /* Run time stack overflow checking is performed if
     configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     function is called if a stack overflow is detected. */
  taskDISABLE_INTERRUPTS();
  for(;;);
}

// Macro to use CCM (Core Coupled Memory) in STM32F4
#define CCM_RAM __attribute__((section(".ccmram")))

StaticTask_t xIdleTaskTCB CCM_RAM;
StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE] CCM_RAM;

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
  /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
  state will be stored. */
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

  /* Pass out the array that will be used as the Idle task's stack. */
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;

  /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
  Note that, as the array is necessarily of type StackType_t,
  configMINIMAL_STACK_SIZE is specified in words, not bytes. */
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

static StaticTask_t xTimerTaskTCB CCM_RAM;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH] CCM_RAM;

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#ifdef __cplusplus
}
#endif
#endif

volatile uint16_t timeForcePowerOffPressed = 0;

bool isForcePowerOffRequested()
{
  if (pwrOffPressed()) {
    if (timeForcePowerOffPressed == 0) {
      timeForcePowerOffPressed = get_tmr10ms();
    }
    else {
      uint16_t delay = (uint16_t)get_tmr10ms() - timeForcePowerOffPressed;
      if (delay > 1000/*10s*/) {
        return true;
      }
    }
  }
  else {
    resetForcePowerOffRequest();
  }
  return false;
}

bool isModuleSynchronous(uint8_t module)
{
  uint8_t protocol = moduleState[module].protocol;
  if (protocol == PROTOCOL_CHANNELS_PXX2 || protocol == PROTOCOL_CHANNELS_CROSSFIRE || protocol == PROTOCOL_CHANNELS_NONE)
    return true;
#if defined(INTMODULE_USART) || defined(EXTMODULE_USART)
  if (protocol == PROTOCOL_CHANNELS_PXX1_SERIAL)
    return true;
#endif
  return false;
}

void sendSynchronousPulses()
{
  for (uint8_t module = 0; module < NUM_MODULES; module++) {
    if (isModuleSynchronous(module) && setupPulses(module)) {
#if defined(HARDWARE_INTERNAL_MODULE)
      if (module == INTERNAL_MODULE)
        intmoduleSendNextFrame();
#endif
      if (module == EXTERNAL_MODULE)
        extmoduleSendNextFrame();
    }
  }
}

uint32_t nextMixerTime[NUM_MODULES];


TASK_FUNCTION(mixerTask)
{
  static uint32_t lastRunTime;
  s_pulses_paused = true;

  TRACE("mixer task started\r\n");
  while (1) {
    wdt_reset(); //add by Chevy for temporary, 2019/07/08
#if defined(PCBX9D) || defined(PCBX7)
    // SBUS on Hearbeat PIN (which is a serial RX)
    processSbusInput();
#endif

#if defined(GYRO)
    gyro.wakeup();
#endif

#if defined(BLUETOOTH)
    bluetooth.wakeup();
#endif

    RTOS_WAIT_TICKS(1);

#if defined(SIMU)
    if (pwrCheck() == e_power_off) {
      TASK_RETURN();
    }
#else
    if (isForcePowerOffRequested()) {
      pwrOff();
    }
#endif

    uint32_t now = RTOS_GET_MS();
    bool run = false;
    if ((now - lastRunTime) >= 10) {     // run at least every 10ms
      run = true;
    }
    else if (now == nextMixerTime[0]) {
      run = true;
    }
#if NUM_MODULES >= 2
    else if (now == nextMixerTime[1]) {
      run = true;
    }
#endif
    if (!run) {
      continue;  // go back to sleep
    }

    lastRunTime = now;

    if (!s_pulses_paused) {
      uint16_t t0 = getTmr2MHz();

      DEBUG_TIMER_START(debugTimerMixer);
      RTOS_LOCK_MUTEX(mixerMutex);
      doMixerCalculations();
      DEBUG_TIMER_START(debugTimerMixerCalcToUsage);
      DEBUG_TIMER_SAMPLE(debugTimerMixerIterval);
      RTOS_UNLOCK_MUTEX(mixerMutex);
      DEBUG_TIMER_STOP(debugTimerMixer);
      //TRACE("mixer, min:%d, max:%d\n", debugTimers[debugTimerMixer].getMin(), debugTimers[debugTimerMixer].getMax());

#if defined(STM32) && !defined(SIMU)
      if (getSelectedUsbMode() == USB_JOYSTICK_MODE) {
        usbJoystickUpdate();
      }
  #if defined(PCBTANGO)
      tangoUpdateChannel();
  #endif
#endif

#if defined(TELEMETRY_FRSKY) || defined(PCBTANGO)
      DEBUG_TIMER_START(debugTimerTelemetryWakeup);
      telemetryWakeup();
      DEBUG_TIMER_STOP(debugTimerTelemetryWakeup);
#endif

      if (heartbeat == HEART_WDT_CHECK) {
        wdt_reset();
        heartbeat = 0;
      }

      t0 = getTmr2MHz() - t0;
      if (t0 > maxMixerDuration) maxMixerDuration = t0;

      sendSynchronousPulses();
    }
  }
}

void scheduleNextMixerCalculation(uint8_t module, uint16_t period_ms)
{
  // Schedule next mixer calculation time,

  if (isModuleSynchronous(module)) {
    nextMixerTime[module] += period_ms / RTOS_MS_PER_TICK;
    if (nextMixerTime[module] < RTOS_GET_TIME()) {
      // we are late ... let's add some small delay
      nextMixerTime[module] = (uint32_t) RTOS_GET_TIME() + (period_ms / RTOS_MS_PER_TICK);
    }
  }
  else {
    // for now assume mixer calculation takes 2 ms.
    nextMixerTime[module] = (uint32_t) RTOS_GET_TIME() + (period_ms / RTOS_MS_PER_TICK) - 1 /* 1 tick in advance*/;
  }

  DEBUG_TIMER_STOP(debugTimerMixerCalcToUsage);
}

#define MENU_TASK_PERIOD_TICKS         (50 / RTOS_MS_PER_TICK)    // 50ms

#if defined(COLORLCD) && defined(CLI)
bool perMainEnabled = true;
#endif

#include "malloc.h"

TASK_FUNCTION(menusTask)
{
  TRACE("menu task started\r\n");
  opentxInit();

#if defined(PCBTANGO)
  getDefaultSwConfig();
#endif

#if defined(PWR_BUTTON_PRESS)
  while (1) {
    uint32_t pwr_check = pwrCheck();
    if (pwr_check == e_power_off) {
      break;
    }
    else if (pwr_check == e_power_press) {
      RTOS_WAIT_TICKS(MENU_TASK_PERIOD_TICKS);
      continue;
    }
#else
  while (pwrCheck() != e_power_off) {
#endif
    uint32_t start = (uint32_t)RTOS_GET_TIME();
    DEBUG_TIMER_START(debugTimerPerMain);
#if defined(COLORLCD) && defined(CLI)
    if (perMainEnabled) {
      perMain();
    }
#else
    perMain();
#endif
    DEBUG_TIMER_STOP(debugTimerPerMain);
    // TODO remove completely massstorage from sky9x firmware
    uint32_t runtime = ((uint32_t)RTOS_GET_TIME() - start);
    // deduct the thread run-time from the wait, if run-time was more than
    // desired period, then skip the wait all together
    if (runtime < MENU_TASK_PERIOD_TICKS) {
      RTOS_WAIT_TICKS(MENU_TASK_PERIOD_TICKS - runtime);
    }

    resetForcePowerOffRequest();
  }

#if defined(PCBTANGO) && defined(CRSF_OPENTX) && defined(CRSF_SD)
    crossfireSharedData.crsfFlag |= CRSF_OPENTX_FLAG_EEPROM_SAVE;
    while(crossfireSharedData.crsfFlag & CRSF_OPENTX_FLAG_EEPROM_SAVE){
        RTOS_WAIT_TICKS(1);
    }
#endif

#if defined(PCBX9E)
  toplcdOff();
#endif

#if defined(PCBHORUS)
  ledOff();
#endif

  drawSleepBitmap();
  opentxClose();
#if defined(PCBTANGO) || defined(PCBMAMBO)
  if (IS_CHARGING_STATE() && !IS_CHARGING_FAULT() && usbPlugged()) {
    NVIC_SystemReset();
  }
#endif
  boardOff(); // Only turn power off if necessary

  TASK_RETURN();
}

void tasksStart()
{
  RTOS_INIT();

#if defined(CLI)
  cliStart();
#endif

  RTOS_CREATE_TASK(mixerTaskId, mixerTask, "Mixer", mixerStack, MIXER_STACK_SIZE, MIXER_TASK_PRIO);
  RTOS_CREATE_TASK(menusTaskId, menusTask, "Menus", menusStack, MENUS_STACK_SIZE, MENUS_TASK_PRIO);
#if (defined(PCBTANGO) || defined(PCBMAMBO)) && defined(CROSSFIRE_TASK) && !defined(SIMU) && 0
  extern RTOS_TASK_HANDLE crossfireTaskId;
  extern RTOS_DEFINE_STACK(crossfireStack, CROSSFIRE_STACK_SIZE);
  extern RTOS_TASK_HANDLE systemTaskId;
  extern RTOS_DEFINE_STACK(systemStack, SYSTEM_STACK_SIZE);
  extern TASK_FUNCTION(systemTask);
  // Test if crossfire task is available and start it
  if (*(uint32_t *)CROSSFIRE_TASK_ADDRESS != 0xFFFFFFFF ) {
    RTOS_CREATE_TASK(crossfireTaskId, (FUNCPtr)CROSSFIRE_TASK_ADDRESS, "crossfire", crossfireStack, CROSSFIRE_STACK_SIZE, CROSSFIRE_TASK_PRIORITY);
  }

  RTOS_CREATE_TASK(systemTaskId, systemTask, "system", systemStack, SYSTEM_STACK_SIZE, RTOS_SYS_TASK_PRIORITY);
#endif

#if !defined(SIMU)
  RTOS_CREATE_TASK(audioTaskId, audioTask, "Audio", audioStack, AUDIO_STACK_SIZE, AUDIO_TASK_PRIO);
#endif

  RTOS_CREATE_MUTEX(audioMutex);
  RTOS_CREATE_MUTEX(mixerMutex);

  RTOS_START();
}
