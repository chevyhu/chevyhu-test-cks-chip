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
#include "io/crsf/crossfire.h"
#include "rtos.h"
#include "stm32f4xx_flash.h"

bool set_model_id_needed = false;

HardwareOptions hardwareOptions;

RTOS_TASK_HANDLE crossfireTaskId;
RTOS_DEFINE_STACK(crossfireStack, CROSSFIRE_STACK_SIZE);

RTOS_TASK_HANDLE systemTaskId;
RTOS_DEFINE_STACK(systemStack, SYSTEM_STACK_SIZE);

static uint32_t DIO_INT_TRAMPOLINE;
static uint32_t NT_INT_TRAMPOLINE;
static uint32_t UART_INT_TRAMPOLINE;
RTOS_TASK_HANDLE Crossfire_Sync_Func_Addr(uint32_t *ptr);

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif
#include "usb_dcd_int.h"
#include "usb_bsp.h"
#if defined(__cplusplus) && !defined(SIMU)
}
#endif

typedef void (*Kernel_API_PTR)(void);
Kernel_API_PTR *Kernel_API = (Kernel_API_PTR*)KERNEL_API_ADDRESS;
void KernelApiInit( void ){
#define DEF_API_CMD(_id, _function, _return_type, ...)  (Kernel_API[_id] = (Kernel_API_PTR)&_function);
#include "rtos_api.h"
}

uint8_t isDisableBoardOff();

void watchdogInit(unsigned int duration)
{
  IWDG->KR = 0x5555;      // Unlock registers
  IWDG->PR = 3;           // Divide by 32 => 1kHz clock
  IWDG->KR = 0x5555;      // Unlock registers
  IWDG->RLR = duration;       // 1.5 seconds nominal
  IWDG->KR = 0xAAAA;      // reload
  IWDG->KR = 0xCCCC;      // start
  TRACE("watchdog init\r\n");
}

// Starts TIMER at 2MHz
void init2MhzTimer()
{
  TIMER_2MHz_TIMER->PSC = (PERI1_FREQUENCY * TIMER_MULT_APB1) / 2000000 - 1 ;       // 0.5 uS, 2 MHz
  TIMER_2MHz_TIMER->ARR = 65535;
  TIMER_2MHz_TIMER->CR2 = 0;
  TIMER_2MHz_TIMER->CR1 = TIM_CR1_CEN;
}

// Starts TIMER at 200Hz (5ms)
void init5msTimer()
{
  INTERRUPT_xMS_TIMER->ARR = 4999 ; // 5mS in uS
  INTERRUPT_xMS_TIMER->PSC = (PERI1_FREQUENCY * TIMER_MULT_APB1) / 1000000 - 1 ; // 1uS
  INTERRUPT_xMS_TIMER->CCER = 0 ;
  INTERRUPT_xMS_TIMER->CCMR1 = 0 ;
  INTERRUPT_xMS_TIMER->EGR = 0 ;
  INTERRUPT_xMS_TIMER->CR1 = 5 ;
  INTERRUPT_xMS_TIMER->DIER |= 1 ;
  NVIC_EnableIRQ(INTERRUPT_xMS_IRQn) ;
  NVIC_SetPriority(INTERRUPT_xMS_IRQn, 7);
}

void stop5msTimer( void )
{
  INTERRUPT_xMS_TIMER->CR1 = 0 ;        // stop timer
  NVIC_DisableIRQ(INTERRUPT_xMS_IRQn) ;
}

// TODO use the same than board_sky9x.cpp
void interrupt5ms()
{
  static uint32_t pre_scale ;       // Used to get 10 Hz counter

  AUDIO_HEARTBEAT();

#if defined(HAPTIC)
  HAPTIC_HEARTBEAT();
#endif
  if (++pre_scale >= 2) {
    pre_scale = 0 ;
    DEBUG_TIMER_START(debugTimerPer10ms);
    DEBUG_TIMER_SAMPLE(debugTimerPer10msPeriod);
    per10ms();
    DEBUG_TIMER_STOP(debugTimerPer10ms);
  }

#if defined(ROTARY_ENCODER_NAVIGATION) && !defined(ROTARY_ENCODER_EXTI_IRQHandler1)
	checkRotaryEncoder();
#endif

    if(!(crossfireSharedData.crsfFlag & CRSF_OPENTX_FLAG_BOOTUP) &&
            (crossfireSharedData.crsfFlag & CRSF_OPENTX_FLAG_SHOW_BOOTLOADER_ICON)){
        drawDownload();
        crossfireSharedData.crsfFlag &= ~CRSF_OPENTX_FLAG_SHOW_BOOTLOADER_ICON;
    }
}

#if !defined(SIMU)
extern "C" void INTERRUPT_xMS_IRQHandler()
{
  INTERRUPT_xMS_TIMER->SR &= ~TIM_SR_UIF ;
  interrupt5ms() ;
  DEBUG_INTERRUPT(INT_5MS);
}
#endif

// Starts TIMER at 1KHz (1ms)
void init1msTimer()
{
  INTERRUPT_1MS_TIMER->ARR = 999 ; // 1mS in uS
  INTERRUPT_1MS_TIMER->PSC = (PERI2_FREQUENCY * TIMER_MULT_APB2) / 1000000 - 1 ; // 1uS
  INTERRUPT_1MS_TIMER->CCER = 0 ;
  INTERRUPT_1MS_TIMER->CCMR1 = 0 ;
  INTERRUPT_1MS_TIMER->EGR = 0 ;
  INTERRUPT_1MS_TIMER->CR1 = 5 ;
  INTERRUPT_1MS_TIMER->DIER |= 1 ;
  NVIC_SetPriority(INTERRUPT_1MS_IRQn, INTERRUPT_1MS_IRQ_PRI);
  NVIC_EnableIRQ(INTERRUPT_1MS_IRQn) ;
}

// typedef void (*RTOS_Tick_CB_FuncPtr) (void);
// RTOS_Tick_CB_FuncPtr RTOS_Tick_CB = 0;
// uint32_t ulPortSetTickCB( uint32_t cb )
// {
//     RTOS_Tick_CB = (RTOS_Tick_CB_FuncPtr)cb;
//     return 1;
// }

#if !defined(SIMU)
extern "C" void INTERRUPT_1MS_IRQHandler()
{
  INTERRUPT_1MS_TIMER->SR &= ~TIM_SR_UIF ;
  // if( RTOS_Tick_CB )
  //     RTOS_Tick_CB();
}
#endif

#if defined(PWR_BUTTON_PRESS) && !defined(SIMU)
  #define PWR_PRESS_DURATION_MIN        100 // 1s
  #define PWR_PRESS_DURATION_MAX        500 // 5s
#endif

#if (defined(PCBX9E) || defined(PCBTANGO) && !defined(SIMU))
const unsigned char bmp_startup[]  = {
  #include "startup.lbm"
};
const unsigned char bmp_lock[]  = {
  #include "lock.lbm"
};
#endif

#if defined(SPORT_UPDATE_PWR_GPIO)
void sportUpdateInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = SPORT_UPDATE_PWR_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SPORT_UPDATE_PWR_GPIO, &GPIO_InitStructure);
}

void sportUpdatePowerOn()
{
  GPIO_SPORT_UPDATE_PWR_GPIO_ON(SPORT_UPDATE_PWR_GPIO, SPORT_UPDATE_PWR_GPIO_PIN);
}

void sportUpdatePowerOff()
{
  GPIO_SPORT_UPDATE_PWR_GPIO_OFF(SPORT_UPDATE_PWR_GPIO, SPORT_UPDATE_PWR_GPIO_PIN);
}
#endif

void getDefaultSwConfig(){
	uint8_t hasMem = 0;
	for(uint8_t i = 0; i < 6; i++){
		if(SWITCH_CONFIG(i) != 0){
			hasMem = 1;
		}
	}
	swconfig_t mask;
	if(!hasMem){
	    mask = (swconfig_t)0x03 << (2*0);
	    g_eeGeneral.switchConfig = (g_eeGeneral.switchConfig & ~mask) | ((swconfig_t(1) & 0x03) << (2*0));
	    mask = (swconfig_t)0x03 << (2*1);
	    g_eeGeneral.switchConfig = (g_eeGeneral.switchConfig & ~mask) | ((swconfig_t(3) & 0x03) << (2*1));
	    mask = (swconfig_t)0x03 << (2*2);
	    g_eeGeneral.switchConfig = (g_eeGeneral.switchConfig & ~mask) | ((swconfig_t(3) & 0x03) << (2*2));
	    mask = (swconfig_t)0x03 << (2*3);
	    g_eeGeneral.switchConfig = (g_eeGeneral.switchConfig & ~mask) | ((swconfig_t(1) & 0x03) << (2*3));
	    mask = (swconfig_t)0x03 << (2*4);
	    g_eeGeneral.switchConfig = (g_eeGeneral.switchConfig & ~mask) | ((swconfig_t(2) & 0x03) << (2*4));
	    mask = (swconfig_t)0x03 << (2*5);
	    g_eeGeneral.switchConfig = (g_eeGeneral.switchConfig & ~mask) | ((swconfig_t(2) & 0x03) << (2*5));
	}
}

void boardInit()
{
#if !defined(SIMU)
  RCC_AHB1PeriphClockCmd(PWR_RCC_AHB1Periph | PCBREV_RCC_AHB1Periph |
                         KEYS_RCC_AHB1Periph | LCD_RCC_AHB1Periph |
                         AUDIO_RCC_AHB1Periph | BACKLIGHT_RCC_AHB1Periph |
                         ADC_RCC_AHB1Periph | I2C_RCC_AHB1Periph |
                         SD_RCC_AHB1Periph | HAPTIC_RCC_AHB1Periph |
                         INTMODULE_RCC_AHB1Periph | EXTMODULE_RCC_AHB1Periph |
                         TELEMETRY_RCC_AHB1Periph | SPORT_UPDATE_RCC_AHB1Periph |
                         SERIAL_RCC_AHB1Periph | TRAINER_RCC_AHB1Periph |
                         HEARTBEAT_RCC_AHB1Periph | BT_RCC_AHB1Periph, ENABLE);

  RCC_APB1PeriphClockCmd(LCD_RCC_APB1Periph | AUDIO_RCC_APB1Periph | ADC_RCC_APB1Periph |
                         BACKLIGHT_RCC_APB1Periph | HAPTIC_RCC_APB1Periph | INTERRUPT_xMS_RCC_APB1Periph |
                         TIMER_2MHz_RCC_APB1Periph | I2C_RCC_APB1Periph |
                         SD_RCC_APB1Periph | TRAINER_RCC_APB1Periph |
                         TELEMETRY_RCC_APB1Periph | SERIAL_RCC_APB1Periph |
                         INTMODULE_RCC_APB1Periph | BT_RCC_APB1Periph, ENABLE);

  RCC_APB2PeriphClockCmd(BACKLIGHT_RCC_APB2Periph | ADC_RCC_APB2Periph |
                         HAPTIC_RCC_APB2Periph | INTMODULE_RCC_APB2Periph |
                         EXTMODULE_RCC_APB2Periph | HEARTBEAT_RCC_APB2Periph |
                         BT_RCC_APB2Periph | INTERRUPT_1MS_RCC_APB1Periph, ENABLE);
  KernelApiInit();
#if !defined(PCBX9E)
  // some X9E boards need that the pwrInit() is moved a little bit later
  pwrInit();
#endif

  if(FLASH_OB_GetBOR() != OB_BOR_LEVEL1){
      FLASH_Unlock();
      FLASH_OB_Unlock();
      FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR |
              FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR);
      FLASH_OB_BORConfig(OB_BOR_LEVEL1);
      FLASH_Lock();
      FLASH_OB_Launch();
  }

#if defined(READ_PROTECTION)
  if(FLASH_OB_GetRDP() != SET){
      FLASH_Unlock();
      FLASH_OB_Unlock();
      FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR |
              FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR);
      FLASH_OB_RDPConfig(OB_RDP_Level_1);
      FLASH_Lock();
      FLASH_OB_Launch();
  }
#endif

#if defined(STATUS_LEDS)
  ledInit();
  ledGreen();
#endif

  keysInit();
  delaysInit();

#if NUM_PWMSTICKS > 0
  sticksPwmInit();
  delay_ms(20);
  if (pwm_interrupt_count < 32) {
    sticks_pwm_disabled = true;
  }
#endif
  adcInit();
  lcdInit(); // delaysInit() must be called before
  audioInit();
  init2MhzTimer();
  init5msTimer();
  init1msTimer();
  CRSF_Init();
  __enable_irq();
  i2cInit();
  usbInit();

#if defined(DEBUG) && defined(SERIAL_GPIO)
  serial2Init(0, 0); // default serial mode (None if DEBUG not defined)
  TRACE("Tango board started :)\r\n");
#endif

#if defined(ESP_SERIAL)
  espInit(ESP_UART_BAUDRATE, false);
#endif

#if defined(HAPTIC)
  hapticInit();
#endif

#if defined(BLUETOOTH)
  bluetoothInit(BLUETOOTH_DEFAULT_BAUDRATE, true);
#endif

#if defined(ROTARY_ENCODER_NAVIGATION) && defined(ROTARY_ENCODER_EXTI_IRQHandler1)
  rotaryEncoderInit();
#endif

#if defined(DEBUG)
  DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP|DBGMCU_TIM1_STOP|DBGMCU_TIM2_STOP|DBGMCU_TIM3_STOP|DBGMCU_TIM6_STOP|DBGMCU_TIM8_STOP|DBGMCU_TIM10_STOP|DBGMCU_TIM13_STOP|DBGMCU_TIM14_STOP, ENABLE);
#endif

#if defined(PWR_BUTTON_PRESS)
#if defined(PCBTANGO)
  if (!WAS_RESET_BY_WATCHDOG()) {
#else
  if (!WAS_RESET_BY_WATCHDOG_OR_SOFTWARE()) {
#endif
    lcdClear();
#if defined(PCBX9E)
    lcdDrawBitmap(76, 2, bmp_lock, 0, 60);
#else
    lcdDrawFilledRect(LCD_W / 2 - 18, LCD_H / 2 - 3, 6, 6, SOLID, 0);
#endif
    lcdRefresh();
    lcdRefreshWait();

    tmr10ms_t start = get_tmr10ms();
    tmr10ms_t duration = 0;
    uint8_t pwr_on = 0;
    while (pwrPressed()) {
      duration = get_tmr10ms() - start;
      if (duration < PWR_PRESS_DURATION_MIN) {
        unsigned index = duration / (PWR_PRESS_DURATION_MIN / 4);
        lcdClear();
#if defined(PCBX9E)
        lcdDrawBitmap(76, 2, bmp_startup, index*60, 60);
#else
        for(uint8_t i= 0; i < 4; i++) {
          if (index >= i) {
            lcdDrawFilledRect(LCD_W / 2 - 18 + 10 * i, LCD_H / 2 - 3, 6, 6, SOLID, 0);
          }
        }
#endif
      }
      else if (duration >= PWR_PRESS_DURATION_MAX) {
        drawSleepBitmap();
        backlightDisable();
      }
      else {
        if (pwr_on != 1) {
          pwr_on = 1;
          pwrInit();
          backlightInit();
          haptic.play(15, 3, PLAY_NOW);
#if defined(PCBTANGO)
          break;
#endif
        }
      }
      lcdRefresh();
      lcdRefreshWait();
    }
    if (duration < PWR_PRESS_DURATION_MIN || duration >= PWR_PRESS_DURATION_MAX) {
    	if(!isDisableBoardOff()){
    		boardOff();
    	}
    }
  }
  else {
    pwrInit();
    backlightInit();
  }
#if defined(TOPLCD_GPIO)
  toplcdInit();
#endif
#else // defined(PWR_BUTTON_PRESS)
  pwrInit();
  backlightInit();
#endif

  if (HAS_SPORT_UPDATE_CONNECTOR()) {
    sportUpdateInit();
  }
#endif // !defined(SIMU)

  //TRACE("PWR_BUTTON = %s\n", PWR_BUTTON_PRESS);

#if defined(CRSF_OPENTX) && defined(CRSF_SD)
  sdInit();
#endif
}

void boardOff()
{
#if defined(STATUS_LEDS)
  ledOff();
#endif

  BACKLIGHT_DISABLE();

#if defined(TOPLCD_GPIO)
  toplcdOff();
#endif

#if defined(PWR_BUTTON_PRESS) && !defined(PCBTANGO)
  while (pwrPressed()) {
    wdt_reset();
  }
#endif
  lcdOff();
  SysTick->CTRL = 0; // turn off systick
  pwrOff();
}

uint8_t currentTrainerMode = 0xff;

void checkTrainerSettings()
{
  uint8_t requiredTrainerMode = g_model.trainerData.mode;
  if (requiredTrainerMode != currentTrainerMode) {
    switch (currentTrainerMode) {
      case TRAINER_MODE_MASTER_TRAINER_JACK:
        stop_trainer_capture();
        break;
      case TRAINER_MODE_SLAVE:
        stop_trainer_ppm();
        break;
      case TRAINER_MODE_MASTER_CPPM_EXTERNAL_MODULE:
        stop_cppm_on_heartbeat_capture() ;
        break;
      case TRAINER_MODE_MASTER_SBUS_EXTERNAL_MODULE:
        stop_sbus_on_heartbeat_capture() ;
        break;
#if defined(TRAINER_BATTERY_COMPARTMENT)
      case TRAINER_MODE_MASTER_BATTERY_COMPARTMENT:
        serial2Stop();
#endif
    }

    currentTrainerMode = requiredTrainerMode;
    switch (requiredTrainerMode) {
      case TRAINER_MODE_SLAVE:
        init_trainer_ppm();
        break;
      case TRAINER_MODE_MASTER_CPPM_EXTERNAL_MODULE:
         init_cppm_on_heartbeat_capture();
         break;
      case TRAINER_MODE_MASTER_SBUS_EXTERNAL_MODULE:
         init_sbus_on_heartbeat_capture();
         break;
#if defined(TRAINER_BATTERY_COMPARTMENT)
      case TRAINER_MODE_MASTER_BATTERY_COMPARTMENT:
        if (g_eeGeneral.serial2Mode == UART_MODE_SBUS_TRAINER) {
          serial2SbusInit();
          break;
        }
        // no break
#endif
      default:
        // master is default
        init_trainer_capture();
        break;
    }
  }
}

uint16_t getBatteryVoltage()
{
#if !defined(SIMU)
  // set the flag when opentx finish bootup
  if(!(crossfireSharedData.crsfFlag & CRSF_OPENTX_FLAG_BOOTUP)){
      crossfireSharedData.crsfFlag |= CRSF_OPENTX_FLAG_BOOTUP;
  }
#endif
  int32_t instant_vbat = anaIn(TX_VOLTAGE); // using filtered ADC value on purpose
  instant_vbat = (instant_vbat * BATT_SCALE * (128 + g_eeGeneral.txVoltageCalibration) ) / 26214;
//  instant_vbat += 20; // add 0.2V because of the diode TODO check if this is needed, but removal will beak existing calibrations!!!
  return (uint16_t)instant_vbat;
}

#if !defined(SIMU)
uint32_t readBackupReg(uint8_t index){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupRegulatorCmd(ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    while(PWR_GetFlagStatus(PWR_FLAG_BRR) == RESET);
    uint32_t value = *(__IO uint32_t *) (BKPSRAM_BASE + index*4);
    PWR_BackupAccessCmd(DISABLE);
    return value;
}

void writeBackupReg(uint8_t index, uint32_t data){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_BackupRegulatorCmd(ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	while(PWR_GetFlagStatus(PWR_FLAG_BRR) == RESET);
	*(__IO uint32_t *) (BKPSRAM_BASE + index*4) = data;
    PWR_BackupAccessCmd(DISABLE);
}

void boot2bootloader(uint32_t isNeedFlash, uint32_t HwId, uint32_t sn){
	usbStop();
	writeBackupReg(BOOTLOADER_IS_NEED_FLASH_ADDR, isNeedFlash);
	writeBackupReg(BOOTLOADER_HW_ID_ADDR, HwId);
	writeBackupReg(BOOTLOADER_SERIAL_NO_ADDR, sn);
	NVIC_SystemReset();
}

uint8_t isDisableBoardOff(){
	uint8_t value = (uint8_t)readBackupReg(BOOTLOADER_IS_SKIP_BOARD_OFF_ADDR);
	writeBackupReg(BOOTLOADER_IS_SKIP_BOARD_OFF_ADDR, 0);
	return value;
}
#endif

void PrintData(char* header, uint8_t* data){
	TRACE_NOCRLF("\r\n%s: ", header);
	for(int i = 0; i < data[1] + 2; i++){
		TRACE_NOCRLF("%x ", data[i]);
	}
	TRACE_NOCRLF("\r\n");
}

uint32_t getTime(void)
{
#if !defined(SIMU)
  return (uint32_t)CoGetOSTime();
#endif
}

RTOS_TASK_HANDLE Crossfire_Sync_Func_Addr(uint32_t *ptr)
{
  DIO_INT_TRAMPOLINE = ptr[0];
  NT_INT_TRAMPOLINE = ptr[1];
  UART_INT_TRAMPOLINE = ptr[2];
  return crossfireTaskId;
};

#if !defined(SIMU)
TASK_FUNCTION(systemTask)
{
  static uint32_t get_modelid_delay = 0;
  set_model_id_needed = true;

  while(1) {

	if(crossfireSharedData.crsfFlag & CRSF_OPENTX_FLAG_SHOW_BOOTLOADER_ICON){
		drawDownload();
		crossfireSharedData.crsfFlag &= ~CRSF_OPENTX_FLAG_SHOW_BOOTLOADER_ICON;
	}

    crsfSharedFifoHandler();
    crsfEspHandler();
#if defined(AGENT) && !defined(SIMU)
    AgentHandler();
#endif
    if (set_model_id_needed && g_model.header.modelId[EXTERNAL_MODULE] != 0) {
      crsfSetModelID();
      set_model_id_needed = false;
      crsfGetModelID();
      get_modelid_delay = get_tmr10ms();
    }
    if (get_modelid_delay && (get_tmr10ms() - get_modelid_delay) > 100) {
      if (current_crsf_model_id == g_model.header.modelId[EXTERNAL_MODULE]) {
        /* Set model id successfully */
        TRACE("Set model id for crossfire success, current id = %d\r\n", current_crsf_model_id);
      }
      else {
        /* Set model id failed */
        TRACE("Set model id for crossfire failed, current id = %d\r\n", current_crsf_model_id);
        /* do something else here? */
      }
      get_modelid_delay = 0;
    }
  }
  TASK_RETURN();
}
#endif

void tangoUpdateChannel( void )
{
  uint8_t i;
  for ( i = 0; i < NUM_STICKS; ++i)
    crossfireSharedData.channels[i] = channelOutputs[i];
  for ( i=0; i<NUM_SWITCHES; ++i)
    crossfireSharedData.channels[i + 4] = getValue(MIXSRC_FIRST_SWITCH+i);
}

#if 1
#define UART_INT_MODE_TX     1
#define UART_INT_MODE_RX     2
extern Fifo<uint8_t, 512> serial2TxFifo;
extern "C" void SERIAL_USART_IRQHandler(void)
{
  DEBUG_INTERRUPT(INT_SER2);
  bool xf_active = false;
	bool (*uart_cb)( uint8_t, uint8_t );
  bool xf_valid = false;
  uint8_t data;

  if ( UART_INT_TRAMPOLINE ){
    uart_cb = (bool (*)( uint8_t, uint8_t ))UART_INT_TRAMPOLINE;
    xf_valid = true;
  }

  // Send
  if (USART_GetITStatus(SERIAL_USART, USART_IT_TXE) != RESET) {
    if( xf_valid )
    	xf_active = uart_cb( UART_INT_MODE_TX, 0);
    if( !xf_active ){
      if ( !serial2TxFifo.isEmpty() ) {
        /* Write one byte to the transmit data register */
        serial2TxFifo.pop(data);
        USART_SendData(SERIAL_USART, data);
      }
      else {
        USART_ITConfig(SERIAL_USART, USART_IT_TXE, DISABLE);
      }
    }
  }

  if ( USART_GetITStatus(SERIAL_USART, USART_IT_RXNE) != RESET ) {
    if ( xf_valid ) {
      // Receive
      data = USART_ReceiveData(SERIAL_USART);
      uart_cb( UART_INT_MODE_RX, data);
    }
    else
      data = USART_ReceiveData(SERIAL_USART);
  }
}
#endif

extern "C" {

#if !defined(SIMU)
void EXTI15_10_IRQHandler(void)
{
	CoEnterISR();
	void (*exti_cb)(void);
  if (DIO_INT_TRAMPOLINE ){
    exti_cb = (void (*)(void))DIO_INT_TRAMPOLINE;
    /* call DIOCN handler of crossfire */
    exti_cb();
  }
  CoExitISR();
}

void TIM8_UP_TIM13_IRQHandler()
{
  CoEnterISR();
  if( TIM13->SR & TIM_SR_UIF )
  {
    TIM13->SR &= ~TIM_SR_UIF;
    void (*timer_cb)(void);
    if( NT_INT_TRAMPOLINE ){
      timer_cb = (void (*)(void))NT_INT_TRAMPOLINE;
      /* call notification timer handler of crossfire */
      timer_cb();
    }
  }
  CoExitISR();
}
#endif

#include <stdio.h>
#include <stdarg.h>
void uart_tx(uint8_t byte)
{
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    USART_SendData(USART3, byte);
}

void hf_printf(const char * TxBuf, ...)
{
	uint8_t UartBuf[200];
	va_list arglist;
	volatile uint8_t *fp;
	uint32_t length=0;

	va_start(arglist, TxBuf);
	vsprintf((char*)UartBuf, (const char*)TxBuf, arglist);
	va_end(arglist);

	fp = UartBuf;
	while(*fp)
	{
		uart_tx(*fp);
		fp++;
	}
}

void _general_exception_handler (unsigned int * hardfault_args)
{
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;

    stacked_r0 = ((unsigned long) hardfault_args[0]);
    stacked_r1 = ((unsigned long) hardfault_args[1]);
    stacked_r2 = ((unsigned long) hardfault_args[2]);
    stacked_r3 = ((unsigned long) hardfault_args[3]);

    stacked_r12 = ((unsigned long) hardfault_args[4]);
    stacked_lr = ((unsigned long) hardfault_args[5]);
    stacked_pc = ((unsigned long) hardfault_args[6]);
    stacked_psr = ((unsigned long) hardfault_args[7]);

    hf_printf ("\r\n\n***OpenTx Hard Fault Handler Debug Printing***\r\n");
    hf_printf ("R0\t\t= 0x%.8x\r\n", stacked_r0);
    hf_printf ("R1\t\t= 0x%.8x\r\n", stacked_r1);
    hf_printf ("R2\t\t= 0x%.8x\r\n", stacked_r2);
    hf_printf ("R3\t\t= 0x%.8x\r\n", stacked_r3);
    hf_printf ("R12\t\t= 0x%.8x\r\n", stacked_r12);
    hf_printf ("LR [R14]\t= 0x%.8x\r\n", stacked_lr);
    hf_printf ("PC [R15]\t= 0x%.8x\r\n", stacked_pc);
    hf_printf ("PSR\t\t= 0x%.8x\r\n", stacked_psr);
    hf_printf ("BFAR\t\t= 0x%.8x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
    hf_printf ("CFSR\t\t= 0x%.8x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
    hf_printf ("HFSR\t\t= 0x%.8x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
    hf_printf ("AFSR\t\t= 0x%.8x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
    hf_printf ("SCB_SHCSR\t= 0x%.8x\r\n", SCB->SHCSR);

    while (1){
#ifdef RELEASE_CONFIGURATION
        SOFT_RESET();
        while(1);
#endif
    }
}

#if !defined(SIMU)
void HardFault_Handler(void)
{
    __asm("TST LR, #4");
    __asm("ITE EQ");
    __asm("MRSEQ R0, MSP");
    __asm("MRSNE R0, PSP");
    __asm("B _general_exception_handler");
}
#endif
}	//extern "C" {
