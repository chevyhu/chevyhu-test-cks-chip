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

#if defined(ROTARY_ENCODER_NAVIGATION)
uint32_t rotencPositionValue;
#endif


#if defined(PCBTANGO)
uint8_t  g_trimState = 0;
#endif

uint32_t readKeys()
{
  uint32_t result = 0;

  if (~KEYS_GPIO_REG_ENTER & KEYS_GPIO_PIN_ENTER)
    result |= 1 << KEY_ENTER;

#if defined(KEYS_GPIO_PIN_MENU)
  if (~KEYS_GPIO_REG_MENU & KEYS_GPIO_PIN_MENU) {
    result |= 1 << KEY_MENU;
  }
#endif

#if defined(KEYS_GPIO_PIN_PAGE)
  if (~KEYS_GPIO_REG_PAGE & KEYS_GPIO_PIN_PAGE)
    result |= 1 << KEY_PAGE;
#endif

  if (~KEYS_GPIO_REG_EXIT & KEYS_GPIO_PIN_EXIT)
    result |= 1 << KEY_EXIT;


#if defined(KEYS_GPIO_PIN_PLUS)
  if (~KEYS_GPIO_REG_PLUS & KEYS_GPIO_PIN_PLUS)
    result |= 1 << KEY_PLUS;
  if (~KEYS_GPIO_REG_MINUS & KEYS_GPIO_PIN_MINUS)
    result |= 1 << KEY_MINUS;
#endif


#if defined(KEYS_GPIO_PIN_LEFT)
  if (~KEYS_GPIO_REG_LEFT & KEYS_GPIO_PIN_LEFT)
    result |= 1 << KEY_LEFT;
  if (~KEYS_GPIO_REG_RIGHT & KEYS_GPIO_PIN_RIGHT)
    result |= 1 << KEY_RIGHT;
  if (~KEYS_GPIO_REG_UP & KEYS_GPIO_PIN_UP)
    result |= 1 << KEY_UP;
  if (~KEYS_GPIO_REG_DOWN & KEYS_GPIO_PIN_DOWN)
    result |= 1 << KEY_DOWN;
#endif

#if defined(KEYS_GPIO_PIN_SHIFT)
  if (~KEYS_GPIO_REG_SHIFT & KEYS_GPIO_PIN_SHIFT)
    result |= 1 << KEY_SHIFT;
#endif

  // if (result != 0) TRACE("readKeys(): result=0x%02x", result);

  return result;
}

uint32_t readTrims()
{
  uint32_t result = 0;
#if defined(PCBTANGO)
  // the trim state from the events of per10ms()
  result = g_trimState;
  g_trimState = 0;
#elif defined(PCBMAMBO) && !defined(SIMU) && !defined (BOOT)
  uint16_t trimValue = anaIn(TX_TRIM);
  if (trimValue > 800) {
    if ((1077 - TRIM_ADC_OFFSET) < trimValue && trimValue < (1077 + TRIM_ADC_OFFSET))
      result |= 0x01;
    if ((1386 - TRIM_ADC_OFFSET) < trimValue && trimValue < (1386 + TRIM_ADC_OFFSET))
      result |= 0x02;
    if ((1543 - TRIM_ADC_OFFSET) < trimValue && trimValue < (1543 + TRIM_ADC_OFFSET))
      result |= 0x04;
    if ((1637 - TRIM_ADC_OFFSET) < trimValue && trimValue < (1637 + TRIM_ADC_OFFSET))
      result |= 0x08;

    if ((1755 - TRIM_ADC_OFFSET) < trimValue && trimValue < (1755 + TRIM_ADC_OFFSET))
      result |= 0x10;
    if ((1705 - TRIM_ADC_OFFSET) < trimValue && trimValue < (1705 + TRIM_ADC_OFFSET))
      result |= 0x20;
    if ((1794 - TRIM_ADC_OFFSET) < trimValue && trimValue < (1794 + TRIM_ADC_OFFSET))
      result |= 0x40;
    if ((1824 - TRIM_ADC_OFFSET) < trimValue && trimValue < (1824 + TRIM_ADC_OFFSET))
      result |= 0x80;
  }

#else
  if (~TRIMS_GPIO_REG_LHL & TRIMS_GPIO_PIN_LHL)
    result |= 0x01;
  if (~TRIMS_GPIO_REG_LHR & TRIMS_GPIO_PIN_LHR)
    result |= 0x02;
  if (~TRIMS_GPIO_REG_LVD & TRIMS_GPIO_PIN_LVD)
    result |= 0x04;
  if (~TRIMS_GPIO_REG_LVU & TRIMS_GPIO_PIN_LVU)
    result |= 0x08;

#if defined(PCBXLITE)
  if (IS_SHIFT_PRESSED())
    result = ((result & 0x03) << 6) | ((result & 0x0c) << 2);
#else
  if (~TRIMS_GPIO_REG_RVD & TRIMS_GPIO_PIN_RVD)
    result |= 0x10;
  if (~TRIMS_GPIO_REG_RVU & TRIMS_GPIO_PIN_RVU)
    result |= 0x20;
  if (~TRIMS_GPIO_REG_RHL & TRIMS_GPIO_PIN_RHL)
    result |= 0x40;
  if (~TRIMS_GPIO_REG_RHR & TRIMS_GPIO_PIN_RHR)
    result |= 0x80;
#endif

  // TRACE("readTrims(): result=0x%02x", result);
#endif
  return result;
}

uint8_t trimDown(uint8_t idx)
{
  return readTrims() & (1 << idx);
}


bool keyDown()
{
  return readKeys() || readTrims();
}

/* TODO common to ARM */
void readKeysAndTrims()
{
  uint8_t index = 0;
  uint32_t keys_input = readKeys();
  for (uint8_t i = 1; i != uint8_t(1 << TRM_BASE); i <<= 1) {
    keys[index++].input(keys_input & i);
  }

  uint32_t trims_input = readTrims();
  for (uint8_t i = 1; i != uint8_t(1 << 8); i <<= 1) {
    keys[index++].input(trims_input & i);
  }
#if defined(PWR_BUTTON_PRESS)
  if ((keys_input || trims_input || pwrPressed()) && (g_eeGeneral.backlightMode & e_backlight_mode_keys)) {
#else
    if ((keys_input || trims_input) && (g_eeGeneral.backlightMode & e_backlight_mode_keys)) {
#endif
    // on keypress turn the light on
    backlightOn();
  }
}

#if defined(PCBX9E)
#define ADD_2POS_CASE(x) \
    case SW_S ## x ## 2: \
      xxx = SWITCHES_GPIO_REG_ ## x  & SWITCHES_GPIO_PIN_ ## x ; \
      break; \
    case SW_S ## x ## 0: \
      xxx = ~SWITCHES_GPIO_REG_ ## x  & SWITCHES_GPIO_PIN_ ## x ; \
      break;
#else
#define ADD_2POS_CASE(x) \
    case SW_S ## x ## 0: \
      xxx = SWITCHES_GPIO_REG_ ## x  & SWITCHES_GPIO_PIN_ ## x ; \
      break; \
    case SW_S ## x ## 2: \
      xxx = ~SWITCHES_GPIO_REG_ ## x  & SWITCHES_GPIO_PIN_ ## x ; \
      break;
#endif

#if defined(PCBMAMBO) && !defined (SIMU)
#define ADD_3POS_CASE(x, i) \
      case SW_S ## x ## 0: \
        xxx = (anaIn(SWITCH_ ## x) < 513) ? 1 : 0; \
        break; \
      case SW_S ## x ## 1: \
        xxx = (anaIn(SWITCH_ ## x) < 1025) ? 1 : 0; \
        break; \
      case SW_S ## x ## 2: \
        xxx = (anaIn(SWITCH_ ## x) < 2049) ? 1 : 0; \
        break
#else
  #define ADD_3POS_CASE(x, i) \
      case SW_S ## x ## 0: \
        xxx = (SWITCHES_GPIO_REG_ ## x ## _H & SWITCHES_GPIO_PIN_ ## x ## _H); \
        if (IS_CONFIG_3POS(i)) { \
          xxx = xxx && (~SWITCHES_GPIO_REG_ ## x ## _L & SWITCHES_GPIO_PIN_ ## x ## _L); \
        } \
        break; \
      case SW_S ## x ## 1: \
        xxx = (SWITCHES_GPIO_REG_ ## x ## _H & SWITCHES_GPIO_PIN_ ## x ## _H) && (SWITCHES_GPIO_REG_ ## x ## _L & SWITCHES_GPIO_PIN_ ## x ## _L); \
        break; \
      case SW_S ## x ## 2: \
        xxx = (~SWITCHES_GPIO_REG_ ## x ## _H & SWITCHES_GPIO_PIN_ ## x ## _H); \
        if (IS_CONFIG_3POS(i)) { \
          xxx = xxx && (SWITCHES_GPIO_REG_ ## x ## _L & SWITCHES_GPIO_PIN_ ## x ## _L); \
        } \
        break
#endif

uint8_t keyState(uint8_t index)
{
  return keys[index].state();
}

#if !defined(BOOT)
uint32_t switchState(uint8_t index)
{
  uint32_t xxx = 0;

  switch (index) {
#if defined(PCBTANGO)
    ADD_2POS_CASE(A);
    ADD_3POS_CASE(B, 0);
    ADD_3POS_CASE(C, 0);
    ADD_3POS_CASE(D, 0);
    ADD_3POS_CASE(E, 0);
    ADD_2POS_CASE(F);
#elif defined(PCBMAMBO)
    ADD_2POS_CASE(A)
    ADD_3POS_CASE(B, 1);
    ADD_3POS_CASE(C, 2);
    ADD_3POS_CASE(D, 3);
    ADD_3POS_CASE(E, 4);
    ADD_2POS_CASE(F);
#else
    ADD_3POS_CASE(A, 0);
    ADD_3POS_CASE(B, 1);
    ADD_3POS_CASE(C, 2);
    ADD_3POS_CASE(D, 3);
#if defined(PCBXLITE)
    // no SWF, SWG and SWH on XLITE
#elif defined(PCBX7)
    ADD_2POS_CASE(F);
    ADD_2POS_CASE(H);
#else
    ADD_3POS_CASE(E, 4);
    ADD_2POS_CASE(F);
    ADD_3POS_CASE(G, 6);
    ADD_2POS_CASE(H);
#endif
#if defined(PCBX9E)
    ADD_3POS_CASE(I, 8);
    ADD_3POS_CASE(J, 9);
    ADD_3POS_CASE(K, 10);
    ADD_3POS_CASE(L, 11);
    ADD_3POS_CASE(M, 12);
    ADD_3POS_CASE(N, 13);
    ADD_3POS_CASE(O, 14);
    ADD_3POS_CASE(P, 15);
    ADD_3POS_CASE(Q, 16);
    ADD_3POS_CASE(R, 17);
#endif
#endif
    default:
      break;
  }

  // TRACE("switch %d => %d", index, xxx);
  return xxx;
}
#endif

#if defined(ROTARY_ENCODER_NAVIGATION) && !defined(BOOT)
void checkRotaryEncoder()
{
  uint32_t newpos = ROTARY_ENCODER_POSITION();
  if (newpos != rotencPositionValue && !keyState(KEY_ENTER)) {
	if ((rotencPositionValue & 0x01) ^ ((newpos & 0x02) >> 1)) {
        --rotencValue;
	}
	else {
        ++rotencValue;
	}

    rotencPositionValue = newpos;
#if !defined(BOOT)
    if (g_eeGeneral.backlightMode & e_backlight_mode_keys) {
      backlightOn();
    }
#endif
  }
}
#endif

void keysInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;

#if defined(KEYS_GPIOA_PINS)
  GPIO_InitStructure.GPIO_Pin = KEYS_GPIOA_PINS;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

#if defined(KEYS_GPIOB_PINS)
  GPIO_InitStructure.GPIO_Pin = KEYS_GPIOB_PINS;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

#if defined(KEYS_GPIOC_PINS)
  GPIO_InitStructure.GPIO_Pin = KEYS_GPIOC_PINS;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

#if defined(KEYS_GPIOD_PINS)
  GPIO_InitStructure.GPIO_Pin = KEYS_GPIOD_PINS;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif

#if defined(KEYS_GPIOE_PINS)
  GPIO_InitStructure.GPIO_Pin = KEYS_GPIOE_PINS;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif

#if defined(KEYS_GPIOF_PINS)
  GPIO_InitStructure.GPIO_Pin = KEYS_GPIOF_PINS;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
#endif

#if defined(KEYS_GPIOG_PINS)
  GPIO_InitStructure.GPIO_Pin = KEYS_GPIOG_PINS;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
#endif

#if defined(ROTARY_ENCODER_NAVIGATION)
  rotencPositionValue = ROTARY_ENCODER_POSITION();
#endif
}
