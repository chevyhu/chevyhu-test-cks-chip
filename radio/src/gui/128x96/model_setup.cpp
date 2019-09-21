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
#include "storage/modelslist.h"
#include "io/crsf/crossfire.h"

uint8_t g_moduleIdx;

enum MenuModelSetupItems {
  ITEM_MODEL_NAME,
  ITEM_MODEL_TIMER1,
  ITEM_MODEL_TIMER1_NAME,
  ITEM_MODEL_TIMER1_PERSISTENT,
  ITEM_MODEL_TIMER1_MINUTE_BEEP,
  ITEM_MODEL_TIMER1_COUNTDOWN_BEEP,
#if TIMERS > 1
  ITEM_MODEL_TIMER2,
  ITEM_MODEL_TIMER2_NAME,
  ITEM_MODEL_TIMER2_PERSISTENT,
  ITEM_MODEL_TIMER2_MINUTE_BEEP,
  ITEM_MODEL_TIMER2_COUNTDOWN_BEEP,
#endif
#if TIMERS > 2
  ITEM_MODEL_TIMER3,
  ITEM_MODEL_TIMER3_NAME,
  ITEM_MODEL_TIMER3_PERSISTENT,
  ITEM_MODEL_TIMER3_MINUTE_BEEP,
  ITEM_MODEL_TIMER3_COUNTDOWN_BEEP,
#endif
#if defined(PCBX9E)
  ITEM_MODEL_TOP_LCD_TIMER,
#endif
  ITEM_MODEL_EXTENDED_LIMITS,
  ITEM_MODEL_EXTENDED_TRIMS,
  ITEM_MODEL_DISPLAY_TRIMS,
  ITEM_MODEL_TRIM_INC,
  ITEM_MODEL_THROTTLE_LABEL,
  ITEM_MODEL_THROTTLE_REVERSED,
  ITEM_MODEL_THROTTLE_TRACE,
  ITEM_MODEL_THROTTLE_TRIM,
  ITEM_MODEL_PREFLIGHT_LABEL,
  ITEM_MODEL_CHECKLIST_DISPLAY,
  ITEM_MODEL_THROTTLE_WARNING,
  ITEM_MODEL_SWITCHES_WARNING,
#if defined(PCBX9E)
  ITEM_MODEL_SWITCHES_WARNING2,
  ITEM_MODEL_SWITCHES_WARNING3,
#endif
  ITEM_MODEL_POTS_WARNING,
#if defined(PCBX9E)
  ITEM_MODEL_POTS_WARNING2,
#endif
  ITEM_MODEL_BEEP_CENTER,
  ITEM_MODEL_USE_GLOBAL_FUNCTIONS,
  ITEM_MODEL_EXTERNAL_MODULE_LABEL,
  ITEM_MODEL_EXTERNAL_MODULE_MODE,
  ITEM_MODEL_SETUP_MAX
};

#define FIELD_PROTOCOL_MAX 1

#define MODEL_SETUP_2ND_COLUMN           (LCD_W-11*FW)
#define MODEL_SETUP_3RD_COLUMN           (MODEL_SETUP_2ND_COLUMN+6*FW)
#define MODEL_SETUP_BIND_OFS             2*FW+1
#define MODEL_SETUP_RANGE_OFS            4*FW+3
#define MODEL_SETUP_SET_FAILSAFE_OFS     7*FW-2


#define CURRENT_MODULE_EDITED(k)      (k >= ITEM_MODEL_EXTERNAL_MODULE_LABEL ? EXTERNAL_MODULE : INTERNAL_MODULE)

void onBindMenu(const char * result)
{
  uint8_t moduleIdx = CURRENT_MODULE_EDITED(menuVerticalPosition);

  if (result == STR_BINDING_1_8_TELEM_ON) {
    g_model.moduleData[moduleIdx].pxx.receiver_telem_off = false;
    g_model.moduleData[moduleIdx].pxx.receiver_channel_9_16 = false;
  }
  else if (result == STR_BINDING_1_8_TELEM_OFF) {
    g_model.moduleData[moduleIdx].pxx.receiver_telem_off = true;
    g_model.moduleData[moduleIdx].pxx.receiver_channel_9_16 = false;
  }
  else if (result == STR_BINDING_9_16_TELEM_ON) {
    g_model.moduleData[moduleIdx].pxx.receiver_telem_off = false;
    g_model.moduleData[moduleIdx].pxx.receiver_channel_9_16 = true;
  }
  else if (result == STR_BINDING_9_16_TELEM_OFF) {
    g_model.moduleData[moduleIdx].pxx.receiver_telem_off = true;
    g_model.moduleData[moduleIdx].pxx.receiver_channel_9_16 = true;
  }
  else {
    return;
  }

  moduleState[moduleIdx].mode = MODULE_MODE_BIND;
}


void onModelSetupBitmapMenu(const char * result)
{
  if (result == STR_UPDATE_LIST) {
    if (!sdListFiles(BITMAPS_PATH, BITMAPS_EXT, sizeof(g_model.header.bitmap), NULL)) {
      POPUP_WARNING(STR_NO_BITMAPS_ON_SD);
    }
  }
  else if (result != STR_EXIT) {
    // The user choosed a bmp file in the list
    copySelection(g_model.header.bitmap, result, sizeof(g_model.header.bitmap));
    memcpy(modelHeaders[g_eeGeneral.currModel].bitmap, g_model.header.bitmap, sizeof(g_model.header.bitmap));
    storageDirty(EE_MODEL);
  }
}

void editTimerMode(int timerIdx, coord_t y, LcdFlags attr, event_t event)
{
  TimerData & timer = g_model.timers[timerIdx];
  drawStringWithIndex(0*FW, y, STR_TIMER, timerIdx+1);
  drawTimerMode(MODEL_SETUP_2ND_COLUMN, y, timer.mode, menuHorizontalPosition==0 ? attr : 0);
  drawTimer(MODEL_SETUP_3RD_COLUMN, y, timer.start, menuHorizontalPosition==1 ? attr|TIMEHOUR : TIMEHOUR, menuHorizontalPosition==2 ? attr|TIMEHOUR : TIMEHOUR);
  if (attr && menuHorizontalPosition < 0) {
    lcdDrawFilledRect(MODEL_SETUP_2ND_COLUMN-1, y-1, 13*FW-3, FH+1);
  }
  if (attr && s_editMode>0) {
    div_t qr = div(timer.start, 60);
    switch (menuHorizontalPosition) {
      case 0:
      {
        swsrc_t timerMode = timer.mode;
        if (timerMode < 0) timerMode -= TMRMODE_COUNT-1;
        CHECK_INCDEC_MODELVAR_CHECK(event, timerMode, -TMRMODE_COUNT-SWSRC_LAST+1, TMRMODE_COUNT+SWSRC_LAST-1, isSwitchAvailableInTimers);
        if (timerMode < 0) timerMode += TMRMODE_COUNT-1;
        timer.mode = timerMode;
#if defined(AUTOSWITCH)
        if (s_editMode>0) {
          swsrc_t val = timer.mode - (TMRMODE_COUNT-1);
          swsrc_t switchVal = checkIncDecMovedSwitch(val);
          if (val != switchVal) {
            timer.mode = switchVal + (TMRMODE_COUNT-1);
            storageDirty(EE_MODEL);
          }
        }
#endif
        break;
      }
      case 1:
        qr.quot = checkIncDec(event, qr.quot, 0, 1439, EE_MODEL | NO_INCDEC_MARKS); // 23h59
        timer.start = qr.rem + qr.quot*60;
        break;
      case 2:
        qr.rem -= checkIncDecModel(event, qr.rem+2, 1, 62)-2;
        timer.start -= qr.rem ;
        if ((int16_t)timer.start < 0) timer.start=0;
        if ((int32_t)timer.start > 86399) timer.start=86399; // 23h59:59
        break;
    }
  }
}

void editTimerCountdown(int timerIdx, coord_t y, LcdFlags attr, event_t event)
{
  TimerData & timer = g_model.timers[timerIdx];
  lcdDrawTextAlignedLeft(y, STR_BEEPCOUNTDOWN);
  lcdDrawTextAtIndex(MODEL_SETUP_2ND_COLUMN, y, STR_VBEEPCOUNTDOWN, timer.countdownBeep, (menuHorizontalPosition==0 ? attr : 0));
  if (timer.countdownBeep != COUNTDOWN_SILENT) {
    lcdDrawNumber(MODEL_SETUP_3RD_COLUMN, y, TIMER_COUNTDOWN_START(timerIdx), (menuHorizontalPosition == 1 ? attr : 0) | LEFT);
    lcdDrawChar(lcdLastRightPos, y, 's');
  }
  if (attr && s_editMode>0) {
    switch (menuHorizontalPosition) {
      case 0:
        CHECK_INCDEC_MODELVAR(event, timer.countdownBeep, COUNTDOWN_SILENT, COUNTDOWN_COUNT - 1);
        break;
      case 1:
        timer.countdownStart = -checkIncDecModel(event, -timer.countdownStart, -1, +2);
        break;
    }
  }
}

int getSwitchWarningsCount()
{
  int count = 0;
  for (int i=0; i<NUM_SWITCHES; ++i) {
    if (SWITCH_WARNING_ALLOWED(i)) {
      ++count;
    }
  }
  return count;
}

#define IF_INTERNAL_MODULE_ON(x)        (IS_INTERNAL_MODULE_ENABLED() ? (uint8_t)(x) : HIDDEN_ROW)
#if defined(TARANIS_INTERNAL_PPM)
#define INTERNAL_MODULE_MODE_ROWS       (isModuleXJT(INTERNAL_MODULE) ? (uint8_t)1 : (uint8_t)0) // Module type + RF protocols
#else
#define INTERNAL_MODULE_MODE_ROWS       0 // (OFF / RF protocols)
#endif
#define IF_EXTERNAL_MODULE_ON(x)          (IS_EXTERNAL_MODULE_ENABLED() ? (uint8_t)(x) : HIDDEN_ROW)

#define PORT_CHANNELS_ROWS(x)             (x==INTERNAL_MODULE ? INTERNAL_MODULE_CHANNELS_ROWS : (x==EXTERNAL_MODULE ? EXTERNAL_MODULE_CHANNELS_ROWS : 1))

#if defined(BLUETOOTH) && defined(USEHORUSBT)
#define TRAINER_LINE1_BLUETOOTH_M_ROWS    ((bluetooth.distantAddr[0] == 0 || bluetooth.state == BLUETOOTH_STATE_CONNECTED) ? (uint8_t)0 : (uint8_t)1)
  #define TRAINER_LINE1_ROWS                (g_model.trainerData.mode == TRAINER_MODE_SLAVE ? (uint8_t)1 : (g_model.trainerData.mode == TRAINER_MODE_MASTER_BLUETOOTH ? TRAINER_LINE1_BLUETOOTH_M_ROWS : (g_model.trainerData.mode == TRAINER_MODE_SLAVE_BLUETOOTH ? (uint8_t)1 : HIDDEN_ROW)))
  #define TRAINER_LINE2_ROWS                (g_model.trainerData.mode == TRAINER_MODE_SLAVE ? (uint8_t)2 : HIDDEN_ROW)
#else
#define TRAINER_LINE1_ROWS                (g_model.trainerData.mode == TRAINER_MODE_SLAVE ? (uint8_t)1 : HIDDEN_ROW)
#define TRAINER_LINE2_ROWS                (g_model.trainerData.mode == TRAINER_MODE_SLAVE ? (uint8_t)2 : HIDDEN_ROW)
#endif

#define TIMER_ROWS(x)                     2|NAVIGATION_LINE_BY_LINE, 0, 0, 0, g_model.timers[x].countdownBeep != COUNTDOWN_SILENT ? (uint8_t) 1 : (uint8_t)0

#define EXTERNAL_MODULE_MODE_ROWS         (isModuleXJT(EXTERNAL_MODULE) || isModuleR9M(EXTERNAL_MODULE) || isModuleDSM2(EXTERNAL_MODULE) || isModuleMultimodule(EXTERNAL_MODULE)) ? (uint8_t)1 : (uint8_t)0

#if TIMERS == 1
#define TIMERS_ROWS                     TIMER_ROWS(0)
#elif TIMERS == 2
#define TIMERS_ROWS                     TIMER_ROWS(0), TIMER_ROWS(1)
#elif TIMERS == 3
#define TIMERS_ROWS                     TIMER_ROWS(0), TIMER_ROWS(1), TIMER_ROWS(2)
#endif
#if defined(PCBX9E)
#define SW_WARN_ITEMS()                 uint8_t(NAVIGATION_LINE_BY_LINE|(getSwitchWarningsCount()-1)), uint8_t(getSwitchWarningsCount() > 8 ? TITLE_ROW : HIDDEN_ROW), uint8_t(getSwitchWarningsCount() > 16 ? TITLE_ROW : HIDDEN_ROW)
  #define POT_WARN_ITEMS()                uint8_t(g_model.potsWarnMode ? NAVIGATION_LINE_BY_LINE|(NUM_POTS+NUM_SLIDERS) : 0), uint8_t(g_model.potsWarnMode ? TITLE_ROW : HIDDEN_ROW)
  #define TOPLCD_ROWS                     0,
#else
#define SW_WARN_ITEMS()                 uint8_t(NAVIGATION_LINE_BY_LINE|getSwitchWarningsCount())
#define POT_WARN_ITEMS()                uint8_t(g_model.potsWarnMode ? NAVIGATION_LINE_BY_LINE|(NUM_POTS+NUM_SLIDERS) : 0)
#define TOPLCD_ROWS
#endif

void checkModelIdUnique(uint8_t moduleIdx)
{
  if(isModulePXX(moduleIdx) && IS_D8_RX(moduleIdx))
    return;

  char* warn_buf = reusableBuffer.moduleSetup.msg;

  // cannot rely exactly on WARNING_LINE_LEN so using WARNING_LINE_LEN-2
  size_t warn_buf_len = sizeof(reusableBuffer.moduleSetup.msg) - WARNING_LINE_LEN - 2;
  if (!modelslist.isModelIdUnique(moduleIdx,warn_buf,warn_buf_len)) {
    if (warn_buf[0] != 0) {
      POPUP_WARNING(STR_MODELIDUSED);
      SET_WARNING_INFO(warn_buf, sizeof(reusableBuffer.moduleSetup.msg), 0);
    }
  }
}

void menuModelSetup(event_t event)
{
  horzpos_t l_posHorz = menuHorizontalPosition;
  bool CURSOR_ON_CELL = (menuHorizontalPosition >= 0);

  MENU_TAB({ 0, TIMERS_ROWS, TOPLCD_ROWS 0, 1, 0, 0,
             LABEL(Throttle), 0, 0, 0,
             LABEL(PreflightCheck), 0, 0, SW_WARN_ITEMS(), POT_WARN_ITEMS(), NAVIGATION_LINE_BY_LINE|(NUM_STICKS+NUM_POTS+NUM_SLIDERS-1), 0,
             LABEL(ExternalModule),
             EXTERNAL_MODULE_MODE_ROWS});

  MENU_CHECK(STR_MENUSETUP, menuTabModel, MENU_MODEL_SETUP, ITEM_MODEL_SETUP_MAX);

  if (event == EVT_ENTRY) {
    reusableBuffer.moduleSetup.r9mPower = g_model.moduleData[EXTERNAL_MODULE].pxx.power;
  }

#if (defined(DSM2) || defined(PXX))
  if (menuEvent) {
    moduleState[0].mode = 0;
    moduleState[1].mode = 0;
  }
#endif

  int sub = menuVerticalPosition;

  for (int i=0; i<NUM_BODY_LINES; ++i) {
    coord_t y = MENU_HEADER_HEIGHT + 1 + i*FH;
    uint8_t k = i+menuVerticalOffset;
    for (int j=0; j<=k; j++) {
      if (mstate_tab[j] == HIDDEN_ROW)
        k++;
    }

    LcdFlags blink = ((s_editMode>0) ? BLINK|INVERS : INVERS);
    LcdFlags attr = (sub == k ? blink : 0);

    switch(k) {
      case ITEM_MODEL_NAME:
        editSingleName(MODEL_SETUP_2ND_COLUMN, y, STR_MODELNAME, g_model.header.name, sizeof(g_model.header.name),
                       event, attr);
        memcpy(modelHeaders[g_eeGeneral.currModel].name, g_model.header.name, sizeof(g_model.header.name));
        break;

      case ITEM_MODEL_TIMER1:
        editTimerMode(0, y, attr, event);
        break;

      case ITEM_MODEL_TIMER1_NAME:
        editSingleName(MODEL_SETUP_2ND_COLUMN, y, STR_TIMER_NAME, g_model.timers[0].name, LEN_TIMER_NAME, event, attr);
        break;

      case ITEM_MODEL_TIMER1_MINUTE_BEEP:
        g_model.timers[0].minuteBeep = editCheckBox(g_model.timers[0].minuteBeep, MODEL_SETUP_2ND_COLUMN, y,
                                                    STR_MINUTEBEEP, attr, event);
        break;

      case ITEM_MODEL_TIMER1_COUNTDOWN_BEEP:
        editTimerCountdown(0, y, attr, event);
        break;

      case ITEM_MODEL_TIMER1_PERSISTENT:
        g_model.timers[0].persistent = editChoice(MODEL_SETUP_2ND_COLUMN, y, STR_PERSISTENT, STR_VPERSISTENT,
                                                  g_model.timers[0].persistent, 0, 2, attr, event);
        break;

#if TIMERS > 1
      case ITEM_MODEL_TIMER2:
        editTimerMode(1, y, attr, event);
        break;

      case ITEM_MODEL_TIMER2_NAME:
        editSingleName(MODEL_SETUP_2ND_COLUMN, y, STR_TIMER_NAME, g_model.timers[1].name, LEN_TIMER_NAME, event, attr);
        break;

      case ITEM_MODEL_TIMER2_MINUTE_BEEP:
        g_model.timers[1].minuteBeep = editCheckBox(g_model.timers[1].minuteBeep, MODEL_SETUP_2ND_COLUMN, y,
                                                    STR_MINUTEBEEP, attr, event);
        break;

      case ITEM_MODEL_TIMER2_COUNTDOWN_BEEP:
        editTimerCountdown(1, y, attr, event);
        break;

      case ITEM_MODEL_TIMER2_PERSISTENT:
        g_model.timers[1].persistent = editChoice(MODEL_SETUP_2ND_COLUMN, y, STR_PERSISTENT, STR_VPERSISTENT,
                                                  g_model.timers[1].persistent, 0, 2, attr, event);
        break;
#endif

#if TIMERS > 2
      case ITEM_MODEL_TIMER3:
        editTimerMode(2, y, attr, event);
        break;

      case ITEM_MODEL_TIMER3_NAME:
        editSingleName(MODEL_SETUP_2ND_COLUMN, y, STR_TIMER_NAME, g_model.timers[2].name, LEN_TIMER_NAME, event, attr);
        break;

      case ITEM_MODEL_TIMER3_MINUTE_BEEP:
        g_model.timers[2].minuteBeep = editCheckBox(g_model.timers[2].minuteBeep, MODEL_SETUP_2ND_COLUMN, y,
                                                    STR_MINUTEBEEP, attr, event);
        break;

      case ITEM_MODEL_TIMER3_COUNTDOWN_BEEP:
        editTimerCountdown(2, y, attr, event);
        break;

      case ITEM_MODEL_TIMER3_PERSISTENT:
        g_model.timers[2].persistent = editChoice(MODEL_SETUP_2ND_COLUMN, y, STR_PERSISTENT, STR_VPERSISTENT,
                                                  g_model.timers[2].persistent, 0, 2, attr, event);
        break;
#endif

#if defined(PCBX9E)
      case ITEM_MODEL_TOP_LCD_TIMER:
        lcdDrawTextAlignedLeft(y, STR_TOPLCDTIMER);
        drawStringWithIndex(MODEL_SETUP_2ND_COLUMN, y, STR_TIMER, g_model.toplcdTimer+1, attr);
        if (attr) {
          g_model.toplcdTimer = checkIncDec(event, g_model.toplcdTimer, 0, TIMERS-1, EE_MODEL);
        }
        break;
#endif

      case ITEM_MODEL_EXTENDED_LIMITS:
        ON_OFF_MENU_ITEM(g_model.extendedLimits, MODEL_SETUP_2ND_COLUMN, y, STR_ELIMITS, attr, event);
        break;

      case ITEM_MODEL_EXTENDED_TRIMS:
        ON_OFF_MENU_ITEM(g_model.extendedTrims, MODEL_SETUP_2ND_COLUMN, y, STR_ETRIMS,
                         menuHorizontalPosition <= 0 ? attr : 0, event == EVT_KEY_BREAK(KEY_ENTER) ? event : 0);
        lcdDrawText(MODEL_SETUP_2ND_COLUMN + 3 * FW, y, STR_RESET_BTN,
                    (menuHorizontalPosition > 0 && !NO_HIGHLIGHT()) ? attr : 0);
        if (attr && menuHorizontalPosition > 0) {
          s_editMode = 0;
          if (event == EVT_KEY_LONG(KEY_ENTER)) {
            START_NO_HIGHLIGHT();
            for (uint8_t i = 0; i < MAX_FLIGHT_MODES; i++) {
              memclear(&g_model.flightModeData[i], TRIMS_ARRAY_SIZE);
            }
            storageDirty(EE_MODEL);
            AUDIO_WARNING1();
          }
        }
        break;

      case ITEM_MODEL_DISPLAY_TRIMS:
        g_model.displayTrims = editChoice(MODEL_SETUP_2ND_COLUMN, y, STR_DISPLAY_TRIMS, STR_VDISPLAYTRIMS,
                                          g_model.displayTrims, 0, 2, attr, event);
        break;

      case ITEM_MODEL_TRIM_INC:
        g_model.trimInc = editChoice(MODEL_SETUP_2ND_COLUMN, y, STR_TRIMINC, STR_VTRIMINC, g_model.trimInc, -2, 2, attr,
                                     event);
        break;

      case ITEM_MODEL_THROTTLE_LABEL:
        lcdDrawTextAlignedLeft(y, STR_THROTTLE_LABEL);
        break;

      case ITEM_MODEL_THROTTLE_REVERSED:
        ON_OFF_MENU_ITEM(g_model.throttleReversed, MODEL_SETUP_2ND_COLUMN, y, STR_THROTTLEREVERSE, attr, event);
        break;

      case ITEM_MODEL_THROTTLE_TRACE: {
        lcdDrawTextAlignedLeft(y, STR_TTRACE);
        if (attr)
          CHECK_INCDEC_MODELVAR_ZERO_CHECK(event, g_model.thrTraceSrc, NUM_POTS + NUM_SLIDERS + MAX_OUTPUT_CHANNELS,
                                           isThrottleSourceAvailable);
        uint8_t idx = g_model.thrTraceSrc + MIXSRC_Thr;
        if (idx > MIXSRC_Thr)
          idx += 1;
        if (idx >= MIXSRC_FIRST_POT + NUM_POTS + NUM_SLIDERS)
          idx += MIXSRC_CH1 - MIXSRC_FIRST_POT - NUM_POTS - NUM_SLIDERS;
        drawSource(MODEL_SETUP_2ND_COLUMN, y, idx, attr);
        break;
      }

      case ITEM_MODEL_THROTTLE_TRIM:
        ON_OFF_MENU_ITEM(g_model.thrTrim, MODEL_SETUP_2ND_COLUMN, y, STR_TTRIM, attr, event);
        break;

      case ITEM_MODEL_PREFLIGHT_LABEL:
        lcdDrawTextAlignedLeft(y, STR_PREFLIGHT);
        break;

      case ITEM_MODEL_CHECKLIST_DISPLAY:
        ON_OFF_MENU_ITEM(g_model.displayChecklist, MODEL_SETUP_2ND_COLUMN, y, STR_CHECKLIST, attr, event);
        break;

      case ITEM_MODEL_THROTTLE_WARNING:
        g_model.disableThrottleWarning = !editCheckBox(!g_model.disableThrottleWarning, MODEL_SETUP_2ND_COLUMN, y,
                                                       STR_THROTTLEWARNING, attr, event);
        break;

#if defined(PCBX9E)
      case ITEM_MODEL_SWITCHES_WARNING2:
      case ITEM_MODEL_SWITCHES_WARNING3:
      case ITEM_MODEL_POTS_WARNING2:
        if (i==0) {
          if (CURSOR_MOVED_LEFT(event))
            menuVerticalOffset--;
          else
            menuVerticalOffset++;
        }
        break;
#endif

      case ITEM_MODEL_SWITCHES_WARNING: {
#if defined(PCBX9E)
        if (i>=NUM_BODY_LINES-2 && getSwitchWarningsCount() > 8*(NUM_BODY_LINES-i)) {
          if (CURSOR_MOVED_LEFT(event))
            menuVerticalOffset--;
          else
            menuVerticalOffset++;
          break;
        }
#endif
        lcdDrawTextAlignedLeft(y, STR_SWITCHWARNING);
        swarnstate_t states = g_model.switchWarningState;
        char c;
        if (attr) {
          s_editMode = 0;
          if (!READ_ONLY()) {
            switch (event) {
              case EVT_KEY_BREAK(KEY_ENTER):
                break;

              case EVT_KEY_LONG(KEY_ENTER):
                if (menuHorizontalPosition < 0) {
                  START_NO_HIGHLIGHT();
                  getMovedSwitch();
                  g_model.switchWarningState = switches_states;
                  AUDIO_WARNING1();
                  storageDirty(EE_MODEL);
                }
                killEvents(event);
                break;
            }
          }
        }

        LcdFlags line = attr;

        int current = 0;
        for (int i = 0; i < NUM_SWITCHES; i++) {
          if (SWITCH_WARNING_ALLOWED(i)) {
            div_t qr = div(current, 8);
            if (!READ_ONLY() && event == EVT_KEY_BREAK(KEY_ENTER) && line && l_posHorz == current) {
              g_model.switchWarningEnable ^= (1 << i);
              storageDirty(EE_MODEL);
            }
            uint8_t swactive = !(g_model.switchWarningEnable & (1 << i));
            c = "\300-\301"[states & 0x03];
            lcdDrawChar(MODEL_SETUP_2ND_COLUMN + qr.rem * (2 * FW + 1), y + FH * qr.quot, 'A' + i,
                        line && (menuHorizontalPosition == current) ? INVERS : 0);
            if (swactive) lcdDrawChar(lcdNextPos, y + FH * qr.quot, c);
            ++current;
          }
          states >>= 2;
        }
        if (attr && menuHorizontalPosition < 0) {
#if defined(PCBX9E)
          lcdDrawFilledRect(MODEL_SETUP_2ND_COLUMN-1, y-1, 8*(2*FW+1), 1+FH*((current+7)/8));
#else
          lcdDrawFilledRect(MODEL_SETUP_2ND_COLUMN - 1, y - 1, current * (2 * FW + 1), FH + 1);
#endif
        }
        break;
      }

      case ITEM_MODEL_POTS_WARNING:
#if defined(PCBX9E)
        if (i==NUM_BODY_LINES-1 && g_model.potsWarnMode) {
          if (CURSOR_MOVED_LEFT(event))
            menuVerticalOffset--;
          else
            menuVerticalOffset++;
          break;
        }
#endif

        lcdDrawTextAlignedLeft(y, STR_POTWARNING);
        lcdDrawTextAtIndex(MODEL_SETUP_2ND_COLUMN, y, "\004""OFF\0""Man\0""Auto", g_model.potsWarnMode,
                           (menuHorizontalPosition == 0) ? attr : 0);
        if (attr && (menuHorizontalPosition == 0)) {
          CHECK_INCDEC_MODELVAR(event, g_model.potsWarnMode, POTS_WARN_OFF, POTS_WARN_AUTO);
          storageDirty(EE_MODEL);
        }

        if (attr) {
          if (menuHorizontalPosition > 0) s_editMode = 0;
          if (!READ_ONLY() && menuHorizontalPosition > 0) {
            switch (event) {
              case EVT_KEY_LONG(KEY_ENTER):
                killEvents(event);
                if (g_model.potsWarnMode == POTS_WARN_MANUAL) {
                  SAVE_POT_POSITION(menuHorizontalPosition - 1);
                  AUDIO_WARNING1();
                  storageDirty(EE_MODEL);
                }
                break;
              case EVT_KEY_BREAK(KEY_ENTER):
                g_model.potsWarnEnabled ^= (1 << (menuHorizontalPosition - 1));
                storageDirty(EE_MODEL);
                break;
            }
          }
        }
        if (g_model.potsWarnMode) {
          coord_t x = MODEL_SETUP_2ND_COLUMN + 28;
          for (int i = 0; i < NUM_POTS + NUM_SLIDERS; ++i) {
            if (i < NUM_XPOTS && !IS_POT_SLIDER_AVAILABLE(POT1 + i)) {
              if (attr && (menuHorizontalPosition == i + 1)) REPEAT_LAST_CURSOR_MOVE();
            } else {
#if defined(PCBX9E)
              if (i == NUM_XPOTS) {
                y += FH;
                x = MODEL_SETUP_2ND_COLUMN;
              }
#endif
              LcdFlags flags = ((menuHorizontalPosition == i + 1) && attr) ? BLINK : 0;
              if ((!attr || menuHorizontalPosition >= 0) && !(g_model.potsWarnEnabled & (1 << i))) {
                flags |= INVERS;
              }

              // TODO add a new function
              lcdDrawSizedText(x, y, STR_VSRCRAW + 2 + STR_VSRCRAW[0] * (NUM_STICKS + 1 + i), STR_VSRCRAW[0] - 1,
                               flags & ~ZCHAR);
              x = lcdNextPos + 3;
            }
          }
        }
        if (attr && menuHorizontalPosition < 0) {
#if defined(PCBX9E)
          lcdDrawFilledRect(MODEL_SETUP_2ND_COLUMN-1, y-FH-1, LCD_W-MODEL_SETUP_2ND_COLUMN-MENUS_SCROLLBAR_WIDTH+1, 2*FH+1);
#else
          lcdDrawFilledRect(MODEL_SETUP_2ND_COLUMN - 1, y - 1,
                            LCD_W - MODEL_SETUP_2ND_COLUMN - MENUS_SCROLLBAR_WIDTH + 1, FH + 1);
#endif
        }
        break;

      case ITEM_MODEL_BEEP_CENTER: {
        lcdDrawTextAlignedLeft(y, STR_BEEPCTR);
        coord_t x = MODEL_SETUP_2ND_COLUMN;
        for (int i = 0; i < NUM_STICKS + NUM_POTS + NUM_SLIDERS; i++) {
          if (i >= POT1 && i < POT1 + NUM_XPOTS && !IS_POT_SLIDER_AVAILABLE(i)) {
            if (attr && menuHorizontalPosition == i) REPEAT_LAST_CURSOR_MOVE();
            continue;
          }
          lcdDrawTextAtIndex(x, y, STR_RETA123, i,
                             ((menuHorizontalPosition == i) && attr) ? BLINK | INVERS : (((g_model.beepANACenter &
                                                                                           ((BeepANACenter) 1 << i)) ||
                                                                                          (attr && CURSOR_ON_LINE()))
                                                                                         ? INVERS : 0));
          x += FW;
        }
        if (attr && CURSOR_ON_CELL) {
          if (event == EVT_KEY_BREAK(KEY_ENTER)) {
            if (READ_ONLY_UNLOCKED()) {
              s_editMode = 0;
              g_model.beepANACenter ^= ((BeepANACenter) 1 << menuHorizontalPosition);
              storageDirty(EE_MODEL);
            }
          }
        }
        break;
      }

      case ITEM_MODEL_USE_GLOBAL_FUNCTIONS:
        lcdDrawTextAlignedLeft(y, STR_USE_GLOBAL_FUNCS);
        drawCheckBox(MODEL_SETUP_2ND_COLUMN, y, !g_model.noGlobalFunctions, attr);
        if (attr) g_model.noGlobalFunctions = !checkIncDecModel(event, !g_model.noGlobalFunctions, 0, 1);
        break;

      case ITEM_MODEL_EXTERNAL_MODULE_LABEL:
        lcdDrawTextAlignedLeft(y, "crossfire");
        break;

      case ITEM_MODEL_EXTERNAL_MODULE_MODE:
      {
        uint8_t moduleIdx = CURRENT_MODULE_EDITED(k);
        lcdDrawTextAlignedLeft(y, STR_RECEIVER_NUM);
        lcdDrawNumber(MODEL_SETUP_2ND_COLUMN, y, g_model.header.modelId[moduleIdx], attr | LEADING0 | LEFT, 2);
        if (attr) {
          CHECK_INCDEC_MODELVAR_ZERO(event, g_model.header.modelId[moduleIdx], MAX_RX_NUM(moduleIdx));
          if (checkIncDec_Ret) {
            modelHeaders[g_eeGeneral.currModel].modelId[moduleIdx] = g_model.header.modelId[moduleIdx];
            set_model_id_needed = true;
          }
        }
        break;
      }
    }
  }
}
