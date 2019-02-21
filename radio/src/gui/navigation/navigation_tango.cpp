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

vertpos_t menuVerticalOffset;
int8_t    s_editMode;
uint8_t   noHighlightCounter;
uint8_t   menuCalibrationState;
vertpos_t menuVerticalPosition;
horzpos_t menuHorizontalPosition;
int8_t    checkIncDec_Ret;

INIT_STOPS(stops100, 3, -100, 0, 100)
INIT_STOPS(stops1000, 3, -1000, 0, 1000)
INIT_STOPS(stopsSwitch, 15, SWSRC_FIRST, CATEGORY_END(-SWSRC_FIRST_LOGICAL_SWITCH), CATEGORY_END(-SWSRC_FIRST_TRIM), CATEGORY_END(-SWSRC_LAST_SWITCH+1), 0, CATEGORY_END(SWSRC_LAST_SWITCH), CATEGORY_END(SWSRC_FIRST_TRIM-1), CATEGORY_END(SWSRC_FIRST_LOGICAL_SWITCH-1), SWSRC_LAST)

int checkIncDec(event_t event, int val, int i_min, int i_max, unsigned int i_flags, IsValueAvailable isValueAvailable, const CheckIncDecStops &stops)
{
  int newval = val;

  if (s_editMode>0 && (event==EVT_KEY_FIRST(KEY_DOWN) || event==EVT_KEY_REPT(KEY_DOWN))) {
    newval += min<int>(s_evt_value, i_max - val);
    while (isValueAvailable && !isValueAvailable(newval) && newval<=i_max) {
      newval++;
    }
    if (newval > i_max) {
      newval = val;
      killEvents(event);
      AUDIO_KEY_ERROR();
    }
  }
  else if (s_editMode>0 && (event==EVT_KEY_FIRST(KEY_UP) || event==EVT_KEY_REPT(KEY_UP))) {
    newval -= min<int>(s_evt_value, val - i_min);
    while (isValueAvailable && !isValueAvailable(newval) && newval>=i_min) {
      newval--;
    }

    if (newval < i_min) {
      newval = val;
      killEvents(event);
      AUDIO_KEY_ERROR();
    }
  }

  if (!READ_ONLY() && i_min==0 && i_max==1 && event==EVT_KEY_BREAK(KEY_ENTER)) {
    s_editMode = 0;
    newval = !val;
  }

#if defined(AUTOSWITCH)
  if (i_flags & INCDEC_SWITCH) {
    newval = checkIncDecMovedSwitch(newval);
  }
#endif

#if defined(AUTOSOURCE)
  if (i_flags & INCDEC_SOURCE) {
    if (s_editMode>0) {
      int8_t source = GET_MOVED_SOURCE(i_min, i_max);
      if (source) {
        newval = source;
      }
#if defined(AUTOSWITCH)
      else {
        uint8_t swtch = abs(getMovedSwitch());
        if (swtch) {
          newval = switchToMix(swtch);
        }
      }
#endif
    }
  }
#endif

  if (newval != val) {
    if (!(i_flags & NO_INCDEC_MARKS) && (newval != i_max) && (newval != i_min) && (newval==0 || newval==-100 || newval==+100) && !IS_ROTARY_EVENT(event)) {
      pauseEvents(event); // delay before auto-repeat continues
    }
    AUDIO_KEY_PRESS();
    storageDirty(i_flags & (EE_GENERAL|EE_MODEL));
    checkIncDec_Ret = (newval > val ? 1 : -1);
  }
  else {
    checkIncDec_Ret = 0;
  }
  return newval;
}

#define SCROLL_TH      64
#define SCROLL_POT1_TH 32

#define CURSOR_NOT_ALLOWED_IN_ROW(row)   ((int8_t)MAXCOL(row) < 0)

#define INC(val, min, max)             if (val<max) {val++;} else {val=min;}
#define DEC(val, min, max)             if (val>min) {val--;} else {val=max;}

#if !defined(CPUAVR) // TODO ifdef FEATURE_MENUS_ENTRY_TIME ?
tmr10ms_t menuEntryTime;
#endif

//#define MAXCOL(row)                    (horTab ? pgm_read_byte(horTab+min(row, (vertpos_t)horTabMax)) : (const uint8_t)0)
//#define POS_HORZ_INIT(posVert)         0
#define MAXCOL_RAW(row)                (horTab ? *(horTab+min(row, (vertpos_t)horTabMax)) : (const uint8_t)0)
#define MAXCOL(row)                    (MAXCOL_RAW(row) >= HIDDEN_ROW ? MAXCOL_RAW(row) : (const uint8_t)(MAXCOL_RAW(row) & (~NAVIGATION_LINE_BY_LINE)))
#define POS_HORZ_INIT(posVert)         0

void check(event_t event, uint8_t curr, const MenuHandlerFunc *menuTab, uint8_t menuTabSize, const uint8_t *horTab, uint8_t horTabMax, vertpos_t maxrow)
{
  vertpos_t l_posVert = menuVerticalPosition;
  horzpos_t l_posHorz = menuHorizontalPosition;

  uint8_t maxcol = MAXCOL(l_posVert);

  #define scrollUD 0

  if (p2valdiff || scrollUD || p1valdiff) backlightOn(); // on keypress turn the light on

  if (menuTab) {
    uint8_t attr = 0;

    if (l_posVert==0 && !menuCalibrationState) {
      attr = INVERS;

      int8_t cc = curr;

      if (p2valdiff) {
        cc = limit((int8_t)0, (int8_t)(cc - p2valdiff), (int8_t)(menuTabSize-1));
      }

      switch (event) {
        case EVT_KEY_FIRST(KEY_LEFT):
          if (curr > 0)
            cc = curr - 1;
          else
            cc = menuTabSize-1;
          break;

        case EVT_KEY_FIRST(KEY_RIGHT):
          if (curr < (menuTabSize-1))
            cc = curr + 1;
          else
            cc = 0;
          break;
      }

      if (cc != curr) {
        //chainMenu((MenuHandlerFunc)pgm_read_adr(&menuTab[cc]));
          chainMenu(menuTab[cc]);
      }
    }

    menuCalibrationState = 0;
    drawScreenIndex(curr, menuTabSize, attr);
  }

  DISPLAY_PROGRESS_BAR(menuTab ? lcdLastRightPos-2*FW-((curr+1)/10*FWNUM)-2 : 20*FW+1);

  if (s_editMode<=0) {
    if (scrollUD) {
      l_posVert = limit((int8_t)0, (int8_t)(l_posVert - scrollUD), (int8_t)maxrow);
      l_posHorz = min((uint8_t)l_posHorz, MAXCOL(l_posVert));
    }

    if (p2valdiff && l_posVert>0) {
      l_posHorz = limit((int8_t)0, (int8_t)((uint8_t)l_posHorz - p2valdiff), (int8_t)maxcol);
    }
  }

  switch (event)
  {
    case EVT_ENTRY:
      menuEntryTime = get_tmr10ms();
      l_posVert = 0;
      l_posHorz = POS_HORZ_INIT(l_posVert);
      s_editMode = EDIT_MODE_INIT;
      break;

    case EVT_KEY_FIRST(KEY_ENTER):
      if (!menuTab || l_posVert>0) {
        if (READ_ONLY_UNLOCKED()) {
          s_editMode = (s_editMode<=0);
        }
      }
      break;

    case EVT_KEY_LONG(KEY_EXIT):
      s_editMode = 0; // TODO needed? we call ENTRY_UP after which does the same
      popMenu();
      break;

    case EVT_KEY_BREAK(KEY_EXIT):
      AUDIO_KEY_PRESS();
#if defined(ROTARY_ENCODER_NAVIGATION)
      if (s_editMode == 0)
        s_editMode = EDIT_MODE_INIT;
      else
#endif
      if (s_editMode>0) {
        s_editMode = 0;
        break;
      }

      if (l_posVert==0 || !menuTab) {
        popMenu();  // beeps itself
      }
      else {
        l_posVert = 0;
        l_posHorz = 0;
      }
      break;

    case EVT_KEY_REPT(KEY_RIGHT):  //inc
      if (l_posHorz==maxcol) break;
      // no break

    case EVT_KEY_FIRST(KEY_RIGHT)://inc
      if (!horTab || s_editMode>0) break;
      INC(l_posHorz, 0, maxcol);
      break;

#if !defined(PCBX7)
    case EVT_KEY_REPT(KEY_DOWN):
      if (!IS_ROTARY_RIGHT(event) && l_posVert==maxrow) break;
      // no break

    case EVT_KEY_FIRST(KEY_DOWN):
      if (s_editMode>0) break;
      do {
        INC(l_posVert, 0, maxrow);
      } while (CURSOR_NOT_ALLOWED_IN_ROW(l_posVert));
      l_posHorz = min<horzpos_t>(l_posHorz, MAXCOL(l_posVert));
      break;
#endif

    case EVT_KEY_REPT(KEY_LEFT):  //dec
      if (l_posHorz==0) break;
      // no break

    case EVT_KEY_FIRST(KEY_LEFT)://dec
      if (!horTab || s_editMode>0) break;
      DEC(l_posHorz, 0, maxcol);
      break;

#if !defined(PCBX7)
    case EVT_KEY_REPT(KEY_UP):
      if (!IS_ROTARY_LEFT(event) && l_posVert==0) break;
      // no break
    case EVT_KEY_FIRST(KEY_UP):
      if (s_editMode>0) break;

      do {
        DEC(l_posVert, 0, maxrow);
      } while (CURSOR_NOT_ALLOWED_IN_ROW(l_posVert));
      l_posHorz = min((uint8_t)l_posHorz, MAXCOL(l_posVert));
      break;
#endif
  }

  uint8_t maxLines = menuTab ? LCD_LINES-1 : LCD_LINES-2;

  int linesCount = maxrow;
  if (l_posVert == 0 || (l_posVert==1 && MAXCOL(vertpos_t(0)) >= HIDDEN_ROW) || (l_posVert==2 && MAXCOL(vertpos_t(0)) >= HIDDEN_ROW && MAXCOL(vertpos_t(1)) >= HIDDEN_ROW)) {
    menuVerticalOffset = 0;
    if (horTab) {
      linesCount = 0;
      for (int i=0; i<maxrow; i++) {
        if (i>=horTabMax || horTab[i] != HIDDEN_ROW) {
          linesCount++;
        }
      }
    }
  }
  else if (horTab) {
    if (maxrow > maxLines) {
      while (1) {
        vertpos_t firstLine = 0;
        for (int numLines=0; firstLine<maxrow && numLines<menuVerticalOffset; firstLine++) {
          if (firstLine>=horTabMax || horTab[firstLine+1] != HIDDEN_ROW) {
            numLines++;
          }
        }
        if (l_posVert <= firstLine) {
          menuVerticalOffset--;
        }
        else {
          vertpos_t lastLine = firstLine;
          for (int numLines=0; lastLine<maxrow && numLines<maxLines; lastLine++) {
            if (lastLine>=horTabMax || horTab[lastLine+1] != HIDDEN_ROW) {
              numLines++;
            }
          }
          if (l_posVert > lastLine) {
            menuVerticalOffset++;
          }
          else {
            linesCount = menuVerticalOffset + maxLines;
            for (int i=lastLine; i<maxrow; i++) {
              if (i>=horTabMax || horTab[i] != HIDDEN_ROW) {
                linesCount++;
              }
            }
            break;
          }
        }
      }
    }
  }
  else {
    if (l_posVert>maxLines+menuVerticalOffset) {
      menuVerticalOffset = l_posVert-maxLines;
    }
    else if (l_posVert<=menuVerticalOffset) {
      menuVerticalOffset = l_posVert-1;
    }
  }

  menuVerticalPosition = l_posVert;
  menuHorizontalPosition = l_posHorz;
  if (menuVerticalOffset > 0) {
    l_posVert--;
    if (l_posVert == menuVerticalOffset && CURSOR_NOT_ALLOWED_IN_ROW(l_posVert)) {
      menuVerticalOffset = l_posVert-1;
    }
  }
}

void check_simple(event_t event, uint8_t curr, const MenuHandlerFunc * menuTab, uint8_t menuTabSize, vertpos_t maxrow)
{
  check(event, curr, menuTab, menuTabSize, 0, 0, maxrow);
}

void check_submenu_simple(event_t event, uint8_t maxrow)
{
  check_simple(event, 0, 0, 0, maxrow);
}

void repeatLastCursorMove(event_t event)
{
  if (CURSOR_MOVED_LEFT(event) || CURSOR_MOVED_RIGHT(event)) {
    putEvent(event);
  }
  else {
    menuHorizontalPosition = 0;
  }
}

int checkIncDecSelection = 0;

void onSourceLongEnterPress(const char * result)
{
  if (result == STR_MENU_INPUTS)
    checkIncDecSelection = getFirstAvailable(MIXSRC_FIRST_INPUT, MIXSRC_LAST_INPUT, isInputAvailable)+1;
#if defined(LUA_MODEL_SCRIPTS)
    else if (result == STR_MENU_LUA)
    checkIncDecSelection = getFirstAvailable(MIXSRC_FIRST_LUA, MIXSRC_LAST_LUA, isSourceAvailable);
#endif
  else if (result == STR_MENU_STICKS)
    checkIncDecSelection = MIXSRC_FIRST_STICK;
  else if (result == STR_MENU_POTS)
    checkIncDecSelection = MIXSRC_FIRST_POT;
  else if (result == STR_MENU_MAX)
    checkIncDecSelection = MIXSRC_MAX;
  else if (result == STR_MENU_HELI)
    checkIncDecSelection = MIXSRC_FIRST_HELI;
  else if (result == STR_MENU_TRIMS)
    checkIncDecSelection = MIXSRC_FIRST_TRIM;
  else if (result == STR_MENU_SWITCHES)
    checkIncDecSelection = MIXSRC_FIRST_SWITCH;
  else if (result == STR_MENU_TRAINER)
    checkIncDecSelection = MIXSRC_FIRST_TRAINER;
  else if (result == STR_MENU_CHANNELS)
    checkIncDecSelection = getFirstAvailable(MIXSRC_FIRST_CH, MIXSRC_LAST_CH, isSourceAvailable);
  else if (result == STR_MENU_GVARS)
    checkIncDecSelection = MIXSRC_FIRST_GVAR;
  else if (result == STR_MENU_TELEMETRY) {
    for (int i = 0; i < MAX_TELEMETRY_SENSORS; i++) {
      TelemetrySensor * sensor = & g_model.telemetrySensors[i];
      if (sensor->isAvailable()) {
        checkIncDecSelection = MIXSRC_FIRST_TELEM + 3*i;
        break;
      }
    }
  }
}