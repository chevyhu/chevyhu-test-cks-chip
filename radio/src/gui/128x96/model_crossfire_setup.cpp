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

enum MenuModelSetupItems {
  ITEM_CROSSFIRE_MENU,
  ITEM_CROSSFIRE_SETUP_LINES_COUNT
};


#define CURRENT_MODULE_EDITED(k)       (EXTERNAL_MODULE)
#define MODEL_SETUP_2ND_COLUMN           (LCD_W-11*FW)
#define TIMER_ROWS                     2, 0, 0, 0, 0

void menuCrossfireSetup(event_t event)
{
  int8_t old_editMode = s_editMode;

  MENU_TAB({ HEADER_LINE_COLUMNS 0, 0, 1});

  MENU_CHECK("", menuTabModel, MENU_MODEL_CROSSFIRE, HEADER_LINE + ITEM_CROSSFIRE_SETUP_LINES_COUNT);

  TITLE("CROSSFIRE SETUP");

  if (event == EVT_ENTRY) {
  }

  uint8_t sub = menuVerticalPosition - HEADER_LINE;
  int8_t editMode = s_editMode;

  for (uint8_t i=0; i<NUM_BODY_LINES; ++i) {
    coord_t y = MENU_HEADER_HEIGHT + 1 + i * FH;
    uint8_t k = i + menuVerticalOffset;
    for (int j = 0; j <= k; j++) {
      if (mstate_tab[j + HEADER_LINE] == HIDDEN_ROW) {
        if (++k >= (int) DIM(mstate_tab)) {
          return;
        }
      }
    }

    LcdFlags blink = ((editMode > 0) ? BLINK | INVERS : INVERS);
    LcdFlags attr = (sub == k ? blink : 0);

    switch (k) {
      case ITEM_CROSSFIRE_MENU:
        lcdDrawText(0, y+10, "Crossfire Menu", attr);
        if (attr && event==EVT_KEY_BREAK(KEY_ENTER)) {
          s_editMode = 0;
          allowNewSensors = !allowNewSensors;
#if defined(LUA)
          // Start crossfire for TANGO
          luaExec("/CROSSFIRE/crossfire.lua");
#endif
        }
        break;
    }
  }
}
