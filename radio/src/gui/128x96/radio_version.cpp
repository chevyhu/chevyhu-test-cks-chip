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

void menuRadioVersion(event_t event)
{
#if defined(EEPROM_RLC)
  if (warningResult) {
    warningResult = 0;
    showMessageBox(STR_STORAGE_FORMAT);
    storageEraseAll(false);
    NVIC_SystemReset();
  }
#endif

  SIMPLE_MENU(STR_MENUVERSION, menuTabGeneral, MENU_RADIO_VERSION, 1);

  lcdDrawTextAlignedLeft(MENU_HEADER_HEIGHT+FH+16, vers_stamp);

#if defined(COPROCESSOR)
  if (Coproc_valid == 1) {
     lcdDrawTextAlignedLeft(6*FH, "CoPr:");
     lcdDraw8bitsNumber(10*FW, 6*FH, Coproc_read);
  }
  else {
     lcdDrawTextAlignedLeft(6*FH, "CoPr: ---");
  }
#elif defined(EEPROM_RLC)
  lcdDrawTextAlignedLeft(MENU_HEADER_HEIGHT+5*FH+1, STR_EEBACKUP);
  lcdDrawTextAlignedLeft(MENU_HEADER_HEIGHT+6*FH+1, STR_FACTORYRESET);
  lcdDrawFilledRect(0, MENU_HEADER_HEIGHT+5*FH, LCD_W, 2*FH+1, SOLID);

#if defined(PCBXLITE)

  if (event == EVT_KEY_LONG(KEY_ENTER)) {
    killEvents(KEY_ENTER);
    if (IS_SHIFT_PRESSED()) {
      POPUP_CONFIRMATION(STR_CONFIRMRESET);
    }
    else {
      eepromBackup();
    }
  }
#else
  if (event == EVT_KEY_LONG(KEY_ENTER)) {
    eepromBackup();
  }
  else if (event == EVT_KEY_LONG(KEY_MENU)) {
    POPUP_CONFIRMATION(STR_CONFIRMRESET);
  }
#endif
#endif
}
