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

const unsigned char ASTERISK_BITMAP[]  = {
#include "asterisk.lbm"
};

void drawAlertBox(const char * title, const char * text, const char * action)
{
  lcdClear();
  lcdDraw1bitBitmap(2, 0, ASTERISK_BITMAP, 0, 0);

#define MESSAGE_LCD_OFFSET   6*FW

#if defined(TRANSLATIONS_FR) || defined(TRANSLATIONS_IT) || defined(TRANSLATIONS_CZ)
  lcdDrawText(MESSAGE_LCD_OFFSET, 0, STR_WARNING, DBLSIZE);
  lcdDrawText(MESSAGE_LCD_OFFSET, 2*FH, title, DBLSIZE);
#else
  lcdDrawText(MESSAGE_LCD_OFFSET, 0, title, DBLSIZE);
  lcdDrawText(MESSAGE_LCD_OFFSET, 2*FH, STR_WARNING, DBLSIZE);
#endif

  lcdDrawSolidFilledRect(0, 0, LCD_W, 32);

  if (text) {
    lcdDrawTextAlignedLeft(5*FH, text);
  }

  if (action) {
    lcdDrawTextAlignedLeft(7*FH, action);
  }

#undef MESSAGE_LCD_OFFSET
}

void runPopupWarning(event_t event)
{
  warningResult = false;

  drawMessageBox(warningText);

  if (warningInfoText) {
	lcdDrawSizedText(WARNING_LINE_X, WARNING_LINE_Y+FH, warningInfoText, warningInfoLength, warningInfoFlags);
  }

  switch (warningType) {
	case WARNING_TYPE_WAIT:
	  return;

	case WARNING_TYPE_INFO:
	  lcdDrawText(WARNING_LINE_X, WARNING_LINE_Y+2*FH+2, STR_OK);
	  break;

	case WARNING_TYPE_ASTERISK:
	  lcdDrawText(WARNING_LINE_X, WARNING_LINE_Y+2*FH+2, STR_EXIT);
	  break;

	default:
	  lcdDrawText(WARNING_LINE_X, WARNING_LINE_Y+2*FH+2, STR_POPUPS_ENTER_EXIT);
	  break;
  }


  switch (event) {
	case EVT_KEY_BREAK(KEY_ENTER):
	  if (warningType == WARNING_TYPE_ASTERISK)
		// key ignored, the user has to press [EXIT]
		break;

	  if (warningType == WARNING_TYPE_CONFIRM) {
		warningType = WARNING_TYPE_ASTERISK;
		warningText = nullptr;
		if (popupMenuHandler){
		  popupMenuHandler(STR_OK);
		}
	    warningResult = true;
		break;
	  }
	  // no break

	case EVT_KEY_BREAK(KEY_EXIT):
	  if (warningType == WARNING_TYPE_CONFIRM) {
		if (popupMenuHandler)
		  popupMenuHandler(STR_EXIT);
	  }
	  warningText = nullptr;
	  warningType = WARNING_TYPE_ASTERISK;
	  break;
  }
}
