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

#include <opentx.h>
#include <math.h>

#define RECEIVER_OPTIONS_2ND_COLUMN  (200)

extern uint8_t g_moduleIdx;

void onTxOptionsUpdateConfirm(const char * result)
{
  if (result == STR_OK)
    moduleState[g_moduleIdx].readModuleSettings(&reusableBuffer.hardwareAndSettings.moduleSettings);
  else
    popMenu();
}

enum {
  ITEM_MODULE_SETTINGS_RF_PROTOCOL,
  ITEM_MODULE_SETTINGS_EXTERNAL_ANTENNA,
  ITEM_MODULE_SETTINGS_POWER,
  ITEM_MODULE_SETTINGS_COUNT
};

#define IF_MODULE_OPTIONS(option, count) uint8_t(isModuleOptionAvailable(modelId, option) ? count : HIDDEN_ROW)

bool isPowerAvailable(int value)
{
  uint8_t modelId = reusableBuffer.hardwareAndSettings.modules[g_moduleIdx].information.modelID;
  uint8_t variant = reusableBuffer.hardwareAndSettings.modules[g_moduleIdx].information.variant;

  if (modelId == PXX2_MODULE_R9M_LITE) {
    if (variant == PXX2_VARIANT_EU)
      return (value == 14 /* 25 mW */ ||
              value == 20 /* 100 mW */);
    else
      return value == 20; /* 100 mW */
  }
  else if (modelId == PXX2_MODULE_R9M || modelId == PXX2_MODULE_R9M_LITE_PRO) {
    if (variant == PXX2_VARIANT_EU)
      return (value == 14 /* 25 mW */ ||
              value == 23 /* 200 mW */ ||
              value == 27 /* 500 mW */);
    else
      return (value == 10 /* 10 mW */ ||
              value == 20 /* 100 mW */ ||
              value == 27 /* 500 mW */ ||
              value == 30 /* 1000 mW */);
  }
  else {
    return (value <= 20); /* 100 mW max for XJTs */
  }
}

bool menuModelModuleOptions(event_t event)
{
  uint8_t modelId = reusableBuffer.hardwareAndSettings.modules[g_moduleIdx].information.modelID;
  // uint8_t variant = reusableBuffer.hardwareAndSettings.modules[g_moduleIdx].information.variant;

  SUBMENU("Module Options", ICON_RADIO, ITEM_MODULE_SETTINGS_COUNT, {
    IF_MODULE_OPTIONS(MODULE_OPTION_RF_PROTOCOL, 0),
    IF_MODULE_OPTIONS(MODULE_OPTION_EXTERNAL_ANTENNA, 0),
    IF_MODULE_OPTIONS(MODULE_OPTION_POWER, 0),
  });

  if (event == EVT_ENTRY) {
#if defined(SIMU)
    reusableBuffer.hardwareAndSettings.moduleSettings.state = PXX2_SETTINGS_OK;
#else
    // no need to initialize reusableBuffer.hardwareAndSettings.moduleState.state to PXX2_HARDWARE_INFO
    moduleState[g_moduleIdx].readModuleInformation(&reusableBuffer.hardwareAndSettings.modules[g_moduleIdx], PXX2_HW_INFO_TX_ID, PXX2_HW_INFO_TX_ID);
#endif
  }

  if (reusableBuffer.hardwareAndSettings.moduleSettings.state == PXX2_HARDWARE_INFO && moduleState[g_moduleIdx].mode == MODULE_MODE_NORMAL) {
    moduleState[g_moduleIdx].readModuleSettings(&reusableBuffer.hardwareAndSettings.moduleSettings);
  }

  if (menuEvent) {
    moduleState[g_moduleIdx].mode = MODULE_MODE_NORMAL;
    if (reusableBuffer.hardwareAndSettings.moduleSettingsDirty) {
      abortPopMenu();
      POPUP_CONFIRMATION(STR_UPDATE_TX_OPTIONS, onTxOptionsUpdateConfirm);
    }
    else {
      return true;
    }
  }

  if (event == EVT_KEY_LONG(KEY_ENTER) && reusableBuffer.hardwareAndSettings.moduleSettingsDirty) {
    killEvents(event);
    reusableBuffer.hardwareAndSettings.moduleSettingsDirty = 0;
    moduleState[g_moduleIdx].writeModuleSettings(&reusableBuffer.hardwareAndSettings.moduleSettings);
  }

  if (reusableBuffer.hardwareAndSettings.moduleSettingsDirty == 2 && reusableBuffer.hardwareAndSettings.moduleSettings.state == PXX2_SETTINGS_OK) {
    popMenu();
  }

  if (modelId != 0 && mstate_tab[menuVerticalPosition] == HIDDEN_ROW) {
    menuVerticalPosition = 0;
    while (menuVerticalPosition < ITEM_MODULE_SETTINGS_COUNT && mstate_tab[menuVerticalPosition] == HIDDEN_ROW) {
      ++menuVerticalPosition;
    }
  }

  int8_t sub = menuVerticalPosition;
  lcdDrawText(300, 0, PXX2modulesModels[modelId] );

  if (reusableBuffer.hardwareAndSettings.moduleSettings.state == PXX2_SETTINGS_OK) {
    for (uint8_t k=0; k<NUM_BODY_LINES+1; k++) {
      coord_t y = MENU_HEADER_HEIGHT + 1 + k*FH;
      uint8_t i = k + menuVerticalOffset;
      for (int j=0; j<=i; ++j) {
        if (j<(int)DIM(mstate_tab) && mstate_tab[j] == HIDDEN_ROW) {
          ++i;
        }
      }
      LcdFlags attr = (sub==i ? (s_editMode>0 ? BLINK|INVERS : INVERS) : 0);

      switch (i) {
        case ITEM_MODULE_SETTINGS_RF_PROTOCOL:
          lcdDrawText(MENUS_MARGIN_LEFT, y, STR_RF_PROTOCOL);
          lcdDrawTextAtIndex(RECEIVER_OPTIONS_2ND_COLUMN, y, STR_XJT_PROTOCOLS, reusableBuffer.hardwareAndSettings.moduleSettings.rfProtocol + 1, attr);
          if (attr) {
            reusableBuffer.hardwareAndSettings.moduleSettings.rfProtocol = checkIncDec(event, reusableBuffer.hardwareAndSettings.moduleSettings.rfProtocol, RF_PROTO_X16, RF_PROTO_LAST, 0, nullptr);
            if (checkIncDec_Ret) {
              reusableBuffer.hardwareAndSettings.moduleSettingsDirty = true;
            }
          }
          break;

        case ITEM_MODULE_SETTINGS_EXTERNAL_ANTENNA:
          lcdDrawText(MENUS_MARGIN_LEFT, y, STR_EXT_ANTENNA);
          reusableBuffer.hardwareAndSettings.moduleSettings.externalAntenna = editCheckBox(reusableBuffer.hardwareAndSettings.moduleSettings.externalAntenna, RECEIVER_OPTIONS_2ND_COLUMN, y, attr, event);
          if (attr && checkIncDec_Ret) {
            reusableBuffer.hardwareAndSettings.moduleSettingsDirty = true;
          }
          break;

        case ITEM_MODULE_SETTINGS_POWER:
          lcdDrawText(MENUS_MARGIN_LEFT, y, STR_POWER);
          lcdDrawNumber(RECEIVER_OPTIONS_2ND_COLUMN, y, reusableBuffer.hardwareAndSettings.moduleSettings.txPower, attr);
          lcdDrawText(lcdNextPos, y, "dBm(");
          drawPower(lcdNextPos, y, reusableBuffer.hardwareAndSettings.moduleSettings.txPower);
          lcdDrawText(lcdNextPos, y, ")");
          if (attr) {
            reusableBuffer.hardwareAndSettings.moduleSettings.txPower = checkIncDec(event, reusableBuffer.hardwareAndSettings.moduleSettings.txPower, 0, 30, 0, &isPowerAvailable);
            if (checkIncDec_Ret) {
              reusableBuffer.hardwareAndSettings.moduleSettingsDirty = true;
            }
          }
          break;
      }
    }
  }
  else {
    lcdDrawCenteredText(LCD_H/2, STR_WAITING_FOR_TX);
  }
  return true;
}
