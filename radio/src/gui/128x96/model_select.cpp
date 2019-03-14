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

#define MODELSIZE_POS_X 170
#define MODELSEL_W 133


#define selectModel(a)
#define eeBackupModel(a)  "Pop up"
#define strcat_modelname(a, b)
#define eeRestoreModel(a, b) "Restore"
#define eeLoadModel(a)
#define eeDeleteModel(a)
#define eeCopyModel(a, b) false
#define eeSwapModels(a, b)
#define eeFindEmptyModel(a, b) 0
#define EeFsGetFree(a) 0
#define eeModelSize(a) 0

uint16_t categoriesVerticalOffset = 0;
uint16_t categoriesVerticalPosition = 0;
#define MODEL_INDEX()       (menuVerticalPosition*2+menuHorizontalPosition)

ModelsCategory * currentCategory;
int currentCategoryIndex;
ModelCell * currentModel;


bool eeModelExists(uint8_t id)
{
  int index = 0;

  for (ModelsCategory::iterator it = currentCategory->begin(); it != currentCategory->end(); ++it, ++index) {
    if (id == index){
      return true;
    }
  }

  return false;
}

void setCurrentModel(unsigned int index)
{
  std::list<ModelCell *>::iterator it = currentCategory->begin();
  std::advance(it, index);
  currentModel = *it;
  menuVerticalPosition = index / 2;
  menuHorizontalPosition = index & 1;
  menuVerticalOffset = limit<int>(menuVerticalPosition-2, menuVerticalOffset, min<int>(menuVerticalPosition, max<int>(0, (currentCategory->size()-7)/2)));
}

void setCurrentCategory(unsigned int index)
{
  currentCategoryIndex = index;
  const std::list<ModelsCategory *>& cats = modelslist.getCategories();
  std::list<ModelsCategory *>::const_iterator it = cats.begin();
  std::advance(it, index);
  currentCategory = *it;
  categoriesVerticalPosition = index;
  categoriesVerticalOffset = limit<int>(categoriesVerticalPosition-4, categoriesVerticalOffset, min<int>(categoriesVerticalPosition, max<int>(0, cats.size()-5)));
  if (currentCategory->size() > 0)
    setCurrentModel(0);
  else
    currentModel = NULL;
}

void onModelSelectMenu(const char * result)
{
  int8_t sub = menuVerticalPosition;

  if (result == STR_SELECT_MODEL) {
    selectModel(sub);
  }
  else if (result == STR_CREATE_MODEL) {
    storageCheck(true);
    modelslist.addModel(currentCategory, createModel());
    //s_editMode = MODE_SELECT_MODEL;
    setCurrentModel(currentCategory->size() - 1);
    modelslist.setCurrentModel(currentModel);
    modelslist.onNewModelCreated(currentModel, &g_model);
#if defined(LUA)
    //chainMenu(menuModelWizard);
#endif
  }
  else if (result == STR_COPY_MODEL) {
    s_copyMode = COPY_MODE;
    s_copyTgtOfs = 0;
    s_copySrcRow = -1;
  }
  else if (result == STR_MOVE_MODEL) {
    s_copyMode = MOVE_MODE;
    s_copyTgtOfs = 0;
    s_copySrcRow = -1;
  }
  else if (result == STR_BACKUP_MODEL) {
    storageCheck(true); // force writing of current model data before this is changed
    POPUP_WARNING(eeBackupModel(sub));
  }
  else if (result == STR_RESTORE_MODEL || result == STR_UPDATE_LIST) {
    if (!sdListFiles(MODELS_PATH, MODELS_EXT, MENU_LINE_LENGTH-1, NULL)) {
      POPUP_WARNING(STR_NO_MODELS_ON_SD);
    }
  }
  else if (result == STR_DELETE_MODEL) {
    char * nametmp =  reusableBuffer.modelsel.mainname;
    strcat_modelname (nametmp, sub);
    POPUP_CONFIRMATION(STR_DELETEMODEL);
    SET_WARNING_INFO(nametmp, sizeof(g_model.header.name), 0);
  }
  else {
    // The user choosed a file on SD to restore
    storageCheck(true);
    POPUP_WARNING(eeRestoreModel(sub, (char *)result));
    if (!warningText && g_eeGeneral.currModel == sub) {
      eeLoadModel(sub);
    }
  }
}

void initModelsList()
{
  modelslist.load();

  categoriesVerticalOffset = 0;
  bool found = false;
  int index = 0;
  const std::list<ModelsCategory *>& cats = modelslist.getCategories();
  for (std::list<ModelsCategory *>::const_iterator it = cats.begin(); it != cats.end(); ++it, ++index) {
    if (*it == modelslist.getCurrentCategory()) {
      setCurrentCategory(index);
      found = true;
      break;
    }
  }
  if (!found) {
    setCurrentCategory(0);
  }

  menuVerticalOffset = 0;
  found = false;
  index = 0;
  for (ModelsCategory::iterator it = currentCategory->begin(); it != currentCategory->end(); ++it, ++index) {
    if (*it == modelslist.getCurrentModel()) {
      setCurrentModel(index);
      found = true;
      break;
    }
  }
  if (!found) {
    setCurrentModel(0);
  }
}

void menuModelSelect(event_t event)
{
  if (warningResult) {
    warningResult = 0;
    storageCheck(true);
    eeDeleteModel(menuVerticalPosition); // delete file
    s_copyMode = 0;
    event = EVT_ENTRY_UP;
  }

  event_t _event_ = ((event==EVT_KEY_BREAK(KEY_ENTER) || event==EVT_KEY_LONG(KEY_ENTER)) ? 0 : event);

  if ((s_copyMode && EVT_KEY_MASK(event) == KEY_EXIT) || event == EVT_KEY_BREAK(KEY_EXIT)) {
    _event_ -= KEY_EXIT;
  }

  int8_t oldSub = menuVerticalPosition;

  check_submenu_simple(NULL, _event_, MAX_MODELS);

  if (s_editMode > 0) s_editMode = 0;

  int sub = menuVerticalPosition;

  switch (event) {
    case EVT_ENTRY:
      menuVerticalPosition = sub = g_eeGeneral.currModel;
      if (sub >= NUM_BODY_LINES)
        menuVerticalOffset = sub-(NUM_BODY_LINES-1);
      s_copyMode = 0;
      s_editMode = EDIT_MODE_INIT;

      initModelsList();
      break;

    case EVT_KEY_LONG(KEY_EXIT):
      if (s_copyMode && s_copyTgtOfs == 0 && g_eeGeneral.currModel != sub && eeModelExists(sub)) {
        char * nametmp =  reusableBuffer.modelsel.mainname;
        strcat_modelname (nametmp, sub);
        POPUP_CONFIRMATION(STR_DELETEMODEL);
        SET_WARNING_INFO(nametmp, sizeof(g_model.header.name), 0);
        killEvents(event);
        break;
      }
      // no break
    case EVT_KEY_BREAK(KEY_EXIT):
      if (s_copyMode) {
        sub = menuVerticalPosition = (s_copyMode == MOVE_MODE || s_copySrcRow<0) ? (MAX_MODELS+sub+s_copyTgtOfs) % MAX_MODELS : s_copySrcRow;
        s_copyMode = 0;
      }
      else {
        if (menuVerticalPosition != g_eeGeneral.currModel) {
          sub = menuVerticalPosition = g_eeGeneral.currModel;
          menuVerticalOffset = 0;
        }
        else if (event != EVT_KEY_LONG(KEY_EXIT)) {
          popMenu();
        }
      }
      break;
    case EVT_KEY_LONG(KEY_ENTER):
    case EVT_KEY_BREAK(KEY_ENTER):
      s_editMode = 0;
      if (READ_ONLY()) {
        if (g_eeGeneral.currModel != sub && eeModelExists(sub)) {
          selectModel(sub);
        }
      }
      else if (s_copyMode && (s_copyTgtOfs || s_copySrcRow>=0)) {
        showMessageBox(s_copyMode==COPY_MODE ? STR_COPYINGMODEL : STR_MOVINGMODEL);
        storageCheck(true); // force writing of current model data before this is changed

        uint8_t cur = (MAX_MODELS + sub + s_copyTgtOfs) % MAX_MODELS;

        if (s_copyMode == COPY_MODE) {
          if (!eeCopyModel(cur, s_copySrcRow)) {
            cur = sub;
          }
        }

        s_copySrcRow = g_eeGeneral.currModel; // to update the currModel value
        while (sub != cur) {
          uint8_t src = cur;
          cur = (s_copyTgtOfs > 0 ? cur+MAX_MODELS-1 : cur+1) % MAX_MODELS;
          eeSwapModels(src, cur);
          if (src == s_copySrcRow)
            s_copySrcRow = cur;
          else if (cur == s_copySrcRow)
            s_copySrcRow = src;
        }

        if (s_copySrcRow != g_eeGeneral.currModel) {
          g_eeGeneral.currModel = s_copySrcRow;
          storageDirty(EE_GENERAL);
        }

        s_copyMode = 0;
        event = EVT_ENTRY_UP;
      }
      else if (event == EVT_KEY_LONG(KEY_ENTER)) {
        s_copyMode = 0;
        killEvents(event);
        if (g_eeGeneral.currModel != sub) {
          if (eeModelExists(sub)) {
            POPUP_MENU_ADD_ITEM(STR_SELECT_MODEL);
            POPUP_MENU_ADD_SD_ITEM(STR_BACKUP_MODEL);
            POPUP_MENU_ADD_ITEM(STR_COPY_MODEL);
            POPUP_MENU_ADD_ITEM(STR_MOVE_MODEL);
            POPUP_MENU_ADD_ITEM(STR_DELETE_MODEL);
          }
          else {
            POPUP_MENU_ADD_ITEM(STR_CREATE_MODEL);
            POPUP_MENU_ADD_ITEM(STR_RESTORE_MODEL);
          }
        }
        else {
          POPUP_MENU_ADD_SD_ITEM(STR_BACKUP_MODEL);
          POPUP_MENU_ADD_ITEM(STR_COPY_MODEL);
          POPUP_MENU_ADD_ITEM(STR_MOVE_MODEL);
        }
        POPUP_MENU_START(onModelSelectMenu);
      }
      else if (eeModelExists(sub)) {
        s_copyMode = (s_copyMode == COPY_MODE ? MOVE_MODE : COPY_MODE);
        s_copyTgtOfs = 0;
        s_copySrcRow = -1;
      }
      break;

    case EVT_KEY_BREAK(KEY_PAGE):
    case EVT_KEY_LONG(KEY_PAGE):
      chainMenu(event == EVT_KEY_BREAK(KEY_PAGE) ? menuModelSetup : menuTabModel[DIM(menuTabModel)-1]);
      killEvents(event);
      break;

    case EVT_KEY_FIRST(KEY_UP):
    case EVT_KEY_REPT(KEY_UP):
    case EVT_KEY_FIRST(KEY_DOWN):
    case EVT_KEY_REPT(KEY_DOWN):
#if defined(ROTARY_ENCODER_NAVIGATION)
    case EVT_ROTARY_LEFT:
    case EVT_ROTARY_RIGHT:
#endif
      if (s_copyMode) {
        int8_t next_ofs = s_copyTgtOfs + oldSub - menuVerticalPosition;
        if (next_ofs == MAX_MODELS || next_ofs == -MAX_MODELS)
          next_ofs = 0;

        if (s_copySrcRow < 0 && s_copyMode==COPY_MODE) {
          s_copySrcRow = oldSub;
          // find a hole (in the first empty slot above / below)
          sub = eeFindEmptyModel(s_copySrcRow, event==EVT_KEY_FIRST(KEY_DOWN) || event==EVT_KEY_REPT(KEY_DOWN));
          if (sub < 0) {
            // no free room for duplicating the model
            AUDIO_ERROR();
            sub = oldSub;
            s_copyMode = 0;
          }
          next_ofs = 0;
          menuVerticalPosition = sub;
        }
        s_copyTgtOfs = next_ofs;
      }
      break;
  }

  lcdDrawNumber(19*FW, 0, EeFsGetFree(), RIGHT);
  lcdDrawText(19*FW + 3, 0, STR_BYTES);
  lcdDrawText(lcdLastRightPos + 3, 0, STR_FREE);

  drawScreenIndex(MENU_MODEL_SELECT, DIM(menuTabModel), 0);
  lcdDrawFilledRect(0, 0, LCD_W, FH, SOLID, FILL_WHITE|GREY_DEFAULT);

  TITLE(STR_MENUMODELSEL);

  for (uint8_t i=0; i<NUM_BODY_LINES; i++) {
    coord_t y = MENU_HEADER_HEIGHT + 1 + i*FH;
    uint8_t k = i+menuVerticalOffset;

    lcdDrawNumber(3*FW+2, y, k+1, RIGHT|LEADING0|((!s_copyMode && sub==k) ? INVERS : 0), 2);

    if (s_copyMode == MOVE_MODE || (s_copyMode == COPY_MODE && s_copySrcRow >= 0)) {
      if (k == sub) {
        if (s_copyMode == COPY_MODE) {
          k = s_copySrcRow;
          lcdDrawChar(MODELSEL_W-FW, y, '+');
        }
        else {
          k = sub + s_copyTgtOfs;
        }
      }
      else if (s_copyTgtOfs < 0 && ((k < sub && k >= sub+s_copyTgtOfs) || (k-MAX_MODELS < sub && k-MAX_MODELS >= sub+s_copyTgtOfs)))
        k += 1;
      else if (s_copyTgtOfs > 0 && ((k > sub && k <= sub+s_copyTgtOfs) || (k+MAX_MODELS > sub && k+MAX_MODELS <= sub+s_copyTgtOfs)))
        k += MAX_MODELS-1;
    }

    k %= MAX_MODELS;

    if (eeModelExists(k)) {
      putsModelName(4*FW, y, modelHeaders[k].name, k, 0);
      lcdDrawNumber(20*FW, y, eeModelSize(k), RIGHT);
      if (k==g_eeGeneral.currModel && (s_copyMode!=COPY_MODE || s_copySrcRow<0 || i+menuVerticalOffset!=(vertpos_t)sub))
        lcdDrawChar(1, y, '*');
    }

    if (s_copyMode && (vertpos_t)sub==i+menuVerticalOffset) {
      lcdDrawSolidFilledRect(9, y, MODELSEL_W-1-9, 7);
      lcdDrawRect(8, y-1, MODELSEL_W-1-7, 9, s_copyMode == COPY_MODE ? SOLID : DOTTED);
    }
  }

  if (event == EVT_ENTRY || sub != oldSub) {
    loadModelBitmap(modelHeaders[sub].bitmap, modelBitmap);
  }

  lcdDrawBitmap(22*FW+2, 2*FH+FH/2, modelBitmap);
}
