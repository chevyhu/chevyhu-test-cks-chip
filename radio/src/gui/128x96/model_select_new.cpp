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


#define CATEGORIES_WIDTH               62
#define MODELES_WIDTH                  126

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

enum ModelSelectMode {
    MODE_SELECT_MODEL,
    MODE_RENAME_CATEGORY,
    MODE_MOVE_MODEL,
};

enum ModelDeleteMode {
    MODE_DELETE_MODEL,
    MODE_DELETE_CATEGORY,
};

uint8_t selectMode, deleteMode;

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
  if (index++ >= currentCategory->size()) {
    menuVerticalPosition = 0;
  }


  //menuVerticalPosition = index;
 // menuHorizontalPosition = index & 1;
  //menuVerticalOffset = limit<int>(menuVerticalPosition-2, menuVerticalOffset, min<int>(menuVerticalPosition, max<int>(0, (currentCategory->size()-7))));
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


static void displayPresetChoice(event_t event)
{
  runPopupWarning(event);
  lcdDrawNumber(WARNING_LINE_X+FW*7, WARNING_LINE_Y, 45*warningInputValue/4, LEFT|INVERS);
  lcdDrawChar(lcdLastRightPos, WARNING_LINE_Y, '@', INVERS);

  if (warningResult) {
    warningResult = 0;
    CurveInfo & crv = g_model.curves[s_curveChan];
    int8_t * points = curveAddress(s_curveChan);
    int k = 25 * warningInputValue;
    int dx = 2000 / (5+crv.points-1);
    for (uint8_t i=0; i<5+crv.points; i++) {
      int x = -1000 + i * dx;
      points[i] = div_and_round(div_and_round(k * x, 100), 10);
    }
    if (crv.type == CURVE_TYPE_CUSTOM) {
      resetCustomCurveX(points, 5+crv.points);
    }
  }
}


void onModelSelectMenu(const char * result)
{
  if (result == STR_SELECT_MODEL) {
    // we store the latest changes if any
    storageFlushCurrentModel();
    storageCheck(true);
    memcpy(g_eeGeneral.currModelFilename, currentModel->modelFilename, LEN_MODEL_FILENAME);
    modelslist.setCurrentModel(currentModel);
    loadModel(g_eeGeneral.currModelFilename, true);
    storageDirty(EE_GENERAL);
    storageCheck(true);
    chainMenu(menuMainView);
  }
  else if (result == STR_DELETE_MODEL) {
    POPUP_CONFIRMATION(STR_DELETEMODEL);
    SET_WARNING_INFO(currentModel->modelName, LEN_MODEL_NAME, 0);
    deleteMode = MODE_DELETE_MODEL;
  }
  else if (result == STR_CREATE_MODEL) {
    storageCheck(true);
    modelslist.addModel(currentCategory, createModel());
    selectMode = MODE_SELECT_MODEL;
    setCurrentModel(currentCategory->size() - 1);
    modelslist.setCurrentModel(currentModel);
    modelslist.onNewModelCreated(currentModel, &g_model);
#if defined(LUA)
    //chainMenu(menuModelWizard);
#endif
  }
  else if (result == STR_DUPLICATE_MODEL) {
    char duplicatedFilename[LEN_MODEL_FILENAME+1];
    memcpy(duplicatedFilename, currentModel->modelFilename, sizeof(duplicatedFilename));
    if (findNextFileIndex(duplicatedFilename, LEN_MODEL_FILENAME, MODELS_PATH)) {
      sdCopyFile(currentModel->modelFilename, MODELS_PATH, duplicatedFilename, MODELS_PATH);
      ModelCell* dup_model = modelslist.addModel(currentCategory, duplicatedFilename);
      dup_model->fetchRfData();
      setCurrentModel(currentCategory->size() - 1);
    }
    else {
      POPUP_WARNING("Invalid File");
    }
  }
  else if (result == STR_MOVE_MODEL) {
    selectMode = MODE_MOVE_MODEL;
  }
  else if (result == STR_CREATE_CATEGORY) {
    currentCategory = modelslist.createCategory();
    setCurrentCategory(modelslist.getCategories().size() - 1);
  }
  else if (result == STR_RENAME_CATEGORY) {
    selectMode = MODE_RENAME_CATEGORY;
    s_editMode = EDIT_MODIFY_STRING;
    editNameCursorPos = 0;
  }
  else if (result == STR_DELETE_CATEGORY) {
    if (currentCategory->size() > 0){
      POPUP_WARNING(STR_DELETE_ERROR);
      SET_WARNING_INFO(STR_CAT_NOT_EMPTY, sizeof(TR_CAT_NOT_EMPTY), 0);
    }
    else {
      POPUP_CONFIRMATION(STR_DELETEMODEL);
      SET_WARNING_INFO(currentCategory->name, LEN_MODEL_FILENAME, 0);
      deleteMode = MODE_DELETE_CATEGORY;
    }
  }
}

void menuModelSelect(event_t event) {
  static uint8_t categoriyIndex = 0;
  static uint8_t subModelIndex = 0;
  static uint8_t categoriyCount = 0;


  if (warningResult) {
    warningResult = 0;
    if (deleteMode == MODE_DELETE_CATEGORY) {
      TRACE("DELETE CATEGORY");
      modelslist.removeCategory(currentCategory);
      modelslist.save();
      setCurrentCategory(currentCategoryIndex > 0 ? currentCategoryIndex-1 : currentCategoryIndex);
    }
    else if (deleteMode == MODE_DELETE_MODEL){
      int modelIndex = MODEL_INDEX();
      modelslist.removeModel(currentCategory, currentModel);
      s_copyMode = 0;
      //event = EVT_REFRESH;
      if (modelIndex > 0) {
        modelIndex--;
      }
      setCurrentModel(modelIndex);
    }
  }

  const std::list<ModelsCategory*>& cats = modelslist.getCategories();

  check_submenu_simple(STR_MENUMODELSEL, event, MAX_MODELS);

  int8_t sub = menuVerticalPosition;

  switch (event) {
    case EVT_ENTRY:
      categoriyCount = 0;
      selectMode = MODE_SELECT_MODEL;
      initModelsList();
      break;

    case EVT_KEY_BREAK(KEY_ENTER):
      if (selectMode == MODE_MOVE_MODEL)
        selectMode = MODE_SELECT_MODEL;
      break;

    case EVT_KEY_FIRST(KEY_EXIT):
      switch (selectMode) {
        case MODE_MOVE_MODEL:
          selectMode = MODE_SELECT_MODEL;
          break;
        case MODE_SELECT_MODEL:
          chainMenu(menuMainView);
          return;
      }
      break;

    case EVT_KEY_BREAK(KEY_PAGE):
    {
      if (selectMode == MODE_SELECT_MODEL) {
        categoriesVerticalPosition += 1;
        if (categoriesVerticalPosition >= cats.size())
          categoriesVerticalPosition = 0;
        setCurrentCategory(categoriesVerticalPosition);
      }
      else if (selectMode == MODE_MOVE_MODEL && categoriesVerticalPosition < cats.size()-1) {
        ModelsCategory * previous_category = currentCategory;
        ModelCell * model = currentModel;
        categoriesVerticalPosition += 1;
        setCurrentCategory(categoriesVerticalPosition);
        modelslist.moveModel(model, previous_category, currentCategory);
        setCurrentModel(currentCategory->size()-1);
      }
      subModelIndex = 0;
      menuVerticalPosition = 0;
      break;
    }
    case EVT_KEY_LONG(KEY_PAGE):
    {
      if (selectMode == MODE_SELECT_MODEL) {
        if (categoriesVerticalPosition == 0)
          categoriesVerticalPosition = cats.size() - 1;
        else
          categoriesVerticalPosition -= 1;
        setCurrentCategory(categoriesVerticalPosition);
      }
      else if (selectMode == MODE_MOVE_MODEL && categoriesVerticalPosition > 0) {
        ModelsCategory * previous_category = currentCategory;
        ModelCell * model = currentModel;
        categoriesVerticalPosition -= 1;
        setCurrentCategory(categoriesVerticalPosition);
        modelslist.moveModel(model, previous_category, currentCategory);
        setCurrentModel(currentCategory->size()-1);
      }
      subModelIndex = 0;
      menuVerticalPosition = 0;
      killEvents(event);
      break;
    }
    case EVT_KEY_LONG(KEY_ENTER):
      if (selectMode == MODE_SELECT_MODEL) {
        killEvents(event);
        if (currentModel && currentModel != modelslist.getCurrentModel()) {
          POPUP_MENU_ADD_ITEM(STR_SELECT_MODEL);
        }
        POPUP_MENU_ADD_ITEM(STR_CREATE_MODEL);
        if (currentModel) {
          POPUP_MENU_ADD_ITEM(STR_DUPLICATE_MODEL);
          POPUP_MENU_ADD_ITEM(STR_MOVE_MODEL);
        }
        // POPUP_MENU_ADD_SD_ITEM(STR_BACKUP_MODEL);
        if (currentModel && currentModel != modelslist.getCurrentModel()) {
          POPUP_MENU_ADD_ITEM(STR_DELETE_MODEL);
        }
        // POPUP_MENU_ADD_ITEM(STR_RESTORE_MODEL);
        POPUP_MENU_ADD_ITEM(STR_CREATE_CATEGORY);
        POPUP_MENU_ADD_ITEM(STR_RENAME_CATEGORY);
        if (cats.size() > 1) {
          POPUP_MENU_ADD_ITEM(STR_DELETE_CATEGORY);
        }
        POPUP_MENU_START(onModelSelectMenu);
      }
      break;
    default:
      break;
  }

  int index = 0;
  coord_t y = 18;

  drawVerticalScrollbar(CATEGORIES_WIDTH-1, y-1, 4*(FH+7)-5, categoriesVerticalOffset, cats.size(), 5);
  // Categories
  for (std::list<ModelsCategory *>::const_iterator it = cats.begin(); it != cats.end(); ++it, ++index) {
    if (index >= categoriesVerticalOffset && index < categoriesVerticalOffset+5) {
      coord_t y = MENU_HEADER_HEIGHT + 1 + (index - categoriesVerticalOffset)*FH;
      uint8_t k = index ;;
      lcdDrawText(2, y, (*it)->name,  ((categoriesVerticalPosition == k) ? INVERS : 0));

      if (s_copyMode && categoriesVerticalPosition == k) {
        lcdDrawSolidFilledRect(9, y, MODELSEL_W - 1 - 9, 7);
        lcdDrawRect(8, y - 1, MODELSEL_W - 1 - 7, 9, s_copyMode == COPY_MODE ? SOLID : DOTTED);
      }
    }
  }

  index = 0;
  // Models
  for (ModelsCategory::iterator it = currentCategory->begin(); it != currentCategory->end(); ++it, ++index) {
    //if (index >= menuVerticalOffset) {
      coord_t y = MENU_HEADER_HEIGHT + 1 + (index - menuVerticalOffset)*FH;
      uint8_t k = index;
      bool selected = ((selectMode == MODE_SELECT_MODEL || selectMode == MODE_MOVE_MODEL) && k == menuVerticalPosition);
      bool current = !strncmp((*it)->modelFilename, g_eeGeneral.currModelFilename, LEN_MODEL_FILENAME);

      if (current) {
        lcdDrawChar(9 * FW + 11, y, '*');
        lcdDrawText(9 * FW + 18, y, (*it)->modelName, LEADING0 | ((selected) ? INVERS : 0));
        subModelIndex = k;
      } else {
        lcdDrawText(9 * FW + 18, y, (*it)->modelName, LEADING0 | ((selected) ? INVERS : 0));
      }

      y += 16;

      if (selected) {
        lcdDrawText(5, LCD_H - FH - 1, (*it)->modelFilename, SMLSIZE);
      }
    }

    if (currentModel) {
      if (menuVerticalPosition != subModelIndex) {
        if (selectMode == MODE_SELECT_MODEL) {
          setCurrentModel(menuVerticalPosition);
        } else if (selectMode == MODE_MOVE_MODEL) {
          //modelslist.moveModel(currentCategory, currentModel, direction);
          setCurrentModel(menuVerticalPosition);
        }
      }
    }
}
