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

#if defined(PXX1)
void init_pxx1_pulses(uint8_t module)
{
  if (module == INTERNAL_MODULE)
    intmodulePxxStart();
  else
    extmodulePxxPulsesStart();
}

void init_pxx1_serial(uint8_t module)
{
  if (module == INTERNAL_MODULE)
    intmodulePxxStart();
#if defined(EXTMODULE_USART)
  else
    extmodulePxxSerialStart();
#endif
}

void disable_pxx1_pulses(uint8_t module)
{
  if (module == INTERNAL_MODULE)
    intmoduleStop();
  else
    extmoduleStop();
}

void disable_pxx1_serial(uint8_t module)
{
  if (module == INTERNAL_MODULE)
    intmoduleStop();
  else
    extmoduleStop();
}
#endif

void init_pxx2(uint8_t module)
{
  if (module == INTERNAL_MODULE)
    intmoduleSerialStart(INTMODULE_PXX_BAUDRATE, true);
  else
    extmodulePxx2Start();
}

void disable_pxx2(uint8_t module)
{
  if (module == INTERNAL_MODULE)
    intmoduleStop();
  else
    extmoduleStop();
}

#if defined(DSM2)
void disable_serial(uint8_t module)
{
  if (module == EXTERNAL_MODULE) {
    extmoduleStop();
  }
}
#endif

void init_ppm(uint8_t module)
{
  if (module == EXTERNAL_MODULE) {
    extmodulePpmStart();
  }
#if defined(TARANIS_INTERNAL_PPM)
  else {
    intmodulePpmStart();
  }
#endif
}

void disable_ppm(uint8_t module)
{
  if (module == EXTERNAL_MODULE) {
    extmoduleStop();
  }
#if defined(TARANIS_INTERNAL_PPM)
  else {
    intmoduleStop();
  }
#endif
}
