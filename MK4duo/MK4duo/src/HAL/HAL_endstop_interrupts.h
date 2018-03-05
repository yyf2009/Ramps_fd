/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef _HAL_ENDSTOP_INTERRUPTS_H_
#define _HAL_ENDSTOP_INTERRUPTS_H_

// This is what is really done inside the interrupts.
FORCE_INLINE void endstop_ISR_worker( void ) {
  endstops.e_hit = 2; // Because the detection of a e-stop hit has a 1 step debouncer it has to be called at least twice.
}

// One ISR for all EXT-Interrupts
void endstop_ISR(void) { endstop_ISR_worker(); }

#if ENABLED(ARDUINO_ARCH_SAM)
  #include "HAL_DUE/endstop_interrupts.h"
#elif ENABLED(__AVR__)
  #include "HAL_AVR/endstop_interrupts.h"
#else
  #error "Unsupported Platform!"
#endif

#endif /* _HAL_ENDSTOP_INTERRUPTS_H_ */
