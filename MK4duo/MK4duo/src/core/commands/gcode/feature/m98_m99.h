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

/**
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(HYSTERESIS)

  #define CODE_M98
  #define CODE_M99

  /**
   * M98: Print Hysteresis value
   */
  inline void gcode_M98(void) { mechanics.report_hysteresis(); }

  /**
   * M99: Set Hysteresis value
   */
  inline void gcode_M99(void) {
    LOOP_XYZE(i) {
      if (parser.seen(axis_codes[i]))
        mechanics.set_hysteresis_axis(i, parser.value_float());
    }
  }

#endif // HYSTERESIS
