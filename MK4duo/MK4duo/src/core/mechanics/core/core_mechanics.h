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
 * core_mechanics.h
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#ifndef _CORE_MECHANICS_H_
#define _CORE_MECHANICS_H_

#if IS_CORE

  class Core_Mechanics: public Mechanics {

    public: /** Constructor */

      Core_Mechanics() {}

    public: /** Public Parameters */

      const float base_max_pos[XYZ]         = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS },
                  base_min_pos[XYZ]         = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
                  base_home_pos[XYZ]        = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS },
                  max_length[XYZ]           = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH };

      #if ENABLED(DUAL_X_CARRIAGE)
        DualXMode dual_x_carriage_mode          = DEFAULT_DUAL_X_CARRIAGE_MODE;
        float     inactive_hotend_x_pos         = X2_MAX_POS,                   // used in mode 0 & 1
                  raised_parked_position[NUM_AXIS],                             // used in mode 1
                  duplicate_hotend_x_offset     = DEFAULT_DUPLICATION_X_OFFSET; // used in mode 2
        int16_t   duplicate_hotend_temp_offset  = 0;                            // used in mode 2
        millis_t  delayed_move_time             = 0;                            // used in mode 1
        bool      active_hotend_parked          = false,                        // used in mode 1 & 2
                  hotend_duplication_enabled    = false;                        // used in mode 2
      #endif

    public: /** Public Function */

      /**
       * Initialize Cartesian parameters
       */
      void init();

      /**
       * Home all axes according to settings
       *
       * Parameters
       *
       *  None  Home to all axes with no parameters.
       *        With QUICK_HOME enabled XY will home together, then Z.
       *
       * Cartesian parameters
       *
       *  X   Home to the X endstop
       *  Y   Home to the Y endstop
       *  Z   Home to the Z endstop
       *
       */
       void home(const bool always_home_all);

      /**
       * Prepare a single move and get ready for the next one
       * If Mesh Bed Leveling is enabled, perform a mesh move.
       */
      bool prepare_move_to_destination_mech_specific();

      /**
       * Set an axis' current position to its home position (after homing).
       *
       * For Core robots this applies one-to-one when an
       * individual axis has been homed.
       *
       * Callers must sync the planner position after calling this!
       */
      void set_axis_is_at_home(const AxisEnum axis);

      #if ENABLED(DUAL_X_CARRIAGE)
        float x_home_pos(const int extruder);
      #endif

      /**
       * Set sensorless homing if the axis has it.
       */
      #if ENABLED(SENSORLESS_HOMING)
        void sensorless_homing_per_axis(const AxisEnum axis, const bool enable=true);
      #endif

    private: /** Private Function */

      /**
       *  Home axis
       */
      void homeaxis(const AxisEnum axis);

      /**
       * Prepare a linear move in a Cartesian setup.
       * Bed Leveling will be applied to the move if enabled.
       *
       * Returns true if current_position[] was set to destination[]
       */
      bool prepare_move_to_destination_cartesian();

      /**
       * Prepare a linear move in a dual X axis setup
       */
      #if ENABLED(DUAL_X_CARRIAGE)
        bool  prepare_move_to_destination_dualx();
        int   x_home_dir(const int extruder) { return extruder ? X2_HOME_DIR : X_HOME_DIR; }
      #endif

      #if ENABLED(QUICK_HOME)
        void quick_home_xy();
      #endif

      #if ENABLED(Z_SAFE_HOMING)
        void home_z_safely();
      #endif

      #if ENABLED(DOUBLE_Z_HOMING)
        void double_home_z();
      #endif

  };

  extern Core_Mechanics mechanics;

#endif // IS_CORE

#endif /* _CORE_MECHANICS_H_ */
