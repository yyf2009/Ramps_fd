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
 * dhtsensor.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if ENABLED(DHT_SENSOR)

  #define DHTMINREADINTERVAL 2000  // ms
  #define DHTMAXREADTIME       50  // ms

  DhtSensor dhtsensor(DHT_DATA_PIN, DHT_TYPE);

  millis_t  DhtSensor::lastReadTime       = 0,
            DhtSensor::lastOperationTime  = 0;

  float DhtSensor::Temperature  = 20,
        DhtSensor::Humidity     = 10;

  DhtSensor::SensorState DhtSensor::state = Init;

  // ISR
  uint32_t  lastPulseTime, pulses[41];  // 1 start bit + 40 data bits
  volatile uint8_t numPulses;

  void DHT_ISR() {
    const uint32_t now = micros();
    if (HAL::digitalRead(dhtsensor.pin) == HIGH) {
      lastPulseTime = now;
    }
    else if (lastPulseTime > 0) {
      pulses[numPulses++] = now - lastPulseTime;
      if (numPulses == COUNT(pulses)) {
        detachInterrupt(dhtsensor.pin);
      }
    }
  }

  DhtSensor::DhtSensor(const pin_t _pin, const uint8_t _type) {
    pin   = _pin;
    type  = _type;
  }

  void DhtSensor::init(void) {
    HAL::pinMode(pin, OUTPUT_LOW);
    state = Init;
  }

  void DhtSensor::change_type(const uint8_t dhtType) {
    switch (dhtType) {
      case DHT11:
        type = DHT11;
        break;
      case DHT21:
        type = DHT21;
        break;
      case DHT22:
        type = DHT22;
        break;
      default:
        SERIAL_LM(ER, "Invalid DHT sensor type");
        break;
    }
  }

  void DhtSensor::print_parameters(void) {
    SERIAL_SMV(CFG, " DHT sensor Pin:", pin);
    SERIAL_EMV(" Type:DHT", type);
  }

  void DhtSensor::spin() {

    if ((millis() - lastReadTime) < DHTMINREADINTERVAL) return;

    switch (state) {

      case Init:
        HAL::digitalWrite(pin, HIGH);
        state = Wait_250ms;
        lastOperationTime = millis();
        break;

      case Wait_250ms:
        if (millis() - lastOperationTime >= 250) {
          HAL::pinMode(pin, OUTPUT_LOW);
          HAL::digitalWrite(pin, LOW);
          state = Wait_20ms;
          lastOperationTime = millis();
        }
        break;

      case Wait_20ms:
        if (millis() - lastOperationTime >= 20) {

          // End the start signal by setting data line high for 40 microseconds
          HAL::digitalWrite(pin, HIGH);
          HAL::delayMicroseconds(40);

          // Now start reading the data line to get the value from the DHT sensor
          HAL::pinMode(pin, INPUT_PULLUP);
          HAL::delayMicroseconds(10);

          // Read from the DHT sensor using an DHT_ISR
          numPulses = 0;
          lastPulseTime = 0;
          attachInterrupt(pin, DHT_ISR, CHANGE);

          // Wait for the next operation to complete
          state = Read;
          lastOperationTime = millis();
        }
        break;

      case Read:
        // Make sure we don't time out
        if (millis() - lastOperationTime > DHTMAXREADTIME) {
          detachInterrupt(pin);
          state = Init;
          lastReadTime = millis();
          break;
        }

        // Wait for the reading to complete (1 start bit + 40 data bits)
        if (numPulses != 41) break;

        // We're reading now - reset the state
        state = Init;
        lastReadTime = millis();

        // Check start bit
        if (pulses[0] < 40) break;

        // Reset 40 bits of received data to zero
        uint8_t data[5] = { 0, 0, 0, 0, 0 };

        // Inspect each high pulse and determine which ones
        // are 0 (less than 40us) or 1 (more than 40us)
        for (uint8_t i = 0; i < 40; ++i) {
          data[i / 8] <<= 1;
          if (pulses[i + 1] > 40)
            data[i / 8] |= 1;
        }

        // Verify checksum
        if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4])
          break;

        // Generate final results
        switch (type) {
          case DHT11:
            Humidity = data[0];
            Temperature = data[2];
            break;
          case DHT21:
          case DHT22:
            Humidity = ((data[0] * 256) + data[1]) * 0.1;
            Temperature = (((data[2] & 0x7F) * 256) + data[3]) * 0.1;
            if (data[2] & 0x80) Temperature *= -1.0;
            break;
          default:
            break;
        }
        break;
    }
  }

#endif // ENABLED(DHT_SENSOR)
