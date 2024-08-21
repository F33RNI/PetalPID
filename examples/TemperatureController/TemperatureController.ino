/**
 * @file TemperatureController.ino
 * @author Fern Lane
 * @brief Example of temperature PID-controller with simulated environment
 *
 * @copyright Copyright (c) 2024 Fern Lane
 *
 * This file is part of the PetalPID distribution.
 * See <https://github.com/F33RNI/PetalPID> for more info.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * long with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>

#include <PetalPID.h>

// PID settings
#define PID_P_GAIN  187.07f
#define PID_I_GAIN  115.42f
#define PID_D_GAIN  10.06f
#define PID_OUT_MIN 0.f
#define PID_OUT_MAX 255.f

// PetalPID class instance
PetalPID pid_1 = PetalPID(PID_P_GAIN, PID_I_GAIN, PID_D_GAIN, PID_OUT_MIN, PID_OUT_MAX);

// For setpoint ramp simulation
#define SETPOINT_LOW             30.f
#define SETPOINT_HIGH            38.f
#define SETPOINT_CHANGE_INTERVAL 2000UL

// Temperature with heater disabled and maximum temperature with heater enabled
#define TEMPERATURE_COLD 19.5f
#define TEMPERATURE_MAX  50.f

// This simulates heater and heating environment
// 0 to 1, where 0 - immediate response, 1 - no response at all
#define HEATING_SLOWNESS .92f

uint8_t heater_pwm;
float setpoint;

// Internal variables for simulation
float _temperature, _heater_temperature;
uint64_t _setpoint_timer;
uint8_t _setpoint_state;

float measure_temperature();
float mapf(float x, float in_min, float in_max, float out_min, float out_max);

void setup() {
    Serial.begin(9600UL);
    Serial.println(F("Setpoint\tTemperature"));

    // Uncomment line below to start auto-tuning (wait for it to complete. You will see this moment on the plot)
    // pid_1.start_auto_tune(TYPE_NO_OVERSHOOT, 50);
}

void loop() {
    // Create setpoint ramp
    //   ____         _____    - SETPOINT_HIGH
    //  /    \       /
    // /      \_____/          - SETPOINT_LOW
    //
    //         |---|           - SETPOINT_CHANGE_INTERVAL
    //
    if (millis() - _setpoint_timer >= SETPOINT_CHANGE_INTERVAL) {
        _setpoint_timer = millis();
        _setpoint_state = _setpoint_state < 3U ? _setpoint_state + 1U : 0U;
    }
    switch (_setpoint_state) {
    case 0:
        setpoint = SETPOINT_LOW;
        break;
    case 1:
        setpoint = mapf(millis() - _setpoint_timer, 0UL, SETPOINT_CHANGE_INTERVAL, SETPOINT_LOW, SETPOINT_HIGH);
        break;
    case 2:
        setpoint = SETPOINT_HIGH;
        break;
    case 3:
        setpoint = mapf(millis() - _setpoint_timer, 0UL, SETPOINT_CHANGE_INTERVAL, SETPOINT_HIGH, SETPOINT_LOW);
        break;

    default:
        break;
    }

    // Measure simulated temperature
    float temperature = measure_temperature();

    // Calculate one cycle of PID (or auto tuning)
    heater_pwm = pid_1.calculate(temperature, setpoint, micros());

    // Uncomment line below to compare ON-OFF output vs PID
    // heater_pwm = temperature < setpoint ? 255U : 0U;

    // Plot simulated temperature, setpoint and PID output divided by 10 (for better visualization in plotter)
    Serial.print(temperature, 4);
    Serial.print("\t");
    Serial.print(setpoint, 4);
    Serial.print("\t");
    // Serial.print(pid_1.get_p());
    // Serial.print("\t");
    // Serial.print(pid_1.get_i());
    // Serial.print("\t");
    // Serial.print(pid_1.get_d());
    // Serial.print("\t");
    Serial.println(heater_pwm / 10);

    // Wait some random time (1 to 5 ms)
    delay(random() % 5 + 1UL);
}

/**
 * @brief Simulates heating some volume with PWM heater by simulating heater and volume as 2 low-pass filters
 * Also adds noise for more realistic output
 *
 * @return float simulated temperature
 */
float measure_temperature() {
    // Simulate heater (low-pass filter)
    if (_heater_temperature == 0.f)
        _heater_temperature = TEMPERATURE_COLD;
    else
        _heater_temperature =
            _heater_temperature * HEATING_SLOWNESS +
            mapf(heater_pwm, 0.f, 255.f, TEMPERATURE_COLD, TEMPERATURE_MAX) * (1.f - HEATING_SLOWNESS);

    // Simulate environment (low-pass filter)
    if (_temperature == 0.f)
        _temperature = TEMPERATURE_COLD;
    else
        _temperature = _temperature * HEATING_SLOWNESS + _heater_temperature * (1.f - HEATING_SLOWNESS);

    // Add a bit of noise -0.1 to +0.1 degC
    _temperature += ((random() % 101) / 50.f - 1.f) / 10.f;
    return _temperature;
}

/**
 * @brief Arduino's map() but for floating-point
 * <https://www.arduino.cc/reference/en/language/functions/math/map/>
 *
 * @param x the number to map
 * @param in_min the lower bound of the value’s current range
 * @param in_max the upper bound of the value’s current range
 * @param out_min the lower bound of the value’s target range
 * @param out_max the upper bound of the value’s target range
 * @return float the mapped value
 */
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
