/**
 * @file PetalPID.h
 * @author Fern Lane
 * @brief Lightweight universal PID controller library with Ziegler–Nichols auto tuning and variable cycle time
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

#ifndef PETAL_PID_H__
#define PETAL_PID_H__

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// From <https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method>
#define TYPE_P                    0U
#define TYPE_PI                   1U
#define TYPE_PD                   2U
#define TYPE_CLASSIC_PID          3U
#define TYPE_PESSEN_INTEGRAL_RULE 4U
#define TYPE_SOME_OVERSHOOT       5U
#define TYPE_NO_OVERSHOOT         6U

// From <https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method>
#ifdef ARDUINO
const float _CONSTANTS[][3] PROGMEM = {{.5f, 0.f, 0.f},  {.45f, .83f, 0.f}, {.8f, 0.f, .125f}, {.6f, .5f, .125f},
                                       {.7f, .4f, .15f}, {.33f, .5f, .33f}, {.2f, .5f, .33f}};
#else
const float _CONSTANTS[][3] = {{.5f, 0.f, 0.f},  {.45f, .83f, 0.f}, {.8f, 0.f, .125f}, {.6f, .5f, .125f},
                               {.7f, .4f, .15f}, {.33f, .5f, .33f}, {.2f, .5f, .33f}};
#endif

#define _N_CYCLES_DEFAULT 100U

// Internal definitions
#define _AUTO_TUNE_STAGE_NONE      0U
#define _AUTO_TUNE_STAGE_STARTING  1U
#define _AUTO_TUNE_STAGE_TUNING    2U
#define _AUTO_TUNE_STAGE_FINISHING 3U

class PetalPID {
  public:
    PetalPID(float p_gain = 0.f, float i_gain = 0.f, float d_gain = 0.f, float min_output = -1.f,
             float max_output = 1.f);
    float get_p(void), get_i(void), get_d(void);
    void set_gains(float p_gain, float i_gain, float d_gain);
    void set_min_max_output(float min, float max);
    void set_min_max_integral(float min = -1000.f, float max = 1000.f);
    void set_reverse(bool reverse);
    float calculate(float input, float setpoint, uint64_t time_us);
    void start_auto_tune(uint8_t type = TYPE_CLASSIC_PID, uint32_t n_cycles = _N_CYCLES_DEFAULT);
    uint32_t stop_auto_tune(void);
    bool is_tuning(void);
    void reset(void);

  private:
    bool reversed;
    float p_gain, i_gain, d_gain, min_output, max_output, min_integral, max_integral;
    uint64_t time_last, time_high_end, time_low_end, time_delta_high, time_delta_low;
    uint8_t type, auto_tune_stage;
    bool tune_output;
    uint32_t cycles_tuned, n_cycles, time_delta_avg_cycles;
    float peak_positive, peak_negative;
    float time_delta_avg, p_gain_avg, i_gain_avg, d_gain_avg;
    float integral_accumulator, error_prev;
};

#endif
