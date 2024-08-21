/**
 * @file PetalPID.cpp
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

#include "PetalPID.h"

/**
 * @brief Construct a new PetalPID::PetalPID object
 * Sets gains, limits and resets PID controller
 * Call set_min_max_integral() to set integral min max limits. By default they will be -1000 to 1000
 * Call set_reverse() to change error calculation policy (invert output)
 *
 * @param p_gain P term gain (Default: 0)
 * @param i_gain I term gain (Default: 0)
 * @param d_gain D term gain (Default: 0)
 * @param min_output minimum possible output of the PID controller (Default: -1)
 * @param max_output maximum possible output of the PID controller (Default: 1)
 */
PetalPID::PetalPID(float p_gain, float i_gain, float d_gain, float min_output, float max_output) {
    set_gains(p_gain, i_gain, d_gain);
    set_min_max_output(min_output, max_output);
    set_min_max_integral();
    set_reverse(false);
    auto_tune_stage = _AUTO_TUNE_STAGE_NONE;
    reset();
}

/**
 * @return float current P term gain
 */
float PetalPID::get_p(void) { return p_gain; }

/**
 * @return float current I term gain
 */
float PetalPID::get_i(void) { return i_gain; }

/**
 * @return float current D term gain
 */
float PetalPID::get_d(void) { return d_gain; }

/**
 * @brief Sets PID gains
 *
 * @param p_gain P term gain
 * @param i_gain I term gain
 * @param d_gain D term gain
 */
void PetalPID::set_gains(float p_gain, float i_gain, float d_gain) {
    this->p_gain = p_gain;
    this->i_gain = i_gain;
    this->d_gain = d_gain;
}

/**
 * @brief Sets limit of the PID controller
 *
 * @param min minimum possible output of the PID controller
 * @param max maximum possible output of the PID controller
 */
void PetalPID::set_min_max_output(float min, float max) {
    this->min_output = min;
    this->max_output = max;
}

/**
 * @brief Sets integral term limits (to prevent integral windup)
 *
 * @param min minimum possible value of integral term
 * @param max maximum possible value of integral term
 */
void PetalPID::set_min_max_integral(float min, float max) {
    this->min_integral = min;
    this->max_integral = max;
}

/**
 * @brief Sets error calculation policy (inverts output)
 *
 * @param reverse true for error = setpoint - input, false for input - setpoint
 */
void PetalPID::set_reverse(bool reverse) { this->reversed = reverse; }

/**
 * @brief Calculates one cycle of the PID-controller
 * Also performs auto-tuning routine. For that, call start_auto_tune()
 * NOTE: This must be called as frequent as possible without any delays in a main loop
 *
 * @param error measured value
 * @param setpoint target value
 * @param time_us current time in microseconds (micros() for arduino)
 * @return float PID output (will be in [-PID_MIN_MAX_OUT, PID_MIN_MAX_OUT] range)
 */
float PetalPID::calculate(float input, float setpoint, uint64_t time_us) {
    // Calculate time delta in seconds
    float time_delta = 0.f;
    if (time_last != 0 && time_us > time_last)
        time_delta = (time_us - time_last) * 1.e-6f;
    time_last = time_us;

    // Auto-tune mode
    if (auto_tune_stage != _AUTO_TUNE_STAGE_NONE) {
        // Reset setpoints before starting auto-tune
        if (auto_tune_stage == _AUTO_TUNE_STAGE_STARTING) {
            auto_tune_stage = _AUTO_TUNE_STAGE_TUNING;
            peak_positive = setpoint;
            peak_negative = setpoint;
            return min_output;
        }

        // Auto-tuning finished -> calculate average gains
        else if (auto_tune_stage == _AUTO_TUNE_STAGE_FINISHING) {
            stop_auto_tune();
            return min_output;
        }

        // Main tuning routine
        else {
            // Record peak input values
            if (input > peak_positive)
                peak_positive = input;
            if (input < peak_negative)
                peak_negative = input;

            // Calculate sum and number of cycles for future calculation of average 1 PID cycle time
            if (time_delta != 0.f) {
                time_delta_avg += time_delta;
                time_delta_avg_cycles++;
            }

            // Output is set to maximum and input reached setpoint
            if (tune_output && input > setpoint) {
                // Set output to minimum
                tune_output = false;

                // Record current time and calculate time delta preventing micros() overflow and startup value
                time_high_end = time_us;
                if (time_low_end != 0 && time_high_end > time_low_end)
                    time_delta_high = time_high_end - time_low_end;
                else
                    time_delta_high = 0;
            }

            // Output is set to minimum and input dropped below setpoint
            else if (!tune_output && input < setpoint) {
                // Set output to maximum
                tune_output = true;

                // Record current time and calculate time delta preventing micros() overflow and startup value
                time_low_end = time_us;
                if (time_high_end != 0 && time_low_end > time_high_end)
                    time_delta_low = time_low_end - time_high_end;
                else
                    time_delta_low = 0;

                // Accept this cycle only if we're able to get all time deltas
                if (time_delta != 0.f && time_delta_low != 0 && time_delta_high != 0) {
                    // Calculate average 1 PID cycle time
                    time_delta_avg /= (float) time_delta_avg_cycles;

                    // Calculate ultimate gain as described in "Workshop 5 Controller tuning for capacity and
                    // dead time processes" "A little experience often upsets a lot of theory." by Samuel Parks Cadman
                    // <https://onlinelibrary.wiley.com/doi/pdf/10.1002/9780470029558.oth5>
                    // ultimate gain Ku = 4h/3.14a, where h - controller output amplitude (half wave),
                    // a - measured amplitude (half wave)
                    float ult_gain =
                        (2.f * (max_output - min_output)) / (M_PI * ((peak_positive - peak_negative) / 2.f));

                    // Oscillations period is a sum of time deltas in seconds
                    float ult_period = (float) (time_delta_low + time_delta_high) / 1.e6f;

#ifdef ARDUINO
                    float const_k_p = pgm_read_float(&_CONSTANTS[type][0]);
                    float const_t_i = pgm_read_float(&_CONSTANTS[type][1]);
                    float const_t_d = pgm_read_float(&_CONSTANTS[type][2]);
#else
                    float const_k_p = _CONSTANTS[type][0];
                    float const_t_i = _CONSTANTS[type][1];
                    float const_t_d = _CONSTANTS[type][2];
#endif

                    // Calculate sums and increment number of cycles
                    // Kp = Const * Ku
                    float p_gain_temp = const_k_p * ult_gain;
                    p_gain_avg += p_gain_temp;

                    // Ki = Kp / Ti, Ti = Const * Tu -> Ki = Kp / (Const * Tu)
                    if (const_t_i != 0.f)
                        i_gain_avg += (p_gain_temp / (const_t_i * ult_period)) * time_delta_avg;
                    else
                        i_gain_avg = 0.f;

                    // Kd = Kp * Td, Td = Const * Tu -> Kd = Kp * (Const * Tu)
                    if (const_t_d != 0.f)
                        // d_gain_avg += (p_gain_temp * (const_t_d * ult_period)) / time_delta_avg;
                        d_gain_avg += (p_gain_temp * (const_t_d * ult_period));
                    else
                        d_gain_avg = 0.f;

                    cycles_tuned++;

                    // Reset for next tuning cycle
                    time_delta_avg = 0.f;
                    time_delta_avg_cycles = 0;

                    // Done
                    if (cycles_tuned >= n_cycles)
                        auto_tune_stage = _AUTO_TUNE_STAGE_FINISHING;
                }

                // Reset peaks
                peak_positive = setpoint;
                peak_negative = setpoint;
            }

            return reversed ? (tune_output ? min_output : max_output) : (tune_output ? max_output : min_output);
        }
    }

    // Normal mode
    else {
        // Calculate error
        float error = reversed ? input - setpoint : setpoint - input;

        // Calculate P term
        float p_output = p_gain * error;

        // Calculate I term
        integral_accumulator += error * time_delta;

        // Prevent integral windup
        if (integral_accumulator > max_integral)
            integral_accumulator = max_integral;
        else if (integral_accumulator < min_integral)
            integral_accumulator = min_integral;

        // Calculate D term
        float d_output = 0.f;
        if (time_delta != 0.f)
            d_output = d_gain * ((error - error_prev) / time_delta);
        error_prev = error;

        // Calculate total output
        float pid_output = p_output + integral_accumulator * i_gain + d_output;

        // Clamp output
        if (pid_output > max_output)
            pid_output = max_output;
        else if (pid_output < min_output)
            pid_output = min_output;

        return pid_output;
    }
}

/**
 * @brief Starts auto-tuning using Ziegler–Nichols method
 *
 * @param type control type (TYPE_P or TYPE_PI or TYPE_...) (Default: TYPE_CLASSIC_PID)
 * @param n_cycles number of tuning cycles (Default: 100)
 */
void PetalPID::start_auto_tune(uint8_t type, uint32_t n_cycles) {
    this->type = type;
    this->n_cycles = n_cycles;

    auto_tune_stage = _AUTO_TUNE_STAGE_STARTING;
    p_gain = 0.f;
    i_gain = 0.f;
    d_gain = 0.f;
    p_gain_avg = 0.f;
    i_gain_avg = 0.f;
    d_gain_avg = 0.f;
    time_delta_low = 0;
    time_delta_high = 0;
    time_delta_avg = 0.f;
    time_delta_avg_cycles = 0;
    cycles_tuned = 0;
    tune_output = false;
    reset();
}

/**
 * @brief Stops auto-tuning and calculates gains
 * Call get_p(), get_i(), get_d() to retrieve calculated gains
 *
 * @return uint32_t number of tune cycles passed
 */
uint32_t PetalPID::stop_auto_tune(void) {
    auto_tune_stage = _AUTO_TUNE_STAGE_NONE;
    if (cycles_tuned != 0) {
        p_gain = p_gain_avg / (float) cycles_tuned;
        i_gain = i_gain_avg / (float) cycles_tuned;
        d_gain = d_gain_avg / (float) cycles_tuned;
    }
    return cycles_tuned;
}

/**
 * @return true if auto-tuning is in progress
 * @return false if auto-tuning not started or finished
 */
bool PetalPID::is_tuning(void) { return auto_tune_stage != _AUTO_TUNE_STAGE_NONE; }

/**
 * @brief Resets PID's I and D variables
 */
void PetalPID::reset(void) {
    integral_accumulator = 0.f;
    error_prev = 0.f;
}
