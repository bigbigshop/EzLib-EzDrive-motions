#pragma once

#include <cmath>

namespace ez_compat {

// Inch or degree PID used by EzDrive and by LemLib chassis motions.
// Drive callers pass inches. Turn callers pass degrees. Do not mix those units.
class PID {
    public:
        double kp = 0;
        double ki = 0;
        double kd = 0;
        double start_i = 0;

        double error = 0;
        double prev_error = 0;
        double integral = 0;
        // Change in the measurement since the previous sample (not per second).
        double derivative = 0;
        double prev_measurement = 0;
        double target = 0;

        double small_error = 0;
        double big_error = 0;
        int small_time = 0;
        int big_time = 0;
        int velocity_time = 0;
        int small_timer = 0;
        int big_timer = 0;
        int velocity_timer = 0;
        // Max |measurement delta| per sample that still counts as "slow".
        // Same unit as the measurement (inches or degrees per loop).
        double velocity_threshold = 0.1;

        PID() = default;
        PID(double p, double i, double d, double start_i_range)
            : kp(p),
              ki(i),
              kd(d),
              start_i(start_i_range) {}

        void set_constants(double p, double i, double d, double start_i_range) {
            kp = p;
            ki = i;
            kd = d;
            start_i = start_i_range;
        }

        void set_exit_conditions(double small_err, int small_t, double big_err, int big_t, int vel_t) {
            small_error = small_err;
            small_time = small_t;
            big_error = big_err;
            big_time = big_t;
            velocity_time = vel_t;
        }

        void set_velocity_threshold(double per_sample) {
            velocity_threshold = std::fabs(per_sample);
        }

        void reset() {
            error = prev_error = integral = derivative = prev_measurement = 0;
            small_timer = big_timer = velocity_timer = 0;
        }

        // target is fixed; measurement is the live odometry reading.
        double compute(double measurement, bool wrap = false) {
            if (!std::isfinite(measurement)) measurement = prev_measurement;
            error = target - measurement;
            if (wrap) error = wrap180(error);

            derivative = measurement - prev_measurement;
            if (wrap) derivative = wrap180(derivative);

            integrate();
            double output = (error * kp) + (integral * ki) - (derivative * kd);
            prev_measurement = measurement;
            prev_error = error;
            if (!std::isfinite(output)) output = 0;
            return output;
        }

        // Geometry already computed signed error (inches or degrees).
        // Derivative is stored as the equivalent measurement delta so settle
        // checks the same physical speed as compute().
        double compute_error(double signed_error, bool wrap = false) {
            if (!std::isfinite(signed_error)) signed_error = 0;
            error = wrap ? wrap180(signed_error) : signed_error;
            double error_delta = error - prev_error;
            if (wrap) error_delta = wrap180(error_delta);
            derivative = -error_delta;
            integrate();
            double output = (error * kp) + (integral * ki) - (derivative * kd);
            prev_error = error;
            if (!std::isfinite(output)) output = 0;
            return output;
        }

        // Settled only when an error band has been held for its time AND the
        // measurement is slow. A tiny derivative by itself is not an exit.
        // velocity_time adds an optional extra "must already have been slow
        // for this long" requirement. 0 disables that extra timer.
        // dt_ms is the sample period; chassis loops use 10.
        bool is_settled(int dt_ms = 10) {
            if (dt_ms < 0) dt_ms = 0;
            if (!std::isfinite(error) || !std::isfinite(derivative)) return false;

            const bool slow = std::fabs(derivative) <= velocity_threshold;
            const bool in_small = small_error > 0.0 && std::fabs(error) <= small_error;
            const bool in_big = big_error > 0.0 && std::fabs(error) <= big_error;

            if (slow) velocity_timer += dt_ms;
            else velocity_timer = 0;

            if (in_small && slow) small_timer += dt_ms;
            else small_timer = 0;

            if (in_big && slow) big_timer += dt_ms;
            else big_timer = 0;

            const bool slow_enough = velocity_time <= 0 || velocity_timer >= velocity_time;
            if (!slow_enough) return false;
            if (in_small && small_timer >= small_time) return true;
            if (in_big && big_timer >= big_time) return true;
            return false;
        }

    private:
        static double wrap180(double value) {
            while (value > 180.0) value -= 360.0;
            while (value < -180.0) value += 360.0;
            return value;
        }

        void integrate() {
            if (std::fabs(error) < start_i && start_i != 0) integral += error;
            else integral = 0;
            if ((error > 0 && prev_error < 0) || (error < 0 && prev_error > 0)) integral = 0;
        }
};

} // namespace ez_compat
