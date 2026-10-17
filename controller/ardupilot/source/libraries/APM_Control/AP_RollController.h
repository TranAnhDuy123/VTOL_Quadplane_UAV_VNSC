#pragma once

#include "AP_FW_Controller.h"

class AP_RollController : public AP_FW_Controller
{
public:
    AP_RollController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_RollController);

    static const struct AP_Param::GroupInfo var_info[];

    void convert_pid();

    /*
      set the in_recovery flag, which is used during a VTOL upset recovery
      this flag only lasts one loop
    */
    void set_in_recovery(void) {
        in_recovery = true;
    }

private:
    float run_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode) override;
    float run_indi_rate_control(float desired_rate_degs,
                                float scaler,
                                bool disable_integrator,
                                bool ground_mode);

    // Return true if the airspeed should be considered as under speed
    bool is_underspeed() const override;

    // Return the measured roll angle in degrees
    float get_measured_angle_deg() const override;

    // Return the measured roll rate in radians per second
    float get_measured_rate_rads() const override;

    // Return true if rate limits should be applied
    bool should_apply_rate_limits() const override;

    // Return positive rate limit in deg per second, zero if disabled
    float get_positive_rate_limit_degs() const override;

    // Return negative rate limit in deg per second (as a positive number) zero if disabled
    float get_negative_rate_limit_degs() const override;

    AP_Int8 indi_enable;
    AP_Float indi_krate;
    AP_Float indi_g1;
    AP_Float indi_filter_hz;

    // INDI filtered states
    bool indi_initialized = false;
    float indi_rate_filtered_degs = 0.0f;
    float indi_rate_filtered_prev_degs = 0.0f;
    float indi_accel_filtered_degss = 0.0f;
    float indi_actuator_filtered_deg = 0.0f;
    float indi_last_output_deg = 0.0f;
    bool indi_active_last = false;
    uint32_t indi_last_log_ms = 0;

    bool in_recovery;
};
