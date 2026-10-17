/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Code by Jon Challinger
//  Modified by Paul Riseborough
//

#include <AP_HAL/AP_HAL.h>
#include "AP_RollController.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RollController::var_info[] = {
    // @Param: 2SRV_TCONST
    // @DisplayName: Roll Time Constant
    // @Description: Time constant in seconds from demanded to achieved roll angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.
    // @Range: 0.4 1.0
    // @Units: s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("2SRV_TCONST",      0, AP_RollController, gains.tau,       0.5f),

    // index 1 to 3 reserved for old PID values

    // @Param: 2SRV_RMAX
    // @DisplayName: Maximum Roll Rate
    // @Description: This sets the maximum roll rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables this limit.
    // @Range: 0 180
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2SRV_RMAX",   4, AP_RollController, gains.rmax_pos,       0),

    // index 5, 6 reserved for old IMAX, FF

    // @Param: _RATE_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain. Corrects in proportion to the difference between the desired roll rate vs actual roll rate
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: _RATE_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 3.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: _RATE_PDMX
    // @DisplayName: Roll axis rate controller PD sum maximum
    // @Description: Roll axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: _RATE_D_FF
    // @DisplayName: Roll Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.03
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _RATE_NTF
    // @DisplayName: Roll Target notch filter index
    // @Description: Roll Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: _RATE_NEF
    // @DisplayName: Roll Error notch filter index
    // @Description: Roll Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(rate_pid, "_RATE_", 9, AP_RollController, AC_PID),

    // @Param: 2SRV_ACCEL
    // @DisplayName: Roll max acceleration
    // @Description: Roll acceleration limit. Setting to zero disables input shaping.
    // @Range: 0 2500
    // @Units: deg/s/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2SRV_ACCEL", 10, AP_RollController, accel_limit, 500),

    // @Param: _ANGLE_P
    // @DisplayName: Roll angle P gain
    // @Description: Roll angle P gain. If zero a gain of (1 / RLL2SRV_TCONST) will be used.
    // @Range: 0.000 12.000
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_ANGLE_P", 11, AP_RollController, angle_p, 0.0),

    // @Param: _INDI_EN
    // @DisplayName: Roll INDI controller mode
    // @Description: Selects PID, INDI shadow logging, or active INDI roll-rate control.
    // @Values: 0:PID,1:Shadow,2:Active
    // @User: Advanced
    AP_GROUPINFO("_INDI_EN", 12, AP_RollController, indi_enable, 0),

    // @Param: _INDI_KRATE
    // @DisplayName: Roll INDI rate error gain
    // @Description: Converts roll-rate error into the desired roll angular acceleration for the INDI controller.
    // @Range: 0.1 20.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_INDI_KRATE", 13, AP_RollController, indi_krate, 4.0f),

    // @Param: _INDI_G1
    // @DisplayName: Roll INDI control effectiveness
    // @Description: Estimated roll angular acceleration produced per degree of aileron deflection. This value must be identified for the aircraft.
    // @Range: 1.0 1000.0
    // @Increment: 1.0
    // @User: Advanced
    AP_GROUPINFO("_INDI_G1", 14, AP_RollController, indi_g1, 100.0f),

    // @Param: _INDI_FILT
    // @DisplayName: Roll INDI filter frequency
    // @Description: Low-pass filter cutoff frequency used for roll rate, roll acceleration and actuator command signals.
    // @Range: 1.0 30.0
    // @Units: Hz
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("_INDI_FILT", 15, AP_RollController, indi_filter_hz, 8.0f),

    AP_GROUPEND
};

// constructor
AP_RollController::AP_RollController(const AP_FixedWing &parms)
    : AP_FW_Controller(parms,
      AC_PID::Defaults{
        .p         = 0.08,
        .i         = 0.15,
        .d         = 0.0,
        .ff        = 0.345,
        .imax      = 0.666,
        .filt_T_hz = 3.0,
        .filt_E_hz = 0.0,
        .filt_D_hz = 12.0,
        .srmax     = 150.0,
        .srtau     = 1.0
    },
    AP_AutoTune::ATType::AUTOTUNE_ROLL)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Return the measured roll angle in degrees
float AP_RollController::get_measured_angle_deg() const
{
    return AP::ahrs().get_roll_deg();
}

// Return the measured roll rate in radians per second
float AP_RollController::get_measured_rate_rads() const
{
    return AP::ahrs().get_gyro().x;
}

// Return true if the airspeed should be considered as under speed
bool AP_RollController::is_underspeed() const
{
    return get_airspeed() <= float(aparm.airspeed_min);
}

// Return positive rate limit in deg per second, zero if disabled
float AP_RollController::get_positive_rate_limit_degs() const
{
    return MAX(gains.rmax_pos.get(), 0.0);
}

// Return negative rate limit in deg per second (as a positive number) zero if disabled
float AP_RollController::get_negative_rate_limit_degs() const
{
    return get_positive_rate_limit_degs();
}

// Return true if rate limits should be applied
bool AP_RollController::should_apply_rate_limits() const
{
    return !in_recovery;
}

/*
  Temporary INDI entry point.

  For this first wiring test it deliberately calls the original PID
  controller. The actual INDI equations will be added only after
  this version compiles and the new parameter is visible in SITL.
*/
float AP_RollController::run_indi_rate_control(float desired_rate_degs,
                                               float scaler,
                                               bool disable_integrator,
                                               bool ground_mode)
{
    /*
      PID always runs in parallel. It controls the aircraft in shadow
      mode and remains available as the fallback controller.
    */
    const float pid_output_cd =
        run_rate_control(desired_rate_degs,
                         scaler,
                         disable_integrator,
                         ground_mode);

    const float pid_output_deg = pid_output_cd * 0.01f;
    const float measured_rate_degs =
        degrees(get_measured_rate_rads());

    const float dt = AP::scheduler().get_loop_period_s();

    const int8_t indi_mode = indi_enable.get();

    /*
      Active INDI is blocked on the ground and while the aircraft
      is considered underspeed.
    */
    const bool active_requested =
        indi_mode >= 2 &&
        !ground_mode &&
        !is_underspeed();

    const float cutoff_hz =
        MAX(indi_filter_hz.get(), 0.1f);

    const float filter_term =
        M_2PI * cutoff_hz * dt;

    const float alpha =
        constrain_float(filter_term / (1.0f + filter_term),
                        0.0f,
                        1.0f);

    // Initialise all states from the PID output.
    if (!indi_initialized || dt <= 0.0f) {
        indi_rate_filtered_degs = measured_rate_degs;
        indi_rate_filtered_prev_degs = measured_rate_degs;
        indi_accel_filtered_degss = 0.0f;

        indi_actuator_filtered_deg = pid_output_deg;
        indi_last_output_deg = pid_output_deg;

        indi_active_last = false;
        indi_initialized = true;

        return pid_output_cd;
    }

    // Filter measured roll rate.
    indi_rate_filtered_degs +=
        alpha *
        (measured_rate_degs -
         indi_rate_filtered_degs);

    /*
      Differentiate the filtered roll rate. Do not add another
      low-pass filter because Act uses one filter of the same order.
    */
    indi_accel_filtered_degss =
        (indi_rate_filtered_degs -
         indi_rate_filtered_prev_degs) / dt;

    indi_rate_filtered_prev_degs =
        indi_rate_filtered_degs;

    /*
      Filter the command actually returned during the preceding
      controller loop—not always the PID command.
    */
    indi_actuator_filtered_deg +=
        alpha *
        (indi_last_output_deg -
         indi_actuator_filtered_deg);

    const float desired_accel_degss =
        indi_krate.get() *
        (desired_rate_degs -
         indi_rate_filtered_degs);

    const float g1_degss_per_deg =
        MAX(indi_g1.get(), 1.0f);

    // Unconstrained INDI command, retained in the log as Cmd.
    const float indi_cmd_deg =
        indi_actuator_filtered_deg +
        (desired_accel_degss -
         indi_accel_filtered_degss) /
        g1_degss_per_deg;

    // Equivalent aileron command must remain within ArduPlane limits.
    const float indi_cmd_limited_deg =
        constrain_float(indi_cmd_deg,
                        -45.0f,
                        45.0f);

    /*
      Require active mode during two consecutive loops. The first
      loop after changing 1 -> 2 still returns PID, providing a
      bumpless initialisation of the active INDI state.
    */
    const bool indi_active =
        active_requested && indi_active_last;

    const float output_deg =
        indi_active ?
        indi_cmd_limited_deg :
        pid_output_deg;

    /*
      Mode recorded in the log:
      1 = PID output/shadow INDI
      2 = active INDI output
    */
    const uint8_t actual_mode =
        indi_active ? 2U : 1U;

    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms - indi_last_log_ms >= 20U) {
        indi_last_log_ms = now_ms;

        AP::logger().WriteStreaming(
            "INDI",
            "TimeUS,DesR,RawR,FltR,DesA,Acc,Act,PID,Cmd,Out,Mode",
            "QfffffffffB",
            AP_HAL::micros64(),
            desired_rate_degs,
            measured_rate_degs,
            indi_rate_filtered_degs,
            desired_accel_degss,
            indi_accel_filtered_degss,
            indi_actuator_filtered_deg,
            pid_output_deg,
            indi_cmd_deg,
            output_deg,
            actual_mode);
    }

    /*
      Save the command actually returned. This becomes the actuator
      state used by INDI during the following loop.
    */
    indi_last_output_deg = output_deg;
    indi_active_last = active_requested;

    return output_deg * 100.0f;
}

/*
 Function returns an equivalent aileron deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
*/
float AP_RollController::run_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode)
{
    /*
      prevent indecision in the roll controller when target roll is
      close to 180 degrees from the current roll
     */
    const float indecision_threshold_deg = 160;
    const float last_desired_rate = _pid_info.target;
    const float abs_angle_err_deg = fabsf(angle_err_deg);
    if (abs_angle_err_deg > indecision_threshold_deg &&
        angle_err_deg <= 180) {
        if (desired_rate_degs * last_desired_rate < 0) {
            desired_rate_degs = -desired_rate_degs;
            // increase the desired rate in proportion to the extra
            // angle we are requesting
            const float new_angle_err_deg = abs_angle_err_deg + (180 - abs_angle_err_deg)*2;
            desired_rate_degs *= new_angle_err_deg / abs_angle_err_deg;
        }
    }

    // in_recovery flag is only valid for single loop, clear it
        // in_recovery flag is only valid for single loop, clear it
    in_recovery = false;

    if (indi_enable.get() != 0) {
        return run_indi_rate_control(desired_rate_degs,
                                     scaler,
                                     disable_integrator,
                                     ground_mode);
    }
    indi_initialized = false;
    return run_rate_control(desired_rate_degs,
                            scaler,
                            disable_integrator,
                            ground_mode);
}

/*
  convert from old to new PIDs
  this is a temporary conversion function during development
 */
void AP_RollController::convert_pid()
{
    AP_Float &ff = rate_pid.ff();
    if (ff.configured()) {
        return;
    }
    float old_ff=0, old_p=1.0, old_i=0.3, old_d=0.08;
    int16_t old_imax=3000;
    bool have_old = AP_Param::get_param_by_index(this, 1, AP_PARAM_FLOAT, &old_p);
    have_old |= AP_Param::get_param_by_index(this, 3, AP_PARAM_FLOAT, &old_i);
    have_old |= AP_Param::get_param_by_index(this, 2, AP_PARAM_FLOAT, &old_d);
    have_old |= AP_Param::get_param_by_index(this, 6, AP_PARAM_FLOAT, &old_ff);
    have_old |= AP_Param::get_param_by_index(this, 5, AP_PARAM_INT16, &old_imax);
    if (!have_old) {
        // none of the old gains were set
        return;
    }

    const float kp_ff = MAX((old_p - old_i * gains.tau) * gains.tau  - old_d, 0);
    rate_pid.ff().set_and_save(old_ff + kp_ff);
    rate_pid.kI().set_and_save_ifchanged(old_i * gains.tau);
    rate_pid.kP().set_and_save_ifchanged(old_d);
    rate_pid.kD().set_and_save_ifchanged(0);
    rate_pid.kIMAX().set_and_save_ifchanged(old_imax/4500.0);
}
