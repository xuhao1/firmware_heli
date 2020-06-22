//
// Created by xuhao on 2018/10/18.
//

#include "heli_att_control.hpp"

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <systemlib/mavlink_log.h>

#define MIN_TAKEOFF_COLL    0.2f
#define MIN_TAKEOFF_SPEED    0.3f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3
orb_advert_t		    _mavlink_log_pub;


float generate_sweeep_signal_base(float t, float T, float omgmin=0.3, float omgmax=12, float c1 = 4.0, float c2 = 0.0187) {
    // def generate_sweep_signal_base_func(T, omgmin=0.3, omgmax=12, c1=4.0, c2=0.0187):
    //     return lambda t: math.sin(t * omgmin + (omgmax - omgmin) * c2 * (T / c1 * (math.exp(c1 * t / T) - 1) - t))
    // func  = generate_sweep_signal_base_func(args.cycle, omgmin=args.fmin*6.28, omgmax=args.fmax*6.28)
    t = fmodf(t, T);
    return sinf(t * omgmin + (omgmax - omgmin) * c2 * (T / c1 * (expf(c1 * t / T) - 1) - t));
}

using namespace matrix;
int HelicopterAttitudeControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
Helicopter attitude control code
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("heli_att_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}
HelicopterAttitudeControl::HelicopterAttitudeControl():
    ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
    _loop_perf(perf_alloc(PC_ELAPSED, "heli_att_control")),
    _lp_filters{
            {initial_update_rate_hz, 50.f},
            {initial_update_rate_hz, 50.f},
            {initial_update_rate_hz, 50.f}}
{

    PX4_WARN("Start Helicopter");

    // _vehicle_status.is_rotary_wing = true;

    /* initialize quaternions in messages to be valid */
    _v_att.q[0] = 1.f;
    _v_att_sp.q_d[0] = 1.f;

    _rates_prev.zero();
    _rates_prev_filtered.zero();
    _rates_sp.zero();
    _rates_int.zero();
    _rotor_speed_sp = 0.0f;
    _coll_sp = 0.0f;
    _att_control.zero();

    parameters_updated();
}


void
HelicopterAttitudeControl::parameters_updated()
{
    /* Store some of the parameters in a more convenient way & precompute often-used values */

    /* roll gains */
    _attitude_p(0) = _roll_p.get();
    _rate_p(0) = _roll_rate_p.get();
    _rate_i(0) = _roll_rate_i.get();
    _rate_int_lim(0) = _roll_rate_integ_lim.get();
    _rate_d(0) = _roll_rate_d.get();
    _rate_ff(0) = _roll_rate_ff.get();

    /* pitch gains */
    _attitude_p(1) = _pitch_p.get();
    _rate_p(1) = _pitch_rate_p.get();
    _rate_i(1) = _pitch_rate_i.get();
    _rate_int_lim(1) = _pitch_rate_integ_lim.get();
    _rate_d(1) = _pitch_rate_d.get();
    _rate_ff(1) = _pitch_rate_ff.get();

    /* yaw gains */
    _attitude_p(2) = _yaw_p.get();
    _rate_p(2) = _yaw_rate_p.get();
    _rate_i(2) = _yaw_rate_i.get();
    _rate_int_lim(2) = _yaw_rate_integ_lim.get();
    _rate_d(2) = _yaw_rate_d.get();
    _rate_ff(2) = _yaw_rate_ff.get();

    yawrate_ff_rotor_speed = _yaw_rate_ff_rotor_speed.get();

    if ( fabsf(_lp_filters[0].get_cutoff_freq() - _output_cutoff_freq_r.get()) > 0.01f) {
        _lp_filters[0].set_cutoff_frequency(_loop_update_rate_hz, _output_cutoff_freq_r.get());
        _lp_filters[0].reset(_rates_prev(0));
    }

    if ( fabsf(_lp_filters[1].get_cutoff_freq() - _output_cutoff_freq_p.get()) > 0.01f) {
        _lp_filters[1].set_cutoff_frequency(_loop_update_rate_hz, _output_cutoff_freq_p.get());
        _lp_filters[1].reset(_rates_prev(1));
    }

    if ( fabsf(_lp_filters[2].get_cutoff_freq() - _output_cutoff_freq_y.get()) > 0.01f) {
        _lp_filters[2].set_cutoff_frequency(_loop_update_rate_hz, _output_cutoff_freq_y.get());
        _lp_filters[2].reset(_rates_prev(2));
    }

    /* angular rate limits */
    _heli_rate_max(0) = math::radians(_roll_rate_max.get());
    _heli_rate_max(1) = math::radians(_pitch_rate_max.get());
    _heli_rate_max(2) = math::radians(_yaw_rate_max.get());

    /* auto angular rate limits */
    _auto_rate_max(0) = math::radians(_roll_rate_max.get());
    _auto_rate_max(1) = math::radians(_pitch_rate_max.get());
    _auto_rate_max(2) = math::radians(_yaw_auto_max.get());

    /* manual rate control acro mode rate limits and expo */
    _acro_rate_max(0) = math::radians(_acro_roll_max.get());
    _acro_rate_max(1) = math::radians(_acro_pitch_max.get());
    _acro_rate_max(2) = math::radians(_acro_yaw_max.get());

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());

    _actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);
}


void
HelicopterAttitudeControl::parameter_update_poll()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		parameters_updated();
	}
}

void
HelicopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	_v_att_sp_sub.update(&_v_att_sp);
}

void
HelicopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	if (_vehicle_status_sub.update(&_vehicle_status)) {
	}
}

void
HelicopterAttitudeControl::vehicle_attitude_poll()
{
	/* check if there is a new message */
	const uint8_t prev_quat_reset_counter = _v_att.quat_reset_counter;

	if (_v_att_sub.update(&_v_att)) {
		// Check for a heading reset
		if (prev_quat_reset_counter != _v_att.quat_reset_counter) {
			// we only extract the heading change from the delta quaternion
			// _man_yaw_sp += Eulerf(Quatf(_v_att.delta_q_reset)).psi();
		}
	}
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
HelicopterAttitudeControl::control_attitude(float dt)
{
    vehicle_attitude_setpoint_poll();
    _thrust_sp = - _v_att_sp.thrust_body[2];

    /* prepare yaw weight from the ratio between roll/pitch and yaw gains */
    Vector3f attitude_gain = _attitude_p;
    const float roll_pitch_gain = (attitude_gain(0) + attitude_gain(1)) / 2.f;
    const float yaw_w = math::constrain(attitude_gain(2) / roll_pitch_gain, 0.f, 1.f);
    attitude_gain(2) = roll_pitch_gain;

    /* get estimated and desired vehicle attitude */
    Quatf q(_v_att.q);
    Quatf qd(_v_att_sp.q_d);

    /* ensure input quaternions are exactly normalized because acosf(1.00001) == NaN */
    q.normalize();
    qd.normalize();

    /* calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch */
    Vector3f e_z = q.dcm_z();
    Vector3f e_z_d = qd.dcm_z();
    Quatf qd_red(e_z, e_z_d);

    if (abs(qd_red(1)) > (1.f - 1e-5f) || abs(qd_red(2)) > (1.f - 1e-5f)) {
        /* In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
         * full attitude control anyways generates no yaw input and directly takes the combination of
         * roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable. */
        qd_red = qd;

    } else {
        /* transform rotation from current to desired thrust vector into a world frame reduced desired attitude */
        qd_red *= q;
    }

    /* mix full and reduced desired attitude */
    Quatf q_mix = qd_red.inversed() * qd;
    q_mix *= math::signNoZero(q_mix(0));
    /* catch numerical problems with the domain of acosf and asinf */
    q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
    q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
    qd = qd_red * Quatf(cosf(yaw_w * acosf(q_mix(0))), 0, 0, sinf(yaw_w * asinf(q_mix(3))));

    /* quaternion attitude control law, qe is rotation from q to qd */
    Quatf qe = q.inversed() * qd;

    /* using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
     * also taking care of the antipodal unit quaternion ambiguity */
    Vector3f eq = 2.f * math::signNoZero(qe(0)) * qe.imag();

    /* calculate angular rates setpoint */
    _rates_sp = eq.emult(attitude_gain);

    /* Feed forward the yaw setpoint rate.
     * The yaw_feedforward_rate is a commanded rotation around the world z-axis,
     * but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
     * Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
     * and multiply it by the yaw setpoint rate (yaw_sp_move_rate) and gain (_yaw_ff).
     * This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
     * such that it can be added to the rates setpoint.
     */
    Vector3f yaw_feedforward_rate = q.inversed().dcm_z();
    yaw_feedforward_rate *= _v_att_sp.yaw_sp_move_rate * _yaw_ff.get();
    _rates_sp += yaw_feedforward_rate;


    /* limit rates */
    for (int i = 0; i < 3; i++) {
        if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
            !_v_control_mode.flag_control_manual_enabled) {
            _rates_sp(i) = math::constrain(_rates_sp(i), -_auto_rate_max(i), _auto_rate_max(i));

        } else {
            _rates_sp(i) = math::constrain(_rates_sp(i), -_heli_rate_max(i), _heli_rate_max(i));
        }
    }
}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
Vector3f
HelicopterAttitudeControl::pid_attenuations(float speed_sp)
{
    float rate = 1.78f;
	if (speed_sp > 0.4f)
	{
        //We trim our pid on rotor speed 1.4
		rate = fminf(1.0f, 1/fabsf(speed_sp)) / 1.4f;
	}

	Vector3f pidAttenuationPerAxis;
	pidAttenuationPerAxis(AXIS_INDEX_ROLL) = rate*rate;
	pidAttenuationPerAxis(AXIS_INDEX_PITCH) = rate*rate;


    if (_heli_tail_mode.get() == 0) {
    	pidAttenuationPerAxis(AXIS_INDEX_YAW) = rate*rate;
    } else {
    	pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1;
    }
	return pidAttenuationPerAxis;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
HelicopterAttitudeControl::control_attitude_rates(float dt, matrix::Vector3f rates)
{
    /* reset integral if disarmed */
    if (!_v_control_mode.flag_armed) {
        _rates_int.zero();
    }

    Vector3f rates_p_scaled = _rate_p.emult(pid_attenuations(_rotor_speed_sp));
    Vector3f rates_i_scaled = _rate_i.emult(pid_attenuations(_rotor_speed_sp));
    Vector3f rates_d_scaled = _rate_d.emult(pid_attenuations(_rotor_speed_sp));

    /* angular rates error */
    Vector3f rates_err = _rates_sp - rates;

    /* apply low-pass filtering to the rates for D-term */

    _att_control = rates_p_scaled.emult(rates_err) +
                   _rates_int -
                   rates_d_scaled.emult(rates - _rates_prev) / dt +
                   _rate_ff.emult(_rates_sp);

    _rates_prev = rates;
    _rates_prev_filtered = rates;

    _att_control(0) = _lp_filters[0].apply(_att_control(0));
    _att_control(1) = _lp_filters[1].apply(_att_control(1));
    _att_control(2) = _lp_filters[2].apply(_att_control(2)) + yawrate_ff_rotor_speed * _rotor_speed_sp;


    if (_manual_control_sp.aux3 > 0.9f) {
        //Will start sweep procedure
        if(!is_in_sweep) {
            is_in_sweep = true;
            t_sweep_start = hrt_absolute_time()/1e6f;
            mavlink_log_info(&_mavlink_log_pub, "Start sweep on channel %d", _heli_iden_C.get());
        }

        float t_sweep = hrt_absolute_time()/1e6f - t_sweep_start;
        int channel = _heli_iden_C.get();
        if (channel <= 4) {
            float s = 0;
            //generate_sweeep_signal_base(double t, double T, double omgmin=0.3, double omgmax=12, double c1 = 4.0, double c2 = 0.0187);
            //Must running on auto thrust mode for collective sweeping
            if (t_sweep < _heli_iden_T.get()*_heli_iden_N.get()) {
                s = _heli_iden_amp.get() * generate_sweeep_signal_base(t_sweep, _heli_iden_T.get(),
                _heli_iden_fmin.get()*M_PI_F*2, _heli_iden_fmax.get()*M_PI_F*2);
            }

            // mavlink_log_info(&_mavlink_log_pub, "T %f s %f", (double)t_sweep, (double)s);

            if(channel < 3) {
                _att_control(channel) = _att_control(channel) + s;
            } else {
                if(channel == 3) {
                    _coll_sp = _coll_sp + s;
                } if (channel == 4) {
                    _rotor_speed_sp = _rotor_speed_sp + s;
                }
            }
        }
    } else {
        is_in_sweep = false;
    }


    /* update integral only if motors are providing enough thrust to be effective */
    // if (_coll_sp > MIN_TAKEOFF_COLL && _rotor_speed_sp > MIN_TAKEOFF_SPEED)
    {
        for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
            // Perform the integration using a first order method and do not propagate the result if out of range or invalid
            float rate_i = _rates_int(i) + rates_i_scaled(i) * rates_err(i) * dt;

            if (PX4_ISFINITE(rate_i) && rate_i > -_rate_int_lim(i) && rate_i < _rate_int_lim(i)) {
                _rates_int(i) = rate_i;

            }
        }
    }

    /* explicitly limit the integrator state */
    for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
        _rates_int(i) = math::constrain(_rates_int(i), -_rate_int_lim(i), _rate_int_lim(i));

    }
}



void
HelicopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}
    perf_begin(_loop_perf);

	vehicle_angular_velocity_s angular_velocity;

    bool auto_thrust_mode = true;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
        const Vector3f rates{angular_velocity.xyz};
		_actuators.timestamp_sample = angular_velocity.timestamp_sample;

        const hrt_abstime now = hrt_absolute_time();
        float dt = (now - _last_run) / 1e6f;
        _last_run = now;

        /* guard against too small (< 2ms) and too large (> 20ms) dt's */
        if (dt < 0.002f) {
            dt = 0.002f;

        } else if (dt > 0.02f) {
            dt = 0.02f;
        }

        /* check for updates in other topics */
        _v_control_mode_sub.update(&_v_control_mode);
        _manual_control_sp_sub.update(&_manual_control_sp);
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

        vehicle_status_poll();
        vehicle_attitude_poll();;

        if (_v_control_mode.flag_control_attitude_enabled) {
            // mavlink_log_info(&_mavlink_log_pub, "Attitude Control Enabled");

            // reset yaw setpoint during transitions, tailsitter.cpp generates
			// attitude setpoint for the transition


            if (_v_control_mode.flag_control_manual_enabled &&
				    !_v_control_mode.flag_control_altitude_enabled &&
				    !_v_control_mode.flag_control_velocity_enabled &&
				    !_v_control_mode.flag_control_position_enabled) {
					generate_attitude_setpoint(dt, _reset_yaw_sp);
                    auto_thrust_mode = false;
                    _reset_yaw_sp = _vehicle_land_detected.landed;
            } else {
                // _reset_yaw_sp = true;
            }

            control_attitude(dt);

            /* publish attitude rates setpoint */
            _v_rates_sp.roll = _rates_sp(0);
            _v_rates_sp.pitch = _rates_sp(1);
            _v_rates_sp.yaw = _rates_sp(2);
            _v_rates_sp.thrust_body[2] = - _thrust_sp;
            _v_rates_sp.timestamp = hrt_absolute_time();

        	_v_rates_sp_pub.publish(_v_rates_sp);

        } else {
            /* attitude controller disabled, poll rates setpoint topic */
            if (_v_control_mode.flag_control_manual_enabled) {
                /* manual rates control - ACRO mode */
                Vector3f man_rate_sp(
                        math::superexpo(_manual_control_sp.y, _acro_expo_rp.get(), _acro_superexpo_rp.get()),
                        math::superexpo(-_manual_control_sp.x, _acro_expo_rp.get(), _acro_superexpo_rp.get()),
                        math::superexpo(_manual_control_sp.r, _acro_expo_y.get(), _acro_superexpo_y.get()));
                _rates_sp = man_rate_sp.emult(_acro_rate_max);
                _thrust_sp = _manual_control_sp.z;
                auto_thrust_mode = false;
                /* publish attitude rates setpoint */
                _v_rates_sp.roll = _rates_sp(0);
                _v_rates_sp.pitch = _rates_sp(1);
                _v_rates_sp.yaw = _rates_sp(2);
                _v_rates_sp.thrust_body[2] = - _thrust_sp;
                _v_rates_sp.timestamp = hrt_absolute_time();

            	_v_rates_sp_pub.publish(_v_rates_sp);

            } else {
                /* attitude controller disabled, poll rates setpoint topic */
                if (_v_rates_sp_sub.update(&_v_rates_sp)) {
                    _rates_sp(0) = _v_rates_sp.roll;
                    _rates_sp(1) = _v_rates_sp.pitch;
                    _rates_sp(2) = _v_rates_sp.yaw;
                    _thrust_sp = -_v_rates_sp.thrust_body[2];
                }
            }
        }

        //Here is param for fix speed rotor.
        if (auto_thrust_mode) {
            if (_heli_rotor_speed_mode.get() == 0) {
                _rotor_speed_sp = _heli_fixed_speed.get();
                _coll_sp = _thrust_sp;
            } else if (_heli_rotor_speed_mode.get() == 1) {
                _rotor_speed_sp = (_manual_control_sp.aux2 + 1.0f) * 0.5f;
                _coll_sp = _thrust_sp;
            } else if (_heli_rotor_speed_mode.get() == 2)  {
                _rotor_speed_sp = _thrust_sp;
                _coll_sp = _thrust_sp;
            }
        } else {
            _rotor_speed_sp = _thrust_sp;
            _coll_sp = _manual_control_sp.aux1;
        }

        if (!_v_control_mode.flag_armed) {
            _rotor_speed_sp = 0;
        }

        if (_v_control_mode.flag_control_rates_enabled) {
            control_attitude_rates(dt, rates);

            /* publish actuator controls */


            publish_actuator_controls();

            /* publish controller status */
            rate_ctrl_status_s rate_ctrl_status;
            rate_ctrl_status.timestamp = hrt_absolute_time();
            rate_ctrl_status.rollspeed_integ = _rates_int(0);
            rate_ctrl_status.pitchspeed_integ = _rates_int(1);
            rate_ctrl_status.yawspeed_integ = _rates_int(2);

            _controller_status_pub.publish(rate_ctrl_status);
        }

        if (_v_control_mode.flag_control_termination_enabled) {
            _rotor_speed_sp = 0.0f;

            _rates_int.zero();
            _rates_prev.zero();
            _rates_prev_filtered.zero();

            // _att_control.zero();
            // _coll_sp = 0.0f;

            publish_actuator_controls();
        }

        /* calculate loop update rate while disarmed or at least a few times (updating the filter is expensive) */
        if (!_v_control_mode.flag_armed || (now - _task_start) < 3300000) {
            _dt_accumulator += dt;
            ++_loop_counter;

            if (_dt_accumulator > 1.f) {
                const float loop_update_rate = (float)_loop_counter / _dt_accumulator;
                _loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
                _dt_accumulator = 0;
                _loop_counter = 0;
                _lp_filters[0].set_cutoff_frequency(_loop_update_rate_hz, _output_cutoff_freq_r.get());
                _lp_filters[1].set_cutoff_frequency(_loop_update_rate_hz, _output_cutoff_freq_p.get());
                _lp_filters[2].set_cutoff_frequency(_loop_update_rate_hz, _output_cutoff_freq_y.get());
            }
        }

        parameter_update_poll();
    }
    perf_end(_loop_perf);
}


void
HelicopterAttitudeControl::generate_attitude_setpoint(float dt, bool reset_yaw_sp)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};
	const float yaw = Eulerf(Quatf(_v_att.q)).psi();
    if (reset_yaw_sp) {
		_man_yaw_sp = yaw;

	} else if (_manual_control_sp.z > 0.05f) {
		const float yaw_rate = math::radians(_acro_yaw_max.get());
		attitude_setpoint.yaw_sp_move_rate = _manual_control_sp.r * yaw_rate;
		_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
	}

	const float x = _manual_control_sp.x * _man_tilt_max;
	const float y = _manual_control_sp.y * _man_tilt_max;

	// we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
	Vector2f v = Vector2f(y, -x);
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rpy = AxisAnglef(v(0), v(1), 0.f);
	Eulerf euler_sp = q_sp_rpy;
	attitude_setpoint.roll_body = euler_sp(0);
	attitude_setpoint.pitch_body = euler_sp(1);
	attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);

	/* copy quaternion setpoint to attitude setpoint topic */
	Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
	q_sp.copyTo(attitude_setpoint.q_d);
	attitude_setpoint.q_d_valid = true;

	attitude_setpoint.thrust_body[2] = -_manual_control_sp.z;
	attitude_setpoint.timestamp = hrt_absolute_time();

    _vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
    /*
	_landing_gear.landing_gear = get_landing_gear_state();
	_landing_gear.timestamp = hrt_absolute_time();
	_landing_gear_pub.publish(_landing_gear);*/
}


void
HelicopterAttitudeControl::publish_actuator_controls()
{
    float coll_max = _heli_coll_max.get();
    float coll_min = _heli_coll_min.get();
    _actual_coll_sp = (coll_max - coll_min) * _coll_sp + coll_min;

    if ( _heli_calib_servo.get() ) {
        _att_control(0) = _heli_trim_ail.get();
        _att_control(1) = _heli_trim_ele.get();
        _att_control(2) = _heli_trim_rud.get();
        // _actual_coll_sp = 0;
    } else {
        _att_control(0) = _att_control(0) + _heli_trim_ail.get();
        _att_control(1) = _att_control(1) + _heli_trim_ele.get();
        _att_control(2) = _att_control(2) + _heli_trim_rud.get();
    }
    _actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
    _actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
    _actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
    _actuators.control[3] = (PX4_ISFINITE(_rotor_speed_sp)) ? _rotor_speed_sp : 0.0f;
    _actuators.control[4] = (PX4_ISFINITE(_actual_coll_sp )) ? _actual_coll_sp  : 0.0f;

    _actuators.timestamp = hrt_absolute_time();
    _actuators_0_pub.publish(_actuators);
}

int HelicopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	HelicopterAttitudeControl *instance = new HelicopterAttitudeControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

HelicopterAttitudeControl *HelicopterAttitudeControl::instantiate(int argc, char *argv[])
{
    return new HelicopterAttitudeControl();
}

bool HelicopterAttitudeControl::init() {
    PX4_WARN("Initial Helicopter Attitude Control");
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

    mavlink_log_info(&_mavlink_log_pub, "Helicopter Attitude Start");

	return true;
}

int HelicopterAttitudeControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int heli_att_control_main(int argc, char *argv[])
{
    return HelicopterAttitudeControl::main(argc, argv);
}

