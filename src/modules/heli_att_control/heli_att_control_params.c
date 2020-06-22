/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file heli_att_control_params.c
 * Parameters for helicopter attitude controller.
 *
 * @author Hao XU <xuhao3e8@gmail.com>
 */

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_P, 7.0f);

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLLRATE_P, 0.2f);

/**
 * Roll rate I gain
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLLRATE_I, 0.01f);

/**
 * Roll rate integrator limit
 *
 * Roll rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large roll moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_RR_INT_LIM, 0.30f);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLLRATE_D, 0.00f);

/**
 * Roll rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLLRATE_FF, 0.0f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_P, 7.0f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCHRATE_P, 0.16f);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCHRATE_I, 0.05f);

/**
 * Pitch rate integrator limit
 *
 * Pitch rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large pitch moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_PR_INT_LIM, 0.30f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCHRATE_D, 0.000f);

/**
 * Pitch rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCHRATE_F, 0.0f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_YAW_P, 2.8f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 2
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_YAWRATE_P, 0.6f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_YAWRATE_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * Yaw rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large yaw moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_YR_INT_LIM, 0.30f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_YAWRATE_FF, 0.0f);


/**
 * Yaw rate feedforward by rotor speed
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_YAWRATE_RS, 0.2f);

/**
 * Yaw feed forward
 *
 * Feed forward weight for manual yaw control. 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_YAW_FF, 0.5f);

/**
 * Max roll rate
 *
 * Limit for roll rate in manual and auto modes (except acro).
 * Has effect for large rotations in autonomous mode, to avoid large control
 * output and mixer saturation.
 *
 * This is not only limited by the vehicle's properties, but also by the maximum
 * measurement rate of the gyro.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLRATE_MAX, 360.0f);

/**
 * Max pitch rate
 *
 * Limit for pitch rate in manual and auto modes (except acro).
 * Has effect for large rotations in autonomous mode, to avoid large control
 * output and mixer saturation.
 *
 * This is not only limited by the vehicle's properties, but also by the maximum
 * measurement rate of the gyro.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_PITRATE_MAX, 360.0f);

/**
 * Max yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_YAWRATE_MAX, 360.0f);

/**
 * Max yaw rate in auto mode
 *
 * Limit for yaw rate, has effect for large rotations in autonomous mode,
 * to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_YAWRAUT_MAX, 180.0f);

/**
 * Max acro roll rate
 * default: 2 turns per second
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ACRO_R_MAX, 300.0f);

/**
 * Max acro pitch rate
 * default: 2 turns per second
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ACRO_P_MAX, 300.0f);

/**
 * Max acro yaw rate
 * default 1.5 turns per second
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ACRO_Y_MAX, 300.0f);

/**
 * Acro mode Expo factor for Roll and Pitch.
 *
 * Exponential factor for tuning the input curve shape.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ACRO_EXPO, 0.0f);

/**
 * Acro mode Expo factor for Yaw.
 *
 * Exponential factor for tuning the input curve shape.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ACRO_EXPO_Y, 0.0f);

/**
 * Acro mode SuperExpo factor for Roll and Pitch.
 *
 * SuperExpo factor for refining the input curve shape tuned using HELI_ACRO_EXPO.
 *
 * 0 Pure Expo function
 * 0.7 resonable shape enhancement for intuitive stick feel
 * 0.95 very strong bent input curve only near maxima have effect
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ACRO_SUPEXP, 0.0f);

/**
 * Acro mode SuperExpo factor for Yaw.
 *
 * SuperExpo factor for refining the input curve shape tuned using HELI_ACRO_EXPO_Y.
 *
 * 0 Pure Expo function
 * 0.7 resonable shape enhancement for intuitive stick feel
 * 0.95 very strong bent input curve only near maxima have effect
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ACRO_SUPEXY, 0.0f);

/**
 * Threshold for Rattitude mode
 *
 * Manual input needed in order to override attitude control rate setpoints
 * and instead pass manual stick inputs as rate setpoints
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_RATT_TH, 0.8f);

/**
 * Helicopter tail mode
 * 0 Normal servo drive tail
 * 1 Use directly drive tail
 * 2 Use coaxial to control tail
 * @boolean
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_INT32(HELI_DIRECT_TAIL, 0);

/**
 * Cutoff frequency for the low pass filter on the D-term in the rate controller
 *
 * The D-term uses the derivative of the rate and thus is the most susceptible to noise.
 * Therefore, using a D-term filter allows to decrease the driver-level filtering, which
 * leads to reduced control latency and permits to increase the P gains.
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 1000
 * @decimal 0
 * @increment 0.1
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_ROLL_CUT, 30.f);


/**
 * Cutoff frequency for the low pass filter on the D-term in the rate controller
 *
 * The D-term uses the derivative of the rate and thus is the most susceptible to noise.
 * Therefore, using a D-term filter allows to decrease the driver-level filtering, which
 * leads to reduced control latency and permits to increase the P gains.
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 1000
 * @decimal 0
 * @increment 0.1
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_PITCH_CUT, 30.f);


/**
 * Cutoff frequency for the low pass filter on the D-term in the rate controller
 *
 * The D-term uses the derivative of the rate and thus is the most susceptible to noise.
 * Therefore, using a D-term filter allows to decrease the driver-level filtering, which
 * leads to reduced control latency and permits to increase the P gains.
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 1000
 * @decimal 0
 * @increment 0.1
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_YAW_CUT, 30.f);


/**
 * Helicopter air-mode
 *
 * The air-mode enables the mixer to increase the total thrust of the multirotor
 * in order to keep attitude and rate control even at low and high throttle.
 * This function should be disabled during tuning as it will help the controller
 * to diverge if the closed-loop is unstable.
 *
 * @boolean
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_INT32(HELI_AIRMODE, 0);

/**
 * Helicopter fixed speed rotor
 *
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_FIXED_SPEED, 0.7f);

/**
 * Helicopter trim aileron
 *
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_TRIM_AIL, 0.0f);

/**
 * Helicopter trim elevator
 *
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_TRIM_ELE, 0.0f);

/**
 * Helicopter trim rudder
 *
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_TRIM_RUD, 0.0f);


/**
 * Helicopter coll max
 *
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_COLL_MAX, 0.4f);

/**
 * Helicopter coll max
 *
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_COLL_MIN, -0.2f);

/**
 * HELI ROTOR SPEED MODE
 *
 * The default uses fix rotor speed
 * If parameter use rotor speed from AUX2
 *
 * @min 0
 * @max 2
 * @value 0 Rotor speed fixed from param
 * @value 1 Rotor speed from AUX2
 * @value 2 Rotor speed from Thrust
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_INT32(HELI_ROTSPD_MODE, 1);

/**
 * HELI YAW MODE
 *
 * The default uses fix rotor speed
 * If parameter use rotor speed from AUX2
 *
 * @min 0
 * @max 2
 * @value 0 Normal tail
 * @value 1 Direct tail
 * @value 2 Coaxial
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_INT32(HELI_TAIL_MODE, 0);

/**
 * HELI CALIB SERVO MODE
 *
 *
 * @min 0
 * @max 1
 * @value 0 Servo move normally
 * @value 1 Servo fixed to install.
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_INT32(HELI_CALIB_SERVO, 0);

/**
 * HELI TRIM ROLL ANGLE
 *
 * Trim pitch so can hover
 *
 *
 * @min -30.0
 * @max 30.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_INT32(HELI_HOVER_ROLL, 0);


/**
 * HELI TRIM PITCH ANGLE
 *
 *  Trim roll so can hover
 *
 * @min -30.0
 * @max 30.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_INT32(HELI_HOVER_PITCH, 0);


/**
 * HELI MODE IDEN MIN frequency
 *
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_IDEN_FMIN, 10);

/**
 * HELI MODE IDEN MAX frequency
 *
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_IDEN_FMAX, 30);

/**
 * HELI MODE IDEN Amplitude
 *
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_IDEN_AMP, 0.1);

/**
 * HELI MODE IDEN Cyclic Time
 *
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HELI_IDEN_T, 10);

/**
 * HELI MODE IDEN Repeat Times
 *
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_INT32(HELI_IDEN_N, 3);

/**
 * HELI MODE IDEN Channel
 *
 * 0, 1, 2 roll pitch yaw, 3 collective, 4 rotor speed
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.05
 * @group Helicopter Attitude Control
 */
PARAM_DEFINE_INT32(HELI_IDEN_C, 2);
