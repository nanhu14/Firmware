/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file motor_control.h
 *
 * ???????
 * ????????????????
 *
 * @author PX4 Development Team
 */

#pragma once

#include <stdint.h>

/**
 * ????????
 */
enum MotorControlMode {
	MODE_RUN = 0,      /**< ???????? */
	MODE_RAMP = 1,     /**< ?????? */
	MODE_STOP = 2      /**< ???? */
};

/**
 * ?????????
 */
struct MotorControlConfig {
	MotorControlMode mode;  /**< ???? */
	int motor_id;           /**< ?????-1??????? */
	float speed;            /**< ?????0.0-1.0? */
	float ramp_time;        /**< ??????? */
	float max_speed;        /**< ???????0.0-1.0? */
	bool emergency_stop;    /**< ???? */
};

/**
 * ???????
 */
struct MotorStatus {
	bool armed;             /**< ???? */
	float current_speed;    /**< ???? */
	uint64_t start_time;    /**< ???????? */
	uint64_t run_time;      /**< ???????? */
	bool fault;             /**< ???? */
};

/**
 * PWM?????
 */
#define MOTOR_PWM_MIN          1000    /**< PWM??????? */
#define MOTOR_PWM_MAX          2000    /**< PWM??????? */
#define MOTOR_PWM_OFF          900     /**< ????PWM? */

/**
 * ??????
 */
#define MOTOR_SPEED_MIN        0.0f    /**< ???? */
#define MOTOR_SPEED_MAX        1.0f    /**< ???? */

/**
 * ???????????
 */
#define MOTOR_RAMP_TIME_MIN    0.1f    /**< ?????? */
#define MOTOR_RAMP_TIME_MAX    60.0f   /**< ?????? */

/**
 * ????????
 */
#define MOTOR_ID_ALL          -1       /**< ??????? */
#define MOTOR_MAX_CHANNELS    16       /**< ??????? */

/**
 * ???????Hz?
 */
#define MOTOR_CONTROL_RATE    50       /**< ???????? */

/**
 * ??????
 */
#define MOTOR_SAFE_VOLTAGE_MIN   10.0f /**< ???????V? */
#define MOTOR_SAFE_CURRENT_MAX   50.0f /**< ???????A? */
#define MOTOR_SAFE_TEMP_MAX      80.0f /**< ????????? */

/**
 * ??????
 */
enum MotorErrorCode {
	MOTOR_ERROR_NONE = 0,              /**< ??? */
	MOTOR_ERROR_INIT_FAILED = -1,      /**< ????? */
	MOTOR_ERROR_ARM_FAILED = -2,       /**< ???? */
	MOTOR_ERROR_DISARM_FAILED = -3,    /**< ???? */
	MOTOR_ERROR_SET_SPEED_FAILED = -4, /**< ?????? */
	MOTOR_ERROR_INVALID_PARAM = -5,    /**< ???? */
	MOTOR_ERROR_DEVICE_NOT_FOUND = -6, /**< ????? */
	MOTOR_ERROR_EMERGENCY_STOP = -7,   /**< ???? */
	MOTOR_ERROR_OVERHEAT = -8,         /**< ?? */
	MOTOR_ERROR_OVERCURRENT = -9,      /**< ?? */
	MOTOR_ERROR_UNDERVOLTAGE = -10     /**< ?? */
};
