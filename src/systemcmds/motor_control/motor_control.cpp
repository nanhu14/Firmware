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
 * @file motor_control.cpp
 *
 * ????????
 * ??????????????????????
 *
 * @author PX4 Development Team
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <platforms/px4_defines.h>

#include "systemlib/err.h"
#include "uORB/topics/actuator_controls.h"

#include "motor_control.h"

/* ???? */
static bool _thread_should_exit = false;
static bool _thread_running = false;
static int _motor_control_task;
static MotorControlConfig _config;

/* ???? */
extern "C" __EXPORT int motor_control_main(int argc, char *argv[]);
int motor_control_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

/* ???????? */
static int motor_init(int fd, unsigned long max_channels);
static int motor_set_speed(int fd, unsigned long max_channels, int motor_id, float speed);
static int motor_set_all_speed(int fd, unsigned long max_channels, float speed);
static int motor_stop(int fd, unsigned long max_channels);
static int motor_arm(int fd);
static int motor_disarm(int fd);
static bool validate_config(MotorControlConfig *cfg);

/**
 * ??????
 */
static void usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### ??
??????????????????
- ??????????????
- ???????
- ??????
- ?????????????
- ???????

??????????????????????
$ mc_att_control stop
$ fw_att_control stop

### ??
# ?????50%????
$ motor_control run --all --speed 0.5

# ?????????1?70%???
$ motor_control run --motor 1 --speed 0.7

# ????????0?100%???5??
$ motor_control ramp --all --time 5

# ??????
$ motor_control stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME_SIMPLE("motor_control", "command");
	
	PRINT_MODULE_USAGE_COMMAND_DESCR("run", "????");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "all", "??????", true);
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 0, 16, "????(0-15)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('s', 0.5, 0.0, 1.0, "??(0.0-1.0)", true);
	
	PRINT_MODULE_USAGE_COMMAND_DESCR("ramp", "????");
	PRINT_MODULE_USAGE_PARAM_FLOAT('t', 5.0, 0.1, 60.0, "????(?)", true);
	
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "??????");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "????");

	PRINT_MODULE_USAGE_PARAM_COMMENT("??: ????????!");
}

/**
 * ??????
 */
static bool validate_config(MotorControlConfig *cfg)
{
	if (cfg->speed < 0.0f || cfg->speed > 1.0f) {
		PX4_ERR("?????0.0?1.0??");
		return false;
	}

	if (cfg->motor_id < -1 || cfg->motor_id >= PWM_OUTPUT_MAX_CHANNELS) {
		PX4_ERR("???????0?%d??", PWM_OUTPUT_MAX_CHANNELS - 1);
		return false;
	}

	if (cfg->ramp_time < 0.1f || cfg->ramp_time > 60.0f) {
		PX4_ERR("???????0.1?60???");
		return false;
	}

	return true;
}

/**
 * ?????
 */
static int motor_init(int fd, unsigned long max_channels)
{
	/* ????????????????? */
	struct actuator_controls_s actuators;
	int act_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);

	orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, act_sub, &actuators);
	px4_usleep(50000);

	bool orb_updated;
	orb_check(act_sub, &orb_updated);

	if (orb_updated) {
		PX4_ERR("????????????????????????");
		PX4_ERR("  mc_att_control stop");
		PX4_ERR("  fw_att_control stop");
		return -1;
	}

	/* ??????????? */
	if (px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&max_channels) != OK) {
		PX4_ERR("????PWM???");
		return -1;
	}

	PX4_INFO("??? %lu ?PWM??", max_channels);

	/* ??PWM??? */
	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));
	pwm_values.channel_count = max_channels;

	for (unsigned i = 0; i < max_channels; i++) {
		pwm_values.values[i] = PWM_DEFAULT_MIN;
	}

	if (px4_ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values) != OK) {
		PX4_ERR("??PWM?????");
		return -1;
	}

	/* ?????????????? */
	if (px4_ioctl(fd, PWM_SERVO_SET_ARM_OK, 0) != OK) {
		PX4_ERR("??ARM_OK??");
		return -1;
	}

	return 0;
}

/**
 * ????
 */
static int motor_arm(int fd)
{
	/* ??????????????????? */
	if (px4_ioctl(fd, PWM_SERVO_ARM, 0) != OK) {
		PX4_ERR("??????");
		return -1;
	}

	/* ????????????????? */
	if (px4_ioctl(fd, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0) != OK) {
		PX4_ERR("????????");
		return -1;
	}

	PX4_INFO("?????");
	return 0;
}

/**
 * ????
 */
static int motor_disarm(int fd)
{
	if (px4_ioctl(fd, PWM_SERVO_DISARM, 0) != OK) {
		PX4_ERR("??????");
		return -1;
	}

	PX4_INFO("?????");
	return 0;
}

/**
 * ????????
 */
static int motor_set_speed(int fd, unsigned long max_channels, int motor_id, float speed)
{
	if (motor_id < 0 || motor_id >= (int)max_channels) {
		PX4_ERR("???? %d ???? (0-%lu)", motor_id, max_channels - 1);
		return -1;
	}

	/* ???(0-1)???PWM? */
	int pwm = PWM_DEFAULT_MIN + (int)((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) * speed);
	
	/* ??PWM??? */
	if (pwm < PWM_DEFAULT_MIN) {
		pwm = PWM_DEFAULT_MIN;
	}
	if (pwm > PWM_DEFAULT_MAX) {
		pwm = PWM_DEFAULT_MAX;
	}

	if (ioctl(fd, PWM_SERVO_SET(motor_id), pwm) != OK) {
		PX4_ERR("???? %d PWM???", motor_id);
		return -1;
	}

	return 0;
}

/**
 * ????????
 */
static int motor_set_all_speed(int fd, unsigned long max_channels, float speed)
{
	/* ???(0-1)???PWM? */
	int pwm = PWM_DEFAULT_MIN + (int)((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) * speed);
	
	/* ??PWM??? */
	if (pwm < PWM_DEFAULT_MIN) {
		pwm = PWM_DEFAULT_MIN;
	}
	if (pwm > PWM_DEFAULT_MAX) {
		pwm = PWM_DEFAULT_MAX;
	}

	for (unsigned i = 0; i < max_channels; i++) {
		if (ioctl(fd, PWM_SERVO_SET(i), pwm) != OK) {
			PX4_ERR("???? %d PWM???", i);
			return -1;
		}
	}

	return 0;
}

/**
 * ??????
 */
static int motor_stop(int fd, unsigned long max_channels)
{
	return motor_set_all_speed(fd, max_channels, 0.0f);
}

/**
 * ???????
 */
int motor_control_thread_main(int argc, char *argv[])
{
	_thread_running = true;

	char *dev = PWM_OUTPUT0_DEVICE_PATH;
	unsigned long max_channels = 0;

	/* ??PWM?? */
	int fd = px4_open(dev, 0);
	if (fd < 0) {
		PX4_ERR("???? %s", dev);
		_thread_running = false;
		return -1;
	}

	/* ????? */
	if (motor_init(fd, max_channels) != 0) {
		px4_close(fd);
		_thread_running = false;
		return -1;
	}

	/* ??????? */
	if (px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&max_channels) != OK) {
		PX4_ERR("???????");
		px4_close(fd);
		_thread_running = false;
		return -1;
	}

	/* ???? */
	if (motor_arm(fd) != 0) {
		px4_close(fd);
		_thread_running = false;
		return -1;
	}

	/* ??????????????? */
	if (_config.mode == MODE_RUN) {
		PX4_INFO("??????? %.2f", (double)_config.speed);
		
		if (_config.motor_id == -1) {
			/* ?????? */
			motor_set_all_speed(fd, max_channels, _config.speed);
			PX4_INFO("??????? %.1f%% ??", (double)(_config.speed * 100.0f));
		} else {
			/* ?????? */
			motor_set_speed(fd, max_channels, _config.motor_id, _config.speed);
			PX4_INFO("?? %d ??? %.1f%% ??", _config.motor_id, (double)(_config.speed * 100.0f));
		}

		/* ?????? */
		while (!_thread_should_exit) {
			px4_usleep(100000); // 100ms
		}

	} else if (_config.mode == MODE_RAMP) {
		PX4_INFO("????????? %.2f ?", (double)_config.ramp_time);
		
		hrt_abstime start_time = hrt_absolute_time();
		hrt_abstime current_time;
		float elapsed_time;
		float current_speed;

		while (!_thread_should_exit) {
			current_time = hrt_absolute_time();
			elapsed_time = (current_time - start_time) * 1e-6f; // ????

			if (elapsed_time >= _config.ramp_time) {
				/* ??????????? */
				current_speed = 1.0f;
				
				if (_config.motor_id == -1) {
					motor_set_all_speed(fd, max_channels, current_speed);
				} else {
					motor_set_speed(fd, max_channels, _config.motor_id, current_speed);
				}
				
				PX4_INFO("???????????");
				
				/* ??2???? */
				px4_usleep(2000000);
				break;
			}

			/* ???? */
			current_speed = elapsed_time / _config.ramp_time;
			
			if (_config.motor_id == -1) {
				motor_set_all_speed(fd, max_channels, current_speed);
			} else {
				motor_set_speed(fd, max_channels, _config.motor_id, current_speed);
			}

			/* ?????50Hz */
			px4_usleep(20000);
		}
	}

	/* ?????? */
	PX4_INFO("??????");
	motor_stop(fd, max_channels);

	/* ?? */
	motor_disarm(fd);

	/* ???? */
	px4_close(fd);

	_thread_running = false;
	return 0;
}

/**
 * ???
 */
int motor_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("????");
		return 1;
	}

	/* ???? */
	if (!strcmp(argv[1], "stop")) {
		if (!_thread_running) {
			PX4_INFO("motor_control ???");
			return 0;
		}

		_thread_should_exit = true;

		/* ?????? */
		for (int i = 0; i < 50 && _thread_running; i++) {
			px4_usleep(100000);
		}

		if (_thread_running) {
			PX4_WARN("????");
		}

		PX4_INFO("???");
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (_thread_running) {
			PX4_INFO("motor_control ????");
			PX4_INFO("  ??: %s", _config.mode == MODE_RUN ? "??" : "????");
			PX4_INFO("  ??: %s", _config.motor_id == -1 ? "??" : "??");
			if (_config.motor_id != -1) {
				PX4_INFO("  ????: %d", _config.motor_id);
			}
			if (_config.mode == MODE_RUN) {
				PX4_INFO("  ??: %.1f%%", (double)(_config.speed * 100.0f));
			} else {
				PX4_INFO("  ????: %.2f ?", (double)_config.ramp_time);
			}
		} else {
			PX4_INFO("motor_control ???");
		}
		return 0;
	}

	if (_thread_running) {
		PX4_WARN("motor_control ????");
		return 0;
	}

	/* ????? */
	memset(&_config, 0, sizeof(_config));
	_config.motor_id = -1; // ??????
	_config.speed = 0.5f;  // ??50%??
	_config.ramp_time = 5.0f; // ??5???

	/* ?????? */
	if (!strcmp(argv[1], "run")) {
		_config.mode = MODE_RUN;
		
		/* ???? */
		for (int i = 2; i < argc; i++) {
			if (!strcmp(argv[i], "--all") || !strcmp(argv[i], "-a")) {
				_config.motor_id = -1;
			} else if (!strcmp(argv[i], "--motor") || !strcmp(argv[i], "-m")) {
				if (i + 1 < argc) {
					_config.motor_id = atoi(argv[++i]);
				}
			} else if (!strcmp(argv[i], "--speed") || !strcmp(argv[i], "-s")) {
				if (i + 1 < argc) {
					_config.speed = atof(argv[++i]);
				}
			}
		}

	} else if (!strcmp(argv[1], "ramp")) {
		_config.mode = MODE_RAMP;
		
		/* ???? */
		for (int i = 2; i < argc; i++) {
			if (!strcmp(argv[i], "--all") || !strcmp(argv[i], "-a")) {
				_config.motor_id = -1;
			} else if (!strcmp(argv[i], "--motor") || !strcmp(argv[i], "-m")) {
				if (i + 1 < argc) {
					_config.motor_id = atoi(argv[++i]);
				}
			} else if (!strcmp(argv[i], "--time") || !strcmp(argv[i], "-t")) {
				if (i + 1 < argc) {
					_config.ramp_time = atof(argv[++i]);
				}
			}
		}

	} else {
		usage("????");
		return 1;
	}

	/* ???? */
	if (!validate_config(&_config)) {
		return 1;
	}

	/* ?????? */
	_thread_should_exit = false;
	_motor_control_task = px4_task_spawn_cmd(
		"motor_control",
		SCHED_DEFAULT,
		SCHED_PRIORITY_DEFAULT + 40,
		2048,
		motor_control_thread_main,
		nullptr
	);

	if (_motor_control_task < 0) {
		PX4_ERR("??????");
		return 1;
	}

	return 0;
}
