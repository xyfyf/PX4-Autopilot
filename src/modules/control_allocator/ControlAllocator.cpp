/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocator.cpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocator.hpp"
#include "VehicleActuatorEffectiveness/ActuatorEffectivenessMultirotor.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace time_literals;

ModuleBase::Descriptor ControlAllocator::desc{task_spawn, custom_command, print_usage};

ControlAllocator::ControlAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_control_allocator_status_pub[0].advertise();
	_control_allocator_status_pub[1].advertise();

	_actuator_motors_pub.advertise();
	_actuator_servos_pub.advertise();
	_actuator_servos_trim_pub.advertise();

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_R%u_SLEW", i);
		_param_handles.slew_rate_motors[i] = param_find(buffer);
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV%u_SLEW", i);
		_param_handles.slew_rate_servos[i] = param_find(buffer);
	}

	parameters_updated();
}

ControlAllocator::~ControlAllocator()
{
	for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
		delete _control_allocation[i];
	}

	delete _actuator_effectiveness;

	perf_free(_loop_perf);
}

bool
ControlAllocator::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	ScheduleDelayed(50_ms);
#endif

	return true;
}

void
ControlAllocator::parameters_updated()
{
	_has_slew_rate = false;

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		param_get(_param_handles.slew_rate_motors[i], &_params.slew_rate_motors[i]);
		_has_slew_rate |= _params.slew_rate_motors[i] > FLT_EPSILON;
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		param_get(_param_handles.slew_rate_servos[i], &_params.slew_rate_servos[i]);
		_has_slew_rate |= _params.slew_rate_servos[i] > FLT_EPSILON;
	}

	// Allocation method & effectiveness source
	// Do this first: in case a new method is loaded, it will be configured below
	bool updated = update_effectiveness_source();
	update_allocation_method(updated); // must be called after update_effectiveness_source()

	if (_num_control_allocation == 0) {
		return;
	}

	for (int i = 0; i < _num_control_allocation; ++i) {
		_control_allocation[i]->updateParameters();
	}

	update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::CONFIGURATION_UPDATE);
}

void
ControlAllocator::update_allocation_method(bool force)
{
	AllocationMethod configured_method = (AllocationMethod)_param_ca_method.get();

	if (!_actuator_effectiveness) {
		PX4_ERR("_actuator_effectiveness null");
		return;
	}

	if (_allocation_method_id != configured_method || force) {

		ActuatorVector actuator_sp[ActuatorEffectiveness::MAX_NUM_MATRICES];

		// Cleanup first
		for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
			// Save current state
			if (_control_allocation[i] != nullptr) {
				actuator_sp[i] = _control_allocation[i]->getActuatorSetpoint();
			}

			delete _control_allocation[i];
			_control_allocation[i] = nullptr;
		}

		_num_control_allocation = _actuator_effectiveness->numMatrices();

		AllocationMethod desired_methods[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getDesiredAllocationMethod(desired_methods);

		bool normalize_rpy[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getNormalizeRPY(normalize_rpy);

		for (int i = 0; i < _num_control_allocation; ++i) {
			AllocationMethod method = configured_method;

			if (configured_method == AllocationMethod::AUTO) {
				method = desired_methods[i];
			}

			switch (method) {
			case AllocationMethod::PSEUDO_INVERSE:
				_control_allocation[i] = new ControlAllocationPseudoInverse();
				break;

			case AllocationMethod::SEQUENTIAL_DESATURATION:
				_control_allocation[i] = new ControlAllocationSequentialDesaturation();
				break;

			default:
				PX4_ERR("Unknown allocation method");
				break;
			}

			if (_control_allocation[i] == nullptr) {
				PX4_ERR("alloc failed");
				_num_control_allocation = 0;

			} else {
				_control_allocation[i]->setNormalizeRPY(normalize_rpy[i]);
				_control_allocation[i]->setActuatorSetpoint(actuator_sp[i]);
			}
		}

		_allocation_method_id = configured_method;
	}
}

bool
ControlAllocator::update_effectiveness_source()
{
	const EffectivenessSource source = (EffectivenessSource)_param_ca_airframe.get();

	if (_effectiveness_source_id != source) {

		// try to instanciate new effectiveness source
		ActuatorEffectiveness *tmp = nullptr;

		switch (source) {
		case EffectivenessSource::NONE:
		case EffectivenessSource::MULTIROTOR:
			tmp = new ActuatorEffectivenessMultirotor(this);
			break;

		case EffectivenessSource::STANDARD_VTOL:
			tmp = new ActuatorEffectivenessStandardVTOL(this);
			break;

		case EffectivenessSource::TILTROTOR_VTOL:
			tmp = new ActuatorEffectivenessTiltrotorVTOL(this);
			break;

		case EffectivenessSource::TAILSITTER_VTOL:
			tmp = new ActuatorEffectivenessTailsitterVTOL(this);
			break;

		case EffectivenessSource::FIXED_WING:
			tmp = new ActuatorEffectivenessFixedWing(this);
			break;

		case EffectivenessSource::MOTORS_6DOF: // just a different UI from MULTIROTOR
			tmp = new ActuatorEffectivenessUUV(this);
			break;

		case EffectivenessSource::MULTIROTOR_WITH_TILT:
			tmp = new ActuatorEffectivenessMCTilt(this);
			break;

		case EffectivenessSource::CUSTOM:
			tmp = new ActuatorEffectivenessCustom(this);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_ESC:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::MOTORS);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_SERVO:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::SERVOS);
			break;

		case EffectivenessSource::HELICOPTER_COAXIAL:
			tmp = new ActuatorEffectivenessHelicopterCoaxial(this);
			break;

		case EffectivenessSource::SPACECRAFT_2D:
			tmp = new ActuatorEffectivenessSpacecraft(this);
			break;

		case EffectivenessSource::ROVER_ACKERMANN: // Unreachable: Rover startup scripts don't load control_allocator. Controllers publish actuator_outputs directly.
		case EffectivenessSource::ROVER_DIFFERENTIAL:
		case EffectivenessSource::ROVER_MECANUM:
		default:
			PX4_ERR("Unknown airframe");
			break;
		}

		// Replace previous source with new one
		if (tmp == nullptr) {
			// It did not work, forget about it
			PX4_ERR("Actuator effectiveness init failed");
			_param_ca_airframe.set((int)_effectiveness_source_id);

		} else {
			// Swap effectiveness sources
			delete _actuator_effectiveness;
			_actuator_effectiveness = tmp;

			// Save source id
			_effectiveness_source_id = source;
		}

		return true;
	}

	return false;
}

void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	// Push backup schedule
	ScheduleDelayed(50_ms);
#endif

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		if (_handled_motor_failure_bitmask == 0) {
			// We don't update the geometry after an actuator failure, as it could lead to unexpected results
			// (e.g. a user could add/remove motors, such that the bitmask isn't correct anymore)
			updateParams();
			parameters_updated();
		}
	}

	if (_num_control_allocation == 0 || _actuator_effectiveness == nullptr) {
		return;
	}

	{
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {

			_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
			_is_vtol = vehicle_status.is_vtol;

			ActuatorEffectiveness::FlightPhase flight_phase{ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT};

			// Check if the current flight phase is HOVER or FIXED_WING
			if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				flight_phase = ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT;

			} else {
				flight_phase = ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;
			}

			// Special cases for VTOL in transition
			if (vehicle_status.is_vtol && vehicle_status.in_transition_mode) {
				if (vehicle_status.in_transition_to_fw) {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_HF_TO_FF;

				} else {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_FF_TO_HF;
				}
			}

			// Forward to effectiveness source
			_actuator_effectiveness->setFlightPhase(flight_phase);
		}
	}

	{
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
			_publish_controls = vehicle_control_mode.flag_control_allocation_enabled;
		}
	}

	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

	_actuator_group_preflight_check.handleCommand(now, _effectiveness_source_id == EffectivenessSource::TILTROTOR_VTOL);
	_actuator_group_preflight_check.updateState(now);

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);

		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);

		_actuator_effectiveness->stopMotorsBasedOnThrustSetpoint(_thrust_sp);
		_thrust_sp.nanToZero();
	}

	if (do_update) {
		_last_run = now;

		check_for_motor_failures();

		update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::NO_EXTERNAL_UPDATE);

		// Set control setpoint vector(s)
		matrix::Vector<float, NUM_AXES> c[ActuatorEffectiveness::MAX_NUM_MATRICES];
		c[0](0) = _torque_sp(0);
		c[0](1) = _torque_sp(1);
		c[0](2) = _torque_sp(2);
		c[0](3) = _thrust_sp(0);
		c[0](4) = _thrust_sp(1);
		c[0](5) = _thrust_sp(2);

		if (_num_control_allocation > 1) {
			if (_vehicle_torque_setpoint1_sub.copy(&vehicle_torque_setpoint)) {
				c[1](0) = vehicle_torque_setpoint.xyz[0];
				c[1](1) = vehicle_torque_setpoint.xyz[1];
				c[1](2) = vehicle_torque_setpoint.xyz[2];
			}

			if (_vehicle_thrust_setpoint1_sub.copy(&vehicle_thrust_setpoint)) {
				c[1](3) = vehicle_thrust_setpoint.xyz[0];
				c[1](4) = vehicle_thrust_setpoint.xyz[1];
				c[1](5) = vehicle_thrust_setpoint.xyz[2];
			}
		}

		_actuator_group_preflight_check.applyOverrides(c, _is_vtol, *_actuator_effectiveness);

		for (int i = 0; i < _num_control_allocation; ++i) {

			_control_allocation[i]->setControlSetpoint(c[i]);

			// Do allocation
			if (i != 0 || !_p04.active() || !allocateP04(dt, now)) {
				_control_allocation[i]->allocate();
			}

			_actuator_effectiveness->allocateAuxilaryControls(dt, i, _control_allocation[i]->_actuator_sp); //flaps and spoilers
			_actuator_effectiveness->updateSetpoint(c[i], i, _control_allocation[i]->_actuator_sp,
								_control_allocation[i]->getActuatorMin(), _control_allocation[i]->getActuatorMax());

			if (i == 0) {
				// The motors are always in allocation 0
				if (_param_p04_seq.get() || _p04_seq_start) {applyP04Sequence(dt, now);}

				if (_param_ca_failure_mode.get() == int(FailureMode::P04) && !_p04.active() && _param_autostart.get() == 10046) {
					updateP04Prediction(dt);
				}

				handle_stopped_motors(now);
			}

			if (_has_slew_rate) {
				_control_allocation[i]->applySlewRateLimit(dt);
			}

			_control_allocation[i]->clipActuatorSetpoint();

			check_allocation_health(i);
		}
	}

	// Publish actuator setpoint and allocator status
	publish_actuator_controls();

	// Publish status at limited rate, as it's somewhat expensive and we use it for slower dynamics
	// (i.e. anti-integrator windup)
	if (now - _last_status_pub >= 5_ms) {
		publish_control_allocator_status(0);

		if (_num_control_allocation > 1) {
			publish_control_allocator_status(1);
		}

		_last_status_pub = now;
	}

	perf_end(_loop_perf);
}

void
ControlAllocator::update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason)
{
	ActuatorEffectiveness::Configuration config{};

	if (reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE
	    && hrt_elapsed_time(&_last_effectiveness_update) < 100_ms) { // rate-limit updates
		return;
	}

	if (_actuator_effectiveness->getEffectivenessMatrix(config, reason)) {
		_last_effectiveness_update = hrt_absolute_time();

		memcpy(_control_allocation_selection_indexes, config.matrix_selection_indexes,
		       sizeof(_control_allocation_selection_indexes));

		// Get the minimum and maximum depending on type and configuration
		ActuatorEffectiveness::ActuatorVector minimum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector maximum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector slew_rate[ActuatorEffectiveness::MAX_NUM_MATRICES];
		int actuator_idx = 0;
		int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

		actuator_servos_trim_s trims{};
		static_assert(actuator_servos_trim_s::NUM_CONTROLS == actuator_servos_s::NUM_CONTROLS, "size mismatch");

		for (int actuator_type = 0; actuator_type < (int)ActuatorType::COUNT; ++actuator_type) {
			_num_actuators[actuator_type] = config.num_actuators[actuator_type];

			for (int actuator_type_idx = 0; actuator_type_idx < config.num_actuators[actuator_type]; ++actuator_type_idx) {
				if (actuator_idx >= NUM_ACTUATORS) {
					_num_actuators[actuator_type] = 0;
					PX4_ERR("Too many actuators");
					break;
				}

				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if ((ActuatorType)actuator_type == ActuatorType::MOTORS) {
					if (actuator_type_idx >= MAX_NUM_MOTORS) {
						PX4_ERR("Too many motors");
						_num_actuators[actuator_type] = 0;
						break;
					}

					if (_runtime_reversible_bitmask & (1u << actuator_type_idx)) {
						// Failure recovery: a forward propeller driven in reverse makes only a fraction of its forward thrust
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -_param_ca_rev_thr_frac.get();

					} else if (_param_r_rev.get() & (1u << actuator_type_idx)) {
						// Statically reversible thruster (symmetric): full reverse authority
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;

					} else {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 0.f;
					}

					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_motors[actuator_type_idx];

				} else if ((ActuatorType)actuator_type == ActuatorType::SERVOS) {
					if (actuator_type_idx >= MAX_NUM_SERVOS) {
						PX4_ERR("Too many servos");
						_num_actuators[actuator_type] = 0;
						break;
					}

					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_servos[actuator_type_idx];
					trims.trim[actuator_type_idx] = config.trim[selected_matrix](actuator_idx_matrix[selected_matrix]);

				} else {
					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
				}

				maximum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 1.f;

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		// Recompute the opposite-motor lookup on any effectiveness update except motor-activation changes
		if (reason != EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE) {
			computeOppositeMotors();
		}

		// Handle failed actuators
		if (_handled_motor_failure_bitmask != 0) {
			actuator_idx = 0;
			memset(&actuator_idx_matrix, 0, sizeof(actuator_idx_matrix));

			for (int motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if (_handled_motor_failure_bitmask & (1 << motors_idx)) {
					config.effectiveness_matrices[selected_matrix].setCol(actuator_idx_matrix[selected_matrix], 0.f);
				}

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {
			_control_allocation[i]->setActuatorMin(minimum[i]);
			_control_allocation[i]->setActuatorMax(maximum[i]);
			_control_allocation[i]->setSlewRateLimit(slew_rate[i]);

			// Set all the elements of a row to 0 if that row has weak authority.
			// That ensures that the algorithm doesn't try to control axes with only marginal control authority,
			// which in turn would degrade the control of the main axes that actually should and can be controlled.

			ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[i];

			if (_param_ca_failure_mode.get() == int(FailureMode::P04) && _param_autostart.get() == 10046) {
				matrix.row(3) = 0.f;
				matrix.row(4) = 0.f;
			}

			for (int n = 0; n < NUM_AXES; n++) {
				bool all_entries_small = true;

				for (int m = 0; m < config.num_actuators_matrix[i]; m++) {
					if (fabsf(matrix(n, m)) > 0.05f) {
						all_entries_small = false;
					}
				}

				if (all_entries_small) {
					matrix.row(n) = 0.f;
				}
			}

			// Assign control effectiveness matrix
			int total_num_actuators = config.num_actuators_matrix[i];
			_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i],
					config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
		}

		trims.timestamp = hrt_absolute_time();
		_actuator_servos_trim_pub.publish(trims);
	}
}


void
ControlAllocator::handle_stopped_motors(const hrt_abstime now)
{
	const ActuatorBitmask stopped_motors_due_to_effectiveness = _actuator_effectiveness->getStoppedMotors();

	const ActuatorBitmask stopped_motors = stopped_motors_due_to_effectiveness
					       | _handled_motor_failure_bitmask
					       | _motor_stop_mask;

	// Handle stopped motors by setting NaN
	const unsigned int allocation_index = 0;  // Motors always in allocation 0
	_control_allocation[allocation_index]->applyNanToActuators(stopped_motors);

	// Apply ice shedding, which applies _only_ to stopped motors
	const bool any_stopped_motor_failed = 0 != (stopped_motors_due_to_effectiveness & (_handled_motor_failure_bitmask | _motor_stop_mask));
	const float ice_shedding_output = get_ice_shedding_output(now);

	if (ice_shedding_output > FLT_EPSILON && !any_stopped_motor_failed) {
		for (int motors_idx = 0; motors_idx < _num_actuators[allocation_index] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
			if (stopped_motors & 1u << motors_idx) {
				_control_allocation[allocation_index]->_actuator_sp(motors_idx) = ice_shedding_output;
			}
		}
	}
}

void
ControlAllocator::check_allocation_health(int matrix_index)
{
	const uint8_t dropped_axes = _control_allocation[matrix_index]->getDroppedAxes();

	if (dropped_axes != _dropped_axes_reported[matrix_index]) {
		_dropped_axes_reported[matrix_index] = dropped_axes;

		if (dropped_axes != 0) {
			PX4_WARN("Control allocation %i: dropped dependent control axes 0x%x", matrix_index, dropped_axes);

		} else {
			PX4_INFO("Control allocation %i: all control axes restored", matrix_index);
		}
	}
}

void
ControlAllocator::publish_control_allocator_status(int matrix_index)
{
	control_allocator_status_s control_allocator_status{};
	control_allocator_status.timestamp = hrt_absolute_time();
	control_allocator_status.actuator_group_preflight_check_active = _actuator_group_preflight_check.isActive();

	// TODO: disabled motors (?)

	// Allocated control
	const matrix::Vector<float, NUM_AXES> &allocated_control = _control_allocation[matrix_index]->getAllocatedControl();

	// Unallocated control
	const matrix::Vector<float, NUM_AXES> unallocated_control = _control_allocation[matrix_index]->getControlSetpoint() -
			allocated_control;
	control_allocator_status.unallocated_torque[0] = unallocated_control(0);
	control_allocator_status.unallocated_torque[1] = unallocated_control(1);
	control_allocator_status.unallocated_torque[2] = unallocated_control(2);
	control_allocator_status.unallocated_thrust[0] = unallocated_control(3);
	control_allocator_status.unallocated_thrust[1] = unallocated_control(4);
	control_allocator_status.unallocated_thrust[2] = unallocated_control(5);

	// override control_allocator_status in customized saturation logic for certain effectiveness types
	_actuator_effectiveness->getUnallocatedControl(matrix_index, control_allocator_status);

	// Allocation success flags
	control_allocator_status.torque_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_torque[0],
			control_allocator_status.unallocated_torque[1],
			control_allocator_status.unallocated_torque[2]).norm_squared() < 1e-6f);
	control_allocator_status.thrust_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_thrust[0],
			control_allocator_status.unallocated_thrust[1],
			control_allocator_status.unallocated_thrust[2]).norm_squared() < 1e-6f);

	// Actuator saturation
	const ActuatorVector &actuator_sp = _control_allocation[matrix_index]->getActuatorSetpoint();
	const ActuatorVector &actuator_min = _control_allocation[matrix_index]->getActuatorMin();
	const ActuatorVector &actuator_max = _control_allocation[matrix_index]->getActuatorMax();

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		if (actuator_sp(i) > (actuator_max(i) - FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;

		} else if (actuator_sp(i) < (actuator_min(i) + FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_LOWER;
		}
	}

	// Handled motor failures
	control_allocator_status.handled_motor_failure_mask = _handled_motor_failure_bitmask;
	control_allocator_status.motor_stop_mask = _motor_stop_mask;

	_control_allocator_status_pub[matrix_index].publish(control_allocator_status);
}

float
ControlAllocator::get_ice_shedding_output(hrt_abstime now)
{
	const float period_sec = _param_ice_shedding_period.get();

	const bool feature_disabled_by_param = period_sec <= FLT_EPSILON;
	const bool in_forward_flight = _actuator_effectiveness->getFlightPhase() == ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;

	// If any stopped motor has failed, the feature will create much more
	// torque than in the nominal case, and becomes pointless anyway as we
	// cannot go back to multicopter
	const bool apply_shedding = _is_vtol && in_forward_flight;

	if (feature_disabled_by_param || !apply_shedding) {
		return 0.0f;

	} else {
		// Square wave output
		const float elapsed_in_period = fmodf(static_cast<float>(now) * 1e-6f, period_sec);
		const float ice_shedding_output = elapsed_in_period < ICE_SHEDDING_ON_SEC ? ICE_SHEDDING_OUTPUT : 0.0f;

		return ice_shedding_output;
	}
}

void
ControlAllocator::publish_actuator_controls()
{
	if (!_publish_controls) {
		return;
	}

	actuator_motors_s actuator_motors;
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _timestamp_sample;

	actuator_servos_s actuator_servos;
	actuator_servos.timestamp = actuator_motors.timestamp;
	actuator_servos.timestamp_sample = _timestamp_sample;

	actuator_motors.reversible_flags = _param_r_rev.get() | _runtime_reversible_bitmask;

	int actuator_idx = 0;
	int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

	// motors
	int motors_idx;

	for (motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
		int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
		float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);

		// Recovery motor was solved with min = -CA_REV_THR_FRAC; re-expand to [-1, 0] (static CA_R_REV uses -1, no rescale).
		if ((_runtime_reversible_bitmask & (1u << motors_idx)) && PX4_ISFINITE(actuator_sp) && actuator_sp < 0.f) {
			actuator_sp /= _param_ca_rev_thr_frac.get();
		}

		actuator_motors.control[motors_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
		++actuator_idx_matrix[selected_matrix];
		++actuator_idx;
	}

	for (int i = motors_idx; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	_actuator_motors_pub.publish(actuator_motors);

	// servos
	if (_num_actuators[1] > 0) {
		int servos_idx;

		for (servos_idx = 0; servos_idx < _num_actuators[1] && servos_idx < actuator_servos_s::NUM_CONTROLS; servos_idx++) {
			int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
			float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
			actuator_servos.control[servos_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
			++actuator_idx_matrix[selected_matrix];
			++actuator_idx;
		}

		for (int i = servos_idx; i < actuator_servos_s::NUM_CONTROLS; i++) {
			actuator_servos.control[i] = NAN;
		}

		_actuator_servos_pub.publish(actuator_servos);
	}
}

void
ControlAllocator::computeOppositeMotors()
{
	memset(_opposite_motor, -1, sizeof(_opposite_motor));

	// Only supported for hexarotor multirotors
	if ((_effectiveness_source_id != EffectivenessSource::MULTIROTOR) ||
	    _num_actuators[(int)ActuatorType::MOTORS] != 6) {
		return;
	}

	// NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
	const auto &geo = static_cast<ActuatorEffectivenessMultirotor *>(_actuator_effectiveness)->geometry();

	for (int i = 0; i < geo.num_rotors; i++) {
		float min_dist_sq = FLT_MAX;

		for (int8_t j = 0; j < geo.num_rotors; j++) {
			if (geo.rotors[i].moment_ratio * geo.rotors[j].moment_ratio >= 0.f) { continue; }

			const float dist_sq = (Vector2f(geo.rotors[i].position) + Vector2f(geo.rotors[j].position)).norm_squared();

			if (dist_sq < min_dist_sq) {
				min_dist_sq = dist_sq;
				_opposite_motor[i] = j;
			}
		}
	}
}

// SIH 10046 PX4 rotor indices in DJI M1..M6 order (zero based).
static constexpr unsigned p04_rotor[6] = {0, 5, 4, 3, 2, 1};

void ControlAllocator::finishP04Sequence()
{
	float c[3] {};
	const auto result = _p04_identification.fit(c);

	if (result != P04Identification::Result::Rejected) {
		_param_p04_null_c1.set(c[0]); _param_p04_null_c1.commit();
		_param_p04_null_c2.set(c[1]); _param_p04_null_c2.commit();
		_param_p04_null_c3.set(c[2]); _param_p04_null_c3.commit();
		_p04.reset_coefficient_check();
	}

	debug_array_s trace{};
	trace.id = 607;
	trace.timestamp = hrt_absolute_time();
	strncpy(trace.name, "p04_fit", sizeof(trace.name) - 1);
	trace.data[0] = float(result);

	for (unsigned p = 0; p < 3; p++) {trace.data[p + 1] = c[p];}

	_p04_debug_pub.publish(trace);
	PX4_INFO("P04 fit %u: %.6f %.6f %.6f", unsigned(result), double(c[0]), double(c[1]), double(c[2]));
}

void ControlAllocator::applyP04Sequence(float dt, hrt_abstime now)
{
	_p04_land_sub.update(&_p04_land);

	if (!_param_p04_seq.get() || !_armed || _p04_land.landed || _p04.active()
	    || _param_autostart.get() != 10046 || _param_ca_failure_mode.get() != int(FailureMode::P04)
	    || _num_actuators[0] != 6 || (_p04_seq_start && now - _p04_seq_start >= 120_s)) {
		if (_param_p04_seq.get() && _p04_seq_start && now - _p04_seq_start >= 120_s && _armed && !_p04.active() && !_p04_land.landed) {
			finishP04Sequence();
		}

		_p04_seq_start = 0;
		_p04_delta = 0.f;

		if (_param_p04_seq.get()) {_param_p04_seq.set(false); _param_p04_seq.commit();}

		return;
	}

	if (!_p04_seq_start) {_p04_seq_start = now; _p04_identification = {};}

	const unsigned stage = (now - _p04_seq_start) / 15_s;
	constexpr unsigned pairs[8] = {0, 0, 0, 1, 1, 2, 2, 0};
	constexpr float amplitude[8] = {0, 0.08f, -0.08f, 0.08f, -0.08f, 0.08f, -0.08f, 0};
	float target = amplitude[stage];

	if (pairs[stage] != _p04_pair) {
		if (fabsf(_p04_delta) < FLT_EPSILON) {_p04_pair = pairs[stage];}

		else {target = 0.f;}
	}

	_p04_delta += math::constrain(target - _p04_delta, -0.2f * dt, 0.2f * dt);
	constexpr unsigned pair_up[3] = {0, 0, 1}, pair_down[3] = {1, 2, 2};
	auto &out = _control_allocation[0]->_actuator_sp;
	float delta = _p04_delta;

	for (unsigned j = 0; j < 6; j += 3) {
		const float up = out(p04_rotor[pair_up[_p04_pair] + j]), down = out(p04_rotor[pair_down[_p04_pair] + j]);
		delta = math::constrain(delta, -math::min(up, 1.f - down), math::min(1.f - up, down));
	}

	for (unsigned j = 0; j < 6; j += 3) {
		out(p04_rotor[pair_up[_p04_pair] + j]) += delta;
		out(p04_rotor[pair_down[_p04_pair] + j]) -= delta;
	}

	_p04_rate_sub.update(&_p04_rates);
	const bool steady = fabsf(_p04_rates.xyz[0]) < math::radians(10.f) && fabsf(_p04_rates.xyz[1]) < math::radians(10.f)
			    && fabsf(_p04_rates.xyz[2]) < math::radians(5.f);

	if ((now - _p04_seq_start) % 15_s >= 8_s && steady) {
		const auto &b = _control_allocation[0]->getEffectivenessMatrix();
		float yaw = 0.f, peak = 0.f, pairs_total[3] {};
		bool limited = false;

		for (unsigned i = 0; i < 6; i++) {
			const unsigned j = p04_rotor[i];
			const float thrust = out(j);
			limited |= !(thrust > 0.f && thrust < 1.f);
			pairs_total[i % 3] += thrust;
			yaw += b(2, j) * thrust;
			peak = math::max(peak, fabsf(b(2, j)));
		}

		if (!limited && peak > FLT_EPSILON) {
			_p04_identification.accumulate(stage, dt, pairs_total, 0.5f * yaw / peak, delta);
		}
	}

	if (now - _p04_debug_time >= 20_ms) {
		debug_array_s trace{};
		trace.timestamp = now; trace.id = 604;
		strncpy(trace.name, "p04_seq", sizeof(trace.name) - 1);
		trace.data[0] = float(stage); trace.data[1] = float(now - _p04_seq_start) * 1e-6f;
		trace.data[2] = delta;

		for (unsigned i = 0; i < 6; i++) {trace.data[3 + i] = out(p04_rotor[i]);}

		_p04_debug_pub.publish(trace);
		_p04_debug_time = now;
	}
}

bool ControlAllocator::configureP04(unsigned failed)
{
	if (_param_autostart.get() != 10046 || _effectiveness_source_id != EffectivenessSource::MULTIROTOR
	    || _num_actuators[0] != 6 || _num_control_allocation != 1 || failed > 6) {
		return false;
	}

	const auto &allocation = *_control_allocation[0];
	const auto &b = allocation.getEffectivenessMatrix();
	const auto &scale = allocation.getControlAllocationScale();
	const int row[4] = {5, 0, 1, 2};
	float units[4] {};

	for (int k = 0; k < 4; k++) {
		float peak = 0.f;

		for (unsigned j = 0; j < 6; j++) {
			if (j != failed || k == 3) {peak = math::max(peak, fabsf(b(row[k], j)));}
		}

		if (!(peak > FLT_EPSILON && scale(row[k]) > FLT_EPSILON)) {return false;}

		units[k] = (k == 0 ? -1.f : 0.5f) / peak;
		_p04_units[k] = units[k] / scale(row[k]);
	}

	unsigned dji = 0;

	for (unsigned i = 0; i < 6; i++) {
		const unsigned j = p04_rotor[i];
		_p04.set_column(i, units[0]*b(5, j), units[1]*b(0, j), units[2]*b(1, j), units[3]*b(2, j));

		if (j == failed) {dji = i;}
	}

	_p04._fail_null_auto = _param_p04_naut.get();
	_p04._fail_null_max = _param_p04_nmax.get();
	_p04._fail_spindown = _param_p04_spdn.get();
	_p04._null_coef[0] = _param_p04_null_c1.get();
	_p04._null_coef[1] = _param_p04_null_c2.get();
	_p04._null_coef[2] = _param_p04_null_c3.get();

	if (failed < 6) {_p04.fail(dji, allocation.getActuatorSetpoint()(failed), uint32_t(hrt_absolute_time() / 1000));}

	return true;
}

void ControlAllocator::updateP04Prediction(float dt)
{
	if (!_armed) {
		_p04.reset_prediction();
		_p04.interrupt_coefficient_check();
		return;
	}

	_p04_land_sub.update(&_p04_land);
	_p04_rate_sub.update(&_p04_rates);

	if (!configureP04(6)) { return; }

	float thrust[6];

	for (unsigned i = 0; i < 6; i++) {thrust[i] = _control_allocation[0]->getActuatorSetpoint()(p04_rotor[i]);}

	const bool steady = !_p04_land.landed && !_param_p04_seq.get() && fabsf(_p04_rates.xyz[2]) < math::radians(5.f)
			    && _thrust_sp(2) < -FLT_EPSILON && _thrust_sp(2) > -1.f;
	_p04.check_coefficients(thrust, dt, steady);

	if (!steady) {return;}

	P04Allocator::Prediction prediction{};

	if (_p04.predict(thrust, dt, prediction)) {
		debug_array_s trace{};
		trace.timestamp = hrt_absolute_time();
		trace.id = 605;
		strncpy(trace.name, "p04_mfpr", sizeof(trace.name) - 1);
		trace.data[0] = prediction.trim; trace.data[1] = prediction.from_model; trace.data[2] = prediction.risk_mask;

		for (unsigned i = 0; i < 6; i++) {trace.data[3 + i] = prediction.balanced[i]; trace.data[9 + i] = prediction.automatic[i];}

		_p04_debug_pub.publish(trace);
		PX4_INFO("P04 prediction source %u risk mask 0x%x", unsigned(prediction.from_model), prediction.risk_mask);
	}
}

bool ControlAllocator::allocateP04(float dt, hrt_abstime now)
{
	_p04_rate_sub.update(&_p04_rates);
	_p04_land_sub.update(&_p04_land);
	const float request[4] = {_p04_units[0] *_thrust_sp(2) / 5.f, _p04_units[1] *_torque_sp(0),
				  _p04_units[2] *_torque_sp(1), _p04_units[3] *_torque_sp(2)
				 };
	float out[6] {};

	if (!_p04.allocate(request, out, uint32_t(now / 1000), dt, _p04_rates.xyz[2], _armed && !_p04_land.landed)) {return false;}

	for (unsigned i = 0; i < 6; i++) {_control_allocation[0]->_actuator_sp(p04_rotor[i]) = out[i];}

	if (now - _p04_debug_time >= 100_ms) {
		debug_array_s trace{};
		trace.timestamp = now;
		trace.id = 606;
		strncpy(trace.name, "p04_alloc", sizeof(trace.name) - 1);
		trace.data[0] = float(_p04.path()); trace.data[1] = _p04.shift();
		trace.data[10] = _p04.limit.roll;
		trace.data[11] = _p04.limit.pitch;
		trace.data[12] = _p04.limit.yaw;
		trace.data[13] = _p04.limit.throttle_lower;
		trace.data[14] = _p04.limit.throttle_upper;
		trace.data[15] = _p04.coefficients_stale();

		for (unsigned i = 0; i < 4; i++) {trace.data[2 + i] = request[i]; trace.data[6 + i] = _p04.achieved(i);}

		_p04_debug_pub.publish(trace);
		_p04_debug_time = now;
	}

	return true;
}

void
ControlAllocator::check_for_motor_failures()
{
	failure_detector_status_s failure_detector_status;
	const FailureMode failure_mode = static_cast<FailureMode>(_param_ca_failure_mode.get());

	if (failure_mode == FailureMode::P04) {
		failure_injection_s prior{};

		if (_armed && !_p04.active() && _prior_sub.update(&prior)) {
			uint16_t mask = 0;

			for (unsigned k = 0; k < prior.count && k < prior.MAX_FAILURES; k++) {
				if (prior.unit[k] == prior.FAILURE_UNIT_SYSTEM_MOTOR && prior.failure_type[k] == prior.FAILURE_TYPE_OFF) {
					mask |= prior.instance_mask[k];
				}
			}

			if (math::countSetBits(mask) == 1 && mask < (1u << 6) && configureP04(math::countTrailingZeros(mask))) {
				_handled_motor_failure_bitmask = mask;
				_control_allocation[0]->setHadActuatorFailure(true);
				update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
				PX4_INFO("P04 prior mask 0x%x", mask);
			}
		}

		return;
	}

	if ((failure_mode == FailureMode::REMOVE_AND_STOP_OPPOSITE || failure_mode == FailureMode::REMOVE_AND_REVERSE_OPPOSITE)
	    && _failure_detector_status_sub.update(&failure_detector_status)) {

		if (_motor_stop_mask != failure_detector_status.motor_stop_mask) {
			_motor_stop_mask = failure_detector_status.motor_stop_mask;
			PX4_WARN("Stopping motors (%d)", _motor_stop_mask);
		}

		if (failure_detector_status.fd_motor) {
			const int num_motors_failed = math::countSetBits(failure_detector_status.motor_failure_mask);

			if (_handled_motor_failure_bitmask == 0 && num_motors_failed == 1) {
				_handled_motor_failure_bitmask = failure_detector_status.motor_failure_mask;

				const int failed = math::countTrailingZeros(_handled_motor_failure_bitmask);
				const int opposite = _opposite_motor[failed];
				const uint16_t opposite_mask = (opposite >= 0) ? (1u << opposite) : 0u;

				// Non-hex frames have no opposite (mask 0), so both modes reduce to dropping the failed motor.
				switch (failure_mode) {
				case FailureMode::REMOVE_AND_STOP_OPPOSITE:
					_handled_motor_failure_bitmask |= opposite_mask;
					break;

				case FailureMode::REMOVE_AND_REVERSE_OPPOSITE:
					_runtime_reversible_bitmask = opposite_mask;
					break;

				default:
					break;
				}

				PX4_WARN("Handling motor %d failure (mode %d, mask 0x%x)", failed,
					 (int)failure_mode, _handled_motor_failure_bitmask);

				for (int i = 0; i < _num_control_allocation; ++i) {
					_control_allocation[i]->setHadActuatorFailure(true);
				}

				update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
			}

		} else if (_handled_motor_failure_bitmask != 0) {
			PX4_INFO("Restoring all motors");
			_handled_motor_failure_bitmask = 0;
			_runtime_reversible_bitmask = 0;

			for (int i = 0; i < _num_control_allocation; ++i) {
				_control_allocation[i]->setHadActuatorFailure(false);
			}

			update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
		}
	}
}

int ControlAllocator::task_spawn(int argc, char *argv[])
{
	ControlAllocator *instance = new ControlAllocator();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int ControlAllocator::print_status()
{
	PX4_INFO("Running");

	// Print current allocation method
	switch (_allocation_method_id) {
	case AllocationMethod::NONE:
		PX4_INFO("Method: None");
		break;

	case AllocationMethod::PSEUDO_INVERSE:
		PX4_INFO("Method: Pseudo-inverse");
		break;

	case AllocationMethod::SEQUENTIAL_DESATURATION:
		PX4_INFO("Method: Sequential desaturation");
		break;

	case AllocationMethod::AUTO:
		PX4_INFO("Method: Auto");
		break;
	}

	// Print current airframe
	if (_actuator_effectiveness != nullptr) {
		PX4_INFO("Effectiveness Source: %s", _actuator_effectiveness->name());
	}

	// Print current effectiveness matrix
	for (int i = 0; i < _num_control_allocation; ++i) {
		const ActuatorEffectiveness::EffectivenessMatrix &effectiveness = _control_allocation[i]->getEffectivenessMatrix();

		if (_num_control_allocation > 1) {
			PX4_INFO("Instance: %i", i);
		}

		PX4_INFO("  Effectiveness =");
		int num_configured = _control_allocation[i]->numConfiguredActuators();

		// print column numbering
		if (num_configured > 1) {
			printf("  ");

			for (int col = 0; col < num_configured; col++) {
				printf("|%2u      ", col);
			}

			printf("\n");
		}

		// Print effectiveness matrix with row labels
		const char *row_labels[] = {"Mx", "My", "Mz", "Fx", "Fy", "Fz"};

		for (int row = 0; row < 6; row++) {
			printf("%2s|", row_labels[row]);

			for (int col = 0; col < num_configured; col++) {
				double d = static_cast<double>(effectiveness(row, col));

				// avoid -0.0 for display
				if (fabs(d - 0.0) < 1e-9) {
					// print fixed width zero
					printf(" 0       ");

				} else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
					printf("% .1e ", d);

				} else {
					printf("% 6.5f ", d);
				}
			}

			printf("\n");
		}

		PX4_INFO("  minimum =");

		// print column numbering
		if (num_configured > 1) {
			printf("  ");

			for (int col = 0; col < num_configured; col++) {
				printf("|%2u      ", col);
			}

			printf("\n");
		}

		printf("  |");

		for (int col = 0; col < num_configured; col++) {
			double d = static_cast<double>(_control_allocation[i]->getActuatorMin()(col));

			// avoid -0.0 for display
			if (fabs(d - 0.0) < 1e-9) {
				// print fixed width zero
				printf(" 0       ");

			} else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
				printf("% .1e ", d);

			} else {
				printf("% 6.5f ", d);
			}
		}

		printf("\n");
		PX4_INFO("  maximum =");

		// print column numbering
		if (num_configured > 1) {
			printf("  ");

			for (int col = 0; col < num_configured; col++) {
				printf("|%2u      ", col);
			}

			printf("\n");
		}

		printf("  |");

		for (int col = 0; col < num_configured; col++) {
			double d = static_cast<double>(_control_allocation[i]->getActuatorMax()(col));

			// avoid -0.0 for display
			if (fabs(d - 0.0) < 1e-9) {
				// print fixed width zero
				printf(" 0       ");

			} else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
				printf("% .1e ", d);

			} else {
				printf("% 6.5f ", d);
			}
		}

		printf("\n");
		PX4_INFO("  Configured actuators: %i", num_configured);

		if (_control_allocation[i]->getDroppedAxes() != 0) {
			PX4_INFO("  Dropped control axes: 0x%x", _control_allocation[i]->getDroppedAxes());
		}
	}

	if (_handled_motor_failure_bitmask) {
		PX4_INFO("Failed motors: %i (0x%x)", math::countSetBits(_handled_motor_failure_bitmask),
			 _handled_motor_failure_bitmask);
	}

	// Print perf
	perf_print_counter(_loop_perf);

	return 0;
}

int ControlAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ControlAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("control_allocator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Control Allocator app start / stop handling function
 */
extern "C" __EXPORT int control_allocator_main(int argc, char *argv[]);

int control_allocator_main(int argc, char *argv[])
{
	return ModuleBase::main(ControlAllocator::desc, argc, argv);
}
