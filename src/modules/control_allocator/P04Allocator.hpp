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

// Port of AP_MotorsMatrix at 942f63eb72. Motor indices in this class are DJI_X.
#pragma once
#include <stdint.h>

class P04Allocator
{
public:
	static constexpr uint8_t MotorCount = 6;
	// Physical rows expressed in AP units. The caller converts the setpoint with the same row multipliers.
	void set_column(unsigned motor, float lift, float roll, float pitch, float yaw);
	void fail(unsigned motor, float last_thrust, uint32_t now_ms);
	bool allocate(const float request[4], float out[MotorCount], uint32_t now_ms, float dt, float yaw_rate, bool flying);
	float _null_coef[3] {};
	int _null_status{};
	bool _vendor_vertices{};
	void set_healthy_thrust(const float thrust[MotorCount]);

	int _fail_null_auto{};
	float _fail_null_max{0.75f}, _fail_spindown{}, _fail_null_shift{};
	float _fail_yaw_suppress{0.169697f}, _fail_yaw_track{1.f}, _fail_relax{10.f};
	int _fail_yaw_null{1}, _fail_yaw_int{1};
	enum class AllocResult : uint8_t {
		OK              = 0,
		TOO_FEW_MOTORS  = 1,   // fewer effectors than rows
		SINGULAR        = 2,   // B*B^T not invertible - rank deficient
		NON_FINITE      = 3,   // NaN/Inf came out of the solve
		RESIDUAL        = 4,   // B*t != rem beyond tolerance - ill conditioned
		NO_SOLUTION     = 5,   // every pass needed clamping, none verified
		NOT_RUN         = 6,   // allocator disabled or no failed motor
		INFEASIBLE      = 7,   // active set: Phase 1 found no feasible vertex
		ITER_LIMIT      = 8,   // active set: hit the iteration cap, point still feasible
		RELAXED         = 9,   // demand unreachable - nearest reachable point committed
	};

	// Which solver produced the committed thrusts.  Without this a degraded
	// flight cannot be told apart from one spending every cycle on the fallback:
	// both write MALC, and the demand/achieved pair looks the same.
	enum class AllocPath : uint8_t {
		NONE            = 0,   // not degraded, or forward mixing
		ACTIVE_SET      = 1,
		REDISTRIBUTED   = 2,   // the older clamp-only solver, as fallback
		RELAXED         = 3,   // demand outside the reachable set - nearest point
	};
	struct YawAuthority {
		bool net{};
		float weight{}, offset{};
		bool  valid;                // active set solved and the projection ran this cycle
		bool  exact;                // the committed point realises the target itself, not the 20/80 blend
		bool  forward_clipped;      // the request was already cut by the forward mixer's yaw_allowed
		bool  clipped_low;          // target clamped to the low end of the reachable interval
		bool  clipped_high;         // ... to the high end
		bool  integrator_released;  // limit.yaw was cleared this cycle (MOT_FAIL_YINT)
		float request_raw;          // before yaw_allowed, in the allocator's demand scale
		float request;              // what the allocator was given (demand[3])
		float target;               // request clamped to [lo, hi]
		float achieved;             // what the committed thrusts produce
		float lo, hi;               // reachable interval at this cycle's thrust, roll and pitch
	};

	const YawAuthority &yaw_authority() const { return _yaw_auth; }
	struct Limits { bool roll{}, pitch{}, yaw{}, throttle_lower{}, throttle_upper{}; } limit{};
	struct Prediction {
		float trim{}, balanced[MotorCount] {}, automatic[MotorCount] {};
		unsigned risk_mask{};
		bool from_model{};
	};
	void reset_prediction();
	void interrupt_coefficient_check() {_ncheck_bad_s = 0.f;}
	void reset_coefficient_check();
	void check_coefficients(const float thrust[MotorCount], float dt, bool steady);
	bool coefficients_stale() const {return _null_stale;}
	bool predict(const float thrust[MotorCount], float dt, Prediction &result);
	bool active() const {return _failed_motor >= 0;}
	AllocPath path() const {return _alloc_path;}
	float shift() const {return _nauto_out;}
	float achieved(unsigned row) const {return _alloc_achieved[row];}
private:
	bool fail_predict_gaps(const float rem[3], const uint8_t idx[5], const int8_t pair_of[],
			       float csum, float &balanced, float &automatic) const;
	float _healthy_pair_sum[3] {}, _spdn_resid{};
	float _predict_s{}, _predict_yaw{}, _predict_max{}, _predict_t[MotorCount] {};
	bool _predicted{};
	bool _null_stale{}, _ncheck_lp_init{};
	float _ncheck_bad_s{}, _ncheck_err_lp{}, _ncheck_pred_lp{};
	int _failed_motor{-1};
	bool motor_enabled[MotorCount] {};
	float _roll_factor[MotorCount] {}, _pitch_factor[MotorCount] {}, _yaw_factor[MotorCount] {};
	float _throttle_factor[MotorCount] {}, _yaw_geom[MotorCount] {};
	int8_t _pair_of[MotorCount] {0, 1, 2, 0, 1, 2};
	bool _pair_map_valid{true}, _flying{};
	mutable YawAuthority _yaw_auth{};
	uint32_t _now_ms{}, _spdn_start_ms{};
	float _dt_s{}, _body_yaw_rate{}, _spdn_t0{}, _alloc_achieved[4] {};
	mutable bool _as_warm{}, _alloc_phase1{}, _alloc_clamp_moved{}, _nauto_engaged{}, _nauto_cl_warned{};
	mutable uint16_t _as_fixed_mask{};
	mutable uint8_t _alloc_iters{}, _alloc_passes{}, _alloc_clamp_mask{};
	mutable uint32_t _nauto_clip_ms{};
	mutable int8_t _nauto_clip_side{};
	mutable float _as_thrust[MotorCount] {}, _nauto_clip_s{}, _nauto_slewed{}, _nauto_engaged_s{}, _nauto_cl{}, _nauto_out{}, _nauto_gain{},
		_nauto_ff{};
	mutable AllocResult _alloc_result{AllocResult::NOT_RUN};
	AllocPath _alloc_path{AllocPath::NONE};
	bool alloc_phase1_vertex(const float rem[3], const uint8_t idx[],
				 uint8_t n, float t_out[], bool fixed[]) const;
	bool allocate_nearest_reachable(const float demand[4],
					float thrust_out[MotorCount]) const;
	bool allocate_active_set(const float demand[4],
				 float thrust_out[MotorCount]) const;
	bool alloc_yaw_extremes(const float rem[3], const uint8_t idx[],
				uint8_t n, float *lo, float *hi,
				float t_lo[], float t_hi[], const float yaw_w[] = nullptr) const;
	bool yaw_null_direction(const uint8_t idx[], uint8_t n,
				float dir[MotorCount], const float yaw_w[] = nullptr) const;
	static bool null_line_range(const float t[], const float dir[], uint8_t n,
				    float &a_bal, float &amin, float &amax);
	static void balance_load_on_null_line(float t[], const float dir[], uint8_t n, float shift);
	float auto_null_shift(const float t[], const float dir[], const uint8_t idx[], uint8_t n,
			      float target, bool yaw_clipped) const;
	void apply_identified_yaw(const float demand[4], float thrust[]) const;
	void apply_yaw_projection_core(const float demand[4],
				       float thrust[MotorCount]) const;
	bool projection_keeps_rows(const float rem[3], const uint8_t idx[], uint8_t n,
				   const float before[MotorCount],
				   const float after[]) const;
	void apply_yaw_projection(const float demand[4],
				  float thrust[MotorCount]) const;
	bool allocate_redistributed(const float demand[4], bool include_yaw,
				    float thrust_out[MotorCount]) const;
	void set_limits_from_allocation(const float demand[4], bool include_yaw,
					const float thrust[MotorCount]);
};
