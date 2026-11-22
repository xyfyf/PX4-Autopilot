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
#include "P04Allocator.hpp"
#include <float.h>
#include <math.h>
#include <string.h>

namespace
{
inline bool is_positive(float x) { return x >= FLT_EPSILON; }
inline bool is_negative(float x) { return x <= -FLT_EPSILON; }
inline bool is_zero(float x) { return fabsf(x) < FLT_EPSILON; }
inline float sq(float x) { return x * x; }
inline float constrain_float(float x, float lo, float hi)
{
	if (isnan(x)) {return (lo + hi) * 0.5f;}

	return x < lo ? lo : (x > hi ? hi : x);
}
template<class T> inline T MAX(T a, T b) {return a > b ? a : b;}
template<class T> inline T MIN(T a, T b) {return a < b ? a : b;}
template<typename T>
static bool inverse3x3(const T m[], T invOut[])
{
	T inv[9];
	// computes the inverse of a matrix m
	T  det = m[0] * (m[4] * m[8] - m[7] * m[5]) -
		 m[1] * (m[3] * m[8] - m[5] * m[6]) +
		 m[2] * (m[3] * m[7] - m[4] * m[6]);

	if (is_zero(det) || isinf(det)) {
		return false;
	}

	T invdet = 1 / det;

	inv[0] = (m[4] * m[8] - m[7] * m[5]) * invdet;
	inv[1] = (m[2] * m[7] - m[1] * m[8]) * invdet;
	inv[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
	inv[3] = (m[5] * m[6] - m[3] * m[8]) * invdet;
	inv[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
	inv[5] = (m[3] * m[2] - m[0] * m[5]) * invdet;
	inv[6] = (m[3] * m[7] - m[6] * m[4]) * invdet;
	inv[7] = (m[6] * m[1] - m[0] * m[7]) * invdet;
	inv[8] = (m[0] * m[4] - m[3] * m[1]) * invdet;

	for (uint16_t i = 0; i < 9; i++) {
		invOut[i] = inv[i];
	}

	return true;
}

/*
 *    fast matrix inverse code only for 4x4 square matrix copied from
 *    gluInvertMatrix implementation in opengl for 4x4 matrices.
 *
 *    @param     m,           input 4x4 matrix
 *    @param     invOut,      Output inverted 4x4 matrix
 *    @returns                false = matrix is Singular, true = matrix inversion successful
 */
template<typename T>
static bool inverse4x4(const T m[], T invOut[])
{
	T inv[16], det;
	uint16_t i;



	inv[0] = m[5]  * m[10] * m[15] -
		 m[5]  * m[11] * m[14] -
		 m[9]  * m[6]  * m[15] +
		 m[9]  * m[7]  * m[14] +
		 m[13] * m[6]  * m[11] -
		 m[13] * m[7]  * m[10];

	inv[4] = -m[4]  * m[10] * m[15] +
		 m[4]  * m[11] * m[14] +
		 m[8]  * m[6]  * m[15] -
		 m[8]  * m[7]  * m[14] -
		 m[12] * m[6]  * m[11] +
		 m[12] * m[7]  * m[10];

	inv[8] = m[4]  * m[9] * m[15] -
		 m[4]  * m[11] * m[13] -
		 m[8]  * m[5] * m[15] +
		 m[8]  * m[7] * m[13] +
		 m[12] * m[5] * m[11] -
		 m[12] * m[7] * m[9];

	inv[12] = -m[4]  * m[9] * m[14] +
		  m[4]  * m[10] * m[13] +
		  m[8]  * m[5] * m[14] -
		  m[8]  * m[6] * m[13] -
		  m[12] * m[5] * m[10] +
		  m[12] * m[6] * m[9];

	inv[1] = -m[1]  * m[10] * m[15] +
		 m[1]  * m[11] * m[14] +
		 m[9]  * m[2] * m[15] -
		 m[9]  * m[3] * m[14] -
		 m[13] * m[2] * m[11] +
		 m[13] * m[3] * m[10];

	inv[5] = m[0]  * m[10] * m[15] -
		 m[0]  * m[11] * m[14] -
		 m[8]  * m[2] * m[15] +
		 m[8]  * m[3] * m[14] +
		 m[12] * m[2] * m[11] -
		 m[12] * m[3] * m[10];

	inv[9] = -m[0]  * m[9] * m[15] +
		 m[0]  * m[11] * m[13] +
		 m[8]  * m[1] * m[15] -
		 m[8]  * m[3] * m[13] -
		 m[12] * m[1] * m[11] +
		 m[12] * m[3] * m[9];

	inv[13] = m[0]  * m[9] * m[14] -
		  m[0]  * m[10] * m[13] -
		  m[8]  * m[1] * m[14] +
		  m[8]  * m[2] * m[13] +
		  m[12] * m[1] * m[10] -
		  m[12] * m[2] * m[9];

	inv[2] = m[1]  * m[6] * m[15] -
		 m[1]  * m[7] * m[14] -
		 m[5]  * m[2] * m[15] +
		 m[5]  * m[3] * m[14] +
		 m[13] * m[2] * m[7] -
		 m[13] * m[3] * m[6];

	inv[6] = -m[0]  * m[6] * m[15] +
		 m[0]  * m[7] * m[14] +
		 m[4]  * m[2] * m[15] -
		 m[4]  * m[3] * m[14] -
		 m[12] * m[2] * m[7] +
		 m[12] * m[3] * m[6];

	inv[10] = m[0]  * m[5] * m[15] -
		  m[0]  * m[7] * m[13] -
		  m[4]  * m[1] * m[15] +
		  m[4]  * m[3] * m[13] +
		  m[12] * m[1] * m[7] -
		  m[12] * m[3] * m[5];

	inv[14] = -m[0]  * m[5] * m[14] +
		  m[0]  * m[6] * m[13] +
		  m[4]  * m[1] * m[14] -
		  m[4]  * m[2] * m[13] -
		  m[12] * m[1] * m[6] +
		  m[12] * m[2] * m[5];

	inv[3] = -m[1] * m[6] * m[11] +
		 m[1] * m[7] * m[10] +
		 m[5] * m[2] * m[11] -
		 m[5] * m[3] * m[10] -
		 m[9] * m[2] * m[7] +
		 m[9] * m[3] * m[6];

	inv[7] = m[0] * m[6] * m[11] -
		 m[0] * m[7] * m[10] -
		 m[4] * m[2] * m[11] +
		 m[4] * m[3] * m[10] +
		 m[8] * m[2] * m[7] -
		 m[8] * m[3] * m[6];

	inv[11] = -m[0] * m[5] * m[11] +
		  m[0] * m[7] * m[9] +
		  m[4] * m[1] * m[11] -
		  m[4] * m[3] * m[9] -
		  m[8] * m[1] * m[7] +
		  m[8] * m[3] * m[5];

	inv[15] = m[0] * m[5] * m[10] -
		  m[0] * m[6] * m[9] -
		  m[4] * m[1] * m[10] +
		  m[4] * m[2] * m[9] +
		  m[8] * m[1] * m[6] -
		  m[8] * m[2] * m[5];

	det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

	if (is_zero(det) || isinf(det)) {
		return false;
	}

	det = 1.0f / det;

	for (i = 0; i < 16; i++) {
		invOut[i] = inv[i] * det;
	}



	return true;
}


bool mat_inverse(const float *a, float *b, unsigned n) {return n == 3 ? inverse3x3(a, b) : n == 4 && inverse4x4(a, b);}
}
// Relative residual (squared) above which a solve is treated as too
// ill-conditioned to use.  1e-4 = 1% relative error.
#define AP_MOTORS_ALLOC_RESIDUAL_REL_SQ 1.0e-4f

// Upper bound on MOT_FAIL_YSUP.  Not a stability limit - alpha stays inside
// the positive-definite region for any non-negative ysup - but residual yaw
// falls as 1/(1+ysup), so beyond about 20 the knob buys tenths of a percent
// while still spending the lowest motor's downward margin.
#define AP_MOTORS_ALLOC_YSUP_MAX      20.0f

// Nearest-reachable-point solver (MOT_FAIL_RELAX).  See allocate_nearest_reachable().
#define AP_MOTORS_ALLOC_RELAX_MAX     100.0f   // ceiling on the attitude weight
#define AP_MOTORS_ALLOC_RELAX_EPS     1.0e-4f  // secondary objective / regulariser
#define AP_MOTORS_ALLOC_RELAX_KKT     1.0e-6f  // gradient tolerance for releasing a bound
#define AP_MOTORS_ALLOC_RELAX_PASSES  12       // pins plus releases; n=5 needs far fewer
// 3x3 solve by Cramer's rule.  The allocator's own mat_inverse() would do, but
// this path only ever needs the solution vector and runs up to 40 times inside
// Phase 1, so forming the inverse would be wasted work.
static bool alloc_solve3(const float A[9], const float b[3], float x[3])
{
	const float det = A[0] * (A[4] * A[8] - A[5] * A[7]) - A[1] * (A[3] * A[8] - A[5] * A[6])
			  + A[2] * (A[3] * A[7] - A[4] * A[6]);

	if (!isfinite(det) || fabsf(det) < 1e-12f) {
		return false;
	}

	for (uint8_t c = 0; c < 3; c++) {
		float M[9];
		memcpy(M, A, sizeof(M));

		for (uint8_t r = 0; r < 3; r++) {
			M[r * 3 + c] = b[r];
		}

		const float d = M[0] * (M[4] * M[8] - M[5] * M[7]) - M[1] * (M[3] * M[8] - M[5] * M[6])
				+ M[2] * (M[3] * M[7] - M[4] * M[6]);
		x[c] = d / det;

		if (!isfinite(x[c])) {
			return false;
		}
	}

	return true;
}

// Solve the m x m system A y = b, m <= 3, by Gaussian elimination with partial
// pivoting.  A and b are overwritten.  Used by the nearest-reachable solver,
// whose systems are never larger than 3x3 whatever the motor count.
static bool alloc_solve_small(float A[3][3], float b[3], uint8_t m, float y[3])
{
	for (uint8_t i = 0; i < m; i++) {
		uint8_t piv = i;

		for (uint8_t r = i + 1; r < m; r++) {
			if (fabsf(A[r][i]) > fabsf(A[piv][i])) {
				piv = r;
			}
		}

		// Written so that a NaN pivot also fails.
		if (!(fabsf(A[piv][i]) > 1e-12f)) {
			return false;
		}

		if (piv != i) {
			for (uint8_t c = 0; c < m; c++) {
				const float tmp = A[i][c]; A[i][c] = A[piv][c]; A[piv][c] = tmp;
			}

			const float tmp = b[i]; b[i] = b[piv]; b[piv] = tmp;
		}

		for (uint8_t r = i + 1; r < m; r++) {
			const float f = A[r][i] / A[i][i];

			for (uint8_t c = i; c < m; c++) {
				A[r][c] -= f * A[i][c];
			}

			b[r] -= f * b[i];
		}
	}

	for (int8_t i = (int8_t)m - 1; i >= 0; i--) {
		float v = b[i];

		for (uint8_t j = (uint8_t)i + 1; j < m; j++) {
			v -= A[i][j] * y[j];
		}

		y[i] = v / A[i][i];

		if (!isfinite(y[i])) {
			return false;
		}
	}

	return true;
}

// Tolerance on the box.  Loose enough that a vertex computed in single
// precision is not rejected for landing a few ulp outside its own bound.
#define AP_MOTORS_ALLOC_BOX_TOL 1.0e-5f

bool P04Allocator::alloc_phase1_vertex(const float rem[3], const uint8_t idx[],
				       uint8_t n, float t_out[], bool fixed[]) const
{
	for (uint8_t a = 0; a < n; a++) {
		for (uint8_t b = a + 1; b < n; b++) {
			for (uint8_t bits = 0; bits < 4; bits++) {
				const float va = (bits & 1) ? 1.0f : 0.0f;
				const float vb = (bits & 2) ? 1.0f : 0.0f;
				const uint8_t ma = idx[a], mb = idx[b];
				float r[3] = {
					rem[0] - va *_throttle_factor[ma] - vb *_throttle_factor[mb],
					rem[1] - va *_roll_factor[ma]     - vb *_roll_factor[mb],
					rem[2] - va *_pitch_factor[ma]    - vb *_pitch_factor[mb],
				};
				uint8_t fi[3], nf = 0;

				for (uint8_t k = 0; k < n && nf < 3; k++) {
					if (k != a && k != b) {
						fi[nf++] = k;
					}
				}

				if (nf < 3) {
					continue;
				}

				float A[9];

				for (uint8_t c = 0; c < 3; c++) {
					const uint8_t m = idx[fi[c]];
					A[0 * 3 + c] = _throttle_factor[m];
					A[1 * 3 + c] = _roll_factor[m];
					A[2 * 3 + c] = _pitch_factor[m];
				}

				float x[3];

				if (!alloc_solve3(A, r, x)) {
					continue;
				}

				if (x[0] < -AP_MOTORS_ALLOC_BOX_TOL || x[0] > 1.0f + AP_MOTORS_ALLOC_BOX_TOL ||
				    x[1] < -AP_MOTORS_ALLOC_BOX_TOL || x[1] > 1.0f + AP_MOTORS_ALLOC_BOX_TOL ||
				    x[2] < -AP_MOTORS_ALLOC_BOX_TOL || x[2] > 1.0f + AP_MOTORS_ALLOC_BOX_TOL) {
					continue;
				}

				for (uint8_t k = 0; k < n; k++) {
					t_out[k] = 0.0f;
					// Keep the independent three-column basis just solved.
					// Incidental bounds on its solution are not additional
					// working constraints: fixing them can destroy rank.
					fixed[k] = k == a || k == b;
				}

				t_out[a] = va;
				t_out[b] = vb;

				for (uint8_t c = 0; c < 3; c++) {
					t_out[fi[c]] = constrain_float(x[c], 0.0f, 1.0f);
				}

				return true;
			}
		}
	}

	return false;
}

// Nearest reachable point.  Called only when allocate_active_set() has reported
// INFEASIBLE - the equality B t = rem has no solution inside [0,1]^n, so the
// controller is asking for a wrench the surviving motors cannot produce.
//
// Rather than fail and let the vehicle drop silently to forward mixing, solve
//
//     min ||W (B t - rem)||^2 + eps ||t - t_ref||^2      s.t.  t in [0,1]^n
//
// W puts 1.0 on the throttle row and MOT_FAIL_RELAX on the two attitude rows,
// so the weight *is* the answer to "what do we give up".  The eps term is the
// secondary objective of the standard two-level weighted least squares - among
// points with equal residual it picks the one closest to what was commanded -
// and it also makes the normal equations solvable when more than `rows` motors
//
// Bound-constrained active set.  Each pass solves the unconstrained problem on
// the free motors; a motor that lands outside the box is pinned to the bound it
// crossed, and a pinned motor whose gradient points back into the box is let go.
// Strict convexity (eps > 0) makes the minimiser unique and the iteration finite.
// This is cheaper than the vertex enumeration Phase 1 does: passes are bounded by
// n plus the release rounds, each pass one small solve, against Phase 1's forty.
//
// The yaw target projection is deliberately not applied here.  It spends the
// null space left over once the equality is met, and on this path there is no
// such null space - the equality is precisely what could not be met.
bool P04Allocator::allocate_nearest_reachable(const float demand[4],
		float thrust_out[MotorCount]) const
{
	const uint8_t rows = 3;
	uint8_t idx[MotorCount];
	uint8_t n = 0;

	for (uint8_t i = 0; i < MotorCount; i++) {
		thrust_out[i] = 0.0f;

		if (motor_enabled[i]) {
			idx[n++] = i;
		}
	}

	if (n < rows) {
		return false;
	}

	const float w_att = constrain_float(_fail_relax, 0.0f, AP_MOTORS_ALLOC_RELAX_MAX);

	if (!is_positive(w_att)) {
		return false;
	}

	const float W[3] = { 1.0f, w_att, w_att };
	// The attitude terms of the gradient carry w_att^2, and so does the
	// single-precision noise in them: at MOT_FAIL_RELAX = 100 a residual
	// rounded at 1e-7 becomes 1e-3 of gradient, a thousand times a fixed 1e-6
	// threshold.  A pinned motor then flips between released and re-pinned
	// until the pass budget runs out and the cycle drops to forward mixing.
	// Scale the threshold with the objective it is tested against.
	const float kkt = AP_MOTORS_ALLOC_RELAX_KKT * MAX(1.0f, sq(w_att));

	float col[MotorCount][3];

	for (uint8_t k = 0; k < n; k++) {
		const uint8_t m = idx[k];
		col[k][0] = _throttle_factor[m];
		col[k][1] = _roll_factor[m];
		col[k][2] = _pitch_factor[m];
	}

	// Same scaling the throttle row is summed with everywhere else.
	const float rem[3] = { demand[0] *(float)n, demand[1], demand[2] };
	const float tref = constrain_float(demand[0], 0.0f, 1.0f);

	float t[MotorCount];
	// state: 0 free, 1 pinned low, 2 pinned high
	uint8_t st[MotorCount];

	for (uint8_t k = 0; k < n; k++) {
		t[k] = tref;
		st[k] = 0;
	}

	for (uint8_t pass = 0; pass < AP_MOTORS_ALLOC_RELAX_PASSES; pass++) {
		uint8_t fi[MotorCount], nf = 0;

		for (uint8_t k = 0; k < n; k++) {
			if (st[k] == 0) {
				fi[nf++] = k;
			}

			t[k] = (st[k] == 2) ? 1.0f : (st[k] == 1 ? 0.0f : t[k]);
		}

		if (nf > 0) {
			float fixed_w[3] = { 0.0f, 0.0f, 0.0f };

			for (uint8_t k = 0; k < n; k++) {
				if (st[k] == 0) {
					continue;
				}

				for (uint8_t r = 0; r < rows; r++) {
					fixed_w[r] += t[k] * col[k][r];
				}
			}

			// Normal equations of the free motors:
			//     (B'W^2 B + eps I) x = B'W^2 r + eps tref 1,   r = rem - fixed_w
			// B is 3 x nf.  Solving them as an nf x nf system needs a square
			// array sized for MotorCount on the stack - 4 KB at
			// 32 motors, for a problem that never has more than five free.  The
			// matrix is eps I plus a rank-3 term, so the Woodbury identity gives
			// the same solution through a 3x3 system instead:
			//     x = tref 1 + B' H^-1 (r - tref B 1),   H = B B' + eps W^-2
			// With nf <= 3 the direct system is already that small, and is better
			// conditioned than H, whose rank can drop below three there.
			float r0[3];

			for (uint8_t r = 0; r < rows; r++) {
				r0[r] = rem[r] - fixed_w[r];
			}

			float A[3][3], b[3], y[3];
			float x[MotorCount];

			if (nf <= rows) {
				for (uint8_t i = 0; i < nf; i++) {
					for (uint8_t j = 0; j < nf; j++) {
						float v = 0.0f;

						for (uint8_t r = 0; r < rows; r++) {
							v += W[r] * W[r] * col[fi[i]][r] * col[fi[j]][r];
						}

						A[i][j] = v + ((i == j) ? AP_MOTORS_ALLOC_RELAX_EPS : 0.0f);
					}

					float v = 0.0f;

					for (uint8_t r = 0; r < rows; r++) {
						v += W[r] * W[r] * col[fi[i]][r] * r0[r];
					}

					b[i] = v + AP_MOTORS_ALLOC_RELAX_EPS * tref;
				}

				if (!alloc_solve_small(A, b, nf, y)) {
					return false;
				}

				for (uint8_t i = 0; i < nf; i++) {
					x[i] = y[i];
				}

			} else {
				for (uint8_t p = 0; p < rows; p++) {
					float sum_col = 0.0f;

					for (uint8_t q = 0; q < rows; q++) {
						float v = 0.0f;

						for (uint8_t i = 0; i < nf; i++) {
							v += col[fi[i]][p] * col[fi[i]][q];
						}

						A[p][q] = v + ((p == q) ? AP_MOTORS_ALLOC_RELAX_EPS / (W[p] * W[p]) : 0.0f);
					}

					for (uint8_t i = 0; i < nf; i++) {
						sum_col += col[fi[i]][p];
					}

					b[p] = r0[p] - tref * sum_col;
				}

				if (!alloc_solve_small(A, b, rows, y)) {
					return false;
				}

				for (uint8_t i = 0; i < nf; i++) {
					float v = tref;

					for (uint8_t r = 0; r < rows; r++) {
						v += col[fi[i]][r] * y[r];
					}

					x[i] = v;
				}
			}

			// Pin the worst violator and re-solve, one per pass.  Pinning them
			// all at once can turn a solvable subproblem into a rank deficient
			// one - the same trap allocate_redistributed() was fixed for.
			uint8_t worst = 0;
			float worst_by = 0.0f;

			for (uint8_t i = 0; i < nf; i++) {
				const float over = (x[i] > 1.0f) ? (x[i] - 1.0f)
						   : ((x[i] < 0.0f) ? -x[i] : 0.0f);

				if (over > worst_by) {
					worst_by = over;
					worst = i;
				}
			}

			if (worst_by > 0.0f) {
				st[fi[worst]] = (x[worst] > 1.0f) ? 2 : 1;
				t[fi[worst]] = (x[worst] > 1.0f) ? 1.0f : 0.0f;
				continue;
			}

			for (uint8_t i = 0; i < nf; i++) {
				t[fi[i]] = x[i];
			}
		}

		// Optimal for this active set.  Release any pinned motor whose gradient
		// points back into the box; without this the answer is only the best on
		// whatever set the clamping happened to produce.
		float resid[3];

		for (uint8_t r = 0; r < rows; r++) {
			float v = 0.0f;

			for (uint8_t k = 0; k < n; k++) {
				v += t[k] * col[k][r];
			}

			resid[r] = v - rem[r];
		}

		bool released = false;

		for (uint8_t k = 0; k < n; k++) {
			if (st[k] == 0) {
				continue;
			}

			float g = AP_MOTORS_ALLOC_RELAX_EPS * (t[k] - tref);

			for (uint8_t r = 0; r < rows; r++) {
				g += W[r] * W[r] * col[k][r] * resid[r];
			}

			if ((st[k] == 1 && g < -kkt) ||
			    (st[k] == 2 && g >  kkt)) {
				st[k] = 0;
				released = true;
			}
		}

		if (!released) {
			for (uint8_t k = 0; k < n; k++) {
				if (!isfinite(t[k])) {
					return false;
				}

				thrust_out[idx[k]] = constrain_float(t[k], 0.0f, 1.0f);
			}

			return true;
		}
	}

	return false;
}

bool P04Allocator::allocate_active_set(const float demand[4],
				       float thrust_out[MotorCount]) const
{
	const uint8_t rows = 3;
	uint8_t idx[MotorCount];
	uint8_t n = 0;

	for (uint8_t i = 0; i < MotorCount; i++) {
		thrust_out[i] = 0.0f;

		if (motor_enabled[i]) {
			idx[n++] = i;
		}
	}

	_alloc_result = AllocResult::NOT_RUN;
	_alloc_iters = 0;
	_alloc_phase1 = false;
	_alloc_clamp_mask = 0;

	if (n < rows) {
		_alloc_result = AllocResult::TOO_FEW_MOTORS;
		_as_warm = false;
		return false;
	}

	// Phase 1 enumerates two fixed plus three free motors. Other sizes
	// continue through the existing redistributed allocator.
	if (n != 5) {
		_as_warm = false;
		return false;
	}

	for (uint8_t r = 0; r < 4; r++) {
		if (!isfinite(demand[r])) {
			_alloc_result = AllocResult::NON_FINITE;
			_as_warm = false;
			return false;
		}
	}

	if (!isfinite(float(_fail_yaw_suppress)) || !isfinite(float(_fail_yaw_track))) {
		_alloc_result = AllocResult::NON_FINITE;
		_as_warm = false;
		return false;
	}

	// The throttle row sums the motors while the demand is per motor, the same
	// scaling allocate_redistributed() applies to rem[0].
	const float rem[3] = { demand[0] *(float)n, demand[1], demand[2] };
	const float ysup = constrain_float(_fail_yaw_suppress, 0.0f,
					   AP_MOTORS_ALLOC_YSUP_MAX);
	const float ytrk = constrain_float(_fail_yaw_track, 0.0f, 1.0f);

	// Warm start, then Phase 1 only if it does not hold up.  The stored point
	// was feasible for the previous cycle's demand; whether it still is for
	// this one has to be checked, not assumed - the equality rows are what
	// moved.
	//
	// What carries across cycles is the *active set*, not the point.  The point
	// cannot carry: the equality rows are exactly what moved between cycles, so
	// last cycle's thrusts satisfy last cycle's demand and essentially never
	// this one - checking them would send almost every cycle to Phase 1.  The
	// active set, by contrast, is a guess about which motors sit on a bound at
	// the optimum, and that guess is nearly always still right when the demand
	// has moved a little.  So: restore the set, re-solve the subproblem for the
	// new demand, and take the result if it lands inside the box.  One 3x3
	// solve, against Phase 1's forty.
	float T[MotorCount] {};
	bool fixed[MotorCount] {};
	bool have_point = false;

	if (_as_warm) {
		for (uint8_t k = 0; k < n; k++) {
			fixed[k] = (_as_fixed_mask & (1U << k)) != 0;
			// Fixed motors sit on a bound, and a bound is still a bound after
			// the demand moves - these values carry across unchanged.
			T[k] = fixed[k] ? (_as_thrust[k] >= 0.5f ? 1.0f : 0.0f) : 0.0f;
		}

		float rw[3] = { rem[0], rem[1], rem[2] };
		float bbtw[9] = {}, svw[3] = {};
		float ytyw = 0.0f, cfixw = 0.0f;
		uint8_t nfw = 0;

		for (uint8_t k = 0; k < n; k++) {
			const uint8_t m = idx[k];

			if (fixed[k]) {
				rw[0] -= T[k] * _throttle_factor[m];
				rw[1] -= T[k] * _roll_factor[m];
				rw[2] -= T[k] * _pitch_factor[m];
				cfixw += T[k] * _yaw_geom[m];
				continue;
			}

			nfw++;
			ytyw += _yaw_geom[m] * _yaw_geom[m];
			const float col[3] = { _throttle_factor[m], _roll_factor[m], _pitch_factor[m] };

			for (uint8_t a = 0; a < rows; a++) {
				svw[a] += col[a] * _yaw_geom[m];

				for (uint8_t b = 0; b < rows; b++) {
					bbtw[a * rows + b] += col[a] * col[b];
				}
			}
		}

		if (nfw >= rows) {
			const float aw = (is_positive(ysup) && is_positive(ytyw))
					 ? ysup / (1.0f + ysup * ytyw) : 0.0f;
			const float tgt = ytrk * demand[3];
			float Aw[9], rhsw[3], lamw[3];

			for (uint8_t a = 0; a < rows; a++) {
				rhsw[a] = rw[a] + aw * (cfixw - tgt) * svw[a];

				for (uint8_t b = 0; b < rows; b++) {
					Aw[a * rows + b] = bbtw[a * rows + b] - aw * svw[a] * svw[b];
				}
			}

			if (alloc_solve3(Aw, rhsw, lamw)) {
				float slw = 0.0f;

				for (uint8_t a = 0; a < rows; a++) {
					slw += svw[a] * lamw[a];
				}

				bool inside = true;

				for (uint8_t k = 0; k < n && inside; k++) {
					if (fixed[k]) {
						continue;
					}

					const uint8_t m = idx[k];
					float t = _throttle_factor[m] * lamw[0] + _roll_factor[m] * lamw[1]
						  + _pitch_factor[m] * lamw[2];
					t -= aw * _yaw_geom[m] * slw;
					t -= aw * (cfixw - tgt) * _yaw_geom[m];

					if (!isfinite(t) || t < -AP_MOTORS_ALLOC_BOX_TOL ||
					    t > 1.0f + AP_MOTORS_ALLOC_BOX_TOL) {
						inside = false;
						break;
					}

					T[k] = constrain_float(t, 0.0f, 1.0f);
				}

				have_point = inside;
			}
		}
	}

	if (!have_point) {
		_alloc_phase1 = true;

		if (!alloc_phase1_vertex(rem, idx, n, T, fixed)) {
			// No usable vertex was found. Numerical rejection is also possible,
			// so let the caller try the older solver before forward mixing.
			_alloc_result = AllocResult::INFEASIBLE;
			_as_warm = false;
			return false;
		}
	}

	// Bound the work, including degenerate zero-length steps. This is not a
	// target-board timing guarantee; the 400 Hz budget must be measured there.
	const uint8_t max_iter = 4 * n;
	bool converged = false;

	for (uint8_t it = 0; it < max_iter; it++) {
		_alloc_iters = it + 1;

		// Equality-constrained subproblem on the free motors.  Objective is
		//   |T|^2 + ysup*(y.T - ytrk*yaw_demand)^2
		// so Q = I + ysup*y*y^T and Q^-1 = I - alpha*y*y^T by Sherman-Morrison,
		// with alpha = ysup/(1 + ysup*y^T y) - the same coefficient
		// allocate_redistributed() uses, and for the same reason.
		uint8_t nf = 0;
		float yty = 0.0f, cfix = 0.0f, sv[3] = {}, bbt[9] = {}, rf[3];

		for (uint8_t r = 0; r < rows; r++) {
			rf[r] = rem[r];
		}

		for (uint8_t k = 0; k < n; k++) {
			const uint8_t m = idx[k];

			if (fixed[k]) {
				rf[0] -= T[k] * _throttle_factor[m];
				rf[1] -= T[k] * _roll_factor[m];
				rf[2] -= T[k] * _pitch_factor[m];
				// Yaw the fixed motors are already making.  allocate_redistributed()
				// leaves this term out, which is the gap the audit recorded as
				// P1: with motors pinned, the yaw still to be served is the
				// target less what they contribute, not the raw target.
				cfix += T[k] * _yaw_geom[m];
				continue;
			}

			nf++;
			yty += _yaw_geom[m] * _yaw_geom[m];
			const float col[3] = { _throttle_factor[m], _roll_factor[m], _pitch_factor[m] };

			for (uint8_t a = 0; a < rows; a++) {
				sv[a] += col[a] * _yaw_geom[m];

				for (uint8_t b = 0; b < rows; b++) {
					bbt[a * rows + b] += col[a] * col[b];
				}
			}
		}

		if (nf < rows) {
			// Not enough free motors to span the rows.  Unlike the clamp-only
			// solver, this is not a dead end: T is still feasible, so commit it.
			_alloc_result = AllocResult::TOO_FEW_MOTORS;
			converged = true;
			break;
		}

		const float alpha = (is_positive(ysup) && is_positive(yty))
				    ? ysup / (1.0f + ysup * yty) : 0.0f;
		const float target = ytrk * demand[3];
		float A[9], rhs[3], lam[3];

		for (uint8_t a = 0; a < rows; a++) {
			rhs[a] = rf[a] + alpha * (cfix - target) * sv[a];

			for (uint8_t b = 0; b < rows; b++) {
				A[a * rows + b] = bbt[a * rows + b] - alpha * sv[a] * sv[b];
			}
		}

		if (!alloc_solve3(A, rhs, lam)) {
			_alloc_result = AllocResult::SINGULAR; // checked feasible below
			converged = true;
			break;
		}

		float slam = 0.0f;

		for (uint8_t a = 0; a < rows; a++) {
			slam += sv[a] * lam[a];
		}

		// Direction from the current point to the subproblem optimum.  B*d = 0
		// because both ends satisfy the equalities, so stepping along it keeps
		// them satisfied exactly - that is the whole reason for working in
		// directions rather than jumping.
		float d[MotorCount] {};
		float dn = 0.0f;
		bool finite = true;

		for (uint8_t k = 0; k < n && finite; k++) {
			if (fixed[k]) {
				continue;
			}

			const uint8_t m = idx[k];
			float t = _throttle_factor[m] * lam[0] + _roll_factor[m] * lam[1]
				  + _pitch_factor[m] * lam[2];
			t -= alpha * _yaw_geom[m] * slam;
			t -= alpha * (cfix - target) * _yaw_geom[m];

			if (!isfinite(t)) {
				finite = false;
				break;
			}

			d[k] = t - T[k];
			dn += d[k] * d[k];
		}

		if (!finite) {
			_alloc_result = AllocResult::NON_FINITE; // checked feasible below
			converged = true;
			break;
		}

		// With exactly three independent free columns, the equalities fix
		// the point uniquely: the feasible search direction is identically
		// zero. Do not ratio-test the rounding error in t - T in that case.
		// At a degenerate Phase 1 vertex it can otherwise add a third bound,
		// leave only two free motors, and accept a non-optimal vertex before
		// ever checking which working bound needs to be released.
		if (nf == rows || dn < 1e-14f) {
			// At the subproblem optimum.  Check the sign of each active bound's
			// multiplier: a bound holding a motor against the objective has the
			// wrong sign and should be released.  Nothing to release means the
			// KKT conditions hold and this is the answer.
			float gy = 0.0f;

			for (uint8_t k = 0; k < n; k++) {
				gy += _yaw_geom[idx[k]] * T[k];
			}

			int8_t rel = -1;
			float worst = 1e-6f;

			for (uint8_t k = 0; k < n; k++) {
				if (!fixed[k]) {
					continue;
				}

				const uint8_t m = idx[k];
				const float g = T[k] + ysup * (gy - target) * _yaw_geom[m]
						- (_throttle_factor[m] * lam[0] + _roll_factor[m] * lam[1]
						   + _pitch_factor[m] * lam[2]);
				const float viol = (T[k] <= 0.5f) ? -g : g;

				if (viol > worst) {
					worst = viol;
					rel = int8_t(k);
				}
			}

			if (rel < 0) {
				_alloc_result = AllocResult::OK;
				converged = true;
				break;
			}

			fixed[rel] = false;
			_alloc_clamp_mask &= (uint8_t)~(1U << (idx[rel] & 7));
			continue;
		}

		// Ratio test: how far along d before the first bound is reached.
		float step = 1.0f;
		int8_t block = -1;

		for (uint8_t k = 0; k < n; k++) {
			if (fixed[k] || fabsf(d[k]) < 1e-12f) {
				continue;
			}

			const float lim = (d[k] > 0.0f) ? (1.0f - T[k]) / d[k] : (0.0f - T[k]) / d[k];

			if (lim < step) {
				step = MAX(lim, 0.0f);
				block = int8_t(k);
			}
		}

		for (uint8_t k = 0; k < n; k++) {
			if (!fixed[k]) {
				T[k] = constrain_float(T[k] + step * d[k], 0.0f, 1.0f);
			}
		}

		if (block >= 0 && step < 1.0f) {
			fixed[block] = true;
			_alloc_clamp_mask |= (uint8_t)(1U << (idx[block] & 7));
		}
	}

	if (!converged) {
		// Ran out of iterations.  T is still feasible - it has been feasible
		// since Phase 1 - so hand it back rather than throwing away a usable
		// answer, and say in the log that it is not the optimum.
		_alloc_result = AllocResult::ITER_LIMIT;
	}

	// Check the final bounded point before committing any output or warm
	// state. Finite precision and clipping can invalidate B*T == demand.
	float achieved[3] {};

	for (uint8_t k = 0; k < n; k++) {
		if (!isfinite(T[k])) {
			_alloc_result = AllocResult::NON_FINITE;
			_as_warm = false;
			return false;
		}

		T[k] = constrain_float(T[k], 0.0f, 1.0f);
		const uint8_t m = idx[k];
		achieved[0] += T[k] * _throttle_factor[m];
		achieved[1] += T[k] * _roll_factor[m];
		achieved[2] += T[k] * _pitch_factor[m];
	}

	for (uint8_t r = 0; r < rows; r++) {
		if (!isfinite(achieved[r]) || fabsf(achieved[r] - rem[r]) > 1.0e-3f) {
			_alloc_result = AllocResult::RESIDUAL;
			_as_warm = false;
			return false;
		}
	}

	// Keep the point and its active set for the next cycle.
	_alloc_clamp_mask = 0;
	_as_fixed_mask = 0;

	for (uint8_t k = 0; k < n; k++) {
		_as_thrust[k] = T[k];

		if (fixed[k]) {
			_as_fixed_mask |= (uint16_t)(1U << k);
		}

		thrust_out[idx[k]] = constrain_float(T[k], 0.0f, 1.0f);

		if (T[k] <= AP_MOTORS_ALLOC_BOX_TOL || T[k] >= 1.0f - AP_MOTORS_ALLOC_BOX_TOL) {
			_alloc_clamp_mask |= uint8_t(1U << (idx[k] & 7));
		}
	}

	_as_warm = true;
	return true;
}

// Fraction of the way from the committed solution toward the yaw-optimal point.
//
// Calibrated offline over s = 0 .. 1 on five working points taken from a real
// degraded flight (test_alloc_feasible, ProtoYawBlendCalibration):
//
//   s     net parasitic yaw   vs s=0     worst thrust margin
//   0.0        2.0279            -            0.0125
//   0.6        1.0971           46%           0.0050
//   0.8        0.7869           61%           0.0025
//   0.9        0.6501           68%           0.0013
//   1.0        0.6718           67%           0.0000
//
// Two readings pick 0.8.  s = 1 lands exactly on a vertex - margin identically
// zero, so any disturbance clips on the very next cycle - and it is not even the
// best yaw: s = 0.9 beats it, because the blend can pass nearer zero than the
// endpoint does.  0.8 takes 61% of the reduction and keeps a margin that is
// small but not nothing.  The margin at s = 0 is already only 0.0125, so this
// buys a fifth of a thin number; that is the honest size of the trade.
#define AP_MOTORS_ALLOC_YAW_BLEND 0.8f

// Least and greatest yaw moment reachable inside the box, with the vertices that
// attain them.  Same vertex enumeration as alloc_phase1_vertex(): a bounded
// polytope's extreme points of a linear objective are vertices, and with 3 hard
// rows a vertex has at least n-3 components on a bound, so fixing every pair and
// solving the remaining 3x3 enumerates them all.
bool P04Allocator::alloc_yaw_extremes(const float rem[3], const uint8_t idx[],
				      uint8_t n, float *lo, float *hi,
				      float t_lo[], float t_hi[], const float yaw_w[]) const
{
	bool any = false;
	float ylo = 0.0f, yhi = 0.0f;

	for (uint8_t a = 0; a < n; a++) {
		for (uint8_t b = a + 1; b < n; b++) {
			for (uint8_t bits = 0; bits < 4; bits++) {
				const float va = (bits & 1) ? 1.0f : 0.0f;
				const float vb = (bits & 2) ? 1.0f : 0.0f;
				const uint8_t ma = idx[a], mb = idx[b];
				const float r[3] = {
					rem[0] - va *_throttle_factor[ma] - vb *_throttle_factor[mb],
					rem[1] - va *_roll_factor[ma]     - vb *_roll_factor[mb],
					rem[2] - va *_pitch_factor[ma]    - vb *_pitch_factor[mb],
				};
				uint8_t fi[3], nf = 0;

				for (uint8_t k = 0; k < n && nf < 3; k++) {
					if (k != a && k != b) {
						fi[nf++] = k;
					}
				}

				if (nf < 3) {
					continue;
				}

				float A[9];

				for (uint8_t c = 0; c < 3; c++) {
					const uint8_t mm = idx[fi[c]];
					A[0 * 3 + c] = _throttle_factor[mm];
					A[1 * 3 + c] = _roll_factor[mm];
					A[2 * 3 + c] = _pitch_factor[mm];
				}

				float x[3];

				if (!alloc_solve3(A, r, x)) {
					continue;
				}

				if (x[0] < -AP_MOTORS_ALLOC_BOX_TOL || x[0] > 1.0f + AP_MOTORS_ALLOC_BOX_TOL ||
				    x[1] < -AP_MOTORS_ALLOC_BOX_TOL || x[1] > 1.0f + AP_MOTORS_ALLOC_BOX_TOL ||
				    x[2] < -AP_MOTORS_ALLOC_BOX_TOL || x[2] > 1.0f + AP_MOTORS_ALLOC_BOX_TOL) {
					continue;
				}

				float t[MotorCount] {};
				t[a] = va;
				t[b] = vb;

				for (uint8_t c = 0; c < 3; c++) {
					t[fi[c]] = constrain_float(x[c], 0.0f, 1.0f);
				}

				float yaw = 0.0f;

				for (uint8_t k = 0; k < n; k++) {
					yaw += t[k] * ((yaw_w != nullptr) ? yaw_w[k] : _yaw_geom[idx[k]]);
				}

				if (!any || yaw < ylo) {
					ylo = yaw;
					memcpy(t_lo, t, sizeof(float) * n);
				}

				if (!any || yaw > yhi) {
					yhi = yaw;
					memcpy(t_hi, t, sizeof(float) * n);
				}

				any = true;
			}
		}
	}

	if (any) {
		*lo = ylo;
		*hi = yhi;
	}

	return any;
}

// 4x4 行列式，用于下面的零空间余子式展开。
static float alloc_det4(const float m[4][4])
{
	float d = 0.0f;

	for (uint8_t c = 0; c < 4; c++) {
		float sub[3][3];

		for (uint8_t i = 1; i < 4; i++) {
			uint8_t jj = 0;

			for (uint8_t j = 0; j < 4; j++) {
				if (j == c) { continue; }

				sub[i - 1][jj++] = m[i][j];
			}
		}

		const float s3 = sub[0][0] * (sub[1][1] * sub[2][2] - sub[1][2] * sub[2][1])
				 - sub[0][1] * (sub[1][0] * sub[2][2] - sub[1][2] * sub[2][0])
				 + sub[0][2] * (sub[1][0] * sub[2][1] - sub[1][1] * sub[2][0]);
		d += ((c & 1) ? -1.0f : 1.0f) * m[0][c] * s3;
	}

	return d;
}

// 推力/滚转/俯仰/偏航四条等式下的零空间方向。五台存活电机时它恰好是一维——
// 沿它移动，四个量一个都不变，只改推力在电机之间的分配。用余子式展开求，
// 定长运算、无迭代。n != 5 时返回 false（自由度不止一维，本函数不适用）。
bool P04Allocator::yaw_null_direction(const uint8_t idx[], uint8_t n,
				      float dir[MotorCount]) const
{
	if (n != 5) {
		return false;
	}

	float M[4][5];

	for (uint8_t k = 0; k < 5; k++) {
		const uint8_t i = idx[k];
		// 推力行用实际推力系数，与求解器、偏航极值、兑现量同一个矩阵（Codex 10-15
		// 审查 PR04-R03）。原先固定为 1：推力系数不等的机架上，沿这条"零空间"走会
		// 改变加权总推力（反例：M1/M4 系数 0.5、失 6 号，1.5 变成 1.76–2.23）。
		// DJI_X 系数全为 1，数值不变。
		M[0][k] = _throttle_factor[i];
		M[1][k] = _roll_factor[i];
		M[2][k] = _pitch_factor[i];
		M[3][k] = (_failed_motor >= 0) ? _yaw_geom[i] : _yaw_factor[i];
	}

	float biggest = 0.0f;

	for (uint8_t k = 0; k < 5; k++) {
		float sub[4][4];
		uint8_t jj = 0;

		for (uint8_t j = 0; j < 5; j++) {
			if (j == k) { continue; }

			for (uint8_t i = 0; i < 4; i++) { sub[i][jj] = M[i][j]; }

			jj++;
		}

		dir[k] = ((k & 1) ? -1.0f : 1.0f) * alloc_det4(sub);
		biggest = MAX(biggest, fabsf(dir[k]));
	}

	if (!isfinite(biggest) || biggest < 1.0e-6f) {
		return false;                   // 退化：四行不独立，没有可用的一维方向
	}

	// 符号定死：第一个非零分量（电机编号最小的那台）取正。平方和最小与符号无关，
	// 但 MOT_FAIL_NSHF 要沿这条线往指定一端偏，正负必须有固定的物理含义。
	float sign = 1.0f;

	for (uint8_t k = 0; k < 5; k++) {
		if (fabsf(dir[k]) > 1.0e-3f * biggest) {
			sign = (dir[k] < 0.0f) ? -1.0f : 1.0f;
			break;
		}
	}

	for (uint8_t k = 0; k < 5; k++) {
		dir[k] *= sign / biggest;
	}

	return true;
}


// 沿零空间方向 dir 移到「各台推力平方和最小」的点，仍在 [0,1] 盒内。
//
// 沿 dir 走，推力、滚转、俯仰、偏航四行一个都不变，只改负荷在电机间怎么分；
// 平方和最小即尽量均衡。闭式解 a* = -(t·d)/(d·d)，再夹到不出盒的步长区间。
// 已被压在 0 的电机若 d_k = 0 不受影响（M6 失效后的 M3 就是这样），
// 若 d_k ≠ 0 则步长区间保证它不被推出盒。
//
// shift（MOT_FAIL_NSHF，−1…1）：从均衡点再沿这条线偏出去，按「均衡点到盒边」的比例。
// 四轴兑现量照样不变——名义上。它存在的理由是真机的偏航偏置未必与负荷怎么摆无关：
// 电机座倾角若各臂不同，同一组四轴兑现量下，负荷压在哪一对电机上，实际偏航力矩就
// 不一样，而名义模型看不见这一项。E616F 失 6 号后这条线是「1、4 号一对」对
// 「2、5 号一对」：已飞的旧固件落在 1、4 号重的一端（PWM 1858/1815 对 1413/1438），
// 均衡点和另一端都没飞过。给出这个量，让试飞能把这条线扫出来。
// 步长对推力连续（均衡点、盒边都连续），不引入新的切换。
bool P04Allocator::null_line_range(const float t[], const float dir[], uint8_t n,
				   float &a_bal, float &amin, float &amax)
{
	amin = -FLT_MAX;
	amax = FLT_MAX;
	float dot = 0.0f, norm2 = 0.0f;

	for (uint8_t k = 0; k < n; k++) {
		dot += t[k] * dir[k];
		norm2 += sq(dir[k]);

		if (fabsf(dir[k]) < 1.0e-6f) {
			continue;
		}

		const float a0 = (0.0f - t[k]) / dir[k];
		const float a1 = (1.0f - t[k]) / dir[k];
		amin = MAX(amin, MIN(a0, a1));
		amax = MIN(amax, MAX(a0, a1));
	}

	if (!(amax >= amin) || norm2 < 1.0e-12f) {
		return false;
	}

	a_bal = constrain_float(-dot / norm2, amin, amax);
	return true;
}

void P04Allocator::balance_load_on_null_line(float t[], const float dir[], uint8_t n, float shift)
{
	float a, amin, amax;

	if (!null_line_range(t, dir, n, a, amin, amax)) {
		return;
	}

	if (is_positive(shift)) {
		a += MIN(shift, 1.0f) * (amax - a);

	} else if (is_negative(shift)) {
		a += MAX(shift, -1.0f) * (a - amin);
	}

	for (uint8_t k = 0; k < n; k++) {
		// 步长已保证不出盒；这里的夹取只防浮点误差。
		t[k] = constrain_float(t[k] + a * dir[k], 0.0f, 1.0f);
	}
}

// MOT_FAIL_NAUT：按机载辨识自动定零空间上的负荷位置（2026-11-12）。
//
// 六旋翼停一台后，推力、滚转、俯仰、偏航四个名义量定下来还剩一维零空间（见
// yaw_null_direction()）。沿它挪负荷，名义上什么都不变；但真机的偏航偏置若来自
// 某几条机臂（各臂电机座倾角不同），偏置就随负荷在各对之间怎么分而变。E616F
// 10-20 五架次辨识一致地把偏置放在 2、5 号这对上；停 6 号时把负荷挪向 1、4 号，
// 偏置变小，自旋可止（仿真 87 °/s → 0；实飞待测）。MOT_FAIL_NSHF 是同一件事的手动
// 版本：每架机离线取参，而且只能为一个失效位置取一个值。
//
// 前馈。MOT_NULL_SEQ 在健康悬停时测得偏置系数 c：维持航向要的偏航行 ≈ c·P(t)，
// P 把六台推力合成三对推力和（偏航行与分配器第四行同一单位，见 null_seq_accumulate()）。
// 失效后分配器兑现的偏航是 target；沿 t + a·d 走，偏置 c·P(t + a·d) 随 a 线性变，
// 斜率 g = c·P(d)。令偏置等于 target：
//     a* = a_bal + (target − c·P(t + a_bal·d)) / g
// 用的是固件自己当拍的推力与零空间方向，不需要把辨识的"推力和比例"换算成 NSHF 的
// "到盒边比例"——两种定义对不上，换算就是误差来源。结果仍按 NSHF 的尺度表示并限幅。
//
// 闭环（取值 2）。c 来自 ±0.08 的小注入，用到 0.6 是外推；仿真里份额错 30% 就会留下
// 约 33 °/s。所以在偏航够不着（目标被夹在可达区间端点）时，按实测偏航角速度慢慢再挪：
// 顺时针转（ω > 0）说明净力矩为正、偏置抵得不够，向使 c·P 增大的方向挪。
// 不用"需求减兑现"作误差：MOT_FAIL_YINT=0 时积分器冻结在健康时的配平值，那个值在
// 失效后本来就够不着，差值永不为零，会把负荷一直推到上限、反转自旋。角速度没有这个问题。
//
// 斜率 g 太小（这一失效位置挪负荷几乎改不了偏置，例如偏置集中在失效那一对上）时
// 不挪，返回 0。没有辨识结果时退回 MOT_FAIL_NSHF，并提示一次。
//
// 只在偏置确实抵不掉时启用（2026-11-12，Codex 两轮仿真验证后定）。偏航够得着时（E616F
// 失 1、3、5 号），分配器兑现的偏航由速率环定，前馈又跟着它挪负荷，两者互相追，平衡点
// 是一整条线：失 3 号负荷位置一直漂，失 5 号只开前馈反而留 −3.7 °/s。够得着时本来就不
// 需要挪。第一版用"偏航连续够不着 0.5 s"判断，分不清偏置抵不掉与失效后的过渡：失 5 号
// 过渡期够不着超过 0.5 s 被误启用，降落收油时失 1、3 号也误启用。
//
// 现判据直接用辨识：不挪负荷时抵消偏置要的偏航 c·P(t_bal) 落在本拍可达区间 [lo, hi] 外
// （超出区间宽的 5% 以上），而且偏航实际也连续够不着 1 s，才启用；启用后本架次保持
// （偏置抵掉后需求贴着区间端点，随标志进出会来回挪、自旋时停时起）。系数整体反号时，
// 预测值落进可达区间，不启用，行为同取值 0——不会往错的方向挪（第一版闭环在反号时
// 让偏航峰值 21 → 27 °/s）。偏移按每秒 1.0 限速：失效瞬间不一步跳到位，又不至于让飞机
// 先转起来（每秒 0.25 时失 6 号先转到 50 °/s、23 s 才停）。闭环修正只在前馈之外补 ±0.3，
// 到限提示一次。
#define AP_MOTORS_NAUT_GAIN_MIN    0.1f              // |g| 相对 Σ|c|·max|d| 的下限
#define AP_MOTORS_NAUT_CL_GAIN     0.02f             // 闭环：每 rad/s 每秒挪多少（NSHF 尺度）
#define AP_MOTORS_NAUT_CL_WAIT_S   5.0f              // 闭环：启用后等这么久（前馈已按限速到位）才介入
#define AP_MOTORS_NAUT_CL_RATE     0.1f              // 闭环：每秒最多挪多少
#define AP_MOTORS_NAUT_CL_DEADBAND (5.0f * 0.017453292519943295f)     // 闭环：小于此角速度不动
#define AP_MOTORS_NAUT_CL_MAX      0.3f              // 闭环：在前馈之外最多补挪多少
#define AP_MOTORS_NAUT_ENGAGE_S    1.0f              // 预测抵不掉且偏航连续够不着这么久才启用
#define AP_MOTORS_NAUT_OUT_MARGIN  0.05f             // 预测偏置超出可达区间的最小幅度（相对区间宽）
#define AP_MOTORS_NAUT_GAP_MS     20                // 两次判据之间隔这么久就算断开，重新计时
#define AP_MOTORS_NAUT_SLEW        1.0f              // 偏移每秒最多变化多少
float P04Allocator::auto_null_shift(const float t[], const float dir[], const uint8_t idx[], uint8_t n,
				    float target, bool yaw_clipped) const
{
	const float c[3] = { _null_coef[0], _null_coef[1], _null_coef[2] };
	float csum = 0.0f;

	for (uint8_t p = 0; p < 3; p++) {
		if (!isfinite(c[p])) {
			csum = 0.0f;
			break;
		}

		csum += fabsf(c[p]);
	}

	// 存着的系数跨上电沿用。换电机、桨、机臂后由现场决定重跑 MOT_NULL_SEQ（2026-11-13 用户定：
	// 测试人员知道换没换件）。没重跑时，偏置总量变了由过时检测兜底；偏置只换到别的机臂、
	// 总量不变时检测不出，旧系数可能让自旋更快（Codex 第四轮：失 6 号 143 → 218 °/s）——
	// 试飞卡写明换件后要重跑。
	if (!_pair_map_valid || csum < 1.0e-6f || _null_stale) {
		return _fail_null_shift;
	}

	float a_bal, amin, amax;

	if (!null_line_range(t, dir, n, a_bal, amin, amax)) {
		return 0.0f;
	}

	float g = 0.0f, bias = 0.0f, dmax = 0.0f;

	for (uint8_t k = 0; k < n; k++) {
		const int8_t p = _pair_of[idx[k]];

		if (p < 0) {
			return 0.0f;
		}

		g += c[p] * dir[k];
		bias += c[p] * (t[k] + a_bal * dir[k]);
		dmax = MAX(dmax, fabsf(dir[k]));
	}

	const float nmax = constrain_float(_fail_null_max, 0.0f, 0.9f);
	const float dt = (_dt_s > 0.0f) ? _dt_s : (1.0f / 400.0f);
	float s_ff = 0.0f;
	const bool usable = fabsf(g) >= AP_MOTORS_NAUT_GAIN_MIN * csum * dmax;

	if (!_nauto_engaged) {
		const float span = MAX(_yaw_auth.hi - _yaw_auth.lo, 1.0e-3f);
		const float m = AP_MOTORS_NAUT_OUT_MARGIN * span;
		// 预测越界与实际够不着必须是同一端：偏置要的偏航低于下端时，速率环的需求也该被夹
		// 在下端，反之亦然。复验里失 3 号收尾时预测越的是上端、实际夹的是下端，两种方向
		// 相反的越界被一起计时而误启用（Codex 复算）。
		const bool out_low = (bias < _yaw_auth.lo - m) && _yaw_auth.clipped_low;
		const bool out_high = (bias > _yaw_auth.hi + m) && _yaw_auth.clipped_high;
		// 也只在正常飞行中判：收油落地时可达区间骤然变窄，不是失效造成的。
		const bool flying = _flying && !limit.throttle_lower;
		// “连续 1 s”要真连续、同一端：中间若有几拍没走到这里（换了求解分支、投影没提交），
		// 或者越界从下端换到上端，都重新计时（Codex 11-20 第 3 节审查）。
		const uint32_t now_ms = _now_ms;
		const int8_t side = out_low ? -1 : (out_high ? 1 : 0);

		if (now_ms - _nauto_clip_ms > AP_MOTORS_NAUT_GAP_MS || side != _nauto_clip_side) {
			_nauto_clip_s = 0.0f;
		}

		_nauto_clip_ms = now_ms;
		_nauto_clip_side = side;
		_nauto_clip_s = (flying && usable && side != 0) ? (_nauto_clip_s + dt) : 0.0f;

		if (_nauto_clip_s >= AP_MOTORS_NAUT_ENGAGE_S) {
			_nauto_engaged = true;
		}
	}

	if (usable) {
		const float a_ff = a_bal + (target - bias) / g;

		if (a_ff >= a_bal) {
			s_ff = (amax > a_bal) ? (a_ff - a_bal) / (amax - a_bal) : 0.0f;

		} else {
			s_ff = (a_bal > amin) ? (a_ff - a_bal) / (a_bal - amin) : 0.0f;
		}

		s_ff = constrain_float(s_ff, -nmax, nmax);

		if (_nauto_engaged) {
			_nauto_engaged_s += dt;
		}

		// 闭环等前馈按限速到位、再稳一阵才介入：前馈还在挪时飞机仍在转，闭环跟着同向叠加
		// 会冲过头再慢慢回调（Codex 第二、三轮：仅前馈约 14 s 止转，加闭环反而约 39 s）。
		if (_nauto_engaged && _nauto_engaged_s >= AP_MOTORS_NAUT_CL_WAIT_S &&
		    _fail_null_auto >= 2 && yaw_clipped &&
		    fabsf(_body_yaw_rate) > AP_MOTORS_NAUT_CL_DEADBAND) {
			const float step = AP_MOTORS_NAUT_CL_RATE * dt;
			const float ds = AP_MOTORS_NAUT_CL_GAIN * _body_yaw_rate * dt * (is_positive(g) ? 1.0f : -1.0f);
			_nauto_cl += constrain_float(ds, -step, step);
		}

		// 不让闭环量越过上限后继续累积（解除饱和后能立即回头）
		_nauto_cl = constrain_float(_nauto_cl, -AP_MOTORS_NAUT_CL_MAX, AP_MOTORS_NAUT_CL_MAX);
		_nauto_cl = constrain_float(_nauto_cl, -nmax - s_ff, nmax - s_ff);

		if (!_nauto_cl_warned && fabsf(_nauto_cl) >= AP_MOTORS_NAUT_CL_MAX - 1.0e-3f) {
			_nauto_cl_warned = true;
		}
	}

	const float s_target = (usable && _nauto_engaged) ? constrain_float(s_ff + _nauto_cl, -nmax, nmax) : 0.0f;
	const float slew = AP_MOTORS_NAUT_SLEW * dt;
	_nauto_slewed += constrain_float(s_target - _nauto_slewed, -slew, slew);
	const float out = _nauto_slewed;

	_nauto_ff = s_ff;
	_nauto_out = out;
	_nauto_gain = g;

	return out;
}
void P04Allocator::apply_yaw_projection_core(const float demand[4],
		float thrust[MotorCount]) const
{
	uint8_t idx[MotorCount];
	uint8_t n = 0;

	for (uint8_t i = 0; i < MotorCount; i++) {
		if (motor_enabled[i]) {
			idx[n++] = i;
		}
	}

	if (n < 4) {
		return;
	}

	// Same scaling on the throttle row the solver used; see allocate_active_set().
	const float rem[3] = { demand[0] *(float)n, demand[1], demand[2] };

	float lo = 0.0f, hi = 0.0f;
	float t_lo[MotorCount] {}, t_hi[MotorCount] {};

	if (!alloc_yaw_extremes(rem, idx, n, &lo, &hi, t_lo, t_hi)) {
		return;             // no vertex: leave the committed solution alone
	}

	const float target = constrain_float(demand[3], lo, hi);
	// Capability record for this cycle's final allocation.  Written only here,
	// past the last early return, so valid means the projection really ran.
	_yaw_auth.valid = true;
	_yaw_auth.lo = lo;
	_yaw_auth.hi = hi;
	_yaw_auth.target = target;
	_yaw_auth.clipped_low = demand[3] < lo;
	_yaw_auth.clipped_high = demand[3] > hi;
	const float span = hi - lo;
	const float frac = (fabsf(span) < 1.0e-9f)
			   ? 0.0f
			   : constrain_float((target - lo) / span, 0.0f, 1.0f);

	float t_yaw[MotorCount] {};

	for (uint8_t k = 0; k < n; k++) {
		t_yaw[k] = constrain_float(t_lo[k] * (1.0f - frac) + t_hi[k] * frac, 0.0f, 1.0f);
	}

	// MOT_FAIL_YNUL：目标落在可达区间**内部**时，把偏航精确打在目标上，再沿零空间
	// 把负荷摊匀（各台推力平方和最小）。两件事都要，而不是用混合拿偏航误差换余量。
	//
	// 次级目标内部与端点用同一个（2026-10-09 改）。原先内部取「离上下限最远」、端点取
	// 枚举顶点，目标从内部跨到端点的那一步，失效 1/3/5 号时单台推力一步跳 0.37——
	// 而难给侧端点恰是实飞需求所在。平方和严格凸，目标连续移动时最优解也连续移动。
	// 「离上下限最远」另有一个问题：有电机被压在 0 时该指标恒为 0，表现不出其余余量。
	//
	// 目标被夹到端点时（需求不可达，即难给侧）同样精确投影，不混合。它针对的是
	// 「分配器自己制造、自己又修不了的自转」：混合保留的那 0.2·y_QP 把机体往可达侧
	// 推，而纠正它所需的方向恰在不可达的一侧。
	//
	// **2026-10-09 更正（Codex 复核发现）**：这里原先写「端点上没有零空间余地」，
	// 并直接取枚举出的顶点。这是错的。端点处若有电机被压到边界，剩余电机的四轴约束
	// 可能秩亏——M6 失效、M3 压到 0 时剩下四台的 [推力;滚转;俯仰;偏航] 秩只有 3，
	// 端点是一整条线段，例如 [a,b,0,a,b,0] 且 2a+2b=S 全部可行。取顶点等于把负荷
	// 压在两台上（单台 0.75 而均衡时只要 0.375），实测把存活电机顶到 PWM 上限。
	// 现在端点同样沿零空间做负荷均衡，四轴兑现量不变。
	//
	// **取值 1 并入 2（2026-10-09，Codex 分配切换连续性复核）。** 原先 1 只在目标
	// 「严格在内部」（离端点超过 1e-3 倍区间宽）时打准偏航，出了这条线就退回 0.2 混合。
	// 两条路在交界处的输出毫无关系：M6 失效、每台推力 0.3、偏航需求跨过阈值 1e-7，
	// 单台推力一步跳 0.32，且缩小步长不消失。「只在内部打准」本身就接不上——要与
	// 内部连续，阈值外一侧也必须打准、也均衡，那就是取值 2。滞回只减少往返次数，
	// 消不掉单次跳变，所以不加。内部与端点不再分支，也就没有阈值。
	//
	// 零空间方向求不出来（几何退化）时仍取打准偏航的那一点，不退回混合：退回混合
	// 正是上面那种不连续。
	if (_fail_yaw_null != 0 && n == 5) {
		float dir[MotorCount] {};

		if (yaw_null_direction(idx, n, dir)) {
			const float shift = (_fail_null_auto > 0)
					    ? auto_null_shift(t_yaw, dir, idx, n, target, _yaw_auth.clipped_low || _yaw_auth.clipped_high)
					    : _fail_null_shift;
			balance_load_on_null_line(t_yaw, dir, n, shift);
		}

		if (!projection_keeps_rows(rem, idx, n, thrust, t_yaw)) {
			return;         // 保留投影前的解，见 projection_keeps_rows()
		}

		_yaw_auth.exact = true;

		for (uint8_t k = 0; k < n; k++) {
			thrust[idx[k]] = t_yaw[k];
		}

		return;
	}

	// MOT_FAIL_YNUL=0：偏航按 0.2·QP + 0.8·投影 混合，这一语义不变。
	//
	// **混合后沿零空间均衡负荷（2026-10-10，Codex 镜像对称性复核）。** t_yaw 是枚举顶点
	// 的插值，而偏航极值面上的顶点往往不唯一（端点是一整条线段，见上），挑中哪一个由
	// 枚举顺序和浮点微差决定。直接混合时四轴兑现量是连续的，负荷却会在电机之间整块
	// 交换：M6 失效、每台 0.3、偏航 −1，俯仰需求从 0 变到 1e-7，单台推力跳 0.60；
	// 失效位置与需求一起镜像，输出也对不上（单台差到 0.79）。四轴残差都很小，所以
	// 只看兑现量或求解退出码发现不了。
	//
	// 混合点的四轴兑现量只取决于 QP 解与目标，二者都唯一；五台电机、四个等式，零空间
	// 是一维，满足同一兑现量的点是一条线段，线段上平方和最小的点唯一。所以在混合点上
	// 沿零空间做一次负荷均衡，结果与挑中哪个顶点无关，偏航语义也不变——这里不改成
	// 打准模式，是否精确追偏航由 MOT_FAIL_YNUL 决定，与负荷唯一性是两件事。
	float t_blend[MotorCount] {};

	for (uint8_t k = 0; k < n; k++) {
		t_blend[k] = constrain_float(thrust[idx[k]] * (1.0f - AP_MOTORS_ALLOC_YAW_BLEND)
					     + t_yaw[k] * AP_MOTORS_ALLOC_YAW_BLEND,
					     0.0f, 1.0f);
	}

	if (n == 5) {
		float dir[MotorCount] {};

		if (yaw_null_direction(idx, n, dir)) {
			balance_load_on_null_line(t_blend, dir, n, _fail_null_shift);
		}
	}

	if (!projection_keeps_rows(rem, idx, n, thrust, t_blend)) {
		return;             // 保留投影前的解
	}

	for (uint8_t k = 0; k < n; k++) {
		thrust[idx[k]] = t_blend[k];
	}
}

// 偏航投影与零空间均衡只该改偏航和负荷分布：推力、滚转、俯仰三行不能比求解器
// 投影前的解离求解目标 rem 更远（偏航极值的两个端点都精确满足 rem，混合与零空间
// 移动在名义上保持它；求解器自身允许 1% 的相对残差，所以比的是"变差了没有"而不是
// 绝对相等）。提交前按实际矩阵复核一次（Codex 10-15 审查 PR04-R03）：变差超过
// 1e-3 就不提交，保留投影前的解，而不是报告成功却交出另一个推力。1e-3 是单精度
// 累加误差的上百倍，远小于任何物理上有意义的偏差。
bool P04Allocator::projection_keeps_rows(const float rem[3], const uint8_t idx[], uint8_t n,
		const float before[MotorCount],
		const float after[]) const
{
	float b[3] {}, a[3] {};

	for (uint8_t k = 0; k < n; k++) {
		const uint8_t m = idx[k];
		const float col[3] = { _throttle_factor[m], _roll_factor[m], _pitch_factor[m] };

		for (uint8_t r = 0; r < 3; r++) {
			b[r] += before[m] * col[r];
			a[r] += after[k] * col[r];
		}
	}

	for (uint8_t r = 0; r < 3; r++) {
		if (!isfinite(a[r]) || fabsf(a[r] - rem[r]) > fabsf(b[r] - rem[r]) + 1.0e-3f) {
			return false;
		}
	}

	return true;
}

// 偏航投影改变了推力，求解器在投影前算出的触限掩码就不再描述最终输出。
// 2026-10-09 Codex 复核指出：日志 AYaw 对应最终解而 Clp 对应投影前的解，
// 不能用 Clp=0 推断最终电机未饱和。这里按最终推力重算，与求解器活动集不同时记一位。
void P04Allocator::apply_yaw_projection(const float demand[4],
					float thrust[MotorCount]) const
{
	const uint8_t solver_mask = _alloc_clamp_mask;
	apply_yaw_projection_core(demand, thrust);
	uint8_t final_mask = 0;

	for (uint8_t i = 0; i < MotorCount; i++) {
		if (motor_enabled[i] &&
		    (thrust[i] <= AP_MOTORS_ALLOC_BOX_TOL || thrust[i] >= 1.0f - AP_MOTORS_ALLOC_BOX_TOL)) {
			final_mask |= uint8_t(1U << (i & 7));
		}
	}

	_alloc_clamp_moved = (final_mask != solver_mask);
	_alloc_clamp_mask = final_mask;
}

bool P04Allocator::allocate_redistributed(const float demand[4], bool include_yaw,
		float thrust_out[MotorCount]) const
{
	const uint8_t rows = include_yaw ? 4 : 3;
	// Effector matrix rows are [throttle, roll, pitch, yaw]; one column per
	// enabled motor.
	uint8_t idx[MotorCount];
	uint8_t n = 0;

	for (uint8_t i = 0; i < MotorCount; i++) {
		thrust_out[i] = 0.0f;

		if (motor_enabled[i]) {
			idx[n++] = i;
		}
	}

	_alloc_result = AllocResult::NOT_RUN;
	_alloc_passes = 0;
	_alloc_clamp_mask = 0;

	if (n < rows) {
		// Fewer effectors than demands: nothing to redistribute between.
		_alloc_result = AllocResult::TOO_FEW_MOTORS;
		return false;
	}

	bool freed[MotorCount];
	float thrust[MotorCount];

	for (uint8_t k = 0; k < n; k++) {
		freed[k] = true;
		thrust[k] = 0.0f;
	}

	// Whether any pass has produced and verified a complete assignment.  Every
	// exit below is a `break`, and thrust[] starts at zero, so without this the
	// failure of the very first solve is indistinguishable from success and
	// hands the caller six zeroed motors as a valid answer.
	bool have_solution = false;

	// At most one clamp per motor, so this terminates.
	for (uint8_t pass = 0; pass <= n; pass++) {
		_alloc_passes = pass + 1;
		// Demand still to be met by the motors that are free, after removing
		// what the already-clamped ones contribute.
		float rem[4];

		for (uint8_t r = 0; r < rows; r++) {
			rem[r] = demand[r];
		}

		// The throttle demand is per motor, not a total to divide up: forward
		// mixing gives every motor throttle_thrust * _throttle_factor[i].  The
		// rows here form a linear system whose throttle row *sums* the motors,
		// so the demand has to be scaled by how many there are or the solve
		// delivers 1/n of the lift that was asked for.  Getting this wrong is
		// not subtle in the air but is easy to miss on paper: with six motors
		// trimmed at 0.161 and one gone, each survivor needs 0.193, the solver
		// handed out 0.039, and the vehicle sank at 2.5 m/s with the throttle
		// pinned at 1.0 - which still hovers, because 1.0/5 = 0.2 lands just
		// above 0.193, so it read as a controlled hover at a lower altitude
		// rather than as a thrust shortfall.
		rem[0] *= (float)n;

		for (uint8_t k = 0; k < n; k++) {
			if (freed[k]) {
				continue;
			}

			const uint8_t m = idx[k];
			rem[0] -= thrust[k] * _throttle_factor[m];
			rem[1] -= thrust[k] * _roll_factor[m];
			rem[2] -= thrust[k] * _pitch_factor[m];

			if (include_yaw) {
				rem[3] -= thrust[k] * _yaw_factor[m];
			}
		}

		// Minimum-norm solution on the free motors: T = B^T (B B^T)^-1 rem.
		//
		// With yaw out of the demand there are more free motors than rows, so
		// the solution is not unique - a null space is left over, and every
		// point in it delivers the same throttle, roll and pitch. Minimum norm
		// picks one of them without regard to how much yaw moment it leaves
		// behind, and on a hexacopter down one motor that choice is a poor one:
		// it lands on a part of the trade-off that is strictly dominated, with
		// both more residual yaw and less roll authority than a neighbouring
		// solution. MOT_FAIL_YSUP steers the pick by minimising
		//   |T|^2 + ysup * (yaw moment)^2
		// over that same null space, which by Sherman-Morrison is the plain
		// minimum-norm solve with (I - alpha*y*y^T) folded in.
		//
		// Note this suppresses the *parasitic* yaw moment - the target is zero,
		// not the pilot's yaw demand. It is not MOT_FAIL_YAW, which puts yaw
		// back into the demand and asks the controller to chase it.
		const float ysup = (rows == 3)
				   ? constrain_float(_fail_yaw_suppress, 0.0f,
						     AP_MOTORS_ALLOC_YSUP_MAX)
				   : 0.0f;
		uint8_t n_free = 0;
		float syaw[4] = {};     // s_a = sum over free motors of B[a][j] * yaw_j
		float yty = 0.0f;       // y^T y over the free motors
		float bbt[16] = {};

		for (uint8_t k = 0; k < n; k++) {
			if (!freed[k]) {
				continue;
			}

			const uint8_t m = idx[k];
			const float col[4] = { _throttle_factor[m], _roll_factor[m],
					       _pitch_factor[m], _yaw_factor[m]
					     };
			n_free++;
			yty += _yaw_geom[m] * _yaw_geom[m];

			for (uint8_t a = 0; a < rows; a++) {
				syaw[a] += col[a] * _yaw_geom[m];

				for (uint8_t b = 0; b < rows; b++) {
					bbt[a * rows + b] += col[a] * col[b];
				}
			}
		}

		// Sherman-Morrison coefficient for folding ysup*(y^T T)^2 into B B^T.
		//
		// This used to read ysup / n_free, on the stated grounds that the yaw
		// factors are +-1 and so y^T y is just the count.  They are not:
		// normalise_rpy_factors() scales every axis to +-0.5, so y^T y is a
		// quarter of that, and _yaw_geom[] carries those same halved factors.
		// The consequence was not a wrong answer - B*T = rem holds for any
		// alpha - but a knob that delivered about a quarter of its nominal
		// weight, with no way to reach the rest.
		//
		// The correct coefficient is ysup / (1 + ysup*y^T y).  Note it is not
		// ysup / y^T y: that form hits the singularity of (I - alpha*y*y^T) at
		// ysup = 1 exactly, while this one stays below 1/(y^T y) for every
		// ysup >= 0, so the positive-definiteness the fold relies on is
		// structural rather than something the parameter range has to protect.
		//
		// With it, the residual parasitic yaw has a closed form that does not
		// depend on the operating point at all:
		//
		//      residual = baseline / (1 + ysup)
		//
		// i.e. ysup is the penalty weight itself.  Verified across failure
		// position, thrust and roll demand by tools/alloc_alpha_bound.py
		// --verify.  It also sets the range: ysup = 1 only halves the parasitic
		// yaw, 75% needs 3 and 90% needs 9, which is why the parameter is no
		// longer capped at 1.
		//
		// The default was moved 0.7 -> 0.169697 at the same time, which is the
		// weight that reproduces the old alpha of 0.14 to eight decimals.  The
		// fix therefore changes no flight behaviour on its own; the wind sweep
		// that picks a better default is a separate, evidence-backed step.
		const float alpha = (is_positive(ysup) && is_positive(yty))
				    ? ysup / (1.0f + ysup * yty) : 0.0f;

		if (is_positive(alpha)) {
			for (uint8_t a = 0; a < rows; a++) {
				for (uint8_t b = 0; b < rows; b++) {
					bbt[a * rows + b] -= alpha * syaw[a] * syaw[b];
				}
			}
		}

		// Rank test.  B B^T is singular exactly when the free columns of B are
		// rank deficient, so this is the real check; counting motors above only
		// rules out the trivially impossible.
		float bbt_inv[16];

		if (!mat_inverse(bbt, bbt_inv, rows)) {
			_alloc_result = AllocResult::SINGULAR;
			break;
		}

		// MOT_FAIL_YTRK moves the target of that same term off zero and onto
		// the yaw demand: the objective becomes
		//   |T|^2 + ysup * (yaw moment - ytrk * yaw demand)^2
		// Working the Lagrangian through, the only thing a non-zero target
		// changes is one scalar: with alpha already carrying the
		// Sherman-Morrison denominator it comes out as gamma = alpha * target,
		// entering in two places - the right-hand side of the solve loses
		// gamma*s, and each thrust gains gamma*y.  That identity is unchanged
		// by the alpha correction above; checked against a direct constrained
		// optimisation of |T|^2 + ysup*(y^T T - target)^2 to 1e-10.
		//
		// The residual check below still validates against the unmodified rem,
		// and that is not a slip.  Substituting back,
		//   B*t = (B B^T - alpha*s*s^T)*lambda + gamma*s
		//       = (rem - gamma*s) + gamma*s
		//       = rem
		// so the identity the check relies on survives the change untouched.
		//
		// Note what this does NOT do: throttle, roll and pitch stay as hard
		// equality constraints in B, so yaw is served only out of the null
		// space they leave behind.  It cannot trade attitude away for heading -
		// which is exactly what MOT_FAIL_YAW does, and why that one crashed the
		// airframe in 2 m/s of wind while this cannot.  When the null space
		// runs out, motors clamp, the active set shrinks, and the solve falls
		// back toward the plain suppression case on its own.
		const float gamma = (is_positive(alpha) && rows == 3)
				    ? alpha * constrain_float(_fail_yaw_track, 0.0f, 1.0f) * demand[3]
				    : 0.0f;

		float lambda[4] = {};

		for (uint8_t a = 0; a < rows; a++) {
			for (uint8_t b = 0; b < rows; b++) {
				lambda[a] += bbt_inv[a * rows + b] * (rem[b] - gamma * syaw[b]);
			}
		}

		float slam = 0.0f;

		if (is_positive(alpha)) {
			for (uint8_t a = 0; a < rows; a++) {
				slam += syaw[a] * lambda[a];
			}
		}

		// Stage the solution, then check it before committing.
		//
		// inverse3x3()/inverse4x4() reject only an exactly zero determinant
		// (is_zero(), i.e. below FLT_EPSILON), so a merely ill-conditioned
		// matrix is inverted happily and yields entries of order 1/det.  The
		// resulting thrusts are nonsense but finite, every one of them clamps,
		// and the result reads as a valid degraded allocation.
		//
		// The cheap honest test is to substitute back.  t = B^T*lambda -
		// alpha*y*(s^T*lambda) satisfies B*t == rem identically: expanding it
		// gives (B B^T - alpha*s*s^T)*lambda, which is the very system that was
		// solved.  That holds whether or not yaw suppression is active and
		// regardless of what the clamping does afterwards, so any real
		// discrepancy is the conditioning showing through and nothing else.
		float t_raw[MotorCount];
		float check[4] = {};
		bool finite = true;

		for (uint8_t k = 0; k < n && finite; k++) {
			if (!freed[k]) {
				continue;
			}

			const uint8_t m = idx[k];
			float t = _throttle_factor[m] * lambda[0] + _roll_factor[m] * lambda[1]
				  + _pitch_factor[m] * lambda[2];

			if (include_yaw) {
				t += _yaw_factor[m] * lambda[3];
			}

			t -= alpha * _yaw_geom[m] * slam;
			t += gamma * _yaw_geom[m];

			if (!isfinite(t)) {
				// constrain_float() turns a NaN into (low+high)/2, i.e. half
				// throttle on every motor - louder than zero and just as wrong.
				finite = false;
				break;
			}

			t_raw[k] = t;
			const float col[4] = { _throttle_factor[m], _roll_factor[m],
					       _pitch_factor[m], _yaw_factor[m]
					     };

			for (uint8_t r = 0; r < rows; r++) {
				check[r] += col[r] * t;
			}
		}

		if (!finite) {
			_alloc_result = AllocResult::NON_FINITE;
			break;
		}

		float err2 = 0.0f, mag2 = 0.0f;

		for (uint8_t r = 0; r < rows; r++) {
			const float d = check[r] - rem[r];
			err2 += d * d;
			mag2 += rem[r] * rem[r];
		}

		// Deliberately loose - a well-conditioned solve lands many orders of
		// magnitude inside this, while rejecting a good solve would drop the
		// vehicle back to forward mixing, which is the failure this allocator
		// exists to avoid.
		if (err2 > AP_MOTORS_ALLOC_RESIDUAL_REL_SQ * MAX(mag2, 1.0f)) {
			_alloc_result = AllocResult::RESIDUAL;
			break;
		}

		// Clamp one motor per pass - the worst violator - not every violator.
		//
		// This is the ordinary active-set step, and the reason for it is that
		// the constraints are not independent: fixing one motor changes what
		// the others have to produce, and a motor that came out of this pass
		// below zero may well land back inside the box once a different one is
		// held.  Clamping all of them together acts on an assignment that is
		// already known to be wrong, and it is one-way - nothing here ever
		// releases a motor - so every extra clamp is a constraint the solve can
		// no longer undo.
		//
		// Three clamps is all it takes to break the solve outright: with yaw out
		// of the demand B is 3 x (free count), so once only two motors are free
		// B*B^T is 3x3 of rank at most 2, its determinant is identically zero,
		// mat_inverse() fails and the loop exits with no solution at all.  The
		// caller then falls back to forward mixing, silently.
		//
		// Measured on the counterexample from the P0 item of
		// P04先验故障注入_SITL验证与改进建议_20260905.md - a thrust set entirely
		// inside [0,1], so a feasible solution provably exists - clamping every
		// violator failed at every non-zero MOT_FAIL_YSUP including the shipping
		// default, always SINGULAR on the third pass; one at a time solves it at
		// all of them, to 1e-7.  Over 2000 randomly drawn feasible thrust sets
		// per failure position, failures at FAIL_YSUP=3 fall from 141 to 48 and
		// at 1 from 12 to 3; both are zero either way at the shipping default.
		// See libraries/AP_Motors/tests/test_alloc_feasible.cpp.
		//
		// It is not free, but nearly: the mean pass count over that same sweep
		// is unchanged to two decimals (1.07-1.21) because the overwhelming
		// majority of solves clamp nothing at all, and the worst case goes from
		// three passes to four - one more 3x3 build and inverse, on the 400 Hz
		// path.
		//
		// This narrows the gap rather than closing it.  What remains is the
		// one-way part: a motor clamped early is never released, so the
		// degeneration above is still reachable, which is what the 48 are.
		// Closing that needs a bounded solve that can release constraints, and
		// that is a structural change with its own flash and timing budget.
		bool clamped_any = false;
		int8_t worst_k = -1;
		float worst_amount = 0.0f;

		for (uint8_t k = 0; k < n; k++) {
			if (!freed[k]) {
				continue;
			}

			const float t = t_raw[k];
			thrust[k] = constrain_float(t, 0.0f, 1.0f);
			// How far outside the box, so the pick does not depend on which
			// end was crossed.
			const float amount = (t < 0.0f) ? -t : ((t > 1.0f) ? t - 1.0f : 0.0f);

			if (amount > worst_amount) {
				worst_amount = amount;
				worst_k = int8_t(k);
			}
		}

		if (worst_k >= 0) {
			freed[worst_k] = false;
			clamped_any = true;
			_alloc_clamp_mask |= (uint8_t)(1U << (idx[worst_k] & 7));
		}

		if (!clamped_any) {
			// Only a pass that needed no clamping counts as a solution.
			//
			// This is an active-set iteration: clamped_any means this round's
			// assignment is *not* usable and the next round must redistribute
			// with the clamped motors' contributions removed from the demand.
			// Setting the flag on such a round made a later degeneration commit
			// it anyway - the clamped motors would hold their clamped values
			// while the free ones still held figures computed on the assumption
			// that those same motors were putting out their unclamped, out of
			// range values.  B*t no longer equals the demand and the moment
			// balance is gone, yet the caller is told the allocation succeeded.
			//
			// It is reachable, and not by numerical accident: with yaw out of
			// the demand (rows = 3), once clamping has frozen all but two
			// motors, B is 3x2, so B*B^T is 3x3 of rank at most 2 and its
			// determinant is identically zero - mat_inverse() fails and the
			// loop breaks.  Three successive clamps in a saturated case is all
			// it takes.
			//
			// No snapshot is needed: this branch breaks immediately, so
			// thrust[] at the exit is exactly the assignment being blessed.
			have_solution = true;
			_alloc_result = AllocResult::OK;
			break;
		}
	}

	if (!have_solution) {
		if (_alloc_result == AllocResult::NOT_RUN) {
			// Loop ran out of passes with every round still clamping.
			_alloc_result = AllocResult::NO_SOLUTION;
		}

		// Never verified a solve.  thrust_out[] is still the zeroed array from
		// the top; reporting success on it commanded zero thrust on every motor
		// and dropped the vehicle.  Say so instead and let the caller fall back
		// to forward mixing, which at least flies.
		return false;
	}

	for (uint8_t k = 0; k < n; k++) {
		thrust_out[idx[k]] = constrain_float(thrust[k], 0.0f, 1.0f);
	}

	return true;
}

// Relative shortfall above which an axis counts as saturated.  The solver meets
// the demand exactly whenever nothing clamps - the residual check inside
// allocate_redistributed() guarantees that - so anything beyond arithmetic noise
// came from clamping, i.e. from running out of motor.  1% keeps float noise out
// while still catching the first real millimetre of shortfall.
#define AP_MOTORS_ALLOC_LIMIT_REL     0.01f
#define AP_MOTORS_ALLOC_LIMIT_ABS     1.0e-4f

void P04Allocator::set_limits_from_allocation(const float demand[4], bool include_yaw,
		const float thrust[MotorCount])
{
	// What the committed thrusts actually produce.  Same B as the solver used:
	// the throttle row sums the motors, the moment rows weight them by their
	// geometry.
	float achieved[4] = {};
	uint8_t n = 0;

	for (uint8_t i = 0; i < MotorCount; i++) {
		if (!motor_enabled[i]) {
			continue;
		}

		n++;
		achieved[0] += thrust[i] * _throttle_factor[i];
		achieved[1] += thrust[i] * _roll_factor[i];
		achieved[2] += thrust[i] * _pitch_factor[i];
		// Yaw uses the *geometric* factors, not _yaw_factor[]: with the default
		// MOT_FAIL_YAW=0 the live factors are scaled to zero at failure, so this
		// sum was identically zero in every degraded flight - the log said the
		// allocation produced no yaw moment while the rotors were producing
		// enough of it to sweep the nose right round. The mixer stops
		// *commanding* yaw; the rotors keep *making* it. Still a normalised
		// allocation estimate, not a measured torque.
		achieved[3] += thrust[i] * (_failed_motor >= 0 ? _yaw_geom[i] : _yaw_factor[i]);
	}

	// The throttle demand is per motor while the throttle row is a sum, the
	// same scaling allocate_redistributed() applies to rem[0].
	const float thr_demand = demand[0] * (float)n;

	// limit.roll/pitch/yaw are consumed by AC_PID::update_i(), which still lets
	// the integrator shrink when the error opposes it and only blocks growth.
	// So the honest signal is simply "this axis did not get what it asked for",
	// with no need to work out a direction here.
	const auto short_of = [](float want, float got) {
		return fabsf(want - got) > MAX(AP_MOTORS_ALLOC_LIMIT_ABS,
					       AP_MOTORS_ALLOC_LIMIT_REL * fabsf(want));
	};

	// Snapshot for the log before the flags collapse it to booleans.  The
	// demand half is taken at the call site instead, so that a failed solve
	// still records what was asked of it.
	for (uint8_t r = 0; r < 4; r++) {
		_alloc_achieved[r] = achieved[r];
	}

	limit.roll  = short_of(demand[1], achieved[1]);
	limit.pitch = short_of(demand[2], achieved[2]);

	// Yaw is deliberately absent from the demand today (see
	// allocate_redistributed), so the allocator is not even trying to track it.
	// Leaving limit.yaw as the forward mixer left it invites the yaw PID to keep
	// integrating against a demand nothing is serving, and a wound-up yaw
	// integrator eats the mixer headroom roll and pitch still need.  Report it
	// as limited for as long as yaw is out of the solve.
	//
	// Keyed off include_yaw rather than hardcoded so that if MOT_FAIL_YAW ever
	// does put yaw back into the demand (R-03), this follows automatically
	// instead of silently reporting a tracked axis as saturated.
	//
	// MOT_FAIL_YINT (2026-10-09).  The paragraph above predates the yaw target
	// projection.  With MOT_FAIL_YTRK and MOT_FAIL_YNUL set the allocation does
	// deliver yaw exactly whenever the request is reachable, and reporting it as
	// limited then stops the integrator from re-trimming: a failure on the side
	// that can still hold heading (M3 on an airframe that needs negative yaw)
	// is left carrying its steady yaw demand on the P term, which only exists
	// while there is an error.  Release the integrator, but only when every one
	// of these holds for *this* cycle's committed allocation:
	//   - the active set solved and the projection ran (not the relaxed or the
	//     redistributing path, which do not track yaw, and not a stale record);
	//   - exact-yaw mode, because the 20/80 blend of MOT_FAIL_YNUL=0 delivers
	//     only part of the target and would have the integrator chase the rest;
	//   - the request was not already cut by the forward mixer's yaw_allowed -
	//     demand[3] is the cut value, so matching it proves nothing about what
	//     the controller asked for;
	//   - the target was not clamped to an end of the reachable interval;
	//   - the thrusts really produce it.
	// Anything else stays limited.  AC_PID::update_i() still lets a limited
	// integrator shrink, so the unreachable side cannot wind up and can leave.
	bool yaw_delivered = false;

	if (!include_yaw && _fail_yaw_int != 0 &&
	    _alloc_path == AllocPath::ACTIVE_SET && _yaw_auth.valid && _yaw_auth.exact &&
	    !_yaw_auth.forward_clipped && !_yaw_auth.clipped_low && !_yaw_auth.clipped_high) {
		yaw_delivered = !short_of(demand[3], achieved[3]) && isfinite(achieved[3]);
	}

	_yaw_auth.achieved = achieved[3];
	_yaw_auth.integrator_released = yaw_delivered;
	limit.yaw = include_yaw ? short_of(demand[3], achieved[3]) : !yaw_delivered;

	// Throttle is directional: too little lift is the upper limit biting, too
	// much is the lower one.
	if (thr_demand - achieved[0] > MAX(AP_MOTORS_ALLOC_LIMIT_ABS,
					   AP_MOTORS_ALLOC_LIMIT_REL * fabsf(thr_demand))) {
		limit.throttle_upper = true;

	} else if (achieved[0] - thr_demand > MAX(AP_MOTORS_ALLOC_LIMIT_ABS,
			AP_MOTORS_ALLOC_LIMIT_REL * fabsf(thr_demand))) {
		limit.throttle_lower = true;
	}
}

void P04Allocator::set_column(unsigned i, float t, float r, float p, float y)
{
	motor_enabled[i] = true;
	_throttle_factor[i] = t;
	_roll_factor[i] = r;
	_pitch_factor[i] = p;
	_yaw_factor[i] = _yaw_geom[i] = y;
}

void P04Allocator::fail(unsigned i, float previous, uint32_t now_ms)
{
	_failed_motor = i;
	motor_enabled[i] = false;
	_spdn_start_ms = now_ms;
	_spdn_t0 = constrain_float(previous, 0.f, 1.f);
	_as_warm = false;
}

bool P04Allocator::allocate(const float request[4], float out[MotorCount], uint32_t now_ms, float dt, float yaw_rate, bool flying)
{
	if (!active()) {return false;}

	_now_ms = now_ms;
	_dt_s = dt;
	_body_yaw_rate = yaw_rate;
	_flying = flying;
	_yaw_auth = {};
	limit = {};
	limit.throttle_lower = !flying;
	_alloc_path = AllocPath::NONE;
	float d[4];
	memcpy(d, request, sizeof(d));

	if (_fail_spindown > 0.f) {
		const float resid = _spdn_t0 * expf(-float(now_ms - _spdn_start_ms) * 0.001f / _fail_spindown);

		if (resid > 0.01f) {
			d[0] -= resid * _throttle_factor[_failed_motor] / 5.f;
			d[1] -= resid * _roll_factor[_failed_motor];
			d[2] -= resid * _pitch_factor[_failed_motor];
		}
	}

	if (allocate_active_set(d, out)) {
		_alloc_path = AllocPath::ACTIVE_SET;

		if (!is_zero(_fail_yaw_track)) {apply_yaw_projection(d, out);}

	} else if (_alloc_result == AllocResult::INFEASIBLE && _fail_relax > 0.f && allocate_nearest_reachable(d, out)) {
		_alloc_path = AllocPath::RELAXED;
		_alloc_result = AllocResult::RELAXED;

	} else if (allocate_redistributed(d, false, out)) {
		_alloc_path = AllocPath::REDISTRIBUTED;

	} else {
		return false;
	}

	_yaw_auth.request_raw = _yaw_auth.request = d[3];
	set_limits_from_allocation(d, false, out);
	return true;
}

bool P04Allocator::fail_predict_gaps(const float rem[3], const uint8_t idx[5], const int8_t pair_of[],
				     float csum, float &balanced, float &automatic) const
{
	float geom[5], coef[5], dir[MotorCount];

	for (uint8_t k = 0; k < 5; k++) {
		geom[k] = _yaw_factor[idx[k]];
		coef[k] = _null_coef[pair_of[idx[k]]];
	}

	float lo, hi, vertices[2][5];

	if (!alloc_yaw_extremes(rem, idx, 5, &lo, &hi, vertices[0], vertices[1], geom) ||
	    !yaw_null_direction(idx, 5, dir)) {
		return false;
	}

	float gain = 0.0f;

	for (uint8_t k = 0; k < 5; k++) {
		gain += coef[k] * dir[k];
	}

	// yaw_null_direction normalises the largest absolute component to one.
	const bool usable = fabsf(gain) >= AP_MOTORS_NAUT_GAIN_MIN * csum;
	const float cap = constrain_float(_fail_null_max, 0.0f, 0.9f);
	const float margin = AP_MOTORS_NAUT_OUT_MARGIN * MAX(hi - lo, 1.0e-3f);
	float qbal[2], qmin = FLT_MAX, qmax = -FLT_MAX;

	for (uint8_t end = 0; end < 2; end++) {
		const float *t = vertices[end];
		float a, amin, amax;

		if (!null_line_range(t, dir, 5, a, amin, amax)) {
			return false;
		}

		float q = end == 0 ? lo : hi;

		for (uint8_t k = 0; k < 5; k++) {
			q -= coef[k] * (t[k] + a * dir[k]);
		}

		qbal[end] = q;
		const bool engage = usable && (end == 0 ? q > margin : q < -margin);
		const float q1 = engage ? q - gain * cap * (amin - a) : q;
		const float q2 = engage ? q - gain * cap * (amax - a) : q;
		qmin = MIN(qmin, MIN(q1, q2));
		qmax = MAX(qmax, MAX(q1, q2));
	}

	balanced = MAX(MIN(qbal[0], qbal[1]), -MAX(qbal[0], qbal[1]));
	automatic = MAX(qmin, -qmax);
	return isfinite(balanced) && isfinite(automatic);
}


// AP_MotorsMatrix::null_coef_check. A fresh, accepted calibration clears the latch.
void P04Allocator::reset_coefficient_check()
{
	_null_stale = false;
	_ncheck_lp_init = false;
	_ncheck_bad_s = 0.f;
	reset_prediction();
}

void P04Allocator::check_coefficients(const float thrust[MotorCount], float dt, bool steady)
{
	if (_null_stale || active()) {return;}

	const float csum = fabsf(_null_coef[0]) + fabsf(_null_coef[1]) + fabsf(_null_coef[2]);

	if (!isfinite(csum) || csum < 1.e-6f) {return;}

	if (!steady) {_ncheck_bad_s = 0.f; return;}

	float yaw = 0.f, pred = 0.f;

	for (unsigned i = 0; i < MotorCount; i++) {yaw += _yaw_factor[i] * thrust[i];}

	for (unsigned p = 0; p < 3; p++) {pred += _null_coef[p] * (thrust[p] + thrust[p + 3]);}

	if (!isfinite(yaw) || !isfinite(pred)) {return;}

	if (!_ncheck_lp_init) {_ncheck_lp_init = true; _ncheck_err_lp = yaw - pred; _ncheck_pred_lp = pred;}

	const float k = constrain_float(dt / 3.f, 0.f, 1.f);
	_ncheck_err_lp += k * ((yaw - pred) - _ncheck_err_lp);
	_ncheck_pred_lp += k * (pred - _ncheck_pred_lp);
	const float limit_err = MAX(0.5f * fabsf(_ncheck_pred_lp), 0.02f);
	_ncheck_bad_s = (fabsf(_ncheck_err_lp) > limit_err) ? (_ncheck_bad_s + dt) : 0.f;

	if (_ncheck_bad_s >= 20.f) {_null_stale = true; reset_prediction();}
}

void P04Allocator::reset_prediction()
{
	_predict_s = 0.f;
	_predicted = false;
}

bool P04Allocator::predict(const float thrust[MotorCount], float dt, Prediction &result)
{
	if (_predicted || active()) {return false;}

	float yaw = 0.f, max_yaw = 0.f;

	for (unsigned i = 0; i < MotorCount; i++) {
		yaw += _yaw_factor[i] * thrust[i];
		max_yaw += fabsf(_yaw_factor[i]) * thrust[i];
	}

	if (!isfinite(yaw) || !(max_yaw > 1e-3f)) {return false;}

	if (is_zero(_predict_s)) {
		memcpy(_predict_t, thrust, sizeof(_predict_t));
		_predict_yaw = yaw;
		_predict_max = max_yaw;
	}

	const float k = constrain_float(dt / 3.f, 0.f, 1.f);
	_predict_yaw += k * (yaw - _predict_yaw);
	_predict_max += k * (max_yaw - _predict_max);

	for (unsigned i = 0; i < MotorCount; i++) {_predict_t[i] += k * (thrust[i] - _predict_t[i]);}

	_predict_s += dt;

	if (_predict_s < 15.f) {return false;}

	_predicted = true;
	result = {};
	result.trim = _predict_yaw / _predict_max;
	const float csum = fabsf(_null_coef[0]) + fabsf(_null_coef[1]) + fabsf(_null_coef[2]);
	result.from_model = !_null_stale && isfinite(csum) && csum >= 1e-6f;
	float rem[3] {};

	for (unsigned i = 0; i < MotorCount; i++) {
		rem[0] += _throttle_factor[i] * _predict_t[i];
		rem[1] += _roll_factor[i] * _predict_t[i];
		rem[2] += _pitch_factor[i] * _predict_t[i];
	}

	for (unsigned j = 0; j < MotorCount; j++) {
		bool risk = false;

		if (result.from_model) {
			uint8_t idx[5], n = 0;

			for (uint8_t i = 0; i < MotorCount; i++) {if (i != j) {idx[n++] = i;}}

			if (!fail_predict_gaps(rem, idx, _pair_of, csum, result.balanced[j], result.automatic[j])) {
				result.balanced[j] = result.automatic[j] = 1.f;
			}

			risk = (_fail_null_auto ? result.automatic[j] : result.balanced[j]) > 0.02f * _predict_max;

		} else {
			risk = fabsf(result.trim) >= 0.1f && _yaw_factor[j] * result.trim > 0.f;
		}

		if (risk) {result.risk_mask |= 1u << j;}
	}

	return true;
}
