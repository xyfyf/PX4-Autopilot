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

#include "P04Identification.hpp"
#include <float.h>
#include <math.h>

void P04Identification::accumulate(unsigned stage, float dt, const float pairs[3], float yaw, float transfer)
{
	if (stage >= 8 || _count[stage] == UINT16_MAX || !isfinite(yaw) || !(dt > 0.f)) {return;}

	for (unsigned p = 0; p < 3; p++) {_sum[stage][p] += pairs[p];}

	_sum[stage][3] += yaw;
	_sum[stage][4] += transfer;
	_count[stage]++;
	_valid_s[stage] += dt;
}

P04Identification::Result P04Identification::fit(float c[3]) const
{
	const uint8_t nseg = 8;
	// Draft calibration in normalized pair-thrust and allocator yaw units.
	float min_transfer = FLT_MAX;
	constexpr float min_pair_transfer = 0.05f * (2.0f * 0.08f);
	constexpr float negligible_slope = 0.02f;
	constexpr float simultaneous_t = 3.535f; // two-sided Bonferroni 95%, 3 contrasts, 8-3 df
	float X[8][3], y[8];

	for (uint8_t k = 0; k < nseg; k++) {
		if (_count[k] < 200 || _valid_s[k] < 5.f) {
			return Result::Rejected;
		}

		const float inv = 1.0f / _count[k];

		for (uint8_t p = 0; p < 3; p++) {
			X[k][p] = _sum[k][p] * inv;
		}

		y[k] = _sum[k][3] * inv;

		if (k > 0 && k + 1 < nseg) {
			const float transfer = _sum[k][4] * inv * ((k & 1) ? 1.0f : -1.0f);
			min_transfer = fminf(min_transfer, transfer);

			if (!isfinite(transfer) || !(min_transfer >= min_pair_transfer)) {
				return Result::Rejected;
			}
		}
	}

	// 正规方程 A c = b
	double A[3][3] {}, b[3] {};

	for (uint8_t k = 0; k < nseg; k++) {
		for (uint8_t i = 0; i < 3; i++) {
			b[i] += double(X[k][i]) * double(y[k]);

			for (uint8_t j = 0; j < 3; j++) {
				A[i][j] += double(X[k][i]) * double(X[k][j]);
			}
		}
	}

	// Symmetric adjugate. Reuse it for both the fit and contrast standard errors.
	double adj[3][3];

	for (uint8_t i = 0; i < 3; i++) {
		const uint8_t p = (i + 1) % 3, q = (i + 2) % 3;

		for (uint8_t j = 0; j < 3; j++) {
			const uint8_t r = (j + 1) % 3, s = (j + 2) % 3;
			adj[i][j] = A[p][r] * A[q][s] - A[p][s] * A[q][r];
		}
	}

	const double det = A[0][0] * adj[0][0] + A[0][1] * adj[0][1] + A[0][2] * adj[0][2];
	const double diag = A[0][0] * A[1][1] * A[2][2];

	if (!isfinite(det) || !(diag > 0) || det / diag < double(1.0e-6f)) {
		return Result::Rejected;
	}

	for (uint8_t i = 0; i < 3; i++) {
		c[i] = float((adj[i][0] * b[0] + adj[i][1] * b[1] + adj[i][2] * b[2]) / det);
	}

	float ymin = y[0], ymax = y[0], r2 = 0.0f;

	for (uint8_t k = 0; k < nseg; k++) {
		ymin = fminf(ymin, y[k]);
		ymax = fmaxf(ymax, y[k]);
		const float e = y[k] - (c[0] * X[k][0] + c[1] * X[k][1] + c[2] * X[k][2]);
		r2 += e * e;
	}

	const float span = ymax - ymin;
	const float resid = sqrtf(r2 / nseg);

	if (!isfinite(c[0]) || !isfinite(c[1]) || !isfinite(c[2]) || !isfinite(resid)) {
		return Result::Rejected;
	}

	if (resid > fmaxf(1.0e-6f, 0.3f * span)) {
		return Result::Rejected;
	}

	if (fabsf(y[0] - y[nseg - 1]) > fmaxf(1.0e-6f, 0.5f * span)) {
		return Result::Rejected;
	}

	float widths[3];
	bool effect = false, equivalent = true;

	for (uint8_t i = 0; i < 3; i++) {
		const uint8_t j = (i + 1) % 3;
		const float slope = fabsf(c[i] - c[j]);
		const double variance_factor = (adj[i][i] + adj[j][j] - 2 * adj[i][j]) / det;

		if (!isfinite(variance_factor) || !(variance_factor > 0)) {
			return Result::Rejected;
		}

		widths[i] = simultaneous_t *sqrtf((r2 / (nseg - 3)) * float(variance_factor));
		effect |= slope - widths[i] > negligible_slope;
		equivalent &= slope + widths[i] <= negligible_slope;
	}

	if (!effect && !equivalent) {return Result::Rejected;}

	if (equivalent) {
		c[0] = c[1] = c[2] = 0.f;
		return Result::Equivalent;
	}

	return Result::Effect;
}
