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

// AP_MotorsMatrix::null_seq_accumulate and null_seq_fit, revision 942f63eb72.
#pragma once
#include <stdint.h>

class P04Identification
{
public:
	enum class Result {Rejected, Effect, Equivalent};
	void accumulate(unsigned stage, float dt, const float pairs[3], float yaw_row, float transfer);
	Result fit(float c[3]) const;
private:
	float _sum[8][5] {}, _valid_s[8] {};
	uint16_t _count[8] {};
};
