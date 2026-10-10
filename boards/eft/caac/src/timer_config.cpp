/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#include <px4_arch/io_timer_hw_description.h>

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer1, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer2, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer3, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer4, DMA{DMA::Index1}),
};

/* PX4 requires the channels of one timer to be consecutive, so the output order
 * differs from the ArduPilot hwdef PWM(n) numbering. Column "AP" is the EFT_CAAC
 * (ArduPilot) PWM number of the same physical pin.
 */
constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel1}, {GPIO::PortA, GPIO::Pin8}),  // PX4 1 = PA8 (AP PWM1)
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortE, GPIO::Pin11}),  // PX4 2 = PE11 (AP PWM6)
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {GPIO::PortE, GPIO::Pin13}),  // PX4 3 = PE13 (AP PWM5)
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel4}, {GPIO::PortE, GPIO::Pin14}),  // PX4 4 = PE14 (AP PWM4)
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel2}, {GPIO::PortB, GPIO::Pin3}),  // PX4 5 = PB3 (AP PWM10)
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel3}, {GPIO::PortB, GPIO::Pin10}),  // PX4 6 = PB10 (AP PWM3)
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel4}, {GPIO::PortB, GPIO::Pin11}),  // PX4 7 = PB11 (AP PWM2)
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel1}, {GPIO::PortB, GPIO::Pin4}),  // PX4 8 = PB4 (AP PWM9)
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel3}, {GPIO::PortB, GPIO::Pin0}),  // PX4 9 = PB0 (AP PWM8)
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel4}, {GPIO::PortB, GPIO::Pin1}),  // PX4 10 = PB1 (AP PWM7)
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),  // PX4 11 = PD14 (AP PWM12)
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel4}, {GPIO::PortD, GPIO::Pin15}),  // PX4 12 = PD15 (AP PWM11)
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
