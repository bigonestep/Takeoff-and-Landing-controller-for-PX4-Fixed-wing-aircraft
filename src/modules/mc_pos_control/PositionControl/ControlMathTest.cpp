/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <ControlMath.hpp>
#include <px4_defines.h>

using namespace matrix;

TEST(ControlMathTest, TiltLimitUnchanged)
{
	Vector3f body = Vector3f(0, .1f, 1).normalized();
	const Vector3f body_before = body;
	ControlMath::limitTilt(body, Vector3f(0, 0, 1), M_DEG_TO_RAD_F * 45.f);
	EXPECT_EQ(body, body_before);
}

TEST(ControlMathTest, TiltLimit45degree)
{
	Vector3f body = Vector3f(1, 0, 0);
	ControlMath::limitTilt(body, Vector3f(0, 0, 1), M_DEG_TO_RAD_F * 45.f);
	EXPECT_EQ(body, Vector3f(M_SQRT1_2_F, 0, M_SQRT1_2_F));

	body = Vector3f(0, 1, 0);
	ControlMath::limitTilt(body, Vector3f(0, 0, 1), M_DEG_TO_RAD_F * 45.f);
	EXPECT_EQ(body, Vector3f(0, M_SQRT1_2_F, M_SQRT1_2_F));
}

TEST(ControlMathTest, TiltLimit10degree)
{
	Vector3f body = Vector3f(1, 1, .1f).normalized();
	ControlMath::limitTilt(body, Vector3f(0, 0, 1), M_DEG_TO_RAD_F * 10.f);
	float angle = acosf(body.dot(Vector3f(0, 0, 1)));
	EXPECT_NEAR(angle * M_RAD_TO_DEG_F, 10.f, 1e-4f);
	EXPECT_FLOAT_EQ(body.length(), 1.f);
	EXPECT_FLOAT_EQ(body(0), body(1));

	body = Vector3f(1, 2, .2f);
	ControlMath::limitTilt(body, Vector3f(0, 0, 1), M_DEG_TO_RAD_F * 10.f);
	angle = acosf(body.dot(Vector3f(0, 0, 1)));
	EXPECT_NEAR(angle * M_RAD_TO_DEG_F, 10.f, 1e-4f);
	EXPECT_FLOAT_EQ(body.length(), 1.f);
	EXPECT_FLOAT_EQ(2.f * body(0), body(1));
}
