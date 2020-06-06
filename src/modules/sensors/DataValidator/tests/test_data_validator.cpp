/****************************************************************************
 *
 *   Copyright (c) 2019 Todd Stellanova. All rights reserved.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be  used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 * @file test_data_validator.cpp
 * Testing the DataValidator class
 *
 * @author Todd Stellanova
 */

#include <gtest/gtest.h>

#include <stdint.h>
#include <cassert>
#include <cstdlib>
#include <stdio.h>
#include <math.h>

#include <DataValidator.hpp>

class DataValidatorTest : public ::testing::Test
{
public:
	DataValidator _validator;
};

TEST_F(DataValidatorTest, init)
{
	uint64_t fake_timestamp = 666;
	const uint32_t timeout_usec = 2000;//from original private value

	// initially there should be no siblings
	ASSERT_TRUE(nullptr == _validator.sibling());
	// initially we should have zero confidence
	ASSERT_TRUE(0.0f == _validator.confidence(fake_timestamp));
	// initially the error count should be zero
	ASSERT_TRUE(0 == _validator.error_count());
	// initially unused
	ASSERT_TRUE(!_validator.used());
	// initially no priority
	ASSERT_TRUE(0 == _validator.priority());
	_validator.set_timeout(timeout_usec);
	ASSERT_TRUE(_validator.get_timeout() == timeout_usec);


	DataValidator *sibling_validator = new DataValidator;
	_validator.setSibling(sibling_validator);
	ASSERT_TRUE(sibling_validator == _validator.sibling());

	//verify that with no data, confidence is zero and error mask is set
	ASSERT_TRUE(0.0f == _validator.confidence(fake_timestamp + 1));
	uint32_t state = _validator.state();
	ASSERT_TRUE(DataValidator::ERROR_FLAG_NO_DATA == (DataValidator::ERROR_FLAG_NO_DATA & state));

	//verify that calling print doesn't crash tests
	_validator.print();
}

static void dump_validator_state(DataValidator &validator)
{
	uint32_t state = validator. state();
	printf("state: 0x%x no_data: %d stale: %d timeout:%d\n",
	       validator.state(),
	       DataValidator::ERROR_FLAG_NO_DATA & state,
	       DataValidator::ERROR_FLAG_STALE_DATA & state,
	       DataValidator::ERROR_FLAG_TIMEOUT & state
	      );
	validator.print();
	printf("\n");
}

static void fill_validator_with_samples(DataValidator *validator, const float incr_value, float *value_io,
					uint64_t *timestamp_io)
{
	uint64_t timestamp = *timestamp_io;
	const uint64_t timestamp_incr = 5; //usec
	const uint32_t timeout_usec = 2000;//derived from class-private value

	float val = *value_io;
	const uint64_t error_count = 0;
	const int priority = 50; //"medium" priority
	const int equal_value_count = 100; //default is private VALUE_EQUAL_COUNT_DEFAULT

	validator->set_equal_value_threshold(equal_value_count);
	validator->set_timeout(timeout_usec);

	//put a bunch of values that are all different
	for (int i = 0; i < equal_value_count; i++, val += incr_value) {
		timestamp += timestamp_incr;
		validator->put(timestamp, val, error_count, priority);
	}

	*timestamp_io = timestamp;
	*value_io = val;
}

TEST_F(DataValidatorTest, put)
{
	uint64_t timestamp = 500;
	const uint32_t timeout_usec = 2000; //derived from class-private value
	float val = 3.14159f;
	//derived from class-private value: this is min change needed to avoid stale detection
	const float sufficient_incr_value = (1.1f * 1E-6f);

	fill_validator_with_samples(&_validator, sufficient_incr_value, &val, &timestamp);

	ASSERT_TRUE(_validator.used());
	//verify that the last value we inserted is the current validator value
	float last_val = val - sufficient_incr_value;
	ASSERT_FLOAT_EQ(_validator.value()[0], last_val);

	// we've just provided a bunch of valid data: should be fully confident
	float conf = _validator.confidence(timestamp);

	if (1.0f != conf) {
		printf("conf: %f\n", (double)conf);
		dump_validator_state(_validator);
	}

	ASSERT_TRUE(1.0f == conf);
	// should be no errors
	ASSERT_TRUE(0 == _validator.state());

	//now check confidence much beyond the timeout window-- should timeout
	conf = _validator.confidence(timestamp + (1.1 * timeout_usec));

	if (0.0f != conf) {
		printf("conf: %f\n", (double)conf);
		dump_validator_state(_validator);
	}

	ASSERT_TRUE(0.0f == conf);
	ASSERT_TRUE(DataValidator::ERROR_FLAG_TIMEOUT == (DataValidator::ERROR_FLAG_TIMEOUT & _validator.state()));
}

/**
 * Verify that the DataValidator detects sensor data that does not vary sufficiently
 */
TEST_F(DataValidatorTest, stale_detector)
{
	uint64_t timestamp = 500;
	float val = 3.14159f;
	//derived from class-private value, this is insufficient to avoid stale detection:
	const float insufficient_incr_value = (0.99f * 1e-6f);

	fill_validator_with_samples(&_validator, insufficient_incr_value, &val, &timestamp);

	// data is stale: should have no confidence
	ASSERT_TRUE(0.0f == _validator.confidence(timestamp));

	// should be a stale error
	uint32_t state = _validator.state();

	if (DataValidator::ERROR_FLAG_STALE_DATA != state) {
		dump_validator_state(_validator);
	}

	ASSERT_TRUE(DataValidator::ERROR_FLAG_STALE_DATA == (DataValidator::ERROR_FLAG_STALE_DATA & state));
}

static void insert_values_around_mean(DataValidator &validator, const float mean, uint32_t count, float *rms_err,
				      uint64_t *timestamp_io)
{
	uint64_t timestamp = *timestamp_io;
	uint64_t timestamp_incr = 5;
	const uint64_t error_count = 0;
	const int priority = 50;
	const float swing = 1E-2f;
	float sum_dev_squares = 0;

	//insert a series of values that swing around the mean
	for (uint32_t i = 0; i < count;  i++) {
		float iter_swing = (0 == (i % 2)) ? swing : -swing;
		float iter_val = mean + iter_swing;
		float iter_dev = iter_val - mean;
		sum_dev_squares += (iter_dev * iter_dev);
		timestamp += timestamp_incr;
		validator.put(timestamp, iter_val, error_count, priority);
	}

	float rms = sqrtf(sum_dev_squares / count);
	//note: this should be approximately equal to "swing"
	*rms_err = (float)rms;
	*timestamp_io = timestamp;
}

/**
 * Verify the RMS error calculated by the DataValidator for a series of samples
 */
TEST_F(DataValidatorTest, rms_calculation)
{
	const int equal_value_count = 100; //default is private VALUE_EQUAL_COUNT_DEFAULT
	const float mean_value = 3.14159f;
	const uint32_t sample_count = 1000;
	float expected_rms_err = 0.0f;
	uint64_t timestamp = 500;

	_validator.set_equal_value_threshold(equal_value_count);

	insert_values_around_mean(_validator, mean_value, sample_count, &expected_rms_err, &timestamp);
	float *rms = _validator.rms();
	ASSERT_TRUE(nullptr != rms);
	float calc_rms_err = rms[0];
	float diff = fabsf(calc_rms_err - expected_rms_err);
	float diff_frac = (diff / expected_rms_err);
	printf("rms: %f expect: %f diff: %f frac: %f\n", (double)calc_rms_err, (double)expected_rms_err,
	       (double)diff, (double)diff_frac);
	ASSERT_TRUE(diff_frac < 0.03f);
}

/**
 * Verify error tracking performed by DataValidator::put
 */
TEST_F(DataValidatorTest, error_tracking)
{
	uint64_t timestamp = 500;
	uint64_t timestamp_incr = 5;
	const uint32_t timeout_usec = 2000;//from original private value
	float val = 3.14159f;
	uint64_t error_count = 0;
	int expected_error_density = 0;
	int priority = 50;
	//from private value: this is min change needed to avoid stale detection
	const float sufficient_incr_value = (1.1f * 1E-6f);
	//default is private VALUE_EQUAL_COUNT_DEFAULT
	const int equal_value_count = 50000;
	//should be less than equal_value_count: ensure this is less than NORETURN_ERRCOUNT
	const int total_iterations = 1000;

	_validator.set_timeout(timeout_usec);
	_validator.set_equal_value_threshold(equal_value_count);

	//put a bunch of values that are all different
	for (int i = 0; i < total_iterations;  i++, val += sufficient_incr_value) {
		timestamp += timestamp_incr;

		//up to a 50% random error rate appears to pass the error density filter
		if ((((float)rand() / (float)RAND_MAX)) < 0.500f) {
			error_count += 1;
			expected_error_density += 1;

		} else if (expected_error_density > 0) {
			expected_error_density -= 1;
		}

		_validator.put(timestamp, val, error_count, priority);
	}

	ASSERT_TRUE(_validator.used());
	//at this point, error_count should be less than NORETURN_ERRCOUNT
	ASSERT_TRUE(_validator.error_count() == error_count);

	// we've just provided a bunch of valid data with some errors:
	// confidence should be reduced by the number of errors
	float conf = _validator.confidence(timestamp);
	printf("error_count: %u validator confidence: %f\n", (uint32_t)error_count, (double)conf);
	ASSERT_TRUE(1.0f != conf);  //we should not be fully confident
	ASSERT_TRUE(0.0f != conf);  //neither should we be completely unconfident
	// should be no errors, even if confidence is reduced, since we didn't exceed NORETURN_ERRCOUNT
	ASSERT_TRUE(0 == _validator.state());

	// the error density will reduce the confidence by 1 - (error_density / ERROR_DENSITY_WINDOW)
	// ERROR_DENSITY_WINDOW is currently private, but == 100.0f
	float reduced_conf = 1.0f - ((float)expected_error_density / 100.0f);
	float diff = fabsf(reduced_conf - conf);

	ASSERT_FLOAT_EQ(reduced_conf, conf);
	//if (reduced_conf != conf) {
	//	printf("conf: %f reduced_conf: %f diff: %f\n", (double)conf, (double)reduced_conf, diff);
	//	dump_validator_state(validator);
	//}

	ASSERT_TRUE(diff < 1E-6f);

	//Now, insert a series of errors and ensure we trip the error detector
	for (int i = 0; i < 250;  i++, val += sufficient_incr_value) {
		timestamp += timestamp_incr;
		//100% error rate
		error_count += 1;
		expected_error_density += 1;
		_validator.put(timestamp, val, error_count, priority);
	}

	conf = _validator.confidence(timestamp);
	ASSERT_TRUE(0.0f == conf);  // should we be completely unconfident
	// we should have triggered the high error density detector
	ASSERT_TRUE(DataValidator::ERROR_FLAG_HIGH_ERRDENSITY == (DataValidator::ERROR_FLAG_HIGH_ERRDENSITY &
			_validator.state()));


	_validator.reset_state();

	//Now insert so many errors that we exceed private NORETURN_ERRCOUNT
	for (int i = 0; i < 10000;  i++, val += sufficient_incr_value) {
		timestamp += timestamp_incr;
		//100% error rate
		error_count += 1;
		expected_error_density += 1;
		_validator.put(timestamp, val, error_count, priority);
	}

	conf = _validator.confidence(timestamp);
	ASSERT_TRUE(0.0f == conf);  // should we be completely unconfident
	// we should have triggered the high error count detector
	ASSERT_TRUE(DataValidator::ERROR_FLAG_HIGH_ERRCOUNT == (DataValidator::ERROR_FLAG_HIGH_ERRCOUNT & _validator.state()));
}
