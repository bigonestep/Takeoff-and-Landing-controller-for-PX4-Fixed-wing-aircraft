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
 * @file test_data_validator_group.cpp
 * Testing the DataValidatorGroup class
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
#include <DataValidatorGroup.hpp>

const uint32_t base_timeout_usec = 2000; // from original private value
static constexpr int equal_value_count = 100; // default is private VALUE_EQUAL_COUNT_DEFAULT
const uint64_t base_timestamp = 666;
const unsigned base_num_siblings = 4;

class DataValidatorGroupTest : public ::testing::Test
{
public:
	DataValidatorGroup _validator_group{4};
};

/**
 * Initialize a DataValidatorGroup with some common settings;
 * @param sibling_count (out) the number of siblings initialized
 */
TEST_F(DataValidatorGroupTest, setup)
{
	//verify that calling print doesn't crash the tests
	_validator_group.print();
	printf("\n");

	//should be no failovers yet
	ASSERT_TRUE(0 == _validator_group.failover_count());
	ASSERT_TRUE(DataValidator::ERROR_FLAG_NO_ERROR == _validator_group.failover_state());
	ASSERT_TRUE(-1 == _validator_group.failover_index());

	//this sets the timeout on all current members of the group, as well as members added later
	_validator_group.set_timeout(base_timeout_usec);
	//the following sets the threshold on all CURRENT members of the group, but not any added later
	//TODO This is likely a bug in DataValidatorGroup
	_validator_group.set_equal_value_threshold(equal_value_count);
}

/**
 * Fill one DataValidator with samples, by index.
 *
 * @param group
 * @param val1_idx Index of the validator to fill with samples
 * @param num_samples
 */
// void fill_one_with_valid_data(DataValidatorGroup *group, int val1_idx,  uint32_t num_samples)
// {
// 	uint64_t timestamp = base_timestamp;
// 	uint64_t error_count = 0;
// 	float last_best_val = 0.0f;

// 	for (uint32_t i = 0; i < num_samples; i++) {
// 		float val = ((float) rand() / (float) RAND_MAX);
// 		float data[DataValidator::dimensions] = {val};
// 		_validator_group.put(val1_idx, timestamp, data, error_count, 100);
// 		last_best_val = val;
// 	}

// 	int best_idx = 0;
// 	float *best_data = _validator_group.get_best(timestamp, &best_idx);
// 	ASSERT_TRUE(last_best_val == best_data[0]);
// 	ASSERT_TRUE(best_idx == val1_idx);
// }

/**
 * Fill two validators in the group with samples, by index.
 * Both validators will be filled with the same data, but
 * the priority of the first validator will be higher than the second.
 *
 * @param group
 * @param val1_idx index of the first validator to fill
 * @param val2_idx index of the second validator to fill
 * @param num_samples
 */
static void fill_two_with_valid_data(DataValidatorGroup *group, int val1_idx, int val2_idx, uint32_t num_samples)
{
	uint64_t timestamp = base_timestamp;
	uint64_t error_count = 0;
	float last_best_val = 0.0f;

	for (uint32_t i = 0; i < num_samples; i++) {
		float val = ((float) rand() / (float) RAND_MAX);
		float data[DataValidator::dimensions] = {val};
		//two sensors with identical values, but different priorities
		group->put(val1_idx, timestamp, data, error_count, 100);
		group->put(val2_idx, timestamp, data, error_count, 10);
		last_best_val = val;
	}

	int best_idx = 0;
	float *best_data = group->get_best(timestamp, &best_idx);
	ASSERT_FLOAT_EQ(last_best_val, best_data[0]);
	ASSERT_TRUE(best_idx == val1_idx);
}

TEST_F(DataValidatorGroupTest, init)
{
	//should not yet be any best value
	int best_index = -1;
	ASSERT_TRUE(nullptr == _validator_group.get_best(base_timestamp, &best_index));
}

/**
 * Happy path test of put method -- ensure the "best" sensor selected is the one with highest priority
 */
TEST_F(DataValidatorGroupTest, put)
{
	unsigned num_siblings = 2;

	uint64_t timestamp = base_timestamp;

	//now we add validators
	DataValidator *validator1 = _validator_group.add_new_validator();
	ASSERT_TRUE(validator1->get_timeout() == base_timeout_usec);
	validator1->set_equal_value_threshold(equal_value_count);

	DataValidator *validator2 = _validator_group.add_new_validator();
	ASSERT_TRUE(validator2->get_timeout() == base_timeout_usec);
	validator2->set_equal_value_threshold(equal_value_count);

	unsigned val1_idx = num_siblings - 2;
	unsigned val2_idx = num_siblings - 1;

	fill_two_with_valid_data(&_validator_group, val1_idx, val2_idx, 500);
	int best_idx = -1;
	float *best_data = _validator_group.get_best(timestamp, &best_idx);
	ASSERT_TRUE(nullptr != best_data);
	float best_val = best_data[0];

	float *cur_val1 = validator1->value();
	ASSERT_TRUE(nullptr != cur_val1);
	//printf("cur_val1 %p \n", cur_val1);
	ASSERT_FLOAT_EQ(best_val, cur_val1[0]);

	float *cur_val2 = validator2->value();
	ASSERT_TRUE(nullptr != cur_val2);
	//printf("cur_val12 %p \n", cur_val2);
	ASSERT_FLOAT_EQ(best_val, cur_val2[0]);
}

/**
 * Verify that the DataValidatorGroup will select the sensor with the latest higher priority as "best".
 */
TEST_F(DataValidatorGroupTest, priority_switch)
{
	unsigned num_siblings = 2;

	DataValidator *validator1 = _validator_group.add_new_validator();
	ASSERT_TRUE(validator1->get_timeout() == base_timeout_usec);
	validator1->set_equal_value_threshold(equal_value_count);

	DataValidator *validator2 = _validator_group.add_new_validator();
	ASSERT_TRUE(validator2->get_timeout() == base_timeout_usec);
	validator2->set_equal_value_threshold(equal_value_count);

	uint64_t timestamp = base_timestamp;

	//printf("num_siblings: %d \n",num_siblings);
	int val1_idx = (int)num_siblings - 2;
	int val2_idx = (int)num_siblings - 1;
	uint64_t error_count = 0;

	fill_two_with_valid_data(&_validator_group, val1_idx, val2_idx, 100);

	int best_idx = -1;
	float *best_data = nullptr;
	//now, switch the priorities, which switches "best" but doesn't trigger a failover
	float new_best_val = 3.14159f;
	float data[DataValidator::dimensions] = {new_best_val};
	//a single sample insertion should be sufficient to trigger a priority switch
	_validator_group.put(val1_idx, timestamp, data, error_count, 1);
	_validator_group.put(val2_idx, timestamp, data, error_count, 100);
	best_data = _validator_group.get_best(timestamp, &best_idx);
	ASSERT_FLOAT_EQ(new_best_val, best_data[0]);
	//the new best sensor should now be the sensor with the higher priority
	ASSERT_TRUE(best_idx == val2_idx);
	//should not have detected a real failover
	ASSERT_TRUE(0 == _validator_group.failover_count());
}

/**
 * Verify that the DataGroupValidator will prefer a sensor with no errors over a sensor with high errors
 */
TEST_F(DataValidatorGroupTest, simple_failover)
{
	unsigned num_siblings = 2;

	uint64_t timestamp = base_timestamp;

	//now we add validators
	DataValidator *validator1 = _validator_group.add_new_validator();
	ASSERT_TRUE(validator1->get_timeout() == base_timeout_usec);
	validator1->set_equal_value_threshold(equal_value_count);

	DataValidator *validator2 = _validator_group.add_new_validator();
	ASSERT_TRUE(validator2->get_timeout() == base_timeout_usec);
	validator2->set_equal_value_threshold(equal_value_count);

	//printf("num_siblings: %d \n",num_siblings);
	int val1_idx = (int)num_siblings - 2;
	int val2_idx = (int)num_siblings - 1;

	fill_two_with_valid_data(&_validator_group, val1_idx, val2_idx, 100);

	int best_idx = -1;
	float *best_data = nullptr;

	//trigger a real failover
	float new_best_val = 3.14159f;
	float data[DataValidator::dimensions] = {new_best_val};
	//trigger a bunch of errors on the previous best sensor
	unsigned val1_err_count = 0;

	for (int i = 0; i < 25; i++) {
		_validator_group.put(val1_idx, timestamp, data, ++val1_err_count, 100);
		_validator_group.put(val2_idx, timestamp, data, 0, 10);
	}

	ASSERT_TRUE(validator1->error_count() == val1_err_count);

	//since validator1 is experiencing errors, we should see a failover to validator2
	best_data = _validator_group.get_best(timestamp + 1, &best_idx);
	ASSERT_TRUE(nullptr != best_data);
	ASSERT_FLOAT_EQ(new_best_val, best_data[0]);
	ASSERT_TRUE(best_idx == val2_idx);
	//should have detected a real failover
	printf("failover_count: %d \n", _validator_group.failover_count());
	ASSERT_TRUE(1 == _validator_group.failover_count());

	//even though validator1 has encountered a bunch of errors, it hasn't failed
	ASSERT_TRUE(DataValidator::ERROR_FLAG_NO_ERROR == validator1->state());

	// although we failed over from one sensor to another, this is not the same thing tracked by failover_index
	int fail_idx = _validator_group.failover_index();
	ASSERT_TRUE(-1 == fail_idx);//no failed sensor

	//since no sensor has actually hard-failed, the group failover state is NO_ERROR
	ASSERT_TRUE(DataValidator::ERROR_FLAG_NO_ERROR == _validator_group.failover_state());
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

/**
 * Force once sensor to fail and ensure that we detect it
 */
TEST_F(DataValidatorGroupTest, sensor_failure)
{
	unsigned num_siblings = 0;
	uint64_t timestamp = base_timestamp;
	const float sufficient_incr_value = (1.1f * 1E-6f);
	const uint32_t timeout_usec = 2000;//derived from class-private value

	float val = 3.14159f;

	//now we add validators
	DataValidator *validator = _validator_group.add_new_validator();
	//verify the previously set timeout applies to the new group member
	ASSERT_TRUE(validator->get_timeout() == base_timeout_usec);
	//for testing purposes, ensure this newly added member is consistent with the rest of the group
	//TODO this is likely a bug in DataValidatorGroup
	validator->set_equal_value_threshold(equal_value_count);

	ASSERT_TRUE(nullptr != validator);
	num_siblings++;
	int val_idx = num_siblings - 1;

	fill_validator_with_samples(validator,  sufficient_incr_value, &val, &timestamp);
	//the best should now be the one validator we've filled with samples

	int best_idx = -1;
	float *best_data = _validator_group.get_best(timestamp, &best_idx);
	ASSERT_TRUE(nullptr != best_data);
	//printf("best_idx: %d val_idx: %d\n", best_idx, val_idx);
	ASSERT_TRUE(best_idx == val_idx);

	//now force a timeout failure in the one validator, by checking confidence long past timeout
	validator->confidence(timestamp + (1.1 * timeout_usec));
	ASSERT_TRUE(DataValidator::ERROR_FLAG_TIMEOUT == (DataValidator::ERROR_FLAG_TIMEOUT & validator->state()));

	//now that the one sensor has failed, the group should detect this as well
	int fail_idx = _validator_group.failover_index();
	ASSERT_TRUE(val_idx == fail_idx);
}
