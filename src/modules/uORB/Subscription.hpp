/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file Subscription.hpp
 *
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/uORBTopics.hpp>

#include <px4_platform_common/defines.h>

#include "uORBDeviceNode.hpp"
#include "uORBManager.hpp"
#include "uORBUtils.hpp"

namespace uORB
{
class SubscriptionBase
{
public:
	~SubscriptionBase()
	{
		unsubscribe();
	}

	bool subscribe()
	{
		// check if already subscribed
		if (_node != nullptr) {
			return true;
		}

		if (_orb_id != ORB_ID::INVALID) {

			DeviceMaster *device_master = uORB::Manager::get_instance()->get_device_master();

			if (device_master != nullptr) {

				if (!device_master->deviceNodeExists(_orb_id, _instance)) {
					return false;
				}

				uORB::DeviceNode *node = device_master->getDeviceNode(get_topic(), _instance);

				if (node != nullptr) {
					_node = node;
					_node->add_internal_subscriber();

					// If there were any previous publications, allow the subscriber to read them
					const unsigned curr_gen = _node->published_message_count();
					const uint8_t q_size = _node->get_queue_size();

					if (q_size < curr_gen) {
						_last_generation = curr_gen - q_size;

					} else {
						_last_generation = 0;
					}

					return true;
				}
			}
		}

		return false;
	}

	void unsubscribe()
	{
		if (_node != nullptr) {
			_node->remove_internal_subscriber();
		}

		_node = nullptr;
		_last_generation = 0;
	}

	bool valid() const { return _node != nullptr; }
	bool advertised()
	{
		if (valid()) {
			return _node->is_advertised();
		}

		// try to initialize
		if (subscribe()) {
			// check again if valid
			if (valid()) {
				return _node->is_advertised();
			}
		}

		return false;
	}

	/**
	 * Check if there is a new update.
	 * */
	bool updated() { return advertised() ? (_node->published_message_count() != _last_generation) : false; }

	uint8_t  get_instance() const { return _instance; }
	orb_id_t get_topic() const { return get_orb_meta(_orb_id); }
	ORB_PRIO get_priority() { return advertised() ? _node->get_priority() : ORB_PRIO_UNINITIALIZED; }

protected:

	/**
	 * Constructor
	 *
	 * @param id The uORB ORB_ID enum for the topic.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionBase(ORB_ID id, uint8_t instance = 0) :
		_orb_id(id),
		_instance(instance)
	{
	}

	DeviceNode *get_node() { return _node; }

	DeviceNode *_node{nullptr};
	unsigned _last_generation{0}; /**< last generation the subscriber has seen */
	ORB_ID _orb_id{ORB_ID::INVALID};
	uint8_t _instance{0};
};

class SubscriptionCallback;

// Base subscription wrapper class
template<typename T>
class Subscriber : public SubscriptionBase
{
public:

	/**
	 * Constructor
	 *
	 * @param id The uORB ORB_ID enum for the topic.
	 * @param instance The instance for multi sub.
	 */
	Subscriber(ORB_ID id, uint8_t instance = 0) :
		SubscriptionBase(id, instance)
	{
		//static_assert(sizeof(T) == )
	}

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param instance The instance for multi sub.
	 */
	Subscriber(const orb_metadata *meta, uint8_t instance = 0) :
		SubscriptionBase((meta == nullptr) ? ORB_ID::INVALID : static_cast<ORB_ID>(meta->o_id), instance)
	{
	}

	/**
	 * Update the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool update(T *dst) { return updated() ? copy(dst) : false; }

	/**
	 * Copy the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool copy(T *dst) { return advertised() ? _node->copy(dst, _last_generation) : false; }

protected:

	friend class SubscriptionCallback;

};

using Subscription = Subscriber<void>;

// Subscription wrapper class with data
template<class T>
class SubscriptionData : public Subscriber<T>
{
public:
	/**
	 * Constructor
	 *
	 * @param id The uORB metadata ORB_ID enum for the topic.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionData(ORB_ID id, uint8_t instance = 0) :
		Subscriber<T>(id, instance)
	{
		copy(&_data);
	}

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionData(const orb_metadata *meta, uint8_t instance = 0) :
		Subscriber<T>(meta, instance)
	{
		Subscriber<T>::copy(&_data);
	}

	~SubscriptionData() = default;

	// no copy, assignment, move, move assignment
	SubscriptionData(const SubscriptionData &) = delete;
	SubscriptionData &operator=(const SubscriptionData &) = delete;
	SubscriptionData(SubscriptionData &&) = delete;
	SubscriptionData &operator=(SubscriptionData &&) = delete;

	// update the embedded struct.
	bool update() { return Subscriber<T>::update(&_data); }

	const T &get() const { return _data; }

private:

	T _data{};
};

} // namespace uORB
