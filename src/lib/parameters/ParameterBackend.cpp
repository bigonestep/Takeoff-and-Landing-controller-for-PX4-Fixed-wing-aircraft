/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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

#include "ParameterBackend.hpp"

#include <parameters/px4_parameters.h>

#include <crc32.h>
#include <float.h>
#include <math.h>

#if defined(FLASH_BASED_PARAMS)
#include "flashparams/flashparams.h"
static const char *param_default_file = nullptr; // nullptr means to store to FLASH
#else
inline static int flash_param_save(bool only_unsaved) { return -1; }
inline static int flash_param_load() { return -1; }
inline static int flash_param_import() { return -1; }
static const char *param_default_file = PX4_ROOTFSDIR"/eeprom/parameters";
#endif


#ifdef __PX4_QURT
#define PARAM_OPEN	px4_open
#define PARAM_CLOSE	px4_close
#else
#define PARAM_OPEN	open
#define PARAM_CLOSE	close
#endif

/**
 * Array of static parameter info.
 */
static const param_info_s *param_info_base = (const param_info_s *) &px4_parameters;

ParameterBackend::ParameterBackend() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	px4_sem_init(&_param_sem, 0, 1);
	px4_sem_init(&_param_sem_save, 0, 1);
	px4_sem_init(&_reader_lock_holders_lock, 0, 1);
}

ParameterBackend::~ParameterBackend()
{
	px4_sem_destroy(&_param_sem);
	px4_sem_destroy(&_param_sem_save);
	px4_sem_destroy(&_reader_lock_holders_lock);

	perf_free(_export_perf);
	perf_free(_find_perf);
	perf_free(_get_perf);
	perf_free(_set_perf);
}

unsigned ParameterBackend::get_param_info_count()
{
	/* Singleton creation of and array of bits to track changed values */
	if (!_param_changed_storage) {
		/* Note that we have a (highly unlikely) race condition here: in the worst case the allocation is done twice */
		_size_param_changed_storage_bytes = (px4_parameters.param_count / bits_per_allocation_unit) + 1;
		_param_changed_storage = (uint8_t *)calloc(_size_param_changed_storage_bytes, 1);

		/* If the allocation fails we need to indicate failure in the
		 * API by returning PARAM_INVALID
		 */
		if (_param_changed_storage == nullptr) {
			return 0;
		}
	}

	return px4_parameters.param_count;
}

/** lock the parameter store for read access */
void ParameterBackend::lock_reader()
{
	do {} while (px4_sem_wait(&_reader_lock_holders_lock) != 0);

	++_reader_lock_holders;

	if (_reader_lock_holders == 1) {
		// the first reader takes the lock, the next ones are allowed to just continue
		do {} while (px4_sem_wait(&_param_sem) != 0);
	}

	px4_sem_post(&_reader_lock_holders_lock);
}

/** lock the parameter store for write access */
void ParameterBackend::lock_writer()
{
	do {} while (px4_sem_wait(&_param_sem) != 0);
}

/** unlock the parameter store */
void ParameterBackend::unlock_reader()
{
	do {} while (px4_sem_wait(&_reader_lock_holders_lock) != 0);

	--_reader_lock_holders;

	if (_reader_lock_holders == 0) {
		// the last reader releases the lock
		px4_sem_post(&_param_sem);
	}

	px4_sem_post(&_reader_lock_holders_lock);
}

/** unlock the parameter store */
void ParameterBackend::unlock_writer()
{
	px4_sem_post(&_param_sem);
}

/** assert that the parameter store is locked */
static void param_assert_locked()
{
	/* XXX */
}

bool ParameterBackend::handle_in_range(param_t param)
{
	unsigned count = get_param_info_count();
	return (count && param < count);
}

/**
 * Compare two modifid parameter structures to determine ordering.
 *
 * This function is suitable for passing to qsort or bsearch.
 */
static int compare_values(const void *a, const void *b)
{
	struct param_wbuf_s *pa = (struct param_wbuf_s *)a;
	struct param_wbuf_s *pb = (struct param_wbuf_s *)b;

	if (pa->param < pb->param) {
		return -1;
	}

	if (pa->param > pb->param) {
		return 1;
	}

	return 0;
}

struct param_wbuf_s *ParameterBackend::find_changed(param_t param)
{
	param_wbuf_s *s = nullptr;

	param_assert_locked();

	if (param_values != nullptr) {
		param_wbuf_s key{};
		key.param = param;
		s = (param_wbuf_s *)utarray_find(param_values, &key, compare_values);
	}

	return s;
}

void ParameterBackend::notifyChanges()
{
	parameter_update_s pup{};
	pup.timestamp = hrt_absolute_time();
	pup.instance = param_instance++;

	/*
	 * If we don't have a handle to our topic, create one now; otherwise
	 * just publish.
	 */
	if (param_topic == nullptr) {
		param_topic = orb_advertise(ORB_ID(parameter_update), &pup);

	} else {
		orb_publish(ORB_ID(parameter_update), param_topic, &pup);
	}
}

param_t ParameterBackend::findParameter(const char *name, bool notification)
{
	perf_begin(_find_perf);

	param_t middle;
	param_t front = 0;
	param_t last = get_param_info_count();

	/* perform a binary search of the known parameters */

	while (front <= last) {
		middle = front + (last - front) / 2;
		int ret = strcmp(name, param_info_base[middle].name);

		if (ret == 0) {
			if (notification) {
				setParameterUsed(middle);
			}

			perf_end(_find_perf);
			return middle;

		} else if (middle == front) {
			/* An end point has been hit, but there has been no match */
			break;

		} else if (ret < 0) {
			last = middle;

		} else {
			front = middle;
		}
	}

	perf_end(_find_perf);

	/* not found */
	return PARAM_INVALID;
}

uint16_t ParameterBackend::count()
{
	return get_param_info_count();
}

uint16_t ParameterBackend::count_used()
{
	unsigned count = 0;

	// ensure the allocation has been done
	if (get_param_info_count()) {
		for (int i = 0; i < _size_param_changed_storage_bytes; i++) {
			for (int j = 0; j < bits_per_allocation_unit; j++) {
				if (_param_changed_storage[i] & (1 << j)) {
					count++;
				}
			}
		}
	}

	return count;
}

param_t ParameterBackend::for_index(unsigned index)
{
	unsigned count = get_param_info_count();

	if (count && index < count) {
		return (param_t)index;
	}

	return PARAM_INVALID;
}

param_t ParameterBackend::for_used_index(unsigned index)
{
	int count = get_param_info_count();

	if (count && (int)index < count) {
		/* walk all params and count used params */
		unsigned used_count = 0;

		for (int i = 0; i < _size_param_changed_storage_bytes; i++) {
			for (int j = 0; j < bits_per_allocation_unit; j++) {
				if (_param_changed_storage[i] & (1 << j)) {

					/* we found the right used count,
					 * return the param value
					 */
					if (index == used_count) {
						return (param_t)(i * bits_per_allocation_unit + j);
					}

					used_count++;
				}
			}
		}
	}

	return PARAM_INVALID;
}

int ParameterBackend::getParameterIndex(param_t param)
{
	if (handle_in_range(param)) {
		return (unsigned)param;
	}

	return -1;
}

int ParameterBackend::getParameterUsedIndex(param_t param)
{
	/* this tests for out of bounds and does a constant time lookup */
	if (!isParameterUsed(param)) {
		return -1;
	}

	/* walk all params and count, now knowing that it has a valid index */
	int used_count = 0;

	for (int i = 0; i < _size_param_changed_storage_bytes; i++) {
		for (int j = 0; j < bits_per_allocation_unit; j++) {
			if (_param_changed_storage[i] & (1 << j)) {

				if ((int)param == i * bits_per_allocation_unit + j) {
					return used_count;
				}

				used_count++;
			}
		}
	}

	return -1;
}

const char *ParameterBackend::getParameterName(param_t param)
{
	return handle_in_range(param) ? param_info_base[param].name : nullptr;
}

bool ParameterBackend::isParameterVolatile(param_t param)
{
	return handle_in_range(param) ? param_info_base[param].volatile_param : false;
}

bool ParameterBackend::isParameterValueIsDefault(param_t param)
{
	lock_reader();
	struct param_wbuf_s *s = find_changed(param);
	unlock_reader();
	return s == nullptr;
}

bool ParameterBackend::isParameterValueUnsaved(param_t param)
{
	lock_reader();
	struct param_wbuf_s *s = find_changed(param);
	bool ret = s && s->unsaved;
	unlock_reader();
	return ret;
}

param_type_t ParameterBackend::getParameterType(param_t param)
{
	return handle_in_range(param) ? param_info_base[param].type : PARAM_TYPE_UNKNOWN;
}

size_t ParameterBackend::getParameterSize(param_t param)
{
	if (handle_in_range(param)) {
		switch (param_type(param)) {
		case PARAM_TYPE_INT32:
		case PARAM_TYPE_FLOAT:
			return 4;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			/* decode structure size from type value */
			return param_type(param) - PARAM_TYPE_STRUCT;
		}
	}

	return 0;
}

const void *ParameterBackend::getParameterValuePointer(param_t param)
{
	const void *result = nullptr;

	param_assert_locked();

	if (handle_in_range(param)) {

		const union param_value_u *v;

		/* work out whether we're fetching the default or a written value */
		struct param_wbuf_s *s = find_changed(param);

		if (s != nullptr) {
			v = &s->val;

		} else {
			v = &param_info_base[param].val;
		}

		if (param_type(param) >= PARAM_TYPE_STRUCT &&
		    param_type(param) <= PARAM_TYPE_STRUCT_MAX) {

			result = v->p;

		} else {
			result = v;
		}
	}

	return result;
}

int ParameterBackend::getParameterValue(param_t param, void *val)
{
	int result = -1;

	lock_reader();
	perf_begin(_get_perf);

	const void *v = getParameterValuePointer(param);

	if (val && v) {
		memcpy(val, v, param_size(param));
		result = 0;
	}

	perf_end(_get_perf);
	unlock_reader();

	return result;
}

void ParameterBackend::Run()
{
	bool disabled = false;

	if (!getDefaultFile()) {
		// In case we save to FLASH, defer param writes until disarmed,
		// as writing to FLASH can stall the entire CPU (in rare cases around 300ms on STM32F7)
		uORB::SubscriptionData<actuator_armed_s> armed_sub{ORB_ID(actuator_armed)};

		if (armed_sub.get().armed) {
			ScheduleDelayed(1);
			return;
		}
	}

	lock_writer();
	_last_autosave_timestamp = hrt_absolute_time();
	_autosave_scheduled = false;
	disabled = _autosave_disabled;
	unlock_writer();

	if (disabled) {
		return;
	}

	PX4_DEBUG("Autosaving params");
	int ret = saveDefault();

	if (ret != 0) {
		PX4_ERR("param auto save failed (%i)", ret);
	}
}

void ParameterBackend::autosave()
{
	if (_autosave_scheduled || _autosave_disabled) {
		return;
	}

	// wait at least 300ms before saving, because:
	// - tasks often call param_set() for multiple params, so this avoids unnecessary save calls
	// - the logger stores changed params. He gets notified on a param change via uORB and then
	//   looks at all unsaved params.
	hrt_abstime delay = 300_ms;

	static constexpr const hrt_abstime rate_limit = 2_s; // rate-limit saving to 2 seconds
	const hrt_abstime last_save_elapsed = hrt_elapsed_time(&_last_autosave_timestamp);

	if (last_save_elapsed < rate_limit && rate_limit > last_save_elapsed + delay) {
		delay = rate_limit - last_save_elapsed;
	}

	_autosave_scheduled = true;
	ScheduleDelayed(delay);
}

void ParameterBackend::control_autosave(bool enable)
{
	lock_writer();

	if (!enable && _autosave_scheduled) {
		ScheduleClear();
		_autosave_scheduled = false;
	}

	_autosave_disabled = !enable;
	unlock_writer();
}

int ParameterBackend::setParameter(param_t param, const void *val, bool mark_saved, bool notify_changes)
{
	int result = -1;
	bool params_changed = false;

	lock_writer();
	perf_begin(_set_perf);

	if (param_values == nullptr) {
		utarray_new(param_values, &param_icd);
	}

	if (param_values == nullptr) {
		PX4_ERR("failed to allocate modified values array");
		goto out;
	}

	if (handle_in_range(param)) {

		param_wbuf_s *s = find_changed(param);

		if (s == nullptr) {

			/* construct a new parameter */
			param_wbuf_s buf = {};
			buf.param = param;

			params_changed = true;

			/* add it to the array and sort */
			utarray_push_back(param_values, &buf);
			utarray_sort(param_values, compare_values);

			/* find it after sorting */
			s = find_changed(param);
		}

		/* update the changed value */
		switch (param_type(param)) {

		case PARAM_TYPE_INT32:
			params_changed = params_changed || s->val.i != *(int32_t *)val;
			s->val.i = *(int32_t *)val;
			break;

		case PARAM_TYPE_FLOAT:
			params_changed = params_changed || fabsf(s->val.f - * (float *)val) > FLT_EPSILON;
			s->val.f = *(float *)val;
			break;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX:
			if (s->val.p == nullptr) {
				size_t psize = param_size(param);

				if (psize > 0) {
					s->val.p = malloc(psize);

				} else {
					s->val.p = nullptr;
				}

				if (s->val.p == nullptr) {
					PX4_ERR("failed to allocate parameter storage");
					goto out;
				}
			}

			memcpy(s->val.p, val, param_size(param));
			params_changed = true;
			break;

		default:
			goto out;
		}

		s->unsaved = !mark_saved;
		result = 0;

		if (!mark_saved) { // this is false when importing parameters
			autosave();
		}
	}

out:
	perf_end(_set_perf);
	unlock_writer();

	/*
	 * If we set something, now that we have unlocked, go ahead and advertise that
	 * a thing has been set.
	 */
	if (params_changed && notify_changes) {
		notifyChanges();
	}

	return result;
}

bool ParameterBackend::isParameterUsed(param_t param)
{
	int param_index = getParameterIndex(param);

	if (param_index < 0) {
		return false;
	}

	return _param_changed_storage[param_index / bits_per_allocation_unit] &
	       (1 << param_index % bits_per_allocation_unit);
}

void ParameterBackend::setParameterUsed(param_t param)
{
	int param_index = getParameterIndex(param);

	if (param_index < 0) {
		return;
	}

	// FIXME: this needs locking too
	_param_changed_storage[param_index / bits_per_allocation_unit] |=
		(1 << param_index % bits_per_allocation_unit);
}

int ParameterBackend::resetParameter(param_t param)
{
	param_wbuf_s *s = nullptr;
	bool param_found = false;

	lock_writer();

	if (handle_in_range(param)) {

		/* look for a saved value */
		s = find_changed(param);

		/* if we found one, erase it */
		if (s != nullptr) {
			int pos = utarray_eltidx(param_values, s);
			utarray_erase(param_values, pos, 1);
		}

		param_found = true;
	}

	autosave();

	unlock_writer();

	if (s != nullptr) {
		notifyChanges();
	}

	return (!param_found);
}

void ParameterBackend::resetAll(bool auto_save)
{
	lock_writer();

	if (param_values != nullptr) {
		utarray_free(param_values);
	}

	/* mark as reset / deleted */
	param_values = nullptr;

	if (auto_save) {
		autosave();
	}

	unlock_writer();

	notifyChanges();
}

void ParameterBackend::resetAllExclude(const char *excludes[], int num_excludes)
{
	for (param_t param = 0; handle_in_range(param); param++) {
		const char *name = getParameterName(param);
		bool exclude = false;

		for (int index = 0; index < num_excludes; index ++) {
			int len = strlen(excludes[index]);

			if ((excludes[index][len - 1] == '*'
			     && strncmp(name, excludes[index], len - 1) == 0)
			    || strcmp(name, excludes[index]) == 0) {

				exclude = true;
				break;
			}
		}

		if (!exclude) {
			resetParameter(param);
		}
	}

	notifyChanges();
}

int ParameterBackend::setDefaultFile(const char *filename)
{
#ifdef FLASH_BASED_PARAMS
	// the default for flash-based params is always the FLASH
	(void)filename;
#else

	if (_param_user_file != nullptr) {
		// we assume this is not in use by some other thread
		free(_param_user_file);
		_param_user_file = nullptr;
	}

	if (filename) {
		_param_user_file = strdup(filename);
	}

#endif /* FLASH_BASED_PARAMS */

	return 0;
}

const char *ParameterBackend::getDefaultFile()
{
	return (_param_user_file != nullptr) ? _param_user_file : param_default_file;
}

int ParameterBackend::saveDefault()
{
	int res = PX4_ERROR;

	const char *filename = getDefaultFile();

	if (!filename) {
		perf_begin(_export_perf);
		lock_writer();
		res = flash_param_save(false);
		unlock_writer();
		perf_end(_export_perf);
		return res;
	}

	/* write parameters to temp file */
	int fd = PARAM_OPEN(filename, O_WRONLY | O_CREAT, PX4_O_MODE_666);

	if (fd < 0) {
		PX4_ERR("failed to open param file: %s", filename);
		return PX4_ERROR;
	}

	int attempts = 5;

	while (res != OK && attempts > 0) {
		res = exportAll(fd, false);
		attempts--;

		if (res != PX4_OK) {
			PX4_ERR("param_export failed, retrying %d", attempts);
			lseek(fd, 0, SEEK_SET); // jump back to the beginning of the file
		}
	}

	if (res != OK) {
		PX4_ERR("failed to write parameters to file: %s", filename);
	}

	PARAM_CLOSE(fd);

	return res;
}

int ParameterBackend::loadDefault()
{
	int res = 0;
	const char *filename = getDefaultFile();

	if (!filename) {
		return flash_param_load();
	}

	int fd_load = PARAM_OPEN(filename, O_RDONLY);

	if (fd_load < 0) {
		/* no parameter file is OK, otherwise this is an error */
		if (errno != ENOENT) {
			PX4_ERR("open '%s' for reading failed", filename);
			return -1;
		}

		return 1;
	}

	int result = loadAll(fd_load);
	PARAM_CLOSE(fd_load);

	if (result != 0) {
		PX4_ERR("error reading parameters from '%s'", filename);
		return -2;
	}

	return res;
}

int ParameterBackend::exportAll(int fd, bool only_unsaved)
{
	int result = -1;
	perf_begin(_export_perf);

	if (fd < 0) {
		lock_writer();
		// flash_param_save() will take the shutdown lock
		result = flash_param_save(only_unsaved);
		unlock_writer();
		perf_end(_export_perf);
		return result;
	}

	param_wbuf_s *s = nullptr;
	struct bson_encoder_s encoder;

	int shutdown_lock_ret = px4_shutdown_lock();

	if (shutdown_lock_ret) {
		PX4_ERR("px4_shutdown_lock() failed (%i)", shutdown_lock_ret);
	}

	// take the file lock
	do {} while (px4_sem_wait(&_param_sem_save) != 0);

	lock_reader();

	uint8_t bson_buffer[256];
	bson_encoder_init_buf_file(&encoder, fd, &bson_buffer, sizeof(bson_buffer));

	/* no modified parameters -> we are done */
	if (param_values == nullptr) {
		result = 0;
		goto out;
	}

	while ((s = (struct param_wbuf_s *)utarray_next(param_values, s)) != nullptr) {
		/*
		 * If we are only saving values changed since last save, and this
		 * one hasn't, then skip it
		 */
		if (only_unsaved && !s->unsaved) {
			continue;
		}

		s->unsaved = false;

		const char *name = getParameterName(s->param);
		const size_t size = getParameterSize(s->param);

		/* append the appropriate BSON type object */
		switch (getParameterType(s->param)) {

		case PARAM_TYPE_INT32: {
				const int32_t i = s->val.i;

				PX4_DEBUG("exporting: %s (%d) size: %d val: %d", name, s->param, size, i);

				if (bson_encoder_append_int(&encoder, name, i)) {
					PX4_ERR("BSON append failed for '%s'", name);
					goto out;
				}
			}
			break;

		case PARAM_TYPE_FLOAT: {
				const double f = (double)s->val.f;

				PX4_DEBUG("exporting: %s (%d) size: %d val: %.3f", name, s->param, size, (double)f);

				if (bson_encoder_append_double(&encoder, name, f)) {
					PX4_ERR("BSON append failed for '%s'", name);
					goto out;
				}
			}
			break;

		case PARAM_TYPE_STRUCT ... PARAM_TYPE_STRUCT_MAX: {
				const void *value_ptr = getParameterValuePointer(s->param);

				/* lock as short as possible */
				if (bson_encoder_append_binary(&encoder,
							       name,
							       BSON_BIN_BINARY,
							       size,
							       value_ptr)) {

					PX4_ERR("BSON append failed for '%s'", name);
					goto out;
				}
			}
			break;

		default:
			PX4_ERR("unrecognized parameter type");
			goto out;
		}
	}

	result = 0;

out:

	if (result == 0) {
		if (bson_encoder_fini(&encoder) != PX4_OK) {
			PX4_ERR("bson encoder finish failed");
		}
	}

	unlock_reader();

	px4_sem_post(&_param_sem_save);

	if (shutdown_lock_ret == 0) {
		px4_shutdown_unlock();
	}

	perf_end(_export_perf);

	return result;
}

int ParameterBackend::import(int fd)
{
	if (fd < 0) {
		return flash_param_import();
	}

	return import_internal(fd, false);
}

struct param_import_state {
	ParameterBackend *backend{nullptr};
	bool mark_saved{true};
};

int ParameterBackend::import_callback_trampoline(bson_decoder_t decoder, void *priv, bson_node_t node)
{
	param_import_state *state = (param_import_state *)priv;

	if (state->backend != nullptr) {
		return state->backend->import_callback(decoder, priv, node);
	}

	return -1;
}

int ParameterBackend::import_callback(bson_decoder_t decoder, void *priv, bson_node_t node)
{
	float f = 0.0f;
	int32_t i = 0;
	void *tmp = nullptr;
	void *v = nullptr;
	int result = -1;
	param_import_state *state = (param_import_state *)priv;

	/*
	 * EOO means the end of the parameter object. (Currently not supporting
	 * nested BSON objects).
	 */
	if (node->type == BSON_EOO) {
		PX4_DEBUG("end of parameters");
		return 0;
	}

	/*
	 * Find the parameter this node represents.  If we don't know it,
	 * ignore the node.
	 */
	param_t param = findParameter(node->name, false);

	if (param == PARAM_INVALID) {
		PX4_ERR("ignoring unrecognised parameter '%s'", node->name);
		return 1;
	}

	/*
	 * Handle setting the parameter from the node
	 */

	switch (node->type) {
	case BSON_INT32: {
			if (getParameterType(param) != PARAM_TYPE_INT32) {
				PX4_WARN("unexpected type for %s", node->name);
				result = 1; // just skip this entry
				goto out;
			}

			i = node->i;
			v = &i;

			PX4_DEBUG("Imported %s with value %d", getParameterName(param), i);
		}
		break;

	case BSON_DOUBLE: {
			if (getParameterType(param) != PARAM_TYPE_FLOAT) {
				PX4_WARN("unexpected type for %s", node->name);
				result = 1; // just skip this entry
				goto out;
			}

			f = node->d;
			v = &f;

			PX4_DEBUG("Imported %s with value %f", getParameterName(param), (double)f);
		}
		break;

	case BSON_BINDATA: {
			if (node->subtype != BSON_BIN_BINARY) {
				PX4_WARN("unexpected subtype for %s", node->name);
				result = 1; // just skip this entry
				goto out;
			}

			if (bson_decoder_data_pending(decoder) != getParameterSize(param)) {
				PX4_WARN("bad size for '%s'", node->name);
				result = 1; // just skip this entry
				goto out;
			}

			/* XXX check actual file data size? */
			size_t psize = getParameterSize(param);

			if (psize > 0) {
				tmp = malloc(psize);

			} else {
				tmp = nullptr;
			}

			if (tmp == nullptr) {
				PX4_ERR("failed allocating for '%s'", node->name);
				goto out;
			}

			if (bson_decoder_copy_data(decoder, tmp)) {
				PX4_ERR("failed copying data for '%s'", node->name);
				goto out;
			}

			v = tmp;
		}
		break;

	default:
		PX4_DEBUG("unrecognised node type");
		goto out;
	}

	if (setParameter(param, v, state->mark_saved, true)) {
		PX4_DEBUG("error setting value for '%s'", node->name);
		goto out;
	}

	if (tmp != nullptr) {
		free(tmp);
		tmp = nullptr;
	}

	/* don't return zero, that means EOF */
	result = 1;

out:

	if (tmp != nullptr) {
		free(tmp);
	}

	return result;
}

int ParameterBackend::import_internal(int fd, bool mark_saved)
{
	bson_decoder_s decoder;
	param_import_state state;
	int result = -1;

	if (bson_decoder_init_file(&decoder, fd, import_callback_trampoline, &state)) {
		PX4_ERR("decoder init failed");
		return PX4_ERROR;
	}

	state.backend = this;
	state.mark_saved = mark_saved;

	do {
		result = bson_decoder_next(&decoder);

	} while (result > 0);

	return result;
}

int ParameterBackend::loadAll(int fd)
{
	if (fd < 0) {
		return flash_param_load();
	}

	resetAll(false);
	return import_internal(fd, true);
}

void ParameterBackend::param_foreach(void (*func)(void *arg, param_t param), void *arg, bool only_changed,
				     bool only_used)
{
	for (param_t param = 0; handle_in_range(param); param++) {

		/* if requested, skip unchanged values */
		if (only_changed && (find_changed(param) == nullptr)) {
			continue;
		}

		if (only_used && !isParameterUsed(param)) {
			continue;
		}

		func(arg, param);
	}
}

uint32_t ParameterBackend::hash_check()
{
	uint32_t param_hash = 0;

	lock_reader();

	/* compute the CRC32 over all string param names and 4 byte values */
	for (param_t param = 0; handle_in_range(param); param++) {
		if (!isParameterUsed(param) || isParameterVolatile(param)) {
			continue;
		}

		const char *name = getParameterName(param);
		const void *val = getParameterValuePointer(param);
		param_hash = crc32part((const uint8_t *)name, strlen(name), param_hash);
		param_hash = crc32part((const uint8_t *)val, getParameterSize(param), param_hash);
	}

	unlock_reader();

	return param_hash;
}

void ParameterBackend::print_status()
{
	PX4_INFO("summary: %d/%d (used/total)", count_used(), count());

#ifndef FLASH_BASED_PARAMS
	const char *filename = getDefaultFile();

	if (filename != nullptr) {
		PX4_INFO("file: %s", getDefaultFile());
	}

#endif /* FLASH_BASED_PARAMS */

	if (param_values != nullptr) {
		PX4_INFO("storage array: %d/%d elements (%zu bytes total)",
			 utarray_len(param_values), param_values->n, param_values->n * sizeof(UT_icd));
	}

	PX4_INFO("auto save: %s", _autosave_disabled ? "off" : "on");

	if (!_autosave_disabled && (_last_autosave_timestamp > 0)) {
		PX4_INFO("last auto save: %.3f seconds ago", hrt_elapsed_time(&_last_autosave_timestamp) * 1e-6);
	}

	perf_print_counter(_export_perf);
	perf_print_counter(_find_perf);
	perf_print_counter(_get_perf);
	perf_print_counter(_set_perf);
}
