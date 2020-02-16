/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include "param.h"

#include "tinybson/tinybson.h"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/uthash/utarray.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

/**
 * Storage for modified parameters.
 */
struct param_wbuf_s {
	union param_value_u	val;
	param_t			param;
	bool			unsaved;
};

class ParameterBackend : public px4::ScheduledWorkItem
{
public:
	ParameterBackend();
	~ParameterBackend() override;

	/**
	 * Look up a parameter by name.
	 *
	 * @param name		The canonical name of the parameter being looked up.
	 * @return		A handle to the parameter, or PARAM_INVALID if the parameter does not exist.
	 *			This call will also set the parameter as "used" in the system, which is used
	*			to e.g. show the parameter via the RC interface
	*/
	param_t findParameter(const char *name, bool notification);

	/**
	 * Wether a parameter is in use in the system.
	 *
	 * @return		True if it has been written or read
	 */
	bool isParameterUsed(param_t param);

	/**
	 * Mark a parameter as used. Only marked parameters will be sent to a GCS.
	 * A call to param_find() will mark a param as used as well.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 */
	void setParameterUsed(param_t param);

	uint16_t count();
	uint16_t count_used();

	param_t for_index(unsigned index);
	param_t for_used_index(unsigned index);

	/**
	 * Look up the index of a parameter.
	 *
	 * @param param		The parameter to obtain the index for.
	 * @return		The index, or -1 if the parameter does not exist.
	 */
	int getParameterIndex(param_t param);

	/**
	 * Look up the index of an used parameter.
	 *
	 * @param param		The parameter to obtain the index for.
	 * @return		The index of the parameter in use, or -1 if the parameter does not exist.
	 */
	int getParameterUsedIndex(param_t param);

	/**
	 * Obtain the name of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		The name assigned to the parameter, or NULL if the handle is invalid.
	 */
	const char *getParameterName(param_t param);

	/**
	 * Obtain the volatile state of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return			true if the parameter is volatile
	 */
	bool isParameterVolatile(param_t param);

	/**
	 * Test whether a parameter's value has changed from the default.
	 *
	 * @return		If true, the parameter's value has not been changed from the default.
	 */
	bool isParameterValueIsDefault(param_t param);

	/**
	 * Test whether a parameter's value has been changed but not saved.
	 *
	 * @return		If true, the parameter's value has not been saved.
	 */
	bool isParameterValueUnsaved(param_t param);

	/**
	 * Obtain the type of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		The type assigned to the parameter.
	 */
	param_type_t getParameterType(param_t param);

	/**
	 * Determine the size of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		The size of the parameter's value.
	 */
	size_t getParameterSize(param_t param);

	/**
	 * Copy the value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param val		Where to return the value, assumed to point to suitable storage for the parameter type.
	 *			For structures, a bitwise copy of the structure is performed to this address.
	* @return		Zero if the parameter's value could be returned, nonzero otherwise.
	*/
	int getParameterValue(param_t param, void *val);

	/**
	 * Enable/disable the param autosaving.
	 * Re-enabling with changed params will not cause an autosave.
	 * @param enable true: enable autosaving, false: disable autosaving
	 */
	void control_autosave(bool enable);


	int setParameter(param_t param, const void *val, bool mark_saved, bool notify_changes);

	/**
	 * Reset a parameter to its default value.
	 *
	 * This function frees any storage used by struct parameters, and returns the parameter
	 * to its default value.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		Zero on success, nonzero on failure
	 */
	int resetParameter(param_t param);

	/**
	 * Reset all parameters to their default values.
	 *
	 * This function also releases the storage used by struct parameters.
	 */
	void resetAll(bool auto_save);

	/**
	 * Reset all parameters to their default values except for excluded parameters.
	 *
	 * This function also releases the storage used by struct parameters.
	 *
	 * @param excludes			Array of param names to exclude from resetting. Use a wildcard
	 *							at the end to exclude parameters with a certain prefix.
	* @param num_excludes		The number of excludes provided.
	*/
	void resetAllExclude(const char *excludes[], int num_excludes);

	/**
	 * Set the default parameter file name.
	 * This has no effect if the FLASH-based storage is enabled.
	 *
	 * @param filename	Path to the default parameter file.  The file is not required to
	 *			exist.
	* @return		Zero on success.
	*/
	int setDefaultFile(const char *filename);

	/**
	 * Get the default parameter file name.
	 *
	 * @return		The path to the current default parameter file; either as
	 *			a result of a call to param_set_default_file, or the
	*			built-in default.
	*/
	const char *getDefaultFile();

	/**
	 * Save parameters to the default file.
	 * Note: this method requires a large amount of stack size!
	 *
	 * This function saves all parameters with non-default values.
	 *
	 * @return		Zero on success.
	 */
	int saveDefault();

	/**
	 * @return 0 on success, 1 if all params have not yet been stored, -1 if device open failed, -2 if writing parameters failed
	 */
	int loadDefault();

	/**
	 * Export changed parameters to a file.
	 * Note: this method requires a large amount of stack size!
	 *
	 * @param fd		File descriptor to export to (-1 selects the FLASH storage).
	 * @param only_unsaved	Only export changed parameters that have not yet been exported.
	 * @return		Zero on success, nonzero on failure.
	 */
	int exportAll(int fd, bool only_unsaved);

	/**
	 * Import parameters from a file, discarding any unrecognized parameters.
	 *
	 * This function merges the imported parameters with the current parameter set.
	 *
	 * @param fd		File descriptor to import from (-1 selects the FLASH storage).
	 * @return		Zero on success, nonzero if an error occurred during import.
	 *			Note that in the failure case, parameters may be inconsistent.
	*/
	int import(int fd);

	/**
	 * Load parameters from a file.
	 *
	 * This function resets all parameters to their default values, then loads new
	 * values from a file.
	 *
	 * @param fd		File descriptor to import from (-1 selects the FLASH storage).
	 * @return		Zero on success, nonzero if an error occurred during import.
	 *			Note that in the failure case, parameters may be inconsistent.
	*/
	int loadAll(int fd);

	/**
	 * Apply a function to each parameter.
	 *
	 * Note that the parameter set is not locked during the traversal. It also does
	 * not hold an internal state, so the callback function can block or sleep between
	 * parameter callbacks.
	 *
	 * @param func		The function to invoke for each parameter.
	 * @param arg		Argument passed to the function.
	 * @param only_changed	If true, the function is only called for parameters whose values have
	 *			been changed from the default.
	* @param only_used	If true, the function is only called for parameters which have been
	*			used in one of the running applications.
	*/
	void param_foreach(void (*func)(void *arg, param_t param), void *arg, bool only_changed, bool only_used);

	/**
	 * Generate the hash of all parameters and their values
	 *
	 * @return		CRC32 hash of all param_ids and values
	 */
	uint32_t hash_check();

	/**
	 * Notify the system about parameter changes. Can be used for example after several calls to
	 * param_set_no_notification() to avoid unnecessary system notifications.
	 */
	void notifyChanges();

	void print_status();

private:

	void Run() override;

	unsigned get_param_info_count();

	void lock_reader();
	void unlock_reader();

	void lock_writer();
	void unlock_writer();

	int import_internal(int fd, bool mark_saved);
	static int import_callback_trampoline(bson_decoder_t decoder, void *priv, bson_node_t node);
	int import_callback(bson_decoder_t decoder, void *priv, bson_node_t node);

	/**
	 * Obtain a pointer to the storage allocated for a parameter.
	 *
	 * @param param			The parameter whose storage is sought.
	 * @return			A pointer to the parameter value, or nullptr
	 *				if the parameter does not exist.
	*/
	const void *getParameterValuePointer(param_t param);

	/**
	 * Test whether a param_t is value.
	 *
	 * @param param			The parameter handle to test.
	 * @return			True if the handle is valid.
	 */
	bool handle_in_range(param_t param);

	/**
	 * Automatically save the parameters after a timeout and limited rate.
	 *
	 * This needs to be called with the writer lock held (it's not necessary that it's the writer lock, but it
	 * needs to be the same lock as autosave_worker() and param_control_autosave() use).
	 */
	void autosave();

	/**
	 * Locate the modified parameter structure for a parameter, if it exists.
	 *
	 * @param param			The parameter being searched.
	 * @return			The structure holding the modified value, or
	 *				nullptr if the parameter has not been modified.
	*/
	struct param_wbuf_s *find_changed(param_t param);

	/** flexible array holding modified parameter values */
	UT_array *param_values{nullptr};

	/** array info for the modified parameters array */
	const UT_icd param_icd = {sizeof(param_wbuf_s), nullptr, nullptr, nullptr};

	/** parameter update topic handle */
	orb_advert_t param_topic = nullptr;
	unsigned int param_instance = 0;

	// the following implements an RW-lock using 2 semaphores (used as mutexes). It gives
	// priority to readers, meaning a writer could suffer from starvation, but in our use-case
	// we only have short periods of reads and writes are rare.
	px4_sem_t _param_sem; ///< this protects against concurrent access to param_values
	int _reader_lock_holders = 0;
	px4_sem_t _reader_lock_holders_lock; ///< this protects against concurrent access to reader_lock_holders

	px4_sem_t _param_sem_save; ///< this protects against concurrent param saves (file or flash access).
	///< we use a separate lock to allow concurrent param reads and saves.
	///< a param_set could still be blocked by a param save, because it
	///< needs to take the reader lock

	/* autosaving variables */
	hrt_abstime _last_autosave_timestamp{0};
	volatile bool _autosave_scheduled{false};
	bool _autosave_disabled{false};

	uint8_t *_param_changed_storage{nullptr};
	int _size_param_changed_storage_bytes = 0;
	static constexpr int bits_per_allocation_unit = (sizeof(*_param_changed_storage) * 8);

	char *_param_user_file{nullptr};


	perf_counter_t _export_perf{perf_alloc(PC_ELAPSED, "param_export")};
	perf_counter_t _find_perf{perf_alloc(PC_ELAPSED, "param_find")};
	perf_counter_t _get_perf{perf_alloc(PC_ELAPSED, "param_get")};
	perf_counter_t _set_perf{perf_alloc(PC_ELAPSED, "param_set")};

};
