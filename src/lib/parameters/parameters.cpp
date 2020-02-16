/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file param.c
 *
 * Global parameter store.
 *
 * Note that it might make sense to convert this into a driver.  That would
 * offer some interesting options regarding state for e.g. ORB advertisements
 * and background parameter saving.
 */

#include "ParameterBackend.hpp"

#include "param.h"

static ParameterBackend *param_backend{nullptr};

void param_init()
{
	if (param_backend == nullptr) {
		param_backend = new ParameterBackend();
	}
}

void param_notify_changes()
{
	if (param_backend != nullptr) {
		param_backend->notifyChanges();
	}
}

param_t param_find(const char *name)
{
	if (param_backend != nullptr) {
		return param_backend->findParameter(name, true);
	}

	return PARAM_INVALID;
}

param_t param_find_no_notification(const char *name)
{
	if (param_backend != nullptr) {
		return param_backend->findParameter(name, false);
	}

	return PARAM_INVALID;
}

unsigned param_count()
{
	if (param_backend != nullptr) {
		return param_backend->count();
	}

	return 0;
}

unsigned param_count_used()
{
	if (param_backend != nullptr) {
		return param_backend->count_used();
	}

	return 0;
}

param_t param_for_index(unsigned index)
{
	if (param_backend != nullptr) {
		return param_backend->for_index(index);
	}

	return PARAM_INVALID;
}

param_t param_for_used_index(unsigned index)
{
	if (param_backend != nullptr) {
		return param_backend->for_used_index(index);
	}

	return PARAM_INVALID;
}

int param_get_index(param_t param)
{
	if (param_backend != nullptr) {
		return param_backend->getParameterIndex(param);
	}

	return -1;
}

int param_get_used_index(param_t param)
{
	if (param_backend != nullptr) {
		return param_backend->getParameterUsedIndex(param);
	}

	return -1;
}

const char *param_name(param_t param)
{
	if (param_backend != nullptr) {
		return param_backend->getParameterName(param);
	}

	return nullptr;
}

bool param_value_is_default(param_t param)
{
	if (param_backend != nullptr) {
		return param_backend->isParameterValueIsDefault(param);
	}

	return false;
}

bool param_value_unsaved(param_t param)
{
	if (param_backend != nullptr) {
		return param_backend->isParameterValueUnsaved(param);
	}

	return false;
}

param_type_t param_type(param_t param)
{
	if (param_backend != nullptr) {
		return param_backend->getParameterType(param);
	}

	return PARAM_TYPE_UNKNOWN;
}

size_t param_size(param_t param)
{
	if (param_backend != nullptr) {
		return param_backend->getParameterSize(param);
	}

	return 0;
}

int param_get(param_t param, void *val)
{
	if (param_backend != nullptr) {
		return param_backend->getParameterValue(param, val);
	}

	return -1;
}

void param_control_autosave(bool enable)
{
	if (param_backend != nullptr) {
		param_backend->control_autosave(enable);
	}
}

#if defined(FLASH_BASED_PARAMS)
int param_set_external(param_t param, const void *val, bool mark_saved, bool notify_changes)
{
	if (param_backend != nullptr) {
		return param_backend->set_internal(param, val, mark_saved, notify_changes);
	}

	return -1;
}

const void *param_get_value_ptr_external(param_t param)
{
	if (param_backend != nullptr) {
		return param_get_value_ptr(param);
	}

	return -1;
}
#endif

int param_set(param_t param, const void *val)
{
	if (param_backend != nullptr) {
		return param_backend->setParameter(param, val, false, true);
	}

	return -1;
}

int
param_set_no_notification(param_t param, const void *val)
{
	if (param_backend != nullptr) {
		return param_backend->setParameter(param, val, false, false);
	}

	return -1;
}

bool param_used(param_t param)
{
	if (param_backend != nullptr) {
		return param_backend->isParameterUsed(param);
	}

	return false;
}

void param_set_used(param_t param)
{
	if (param_backend != nullptr) {
		return param_backend->setParameterUsed(param);
	}
}

int param_reset(param_t param)
{
	if (param_backend != nullptr) {
		return param_backend->resetParameter(param);
	}

	return -1;
}

void param_reset_all()
{
	if (param_backend != nullptr) {
		param_backend->resetAll(true);
	}
}

void param_reset_excludes(const char *excludes[], int num_excludes)
{
	if (param_backend != nullptr) {
		param_backend->resetAllExclude(excludes, num_excludes);
	}
}

int param_set_default_file(const char *filename)
{
	if (param_backend != nullptr) {
		param_backend->setDefaultFile(filename);
	}

	return 0;
}

const char *param_get_default_file()
{
	if (param_backend != nullptr) {
		return param_backend->getDefaultFile();
	}

	return nullptr;
}

int param_save_default()
{
	if (param_backend != nullptr) {
		return param_backend->saveDefault();
	}

	return -1;
}

int param_load_default()
{
	if (param_backend != nullptr) {
		return param_backend->loadDefault();
	}

	return -1;
}

int param_export(int fd, bool only_unsaved)
{
	if (param_backend != nullptr) {
		return param_backend->exportAll(fd, only_unsaved);
	}

	return -1;
}

int param_import(int fd)
{
	if (param_backend != nullptr) {
		return param_backend->import(fd);
	}

	return -1;
}

int param_load(int fd)
{
	if (param_backend != nullptr) {
		return param_backend->loadAll(fd);
	}

	return -1;
}

void param_foreach(void (*func)(void *arg, param_t param), void *arg, bool only_changed, bool only_used)
{
	if (param_backend != nullptr) {
		return param_backend->param_foreach(func, arg, only_changed, only_used);
	}
}

uint32_t param_hash_check()
{
	if (param_backend != nullptr) {
		return param_backend->hash_check();
	}

	return 0;
}

void param_print_status()
{
	if (param_backend != nullptr) {
		param_backend->print_status();

	} else {
		PX4_ERR("not initialized");
	}
}
