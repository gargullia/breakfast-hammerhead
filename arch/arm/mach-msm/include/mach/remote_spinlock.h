/*
 * Copyright (c) 2008-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM__ARCH_QC_REMOTE_SPINLOCK_H
#define __ASM__ARCH_QC_REMOTE_SPINLOCK_H

#include <linux/io.h>

/* Remote spinlock implementation */
typedef struct {
	volatile uint32_t lock;
} raw_remote_spinlock_t;

typedef raw_remote_spinlock_t *_remote_spinlock_t;

#if defined(CONFIG_MSM_SMD) || defined(CONFIG_MSM_REMOTE_SPINLOCK_SFPB)
int _remote_spin_lock_init(const char *id, _remote_spinlock_t *lock);
int _remote_spin_trylock(_remote_spinlock_t *lock);
int _remote_spin_release(_remote_spinlock_t *lock, uint32_t pid);
int _remote_spin_owner(_remote_spinlock_t *lock);
void _remote_spin_release_all(uint32_t pid);
void _remote_spin_lock(_remote_spinlock_t *lock);
void _remote_spin_unlock(_remote_spinlock_t *lock);
#else
static inline int
_remote_spin_lock_init(const char *id, _remote_spinlock_t *lock)
{
	return -EINVAL;
}
static inline int _remote_spin_trylock(_remote_spinlock_t *lock)
{
	return -ENODEV;
}
static inline int _remote_spin_release(_remote_spinlock_t *lock, uint32_t pid)
{
	return -ENODEV;
}
static inline int _remote_spin_owner(_remote_spinlock_t *lock)
{
	return -ENODEV;
}
static inline void _remote_spin_release_all(uint32_t pid) { }
static inline void _remote_spin_lock(_remote_spinlock_t *lock) { }
static inline void _remote_spin_unlock(_remote_spinlock_t *lock) { }
#endif

/* Remote mutex implementation */
typedef struct {
	_remote_spinlock_t r_spinlock;
	uint32_t delay_us;
} _remote_mutex_t;

struct remote_mutex_id {
	const char *r_spinlock_id;
	uint32_t delay_us;
};

#ifdef CONFIG_MSM_SMD
int _remote_mutex_init(struct remote_mutex_id *id, _remote_mutex_t *lock);
int _remote_mutex_trylock(_remote_mutex_t *lock);
void _remote_mutex_lock(_remote_mutex_t *lock);
void _remote_mutex_unlock(_remote_mutex_t *lock);
#else
static inline int
_remote_mutex_init(struct remote_mutex_id *id, _remote_mutex_t *lock)
{
	return -EINVAL;
}
static inline int _remote_mutex_trylock(_remote_mutex_t *lock)
{
	return 0;
}
static inline void _remote_mutex_lock(_remote_mutex_t *lock) { }
static inline void _remote_mutex_unlock(_remote_mutex_t *lock) { }
#endif

#endif /* __ASM__ARCH_QC_REMOTE_SPINLOCK_H */
