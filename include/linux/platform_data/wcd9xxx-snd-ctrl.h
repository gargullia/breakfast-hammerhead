/*
 * Copyright (C) 2017-2018, Alex Saiko <solcmdr@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __WCD9XXX_SND_CTRL_PDATA__
#define __WCD9XXX_SND_CTRL_PDATA__

#include <linux/mfd/wcd9xxx/wcd9xxx-snd-ctrl.h>

struct snd_ctrl_pdata {
	/* Name of an expected codec */
	const char *name;

	/* Basic audio input lines */
	struct snd_ctrl_lines lines;

	/* Default sound gains */
	u32 default_gain[6];
};

#endif /* __WCD9XXX_SND_CTRL_PDATA__ */
