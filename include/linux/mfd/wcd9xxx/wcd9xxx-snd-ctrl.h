/*
 * Copyright (C) 2017, Alex Saiko <solcmdr@gmail.com>
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

#ifndef __WCD9XXX_SND_CTRL_H__
#define __WCD9XXX_SND_CTRL_H__

#include <linux/list.h>
#include <sound/soc.h>

struct snd_ctrl_data {
	struct snd_soc_codec *codec;
	struct list_head member;

	const char *name;

	u32 mic_line;
	u32 cam_mic_line;
	u32 headphone_l_line;
	u32 headphone_r_line;
	u32 speaker_l_line;
	u32 speaker_r_line;

	unsigned int	(*codec_read)	(struct snd_soc_codec *codec,
					 unsigned int reg);
	int		(*codec_write)	(struct snd_soc_codec *codec,
					 unsigned int reg, unsigned int val);
};

int snd_ctrl_register(struct snd_ctrl_data *snd_data);
void snd_ctrl_unregister(struct snd_ctrl_data *snd_data);

#endif /* __WCD9XXX_SND_CTRL_H__ */
