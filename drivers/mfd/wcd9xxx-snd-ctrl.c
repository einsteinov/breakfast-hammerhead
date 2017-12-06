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

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mfd/wcd9xxx/wcd9xxx-snd-ctrl.h>

#define CTRL_NAME_LEN	32

/* Global sound control data which will be used in all sysfs nodes */
static struct snd_ctrl_data *ctrl_data;

/* Mutex that protects control data switch */
static DEFINE_MUTEX(snd_ctrl_mutex);

/* List of connected codecs */
static LIST_HEAD(codec_list);

/* Just prototypes. See the information below. */
static struct snd_ctrl_data *__find_ctrl_data(const char *ctrl_name);
static bool ctrl_data_is_global(struct snd_ctrl_data *ctrl_data);

/**
 * snd_ctrl_register() - register a new sound control data.
 * @snd_data: pointer to sound control data.
 *
 * Tries to register a passed control data. If the passed control data is
 * uncomplete or is already registered, an appropriate negative (-EINVAL)
 * will be returned. If this is a first control data in a global codec list,
 * it will replace the global one.
 *
 * Returns zero on success or -EINVAL on error.
 */
int snd_ctrl_register(struct snd_ctrl_data *snd_data)
{
	int err = -EINVAL;

	if (IS_ERR_OR_NULL(snd_data))
		return err;

	mutex_lock(&snd_ctrl_mutex);
	/* Ensure that control data hasn't been already registered */
	if (!__find_ctrl_data(snd_data->name)) {
		err = 0;

		list_add(&snd_data->member, &codec_list);
		/* Make the first passed control data global */
		if (list_is_singular(&codec_list))
			ctrl_data = snd_data;
	}
	mutex_unlock(&snd_ctrl_mutex);

	return err;
}
EXPORT_SYMBOL_GPL(snd_ctrl_register);

/**
 * snd_ctrl_unregister() - unregister a sound control data.
 * @snd_data: pointer to sound control data.
 *
 * Tries to unregister a sound control data. If the passed data is incomplete
 * or even hasn't been registered yet, function will return early.  If the
 * unregistered control data was a global one, the first codec in the list
 * will pretend to be a replacement. In case there are no available codecs,
 * global control data will be nulled.
 */
void snd_ctrl_unregister(struct snd_ctrl_data *snd_data)
{
	if (IS_ERR_OR_NULL(snd_data))
		return;

	mutex_lock(&snd_ctrl_mutex);
	if (unlikely(!__find_ctrl_data(snd_data->name)))
		goto fail;

	list_del(&snd_data->member);
	if (!list_empty(&codec_list)) {
		/* Replace a global control data with a previous local one */
		if (ctrl_data_is_global(snd_data))
			ctrl_data = list_first_entry(&codec_list,
					struct snd_ctrl_data, member);
	} else {
		ctrl_data = NULL;
	}

fail:
	mutex_unlock(&snd_ctrl_mutex);

	return;
}
EXPORT_SYMBOL_GPL(snd_ctrl_unregister);

/**
 * find_ctrl_data() - search for a control data in a global codec list.
 * @ctrl_name: name of a target member.
 *
 * Returns target member structure if found or NULL to the contrary.
 */
static struct snd_ctrl_data *__find_ctrl_data(const char *ctrl_name)
{
	struct snd_ctrl_data *entry;

	/* Return early if there is nothing to search */
	if (IS_ERR_OR_NULL(ctrl_name) || list_empty(&codec_list))
		return NULL;

	list_for_each_entry(entry, &codec_list, member)
		if (!strnicmp(ctrl_name, entry->name, CTRL_NAME_LEN))
			return entry;

	return NULL;
}

/**
 * parse_ctrl_data() - try to switch a sound control data.
 * @snd_data: pointer to pointer to a sound control data to be swithed.
 * @ctrl_name: name of a desired codec whose control data will be used instead.
 *
 * Returns zero on success or -EINVAL to the contrary.
 */
static inline int parse_ctrl_data(struct snd_ctrl_data **snd_data,
				  const char *ctrl_name)
{
	struct snd_ctrl_data *d;

	mutex_lock(&snd_ctrl_mutex);
	d = __find_ctrl_data(ctrl_name);
	if (IS_ERR_OR_NULL(d)) {
		mutex_unlock(&snd_ctrl_mutex);
		return -EINVAL;
	}

	*snd_data = d;
	mutex_unlock(&snd_ctrl_mutex);

	return 0;
}

/**
 * ctrl_data_is_ready() - check the status of a global control data.
 * @read_only: ignore write to codec call status.
 *
 * Returns false if some of the critical parts of control data are empty.
 * Otherwise, returns true.
 */
static bool ctrl_data_is_ready(bool read_only)
{
	if (ctrl_data && ctrl_data->codec &&
	   (ctrl_data->codec_read &&
	   (ctrl_data->codec_write || read_only)))
		return true;

	return false;
}

/**
 * ctrl_data_is_global() - check the state of a passed control data.
 * @snd_data: pointer to sound control data.
 *
 * Compares a passed control data and a global one.
 * Returns true if they're the same, false otherwise.
 */
static bool ctrl_data_is_global(struct snd_ctrl_data *snd_data)
{
	if (!ctrl_data_is_ready(false) ||
	    !!strnicmp(ctrl_data->name, snd_data->name, CTRL_NAME_LEN))
		return false;

	return true;
}

#define create_rw_kobj_attr(_name)					    \
static struct kobj_attribute _name =					    \
	__ATTR(gpl_##_name, S_IRUGO | S_IWUSR, show_##_name, store_##_name);

#define create_one_single(_name, line)					    \
static ssize_t show_##_name(struct kobject *kobj,			    \
			    struct kobj_attribute *attr,		    \
			    char *buf)					    \
{									    \
	if (!ctrl_data_is_ready(true) || !ctrl_data->line)		    \
		return scnprintf(buf, sizeof(u8) * 16, "<unsupported>\n");  \
									    \
	return scnprintf(buf, sizeof(u32) * 8 + 1, "%u\n",		    \
		ctrl_data->codec_read(ctrl_data->codec, ctrl_data->line));  \
}									    \
									    \
static ssize_t store_##_name(struct kobject *kobj,			    \
			     struct kobj_attribute *attr,		    \
			     const char *buf, size_t count)		    \
{									    \
	int ret;							    \
	u32 val;							    \
									    \
	if (!ctrl_data_is_ready(false) || !ctrl_data->line)		    \
		return -EINVAL;						    \
									    \
	ret = sscanf(buf, "%u", &val);					    \
	if (ret != 1 || (s32)val < 0 || val > 256)			    \
		return -EINVAL;						    \
									    \
	ctrl_data->codec_write(ctrl_data->codec, ctrl_data->line, val);	    \
									    \
	return count;							    \
}									    \
create_rw_kobj_attr(_name);

#define create_one_double(_name, lline, rline)				    \
static ssize_t show_##_name(struct kobject *kobj,			    \
			    struct kobj_attribute *attr,		    \
			    char *buf)					    \
{									    \
	if (!ctrl_data_is_ready(true) ||				    \
	    !ctrl_data->lline || !ctrl_data->rline)			    \
		return scnprintf(buf, sizeof(u8) * 16, "<unsupported>\n");  \
									    \
	return scnprintf(buf, sizeof(u32) * 16 + 2, "%u %u\n",		    \
		ctrl_data->codec_read(ctrl_data->codec, ctrl_data->lline),  \
		ctrl_data->codec_read(ctrl_data->codec, ctrl_data->rline)); \
}									    \
									    \
static ssize_t store_##_name(struct kobject *kobj,			    \
			     struct kobj_attribute *attr,		    \
			     const char *buf, size_t count)		    \
{									    \
	int ret;							    \
	u32 lval, rval;							    \
									    \
	if (!ctrl_data_is_ready(false) ||				    \
	    !ctrl_data->lline || !ctrl_data->rline)			    \
		return -EINVAL;						    \
									    \
	ret = sscanf(buf, "%u %u", &lval, &rval);			    \
	if (ret != 2 ||							    \
	   (s32)lval < 0 || lval > 256 ||				    \
	   (s32)rval < 0 || rval > 256)					    \
		return -EINVAL;						    \
									    \
	ctrl_data->codec_write(ctrl_data->codec, ctrl_data->lline, lval);   \
	ctrl_data->codec_write(ctrl_data->codec, ctrl_data->rline, rval);   \
									    \
	return count;							    \
}									    \
create_rw_kobj_attr(_name);

static ssize_t show_active_codec(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	struct snd_ctrl_data *entry;
	ssize_t len = 0;

	if (list_empty(&codec_list))
		return scnprintf(buf, sizeof(u8) * 8, "<none>\n");

	list_for_each_entry(entry, &codec_list, member) {
		if (unlikely(len >= (ssize_t)((PAGE_SIZE / sizeof(u8)) -
		   (CTRL_NAME_LEN + 2))))
			goto out;

		len += scnprintf(buf + len, CTRL_NAME_LEN + 5,
				 list_is_last(&entry->member, &codec_list) ?
				(ctrl_data_is_global(entry) ? "[%s]"  : "%s") :
				(ctrl_data_is_global(entry) ? "[%s] " : "%s "),
				 entry->name);
	}

out:
	len += scnprintf(buf + len, sizeof(u8) * 2, "\n");

	return len;
}

static ssize_t store_active_codec(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	int ret;
	char name[CTRL_NAME_LEN];

	ret = sscanf(buf, "%31s", name);
	if (ret != 1)
		return -EINVAL;

	if (parse_ctrl_data(&ctrl_data, name)) {
		pr_err("unable to set %s\n", name);
		return -EINVAL;
	}

	return count;
}

create_one_single(mic_gain, mic_line);
create_one_single(cam_mic_gain, cam_mic_line);
create_one_double(headphone_gain, headphone_l_line, headphone_r_line);
create_one_double(speaker_gain, speaker_l_line, speaker_r_line);
create_rw_kobj_attr(active_codec);

static struct attribute *snd_ctrl_attrs[] = {
	&mic_gain.attr,
	&cam_mic_gain.attr,
	&headphone_gain.attr,
	&speaker_gain.attr,
	&active_codec.attr,
	NULL,
};

static struct attribute_group snd_ctrl_attr_group = {
	.name = "sound_control_3",
	.attrs = snd_ctrl_attrs,
};

static int __init snd_ctrl_init(void)
{
	int ret;

	ret = sysfs_create_group(kernel_kobj, &snd_ctrl_attr_group);
	if (IS_ERR_VALUE(ret))
		pr_err("unable to create sysfs group\n");

	return ret;
}

static void __exit snd_ctrl_exit(void)
{
	sysfs_remove_group(kernel_kobj, &snd_ctrl_attr_group);
}

module_init(snd_ctrl_init);
module_exit(snd_ctrl_exit);
