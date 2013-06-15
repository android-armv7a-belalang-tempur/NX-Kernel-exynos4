/*
 * mdnie_control.c - mDNIe register sequence intercept and control
 *
 * @Author	: Pranav Vashi <https://github.com/neobuddy89>
 * @Date	: June 2013
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/earlysuspend.h>

#include "mdnie.h"
#include "s3cfb.h"
#include "s3cfb_mdnie.h"

#define REFRESH_DELAY		HZ / 2
struct delayed_work mdnie_refresh_work;

bool reg_hook = 0;
bool scenario_hook = 0;
struct mdnie_info *mdnie = NULL;

extern struct mdnie_info *g_mdnie;
extern void set_mdnie_value(struct mdnie_info *mdnie, u8 force);

enum mdnie_registers {

	DNR_1		= 0x2c,	/*DNR dirTh*/
	DNR_2		= 0x2d, /*DNR dirnumTh decon7Th*/
	DNR_3		= 0x2e,	/*DNR decon5Th maskTh*/
	DNR_4		= 0x2f,	/*DNR blTh*/

	DE_TH		= 0x42,	/*DE TH (MAX DIFF) */

	SCR_KR_BR	= 0xc8,	/*kb R	SCR */
	SCR_GR_CR	= 0xc9,	/*gc R  SCR */
	SCR_RR_MR	= 0xca,	/*rm R  SCR */
	SCR_YR_WR	= 0xcb,	/*rm R  SCR */

	SCR_KG_BG	= 0xcc,	/*kb G	SCR */
	SCR_GG_CG	= 0xcd,	/*gc G  SCR */
	SCR_RG_MG	= 0xce,	/*rm G  SCR */
	SCR_YG_WG	= 0xcf,	/*rm G  SCR */

	SCR_KB_BB	= 0xd0,	/*kb B	SCR */
	SCR_GB_CB	= 0xd1,	/*gc B  SCR */
	SCR_RB_MB	= 0xd2,	/*rm B  SCR */
	SCR_YB_WB	= 0xd3,	/*rm B  SCR */

	MCM_TEMPERATURE = 0x5b, /*MCM 0x64 10000K 0x28 4000K */
};

static unsigned short master_sequence[] = { 
	0x0001, 0x0000,	0x002c, 0x0fff,	0x002d, 0x19ff,	0x002e, 0xff16,
	0x002f, 0x0000,	0x003a, 0x000d,	0x003b, 0x0030,	0x003c, 0x0000,
	0x003f, 0x0140,	0x0042, 0x0080,

	0x00c8, 0x0000,	0x00c9, 0x0000, 0x00ca, 0xffff,	0x00cb, 0xffff,
	0x00cc, 0x0000,	0x00cd, 0xffff,	0x00ce, 0x0000,	0x00cf, 0xffff,
	0x00d0, 0x00ff,	0x00d1, 0x00ff,	0x00d2, 0x00ff,	0x00d3, 0x00ff,

	0x00d6, 0x4000, 0x00d7, 0x2104,	0x00d8, 0x2104,	0x00d9, 0x2104,
	0x00da, 0x2104,	0x00db, 0x2104,	0x00dc, 0x2104,	0x00dd, 0x2104,
	0x00de, 0x2104,	0x00df, 0x2104,	0x00e0, 0x2104,	0x00e1, 0x2104,
	0x00e2, 0x2104,	0x00e3, 0x2104,	0x00e4, 0x2104,	0x00e5, 0x2104,
	0x00e6, 0x2104,	0x00e7, 0x2104,	0x00e8, 0x2104,	0x00e9, 0x2104,
	0x00ea, 0x2104,	0x00eb, 0x2104,	0x00ec, 0x2104,	0x00ed, 0xff00,

	0x00d5, 0x0001,	0x0028, 0x0000, 0xffff, 0x0000,
};

static ssize_t show_mdnie_property(struct device *dev,
				    struct device_attribute *attr, char *buf);

static ssize_t store_mdnie_property(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count);

#define _effect(name_, reg_, mask_, shift_, regval_)\
{ 									\
	.attribute = {							\
			.attr = {					\
				  .name = name_,			\
				  .mode = S_IRUGO | S_IWUSR | S_IWGRP,	\
				},					\
			.show = show_mdnie_property,			\
			.store = store_mdnie_property,			\
		     },							\
	.reg 	= reg_ ,						\
	.mask 	= mask_ ,						\
	.shift 	= shift_ ,						\
	.delta 	= 1 ,							\
	.value 	= 0 ,							\
	.regval = regval_						\
}

struct mdnie_effect {
	const struct device_attribute	attribute;
	u8				reg;
	u16				mask;
	u8				shift;
	bool				delta;
	int				value;
	u16				regval;
};

struct mdnie_effect mdnie_controls[] = {

	/* Digital noise reduction */
	_effect("dnr_dirTh"		, DNR_1		, (0xffff), 0	, 4095	),
	_effect("dnr_dirnumTh"		, DNR_2		, (0xff00), 8	, 25	),
	_effect("dnr_decon7Th"		, DNR_2		, (0x00ff), 0	, 255	),
	_effect("dnr_decon5Th"		, DNR_3		, (0xff00), 8	, 255	),
	_effect("dnr_maskTh"		, DNR_3		, (0x00ff), 0	, 22	),
	_effect("dnr_blTh"		, DNR_4		, (0x00ff), 0	, 0	),

	/* Digital edge enhancement */
	_effect("de_th"			, DE_TH		, (0x00ff), 0	, 128	),

	/* Colour channel pass-through filters */
	_effect("scr_red_red"		, SCR_RR_MR	, (0xff00), 8	, 237	),
	_effect("scr_red_green"		, SCR_RG_MG	, (0xff00), 8	, 15	),
	_effect("scr_red_blue"		, SCR_RB_MB	, (0xff00), 8	, 0	),

	_effect("scr_cyan_red"		, SCR_GR_CR	, (0x00ff), 0	, 68	),
	_effect("scr_cyan_green"	, SCR_GG_CG	, (0x00ff), 0	, 245	),
	_effect("scr_cyan_blue"		, SCR_GB_CB	, (0x00ff), 0	, 255	),
	
	_effect("scr_green_red"		, SCR_GR_CR	, (0xff00), 8	, 97	),
	_effect("scr_green_green"	, SCR_GG_CG	, (0xff00), 8	, 230	),
	_effect("scr_green_blue"	, SCR_GB_CB	, (0xff00), 8	, 18	),

	_effect("scr_magenta_red"	, SCR_RR_MR	, (0x00ff), 0	, 251	),
	_effect("scr_magenta_green"	, SCR_RG_MG	, (0x00ff), 0	, 36	),
	_effect("scr_magenta_blue"	, SCR_RB_MB	, (0x00ff), 0	, 255	),
	
	_effect("scr_blue_red"		, SCR_KR_BR	, (0x00ff), 0	, 0	),
	_effect("scr_blue_green"	, SCR_KG_BG	, (0x00ff), 0	, 43	),
	_effect("scr_blue_blue"		, SCR_KB_BB	, (0x00ff), 0	, 255	),

	_effect("scr_yellow_red"	, SCR_YR_WR	, (0xff00), 8	, 255	),
	_effect("scr_yellow_green"	, SCR_YG_WG	, (0xff00), 8	, 239	),
	_effect("scr_yellow_blue"	, SCR_YB_WB	, (0xff00), 8	, 27	),

	_effect("scr_black_red"		, SCR_KR_BR	, (0xff00), 8	, 0	),
	_effect("scr_black_green"	, SCR_KG_BG	, (0xff00), 8	, 0	),
	_effect("scr_black_blue"	, SCR_KB_BB	, (0xff00), 8	, 0	),

	_effect("scr_white_red"		, SCR_YR_WR	, (0x00ff), 0	, 255	),
	_effect("scr_white_green"	, SCR_YG_WG	, (0x00ff), 0	, 240	),
	_effect("scr_white_blue"	, SCR_YB_WB	, (0x00ff), 0	, 240	),

	/* MCM */
	_effect("mcm_temperature"	, MCM_TEMPERATURE, (0x00ff), 0	, 65	),

};

static ssize_t show_mdnie_property(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mdnie_effect *effect = (struct mdnie_effect*)(attr);
	
	return sprintf(buf, "%s %d", effect->delta ? "delta" : "override", effect->value);
};

static ssize_t store_mdnie_property(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct mdnie_effect *effect = (struct mdnie_effect*)(attr);
	int val, ret;
	
	if(sscanf(buf, "%d", &val) != 1) {
		char *s = kzalloc(10 * sizeof(char), GFP_KERNEL);
		if(sscanf(buf, "%10c", s) != 1) {
			ret = -EINVAL;
		} else {
			printk("input: '%s'\n", s);

			if(strncmp(s, "override", 8)) {
				effect->delta = 0;
				ret = count;
			}
			
			if(strncmp(s, "delta", 5)) {
				effect->delta = 1;
				ret = count;
			}

			ret = -EINVAL;
		}

		kfree(s);
		if(ret != -EINVAL)
			goto refresh;
		return ret;
	}

	
	if(val > (effect->mask >> effect->shift))
		val = (effect->mask >> effect->shift);

	if(val < -(effect->mask >> effect->shift))
		val = -(effect->mask >> effect->shift);

	effect->value = val;


refresh:
	cancel_delayed_work_sync(&mdnie_refresh_work);
	schedule_delayed_work_on(0, &mdnie_refresh_work, REFRESH_DELAY);

	return count;
};

static ssize_t show_reg_hook(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", reg_hook);
};

static ssize_t store_reg_hook(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int val;
	
	if(sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	reg_hook = !!val;

	cancel_delayed_work_sync(&mdnie_refresh_work);
	schedule_delayed_work_on(0, &mdnie_refresh_work, REFRESH_DELAY);

	return count;
};
static ssize_t show_sequence_hook(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", scenario_hook);
};

static ssize_t store_sequence_hook(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int val;
	
	if(sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	scenario_hook = !!val;

	cancel_delayed_work_sync(&mdnie_refresh_work);
	schedule_delayed_work_on(0, &mdnie_refresh_work, REFRESH_DELAY);

	return count;
};

unsigned short mdnie_reg_hook(unsigned short reg, unsigned short value)
{
	struct mdnie_effect *effect = (struct mdnie_effect*)&mdnie_controls;
	int i;
	int tmp, original;
	unsigned short regval;

	original = value;

	if((!scenario_hook && !reg_hook) || mdnie->negative == NEGATIVE_ON)
		return value;

	for(i = 0; i < ARRAY_SIZE(mdnie_controls); i++) {
	    if(effect->reg == reg) {
		if(scenario_hook) {
			tmp = regval = effect->regval;
		} else {
			tmp = regval = (value & effect->mask) >> effect->shift;
		}

		if(reg_hook) {
			if(effect->delta) {
				tmp += effect->value;
			} else {
				tmp = effect->value;
			}

			if(tmp > (effect->mask >> effect->shift))
				tmp = (effect->mask >> effect->shift);

			if(tmp < 0)
				tmp = 0;

			regval = (unsigned short)tmp;
		}

		value &= ~effect->mask;
		value |= regval << effect->shift;

/*
		printk("mdnie: hook on: 0x%X val: 0x%X -> 0x%X effect:%4d\n",
			reg, original, value, tmp);
*/
	    }
	    ++effect;
	}
	
	return value;
}

unsigned short *mdnie_sequence_hook(struct mdnie_info *pmdnie, unsigned short *seq)
{
	if(mdnie == NULL)
		mdnie = pmdnie;

	if(!scenario_hook || mdnie->negative == NEGATIVE_ON)
		return seq;

	return (unsigned short *)&master_sequence;
}

static void do_mdnie_refresh(struct work_struct *work)
{
	set_mdnie_value(g_mdnie, 1);
}

const struct device_attribute master_switch_attr[] = {
	{ 
		.attr = { .name = "hook_intercept",
			  .mode = S_IRUGO | S_IWUSR | S_IWGRP, },
		.show = show_reg_hook,
		.store = store_reg_hook,
	},{
		.attr = { .name = "sequence_intercept",
			  .mode = S_IRUGO | S_IWUSR | S_IWGRP, },
		.show = show_sequence_hook,
		.store = store_sequence_hook,
	},
};

void init_intercept_control(struct kobject *kobj)
{
	int i, ret;
	struct kobject *subdir;

	subdir = kobject_create_and_add("hook_control", kobj);

	for(i = 0; i < ARRAY_SIZE(mdnie_controls); i++) {
		ret = sysfs_create_file(subdir, &mdnie_controls[i].attribute.attr);
	}

	ret = sysfs_create_file(kobj, &master_switch_attr[0].attr);
	ret = sysfs_create_file(kobj, &master_switch_attr[1].attr);

	INIT_DELAYED_WORK(&mdnie_refresh_work, do_mdnie_refresh);
}
