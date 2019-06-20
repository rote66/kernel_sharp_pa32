/* drivers/video/msm/mdss_mdp_fps_debugfs.c  (Display Driver)
 *
 * Copyright (C) 2015 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/timer.h>

#include "mdss_fb.h"
#include "mdss_mdp.h"
#include "mdss_mdp_hwio.h"
#include "mdss_shdisp.h"

struct mdss_mdp_fps_ctx {
	struct mdss_mdp_ctl *ctl;
	int fps;
	u32 commit_cnt;
	struct timeval tm;
	struct work_struct fps_work;
	struct timer_list fps_timer;
};

struct mdss_mdp_fps_ctx fps_ctx;

int thresh_white   = 120;
int thresh_red     = 105;
int thresh_yellow  =  90;
int thresh_green   =  75;
int check_duration = 500;
module_param_named(thresh_white, thresh_white, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(thresh_red, thresh_red, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(thresh_yellow, thresh_yellow, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(thresh_green, thresh_green, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(check_duration, check_duration, int, S_IRUGO | S_IWUSR | S_IWGRP);

static void mdss_mdp_fps_worker(struct work_struct *work)
{
	struct mdss_mdp_fps_ctx *ctx = container_of(work, typeof(*ctx), fps_work);
	char r, g, b;

	if (ctx->fps >= thresh_white) {
		//white
		r = 1;
		g = 1;
		b = 1;
	}
	else if (ctx->fps >= thresh_red){
		//red
		r = 1;
		g = 0;
		b = 0;
	}
	else if (ctx->fps >= thresh_yellow){
		//yellow
		r = 1;
		g = 1;
		b = 0;
	}
	else if (ctx->fps >= thresh_green){
		//green
		r = 0;
		g = 1;
		b = 0;
	}
	else {
		//blue
		r = 0;
		g = 0;
		b = 1;
	}

	mdss_shdisp_tri_led_set_color(r, g, b);

	return;
}

static void fps_check_timer_cb(unsigned long arg)
{
	struct mdss_mdp_fps_ctx *ctx = (struct mdss_mdp_fps_ctx *)arg;
	u32 cnt;
	struct timeval now;
	u32 duration;

	do_gettimeofday(&now);

	if (ctx->commit_cnt) {
		cnt = ctx->ctl->play_cnt - ctx->commit_cnt;

		duration = (now.tv_sec - ctx->tm.tv_sec) * 1000;
		duration += (now.tv_usec - ctx->tm.tv_usec) / 1000;

		ctx->fps = (1000 * cnt) / duration;
		pr_info("%s fps = %d\n", __func__, ctx->fps);

		schedule_work(&ctx->fps_work);
	}
	ctx->commit_cnt = ctx->ctl->play_cnt;
	ctx->tm = now;

	mod_timer(&ctx->fps_timer, jiffies + msecs_to_jiffies(check_duration));
}

static ssize_t fps_check(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct msm_fb_data_type *mfd = s->private;
	struct mdss_panel_data *pdata;
	struct mdss_mdp_ctl *ctl;
	char *buf;
	int param;
	struct mdss_mdp_fps_ctx *ctx = &fps_ctx;

	if (mdss_fb_is_power_off(mfd)) {
		pr_err("%s: panel is NOT on\n", __func__);
		goto exit;
	}

	ctl = mfd_to_ctl(mfd);
	if (!ctl)
		goto exit;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("no panel connected\n");
		goto exit;
	}

	buf = kzalloc(sizeof(char) * count, GFP_KERNEL);
	if (!buf) {
		pr_err("%s: Failed to allocate buffer\n", __func__);
		goto exit;
	}

	if (copy_from_user(buf, ubuf, count)) {
		goto fail_free_all;
	}

	ctx->ctl = ctl;

	param = simple_strtol(buf, NULL, 10);

	pr_info("%s (%d)\n", __func__, param);

	cancel_work_sync(&ctx->fps_work);
	if (param) {
		init_timer(&ctx->fps_timer);
		ctx->fps_timer.function = fps_check_timer_cb;
		ctx->fps_timer.data = (unsigned long)ctx;
		ctx->fps_timer.expires = jiffies + msecs_to_jiffies(8);
		add_timer(&ctx->fps_timer);
	}
	else {
		del_timer(&ctx->fps_timer);
		cancel_work_sync(&ctx->fps_work);
		mdss_shdisp_tri_led_set_color(0, 0, 0);
	}

fail_free_all:
	kfree(buf);
exit:

	return count;
}

static int fps_check_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}

static const struct file_operations fps_check_fops = {
	.owner			= THIS_MODULE,
	.open			= fps_check_open,
	.write			= fps_check,
	.llseek			= seq_lseek,
	.release		= single_release,
};

void mdss_mdp_fps_create_debugfs(struct msm_fb_data_type *mfd)
{
	struct dentry *root;
	struct mdss_panel_data *pdata;
	struct mdss_mdp_fps_ctx *ctx = &fps_ctx;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("no panel connected\n");
		return;
	}

	pr_info("%s: create folder\n", __func__);

	root = debugfs_create_dir("mdss_debug", 0);

	if (!root) {
		pr_err("%s: dbgfs create dir failed\n", __func__);
	} else {
		if (!debugfs_create_file("fps_check", S_IWUSR, root, mfd,
								&fps_check_fops)) {
			pr_err("%s: failed to create dbgfs fps check file\n",
								__func__);
			return;
		}
	}
	ctx->commit_cnt = 0;
	INIT_WORK(&ctx->fps_work, mdss_mdp_fps_worker);
}
