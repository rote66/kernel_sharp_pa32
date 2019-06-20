/* drivers/sharp/shaudio/shsmartamp_ssm4329.c
 *
 * Copyright (C) 2011 SHARP CORPORATION
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
/* CONFIG_SH_AUDIO_DRIVER newly created */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <sharp/shsmartamp_ssm4329.h>
#ifdef SHSMARTAMP_FBUILD
#include <sharp/shsmartamp_ssm4329_fw_fbuild.h>
#else
#include <sharp/shsmartamp_ssm4329_fw.h>
#endif
#include <linux/of_gpio.h>
#include <linux/qpnp/pin.h>
#ifdef SHSMARTAMP_ENG
#include <linux/syscalls.h>
#include <linux/debugfs.h>
#endif

#define SHAUDIO_GPIO_CLK              qpnp_pin_map("pm8994-gpio", 17)

static struct i2c_client *shsmartamp_client;
#ifdef SHSMARTAMP_ENG
static struct dentry *shsmartamp_dbgfile;
uint8_t firmware_array[FIRMWARE_ARRAY_MAX] = {0};
#endif

static DEFINE_MUTEX(smartamp_lock);
static int shsmartamp_opened;
static int shsmartamp_en = -1;
struct qpnp_pin_cfg shsmartamp_pin_cfg_on =
{
	.mode = QPNP_PIN_MODE_DIG_OUT,
	.output_type = QPNP_PIN_OUT_BUF_CMOS,
	.invert = QPNP_PIN_INVERT_DISABLE,
	.pull = QPNP_PIN_GPIO_PULL_NO,
	.vin_sel = QPNP_PIN_VIN2,
	.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel = QPNP_PIN_SEL_FUNC_2,
	.master_en = QPNP_PIN_MASTER_ENABLE,
	.aout_ref = 0,
	.ain_route = 0,
	.cs_out = 0,
};

struct qpnp_pin_cfg shsmartamp_pin_cfg_off =
{
	.mode = QPNP_PIN_MODE_DIG_OUT,
	.output_type = QPNP_PIN_OUT_BUF_CMOS,
	.invert = QPNP_PIN_INVERT_DISABLE,
	.pull = QPNP_PIN_GPIO_PULL_NO,
	.vin_sel = QPNP_PIN_VIN2,
	.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
	.src_sel = QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en = QPNP_PIN_MASTER_ENABLE,
	.aout_ref = 0,
	.ain_route = 0,
	.cs_out = 0,
};

static int shsmartamp_i2c_write(const uint8_t* data, int data_sz, int package_sz)
{
	int ret = 0;
	int retry_max = 20;
	int try_count = 0;
	int tail, front, buf_sz;
	unsigned short addr;
	uint8_t buf[package_sz + SSM4329_ADDR_LEN];

	struct i2c_msg msgs[] =
	{
		{
			.addr = shsmartamp_client->addr,
			.flags = 0,
			.buf = buf,
		},
	};

	pr_debug("%s(): Writing to address 0x%02x%02x data_sz = %d\n",
		__func__, data[0], data[1], data_sz);

	addr = ((data[0] << 8) & 0xff00) + (data[1] & 0xff);
	tail = data_sz;
	front = SSM4329_ADDR_LEN;

	for (; tail != front;) {
		if ((tail - front) >= package_sz) {
			buf_sz = package_sz;
		} else {
			buf_sz = tail - front;
		}

		buf[0] = (addr >> 8) & 0xff;
		buf[1] = addr & 0xff;
		memcpy(&buf[SSM4329_ADDR_LEN], &data[front], buf_sz);
		msgs[0].len = buf_sz + SSM4329_ADDR_LEN;
		front += buf_sz;

		for (try_count = 0; try_count <= retry_max; try_count++) {
			ret = i2c_transfer(shsmartamp_client->adapter, msgs, 1);
			if(ret < 0 && ret != -EAGAIN) {
				pr_err("%s(): FAILED. writing to address 0x%02x%02x\n",
					__func__, buf[0], buf[1]);
				return ret;
			} else if (ret > 0) {
				break;
			}
		}

		addr += buf_sz / package_sz * SSM4329_PACKAGE;
	}

	return (ret > 0)? 0 : ret;
}

static int shsmartamp_i2c_read(const uint8_t *data, int *output)
{
	int ret = 0;
	int retry_max = 20;
	int try_count = 0;
	uint8_t write_buf[SSM4329_ADDR_LEN];
	uint8_t read_buf;
	struct i2c_msg msg[] =
	{
		{
			.addr = shsmartamp_client->addr,
			.flags= 0,
			.len  = 2,
			.buf  = write_buf,
		},
		{
			.addr = shsmartamp_client->addr,
			.flags= I2C_M_RD,
			.len  = 1,
			.buf  = &read_buf,
		},
	};

	memcpy(write_buf, data, SSM4329_ADDR_LEN);

	pr_debug("%s()\n", __func__);

	for (try_count = 0; try_count <= retry_max; try_count++) {
		ret = i2c_transfer(shsmartamp_client->adapter, &msg[0], 1);
		if (ret == -EAGAIN)
			continue;
		if(ret < 0 && ret != -EAGAIN) {
			pr_err("%s(): FAILED. writing to address 0x%02x%02x\n",
				__func__, write_buf[0], write_buf[1]);
			return ret;
		}
		ret = i2c_transfer(shsmartamp_client->adapter, &msg[1], 1);
		if (ret == -EAGAIN)
			continue;
		if(ret < 0 && ret != -EAGAIN) {
			pr_err("%s(): FAILED. read from address 0x%02x%02x\n",
				__func__, write_buf[0], write_buf[1]);
			return ret;
		}
		*output = read_buf;
		pr_debug("%s(): read 0x%02x from address 0x%02x%02x\n",
			__func__, read_buf, write_buf[0], write_buf[1]);
		break;
	}

	return (ret > 0)? 0 : ret;
}

#ifdef SHSMARTAMP_ENG
static int shsmartamp_get_package_size(const uint8_t *data)
{
	unsigned short addr = ((data[0] << 8) & 0xff00) + (data[1] & 0xff);

	pr_debug("%s(): addr = 0x%04x\n", __func__, addr);

	if ((SSM4329_PROGRAM_START <= addr) && (addr <= SSM4329_PROGRAM_END)) {
		return SSM4329_PROGRAM;
	}

	return SSM4329_DATA;
}

static uint8_t shsmartamp_atoi(const uint8_t data)
{
	int ret = 0;

	if(data >= '0' && data <= '9') {
		ret = data - '0';
	} else {
		ret = data - 'A' + 10;
	}

	return ret;
}

static int shsmartamp_fw_decode(const char *buf)
{
	int buf_stamp = 2;
	int array_stamp = 1;

	firmware_array[0] = shsmartamp_atoi(buf[2]) * 16 + shsmartamp_atoi(buf[3]);
	firmware_array[1] = shsmartamp_atoi(buf[4]) * 16 + shsmartamp_atoi(buf[5]);

	do {
		buf_stamp += 5;
		array_stamp++;
		firmware_array[array_stamp] = shsmartamp_atoi(buf[buf_stamp + 2]) * 16 + shsmartamp_atoi(buf[buf_stamp + 3]);
	} while (buf[buf_stamp + 4] == ',');

	return array_stamp + 1;
}

static char * fget_line(char * buf, int max_num, int fd,off_t *fd_seek)
{
	int cur_seek = *fd_seek;
	char c_buf[1];
	int i = 0;

	for(i = 0; i < max_num; i++) {
		buf[i] = '\0';
	}

	sys_lseek(fd, (off_t)cur_seek, 0);

	for(i = 0; i < max_num; i++) {
		if ((unsigned)sys_read(fd, (char *)c_buf, 1) != 1) {
			pr_err("%s(): Can not read %d \n",
				__func__, i);
			return NULL;
		} else {
			buf[i] = c_buf[0];
			cur_seek++;
			sys_lseek(fd, (off_t)cur_seek, 0);
		}

		if(buf[i] == '\n') {
			break;
		}
	}
	*fd_seek = cur_seek;
	return buf;
}

static int shsmartamp_fw_load(void)
{
    int fw_fd, oldfs;
	off_t fd_seek = 0;
	char *buf;
	int array_sz = 0, package_sz = 0, ret = 0;

	pr_debug("%s(): Entry\n", __func__);

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	fw_fd = sys_open((const char __force *) SSM4329_FIRMWARE_PATH, O_RDONLY, 0);
	if (fw_fd < 0) {
		pr_err("%s(): firmware %s open failed %d\n", __func__, SSM4329_FIRMWARE_PATH, fw_fd);
		ret = fw_fd;
		goto sys_open_err;
	}

	buf = (char *)kmalloc(FIRMWARE_BUF_MAX, GFP_KERNEL);
	if (buf == NULL) {
		pr_err("%s(): alloc firmware buf failed\n", __func__);
		ret = -1;
		goto kmalloc_err;
	}

	sys_lseek(fw_fd, (off_t)0, 0);

	while (NULL != (fget_line(buf, FIRMWARE_BUF_MAX, fw_fd, &fd_seek))) {
		array_sz = shsmartamp_fw_decode(buf);
		package_sz = shsmartamp_get_package_size(firmware_array);
		ret = shsmartamp_i2c_write(firmware_array, array_sz, package_sz);
		if (ret != 0) {
			pr_err("%s(): Failed to writing data for address 0x%02x%02x", __func__,
				firmware_array[0], firmware_array[1]);
			goto i2c_write_err;
		}
	}

i2c_write_err:
	kfree(buf);
kmalloc_err:
	sys_close(fw_fd);
sys_open_err:
	set_fs(oldfs);
	pr_debug("%s(): Leave\n", __func__);

	return ret;
}

static int shsmartamp_fw_update(void)
{
	int ret;

	pr_debug("%s(): Entry\n", __func__);
	gpio_direction_output(shsmartamp_en, 0);
	msleep(100);

	qpnp_pin_config(SHAUDIO_GPIO_CLK, &shsmartamp_pin_cfg_on);
	gpio_direction_output(shsmartamp_en, 1);

	ret = shsmartamp_fw_load();
	if (ret) {
		pr_err("%s(): Failed for loading new firmware %d", __func__, ret);
		gpio_direction_output(shsmartamp_en, 0);
		return ret;
	}

	ret = shsmartamp_i2c_write(ssm4329_sw_control[SSM4329_SW_OFF],
		SSM4329_SW_CONTROL_Y, SSM4329_DATA);

	qpnp_pin_config(SHAUDIO_GPIO_CLK, &shsmartamp_pin_cfg_off);
	pr_debug("%s(): Leave\n", __func__);

	return ret;
}
#endif

static int shsmartamp_fw_init(void)
{
	int ret;
	int i, j;

	pr_debug("%s() SSM4329_PACKAGE = %d\n", __func__, SSM4329_PACKAGE);

	qpnp_pin_config(SHAUDIO_GPIO_CLK, &shsmartamp_pin_cfg_on);

	gpio_direction_output(shsmartamp_en, 1);
	for (i = 0; i < SSM4329_FW_SIGNEL_X; i++) {
		ret = shsmartamp_i2c_write(ssm4329_fw_signel[i],
			SSM4329_FW_SIGNEL_Y, SSM4329_DATA);
		if (ret != 0)
			break;
		if (i == 28) {
			ret = shsmartamp_i2c_write(ssm4329_fw_large_0800,
				sizeof(ssm4329_fw_large_0800)/sizeof(uint8_t),
				SSM4329_PROGRAM);
			if (ret != 0)
				break;
			ret = shsmartamp_i2c_write(ssm4329_fw_large_0000,
				sizeof(ssm4329_fw_large_0000)/sizeof(uint8_t),
				SSM4329_DATA);
			if (ret != 0)
				return ret;
			ret = shsmartamp_i2c_write(ssm4329_fw_large_177f,
				sizeof(ssm4329_fw_large_177f)/sizeof(uint8_t),
				SSM4329_DATA);
			if (ret != 0)
				return ret;
			pr_debug(" %s [i=28]write fw_medium to 101 or 102 or 201 or 301\n",__func__);
			for (j = 0; j < SSM4329_FW_MEDIUM_X; j++) {
				ret = shsmartamp_i2c_write(ssm4329_fw_medium[j],
					SSM4329_FW_MEDIUM_Y,
					SSM4329_DATA);
				if (ret != 0)
					break;
			}
		}
	}

#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 101 || CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 201 || CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 301)
    pr_debug(" %s: little data only for 8996 models\n",__func__);
    for (i = 0; i < SSM4329_FW_LITTLE_X; i++)
    {
        ret = shsmartamp_i2c_write(ssm4329_fw_little_017C_0190[i],
                SSM4329_FW_LITTLE_Y, SSM4329_DATA);

        if (ret != 0)
            break;
    }
#endif

	ret = shsmartamp_i2c_write(ssm4329_sw_control[SSM4329_SW_OFF],
		SSM4329_SW_CONTROL_Y, SSM4329_DATA);

	qpnp_pin_config(SHAUDIO_GPIO_CLK, &shsmartamp_pin_cfg_off);

	return ret;
}

static int shsmartamp_fw_check(void)
{
	int ret;
	int i;
	int output = 0xFF;

	pr_debug("%s()\n", __func__);

	for (i = 0; i < SSM4329_FW_SIGNEL_X; i++) {
		ret = shsmartamp_i2c_read(ssm4329_fw_signel[i], &output);
		if (ret != 0)
			break;
	}

	return ret;
}

static int shsmartamp_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	pr_debug("%s()\n", __func__);

	mutex_lock(&smartamp_lock);

	if (shsmartamp_opened) {
		pr_err("%s(): busy\n", __func__);
		rc = -EBUSY;
		goto done;
	}

	shsmartamp_opened = 1;
done:
	mutex_unlock(&smartamp_lock);
	return rc;
}

static int shsmartamp_release(struct inode *inode, struct file *file)
{
	pr_debug("%s()\n", __func__);

	mutex_lock(&smartamp_lock);
	shsmartamp_opened = 0;
	mutex_unlock(&smartamp_lock);
	return 0;
}

static struct file_operations shsmartamp_fops = {
	.owner   = THIS_MODULE,
	.open    = shsmartamp_open,
	.release = shsmartamp_release,
};

static struct miscdevice shsmartamp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "shsmartamp",
	.fops = &shsmartamp_fops,
};

void shsmartamp_control(int on)
{
	static int is_on;
	int i, ret, output, count_lock;

	pr_debug("%s()\n", __func__);

	mutex_lock(&smartamp_lock);
	output = 0xFF;
	count_lock = 0;
	if (on && !is_on) {
		qpnp_pin_config(SHAUDIO_GPIO_CLK, &shsmartamp_pin_cfg_on);
		shsmartamp_i2c_write(ssm4329_sw_control[SSM4329_SW_ON],
			SSM4329_SW_CONTROL_Y, SSM4329_DATA);

		for(i = 0; i < 30; i++){
			usleep_range(1 * 1000, 1 * 1000);
			ret = shsmartamp_i2c_read(ssm4329_pll_lock, &output);
			if(output & 0x01){
				pr_debug("%s()  PLL already locked!\n",__func__);
				count_lock++;
			}else{
				pr_debug("%s()  PLL still unlocked...\n",__func__);
			}
			if(count_lock >= 2){
				break;
			}
		}
		if(i == 30){
			pr_err("%s()  loop_end: PLL still unlocked...\n",__func__);
		}
		is_on = 1;
		pr_debug("%s: ON\n", __func__);
	} else if (!on && is_on){
		shsmartamp_i2c_write(ssm4329_sw_control[SSM4329_SW_OFF],
			SSM4329_SW_CONTROL_Y, SSM4329_DATA);
		is_on = 0;
		qpnp_pin_config(SHAUDIO_GPIO_CLK, &shsmartamp_pin_cfg_off);
		msleep(30);
		pr_debug("%s: OFF\n", __func__);
	}
	mutex_unlock(&smartamp_lock);

	return ;
}

#ifdef SHSMARTAMP_ENG
static int shsmartamp_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t shsmartamp_reg_read_file(struct file *file, char __user *user_buf,
										size_t count, loff_t *pos)
{
	shsmartamp_fw_check();
	return 0;
}

static ssize_t shsmartamp_reg_write_file(struct file *file,
						const char __user *user_buf, size_t count, loff_t *ppos)
{
	shsmartamp_fw_update();
	return 0;
}

static const struct file_operations shsmartamp_dbg_ops = {
	.open = shsmartamp_debug_open,
	.read = shsmartamp_reg_read_file,
	.write = shsmartamp_reg_write_file,
};

static void shsmartamp_init_debugfs(void)
{
	shsmartamp_dbgfile = debugfs_create_file("shsmartamp", S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
		NULL, NULL, &shsmartamp_dbg_ops);
	if (shsmartamp_dbgfile == NULL || IS_ERR(shsmartamp_dbgfile)) {
		pr_err("%s debugfs_create_file failed: ret=%ld\n", __func__, PTR_ERR(shsmartamp_dbgfile));
	}
}

static void shsmartamp_cleanup_debugfs(void)
{
	debugfs_remove(shsmartamp_dbgfile);
}
#else
static void shsmartamp_init_debugfs(void)
{
}

static void shsmartamp_cleanup_debugfs(void)
{
}
#endif

static int shsmartamp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	pr_debug("%s()\n", __func__);

	shsmartamp_client = client;

	ret = misc_register(&shsmartamp_device);
	if (ret) {
		pr_err("%s(): shsmartamp_device register failed\n", __func__);
	}

	shsmartamp_en = of_get_named_gpio(client->dev.of_node,
		"sharp,shsmartamp-en-gpio", 0);

	if (shsmartamp_en < -1) {
		pr_err("%s(): Get gpio info failed shsmartamp_en = %d\n",
			__func__, shsmartamp_en);
	} else {
		pr_debug("%s(): shsmartamp_en = %d\n", __func__, shsmartamp_en);
	}

	ret = gpio_request(shsmartamp_en, "shsmartamp");
	if (ret) {
		pr_err("%s(): Failed to request gpio %d\n", __func__,
			shsmartamp_en);
		goto err_free_gpio;
	}

	if (0 != shsmartamp_fw_init()) {
		pr_err("%s(): firmware init failed\n", __func__);
		shsmartamp_fw_check();
		goto err_free_gpio;
	} else {
		pr_err("%s(): firmware init successful\n", __func__);
	}

	shsmartamp_init_debugfs();

	return 0;

err_free_gpio:
	gpio_free(shsmartamp_en);
	shsmartamp_en = -1;
	return ret;
}

static int shsmartamp_remove(struct i2c_client *client)
{
	shsmartamp_client = i2c_get_clientdata(client);
	shsmartamp_cleanup_debugfs();
	return 0;
}

static const struct i2c_device_id shsmartamp_id[] = {
	{ SHSMARTAMP_I2C_NAME, 0 },
	{ }
};

static const struct of_device_id mdss_dsi_ctrl_dt_match[] = {
	{.compatible = "sharp,shsmartamp_i2c"},
	{}
};
MODULE_DEVICE_TABLE(of, mdss_dsi_ctrl_dt_match);

static struct i2c_driver shsmartamp_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = SHSMARTAMP_I2C_NAME,
		.of_match_table = mdss_dsi_ctrl_dt_match,
	},
	.class    = I2C_CLASS_HWMON,
	.probe    = shsmartamp_probe,
	.id_table = shsmartamp_id,
	.remove   = shsmartamp_remove,
};

static int __init shsmartamp_init(void)
{
	pr_debug("%s()\n", __func__);
	return i2c_add_driver(&shsmartamp_driver);
}

module_init(shsmartamp_init);

MODULE_DESCRIPTION("shsmartamp driver");
MODULE_LICENSE("GPL");
