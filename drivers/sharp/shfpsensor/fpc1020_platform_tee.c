/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, controlling GPIOs such as sensor reset
 * line, sensor IRQ line.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node.
 *
 * This driver will NOT send any commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
#ifdef CONFIG_SHTERM
#include "sharp/shterm_k.h"
#endif /* CONFIG_SHTERM */
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

#define RESET_LOW_SLEEP_MIN_US 1000
#define RESET_LOW_SLEEP_MAX_US (RESET_LOW_SLEEP_MIN_US + 100)
#define RESET_HIGH_SLEEP_MIN_US 100
#define RESET_HIGH_SLEEP_MAX_US (RESET_HIGH_SLEEP_MIN_US + 100)
#define PWR_ON_SLEEP_MIN_US 100
#define PWR_ON_SLEEP_MAX_US (PWR_ON_SLEEP_MIN_US + 900)

#define NUM_PARAMS_REG_ENABLE_SET 2

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
enum SHFP_LOG_FLG {
    SHFP_FLG_VERBOSE = 1 << 0,
    SHFP_FLG_IRQ   = 1 << 1,
};

static int sh_debug_shfpsensor = 0;
module_param(sh_debug_shfpsensor, int, 0664);

#define shfp_sensor_log(flg, dev, fmt, ...) \
    if (sh_debug_shfpsensor & flg) {\
            dev_info(dev, fmt, ##__VA_ARGS__);\
    }
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

static const char * const pctl_names[] = {
	"fpc1020_reset_reset",
	"fpc1020_reset_active",
	"fpc1020_irq_active",
};

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

static const struct vreg_config const vreg_conf[] = {
#ifndef CONFIG_SENSORS_FPRINT_SH_CUST
	{ "vdd_ana", 1800000UL, 1800000UL, 6000, },
	{ "vcc_spi", 1800000UL, 1800000UL, 10, },
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	{ "vdd_io", 1800000UL, 1800000UL, 6000, },
};

struct fpc1020_data {
	struct device *dev;

	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
	struct regulator *vreg[ARRAY_SIZE(vreg_conf)];

	int irq_gpio;
	int rst_gpio;
	struct mutex lock; /* To set/get exported values in sysfs */
	bool prepared;
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	int irqcounter_enable;
#endif
};

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
static int shfp_irqcount = 0;
#endif

static int vreg_setup(struct fpc1020_data *fpc1020, const char *name,
	bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;
	struct device *dev = fpc1020->dev;

	for (i = 0; i < ARRAY_SIZE(fpc1020->vreg); i++) {
		const char *n = vreg_conf[i].name;

		if (!strncmp(n, name, strlen(n)))
			goto found;
	}

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	dev_dbg(dev, "Regulator %s not found\n", name);
	return 0;
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
	dev_err(dev, "Regulator %s not found\n", name);
	return -EINVAL;
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

found:
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[S]%s name=%s enable=%d\n", __func__, name, enable);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	vreg = fpc1020->vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = regulator_get(dev, name);
			if (IS_ERR(vreg)) {
				dev_err(dev, "Unable to get %s\n", name);
				return PTR_ERR(vreg);
			}
		}

		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc)
				dev_err(dev,
					"Unable to set voltage on %s, %d\n",
					name, rc);
		}

		rc = regulator_set_optimum_mode(vreg, vreg_conf[i].ua_load);
		if (rc < 0)
			dev_err(dev, "Unable to set current on %s, %d\n",
					name, rc);

		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "error enabling %s: %d\n", name, rc);
			regulator_put(vreg);
			vreg = NULL;
		}
		fpc1020->vreg[i] = vreg;
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
		shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "enable %s\n", name);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
				shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "disabled %s\n", name);
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
				dev_dbg(dev, "disabled %s\n", name);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
			}
			regulator_put(vreg);
			fpc1020->vreg[i] = NULL;
		}
		rc = 0;
	}
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
#ifdef CONFIG_SHTERM
	if (fpc1020->vreg[i] != NULL) {
		shterm_k_set_info( SHTERM_INFO_FINGER_AUTH, 1);
	} else {
		shterm_k_set_info( SHTERM_INFO_FINGER_AUTH, 0);
	}
#endif
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[E]%s rc=%d\n", __func__, rc);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

	return rc;
}

/**
 * sysfs node for controlling clocks.
 *
 * This is disabled in platform variant of this driver but kept for
 * backwards compatibility. Only prints a debug print that it is
 * disabled.
 */
static ssize_t clk_enable_set(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev,
		"clk_enable sysfs node not enabled in platform driver\n");
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
	dev_dbg(dev,
		"clk_enable sysfs node not enabled in platform driver\n");
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

	return count;
}
static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, clk_enable_set);

/**
 * Will try to select the set of pins (GPIOS) defined in a pin control node of
 * the device tree named @p name.
 *
 * The node can contain several eg. GPIOs that is controlled when selecting it.
 * The node may activate or deactivate the pins it contains, the action is
 * defined in the device tree node itself and not here. The states used
 * internally is fetched at probe time.
 *
 * @see pctl_names
 * @see fpc1020_probe
 */
static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
	size_t i;
	int rc;
	struct device *dev = fpc1020->dev;

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[S]%s name=%s\n", __func__, name);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];

		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
					fpc1020->pinctrl_state[i]);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
				shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "Selected '%s'\n", name);
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
				dev_dbg(dev, "Selected '%s'\n", name);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
			goto exit;
		}
	}

	rc = -EINVAL;
	dev_err(dev, "%s:'%s' not found\n", __func__, name);

exit:
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[E]%s rc=%d\n", __func__, rc);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	return rc;
}

static ssize_t pinctl_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc;

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[S]%s count=%d\n", __func__, (int)count);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	mutex_lock(&fpc1020->lock);
	rc = select_pin_ctl(fpc1020, buf);
	mutex_unlock(&fpc1020->lock);
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[E]%s rc=%d\n", __func__, rc ? rc : (int)count);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

	return rc ? rc : count;
}
static DEVICE_ATTR(pinctl_set, S_IWUSR, NULL, pinctl_set);

static ssize_t regulator_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	char op;
	char name[16];
	int rc;
	bool enable;
    int i;

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[S]%s count=%d\n", __func__, (int)count);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
    for (i=0; i<15 && buf[i]!=','; i++) {
        name[i] = buf[i];
    }

    op = buf[i+1];
    name[i] = 0;
    if (!(buf[i]==',' && (op=='d' || op=='e'))) {
        return -EINVAL;
    }

	if (op == 'e')
		enable = true;
	else if (op == 'd')
		enable = false;
	else
		return -EINVAL;

	mutex_lock(&fpc1020->lock);
	rc = vreg_setup(fpc1020, name, enable);
	mutex_unlock(&fpc1020->lock);

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[E]%s rc=%d\n", __func__, rc ? rc : (int)count);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	return rc ? rc : count;
}
static DEVICE_ATTR(regulator_enable, S_IWUSR, NULL, regulator_enable_set);

static int hw_reset(struct fpc1020_data *fpc1020)
{
	int irq_gpio;
	struct device *dev = fpc1020->dev;
	int rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[S]%s\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	if (rc)
		goto exit;
	usleep_range(RESET_HIGH_SLEEP_MIN_US, RESET_HIGH_SLEEP_MAX_US);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;
	usleep_range(RESET_LOW_SLEEP_MIN_US, RESET_LOW_SLEEP_MAX_US);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;
	usleep_range(RESET_HIGH_SLEEP_MIN_US, RESET_HIGH_SLEEP_MAX_US);

	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_IRQ, dev, "IRQ after reset %d\n", irq_gpio);
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
	dev_info(dev, "IRQ after reset %d\n", irq_gpio);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

exit:
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[E]%s rc=%d\n", __func__, rc);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	return rc;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[S]%s name=%s count=%d\n", __func__, buf, (int)count);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	if (!strncmp(buf, "reset", strlen("reset"))) {
		mutex_lock(&fpc1020->lock);
		rc = hw_reset(fpc1020);
		mutex_unlock(&fpc1020->lock);
	} else {
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
		shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[E]%s name=%s not found\n", __func__, buf);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
		return -EINVAL;
	}
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[E]%s rc=%d\n", __func__, rc ? rc : (int)count);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

/**
 * Will setup GPIOs, and regulators to correctly initialize the touch sensor to
 * be ready for work.
 *
 * In the correct order according to the sensor spec this function will
 * enable/disable regulators, and reset line, all to set the sensor in a
 * correct power on or off state "electrical" wise.
 *
 * @see  device_prepare_set
 * @note This function will not send any commands to the sensor it will only
 *       control it "electrically".
 */
static int device_prepare(struct fpc1020_data *fpc1020, bool enable)
{
	int rc;

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[S]%s enable=%d\n", __func__, enable);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	mutex_lock(&fpc1020->lock);
	if (enable && !fpc1020->prepared) {
		fpc1020->prepared = true;
		select_pin_ctl(fpc1020, "fpc1020_reset_reset");

		rc = vreg_setup(fpc1020, "vcc_spi", true);
		if (rc)
			goto exit;

		rc = vreg_setup(fpc1020, "vdd_io", true);
		if (rc)
			goto exit_1;

		rc = vreg_setup(fpc1020, "vdd_ana", true);
		if (rc)
			goto exit_2;

		usleep_range(PWR_ON_SLEEP_MIN_US, PWR_ON_SLEEP_MAX_US);

		/* As we can't control chip select here the other part of the
		 * sensor driver eg. the TEE driver needs to do a _SOFT_ reset
		 * on the sensor after power up to be sure that the sensor is
		 * in a good state after power up. Okeyed by ASIC. */

		(void)select_pin_ctl(fpc1020, "fpc1020_reset_active");
	} else if (!enable && fpc1020->prepared) {
		rc = 0;
		(void)select_pin_ctl(fpc1020, "fpc1020_reset_reset");

		usleep_range(PWR_ON_SLEEP_MIN_US, PWR_ON_SLEEP_MAX_US);

		(void)vreg_setup(fpc1020, "vdd_ana", false);
exit_2:
		(void)vreg_setup(fpc1020, "vdd_io", false);
exit_1:
		(void)vreg_setup(fpc1020, "vcc_spi", false);
exit:
		fpc1020->prepared = false;
	} else {
		rc = 0;
	}
	mutex_unlock(&fpc1020->lock);

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[E]%s rc=%d\n", __func__, rc);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	return rc;
}

/**
 * sysfs node to enable/disable (power up/power down) the touch sensor
 *
 * @see device_prepare
 */
static ssize_t device_prepare_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[S]%s count=%d\n", __func__, (int)count);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	if (!strncmp(buf, "enable", strlen("enable")))
		rc = device_prepare(fpc1020, true);
	else if (!strncmp(buf, "disable", strlen("disable")))
		rc = device_prepare(fpc1020, false);
	else
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	{
		shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[E]%s -EINVAL\n", __func__);
		return -EINVAL;
	}
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
		return -EINVAL;
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[E]%s rc=%d\n", __func__, rc ? rc : (int)count);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	return rc ? rc : count;
}
static DEVICE_ATTR(device_prepare, S_IWUSR, NULL, device_prepare_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "%s count=%d\n", __func__, (int)count);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

	return count;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int irq = gpio_get_value(fpc1020->irq_gpio);

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	ssize_t ret = 0;
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[S]%s\n", __func__);
	ret =scnprintf(buf, PAGE_SIZE, "%i\n", irq);
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[E]%s ret=%d buf=%s", __func__, (int)ret, buf);
	return ret;
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
	return scnprintf(buf, PAGE_SIZE, "%i\n", irq);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
}

/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_IRQ, fpc1020->dev, "%s\n", __func__);
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
	dev_dbg(fpc1020->dev, "%s\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

	return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
static ssize_t sh_irqcounter_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = count;

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[S]%s count=%d\n", __func__, (int)count);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	mutex_lock(&fpc1020->lock);
	if (!strncmp(buf, "enable", strlen("enable"))) {
		fpc1020->irqcounter_enable = 1;
		shfp_irqcount = 0;
	}
	else if (!strncmp(buf, "disable", strlen("disable"))) {
		fpc1020->irqcounter_enable = 0;
		shfp_irqcount = 0;
	}
	else {
		ret = -EINVAL;
	}
	mutex_unlock(&fpc1020->lock);
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[E]%s ret=%d\n", __func__, (int)ret);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

	return ret;
}
static DEVICE_ATTR(irqcounter_enable, S_IWUSR, NULL, sh_irqcounter_enable);

static ssize_t sh_irqcount_get(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = 0;
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[S]%s buf=%s\n", __func__, buf);
	ret = scnprintf(buf, PAGE_SIZE, "%d\n", shfp_irqcount);
	shfp_sensor_log(SHFP_FLG_VERBOSE, fpc1020->dev, "[E]%s ret=%d\n", __func__, (int)ret);
	return ret;
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */

	return scnprintf(buf, PAGE_SIZE, "%d\n", shfp_irqcount);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
}
static DEVICE_ATTR(irqcount, S_IRUSR, sh_irqcount_get, NULL);
#endif

static struct attribute *attributes[] = {
	&dev_attr_pinctl_set.attr,
	&dev_attr_device_prepare.attr,
	&dev_attr_regulator_enable.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	&dev_attr_irqcounter_enable.attr,
	&dev_attr_irqcount.attr,
#endif
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_IRQ, fpc1020->dev, "%s\n", __func__);
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
	dev_dbg(fpc1020->dev, "%s\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	if (fpc1020->irqcounter_enable) {
		shfp_irqcount++;
	}
#endif

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
	const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[S]%s label=%s gpio=%d\n", __func__, label, *gpio);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;

	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "[E]%s\n", __func__);
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
	dev_dbg(dev, "%s %d\n", label, *gpio);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

	return 0;
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc = 0;
	size_t i;
	int irqf;
	struct device_node *np = dev->of_node;
	struct fpc1020_data *fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020),
			GFP_KERNEL);

	if (!fpc1020) {
		dev_err(dev,
			"failed to allocate memory for struct fpc1020_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->dev = dev;
	platform_set_drvdata(pdev, fpc1020);

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_irq",
			&fpc1020->irq_gpio);
	if (rc)
		goto exit;
	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_rst",
			&fpc1020->rst_gpio);
	if (rc)
		goto exit;

	fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
		if (PTR_ERR(fpc1020->fingerprint_pinctrl) == -EPROBE_DEFER) {
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
			shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "pinctrl not ready\n");
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
			dev_info(dev, "pinctrl not ready\n");
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
			rc = -EPROBE_DEFER;
			goto exit;
		}
		dev_err(dev, "Target does not use pinctrl\n");
		fpc1020->fingerprint_pinctrl = NULL;
		rc = -EINVAL;
		goto exit;
	}

	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			dev_err(dev, "cannot find '%s'\n", n);
			rc = -EINVAL;
			goto exit;
		}
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
		shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "found pin control %s\n", n);
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
		dev_info(dev, "found pin control %s\n", n);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
		fpc1020->pinctrl_state[i] = state;
	}

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;
	rc = select_pin_ctl(fpc1020, "fpc1020_irq_active");
	if (rc)
		goto exit;


#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	fpc1020->irqcounter_enable = 0;
#endif

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;

	mutex_init(&fpc1020->lock);
	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto exit;
	}

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
	dev_dbg(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */


	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

	if (of_property_read_bool(dev->of_node, "fpc,enable-on-boot")) {
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
		shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "Enabling hardware\n");
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
		dev_info(dev, "Enabling hardware\n");
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
		(void)device_prepare(fpc1020, true);
	}

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, dev, "%s: ok\n", __func__);
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
	dev_info(dev, "%s: ok\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

exit:
	return rc;
}

static int fpc1020_remove(struct platform_device *pdev)
{
	struct fpc1020_data *fpc1020 = platform_get_drvdata(pdev);

#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, &pdev->dev, "[S]%s\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */
	sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
	mutex_destroy(&fpc1020->lock);

	(void)vreg_setup(fpc1020, "vdd_ana", false);
	(void)vreg_setup(fpc1020, "vdd_io", false);
	(void)vreg_setup(fpc1020, "vcc_spi", false);
#ifdef CONFIG_SENSORS_FPRINT_SH_CUST
	shfp_sensor_log(SHFP_FLG_VERBOSE, &pdev->dev, "[E]%s\n", __func__);
#else /* CONFIG_SENSORS_FPRINT_SH_CUST */
	dev_info(&pdev->dev, "%s\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SH_CUST */

	return 0;
}

static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name	= "fpc1020",
		.owner	= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
	},
	.probe	= fpc1020_probe,
	.remove	= fpc1020_remove,
};

static int __init fpc1020_init(void)
{
	int rc = platform_driver_register(&fpc1020_driver);

	if (!rc)
		pr_info("%s OK\n", __func__);
	else
		pr_err("%s %d\n", __func__, rc);

	return rc;
}

static void __exit fpc1020_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&fpc1020_driver);
}

module_init(fpc1020_init);
module_exit(fpc1020_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
