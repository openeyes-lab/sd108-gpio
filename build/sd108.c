/*
 * sd108.c - Part of OPEN-EYES-II products, Linux kernel modules for GPIO 
 * expansion.
 * This driver handles the SD108 FW running on ATTINY817.
 * Author:
 * Massimiliano Negretti <massimiliano.negretti@open-eyes.it> 2020-07-12
 *
 * This driver enable 3 different feature:
 * 1) GPIO
 * 2) PWM
 * 3) LCD BACKLIGHT
 *
 * This file is part of sd108-gpio distribution
 * https://github.com/openeyes-lab/sd108-gpio
 *
 * Copyright (c) 2021 OPEN-EYES Srl 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/acpi.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/bitmap.h>
#include <linux/pwm_backlight.h>
#include <linux/backlight.h>

#define SD108_NUM_REGS                  16
#define SD108_NPWM_LED                  2
#define SD108_MAXGPIO                   1

/* define the base of filesys: /sys/class/pwm/pwmchip12 */
#define SD108_PWM_BASE                  12

#define SD108_CHIP_ID_REG               0x00
#define SD108_CHIP_ID                   0xd108
#define SD108_CHIP_VER_REG              0x01

#define SD108_CHIP_LED_REQUEST          0x03
#define SD108_CHIP_LED_PERIOD           0x04
#define SD108_CHIP_LED_RIGHT            0x05
#define SD108_CHIP_LED_LEFT             0x06

#define SD108_CHIP_BACKLIGHT_PERIOD     0x08
#define SD108_CHIP_BACKLIGHT            0x09

#define SD108_CHIP_GPIO_DIRECTION       0x0B
#define SD108_CHIP_GPIO_VALUE           0x0C

#define SD108_CHIP_COMMAND              0x0E
#define SD108_CHIP_SET_SLEEP_CMD        0x8001
#define SD108_CHIP_SET_ACTIVE_CMD       0x8002
#define SD108_CHIP_STATUS               0x0F
#define SD108_CHIP_ACTIVE_FLAG          0x0001
#define SD108_CHIP_SLEEP_FLAG           0x0002

struct sd108 {
	struct device                      *dev;
	struct pwm_chip                    chip;
	struct backlight_device            *bl;
	struct regmap                      *regmap;
	struct platform_pwm_backlight_data lcd;
	struct gpio_chip                   gpio;
	struct pwm_state                   state[SD108_NPWM_LED];
	bool                               led_request[SD108_NPWM_LED];
	int                                fw_revision;
	int                                chip_status;
	int                                gpio_val;
	int                                gpio_dir;
	struct mutex                       lock;
};

static inline struct sd108 *pwm_to_sd108(struct pwm_chip *chip)
{
	return container_of(chip, struct sd108, chip);
}

static inline struct sd108 *backlight_to_sd108(struct backlight_device *bl)
{
	return container_of(bl, struct sd108, bl);
}

/**
 * GPIO CALLBACK
 */

static int sd108_gpio_get_value(struct gpio_chip *chip, unsigned off)
{
	struct sd108 *data = gpiochip_get_data(chip);
	int val;
	int bit = 1<<off;
	int ret;

	//mutex_lock(&data->lock);

	/* Get GPIO value */
	ret = regmap_read(data->regmap, SD108_CHIP_GPIO_VALUE, &val);
	if (ret < 0) {
		dev_err(data->dev, "sd108_gpio_get_value failed to read I2C\n");
		return ret;
	}

	data->gpio_val = val;

	//mutex_unlock(&dev->lock);

	return !!(val & bit);
}

static void sd108_gpio_set_value(struct gpio_chip *chip, unsigned off, int val)
{
	struct sd108 *data = gpiochip_get_data(chip);
	int value = data->gpio_val;
	int bit = 1<<off;
	int ret;

	//mutex_lock(&dev->lock);

	if (val)
		value |= bit;
	else
		value &= ~bit;

	/* Set GPIO value */
	ret = regmap_write(data->regmap, SD108_CHIP_GPIO_VALUE, value);
	if (ret < 0) {
		dev_err(data->dev, "sd108_gpio_set_value failed to read I2C\n");
		return;
	}

	data->gpio_val = value;

	//mutex_unlock(&dev->lock);
}

static int sd108_gpio_direction_input(struct gpio_chip *chip, unsigned off)
{
	struct sd108 *data = gpiochip_get_data(chip);
	int direction = data->gpio_dir & ~(1<<off);
	int ret;

	//mutex_lock(&dev->lock);
	/* Set GPIO as input  */
	ret = regmap_write(data->regmap, SD108_CHIP_GPIO_DIRECTION, direction);
	if (ret < 0) {
		dev_err(data->dev, "sd108_gpio_set_value failed to read I2C\n");
		return ret;
	}
	data->gpio_dir = direction;
	//mutex_unlock(&dev->lock);

	return 0;
}

static int sd108_gpio_direction_output(struct gpio_chip *chip,
					 unsigned off, int val)
{
	struct sd108 *data = gpiochip_get_data(chip);
	int direction = data->gpio_dir | (1<<off);
	int ret;

	sd108_gpio_set_value(chip,off,val);

	//mutex_lock(&dev->lock);
	/* Set GPIO as output  */
	ret = regmap_write(data->regmap, SD108_CHIP_GPIO_DIRECTION, direction);
	if (ret < 0) {
		dev_err(data->dev, "sd108_gpio_set_value failed to read I2C\n");
		return ret;
	}
	data->gpio_dir = direction;
	//mutex_unlock(&dev->lock);

	return 0;
}

/** **************************************************************************
 * PWM BACKLIGHT CONTROL CALLBACK
 *****************************************************************************/
/**
 * @brief BACKLIGHT update status
 * @param [in] bl backlight_device struct descriptor
 * @return 0 on success.
 * @details This is the backlight callback that handles the LCD backlight
 * PWM duty cycle.
 */
static int sd108_backlight_update_status(struct backlight_device *bl)
{
	struct sd108 *data = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int pwmbri;
	int ret = 0;

	mutex_lock(&data->lock);

 	if (bl->props.power != FB_BLANK_UNBLANK ||
 	    bl->props.fb_blank != FB_BLANK_UNBLANK ||
 	    bl->props.state & BL_CORE_FBBLANK)
 		brightness = 0;

	if (brightness>0) {
		pwmbri = (brightness*100)/data->lcd.max_brightness;
		pwmbri |= 0x8000;
	} else {
		pwmbri = 0;
	}

	ret = regmap_write(data->regmap, SD108_CHIP_BACKLIGHT, pwmbri);
	if (ret < 0) {
		dev_err(data->dev, "pwm_backlight_update_status fail i2c write\n");
	}

	mutex_unlock(&data->lock);
	return ret;
}

static void sd108_set_sleep_mode(struct sd108 *data, bool enable)
{
	int cmd = (enable)?SD108_CHIP_SET_ACTIVE_CMD:SD108_CHIP_SET_SLEEP_CMD;
	int ret = regmap_write(data->regmap, SD108_CHIP_COMMAND, cmd);
	if (ret < 0) {
		dev_err(data->dev, "sd108_set_sleep_mode fail i2c write\n");
	}
}


/** **************************************************************************
 * PWM FRONT LEDS CONTROL CALLBACK
 *****************************************************************************/

/**
 * @brief PWM request callback
 * @param [in] chip pwm_chip struct descriptor
 * @param [in] pwm struct pwm_device descriptor
 * @return 0 on success.
 * @details This is the PWM callback that handles the PWM creation on the sd108
 * chip when the correct index is exported.
 */
static int sd108_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct sd108 *data = pwm_to_sd108(chip);
	unsigned int request = 0;
	int          ret = -EINVAL;
	int          i;

	mutex_lock(&data->lock);

	if (pwm->hwpwm<SD108_NPWM_LED) {
		if (!data->led_request[pwm->hwpwm]) {
			/* Set LED request */
			for (i=0;i<SD108_NPWM_LED;i++)
				request |= (data->led_request[i]&1)<<i;
			request |= 1<<pwm->hwpwm;

			// write request on chip
			ret = regmap_write(data->regmap, SD108_CHIP_LED_REQUEST, request);
			if (ret < 0) {
				dev_err(data->dev, "PWM request fail i2c write\n");
				goto close;
			}
			// flag request done if transaction OK!
			data->led_request[pwm->hwpwm]=1;
		}
		ret = 0;
	}

close:
	mutex_unlock(&data->lock);
	return ret;
}

/**
 * @brief PWM free callback
 * @param [in] chip pwm_chip struct descriptor
 * @param [in] pwm struct pwm_device descriptor
 * @return None.
 * @details This is the PWM callback that handles the PWM destruction on sd108
 * chip when the correct index is unexported.
 */
static void sd108_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct sd108 *data = pwm_to_sd108(chip);
	unsigned int request = 0;
	int          ret;
	int          i;

	mutex_lock(&data->lock);

	if (pwm->hwpwm<SD108_NPWM_LED) {
		if (data->led_request[pwm->hwpwm]) {
			/* Set LED request */
			for (i=0;i<SD108_NPWM_LED;i++)
				if (i!=pwm->hwpwm)
					request |= (data->led_request[i]&1)<<i;

			// write request on chip
			ret = regmap_write(data->regmap, SD108_CHIP_LED_REQUEST, request);
			if (ret < 0) {
				dev_err(data->dev, "PWM free fail i2c write\n");
				goto close;
			}
			// flag request if transaction OK!
			data->led_request[pwm->hwpwm]=0;
		}
	}

close:
	mutex_unlock(&data->lock);
}

/**
 * @brief PWM get state callback
 * @param [in] chip pwm_chip struct descriptor
 * @param [in] pwm struct pwm_device descriptor
 * @param [out] state current pwm state
 * @return None.
 * @details This is the PWM callback that handles the PWM status request,
 * that is called when pwm files are read.
 * TODO: add time controls to avoid continue i2c request
 */
static void sd108_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct sd108 *data = pwm_to_sd108(chip);
	unsigned int val;
	unsigned int reg;
	int          ret;

	if (!state)
		return;

	mutex_lock(&data->lock);

	if (pwm->hwpwm<SD108_NPWM_LED) {
		/* Get LED period */
		ret = regmap_read(data->regmap, SD108_CHIP_LED_PERIOD, &val);
		if (ret < 0) {
			dev_err(data->dev, "Get state fail to read period\n");
			goto close;
		}

		state->period = val * 100000;

		reg = SD108_CHIP_LED_RIGHT + pwm->hwpwm;

		/* Get LED status */
		ret = regmap_read(data->regmap, reg, &val);
		if (ret < 0) {
			dev_err(data->dev, "Get state fail to read PWM%d status\n",pwm->hwpwm);
			goto close;
		}

		state->polarity = PWM_POLARITY_NORMAL;
		state->enabled = (val&0x8000)?true:false;
		state->duty_cycle = (val&0x7fff) * 1000000;

		// update local state struct
		memcpy(&data->state[pwm->hwpwm],state,sizeof(struct pwm_state));
	}

close:
	mutex_unlock(&data->lock);
}

/**
 * @brief PWM set state callback
 * @param [in] chip pwm_chip struct descriptor
 * @param [in] pwm struct pwm_device descriptor
 * @param [out] state current pwm state
 * @return 0 on success.
 * @details This is the PWM callback that handles the PWM status setting,
 * that is called when pwm files are written.
 */
static int sd108_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			   const struct pwm_state *state)
{
	struct sd108 *data = pwm_to_sd108(chip);
	u64 period;
	u64 duty;
	unsigned int regmap_val;
	unsigned int reg;
	int          ret = -EINVAL;

	mutex_lock(&data->lock);

	if (pwm->hwpwm<SD108_NPWM_LED) {
		if (state->enabled) {
			// all changes must be done now
			if (state->period != data->state[pwm->hwpwm].period) {
				// The period is changed, remember that this value is sheared
				if (state->period>300000000)
					goto close;
				if (state->period<500000)
					goto close;
				period=state->period;
				do_div(period,100000);
				regmap_val = (unsigned int)(period);
				ret = regmap_write(data->regmap, SD108_CHIP_LED_PERIOD, regmap_val);
				if (ret < 0) {
					dev_err(data->dev, "PWM apply fail i2c write on period\n");
					goto close;
				}
				data->state[pwm->hwpwm].period = state->period;
			}

			if (state->duty_cycle!=data->state[pwm->hwpwm].duty_cycle) {
				// duty is changed
				duty = state->duty_cycle*100;
				do_div(duty,state->period);
				regmap_val = (unsigned int)(duty);
				regmap_val |= 0x8000;
				reg = SD108_CHIP_LED_RIGHT+pwm->hwpwm;
				ret = regmap_write(data->regmap, reg, regmap_val);
				if (ret < 0) {
					dev_err(data->dev, "PWM apply fail i2c write on duty\n");
					goto close;
				}
				data->state[pwm->hwpwm].duty_cycle = state->duty_cycle;
				data->state[pwm->hwpwm].enabled = 1;
			}
		}
		else
		{
			if (data->state[pwm->hwpwm].enabled) {
				// force disable
				//duty = state->duty_cycle*100/state->period;
				//duty |= 0x8000;

				reg = SD108_CHIP_LED_RIGHT+pwm->hwpwm;
				ret = regmap_write(data->regmap, reg, 0);
				if (ret < 0) {
					dev_err(data->dev, "PWM apply disable fail i2c write\n");
					goto close;
				}
				data->state[pwm->hwpwm].enabled = 0;
				data->state[pwm->hwpwm].duty_cycle = 0;
			}
		}
		ret = 0;
	}

close:
	mutex_unlock(&data->lock);
	return ret;
}

/** **************************************************************************
 * PWM FRONT LEDS OPS
 *****************************************************************************/
static const struct pwm_ops sd108_pwm_ops = {
	.apply = sd108_pwm_apply,
	.request = sd108_pwm_request,
	.free = sd108_pwm_free,
	.get_state = sd108_pwm_get_state,
	.owner = THIS_MODULE,
};

/** **************************************************************************
 * BACKLIGHT OPS
 *****************************************************************************/
static const struct backlight_ops sd108_bl_ops = {
	.update_status	= sd108_backlight_update_status,
};

static const struct regmap_config sd108_regmap_i2c_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = SD108_NUM_REGS,
	.cache_type = REGCACHE_NONE,
};

static int sd108_pwm_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct sd108 *data;
	struct backlight_device *bl;
	struct backlight_properties props;
	int ret,i;
	unsigned int val;
	size_t size;

	/* Driver memory allocation */
	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* initialize regmap */
	data->regmap = devm_regmap_init_i2c(client, &sd108_regmap_i2c_config);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(&client->dev, "Failed to initialize register map: %d\n",ret);
		return ret;
	}

	mutex_init(&data->lock);

	/* Fill and register LCD driver memory segment */
	//data->lcd.enable_gpio = -EINVAL;
	data->lcd.max_brightness = 10;
	size = sizeof(*data->lcd.levels) * data->lcd.max_brightness;
	data->lcd.levels = devm_kzalloc(&client->dev, size, GFP_KERNEL);
	if (!data->lcd.levels)
		return -ENOMEM;
	for(i=0;i<data->lcd.max_brightness;i++)
		data->lcd.levels[i]=i;
	data->lcd.dft_brightness = 5;

	i2c_set_clientdata(client, data);

	/* Verify that we have sd108 connected */
	ret = regmap_read(data->regmap, SD108_CHIP_ID_REG, &val);
	if (ret < 0) {
		dev_err(&client->dev, "Fail to get chip ID\n");
		return -ENODEV;
	}

	if (val!=SD108_CHIP_ID) {
		dev_err(&client->dev, "Invalid chip id: %x\n", val);
		return -ENODEV;
	}

	/* Get version */
	ret = regmap_read(data->regmap, SD108_CHIP_VER_REG, &val);
	if (ret < 0) {
		dev_err(&client->dev, "Fail to get chip revision\n");
		return -EIO;
	}
	data->fw_revision = val;

	/* Get status */
	ret = regmap_read(data->regmap, SD108_CHIP_STATUS, &val);
	if (ret < 0) {
		dev_err(&client->dev, "failed to get chip status I2C\n");
		return -EIO;
	}
	data->chip_status = val;

	/* Get LCD period */
	ret = regmap_read(data->regmap, SD108_CHIP_BACKLIGHT_PERIOD, &val);
	if (ret < 0) {
		dev_err(&client->dev, "failed to get PWM period\n");
		return -EIO;
	}
	data->lcd.pwm_period_ns = val*1000;

	/* Fill and register PWM memory segment */
	data->dev = &client->dev;
	data->chip.ops = &sd108_pwm_ops;
	data->chip.npwm = SD108_NPWM_LED;
	data->chip.dev = &client->dev;
	data->chip.base = SD108_PWM_BASE;

	ret = pwmchip_add(&data->chip);
	if (ret < 0) {
		dev_err(&client->dev, "Fail to add PWM chip\n");
	}

	/* the chip comes out of power-up in the active state */
	pm_runtime_set_active(&client->dev);
	/*
	 * enable will put the chip into suspend, which is what we
	 * want as all outputs are disabled at this point
	 */
	pm_runtime_enable(&client->dev);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.scale = BACKLIGHT_SCALE_LINEAR;
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->lcd.max_brightness;

	bl = devm_backlight_device_register(&client->dev,
				dev_driver_string(&client->dev),
				&client->dev, data, &sd108_bl_ops, &props);

	if (IS_ERR(bl)) {
		dev_err(&client->dev, "Fail to register backlight\n");
		return PTR_ERR(bl);
	}

	bl->props.brightness = data->lcd.dft_brightness;
	bl->props.power = FB_BLANK_UNBLANK;

	backlight_update_status(bl);

	sd108_set_sleep_mode(data, false);

	data->bl = bl;

	dev_set_drvdata(&bl->dev, data);

	/* GPIO setup */

	data->gpio.direction_input = sd108_gpio_direction_input;
	data->gpio.direction_output = sd108_gpio_direction_output;
	data->gpio.get = sd108_gpio_get_value;
	data->gpio.set = sd108_gpio_set_value;
	data->gpio.can_sleep = true;
	data->gpio.base = 208;
	data->gpio.parent = &client->dev;
	data->gpio.ngpio = SD108_MAXGPIO;
	data->gpio.label = client->name;
	data->gpio.owner = THIS_MODULE;

	ret = devm_gpiochip_add_data(&client->dev, &data->gpio, data);
	if (ret < 0) {
		dev_err(&client->dev, "failed add GPIO\n");
		return ret;
	}

	dev_info(&client->dev, "correctly probed (chip rev %d status=%x per=%dns)\n",
	 				data->fw_revision,data->chip_status,data->lcd.pwm_period_ns);

	return 0;
}

static int sd108_pwm_remove(struct i2c_client *client)
{
	struct sd108 *data = i2c_get_clientdata(client);
	int ret;

	ret = pwmchip_remove(&data->chip);
	if (ret)
		return ret;
	pm_runtime_disable(&client->dev);
	return 0;
}

static int sd108_pwm_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sd108 *data = i2c_get_clientdata(client);

	sd108_set_sleep_mode(data, true);
	return 0;
}

static int sd108_pwm_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sd108 *data = i2c_get_clientdata(client);

	sd108_set_sleep_mode(data, false);
	return 0;
}

static const struct i2c_device_id sd108_id[] = {
	{ "sd108", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, sd108_id);

static const struct of_device_id sd108_dt_ids[] = {
	{ .compatible = "sd108", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sd108_dt_ids);

static const struct dev_pm_ops sd108_pwm_pm = {
	SET_RUNTIME_PM_OPS(sd108_pwm_runtime_suspend,
			   sd108_pwm_runtime_resume, NULL)
};

static struct i2c_driver sd108_i2c_driver = {
	.driver = {
		.name = "sd108",
		.of_match_table = of_match_ptr(sd108_dt_ids),
		.pm = &sd108_pwm_pm,
	},
	.probe = sd108_pwm_probe,
	.remove = sd108_pwm_remove,
	.id_table = sd108_id,
};

module_i2c_driver(sd108_i2c_driver);

MODULE_AUTHOR("Massimiliano Negretti <massimiliano.negretti@open-eyes.it>");
MODULE_DESCRIPTION("PWM, GPIO and backlight driver for OPEN-EYES");
MODULE_LICENSE("GPL");
