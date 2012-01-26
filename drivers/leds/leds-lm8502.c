/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/leds-lm8502.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include "../staging/android/timed_output.h"

/* Registers */
#define ENGINE_CNTRL1       0x00
#define ENGINE_CNTRL2       0x01
#define GROUP_FADING1       0x02
#define GROUP_FADING2       0x03
#define GROUP_FADING3       0x04

#define D1_CONTROL      0x06
#define D2_CONTROL      0x07
#define D3_CONTROL      0x08
#define D4_CONTROL      0x09
#define D5_CONTROL      0x0A
#define D6_CONTROL      0x0B
#define D7_CONTROL      0x0C
#define D8_CONTROL      0x0D
#define D9_CONTROL      0x0E
#define D10_CONTROL     0x0F

#define HAPTIC_CONTROL      0x10

#define ALS_CONTROL         0x11
#define ZLINE0          0x12
#define ZLINE1          0x13
#define ZLINE2          0x14
#define ZLINE3          0x15
#define TARGET_LIGHT_Z0     0x16
#define TARGET_LIGHT_Z1     0x17
#define TARGET_LIGHT_Z2     0x18
#define TARGET_LIGHT_Z3     0x19
#define TARGET_LIGHT_Z4     0x1A
#define ALS_START_VALUE     0x1B
#define DBC_CONTROL     0x1D

#define HAPTIC_FEEDBACK_CTRL    0x21
#define HAPTIC_PWM_DUTY_CYCLE   0x22

#define D1_CURRENT_CTRL     0x26
#define D2_CURRENT_CTRL     0x27
#define D3_CURRENT_CTRL     0x28
#define D4_CURRENT_CTRL     0x29
#define D5_CURRENT_CTRL     0x2A
#define D6_CURRENT_CTRL     0x2B
#define D7_CURRENT_CTRL     0x2C
#define D8_CURRENT_CTRL     0x2D
#define D9_CURRENT_CTRL     0x2E
#define D10_CURRENT_CTRL    0x2F

#define ADAPT_FLASH_CTRL    0x35
#define MISC            0x36

#define ENGINE1_PC      0x37
#define ENGINE2_PC      0x38

#define STATUS          0x3A
#define INT         0x3B
#define I2C_VARIABLE        0x3C
#define RESET           0x3D

#define LED_TEST_CONTROL    0x41
#define LED_TEST_ADC        0x42

#define GROUP_FADER1        0x48
#define GROUP_FADER2        0x49
#define GROUP_FADER3        0x4A

#define ENG1_PROG_START_ADDR    0x4C
#define ENG2_PROG_START_ADDR    0x4D
#define PROG_MEM_PAGE_SELECT    0x4F

#define PROG_MEM_START      0x50
#define PROG_MEM_END        0x6F

#define TORCH_BRIGHTNESS    0xA0
#define FLASH_BRIGHTNESS    0xB0
#define FLASH_DURATION      0xC0
#define FLAG_REGISTER       0xD0
#define CONFIG_REG1     0xE0
#define CONFIG_REG2     0xF0

#define ENGINE_CNTRL_ENG1_SHIFT 4
#define ENGINE_CNTRL_ENG2_SHIFT 2

#define ENGINE_CNTRL1_HOLD  0
#define ENGINE_CNTRL1_STEP  1
#define ENGINE_CNTRL1_FREERUN   2
#define ENGINE_CNTRL1_EXECONCE  3

#define ENGINE_CNTRL2_DISABLE   0
#define ENGINE_CNTRL2_LOAD  1
#define ENGINE_CNTRL2_RUN   2
#define ENGINE_CNTRL2_HALT  3

#define STROBE_TIMEOUT      (1 << 7)

#define FLASH_MODE      (3 << 0)
#define TORCH_MODE      (2 << 0)


struct lm8502_vib_data {
    struct i2c_client *client;
    struct hrtimer vib_timer;
    struct timed_output_dev timed_dev;
    spinlock_t  value_lock;
    struct work_struct work;
    int max_timeout_ms;
    int state;
    int level;
};

struct lm8502_flashtorch_data {
    struct class *flash_class;
    struct device *flash_class_dev;
    u8 is_flash_mode;  // true => flash_mode false=> torch_mode
    u8 flash_enable;
    u8 flash_start;
    u16 flash_duration;
    u16 flash_current;
    u16 torch_current;
};


struct lm8502_device_state {
    struct i2c_client *client;
    struct lm8502_platform_data pdata;
    struct lm8502_vib_data vib;
    struct lm8502_flashtorch_data flashtorch;
    //struct mutex lock;
};

static struct lm8502_device_state *lm8502_state=NULL;


struct brightness_entry {
    u8 code;
    u16 current_value;
};

static struct brightness_entry flash_brightness_map[] = {
    {0x0, 38},
    {0x1, 75},
    {0x2, 113},
    {0x3, 150},
    {0x4, 188},
    {0x5, 225},
    {0x6, 263},
    {0x7, 300},
    {0x8, 338},
    {0x9, 375},
    {0xA, 413},
    {0xB, 450},
    {0xC, 488},
    {0xD, 525},
    {0xE, 563},
    {0xF, 600}
};

static struct brightness_entry torch_brightness_map[] = {
    {0x0, 18},
    {0x1, 37},
    {0x2, 56},
    {0x3, 75},
    {0x4, 93},
    {0x5, 112},
    {0x6, 131},
    {0x7, 150},
};

static u8 lm8502_get_closest_flash_current(u16 desired_current)
{
    u16 i;
    for(i = 0; i < sizeof(flash_brightness_map) / sizeof(flash_brightness_map[0]); i++) {
        if (flash_brightness_map[i].current_value >= desired_current) {
            return i;
        }
    }
    return (sizeof(flash_brightness_map)/sizeof(flash_brightness_map[0]))-1;
}

static u8 lm8502_get_closest_torch_current(u16 desired_current)
{
    u16 i;
    for(i = 0; i < sizeof(torch_brightness_map) / sizeof(torch_brightness_map[0]); i++) {
        if(torch_brightness_map[i].current_value >= desired_current) {
            return i;
        }
    }
    return (sizeof(torch_brightness_map)/sizeof(torch_brightness_map[0]))-1;
}


/*****************************************************************************
*  FUNCTION
*  PURPOSE: I2C interface
*  INPUT
*  OUTPUT
*
* ***************************************************************************/

static uint8_t i2c_write_mem(struct i2c_client *client, uint8_t Address, uint8_t ByteCount, uint8_t *Data )
{
    int ret;
    uint8_t buffer[10];
    struct i2c_msg msgs = {
            .addr   = client->addr,
            .flags  = 0,
            .buf    = (void *)buffer,
            .len    = ByteCount+1
    };

    buffer[0] = (uint8_t)Address;
    memcpy(buffer+1, Data, ByteCount);
    msgs.len    = ByteCount+1;

    ret = i2c_transfer( client->adapter, &msgs, 1);
    if(ret > 0) return 0;
    dev_info(&client->dev, "i2c_transfer error: ret=%d\n", ret);
    return  1;
}

static uint8_t i2c_write_reg(struct i2c_client *client, uint8_t reg, uint8_t Data )
{
    uint8_t value;

    value = Data;
    return i2c_write_mem(client, reg, 1, &value);
}


static uint8_t i2c_read_mem(struct i2c_client *client, uint8_t Address, uint8_t ByteCount, uint8_t *Data )
{
    int ret;

    struct i2c_msg msgs[] = {
        [0] = {
            .addr   = client->addr,
            .flags  = 0,
            .buf    = (void *)&Address,
            .len    = 1
        },
        [1] = {
            .addr   = client->addr,
            .flags  = I2C_M_RD,
            .buf    = (void *)Data,
            .len    = ByteCount
        }
    };

    ret =  i2c_transfer(client->adapter, msgs, 2);
    if(ret > 0)return 0;
    return  1;
}

static uint8_t i2c_read_reg(struct i2c_client *client, uint8_t reg, uint8_t *Data )
{
    return i2c_read_mem(client, reg, 1, (uint8_t *)Data);
}

/*****************************************************************************
*
*  LM8502 leds functions
*
* ***************************************************************************/

static void lm8502_enable(struct i2c_client *client)
{
    uint8_t reg;

    /*enable lm8502*/
    i2c_read_reg(client, ENGINE_CNTRL1, &reg);
    i2c_write_reg(client, ENGINE_CNTRL1, (reg|0x40) ); //CHIP_EN=1
}

#if 0 //remove it for the LM8502 SPEC issue(During standby mode, the i2c register operation will be failed)
static void lm8502_tryto_enable(struct i2c_client *client)
{
    uint8_t reg;

    //if lm8502 not enable, enable it
    i2c_read_reg(client, ENGINE_CNTRL1, &reg);
    if(!(reg&0x40))
    {
        lm8502_enable(client);
    }
}
#endif

static void lm8502_disable(struct i2c_client *client)
{
    /*disable LM8502, enter standby mode*/
    i2c_write_reg(client, ENGINE_CNTRL1, 0x00); //CHIP_EN=0
}

#if 0 //remove it for the LM8502 SPEC issue(During standby mode, the i2c register operation will be failed)
static void lm8502_tryto_disable(struct i2c_client *client)
{
    struct lm8502_device_state *state = i2c_get_clientdata(client);
    int i;
    int led_state=0;

    for(i = 0; i < state->pdata.num_leds; i++)
        led_state += (state->pdata.leds[i].brightness)?1:0;

    //if all leds and vib and flashtorch off, we can disable lm8502 for power saver
    if((!led_state) && (!state->vib.state) && (!state->flashtorch.flash_start))
    {
        lm8502_disable(client);
    }
}
#endif

static void lm8502_led_set(struct lm8502_led_config *led, enum led_brightness value)
{
    int i;
    //struct lm8502_device_state *state = i2c_get_clientdata(led->client);

    //mutex_lock(&state->lock);
    //lm8502_tryto_enable(state->client);

    switch (led->hw_group) {
	case LED_HW_GRP_NONE:
        for(i = 0; i < led->nleds; i++)
        {
            i2c_write_reg(led->client, led->led_list[i].id + D1_CURRENT_CTRL, value);
        }
        break;
	case LED_HW_GRP_1:
        i2c_write_reg(led->client, GROUP_FADER1, value);
        break;
    case LED_HW_GRP_2:
        i2c_write_reg(led->client, GROUP_FADER2, value);
        break;
    case LED_HW_GRP_3:
        i2c_write_reg(led->client, GROUP_FADER3, value);
        break;
    default:
        break;
	}

    //lm8502_tryto_disable(state->client);
    //mutex_unlock(&state->lock);
}

static enum led_brightness lm8502_led_get(struct lm8502_led_config *led)
{
    return led->brightness;
}

static void lm8502_led_set_brightness(struct led_classdev *led_cdev,
	enum led_brightness value)
{
    struct lm8502_led_config *led;
    unsigned long flags;

    led = container_of(led_cdev, struct lm8502_led_config, cdev);
    spin_lock_irqsave(&led->value_lock, flags);
    led->brightness = value;
    spin_unlock_irqrestore(&led->value_lock, flags);

    schedule_work(&led->work);
}

static enum led_brightness lm8502_led_get_brightness(struct led_classdev *led_cdev)
{
    struct lm8502_led_config *led;

    led = container_of(led_cdev, struct lm8502_led_config, cdev);
    return lm8502_led_get(led);
}

static void lm8502_led_work(struct work_struct *work)
{
    struct lm8502_led_config *led = container_of(work, struct lm8502_led_config, work);

    lm8502_led_set(led, led->brightness);
}

static void lm8502_select_flash(struct i2c_client *client)
{
    //LED10 as switch between flash and vibrator
    i2c_write_reg(client, D10_CURRENT_CTRL, 0xFF);
}

static void lm8502_select_vibrator(struct i2c_client *client)
{
    //LED10 as switch between flash and vibrator
    i2c_write_reg(client, D10_CURRENT_CTRL, 0);
}

/*****************************************************************************
*
*  LM8502 vibrator functions
*
* Haptic feedback controller(0x21)
* Bit[4:3],PWM Frequency
*        00 for 157Hz
*        01 for 490Hz
*        10 for 980Hz
*        11 for 3.9kHz
*HAPT_SEL_MODE[2:0] Description
*        000 Haptic disabled
*        001 PWM to both channels (2*pwm_freq)
*        010 PWM on p channel, n channel in low state
*        011 PWM on n channel, p channel in low state
*        100 Haptic disabled
*        101 PWM to both channels, no activity at center value (2*pwm_freq)
*        110 PWM on p channel, n channel in high state
*        111 PWM on n channel, p channel in high state
*
* ***************************************************************************/

static void lm8502_vib_set(struct lm8502_vib_data *vib, int on)
{
    //struct lm8502_device_state *state = i2c_get_clientdata(vib->client);

    //mutex_lock(&state->lock);
    if (on) {
        //lm8502_tryto_enable(vib->client);
        i2c_write_reg(vib->client, HAPTIC_FEEDBACK_CTRL, 0x06); //110 PWM on p channel, n channel in high state, Bit[4:3], 00 for 157Hz
        //lm8502_tryto_disable(vib->client);
    } else {
        //lm8502_tryto_enable(vib->client);
        i2c_write_reg(vib->client, HAPTIC_FEEDBACK_CTRL, 0x0); // Disable Haptic feedback
        //lm8502_tryto_disable(vib->client);
    }
    //mutex_unlock(&state->lock);
}

static void lm8502_vib_enable(struct timed_output_dev *dev, int value)
{
    struct lm8502_vib_data *vib = container_of(dev, struct lm8502_vib_data,
					 timed_dev);
    struct lm8502_device_state *state = i2c_get_clientdata(vib->client);
    unsigned long flags;

    if (state->flashtorch.flash_enable) {
        dev_err(&vib->client->dev, "Flash and vibrator cannot be used together\n");
        return;
    }

    lm8502_select_vibrator(state->client);

    spin_lock_irqsave(&vib->value_lock, flags);
    hrtimer_cancel(&vib->vib_timer);

    if (value == 0)
    {
        vib->state = 0;
    }
    else
    {
        vib->state = 1;
        hrtimer_start(&vib->vib_timer,
                      ktime_set(value / 1000, (value % 1000) * 1000000),
                      HRTIMER_MODE_REL);
    }
    spin_unlock_irqrestore(&vib->value_lock, flags);

    schedule_work(&vib->work);
}

static void lm8502_vib_work(struct work_struct *work)
{
    struct lm8502_vib_data *vib = container_of(work, struct lm8502_vib_data, work);

    lm8502_vib_set(vib, vib->state);
}

static int lm8502_vib_get_time(struct timed_output_dev *dev)
{
    struct lm8502_vib_data *vib = container_of(dev, struct lm8502_vib_data, timed_dev);

    if (hrtimer_active(&vib->vib_timer)) {
        ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
        return r.tv.sec * 1000 + r.tv.nsec / 1000000;
    } else
        return 0;
}

static enum hrtimer_restart lm8502_vib_timer_func(struct hrtimer *timer)
{
    struct lm8502_vib_data *vib = container_of(timer, struct lm8502_vib_data, vib_timer);

    vib->state = 0;
    schedule_work(&vib->work);
    return HRTIMER_NORESTART;
}

/*****************************************************************************
*
*  LM8502 Flash and Torch functions
*
*
* ***************************************************************************/
static int lm8502_flashtorch_start(struct lm8502_device_state *state, int start)
{
    u8 reg;

    //mutex_lock(&state->lock);
    //lm8502_tryto_enable(state->client);
    i2c_read_reg(state->client, FLASH_BRIGHTNESS, &reg);
    reg = reg & ~0x03;
    if (start) {
        if (state->flashtorch.is_flash_mode)
            reg = reg | FLASH_MODE;
        else
            reg = reg | TORCH_MODE;
    }
    i2c_write_reg(state->client, FLASH_BRIGHTNESS, reg);
    state->flashtorch.flash_start = start;
    //lm8502_tryto_disable(state->client);
    //mutex_unlock(&state->lock);

    return 0;
}

static ssize_t lm8502_flashtorch_start_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", state->flashtorch.flash_start);
}

static ssize_t lm8502_flashtorch_start_store(struct device *dev, struct device_attribute *attr,
                       const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 start;
    int ret;

    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &start) != 1)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    if (!state->flashtorch.flash_enable) {
        dev_err(&state->client->dev, "Flash must be enabled before use\n");
        return -EINVAL;
    }

    ret = lm8502_flashtorch_start(state, start);
    if(ret)
        return -EINVAL;

    return count;
}

int lm8502_flashtorch_start_export(int start)
{
    if (!lm8502_state->flashtorch.flash_enable) {
        dev_err(&lm8502_state->client->dev, "Flash must be enabled before use\n");
        return -EINVAL;
    }
    return lm8502_flashtorch_start(lm8502_state, start);
}
EXPORT_SYMBOL(lm8502_flashtorch_start_export);

static int lm8502_flash_duration(struct lm8502_device_state *state, u32 duration)
{
    u8 reg;

    if (duration >= 1024) {
        dev_err(&state->client->dev, "Duration is too long\n");
        return -EINVAL;
    }

    if (!state->flashtorch.flash_enable) {
        dev_err(&state->client->dev, "Flash must be enabled before use\n");
        return -EINVAL;
    }

    state->flashtorch.flash_duration = duration;
    duration = duration >> 5;

    i2c_read_reg(state->client, FLASH_DURATION, &reg);
    reg = (reg & ~0x1F) | duration;
    i2c_write_reg(state->client, FLASH_DURATION, reg);

    return 0;
}

static ssize_t lm8502_flash_duration_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", state->flashtorch.flash_duration);
}

static ssize_t lm8502_flash_duration_store(struct device *dev, struct device_attribute *attr,
                   const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 duration;
    int ret;


    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &duration) != 1)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    ret = lm8502_flash_duration(state, duration);
    if(ret)
        return -EINVAL;

    return count;
}

int lm8502_flash_duration_export( u32 duration)
{
   return lm8502_flash_duration(lm8502_state, duration);
}
EXPORT_SYMBOL(lm8502_flash_duration_export);

static int lm8502_flash_current(struct lm8502_device_state *state, u32 desired_current)
{
    u8 index;
    u8 reg;

    if (!state->flashtorch.flash_enable) {
        dev_err(&state->client->dev, "Flash must be enabled before use\n");
        return -EINVAL;
    }

    index = lm8502_get_closest_flash_current(desired_current);
    i2c_read_reg(state->client, FLASH_BRIGHTNESS, &reg);
    reg = reg & 0x07; //bit[7:3]
    reg = reg | STROBE_TIMEOUT | (flash_brightness_map[index].code << 3);
    i2c_write_reg(state->client, FLASH_BRIGHTNESS, reg);

    state->flashtorch.flash_current = flash_brightness_map[index].current_value;

    return 0;

}

static ssize_t lm8502_flash_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%u\n", state->flashtorch.flash_current);
}

static ssize_t lm8502_flash_current_store(struct device *dev, struct device_attribute *attr,
                       const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 desired_current;
    int ret;

    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &desired_current) != 1)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    ret = lm8502_flash_current(state, desired_current);
    if(ret)
        return -EINVAL;

    return count;
}

int lm8502_flash_current_export(u32 desired_current)
{
    return lm8502_flash_current(lm8502_state, desired_current);
}
EXPORT_SYMBOL(lm8502_flash_current_export);


static int lm8502_torch_current(struct lm8502_device_state *state, u32 desired_current)
{
    u8 index;
    u8 reg;

    if (!state->flashtorch.flash_enable) {
        dev_err(&state->client->dev, "Flash must be enabled before use\n");
        return -EINVAL;
    }

    index = lm8502_get_closest_torch_current(desired_current);
    i2c_read_reg(state->client, TORCH_BRIGHTNESS, &reg);
    reg = reg & ~0x38; //bit[5:3]
    reg = reg | (torch_brightness_map[index].code << 3);
    i2c_write_reg(state->client, TORCH_BRIGHTNESS, reg);

    state->flashtorch.torch_current = torch_brightness_map[index].current_value;

    return 0;
}
static ssize_t lm8502_torch_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%u\n", state->flashtorch.torch_current);
}

static ssize_t lm8502_torch_current_store(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 desired_current;
    int ret;

    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &desired_current) != 1)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);
    ret = lm8502_torch_current(state, desired_current);
    if(ret)
        return -EINVAL;

    return count;
}

int lm8502_torch_current_export(u32 desired_current)
{
    return lm8502_torch_current(lm8502_state, desired_current);
}
EXPORT_SYMBOL(lm8502_torch_current_export);

static int lm8502_flash_enable(struct lm8502_device_state *state, u32 enable)
{
    if (!enable) {
        state->flashtorch.flash_enable = (u8)enable;
        return 0;
    }

    if (state->vib.state) {
        dev_err(&state->client->dev, "Flash and vibrator cannot be used together\n");
        return -EINVAL;
    }

    state->flashtorch.flash_enable = (u8)enable;
    lm8502_select_flash(state->client);
    state->flashtorch.is_flash_mode = (enable == 1); // 1 == flash, 2 == torch

    return 0;
}

static ssize_t lm8502_flash_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    struct lm8502_device_state *state;

    if (!buf)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", state->flashtorch.flash_enable);
}

static ssize_t lm8502_flash_enable_store(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
    struct lm8502_device_state *state;
    u32 enable;
    int ret;

    if (!buf || !count)
        return -EINVAL;

    if (sscanf(buf, "%u", &enable) != 1)
        return -EINVAL;

    if (enable > 2)
        return -EINVAL;

    state = (struct lm8502_device_state *) dev_get_drvdata(dev);

    ret = lm8502_flash_enable(state, enable);
    if(ret)
        return -EINVAL;

    return count;
}

int lm8502_flash_enable_export(u32 enable)
{
    return lm8502_flash_enable(lm8502_state, enable);
}
EXPORT_SYMBOL(lm8502_flash_enable_export);

static DEVICE_ATTR(flash_enable, S_IRUGO | S_IWUSR, lm8502_flash_enable_show, lm8502_flash_enable_store);
static DEVICE_ATTR(torch_current, S_IRUGO | S_IWUSR, lm8502_torch_current_show, lm8502_torch_current_store);
static DEVICE_ATTR(flash_current, S_IRUGO | S_IWUSR, lm8502_flash_current_show, lm8502_flash_current_store);
static DEVICE_ATTR(flash_duration, S_IRUGO | S_IWUSR, lm8502_flash_duration_show, lm8502_flash_duration_store);
static DEVICE_ATTR(flashtorch_start, S_IRUGO | S_IWUSR, lm8502_flashtorch_start_show, lm8502_flashtorch_start_store);


/*****************************************************************************
*
*  LM8502 driver  functions
*
* ***************************************************************************/
static void lm8502_configuration(struct i2c_client *client)
{
    struct lm8502_device_state *state = i2c_get_clientdata(client);
    struct lm8502_platform_data *pdata = &state->pdata;
    struct lm8502_led_config *leds = pdata->leds;
    struct lm8502_flashtorch_data *flashtorch = &state->flashtorch;
    uint8_t reg;
    int i,j;
    u8 index;

    // Configure power savings mode and enable boost,select pwm input
    i2c_read_reg(client, MISC, &reg);
    i2c_write_reg(client, MISC, reg | (pdata->power_mode) | (1<<3) | (1<<1));

    // Haptic PWM duty cycle, can be adjust for different motor speed(0-255)
    i2c_write_reg(client, HAPTIC_PWM_DUTY_CYCLE, 127);

    for (i = 0; i < pdata->num_leds; i++) {
        /*assign hardware groups to each led in the group*/
        if(leds[i].hw_group != LED_HW_GRP_NONE)
        {
            for(j = 0; j < leds[i].nleds; j++)
            {
                i2c_read_reg(client, leds[i].led_list[j].id + D1_CONTROL, &reg);
                reg = reg & (~0xC0); //clear bit[7:6]
                reg = reg | (leds[i].hw_group << 6); //set bit[7:6]
                i2c_write_reg(client, leds[i].led_list[j].id + D1_CONTROL, reg);
            }
        }

        /* Set current full-scale setting for led output*/
        for(j = 0; j < leds[i].nleds; j++)
        {
            i2c_read_reg(client, leds[i].led_list[j].id + D1_CONTROL, &reg);
            reg = reg & (~0x18); //clear bit[4:3]
            reg = reg | (leds[i].default_max_current << 3); //set bit[4:3]
            i2c_write_reg(client, leds[i].led_list[j].id + D1_CONTROL, reg);
        }
    }

    /* Program the default flash&torch settings as specified in the board file */
    index = lm8502_get_closest_flash_current(pdata->flash_default_current);
    flashtorch->flash_current = flash_brightness_map[index].current_value;
    i2c_write_reg(client, FLASH_BRIGHTNESS, STROBE_TIMEOUT | (flash_brightness_map[index].code << 3));

    flashtorch->flash_duration = pdata->flash_default_duration;
    i2c_write_reg(client, FLASH_DURATION, flashtorch->flash_duration >> 5);

    index = lm8502_get_closest_torch_current(pdata->torch_default_current);
    flashtorch->torch_current = torch_brightness_map[index].current_value;
    i2c_write_reg(client, TORCH_BRIGHTNESS, torch_brightness_map[index].code << 3);

}

static int lm8502_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
    struct lm8502_platform_data *pdata = client->dev.platform_data;
    struct lm8502_device_state *state= NULL;
    struct lm8502_vib_data *vib = NULL;
    struct lm8502_led_config *leds = NULL;
    struct lm8502_flashtorch_data *flashtorch = NULL;
    int i, rc;
    uint8_t reg;

    //verify platform data
    if (pdata == NULL) {
        dev_err(&client->dev, "missing platform data\n");
        return -EINVAL;
    }

    //detect LM8502 chipset
    rc = i2c_read_reg(client, ENGINE_CNTRL1, &reg); //CHIP_EN=1
    rc += i2c_write_reg(client, ENGINE_CNTRL1, (reg|0x40) );
    if(rc)
    {
        dev_err(&client->dev, "lm8502 not found\n");
        goto err0;
    }
    i2c_write_reg(client, ENGINE_CNTRL1, 0x00);//CHIP_EN=0

    //alloc lm8502 data and leds data buffer
    state = kzalloc(sizeof(struct lm8502_device_state), GFP_KERNEL);
    if (!state)
        return -ENOMEM;

    // for export functions
    lm8502_state = state;

    /* attach platform data */
    memcpy(&state->pdata, pdata, sizeof(struct lm8502_platform_data));

    //init lm8502 data
    state->client = client;
    i2c_set_clientdata(client, state);
    //mutex_init(&state->lock);

    //configure leds
    for (i = 0, leds = pdata->leds; i < pdata->num_leds; i++) {
        /* Save the i2c client information for each led. */
        leds[i].client = client;

        leds[i].cdev.brightness_set    = lm8502_led_set_brightness;
        leds[i].cdev.brightness_get    = lm8502_led_get_brightness;
        leds[i].cdev.brightness        = leds[i].default_brightness;
        leds[i].cdev.max_brightness    = leds[i].max_brightness;
        leds[i].cdev.flags     = LED_CORE_SUSPENDRESUME;

        /* Register this LED instance to the LED class. */
        rc = led_classdev_register(&client->dev, &leds[i].cdev);
        if (rc) {
            dev_err(&client->dev,
                "unable to register led %s\n",
                leds[i].cdev.name);
            goto err1;
        }

        //init spin lock and work
        spin_lock_init(&leds[i].value_lock);
        INIT_WORK(&leds[i].work, lm8502_led_work);

        /*set led default brightness*/
        leds[i].brightness = leds[i].default_brightness;
        schedule_work(&leds[i].work);
    }

    //configure vibrator
    vib = &state->vib;
    vib->level = pdata->level_pwm;
    vib->max_timeout_ms = pdata->max_timeout_ms;
    vib->client = client;
    spin_lock_init(&vib->value_lock);
    INIT_WORK(&vib->work, lm8502_vib_work);
    hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    vib->vib_timer.function = lm8502_vib_timer_func;
    vib->timed_dev.name = "vibrator";
    vib->timed_dev.get_time = lm8502_vib_get_time;
    vib->timed_dev.enable = lm8502_vib_enable;

    rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0)
		goto err1;

    /* Create sysfs entries for the flash and torch */
    flashtorch = &state->flashtorch;
    flashtorch->flash_class = class_create(THIS_MODULE, "flash");
    if (!flashtorch->flash_class){
        rc = -ENOMEM;
        dev_err(&client->dev, "Unable to create flash class node\n");
        goto err2;
    }

    flashtorch->flash_class_dev = device_create(flashtorch->flash_class, &client->dev, 0, state, "flash0");
    if (!flashtorch->flash_class_dev) {
        rc = -ENOMEM;
        dev_err(&client->dev, "Unable to create flash class device node\n");
        goto err3;
    }

    if ((rc = device_create_file(flashtorch->flash_class_dev, &dev_attr_flash_enable)) < 0) {
        dev_err(&client->dev, "Unable to create flash current node\n");
        goto err4;
    }

    if ((rc = device_create_file(flashtorch->flash_class_dev, &dev_attr_flash_current)) < 0) {
        dev_err(&client->dev, "Unable to create flash current node\n");
        goto err5;
    }

    if ((rc = device_create_file(flashtorch->flash_class_dev, &dev_attr_flash_duration)) < 0) {
        dev_err(&client->dev, "Unable to create flash duration node\n");
        goto err6;
    }

    if ((rc = device_create_file(flashtorch->flash_class_dev, &dev_attr_torch_current)) < 0) {
        dev_err(&client->dev, "Unable to create flash duration node\n");
        goto err7;
    }

    if ((rc = device_create_file(flashtorch->flash_class_dev, &dev_attr_flashtorch_start)) < 0) {
        dev_err(&client->dev, "Unable to create flash fire node\n");
        goto err8;
    }

    // configure lm8502 registers
    lm8502_enable(client);
    lm8502_configuration(client);

    return 0;

err8:
    device_remove_file(flashtorch->flash_class_dev, &dev_attr_torch_current);
err7:
    device_remove_file(flashtorch->flash_class_dev, &dev_attr_flash_duration);
err6:
    device_remove_file(flashtorch->flash_class_dev, &dev_attr_flash_current);
err5:
    device_remove_file(flashtorch->flash_class_dev, &dev_attr_flash_enable);
err4:
    device_unregister(flashtorch->flash_class_dev);
err3:
    class_destroy(flashtorch->flash_class);
err2:
    timed_output_dev_unregister(&vib->timed_dev);
err1:
    if (i > 0)
        for (i = i - 1; i >= 0; i--)
        {
            led_classdev_unregister(&leds[i].cdev);
        }
    kfree(state);
    lm8502_state = NULL;
err0:
    dev_info(&client->dev, "lm8502_probe: failed\n");
    return rc;
}

static int lm8502_remove(struct i2c_client *client)
{
    int i;
    struct lm8502_platform_data *pdata = client->dev.platform_data;
    struct lm8502_device_state *state = i2c_get_clientdata(client);
    struct lm8502_flashtorch_data *flashtorch = &state->flashtorch;

    //leds
    for (i = 0; i < pdata->num_leds; i++) {
        led_classdev_unregister(&pdata->leds[i].cdev);
        cancel_work_sync(&pdata->leds[i].work);
    }
    //vibrator
    cancel_work_sync(&state->vib.work);
    hrtimer_cancel(&state->vib.vib_timer);
    timed_output_dev_unregister(&state->vib.timed_dev);

    //flash and torch
    device_remove_file(flashtorch->flash_class_dev, &dev_attr_flashtorch_start);
    device_remove_file(flashtorch->flash_class_dev, &dev_attr_torch_current);
    device_remove_file(flashtorch->flash_class_dev, &dev_attr_flash_duration);
    device_remove_file(flashtorch->flash_class_dev, &dev_attr_flash_current);
    device_remove_file(flashtorch->flash_class_dev, &dev_attr_flash_enable);
    device_unregister(flashtorch->flash_class_dev);
    class_destroy(flashtorch->flash_class);

    i2c_set_clientdata(client, NULL);
    kfree(state);
    lm8502_state = NULL;

    return 0;
}

#ifdef CONFIG_PM
static int lm8502_suspend(struct i2c_client *client, pm_message_t pm_state)
{
    struct lm8502_platform_data *pdata = client->dev.platform_data;
    struct lm8502_device_state *state = i2c_get_clientdata(client);
    struct lm8502_vib_data *vib = &state->vib;
    int i;

    dev_info(&client->dev, "lm8502_suspend\n");

    //vibrator
    hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
    if(vib->state)
	    lm8502_vib_set(vib, 0);

    //leds support LED_CORE_SUSPENDRESUME
    for(i=0; i< pdata->num_leds; i++)
	    cancel_work_sync(&pdata->leds[i].work);

    //torch
    if(state->flashtorch.flash_start && !state->flashtorch.is_flash_mode) {
        lm8502_flashtorch_start(state, 0);
    }

    //standby
    lm8502_disable(client);

	return 0;
}

static int lm8502_resume(struct i2c_client *client)
{
    struct lm8502_device_state *state = i2c_get_clientdata(client);
    struct lm8502_vib_data *vib = &state->vib;

    dev_info(&client->dev, "lm8502_resume\n");

    //enable lm8502
    lm8502_enable(client);

    //vibrator
    if(vib->state)
	    lm8502_vib_set(vib, vib->state);

    // leds support LED_CORE_SUSPENDRESUME

    //torch
    if(state->flashtorch.flash_start && !state->flashtorch.is_flash_mode) {
        lm8502_flashtorch_start(state, 1);
    }

    return 0;
}
#endif


static const struct i2c_device_id lm8502_id[] = {
    { LM8502_I2C_DRIVER, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, lm8502_id);

static struct i2c_driver lm8502_driver = {
    .probe		= lm8502_probe,
    .remove		= __devexit_p(lm8502_remove),
    .driver		= {
        .name	= LM8502_I2C_DRIVER,
        .owner	= THIS_MODULE,
    },
#ifdef CONFIG_PM
    .suspend  = lm8502_suspend,
    .resume   = lm8502_resume,
#endif
    .id_table = lm8502_id,
};

static int __init lm8502_init(void)
{
    return i2c_add_driver(&lm8502_driver);
}

static void __exit lm8502_exit(void)
{
    i2c_del_driver(&lm8502_driver);
}

module_init(lm8502_init);
module_exit(lm8502_exit);

MODULE_DESCRIPTION("LM8502 LEDs and Vibrator driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
