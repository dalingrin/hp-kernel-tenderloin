/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __LEDS_LM8502_H__
#define __LEDS_LM8502_H__

#define LM8502_I2C_DEVICE       "LM8502"
#define LM8502_I2C_DRIVER       "LM8502"
#define LM8502_I2C_ADDR     0x33

#define LM8502_MISC_POWER_SAVE_ON  (1 << 5)
#define LM8502_MISC_POWER_SAVE_OFF (0 << 5)

enum {
    LED_ID_D1,
    LED_ID_D2,
    LED_ID_D3,
    LED_ID_D4,
    LED_ID_D5,
    LED_ID_D6,
    LED_ID_D7,
    LED_ID_D8,
    LED_ID_D9,
    LED_ID_D10,
};

enum {
    LED_NONE = 0,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_WHITE,
};

enum {
    LED_HW_GRP_NONE = 0,
    LED_HW_GRP_1,
    LED_HW_GRP_2,
    LED_HW_GRP_3,
};

struct lm8502_led_list {
    int type;
    int id;
};

struct lm8502_led_config {
    struct i2c_client *client;
    struct led_classdev	cdev;
    struct work_struct	work;
    spinlock_t  value_lock;
    enum led_brightness	brightness;
    
    struct lm8502_led_list *led_list;
    int nleds;
    int hw_group;
   	int max_brightness;
    int default_max_current;
    int default_brightness;
};

struct lm8502_platform_data {
    //for leds
	int	num_leds;
	struct lm8502_led_config *leds;
	
	// others
	u8 power_mode;
    
    // for vibrator
	int max_timeout_ms;
    int level_pwm;

    // for flash or torch
    u16 flash_default_duration;
    u16 flash_default_current;
    u16 torch_default_current;
};

int lm8502_flash_enable_export(u32 enable);
int lm8502_flashtorch_start_export(int start);
int lm8502_flash_duration_export( u32 duration);
int lm8502_flash_current_export(u32 desired_current);
int lm8502_torch_current_export(u32 desired_current);

#endif /* __LEDS_LM8502_H__ */
