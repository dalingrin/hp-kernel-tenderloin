#ifndef HSS_H
#define HSS_H

struct msm_headset_platform_data {
	const char *hs_name;
	uint32_t gpio_detection_irq; 
	uint32_t gpio_hookkey_irq;
	uint32_t gpio_func_sel;
};

void report_headset_status(bool connected);


#endif