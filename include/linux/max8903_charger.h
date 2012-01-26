#ifndef __MAX8903_CHARGER_H__
#define __MAX8903_CHARGER_H__
#include <linux/power_supply.h>

#define AC_CHG     0x00000001
#define USB_CHG    0x00000002

struct max8903_charger_platfom_data {
	//max8903 pin number 5, Current-Limit Mode Setting for the DC Power Input
	int DCM_pin;
	int DCM_pin_polarity;
	//max8903 pin number 7, USB Current-Limit Set Input
	int IUSB_pin;
	int IUSB_pin_polarity;
	//max8903 pin number 8, DC Power-OK Output
	int DOK_N_pin;
	//max8903 pin number 14, Charger Enable Input
	int CEN_N_pin;
	int CEN_N_pin_polarity;
	//max8903 pin number 15, USB Suspend Input
	int USUS_pin;
	int USUS_pin_polarity;
	//max8903 pin number 18, Fault Output
	int FLT_N_pin;
	//max8903 pin number 19, USB Power-OK Output
	int UOK_N_pin;
	//max8903 pin number 22, Charger Status Output
	int CHG_N_pin;
	//avaliable charge source 
	unsigned int avail_charge_sources;
	//program the current limit of the step-down regulator from 0.5A to 2A when DCM is logic-high
	void (*config_DC_current)(unsigned mA);
	//programs the fast-charge current up to 2A
	void (*config_charge_current)(unsigned mA);
	//do the necessary initialization with board 
	int (*board_init)(struct platform_device *pdev);

};

struct max8903_charger{
	int dok_irq;
	int flt_irq;
	int chg_irq;
	int uok_irq;

	//current charge source 
	unsigned int current_charge_source;
	//current input current form adapter source
	unsigned int ac_input_current;
	//current input current form usb source
	unsigned int usb_input_current;
	//charge current to battery 
	unsigned int charge_current;
	struct work_struct irq_work;
	struct workqueue_struct *irq_workqueue;
	struct max8903_charger_platfom_data *platform_data;

	struct power_supply ac;
	struct power_supply usb;
};
bool max8903_charger_is_connecting(void);
bool max8903_charger_is_charging(void);
void max8903_charger_connected(int is_connect, enum power_supply_type pst);
void max8903_charger_draw_current(unsigned mA);

#endif /* __MAX8903_CHARGER_H__ */

