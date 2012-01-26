/* MAXIM MAX8903 Charger Driver
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/irq.h>
#include <linux/max8903_charger.h>
#include <linux/completion.h>
#include <asm/gpio.h>

DECLARE_COMPLETION(max8903_charger_init_done);
static struct max8903_charger charger_info;

bool max8903_charger_is_connecting(void)
{
	bool ret = false;
	int DOK_N_pin_value = 1, UOK_N_pin_value = 1;

	if (charger_info.platform_data) {
		if (charger_info.platform_data->DOK_N_pin >= 0)
			DOK_N_pin_value = gpio_get_value_cansleep(charger_info.platform_data->DOK_N_pin);
		if (charger_info.platform_data->UOK_N_pin >= 0)
			UOK_N_pin_value = gpio_get_value_cansleep(charger_info.platform_data->UOK_N_pin);

	}
	if (!DOK_N_pin_value || !UOK_N_pin_value)
		ret = true;
	return ret;

}

bool max8903_charger_is_charging(void)
{
	bool ret = false;
	
	if (charger_info.platform_data && (charger_info.platform_data->CHG_N_pin >= 0)) {
		if (gpio_get_value_cansleep(charger_info.platform_data->CHG_N_pin)) {
			ret = false;
		} else {
			ret = true;
		}
	}
	return ret;
}

void max8903_charger_connected(int is_connect, enum power_supply_type pst)
{

	printk("MAX8903_CHARGER: %s : is_connect = %d, pst = %d \n", __func__, is_connect, pst);

	if (!charger_info.platform_data) {
		wait_for_completion(&max8903_charger_init_done);
	}
	if (is_connect) {
		if ((pst == POWER_SUPPLY_TYPE_MAINS) && (charger_info.platform_data->avail_charge_sources & AC_CHG)) {
			charger_info.current_charge_source = AC_CHG;
		} else {
			if ((pst == POWER_SUPPLY_TYPE_USB) && (charger_info.platform_data->avail_charge_sources & AC_CHG)) {
				charger_info.current_charge_source = USB_CHG;
			}
		}
	} else {
	
		if (charger_info.platform_data->CEN_N_pin >= 0)
			gpio_set_value_cansleep(charger_info.platform_data->CEN_N_pin, charger_info.platform_data->CEN_N_pin_polarity ? 0 : 1);
		charger_info.ac_input_current = 0;
		charger_info.current_charge_source = 0;
		max8903_charger_draw_current(0);
	}
	
	if (charger_info.irq_workqueue)
		queue_work(charger_info.irq_workqueue, &(charger_info.irq_work));
	pr_debug("MAX8903_CHARGER: %s : is_connect = %d, pst = %d ,current_charge_source = %x\n", __func__, is_connect, pst, charger_info.current_charge_source);

}
void max8903_charger_draw_current(unsigned mA)
{

	printk("MAX8903_CHARGER: %s : mA = %d \n", __func__, mA);

	if (!charger_info.platform_data) {
		wait_for_completion(&max8903_charger_init_done);
	}

	if (charger_info.platform_data->config_DC_current) {
		charger_info.platform_data->config_DC_current(mA);
	}
	
	if (mA <= 0) {
		if (charger_info.platform_data->DCM_pin >= 0)
			gpio_set_value_cansleep(charger_info.platform_data->DCM_pin, charger_info.platform_data->DCM_pin_polarity ? 1 : 0);
		if (charger_info.platform_data->USUS_pin >= 0)
			gpio_set_value_cansleep(charger_info.platform_data->USUS_pin, charger_info.platform_data->USUS_pin_polarity ? 0 : 1);
		if (charger_info.platform_data->IUSB_pin>= 0)
			gpio_set_value_cansleep(charger_info.platform_data->IUSB_pin, charger_info.platform_data->IUSB_pin_polarity ? 1 : 0);
	} else {
		if (mA <= 100) {
			if (charger_info.platform_data->DCM_pin >= 0)
				gpio_set_value_cansleep(charger_info.platform_data->DCM_pin, charger_info.platform_data->DCM_pin_polarity ? 1 : 0);
			if (charger_info.platform_data->USUS_pin >= 0)
				gpio_set_value_cansleep(charger_info.platform_data->USUS_pin, charger_info.platform_data->USUS_pin_polarity ? 1: 0);
			if (charger_info.platform_data->IUSB_pin)
				gpio_set_value_cansleep(charger_info.platform_data->IUSB_pin, charger_info.platform_data->IUSB_pin_polarity ? 1: 0);

		} else {
			if (mA <= 500) {
				if (charger_info.platform_data->DCM_pin >= 0)
					gpio_set_value_cansleep(charger_info.platform_data->DCM_pin, charger_info.platform_data->DCM_pin_polarity ? 1 : 0);
				if (charger_info.platform_data->USUS_pin >= 0)
					gpio_set_value_cansleep(charger_info.platform_data->USUS_pin, charger_info.platform_data->USUS_pin_polarity ? 1 : 0);
				if (charger_info.platform_data->IUSB_pin >= 0)
					gpio_set_value_cansleep(charger_info.platform_data->IUSB_pin, charger_info.platform_data->IUSB_pin_polarity ? 0: 1);
			} else {
				if (charger_info.platform_data->DCM_pin >= 0)
					gpio_set_value_cansleep(charger_info.platform_data->DCM_pin, charger_info.platform_data->DCM_pin_polarity ? 0 : 1);
				if (charger_info.platform_data->USUS_pin >= 0)
					gpio_set_value_cansleep(charger_info.platform_data->USUS_pin, charger_info.platform_data->USUS_pin_polarity ? 1 : 0);
				if (charger_info.platform_data->IUSB_pin >= 0)
					gpio_set_value_cansleep(charger_info.platform_data->IUSB_pin, charger_info.platform_data->IUSB_pin_polarity ? 1 : 0);

			}
		}
	}
	if (charger_info.platform_data->CEN_N_pin >= 0)
		gpio_set_value_cansleep(charger_info.platform_data->CEN_N_pin, charger_info.platform_data->CEN_N_pin_polarity ? 1 : 0);
	
	if (charger_info.current_charge_source & AC_CHG) {
		charger_info.ac_input_current = mA;
		pr_debug("MAX8903_CHARGER: %s : usb_input_current = %d\n", __func__, charger_info.usb_input_current);
	}
	if (charger_info.current_charge_source & USB_CHG) {
		charger_info.usb_input_current = mA;
		pr_debug("MAX8903_CHARGER: %s : ac_input_current = %d\n", __func__, charger_info.ac_input_current);
	}
	if (charger_info.irq_workqueue)
		queue_work(charger_info.irq_workqueue, &(charger_info.irq_work));

	
}

static int max8903_charger_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
				val->intval = charger_info.current_charge_source & AC_CHG ? 1 : 0;
			}
			if (psy->type == POWER_SUPPLY_TYPE_USB) {
				val->intval = charger_info.current_charge_source & USB_CHG ? 1 : 0;
			}
		break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
				val->intval = charger_info.ac_input_current;
			}
			if (psy->type == POWER_SUPPLY_TYPE_USB) {
				val->intval = charger_info.usb_input_current;
			}
		break;
		default:
			ret = -EINVAL;
	}
	return ret;
}

static enum power_supply_property max8903_charger_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static char *max8903_charger_supplied_to[] = {
	"battery",
};


void max8903_irq_work_handler(struct work_struct *work)
{
	power_supply_changed(&(charger_info.ac));
	power_supply_changed(&(charger_info.usb));
}
static irqreturn_t max8903_charger_dok_irq_handler(int irq, void *dev_id)
{

	pr_debug("MAX8903_CHARGER: %s : \n", __func__);
	if (charger_info.irq_workqueue)
		queue_work(charger_info.irq_workqueue, &(charger_info.irq_work));
	return IRQ_HANDLED;
}
static irqreturn_t max8903_charger_flt_irq_handler(int irq, void *dev_id)
{

	pr_debug("MAX8903_CHARGER: %s : \n", __func__);
	if (charger_info.irq_workqueue)
		queue_work(charger_info.irq_workqueue, &(charger_info.irq_work));
	return IRQ_HANDLED;
}
static irqreturn_t max8903_charger_uok_irq_handler(int irq, void *dev_id)
{

	pr_debug("MAX8903_CHARGER: %s : \n", __func__);
	if (charger_info.irq_workqueue)
		queue_work(charger_info.irq_workqueue, &(charger_info.irq_work));

	return IRQ_HANDLED;
}
static irqreturn_t max8903_charger_chg_irq_handler(int irq, void *dev_id)
{

	pr_debug("MAX8903_CHARGER: %s : \n", __func__);
	
	if (charger_info.irq_workqueue)
		queue_work(charger_info.irq_workqueue, &(charger_info.irq_work));

	return IRQ_HANDLED;
}

static int __devinit max8903_charger_probe(struct platform_device *pdev)
{
	struct max8903_charger_platfom_data* pdata = pdev->dev.platform_data;
	int ret = 0;

	pr_debug("MAX8903_CHARGER: %s : +++\n", __func__);
	if (pdev->id != -1) {
		dev_err(&pdev->dev,"%s: Can only support one MAX8903 charger!!!\n", __func__);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, &charger_info);
	if ((pdata->board_init)) {
		ret = pdata->board_init(pdev);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to do board_init\n");
			goto err_board_init;
		}
	}

        INIT_WORK(&(charger_info.irq_work), max8903_irq_work_handler);

	if (pdata->avail_charge_sources & AC_CHG) {
		/* Create power supplies for ac*/
		charger_info.ac.name				= "ac";
		charger_info.ac.type				= POWER_SUPPLY_TYPE_MAINS;
		charger_info.ac.properties			= max8903_charger_power_props;
		charger_info.ac.num_properties		= ARRAY_SIZE(max8903_charger_power_props);
		charger_info.ac.get_property		= max8903_charger_get_property;
		charger_info.ac.supplied_to			= max8903_charger_supplied_to;
		charger_info.ac.num_supplicants		= ARRAY_SIZE(max8903_charger_supplied_to);
		ret = power_supply_register(&pdev->dev, &charger_info.ac);
		if (ret) {
			dev_err(&pdev->dev, "failed to register ac\n");
			goto err_register_ac;
		}
	}

	if (pdata->avail_charge_sources & USB_CHG) {
		/* Create power supplies for usb*/
		charger_info.usb.name				= "usb";
		charger_info.usb.type				= POWER_SUPPLY_TYPE_USB;
		charger_info.usb.properties			= max8903_charger_power_props;
		charger_info.usb.num_properties		= ARRAY_SIZE(max8903_charger_power_props);
		charger_info.usb.get_property		= max8903_charger_get_property;
		charger_info.usb.supplied_to		= max8903_charger_supplied_to;
		charger_info.usb.num_supplicants	= ARRAY_SIZE(max8903_charger_supplied_to);
		ret = power_supply_register(&pdev->dev, &charger_info.usb);
		if (ret) {
			dev_err(&pdev->dev, "failed to register usb\n");
			goto err_register_usb;
		}
	}

	if (pdata->DCM_pin >=  0)  {
		ret = gpio_request(pdata->DCM_pin, "max8903_DCM_pin");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register DCM_pin (%d)\n", pdata->DCM_pin);
			goto err_request_max8903_DCM_pin;
		}
		ret = gpio_direction_output(pdata->DCM_pin, gpio_get_value_cansleep(pdata->DCM_pin));
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to set the direction of the  DCM_pin (%d)\n", pdata->DCM_pin);
			goto err_set_max8903_DCM_pin;
		}
		gpio_export(pdata->DCM_pin, true);
		gpio_export_link(&pdev->dev, "DCM_pin", pdata->DCM_pin);
	}

	if (pdata->IUSB_pin >=  0)  {
		ret = gpio_request(pdata->IUSB_pin, "max8903_IUSB_pin");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register IUSB_pin (%d)\n", pdata->IUSB_pin);
			goto err_request_max8903_IUSB_pin;
		}
		ret = gpio_direction_output(pdata->IUSB_pin, gpio_get_value_cansleep(pdata->IUSB_pin));
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to set the direction of the  IUSB_pin (%d)\n", pdata->IUSB_pin);
			goto err_set_max8903_IUSB_pin;
		}
		gpio_export(pdata->IUSB_pin, true);
		gpio_export_link(&pdev->dev, "IUSB_pin", pdata->IUSB_pin);
	}

	if (pdata->DOK_N_pin >=  0)  {
		ret = gpio_request(pdata->DOK_N_pin, "max8903_DOK_N_pin");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register DOK_N_pin (%d)\n", pdata->DOK_N_pin);
			goto err_request_max8903_DOK_N_pin;
		}
		ret = gpio_direction_input(pdata->DOK_N_pin);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to set the direction of the  DOK_N_pin (%d)\n", pdata->DOK_N_pin);
			goto err_set_max8903_DOK_N_pin;
		}
		charger_info.dok_irq = gpio_to_irq(pdata->DOK_N_pin);
		if (charger_info.dok_irq < 0) 
			goto err_get_max8903_dok_irq_num;
		ret = request_irq(charger_info.dok_irq, max8903_charger_dok_irq_handler, IRQF_TRIGGER_FALLING, "max8903_DOK_N_pin", NULL);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register dok_irq (%d)\n",charger_info.dok_irq );
			goto err_request_max8903_dok_irq;
		}
		gpio_export(pdata->DOK_N_pin, true);
		gpio_export_link(&pdev->dev, "DOK_N_pin", pdata->DOK_N_pin);
	}

	if (pdata->CEN_N_pin >=  0)  {
		ret = gpio_request(pdata->CEN_N_pin, "max8903_CEN_N_pin");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register CEN_N_pin (%d)\n", pdata->CEN_N_pin);
			goto err_request_max8903_CEN_N_pin;
		}
		ret = gpio_direction_output(pdata->CEN_N_pin, gpio_get_value_cansleep(pdata->CEN_N_pin));
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to set the direction of the  CEN_N_pin (%d)\n", pdata->CEN_N_pin);
			goto err_set_max8903_CEN_N_pin;
		}
		gpio_export(pdata->CEN_N_pin, true);
		gpio_export_link(&pdev->dev, "CEN_N_pin", pdata->CEN_N_pin);
	}

	if (pdata->USUS_pin >=  0)  {
		ret = gpio_request(pdata->USUS_pin, "max8903_USUS_pin");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register USUS_pin (%d)\n", pdata->USUS_pin);
			goto err_request_max8903_USUS_pin;
		}
		ret = gpio_direction_output(pdata->USUS_pin, gpio_get_value_cansleep(pdata->USUS_pin));
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to set the direction of the  USUS_pin (%d)\n", pdata->USUS_pin);
			goto err_set_max8903_USUS_pin;
		}
		gpio_export(pdata->USUS_pin, true);
		gpio_export_link(&pdev->dev, "USUS_pin", pdata->USUS_pin);
	}

	if (pdata->FLT_N_pin >= 0)  {
		ret = gpio_request(pdata->FLT_N_pin, "max8903_FLT_N_pin");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register FLT_N_pin (%d)\n", pdata->FLT_N_pin);
			goto err_request_max8903_FLT_N_pin;
		}
		ret = gpio_direction_input(pdata->FLT_N_pin);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to set the direction of the  FLT_N_pin (%d)\n", pdata->FLT_N_pin);
			goto err_set_max8903_FLT_N_pin;
		}
		charger_info.flt_irq= gpio_to_irq(pdata->FLT_N_pin);
		if (charger_info.flt_irq < 0)
			goto err_get_max8903_flt_irq_num;
		ret = request_irq(charger_info.flt_irq, max8903_charger_flt_irq_handler, IRQF_TRIGGER_FALLING, "max8903_FLT_N_pin", NULL);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register flt_irq (%d)\n",charger_info.flt_irq );
			goto err_request_max8903_flt_irq;
		}
		gpio_export(pdata->FLT_N_pin, true);
		gpio_export_link(&pdev->dev, "FLT_N_pin", pdata->FLT_N_pin);
	}

	if (pdata->UOK_N_pin >= 0)  {
		ret = gpio_request(pdata->UOK_N_pin, "max8903_UOK_N_pin");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register UOK_N_pin (%d)\n", pdata->UOK_N_pin);
			goto err_request_max8903_UOK_N_pin;
		}
		ret = gpio_direction_input(pdata->UOK_N_pin);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to set the direction of the  UOK_N_pin (%d)\n", pdata->UOK_N_pin);
			goto err_set_max8903_UOK_N_pin;
		}
		charger_info.uok_irq= gpio_to_irq(pdata->UOK_N_pin);
		if (charger_info.uok_irq < 0)
			goto err_get_max8903_uok_irq_num;
		ret = request_irq(charger_info.uok_irq, max8903_charger_uok_irq_handler, IRQF_TRIGGER_FALLING, "max8903_UOK_N_pin", NULL);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register uok_irq (%d)\n",charger_info.uok_irq );
			goto err_request_max8903_uok_irq;
		}
		gpio_export(pdata->UOK_N_pin, true);
		gpio_export_link(&pdev->dev, "UOK_N_pin", pdata->UOK_N_pin);
	}

	if (pdata->CHG_N_pin >= 0)  {
		ret = gpio_request(pdata->CHG_N_pin, "max8903_CHG_N_pin");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register CHG_N_pin (%d)\n", pdata->CHG_N_pin);
			goto err_request_max8903_CHG_N_pin;
		}
		ret = gpio_direction_input(pdata->CHG_N_pin);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to set the direction of the  CHG_N_pin (%d)\n", pdata->CHG_N_pin);
			goto err_set_max8903_CHG_N_pin;
		}
		
		charger_info.chg_irq= gpio_to_irq(pdata->CHG_N_pin);
		if (charger_info.chg_irq < 0)
			goto err_get_max8903_chg_irq_num;
		ret = request_irq(charger_info.chg_irq, max8903_charger_chg_irq_handler,  IRQF_TRIGGER_FALLING, "max8903_CHG_N_pin", NULL);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register chg_irq (%d)\n",charger_info.chg_irq );
			goto err_request_max8903_chg_irq;
		}
		gpio_export(pdata->CHG_N_pin, true);
		gpio_export_link(&pdev->dev, "CHG_N_pin", pdata->CHG_N_pin);
	}

	charger_info.platform_data = pdata;
	complete_all(&max8903_charger_init_done);

	pr_debug("MAX8903_CHARGER: %s : ---\n", __func__);
	return ret;

err_request_max8903_chg_irq:
err_get_max8903_chg_irq_num:
	if(pdata->CHG_N_pin >= 0)
		gpio_free(pdata->CHG_N_pin);
err_set_max8903_CHG_N_pin:
err_request_max8903_CHG_N_pin:
	if (pdata->UOK_N_pin >= 0)
		free_irq(charger_info.uok_irq, NULL);
err_request_max8903_uok_irq:
err_get_max8903_uok_irq_num:
	if (pdata->UOK_N_pin >= 0)
		gpio_free(pdata->UOK_N_pin);
err_set_max8903_UOK_N_pin:
err_request_max8903_UOK_N_pin:
	if (pdata->FLT_N_pin >= 0)
		free_irq(charger_info.flt_irq, NULL);
err_request_max8903_flt_irq:
err_get_max8903_flt_irq_num:
	if (pdata->FLT_N_pin >= 0)
		gpio_free(pdata->FLT_N_pin);
err_set_max8903_FLT_N_pin:
err_request_max8903_FLT_N_pin:
	if (pdata->USUS_pin >= 0)
		gpio_free(pdata->USUS_pin);
err_set_max8903_USUS_pin:
err_request_max8903_USUS_pin:
	if (pdata->CEN_N_pin >= 0)
		gpio_free(pdata->CEN_N_pin);
err_set_max8903_CEN_N_pin:
err_request_max8903_CEN_N_pin:
	if (pdata->DOK_N_pin >= 0)
		free_irq(charger_info.dok_irq, NULL);
err_request_max8903_dok_irq:
err_get_max8903_dok_irq_num:
	if (pdata->DOK_N_pin >= 0)
		gpio_free(pdata->DOK_N_pin);
err_set_max8903_DOK_N_pin:
err_request_max8903_DOK_N_pin:
	if (pdata->IUSB_pin >= 0)
		gpio_free(pdata->IUSB_pin);
err_set_max8903_IUSB_pin:
err_request_max8903_IUSB_pin:
	if (pdata->DCM_pin >= 0)
		gpio_free(pdata->DCM_pin);
err_set_max8903_DCM_pin:
err_request_max8903_DCM_pin:
	if (pdata->avail_charge_sources & USB_CHG)
		power_supply_unregister(&charger_info.usb);
err_register_usb:
	if (pdata->avail_charge_sources & AC_CHG)
		power_supply_unregister(&charger_info.ac);
err_register_ac:
err_board_init:

	return ret;

}

static int __devexit max8903_charger_remove(struct platform_device *pdev)
{
	struct max8903_charger_platfom_data* pdata = pdev->dev.platform_data;

	if(pdata->CHG_N_pin >= 0)
		free_irq(charger_info.chg_irq, NULL);
	if(pdata->CHG_N_pin >= 0)
		gpio_free(pdata->CHG_N_pin);
	if (pdata->UOK_N_pin >= 0)
		free_irq(charger_info.uok_irq, NULL);
	if (pdata->UOK_N_pin >= 0)
		gpio_free(pdata->UOK_N_pin);
	if (pdata->FLT_N_pin >= 0)
		free_irq(charger_info.flt_irq, NULL);
	if (pdata->FLT_N_pin >= 0)
		gpio_free(pdata->FLT_N_pin);
	if (pdata->USUS_pin >= 0)
		gpio_free(pdata->USUS_pin);
	if (pdata->CEN_N_pin >=  0)
		gpio_free(pdata->CEN_N_pin);
	if (pdata->DOK_N_pin >= 0)
		free_irq(charger_info.dok_irq, NULL);
	if (pdata->DOK_N_pin >= 0)
		gpio_free(pdata->DOK_N_pin);
	if (pdata->IUSB_pin >= 0)
		gpio_free(pdata->IUSB_pin);
	if (pdata->DCM_pin >=  0) 
		gpio_free(pdata->DCM_pin);

	if (pdata->avail_charge_sources & USB_CHG)
		power_supply_unregister(&charger_info.usb);
	if (pdata->avail_charge_sources & AC_CHG)
		power_supply_unregister(&charger_info.ac);

	return 0;
}

static struct platform_driver max8903_charger_driver = {
	.driver = {
		.name = "max8903-charger",
	},
	.probe = max8903_charger_probe,
	.remove = __devexit_p(max8903_charger_remove),
};

static int __init max8903_charger_init(void)
{
	charger_info.irq_workqueue = create_workqueue("max8903_charger_irq_workqueue");

	return platform_driver_register(&max8903_charger_driver);
}
module_init(max8903_charger_init);

static void __exit max8903_charger_exit(void)
{
	if (charger_info.irq_workqueue)
		destroy_workqueue(charger_info.irq_workqueue);
	platform_driver_unregister(&max8903_charger_driver);
}
module_exit(max8903_charger_exit);

MODULE_AUTHOR("Dijack Dong <dijack.dong@gmail.com>");
MODULE_DESCRIPTION("MAX8903 charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:max8903-charger");
