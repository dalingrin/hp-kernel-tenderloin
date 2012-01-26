/* -------------------------------------------------------------------------
 * Copyright (C) 2010 Inside contactless
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ------------------------------------------------------------------------- */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/slab.h>  //wanqin add for kmalloc/kfree
#include <mach/gpio.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>
#include <asm/delay.h>
#include <linux/i2c/nfc.h>  //wanqin add

//wanqin 
#if 0
#define GPIO_NFC_IRQOUT	      69
#define GPIO_NFC_WAKEUP	      33
#endif


#define I2C_READ_DATA_LENGTH  32    /* Number of bytes to Read during an I2C Read cycle (MicroRead TX FIFO Size + Len + CRC) */

   /* Context variable */
struct open_nfc_custom_device
{
	/* configuration stuff */

	/* ... extend, when needed ... */
	bool        configured;    /* set to true if OPEN_NFC_IOC_CONFIGURE has been done */

	/* I2C Driver related stuff */

	struct i2c_client *i2c_client;         /* I2C Driver registering structure */
	unsigned int irq;
	atomic_t irq_disable;
	struct timer_list  sResetTimer;        /* 10 ms one-shot Timer, used for RESET sequence */
	struct work_struct irqout_event_work;  /* WorkQueue, for bottom-half processing of IRQOUT */

	/* lock */	
	struct mutex   mutex;             /* mutex for concurrency */

	/* I2C receiver */

	uint8_t     rx_buffer[I2C_READ_DATA_LENGTH];
	int         nb_available_bytes;

	/* I2C transmitter */

	int         reset_pending;

	/* process synchronization */
	wait_queue_head_t read_queue;
   	wait_queue_head_t	write_queue;
   
   	//wanqin 
	struct nfc_platform_data *pdata;
};

static struct open_nfc_custom_device * open_nfc_p_device = NULL;

   /* Local Prototypes */ 
static void open_nfc_irqout_worker(struct work_struct *work);
static irqreturn_t open_nfc_i2c_interrupt(int irq, void *dev_id);

/**
  * Function called when the user opens /dev/nfcc
  *
  * @return 0 on success, a negative value on failure.
  */
int open_nfc_custom_open(struct inode *inode, struct file *filp)
{
	struct open_nfc_custom_device * p_device = open_nfc_p_device;
	int ret = 0;
	
	printk("##NFC##open_nfc_custom_open\n");
	/* IRQ started here only, and not in .probe, to avoid time-consuming as long as server is not launched */

	INIT_WORK(&p_device->irqout_event_work, open_nfc_irqout_worker);

	ret = request_any_context_irq(p_device->irq, open_nfc_i2c_interrupt, IRQF_TRIGGER_RISING, "IRQOUT_input", NULL);
	if (ret < 0) 
	{
		printk(KERN_ERR "open_nfc_custom_open : failed to register IRQOUT\n");
		return ret;
	}

	atomic_set(&p_device->irq_disable, 0);
	filp->private_data = open_nfc_p_device;

	return 0;
}

/**
  * Function called when the user closes file
  *
  * @return 0 on success, a negative value on failure.
  */
int open_nfc_custom_release(struct inode *inode, struct file *filp)
{
	struct open_nfc_custom_device * p_device = filp->private_data;

	mutex_lock(&p_device->mutex);

	/* ... */

	mutex_unlock(&p_device->mutex);

	return 0;
}

/**
  * Function called when the user reads data
  *
  * @return 0 on success, a negative value on failure.
  */
ssize_t open_nfc_custom_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct open_nfc_custom_device * p_device =filp->private_data;
	ssize_t retval;

	/* we allow read only if the connection with the NFC Controller has been established */
	mutex_lock(&p_device->mutex);

	retval = p_device->nb_available_bytes;

	if (retval == 0)
	{
		/* no data available */
		retval =  -EAGAIN;
		goto end;
	}

	if (count >= retval)
	{
		if (copy_to_user(buf, p_device->rx_buffer, p_device->nb_available_bytes))
		{
			printk(KERN_ERR "open_nfc_custom_read : unable to access to user buffer. data lost\n");
			retval = -EFAULT;
		}
	}
	else
	{
		printk(KERN_ERR "open_nfc_custom_read : provided buffer too short. data lost\n");
		retval = 0;
	}

	p_device->nb_available_bytes = 0;

end:
	mutex_unlock(&p_device->mutex);

	return retval;
}

/**
  * Function called when the user writes data
  *
  * @return 0 on success, a negative value on failure.
  */

ssize_t open_nfc_custom_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct open_nfc_custom_device * p_device = filp->private_data;
	void * temp;
	int retval;
	int rc;

	mutex_lock(&p_device->mutex);

	/* allocate a temporary kernel buffer to store user data */
	temp = kmalloc(count, GFP_KERNEL);

	if (temp == NULL)
	{
		printk(KERN_ERR "open_nfc_custom_write  : kmalloc failed\n");

		/* no memory available... */
		retval = -ENOMEM;
		goto end;
	}

	if (copy_from_user(temp, buf, count) == 0)
	{
		rc = i2c_master_send(p_device->i2c_client, temp, count);

		if (rc == count)
		{
			retval = count;
		}
		else
		{
			printk(KERN_ERR "open_nfc_custom_write : i2c_master_send() failed (returns %d)\n", rc);
			retval = -EFAULT;
		}
	}
	else
	{
		printk(KERN_ERR "open_nfc_custom_write  : copy_from_user failed\n");
		retval = -EFAULT;
   	}

	/* free the allocated buffer */
	kfree(temp);

end :

	mutex_unlock(&p_device->mutex);

	return retval;
}

/**
  * Processes the OPEN_NFC_IOC_CONFIGURE ioctl
  *
  * @return 0 on success, a negative value on failure.
  */

int open_nfc_custom_ioctl_configure(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct open_nfc_custom_device * p_device = filp->private_data;

	mutex_lock(&p_device->mutex);

	/* For now, no dynamic configuration parameter has been identified */

	p_device->configured = true;

	mutex_unlock(&p_device->mutex);

	return 0;
}

/**
  * Processes the OPEN_NFC_IOC_CONNECT ioctl
  *
  * @return 0 on success, a negative value on failure.
  */

int open_nfc_custom_ioctl_connect(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct open_nfc_custom_device * p_device = filp->private_data;
	int retval;

	mutex_lock(&p_device->mutex);

	/* nothing to connect actually */
	retval = 0;

	mutex_unlock(&p_device->mutex);

	return retval;
}

/**
  * Processes the 10 ms RESET Timeout handler
  *
  * @return 0 on success, a negative value on failure.
  */

static void open_nfc_reset_timeout (unsigned long arg)
{
	struct open_nfc_custom_device *p_device = open_nfc_p_device;

	mutex_lock(&p_device->mutex);

   	p_device->reset_pending--;

	if (p_device->reset_pending == 0)
	{
		wake_up(&p_device->write_queue);
	}
   	mutex_unlock(&p_device->mutex);
}

/**
  * Processes the OPEN_NFC_IOC_RESET ioctl
  *
  * @return 0 on success, a negative value on failure.
  */

int open_nfc_custom_ioctl_reset(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct open_nfc_custom_device * p_device = filp->private_data;

	mutex_lock(&p_device->mutex);

	printk("##NFC##open_nfc_custom_ioctl_reset\n");
#if 0
   /* Assert RST/WakeUP for at least 1 micro-second */
   gpio_direction_output(GPIO_NFC_WAKEUP, 1);
   udelay(2);
   gpio_direction_output(GPIO_NFC_WAKEUP, 0);
#endif

	if (p_device->pdata->nfc_reset() < 0)
   	{
		printk(KERN_ERR "open_nfc_custom_ioctl_reset\n");
		return -ENOMEM;
   	}

   	/* Hold Time of 10 ms before sending data to MicroRead */
   	p_device->sResetTimer.expires= jiffies + msecs_to_jiffies(10);
   	p_device->sResetTimer.data = (unsigned long)0;
   	p_device->sResetTimer.function = open_nfc_reset_timeout;

	add_timer(&p_device->sResetTimer);

	p_device->reset_pending++;

	mutex_unlock(&p_device->mutex);

	return 0;
}

/**
  * Process the poll()
  *
  * @return the poll status
  */

unsigned int open_nfc_custom_poll(struct file *filp, poll_table *wait)
{
	struct open_nfc_custom_device * p_device = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &p_device->read_queue, wait);
	poll_wait(filp, &p_device->write_queue, wait);

	mutex_lock(&p_device->mutex);

	if (p_device->reset_pending == 0)
	{
		mask = POLLOUT | POLLWRNORM;
	}

	if (p_device->nb_available_bytes)
   	{
		mask |= POLLIN | POLLRDNORM;
	}

	mutex_unlock(&p_device->mutex);

	return mask;
}

/**
  * Perform an I2C Read cycle, upon IRQOUT assertion.
  *
  * @return void
  */
static void open_nfc_irqout_read (struct open_nfc_custom_device *p_device)
{
	int rc = 0;
	unsigned char *p_buffer;
	static unsigned char dummy[I2C_READ_DATA_LENGTH];

	/* Do NOT interrupt an outgoing WRITE cycle */
	mutex_lock(&p_device->mutex);

	/* Trivial implementation: only one buffering */
	if (p_device->nb_available_bytes != 0) 
	{
		p_buffer = dummy;  /* Read to have IRQOUT acknowledged, but discard the DATA to dummy buffer */
	} 
	else 
	{
		p_buffer = p_device->rx_buffer;
   	}

	rc = i2c_master_recv(p_device->i2c_client, p_buffer, I2C_READ_DATA_LENGTH);

	if (rc != I2C_READ_DATA_LENGTH) 
	{
		printk( "open_nfc_irqout_read failed (returns %d)", rc );
	}
	else
	{
   	if (p_device->rx_buffer[0] > I2C_READ_DATA_LENGTH - 2) 
 	{
      		printk( KERN_ERR "open_nfc_irqout_read ERROR: invalid length read from I2C Device" );
   	}
   	else
   	{
		p_device->nb_available_bytes = p_device->rx_buffer[0] + 2; /* Add "Len" + CRC fields ; ignore padding bytes */
		/* wake up poll() */
		wake_up(&p_device->read_queue);
   	}
   }

	mutex_unlock(&p_device->mutex);
}

/**
  * Processing, after IRQOUT processing in bot2c_client *tom-half.
  * Re-enable he LEVEL-sensitive IRQOUT.
  *
  * @return void
  */
static void open_nfc_irqout_worker(struct work_struct *work)
{
	struct open_nfc_custom_device *p_device = open_nfc_p_device;

	/* Process the IRQOUT event */
	open_nfc_irqout_read(p_device);

	/* Re-enable the interrupt */

	atomic_dec(&p_device->irq_disable);
	enable_irq(p_device->irq);
}

/**
  * IRQOUT Interrupt handler.
  * Schedules the bottom-half.
  *
  * @return the IRQ processing status
  */

static irqreturn_t open_nfc_i2c_interrupt(int irq, void *dev_id)
{
	struct open_nfc_custom_device *p_device = open_nfc_p_device;

	if (!work_pending(&p_device->irqout_event_work)) 
	{
		disable_irq_nosync(p_device->irq);
		atomic_inc(&p_device->irq_disable);
		schedule_work(&p_device->irqout_event_work);
	}
	return IRQ_HANDLED;
}

/**
  * Device/Driver binding: probe
  *
  * @return 0 if successfull, or error code
  */
static int opennfc_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	struct open_nfc_custom_device * p_device;
	
	printk("##NFC##opennfc_probe\n");
	
	if(client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		return -ENOMEM;
	}
	p_device = open_nfc_p_device;

	p_device->i2c_client = client;
	
	p_device->pdata = kmalloc(sizeof(struct nfc_platform_data), GFP_KERNEL);
	if(p_device == NULL)
	{
		dev_err(&client->dev, "opennfc_probe platform data is malloc error\n");
		return -ENOMEM;
	}
	
	memcpy(p_device->pdata, client->dev.platform_data, sizeof(struct nfc_platform_data));
	if(p_device->pdata->nfc_iocfg() < 0)
	{
		printk(KERN_ERR "opennfc_probe io cfg fail");
		return -ENOMEM;
	}
	
	p_device->irq = p_device->pdata->nfc_irq();
	
	if(p_device->irq < 0) 
	{
		printk(KERN_ERR "opennfc_probe no irq support");
		return -ENOMEM;
	}
   
	//wanqin add reset function to enable UICC power when booting
	if (p_device->pdata->nfc_reset() < 0)
		return -ENOMEM;
	i2c_set_clientdata(client, p_device);

	return 0;
}	

/**
  * Device/Driver binding: remove
  *
  * @return 0 if successfull, or error code
  */
static int opennfc_remove(struct i2c_client *client)
{
	struct open_nfc_custom_device * p_device;

	p_device = open_nfc_p_device;

	i2c_set_clientdata(p_device->i2c_client, NULL);

	return 0;
}

/* 
   Client structure holds device-specific information like the 
   driver model device node, and its I2C address 
*/
static const struct i2c_device_id opennfc_id[] = {
	{ "opennfc", 0x5E },
};

MODULE_DEVICE_TABLE(i2c, opennfc_id);

static struct i2c_driver open_nfc_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name = "opennfc",
	},
	.probe		= opennfc_probe,
	.remove		= __devexit_p(opennfc_remove),
	.id_table	= opennfc_id,
};

/**
  * Specific initialization, when driver module is inserted.
  *
  * @return 0 if successfull, or error code
  */
int open_nfc_custom_init(void)
{
	int ret = 0;
#if 0  //wanqin
	int rc;
#endif

	struct open_nfc_custom_device * p_device;


	p_device = kmalloc(sizeof(struct open_nfc_custom_device), GFP_KERNEL);
	if (p_device == NULL) 
   	{
	   return -ENOMEM;
	}
	memset(p_device, 0, sizeof(struct open_nfc_custom_device));	

      	/* Save device context (needed in .probe()) */
	open_nfc_p_device = p_device;
	

	init_waitqueue_head(&p_device->read_queue);
	init_waitqueue_head(&p_device->write_queue);

	mutex_init(&p_device->mutex);

  	init_timer(&p_device->sResetTimer);

//wanqin 
#if 0
      /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
      /* management of IRQOUT (IRQ + WorkQueue)                      */
      /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
	if(gpio_tlmm_config(GPIO_CFG(GPIO_NFC_IRQOUT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
   {
		printk("ERR in config IRQ_OUT\n");
   }

   rc = gpio_request(GPIO_NFC_IRQOUT, "nfc_int");
   if(rc < 0)
   {
	   printk("failed to request GPIO for IRQOUT! error = %d\n", rc);
	   return rc;
   }

   gpio_configure(GPIO_NFC_IRQOUT, GPIOF_INPUT);

   rc = gpio_direction_input(GPIO_NFC_IRQOUT);
   if(rc < 0)
   {
	   printk("failed to set direction of IRQOUT GPIO! error = %d\n", rc);
	   return rc;
   }

   p_device->irq = MSM_GPIO_TO_INT(GPIO_NFC_IRQOUT);
   atomic_set(&p_device->irq_disable, 0);

      /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
      /* RST/WakeUP programming                                      */
      /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

	if(gpio_tlmm_config(GPIO_CFG(GPIO_NFC_WAKEUP, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
   {
		printk("ERR in config WAKEUP\n");
   }

	rc = gpio_request(GPIO_NFC_WAKEUP, "nfc_wake");
	if (rc < 0)
	{
		printk (KERN_ERR "failed to request GPIO for RST/WakeUP ! error = %d\n", rc);
		return rc;
	}

	gpio_configure(GPIO_NFC_WAKEUP, GPIOF_DRIVE_OUTPUT);

	rc = gpio_direction_output(GPIO_NFC_WAKEUP, 0);
	if (rc < 0)
	{
		printk("fail to set direction of RST/WakeUP GPIO output HIGH! error = %d\n", rc);
		return rc;
	}
#endif

      /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
      /* I2C Driver connection                                       */
      /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

   	ret = i2c_add_driver(&open_nfc_i2c_driver);

   	return ret;
}

/**
  * Specific cleanup, when driver module is removed.
  *
  * @return void
  */
void open_nfc_custom_exit(void)
{
	struct open_nfc_custom_device * p_device;

	p_device = open_nfc_p_device;

	free_irq(p_device->irq, p_device);
	cancel_work_sync(&p_device->irqout_event_work);

	/*
	 * If work indeed has been cancelled, disable_irq() will have been left
	 * unbalanced from open_nfc_irqout_interrupt().
	 */
	while (atomic_dec_return(&p_device->irq_disable) >= 0)
   	{
		enable_irq(p_device->irq);
	}

	kfree(p_device->pdata);
	/* free the custom device context */
	kfree (p_device);

	open_nfc_p_device = NULL;

	i2c_del_driver(&open_nfc_i2c_driver);
}

/* EOF */

