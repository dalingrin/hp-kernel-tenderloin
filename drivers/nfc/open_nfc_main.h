/* -------------------------------------------------------------------------
 * Copyright (C) 2010 Inside Secure
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

/*
 * Open NFC driver module.
 *
 */

#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/poll.h>

typedef int32_t    sint32_t;
typedef uint16_t    tchar;

 /* Set Open NFC major number.
  *
  * If set to 0, a free major number will be allocated for the driver.
  *   If set to a value different from 0, the specified value will be used as a major value
  */

#define OPEN_NFC_MAJOR            0

/*
 * Use 'o' as magic number
 *
 * May be customized if conflict occurs
 */

#define OPEN_NFC_IOC_MAGIC       'o'

/**
 * OPEN_NFC_IOC_CONFIGURE configuration structure
 *
 * Customer implementation may increase the OPEN_NFC_IOC_MAX_CONFIG_LENGTH if needed
 */

#define OPEN_NFC_IOC_MAX_CONFIG_LENGTH      16

struct open_nfc_ioc_configure
{
    int length;                                       /* actual configuration length */
    unsigned char buffer[OPEN_NFC_IOC_MAX_CONFIG_LENGTH];      /* buffer containing the config value */
};

/**
 * OPEN_NFC_IOC_CONFIGURE is used to pass configuration data to the Open NFC driver.
 *
 * In our driver implementation, this is used to pass the IP address of the connection center
 *
 * Customer implementation can use this IOCTL for any purpose
 */

#define OPEN_NFC_IOC_CONFIGURE   _IOW(OPEN_NFC_IOC_MAGIC, 0, struct open_nfc_ioc_configure)


/**
  * OPEN_NFC_IOC_CONNECT is used to request Open NFC driver to prepare to communicate with the NFC Controller.
  *
  * In our driver implementation, this is used to establish TCP connection with the connection center
  *
  * Customer implementation can use this IOCTL for any purpose
  */

#define OPEN_NFC_IOC_CONNECT     _IO(OPEN_NFC_IOC_MAGIC, 1)


/**
  * OPEN_NFC_IOC_RESET is used to request Open NFC driver to reset the NFC Controller.
  *
  * Note : The reset IOCTL is asynchronous. The Open NFC stack checks the completion of
  * the reset by selecting the fd for writing (using select()).
  */

#define OPEN_NFC_IOC_RESET       _IO(OPEN_NFC_IOC_MAGIC, 2)


#define OPEN_NFC_MAX_IOCTL_VALUE  2


/* function prototypes */

int open_nfc_custom_init(void);
void open_nfc_custom_exit(void);
int open_nfc_custom_open(struct inode *inode, struct file *filp);
int open_nfc_custom_release(struct inode *inode, struct file *filp);
ssize_t open_nfc_custom_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t open_nfc_custom_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
unsigned int open_nfc_custom_poll(struct file * filp, poll_table *wait);
int open_nfc_custom_ioctl_configure(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
int open_nfc_custom_ioctl_connect(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
int open_nfc_custom_ioctl_reset(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);

/* EOF */
