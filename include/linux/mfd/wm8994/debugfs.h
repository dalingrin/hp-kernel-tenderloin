
#ifndef __MFD_WM8994_DEBUGFS_H__
#define __MFD_WM8994_DEBUGFS_H__



#include <linux/types.h>
#include <linux/ioctl.h>
#include <asm/sizes.h>


#define WM8994_IOCTL_MAGIC 'b'

#define WM8994_REG_WRITE        _IOW(WM8994_IOCTL_MAGIC, 0, unsigned)
#define WM8994_REG_READ         _IOW(WM8994_IOCTL_MAGIC, 1, unsigned)


struct wm8994_reg_config {
        u16 addr;
        u16 val;
};

#endif
