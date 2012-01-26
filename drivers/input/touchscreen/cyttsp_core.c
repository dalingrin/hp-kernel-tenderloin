/* Source for:
 * Cypress TrueTouch(TM) Standard Product touchscreen driver.
 * drivers/input/touchscreen/cyttsp_core.c
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/byteorder/generic.h>
#include <linux/bitops.h>
#include <linux/earlysuspend.h>
#include <linux/cyttsp_hp.h>
#include <linux/ctype.h>
#include "cyttsp_core.h"

#define DBG(x) 
#define DBG2(x)
#define DBG3(x)
#define DBG4(x)

/* rely on kernel input.h to define Multi-Touch capability */
#ifndef ABS_MT_TRACKING_ID
/* define only if not defined already by system; */
/* value based on linux kernel 2.6.30.10 */
#define ABS_MT_TRACKING_ID (ABS_MT_BLOB_ID + 1)
#endif /* ABS_MT_TRACKING_ID */

#define	TOUCHSCREEN_TIMEOUT (msecs_to_jiffies(28))
/* Bootloader File 0 offset */
#define CY_BL_FILE0       0x00
/* Bootloader command directive */
#define CY_BL_CMD         0xFF
/* Bootloader Enter Loader mode */
#define CY_BL_ENTER       0x38
/* Bootloader Write a Block */
#define CY_BL_WRITE_BLK   0x39
/* Bootloader Terminate Loader mode */
#define CY_BL_TERMINATE   0x3B
/* Bootloader Exit and Verify Checksum command */
#define CY_BL_EXIT        0xA5
/* Bootloader default keys */
#define CY_BL_KEY0 0
#define CY_BL_KEY1 1
#define CY_BL_KEY2 2
#define CY_BL_KEY3 3
#define CY_BL_KEY4 4
#define CY_BL_KEY5 5
#define CY_BL_KEY6 6
#define CY_BL_KEY7 7

#define CY_DIFF(m, n)               ((m) != (n))
#define GET_NUM_TOUCHES(x)          ((x) & 0x0F)
#define GET_TCH_ID_HI(x)            (((x) & 0xF0) >> 4)
#define GET_TCH_ID_LO(x)            ((x) & 0x0F)
#define GET_TOUCH1_ID(x)            GET_TCH_ID_HI(x)
#define GET_TOUCH2_ID(x)            GET_TCH_ID_LO(x)
#define GET_TOUCH3_ID(x)            GET_TCH_ID_HI(x)
#define GET_TOUCH4_ID(x)            GET_TCH_ID_LO(x)
#define GET_TOUCH5_ID(x)            GET_TCH_ID_HI(x)
#define GET_TOUCH6_ID(x)            GET_TCH_ID_LO(x)
#define GET_TOUCH7_ID(x)            GET_TCH_ID_HI(x)
#define GET_TOUCH8_ID(x)            GET_TCH_ID_LO(x)
#define GET_TOUCH9_ID(x)            GET_TCH_ID_HI(x)
#define GET_TOUCH10_ID(x)           GET_TCH_ID_LO(x)
#define IS_LARGE_AREA(x)            (((x) & 0x10) >> 4)
#define IS_BAD_PKT(x)               GET_TCH_ID_LO(x)
#define FLIP_DATA_FLAG              0x01
#define REVERSE_X_FLAG              0x02
#define REVERSE_Y_FLAG              0x04
#define FLIP_DATA(flags)            ((flags) & FLIP_DATA_FLAG)
#define REVERSE_X(flags)            ((flags) & REVERSE_X_FLAG)
#define REVERSE_Y(flags)            ((flags) & REVERSE_Y_FLAG)
#define FLIP_XY(x, y)      {typeof(x) tmp; tmp = (x); (x) = (y); (y) = tmp; }
#define INVERT_X(x, xmax)           ((xmax) - (x))
#define INVERT_Y(y, ymax)           ((ymax) - (y))
#define SET_HSTMODE(reg, mode)      ((reg) & (mode))
#define GET_HSTMODE(reg)            ((reg & 0x70) >> 4)
#define GET_BOOTLOADERMODE(reg)     ((reg & 0x10) >> 4)

/* maximum number of concurrent ST track IDs */
#define CY_NUM_ST_TCH_ID            2
/* maximum number of concurrent MT track IDs */
#define CY_NUM_MT_4TCH              4
#define CY_NUM_MT_10TCH             10
#define CY_NUM_MT_TCH_ID            CY_NUM_MT_10TCH
/* maximum number of track IDs */
#define CY_NUM_TRK_ID               16

#define CY_NTCH                     0 /* lift off */
#define CY_TCH                      1 /* touch down */
#define CY_ST_FNGR1_IDX             0
#define CY_ST_FNGR2_IDX             1
#define CY_MT_TCH1_IDX              0
#define CY_MT_TCH2_IDX              1
#define CY_MT_TCH3_IDX              2
#define CY_MT_TCH4_IDX              3
#define CY_MT_TCH5_IDX              4
#define CY_MT_TCH6_IDX              5
#define CY_MT_TCH7_IDX              6
#define CY_MT_TCH8_IDX              7
#define CY_MT_TCH9_IDX              8
#define CY_MT_TCH10_IDX             9
#define CY_XPOS                     0
#define CY_YPOS                     1
#define CY_IGNR_TCH               (-1)
#define CY_SMALL_TOOL_WIDTH         10
#define CY_LARGE_TOOL_WIDTH         255
#define CY_REG_BASE                 0x00
#define CY_REG_GEST_SET             0x1E
#define CY_REG_ACT_INTRVL           0x1D
#define CY_REG_TCH_TMOUT            (CY_REG_ACT_INTRVL+1)
#define CY_REG_LP_INTRVL            (CY_REG_TCH_TMOUT+1)
#define CY_SOFT_RESET               (1 << 0)
#define CY_DEEP_SLEEP               (1 << 1)
#define CY_LOW_POWER                (1 << 2)
#define CY_MAXZ                     255
#define CY_OK                       0
#define CY_INIT                     1
#define CY_DELAY_DFLT               10 /* ms */
#define CY_DELAY_SYSINFO            20 /* ms */
#define CY_DELAY_BL                 300
#define CY_DELAY_DNLOAD             100 /* ms */
#define CY_HNDSHK_BIT               0x80
#define CY_NUM_RETRY                4 /* max number of retries for read ops */
/* device mode bits */
#define CY_OPERATE_MODE             0x00
#define CY_SYSINFO_MODE             0x10
/* power mode select bits */
#define CY_SOFT_RESET_MODE          0x01 /* return to Bootloader mode */
#define CY_DEEP_SLEEP_MODE          0x08
#define CY_LOW_POWER_MODE           0x04
#define CY_NUM_KEY                  8
#define CY_DFLT_Z                   200

/* Touch structure */
struct cyttsp_touch {
	u16 x __attribute__ ((packed));
	u16 y __attribute__ ((packed));
	u8 z;
};
/* TrueTouch Standard Product Gen2 (Txx2xx) interface definition */
struct cyttsp_xydata_gen2 {
	u8 hst_mode;
	u8 tt_mode;
	u8 tt_stat;
	u16 x1 __attribute__ ((packed));
	u16 y1 __attribute__ ((packed));
	u8 z1;
	u8 evnt_idx;
	u16 x2 __attribute__ ((packed));
	u16 y2 __attribute__ ((packed));
	u8 tt_undef1;
	u8 gest_cnt;
	u8 gest_id;
	u8 tt_undef[14];
	u8 gest_set;
	u8 tt_reserved;
};

/* TrueTouch Standard Product Gen3 (Txx3xx) interface definition */
struct cyttsp_xydata_gen3 {
	u8 hst_mode;
	u8 tt_mode;
	u8 tt_stat;
	struct cyttsp_touch tch1;
	u8 touch12_id;
	struct cyttsp_touch tch2;
	u8 gest_cnt;
	u8 gest_id;
	struct cyttsp_touch tch3;
	u8 touch34_id;
	struct cyttsp_touch tch4;
	u8 tt_undef[3];
	u8 gest_set;
	u8 tt_reserved;
};

/* TrueTouch Standard Product Gen4 (Txx4xx) interface definition */
struct cyttsp_xydata_ext {
	struct cyttsp_touch tch5;
	u8 touch56_id;
	struct cyttsp_touch tch6;
	struct cyttsp_touch tch7;
	u8 touch78_id;
	struct cyttsp_touch tch8;
	struct cyttsp_touch tch9;
	u8 touch910_id;
	struct cyttsp_touch tch10;
};

struct cyttsp_xydata_gen4 {
	struct cyttsp_xydata_gen3 regs;
	struct cyttsp_xydata_ext xregs;
};

// yegw  2010-10-2
// yegw  2010-10-2

#define CY_REG_BASE                 0x00
#define MAX_REG_ADDR                0X40

#define   MAX_SUPPORT_FINGERS    10

u16  fingerXH_addr[MAX_SUPPORT_FINGERS] = 
    { 0x03, 0x09, 0x10, 0x16, 0x20, 0x26, 0x2B, 0x31, 0x36, 0x3C };
    
u16  fingerIndex_addr[MAX_SUPPORT_FINGERS] =
    {0x08, 0x08, 0x15, 0x15, 0x25, 0x25, 0x30, 0x30, 0x3B,0x3B};

/* Touch structure */
struct cyttsp_touch_hp {
    u16 x ;
    u16 y ;
    u8  z;
    u8  fingerID;
};

struct cyttsp_xydata_gen4_hp {
    struct cyttsp_touch_hp fingers_reg[MAX_SUPPORT_FINGERS];
};

static void        touchscreen_work_f(struct work_struct *work);

// yegw --



enum cyttsp_gen2_std {
	CY_GEN2_NOTOUCH = 0x03, /* Both touches removed */
	CY_GEN2_GHOST =   0x02, /* ghost */
	CY_GEN2_2TOUCH =  0x03, /* 2 touch; no ghost */
	CY_GEN2_1TOUCH =  0x01, /* 1 touch only */
	CY_GEN2_TOUCH2 =  0x01, /* 1st touch removed; 2nd touch remains */
};

/* TTSP System Information interface definition */
struct cyttsp_sysinfo_data {
	u8 hst_mode;
	u8 mfg_stat;
	u8 mfg_cmd;
	u8 cid[3];
	u8 tt_undef1;
	u8 uid[8];
	u8 bl_verh;
	u8 bl_verl;
	u8 tts_verh;
	u8 tts_verl;
	u8 app_idh;
	u8 app_idl;
	u8 app_verh;
	u8 app_verl;
	u8 tt_undef[5];
	u8 scn_typ;	/* Gen3 only: scan type [0:Mutual, 1:Self] */
	u8 act_intrvl;
	u8 tch_tmout;
	u8 lp_intrvl;
};

/* TTSP Bootloader Register Map interface definition */
#define CY_BL_CHKSUM_OK 0x01
struct cyttsp_bootloader_data {
	u8 bl_file;
	u8 bl_status;
	u8 bl_error;
	u8 blver_hi;
	u8 blver_lo;
	u8 bld_blver_hi;
	u8 bld_blver_lo;
	u8 ttspver_hi;
	u8 ttspver_lo;
	u8 appid_hi;
	u8 appid_lo;
	u8 appver_hi;
	u8 appver_lo;
	u8 cid_0;
	u8 cid_1;
	u8 cid_2;
};

#define cyttsp_wake_data cyttsp_xydata

struct cyttsp {
	struct device *pdev;
	int irq;
	struct input_dev *input;
	struct work_struct work;
    struct workqueue_struct * irq_workqueue;
	struct timer_list timer;
	struct mutex mutex;
	struct early_suspend early_suspend;
	char phys[32];
	struct cyttsp_hp_platform_data *platform_data;
	struct cyttsp_bootloader_data bl_data;
	struct cyttsp_sysinfo_data sysinfo_data;
	u8 num_prv_st_tch;
	u16 act_trk[CY_NUM_TRK_ID];
	u16 prv_mt_tch[CY_NUM_MT_TCH_ID];
	u16 prv_mt_pos[CY_NUM_TRK_ID][2];
	struct cyttsp_bus_ops *bus_ops;
	unsigned fw_loader_mode:1;
	unsigned suspended:1;
	struct timer_list to_timer;
	bool to_timeout;
	bool bl_ready;
	u8 reg_id;
    u8 prev_touches;
    bool isInterrupt_Enabled;
    bool isSuspend;
};

struct cyttsp_track_data {
	u8 prv_tch;
	u8 cur_tch;
	u16 tmp_trk[CY_NUM_MT_TCH_ID];
	u16 snd_trk[CY_NUM_MT_TCH_ID];
	u16 cur_trk[CY_NUM_TRK_ID];
	/* if NOT CY_USE_TRACKING_ID then only */
	/* uses CY_NUM_MT_TCH_ID positions */
	u16 cur_mt_tch[CY_NUM_TRK_ID];
	/* if NOT CY_USE_TRACKING_ID then only */
	/* uses CY_NUM_MT_TCH_ID positions */
	u16 cur_mt_pos[CY_NUM_TRK_ID][2];
	/* if NOT CY_USE_TRACKING_ID then only */
	/* uses CY_NUM_MT_TCH_ID positions */
	u8 cur_mt_z[CY_NUM_TRK_ID];
	u8 tool_width;
};

static const u8 bl_cmd[] = {
	CY_BL_FILE0, CY_BL_CMD, CY_BL_EXIT,
	CY_BL_KEY0, CY_BL_KEY1, CY_BL_KEY2,
	CY_BL_KEY3, CY_BL_KEY4, CY_BL_KEY5,
	CY_BL_KEY6, CY_BL_KEY7
};

#define LOCK(m) do { \
	DBG(printk(KERN_INFO "%s: lock\n", __func__);) \
	mutex_lock(&(m)); \
} while (0);

#define UNLOCK(m) do { \
	DBG(printk(KERN_INFO "%s: unlock\n", __func__);) \
	mutex_unlock(&(m)); \
} while (0);

DBG(
static void print_data_block(const char *func, u8 command,
			u8 length, void *data)
{
	char buf[1024];
	unsigned buf_len = sizeof(buf);
	char *p = buf;
	int i;
	int l;

	l = snprintf(p, buf_len, "cmd 0x%x: ", command);
	buf_len -= l;
	p += l;
	for (i = 0; i < length && buf_len; i++, p += l, buf_len -= l)
		l = snprintf(p, buf_len, "%02x ", *((char *)data + i));
	printk(KERN_DEBUG "%s: %s\n", func, buf);
})

static irqreturn_t cyttsp_bl_ready_irq(int irq, void *handle);
static int cyttsp_soft_reset(struct cyttsp *ts, bool *status);
static int cyttsp_set_operational_mode(struct cyttsp *ts);
static int cyttsp_exit_bl_mode(struct cyttsp *ts);
static void cyttsp_init_tch(struct cyttsp *ts)
{
	/* init the touch structures */
	ts->num_prv_st_tch = CY_NTCH;
	memset(ts->act_trk, CY_NTCH, sizeof(ts->act_trk));
	memset(ts->prv_mt_pos, CY_NTCH, sizeof(ts->prv_mt_pos));
	memset(ts->prv_mt_tch, CY_IGNR_TCH, sizeof(ts->prv_mt_tch));
}
#if 0
static u8 ttsp_convert_gen2(u8 cur_tch, struct cyttsp_xydata_gen3 *pxy_data)
{
	struct cyttsp_xydata_gen2 *pxy_data_gen2;
	pxy_data_gen2 = (struct cyttsp_xydata_gen2 *)(pxy_data);

	if (pxy_data_gen2->evnt_idx == CY_GEN2_NOTOUCH) {
		cur_tch = 0;
	} else if (cur_tch == CY_GEN2_GHOST) {
		cur_tch = 0;
	} else if (cur_tch == CY_GEN2_2TOUCH) {
		/* stuff artificial track ID1 and ID2 */
		pxy_data->touch12_id = 0x12;
		pxy_data->tch1.z = CY_MAXZ;
		pxy_data->tch2.z = CY_MAXZ;
		cur_tch--; /* 2 touches */
	} else if (cur_tch == CY_GEN2_1TOUCH) {
		/* stuff artificial track ID1 and ID2 */
		pxy_data->touch12_id = 0x12;
		pxy_data->tch1.z = CY_MAXZ;
		pxy_data->tch2.z = CY_NTCH;
		if (pxy_data_gen2->evnt_idx == CY_GEN2_TOUCH2) {
			/* push touch 2 data into touch1
			 * (first finger up; second finger down) */
			/* stuff artificial track ID1 for touch2 info */
			pxy_data->touch12_id = 0x20;
			/* stuff touch 1 with touch 2 coordinate data */
			pxy_data->tch1.x = pxy_data->tch2.x;
			pxy_data->tch1.y = pxy_data->tch2.y;
		}
	} else {
		cur_tch = 0;
	}
	return cur_tch;
}
#endif
static int ttsp_read_block_data(struct cyttsp *ts, u8 command,
	u8 length, void *buf)
{
	int rc;
	int tries;
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	if (!buf || !length) {
		printk(KERN_ERR "%s: Error, buf:%s len:%u\n",
				__func__, !buf ? "NULL" : "OK", length);
		return -EIO;
	}

	for (tries = 0, rc = -1; tries < CY_NUM_RETRY && (rc < 0); tries++)
		rc = ts->bus_ops->read(ts->bus_ops, command, length, buf);

	if (rc < 0)
		printk(KERN_ERR "%s: error %d\n", __func__, rc);
	DBG(print_data_block(__func__, command, length, buf);)
	return rc;
}

static int ttsp_write_block_data(struct cyttsp *ts, u8 command,
	u8 length, void *buf)
{
	int rc;
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	if (!buf || !length) {
		printk(KERN_ERR "%s: Error, buf:%s len:%u\n",
				__func__, !buf ? "NULL" : "OK", length);
		return -EIO;
	}
	rc = ts->bus_ops->write(ts->bus_ops, command, length, buf);
	if (rc < 0)
		printk(KERN_ERR "%s: error %d\n", __func__, rc);
	DBG(print_data_block(__func__, command, length, buf);)
	return rc;
}
/*
static int ttsp_tch_ext(struct cyttsp *ts, void *buf)
{
	int rc;
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	if (!buf) {
		printk(KERN_ERR "%s: Error, buf:%s\n",
				__func__, !buf ? "NULL" : "OK");
		return -EIO;
	}
	rc = ts->bus_ops->ext(ts->bus_ops, buf);
	if (rc < 0)
		printk(KERN_ERR "%s: error %d\n", __func__, rc);
	return rc;
}
*/
static int cyttsp_inlist(u16 prev_track[], u8 cur_trk_id, u8 *prev_loc,
	u8 num_touches)
{
	u8 id = 0;

	DBG(printk(KERN_INFO"%s: IN p[%d]=%d c=%d n=%d loc=%d\n",
		__func__, id, prev_track[id], cur_trk_id,
		num_touches, *prev_loc);)

	for (*prev_loc = CY_IGNR_TCH; id < num_touches; id++) {
		DBG(printk(KERN_INFO"%s: p[%d]=%d c=%d n=%d loc=%d\n",
			__func__, id, prev_track[id], cur_trk_id,
				num_touches, *prev_loc);)
		if (prev_track[id] == cur_trk_id) {
			*prev_loc = id;
			break;
		}
	}
	DBG(printk(KERN_INFO"%s: OUT p[%d]=%d c=%d n=%d loc=%d\n", __func__,
		id, prev_track[id], cur_trk_id, num_touches, *prev_loc);)

	return *prev_loc < CY_NUM_TRK_ID;
}

static int cyttsp_next_avail_inlist(u16 cur_trk[], u8 *new_loc,
	u8 num_touches)
{
	u8 id = 0;
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	for (*new_loc = CY_IGNR_TCH; id < num_touches; id++) {
		if (cur_trk[id] > CY_NUM_TRK_ID) {
			*new_loc = id;
			break;
		}
	}
	return *new_loc < CY_NUM_TRK_ID;
}

/* Timer function used as dummy interrupt driver */
static void cyttsp_timer(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	DBG(printk(KERN_INFO"%s: TTSP timer event!\n", __func__);)
	/* schedule motion signal handling */
	if (!work_pending(&ts->work))
		schedule_work(&ts->work);
	mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
	return;
}


/* ************************************************************************
 * ISR function. This function is general, initialized in drivers init
 * function and disables further IRQs until this IRQ is processed in worker.
 * *************************************************************************/
static irqreturn_t cyttsp_irq(int irq, void *handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	DBG(printk(KERN_INFO"%s: Got IRQ!\n", __func__);)
     
     if(!(ts->isSuspend))
     {
        disable_irq_nosync(irq);
        queue_work(ts->irq_workqueue, &ts->work);
     }

	return IRQ_HANDLED;
}

/* ************************************************************************
 * The cyttsp_xy_worker function reads the XY coordinates and sends them to
 * the input layer.  It is scheduled from the interrupt (or timer).
 * *************************************************************************/
void handle_multi_touch(struct cyttsp_track_data *t, struct cyttsp *ts)
{

	u8 id;
	u8 i, loc;
	void (*mt_sync_func)(struct input_dev *) = ts->platform_data->mt_sync;

	if (!ts->platform_data->use_trk_id)
		goto no_track_id;

	/* terminate any previous touch where the track
	 * is missing from the current event */
	for (id = 0; id < CY_NUM_TRK_ID; id++) {
		if ((ts->act_trk[id] == CY_NTCH) || (t->cur_trk[id] != CY_NTCH))
			continue;

		input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, CY_NTCH);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, t->tool_width);
		input_report_abs(ts->input, ABS_MT_POSITION_X,
					ts->prv_mt_pos[id][CY_XPOS]);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,
					ts->prv_mt_pos[id][CY_YPOS]);
		if (mt_sync_func)
			mt_sync_func(ts->input);
		ts->act_trk[id] = CY_NTCH;
		ts->prv_mt_pos[id][CY_XPOS] = 0;
		ts->prv_mt_pos[id][CY_YPOS] = 0;
	}
	/* set Multi-Touch current event signals */
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		if (t->cur_mt_tch[id] >= CY_NUM_TRK_ID)
			continue;

		input_report_abs(ts->input, ABS_MT_TRACKING_ID,
						t->cur_mt_tch[id]);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
						t->cur_mt_z[id]);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
						t->tool_width);
		input_report_abs(ts->input, ABS_MT_POSITION_X,
						t->cur_mt_pos[id][CY_XPOS]);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,
						t->cur_mt_pos[id][CY_YPOS]);
		if (mt_sync_func)
			mt_sync_func(ts->input);

		ts->act_trk[id] = CY_TCH;
		ts->prv_mt_pos[id][CY_XPOS] = t->cur_mt_pos[id][CY_XPOS];
		ts->prv_mt_pos[id][CY_YPOS] = t->cur_mt_pos[id][CY_YPOS];
	}
	/* signal the view motion event */
	input_sync(ts->input);
	return;
no_track_id:

	/* set temporary track array elements to voids */
	memset(t->tmp_trk, CY_IGNR_TCH, sizeof(t->tmp_trk));
	memset(t->snd_trk, CY_IGNR_TCH, sizeof(t->snd_trk));

	/* get what is currently active */
	for (i = id = 0; id < CY_NUM_TRK_ID && i < CY_NUM_MT_TCH_ID; id++) {
		if (t->cur_trk[id] == CY_TCH) {
			/* only incr counter if track found */
			t->tmp_trk[i] = id;
			i++;
		}
	}
	DBG(printk(KERN_INFO"%s: T1: t0=%d, t1=%d, t2=%d, t3=%d\n", __func__,
					t->tmp_trk[0], t->tmp_trk[1],
					t->tmp_trk[2], t->tmp_trk[3]);)
	DBG(printk(KERN_INFO"%s: T1: p0=%d, p1=%d, p2=%d, p3=%d\n", __func__,
					ts->prv_mt_tch[0], ts->prv_mt_tch[1],
					ts->prv_mt_tch[2], ts->prv_mt_tch[3]);)

	/* pack in still active previous touches */
	for (id = t->prv_tch = 0; id < CY_NUM_MT_TCH_ID; id++) {
		if (t->tmp_trk[id] >= CY_NUM_TRK_ID)
			continue;

		if (cyttsp_inlist(ts->prv_mt_tch, t->tmp_trk[id], &loc,
							CY_NUM_MT_TCH_ID)) {
			loc %= CY_NUM_MT_TCH_ID;
			t->snd_trk[loc] = t->tmp_trk[id];
			t->prv_tch++;
			DBG(printk(KERN_INFO"%s: in list s[%d]=%d "
					"t[%d]=%d, loc=%d p=%d\n", __func__,
					loc, t->snd_trk[loc],
					id, t->tmp_trk[id],
					loc, t->prv_tch);)
		} else {
			DBG(printk(KERN_INFO"%s: is not in list s[%d]=%d"
					" t[%d]=%d loc=%d\n", __func__,
					id, t->snd_trk[id],
					id, t->tmp_trk[id],
					loc);)
		}
	}
	DBG(printk(KERN_INFO"%s: S1: s0=%d, s1=%d, s2=%d, s3=%d p=%d\n",
		   __func__,
		   t->snd_trk[0], t->snd_trk[1], t->snd_trk[2],
		   t->snd_trk[3], t->prv_tch);)

	/* pack in new touches */
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		if (t->tmp_trk[id] >= CY_NUM_TRK_ID)
			continue;

		if (!cyttsp_inlist(t->snd_trk, t->tmp_trk[id], &loc,
							CY_NUM_MT_TCH_ID)) {

			DBG(
			printk(KERN_INFO"%s: not in list t[%d]=%d, loc=%d\n",
				   __func__,
				   id, t->tmp_trk[id], loc);)

			if (cyttsp_next_avail_inlist(t->snd_trk, &loc,
							CY_NUM_MT_TCH_ID)) {
				loc %= CY_NUM_MT_TCH_ID;
				t->snd_trk[loc] = t->tmp_trk[id];
				DBG(printk(KERN_INFO "%s: put in list s[%d]=%d"
					" t[%d]=%d\n", __func__,
					loc,
					t->snd_trk[loc], id, t->tmp_trk[id]);
				    )
			}
		} else {
			DBG(printk(KERN_INFO"%s: is in list s[%d]=%d "
				"t[%d]=%d loc=%d\n", __func__,
				id, t->snd_trk[id], id, t->tmp_trk[id], loc);)
		}
	}
	DBG(printk(KERN_INFO"%s: S2: s0=%d, s1=%d, s2=%d, s3=%d\n", __func__,
			t->snd_trk[0], t->snd_trk[1],
			t->snd_trk[2], t->snd_trk[3]);)

	/* sync motion event signals for each current touch */
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		/* z will either be 0 (NOTOUCH) or
		 * some pressure (TOUCH)
		 */
		DBG(printk(KERN_INFO "%s: MT0 prev[%d]=%d "
				"temp[%d]=%d send[%d]=%d\n",
				__func__, id, ts->prv_mt_tch[id],
				id, t->tmp_trk[id], id, t->snd_trk[id]);)

		if (t->snd_trk[id] < CY_NUM_TRK_ID) {
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
					t->cur_mt_z[t->snd_trk[id]]);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
					t->tool_width);
			input_report_abs(ts->input, ABS_MT_POSITION_X,
					t->cur_mt_pos[t->snd_trk[id]][CY_XPOS]);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,
					t->cur_mt_pos[t->snd_trk[id]][CY_YPOS]);

			if (mt_sync_func)
				mt_sync_func(ts->input);
			DBG2(printk(KERN_INFO"%s: MT1 -> TID:"
				"%3d X:%3d  Y:%3d  Z:%3d\n", __func__,
				t->snd_trk[id],
				t->cur_mt_pos[t->snd_trk[id]][CY_XPOS],
				t->cur_mt_pos[t->snd_trk[id]][CY_YPOS],
				t->cur_mt_z[t->snd_trk[id]]);)

		} else if (ts->prv_mt_tch[id] < CY_NUM_TRK_ID) {
			/* void out this touch */
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
							CY_NTCH);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
							t->tool_width);
			input_report_abs(ts->input, ABS_MT_POSITION_X,
				ts->prv_mt_pos[ts->prv_mt_tch[id]][CY_XPOS]);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,
				ts->prv_mt_pos[ts->prv_mt_tch[id]][CY_YPOS]);

			if (mt_sync_func)
				mt_sync_func(ts->input);
			DBG2(printk(KERN_INFO"%s: "
				"MT2->TID:%2d X:%3d Y:%3d Z:%3d liftoff-sent\n",
				__func__, ts->prv_mt_tch[id],
				ts->prv_mt_pos[ts->prv_mt_tch[id]][CY_XPOS],
				ts->prv_mt_pos[ts->prv_mt_tch[id]][CY_YPOS],
				CY_NTCH);)
		} else {
			/* do not stuff any signals for this
			 * previously and currently void touches
			 */
			DBG3(printk(KERN_INFO"%s: "
				"MT3->send[%d]=%d - No touch - NOT sent\n",
				__func__, id, t->snd_trk[id]);)
		}
	}
	/* signal the view motion event */
	input_sync(ts->input);


	/* save current posted tracks to
	 * previous track memory */
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		ts->prv_mt_tch[id] = t->snd_trk[id];
		if (t->snd_trk[id] < CY_NUM_TRK_ID) {
			ts->prv_mt_pos[t->snd_trk[id]][CY_XPOS] =
					t->cur_mt_pos[t->snd_trk[id]][CY_XPOS];
			ts->prv_mt_pos[t->snd_trk[id]][CY_YPOS] =
					t->cur_mt_pos[t->snd_trk[id]][CY_YPOS];
			DBG(printk(KERN_INFO"%s: "
				"MT4->TID:%2d X:%3d Y:%3d Z:%3d save for prv\n",
				__func__, t->snd_trk[id],
				ts->prv_mt_pos[t->snd_trk[id]][CY_XPOS],
				ts->prv_mt_pos[t->snd_trk[id]][CY_YPOS],
				CY_NTCH);)
		}
	}
	memset(ts->act_trk, CY_NTCH, sizeof(ts->act_trk));
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		if (t->snd_trk[id] < CY_NUM_TRK_ID)
			ts->act_trk[t->snd_trk[id]] = CY_TCH;
	}
}


void touchscreen_work_f(struct work_struct *work)
{
    struct cyttsp *ts = container_of(work, struct cyttsp, work);

    u8  ix = 0;
    u8  retval;
    u8  sa;
    u8  touches = 0;
    struct cyttsp_xydata_gen4_hp   finger_data;
    static u8 tmp_buf[MAX_REG_ADDR+1];

    DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

    /* get event data from CYTTSP device */
    /* added shift hack since missing hst_mode register data */
    retval = ttsp_read_block_data(ts,
        CY_REG_BASE,
        3,
        tmp_buf); 
    if (retval < 0) {
        printk(KERN_ERR "%s: Error, "
            "fail to read operational registers\n",
            __func__);
        goto exit_touch_worker;
    }


    touches = tmp_buf[2]&0xF;
    if(touches >  MAX_SUPPORT_FINGERS)
        goto exit_touch_worker;
    
    DBG(printk(KERN_INFO "fingers = %d: \n", touches);)

    retval = ttsp_read_block_data(ts,
        CY_REG_BASE,
        (touches == MAX_SUPPORT_FINGERS)? (MAX_REG_ADDR+1) : fingerXH_addr[touches],
        tmp_buf);
    if (retval < 0) {
        printk(KERN_ERR "%s: Error, "
            "fail to read operational registers\n",
            __func__);
        goto exit_touch_worker;
    }



    for(ix = 0; ix < touches; ix++)
    {
        sa = fingerXH_addr[ix];
        finger_data.fingers_reg[ix].x = tmp_buf[sa]*256 + tmp_buf[sa+1];
        finger_data.fingers_reg[ix].y = tmp_buf[sa+2]*256 + tmp_buf[sa+3];
        finger_data.fingers_reg[ix].z = tmp_buf[sa+4];
        sa = fingerIndex_addr[ix];
        finger_data.fingers_reg[ix].fingerID = (ix%2 == 0)?((tmp_buf[sa]>>4)&0xF):(tmp_buf[sa]&0xF);

        // check data validity
        if((finger_data.fingers_reg[ix].x > ts->platform_data->maxx) || (finger_data.fingers_reg[ix].y > ts->platform_data->maxy) ||
            finger_data.fingers_reg[ix].z > 50 || finger_data.fingers_reg[ix].fingerID == 0xF)
        {
            printk(KERN_INFO " finger%d ( x=%d y=%d z=%d id=%d )\n", ix,finger_data.fingers_reg[ix].x,
                finger_data.fingers_reg[ix].y,finger_data.fingers_reg[ix].z,finger_data.fingers_reg[ix].fingerID);
            goto  exit_touch_worker;
        }
    }

    for (ix = 0; ix < touches; ix++) {
        input_report_abs(ts->input, ABS_MT_TRACKING_ID, finger_data.fingers_reg[ix].fingerID);
        input_report_abs(ts->input, ABS_MT_POSITION_X, finger_data.fingers_reg[ix].x);
        input_report_abs(ts->input, ABS_MT_POSITION_Y, finger_data.fingers_reg[ix].y);
        input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 255);
        input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, finger_data.fingers_reg[ix].z);
        input_mt_sync(ts->input);
        DBG(printk(KERN_INFO " finger%d ( x=%d y=%d z=%d id=%d )\n", ix,finger_data.fingers_reg[ix].x,
                 finger_data.fingers_reg[ix].y,finger_data.fingers_reg[ix].z,finger_data.fingers_reg[ix].fingerID);)

    }

    if(ts->prev_touches > touches){
        input_mt_sync(ts->input);
    }

    ts->prev_touches = touches;
    input_sync(ts->input);    

exit_touch_worker:
     DBG(printk(KERN_INFO"%s: finished.\n", __func__);)
     enable_irq(ts->irq);
    return;

    
}


#if 0
void cyttsp_xy_worker(struct work_struct *work)
{
	struct cyttsp *ts = container_of(work, struct cyttsp, work);
	struct cyttsp_xydata_gen4 xy_data;
	u8 tmp_buf[sizeof(xy_data)];
	u8 id, tilt, rev_x, rev_y;
	struct cyttsp_track_data trc;
	s32 retval;

	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)
	/* get event data from CYTTSP device */
	/* added shift hack since missing hst_mode register data */
	retval = ttsp_read_block_data(ts,
		CY_REG_BASE,
		(ts->platform_data->gen == CY_GEN4) ?
		sizeof(xy_data) : sizeof(xy_data.regs),
		&tmp_buf);
		/* Hacks */
		memcpy((void *)((u8 *)(&xy_data) + 1),
			(const void *)tmp_buf,
			sizeof(xy_data) - 1);
		xy_data.regs.hst_mode = 0;
		
	if (retval < 0) {
		printk(KERN_ERR "%s: Error, "
			"fail to read operational registers\n",
			__func__);
		goto exit_xy_worker;
	}

	/* touch extension handling */
	retval = ttsp_tch_ext(ts, &xy_data);

	if (retval < 0) {
		printk(KERN_ERR "%s: Error, touch extension handling\n",
			__func__);
		goto exit_xy_worker;
	} else if (retval > 0) {
		DBG(printk(KERN_INFO "%s: Touch extension handled\n",
			__func__);)
		goto exit_xy_worker;
	}

	/* provide flow control handshake */
	if (ts->irq) {
		if (ts->platform_data->use_hndshk) {
			u8 cmd;
			cmd = xy_data.regs.hst_mode & CY_HNDSHK_BIT ?
				xy_data.regs.hst_mode & ~CY_HNDSHK_BIT :
				xy_data.regs.hst_mode | CY_HNDSHK_BIT;
			retval = ttsp_write_block_data(ts, CY_REG_BASE,
						       sizeof(cmd), (u8 *)&cmd);
		}
	}
	trc.cur_tch = GET_NUM_TOUCHES(xy_data.regs.tt_stat);
	if (GET_BOOTLOADERMODE(xy_data.regs.tt_mode)) {
		/* TTSP device has reset back to bootloader mode */
		/* reset driver touch history */
		bool timeout;
		DBG3(printk(KERN_INFO
			"%s: Bootloader detected; reset driver\n",
			__func__);)
		cyttsp_init_tch(ts);
		free_irq(ts->irq, ts);
		retval = request_irq(ts->irq, cyttsp_bl_ready_irq,
			IRQF_TRIGGER_FALLING, ts->platform_data->name, ts);
		cyttsp_soft_reset(ts, &timeout);
		cyttsp_exit_bl_mode(ts);
		cyttsp_set_operational_mode(ts);
		free_irq(ts->irq, ts);
		retval = request_irq(ts->irq, cyttsp_irq, IRQF_TRIGGER_FALLING,
			ts->platform_data->name, ts);
		DBG3(printk(KERN_INFO
			"%s: Bootloader detected; exit worker\n",
			__func__);)
		goto exit_xy_worker;
	} else if (IS_LARGE_AREA(xy_data.regs.tt_stat) == 1) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		DBG2(printk(KERN_INFO "%s: Large area detected\n",
			__func__);)
	} else if ((ts->platform_data->gen == CY_GEN4) &&
		(trc.cur_tch > CY_NUM_MT_10TCH)) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		DBG2(printk(KERN_INFO "%s: Num touch error detected (n=%d)\n",
			__func__, trc.cur_tch);)
	} else if (!(ts->platform_data->gen == CY_GEN4) &&
		(trc.cur_tch > CY_NUM_MT_4TCH)) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		DBG2(printk(KERN_INFO "%s: Num touch error detected (n=%d)\n",
			__func__, trc.cur_tch);)	 
	} else if (IS_BAD_PKT(xy_data.regs.tt_mode)) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		DBG2(printk(KERN_INFO "%s: Invalid buffer detected\n",
			__func__);)
	}

	/* set tool size */
	trc.tool_width = CY_SMALL_TOOL_WIDTH;

	if (ts->platform_data->gen == CY_GEN2) {
		/* translate Gen2 interface data into comparable Gen3 data */
		trc.cur_tch = ttsp_convert_gen2(trc.cur_tch, &(xy_data.regs));
	}

	/* clear current active track ID array and count previous touches */
	for (id = 0, trc.prv_tch = CY_NTCH; id < CY_NUM_TRK_ID; id++) {
		trc.cur_trk[id] = CY_NTCH;
		trc.prv_tch += ts->act_trk[id];
	}

	/* send no events if there were no previous touches */
	/* and no new touches */
	if ((trc.prv_tch == CY_NTCH) && ((trc.cur_tch == CY_NTCH) ||
				(trc.cur_tch > CY_NUM_MT_TCH_ID)))
		goto exit_xy_worker;

	DBG(printk(KERN_INFO "%s: prev=%d  curr=%d\n", __func__,
		   trc.prv_tch, trc.cur_tch);)

	/* clear current multi-touch arrays */
	memset(trc.cur_mt_tch, CY_IGNR_TCH, sizeof(trc.cur_mt_tch));
	memset(trc.cur_mt_pos, CY_NTCH, sizeof(trc.cur_mt_pos));
	memset(trc.cur_mt_z, CY_NTCH, sizeof(trc.cur_mt_z));

	DBG4(
		if (trc.cur_tch) {
			unsigned i;
			unsigned count;
			u8 *pdata = (u8 *)&xy_data;
			printk(KERN_INFO "%s: TTSP data_pack: ", __func__);
			count = (ts->platform_data->gen == CY_GEN4) ?
				sizeof(xy_data) : sizeof(xy_data.regs);
			for (i = 0; i < count; i++)
				printk(KERN_INFO "[%d]=0x%x ", i, pdata[i]);
			printk(KERN_INFO "\n");
		}
	)

	/* Determine if display is tilted */
	tilt = !!FLIP_DATA(ts->platform_data->flags);
	/* Check for switch in origin */
	rev_x = !!REVERSE_X(ts->platform_data->flags);
	rev_y = !!REVERSE_Y(ts->platform_data->flags);

	/* process the touches */
	switch (trc.cur_tch) {
	case 10:
		xy_data.xregs.tch10.x = be16_to_cpu(xy_data.xregs.tch10.x);
		xy_data.xregs.tch10.y = be16_to_cpu(xy_data.xregs.tch10.y);
		if (tilt)
			FLIP_XY(xy_data.xregs.tch10.x, xy_data.xregs.tch10.y);

		if (rev_x)
			xy_data.xregs.tch10.x = INVERT_X(xy_data.xregs.tch10.x,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.xregs.tch10.y = INVERT_X(xy_data.xregs.tch10.y,
					ts->platform_data->maxy);

		id = GET_TOUCH10_ID(xy_data.xregs.touch910_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH10_IDX][CY_XPOS] =
				xy_data.xregs.tch10.x;
			trc.cur_mt_pos[CY_MT_TCH10_IDX][CY_YPOS] =
				xy_data.xregs.tch10.y;
			trc.cur_mt_z[CY_MT_TCH10_IDX] = xy_data.xregs.tch10.z ?
				xy_data.xregs.tch10.z : CY_DFLT_Z;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.xregs.tch10.x;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.xregs.tch10.y;
			trc.cur_mt_z[id] = xy_data.xregs.tch10.z ?
				xy_data.xregs.tch10.z : CY_DFLT_Z;
		}
		trc.cur_mt_tch[CY_MT_TCH10_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		DBG3(printk(KERN_INFO"%s: 10th XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.xregs.tch10.x,
				xy_data.xregs.tch10.y, xy_data.xregs.tch10.z,
				GET_TOUCH10_ID(xy_data.xregs.touch910_id));)

		/* do not break */
	case 9:
		xy_data.xregs.tch9.x = be16_to_cpu(xy_data.xregs.tch9.x);
		xy_data.xregs.tch9.y = be16_to_cpu(xy_data.xregs.tch9.y);
		if (tilt)
			FLIP_XY(xy_data.xregs.tch9.x, xy_data.xregs.tch9.y);

		if (rev_x)
			xy_data.xregs.tch9.x = INVERT_X(xy_data.xregs.tch9.x,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.xregs.tch9.y = INVERT_X(xy_data.xregs.tch9.y,
					ts->platform_data->maxy);

		id = GET_TOUCH9_ID(xy_data.xregs.touch910_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH9_IDX][CY_XPOS] =
				xy_data.xregs.tch9.x;
			trc.cur_mt_pos[CY_MT_TCH9_IDX][CY_YPOS] =
				xy_data.xregs.tch9.y;
			trc.cur_mt_z[CY_MT_TCH9_IDX] = xy_data.xregs.tch9.z ?
				xy_data.xregs.tch9.z : CY_DFLT_Z;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.xregs.tch9.x;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.xregs.tch9.y;
			trc.cur_mt_z[id] = xy_data.xregs.tch9.z ?
				xy_data.xregs.tch9.z : CY_DFLT_Z;
		}
		trc.cur_mt_tch[CY_MT_TCH9_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		DBG3(printk(KERN_INFO"%s: 9th XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.xregs.tch9.x,
				xy_data.xregs.tch9.y, xy_data.xregs.tch9.z,
				GET_TOUCH9_ID(xy_data.xregs.touch910_id));)

		/* do not break */
	case 8:
		xy_data.xregs.tch8.x = be16_to_cpu(xy_data.xregs.tch8.x);
		xy_data.xregs.tch8.y = be16_to_cpu(xy_data.xregs.tch8.y);
		if (tilt)
			FLIP_XY(xy_data.xregs.tch8.x, xy_data.xregs.tch8.y);

		if (rev_x)
			xy_data.xregs.tch8.x = INVERT_X(xy_data.xregs.tch8.x,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.xregs.tch8.y = INVERT_X(xy_data.xregs.tch8.y,
					ts->platform_data->maxy);

		id = GET_TOUCH8_ID(xy_data.xregs.touch78_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH8_IDX][CY_XPOS] =
				xy_data.xregs.tch8.x;
			trc.cur_mt_pos[CY_MT_TCH8_IDX][CY_YPOS] =
				xy_data.xregs.tch8.y;
			trc.cur_mt_z[CY_MT_TCH8_IDX] = xy_data.xregs.tch8.z ?
				xy_data.xregs.tch8.z : CY_DFLT_Z;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.xregs.tch8.x;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.xregs.tch8.y;
			trc.cur_mt_z[id] = xy_data.xregs.tch8.z ?
				xy_data.xregs.tch8.z : CY_DFLT_Z;
		}
		trc.cur_mt_tch[CY_MT_TCH8_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		DBG3(printk(KERN_INFO"%s: 8th XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.xregs.tch8.x,
				xy_data.xregs.tch8.y, xy_data.xregs.tch8.z,
				GET_TOUCH8_ID(xy_data.xregs.touch78_id));)

		/* do not break */
	case 7:
		xy_data.xregs.tch7.x = be16_to_cpu(xy_data.xregs.tch7.x);
		xy_data.xregs.tch7.y = be16_to_cpu(xy_data.xregs.tch7.y);
		if (tilt)
			FLIP_XY(xy_data.xregs.tch7.x, xy_data.xregs.tch7.y);

		if (rev_x)
			xy_data.xregs.tch7.x = INVERT_X(xy_data.xregs.tch7.x,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.xregs.tch7.y = INVERT_X(xy_data.xregs.tch7.y,
					ts->platform_data->maxy);

		id = GET_TOUCH7_ID(xy_data.xregs.touch78_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH7_IDX][CY_XPOS] =
				xy_data.xregs.tch7.x;
			trc.cur_mt_pos[CY_MT_TCH7_IDX][CY_YPOS] =	
				xy_data.xregs.tch7.y;
			trc.cur_mt_z[CY_MT_TCH7_IDX] = xy_data.xregs.tch7.z ?
				xy_data.xregs.tch7.z : CY_DFLT_Z;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.xregs.tch7.x;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.xregs.tch7.y;
			trc.cur_mt_z[id] = xy_data.xregs.tch7.z ?
				xy_data.xregs.tch7.z : CY_DFLT_Z;
		}
		trc.cur_mt_tch[CY_MT_TCH7_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		DBG3(printk(KERN_INFO"%s: 7th XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.xregs.tch7.x,
				xy_data.xregs.tch7.y, xy_data.xregs.tch7.z,
				GET_TOUCH7_ID(xy_data.xregs.touch78_id));)

		/* do not break */
	case 6:
		xy_data.xregs.tch6.x = be16_to_cpu(xy_data.xregs.tch6.x);
		xy_data.xregs.tch6.y = be16_to_cpu(xy_data.xregs.tch6.y);
		if (tilt)
			FLIP_XY(xy_data.xregs.tch6.x, xy_data.xregs.tch6.y);

		if (rev_x)
			xy_data.xregs.tch6.x = INVERT_X(xy_data.xregs.tch6.x,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.xregs.tch6.y = INVERT_X(xy_data.xregs.tch6.y,
					ts->platform_data->maxy);

		id = GET_TOUCH6_ID(xy_data.xregs.touch56_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH6_IDX][CY_XPOS] =
				xy_data.xregs.tch6.x;
			trc.cur_mt_pos[CY_MT_TCH6_IDX][CY_YPOS] =
				xy_data.xregs.tch6.y;
			trc.cur_mt_z[CY_MT_TCH6_IDX] = xy_data.xregs.tch6.z ?
				xy_data.xregs.tch6.z : CY_DFLT_Z;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.xregs.tch6.x;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.xregs.tch6.y;
			trc.cur_mt_z[id] = xy_data.xregs.tch6.z ?
				xy_data.xregs.tch6.z : CY_DFLT_Z;
		}
		trc.cur_mt_tch[CY_MT_TCH6_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		DBG3(printk(KERN_INFO"%s: 6th XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.xregs.tch6.x,
				xy_data.xregs.tch6.y, xy_data.xregs.tch6.z,
				GET_TOUCH6_ID(xy_data.xregs.touch56_id));)

		/* do not break */
	case 5:
		xy_data.xregs.tch5.x = be16_to_cpu(xy_data.xregs.tch5.x);
		xy_data.xregs.tch5.y = be16_to_cpu(xy_data.xregs.tch5.y);
		if (tilt)
			FLIP_XY(xy_data.xregs.tch5.x, xy_data.xregs.tch5.y);

		if (rev_x)
			xy_data.xregs.tch5.x = INVERT_X(xy_data.xregs.tch5.x,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.xregs.tch5.y = INVERT_X(xy_data.xregs.tch5.y,
					ts->platform_data->maxy);

		id = GET_TOUCH5_ID(xy_data.xregs.touch56_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH5_IDX][CY_XPOS] =
				xy_data.xregs.tch5.x;
			trc.cur_mt_pos[CY_MT_TCH5_IDX][CY_YPOS] =
				xy_data.xregs.tch5.y;
			trc.cur_mt_z[CY_MT_TCH5_IDX] = xy_data.xregs.tch5.z ?
				xy_data.xregs.tch5.z : CY_DFLT_Z;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.xregs.tch5.x;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.xregs.tch5.y;
			trc.cur_mt_z[id] = xy_data.xregs.tch5.z ?
				xy_data.xregs.tch5.z : CY_DFLT_Z;
		}
		trc.cur_mt_tch[CY_MT_TCH5_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		DBG3(printk(KERN_INFO"%s: 5th XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.xregs.tch5.x,
				xy_data.xregs.tch5.y, xy_data.xregs.tch5.z,
				GET_TOUCH5_ID(xy_data.xregs.touch56_id));)

		/* do not break */
	case 4:
		xy_data.regs.tch4.x = be16_to_cpu(xy_data.regs.tch4.x);
		xy_data.regs.tch4.y = be16_to_cpu(xy_data.regs.tch4.y);
		if (tilt)
			FLIP_XY(xy_data.regs.tch4.x, xy_data.regs.tch4.y);

		if (rev_x)
			xy_data.regs.tch4.x = INVERT_X(xy_data.regs.tch4.x,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.regs.tch4.y = INVERT_X(xy_data.regs.tch4.y,
					ts->platform_data->maxy);

		id = GET_TOUCH4_ID(xy_data.regs.touch34_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH4_IDX][CY_XPOS] =
				xy_data.regs.tch4.x;
			trc.cur_mt_pos[CY_MT_TCH4_IDX][CY_YPOS] =
				xy_data.regs.tch4.y;
			trc.cur_mt_z[CY_MT_TCH4_IDX] = xy_data.regs.tch4.z ?
				xy_data.regs.tch4.z : CY_DFLT_Z;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.regs.tch4.x;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.regs.tch4.y;
			trc.cur_mt_z[id] = xy_data.regs.tch4.z ?
				xy_data.regs.tch4.z : CY_DFLT_Z;
		}
		trc.cur_mt_tch[CY_MT_TCH4_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		DBG3(printk(KERN_INFO"%s: 4th XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.regs.tch4.x,
				xy_data.regs.tch4.y, xy_data.regs.tch4.z,
				GET_TOUCH4_ID(xy_data.regs.touch34_id));)

		/* do not break */
	case 3:
		xy_data.regs.tch3.x = be16_to_cpu(xy_data.regs.tch3.x);
		xy_data.regs.tch3.y = be16_to_cpu(xy_data.regs.tch3.y);
		if (tilt)
			FLIP_XY(xy_data.regs.tch3.x, xy_data.regs.tch3.y);

		if (rev_x)
			xy_data.regs.tch3.x = INVERT_X(xy_data.regs.tch3.x,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.regs.tch3.y = INVERT_X(xy_data.regs.tch3.y,
					ts->platform_data->maxy);

		id = GET_TOUCH3_ID(xy_data.regs.touch34_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH3_IDX][CY_XPOS] =
				xy_data.regs.tch3.x;
			trc.cur_mt_pos[CY_MT_TCH3_IDX][CY_YPOS] =
				xy_data.regs.tch3.y;
			trc.cur_mt_z[CY_MT_TCH3_IDX] = xy_data.regs.tch3.z ?
				xy_data.regs.tch3.z : CY_DFLT_Z;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.regs.tch3.x;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.regs.tch3.y;
			trc.cur_mt_z[id] = xy_data.regs.tch3.z ?
				xy_data.regs.tch3.z : CY_DFLT_Z;
		}
		trc.cur_mt_tch[CY_MT_TCH3_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		DBG3(printk(KERN_INFO"%s: 3rd XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
			__func__, xy_data.regs.tch3.x,
			xy_data.regs.tch3.y, xy_data.regs.tch3.z,
			GET_TOUCH3_ID(xy_data.regs.touch34_id));)

		/* do not break */
	case 2:
		xy_data.regs.tch2.x = be16_to_cpu(xy_data.regs.tch2.x);
		xy_data.regs.tch2.y = be16_to_cpu(xy_data.regs.tch2.y);
		if (tilt)
			FLIP_XY(xy_data.regs.tch2.x, xy_data.regs.tch2.y);

		if (rev_x)
			xy_data.regs.tch2.x = INVERT_X(xy_data.regs.tch2.x,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.regs.tch2.y = INVERT_X(xy_data.regs.tch2.y,
					ts->platform_data->maxy);
		id = GET_TOUCH2_ID(xy_data.regs.touch12_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH2_IDX][CY_XPOS] =
				xy_data.regs.tch2.x;
			trc.cur_mt_pos[CY_MT_TCH2_IDX][CY_YPOS] =
				xy_data.regs.tch2.y;
			trc.cur_mt_z[CY_MT_TCH2_IDX] = xy_data.regs.tch2.z ?
				xy_data.regs.tch2.z : CY_DFLT_Z;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.regs.tch2.x;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.regs.tch2.y;
			trc.cur_mt_z[id] = xy_data.regs.tch2.z ?
				xy_data.regs.tch2.z : CY_DFLT_Z;
		}
		trc.cur_mt_tch[CY_MT_TCH2_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		DBG3(printk(KERN_INFO"%s: 2nd XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.regs.tch2.x,
				xy_data.regs.tch2.y, xy_data.regs.tch2.z,
				GET_TOUCH2_ID(xy_data.regs.touch12_id));)

		/* do not break */
	case 1:
		xy_data.regs.tch1.x = be16_to_cpu(xy_data.regs.tch1.x);
		xy_data.regs.tch1.y = be16_to_cpu(xy_data.regs.tch1.y);
		if (tilt)
			FLIP_XY(xy_data.regs.tch1.x, xy_data.regs.tch1.y);

		if (rev_x)
			xy_data.regs.tch1.x = INVERT_X(xy_data.regs.tch1.x,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.regs.tch1.y = INVERT_X(xy_data.regs.tch1.y,
					ts->platform_data->maxy);

		id = GET_TOUCH1_ID(xy_data.regs.touch12_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH1_IDX][CY_XPOS] =
				xy_data.regs.tch1.x;
			trc.cur_mt_pos[CY_MT_TCH1_IDX][CY_YPOS] =
				xy_data.regs.tch1.y;
			trc.cur_mt_z[CY_MT_TCH1_IDX] = xy_data.regs.tch1.z ?
				xy_data.regs.tch1.z : CY_DFLT_Z;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.regs.tch1.x;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.regs.tch1.y;
			trc.cur_mt_z[id] = xy_data.regs.tch1.z ?
				xy_data.regs.tch1.z : CY_DFLT_Z;
		}
		trc.cur_mt_tch[CY_MT_TCH1_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		DBG3(printk(KERN_INFO"%s: 1st XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.regs.tch1.x,
				xy_data.regs.tch1.y, xy_data.regs.tch1.z,
				GET_TOUCH1_ID(xy_data.regs.touch12_id));)

		break;
	case 0:
	default:
		break;
	}

	if (ts->platform_data->use_mt)
		handle_multi_touch(&trc, ts);

	DBG(printk(KERN_INFO"%s: \n", __func__);)

	/* handle gestures */
	if (ts->platform_data->use_gestures && xy_data.regs.gest_id) {
		input_report_key(ts->input, BTN_3, CY_TCH);
		input_report_abs(ts->input, ABS_HAT1X, xy_data.regs.gest_id);
		input_report_abs(ts->input, ABS_HAT1Y, xy_data.regs.gest_cnt);
		/* signal the view motion event */
		input_sync(ts->input);
	}

	/* update platform data for the current multi-touch information */
	memcpy(ts->act_trk, trc.cur_trk, CY_NUM_TRK_ID);

exit_xy_worker:
	DBG(printk(KERN_INFO"%s: finished.\n", __func__);)
	return;
}
#endif


/* ************************************************************************
 * Probe initialization functions
 * ************************************************************************ */
	
static int cyttsp_check_polling(struct cyttsp *ts)
{
	return (ts->platform_data->use_timer);
}

/* Timeout timer */
static void cyttsp_to_timer(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	DBG(printk(KERN_INFO"%s: TTSP timeout timer event!\n", __func__);)
	ts->to_timeout = true;
	return;
}

static void cyttsp_setup_to_timer(struct cyttsp *ts)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	setup_timer(&ts->to_timer, cyttsp_to_timer, (unsigned long) ts);
}

static void cyttsp_kill_to_timer(struct cyttsp *ts)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	del_timer(&ts->to_timer);
}

static void cyttsp_start_to_timer(struct cyttsp *ts, int ms)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	ts->to_timeout = false;
	mod_timer(&ts->to_timer, jiffies + ms);
}

static bool cyttsp_timeout(struct cyttsp *ts)
{
	if (cyttsp_check_polling(ts))
		return false;
	else
		return ts->to_timeout;
}

static irqreturn_t cyttsp_bl_ready_irq(int irq, void *handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	DBG(printk(KERN_INFO"%s: Got BL IRQ!\n", __func__);)
	ts->bl_ready = true;
	return IRQ_HANDLED;
}

static void cyttsp_set_bl_ready(struct cyttsp *ts, bool set)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	ts->bl_ready = set;
	DBG(printk(KERN_INFO"%s: bl_ready=%d\n", __func__, (int)ts->bl_ready);)
}

static bool cyttsp_check_bl_ready(struct cyttsp *ts)
{
	if (cyttsp_check_polling(ts))
		return true;
	else
		return ts->bl_ready;
}

static int cyttsp_load_bl_regs(struct cyttsp *ts)
{
	int retval;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	retval =  ttsp_read_block_data(ts, CY_REG_BASE,
				sizeof(ts->bl_data), &(ts->bl_data));

	if (retval < 0) {
		DBG(printk(KERN_INFO "%s: Failed reading block data, err:%d\n",
			__func__, retval);)
		goto fail;
	}

	DBG({
	      int i;
	      u8 *bl_data = (u8 *)&(ts->bl_data);
	      for (i = 0; i < sizeof(struct cyttsp_bootloader_data); i++)
			printk(KERN_INFO "%s bl_data[%d]=0x%x\n",
				__func__, i, bl_data[i]);
	})

	return 0;
fail:
	return retval;
}

static bool cyttsp_bl_status(struct cyttsp *ts)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	return ((ts->bl_data.bl_status == 0x10) ||
		(ts->bl_data.bl_status == 0x11));
}

static bool cyttsp_bl_err_status(struct cyttsp *ts)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	return (((ts->bl_data.bl_status == 0x10) &&
		(ts->bl_data.bl_error == 0x20)) ||
		((ts->bl_data.bl_status == 0x11) &&
		(ts->bl_data.bl_error == 0x20)));
}
			
static bool cyttsp_wait_bl_ready(struct cyttsp *ts,
	int pre_delay, int loop_delay, int max_try,
	bool (*done)(struct cyttsp *ts))
{
	int tries;
	bool rdy = false, tmo = false;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	DBG(printk(KERN_INFO"%s: pre-dly=%d loop-dly=%d, max-try=%d\n",
		__func__, pre_delay, loop_delay, max_try);)

	tries = 0;
	ts->bl_data.bl_file = 0;
	ts->bl_data.bl_status = 0;
	ts->bl_data.bl_error = 0;
	if (cyttsp_check_polling(ts)) {
		msleep(pre_delay);
		do {
			msleep(abs(loop_delay));
			cyttsp_load_bl_regs(ts);
		} while (!done(ts) &&
			tries++ < max_try);
		DBG(printk(KERN_INFO"%s: polling mode tries=%d\n",
			__func__, tries);)
	}
	else {
		cyttsp_start_to_timer(ts, abs(loop_delay) * max_try);
		while (!rdy && !tmo) {
			rdy = cyttsp_check_bl_ready(ts);
			tmo = cyttsp_timeout(ts);
			if (loop_delay < 0)
				udelay(abs(loop_delay));
			else
				msleep(abs(loop_delay));
			tries++;
		}
		DBG(printk(KERN_INFO"%s: irq mode tries=%d rdy=%d tmo=%d\n",
			__func__, tries, (int)rdy, (int)tmo);)
		cyttsp_load_bl_regs(ts);
	}
	
	if (tries >= max_try || tmo)
		return true;	/* timeout */
	else
		return false;
}

static int cyttsp_exit_bl_mode(struct cyttsp *ts)
{
	int retval;
	int tries = 0;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(bl_cmd),
				       (void *)bl_cmd);
	if (retval < 0) {
		printk(KERN_ERR "%s: Failed writing block data, err:%d\n",
			__func__, retval);
		goto fail;
	}
	do {
		msleep(500);
		cyttsp_load_bl_regs(ts);
	} while (GET_BOOTLOADERMODE(ts->bl_data.bl_status) && tries++ < 10);
	return 0;
fail:
	return retval;
}

static int cyttsp_set_sysinfo_mode(struct cyttsp *ts)
{
	int retval;
	u8 cmd = CY_SYSINFO_MODE;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	
	memset(&(ts->sysinfo_data), 0, sizeof(struct cyttsp_sysinfo_data));

	/* switch to sysinfo mode */
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	if (retval < 0) {
		printk(KERN_ERR "%s: Failed writing block data, err:%d\n",
			__func__, retval);
		goto write_block_data_fail;
	}
	msleep(500);	/* wait long enough for sysinfo handshake timeout */
	
	/* read sysinfo registers */
	retval = ttsp_read_block_data(ts, CY_REG_BASE, sizeof(ts->sysinfo_data),
			&(ts->sysinfo_data));
	if (retval < 0) {
		printk(KERN_ERR "%s: Failed reading block data, err:%d\n",
			__func__, retval);
		return retval;
	}

write_block_data_fail:
	DBG(printk(KERN_INFO"%s:SI2: hst_mode=0x%02X mfg_cmd=0x%02X "
		"mfg_stat=0x%02X\n", __func__, ts->sysinfo_data.hst_mode,
		ts->sysinfo_data.mfg_cmd,
		ts->sysinfo_data.mfg_stat);)

	DBG(printk(KERN_INFO"%s:SI2: bl_ver=0x%02X%02X\n",
		__func__, ts->sysinfo_data.bl_verh, ts->sysinfo_data.bl_verl);)

	DBG(printk(KERN_INFO"%s:SI2: sysinfo act_intrvl=0x%02X "
		"tch_tmout=0x%02X lp_intrvl=0x%02X\n",
		__func__, ts->sysinfo_data.act_intrvl,
		ts->sysinfo_data.tch_tmout,
		ts->sysinfo_data.lp_intrvl);)

	printk(KERN_INFO"%s:SI2:tts_ver=0x%02X%02X app_id=0x%02X%02X "
		"app_ver=0x%02X%02X c_id=0x%02X%02X%02X\n", __func__,
		ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
		ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
		ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
		ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1],
		ts->sysinfo_data.cid[2]);
	return 0;

	return retval;
}

static int cyttsp_set_operational_mode(struct cyttsp *ts)
{
	int retval;
	u8 cmd = CY_OPERATE_MODE;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	if (retval < 0) {
		printk(KERN_ERR "%s: Failed writing block data, err:%d\n",
			__func__, retval);
		goto write_block_data_fail;
	}
	/* wait for TTSP Device to complete switch to Operational mode */
	msleep(500);
	return 0;
write_block_data_fail:
	return retval;
}

static int cyttsp_set_gesture_mode(struct cyttsp *ts)
{
	int retval;
	u8 cmd = ts->platform_data->gest_set;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	retval = ttsp_write_block_data(ts, CY_REG_GEST_SET, sizeof(cmd), &cmd);
	if (retval < 0) {
		printk(KERN_ERR "%s: Failed writing block data, err:%d\n",
			__func__, retval);
		goto write_block_data_fail;
	}
	/* wait for TTSP Device to complete gesture group
	 * and active distance setups
	 */
	msleep(CY_DELAY_DFLT);
	return 0;
write_block_data_fail:
	return retval;
}

static int cyttsp_soft_reset(struct cyttsp *ts, bool *status)
{
	int retval;
	u8 cmd = CY_SOFT_RESET_MODE;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	/* reset TTSP Device back to bootloader mode */
	cyttsp_set_bl_ready(ts, false);
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	/* wait for TTSP Device to complete reset back to bootloader */
	if (!retval)
		*status = cyttsp_wait_bl_ready(ts, 300, 10, 100,
			cyttsp_bl_status);

	return retval;
}

#ifdef CY_INCLUDE_LOAD_RECS
#include "cyttsp_ldr.h"
#else
static int cyttsp_loader(struct cyttsp *ts)
{
	void *fptr = cyttsp_bl_err_status;	/* kill warning */

	if (ts) {
		DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
		DBG(printk(KERN_INFO"%s: Exit\n", __func__);)
	}
	
	if (!fptr)
		return -EIO;
	else
		return 0;
}
#endif

static int cyttsp_power_on(struct cyttsp *ts)
{
	int retval = 0;
	bool timeout;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	/* check if the TTSP device has a bootloader installed */
	retval = cyttsp_soft_reset(ts, &timeout);
	DBG(printk(KERN_INFO"%s: after softreset r=%d\n", __func__, retval);)
	if (retval < 0 || timeout)
		goto bypass;

	if (ts->platform_data->use_load_file)
		retval = cyttsp_loader(ts);

	retval = cyttsp_exit_bl_mode(ts);

	if (retval < 0)
		goto bypass;

	/* switch to System Information mode to read */
	/* versions and set interval registers */
	retval = cyttsp_set_sysinfo_mode(ts);
	if (retval < 0)
		goto bypass;
	if ((CY_DIFF(ts->platform_data->act_intrvl, CY_ACT_INTRVL_DFLT) ||
		CY_DIFF(ts->platform_data->tch_tmout, CY_TCH_TMOUT_DFLT) ||
		CY_DIFF(ts->platform_data->lp_intrvl, CY_LP_INTRVL_DFLT))) {

		u8 intrvl_ray[3];

		intrvl_ray[0] = ts->platform_data->act_intrvl;
		intrvl_ray[1] = ts->platform_data->tch_tmout;
		intrvl_ray[2] = ts->platform_data->lp_intrvl;

		DBG(printk(KERN_INFO"%s: act_intrvl=0x%02X"
			"tch_tmout=0x%02X lp_intrvl=0x%02X\n",
			__func__, ts->platform_data->act_intrvl,
			ts->platform_data->tch_tmout,
			ts->platform_data->lp_intrvl);)

		/* set intrvl registers */
		retval = ttsp_write_block_data(ts,
				CY_REG_ACT_INTRVL,
				sizeof(intrvl_ray), intrvl_ray);

		msleep(CY_DELAY_SYSINFO);
	}
	if (retval < 0)
		goto bypass;
		
	/* switch back to Operational mode */
	DBG(printk(KERN_INFO"%s: switch back to operational mode\n",
		__func__);)
	retval = cyttsp_set_operational_mode(ts);
	if (retval < 0)
		goto bypass;
	
	/* init gesture setup and active distance */
	DBG(printk(KERN_INFO"%s: "
		"Init gesture and active distance setup=0x%02X\n",
		__func__, ts->platform_data->gest_set);)
	retval = cyttsp_set_gesture_mode(ts);
	
bypass:
	if (retval < 0)
		ts->platform_data->power_state = CY_IDLE_STATE;
	else
		ts->platform_data->power_state = CY_ACTIVE_STATE;

	DBG(printk(KERN_INFO"%s: Power state is %s\n",
			__func__, (ts->platform_data->power_state ==
			CY_ACTIVE_STATE) ? "ACTIVE" : "IDLE");)
	return retval;
}

static int cyttsp_resume(struct cyttsp *ts)
{
	int retval = 0;
	u8  tmp_data;

    retval = ttsp_read_block_data(ts, CY_REG_BASE, 1, &tmp_data);
    if (retval < 0) {
       mdelay(150);
    }
    // Clear suspend flag
    ts->isSuspend = 0;

	return retval;
}

static int cyttsp_suspend(struct cyttsp *ts)
{
    int retval = 0;

	u8  sleep_mode = CY_DEEP_SLEEP_MODE;

    // Set Suspend Flag
    ts->isSuspend = 1;
    
	  retval = ttsp_write_block_data(ts,
		CY_REG_BASE, sizeof(sleep_mode), &sleep_mode);    
    if(retval < 0)
    {
        pr_err("error: cypress can not enter into deep mode %d\n",retval);
        return retval;
    }
        	
	return retval;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp_ts_early_suspend(struct early_suspend *h)
{
	struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	LOCK(ts->mutex);
	if (!ts->fw_loader_mode) {
		if (ts->platform_data->use_timer)
			del_timer(&ts->timer);
		else
			disable_irq_nosync(ts->irq);
		ts->suspended = 1;
		cancel_work_sync(&ts->work);
		cyttsp_suspend(ts);
	}
	UNLOCK(ts->mutex);
}

static void cyttsp_ts_late_resume(struct early_suspend *h)
{
	struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	LOCK(ts->mutex);
	if (!ts->fw_loader_mode && ts->suspended) {
		ts->suspended = 0;
		if (cyttsp_resume(ts) < 0)
			printk(KERN_ERR "%s: Error, cyttsp_resume.\n",
				__func__);
		if (ts->platform_data->use_timer)
			mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
		else
			enable_irq(ts->irq);
	}
	UNLOCK(ts->mutex);
}
#endif


#if  0

static int cyttsp_wr_reg(struct cyttsp *ts, u8 reg_id, u8 reg_data)
{

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	return ttsp_write_block_data(ts,
		CY_REG_BASE + reg_id, sizeof(u8), &reg_data);
}

static int cyttsp_rd_reg(struct cyttsp *ts, u8 reg_id, u8 *reg_data)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	return ttsp_read_block_data(ts,
		CY_REG_BASE + reg_id, sizeof(u8), reg_data);
}

static ssize_t firmware_write(struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t pos, size_t size)
{
	unsigned short val;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp *ts = dev_get_drvdata(dev);
	LOCK(ts->mutex);
	if (!ts->fw_loader_mode) {
		val = *(unsigned short *)buf;
		ts->reg_id = (val & 0xff00) >> 8;
		if (!(ts->reg_id & 0x80)) {		
			/* write user specified operational register */
			cyttsp_wr_reg(ts, ts->reg_id, (u8)(val & 0xff));
			DBG2(printk(KERN_INFO "%s: write(r=0x%02X d=0x%02X)\n",
				__func__, ts->reg_id, (val & 0xff));)
		} else {
			/* save user specified operational read register */
			DBG2(printk(KERN_INFO "%s: read(r=0x%02X)\n",
				__func__, ts->reg_id);)
		}
	} else {
		DBG({
			char str[128];
			char *p = str;
			int i;
			for (i = 0; i < size; i++, p += 2)
				sprintf(p, "%02x", (unsigned char)buf[i]);
			printk(KERN_DEBUG "%s: size %d, pos %ld payload %s\n",
			       __func__, size, (long)pos, str);
		})
		ttsp_write_block_data(ts, CY_REG_BASE, size, buf);
	}
	UNLOCK(ts->mutex);
	return size;
}


static ssize_t firmware_read(struct kobject *kobj,
	struct bin_attribute *ba,
	char *buf, loff_t pos, size_t size)
{
	int count = 0;
	u8 reg_data;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp *ts = dev_get_drvdata(dev);

	LOCK(ts->mutex);
	if (!ts->fw_loader_mode) {
		/* read user specified operational register */
		cyttsp_rd_reg(ts, ts->reg_id & ~0x80, &reg_data);
		*(unsigned short *)buf = reg_data << 8;
		count = sizeof(unsigned short);
		DBG2(printk(KERN_INFO "%s: read(d=0x%02X)\n",
			__func__, reg_data);)
	} else if (!cyttsp_load_bl_regs(ts)) {
		*(unsigned short *)buf = ts->bl_data.bl_status << 8 |
			ts->bl_data.bl_error;
		count = sizeof(unsigned short);
	} else {
		printk(KERN_ERR "%s: error reading status\n", __func__);
		count = 0;
	}
	UNLOCK(ts->mutex);
	return count;
}
#endif
static struct bin_attribute cyttsp_firmware = {
	.attr = {
		.name = "firmware",
		.mode = 0644,
	},
	.size = 128,
	.read = NULL,//firmware_read,
	.write = NULL,//firmware_write,
};

static ssize_t attr_fwloader_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	return sprintf(buf, "0x%02X%02X 0x%02X%02X 0x%02X%02X 0x%02X%02X%02X\n",
		ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
		ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
		ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
		ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1],
		ts->sysinfo_data.cid[2]);
}

static ssize_t attr_fwloader_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	char *p;
	int ret;
	bool timeout;
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned val = simple_strtoul(buf, &p, 10);

	ret = p - buf;
	if (*p && isspace(*p))
		ret++;
	//printk(KERN_DEBUG "%s: %u\n", __func__, val);

	LOCK(ts->mutex)
	if (val == 3) {
		sysfs_remove_bin_file(&dev->kobj, &cyttsp_firmware);
		DBG2(printk(KERN_INFO "%s: FW loader closed for reg r/w\n",
			__func__);)
	} else if (val == 2) {
		if (sysfs_create_bin_file(&dev->kobj, &cyttsp_firmware))
			printk(KERN_ERR "%s: unable to create file\n",
				__func__);
		DBG2(printk(KERN_INFO "%s: FW loader opened for reg r/w\n",
			__func__);)
	} else if ((val == 1) && !ts->fw_loader_mode) {
		ts->fw_loader_mode = 1;
		if (ts->suspended) {
			cyttsp_resume(ts);
		} else {
			if (ts->platform_data->use_timer)
				del_timer(&ts->timer);
			else
				disable_irq_nosync(ts->irq);
			cancel_work_sync(&ts->work);
		}
		ts->suspended = 0;
		if (sysfs_create_bin_file(&dev->kobj, &cyttsp_firmware))
			printk(KERN_ERR "%s: unable to create file\n",
				__func__);
		cyttsp_soft_reset(ts, &timeout);
		printk(KERN_INFO "%s: FW loader started.\n", __func__);
	} else if (!val && ts->fw_loader_mode) {
		sysfs_remove_bin_file(&dev->kobj, &cyttsp_firmware);
		cyttsp_soft_reset(ts, &timeout);
		cyttsp_exit_bl_mode(ts);
		cyttsp_set_sysinfo_mode(ts);
		cyttsp_set_operational_mode(ts);

		if (ts->platform_data->use_timer)
			mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
		else
			enable_irq(ts->irq);
		ts->fw_loader_mode = 0;
		printk(KERN_INFO "%s: FW loader finished.\n", __func__);
	}
	UNLOCK(ts->mutex);
	return  ret == size ? ret : -EINVAL;
}

static struct device_attribute fwloader =
	__ATTR(fwloader, 0644, attr_fwloader_show, attr_fwloader_store);

static ssize_t interrupt_enable_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t size)
{
    int rc;
    unsigned long is_enable;    
    struct cyttsp *ts = dev_get_drvdata(dev);


    rc = strict_strtoul(buf, 10, &is_enable);
    if (rc)
        return -EINVAL;
    printk("%s : is_enable = %d, size = %d \n", __func__, (unsigned int)is_enable, size);

    if(is_enable && ts->isInterrupt_Enabled)
        return size;


    if(is_enable && !(ts->isInterrupt_Enabled))
    {
        rc = request_irq(ts->irq, cyttsp_irq, IRQF_TRIGGER_RISING,
            ts->platform_data->name, ts);
        
        if (rc) {
            printk(KERN_ERR "%s: Error, could not request irq\n",
                __func__);
            return 0;
        }

        
        ts->isInterrupt_Enabled = 1;
    }
    
    if(!is_enable && ts->isInterrupt_Enabled)
    {
        free_irq(ts->irq, ts);
        ts->isInterrupt_Enabled = 0;
    }
    
    return size;
}



static DEVICE_ATTR(interrupt_enable, S_IRUGO | S_IWUSR, NULL, interrupt_enable_store);


void *cyttsp_core_init(struct cyttsp_bus_ops *bus_ops, struct device *pdev)
{
	struct input_dev *input_device;
	struct cyttsp *ts;
	int retval = 0;
    u8  tmp_data;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "%s: Error, kzalloc\n", __func__);
		goto error_alloc_data_failed;
	}
	mutex_init(&ts->mutex);
	ts->pdev = pdev;
	ts->platform_data = pdev->platform_data;
	ts->bus_ops = bus_ops;
    ts->prev_touches = 0;

	if (ts->platform_data->init)
	{
        retval = ts->platform_data->init(1);
    	if (retval) {
    		printk(KERN_ERR "%s: platform init failed! \n", __func__);
    		goto error_init;
    	}
	}

        retval = ttsp_read_block_data(ts, CY_REG_BASE, 1, &tmp_data);
        if (retval < 0) {
            printk(KERN_ERR "%s: ttsp_read_block_data failed! \n", __func__);
            goto error_init;
        }

	if (ts->platform_data->use_timer)
		ts->irq = -1;
	else
	{
        if(ts->platform_data->irq_gpio)
        {
		    ts->irq = gpio_to_irq(ts->platform_data->irq_gpio());
        }
        else
        {
            printk(KERN_ERR "%s:  not set irq gpio! \n", __func__);
            goto error_init;
        }
	}

	/* Create the input device and register it. */
	input_device = input_allocate_device();
	if (!input_device) {
		retval = -ENOMEM;
		printk(KERN_ERR "%s: Error, failed to allocate input device\n",
			__func__);
		goto error_input_allocate_device;
	}

	ts->input = input_device;
	input_device->name = ts->platform_data->name;
	input_device->phys = ts->phys;
	input_device->dev.parent = ts->pdev;

if(0)	
{
	if (!cyttsp_check_polling(ts)) {
		DBG(
		printk(KERN_INFO
			"%s: Setting up BL Ready Interrupt. Device name=%s\n",
			__func__, input_device->name);)
		retval = request_irq(ts->irq, cyttsp_bl_ready_irq,
			IRQF_TRIGGER_FALLING, input_device->name, ts);

		if (retval) {
			printk(KERN_ERR
				"%s: Error, could not request bl ready irq\n",
				__func__);
			goto error_input_allocate_device;
		} else {
			cyttsp_setup_to_timer(ts);
			DBG(printk(KERN_INFO "%s: BL Ready Interrupt=%d\n",
				__func__, ts->irq);)
		}
	}
	retval = cyttsp_power_on(ts);
	if (!cyttsp_check_polling(ts)) {
		cyttsp_kill_to_timer(ts);
		free_irq(ts->irq, ts);
	}
	if (retval < 0) {
		printk(KERN_ERR "%s: Error, power on failed! \n", __func__);
		goto error_power_on;
	}
}
	cyttsp_init_tch(ts);

	set_bit(EV_SYN, input_device->evbit);
	set_bit(EV_KEY, input_device->evbit);
	set_bit(EV_ABS, input_device->evbit);
	set_bit(BTN_TOUCH, input_device->keybit);
	set_bit(BTN_2, input_device->keybit);
	if (ts->platform_data->use_gestures)
		set_bit(BTN_3, input_device->keybit);

	input_set_abs_params(input_device, ABS_X, 0, ts->platform_data->maxx,
			     0, 0);
	input_set_abs_params(input_device, ABS_Y, 0, ts->platform_data->maxy,
			     0, 0);
	input_set_abs_params(input_device, ABS_TOOL_WIDTH, 0,
			     CY_LARGE_TOOL_WIDTH, 0, 0);
	input_set_abs_params(input_device, ABS_PRESSURE, 0, CY_MAXZ, 0, 0);
	input_set_abs_params(input_device, ABS_HAT0X, 0,
			     ts->platform_data->maxx, 0, 0);
	input_set_abs_params(input_device, ABS_HAT0Y, 0,
			     ts->platform_data->maxy, 0, 0);
	if (ts->platform_data->use_gestures) {
		input_set_abs_params(input_device, ABS_HAT1X, 0, CY_MAXZ,
				     0, 0);
		input_set_abs_params(input_device, ABS_HAT1Y, 0, CY_MAXZ,
				     0, 0);
	}
	if (ts->platform_data->use_mt) {
		input_set_abs_params(input_device, ABS_MT_POSITION_X, 0,
				     ts->platform_data->maxx, 0, 0);
		input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0,
				     ts->platform_data->maxy, 0, 0);
		input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 0,
				     CY_MAXZ, 0, 0);
		input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR, 0,
				     CY_LARGE_TOOL_WIDTH, 0, 0);
		if (ts->platform_data->use_trk_id)
			input_set_abs_params(input_device, ABS_MT_TRACKING_ID,
					0, CY_NUM_TRK_ID, 0, 0);
	}

	if (ts->platform_data->use_virtual_keys)
		input_set_capability(input_device, EV_KEY, KEY_PROG1);

	retval = input_register_device(input_device);
	if (retval) {
		printk(KERN_ERR "%s: Error, failed to register input device\n",
			__func__);
		goto error_input_register_device;
	}
	DBG(printk(KERN_INFO "%s: Registered input device %s\n",
		   __func__, input_device->name);)

	/* Prepare our worker structure prior to setting up the timer/ISR */
	INIT_WORK(&ts->work, touchscreen_work_f);
    ts->irq_workqueue = create_workqueue("irq_wq");

    // Set default value for suspend flag
    ts->isSuspend = 0;

	/* Timer or Interrupt setup */
	if (ts->platform_data->use_timer) {
		DBG(printk(KERN_INFO "%s: Setting up Timer\n", __func__);)
		setup_timer(&ts->timer, cyttsp_timer, (unsigned long) ts);
		mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
	} else {
		DBG(
		printk(KERN_INFO "%s: Setting up Interrupt. Device name=%s\n",
			__func__, input_device->name);)
		retval = request_irq(ts->irq, cyttsp_irq, IRQF_TRIGGER_RISING,
			input_device->name, ts);

		if (retval) {
			printk(KERN_ERR "%s: Error, could not request irq\n",
				__func__);
			goto error_free_irq;
		} else {
			DBG(printk(KERN_INFO "%s: Interrupt=%d\n",
				__func__, ts->irq);)
		}
        // default set Interrupt enable
        ts->isInterrupt_Enabled = 1;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = cyttsp_ts_early_suspend;
	ts->early_suspend.resume = cyttsp_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	retval = device_create_file(pdev, &fwloader);
	if (retval) {
		printk(KERN_ERR "%s: Error, could not create attribute\n",
			__func__);
		goto device_create_error;
	}
	dev_set_drvdata(pdev, ts);
    
    retval = device_create_file(pdev, &dev_attr_interrupt_enable);
    if (retval) {
        dev_err(pdev, "failed to create the dev_attr_interrupt_enable\n");
        goto device_create_error;
    }

	printk(KERN_INFO "%s: Successful.\n", __func__);
	return ts;

device_create_error:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
error_free_irq:
	if (ts->irq >= 0)
		free_irq(ts->irq, ts);
	input_unregister_device(input_device);
error_input_register_device:
error_power_on:
	cyttsp_kill_to_timer(ts);
	input_free_device(input_device);
error_input_allocate_device:
	if (ts->platform_data->init)
		ts->platform_data->init(0);
error_init:
	kfree(ts);
error_alloc_data_failed:
	return NULL;
}

/* registered in driver struct */
void cyttsp_core_release(void *handle)
{
	struct cyttsp *ts = handle;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	cancel_work_sync(&ts->work);
	if (ts->platform_data->use_timer)
		del_timer_sync(&ts->timer);
	else
		free_irq(ts->irq, ts);
	input_unregister_device(ts->input);
	input_free_device(ts->input);
	if (ts->platform_data->init)
		ts->platform_data->init(0);
	kfree(ts);
}
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver core");
MODULE_AUTHOR("Cypress");

