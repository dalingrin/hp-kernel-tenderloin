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
 *
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/mfd/msm-adie-codec.h>
#include <linux/mfd/timpani-audio.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/spinlock.h>
#include <linux/mfd/wm8994/core.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/registers.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mfd/wm8994/debugfs.h>
#include <linux/workqueue.h>
#include <linux/mfd/wm8994/wm8958_topaz_profile.h>
#include <mach/board.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>


static struct work_struct reg_map_work;
static struct work_struct adb_work;
static u8 * adb_buf;
static u32 adb_flag = 0;

/* wm8994 codec driver is activated through wm8994 core driver */

static int seq_lineout_path_index;
static int seq_lineout_acoustic_index;
static int seq_headphone_path_index;
static int seq_headphone_acoustic_index;
//static int seq_tx_path_index;
//static int seq_tx_acoustic_index;
//static int wm8958_sysclk_en;
static int current_sequence_num;

struct adie_codec_path {
	struct adie_codec_dev_profile *profile;
	struct adie_codec_register_image img;
	u32 hwsetting_idx;
	u32 stage_idx;
	u32 curr_stage;
	u32 reg_owner;
};

struct adie_codec_state {
	struct adie_codec_path path[ADIE_CODEC_MAX];
	u32 ref_cnt;
	struct wm8994 *codec_pdata;
	struct mutex lock;
};

static struct adie_codec_state adie_codec;

static int adie_codec_write(u16 reg,  u16 val)
{
//	printk(KERN_ERR "adie_code_write reg=%x, val=%x\n",reg, val);

	wm8994_reg_write(adie_codec.codec_pdata,reg, val);

	return 0;
}

static int adie_codec_read(u16 reg)
{

 return wm8994_reg_read(adie_codec.codec_pdata,reg);

}

struct wm8958_seq_item wm8958_topaz_sequence[]={
/*line out parameters  index from 0x3b to 0x50*/	
	{SEQ_LINEOUT_HEAD,0x3b,0x620,0x0,0x1,0x0}, //  0	620H Oversample low power
	{SEQ_DATA,0x3c,0x601,0x1,0x0,0x0}, //  1	601H enable AIF1R to DAC1R
	{SEQ_DATA,0x3d,0x602,0x1,0x0,0x0}, //  2	602H enable AIF1L to DAC1L
	{SEQ_DATA,0x3e,0x450,0x0,0x7,0x0}, //  3	450H  AIF1 DRC2 Signal Detect Mode=0  Peak threshold mode
	{SEQ_DATA,0x3f,0x440,0x0,0x7,0x0}, //  4	440H  AIF1 DRC1 Signal Detect Mode=0  Peak threshold mode
	{SEQ_DATA,0x40,0x420,0x0,0x9,0x0}, //  5	420H AIF1DAC1 input path (AIF1, Timeslot 0) Soft Mute Control = 0 unmute
	{SEQ_DATA,0x41,0xff,0x0,0x0,0x0},// 6
	{SEQ_DATA,0x42,0x37, 0xc,0x304,0},  		// 7	 TBD
	{SEQ_DATA,0x43,0x35,  0x1,0,0x0},     // 8	35H MIXOUTR_TO_LINEOUT2N  MIXOUTR to Differential Output on LINEOUT2  un-mute
	{SEQ_DATA,0x44,0x34,  0x1,0,0x0},     // 9	35H MIXOUTR_TO_LINEOUT1N  MIXOUTR to Differential Output on LINEOUT1  un-mute
	{SEQ_DATA,0x45,0x2d,  0x1,0x0,0x0},      // 10	2DH DAC1L TO MIXOUTL
	{SEQ_DATA,0x46,0x2e,  0x1,0x0,0x0},	    // 11	2EH  DAC1R TO MIXOUTR	
//	{SEQ_DATA,0x47,0x21,  0x1f,0x404,0x0},     // 12	Mixer output PGA update
//	{SEQ_DATA,0x48,0x20,  0x1f,0x404,0x0},	   // 13	Mixer output PGA update	
	{SEQ_DATA,0x49,0x5,    0x3,0x100,0x0}, // 14	Left DAC1 and Right DAC1 enable
	{SEQ_DATA,0x4a,0x5,    0x3,0x108,0x0}, // 15	Enable AIF1DAC1 L&R input path
	{SEQ_DATA,0x4b,0x610, 0x0,0x9,0x0},   //16	unmute DAC1L
	{SEQ_DATA,0x4c,0x611, 0x0,0x9,0x0},   //17	unmute DAC1R
	{SEQ_DATA,0x4d,0x300, 0x0,0x6,0x0},   //18	AIF1 Word Length = 16-bits
	{SEQ_DATA,0x4e,0x610, 0xa0,0x700,0x0},   //16	 decrease 14 steps * 0.3db
	{SEQ_DATA,0x4f,0x611, 0xa0,0x700,0x0},   //17	decrease 14 steps * 0.3db
	{SEQ_DATA,0x50,0x3, 0xf,0x304,0x100},//22 MIXOUTL L&R Volume Control Enable, Mixer output L&R enable 

/*headphone index from 0x55 t0 0x68 */
	{SEQ_HEADPHONE_HEAD,0x55,0x601,0x1,0      ,0}, //23
	{SEQ_DATA,0x56,0xff,0x0,0x0,0},//27
	{SEQ_DATA,0x57,0x51 ,0x1 ,0       ,0},//28
	{SEQ_DATA,0x58,0xff  ,0,0,0},		//29
//	{SEQ_DATA,0x59,0x1c  ,0x6a,0x700,0},		//30
//	{SEQ_DATA,0x5a,0x1d  ,0x6a,0x700,0},		//31
	{SEQ_DATA,0x5b,0x5  ,0x3  ,0x300, 0},//32
	{SEQ_DATA,0x5c,0x5  ,0x3  ,0x308, 0},//33
	{SEQ_DATA,0x5d,0x2d,0x1  ,0       ,0},//34
	{SEQ_DATA,0x5e,0x2e,0x1  ,0       ,0},//35
	{SEQ_DATA,0x5f,0x3  ,0x3  ,0x304,0},//36
	{SEQ_DATA,0x60,0xff  ,0,0,0},		//37
//	{SEQ_DATA,0x61,0x60,0xee,0x700,0},//38   TBD it causes pop noise
	{SEQ_DATA,0x62,0x610,0    ,0x108,0},//39
	{SEQ_DATA,0x63,0x611,0    ,0x108,0},//40
	{SEQ_DATA,0x64,0x602,0x1,0      ,0},//24
	{SEQ_DATA,0x65,0xff, 0,0,0},//25    keep 48KHz
	{SEQ_DATA,0x66,0x300,0   , 6      ,0},//26
	{SEQ_DATA,0x67,0xff  ,0,0,0},	
	{SEQ_DATA,0x68,0x420,0    ,0x108,0x100}//41
	
};

/*sequncer number*/
enum 
{
	LINEOUT_CONFIG_PATH = 1,
	LINEOUT_CONFIG_ACCOUSIC,
	LINEOUT_CONFIG_ACCOUSIC_RESTORE,
	LINEOUT_START,
	LINEOUT_STOP,
	HEADSET_CONFIG_PATH,
	HEADSET_CONFIG_ACCOUSIC,
	HEADSET_CONFIG_ACCOUSIC_RESOTRE,
	HEADSET_START,
	HEADSET_STOP,
	HANDSET_MIC_PATH_START,
	HANDSET_MIC_PATH_STOP,
	HEADSET_MIC_PATH_START,
	HEADSET_MIC_PATH_STOP,
	LINEOUT_HEADSET_PATH_START,
	LINEOUT_HEADSET_PATH_STOP,
	SEQUENCER_NUM = LINEOUT_HEADSET_PATH_STOP,
};

struct adb_seq_info {
	u32 seq;
	u32 seq_size;
};

struct regs_setting {
	u32 reg;
	u32 value;
};

struct regs_setting *lineout_config_path_from_adb = NULL;
struct regs_setting *lineout_config_accousic_from_adb = NULL;
struct regs_setting *lineout_config_accousic_restore_from_adb = NULL;
struct regs_setting *lineout_start_from_adb = NULL;
struct regs_setting *lineout_stop_from_adb = NULL;
struct regs_setting *headset_config_path_from_adb = NULL;
struct regs_setting *headset_config_accousic_from_adb = NULL;
struct regs_setting *headset_config_accousic_restore_from_adb = NULL;
struct regs_setting *headset_start_from_adb = NULL;
struct regs_setting *headset_stop_from_adb = NULL;
struct regs_setting *handset_mic_path_start_from_adb = NULL;
struct regs_setting *handset_mic_path_stop_from_adb = NULL;
struct regs_setting *headset_mic_path_start_from_adb = NULL;
struct regs_setting *headset_mic_path_stop_from_adb = NULL;
struct regs_setting *lineout_headset_path_start_from_adb = NULL;
struct regs_setting *lineout_headset_path_stop_from_adb = NULL;

struct regs_setting lineout_config_path[]= {
	{0x620,0},
	{0x601,0x1},
	{0x602,0x1},
	{0x450,0x18},
	{0x440,0x18},
	{0x420,0},
	{0x37,0xc0},
	{0x35,0x1},
	{0x34,0x1},
	{0x2d,0x1},
	{0x2e,0x1},
//	{0x21,0x1f9},
//	{0x20,0x1f9},
	{0x5,0x303},
	{0x610,0xa0},
	{0x611,0xa0},
	{0x300,0x4010},
	{0x3,0xf0},
	{0xff,0x0}
};
struct regs_setting lineout_config_accousic[]= {
	{0x480,0x32DB}, 
	{0x481,0x59C0},
	{0x482,0x0FB0}, 
	{0x483,0x03FB}, 
	{0x484,0x0140}, 
	{0x485,0x1FA8}, 
	{0x486,0xF055}, 
	{0x487,0x0408}, 
	{0x488,0x0151}, 
	{0x489,0x1DF8}, 
	{0x48A,0xF1F8}, 
	{0x48B,0x040A}, 
	{0x48C,0x07CC}, 
	{0x48D,0x147C}, 
	{0x48E,0xFB5A}, 
	{0x48F,0x040A}, 
	{0x490,0x2CF5}, 
	{0x491,0x0AB1}, 
	{0x492,0x06AC}, 
	{0x493,0x4000},  
	{0x20, 0x1fb},
	{0x21, 0x1fb},
	{0xff, 0x0}
};

struct regs_setting lineout_config_accousic_restore[]= {
	{0x480,0x6318},
	{0x481,0x6300},
	{0x482,0xfca},
	{0x483,0x400},
	{0x484,0xd8},
	{0x485,0x1eb5},
	{0x486,0xf145},
	{0x487,0xb75},
	{0x488,0x1c5},
	{0x489,0x1c58},
	{0x48A,0xf373},
	{0x48B,0xa54},
	{0x48C,0x558},
	{0x48D,0x168e},
	{0x48E,0xf829},
	{0x48F,0x7ad},
	{0x490,0x1103},
	{0x491,0x564},
	{0x492,0x559},
	{0x493,0x4000},
	{0x20, 0x79},
	{0x21, 0x79},
	{0xff, 0x0}
};

struct regs_setting lineout_start[]= {
	{0xff, 0x0}
};
struct regs_setting lineout_stop[]= {
	{0x1, 0x0},
	{0x3, 0x0},
	{0x1e ,0x66},
	{0x1f, 0x20},
	{0x37, 0x0},
	{0x38, 0x30},
	{0x39, 0x180},
	{0x4c, 0x1f25},
	{0x54, 0x0},
	{0x60, 0x0},
	{0x610,0x2c0},
	{0x611,0x2c0},
	{0x21,0x79},
	{0x20,0x79},
	{0xff,0x0}
};
struct regs_setting headset_config_path[]= {
	{0xff, 0x0}
};
struct regs_setting headset_config_accousic[]= {
	{0x1c, 0x16f},
	{0x1d, 0x16f},
	{0xff, 0x0}
};
struct regs_setting headset_config_accousic_restore[]= {
	{0x1c, 0x6d},
	{0x1d, 0x6d},
	{0xff, 0x0}
};
struct regs_setting headset_start[]= {
	{0xff, 0x0}
};
struct regs_setting headset_stop[]= {
	{0xff, 0x0}
};
struct regs_setting handset_mic_path_start[]= {
	{0x3b, 0x1},
	{0x3d, 0x21},
	{0x1 ,0x13},
	{0x2, 0x6240},
	{0x4, 0x2002},
	{0x18, 0x11b},
	{0x1a, 0x10b},
	{0x28, 0x30},
	{0x29, 0x20},
	{0x39, 0x1e8},
	{0x310, 0x10b},
	{0x311, 0x4000},
	{0x620, 0},
	{0x603, 0x18c},
	{0x604, 0x10},
	{0x612, 0xc0},
	{0x701, 0x8001},
	{0x702, 0x8001},
	{0x703, 0x8001},
	{0x704, 0x8001},
	{0xff,0x0}
};
struct regs_setting handset_mic_path_stop[]= {
	{0x3b, 0xd},
	{0x3d, 0x39},
	{0x1 ,0x0},// mic1 bias will not control by headset detection fucntion, so restore path.
	{0x2, 0x6000},
	{0x4, 0x0},
	{0x18, 0x8b},
	{0x1a, 0x8b},
	{0x28, 0x0},
	{0x29, 0x0},
	{0x39, 0x180},
	{0x310, 0x4053},
	{0x311, 0x4000},
	{0x620, 2},
	{0x603, 0x0},
	{0x604, 0x0},
	{0x612, 0x2c0},
	{0x701, 0xa001},
	{0x702, 0xa001},
	{0x703, 0xa001},
	{0x704, 0xa001},
	{0xff,0x0}
};
struct regs_setting headset_mic_path_start[]= {
       {0x310, 0x10b},
	{0x3b, 0x1},
	{0x3e, 0x21},
	//IO CONFIG
	{0x701, 0x8101},
	{0x702, 0x8101},
	{0x703, 0x8101},
	{0x704, 0x8101},
	//ANALOGUE CONFIG
	{0x39, 0x1ea},
	{0x1 ,0x23},
	//ANALOGUE INPUT CONFIG
	{0x2, 0x6280},
	{0x19, 0x10d},
	{0x1b, 0x18b},
	{0x1a, 0x18b},
	{0x28, 0xc0},
	{0x29, 0x180},
  	//PATH CONFIG
	{0x4, 0x3002},
	{0x603, 0x18c},
	{0x604, 0x10},
	//UNMUTES
	{0x612, 0xc0},
	{0xff,0x0}
};
struct regs_setting headset_mic_path_stop[]= {
	{0x310, 0x4053},
	{0x3b, 0xd},
	{0x3e, 0x39},
	//IO CONFIG
	{0x701, 0xa101},
	{0x702, 0xa101},
	{0x703, 0xa101},
	{0x704, 0xa101},
	//ANALOGUE CONFIG
	{0x39, 0x180},
	{0x1 ,0x23}, // mic bias2 will control by headset detection fucntion, so restore path will not change.
	//ANALOGUE INPUT CONFIG
	{0x2, 0x6000},
	{0x19, 0x8b},
	{0x1a, 0x8b},
	{0x28, 0x0},
	{0x29, 0x0},
	//PATH CONFIG
	{0x4, 0x0},
	{0x603, 0x0},
	{0x604, 0x00},
	//UNMUTES
	{0x612, 0x2c0},
	{0xff,0x0}
};


struct regs_setting lineout_headset_path_start[]= {
	{0x620,0},
	{0x601,0x1},
	{0x602,0x1},
	{0x450,0x18},
	{0x440,0x18},
	{0x420,0},
	{0x37,0xc0},
	{0x35,0x1},
	{0x34,0x1},
	{0x2d,0x1},
	{0x2e,0x1},
	{0x21,0x1f9},
	{0x20,0x1f9},
	{0x5,0x303},
	{0x610,0xa0},
	{0x611,0xa0},
	{0x300,0x4010},
	{0x3,0x3cf0},
	{0x38,0x80},
	{0x1c,0x67}, 
	{0x1d,0x67},
	{0x1e,0x0}, // unmute lineout
	{0xff,0x0}
};
struct regs_setting lineout_headset_path_stop[]= {
	{0x1, 0x0},
	{0x3, 0x0},
	{0x1e ,0x66},
	{0x1f, 0x20},
	{0x37, 0x0},
	{0x38, 0x30},
	{0x39, 0x180},
	{0x4c, 0x1f25},
	{0x54, 0x0},
	{0x60, 0x0},
	{0x610,0x2c0},
	{0x611,0x2c0},
	{0x1c,0x6d}, 
	{0x1d,0x6d},
	{0xff,0x0}
};


// OPAL sequnecer acoustic parameters
struct wm8958_seq_item wm8958_opal_sequence[]={
/*line out parameters  index from 0x3b to 0x50*/	
	{SEQ_LINEOUT_HEAD,0x3b,0x620,0x0,0x1,0x0}, //  0	620H Oversample low power
	{SEQ_DATA,0x3c,0x601,0x1,0x0,0x0}, //  1	601H enable AIF1R to DAC1R
	{SEQ_DATA,0x3d,0x602,0x1,0x0,0x0}, //  2	602H enable AIF1L to DAC1L
	{SEQ_DATA,0x3e,0x450,0x0,0x7,0x0}, //  3	450H  AIF1 DRC2 Signal Detect Mode=0  Peak threshold mode
	{SEQ_DATA,0x3f,0x440,0x0,0x7,0x0}, //  4	440H  AIF1 DRC1 Signal Detect Mode=0  Peak threshold mode
	{SEQ_DATA,0x40,0x420,0x0,0x9,0x0}, //  5	420H AIF1DAC1 input path (AIF1, Timeslot 0) Soft Mute Control = 0 unmute
	{SEQ_DATA,0x41,0xff,0x0,0x0,0x0},// 6
	{SEQ_DATA,0x42,0x37, 0xc,0x304,0},  		// 7	 TBD
	{SEQ_DATA,0x43,0x35,  0x1,0,0x0},     // 8	35H MIXOUTR_TO_LINEOUT2N  MIXOUTR to Differential Output on LINEOUT2  un-mute
	{SEQ_DATA,0x44,0x34,  0x1,0,0x0},     // 9	35H MIXOUTR_TO_LINEOUT1N  MIXOUTR to Differential Output on LINEOUT1  un-mute
	{SEQ_DATA,0x45,0x2d,  0x1,0x0,0x0},      // 10	2DH DAC1L TO MIXOUTL
	{SEQ_DATA,0x46,0x2e,  0x1,0x0,0x0},	    // 11	2EH  DAC1R TO MIXOUTR	
//	{SEQ_DATA,0x47,0x21,  0x1f,0x404,0x0},     // 12	Mixer output PGA update
//	{SEQ_DATA,0x48,0x20,  0x1f,0x404,0x0},	   // 13	Mixer output PGA update	
	{SEQ_DATA,0x49,0x5,    0x3,0x100,0x0}, // 14	Left DAC1 and Right DAC1 enable
	{SEQ_DATA,0x4a,0x5,    0x3,0x108,0x0}, // 15	Enable AIF1DAC1 L&R input path
	{SEQ_DATA,0x4b,0x610, 0x0,0x9,0x0},   //16	unmute DAC1L
	{SEQ_DATA,0x4c,0x611, 0x0,0x9,0x0},   //17	unmute DAC1R
	{SEQ_DATA,0x4d,0x300, 0x0,0x6,0x0},   //18	AIF1 Word Length = 16-bits
	{SEQ_DATA,0x4e,0x610, 0xa0,0x700,0x0},   //16	 decrease 14 steps * 0.3db
	{SEQ_DATA,0x4f,0x611, 0xa0,0x700,0x0},   //17	decrease 14 steps * 0.3db
	{SEQ_DATA,0x50,0x3, 0xf,0x304,0x100},//22 MIXOUTL L&R Volume Control Enable, Mixer output L&R enable 

/*headphone index from 0x55 t0 0x68 */
	{SEQ_HEADPHONE_HEAD,0x55,0x601,0x1,0      ,0}, //23
	{SEQ_DATA,0x56,0xff,0x0,0x0,0},//27
	{SEQ_DATA,0x57,0x51 ,0x1 ,0       ,0},//28
	{SEQ_DATA,0x58,0xff  ,0,0,0},		//29
//	{SEQ_DATA,0x59,0x1c  ,0x6a,0x700,0},		//30
//	{SEQ_DATA,0x5a,0x1d  ,0x6a,0x700,0},		//31
	{SEQ_DATA,0x5b,0x5  ,0x3  ,0x300, 0},//32
	{SEQ_DATA,0x5c,0x5  ,0x3  ,0x308, 0},//33
	{SEQ_DATA,0x5d,0x2d,0x1  ,0       ,0},//34
	{SEQ_DATA,0x5e,0x2e,0x1  ,0       ,0},//35
	{SEQ_DATA,0x5f,0x3  ,0x3  ,0x304,0},//36
	{SEQ_DATA,0x60,0xff  ,0,0,0},		//37
//	{SEQ_DATA,0x61,0x60,0xee,0x700,0},//38 TBD it causes pop noise
	{SEQ_DATA,0x62,0x610,0    ,0x108,0},//39
	{SEQ_DATA,0x63,0x611,0    ,0x108,0},//40
	{SEQ_DATA,0x64,0x602,0x1,0      ,0},//24
	{SEQ_DATA,0x65,0xff, 0,0,0},//25    keep 48KHz
	{SEQ_DATA,0x66,0x300,0   , 6      ,0},//26
	{SEQ_DATA,0x67,0xff  ,0,0,0},	
	{SEQ_DATA,0x68,0x420,0    ,0x108,0x100}//41
	
};

struct regs_setting lineout_config_path_opal[]= {
	{0x620,0},
	{0x601,0x1},
	{0x602,0x1},
	{0x450,0x18},
	{0x440,0x18},
	{0x420,0},
	{0x37,0xc0},
	{0x35,0x1},
	{0x34,0x1},
	{0x2d,0x1},
	{0x2e,0x1},
//	{0x21,0x1f9},
//	{0x20,0x1f9},
	{0x5,0x303},
	{0x610,0xa0},
	{0x611,0xa0},
	{0x300,0x4010},
	{0x3,0xf0},
	{0xff,0x0}
};

struct regs_setting lineout_config_accousic_opal[]= {
	{0x480,0x3299}, 
	{0x481,0x6300},
	{0x482,0x0FA4}, 
	{0x483,0x0404}, 
	{0x484,0x0170}, 
	{0x485,0x1EA9}, 
	{0x486,0xF150}, 
	{0x487,0x040A}, 
	{0x488,0x0533}, 
	{0x489,0x17C1}, 
	{0x48A,0xF740}, 
	{0x48B,0x040A}, 
	{0x48C,0x1CB6}, 
	{0x48D,0x106C}, 
	{0x48E,0xFC7F}, 
	{0x48F,0x040A}, 
	{0x490,0x317D}, 
	{0x491,0x0564}, 
	{0x492,0x0559}, 
	{0x493,0x4000},  
	{0X4A0,0x6318},
	{0x20, 0x17d},
	{0x21, 0x17d},
	{0xff, 0x0}
};

struct regs_setting lineout_config_accousic_restore_opal[]= {
	{0x480,0x6318},
	{0x481,0x6300},
	{0x482,0xfca},
	{0x483,0x400},
	{0x484,0xd8},
	{0x485,0x1eb5},
	{0x486,0xf145},
	{0x487,0xb75},
	{0x488,0x1c5},
	{0x489,0x1c58},
	{0x48A,0xf373},
	{0x48B,0xa54},
	{0x48C,0x558},
	{0x48D,0x168e},
	{0x48E,0xf829},
	{0x48F,0x7ad},
	{0x490,0x1103},
	{0x491,0x564},
	{0x492,0x559},
	{0x493,0x4000},
	{0x20, 0x79},
	{0x21, 0x79},
	{0xff, 0x0}
};

struct regs_setting lineout_start_opal[]= {
	{0xff, 0x0}
};
struct regs_setting lineout_stop_opal[]= {
	{0x1, 0x0},
	{0x3, 0x0},
	{0x1e ,0x66},
	{0x1f, 0x20},
	{0x37, 0x0},
	{0x38, 0x30},
	{0x39, 0x180},
	{0x4c, 0x1f25},
	{0x54, 0x0},
	{0x60, 0x0},
	{0x610,0x2c0},
	{0x611,0x2c0},
	{0x21,0x79},
	{0x20,0x79},
	{0xff,0x0}
};
struct regs_setting headset_config_path_opal[]= {
	{0xff, 0x0}
};
struct regs_setting headset_config_accousic_opal[]= {
	{0x1c, 0x167},
	{0x1d, 0x167},
	{0xff, 0x0}
};
struct regs_setting headset_config_accousic_restore_opal[]= {
	{0x1c, 0x6d},
	{0x1d, 0x6d},
	{0xff, 0x0}
};
struct regs_setting headset_start_opal[]= {
	{0xff, 0x0}
};
struct regs_setting headset_stop_opal[]= {
	{0xff, 0x0}
};
struct regs_setting handset_mic_path_start_opal[]= {
	{0x3b, 0x1},
	{0x3d, 0x29},
	{0x1 ,0x13},
	{0x2, 0x6240},
	{0x4, 0x200c},
	{0x18, 0x11b},
	{0x1a, 0x10b},
	{0x28, 0x30},
	{0x29, 0x20},
	{0x39, 0x1e8},
	{0x310, 0x10b},
	{0x311, 0x4000},
	{0x620, 0},
	{0x604, 0x10},
	{0x612, 0xc0},
	{0x701, 0x8001},
	{0x702, 0x8001},
	{0x703, 0x8001},
	{0x704, 0x8001},
	{0x603, 0xcc},
	{0xff,0x0}
};
struct regs_setting handset_mic_path_stop_opal[]= {
	{0x3b, 0xd},
	{0x3d, 0x39},
	{0x1 ,0x0},// mic1 bias will not control by headset detection fucntion, so restore path.
	{0x2, 0x6000},
	{0x4, 0x0},
	{0x18, 0x8b},
	{0x1a, 0x8b},
	{0x28, 0x0},
	{0x29, 0x0},
	{0x39, 0x180},
	{0x310, 0x4053},
	{0x311, 0x4000},
	{0x620, 2},
	{0x604, 0x0},
	{0x612, 0x2c0},
	{0x701, 0xa001},
	{0x702, 0xa001},
	{0x703, 0xa001},
	{0x704, 0xa001},
	{0x603, 0x0},
	{0xff,0x0}
};
struct regs_setting headset_mic_path_start_opal[]= {
       {0x310, 0x10b},
	{0x3b, 0x1},
	{0x3e, 0x2d},
	//IO CONFIG
	{0x701, 0x8101},
	{0x702, 0x8101},
	{0x703, 0x8101},
	{0x704, 0x8101},
	//ANALOGUE CONFIG
	{0x39, 0x1ea},
	{0x1 ,0x23},
	//ANALOGUE INPUT CONFIG
	{0x2, 0x6240},
	{0x18, 0x10d},
	{0x1b, 0x18b},
	{0x1a, 0x18b},
	{0x28, 0x30},
	{0x29, 0x30},
  	//PATH CONFIG
	{0x4, 0x3002},
	{0x603, 0x18c},
	{0x604, 0x10},
	//UNMUTES
	{0x612, 0xc0},
	{0xff,0x0}
};
struct regs_setting headset_mic_path_stop_opal[]= {
	{0x310, 0x4053},
	{0x3b, 0xd},
	{0x3e, 0x39},
	//IO CONFIG
	{0x701, 0xa101},
	{0x702, 0xa101},
	{0x703, 0xa101},
	{0x704, 0xa101},
	//ANALOGUE CONFIG
	{0x39, 0x180},
	{0x1 ,0x23}, // mic bias2 will control by headset detection fucntion, so restore path will not change.
	//ANALOGUE INPUT CONFIG
	{0x2, 0x6000},
	{0x19, 0x8b},
	{0x1a, 0x8b},
	{0x28, 0x0},
	{0x29, 0x0},
	//PATH CONFIG
	{0x4, 0x0},
	{0x603, 0x0},
	{0x604, 0x00},
	//UNMUTES
	{0x612, 0x2c0},
	{0xff,0x0}
};


struct regs_setting lineout_headset_path_start_opal[]= {
	{0x620,0},
	{0x601,0x1},
	{0x602,0x1},
	{0x450,0x18},
	{0x440,0x18},
	{0x420,0},
	{0x37,0xc0},
	{0x35,0x1},
	{0x34,0x1},
	{0x2d,0x1},
	{0x2e,0x1},
	{0x21,0x1f9},
	{0x20,0x1f9},
	{0x5,0x303},
	{0x610,0xa0},
	{0x611,0xa0},
	{0x300,0x4010},
	{0x3,0x3cf0},
	{0x38,0x80},
	{0x1c,0x67}, 
	{0x1d,0x67},
	{0x1e,0x0}, // unmute lineout
	{0xff,0x0}
};
struct regs_setting lineout_headset_path_stop_opal[]= {
	{0x1, 0x0},
	{0x3, 0x0},
	{0x1e ,0x66},
	{0x1f, 0x20},
	{0x37, 0x0},
	{0x38, 0x30},
	{0x39, 0x180},
	{0x4c, 0x1f25},
	{0x54, 0x0},
	{0x60, 0x0},
	{0x610,0x2c0},
	{0x611,0x2c0},
	{0x1c,0x6d}, 
	{0x1d,0x6d},
	{0xff,0x0}
};




struct seq_list {
	struct regs_setting *list;
	u32 seq_num;
};

struct seq_list seql [] = {
// Topaz Sequencer
	{lineout_config_path,LINEOUT_CONFIG_PATH},
	{lineout_config_accousic,LINEOUT_CONFIG_ACCOUSIC},
	{lineout_config_accousic_restore,LINEOUT_CONFIG_ACCOUSIC_RESTORE},
	{lineout_start,LINEOUT_START},
	{lineout_stop,LINEOUT_STOP},
	{headset_config_path,HEADSET_CONFIG_PATH},
	{headset_config_accousic,HEADSET_CONFIG_ACCOUSIC},
	{headset_config_accousic_restore,HEADSET_CONFIG_ACCOUSIC_RESOTRE},
	{headset_start,HEADSET_START},
	{headset_stop,HEADSET_STOP},
	{handset_mic_path_start,HANDSET_MIC_PATH_START},
	{handset_mic_path_stop,HANDSET_MIC_PATH_STOP},
	{headset_mic_path_start,HEADSET_MIC_PATH_START},
	{headset_mic_path_stop,HEADSET_MIC_PATH_STOP},
	{lineout_headset_path_start,LINEOUT_HEADSET_PATH_START},
	{lineout_headset_path_stop,LINEOUT_HEADSET_PATH_STOP},

// OPAL Sequencer
	{lineout_config_path_opal,NONE},
	{lineout_config_accousic_opal,NONE},
	{lineout_config_accousic_restore_opal,NONE},
	{lineout_start_opal,NONE},
	{lineout_stop_opal,NONE},
	{headset_config_path_opal,NONE},
	{headset_config_accousic_opal,NONE},
	{headset_config_accousic_restore_opal,NONE},
	{headset_start_opal,NONE},
	{headset_stop_opal,NONE},
	{handset_mic_path_start_opal,NONE},
	{handset_mic_path_stop_opal,NONE},
	{headset_mic_path_start_opal,NONE},
	{headset_mic_path_stop_opal,NONE},
	{lineout_headset_path_start_opal,NONE},
	{lineout_headset_path_stop_opal,NONE}
};

struct codec_reg_status { 
	u32 reg;
	u32 default_value;
	u32 current_value;
	u32 reg_owner;
};

static DEFINE_SPINLOCK(reg_lock);

static struct codec_reg_status reg_map[] = {

	{0x1,0,0,NONE},{0x2,0x6000,0,NONE},{0x3,0,0,NONE},
	{0x4,0,0,NONE},{0x5,0,0,NONE},{0x18,0x8b,0,NONE},
	{0x19,0x8b,0,NONE},{0x1A,0x8b,0,NONE},{0x1E,0x66,0,NONE},
	{0x1F,0x20,0,NONE},{0x20,0x79,0,NONE},{0x21,0x79,0,NONE},
	{0x28,0,0,NONE},{0x29,0,0,NONE},{0x2D,0,0,NONE},
	{0x2E,0,0,NONE},{0x35,0,0,NONE},{0x37,0,0,NONE},
	{0x38,0,0,NONE},{0x39,0x180,0,NONE},
	{0x3D,0x39,0,NONE},{0x3E,0x39,0,NONE},{0x4C,0x1f25,0,NONE},
	{0x51,0x4,0,NONE},{0x54,0,0,NONE},{0x60,0,0,NONE},
	{0x300,0x4050,0,NONE},{0x1c,0x6d,0,NONE},{0x1d,0x6d,0,NONE},
	{0x310,0x4053,0,NONE},{0x311,0x4000,0,NONE},{0x420,0x200,0,NONE},
	{0x440,0x98,0,NONE},{0x450,0x98,0,NONE},{0x601,0,0,NONE},
	{0x602,0,0,NONE},{0x603,0,0,NONE},{0x604,0,0,NONE},
	{0x610,0,0,NONE},{0x611,0,0,NONE},{0x620,0x2,0,NONE},
	{0x480,0x6318,0,NONE},{0x481,0x6300,0,NONE},{0x482,0xfca,0,NONE},
	{0x483,0x400,0,NONE},{0x484,0xd8,0,NONE},{0x485,0x1eb5,0,NONE},
	{0x486,0xf145,0,NONE},{0x487,0xb75,0,NONE},{0x488,0x1c5,0,NONE},
	{0x489,0x1c58,0,NONE},{0x48A,0xf373,0,NONE},{0x48B,0xa54,0,NONE},
	{0x48C,0x558,0,NONE},{0x48D,0x168e,0,NONE},{0x48E,0xf829,0,NONE},
	{0x48F,0x7ad,0,NONE},{0x490,0x1103,0,NONE},{0x491,0x564,0,NONE},
	{0x492,0x559,0,NONE},{0x493,0x4000,0,NONE}
};	

int find_path_owner(u32 sequncer_num)
{
	int rc = 0;
	switch (sequncer_num){
		case LINEOUT_CONFIG_PATH :
		case LINEOUT_CONFIG_ACCOUSIC:
		case LINEOUT_START:
		case LINEOUT_STOP:
			rc = SPEAKER_RX ;
			break;
		case HEADSET_CONFIG_PATH:
		case HEADSET_CONFIG_ACCOUSIC:
		case HEADSET_START:
		case HEADSET_STOP:
			rc = HEADSET_RX ;
			break;
		case LINEOUT_HEADSET_PATH_START:
		case LINEOUT_HEADSET_PATH_STOP:
			rc = SPEAKER_HEADSET_RX;
			break;
		case HANDSET_MIC_PATH_START:
		case HANDSET_MIC_PATH_STOP:
			rc = HANDSET_TX;
			break;
		case HEADSET_MIC_PATH_START:
		case HEADSET_MIC_PATH_STOP:
			rc= HEADSET_TX;
			break;
	}
	return rc;
}

struct regs_setting * find_adb_seq(int i){
	switch(i) {
	case  LINEOUT_CONFIG_PATH : 
		return lineout_config_path_from_adb;
	case  LINEOUT_CONFIG_ACCOUSIC:
		return lineout_config_accousic_from_adb;
	case  LINEOUT_CONFIG_ACCOUSIC_RESTORE:
		return lineout_config_accousic_restore_from_adb;
	case  LINEOUT_START:
		return lineout_start_from_adb;
	case  LINEOUT_STOP:
		return lineout_stop_from_adb;
	case  HEADSET_CONFIG_PATH:
		return headset_config_path_from_adb;
	case  HEADSET_CONFIG_ACCOUSIC:
		return headset_config_accousic_from_adb;
	case  HEADSET_CONFIG_ACCOUSIC_RESOTRE:
		return headset_config_accousic_restore_from_adb;
	case  HEADSET_START:
		return headset_start_from_adb;
	case  HEADSET_STOP:
		return headset_stop_from_adb;
	case  HANDSET_MIC_PATH_START:
		return handset_mic_path_start_from_adb;
	case  HANDSET_MIC_PATH_STOP:
		return handset_mic_path_stop_from_adb;
	case  HEADSET_MIC_PATH_START:
		return headset_mic_path_start_from_adb;
	case  HEADSET_MIC_PATH_STOP:
		return headset_mic_path_stop_from_adb;
	case  LINEOUT_HEADSET_PATH_START:
		return lineout_headset_path_start_from_adb;
	case  LINEOUT_HEADSET_PATH_STOP:
		return lineout_headset_path_stop_from_adb;
	default:
	break;
	}
	return NULL;
}
	
		
struct regs_setting * find_seq_list(u32 seq_number)
{
 	int i = 0;
//	while(i<ARRAY_SIZE(seql)){
	while(i<SEQUENCER_NUM){
		if(seql[i].seq_num == seq_number)
		break;
		i++;
		}
	if((adb_flag >> seq_number) & 0x1){
		return find_adb_seq(seq_number);
	}else{
		if (wm8994_get_boardtype() == 0) // for topaz
			return seql[i].list;
		else // for opal
			return seql[i+SEQUENCER_NUM].list;
	}
}

static struct regs_setting restorelist[10];
static int restorelist_point;
static int restore_flag;

int run_sequencer(u32 sequncer_num)
{
	int i=0;
	struct regs_setting *run_seq;
	run_seq = find_seq_list(sequncer_num);
	while(1){
		if(run_seq[i].reg==0xff)
		break;
		adie_codec_write(run_seq[i].reg,run_seq[i].value);
		i++;
	}
	return 0;
}

int insert_restorelist(u32 reg, u32 value)
{
	restorelist[restorelist_point].reg=reg;
	restorelist[restorelist_point].value=value;
	restorelist_point++;
	return 0;
}

int run_restorelist(void)
{	
	int i=0;
	while(i<restorelist_point){
		adie_codec_write(restorelist[i].reg,restorelist[i].value);
		i++;
		}
	restorelist_point = 0;
	return 0;
}

int restore_audio_path(u32 path)
{
switch (path) {
	case HEADSET_RX:
		run_sequencer(HEADSET_CONFIG_ACCOUSIC_RESOTRE);
		break;
	case SPEAKER_RX:
		//sequencer will restore lineout setting, current place only restore accoustic parameters. 
		run_sequencer(LINEOUT_CONFIG_ACCOUSIC_RESTORE);
		break;
	case HANDSET_TX :
		run_sequencer(HANDSET_MIC_PATH_STOP);
		break;
	case HEADSET_TX:
		run_sequencer(HEADSET_MIC_PATH_STOP);
		break;
	case SPEAKER_HEADSET_RX:
		run_sequencer(LINEOUT_HEADSET_PATH_STOP);
		break;
	}
return 0;
}

u32 reg_search(u32 reg,u32 map_size){ 
	u32 low=0;
	u32 high=map_size;
	u32 mid;
	while(low<=high){
		mid=(low+high)/2;
		if(reg_map[mid].reg==reg) 
			return mid;
		if(reg_map[mid].reg>reg)
			high=mid-1;
		else
			low=mid+1;
	}
	return 0;
} 

int mark_reg_map(u32 sequncer_num)
{
	int n_regmap;
	int i=0;
	int n=0;
	int isstopseq = 0;
	int path_owner;
	struct regs_setting *curr_seq;
	path_owner = find_path_owner(sequncer_num);
	n_regmap = ARRAY_SIZE(reg_map);
	curr_seq = find_seq_list(sequncer_num);
	if((sequncer_num == LINEOUT_STOP) || (sequncer_num == HEADSET_STOP) || (sequncer_num == HANDSET_MIC_PATH_STOP) || (sequncer_num == HEADSET_MIC_PATH_STOP))
	   isstopseq = 1;
//	pr_err("mark_reg_map  sequncer_num = %d  ,  isstopseq = %d  n_regmap = %d  path_owner = %d \n",sequncer_num, isstopseq,n_regmap,path_owner);
	spin_lock(&reg_lock);
	while(1){
		if(0xff == curr_seq[i].reg)
		break;
		n = reg_search(curr_seq[i].reg,n_regmap);
		if(isstopseq==1){
			if(reg_map[n].reg_owner==path_owner){
				reg_map[n].current_value= reg_map[n].default_value;
				reg_map[n].reg_owner = NONE;
			}else{
				if(sequncer_num==LINEOUT_STOP || sequncer_num== HEADSET_STOP ){
					insert_restorelist(reg_map[n].reg,reg_map[n].current_value);
					restore_flag = 1;
				}
				//other sequncer will restore by itself at the stop sequncer
			}
		}else{
			//config seq will update reg map all the way
			reg_map[n].current_value=curr_seq[i].value;
			reg_map[n].reg_owner = path_owner;
		}
		i++;
		n=0;
	}
	spin_unlock(&reg_lock);
	return 0;
}


static void reg_map_worker( struct work_struct *work){
//	pr_err("reg_map_worker current_sequence_num = %d\n",current_sequence_num);
	mark_reg_map(current_sequence_num);
}

void reg_swap(struct codec_reg_status *a, struct codec_reg_status *b){ 
  struct codec_reg_status t=*a; 
  *a=*b;
  *b=t; 
}
void reg_qsort(struct codec_reg_status arr[],int l,int r)
{

	int i = l;
 	int j = r;
 	u32 key = arr[(i+j)/2].reg;
 	while(i < j)
        {
            while((arr[i].reg< key)&&(i < r))
                i++;
            while((arr[j].reg> key)&&(j > l))
                j--;
            if (i <= j)
               {
                   reg_swap(&arr[i],&arr[j]);
                   i++;
                   j--;
                }
        }
        if (i < r)
            reg_qsort(arr,i,r);
        if (j > l)
            reg_qsort(arr,l,j);

}

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_sdev_dent;
static struct dentry *debugfs_wm8994_i2c;
#endif

static void adb_worker( struct work_struct *work){
	struct adb_seq_info *adb_point;
	adb_point = (struct adb_seq_info *)adb_buf;
	while(adb_point->seq!=0xffff){
//		pr_err(" 0x%x \n",adb_point->seq);
		switch (adb_point->seq) {
			case LINEOUT_CONFIG_PATH :
					lineout_config_path_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<LINEOUT_CONFIG_PATH;
					adb_point+=(adb_point->seq_size+1);
					break;
			case LINEOUT_CONFIG_ACCOUSIC:
					lineout_config_accousic_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<LINEOUT_CONFIG_ACCOUSIC;
					adb_point+=(adb_point->seq_size+1);
					break;
			case LINEOUT_CONFIG_ACCOUSIC_RESTORE:
					lineout_config_accousic_restore_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<LINEOUT_CONFIG_ACCOUSIC_RESTORE;
					adb_point+=(adb_point->seq_size+1);
					break;
			case LINEOUT_START:
					lineout_start_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<LINEOUT_START;
					adb_point+=(adb_point->seq_size+1);
					break;
			case LINEOUT_STOP:
					lineout_stop_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<LINEOUT_STOP;
					adb_point+=(adb_point->seq_size+1);
					break;
			case HEADSET_CONFIG_PATH:
					headset_config_path_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<HEADSET_CONFIG_PATH;
					adb_point+=(adb_point->seq_size+1);
					break;
			case HEADSET_CONFIG_ACCOUSIC:
					headset_config_accousic_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<HEADSET_CONFIG_ACCOUSIC;
					adb_point+=(adb_point->seq_size+1);
					break;
			case HEADSET_CONFIG_ACCOUSIC_RESOTRE:
					headset_config_accousic_restore_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<HEADSET_CONFIG_ACCOUSIC_RESOTRE;
					adb_point+=(adb_point->seq_size+1);
					break;
			case HEADSET_START:
					headset_start_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<HEADSET_START;
					adb_point+=(adb_point->seq_size+1);
					break;
			case HEADSET_STOP:
					headset_stop_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<HEADSET_STOP;
					adb_point+=(adb_point->seq_size+1);
					break;
			case HANDSET_MIC_PATH_START:
					handset_mic_path_start_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<HANDSET_MIC_PATH_START;
					adb_point+=(adb_point->seq_size+1);
					break;
			case HANDSET_MIC_PATH_STOP:
					handset_mic_path_stop_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<HANDSET_MIC_PATH_STOP;
					adb_point+=(adb_point->seq_size+1);
					break;
			case HEADSET_MIC_PATH_START:
					headset_mic_path_start_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<HEADSET_MIC_PATH_START;
					adb_point+=(adb_point->seq_size+1);
					break;
			case HEADSET_MIC_PATH_STOP:
					headset_mic_path_stop_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<HEADSET_MIC_PATH_STOP;
					adb_point+=(adb_point->seq_size+1);
					break;
			case LINEOUT_HEADSET_PATH_START:
					lineout_headset_path_start_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<LINEOUT_HEADSET_PATH_START;
					adb_point+=(adb_point->seq_size+1);
					break;
			case LINEOUT_HEADSET_PATH_STOP:
					lineout_headset_path_stop_from_adb = (struct regs_setting *)(adb_point+1);
					adb_flag |= 1<<LINEOUT_HEADSET_PATH_STOP;
					adb_point+=(adb_point->seq_size+1);
					break;
			default :
					pr_err("not support seq number %d \n",adb_point->seq);
					break;
			}	

	}
	pr_err("end adb worker adb_flag = %d  \n",adb_flag);
}



int wait_sequencer_complete(void)
{
	int rc;
	rc=adie_codec_read(WM8994_WRITE_SEQUENCER_CTRL_2); //check sequencer execute completely.
	while(rc&0x100)
	{
	rc=adie_codec_read(WM8994_WRITE_SEQUENCER_CTRL_2);
	mdelay(10);
	}
	adie_codec_write(WM8994_INTERRUPT_STATUS_2,0x2000);  //clear WSEQ_DONE_EINT flag
	return 0;
}

void set_mclk_as_sysclock(void)
{
	adie_codec_write(WM8994_AIF1_CLOCKING_1, 1);
	adie_codec_write(WM8994_AIF2_CLOCKING_1, 1);
}

void set_FLL1_as_sysclock(void)
{
	adie_codec_write(WM8994_FLL1_CONTROL_2,0x0704);
	adie_codec_write(WM8994_FLL1_CONTROL_3,0x0);
	adie_codec_write(WM8994_FLL1_CONTROL_4,0x1000);
	adie_codec_write(WM8994_FLL1_CONTROL_5,0x0C82);
	adie_codec_write(WM8994_FLL1_CONTROL_1,0x0001);
	adie_codec_write(WM8994_AIF1_CLOCKING_1,0x11);
	adie_codec_write(WM8994_CLOCKING_1, 0xa);
	mdelay(10);
}

void set_FLL2_as_sysclock(void)
{
	adie_codec_write(WM8994_FLL2_CONTROL_2, 0x704);
	adie_codec_write(WM8994_FLL2_CONTROL_3, 0x0);
	adie_codec_write(WM8994_FLL2_CONTROL_4, 0x1000);
	adie_codec_write(WM8994_FLL2_CONTROL_5, 0xc82);
	adie_codec_write(WM8994_FLL2_CONTROL_1, 0x1);
	adie_codec_write(WM8994_AIF2_CLOCKING_1, 0x19);
	adie_codec_write(WM8994_CLOCKING_1, 0x7);
	mdelay(10);
}

int adie_codec_wm8958_sequencer_init(struct wm8958_seq_item *codec_seq, u32 count)
{
	int n;
	for(n=0;n<count;n++)
		{
//		pr_err("index = 0x%x  , se1=0x%xwrite\n", codec_seq[n].index,codec_seq[n].sequencer_0);
		adie_codec_write(WM8994_WRITE_SEQUENCER_0+(codec_seq[n].index*4),     codec_seq[n].sequencer_0);
		adie_codec_write(WM8994_WRITE_SEQUENCER_0+(codec_seq[n].index*4+1), codec_seq[n].sequencer_1);
		adie_codec_write(WM8994_WRITE_SEQUENCER_0+(codec_seq[n].index*4+2), codec_seq[n].sequencer_2);
		adie_codec_write(WM8994_WRITE_SEQUENCER_0+(codec_seq[n].index*4+3), codec_seq[n].sequencer_3);
		switch (codec_seq[n].seq_type) {
			case  SEQ_LINEOUT_HEAD :
				  seq_lineout_path_index = codec_seq[n].index;
				  break;
			case SEQ_LINEOUT_ACOUSTIC_HEAD :
				  seq_lineout_acoustic_index = codec_seq[n].index;
				  break;
			case SEQ_HEADPHONE_HEAD:
				  seq_headphone_path_index = codec_seq[n].index;
				  break;
			case SEQ_HEADPHONE_ACOUSTIC_HEAD:
				  seq_headphone_acoustic_index = codec_seq[n].index;
				  break;
			default: break;
		}
		}
	return 0;
}

int adie_codec_wm8958_sysclk_en(u32 en)
{
	pr_err("open FLL1 clock  \n");
	/*use LRCLK as FLL1 src, LRCLK = 48KHz, FLL1CLK=SYSCLK=12.228KHz*/
	adie_codec_write(WM8994_FLL1_CONTROL_2,0x0704);
	adie_codec_write(WM8994_FLL1_CONTROL_3,0x0);
	adie_codec_write(WM8994_FLL1_CONTROL_4,0x1000);
	adie_codec_write(WM8994_FLL1_CONTROL_5,0x0C82);
	adie_codec_write(WM8994_FLL1_CONTROL_1,0x0001);
	adie_codec_write(WM8994_AIF1_CLOCKING_1,0x11);
	mdelay(10);
	return 0;
}


int adie_codec_wm8958_path_config(u32 path)
{
	int wseq_path_index, val;
	pr_err("adie_codec_wm8958_path_config path = 0x%x  \n",path);
	switch (path){
		case HEADSET_RX :
			wait_sequencer_complete();
			set_mclk_as_sysclock();
			current_sequence_num = HEADSET_CONFIG_PATH;
			schedule_work(&reg_map_work);
			wseq_path_index = seq_headphone_path_index;
			val = (1<<15 | 1<<8 | wseq_path_index);
			adie_codec_write(WM8994_WRITE_SEQUENCER_CTRL_1,val);
			break;
		
		case SPEAKER_RX:
			wait_sequencer_complete();
			set_mclk_as_sysclock();
			current_sequence_num = LINEOUT_CONFIG_PATH;
			schedule_work(&reg_map_work);
			wseq_path_index = seq_lineout_path_index;
			val = (1<<15 | 1<<8 | wseq_path_index);
			adie_codec_write(WM8994_WRITE_SEQUENCER_CTRL_1,val);
			break;

		case SPEAKER_HEADSET_RX:
			wait_sequencer_complete();
			set_mclk_as_sysclock();
			current_sequence_num = LINEOUT_HEADSET_PATH_START;
			schedule_work(&reg_map_work);
			run_sequencer(LINEOUT_HEADSET_PATH_START);
			adie_codec_write(WM8994_WRITE_SEQUENCER_CTRL_1,val = (1<<15 | 1<<8 | HEADPHONE_COLD_STARTUP));
			wait_sequencer_complete();
			set_FLL1_as_sysclock();
			break;
		
		case HANDSET_TX:
			wait_sequencer_complete();
			set_FLL2_as_sysclock();
			current_sequence_num = HANDSET_MIC_PATH_START;
			schedule_work(&reg_map_work);
			run_sequencer(HANDSET_MIC_PATH_START);
			break;

		case HEADSET_TX:
			wait_sequencer_complete();
			set_FLL2_as_sysclock();
			current_sequence_num = HEADSET_MIC_PATH_START;
			schedule_work(&reg_map_work);
			run_sequencer(HEADSET_MIC_PATH_START);
			break;

		default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL(adie_codec_wm8958_path_config);



int adie_codec_wm8958_path_run(u32 path)
{
	int wseq_path_index, val;
	pr_err("adie_codec_wm8958_path_run path = 0x%x  \n",path);
	switch (path){
		case HEADSET_RX :
			current_sequence_num = HEADSET_START;
			schedule_work(&reg_map_work);
			wseq_path_index = HEADPHONE_COLD_STARTUP;
			val = (1<<15 | 1<<8 | wseq_path_index);
			wait_sequencer_complete();
			adie_codec_write(WM8994_WRITE_SEQUENCER_CTRL_1,val);
			wait_sequencer_complete();
			current_sequence_num = HEADSET_CONFIG_ACCOUSIC;
			schedule_work(&reg_map_work);	
			run_sequencer(HEADSET_CONFIG_ACCOUSIC);
			set_FLL1_as_sysclock();
			break;
		
		case SPEAKER_RX:
			current_sequence_num = LINEOUT_START;
			schedule_work(&reg_map_work);
			wseq_path_index = LINEOUT_STARTUP;
			val = (1<<15 | 1<<8 | wseq_path_index);
			wait_sequencer_complete();
			adie_codec_write(WM8994_WRITE_SEQUENCER_CTRL_1,val);
			wait_sequencer_complete();
			current_sequence_num = LINEOUT_CONFIG_ACCOUSIC;
			schedule_work(&reg_map_work);	
			run_sequencer(LINEOUT_CONFIG_ACCOUSIC);
			set_FLL1_as_sysclock();
			break;
		case HANDSET_TX:
		case HEADSET_TX:
		case SPEAKER_HEADSET_RX:
			 break;
		default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL(adie_codec_wm8958_path_run);


int adie_codec_wm8958_path_stop(u32 path)
{
	int wseq_path_index, val;
	pr_err("adie_codec_wm8958_path_stop path  = 0x%x\n",path);
	
	if(path == HEADSET_RX || path == SPEAKER_RX){
		if(path == HEADSET_RX){
			current_sequence_num = HEADSET_STOP;
			schedule_work(&reg_map_work);
			current_sequence_num = HEADSET_CONFIG_ACCOUSIC_RESOTRE;
			schedule_work(&reg_map_work);
			restore_audio_path(path);
		}else{
			current_sequence_num = LINEOUT_STOP;
			schedule_work(&reg_map_work);
			current_sequence_num = LINEOUT_CONFIG_ACCOUSIC_RESTORE;
			schedule_work(&reg_map_work);
			restore_audio_path(path);
		}
			wseq_path_index = GENERIC_SHUTDOWN;
			val = (1<<15 | 1<<8 | wseq_path_index);
			wait_sequencer_complete();
			adie_codec_write(WM8994_WRITE_SEQUENCER_CTRL_1,val);
			if(restore_flag){
				wait_sequencer_complete();
				run_restorelist();
				restore_flag = 0;
	  		}
	  	// restore other register in path,such as EQ accoustic.....
	}
	else	{
		//HEADSET_TX or HANDSET_TX
		if(path == HEADSET_TX){
			current_sequence_num = HEADSET_MIC_PATH_STOP;
			schedule_work(&reg_map_work);
		}else if(path == HANDSET_TX){ 
			current_sequence_num = HANDSET_MIC_PATH_STOP;
			schedule_work(&reg_map_work);
		}else {
			current_sequence_num = LINEOUT_HEADSET_PATH_STOP;
			schedule_work(&reg_map_work);
		}
		wait_sequencer_complete();
		restore_audio_path(path);
		// need to run general shutdown for lineout_headset path ?
		if(restore_flag){
	  		run_restorelist();
	  		restore_flag = 0;
		}
	}
	wait_sequencer_complete();
	set_mclk_as_sysclock();
	return 0;
}
EXPORT_SYMBOL(adie_codec_wm8958_path_stop);

int adie_codec_wm8958_pa_en(u32 en)
{
	int val;
	if(en ==1)
		val = 0x41; //GPIO 1 as PA switch
	else
		val = 0x1;
	pr_err("adie_codec_wm8958_pa_en  en = %d\n",en);
	wait_sequencer_complete();
	adie_codec_write(WM8994_GPIO_1,val);
	return 0;
}

void adie_codec_wm8958_poweramp_on(void)
{
        adie_codec_wm8958_pa_en(1);

}
EXPORT_SYMBOL(adie_codec_wm8958_poweramp_on);

void adie_codec_wm8958_poweramp_off(void)
{
        adie_codec_wm8958_pa_en(0);

}
EXPORT_SYMBOL(adie_codec_wm8958_poweramp_off);

int adie_codec_set_device_digital_volume(struct adie_codec_path *path_ptr,
		u32 num_channels, u32 vol_percentage /* in percentage */)
{

	return -EPERM;
}
EXPORT_SYMBOL(adie_codec_set_device_digital_volume);

int adie_codec_set_device_analog_volume(struct adie_codec_path *path_ptr,
		u32 num_channels, u32 volume /* in percentage */)
{
	pr_err("%s: analog device volume not supported\n", __func__);

	return -EPERM;
}
EXPORT_SYMBOL(adie_codec_set_device_analog_volume);



int wm8958_reg_write(u16 reg,  u16 val)
{
//	printk(KERN_ERR "adie_code_write reg=%x, val=%x\n",reg, val);

	wm8994_reg_write(adie_codec.codec_pdata,reg, val);

	return 0;
}
EXPORT_SYMBOL(wm8958_reg_write);

int wm8958_reg_read(u16 reg)
{

 return wm8994_reg_read(adie_codec.codec_pdata,reg);

}
EXPORT_SYMBOL(wm8958_reg_read);



u32 adie_codec_freq_supported(struct adie_codec_dev_profile *profile,
	u32 requested_freq)
{
	u32 i, rc = -EINVAL;

	for (i = 0; i < profile->setting_sz; i++) {
		if (profile->settings[i].freq_plan >= requested_freq) {
			rc = 0;
			break;
		}
	}
	return rc;
}
EXPORT_SYMBOL(adie_codec_freq_supported);

int adie_codec_enable_sidetone(struct adie_codec_path *rx_path_ptr,
	u32 enable)
{

	pr_debug("%s()\n", __func__);

	return -EPERM;
}
EXPORT_SYMBOL(adie_codec_enable_sidetone);

static int wm8994_codec_probe(struct platform_device *pdev)
{

	pr_err("%s: wm8994 codec driver init\n", __func__);
	adie_codec.codec_pdata = dev_get_drvdata(pdev->dev.parent);
	// init codec sequencer
	if (wm8994_get_boardtype() == 0) // for topaz
		adie_codec_wm8958_sequencer_init(wm8958_topaz_sequence, ARRAY_SIZE(wm8958_topaz_sequence));
	else // for opal
		adie_codec_wm8958_sequencer_init(wm8958_opal_sequence, ARRAY_SIZE(wm8958_opal_sequence));
	INIT_WORK(&reg_map_work,reg_map_worker);
	reg_qsort(reg_map,0,ARRAY_SIZE(reg_map));

	//this is workaround, register sequener to correct LDO output voltage from Wolfson. 
	adie_codec_write(0x102,0x0003);
	adie_codec_write(0xCB,0x0080);
	adie_codec_write(0x102,0x0);
	// confiure CDC_MIC_I2S_WS<-->LRCLK2 as no pull
	adie_codec_write(0x703,0x8101);
	return 0;
}


#ifdef CONFIG_DEBUG_FS

static int wm8994_codec_debug_open(struct inode *inode, struct file *file)
{
        file->private_data = inode->i_private;
        pr_info("wm8994_codec: debug intf %s\n", (char *) file->private_data);
        return 0;
}

static ssize_t wm8994_codec_debug_write(struct file *file, const char __user *buf, size_t count, loff_t *f_pos )
{
	ssize_t retval = -ENOMEM;

	INIT_WORK(&adb_work,adb_worker);
	adb_buf = kmalloc(count,GFP_KERNEL);
	if(copy_from_user(adb_buf,buf,count)){
		retval = -EFAULT;
		return retval;
	}
	*f_pos += count;
	retval = count;
	schedule_work(&adb_work);
	return  retval;

}

static long wm8994_codec_debug_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct wm8994_reg_config config;
	int ret;
        int rc = 0;
	pr_info("wm8994_codec: debug ioctl %s\n", (char *) file->private_data);
	switch(cmd)
	{
		case WM8994_REG_WRITE:
			if (copy_from_user(&config, (void *) arg, sizeof(config))) {
				rc = -EFAULT;
				break;
			}
			pr_info("WM8994 regiter write: addr = %x, val = %x\n",config.addr,config.val);
			wm8994_reg_write(adie_codec.codec_pdata,config.addr, config.val);
			break;
		case WM8994_REG_READ:
			pr_info("WM8994 regiter read\n");
			if (copy_from_user(&config, (void *) arg, sizeof(config))) {
                                rc = -EFAULT;
                                break;
                        }
                        ret = wm8994_reg_read(adie_codec.codec_pdata,config.addr);
			if (ret < 0) {
                		dev_err(adie_codec.codec_pdata->dev, "Failed to read register in wm8994 codec debug ioctl\n");
				rc = -EFAULT;
				break;
        		}
			config.val = ret;
			pr_info("WM8994 regiter read: addr = %x, val = %x\n",config.addr,config.val);
			if (copy_to_user((void *) arg, &config,
					sizeof(config)))
			break;
		default:
                	rc = -EINVAL;
                	break;
	}
        return rc;
}

static const struct file_operations wm8994_codec_debug_fops = {
        .open = wm8994_codec_debug_open,
        .unlocked_ioctl = wm8994_codec_debug_ioctl,
        .write = wm8994_codec_debug_write
};

#endif

static struct platform_driver wm8994_codec_driver = {
	.probe = wm8994_codec_probe,
	.driver = {
		.name = "wm8994-codec",
		.owner = THIS_MODULE,
	},
};

static int __init wm8994_codec_init(void)
{
	s32 rc;

	rc = platform_driver_register(&wm8994_codec_driver);
	if (IS_ERR_VALUE(rc))
		goto error;

#ifdef CONFIG_DEBUG_FS

	debugfs_sdev_dent = debugfs_create_dir("wm8994-codec", 0);
	if (!IS_ERR(debugfs_sdev_dent)) {
		debugfs_wm8994_i2c = debugfs_create_file("i2c",
		S_IFREG | S_IRUGO | S_IWUGO, debugfs_sdev_dent,
		(void *) "i2c", &wm8994_codec_debug_fops);
	}
#endif

	mutex_init(&adie_codec.lock);
error:
	return rc;
}

static void __exit wm8994_codec_exit(void)
{

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(debugfs_wm8994_i2c);
	debugfs_remove(debugfs_sdev_dent);
#endif
	platform_driver_unregister(&wm8994_codec_driver);
}

module_init(wm8994_codec_init);
module_exit(wm8994_codec_exit);

MODULE_DESCRIPTION("wm8994 codec driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
