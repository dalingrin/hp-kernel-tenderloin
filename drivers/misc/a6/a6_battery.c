#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/wait.h>
#include <linux/bitops.h>
#include <linux/ctype.h>
#include <linux/a6_sbw_interface.h>
#include <linux/a6.h>
#ifdef CONFIG_HIGH_RES_TIMERS
#include <linux/hrtimer.h>
#endif
#include <linux/cpufreq.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <asm/io.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include "high_level_funcs.h"
#include <linux/firmware.h>

#include <linux/power_supply.h>
#include <linux/max8903_charger.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif

/* page 0x01 */
/* battery (airboard only); interface defined by phone teams */
#define TS2_I2C_BAT_STATUS                             0x0100
#define TS2_I2C_BAT_RARC                               0x0101
#define TS2_I2C_BAT_RSRC                               0x0102
#define TS2_I2C_BAT_AVG_CUR_MSB                        0x0103
#define TS2_I2C_BAT_AVG_CUR_LSB                        0x0104
#define TS2_I2C_BAT_TEMP_MSB                           0x0105
#define TS2_I2C_BAT_TEMP_LSB                           0x0106
#define TS2_I2C_BAT_VOLT_MSB                           0x0107
#define TS2_I2C_BAT_VOLT_LSB                           0x0108
#define TS2_I2C_BAT_CUR_MSB                            0x0109
#define TS2_I2C_BAT_CUR_LSB                            0x010a
#define TS2_I2C_BAT_COULOMB_MSB                        0x010b
#define TS2_I2C_BAT_COULOMB_LSB                        0x010c
#define TS2_I2C_BAT_AS                                 0x010d
#define TS2_I2C_BAT_FULL_MSB                           0x010e
#define TS2_I2C_BAT_FULL_LSB                           0x010f
#define TS2_I2C_BAT_FULL40_MSB                         0x0110
#define TS2_I2C_BAT_FULL40_LSB                         0x0111
#define TS2_I2C_BAT_RSNSP                              0x0112
#define TS2_I2C_BAT_RAAC_MSB                           0x0113
#define TS2_I2C_BAT_RAAC_LSB                           0x0114


#define TS2_I2C_BAT_ROMID_0                            0x0120
#define TS2_I2C_BAT_ROMID(x) \
        (TS2_I2C_BAT_ROMID_0 + (x))

#define TS2_I2C_BAT_COMMAND_STATUS                     0x0140
#define TS2_I2C_BAT_COMMAND_AUTH                 	0x81
#define TS2_I2C_BAT_COMMAND_REFRESH              	0x82
#define TS2_I2C_BAT_COMMAND_WAKE                 	0x83
#define TS2_I2C_BAT_COMMAND_OFF                  	0xe9
#define TS2_I2C_BAT_STATUS_AUTH_FAIL             	0x08
#define TS2_I2C_BAT_STATUS_AUTH_PASS             	0x04
#define TS2_I2C_BAT_STATUS_REGS_VALID            	0x02
#define TS2_I2C_BAT_STATUS_BUSY                  	0x01


/* battery configuration (airboard only) */
#define TS2_I2C_BAT_TEMP_LOW_MSB                       0x0180
#define TS2_I2C_BAT_TEMP_LOW_LSB                       0x0181
#define TS2_I2C_BAT_TEMP_HIGH_MSB                      0x0182
#define TS2_I2C_BAT_TEMP_HIGH_LSB                      0x0183
#define TS2_I2C_BAT_VOLT_LOW_MSB                       0x0184
#define TS2_I2C_BAT_VOLT_LOW_LSB                       0x0185
#define TS2_I2C_BAT_RARC_CRIT                          0x0186
#define TS2_I2C_BAT_RARC_LOW_2                         0x0187
#define TS2_I2C_BAT_RARC_LOW_1                         0x0188

#define TS2_I2C_BAT_CHALLENGE_0                        0x01e0
#define TS2_I2C_BAT_CHALLENGE(x) \
        (TS2_I2C_BAT_CHALLENGE_0 + (x))
#define TS2_I2C_BAT_RESPONSE_0 \
        (TS2_I2C_BAT_CHALLENGE_0 + 8)
#define TS2_I2C_BAT_RESPONSE(x) \
        (TS2_I2C_BAT_RESPONSE_0 + (x))

#if defined(current)
#undef current
#endif
struct a6_battery_info {
	struct i2c_client	*client;
	struct power_supply	battery;	
	struct delayed_work	poll_work;
	struct workqueue_struct	*poll_workqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	suspend_state_t pm_state;
	struct early_suspend early_suspend;
#endif
	int 			present;
	int 			authentic;
	int 			percentage;
	int 			voltage;
	int 			current;
	int 			temperature;
	unsigned int 		capacity; /* in uAh */
};
static struct a6_battery_info battery_info;
typedef enum {
	A6_ERR_OK		= 0,
	A6_ERROR   		= -1,
	A6_ERR_I2C    		= -2,
	A6_ERR_TIMEOUT 		= -3,
	A6_ERR_NOT_SUPPORTED 	= -4,
	A6_ERR_BUSY 	 	= -5,
} a6_err_code;

typedef enum {
	A6_CMD_REG_READ ,
	A6_CMD_REG_WRITE,
} a6_reg_cmd_code;

enum {
	DEVICE_BUSY_BIT = 0,
	IS_OPENED,
	IS_INITIALIZED_BIT,
	BOOTLOAD_ACTIVE_BIT,
	FORCE_WAKE_ACTIVE_BIT,

	// capabilities
	CAP_PERIODIC_WAKE,
#ifdef A6_PQ
	STARTING_AID_TASK,
	KILLING_AID_TASK,
	IS_QUIESCED,
#endif
	SIZE_FLAGS
};


struct a6_device_state {
	struct i2c_client *i2c_dev;
	struct a6_platform_data *plat_data;
	struct file_operations fops;
	struct miscdevice mdev;

	struct mutex dev_mutex;
	unsigned int timestamping;

	struct timer_list	a6_force_wake_timer;
	struct work_struct	a6_force_wake_work;
	struct mutex		a6_force_wake_mutex;

	wait_queue_head_t	dev_busyq;
	struct work_struct	a6_irq_work;

	int32_t cpufreq_hold_flag;
	uint32_t cached_rsense_val: 	16;
	struct workqueue_struct* ka6d_workqueue;
	struct workqueue_struct* ka6d_fw_workqueue;

	DECLARE_BITMAP(flags, SIZE_FLAGS);

#ifdef A6_PQ
	struct completion aq_enq_complete;
	struct completion aid_exit_complete;
	struct mutex aq_mutex;
	struct list_head aq_head;
	struct task_struct* ai_dispatch_task;
#ifdef A6_DEBUG
	uint32_t dbgflg_kill_raid: 	1;

	uint8_t debug_restart_aid;
	uint8_t debug_flush_aiq;
	uint8_t debug_unused_01;
	uint8_t debug_unused_02;
#endif
#endif

	int cpufreq_hold;
};


struct a6_reg_cmd
{
	a6_reg_cmd_code cmd;
	uint16_t 	reg;
	uint8_t 	data;
};

#define CHALLENGE_SIZE       8
#define MAC_SIZE             20

#define R_SENSE    r_sense

#define SIGN_EXTEND16(x)		((x)-(((x)&0x8000)?65536:0))
#define CURRENT_VALUE(x)		((SIGN_EXTEND16(x)*3125)/2/R_SENSE) // in uA
#define VOLTAGE_VALUE(x)	(4880*((x)>>5)) // in micro volt
#define COULOMB_VALUE(x)        ((6250*(x))/R_SENSE)
#define REG_COULOMB_VALUE(x)    ((R_SENSE*(x))/6250)
#define CAPACITY_VALUE(x)		(1600*(x))      // in micro Ahr
#define CAPACITY_PERCENT(x)		(392*x)		// in thousands of %

#define BATTERY_DEFAULT_CHALLENGE  \
	{ 0x74, 0xca, 0x85, 0x99, 0x19, 0xde, 0xd1, 0xb3 }
#define BATTERY_DEFAULT_RESPONSE   \
	{ 0x87, 0xed, 0x20, 0x89, 0xad, 0x68, 0xa2, 0xcd, 0x6f, 0x93, \
	  0x13, 0x03, 0x07, 0x5a, 0x29, 0x85, 0xdc, 0x2e, 0xe9, 0x50 }

static int battery_authentic = 0;
static int r_sense = 20;

#define A6_BATTERY_POLLING_DELAY_MS (10)
#define A6_BATTERY_POLLING_TIMEOUT_CNT (20) 
void a6_delay(uint32_t delay_ms){
	msleep(delay_ms);
}
int32_t a6_i2c_read_reg(struct i2c_client* client, const uint16_t* ids, uint32_t num_ids, uint8_t* out);
int32_t a6_i2c_write_reg(struct i2c_client* client, const uint16_t* ids, uint32_t num_ids, const uint8_t* in);


a6_err_code a6_send_reg_commands(struct a6_reg_cmd *cmdlist, int count)
{
	a6_err_code res = A6_ERR_OK;
	int i;
	struct a6_reg_cmd *p_cmd = cmdlist;
	struct a6_device_state* state = i2c_get_clientdata(battery_info.client);

		/* return if a6 un-initialized */
	if (!test_bit(IS_INITIALIZED_BIT, state->flags)) {
		pr_debug("%s: a6 un-initialized: exiting.\n", __func__);
		res = A6_ERROR;
		return res;
	}
	/* loop through to send all commands */
	for (i = 0; i < count; i++) {
		int i2c_res;

		if (p_cmd->cmd == A6_CMD_REG_READ ) {
			
			i2c_res = a6_i2c_read_reg(battery_info.client, &(p_cmd->reg), 1, &(p_cmd->data));

			if (i2c_res < 0) {
				res = A6_ERR_I2C;
			}
		} else if (p_cmd->cmd == A6_CMD_REG_WRITE) {
			i2c_res = a6_i2c_write_reg(battery_info.client, &(p_cmd->reg), 1, &(p_cmd->data));

			if (i2c_res < 0) {
				res = A6_ERR_I2C;
			}
		} else {
			printk("A6 a6_send_reg_commands : Unknown Cmd %x\n", p_cmd->cmd);
			res = A6_ERROR;
		}

		//Workaround for A6 I2C read issue
		//a6_delay(1);

		p_cmd++;
	}
	return res;
}
static a6_err_code a6_battery_wait_busy(uint8_t clear_flags, uint8_t set_flags, uint8_t *p_status) {
	int i;
	a6_err_code res = A6_ERR_OK;
	uint8_t status = 0;
	static struct a6_reg_cmd cmdlist[] = {
		{
			.cmd = A6_CMD_REG_READ,
			.reg = TS2_I2C_BAT_COMMAND_STATUS,
		},
	};

	for ( i = A6_BATTERY_POLLING_TIMEOUT_CNT; i > 0; i-- ) {
		res = a6_send_reg_commands(cmdlist, 1);

		if (res != A6_ERR_OK) {
			goto a6_battery_wait_busy_exit;
		} else {
			status = cmdlist[0].data;
		}

		if ( ( !set_flags || (status & set_flags) != 0) && ( !clear_flags || ((~status) & clear_flags) != 0 )) {
			break;
		}

		a6_delay(A6_BATTERY_POLLING_DELAY_MS);
	};

	if ( i <= 0 ) {
		res = A6_ERROR;
	} else if ( p_status ) {
		*p_status = status;
	}
a6_battery_wait_busy_exit:
	return res;
}
//:TODO: Add busy handling - probably return the last value
//:TODO: Add A6 capabilites handling

int battery_get_percentage(void)
{
	ushort percentage = 0;

	static struct a6_reg_cmd cmdlist[] = {
		{
			.cmd = A6_CMD_REG_READ,
			.reg = TS2_I2C_BAT_RARC,
		},
	};

	if ( A6_ERR_OK != a6_battery_wait_busy(0, TS2_I2C_BAT_STATUS_REGS_VALID, NULL)){
		goto battery_get_percentage_exit;
	}

	if (A6_ERR_OK != a6_send_reg_commands(cmdlist, 1)) {
		goto battery_get_percentage_exit;
	} else {
		percentage = cmdlist[0].data;
	}

battery_get_percentage_exit:
	return percentage;
}

int battery_get_voltage(void)
{
	int res = 0;
	ushort voltage = 0;

	static struct a6_reg_cmd cmdlist[] = {
		{
			.cmd = A6_CMD_REG_READ,
			.reg = TS2_I2C_BAT_VOLT_MSB,
		},
		{
			.cmd = A6_CMD_REG_READ,
			.reg = TS2_I2C_BAT_VOLT_LSB,
		},
	};

	if ( A6_ERR_OK != a6_battery_wait_busy(0, TS2_I2C_BAT_STATUS_REGS_VALID, NULL)){
		goto battery_get_voltage_exit;
	}

	if (A6_ERR_OK != a6_send_reg_commands(cmdlist, 2)) {
		goto battery_get_voltage_exit;
	} else {
		voltage = (cmdlist[0].data << 8) | cmdlist[1].data;
		res =  VOLTAGE_VALUE(voltage);
	}

battery_get_voltage_exit :
	return res;
}

static int battery_get_r_sense(void)
{
	int res = 0;

	static struct a6_reg_cmd cmdlist[] = {
		{
			.cmd = A6_CMD_REG_READ,
			.reg = TS2_I2C_BAT_RSNSP,
		},
	};

	if ( A6_ERR_OK != a6_battery_wait_busy(0, TS2_I2C_BAT_STATUS_REGS_VALID, NULL)){
		goto battery_get_r_sense_exit;
	}

	if (A6_ERR_OK != a6_send_reg_commands(cmdlist, 1)) {
		goto battery_get_r_sense_exit;
	} else {
		res = 1000 / cmdlist[0].data;
	}

battery_get_r_sense_exit :
	return res;
}

int battery_get_current(void)
{
	int res = 0;
	ushort cur;

	static struct a6_reg_cmd cmdlist[] = {
		{
			.cmd = A6_CMD_REG_READ,
			.reg = TS2_I2C_BAT_CUR_MSB,
		},
		{
			.cmd = A6_CMD_REG_READ,
			.reg = TS2_I2C_BAT_CUR_LSB,
		},
	};

	if ( A6_ERR_OK != a6_battery_wait_busy(0, TS2_I2C_BAT_STATUS_REGS_VALID, NULL)){
		goto battery_get_current_exit;
	}

	if (A6_ERR_OK != a6_send_reg_commands(cmdlist, 2)) {
		goto battery_get_current_exit;
	} else {
		cur = (cmdlist[0].data << 8) | cmdlist[1].data;
		res = CURRENT_VALUE(cur);
	}

battery_get_current_exit :
	return res;
}

/** 
* @brief Return the battery temperature.
* 
* Temperature value is a 11-bit two's-complement integer with a 1/8
* degrees Centigrade resolution:
*   MSB: 7 6 5 4 3 2 1 0  LSB: 7 6 5 4 3 2 1 0
*        s i i i i i i i       f f f _ _ _ _ _
*
*  s: sign
*  i: integer part
*  f: fraction part
*  _: N/A
*
* We discard the fraction and only return the signed integer value.
*/
int battery_get_temperature(void)
{
	int res = 0;
	static struct a6_reg_cmd cmdlist[] = {
		{
			.cmd = A6_CMD_REG_READ,
			.reg = TS2_I2C_BAT_TEMP_MSB,
		},
		{
			.cmd = A6_CMD_REG_READ,
			.reg = TS2_I2C_BAT_TEMP_LSB,
		},
	};

	if ( A6_ERR_OK != a6_battery_wait_busy(0, TS2_I2C_BAT_STATUS_REGS_VALID, NULL)){
		goto battery_get_temperature_exit;
	}

	if (A6_ERR_OK != a6_send_reg_commands(cmdlist, 2)) {
		goto battery_get_temperature_exit;
	} else {
		/* Only return the signed integer value */
		res = (int8_t)cmdlist[0].data;
	}

battery_get_temperature_exit:
	return res;
}

unsigned int battery_get_capacity(void)
{
	unsigned int res = 0;
	
	static struct a6_reg_cmd cmdlist[] = {
		{
			.cmd = A6_CMD_REG_READ,
			.reg = TS2_I2C_BAT_RAAC_MSB,
		},
		{
			.cmd = A6_CMD_REG_READ,
			.reg = TS2_I2C_BAT_RAAC_LSB,
		},
	};

	if ( A6_ERR_OK != a6_battery_wait_busy(0, TS2_I2C_BAT_STATUS_REGS_VALID, NULL)){
		goto battery_get_capacity_exit;
	}

	if (A6_ERR_OK != a6_send_reg_commands(cmdlist, 2)) {
		goto battery_get_capacity_exit;
	} else {
		uint32_t capacity = (cmdlist[0].data << 8) | cmdlist[1].data;
		res = CAPACITY_VALUE(capacity);
	}

battery_get_capacity_exit:

	return res;
}

int battery_is_present(void)
{
	int rc = 0;
	if (battery_get_voltage() == 0 && battery_get_current() == 0)
	{
		// We do not have a battery, or best case the battery we have is unusable
		rc = -1;
	}
	return (rc < 0) ? 0 : 1;
}

int battery_reauthenticate(void)
{
	int res = 0;
	int i;
	uint8_t batt_cmd_status = 0;
	static int cmd_lists_initiated = 0;
	static const uint8_t dc[] = BATTERY_DEFAULT_CHALLENGE;
	static const uint8_t dr[] = BATTERY_DEFAULT_RESPONSE;
	static struct a6_reg_cmd send_challenge_cmdlist[CHALLENGE_SIZE];
	static struct a6_reg_cmd get_response_cmdlist[MAC_SIZE];
	static struct a6_reg_cmd authenticate_cmdlist[] = {
		{
			.cmd  = A6_CMD_REG_WRITE,
			.reg  = TS2_I2C_BAT_COMMAND_STATUS,
			.data = TS2_I2C_BAT_COMMAND_AUTH,
		},
	};

	printk("Starting battery authentication...\n");

	if ( !cmd_lists_initiated ) {

		//Init the challenge a6 cmd list
		for ( i = 0; i < CHALLENGE_SIZE; i ++ ) {
			send_challenge_cmdlist[i].cmd  = A6_CMD_REG_WRITE;
			send_challenge_cmdlist[i].reg  = TS2_I2C_BAT_CHALLENGE(i);
			send_challenge_cmdlist[i].data = dc[i];
		}

		//Init the response a6 cmd list
		for ( i = 0; i < MAC_SIZE; i ++ ) {
			get_response_cmdlist[i].cmd  = A6_CMD_REG_READ;
			get_response_cmdlist[i].reg  = TS2_I2C_BAT_RESPONSE(i);
		}
		
		cmd_lists_initiated = 1;		
	}

	if ( A6_ERR_OK != a6_battery_wait_busy(TS2_I2C_BAT_STATUS_BUSY, 0, NULL)){
		goto battery_reauthenticate_exit;
	}

	if (A6_ERR_OK != a6_send_reg_commands(send_challenge_cmdlist, CHALLENGE_SIZE)) {
		goto battery_reauthenticate_exit;
	} 

	if (A6_ERR_OK != a6_send_reg_commands(authenticate_cmdlist, 1)) {
		goto battery_reauthenticate_exit;
	} 
	
	a6_delay(10);

	if ( A6_ERR_OK != a6_battery_wait_busy(TS2_I2C_BAT_STATUS_BUSY, 0, &batt_cmd_status)){
		goto battery_reauthenticate_exit;
	}

	if ( batt_cmd_status & TS2_I2C_BAT_STATUS_AUTH_FAIL ) {
		goto battery_reauthenticate_exit;
	}

	if (A6_ERR_OK != a6_send_reg_commands(get_response_cmdlist, MAC_SIZE)) {
		goto battery_reauthenticate_exit;
	} 

	res = 1;
	//Validate response
	for ( i = 0; i < MAC_SIZE; i++) {
		if ( dr[i] != get_response_cmdlist[i].data ) {
			res = 0;
			break;
		}
	}

	if (0 == res) {
	        printk("*** Battery Authentication Failure:     ***\n");
	        printk("*** [Expected and actual response dump] ***\n");
	        for ( i = 0; i < MAC_SIZE; i++) {
		        printk("exp-resp: 0x%02x act-resp: 0x%02x\n", dr[i], get_response_cmdlist[i].data);
		}
	} else {
	  printk("Battery authentication successful.\n");
	}
	
battery_reauthenticate_exit :

	battery_authentic = res;
	return res;
}

int battery_set_authentic(bool authentic)
{
	battery_authentic = authentic;
	return battery_authentic;
}

static a6_err_code a6_battery_send_command(uint8_t command)
{
	a6_err_code res = A6_ERR_OK;

	res = a6_battery_wait_busy(TS2_I2C_BAT_STATUS_BUSY, 0, NULL);

	if ( A6_ERR_OK == res ){
		struct a6_reg_cmd cmdlist[] = {
			{
				.cmd = A6_CMD_REG_WRITE,
				.reg = TS2_I2C_BAT_COMMAND_STATUS,
				.data = command,
			}
		};

		res = a6_send_reg_commands(cmdlist, 1);
	}

	return res;
}

static a6_err_code battery_refresh_regs(void)
{
	a6_err_code res = A6_ERR_OK;

	res = a6_battery_send_command(TS2_I2C_BAT_COMMAND_REFRESH);

	if ( res == A6_ERR_OK ) {
		msleep(20);
	}

	return res;
}

void battery_startup(void)
{
	if ( a6_battery_send_command(TS2_I2C_BAT_COMMAND_WAKE) == A6_ERR_OK ) {
		//Wait for 200 ms for battery to be woken up
		msleep(200);
		
		//Refresh all the registers
		if ( battery_refresh_regs() == A6_ERR_OK ) {
			int r_sense_tmp;

			r_sense_tmp = battery_get_r_sense();
		
			if ( r_sense_tmp ) {
				r_sense = r_sense_tmp;
			}
		}
	}
}
	
void battery_sleep(void)
{
	a6_battery_send_command(TS2_I2C_BAT_COMMAND_OFF);
}

int battery_read(struct a6_battery_info *batt_info)
{
	battery_refresh_regs();
	
	batt_info->present = battery_is_present();
	if (!batt_info->present) {
		return 0;
	}

	batt_info->authentic = battery_authentic;    /* read once on init */
	batt_info->percentage = battery_get_percentage();
	batt_info->voltage = battery_get_voltage();
	batt_info->current = battery_get_current();
	batt_info->temperature = battery_get_temperature();
	batt_info->capacity = battery_get_capacity();

	return 0;
}

static enum power_supply_property a6_battery_props[] = {
        POWER_SUPPLY_PROP_STATUS,
        POWER_SUPPLY_PROP_CAPACITY,
        POWER_SUPPLY_PROP_VOLTAGE_NOW,
        POWER_SUPPLY_PROP_TEMP,
        POWER_SUPPLY_PROP_HEALTH,
        POWER_SUPPLY_PROP_TECHNOLOGY,
        POWER_SUPPLY_PROP_PRESENT,
};
static void a6_battery_ext_power_changed(struct power_supply *psy)
{
	//battery_read(&battery_info);
	//pr_debug("%s(%d) : battery_info.percentage = %d, battery_info.voltage = %d, battery_info.current = %d, battery_info.temperature = %d\n", 
	//	__func__, __LINE__, battery_info.percentage, battery_info.voltage, battery_info.current, battery_info.temperature);
	//power_supply_changed(psy);
	
	pr_debug("%s(%d): battery_info.pm_state = %d", __func__, __LINE__, battery_info.pm_state);
	if (battery_info.pm_state < PM_SUSPEND_MEM) {
		if (delayed_work_pending(&(battery_info.poll_work))) {
			cancel_delayed_work_sync(&(battery_info.poll_work));
		}
		queue_delayed_work(battery_info.poll_workqueue , &(battery_info.poll_work), msecs_to_jiffies(10));
	}
}
static int a6_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{

	int status, ret = 0;
	bool charger_is_charging = false, charger_is_connecting = false;
	
	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		charger_is_charging = max8903_charger_is_charging();
		charger_is_connecting = max8903_charger_is_connecting();
		
		//status
		if (!battery_info.present) {
			status = POWER_SUPPLY_STATUS_UNKNOWN;
		} else {
			if ((battery_info.percentage== 100) && charger_is_connecting && !charger_is_charging)
				status = POWER_SUPPLY_STATUS_FULL;
			else if (charger_is_connecting && !charger_is_charging)
					status = POWER_SUPPLY_STATUS_NOT_CHARGING;
				else if (!charger_is_connecting)
					status = POWER_SUPPLY_STATUS_DISCHARGING;
					else
					status = POWER_SUPPLY_STATUS_CHARGING;
		}
		pr_debug("A6_BATTEERY : (%s) (%d) : prop = %d, status = %d, percentage = %d, charger_is_connecting = %d, charger_is_charging = %d\n",
				 __func__, __LINE__, prop, status, battery_info.percentage, charger_is_connecting, charger_is_charging);
		val->intval = status;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		//capacity
		pr_debug("A6_BATTEERY : (%s) (%d) : prop = %d, capacity = %d\n", __func__, __LINE__, prop, battery_info.percentage);
		val->intval = battery_info.percentage;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		pr_debug("A6_BATTEERY : (%s) (%d) : prop = %d, voltage_mV = %d\n", __func__, __LINE__, prop, battery_info.voltage);
		val->intval = battery_info.voltage;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		//temp
		pr_debug("A6_BATTEERY : (%s) (%d) : prop = %d, temp_c = %d\n", __func__, __LINE__, prop, battery_info.temperature);
		val->intval = battery_info.temperature * 10;
		break;
		
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
		
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battery_info.present;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}
void a6_battery_poll_work_handler(struct work_struct *work)
{
	battery_read(&battery_info);
	pr_debug("%s(%d) : battery_info.percentage = %d, battery_info.voltage = %d, battery_info.current = %d, battery_info.temperature = %d\n", 
		__func__, __LINE__, battery_info.percentage, battery_info.voltage, battery_info.current, battery_info.temperature);
	power_supply_changed(&(battery_info.battery));
	queue_delayed_work(battery_info.poll_workqueue , &(battery_info.poll_work), msecs_to_jiffies(10 * 1000));
}
#ifdef CONFIG_HAS_EARLYSUSPEND
void batt_early_suspend(struct early_suspend *h)
{
	pr_debug("%s(%d): enter\n", __func__, __LINE__);

	if (delayed_work_pending(&(battery_info.poll_work))) {
		cancel_delayed_work_sync(&(battery_info.poll_work));
	} 
	battery_info.pm_state = PM_SUSPEND_MEM;
	pr_debug("%s: exit\n", __func__);
}

void batt_late_resume(struct early_suspend *h)
{
	pr_debug("%s(%d): enter\n", __func__, __LINE__);
	if (delayed_work_pending(&(battery_info.poll_work))) {
		cancel_delayed_work_sync(&(battery_info.poll_work));
	}
	queue_delayed_work(battery_info.poll_workqueue , &(battery_info.poll_work), msecs_to_jiffies(1));
	battery_info.pm_state = PM_SUSPEND_ON;
	pr_debug("%s(%d): exit\n", __func__, __LINE__);
}

#endif
void a6_battery_init(struct i2c_client *a6_i2c_client)
{
	battery_info.battery.name = "battery";
	battery_info.battery.type = POWER_SUPPLY_TYPE_BATTERY;
	battery_info.battery.properties	= a6_battery_props;
	battery_info.battery.num_properties = ARRAY_SIZE(a6_battery_props);
	battery_info.battery.get_property = a6_battery_get_property;
	battery_info.battery.external_power_changed = a6_battery_ext_power_changed;
	battery_info.client = a6_i2c_client;
#ifdef CONFIG_HAS_EARLYSUSPEND
	battery_info.pm_state = PM_SUSPEND_ON;
	battery_info.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	battery_info.early_suspend.suspend =batt_early_suspend;
	battery_info.early_suspend.resume = batt_late_resume;
	register_early_suspend(&battery_info.early_suspend);
#endif
	battery_startup();
	battery_reauthenticate();
	battery_read(&battery_info);
	printk("battery_info.percentage = %d, battery_info.voltage = %d, battery_info.current = %d, battery_info.temperature = %d\n", 
		battery_info.percentage, battery_info.voltage, battery_info.current, battery_info.temperature);
	battery_info.poll_workqueue = create_workqueue("a6_battery_poll_workqueue");
        INIT_DELAYED_WORK(&(battery_info.poll_work), a6_battery_poll_work_handler);
	power_supply_register(&(a6_i2c_client->dev), &(battery_info.battery));
	queue_delayed_work(battery_info.poll_workqueue , &(battery_info.poll_work), msecs_to_jiffies(10 * 1000));
}

