#ifndef __LINUX_CY8C20236A_H__
#define __LINUX_CY8C20236A_H__

#define PS_POLLING_ENABLE

/* Cypress proximity sensor registers */
#define CY8C_CTL_REG 0x00
#define CY8C_STATUS_REG 0x01

#define CY8C_DATA_CH1_LSB_REG   0x03
#define CY8C_DATA_CH1_MSB_REG   0x04
#define CY8C_DATA_CH2_LSB_REG   0x05
#define CY8C_DATA_CH2_MSB_REG   0x06
#define CY8C_DATA_CH3_LSB_REG    0x07
#define CY8C_DATA_CH3_MSB_REG   0x08

#define CY8C_THLD_LOW_CH1_REG   0x09
#define CY8C_THLD_HIGH_CH1_REG  0x0A
#define CY8C_THLD_LOW_CH2_REG   0x0b
#define CY8C_THLD_HIGH_CH2_REG  0x0c
#define CY8C_THLD_LOW_CH3_REG   0x0d
#define CY8C_THLD_HIGH_CH3_REG  0x0e

#define CY8C_SENTIVISITY_REG   0x0f

#define CY8C_DEVICE_ID_REG      0x10

/* control register*/
#define POWER_DOWN 0x00

/* proximity sensor level */
#define PS_LEVEL0      0x00
#define PS_LEVEL1      0x01
#define PS_LEVEL2      0x02
#define PS_LEVEL3      0x03

#define GPIO_INT_TYPE     0x1
#define GPIO_RESET_TYPE 0x2

struct cypress_cy8c20236a_platform_data {
  int (*power)(int);
	int (*p_out)(u8);
	int (*reset_pin)(u8);
};

/* Each client has this additional data */
struct cy8c20236a_data_t {
	spinlock_t	lock;
	//u8	minor;
	u8	dev_open_cnt;
	struct i2c_client* client;
	struct input_dev *input_dev;
	u8	enabled;
	#ifdef PS_POLLING_ENABLE
	struct delayed_work polling_work;
	#else
	struct workqueue_struct *psworkqueue;
	struct wake_lock ps_delayed_work_wake_lock;
	struct delayed_work datareport;
	#endif
	int pre_status;
	u8 resume_state[7]; //for control register
	struct cypress_cy8c20236a_platform_data *pdata;
	//HP zhanghong: Nov 2 10:52 CST 2010,begin
	int p_out;
	int reset_pin;
	//End
	//HP zhanghong: Dec 9 13:02 CST 2010, begin
	struct mutex mutex_lock;
	//End
};
#endif
