#ifndef __LINUX_ISL29023_H__
#define __LINUX_ISL29023_H__
struct intersil_isl29023_platform_data {
	int (*power)(int);
	//HP zhanghong: Oct 15 18:05 CST 2010, begin
	int (*configure_int_pin)(int);
	//End
};
#endif
