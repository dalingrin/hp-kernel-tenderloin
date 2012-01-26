#ifndef _LINUX_BOARDID_H
#define _LINUX_BOARDID_H

struct boardid_info_s
{
    int emu;
    int product;
    int sku;
    int hwbuild;
};

extern struct boardid_info_s boardid_info;


#endif //_LINUX_BOARDID_H
