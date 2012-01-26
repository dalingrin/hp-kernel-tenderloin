#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/boardid.h>
#include <linux/module.h>



struct boardid_info_s boardid_info;
EXPORT_SYMBOL(boardid_info);

static int boardid_proc_show(struct seq_file *m, void *v)
{

    int emu, product, hwbuild, sku;

    emu = boardid_info.emu;
    product = boardid_info.product;
    sku = boardid_info.sku;
    hwbuild = boardid_info.hwbuild;
    
    
    seq_printf(m, "topaz hardware info\n");
    seq_printf(m, "------------------------\n");
    seq_printf(m, "EMU       : %d\n", emu);
    seq_printf(m, "PRODUCT   : %d\n", product);
    seq_printf(m, "SKU       : %d\n", sku);
    seq_printf(m, "HWBUILD   : %d\n", hwbuild);
    seq_printf(m, "------------------------\n");
    
    if(emu)
        seq_printf(m, "EMU       : EMU\n");
    else
        seq_printf(m, "EMU       : PRODUCT\n");

    switch(product)
    {
    case 0:
        seq_printf(m, "PRODUCT   : Stingary\n");
        break;
    case 1:
        seq_printf(m, "PRODUCT   : Topaz\n");
        break;
    case 2:
        seq_printf(m, "PRODUCT   : Opal\n");
        break;
    default:
        seq_printf(m, "PRODUCT   : not define\n");
        break;
    }

    if(sku)
        seq_printf(m, "SKU       : 3G\n");
    else
        seq_printf(m, "SKU       : Wifi only\n");

    switch(hwbuild)
    {
    case 7:
        seq_printf(m, "HW BUILD  : 1st build\n");
        break;
    case 6:
        seq_printf(m, "HW BUILD  : 2nd build\n");
        break;
    case 5:
        seq_printf(m, "HW BUILD  : 3rd build\n");
        break;
    case 4:
        seq_printf(m, "HW BUILD  : 4th build\n");
        break;
    case 3:
        seq_printf(m, "HW BUILD  : 5th build\n");
        break;
    case 2:
        seq_printf(m, "HW BUILD  : 6th build\n");
        break;
    case 1:
        seq_printf(m, "HW BUILD  : 7th build\n");
        break;
    case 0:
        seq_printf(m, "HW BUILD  : PVT\n");
        break;
    default:
        seq_printf(m, "HW BUILD  : not define\n");
        break;
    }
    
    return 0;
}

static int boardid_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, boardid_proc_show, NULL);
}

static const struct file_operations boardid_proc_fops = {
    .open       = boardid_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static int __init proc_boardid_init(void)
{
    proc_create("boardid", 0, NULL, &boardid_proc_fops);
    return 0;
}
module_init(proc_boardid_init);
