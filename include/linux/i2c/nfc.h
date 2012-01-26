#ifndef __NFC_H__
#define __NFC_H__

#define PM8058_NFC_IRQOUT	      15
#define PM8058_NFC_WAKEUP	      16

struct nfc_platform_data {
  int (*nfc_reset)(void);
  int (*nfc_irq)(void);
  int (*nfc_iocfg)(void);
};

#endif