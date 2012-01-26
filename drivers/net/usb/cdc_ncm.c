/*
 * cdc_ncm.c
 * Copyright (C) ST-Ericsson AB 2010
 * Contact: Sjur Braendeland <sjur.brandeland@xxxxxxxxxxxxxx>
 * Original author:Hans Petter Selasky <hans.petter.selasky@xxxxxxxxxxxxxx>
 *
 * USB Host Driver for Network Control Model (NCM)
 * http://www.usb.org/developers/devclass_docs/NCM10.zip
 *
 * The NCM encoding, decoding and initialisation logic
 * derives from FreeBSD 8.x. if_cdce.c and if_cdcereg.h
 *
 * This software is available to you under a choice of one of two
 * licenses. You may choose this file to be licensed under the terms
 * of the GNU General Public License (GPL) Version 2 or the 2-clause
 * BSD license listed below:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/ctype.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/crc32.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <linux/version.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <asm/atomic.h>

#define	DRIVER_VERSION "0.9"

#include <linux/usb/usbnet.h>
#include "cdc_ncm.h"

static void cdc_ncm_tx_set_rem_skb(struct cdc_ncm_softc *,
	struct sk_buff *);
static void cdc_ncm_tx_timeout(unsigned long arg);
static const struct driver_info cdc_ncm_info;
static struct usb_driver cdc_ncm_driver;
static struct ethtool_ops cdc_ncm_ethtool_ops;

/*HP SamLin@20101216, Begin: Added for registering external modem device for ste u300 modem*/
int steu300_exmdm_register(void);
void steu300_exmdm_unregister(void);
/*HP SamLin@20101216, End: Added for registering external modem device for ste u300 modem*/

static const struct usb_device_id cdc_devs[] = {
	{ USB_INTERFACE_INFO(USB_CLASS_COMM,
		USB_SUBCLASS_CODE_NETWORK_CONTROL_MODEL, USB_CDC_PROTO_NONE),
		.driver_info = (unsigned long)&cdc_ncm_info,
	},
	{
	},
};

MODULE_DEVICE_TABLE(usb, cdc_devs);

static void
cdc_ncm_get_drvinfo(struct net_device *net, struct ethtool_drvinfo *info)
{
	struct usbnet *dev = netdev_priv(net);

	strncpy(info->driver, dev->driver_name, sizeof(info->driver));
	strncpy(info->version, DRIVER_VERSION, sizeof(info->version));
	strncpy(info->fw_version, dev->driver_info->description,
		sizeof(info->fw_version));
	usb_make_path(dev->udev, info->bus_info, sizeof(info->bus_info));
}

static int
cdc_ncm_do_request(struct cdc_ncm_softc *sc,
			struct cdc_ncm_device_request *req,
			void *data, u16 flags, u16 *actlen, u16 timeout)
{
	int err;

	err = usb_control_msg(sc->sc_udev, (req->bmRequestType & 0x80) ?
		usb_rcvctrlpipe(sc->sc_udev, 0) :
		usb_sndctrlpipe(sc->sc_udev, 0),
		req->bRequest, req->bmRequestType,
		get_unaligned_le16(req->wValue),
		get_unaligned_le16(req->wIndex), data,
		get_unaligned_le16(req->wLength), timeout);

	if (err < 0) {
		if (actlen)
			*actlen = 0;
		return err;
	}

	if (actlen)
		*actlen = err;

	return 0;
}

static u8
cdc_ncm_get_nibble(char c)
{
	if (c >= 'a' && c <= 'z')
		return (c - 'a' + 10) & 0xF;
	else if (c >= 'A' && c <= 'Z')
		return (c - 'A' + 10) & 0xF;
	else
		return (c - '0') & 0xF;
}

static int
cdc_ncm_get_ethernet_addr(struct usbnet *dev,
				const struct usb_cdc_ether_desc *e)
{
	char buf[14];
	int tmp;

	tmp = usb_string(dev->udev, e->iMACAddress, buf, sizeof(buf));
	if (tmp < 12)
		return -EINVAL;

	for (tmp = 0; tmp != 12; tmp += 2) {
		dev->net->dev_addr[tmp / 2] =
		  (cdc_ncm_get_nibble(buf[tmp]) << 4) |
		  cdc_ncm_get_nibble(buf[tmp + 1]);
	}
	return 0;
}

static u8
cdc_ncm_setup(struct cdc_ncm_softc *sc)
{
	struct cdc_ncm_device_request req;
	u32 val;
	u8 flags;
	u8 iface_no;
	int err;

	iface_no = sc->sc_control->cur_altsetting->desc.bInterfaceNumber;

	req.bmRequestType = USB_TYPE_CLASS |
		USB_DIR_IN | USB_RECIP_INTERFACE;
	req.bRequest = CDC_NCM_GET_NTB_PARAMETERS;
	req.wValue[0] = 0;
	req.wValue[1] = 0;
	req.wIndex[0] = iface_no;
	req.wIndex[1] = 0;
	req.wLength[0] = sizeof(sc->sc_ncm_parm);
	req.wLength[1] = 0;

	err = cdc_ncm_do_request(sc, &req,
				&sc->sc_ncm_parm, 0, NULL, 1000 /* ms */);
	if (err)
		return 1;

	/* Read correct set of parameters according to device mode */

	sc->sc_rx_ncm.rx_max = get_unaligned_le16(
		sc->sc_ncm_parm.dwNtbInMaxSize);
	sc->sc_rx_ncm.tx_max = get_unaligned_le16(
		sc->sc_ncm_parm.dwNtbOutMaxSize);
	sc->sc_rx_ncm.tx_remainder = get_unaligned_le16(
		sc->sc_ncm_parm.wNdpOutPayloadRemainder);
	sc->sc_rx_ncm.tx_modulus = get_unaligned_le16(
		sc->sc_ncm_parm.wNdpOutDivisor);
	sc->sc_rx_ncm.tx_struct_align = get_unaligned_le16(
		sc->sc_ncm_parm.wNdpOutAlignment);

	if (sc->sc_func_desc != NULL)
		flags = sc->sc_func_desc->bmNetworkCapabilities;
	else
		flags = 0;

	/* compute the maximum number of TX datagrams */
	/* we leave one entry for zero-padding */

	if (flags & CDC_NCM_CAP_NTBINPUTSIZE) {
		val = get_unaligned_le16(sc->sc_ncm_parm.wNtbOutMaxDatagrams);

		sc->sc_rx_ncm.tx_max_datagrams = val;

		if ((val <= 0) || (val > (CDC_NCM_DPT_DATAGRAMS_MAX - 1))) {
			sc->sc_rx_ncm.tx_max_datagrams =
			    (CDC_NCM_DPT_DATAGRAMS_MAX - 1);
		}
	} else {
		sc->sc_rx_ncm.tx_max_datagrams =
		    (CDC_NCM_DPT_DATAGRAMS_MAX - 1);
	}

	/* Verify maximum receive length */

	if (err || (sc->sc_rx_ncm.rx_max < 32) ||
	    (sc->sc_rx_ncm.rx_max > CDC_NCM_RX_MAXLEN)) {
		pr_debug("Using default maximum receive length\n");
		sc->sc_rx_ncm.rx_max = CDC_NCM_RX_MAXLEN;
	}

	/* Verify maximum transmit length */

	if (err || (sc->sc_rx_ncm.tx_max < 32) ||
	    (sc->sc_rx_ncm.tx_max > CDC_NCM_TX_MAXLEN)) {
		pr_debug("Using default maximum transmit length\n");
		sc->sc_rx_ncm.tx_max = CDC_NCM_TX_MAXLEN;
	}

	/*
	 * Verify that the structure alignment is:
	 * - power of two
	 * - not greater than the maximum transmit length
	 * - not less than four bytes
	 */
	val = sc->sc_rx_ncm.tx_struct_align;

	if (err || (val < 4) || (val != ((-val) & val)) ||
	    (val >= sc->sc_rx_ncm.tx_max)) {
		pr_debug("Using default other alignment: 4 bytes\n");
		sc->sc_rx_ncm.tx_struct_align = 4;
	}

	/*
	 * Verify that the payload alignment is:
	 * - power of two
	 * - not greater than the maximum transmit length
	 * - not less than four bytes
	 */
	val = sc->sc_rx_ncm.tx_modulus;

	if (err || (val < 4) || (val != ((-val) & val)) ||
	    (val >= sc->sc_rx_ncm.tx_max)) {
		pr_debug("Using default transmit modulus: 4 bytes\n");
		sc->sc_rx_ncm.tx_modulus = 4;
	}

	/* Verify that the payload remainder */

	if (err || (sc->sc_rx_ncm.tx_remainder >= sc->sc_rx_ncm.tx_modulus)) {
		pr_debug("Using default transmit remainder: 0 bytes\n");
		sc->sc_rx_ncm.tx_remainder = 0;
	}

	/* Additional configuration */

	req.bmRequestType = USB_TYPE_CLASS |
		USB_DIR_OUT | USB_RECIP_INTERFACE;
	req.bRequest = CDC_NCM_SET_NTB_INPUT_SIZE;
	req.wValue[0] = 0;
	req.wValue[1] = 0;
	req.wIndex[0] = iface_no;
	req.wIndex[1] = 0;

	if (flags & CDC_NCM_CAP_NTBINPUTSIZE) {
		req.wLength[0] = 8;
		req.wLength[1] = 0;
		put_unaligned_le32(sc->sc_rx_ncm.rx_max, sc->sc_ncm_value);
		put_unaligned_le16(CDC_NCM_DPT_DATAGRAMS_MAX,
			sc->sc_ncm_value + 4);
		put_unaligned_le16(0, sc->sc_ncm_value + 6);
	} else {
		req.wLength[0] = 4;
		req.wLength[1] = 0;
		put_unaligned_le32(sc->sc_rx_ncm.rx_max, sc->sc_ncm_value);
	}

	err = cdc_ncm_do_request(sc, &req,
		&sc->sc_ncm_value, 0, NULL, 1000 /* ms */);
	if (err)
		pr_debug("Setting input size "
		    "to %u failed.\n", sc->sc_rx_ncm.rx_max);

	req.bmRequestType = USB_TYPE_CLASS |
		USB_DIR_OUT | USB_RECIP_INTERFACE;
	req.bRequest = CDC_NCM_SET_CRC_MODE;
	req.wValue[0] = 0;	/* no CRC */
	req.wValue[1] = 0;
	req.wIndex[0] = iface_no;
	req.wIndex[1] = 0;
	req.wLength[0] = 0;
	req.wLength[1] = 0;

	err = cdc_ncm_do_request(sc, &req,
		NULL, 0, NULL, 1000 /* ms */);
	if (err)
		pr_debug("Setting CRC mode to off failed.\n");

	req.bmRequestType = USB_TYPE_CLASS |
		USB_DIR_OUT | USB_RECIP_INTERFACE;
	req.bRequest = CDC_NCM_SET_NTB_FORMAT;
	req.wValue[0] = 0;	/* NTB-16 */
	req.wValue[1] = 0;
	req.wIndex[0] = iface_no;
	req.wIndex[1] = 0;
	req.wLength[0] = 0;
	req.wLength[1] = 0;

	err = cdc_ncm_do_request(sc, &req,
		NULL, 0, NULL, 1000 /* ms */);
	if (err)
		pr_debug("Setting NTB format to 16-bit failed.\n");

	sc->sc_tx_ncm = sc->sc_rx_ncm;

	return 0;
}

static void
cdc_ncm_find_endpoints(struct cdc_ncm_softc *sc, struct usb_interface *intf)
{
	struct usb_host_endpoint *e;
	u8 ep;

	for (ep = 0; ep < intf->cur_altsetting->desc.bNumEndpoints; ep++) {

		e = intf->cur_altsetting->endpoint + ep;
		switch (e->desc.bmAttributes & 0x03) {
		case USB_ENDPOINT_XFER_INT:
			if (usb_endpoint_dir_in(&e->desc)) {
				if (sc->sc_status_ep == NULL)
					sc->sc_status_ep = e;
			}
			break;

		case USB_ENDPOINT_XFER_BULK:
			if (usb_endpoint_dir_in(&e->desc)) {
				if (sc->sc_in_ep == NULL)
					sc->sc_in_ep = e;
			} else {
				if (sc->sc_out_ep == NULL)
					sc->sc_out_ep = e;
			}
			break;

		default:
			break;
		}
	}
}

static void
cdc_ncm_softc_free(struct cdc_ncm_softc *sc)
{
	if (sc == NULL)
		return;

	del_timer_sync(&sc->sc_tx_timer);

	if (sc->sc_data_claimed) {
		usb_set_intfdata(sc->sc_data, NULL);
		usb_driver_release_interface(driver_of(sc->sc_intf),
		    sc->sc_data);
	}

	if (sc->sc_control_claimed) {
		usb_set_intfdata(sc->sc_control, NULL);
		usb_driver_release_interface(driver_of(sc->sc_intf),
		    sc->sc_control);
	}

	cdc_ncm_tx_set_rem_skb(sc, NULL);

	if (sc->sc_tx_curr_skb != NULL) {
		dev_kfree_skb_any(sc->sc_tx_curr_skb);
		sc->sc_tx_curr_skb = NULL;
	}

	kfree(sc);
}

static int
cdc_ncm_bind(struct usbnet *dev, struct usb_interface *intf)
{
	struct cdc_ncm_softc *sc;
	struct usb_driver *driver;
	u8 *buf;
	int len;
	int temp;
	u8 iface_no;

	/* allocate our softc */

	sc = kmalloc(sizeof(*sc), GFP_KERNEL);
	if (sc == NULL)
		goto error;

	memset(sc, 0, sizeof(*sc));

	init_timer(&sc->sc_tx_timer);
	spin_lock_init(&sc->sc_mtx);
	sc->sc_netdev = dev->net;

	/* store softc pointer in dev's data field */
	dev->data[0] = (unsigned long)sc;

	/* get some pointers */
	driver = driver_of(intf);
	buf = intf->cur_altsetting->extra;
	len = intf->cur_altsetting->extralen;

	sc->sc_udev = dev->udev;
	sc->sc_intf = intf;

	/* parse through descriptors associated with control interface */

	while ((len > 0) && (buf[0] > 2) && (buf[0] <= len)) {

		if (buf[1] != USB_DT_CS_INTERFACE)
			goto advance;

		switch (buf[2]) {
		case USB_CDC_UNION_TYPE:
			if (buf[0] < sizeof(*(sc->sc_union)))
				break;

			sc->sc_union = (const struct usb_cdc_union_desc *)buf;

			sc->sc_control = usb_ifnum_to_if(dev->udev,
			    sc->sc_union->bMasterInterface0);
			sc->sc_data = usb_ifnum_to_if(dev->udev,
			    sc->sc_union->bSlaveInterface0);
			break;

		case USB_CDC_ETHERNET_TYPE:
			if (buf[0] < sizeof(*(sc->sc_ether)))
				break;

			sc->sc_ether = (const struct usb_cdc_ether_desc *)buf;

			dev->hard_mtu =
			    le16_to_cpu(sc->sc_ether->wMaxSegmentSize);

			if (dev->hard_mtu < 256)
				dev->hard_mtu = 256;
			else if (dev->hard_mtu > 2048)
				dev->hard_mtu = 2048;
			break;

		case CDC_NCM_FUNC_DESC_CODE:
			if (buf[0] < sizeof(*(sc->sc_func_desc)))
				break;

			sc->sc_func_desc =
			    (const struct cdc_ncm_func_descriptor *)buf;
			break;

		default:
			break;
		}
advance:
		/* advance to next descriptor */
		temp = buf[0];
		buf += temp;
		len -= temp;
	}

	/* check if we got everything */

	if ((sc->sc_control == NULL) ||
		(sc->sc_data == NULL) ||
		(sc->sc_ether == NULL))
		goto error;

	/* claim interfaces, if any */

	if (sc->sc_data != intf) {
		temp = usb_driver_claim_interface(driver, sc->sc_data, dev);
		if (temp)
			goto error;
		sc->sc_data_claimed = 1;
	}

	if (sc->sc_control != intf) {
		temp = usb_driver_claim_interface(driver, sc->sc_control, dev);
		if (temp)
			goto error;
		sc->sc_control_claimed = 1;
	}

	iface_no = sc->sc_data->cur_altsetting->desc.bInterfaceNumber;

	/* reset data interface */

	temp = usb_set_interface(dev->udev, iface_no, 0);
	if (temp)
		goto error;

	/* initialize data interface */

	if (cdc_ncm_setup(sc))
		goto error;

	/* configure data interface */

	temp = usb_set_interface(dev->udev, iface_no, 1);
	if (temp)
		goto error;

	cdc_ncm_find_endpoints(sc, sc->sc_data);
	cdc_ncm_find_endpoints(sc, sc->sc_control);

	if ((sc->sc_in_ep == NULL) || (sc->sc_out_ep == NULL) ||
	    (sc->sc_status_ep == NULL))
		goto error;

	dev->net->ethtool_ops = &cdc_ncm_ethtool_ops;

	usb_set_intfdata(sc->sc_data, dev);
	usb_set_intfdata(sc->sc_control, dev);
	usb_set_intfdata(sc->sc_intf, dev);

	temp = cdc_ncm_get_ethernet_addr(dev, sc->sc_ether);
	if (temp)
		goto error;

	dev_info(&dev->udev->dev, "MAC-Address: "
	    "0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n",
	    dev->net->dev_addr[0], dev->net->dev_addr[1],
	    dev->net->dev_addr[2], dev->net->dev_addr[3],
	    dev->net->dev_addr[4], dev->net->dev_addr[5]);

	dev->in = usb_rcvbulkpipe(dev->udev,
	    sc->sc_in_ep->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	dev->out = usb_sndbulkpipe(dev->udev,
	    sc->sc_out_ep->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	dev->status = sc->sc_status_ep;
	dev->rx_urb_size = sc->sc_rx_ncm.rx_max;

	/*
	 * We should get an event when the carrier is ON/OFF. Make
	 * sure the carrier is OFF during attach, so that the
	 * network stack does not start IPv6 negotiation and more.
	 */
	netif_carrier_off(dev->net);

	return 0;

error:
	cdc_ncm_softc_free((struct cdc_ncm_softc *)dev->data[0]);
	dev->data[0] = 0;
	dev_info(&dev->udev->dev, "Descriptor failure\n");

	return -ENODEV;
}

static void
cdc_ncm_unbind(struct usbnet *dev, struct usb_interface *intf)
{
	struct cdc_ncm_softc *sc = (void *)dev->data[0];
	struct usb_driver *driver;

	if (sc == NULL)
		return;		/* not setup */

	driver = driver_of(intf);

	usb_set_intfdata(sc->sc_data, NULL);
	usb_set_intfdata(sc->sc_control, NULL);
	usb_set_intfdata(sc->sc_intf, NULL);

	/* claim interfaces, if any */

	if (sc->sc_data_claimed) {
		usb_driver_release_interface(driver, sc->sc_data);
		sc->sc_data_claimed = 0;
	}

	if (sc->sc_control_claimed) {
		usb_driver_release_interface(driver, sc->sc_control);
		sc->sc_control_claimed = 0;
	}
}

static void
cdc_ncm_fill_tx_frame(struct cdc_ncm_softc *sc, struct sk_buff *skb,
				struct sk_buff **ppskb_out)
{
	struct sk_buff *skb_out;

	u32 rem;
	u32 offset;
	u32 last_offset;
	u16 n;

	u8 timeout = (skb == NULL);

	/* if there is a remaining SKB, it gets priority */

	if (skb != NULL)
		swap(skb, sc->sc_tx_rem_skb);

	/*
	 * +----------------+
	 * | skb_out        |
	 * +----------------+
	 *           ^ offset
	 *        ^ last_offset
	 */

	/* check if we are resuming an OUT-SKB */

	if (sc->sc_tx_curr_skb != NULL) {

		/* pop variables */

		skb_out = sc->sc_tx_curr_skb;
		offset = sc->sc_tx_curr_offset;
		last_offset = sc->sc_tx_curr_last_offset;
		n = sc->sc_tx_curr_frame_num;
	} else {

		/* reset variables */

		skb_out = alloc_skb(sc->sc_tx_ncm.tx_max, GFP_ATOMIC);
		if (skb_out == NULL) {
			if (skb != NULL)
				dev_kfree_skb_any(skb);

			goto error;
		}

		/* make room for headers */
		offset = sizeof(sc->sc_tx_ncm.nth16) +
		    sizeof(sc->sc_tx_ncm.ndp16) + sizeof(sc->sc_tx_ncm.dpe16);

		/* store last valid offset before alignment */
		last_offset = offset;

		/* align offset correctly */
		offset = sc->sc_tx_ncm.tx_remainder -
		    ((0UL - offset) & (0UL - sc->sc_tx_ncm.tx_modulus));

		n = 0;
	}

	for (; n != sc->sc_tx_ncm.tx_max_datagrams; n++) {

		/* check if end of transmit buffer is reached */

		if (offset >= sc->sc_tx_ncm.tx_max)
			break;

		/* compute maximum buffer size */

		rem = sc->sc_tx_ncm.tx_max - offset;

		if (skb == NULL) {
			skb = sc->sc_tx_rem_skb;
			sc->sc_tx_rem_skb = NULL;

			/* check for end of SKB's */
			if (skb == NULL)
				break;
		}

		if (skb->len > rem) {

			if (n == 0) {
				/* won't fit */
				dev_kfree_skb_any(skb);
				skb = NULL;
			} else {
				/* no room for SKB - store for later */
				cdc_ncm_tx_set_rem_skb(sc, skb);
				skb = NULL;

				/* loop one more time */
				timeout = 1;
			}
			break;
		}

		memcpy(((u8 *)skb_out->data) + offset, skb->data, skb->len);

		put_unaligned_le16(skb->len,
			sc->sc_tx_ncm.dpe16[n].wDatagramLength);
		put_unaligned_le16(offset,
			sc->sc_tx_ncm.dpe16[n].wDatagramIndex);

		/* Update offset */
		offset += skb->len;

		/* Store last valid offset before alignment */
		last_offset = offset;

		/* Align offset correctly */
		offset = sc->sc_tx_ncm.tx_remainder -
		    ((0UL - offset) & (0UL - sc->sc_tx_ncm.tx_modulus));

		dev_kfree_skb_any(skb);
		skb = NULL;
	}

	/* free up any dangling skb */
	if (skb != NULL) {
		dev_kfree_skb_any(skb);
		skb = NULL;
	}

	if (n == 0) {
		/* wait for more frames */
		/* push variables */
		sc->sc_tx_curr_skb = skb_out;
		sc->sc_tx_curr_offset = offset;
		sc->sc_tx_curr_last_offset = last_offset;
		sc->sc_tx_curr_frame_num = n;

		goto error;

	} else if ((n != sc->sc_tx_ncm.tx_max_datagrams) && (timeout == 0)) {
		/* wait for more frames */
		/* push variables */
		sc->sc_tx_curr_skb = skb_out;
		sc->sc_tx_curr_offset = offset;
		sc->sc_tx_curr_last_offset = last_offset;
		sc->sc_tx_curr_frame_num = n;

		/* set the pending count */
		if (n < 8)
			sc->sc_tx_timer_pending = 2;

		goto error;
	} else {
		/* frame goes out */
		/* variables will be reset at next call */
	}

	rem = sc->sc_tx_ncm.tx_max - last_offset;

	/* XXX if there is a FORCE SHORT packet flag, use that instead! */
	if (((rem & 63) == 0) && (rem != 0)) {
		/* force short packet */
		*(((u8 *)skb_out->data) + last_offset) = 0;
		last_offset++;
	}

	rem = (sizeof(sc->sc_tx_ncm.ndp16) + (4 * n) + 4);

	put_unaligned_le16(rem, sc->sc_tx_ncm.ndp16.wLength);

	/* zero the rest of the data pointer entries */
	for (; n != CDC_NCM_DPT_DATAGRAMS_MAX; n++) {
		put_unaligned_le16(0, sc->sc_tx_ncm.dpe16[n].wDatagramLength);
		put_unaligned_le16(0, sc->sc_tx_ncm.dpe16[n].wDatagramIndex);
	}

	/* Fill out 16-bit header */
	sc->sc_tx_ncm.nth16.dwSignature[0] = 'N';
	sc->sc_tx_ncm.nth16.dwSignature[1] = 'C';
	sc->sc_tx_ncm.nth16.dwSignature[2] = 'M';
	sc->sc_tx_ncm.nth16.dwSignature[3] = 'H';
	put_unaligned_le16(sizeof(sc->sc_tx_ncm.nth16),
		sc->sc_tx_ncm.nth16.wHeaderLength);
	put_unaligned_le16(last_offset, sc->sc_tx_ncm.nth16.wBlockLength);
	put_unaligned_le16(sc->sc_tx_ncm.tx_seq,
		sc->sc_tx_ncm.nth16.wSequence);
	put_unaligned_le16(sizeof(sc->sc_tx_ncm.nth16),
		sc->sc_tx_ncm.nth16.wNdpIndex);

	sc->sc_tx_ncm.tx_seq++;

	/* Fill out 16-bit frame table header */
	sc->sc_tx_ncm.ndp16.dwSignature[0] = 'N';
	sc->sc_tx_ncm.ndp16.dwSignature[1] = 'C';
	sc->sc_tx_ncm.ndp16.dwSignature[2] = 'M';
	sc->sc_tx_ncm.ndp16.dwSignature[3] = '0';
	/* Reserved: */
	sc->sc_tx_ncm.ndp16.wNextNdpIndex[0] = 0;
	sc->sc_tx_ncm.ndp16.wNextNdpIndex[1] = 0;

	memcpy(skb_out->data, &(sc->sc_tx_ncm.nth16),
	    sizeof(sc->sc_tx_ncm.nth16));
	memcpy(((u8 *)skb_out->data) + sizeof(sc->sc_tx_ncm.nth16),
	    &(sc->sc_tx_ncm.ndp16), sizeof(sc->sc_tx_ncm.ndp16));
	memcpy(((u8 *)skb_out->data) + sizeof(sc->sc_tx_ncm.nth16) +
	    sizeof(sc->sc_tx_ncm.ndp16), &(sc->sc_tx_ncm.dpe16),
	    sizeof(sc->sc_tx_ncm.dpe16));

	/* set frame length */
	skb_put(skb_out, last_offset);

	/* return SKB */
	sc->sc_tx_curr_skb = NULL;

	*ppskb_out = skb_out;

	return;

error:
	*ppskb_out = NULL;
}

static void
cdc_ncm_tx_timeout_start(struct cdc_ncm_softc *sc)
{
	/* start timer, if not already started */
	if (timer_pending(&sc->sc_tx_timer) == 0) {
		sc->sc_tx_timer.function = &cdc_ncm_tx_timeout;
		sc->sc_tx_timer.data = (unsigned long)sc;
		sc->sc_tx_timer.expires = jiffies + ((HZ + 999) / 1000);
		add_timer(&sc->sc_tx_timer);
	}
}

static void
cdc_ncm_tx_timeout(unsigned long arg)
{
	struct cdc_ncm_softc *sc = (void *)arg;
	u8 restart;

	spin_lock(&sc->sc_mtx);
	if (sc->sc_tx_timer_pending != 0) {
		sc->sc_tx_timer_pending--;
		restart = 1;
	} else {
		restart = 0;
	}
	spin_unlock(&sc->sc_mtx);

	if (restart)
		cdc_ncm_tx_timeout_start(sc);
	else if (sc->sc_netdev != NULL)
		usbnet_start_xmit(NULL, sc->sc_netdev);
}

static void
cdc_ncm_tx_set_rem_skb(struct cdc_ncm_softc *sc, struct sk_buff *skb)
{
	if (sc->sc_tx_rem_skb != NULL)
		dev_kfree_skb_any(sc->sc_tx_rem_skb);

	sc->sc_tx_rem_skb = skb;
}

static struct sk_buff *
cdc_ncm_tx_fixup(struct usbnet *dev, struct sk_buff *skb, gfp_t flags)
{
	struct sk_buff *skb_out;
	struct cdc_ncm_softc *sc;

	u8 need_timer;

	/*
	 * The ethernet API we are using does not support transmitting
	 * multiple ethernet frames in a single call. This driver will
	 * accumulate multiple ethernet frames and send out a larger
	 * USB frame when the USB buffer is full, or a single jiffies
	 * timeout happens.
	 */
	sc = (struct cdc_ncm_softc *)dev->data[0];
	if (sc == NULL)
		goto error;

	/* The following function will setup the skb */
	spin_lock(&sc->sc_mtx);
	cdc_ncm_fill_tx_frame(sc, skb, &skb_out);
	need_timer = (sc->sc_tx_curr_skb != NULL);
	spin_unlock(&sc->sc_mtx);

	/* Start timer, if there is a remaining skb */
	if (need_timer)
		cdc_ncm_tx_timeout_start(sc);

	if (skb_out == NULL) {
		/* compensate the usbnet drop count increment */
		dev->net->stats.tx_dropped--;
	}

	return skb_out;

error:
	if (skb != NULL)
		dev_kfree_skb_any(skb);

	return NULL;
}

static int
cdc_ncm_rx_fixup(struct usbnet *dev, struct sk_buff *skb_in)
{
	struct sk_buff *skb;
	struct cdc_ncm_softc *sc;
	int sumdata;
	int sumlen;
	int actlen;
	int temp;
	int nframes;
	int x;
	int offset;

	sc = (struct cdc_ncm_softc *)dev->data[0];
	if (sc == NULL)
		goto done;

	actlen = skb_in->len;
	sumlen = CDC_NCM_RX_MAXLEN;

	pr_debug("received %u bytes in 1 frame\n", actlen);

	if (actlen < (sizeof(sc->sc_rx_ncm.nth16) +
	    sizeof(sc->sc_rx_ncm.ndp16))) {
		pr_debug("frame too short\n");
		goto done;
	}

	memcpy(&(sc->sc_rx_ncm.nth16), ((u8 *)skb_in->data),
	    sizeof(sc->sc_rx_ncm.nth16));

	if ((sc->sc_rx_ncm.nth16.dwSignature[0] != 'N') ||
		(sc->sc_rx_ncm.nth16.dwSignature[1] != 'C') ||
		(sc->sc_rx_ncm.nth16.dwSignature[2] != 'M') ||
		(sc->sc_rx_ncm.nth16.dwSignature[3] != 'H')) {
		pr_debug("invalid HDR signature\n");
		goto done;
	}
	temp = get_unaligned_le16(sc->sc_rx_ncm.nth16.wBlockLength);
	if (temp > sumlen) {
		pr_debug("unsupported block length %u/%u\n",
		    temp, sumlen);
		goto done;
	}
	temp = get_unaligned_le16(sc->sc_rx_ncm.nth16.wNdpIndex);
	if ((temp + sizeof(sc->sc_rx_ncm.ndp16)) > actlen) {
		pr_debug("invalid DPT index\n");
		goto done;
	}

	memcpy(&(sc->sc_rx_ncm.ndp16), ((u8 *)skb_in->data) + temp,
	    sizeof(sc->sc_rx_ncm.ndp16));

	if ((sc->sc_rx_ncm.ndp16.dwSignature[0] != 'N') ||
		(sc->sc_rx_ncm.ndp16.dwSignature[1] != 'C') ||
		(sc->sc_rx_ncm.ndp16.dwSignature[2] != 'M') ||
		(sc->sc_rx_ncm.ndp16.dwSignature[3] != '0')) {

		pr_debug("invalid DPT signature\n");
		goto done;
	}
	nframes = get_unaligned_le16(sc->sc_rx_ncm.ndp16.wLength) / 4;

	/* Subtract size of header and last zero padded entry */
	if (nframes >= (2 + 1))
		nframes -= (2 + 1);
	else
		nframes = 0;

	pr_debug("nframes = %u\n", nframes);

	temp += sizeof(sc->sc_rx_ncm.ndp16);

	if ((temp + (4 * nframes)) > actlen) {
		pr_debug("Invalid nframes = %d\n", nframes);
		goto done;
	}

	if (nframes > CDC_NCM_DPT_DATAGRAMS_MAX) {
		pr_debug("Truncating number of frames from %u to %u\n",
		    nframes, CDC_NCM_DPT_DATAGRAMS_MAX);
		nframes = CDC_NCM_DPT_DATAGRAMS_MAX;
	}

	memcpy(&(sc->sc_rx_ncm.dpe16), ((u8 *)skb_in->data) + temp,
	    (4 * nframes));

	sumdata = 0;

	for (x = 0; x != nframes; x++) {

		offset = get_unaligned_le16(
			sc->sc_rx_ncm.dpe16[x].wDatagramIndex);
		temp = get_unaligned_le16(
			sc->sc_rx_ncm.dpe16[x].wDatagramLength);

		if ((offset == 0) || (temp == 0))
			continue;

		/* sanity checking */

		if ((offset + temp) > actlen)
			skb = NULL;
		else if (temp > 2048)
			skb = NULL;
		else if (temp > 14)
			skb = skb_clone(skb_in, GFP_ATOMIC);
		else
			skb = NULL;

		pr_debug("frame %u, offset = %u, length = %u, skb = %p\n",
		    x, offset, temp, skb);

		sumdata += temp;

		/* check if we have a buffer */
		if (skb != NULL) {

			skb->len = temp;
			skb->data = ((u8 *)skb_in->data) + offset;
			skb_set_tail_pointer(skb, temp);
			usbnet_skb_return(dev, skb);
		} else {
			pr_debug("invalid frame detected (ignored)\n");
		}
	}

	pr_debug("Efficiency: %u/%u bytes\n", sumdata, actlen);
done:
	/* compensate for increment of RX error counter in usbnet.c */
	dev->net->stats.rx_errors--;
	return 0;
}

static void
cdc_ncm_dumpspeed(struct cdc_ncm_softc *sc, u32 *ptr)
{
	/* TODO */
}

static void
cdc_ncm_status(struct usbnet *dev, struct urb *urb)
{
	struct cdc_ncm_softc *sc;
	struct usb_cdc_notification *event;

	sc = (struct cdc_ncm_softc *)dev->data[0];

	if (urb->actual_length < sizeof(*event))
		return;

	/* test for split data in 8-byte chunks */

	if (test_and_clear_bit(EVENT_STS_SPLIT, &dev->flags)) {
		cdc_ncm_dumpspeed(sc, (u32 *) urb->transfer_buffer);
		return;
	}

	event = urb->transfer_buffer;
	switch (event->bNotificationType) {
	case USB_CDC_NOTIFY_NETWORK_CONNECTION:
		sc->sc_connected = event->wValue;
		if (netif_msg_timer(dev)) {
			dev_dbg(&dev->udev->dev, "CDC: carrier %s\n",
				sc->sc_connected ? "on" : "off");
		}

		if (sc->sc_connected)
			netif_carrier_on(dev->net);
		else
			netif_carrier_off(dev->net);
		break;

	case USB_CDC_NOTIFY_SPEED_CHANGE:	/* tx/rx rates */
		if (netif_msg_timer(dev)) {
			dev_dbg(&dev->udev->dev, "CDC: speed "
				"change (len %d)\n",
				urb->actual_length);
		}

		if (urb->actual_length < (sizeof(*event) + 8))
			set_bit(EVENT_STS_SPLIT, &dev->flags);
		else
			cdc_ncm_dumpspeed(sc, (u32 *) &event[1]);
		break;

	default:
		dev_err(&dev->udev->dev, "CDC: unexpected "
			"notification 0x%02x!\n",
			event->bNotificationType);
		break;
	}
}

static int
cdc_ncm_check_connect(struct usbnet *dev)
{
	struct cdc_ncm_softc *sc;
	sc = (struct cdc_ncm_softc *)dev->data[0];
	if (sc == NULL)
		return 1;	/* disconnected */

	return !sc->sc_connected;
}

static int
cdc_ncm_probe(struct usb_interface *udev, const struct usb_device_id *prod)
{
	int err;

	err = usbnet_probe(udev, prod);
	
	/*HP SamLin@20101216, Begin: Added for registering external modem device for ste u300 modem*/
	steu300_exmdm_register();
	/*HP SamLin@20101216, End: Added for registering external modem device for ste u300 modem*/
	
	return err;
}

static int
cdc_ncm_suspend(struct usb_interface *intf, pm_message_t message)
{
	return usbnet_suspend(intf, message);
}

static int
cdc_ncm_resume(struct usb_interface *intf)
{
	return usbnet_resume(intf);
}

static void
cdc_ncm_disconnect(struct usb_interface *intf)
{
	struct usbnet *dev;
	struct cdc_ncm_softc *sc;

	dev = usb_get_intfdata(intf);
	if (dev == NULL)
		return;		/* already disconnected */

	sc = (struct cdc_ncm_softc *)dev->data[0];
	if (sc == NULL)
		return;		/* should not happen */

	usbnet_disconnect(intf);

	cdc_ncm_softc_free(sc);
	
	/*HP SamLin@20101216, Begin: Added for registering external modem device for ste u300 modem*/
	steu300_exmdm_unregister();
	/*HP SamLin@20101216, End: Added for registering external modem device for ste u300 modem*/
}

static const struct driver_info cdc_ncm_info = {
	.description = "CDC NCM",
	.flags = FLAG_NO_SETINT | FLAG_WWAN,
	.check_connect = cdc_ncm_check_connect,
	.bind = cdc_ncm_bind,
	.unbind = cdc_ncm_unbind,
	.status = cdc_ncm_status,
	.rx_fixup = cdc_ncm_rx_fixup,
	.tx_fixup = cdc_ncm_tx_fixup,
};

static struct usb_driver cdc_ncm_driver = {
	.name = "cdc_ncm",
	.id_table = cdc_devs,
	.probe = cdc_ncm_probe,
	.disconnect = cdc_ncm_disconnect,
	.suspend = cdc_ncm_suspend,
	.resume = cdc_ncm_resume,
	.supports_autosuspend = 1,
};

static struct ethtool_ops cdc_ncm_ethtool_ops = {
	.get_drvinfo = cdc_ncm_get_drvinfo,
	.get_link = usbnet_get_link,
	.get_msglevel = usbnet_get_msglevel,
	.set_msglevel = usbnet_set_msglevel,
	.get_settings = usbnet_get_settings,
	.set_settings = usbnet_set_settings,
	.nway_reset = usbnet_nway_reset,
};

static int __init
cdc_ncm_init(void)
{
	return usb_register(&cdc_ncm_driver);
}

module_init(cdc_ncm_init);

static void __exit
cdc_ncm_exit(void)
{
	usb_deregister(&cdc_ncm_driver);
}

module_exit(cdc_ncm_exit);

MODULE_AUTHOR("Hans Petter Selasky");
MODULE_DESCRIPTION("CDC NCM");
MODULE_LICENSE("Dual BSD/GPL");
