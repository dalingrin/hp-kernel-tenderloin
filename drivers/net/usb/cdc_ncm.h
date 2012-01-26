/*                                                                           
 * cdc_ncm.h                                                                 
 *                                                                           
 * Copyright (C) ST-Ericsson AB 2010                                         
 * Contact: Sjur Braendeland <sjur.brandeland@xxxxxxxxxxxxxx>                
 * Original author:Hans Petter Selasky <hans.petter.selasky@xxxxxxxxxxxxxx>  
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
 */                                                                          
                                                                             
#ifndef _CDC_NCM_H_                                                          
#define	_CDC_NCM_H_                                                         
                                                                             
#include <linux/unaligned/le_byteshift.h>                                    
                                                                             
#define	CDC_IND_SIZE_MAX			32      /* bytes */                           
#define	CDC_NCM_TX_MAXLEN			0x4000	/* bytes */                           
#define	CDC_NCM_RX_MAXLEN			0x4000	/* bytes */                           
/* maximum amount of datagrams in NCM Datagram Pointer Table */              
#define	CDC_NCM_DPT_DATAGRAMS_MAX		32                                      
                                                                             
#define	USB_SUBCLASS_CODE_NETWORK_CONTROL_MODEL	0x0D                        
                                                                             
/*                                                                           
 * USB device request structure.                                             
 *                                                                           
 * The protocol structure definitions use Hungarian style                    
 * like in the NCM specification.                                            
 */                                                                          
struct cdc_ncm_device_request {                                              
	u8	bmRequestType;                                                          
	u8	bRequest;                                                               
	u8	wValue[2];                                                              
	u8	wIndex[2];                                                              
	u8	wLength[2];                                                             
} __attribute__((__packed__));                                               
                                                                             
/* 16-bit NCM Transfer Header */                                             
struct cdc_ncm_nth16 {                                                       
	u8	dwSignature[4];                                                         
	u8	wHeaderLength[2];                                                       
	u8	wSequence[2];                                                           
	u8	wBlockLength[2];                                                        
	u8	wNdpIndex[2];                                                           
} __attribute__((__packed__));                                               
                                                                             
/* 16-bit NCM Datagram Pointer Entry */                                      
struct cdc_ncm_dpe16 {                                                       
	u8	wDatagramIndex[2];                                                      
	u8	wDatagramLength[2];                                                     
} __attribute__((__packed__));                                               
                                                                             
/* 16-bit NCM Datagram Pointer Table */                                      
struct cdc_ncm_ndp16 {                                                       
	u8	dwSignature[4];                                                         
	u8	wLength[2];                                                             
	u8	wNextNdpIndex[2];                                                       
	struct cdc_ncm_dpe16 dpe16[0];                                              
} __attribute__((__packed__));                                               
                                                                             
/* 32-bit NCM Transfer Header */                                             
struct cdc_ncm_nth32 {                                                       
	u8	dwSignature[4];                                                         
	u8	wHeaderLength[2];                                                       
	u8	wSequence[2];                                                           
	u8	dwBlockLength[4];                                                       
	u8	dwNdpIndex[4];                                                          
} __attribute__((__packed__));                                               
                                                                             
/* 32-bit NCM Datagram Pointer Entry */                                      
struct cdc_ncm_dpe32 {                                                       
	u8	dwDatagramIndex[4];                                                     
	u8	dwDatagramLength[4];                                                    
} __attribute__((__packed__));                                               
                                                                             
/* 32-bit NCM Datagram Pointer Table */                                      
struct cdc_ncm_ndp32 {                                                       
	u8	dwSignature[4];                                                         
	u8	wLength[2];                                                             
	u8	wReserved6[2];                                                          
	u8	dwNextNdpIndex[4];                                                      
	u8	dwReserved12[4];                                                        
	struct cdc_ncm_dpe32 dpe32[0];                                              
} __attribute__((__packed__));                                               
                                                                             
/* Communications interface class specific descriptors */                    
#define	CDC_NCM_FUNC_DESC_CODE		0x1A                                      
                                                                             
/* Network Capabilities bit fields */                                        
#define	CDC_NCM_CAP_FILTER		0x01                                          
#define	CDC_NCM_CAP_NETADDRESS		0x02                                      
#define	CDC_NCM_CAP_ENCAP		0x04                                            
#define	CDC_NCM_CAP_MAXDATAGRAMSIZE	0x08                                    
#define	CDC_NCM_CAP_CRCMODE		0x10                                          
#define	CDC_NCM_CAP_NTBINPUTSIZE	0x20                                      
                                                                             
struct cdc_ncm_func_descriptor {                                             
	u8	bLength;                                                                
	u8	bDescriptorType;                                                        
	u8	bDescriptorSubtype;                                                     
	u8	bcdNcmVersion[2];                                                       
	u8	bmNetworkCapabilities;                                                  
} __attribute__((__packed__));                                               
                                                                             
/* Class-Specific Request Codes for NCM subclass */                          
#define	CDC_NCM_SET_ETHERNET_MULTICAST_FILTERS			0x40                    
#define	CDC_NCM_SET_ETHERNET_POWER_MGMT_PATTERN_FILTER		0x41              
#define	CDC_NCM_GET_ETHERNET_POWER_MGMT_PATTERN_FILTER		0x42              
#define	CDC_NCM_SET_ETHERNET_PACKET_FILTER			0x43                        
#define	CDC_NCM_GET_ETHERNET_STATISTIC				0x44                          
#define	CDC_NCM_GET_NTB_PARAMETERS				0x80                              
#define	CDC_NCM_GET_NET_ADDRESS					0x81                                
#define	CDC_NCM_SET_NET_ADDRESS					0x82                                
#define	CDC_NCM_GET_NTB_FORMAT					0x83                                
#define	CDC_NCM_SET_NTB_FORMAT					0x84                                
#define	CDC_NCM_GET_NTB_INPUT_SIZE				0x85                              
#define	CDC_NCM_SET_NTB_INPUT_SIZE				0x86                              
#define	CDC_NCM_GET_MAX_DATAGRAM_SIZE				0x87                            
#define	CDC_NCM_SET_MAX_DATAGRAM_SIZE				0x88                            
#define	CDC_NCM_GET_CRC_MODE					0x89                                  
#define	CDC_NCM_SET_CRC_MODE					0x8A                                  
                                                                             
struct cdc_ncm_parameters {                                                  
	u8	wLength[2];                                                             
	u8	bmNtbFormatsSupported[2];                                               
#define	CDC_NCM_FORMAT_NTB16	0x0001                                        
#define	CDC_NCM_FORMAT_NTB32	0x0002                                        
	u8	dwNtbInMaxSize[4];                                                      
	u8	wNdpInDivisor[2];                                                       
	u8	wNdpInPayloadRemainder[2];                                              
	u8	wNdpInAlignment[2];                                                     
	u8	wReserved14[2];                                                         
	u8	dwNtbOutMaxSize[4];                                                     
	u8	wNdpOutDivisor[2];                                                      
	u8	wNdpOutPayloadRemainder[2];                                             
	u8	wNdpOutAlignment[2];                                                    
	u8	wNtbOutMaxDatagrams[2];                                                 
} __attribute__((__packed__));                                               
                                                                             
/* Class-Specific Notification Codes for NCM subclass */                     
#define	CDC_NCM_NOTIF_NETWORK_CONNECTION	0x00                              
#define	CDC_NCM_NOTIF_RESPONSE_AVAILABLE	0x01                              
#define	CDC_NCM_NOTIF_CONNECTION_SPEED_CHANGE	0x2A                          
                                                                             
struct cdc_ncm {                                                             
	struct cdc_ncm_nth16 nth16;                                                 
	struct cdc_ncm_ndp16 ndp16;                                                 
	struct cdc_ncm_dpe16 dpe16[CDC_NCM_DPT_DATAGRAMS_MAX];                      
	u32 rx_max;                                                                 
	u32 tx_max;                                                                 
	u16 tx_max_datagrams;                                                       
	u16 tx_remainder;                                                           
	u16 tx_modulus;                                                             
	u16 tx_struct_align;                                                        
	u16 tx_seq;                                                                 
};                                                                           
                                                                             
struct cdc_ncm_softc {                                                       
	struct cdc_ncm sc_rx_ncm;                                                   
	struct cdc_ncm sc_tx_ncm;                                                   
	struct cdc_ncm_parameters sc_ncm_parm;                                      
                                                                             
	struct timer_list sc_tx_timer;                                              
                                                                             
	const struct cdc_ncm_func_descriptor *sc_func_desc;                         
	const struct usb_cdc_header_desc *sc_header;                                
	const struct usb_cdc_union_desc *sc_union;                                  
	const struct usb_cdc_ether_desc *sc_ether;                                  
                                                                             
	struct usb_device *sc_udev;                                                 
                                                                             
	struct usb_host_endpoint *sc_in_ep;                                         
	struct usb_host_endpoint *sc_out_ep;                                        
	struct usb_host_endpoint *sc_status_ep;                                     
                                                                             
	struct usb_interface *sc_intf;                                              
	struct usb_interface *sc_control;                                           
	struct usb_interface *sc_data;                                              
	struct sk_buff *sc_tx_curr_skb;                                             
	struct sk_buff *sc_tx_rem_skb;                                              
	struct net_device *sc_netdev;                                               
                                                                             
	spinlock_t sc_mtx;                                                          
                                                                             
	u32 sc_tx_timer_pending;                                                    
	u32 sc_tx_curr_offset;                                                      
	u32 sc_tx_curr_last_offset;                                                 
	u32 sc_tx_curr_frame_num;                                                   
                                                                             
	u32 sc_rx_speed;                                                            
	u32 sc_tx_sped;                                                             
	u16 sc_connected;                                                           
	u8 sc_data_claimed;                                                         
	u8 sc_control_claimed;                                                      
	u8 sc_ncm_value[16];	/* temporary data */                                  
};                                                                           
                                                                             
#endif /* _CDC_NCM_H_ */                                                     