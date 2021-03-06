/* linux/include/asm-arm/arch-s3c2410/regs-udc.h
 *
 * Copyright (C) 2008 Samsung Electronics
 * Copyright (C) 2004 Herbert Poetzl <herbert@13thfloor.at>
 *
 * This include file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
*/

#ifndef __ASM_ARCH_REGS_USB_HS_OTG_H
#define __ASM_ARCH_REGS_USB_HS_OTG_H

/* USB2.0 OTG Controller register */
#define S3C_USBOTG_PHYREG(x)	((x) + 0x100000 /* S3C64XX_VA_OTGSFR */)
#define S3C_USBOTG_PHYPWR		S3C_USBOTG_PHYREG(0x0)
#define S3C_USBOTG_PHYCLK		S3C_USBOTG_PHYREG(0x4)
#define S3C_USBOTG_RSTCON		S3C_USBOTG_PHYREG(0x8)

/* USB2.0 OTG Controller register */
/* Core Global Registers */
#define S3C_USBOTGREG(x)		((x) /*+ S3C64XX_VA_OTG */)
/* OTG Control & Status */
#define S3C_UDC_OTG_GOTGCTL		S3C_USBOTGREG(0x000)
/* OTG Interrupt */
#define S3C_UDC_OTG_GOTGINT		S3C_USBOTGREG(0x004)
/* Core AHB Configuration */
#define S3C_UDC_OTG_GAHBCFG		S3C_USBOTGREG(0x008)
/* Core USB Configuration */
#define S3C_UDC_OTG_GUSBCFG		S3C_USBOTGREG(0x00C)
/* Core Reset */
#define S3C_UDC_OTG_GRSTCTL		S3C_USBOTGREG(0x010)
/* Core Interrupt */
#define S3C_UDC_OTG_GINTSTS		S3C_USBOTGREG(0x014)
/* Core Interrupt Mask */
#define S3C_UDC_OTG_GINTMSK		S3C_USBOTGREG(0x018)
/* Receive Status Debug Read/Status Read */
#define S3C_UDC_OTG_GRXSTSR		S3C_USBOTGREG(0x01C)
/* Receive Status Debug Pop/Status Pop */
#define S3C_UDC_OTG_GRXSTSP		S3C_USBOTGREG(0x020)
/* Receive FIFO Size */
#define S3C_UDC_OTG_GRXFSIZ		S3C_USBOTGREG(0x024)
/* Non-Periodic Transmit FIFO Size */
#define S3C_UDC_OTG_GNPTXFSIZ	S3C_USBOTGREG(0x028)
/* Non-Periodic Transmit FIFO/Queue Status */
#define S3C_UDC_OTG_GNPTXSTS	S3C_USBOTGREG(0x02C)

/* Host Periodic Transmit FIFO Size */
#define S3C_UDC_OTG_HPTXFSIZ	S3C_USBOTGREG(0x100)
/* Device Periodic Transmit FIFO-1 Size */
#define S3C_UDC_OTG_DPTXFSIZ1	S3C_USBOTGREG(0x104)
/* Device Periodic Transmit FIFO-2 Size */
#define S3C_UDC_OTG_DPTXFSIZ2	S3C_USBOTGREG(0x108)
/* Device Periodic Transmit FIFO-3 Size */
#define S3C_UDC_OTG_DPTXFSIZ3	S3C_USBOTGREG(0x10C)
/* Device Periodic Transmit FIFO-4 Size */
#define S3C_UDC_OTG_DPTXFSIZ4	S3C_USBOTGREG(0x110)
/* Device Periodic Transmit FIFO-5 Size */
#define S3C_UDC_OTG_DPTXFSIZ5	S3C_USBOTGREG(0x114)
/* Device Periodic Transmit FIFO-6 Size */
#define S3C_UDC_OTG_DPTXFSIZ6	S3C_USBOTGREG(0x118)
/* Device Periodic Transmit FIFO-7 Size */
#define S3C_UDC_OTG_DPTXFSIZ7	S3C_USBOTGREG(0x11C)
/* Device Periodic Transmit FIFO-8 Size */
#define S3C_UDC_OTG_DPTXFSIZ8	S3C_USBOTGREG(0x120)
/* Device Periodic Transmit FIFO-9 Size */
#define S3C_UDC_OTG_DPTXFSIZ9	S3C_USBOTGREG(0x124)
/* Device Periodic Transmit FIFO-10 Size */
#define S3C_UDC_OTG_DPTXFSIZ10	S3C_USBOTGREG(0x128)
/* Device Periodic Transmit FIFO-11 Size */
#define S3C_UDC_OTG_DPTXFSIZ11	S3C_USBOTGREG(0x12C)
/* Device Periodic Transmit FIFO-12 Size */
#define S3C_UDC_OTG_DPTXFSIZ12	S3C_USBOTGREG(0x130)
/* Device Periodic Transmit FIFO-13 Size */
#define S3C_UDC_OTG_DPTXFSIZ13	S3C_USBOTGREG(0x134)
/* Device Periodic Transmit FIFO-14 Size */
#define S3C_UDC_OTG_DPTXFSIZ14	S3C_USBOTGREG(0x138)
/* Device Periodic Transmit FIFO-15 Size */
#define S3C_UDC_OTG_DPTXFSIZ15	S3C_USBOTGREG(0x13C)

/* Host Mode Registers
 * Host Global Registers */
/* Host Configuration */
#define S3C_UDC_OTG_HCFG		S3C_USBOTGREG(0x400)
/* Host Frame Interval */
#define S3C_UDC_OTG_HFIR		S3C_USBOTGREG(0x404)
/* Host Frame Number/Frame Time Remaining */
#define S3C_UDC_OTG_HFNUM		S3C_USBOTGREG(0x408)
/* Host Periodic Transmit FIFO/Queue Status */
#define S3C_UDC_OTG_HPTXSTS		S3C_USBOTGREG(0x410)
/* Host All Channels Interrupt */
#define S3C_UDC_OTG_HAINT		S3C_USBOTGREG(0x414)
/* Host All Channels Interrupt Mask */
#define S3C_UDC_OTG_HAINTMSK	S3C_USBOTGREG(0x418)

/* Host Port Control & Status Registers */
#define S3C_UDC_OTG_HPRT		S3C_USBOTGREG(0x440)

/* Host Channel-Specific Registers */
/* Host Channel-0 Characteristics */
#define  S3C_UDC_OTG_HCCHAR0	S3C_USBOTGREG(0x500)
/* Host Channel-0 Split Control */
#define  S3C_UDC_OTG_HCSPLT0	S3C_USBOTGREG(0x504)
/* Host Channel-0 Interrupt */
#define  S3C_UDC_OTG_HCINT0		S3C_USBOTGREG(0x508)
/* Host Channel-0 Interrupt Mask */
#define  S3C_UDC_OTG_HCINTMSK0	S3C_USBOTGREG(0x50C)
/* Host Channel-0 Transfer Size */
#define  S3C_UDC_OTG_HCTSIZ0	S3C_USBOTGREG(0x510)
/* Host Channel-0 DMA Address */
#define  S3C_UDC_OTG_HCDMA0		S3C_USBOTGREG(0x514)

/* Device Mode Registers
 * Device Global Registers */
/* Device Configuration */
#define  S3C_UDC_OTG_DCFG		S3C_USBOTGREG(0x800)
/* Device Control */
#define  S3C_UDC_OTG_DCTL		S3C_USBOTGREG(0x804)
/* Device Status */
#define  S3C_UDC_OTG_DSTS		S3C_USBOTGREG(0x808)
/* Device IN Endpoint Common Interrupt Mask */
#define  S3C_UDC_OTG_DIEPMSK	S3C_USBOTGREG(0x810)
/* Device OUT Endpoint Common Interrupt Mask */
#define  S3C_UDC_OTG_DOEPMSK	S3C_USBOTGREG(0x814)
/* Device All Endpoints Interrupt */
#define  S3C_UDC_OTG_DAINT		S3C_USBOTGREG(0x818)
/* Device All Endpoints Interrupt Mask */
#define  S3C_UDC_OTG_DAINTMSK	S3C_USBOTGREG(0x81C)
/* Device IN Token Sequence Learning Queue Read 1 */
#define  S3C_UDC_OTG_DTKNQR1	S3C_USBOTGREG(0x820)
/* Device IN Token Sequence Learning Queue Read 2 */
#define  S3C_UDC_OTG_DTKNQR2	S3C_USBOTGREG(0x824)
/* Device VBUS Discharge Time */
#define  S3C_UDC_OTG_DVBUSDIS	S3C_USBOTGREG(0x828)
/* Device VBUS Pulsing Time */
#define  S3C_UDC_OTG_DVBUSPULSE	S3C_USBOTGREG(0x82C)
/* Device IN Token Sequence Learning Queue Read 3 */
#define  S3C_UDC_OTG_DTKNQR3	S3C_USBOTGREG(0x830)
/* Device IN Token Sequence Learning Queue Read 4 */
#define  S3C_UDC_OTG_DTKNQR4	S3C_USBOTGREG(0x834)

/* Device Logical IN Endpoint-Specific Registers */
/* Device IN Endpoint 0 Control */
#define  S3C_UDC_OTG_DIEPCTL0	S3C_USBOTGREG(0x900)
/* Device IN Endpoint 0 Interrupt */
#define  S3C_UDC_OTG_DIEPINT0	S3C_USBOTGREG(0x908)
/* Device IN Endpoint 0 Transfer Size */
#define  S3C_UDC_OTG_DIEPTSIZ0	S3C_USBOTGREG(0x910)
/* Device IN Endpoint 0 DMA Address */
#define  S3C_UDC_OTG_DIEPDMA0	S3C_USBOTGREG(0x914)

/* Device IN Endpoint 2 Control */
#define  S3C_UDC_OTG_DIEPCTL2	S3C_USBOTGREG(0x940)
/* Device IN Endpoint 2 Interrupt */
#define  S3C_UDC_OTG_DIEPINT2	S3C_USBOTGREG(0x948)
/* Device IN Endpoint 2 Transfer Size */
#define  S3C_UDC_OTG_DIEPTSIZ2	S3C_USBOTGREG(0x950)
/* Device IN Endpoint 2 DMA Address */
#define  S3C_UDC_OTG_DIEPDMA2	S3C_USBOTGREG(0x954)

/* Device IN Endpoint 3 Control */
#define  S3C_UDC_OTG_DIEPCTL3	S3C_USBOTGREG(0x960)
/* Device IN Endpoint 3 Interrupt */
#define  S3C_UDC_OTG_DIEPINT3	S3C_USBOTGREG(0x968)
/* Device IN Endpoint 3 Transfer Size */
#define  S3C_UDC_OTG_DIEPTSIZ3	S3C_USBOTGREG(0x970)
/* Device IN Endpoint 3 DMA Address */
#define  S3C_UDC_OTG_DIEPDMA3	S3C_USBOTGREG(0x974)

/* Device Logical OUT Endpoint-Specific Registers */
/* Device OUT Endpoint 0 Control */
#define  S3C_UDC_OTG_DOEPCTL0	S3C_USBOTGREG(0xB00)
/* Device OUT Endpoint 0 Interrupt */
#define  S3C_UDC_OTG_DOEPINT0	S3C_USBOTGREG(0xB08)
/* Device OUT Endpoint 0 Transfer Size */
#define  S3C_UDC_OTG_DOEPTSIZ0	S3C_USBOTGREG(0xB10)
/* Device OUT Endpoint 0 DMA Address */
#define  S3C_UDC_OTG_DOEPDMA0	S3C_USBOTGREG(0xB14)

/* Device OUT Endpoint 1 Control */
#define  S3C_UDC_OTG_DOEPCTL1	S3C_USBOTGREG(0xB20)
/* Device OUT Endpoint 1 Interrupt */
#define  S3C_UDC_OTG_DOEPINT1	S3C_USBOTGREG(0xB28)
/* Device OUT Endpoint 1 Transfer Size */
#define  S3C_UDC_OTG_DOEPTSIZ1	S3C_USBOTGREG(0xB30)
/* Device OUT Endpoint 1 DMA Address */
#define  S3C_UDC_OTG_DOEPDMA1	S3C_USBOTGREG(0xB34)

/* Endpoint FIFO address */
#define S3C_UDC_OTG_EP0_FIFO	S3C_USBOTGREG(0x1000)
#define S3C_UDC_OTG_EP1_FIFO	S3C_USBOTGREG(0x2000)
#define S3C_UDC_OTG_EP2_FIFO	S3C_USBOTGREG(0x3000)
#define S3C_UDC_OTG_EP3_FIFO	S3C_USBOTGREG(0x4000)
#define S3C_UDC_OTG_EP4_FIFO	S3C_USBOTGREG(0x5000)
#define S3C_UDC_OTG_EP5_FIFO	S3C_USBOTGREG(0x6000)
#define S3C_UDC_OTG_EP6_FIFO	S3C_USBOTGREG(0x7000)
#define S3C_UDC_OTG_EP7_FIFO	S3C_USBOTGREG(0x8000)
#define S3C_UDC_OTG_EP8_FIFO	S3C_USBOTGREG(0x9000)

/* S3C_USBOTG_PHYPWR */
#define OTG_ENABLE			(0x0<<4)
#define OTG_DISABLE			(0x1<<4)
#define ANALOG_PWR_UP		(0x0<<3)
#define ANALOG_PWR_DOWN		(0x1<<3)
#define SUSPEND_DISABLE		(0x0<<0)
#define SUSPEND_ENABLE		(0x1<<0)

/* S3C_USBOTG_PHYCLK */
#define REF_CLK_CRYSTAL		(0x0<<5)
#define REF_CLK_OSCC		(0x1<<5)

/* S3C_USBOTG_RSTCON */
#define SW_RST_OFF			(0x0<<0)
#define SW_RST_ON			(0x1<<0)

/* S3C_UDC_OTG_GOTGCTL */
#define B_SESSION_VALID		(0x1<<19)
#define A_SESSION_VALID		(0x1<<18)

/* S3C_UDC_OTG_GAHBCFG */
#define PTXFE_HALF			(0x0<<8)
#define PTXFE_ZERO			(0x1<<8)
#define NPTXFE_HALF			(0x0<<7)
#define NPTXFE_ZERO			(0x1<<7)
#define MODE_SLAVE			(0x0<<5)
#define MODE_DMA			(0x1<<5)
#define BURST_SINGLE		(0x0<<1)
#define BURST_INCR			(0x1<<1)
#define BURST_INCR4			(0x3<<1)
#define BURST_INCR8			(0x5<<1)
#define BURST_INCR16		(0x7<<1)
#define GBL_INT_UNMASK		(0x1<<0)
#define GBL_INT_MASK		(0x0<<0)

/* S3C_UDC_OTG_GUSBCFG */
#define PHY_CLK_480M		(0x0<<15)
#define PHY_CLK_48M			(0x1<<15)
#define TXFIFO_RE_DIS		(0x0<<14)
#define TXFIFO_RE_EN		(0x1<<14)
#define TURN_AROUND			(0x5<<10)
#define HNP_DISABLE			(0x0<<9)
#define HNP_ENABLE			(0x1<<9)
#define SRP_DISABLE			(0x0<<8)
#define SRP_ENABLE			(0x1<<8)
#define ULPI_DDR			(0x0<<7)
#define HS_UTMI				(0x0<<6)
#define INTERF_UTMI			(0x0<<4)
#define INTERF_ULPI			(0x1<<4)
#define PHY_INTERF_8		(0x0<<3)
#define PHY_INTERF_16		(0x1<<3)
#define TIME_OUT_CAL		(0x7<<0)

/* S3C_UDC_OTG_GRSTCTL */
#define AHB_MASTER_IDLE		(1u<<31)
#define CORE_SOFT_RESET		(0x1<<0)

/* S3C_UDC_OTG_GINTSTS/S3C_UDC_OTG_GINTMSK core interrupt register */
#define INT_RESUME			(0x1<<31)
#define INT_DISCONN			(0x1<<29)
#define INT_CONN_CNG		(0x1<<28)
#define INT_OUT_EP			(0x1<<19)
#define INT_IN_EP			(0x1<<18)
#define INT_ENUMDONE		(0x1<<13)
#define INT_RESET			(0x1<<12)
#define INT_SUSPEND			(0x1<<11)
#define INT_EARLY_SUSPEND	(0x1<<10)
#define INT_TX_FIFO_EMPTY	(0x1<<5)
#define INT_RX_FIFO_NOT_EMPTY	(0x1<<4)
#define INT_SOF				(0x1<<3)
#define INT_DEV_MODE		(0x0<<0)
#define INT_HOST_MODE		(0x1<<1)

#define FULL_SPEED_CONTROL_PKT_SIZE		8
#define FULL_SPEED_BULK_PKT_SIZE		64

#define HIGH_SPEED_CONTROL_PKT_SIZE		64
#define HIGH_SPEED_BULK_PKT_SIZE		512

/* S3C_UDC_OTG_DSTS */
#define RX_FIFO_SIZE		(2048<<0)
#define NPTX_FIFO_START_ADDR	(RX_FIFO_SIZE<<0)
#define NPTX_FIFO_SIZE		(2048<<16)
#define PTX_FIFO_SIZE		(2048<<16)
#define USB_HIGH_30_60MHZ	(0x0<<1)
#define USB_FULL_30_60MHZ	(0x1<<1)
#define USB_LOW_6MHZ		(0x2<<1)
#define USB_FULL_48MHZ		(0x3<<1)

/* S3C_UDC_OTG_GRXSTSP */
#define BYTE_COUNT(x)		((x & (0x7FF<<4)) >> 4)
#define PKT_STS(x)			((x & (0xF<<17)) >> 17)
#define EP_NUM(x)			(x & 0xF)

#define OUT_PKT_RECEIVED	(0x2)
#define OUT_COMPLELTED		(0x3)
#define SETUP_COMPLETED		(0x4)
#define SETUP_PKT_RECEIVED	(0x6)

/* S3C_UDC_OTG_DCFG */
#define EP_MIS_CNT(x)		(x<<18)
#define DEVICE_ADDR(x)		(x<<4)
#define SPEED_2_HIGH		(0x0<<0)
#define SPEED_2_FULL		(0x1<<0)
#define SPEED_1_LOW			(0x2<<0)
#define SPEED_1_FULL		(0x3<<0)

/* S3C_UDC_OTG_DCTL device control register */
#define NORMAL_OPERATION	(0x1<<0)
#define SOFT_DISCONNECT		(0x1<<1)

/* S3C_UDC_OTG_DSTS */
#define ENUM_SPEED(x)		(x & (0x3<<1))
#define FRAME_CNT(x)		(x & (0x3ff<<8))

/* S3C_UDC_OTG_DAINT device all endpoint interrupt register */
#define S3C_UDC_INT_IN_EP0	(0x1<<0)
#define S3C_UDC_INT_IN_EP2	(0x1<<2)
#define S3C_UDC_INT_IN_EP3	(0x1<<3)
#define S3C_UDC_INT_OUT_EP0	(0x1<<16)
#define S3C_UDC_INT_OUT_EP1	(0x1<<17)
#define S3C_UDC_INT_OUT_EP4	(0x1<<20)

/* S3C_UDC_OTG_DIEPCTL0/DOEPCTL0 device control
   IN/OUT endpoint 0 control register */
#define DEPCTL_EPENA		(0x1<<31)
#define DEPCTL_EPDIS		(0x1<<30)
#define DEPCTL_SNAK			(0x1<<27)
#define DEPCTL_CNAK			(0x1<<26)
#define DEPCTL_CTRL_TYPE	(0x0<<18)
#define DEPCTL_ISO_TYPE		(0x1<<18)
#define DEPCTL_BULK_TYPE	(0x2<<18)
#define DEPCTL_INTR_TYPE	(0x3<<18)
#define DEPCTL_USBACTEP		(0x1<<15)
#define DEPCTL0_MPS_64		(0x0<<0)
#define DEPCTL0_MPS_32		(0x1<<0)
#define DEPCTL0_MPS_16		(0x2<<0)
#define DEPCTL0_MPS_8		(0x3<<0)

/* S3C_UDC_OTG_DIEPINTn */
#define IN_EP_NAK_EFF		(0x1<<6)
#define IN_TK_EPMIS			(0x1<<5)
#define IN_TK_TXFEMP		(0x1<<4)
#define IN_EP_TIMEOUT		(0x1<<3)

/* S3C_UDC_OTG_DOEPINTn */
#define BACK2BACK_SETUP		(0x1<<6)
#define OUT_TK_EP_DIS		(0x1<<4)
#define SETUP_PHASE_DONE	(0x1<<3)

/* S3C_UDC_OTG_DIEPINTn/DOEPINTn */
#define AHB_ERROR			(0x1<<2)
#define EPDISBLD			(0x1<<1)
#define TRANSFER_DONE		(0x1<<0)

/* S3C_UDC_OTG_DIEPTSIZn */
#define PKT_CNT(x)			(x<<19)
#define XFERSIZE(x)			(x<<0)

#endif
