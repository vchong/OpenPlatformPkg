/** @file

  Copyright (c) 2017, Linaro Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <IndustryStandard/Usb.h>
#include <Library/ArmLib.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiDriverEntryPoint.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/UncachedMemoryAllocationLib.h>
#include <Protocol/DwUsb.h>
#include <Protocol/UsbDevice.h>

#include "DwUsb3Dxe.h"

#define FIFO_DIR_TX                  0
#define FIFO_DIR_RX                  1

#define TX_FIFO_ADDR                 0
#define RX_FIFO_ADDR                 0

#define RAM_WIDTH                    8
#define RAM_TX0_DEPTH                2048
#define RAM_TX1_DEPTH                4096
#define RAM_RX_DEPTH                 8192

#define USB_TYPE_LENGTH              16
#define USB_BLOCK_HIGH_SPEED_SIZE    512
#define DATA_SIZE                    32768
#define CMD_SIZE                     512
#define MATCH_CMD_LITERAL(Cmd, Buf) !AsciiStrnCmp (Cmd, Buf, sizeof (Cmd) - 1)

// The time between interrupt polls, in units of 100 nanoseconds
// 10 Microseconds
#define DW_INTERRUPT_POLL_PERIOD     10000

#define DWUSB3_EVENT_BUF_SIZE        256

// Maxpacket size for EP0, defined by USB3 spec
#define USB3_MAX_EP0_SIZE            512

// Maxpacket size for any EP, defined by USB3 spec
#define USB3_MAX_PACKET_SIZE         1024
#define USB2_HS_MAX_PACKET_SIZE      512
#define USB2_FS_MAX_PACKET_SIZE      64

#define USB3_STATE_UNCONNECTED       0
#define USB3_STATE_DEFAULT           1
#define USB3_STATE_ADDRESSED         2
#define USB3_STATE_CONFIGURED        3

EFI_GUID  gDwUsbProtocolGuid = DW_USB_PROTOCOL_GUID;

STATIC DW_USB_PROTOCOL          *DwUsb;

STATIC usb3_pcd_t               gPcd;
STATIC UINT32                   gEventBuf[DWUSB3_EVENT_BUF_SIZE];
STATIC UINT32                   *gEventPtr;

STATIC
VOID
DwUsb3SetFifoSize (
  IN UINT32              Addr,
  IN UINT32              Depth,
  IN UINT32              Dir,
  IN UINT32              FifoNum
  )
{
  UINT32                 Reg;

  if (Dir == FIFO_DIR_TX) {
    Reg = GTXFIFOSIZ (FifoNum);
  } else if (Dir == FIFO_DIR_RX) {
    Reg = GRXFIFOSIZ (FifoNum);
  } else {
    ASSERT (0);
  }
  MmioWrite32 (Reg, FIFOSIZ_DEP (Depth) | FIFOSIZ_ADDR (Addr));
}

STATIC
VOID
DwUsb3SetAddress (
  IN UINT32              Addr
  )
{
  UINT32                 Data;

  Data = MmioRead32 (DCFG) & ~DCFG_DEVADDR_MASK;
  Data |= DCFG_DEVADDR (Addr);
  MmioWrite32 (DCFG, Data);
}

STATIC
VOID
DwUsb3DisableFlushEventbufIntr (
  VOID
  )
{
  UINT32                 Data;

  Data = MmioRead32 (GEVNTSIZ (0)) | GEVNTSIZ_EVNTINTMASK;
  MmioWrite32 (GEVNTSIZ (0), Data);
  Data = MmioRead32 (GEVNTCOUNT (0));
  MmioWrite32 (GEVNTCOUNT (0), Data);
}

STATIC
UINTN
DwUsb3GetEventBufCount (
  VOID
  )
{
  UINT32                 Data;

  Data = MmioRead32 (GEVNTCOUNT (0));
  return GEVNTCOUNT_EVNTCOUNT (Data);
}

STATIC
VOID
DwUsb3UpdateEventBufCount (
  IN UINTN               Cnt
  )
{
  MmioWrite32 (GEVNTCOUNT (0), GEVNTCOUNT_EVNTCOUNT (Cnt));
}

STATIC
UINT32
DwUsb3GetEventBufEvent (
  IN UINTN               Size
  )
{
  UINT32                 Event;

  Event = *gEventPtr++;
  if ((UINT32)(UINTN)gEventPtr >= (UINT32)(UINTN)&gEventBuf + Size) {
    gEventPtr = gEventBuf;
  }
  return Event;
}

STATIC
UINT32
Handshake (
  IN UINT32              Reg,
  IN UINT32              Mask,
  IN UINT32              Done
  )
{
  UINT32                 Timeout = 100000;

  do {
    if ((MmioRead32 (Reg) & Mask) == Done) {
      return 1;
    }
    MicroSecondDelay (1);
    Timeout--;
  } while (Timeout > 0);
  return 0;
}

STATIC
VOID
DwUsb3FillDesc (
  IN usb3_dma_desc_t     *desc,
  IN UINT64              dma_addr,
  IN UINT32              dma_len,
  IN UINT32              stream,
  IN UINT32              type,
  IN UINT32              ctrlbits,
  IN UINT32              own
  )
{
  desc->bptl = (UINT32)(dma_addr & 0xFFFFFFFF);
  desc->bpth = (UINT32)(dma_addr >> 32);
  desc->status = DSCSTS_XFERCNT (dma_len);
  if (type) {
    desc->control = DSCCTL_TRBCTL (type);
  }
  desc->control |= DSCCTL_STRMID_SOFN (stream) | ctrlbits;
  /* must execute this operation at last */
  if (own) {
    desc->control |= DSCCTL_HWO;
  }
}

STATIC
VOID
DwUsb3DepStartNewCfg (
  IN UINT32              EpIdx,
  IN UINT32              RsrcIdx
  )
{
  /* start the command */
  MmioWrite32 (
    DEPCMD (EpIdx),
    DEPCMD_COMMANDPARAM (RsrcIdx) | CMDTYPE_START_NEW_CFG | DEPCMD_CMDACT
    );
  Handshake (DEPCMD (EpIdx), DEPCMD_CMDACT, 0);
}

STATIC
VOID
DwUsb3DepCfg (
  IN UINT32              EpIdx,
  IN UINT32              DepCfg0,
  IN UINT32              DepCfg1,
  IN UINT32              DepCfg2
  )
{
  MmioWrite32 (DEPCMDPAR2 (EpIdx), DepCfg2);
  MmioWrite32 (DEPCMDPAR1 (EpIdx), DepCfg1);
  MmioWrite32 (DEPCMDPAR0 (EpIdx), DepCfg0);
  MmioWrite32 (DEPCMD (EpIdx), CMDTYPE_SET_EP_CFG | DEPCMD_CMDACT);
  Handshake (DEPCMD (EpIdx), DEPCMD_CMDACT, 0);
}

STATIC
VOID
DwUsb3DepXferCfg (
  IN UINT32              EpIdx,
  IN UINT32              DepStrmCfg
  )
{
  MmioWrite32 (DEPCMDPAR0 (EpIdx), DepStrmCfg);
  MmioWrite32 (DEPCMD (EpIdx), CMDTYPE_SET_XFER_CFG | DEPCMD_CMDACT);
  Handshake (DEPCMD (EpIdx), DEPCMD_CMDACT, 0);
}

STATIC
UINT8
DwUsb3DepStartXfer (
  IN UINT32              EpIdx,
  IN UINT64              DmaAddr,
  IN UINT32              StreamOrUf
  )
{
  UINT32                 Data;

  MmioWrite32 (DEPCMDPAR1 (EpIdx), (UINT32)DmaAddr);
  MmioWrite32 (DEPCMDPAR0 (EpIdx), (UINT32)(DmaAddr >> 32));
  MmioWrite32 (
    DEPCMD (EpIdx),
    DEPCMD_STR_NUM_OR_UF (StreamOrUf) | CMDTYPE_START_XFER | DEPCMD_CMDACT
    );
  Handshake (DEPCMD (EpIdx), DEPCMD_CMDACT, 0);
  Data = MmioRead32 (DEPCMD (EpIdx));
  return GET_DEPCMD_XFER_RSRC_IDX(Data);
}

STATIC
VOID
DwUsb3DepStopXfer (
  IN UINT32               EpIdx,
  IN UINT32               Tri
  )
{
  MmioWrite32 (DEPCMDPAR2 (EpIdx), 0);
  MmioWrite32 (DEPCMDPAR1 (EpIdx), 0);
  MmioWrite32 (DEPCMDPAR0 (EpIdx), 0);
  MmioWrite32 (
    DEPCMD (EpIdx),
    DEPCMD_XFER_RSRC_IDX (Tri) | CMDTYPE_END_XFER | DEPCMD_CMDACT
    );
  Handshake (DEPCMD (EpIdx), DEPCMD_CMDACT, 0);
}

VOID
DwUsb3DepUpdateXfer (
  IN UINT32               EpIdx,
  IN UINT32               Tri
  )
{
  MmioWrite32 (
    DEPCMD (EpIdx),
    DEPCMD_XFER_RSRC_IDX (Tri) | CMDTYPE_UPDATE_XFER | DEPCMD_CMDACT
    );
  Handshake (DEPCMD (EpIdx), DEPCMD_CMDACT, 0);
}

STATIC
VOID
DwUsb3DepClearStall (
  IN UINTN            EpIdx
  )
{
  MmioWrite32 (DEPCMD (EpIdx), CMDTYPE_CLR_STALL | DEPCMD_CMDACT);
  Handshake (DEPCMD (EpIdx), DEPCMD_CMDACT, 0);
}


STATIC
VOID
DwUsb3DepSetStall (
  IN UINTN            EpIdx
  )
{
  MmioWrite32 (DEPCMD (EpIdx), CMDTYPE_SET_STALL | DEPCMD_CMDACT);
  Handshake (DEPCMD (EpIdx), DEPCMD_CMDACT, 0);
}

STATIC
VOID
DwUsb3EnableEp (
  IN UINT32                EpIdx,
  IN usb3_pcd_ep_t         *ep
  )
{
  UINT32                   EpIdxNum, Dalepena;

  EpIdxNum = ep->num * 2;
  if (ep->is_in) {
    EpIdxNum += 1;
  }
  Dalepena = MmioRead32 (DALEPENA);
  /* If the EP is already enabled, skip to set it again. */
  if (Dalepena & (1 << EpIdxNum)) {
    return;
  }
  Dalepena |= 1 << EpIdxNum;
  MmioWrite32 (DALEPENA, Dalepena);
}

STATIC
VOID
DwUsb3Ep0Activate (
  IN OUT usb3_pcd_t         *pcd
  )
{
  UINT32                  DiepCfg0, DoepCfg0, DiepCfg1, DoepCfg1;
  UINT32                  DiepCfg2 = 0, DoepCfg2 = 0;

  DiepCfg0 = EPCFG0_EPTYPE (EPTYPE_CONTROL);
  DiepCfg1 = EPCFG1_XFER_NRDY | EPCFG1_XFER_CMPL | EPCFG1_EP_DIR;
  DoepCfg0 = EPCFG0_EPTYPE (EPTYPE_CONTROL);
  DoepCfg1 = EPCFG1_XFER_CMPL | EPCFG1_XFER_NRDY;

  /* Default to MPS of 512 (will reconfigure after ConnectDone event) */
  DiepCfg0 |= EPCFG0_MPS (512);
  DoepCfg0 |= EPCFG0_MPS (512);

  DiepCfg0 |= EPCFG0_TXFNUM (pcd->ep0.tx_fifo_num);

  /* issue DEPCFG command to EP0 OUT */
  DwUsb3DepStartNewCfg (EP_OUT_IDX (0), 0);
  DwUsb3DepCfg (EP_OUT_IDX (0), DoepCfg0, DoepCfg1, DoepCfg2);
  /* issue DEPSTRMCFG command to EP0 OUT */
  DwUsb3DepXferCfg (EP_OUT_IDX (0), 1);  // one stream
  /* issue DEPCFG command to EP0 IN */
  DwUsb3DepCfg (EP_IN_IDX (0), DiepCfg0, DiepCfg1, DiepCfg2);
  /* issue DEPSTRMCFG command to EP0 IN */
  DwUsb3DepXferCfg (EP_IN_IDX (0), 1);  // one stream
  pcd->ep0.active = 1;
}

STATIC
VOID
DwUsb3EpActivate (
  IN OUT usb3_pcd_t         *pcd,
  IN OUT usb3_pcd_ep_t      *ep
  )
{
  UINT32                    EpIdx, DepCfg0, DepCfg1;
  if (ep->is_in) {
    EpIdx = EP_IN_IDX (ep->num);
  } else {
    EpIdx = EP_OUT_IDX (ep->num);
  }

  /* Start a new configurate when enable the first EP. */
  if (!pcd->eps_enabled) {
    pcd->eps_enabled = 1;
    /* Issue DEPCFG command to physical EP1 (logical EP0 IN) first.
     * It resets the core's Tx FIFO mapping table.
     */
    DepCfg0 = EPCFG0_EPTYPE (EPTYPE_CONTROL);
    DepCfg0 |= EPCFG0_CFG_ACTION (CFG_ACTION_MODIFY);
    DepCfg1 = EPCFG1_XFER_CMPL | EPCFG1_XFER_NRDY | EPCFG1_EP_DIR;

    switch (pcd->speed) {
    case USB_SPEED_SUPER:
      DepCfg0 |= EPCFG0_MPS (512);
      break;
    case USB_SPEED_HIGH:
    case USB_SPEED_FULL:
      DepCfg0 |= EPCFG0_MPS (64);
      break;
    case USB_SPEED_LOW:
      DepCfg0 |= EPCFG0_MPS (8);
      break;
    default:
      ASSERT (0);
      break;
    }
    DwUsb3DepCfg (EP_IN_IDX (0), DepCfg0, DepCfg1, 0);
    DwUsb3DepStartNewCfg (EP_OUT_IDX (0), 2);
  }
  /* issue DEPCFG command to EP */
  DepCfg0 = EPCFG0_EPTYPE (ep->type);
  DepCfg0 |= EPCFG0_MPS (ep->maxpacket);
  if (ep->is_in) {
    DepCfg0 |= EPCFG0_TXFNUM (ep->tx_fifo_num);
  }
  DepCfg0 |= EPCFG0_BRSTSIZ (ep->maxburst);
  DepCfg1 = EPCFG1_EP_NUM (ep->num);
  if (ep->is_in) {
    DepCfg1 |= EPCFG1_EP_DIR;
  } else {
    DepCfg1 |= EPCFG1_XFER_CMPL;
  }
  DwUsb3DepCfg (EpIdx, DepCfg0, DepCfg1, 0);
  /* issue DEPSTRMCFG command to EP */
  DwUsb3DepXferCfg (EpIdx, 1);
  DwUsb3EnableEp (EpIdx, ep);
  ep->active = 1;
}

STATIC
VOID
DwUsb3Ep0OutStart (
  IN usb3_pcd_t          *pcd
  )
{
  usb3_dma_desc_t        *desc;
  UINT64                 desc_dma;
  UINT8                  tri;

  /* Get the SETUP packet DMA Descriptor (TRB) */
  desc = pcd->ep0_setup_desc;
  desc_dma = (UINT64)pcd->ep0_setup_desc;

  /* DMA Descriptor setup */
  DwUsb3FillDesc (
    desc,
    (UINT64)pcd->ep0_setup_pkt,
    pcd->ep0.maxpacket,
    0,
    TRBCTL_SETUP,
    DSCCTL_IOC | DSCCTL_ISP | DSCCTL_LST,
    1
    );

  /* issue DEPSTRTXFER command to EP0 OUT */
  tri = DwUsb3DepStartXfer (EP_OUT_IDX (0), desc_dma, 0);
  pcd->ep0.tri_out = tri;
}

STATIC
VOID
DwUsb3Init (
  VOID
  )
{
  UINT32                 Data, Addr;
  usb3_pcd_t             *pcd = &gPcd;

  /* soft reset the usb core */
  do {
    Data = MmioRead32 (DCTL);
    Data &= ~DCTL_RUN_STOP;
    Data |= DCTL_CSFTRST;
    MmioWrite32 (DCTL, Data);

    do {
      MicroSecondDelay (1000);
      Data = MmioRead32 (DCTL);
    } while (Data & DCTL_CSFTRST);
    MicroSecondDelay (1000);
  } while (0);

  pcd->link_state = 0;

  /* TI PHY: Set Turnaround Time = 9 (8-bit UTMI+ / ULPI) */
  Data = MmioRead32 (GUSB2PHYCFG (0));
  Data &= ~GUSB2PHYCFG_USBTRDTIM_MASK;
  Data |= GUSB2PHYCFG_USBTRDTIM (9);
  MmioWrite32 (GUSB2PHYCFG (0), Data);

  /* set TX FIFO size */
  Addr = TX_FIFO_ADDR;
  DwUsb3SetFifoSize (Addr, RAM_TX0_DEPTH / RAM_WIDTH, FIFO_DIR_TX, 0);
  Addr += RAM_TX0_DEPTH / RAM_WIDTH;
  DwUsb3SetFifoSize (Addr, RAM_TX1_DEPTH / RAM_WIDTH, FIFO_DIR_TX, 1);
  /* set RX FIFO size */
  Addr = RX_FIFO_ADDR;
  DwUsb3SetFifoSize (Addr, RAM_RX_DEPTH / RAM_WIDTH, FIFO_DIR_RX, 0);

  /* set LFPS filter delay1trans */
  Data = MmioRead32 (GUSB3PIPECTL (0));
  Data &= ~(PIPECTL_DELAYP1TRANS | PIPECTL_TX_DEMPH_MASK);
  Data |= PIPECTL_LFPS_FILTER | PIPECTL_TX_DEMPH (1);
  MmioWrite32 (GUSB3PIPECTL (0), Data);

  /* set GCTL */
  Data = GCTL_U2EXIT_LFPS | GCTL_PRTCAPDIR_DEVICE | GCTL_U2RSTECN |
         GCTL_PWRDNSCALE(2);
  MmioWrite32 (GCTL, Data);

  /* init event buf */
  MmioWrite32 (GEVNTADRL(0), (UINT32)(UINTN)&gEventBuf);
  MmioWrite32 (GEVNTADRH(0), (UINTN)&gEventBuf >> 32);
  MmioWrite32 (GEVNTSIZ(0), DWUSB3_EVENT_BUF_SIZE << 2);
  MmioWrite32 (GEVNTCOUNT(0), 0);

  /* set max speed to super speed */
  Data = MmioRead32 (DCFG) & ~DCFG_DEVSPD_MASK;
  Data |= DCFG_DEVSPD (DEVSPD_SS_PHY_125MHZ_OR_250MHZ);
  MmioWrite32 (DCFG, Data);
  /* set nump */
  Data = MmioRead32 (DCFG) & ~DCFG_NUMP_MASK;
  Data |= DCFG_NUMP (16);
  MmioWrite32 (DCFG, Data);

  /* init address */
  DwUsb3SetAddress (0);

  /* disable phy suspend */
  Data = MmioRead32 (GUSB3PIPECTL (0)) & ~PIPECTL_SUSPEND_EN;
  MmioWrite32 (GUSB3PIPECTL (0), Data);
  Data = MmioRead32 (GUSB2PHYCFG (0)) & ~GUSB2PHYCFG_SUSPHY;
  MmioWrite32 (GUSB2PHYCFG (0), Data);

  /* clear any pending interrupts */
  DwUsb3DisableFlushEventbufIntr ();
  /* enable device interrupts */
  MmioWrite32 (DEVTEN, DEVTEN_CONNECTDONEEN | DEVTEN_USBRSTEN);
  /* activate EP0 */
  DwUsb3Ep0Activate (pcd);
  /* start EP0 to receive SETUP packets */
  DwUsb3Ep0OutStart (pcd);

  /* enable EP0 OUT/IN in DALEPENA */
  MmioWrite32 (DALEPENA, EP_IN_IDX (0) | EP_OUT_IDX (0));

  /* set RUN/STOP bit */
  Data = MmioRead32 (DCTL) | DCTL_RUN_STOP;
  MmioWrite32 (DCTL, Data);
}

STATIC
VOID
DriverInit (
  VOID
  )
{
}

STATIC
VOID
DwUsb3HandleUsbResetInterrupt (
  IN usb3_pcd_t       *pcd
  )
{
  usb3_pcd_ep_t        *ep;

  // clear stall on each EP
  ep = &pcd->in_ep;
  if (ep->xfer_started) {
    if (ep->is_in) {
      DwUsb3DepStopXfer (EP_IN_IDX (ep->num), ep->tri_in);
    } else {
      DwUsb3DepStopXfer (EP_OUT_IDX (ep->num), ep->tri_out);
    }
  }
  if (ep->stopped) {
    if (ep->is_in) {
      DwUsb3DepClearStall (EP_IN_IDX (ep->num));
    } else {
      DwUsb3DepClearStall (EP_OUT_IDX (ep->num));
    }
  }

  ep = &pcd->out_ep;
  if (ep->xfer_started) {
    if (ep->is_in) {
      DwUsb3DepStopXfer (EP_IN_IDX (ep->num), ep->tri_in);
    } else {
      DwUsb3DepStopXfer (EP_OUT_IDX (ep->num), ep->tri_out);
    }
  }
  if (ep->stopped) {
    if (ep->is_in) {
      DwUsb3DepClearStall (EP_IN_IDX (ep->num));
    } else {
      DwUsb3DepClearStall (EP_OUT_IDX (ep->num));
    }
  }

  // set device address to 0
  DwUsb3SetAddress (0);

  pcd->ltm_enable = 0;
  DEBUG ((DEBUG_INFO, "usb reset\n"));
}

STATIC
UINT32
DwUsb3GetDeviceSpeed (
  IN usb3_pcd_t         *pcd
  )
{
  UINT32                Data, Speed;

  Data = MmioRead32 (DSTS);
  switch (Data & DCFG_DEVSPD_MASK) {
  case DEVSPD_HS_PHY_30MHZ_OR_60MHZ:
    Speed = USB_SPEED_HIGH;
    break;
  case DEVSPD_FS_PHY_30MHZ_OR_60MHZ:
  case DEVSPD_FS_PHY_48MHZ:
    Speed = USB_SPEED_FULL;
    break;
  case DEVSPD_LS_PHY_6MHZ:
    Speed = USB_SPEED_LOW;
    break;
  case DEVSPD_SS_PHY_125MHZ_OR_250MHZ:
    Speed = USB_SPEED_SUPER;
    break;
  default:
    DEBUG ((DEBUG_ERROR, "DwUsb3GetDeviceSpeed: invalid DSTS:0x%x\n", Data));
    break;
  }
  return Speed;
}

STATIC
VOID
DwUsb3PcdSetSpeed (
  IN usb3_pcd_t         *pcd,
  IN UINTN              speed
  )
{
  // set the MPS of EP0 based on the connection speed
  switch (speed) {
  case USB_SPEED_SUPER:
    pcd->ep0.maxpacket = 512;
    pcd->in_ep.maxpacket = USB3_MAX_PACKET_SIZE;
    pcd->out_ep.maxpacket = USB3_MAX_PACKET_SIZE;
    break;
  case USB_SPEED_HIGH:
    pcd->ep0.maxpacket = 64;
    pcd->in_ep.maxpacket = USB2_HS_MAX_PACKET_SIZE;
    pcd->out_ep.maxpacket = USB2_HS_MAX_PACKET_SIZE;
    break;
  case USB_SPEED_FULL:
    pcd->ep0.maxpacket = 64;
    pcd->in_ep.maxpacket = USB2_FS_MAX_PACKET_SIZE;
    pcd->out_ep.maxpacket = USB2_FS_MAX_PACKET_SIZE;
    break;
  default:
    DEBUG ((DEBUG_ERROR, "invalid speed: %d\n", speed));
    break;
  }
}

STATIC
VOID
DwUsb3HandleConnectDoneInterrupt (
  IN usb3_pcd_t         *pcd
  )
{
  usb3_pcd_ep_t         *ep0 = &pcd->ep0;
  UINT32                DiepCfg0, DoepCfg0, DiepCfg1, DoepCfg1;
  UINT32                Speed;

  ep0->stopped = 0;
  Speed = (UINT32)DwUsb3GetDeviceSpeed (pcd);
  pcd->speed = (UINT8)Speed;

  DwUsb3PcdSetSpeed (pcd, Speed);
  // set the MPS of EP0 based on the connection speed
  DiepCfg0 = EPCFG0_EPTYPE (EPTYPE_CONTROL) | EPCFG0_CFG_ACTION (CFG_ACTION_MODIFY);
  DiepCfg1 = EPCFG1_XFER_CMPL | EPCFG1_XFER_NRDY | EPCFG1_EP_DIR;
  DoepCfg0 = EPCFG0_EPTYPE (EPTYPE_CONTROL) | EPCFG0_CFG_ACTION (CFG_ACTION_MODIFY);
  DoepCfg1 = EPCFG1_XFER_CMPL | EPCFG1_XFER_NRDY;

  switch (Speed) {
  case USB_SPEED_SUPER:
    DiepCfg0 |= EPCFG0_MPS (512);
    DoepCfg0 |= EPCFG0_MPS (512);
    break;
  case USB_SPEED_HIGH:
  case USB_SPEED_FULL:
    DiepCfg0 |= EPCFG0_MPS (64);
    DoepCfg0 |= EPCFG0_MPS (64);
    break;
  case USB_SPEED_LOW:
    DiepCfg0 |= EPCFG0_MPS (8);
    DoepCfg0 |= EPCFG0_MPS (8);
    break;
  default:
    DEBUG ((DEBUG_ERROR, "DwUsb3HandleConnectDoneInterrupt: invalid speed %d\n", Speed));
    break;
  }
  DiepCfg0 |= EPCFG0_TXFNUM (ep0->tx_fifo_num);
  // issue DEPCFG command to EP0 OUT
  DwUsb3DepCfg (EP_OUT_IDX (0), DoepCfg0, DoepCfg1, 0);
  // issue DEPCFG command to EP0 IN
  DwUsb3DepCfg (EP_IN_IDX (0), DiepCfg0, DiepCfg1, 0);
  pcd->state = USB3_STATE_DEFAULT;
}

STATIC
VOID
DwUsb3HandleDeviceInterrupt (
  IN usb3_pcd_t       *pcd,
  IN UINT32           Event
  )
{
  switch (Event & GEVNT_DEVT_MASK) {
  case GEVNT_DEVT_USBRESET:
    DwUsb3HandleUsbResetInterrupt (pcd);
    break;
  case GEVNT_DEVT_CONNDONE:
    DwUsb3HandleConnectDoneInterrupt (pcd);
    break;
  default:
    DEBUG ((DEBUG_ERROR, "DwUsb3HandleDeviceInterrupt: invalid event\n"));
    break;
  }
}

STATIC
usb3_pcd_ep_t *
DwUsb3GetOutEndPoint (
  IN usb3_pcd_t       *pcd,
  IN UINT32           EndPointNum
  )
{
  if (EndPointNum == 0) {
    return &pcd->ep0;
  }
  return &pcd->out_ep;
}

STATIC
usb3_pcd_ep_t *
DwUsb3GetInEndPoint (
  IN usb3_pcd_t       *pcd,
  IN UINT32           EndPointNum
  )
{
  if (EndPointNum == 0) {
    return &pcd->ep0;
  }
  return &pcd->in_ep;
}

STATIC
VOID
EndPoint0DoStall (
  IN usb3_pcd_t       *pcd
  )
{
  usb3_pcd_ep_t       *ep0 = &pcd->ep0;

  // stall EP0 IN & OUT simultanelusly
  DwUsb3DepSetStall (EP_IN_IDX (0));
  DwUsb3DepSetStall (EP_OUT_IDX (0));
  // prepare for the next setup transfer
  ep0->stopped = 1;
  pcd->ep0state = EP0_IDLE;
  DwUsb3Ep0OutStart (pcd);
}

STATIC
VOID
EndPoint0ContinueTransfer (
  IN usb3_pcd_t       *pcd,
  IN usb3_pcd_req_t   *req
  )
{
  usb3_pcd_ep_t       *ep0 = &pcd->ep0;
  usb3_dma_desc_t     *desc;
  UINT64              desc_dma;
  UINT8               tri;

  // send a 0-byte length packet after the end of transfer
  if (ep0->is_in) {
    desc = pcd->ep0_in_desc;
    desc_dma = (UINT64)pcd->ep0_in_desc;
    // DMA descriptor setup
    DwUsb3FillDesc (
      desc,
      (UINT64)req->bufdma,
      0,
      0,
      TRBCTL_NORMAL,
      DSCCTL_IOC | DSCCTL_ISP | DSCCTL_LST,
      1
      );
    tri = DwUsb3DepStartXfer (EP_IN_IDX (0), desc_dma, 0);
    ep0->tri_in = tri;
  }
}

STATIC
VOID
EndPoint0CompleteRequest (
  IN usb3_pcd_t       *pcd,
  IN usb3_pcd_req_t   *req,
  IN usb3_dma_desc_t  *desc
  )
{
  usb3_pcd_ep_t      *ep = &pcd->ep0;

  if (req == NULL) {
    return;
  }

  if ((pcd->ep0state == EP0_OUT_DATA_PHASE) ||
      (pcd->ep0state == EP0_IN_DATA_PHASE)) {
    if (ep->is_in) {
      if (GET_DSCSTS_XFERCNT (desc->status) == 0) {
        pcd->ep0.is_in = 0;
        pcd->ep0state = EP0_OUT_WAIT_NRDY;
      }
    } else {
      pcd->ep0.is_in = 1;
      pcd->ep0state = EP0_IN_WAIT_NRDY;
    }
  }
}

STATIC
VOID
DwUsb3OsGetTrb (
  IN usb3_pcd_t       *pcd,
  IN usb3_pcd_ep_t    *ep,
  IN usb3_pcd_req_t   *req
  )
{
  // If EP0, fill request with EP0 IN/OUT data TRB
  if (ep == &pcd->ep0) {
    if (ep->is_in) {
      req->trb = pcd->ep0_in_desc;
      req->trbdma = (UINT64)pcd->ep0_in_desc;
    } else {
      req->trb = pcd->ep0_out_desc;
      req->trbdma = (UINT64)pcd->ep0_out_desc;
    }
  } else {
    // fill request with TRB from the non-EP0 allocation
    req->trb = ep->ep_desc;
    req->trbdma = (UINT64)ep->ep_desc;
  }
}

STATIC
VOID
DwUsb3StartXfer (
  IN usb3_pcd_t       *pcd,
  IN usb3_pcd_req_t   *req
  )
{
  usb3_pcd_ep_t       *ep0 = &pcd->ep0;
  usb3_dma_desc_t     *desc;
  UINT64              desc_dma;
  UINT32              desc_type, len;

  // get the DMA descriptor (TRB) for this request
  DwUsb3OsGetTrb (pcd, ep0, req);
  desc = req->trb;
  desc_dma = req->trbdma;

  if (ep0->is_in) {
    // start DMA on EP0 IN
    // DMA Descriptor (TRB) setup
    len = req->length;
    if (pcd->ep0state == EP0_IN_STATUS_PHASE) {
      if (ep0->three_stage) {
        desc_type = DSCCTL_TRBCTL (TRBCTL_STATUS_3);
      } else {
        desc_type = DSCCTL_TRBCTL (TRBCTL_STATUS_2);
      }
    } else {
      desc_type = DSCCTL_TRBCTL (TRBCTL_CTLDATA_1ST);
    }
    DwUsb3FillDesc (
      desc,
      (UINT64)req->bufdma,
      len,
      0,
      desc_type,
      DSCCTL_IOC | DSCCTL_ISP | DSCCTL_LST,
      1
      );
    // issue DEPSTRTXFER command to EP0 IN
    ep0->tri_in = DwUsb3DepStartXfer (EP_IN_IDX (0), desc_dma, 0);
  } else {
    // start DMA on EP0 OUT
    // DMA Descriptor (TRB) setup
    len = (req->length + ep0->maxpacket - 1) & ~(ep0->maxpacket - 1);
    if (pcd->ep0state == EP0_OUT_STATUS_PHASE) {
      if (ep0->three_stage) {
        desc_type = DSCCTL_TRBCTL (TRBCTL_STATUS_3);
      } else {
        desc_type = DSCCTL_TRBCTL (TRBCTL_STATUS_2);
      }
    } else {
      desc_type = DSCCTL_TRBCTL (TRBCTL_CTLDATA_1ST);
    }
    DwUsb3FillDesc (
      desc,
      (UINT64)req->bufdma,
      len,
      0,
      desc_type,
      DSCCTL_IOC | DSCCTL_ISP | DSCCTL_LST,
      1
      );
    // issue DEPSTRTXFER command to EP0 OUT
    ep0->tri_out = DwUsb3DepStartXfer (EP_OUT_IDX (0), desc_dma, 0);
  }
}

STATIC
VOID
SetupInStatusPhase (
  IN usb3_pcd_t       *pcd,
  IN VOID             *buf
  )
{
  usb3_pcd_ep_t       *ep0 = &pcd->ep0;

  if (pcd->ep0state == EP0_STALL)
    return;

  ep0->is_in = 1;
  pcd->ep0state = EP0_IN_STATUS_PHASE;
  pcd->ep0_req.bufdma = buf;
  pcd->ep0_req.length = 0;
  pcd->ep0_req.actual = 0;
  DwUsb3StartXfer (pcd, &pcd->ep0_req);
}

STATIC
VOID
SetupOutStatusPhase (
  IN usb3_pcd_t       *pcd,
  IN VOID             *buf
  )
{
  usb3_pcd_ep_t       *ep0 = &pcd->ep0;

  if (pcd->ep0state == EP0_STALL)
    return;

  ep0->is_in = 0;
  pcd->ep0state = EP0_OUT_STATUS_PHASE;
  pcd->ep0_req.bufdma = buf;
  pcd->ep0_req.length = 0;
  pcd->ep0_req.actual = 0;
  DwUsb3StartXfer (pcd, &pcd->ep0_req);
}

STATIC
VOID
DwUsb3HandleEndPoint0 (
  IN usb3_pcd_t       *pcd,
  IN usb3_pcd_req_t   *req,
  IN UINT32           event
  )
{
  usb3_pcd_ep_t       *ep0 = &pcd->ep0;
  usb3_dma_desc_t     *desc;
  UINT32              byte_count, len;

  switch (pcd->ep0state) {
  case EP0_IN_DATA_PHASE:
    if (req == NULL) {
      req = &pcd->ep0_req;
    }
    desc = pcd->ep0_in_desc;

    if (desc->control & DSCCTL_HWO) {
      goto out;
    }

    if (GET_DSCSTS_TRBRSP (desc->status) == TRBRSP_SETUP_PEND) {
      // start of a new control transfer
      desc->status = 0;
    }
    byte_count = req->length - GET_DSCSTS_XFERCNT (desc->status);
    req->actual += byte_count;
    req->bufdma += byte_count;

    if (req->actual < req->length) {
      // IN CONTINUE, stall EP0
      EndPoint0DoStall (pcd);
    } else if (ep0->send_zlp) {
      // CONTINUE TRANSFER IN ZLP
      EndPoint0ContinueTransfer (pcd, req);
      ep0->send_zlp = 0;
    } else {
      // COMPLETE IN TRANSFER
      EndPoint0CompleteRequest (pcd, req, desc);
    }
    break;
  case EP0_OUT_DATA_PHASE:
    if (req == NULL) {
      req = &pcd->ep0_req;
    }
    desc = pcd->ep0_out_desc;

    if (desc->control & DSCCTL_HWO) {
      goto out;
    }

    if (GET_DSCSTS_TRBRSP (desc->status) == TRBRSP_SETUP_PEND) {
      // start of a new control transfer
      desc->status = 0;
    }
    len = (req->length + ep0->maxpacket - 1) & ~(ep0->maxpacket - 1);
    byte_count = len - GET_DSCSTS_XFERCNT (desc->status);
    req->actual += byte_count;
    req->bufdma += byte_count;

    if (req->actual < req->length) {
      // IN CONTINUE, stall EP0
      EndPoint0DoStall (pcd);
    } else if (ep0->send_zlp) {
      // CONTINUE TRANSFER IN ZLP
      EndPoint0ContinueTransfer (pcd, req);
      ep0->send_zlp = 0;
    } else {
      // COMPLETE IN TRANSFER
      EndPoint0CompleteRequest (pcd, req, desc);
    }
    break;
  case EP0_IN_WAIT_NRDY:
  case EP0_OUT_WAIT_NRDY:
    if (ep0->is_in) {
      SetupInStatusPhase (pcd, pcd->ep0_setup_pkt);
    } else {
      SetupOutStatusPhase (pcd, pcd->ep0_setup_pkt);
    }
    break;
  case EP0_IN_STATUS_PHASE:
  case EP0_OUT_STATUS_PHASE:
    if (ep0->is_in) {
      desc = pcd->ep0_in_desc;
    } else {
      desc = pcd->ep0_out_desc;
    }
    EndPoint0CompleteRequest (pcd, req, desc);
    // skip test mode
    pcd->ep0state = EP0_IDLE;
    ep0->stopped = 1;
    ep0->is_in = 0;  // OUT for next SETUP
    // prepare for more SETUP packets
    DwUsb3Ep0OutStart (pcd);
    break;
  case EP0_STALL:
    break;
  case EP0_IDLE:
    break;
  default:
    DEBUG ((DEBUG_ERROR, "%a: invalid state %d\n", __func__, pcd->ep0state));
    break;
  }
out:
  return;
}

STATIC
usb3_pcd_ep_t *
Addr2EndPoint (
  IN usb3_pcd_t       *pcd,
  IN UINT16           index
  )
{
  UINT32              ep_num;

  ep_num = UE_GET_ADDR (index);
  if (ep_num == 0) {
    return &pcd->ep0;
  } else {
    if (UE_GET_DIR (index) == UE_DIR_IN) {
      return &pcd->in_ep;
    }
    return &pcd->out_ep;
  }
}

STATIC
VOID
DwUsb3DoGetStatus (
  IN usb3_pcd_t       *pcd
  )
{
  usb_device_request_t   *ctrl = &pcd->ep0_setup_pkt[0].req;
  UINT8                  *status = pcd->ep0_status_buf;
  usb3_pcd_ep_t          *ep;

  if (ctrl->wLength != 2) {
    EndPoint0DoStall (pcd);
    return;
  }

  switch (UT_GET_RECIPIENT (ctrl->bmRequestType)) {
  case UT_DEVICE:
    *status = 0;   // bus powered
    if (pcd->speed == USB_SPEED_SUPER) {
      if (pcd->state == USB3_STATE_CONFIGURED) {
        if (MmioRead32 (DCTL) & DCTL_INIT_U1_EN) {
          *status |= 1 << 2;
        }
        if (MmioRead32 (DCTL) & DCTL_INIT_U2_EN) {
          *status |= 1 << 3;
        }
        *status |= (UINT8)(pcd->ltm_enable << 4);
      }
    }
    *(status + 1) = 0;
    break;
  case UT_INTERFACE:
    *status = 0;
    *(status + 1) = 0;
    break;
  case UT_ENDPOINT:
    ep = Addr2EndPoint (pcd, ctrl->wIndex);
    *status = ep->stopped;
    *(status + 1) = 0;
    break;
  default:
    EndPoint0DoStall (pcd);
    return;
  }
  pcd->ep0_req.bufdma = (UINT64 *)status;
  pcd->ep0_req.length = 2;
  pcd->ep0_req.actual = 0;
  DwUsb3StartXfer (pcd, &pcd->ep0_req);
}

STATIC
VOID
DoClearHalt (
  IN usb3_pcd_t       *pcd,
  IN usb3_pcd_ep_t    *ep
  )
{
  if (ep->is_in) {
    DwUsb3DepClearStall (EP_IN_IDX (ep->num));
  } else {
    DwUsb3DepClearStall (EP_OUT_IDX (ep->num));
  }
  if (ep->stopped) {
    ep->stopped = 0;
  }
}

STATIC
VOID
Usb3PcdEpEnable (
  IN usb3_pcd_t        *pcd,
  IN usb3_pcd_ep_t     *ep
  )
{
  // activate the EP
  ep->stopped = 0;
  ep->xfer_started = 0;
  ep->ep_desc->control = 0;
  ep->ep_desc->status = 0;
  // set initial data pid.
  if (ep->type == EPTYPE_BULK) {
    ep->data_pid_start = 0;
  }
  DwUsb3EpActivate (pcd, ep);
}

STATIC
VOID
DwUsb3DoClearFeature (
  IN usb3_pcd_t       *pcd
  )
{
  usb_device_request_t  *ctrl = &pcd->ep0_setup_pkt[0].req;
  usb3_pcd_ep_t  *ep;

  switch (UT_GET_RECIPIENT (ctrl->bmRequestType)) {
  case UT_DEVICE:
    switch (ctrl->wValue) {
    case UF_U1_ENABLE:
      if ((pcd->speed != USB_SPEED_SUPER) ||
          (pcd->state != USB3_STATE_CONFIGURED)) {
        EndPoint0DoStall (pcd);
        return;
      }
      MmioAnd32 (DCTL, ~DCTL_INIT_U1_EN);
      break;
    case UF_U2_ENABLE:
      if ((pcd->speed != USB_SPEED_SUPER) ||
          (pcd->state != USB3_STATE_CONFIGURED)) {
        EndPoint0DoStall (pcd);
        return;
      }
      MmioAnd32 (DCTL, ~DCTL_INIT_U2_EN);
      break;
    case UF_LTM_ENABLE:
      if ((pcd->speed != USB_SPEED_SUPER) ||
          (pcd->state != USB3_STATE_CONFIGURED) ||
          (ctrl->wIndex != 0)) {
        EndPoint0DoStall (pcd);
        return;
      }
      pcd->ltm_enable = 0;
      break;
    default:
      EndPoint0DoStall (pcd);
      return;
    }
    break;
  case UT_INTERFACE:
    // if FUNCTION_SUSPEND
    if (ctrl->wValue) {
      EndPoint0DoStall (pcd);
      return;
    }
    break;
  case UT_ENDPOINT:
    ep = Addr2EndPoint (pcd, ctrl->wIndex);
    if (ctrl->wValue != UF_ENDPOINT_HALT) {
      EndPoint0DoStall (pcd);
      return;
    }
    DoClearHalt (pcd, ep);
    break;
  default:
    DEBUG ((DEBUG_ERROR, "invalid bmRequestType :%d\n", UT_GET_RECIPIENT (ctrl->bmRequestType)));
    break;
  }
  pcd->ep0.is_in = 1;
  pcd->ep0state = EP0_IN_WAIT_NRDY;
}

STATIC
VOID
DwUsb3DoSetFeature (
  IN usb3_pcd_t       *pcd
  )
{
  usb_device_request_t  *ctrl = &pcd->ep0_setup_pkt[0].req;
  usb3_pcd_ep_t  *ep;

  switch (UT_GET_RECIPIENT (ctrl->bmRequestType)) {
  case UT_DEVICE:
    switch (ctrl->wValue) {
    case UF_DEVICE_REMOTE_WAKEUP:
      break;
    case UF_TEST_MODE:
      pcd->test_mode_nr = ctrl->wIndex >> 8;
      pcd->test_mode = 1;
      break;
    case UF_U1_ENABLE:
      if ((pcd->speed != USB_SPEED_SUPER) ||
          (pcd->state != USB3_STATE_CONFIGURED)) {
        EndPoint0DoStall (pcd);
        return;
      }
      MmioOr32 (DCTL, DCTL_INIT_U1_EN);
      break;
    case UF_U2_ENABLE:
      if ((pcd->speed != USB_SPEED_SUPER) ||
          (pcd->state != USB3_STATE_CONFIGURED)) {
        EndPoint0DoStall (pcd);
        return;
      }
      MmioOr32 (DCTL, DCTL_INIT_U2_EN);
      break;
    case UF_LTM_ENABLE:
      if ((pcd->speed != USB_SPEED_SUPER) ||
          (pcd->state != USB3_STATE_CONFIGURED) ||
          (ctrl->wIndex != 0)) {
        EndPoint0DoStall (pcd);
        return;
      }
      pcd->ltm_enable = 1;
      break;
    default:
      EndPoint0DoStall (pcd);
      return;
    }
    break;
  case UT_INTERFACE:
    // if FUNCTION_SUSPEND
    if (ctrl->wValue) {
      EndPoint0DoStall (pcd);
      return;
    }
    break;
  case UT_ENDPOINT:
    ep = Addr2EndPoint (pcd, ctrl->wIndex);
    if (ctrl->wValue != UF_ENDPOINT_HALT) {
      EndPoint0DoStall (pcd);
      return;
    }
    ep->stopped = 1;
    if (ep->is_in) {
      DwUsb3DepClearStall (EP_IN_IDX (ep->num));
    } else {
      DwUsb3DepClearStall (EP_OUT_IDX (ep->num));
    }
    break;
  default:
    DEBUG ((DEBUG_ERROR, "invalid bmRequestType %d\n", UT_GET_RECIPIENT (ctrl->bmRequestType)));
    break;
  }
  pcd->ep0.is_in = 1;
  pcd->ep0state = EP0_IN_WAIT_NRDY;
}

STATIC
VOID
DwUsb3DoSetAddress (
  IN usb3_pcd_t          *pcd
  )
{
  usb_device_request_t *ctrl = &pcd->ep0_setup_pkt[0].req;

  if (ctrl->bmRequestType == UT_DEVICE) {
    DwUsb3SetAddress (ctrl->wValue);
    pcd->ep0.is_in = 1;
    pcd->ep0state = EP0_IN_WAIT_NRDY;
    if (ctrl->wValue) {
      pcd->state = USB3_STATE_ADDRESSED;
    } else {
      pcd->state = USB3_STATE_DEFAULT;
    }
  }
}

#if 0
STATIC
UINTN
UsbStatus (
  IN UINTN                        online,
  IN UINTN                        speed
  )
{
  if (online) {
}
#endif

VOID
DwUsb3SetConfig (
  IN usb3_pcd_t           *pcd
  )
{
  usb_device_request_t   *ctrl = &pcd->ep0_setup_pkt[0].req;
  UINT16         wvalue = ctrl->wValue;
  usb3_pcd_ep_t     *ep;

  if (ctrl->bmRequestType != (UT_WRITE | UT_STANDARD | UT_DEVICE)) {
    EndPoint0DoStall (pcd);
    return;
  }

  if (!wvalue || (wvalue == CONFIG_VALUE)) {
    UINT32         speed;
    pcd->new_config = (UINT8)wvalue;
    // set new configuration
    if (wvalue) {
      // activate bulk in endpoint
      ep = &pcd->in_ep;
      Usb3PcdEpEnable (pcd, ep);
      // activate bulk out endpoint
      ep = &pcd->out_ep;
      Usb3PcdEpEnable (pcd, ep);
      // prepare for next bulk transfer
      speed = DwUsb3GetDeviceSpeed (pcd);
      (VOID)speed;

#if 0
      if (g_usb_ops->status) {
        g_usb_ops->status (ctrl->wValue ? 1 : 0,
                           speed == USB_SPEED_SUPER ? USB_SS : speed == USB_SPEED_HIGH ? USB_HS : USB_FS);
      }
      usb_status ();
#endif
      pcd->state = USB3_STATE_CONFIGURED;
    } else {
      pcd->state = USB3_STATE_ADDRESSED;
    }
    pcd->ep0.is_in = 1;
    pcd->ep0state = EP0_IN_WAIT_NRDY;
  } else {
    EndPoint0DoStall (pcd);
  }
}

STATIC
VOID
DwUsb3DoGetConfig (
  IN usb3_pcd_t       *pcd
  )
{
  usb_device_request_t  *ctrl = &pcd->ep0_setup_pkt[0].req;
  UINT8  *status = pcd->ep0_status_buf;

  if (ctrl->bmRequestType != (UT_READ | UT_STANDARD | UT_DEVICE)) {
    EndPoint0DoStall (pcd);
    return;
  }
  // Notify host the current config value
  *status = pcd->new_config;
  pcd->ep0_req.bufdma = (UINT64 *)status;
  pcd->ep0_req.length = 1;
  pcd->ep0_req.actual = 0;
  DwUsb3StartXfer (pcd, &pcd->ep0_req);
}

STATIC
VOID
DwUsb3DoSetConfig (
  IN usb3_pcd_t       *pcd
  )
{
  usb_device_request_t  *ctrl = &pcd->ep0_setup_pkt[0].req;
  UINT16  wvalue = ctrl->wValue;
  usb3_pcd_ep_t  *ep;

  if (ctrl->bmRequestType != (UT_WRITE | UT_STANDARD | UT_DEVICE)) {
    EndPoint0DoStall (pcd);
    return;
  }

  if (!wvalue || (wvalue == CONFIG_VALUE)) {
    UINT32 speed;

    pcd->new_config = (UINT8)wvalue;
    // set new configuration
    if (wvalue) {
      // activate bulk in endpoint
      ep = &pcd->in_ep;
      Usb3PcdEpEnable (pcd, ep);
      // activate bulk out endpoint
      ep = &pcd->out_ep;
      Usb3PcdEpEnable (pcd, ep);
      // prepare for next bulk transfer
      speed = DwUsb3GetDeviceSpeed (pcd);
      (VOID)speed;
#if 0
      g_usb_ops->status
#endif
      pcd->state = USB3_STATE_CONFIGURED;
    } else {
      pcd->state = USB3_STATE_ADDRESSED;
    }
    pcd->ep0.is_in = 1;
    pcd->ep0state = EP0_IN_WAIT_NRDY;
  } else {
    EndPoint0DoStall (pcd);
  }
}

STATIC
VOID
DwUsb3DoGetDescriptor (
  IN usb3_pcd_t       *pcd
  )
{
}

STATIC
VOID
DwUsb3DoSetup (
  IN usb3_pcd_t       *pcd
  )
{
  usb_device_request_t  *ctrl = &pcd->ep0_setup_pkt[0].req;
  usb3_pcd_ep_t         *ep0 = &pcd->ep0;
  UINT16                wLength;

  wLength = ctrl->wLength;
  ep0->stopped = 0;
  ep0->three_stage = 1;
  if (ctrl->bmRequestType & UE_DIR_IN) {
    ep0->is_in = 1;
    pcd->ep0state = EP0_IN_DATA_PHASE;
  } else {
    ep0->is_in = 0;
    pcd->ep0state = EP0_OUT_DATA_PHASE;
  }

  if (wLength == 0) {
    ep0->is_in = 1;
    pcd->ep0state = EP0_IN_WAIT_NRDY;
    ep0->three_stage = 0;
  }
  if (UT_GET_TYPE (ctrl->bmRequestType) != UT_STANDARD) {
    EndPoint0DoStall (pcd);
    return;
  }

  switch (ctrl->bRequest) {
  case UR_GET_STATUS:
    DwUsb3DoGetStatus (pcd);
    break;
  case UR_CLEAR_FEATURE:
    DwUsb3DoClearFeature (pcd);
    break;
  case UR_SET_FEATURE:
    DwUsb3DoSetFeature (pcd);
    break;
  case UR_SET_ADDRESS:
    DwUsb3DoSetAddress (pcd);
    break;
  case UR_SET_CONFIG:
    DwUsb3DoSetConfig (pcd);
    MmioAnd32 (DCTL, DCTL_ACCEPT_U1_EN);
    MmioAnd32 (DCTL, DCTL_ACCEPT_U2_EN);
    DEBUG ((DEBUG_INFO, "enum done"));
    pcd->ltm_enable = 0;
    break;
  case UR_GET_CONFIG:
    DwUsb3DoGetConfig (pcd);
    break;
  case UR_GET_DESCRIPTOR:
    // FIXME
    DwUsb3DoGetDescriptor (pcd);
    break;
  case UR_SET_SEL:
    // for now this is a no-op
    pcd->ep0_req.bufdma = (UINT64 *)pcd->ep0_status_buf;
    pcd->ep0_req.length = USB3_STATUS_BUF_SIZE;
    pcd->ep0_req.actual = 0;
    ep0->send_zlp = 0;
    DwUsb3StartXfer (pcd, &pcd->ep0_req);
    break;
  default:
    EndPoint0DoStall (pcd);
    break;
  }
}

STATIC
VOID
DwUsb3OsHandleEndPoint0 (
  IN usb3_pcd_t       *pcd,
  IN UINT32           event
  )
{
  if (pcd->ep0state == EP0_IDLE) {
    DwUsb3DoSetup (pcd);
  } else {
    DwUsb3HandleEndPoint0 (pcd, NULL, event);
  }
}

STATIC
VOID
DwUsb3RequestDone (
  IN usb3_pcd_t       *pcd,
  IN usb3_pcd_ep_t    *ep,
  IN usb3_pcd_req_t   *req,
  IN UINTN            status
  )
{
  if (ep != &pcd->ep0) {
    req->trb = NULL;
  }
  if (req->complete) {
    req->complete (req->actual, status);
  }
  req->actual = 0;
}

STATIC
VOID
DwUsb3EndPointcompleteRequest (
  IN usb3_pcd_t       *pcd,
  IN usb3_pcd_ep_t    *ep,
  IN UINT32           event
  )
{
  usb3_pcd_req_t           *req = &ep->req;
  usb3_dma_desc_t          *desc = req->trb;
  UINT32 byte_count;

  ep->send_zlp = 0;
  if (!desc) {
    return;
  }

  if (desc->control & DSCCTL_HWO) {
    return;
  }

  if (ep->is_in) {
    // IN ep
    if (GET_DSCSTS_XFERCNT (desc->status) == 0) {
      req->actual += req->length;
    }
    // reset IN tri
    ep->tri_in = 0;
    // complete the IN request
    // flush for dma?
    DwUsb3RequestDone (pcd, ep, req, 0);
  } else {
    // OUT ep
    byte_count = req->length - GET_DSCSTS_XFERCNT (desc->status);
    req->actual += byte_count;
    req->bufdma += byte_count;
    // reset OUT tri
    ep->tri_out = 0;
    // OUT transfer complete or not
    // complete the OUT request
    // FIXME flush dma?
    DwUsb3RequestDone (pcd, ep, req, 0);
  }
}

STATIC
VOID
DwUsb3HandleEndPointInterrupt (
  IN usb3_pcd_t       *pcd,
  IN UINTN            PhySep,
  IN UINT32           event
  )
{
  usb3_pcd_ep_t       *ep;
  UINT32              epnum, is_in;

  // Physical Out EPs are even, physical In EPs are odd
  is_in = (UINT32)PhySep & 1;
  epnum = ((UINT32)PhySep >> 1) & 0xF;

  // Get the EP pointer
  if (is_in) {
    ep = DwUsb3GetInEndPoint (pcd, epnum);
  } else {
    ep = DwUsb3GetOutEndPoint (pcd, epnum);
  }

  switch (event & GEVNT_DEPEVT_INTTYPE_MASK) {
  case GEVNT_DEPEVT_INTTYPE_XFER_CMPL:
    ep->xfer_started = 0;
    // complete the transfer
    if (epnum == 0) {
      DwUsb3OsHandleEndPoint0 (pcd, event);
    } else {
      DwUsb3EndPointcompleteRequest (pcd, ep, event);
    }
    break;
  case GEVNT_DEPEVT_INTTYPE_XFER_NRDY:
    if (epnum == 0) {
      switch (pcd->ep0state) {
      case EP0_IN_WAIT_NRDY:
        if (is_in) {
          DwUsb3OsHandleEndPoint0 (pcd, event);
        }
        break;
      case EP0_OUT_WAIT_NRDY:
        if (!is_in) {
          DwUsb3OsHandleEndPoint0 (pcd, event);
        }
        break;
      default:
        break;
      }
    }
    break;
  default:
    DEBUG ((DEBUG_ERROR, "invalid event %d\n", event & GEVNT_DEPEVT_INTTYPE_MASK));
    break;
  }
}

STATIC
UINTN
DwUsb3HandleEvent (
  VOID
  )
{
  usb3_pcd_t          *pcd = &gPcd;
  UINT32              Count, Index, Event, Intr;
  UINT32              PhySep;

  Count = DwUsb3GetEventBufCount ();
  // reset event buffer when it's full
  if ((GEVNTCOUNT_EVNTCOUNT (Count) == GEVNTCOUNT_EVNTCOUNT_MASK) ||
      (Count >= DWUSB3_EVENT_BUF_SIZE * sizeof (UINT32))) {
    DwUsb3UpdateEventBufCount (Count);
    Count = 0;
  }

  for (Index = 0; Index < Count; Index += sizeof (UINT32)) {
    Event = DwUsb3GetEventBufEvent (DWUSB3_EVENT_BUF_SIZE);
    DwUsb3UpdateEventBufCount (sizeof (UINT32));
    if (Event == 0) {
      // ignore null events
      continue;
    }
    if (Event & GEVNT_NON_EP) {
      Intr = Event & GEVNT_INTTYPE_MASK;
      if (Intr == GEVNT_INTTYPE_DEV) {
        DwUsb3HandleDeviceInterrupt (pcd, Event);
      }
    } else {
      PhySep = (Event & GEVNT_DEPEVT_EPNUM_MASK) >> GEVNT_DEPEVT_EPNUM_SHIFT;
      DwUsb3HandleEndPointInterrupt (pcd, PhySep, Event);
    }
  }
  return 0;
}

STATIC
VOID
DwUsb3Poll (
  IN EFI_EVENT        Event,
  IN VOID            *Context
  )
{
  if (DwUsb3HandleEvent ()) {
    DEBUG ((DEBUG_ERROR, "error: exit from usb_poll\n"));
    return;
  }
}

EFI_STATUS
EFIAPI
DwUsb3Start (
  IN USB_DEVICE_DESCRIPTOR   *DeviceDescriptor,
  IN VOID                   **Descriptors,
  IN USB_DEVICE_RX_CALLBACK   RxCallback,
  IN USB_DEVICE_TX_CALLBACK   TxCallback
  )
{
#if 0
  UINT8                    *Ptr;
  EFI_STATUS                Status;
  EFI_EVENT                 TimerEvent;
  UINTN                     StringDescriptorSize;

  ASSERT (DeviceDescriptor != NULL);
  ASSERT (Descriptors[0] != NULL);
  ASSERT (RxCallback != NULL);
  ASSERT (TxCallback != NULL);

  StringDescriptorSize = sizeof (EFI_USB_STRING_DESCRIPTOR) +
	                 sizeof (mLangString) + 1;
  mLangStringDescriptor = AllocateZeroPool (StringDescriptorSize);
  ASSERT (mLangStringDescriptor != NULL);
  mLangStringDescriptor->Length = sizeof (mLangString);
  mLangStringDescriptor->DescriptorType = USB_DESC_TYPE_STRING;
  CopyMem (mLangStringDescriptor->String, mLangString,
	   mLangStringDescriptor->Length);

  StringDescriptorSize = sizeof (EFI_USB_STRING_DESCRIPTOR) +
	                 sizeof (mManufacturerString) + 1;
  mManufacturerStringDescriptor = AllocateZeroPool (StringDescriptorSize);
  ASSERT (mManufacturerStringDescriptor != NULL);
  mManufacturerStringDescriptor->Length = sizeof (mManufacturerString);
  mManufacturerStringDescriptor->DescriptorType = USB_DESC_TYPE_STRING;
  CopyMem (mManufacturerStringDescriptor->String, mManufacturerString,
	   mManufacturerStringDescriptor->Length);

  StringDescriptorSize = sizeof (EFI_USB_STRING_DESCRIPTOR) +
	                 sizeof (mProductString) + 1;
  mProductStringDescriptor = AllocateZeroPool (StringDescriptorSize);
  ASSERT (mProductStringDescriptor != NULL);
  mProductStringDescriptor->Length = sizeof (mProductString);
  mProductStringDescriptor->DescriptorType = USB_DESC_TYPE_STRING;
  CopyMem (mProductStringDescriptor->String, mProductString,
	   mProductStringDescriptor->Length);

  StringDescriptorSize = sizeof (EFI_USB_STRING_DESCRIPTOR) +
	                 sizeof (mSerialString) + 1;
  mSerialStringDescriptor = AllocateZeroPool (StringDescriptorSize);
  ASSERT (mSerialStringDescriptor != NULL);
  mSerialStringDescriptor->Length = sizeof (mSerialString);
  mSerialStringDescriptor->DescriptorType = USB_DESC_TYPE_STRING;
  CopyMem (mSerialStringDescriptor->String, mSerialString,
	   mSerialStringDescriptor->Length);

  DwUsb3Init();

  mDeviceDescriptor = DeviceDescriptor;
  mDescriptors = Descriptors[0];

  // Right now we just support one configuration
  ASSERT (mDeviceDescriptor->NumConfigurations == 1);
  mDeviceDescriptor->StrManufacturer = 1;
  mDeviceDescriptor->StrProduct = 2;
  mDeviceDescriptor->StrSerialNumber = 3;
  // ... and one interface
  mConfigDescriptor = (USB_CONFIG_DESCRIPTOR *)mDescriptors;
  ASSERT (mConfigDescriptor->NumInterfaces == 1);

  Ptr = ((UINT8 *) mDescriptors) + sizeof (USB_CONFIG_DESCRIPTOR);
  mInterfaceDescriptor = (USB_INTERFACE_DESCRIPTOR *) Ptr;
  Ptr += sizeof (USB_INTERFACE_DESCRIPTOR);

  mEndpointDescriptors = (USB_ENDPOINT_DESCRIPTOR *) Ptr;

  mDataReceivedCallback = RxCallback;
  mDataSentCallback = TxCallback;

  // Register a timer event so CheckInterupts gets called periodically
  Status = gBS->CreateEvent (
                  EVT_TIMER | EVT_NOTIFY_SIGNAL,
                  TPL_CALLBACK,
                  CheckInterrupts,
                  NULL,
                  &TimerEvent
                  );
  ASSERT_EFI_ERROR (Status);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = gBS->SetTimer (
                  TimerEvent,
                  TimerPeriodic,
                  DW_INTERRUPT_POLL_PERIOD
                  );
  ASSERT_EFI_ERROR (Status);

  return Status;
#else
  EFI_STATUS             Status;
  EFI_EVENT              TimerEvent;

  DriverInit ();
  DwUsb3Init ();
  Status = gBS->CreateEvent (
                  EVT_TIMER | EVT_NOTIFY_SIGNAL,
                  TPL_CALLBACK,
                  DwUsb3Poll,
                  NULL,
                  &TimerEvent
                  );
  ASSERT_EFI_ERROR (Status);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = gBS->SetTimer (
                  TimerEvent,
                  TimerPeriodic,
                  DW_INTERRUPT_POLL_PERIOD
                  );
  ASSERT_EFI_ERROR (Status);
  return Status;
#endif
}

EFI_STATUS
DwUsb3Send (
  IN        UINT8  EndpointIndex,
  IN        UINTN  Size,
  IN  CONST VOID  *Buffer
  )
{
#if 0
    EpTx (EndpointIndex, Buffer, Size);
    return EFI_SUCCESS;
#else
  return EFI_SUCCESS;
#endif
}

USB_DEVICE_PROTOCOL mUsbDevice = {
  DwUsb3Start,
  DwUsb3Send
};

EFI_STATUS
EFIAPI
DwUsb3EntryPoint (
  IN EFI_HANDLE                            ImageHandle,
  IN EFI_SYSTEM_TABLE                      *SystemTable
  )
{
  EFI_STATUS      Status;

  Status = gBS->LocateProtocol (&gDwUsbProtocolGuid, NULL, (VOID **) &DwUsb);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  Status = DwUsb->PhyInit(USB_DEVICE_MODE);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return gBS->InstallProtocolInterface (
		  &ImageHandle,
		  &gUsbDeviceProtocolGuid,
		  EFI_NATIVE_INTERFACE,
		  &mUsbDevice
		  );
}
