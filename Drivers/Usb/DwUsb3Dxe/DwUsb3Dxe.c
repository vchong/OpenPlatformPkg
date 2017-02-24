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

#define USB_TYPE_LENGTH              16
#define USB_BLOCK_HIGH_SPEED_SIZE    512
#define DATA_SIZE 32768
#define CMD_SIZE 512
#define MATCH_CMD_LITERAL(Cmd, Buf) !AsciiStrnCmp (Cmd, Buf, sizeof (Cmd) - 1)

// The time between interrupt polls, in units of 100 nanoseconds
// 10 Microseconds
#define DW_INTERRUPT_POLL_PERIOD 10000

EFI_GUID  gDwUsbProtocolGuid = DW_USB_PROTOCOL_GUID;

STATIC DW_USB_PROTOCOL          *DwUsb;

STATIC
VOID
DwUsb3Init (
  VOID
  )
{
#if 0
  VOID     *buf;
  UINT32   data;

  buf = UncachedAllocatePages (16);
  gDmaDesc = buf;
  gDmaDescEp0 = gDmaDesc + sizeof(DWC_OTG_DEV_DMA_DESC);
  gDmaDescIn = gDmaDescEp0 + sizeof(DWC_OTG_DEV_DMA_DESC);
  pCtrlReq = (USB_DEVICE_REQUEST *)gDmaDescIn + sizeof(DWC_OTG_DEV_DMA_DESC);

  SetMem(gDmaDesc, sizeof(DWC_OTG_DEV_DMA_DESC), 0);
  SetMem(gDmaDescEp0, sizeof(DWC_OTG_DEV_DMA_DESC), 0);
  SetMem(gDmaDescIn, sizeof(DWC_OTG_DEV_DMA_DESC), 0);

  /*Reset usb controller.*/
  /* Wait for OTG AHB master idle */
  do {
    data = MmioRead32 (DW_USB_BASE + GRSTCTL) & GRSTCTL_AHBIDLE;
  } while (data == 0);

  /* OTG: Assert Software Reset */
  MmioWrite32 (DW_USB_BASE + GRSTCTL, GRSTCTL_CSFTRST);

  /* Wait for OTG to ack reset */
  while (MmioRead32 (DW_USB_BASE + GRSTCTL) & GRSTCTL_CSFTRST);

  /* Wait for OTG AHB master idle */
  while ((MmioRead32 (DW_USB_BASE + GRSTCTL) & GRSTCTL_AHBIDLE) == 0);

  MmioWrite32 (DW_USB_BASE + GDFIFOCFG, DATA_FIFO_CONFIG);
  MmioWrite32 (DW_USB_BASE + GRXFSIZ, RX_SIZE);
  MmioWrite32 (DW_USB_BASE + GNPTXFSIZ, ENDPOINT_TX_SIZE);
  MmioWrite32 (DW_USB_BASE + DIEPTXF1, DATA_IN_ENDPOINT_TX_FIFO1);
  MmioWrite32 (DW_USB_BASE + DIEPTXF2, DATA_IN_ENDPOINT_TX_FIFO2);
  MmioWrite32 (DW_USB_BASE + DIEPTXF3, DATA_IN_ENDPOINT_TX_FIFO3);
  MmioWrite32 (DW_USB_BASE + DIEPTXF4, DATA_IN_ENDPOINT_TX_FIFO4);
  MmioWrite32 (DW_USB_BASE + DIEPTXF5, DATA_IN_ENDPOINT_TX_FIFO5);
  MmioWrite32 (DW_USB_BASE + DIEPTXF6, DATA_IN_ENDPOINT_TX_FIFO6);
  MmioWrite32 (DW_USB_BASE + DIEPTXF7, DATA_IN_ENDPOINT_TX_FIFO7);
  MmioWrite32 (DW_USB_BASE + DIEPTXF8, DATA_IN_ENDPOINT_TX_FIFO8);
  MmioWrite32 (DW_USB_BASE + DIEPTXF9, DATA_IN_ENDPOINT_TX_FIFO9);
  MmioWrite32 (DW_USB_BASE + DIEPTXF10, DATA_IN_ENDPOINT_TX_FIFO10);
  MmioWrite32 (DW_USB_BASE + DIEPTXF11, DATA_IN_ENDPOINT_TX_FIFO11);
  MmioWrite32 (DW_USB_BASE + DIEPTXF12, DATA_IN_ENDPOINT_TX_FIFO12);
  MmioWrite32 (DW_USB_BASE + DIEPTXF13, DATA_IN_ENDPOINT_TX_FIFO13);
  MmioWrite32 (DW_USB_BASE + DIEPTXF14, DATA_IN_ENDPOINT_TX_FIFO14);
  MmioWrite32 (DW_USB_BASE + DIEPTXF15, DATA_IN_ENDPOINT_TX_FIFO15);

  /*
   * set Periodic TxFIFO Empty Level,
   * Non-Periodic TxFIFO Empty Level,
   * Enable DMA, Unmask Global Intr
   */
  MmioWrite32 (DW_USB_BASE + GAHBCFG, GAHBCFG_CTRL_MASK);

  /* select 8bit UTMI+, ULPI Inerface */
  MmioWrite32 (DW_USB_BASE + GUSBCFG, GUSBCFG_PHY_8BIT);

  /* Detect usb work mode,host or device? */
  do {
    data = MmioRead32 (DW_USB_BASE + GINTSTS);
  } while (data & GINTSTS_CURMODE_HOST);
  MicroSecondDelay(3);

  /*Init global and device mode csr register.*/
  /*set Non-Zero-Length status out handshake */
  data = (0x20 << DCFG_EPMISCNT_SHIFT) | DCFG_NZ_STS_OUT_HSHK;
  MmioWrite32 (DW_USB_BASE + DCFG, data);

  /* Interrupt unmask: IN event, OUT event, bus reset */
  data = GINTSTS_OEPINT | GINTSTS_IEPINT | GINTSTS_ENUMDONE | GINTSTS_USBRST;
  MmioWrite32 (DW_USB_BASE + GINTMSK, data);

  do {
    data = MmioRead32 (DW_USB_BASE + GINTSTS) & GINTSTS_ENUMDONE;
  } while (data);

  /* Clear any pending OTG Interrupts */
  MmioWrite32 (DW_USB_BASE + GOTGINT, ~0);
  /* Clear any pending interrupts */
  MmioWrite32 (DW_USB_BASE + GINTSTS, ~0);
  MmioWrite32 (DW_USB_BASE + GINTMSK, ~0);
  data = MmioRead32 (DW_USB_BASE + GOTGINT);
  data &= ~0x3000;
  MmioWrite32 (DW_USB_BASE + GOTGINT, data);

  /*endpoint settings cfg*/
  ResetEndpoints();
  MicroSecondDelay (1);

  /*init finish. and ready to transfer data*/

  /* Soft Disconnect */
  MmioWrite32 (DW_USB_BASE + DCTL, DCTL_PWRONPRGDONE | DCTL_SFTDISCON);
  MicroSecondDelay(10000);

  /* Soft Reconnect */
  MmioWrite32 (DW_USB_BASE + DCTL, DCTL_PWRONPRGDONE);
#else
#endif
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
  DwUsb3Init ();
  return EFI_SUCCESS;
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
