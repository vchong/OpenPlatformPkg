/** @file
*
*  Copyright (c) 2015-2016, Linaro Ltd. All rights reserved.
*  Copyright (c) 2015-2016, Hisilicon Ltd. All rights reserved.
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#include <Library/BaseMemoryLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PrintLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>

#include <Protocol/Abootimg.h>
#include <Protocol/BlockIo.h>

#include <Hi6220.h>
#include <libfdt.h>

#include "Hi6220RegsPeri.h"

#define SERIAL_NUMBER_SIZE               17
#define SERIAL_NUMBER_BLOCK_SIZE         EFI_PAGE_SIZE
#define SERIAL_NUMBER_LBA                1024
#define RANDOM_MAX                       0x7FFFFFFFFFFFFFFF
#define RANDOM_MAGIC                     0x9A4DBEAF

typedef struct {
  UINT64        Magic;
  UINT64        Data;
  CHAR16        UnicodeSN[SERIAL_NUMBER_SIZE];
} RANDOM_SERIAL_NUMBER;

STATIC
VOID
UartInit (
  IN VOID
  )
{
  UINT32     Val;

  /* make UART1 out of reset */
  MmioWrite32 (PERI_CTRL_BASE + SC_PERIPH_RSTDIS3, PERIPH_RST3_UART1);
  MmioWrite32 (PERI_CTRL_BASE + SC_PERIPH_CLKEN3, PERIPH_RST3_UART1);
  /* make UART2 out of reset */
  MmioWrite32 (PERI_CTRL_BASE + SC_PERIPH_RSTDIS3, PERIPH_RST3_UART2);
  MmioWrite32 (PERI_CTRL_BASE + SC_PERIPH_CLKEN3, PERIPH_RST3_UART2);
  /* make UART3 out of reset */
  MmioWrite32 (PERI_CTRL_BASE + SC_PERIPH_RSTDIS3, PERIPH_RST3_UART3);
  MmioWrite32 (PERI_CTRL_BASE + SC_PERIPH_CLKEN3, PERIPH_RST3_UART3);
  /* make UART4 out of reset */
  MmioWrite32 (PERI_CTRL_BASE + SC_PERIPH_RSTDIS3, PERIPH_RST3_UART4);
  MmioWrite32 (PERI_CTRL_BASE + SC_PERIPH_CLKEN3, PERIPH_RST3_UART4);

  /* make DW_MMC2 out of reset */
  MmioWrite32 (PERI_CTRL_BASE + SC_PERIPH_RSTDIS0, PERIPH_RST0_MMC2);

  /* enable clock for BT/WIFI */
  Val = MmioRead32 (PMUSSI_REG(0x1c)) | 0x40;
  MmioWrite32 (PMUSSI_REG(0x1c), Val);
}

STATIC
VOID
MtcmosInit (
  IN VOID
  )
{
  UINT32     Data;

  /* enable MTCMOS for GPU */
  MmioWrite32 (AO_CTRL_BASE + SC_PW_MTCMOS_EN0, PW_EN0_G3D);
  do {
    Data = MmioRead32 (AO_CTRL_BASE + SC_PW_MTCMOS_ACK_STAT0);
  } while ((Data & PW_EN0_G3D) == 0);
}

EFI_STATUS
HiKeyInitPeripherals (
  IN VOID
  )
{
  UINT32     Data, Bits;

  /* make I2C0/I2C1/I2C2/SPI0 out of reset */
  Bits = PERIPH_RST3_I2C0 | PERIPH_RST3_I2C1 | PERIPH_RST3_I2C2 | \
         PERIPH_RST3_SSP;
  MmioWrite32 (PERI_CTRL_BASE + SC_PERIPH_RSTDIS3, Bits);

  do {
    Data = MmioRead32 (PERI_CTRL_BASE + SC_PERIPH_RSTSTAT3);
  } while (Data & Bits);

  UartInit ();
  MtcmosInit ();

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
AbootimgAppendKernelArgs (
  IN CHAR16            *Args,
  IN UINTN              Size
  )
{
  EFI_STATUS                  Status;
  EFI_BLOCK_IO_PROTOCOL      *BlockIoProtocol;
  VOID                       *DataPtr;
  RANDOM_SERIAL_NUMBER       *RandomSN;
  EFI_DEVICE_PATH_PROTOCOL   *FlashDevicePath;
  EFI_HANDLE                  FlashHandle;

  if (Args == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  FlashDevicePath = ConvertTextToDevicePath ((CHAR16*)FixedPcdGetPtr (PcdAndroidFastbootNvmDevicePath));
  Status = gBS->LocateDevicePath (&gEfiBlockIoProtocolGuid, &FlashDevicePath, &FlashHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Warning: Couldn't locate Android NVM device (status: %r)\n", Status));
    // Failing to locate partitions should not prevent to do other Android FastBoot actions
    return EFI_SUCCESS;
  }
  Status = gBS->OpenProtocol (
                  FlashHandle,
                  &gEfiBlockIoProtocolGuid,
                  (VOID **) &BlockIoProtocol,
                  gImageHandle,
                  NULL,
                  EFI_OPEN_PROTOCOL_GET_PROTOCOL
                  );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_WARN, "Warning: Couldn't open block device (status: %r)\n", Status));
    return EFI_DEVICE_ERROR;
  }

  DataPtr = AllocatePages (1);
  if (DataPtr == NULL) {
    return EFI_BUFFER_TOO_SMALL;
  }
  Status = BlockIoProtocol->ReadBlocks (
                              BlockIoProtocol,
                              BlockIoProtocol->Media->MediaId,
                              SERIAL_NUMBER_LBA,
                              SERIAL_NUMBER_BLOCK_SIZE,
                              DataPtr
                              );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_WARN, "Warning: Failed on reading blocks\n"));
    goto Exit;
  }
  RandomSN = (RANDOM_SERIAL_NUMBER *)DataPtr;
  if (RandomSN->Magic != RANDOM_MAGIC) {
    UnicodeSPrint(
      RandomSN->UnicodeSN, SERIAL_NUMBER_SIZE * sizeof (CHAR16),
      L"0123456789abcdef"
      );
  }
  UnicodeSPrint (
    Args + StrLen (Args), Size - StrLen (Args),
    L" androidboot.serialno=%s",
    RandomSN->UnicodeSN
    );
  FreePages (DataPtr, 1);
  return EFI_SUCCESS;
Exit:
  FreePages (DataPtr, 1);
  return Status;
}

EFI_STATUS
EFIAPI
AbootimgUpdateDtb (
  IN  EFI_PHYSICAL_ADDRESS        OrigFdtBase,
  OUT EFI_PHYSICAL_ADDRESS       *NewFdtBase
  )
{
  UINTN             FdtSize, NumPages;
  INTN              err;
  EFI_STATUS        Status;

  //
  // Sanity checks on the original FDT blob.
  //
  err = fdt_check_header ((VOID*)(UINTN)OrigFdtBase);
  if (err != 0) {
    DEBUG ((DEBUG_ERROR, "ERROR: Device Tree header not valid (err:%d)\n", err));
    return EFI_INVALID_PARAMETER;
  }

  //
  // Store the FDT as Runtime Service Data to prevent the Kernel from
  // overwritting its data.
  //
  FdtSize = fdt_totalsize ((VOID *)(UINTN)OrigFdtBase);
  NumPages = EFI_SIZE_TO_PAGES (FdtSize) + 20;
  Status = gBS->AllocatePages (
                  AllocateAnyPages, EfiRuntimeServicesData,
                  NumPages, NewFdtBase);
  if (EFI_ERROR (Status)) {
    return EFI_BUFFER_TOO_SMALL;
  }

  CopyMem (
    (VOID*)(UINTN)*NewFdtBase,
    (VOID*)(UINTN)OrigFdtBase,
    FdtSize
    );

  fdt_pack ((VOID*)(UINTN)*NewFdtBase);
  err = fdt_check_header ((VOID*)(UINTN)*NewFdtBase);
  if (err != 0) {
    DEBUG ((DEBUG_ERROR, "ERROR: Device Tree header not valid (err:%d)\n", err));
    gBS->FreePages (*NewFdtBase, NumPages);
    return EFI_INVALID_PARAMETER;
  }
  return EFI_SUCCESS;
}

ABOOTIMG_PROTOCOL mAbootimg = {
  AbootimgAppendKernelArgs,
  AbootimgUpdateDtb
};

EFI_STATUS
EFIAPI
HiKeyEntryPoint (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS            Status;

  Status = HiKeyInitPeripherals ();
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = gBS->InstallProtocolInterface (
                  &ImageHandle,
                  &gAbootimgProtocolGuid,
                  EFI_NATIVE_INTERFACE,
                  &mAbootimg
                  );
  return Status;
}
