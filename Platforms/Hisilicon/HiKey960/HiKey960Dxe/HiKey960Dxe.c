/** @file
*
*  Copyright (c) 2016-2017, Linaro Ltd. All rights reserved.
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

#include <Guid/EventGroup.h>

#include <Hi3660.h>
#include <Hkadc.h>
#include <libfdt.h>

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/NonDiscoverableDeviceRegistrationLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>

#include <Protocol/Abootimg.h>
#include <Protocol/NonDiscoverableDevice.h>

#define ADC_ADCIN0                       0
#define ADC_ADCIN1                       1
#define ADC_ADCIN2                       2

#define HKADC_DATA_GRADE0                0
#define HKADC_DATA_GRADE1                100
#define HKADC_DATA_GRADE2                300
#define HKADC_DATA_GRADE3                500
#define HKADC_DATA_GRADE4                700
#define HKADC_DATA_GRADE5                900
#define HKADC_DATA_GRADE6                1100
#define HKADC_DATA_GRADE7                1300
#define HKADC_DATA_GRADE8                1500
#define HKADC_DATA_GRADE9                1700
#define HKADC_DATA_GRADE10               1800

#define BOARDID_VALUE0                   0
#define BOARDID_VALUE1                   1
#define BOARDID_VALUE2                   2
#define BOARDID_VALUE3                   3
#define BOARDID_VALUE4                   4
#define BOARDID_VALUE5                   5
#define BOARDID_VALUE6                   6
#define BOARDID_VALUE7                   7
#define BOARDID_VALUE8                   8
#define BOARDID_VALUE9                   9
#define BOARDID_UNKNOW                   0xF

#define BOARDID3_BASE                    5

#define HIKEY960_BOARDID_V1              5300
#define HIKEY960_BOARDID_V2              5301

#define HIKEY960_COMPATIBLE_LEDS_V1      "gpio-leds_v1"
#define HIKEY960_COMPATIBLE_LEDS_V2      "gpio-leds_v2"
#define HIKEY960_COMPATIBLE_HUB_V1       "hisilicon,gpio_hubv1"
#define HIKEY960_COMPATIBLE_HUB_V2       "hisilicon,gpio_hubv2"

STATIC UINTN    mBoardId;

STATIC
VOID
InitAdc (
  VOID
  )
{
  // reset hkadc
  MmioWrite32 (CRG_PERRSTEN2, PERRSTEN2_HKADCSSI);
  // wait a few clock cycles
  MicroSecondDelay (2);
  MmioWrite32 (CRG_PERRSTDIS2, PERRSTEN2_HKADCSSI);
  MicroSecondDelay (2);
  // enable hkadc clock
  MmioWrite32 (CRG_PERDIS2, PEREN2_HKADCSSI);
  MicroSecondDelay (2);
  MmioWrite32 (CRG_PEREN2, PEREN2_HKADCSSI);
  MicroSecondDelay (2);
}

STATIC
EFI_STATUS
AdcGetAdc (
  IN  UINTN         Channel,
  OUT UINTN         *Value
  )
{
  UINT32            Data;
  UINT16            Value1, Value0;

  if (Channel > HKADC_CHANNEL_MAX) {
    DEBUG ((DEBUG_ERROR, "invalid channel:%d\n", Channel));
    return EFI_OUT_OF_RESOURCES;
  }
  // configure the read/write operation for external HKADC
  MmioWrite32 (HKADC_WR01_DATA, HKADC_WR01_VALUE | Channel);
  MmioWrite32 (HKADC_WR23_DATA, HKADC_WR23_VALUE);
  MmioWrite32 (HKADC_WR45_DATA, HKADC_WR45_VALUE);
  // configure the number of accessing registers
  MmioWrite32 (HKADC_WR_NUM, HKADC_WR_NUM_VALUE);
  // configure delay of accessing registers
  MmioWrite32 (HKADC_DELAY01, HKADC_CHANNEL0_DELAY01_VALUE);
  MmioWrite32 (HKADC_DELAY23, HKADC_DELAY23_VALUE);

  // start HKADC
  MmioWrite32 (HKADC_DSP_START, 1);
  do {
    Data = MmioRead32 (HKADC_DSP_START);
  } while (Data & 1);

  // convert AD result
  Value1 = (UINT16)MmioRead32 (HKADC_DSP_RD2_DATA);
  Value0 = (UINT16)MmioRead32 (HKADC_DSP_RD3_DATA);

  Data = ((Value1 << 4) & HKADC_VALUE_HIGH) | ((Value0 >> 4) & HKADC_VALUE_LOW);
  *Value = Data;
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
AdcGetValue (
  IN UINTN         Channel,
  IN OUT UINTN     *Value
  )
{
  EFI_STATUS       Status;
  UINTN            Result;

  Status = AdcGetAdc (Channel, Value);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // convert ADC value to micro-volt
  Result = ((*Value & HKADC_VALID_VALUE) * HKADC_VREF_1V8) / HKADC_ACCURACY;
  *Value = Result;
  return EFI_SUCCESS;
}

STATIC
UINTN
AdcinDataRemap (
  IN UINTN           AdcinValue
  )
{
  UINTN              Result;

  if (AdcinValue < HKADC_DATA_GRADE0) {
    Result = BOARDID_UNKNOW;
  } else if (AdcinValue < HKADC_DATA_GRADE1) {
    Result = BOARDID_VALUE0;
  } else if (AdcinValue < HKADC_DATA_GRADE2) {
    Result = BOARDID_VALUE1;
  } else if (AdcinValue < HKADC_DATA_GRADE3) {
    Result = BOARDID_VALUE2;
  } else if (AdcinValue < HKADC_DATA_GRADE4) {
    Result = BOARDID_VALUE3;
  } else if (AdcinValue < HKADC_DATA_GRADE5) {
    Result = BOARDID_VALUE4;
  } else if (AdcinValue < HKADC_DATA_GRADE6) {
    Result = BOARDID_VALUE5;
  } else if (AdcinValue < HKADC_DATA_GRADE7) {
    Result = BOARDID_VALUE6;
  } else if (AdcinValue < HKADC_DATA_GRADE8) {
    Result = BOARDID_VALUE7;
  } else if (AdcinValue < HKADC_DATA_GRADE9) {
    Result = BOARDID_VALUE8;
  } else if (AdcinValue < HKADC_DATA_GRADE10) {
    Result = BOARDID_VALUE9;
  } else {
    Result = BOARDID_UNKNOW;
  }
  return Result;
}

STATIC
EFI_STATUS
InitBoardId (
  OUT UINTN          *Id
  )
{
  UINTN              Adcin0, Adcin1, Adcin2;
  UINTN              Adcin0Remap, Adcin1Remap, Adcin2Remap;

  InitAdc ();

  // read ADC channel0 data
  AdcGetValue (ADC_ADCIN0, &Adcin0);
  DEBUG ((DEBUG_ERROR, "[BDID]Adcin0:%d\n", Adcin0));
  Adcin0Remap = AdcinDataRemap (Adcin0);
  DEBUG ((DEBUG_ERROR, "[BDID]Adcin0Remap:%d\n", Adcin0Remap));
  if (Adcin0Remap == BOARDID_UNKNOW) {
    return EFI_INVALID_PARAMETER;
  }
  // read ADC channel1 data
  AdcGetValue (ADC_ADCIN1, &Adcin1);
  DEBUG ((DEBUG_ERROR, "[BDID]Adcin1:%d\n", Adcin1));
  Adcin1Remap = AdcinDataRemap (Adcin1);
  DEBUG ((DEBUG_ERROR, "[BDID]Adcin1Remap:%d\n", Adcin1Remap));
  if (Adcin1Remap == BOARDID_UNKNOW) {
    return EFI_INVALID_PARAMETER;
  }
  // read ADC channel2 data
  AdcGetValue (ADC_ADCIN2, &Adcin2);
  DEBUG ((DEBUG_ERROR, "[BDID]Adcin2:%d\n", Adcin2));
  Adcin2Remap = AdcinDataRemap (Adcin2);
  DEBUG ((DEBUG_ERROR, "[BDID]Adcin2Remap:%d\n", Adcin2Remap));
  if (Adcin2Remap == BOARDID_UNKNOW) {
    return EFI_INVALID_PARAMETER;
  }
  *Id = BOARDID3_BASE * 1000 + (Adcin2Remap * 100) + (Adcin1Remap * 10) + Adcin0Remap;
  DEBUG ((DEBUG_ERROR, "[BDID]boardid: %d\n", *Id));
  return EFI_SUCCESS;
}


/**
  Notification function of the event defined as belonging to the
  EFI_END_OF_DXE_EVENT_GROUP_GUID event group that was created in
  the entry point of the driver.

  This function is called when an event belonging to the
  EFI_END_OF_DXE_EVENT_GROUP_GUID event group is signalled. Such an
  event is signalled once at the end of the dispatching of all
  drivers (end of the so called DXE phase).

  @param[in]  Event    Event declared in the entry point of the driver whose
                       notification function is being invoked.
  @param[in]  Context  NULL
**/
STATIC
VOID
OnEndOfDxe (
  IN EFI_EVENT  Event,
  IN VOID       *Context
  )
{
}

EFI_STATUS
EFIAPI
AbootimgAppendKernelArgs (
  IN CHAR16            *Args,
  IN UINTN              Size
  )
{
  if (Args == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  if (mBoardId == HIKEY960_BOARDID_V1) {
    StrCatS (Args, Size, L" earlycon=pl011,0xfdf05000,115200 console=ttyAMA5");
  } else {
    StrCatS (Args, Size, L" earlycon=pl011,0xfff32000,115200 console=ttyAMA6");
  }
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
AbootimgUpdateDtb (
  IN  EFI_PHYSICAL_ADDRESS        OrigFdtBase,
  OUT EFI_PHYSICAL_ADDRESS       *NewFdtBase
  )
{
  //UINT8            *FdtPtr;
  UINTN             FdtSize, NumPages;
  INTN              err, offset;
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

  if (mBoardId == HIKEY960_BOARDID_V1) {
    offset = fdt_node_offset_by_compatible (
               (VOID*)(UINTN)*NewFdtBase, -1, HIKEY960_COMPATIBLE_LEDS_V1
               );
  } else {
    offset = fdt_node_offset_by_compatible (
               (VOID*)(UINTN)*NewFdtBase, -1, HIKEY960_COMPATIBLE_LEDS_V2
               );
  }
  if (offset < 0) {
    DEBUG ((DEBUG_ERROR, "ERROR: Failed to find node with compatible (err:%d)\n", err));
    gBS->FreePages (*NewFdtBase, NumPages);
    return EFI_INVALID_PARAMETER;
  }
  err = fdt_setprop_string ((VOID*)(UINTN)*NewFdtBase, offset, "status", "ok");
  if (err) {
    DEBUG ((DEBUG_ERROR, "ERROR: Failed to update status property\n"));
    return EFI_INVALID_PARAMETER;
  }
  err = fdt_set_name ((VOID*)(UINTN)*NewFdtBase, offset, "gpio-leds");
  if (err) {
    DEBUG ((DEBUG_ERROR, "ERROR: Failed to update compatible name\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (mBoardId == HIKEY960_BOARDID_V1) {
    offset = fdt_node_offset_by_compatible (
               (VOID*)(UINTN)*NewFdtBase, -1, HIKEY960_COMPATIBLE_HUB_V1
               );
  } else {
    offset = fdt_node_offset_by_compatible (
               (VOID*)(UINTN)*NewFdtBase, -1, HIKEY960_COMPATIBLE_HUB_V2
               );
  }
  if (offset < 0) {
    DEBUG ((DEBUG_ERROR, "ERROR: Failed to find node with compatible (err:%d)\n", err));
    gBS->FreePages (*NewFdtBase, NumPages);
    return EFI_INVALID_PARAMETER;
  }
  err = fdt_setprop_string ((VOID*)(UINTN)*NewFdtBase, offset, "status", "ok");
  if (err) {
    DEBUG ((DEBUG_ERROR, "ERROR: Failed to update status property\n"));
    return EFI_INVALID_PARAMETER;
  }

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
HiKey960EntryPoint (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS            Status;
  EFI_EVENT             EndOfDxeEvent;

  Status = InitBoardId (&mBoardId);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  //
  // Create an event belonging to the "gEfiEndOfDxeEventGroupGuid" group.
  // The "OnEndOfDxe()" function is declared as the call back function.
  // It will be called at the end of the DXE phase when an event of the
  // same group is signalled to inform about the end of the DXE phase.
  // Install the INSTALL_FDT_PROTOCOL protocol.
  //
  Status = gBS->CreateEventEx (
                  EVT_NOTIFY_SIGNAL,
                  TPL_CALLBACK,
                  OnEndOfDxe,
                  NULL,
                  &gEfiEndOfDxeEventGroupGuid,
                  &EndOfDxeEvent
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // RegisterNonDicoverableMmioDevice
  Status = RegisterNonDiscoverableMmioDevice (
             NonDiscoverableDeviceTypeUfs,
             NonDiscoverableDeviceDmaTypeNonCoherent,
             NULL,
             NULL,
             1,
             FixedPcdGet32 (PcdDwUfsHcDxeBaseAddress),
             SIZE_4KB
             );
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
