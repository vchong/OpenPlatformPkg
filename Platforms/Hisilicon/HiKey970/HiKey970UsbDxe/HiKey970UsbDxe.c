/** @file
*
*  Copyright (c) 2018, Linaro. All rights reserved.
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
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/UsbSerialNumberLib.h>

#include <Protocol/BlockIo.h>
#include <Protocol/DwUsb.h>

#define SCTRL_SCDEEPSLEEPED            0xFFF0A000
#define USB_CLK_SELECTED               (1 << 20)

#define REG_BASE_USB_OTG_BC            0xFF200000
#define USB3OTG_CTRL0                  (REG_BASE_USB_OTG_BC + 0x00)
#define USB3OTG_CTRL3                  (REG_BASE_USB_OTG_BC + 0x0C)
#define USB3OTG_CTRL4                  (REG_BASE_USB_OTG_BC + 0x10)
#define USB3OTG_CTRL5                  (REG_BASE_USB_OTG_BC + 0x14)
#define USB3OTG_CTRL7                  (REG_BASE_USB_OTG_BC + 0x1C)
#define USB_MISC_CFG50                 (REG_BASE_USB_OTG_BC + 0x50)
#define USB_MISC_CFG54                 (REG_BASE_USB_OTG_BC + 0x54)
#define USB_MISC_CFG58                 (REG_BASE_USB_OTG_BC + 0x58)
#define USB_MISC_CFG5C                 (REG_BASE_USB_OTG_BC + 0x5C)
#define USB_MISC_CFGA0                 (REG_BASE_USB_OTG_BC + 0xA0)

#define CTRL0_USB3_VBUSVLD             (1 << 7)
#define CTRL0_USB3_VBUSVLD_SEL         (1 << 6)

#define CTRL3_USB2_VBUSVLDEXT0         (1 << 6)
#define CTRL3_USB2_VBUSVLDEXTSEL0      (1 << 5)

#define CTRL5_USB2_SIDDQ               (1 << 0)

#define CFG50_USB3_PHY_TEST_POWERDOWN  (1 << 23)

#define CFG54_USB31PHY_CR_ADDR_MASK    (0xFFFF)
#define CFG54_USB31PHY_CR_ADDR_SHIFT   (16)
#define CFG54_USB3PHY_REF_USE_PAD      (1 << 12)
#define CFG54_PHY0_PMA_PWR_STABLE      (1 << 11)
#define CFG54_PHY0_PCS_PWR_STABLE      (1 << 9)
#define CFG54_USB31PHY_CR_ACK          (1 << 7)
#define CFG54_USB31PHY_CR_WR_EN        (1 << 5)
#define CFG54_USB31PHY_CR_SEL          (1 << 4)
#define CFG54_USB31PHY_CR_RD_EN        (1 << 3)
#define CFG54_USB31PHY_CR_CLK          (1 << 2)
#define CFG54_USB3_PHY0_ANA_PWR_EN     (1 << 1)

#define CFG58_USB31PHY_CR_DATA_MASK     (0xFFFF)
#define CFG58_USB31PHY_CR_DATA_RD_START (16)

#define CFG5C_USB3_PHY0_SS_MPLLA_SSC_EN (1 << 1)

#define CFGA0_VAUX_RESET               (1 << 9)
#define CFGA0_USB31C_RESET             (1 << 8)
#define CFGA0_USB2PHY_REFCLK_SELECT    (1 << 4)
#define CFGA0_USB3PHY_RESET            (1 << 1)
#define CFGA0_USB2PHY_POR              (1 << 0)

#define CTRL7_USB2_REFCLKSEL_ABB       (3 << 3)
#define CTRL7_USB2_REFCLKSEL_PAD       (2 << 3)

#define REG_BASE_USB3_TCA              0xFF200200
#define TCA_CLK_RST                    (REG_BASE_USB3_TCA + 0x00)
#define TCA_INTR_EN                    (REG_BASE_USB3_TCA + 0x04)
#define TCA_INTR_STS                   (REG_BASE_USB3_TCA + 0x08)
#define TCA_GCFG                       (REG_BASE_USB3_TCA + 0x10)
#define TCA_TCPC                       (REG_BASE_USB3_TCA + 0x14)
#define TCA_VBUS_CTRL                  (REG_BASE_USB3_TCA + 0x40)

#define INTR_EN_XA_TIMEOUT_EVT_EN      (1 << 1)
#define INTR_EN_XA_ACK_EVT_EN          (1 << 0)

#define CLK_RST_TCA_REF_CLK_EN         (1 << 1)
#define CLK_RST_SUSPEND_CLK_EN         (1 << 0)

#define GCFG_ROLE_HSTDEV               (1 << 4)

#define TCPC_VALID                     (1 << 4)
#define TCPC_LOW_POWER_EN              (1 << 3)
#define TCPC_MUX_CONTROL_MASK          (3 << 0)
#define TCPC_MUX_CONTROL_USB31         (1 << 0)

#define VBUS_CTRL_POWERPRESENT_OVERRD  (3 << 2)
#define VBUS_CTRL_VBUSVALID_OVERRD     (3 << 0)

#define REG_BASE_PCTRL                 0xE8A09000
#define PCTRL_CTRL3                    (REG_BASE_PCTRL + 0x10)
#define PCTRL_CTRL24                   (REG_BASE_PCTRL + 0x64)

#define PCTRL_CTRL3_USB_TCXO_EN        (1 << 1)
#define PCTRL_CTRL24_USB3PHY_3MUX1_SEL (1 << 25)

#define REG_BASE_PERI_CRG              0xFFF35000
#define PERI_CRG_PEREN0                (REG_BASE_PERI_CRG + 0x00)
#define PERI_CRG_PERDIS0               (REG_BASE_PERI_CRG + 0x04)
#define PERI_CRG_PEREN4                (REG_BASE_PERI_CRG + 0x40)
#define PERI_CRG_PERDIS4               (REG_BASE_PERI_CRG + 0x44)
#define PERI_CRG_PERRSTEN4             (REG_BASE_PERI_CRG + 0x90)
#define PERI_CRG_PERRSTDIS4            (REG_BASE_PERI_CRG + 0x94)
#define PERI_CRG_ISODIS                (REG_BASE_PERI_CRG + 0x148)
#define PERI_CRG_PEREN6                (REG_BASE_PERI_CRG + 0x410)
#define PERI_CRG_PERDIS6               (REG_BASE_PERI_CRG + 0x414)

#define GT_CLK_USB2PHY_REF             (1 << 19)

#define IP_RST_USB3OTG_MISC            (1 << 7)
#define IP_RST_USB3OTG_32K             (1 << 6)

#define GT_ACLK_USB3OTG                (1 << 1)
#define GT_CLK_USB3OTG_REF             (1 << 0)

#define GT_HCLK_USB3OTG_MISC           (1 << 25)

#define USB_REFCLK_ISO_EN              (1 << 25)

#define USB_EYE_PARAM                  0xFDFEE4

#define SERIAL_NUMBER_LBA              20

#define PHY_CR_READ                    1
#define PHY_CR_WRITE                    0

#define TX_VBOOST_LVL_REG              (0xf)
#define TX_VBOOST_LVL_START            (6)
#define TX_VBOOST_LVL_ENABLE           (1 << 9)
#define TX_VBOOST_LVL                  (5)

STATIC EFI_HANDLE mFlashHandle;

UINTN
HiKey970GetUsbMode (
  IN VOID
  )
{
  return USB_DEVICE_MODE;
}

VOID
HiKey970UsbPhyCrClk (
  VOID
  )
{
  UINT32               Data;

  /* Clock up */
  Data = MmioRead32 (USB_MISC_CFG54);
  Data |= CFG54_USB31PHY_CR_CLK;
  MmioWrite32 (USB_MISC_CFG54, Data);

  /* Clock down */
  Data = MmioRead32 (USB_MISC_CFG54);
  Data &= ~CFG54_USB31PHY_CR_CLK;
  MmioWrite32 (USB_MISC_CFG54, Data);
}

VOID
HiKey970UsbPhyCrSetSel (
  VOID
  )
{
  UINT32               Data;

  Data = MmioRead32 (USB_MISC_CFG54);
  Data |= CFG54_USB31PHY_CR_SEL;
  MmioWrite32 (USB_MISC_CFG54, Data);
}

VOID
HiKey970UsbPhyCrStart (
  IN INT32             IsRead
  )
{
  UINT32               Data;

  Data = MmioRead32 (USB_MISC_CFG54);
  if (IsRead) {
      Data |= CFG54_USB31PHY_CR_RD_EN;
  } else {
      Data |= CFG54_USB31PHY_CR_WR_EN;
  }
  MmioWrite32 (USB_MISC_CFG54, Data);
  HiKey970UsbPhyCrClk();

  Data = MmioRead32 (USB_MISC_CFG54);
  Data &= ~(CFG54_USB31PHY_CR_RD_EN | CFG54_USB31PHY_CR_WR_EN);
  MmioWrite32 (USB_MISC_CFG54, Data);
}

VOID
HiKey970UsbPhyCrWaitAck (
  VOID
  )
{
  INT32                Timeout = 10000;
  UINT32               Data;

  while (TRUE) {
    Data = MmioRead32 (USB_MISC_CFG54);
    if ((Data & CFG54_USB31PHY_CR_ACK) == CFG54_USB31PHY_CR_ACK) {
      return;
    }
    HiKey970UsbPhyCrClk();
    if (Timeout-- <= 0) {
      DEBUG ((DEBUG_ERROR, "Wait PHY_CR_ACK timeout!\n"));
      return;
    }
  }
}

VOID
HiKey970UsbPhyCrSetAddr (
  IN UINT32            Addr
  )
{
  UINT32               Data;

  Data = MmioRead32 (USB_MISC_CFG54);
  Data &= ~(CFG54_USB31PHY_CR_ADDR_MASK << CFG54_USB31PHY_CR_ADDR_SHIFT);
  Data |= ((Addr & CFG54_USB31PHY_CR_ADDR_MASK) << CFG54_USB31PHY_CR_ADDR_SHIFT);
  MmioWrite32 (USB_MISC_CFG54, Data);
}

UINT16
HiKey970UsbPhyCrRead (
  IN UINT32            Addr
  )
{
  UINT32               Data;
  INT32                i;

  for (i = 0; i < 100; i++) {
      HiKey970UsbPhyCrClk();
  }

  HiKey970UsbPhyCrSetSel();
  HiKey970UsbPhyCrSetAddr(Addr);
  HiKey970UsbPhyCrStart(PHY_CR_READ);
  HiKey970UsbPhyCrWaitAck();

  Data = MmioRead32 (USB_MISC_CFG58);
  return (Data >> CFG58_USB31PHY_CR_DATA_RD_START) & CFG58_USB31PHY_CR_DATA_MASK;
}

VOID
HiKey970UsbPhyCrWrite (
  IN UINT32            Addr,
  IN UINT32            Value
  )
{
  HiKey970UsbPhyCrSetSel();
  HiKey970UsbPhyCrSetAddr(Addr);
  MmioWrite32 (USB_MISC_CFG58, Value & CFG58_USB31PHY_CR_DATA_MASK);
  HiKey970UsbPhyCrStart(PHY_CR_WRITE);
  HiKey970UsbPhyCrWaitAck();
}

VOID
HiKey970UsbSetEyeDiagramParam (
  VOID
  )
{
  UINT32               Data;

  /* set eye diagram for usb 2.0 */
  MmioWrite32 (USB3OTG_CTRL4, USB_EYE_PARAM);
  Data = HiKey970UsbPhyCrRead(TX_VBOOST_LVL_REG);
  Data |= (TX_VBOOST_LVL_ENABLE | (TX_VBOOST_LVL << TX_VBOOST_LVL_START));
  HiKey970UsbPhyCrWrite(TX_VBOOST_LVL_REG, Data);

  DEBUG ((
    DEBUG_INFO,
    "set phy tx vboost lvl 0x%x\n",
    HiKey970UsbPhyCrRead(TX_VBOOST_LVL_REG)
    ));
}

STATIC
INT32
HiKey970UsbIsAbbClkSelected (
  VOID
  )
{
  UINT32               Data;

  Data = MmioRead32 (SCTRL_SCDEEPSLEEPED);

  if ((Data & USB_CLK_SELECTED) == 0) {
      return 1;
  }

  return 0;
}

STATIC
VOID
HiKey970UsbReset (
  VOID
  )
{
  MmioWrite32 (PERI_CRG_PEREN0, GT_HCLK_USB3OTG_MISC);
  MmioWrite32 (PERI_CRG_PEREN4, GT_ACLK_USB3OTG | GT_CLK_USB3OTG_REF);
  MmioWrite32 (PERI_CRG_PERRSTDIS4, IP_RST_USB3OTG_MISC | IP_RST_USB3OTG_32K);

  // reset controller and phy
  MmioAnd32 (
    USB_MISC_CFGA0,
    ~(CFGA0_VAUX_RESET | CFGA0_USB31C_RESET | CFGA0_USB3PHY_RESET | CFGA0_USB2PHY_POR)
    );

  if (HiKey970UsbIsAbbClkSelected()) {
    MmioWrite32 (PCTRL_CTRL3, (PCTRL_CTRL3_USB_TCXO_EN << 16) | 0);
  } else {
    MmioWrite32 (PERI_CRG_PERDIS6, GT_CLK_USB2PHY_REF);
  }

  MmioWrite32 (PERI_CRG_PERRSTEN4, IP_RST_USB3OTG_MISC | IP_RST_USB3OTG_32K);
  MmioWrite32 (PERI_CRG_PERDIS4, GT_ACLK_USB3OTG | GT_CLK_USB3OTG_REF);
  MmioWrite32 (PERI_CRG_PERDIS0, GT_HCLK_USB3OTG_MISC);
  MicroSecondDelay (10000);
}

STATIC
VOID
HiKey970UsbRelease (
  VOID
  )
{
  UINT32               Data;

  MmioWrite32 (PERI_CRG_PEREN0, GT_HCLK_USB3OTG_MISC);
  MmioWrite32 (PERI_CRG_PEREN4, GT_ACLK_USB3OTG | GT_CLK_USB3OTG_REF);
  MmioWrite32 (PERI_CRG_PERRSTDIS4, IP_RST_USB3OTG_MISC | IP_RST_USB3OTG_32K);

  if (HiKey970UsbIsAbbClkSelected()) {
    /* enable USB REFCLK ISO */
    MmioWrite32 (PERI_CRG_ISODIS, USB_REFCLK_ISO_EN);

    /* enable USB_TCXO_EN */
    MmioWrite32 (PCTRL_CTRL3, (PCTRL_CTRL3_USB_TCXO_EN << 16) | PCTRL_CTRL3_USB_TCXO_EN);

    MicroSecondDelay (10000);

    MmioAnd32 (PCTRL_CTRL24, ~PCTRL_CTRL24_USB3PHY_3MUX1_SEL);
    MmioAnd32 (USB_MISC_CFGA0, ~CFGA0_USB2PHY_REFCLK_SELECT);
    MmioOr32 (USB3OTG_CTRL7, CTRL7_USB2_REFCLKSEL_ABB);
  } else {
    MmioOr32 (USB_MISC_CFG54, CFG54_USB3PHY_REF_USE_PAD);

    MmioOr32 (USB_MISC_CFGA0, CFGA0_USB2PHY_REFCLK_SELECT);

    Data = MmioRead32 (USB3OTG_CTRL7);
    Data &= ~(CTRL7_USB2_REFCLKSEL_ABB);
    Data |= CTRL7_USB2_REFCLKSEL_PAD;
    MmioWrite32 (USB3OTG_CTRL7, Data);

    MmioWrite32 (PERI_CRG_PEREN6, GT_CLK_USB2PHY_REF);
  }

  /* exit from IDDQ mode */
  MmioAnd32 (USB3OTG_CTRL5, ~CTRL5_USB2_SIDDQ);

  /* Release USB31 PHY out of TestPowerDown mode */
  MmioAnd32 (USB_MISC_CFG50, ~CFG50_USB3_PHY_TEST_POWERDOWN);

  /* Tell the PHY power is stable */
  MmioOr32 (
    USB_MISC_CFG54,
    CFG54_USB3_PHY0_ANA_PWR_EN | CFG54_PHY0_PCS_PWR_STABLE | CFG54_PHY0_PMA_PWR_STABLE
    );

  MmioWrite32 (TCA_INTR_STS, 0xFFFF);
  MmioWrite32 (TCA_INTR_EN, INTR_EN_XA_ACK_EVT_EN | INTR_EN_XA_TIMEOUT_EVT_EN);
  MmioAnd32 (TCA_CLK_RST, ~(CLK_RST_TCA_REF_CLK_EN | CLK_RST_SUSPEND_CLK_EN));
  MmioOr32 (TCA_GCFG, GCFG_ROLE_HSTDEV);

  Data = MmioRead32(TCA_TCPC);
  Data &= ~(TCPC_VALID | TCPC_LOW_POWER_EN | TCPC_MUX_CONTROL_MASK);
  Data |= (TCPC_VALID | TCPC_MUX_CONTROL_USB31);
  MmioWrite32 (TCA_TCPC, Data);

  MmioWrite32 (TCA_VBUS_CTRL, VBUS_CTRL_POWERPRESENT_OVERRD | VBUS_CTRL_VBUSVALID_OVERRD);

  /* Enable SSC */
  MmioOr32 (USB_MISC_CFG5C, CFG5C_USB3_PHY0_SS_MPLLA_SSC_EN);

  /* Unreset PHY */
  MmioOr32 (USB_MISC_CFGA0, CFGA0_USB3PHY_RESET | CFGA0_USB2PHY_POR);
  MicroSecondDelay (100);

  /* Unreset Controller */
  MmioOr32 (USB_MISC_CFGA0, CFGA0_VAUX_RESET | CFGA0_USB31C_RESET);
  MicroSecondDelay (100);

  MmioOr32 (USB3OTG_CTRL0, CTRL0_USB3_VBUSVLD | CTRL0_USB3_VBUSVLD_SEL);
  MmioOr32 (USB3OTG_CTRL3, CTRL3_USB2_VBUSVLDEXT0 | CTRL3_USB2_VBUSVLDEXTSEL0);
  MicroSecondDelay (100);

  HiKey970UsbSetEyeDiagramParam ();
}

EFI_STATUS
HiKey970UsbPhyInit (
  IN UINT8        Mode
  )
{
  HiKey970UsbReset ();
  MicroSecondDelay (10000);
  HiKey970UsbRelease ();

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
HiKey970UsbGetLang (
  OUT CHAR16            *Lang,
  OUT UINT8             *Length
  )
{
  if ((Lang == NULL) || (Length == NULL)) {
    return EFI_INVALID_PARAMETER;
  }
  Lang[0] = 0x409;
  *Length = sizeof (CHAR16);
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
HiKey970UsbGetManuFacturer (
  OUT CHAR16            *ManuFacturer,
  OUT UINT8             *Length
  )
{
  UINTN                  VariableSize;
  CHAR16                 DataUnicode[MANU_FACTURER_STRING_LENGTH];

  if ((ManuFacturer == NULL) || (Length == NULL)) {
    return EFI_INVALID_PARAMETER;
  }
  VariableSize = MANU_FACTURER_STRING_LENGTH * sizeof (CHAR16);
  ZeroMem (DataUnicode, MANU_FACTURER_STRING_LENGTH * sizeof(CHAR16));
  StrCpy (DataUnicode, L"96Boards");
  CopyMem (ManuFacturer, DataUnicode, VariableSize);
  *Length = StrSize (DataUnicode) - sizeof (CHAR16);
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
HiKey970UsbGetProduct (
  OUT CHAR16            *Product,
  OUT UINT8             *Length
  )
{
  UINTN                  VariableSize;
  CHAR16                 DataUnicode[PRODUCT_STRING_LENGTH];

  if ((Product == NULL) || (Length == NULL)) {
    return EFI_INVALID_PARAMETER;
  }
  VariableSize = PRODUCT_STRING_LENGTH * sizeof (CHAR16);
  ZeroMem (DataUnicode, PRODUCT_STRING_LENGTH * sizeof(CHAR16));
  StrCpy (DataUnicode, L"HiKey970");
  CopyMem (Product, DataUnicode, VariableSize);
  *Length = StrSize (DataUnicode) - sizeof (CHAR16);
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
HiKey970UsbGetSerialNo (
  OUT CHAR16            *SerialNo,
  OUT UINT8             *Length
  )
{
  EFI_STATUS                          Status;
  EFI_DEVICE_PATH_PROTOCOL           *FlashDevicePath;
  UINTN                  VariableSize;
  CHAR16                 DataUnicode[SERIAL_STRING_LENGTH];

  if (mFlashHandle == 0) {
    FlashDevicePath = ConvertTextToDevicePath ((CHAR16*)FixedPcdGetPtr (PcdAndroidFastbootNvmDevicePath));
    Status = gBS->LocateDevicePath (&gEfiBlockIoProtocolGuid, &FlashDevicePath, &mFlashHandle);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Warning: Couldn't locate Android NVM device (status: %r)\n", Status));
      // Failing to locate partitions should not prevent to do other Android FastBoot actions
      //return EFI_SUCCESS;
    }
  }

  if ((SerialNo == NULL) || (Length == NULL)) {
    return EFI_INVALID_PARAMETER;
  }
  if (mFlashHandle == 0) {
    VariableSize = SERIAL_STRING_LENGTH * sizeof (CHAR16);
    ZeroMem (DataUnicode, SERIAL_STRING_LENGTH * sizeof(CHAR16));
    StrCpy (DataUnicode, L"0123456789ABCDEF");
    CopyMem (SerialNo, DataUnicode, VariableSize);
    *Length = StrSize (DataUnicode) - sizeof (CHAR16);
    Status = EFI_SUCCESS;
  } else {
    Status = LoadSNFromBlock (mFlashHandle, SERIAL_NUMBER_LBA, SerialNo);
    *Length = StrSize (SerialNo) - sizeof (CHAR16);
  }
  return Status;
}

DW_USB_PROTOCOL mDwUsbDevice = {
  HiKey970UsbGetLang,
  HiKey970UsbGetManuFacturer,
  HiKey970UsbGetProduct,
  HiKey970UsbGetSerialNo,
  HiKey970UsbPhyInit
};

EFI_STATUS
EFIAPI
HiKey970UsbEntryPoint (
  IN EFI_HANDLE                            ImageHandle,
  IN EFI_SYSTEM_TABLE                      *SystemTable
  )
{
  return gBS->InstallProtocolInterface (
                &ImageHandle,
                &gDwUsbProtocolGuid,
                EFI_NATIVE_INTERFACE,
                &mDwUsbDevice
                );
}
