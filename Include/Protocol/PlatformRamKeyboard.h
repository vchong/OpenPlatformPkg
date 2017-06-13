/** @file

  Copyright (c) 2017, Linaro. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __PLATFORM_RAM_KEYBOARD_H__
#define __PLATFORM_RAM_KEYBOARD_H__

//
// Protocol interface structure
//
typedef struct _PLATFORM_RAM_KBD_PROTOCOL  PLATFORM_RAM_KBD_PROTOCOL;

typedef struct _RAM_KBD_KEY                RAM_KBD_KEY;

#define RAM_KBD_KEY_NEXT_SIGNATURE SIGNATURE_32 ('r', 'k', 'b', 'd')

struct _RAM_KBD_KEY {
  UINTN                    Signature;
  UINTN                    Base;
  EFI_INPUT_KEY            Key;
};

typedef
EFI_STATUS
(EFIAPI *PLATFORM_RAM_KBD_REGISTER) (
  IN OUT RAM_KBD_KEY                       *RamKey
  );

typedef
BOOLEAN
(EFIAPI *PLATFORM_RAM_KBD_QUERY) (
  IN RAM_KBD_KEY                           *RamKey
  );

typedef
EFI_STATUS
(EFIAPI *PLATFORM_RAM_KBD_CLEAR) (
  IN RAM_KBD_KEY                           *RamKey
  );

struct _PLATFORM_RAM_KBD_PROTOCOL {
  PLATFORM_RAM_KBD_REGISTER                Register;
  PLATFORM_RAM_KBD_QUERY                   Query;
  PLATFORM_RAM_KBD_CLEAR                   Clear;
};

extern EFI_GUID gPlatformRamKeyboardProtocolGuid;

#endif /* __PLATFORM_RAM_KEYBOARD_H__ */
