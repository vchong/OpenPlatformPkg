/** @file

  Copyright (c) 2017, Linaro. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __PLATFORM_GPIO_KEYBOARD_H__
#define __PLATFORM_GPIO_KEYBOARD_H__

#include <Protocol/EmbeddedGpio.h>

//
// Protocol interface structure
//
typedef struct _PLATFORM_GPIO_KBD_PROTOCOL  PLATFORM_GPIO_KBD_PROTOCOL;

typedef struct _GPIO_KBD_KEY                GPIO_KBD_KEY;

#define GPIO_KBD_KEY_NEXT_SIGNATURE SIGNATURE_32 ('g', 'k', 'b', 'd')

struct _GPIO_KBD_KEY {
  UINTN                                     Signature;
  EMBEDDED_GPIO_PIN                         Pin;
  UINT32                                    Value;
  EFI_INPUT_KEY                             Key;
  LIST_ENTRY                                Next;
};

typedef
EFI_STATUS
(EFIAPI *PLATFORM_GPIO_KBD_REGISTER) (
  IN OUT GPIO_KBD_KEY                       *GpioKey
  );

struct _PLATFORM_GPIO_KBD_PROTOCOL {
  PLATFORM_GPIO_KBD_REGISTER                Register;
};

extern EFI_GUID gPlatformGpioKeyboardProtocolGuid;

#endif /* __PLATFORM_GPIO_KEYBOARD_H__ */
