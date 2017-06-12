/** @file
  GpioKeyboard driver

Copyright (c) 2006 - 2016, Intel Corporation. All rights reserved.<BR>
Copyright (c) 2017, Linaro Ltd. All rights reserved.<BR>

This program and the accompanying materials
are licensed and made available under the terms and conditions
of the BSD License which accompanies this distribution.  The
full text of the license may be found at
http://opensource.org/licenses/bsd-license.php

THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include "GpioKeyboard.h"

//
// GPIO Keyboard Driver Binding Protocol Instance
//
EFI_DRIVER_BINDING_PROTOCOL gGpioKeyboardDriverBinding = {
  GpioKeyboardDriverBindingSupported,
  GpioKeyboardDriverBindingStart,
  GpioKeyboardDriverBindingStop,
  0x10,
  NULL,
  NULL
};

//
// EFI Driver Binding Protocol Functions
//

/**
  Check whether the driver supports this device.

  @param  This                   The Udriver binding protocol.
  @param  Controller             The controller handle to check.
  @param  RemainingDevicePath    The remaining device path.

  @retval EFI_SUCCESS            The driver supports this controller.
  @retval other                  This device isn't supported.

**/
EFI_STATUS
EFIAPI
GpioKeyboardDriverBindingSupported (
  IN EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN EFI_HANDLE                   Controller,
  IN EFI_DEVICE_PATH_PROTOCOL     *RemainingDevicePath
  )
{
  EFI_STATUS                                Status;
  EMBEDDED_GPIO                             *Gpio;
  PLATFORM_GPIO_KBD_PROTOCOL                *PlatformGpio;

  Status = gBS->LocateProtocol (
                  &gPlatformGpioKeyboardProtocolGuid,
                  NULL,
                  (VOID **) &PlatformGpio
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = gBS->OpenProtocol (
                  Controller,
                  &gEmbeddedGpioProtocolGuid,
                  (VOID **) &Gpio,
                  This->DriverBindingHandle,
                  Controller,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  gBS->CloseProtocol (
         Controller,
         &gEmbeddedGpioProtocolGuid,
         This->DriverBindingHandle,
         Controller
         );

  return Status;
}

/**
  Starts the device with this driver.

  @param  This                   The driver binding instance.
  @param  Controller             Handle of device to bind driver to.
  @param  RemainingDevicePath    Optional parameter use to pick a specific child
                                 device to start.

  @retval EFI_SUCCESS            The controller is controlled by the driver.
  @retval Other                  This controller cannot be started.

**/
EFI_STATUS
EFIAPI
GpioKeyboardDriverBindingStart (
  IN EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN EFI_HANDLE                   Controller,
  IN EFI_DEVICE_PATH_PROTOCOL     *RemainingDevicePath
  )
{
  EFI_STATUS                                Status;
  GPIO_KEYBOARD_DEV                         *GpioKeyboardPrivate;
  EMBEDDED_GPIO                             *Gpio;
  PLATFORM_GPIO_KBD_PROTOCOL                *PlatformGpio;
  EFI_STATUS_CODE_VALUE                     StatusCode;

  GpioKeyboardPrivate = NULL;
  //StatusCode          = 0;

  //
  // Open the GPIO Interface
  //
  Status = gBS->LocateProtocol (
                  &gPlatformGpioKeyboardProtocolGuid,
                  NULL,
                  (VOID **) &PlatformGpio
                  );

  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = gBS->OpenProtocol (
                  Controller,
                  &gEmbeddedGpioProtocolGuid,
                  (VOID **) &Gpio,
                  This->DriverBindingHandle,
                  Controller,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  //
  // Allocate the private device structure
  //
  GpioKeyboardPrivate = (GPIO_KEYBOARD_DEV *) AllocateZeroPool (sizeof (GPIO_KEYBOARD_DEV));
  if (NULL == GpioKeyboardPrivate) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Done;
  }

  //
  // Initialize the private device structure
  //
  GpioKeyboardPrivate->Signature                  = GPIO_KEYBOARD_DEV_SIGNATURE;
  GpioKeyboardPrivate->Handle                     = Controller;
  GpioKeyboardPrivate->Gpio                       = Gpio;
  GpioKeyboardPrivate->PlatformGpio               = PlatformGpio;
  GpioKeyboardPrivate->Queue.Front                = 0;
  GpioKeyboardPrivate->Queue.Rear                 = 0;
  GpioKeyboardPrivate->QueueForNotify.Front       = 0;
  GpioKeyboardPrivate->QueueForNotify.Rear        = 0;

  GpioKeyboardPrivate->SimpleTextIn.Reset         = GpioKeyboardReset;
  GpioKeyboardPrivate->SimpleTextIn.ReadKeyStroke = GpioKeyboardReadKeyStroke;

  GpioKeyboardPrivate->SimpleTextInputEx.Reset               = GpioKeyboardResetEx;
  GpioKeyboardPrivate->SimpleTextInputEx.ReadKeyStrokeEx     = GpioKeyboardReadKeyStrokeEx;
  GpioKeyboardPrivate->SimpleTextInputEx.SetState            = GpioKeyboardSetState;

  GpioKeyboardPrivate->SimpleTextInputEx.RegisterKeyNotify   = GpioKeyboardRegisterKeyNotify;
  GpioKeyboardPrivate->SimpleTextInputEx.UnregisterKeyNotify = GpioKeyboardUnregisterKeyNotify;
  InitializeListHead (&GpioKeyboardPrivate->NotifyList);

  //
  // Report that the keyboard is being enabled
  //
  REPORT_STATUS_CODE (
    EFI_PROGRESS_CODE,
    EFI_PERIPHERAL_KEYBOARD | EFI_P_PC_ENABLE
    );

  //
  // Setup the WaitForKey event
  //
  Status = gBS->CreateEvent (
                  EVT_NOTIFY_WAIT,
                  TPL_NOTIFY,
                  GpioKeyboardWaitForKey,
                  &(GpioKeyboardPrivate->SimpleTextIn),
                  &((GpioKeyboardPrivate->SimpleTextIn).WaitForKey)
                  );
  if (EFI_ERROR (Status)) {
    (GpioKeyboardPrivate->SimpleTextIn).WaitForKey = NULL;
    goto Done;
  }
  Status = gBS->CreateEvent (
                  EVT_NOTIFY_WAIT,
                  TPL_NOTIFY,
                  GpioKeyboardWaitForKeyEx,
                  &(GpioKeyboardPrivate->SimpleTextInputEx),
                  &(GpioKeyboardPrivate->SimpleTextInputEx.WaitForKeyEx)
                  );
  if (EFI_ERROR (Status)) {
    GpioKeyboardPrivate->SimpleTextInputEx.WaitForKeyEx = NULL;
    goto Done;
  }

  //
  // Setup a periodic timer, used for reading keystrokes at a fixed interval
  //
  Status = gBS->CreateEvent (
                  EVT_TIMER | EVT_NOTIFY_SIGNAL,
                  TPL_NOTIFY,
                  GpioKeyboardTimerHandler,
                  GpioKeyboardPrivate,
                  &GpioKeyboardPrivate->TimerEvent
                  );
  if (EFI_ERROR (Status)) {
    Status      = EFI_OUT_OF_RESOURCES;
    StatusCode  = EFI_PERIPHERAL_KEYBOARD | EFI_P_EC_CONTROLLER_ERROR;
    goto Done;
  }

  Status = gBS->SetTimer (
                  GpioKeyboardPrivate->TimerEvent,
                  TimerPeriodic,
                  KEYBOARD_TIMER_INTERVAL
                  );
  if (EFI_ERROR (Status)) {
    Status      = EFI_OUT_OF_RESOURCES;
    StatusCode  = EFI_PERIPHERAL_KEYBOARD | EFI_P_EC_CONTROLLER_ERROR;
    goto Done;
  }

  Status = gBS->CreateEvent (
                  EVT_NOTIFY_SIGNAL,
                  TPL_CALLBACK,
                  KeyNotifyProcessHandler,
                  GpioKeyboardPrivate,
                  &GpioKeyboardPrivate->KeyNotifyProcessEvent
                  );
  if (EFI_ERROR (Status)) {
    Status      = EFI_OUT_OF_RESOURCES;
    StatusCode  = EFI_PERIPHERAL_KEYBOARD | EFI_P_EC_CONTROLLER_ERROR;
    goto Done;
  }

  //
  // Report a Progress Code for an attempt to detect the precense of the keyboard device in the system
  //
  REPORT_STATUS_CODE (
    EFI_PROGRESS_CODE,
    EFI_PERIPHERAL_KEYBOARD | EFI_P_PC_PRESENCE_DETECT
    );

  //
  // Reset the keyboard device
  //
  Status = GpioKeyboardPrivate->SimpleTextInputEx.Reset (
                                                    &GpioKeyboardPrivate->SimpleTextInputEx,
                                                    FALSE
                                                    );
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "[KBD]Reset Failed. Status - %r\n", Status));
    StatusCode = EFI_PERIPHERAL_KEYBOARD | EFI_P_EC_NOT_DETECTED;
    goto Done;
  }
#if 0
  //
  // Do platform specific policy like port swapping and keyboard light default
  //
  if (Ps2Policy != NULL) {

    Ps2Policy->Ps2InitHardware (Controller);

    Command = 0;
    if ((Ps2Policy->KeyboardLight & EFI_KEYBOARD_CAPSLOCK) == EFI_KEYBOARD_CAPSLOCK) {
      Command |= 4;
    }

    if ((Ps2Policy->KeyboardLight & EFI_KEYBOARD_NUMLOCK) == EFI_KEYBOARD_NUMLOCK) {
      Command |= 2;
    }

    if ((Ps2Policy->KeyboardLight & EFI_KEYBOARD_SCROLLLOCK) == EFI_KEYBOARD_SCROLLLOCK) {
      Command |= 1;
    }

    KeyboardWrite (GpioKeyboardPrivate, 0xed);
    KeyboardWaitForValue (GpioKeyboardPrivate, 0xfa, KEYBOARD_WAITFORVALUE_TIMEOUT);
    KeyboardWrite (GpioKeyboardPrivate, Command);
    //
    // Call Legacy GPIO Protocol to set whatever is necessary
    //
    LegacyGpio->UpdateKeyboardLedStatus (LegacyGpio, Command);
  }
  //
  // Get Configuration
  //
  Regs.H.AH = 0xc0;
  CarryFlag = GpioKeyboardPrivate->LegacyGpio->Int86 (
                                                 GpioKeyboardPrivate->LegacyGpio,
                                                 0x15,
                                                 &Regs
                                                 );

  if (!CarryFlag) {
    //
    // Check bit 6 of Feature Byte 2.
    // If it is set, then Int 16 Func 09 is supported
    //
    if (*(UINT8 *)(UINTN) ((Regs.X.ES << 4) + Regs.X.BX + 0x06) & 0x40) {
      //
      // Get Keyboard Functionality
      //
      Regs.H.AH = 0x09;
      CarryFlag = GpioKeyboardPrivate->LegacyGpio->Int86 (
                                                     GpioKeyboardPrivate->LegacyGpio,
                                                     0x16,
                                                     &Regs
                                                     );

      if (!CarryFlag) {
        //
        // Check bit 5 of AH.
        // If it is set, then INT 16 Finc 10-12 are supported.
        //
        if ((Regs.H.AL & 0x40) != 0) {
          //
          // Set the flag to use INT 16 Func 10-12
          //
          GpioKeyboardPrivate->ExtendedKeyboard = TRUE;
        }
      }
    }
  }
  DEBUG ((EFI_D_INFO, "[KBD]Extended keystrokes supported by CSM16 - %02x\n", (UINTN)GpioKeyboardPrivate->ExtendedKeyboard));
#endif
  //
  // Install protocol interfaces for the keyboard device.
  //
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Controller,
                  &gEfiSimpleTextInProtocolGuid,
                  &GpioKeyboardPrivate->SimpleTextIn,
                  &gEfiSimpleTextInputExProtocolGuid,
                  &GpioKeyboardPrivate->SimpleTextInputEx,
                  NULL
                  );

Done:
  if (StatusCode != 0) {
    //
    // Report an Error Code for failing to start the keyboard device
    //
    REPORT_STATUS_CODE (
      EFI_ERROR_CODE | EFI_ERROR_MINOR,
      StatusCode
      );
  }

  if (EFI_ERROR (Status)) {

    if (GpioKeyboardPrivate != NULL) {
      if ((GpioKeyboardPrivate->SimpleTextIn).WaitForKey != NULL) {
        gBS->CloseEvent ((GpioKeyboardPrivate->SimpleTextIn).WaitForKey);
      }

      if ((GpioKeyboardPrivate->SimpleTextInputEx).WaitForKeyEx != NULL) {
        gBS->CloseEvent ((GpioKeyboardPrivate->SimpleTextInputEx).WaitForKeyEx);
      }

      if (GpioKeyboardPrivate->KeyNotifyProcessEvent != NULL) {
        gBS->CloseEvent (GpioKeyboardPrivate->KeyNotifyProcessEvent);
      }

      GpioKeyboardFreeNotifyList (&GpioKeyboardPrivate->NotifyList);

      if (GpioKeyboardPrivate->TimerEvent != NULL) {
        gBS->CloseEvent (GpioKeyboardPrivate->TimerEvent);
      }
      FreePool (GpioKeyboardPrivate);
    }

Exit:
    gBS->CloseProtocol (
           Controller,
           &gEmbeddedGpioProtocolGuid,
           This->DriverBindingHandle,
           Controller
           );
  }

  return Status;
}

/**
  Stop the device handled by this driver.

  @param  This                   The driver binding protocol.
  @param  Controller             The controller to release.
  @param  NumberOfChildren       The number of handles in ChildHandleBuffer.
  @param  ChildHandleBuffer      The array of child handle.

  @retval EFI_SUCCESS            The device was stopped.
  @retval EFI_DEVICE_ERROR       The device could not be stopped due to a device error.
  @retval Others                 Fail to uninstall protocols attached on the device.

**/
EFI_STATUS
EFIAPI
GpioKeyboardDriverBindingStop (
  IN  EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN  EFI_HANDLE                   Controller,
  IN  UINTN                        NumberOfChildren,
  IN  EFI_HANDLE                   *ChildHandleBuffer
  )
{
  return EFI_SUCCESS;
}


/**
  Enqueue the key.

  @param  Queue                 The queue to be enqueued.
  @param  KeyData               The key data to be enqueued.

  @retval EFI_NOT_READY         The queue is full.
  @retval EFI_SUCCESS           Successfully enqueued the key data.

**/
EFI_STATUS
Enqueue (
  IN SIMPLE_QUEUE         *Queue,
  IN EFI_KEY_DATA         *KeyData
  )
{
  if ((Queue->Rear + 1) % QUEUE_MAX_COUNT == Queue->Front) {
    return EFI_NOT_READY;
  }

  CopyMem (&Queue->Buffer[Queue->Rear], KeyData, sizeof (EFI_KEY_DATA));
  Queue->Rear = (Queue->Rear + 1) % QUEUE_MAX_COUNT;

  return EFI_SUCCESS;
}

/**
  Dequeue the key.

  @param  Queue                 The queue to be dequeued.
  @param  KeyData               The key data to be dequeued.

  @retval EFI_NOT_READY         The queue is empty.
  @retval EFI_SUCCESS           Successfully dequeued the key data.

**/
EFI_STATUS
Dequeue (
  IN SIMPLE_QUEUE         *Queue,
  IN EFI_KEY_DATA         *KeyData
  )
{
  if (Queue->Front == Queue->Rear) {
    return EFI_NOT_READY;
  }

  CopyMem (KeyData, &Queue->Buffer[Queue->Front], sizeof (EFI_KEY_DATA));
  Queue->Front  = (Queue->Front + 1) % QUEUE_MAX_COUNT;

  return EFI_SUCCESS;
}

/**
  Check whether the queue is empty.

  @param  Queue                 The queue to be checked.

  @retval EFI_NOT_READY         The queue is empty.
  @retval EFI_SUCCESS           The queue is not empty.

**/
EFI_STATUS
CheckQueue (
  IN SIMPLE_QUEUE         *Queue
  )
{
  if (Queue->Front == Queue->Rear) {
    return EFI_NOT_READY;
  }

  return EFI_SUCCESS;
}

/**
  Check key buffer to get the key stroke status.

  @param  This         Pointer of the protocol EFI_SIMPLE_TEXT_IN_PROTOCOL.

  @retval EFI_SUCCESS  A key is being pressed now.
  @retval Other        No key is now pressed.

**/
EFI_STATUS
EFIAPI
GpioKeyboardCheckForKey (
  IN  EFI_SIMPLE_TEXT_INPUT_PROTOCOL  *This
  )
{
  GPIO_KEYBOARD_DEV     *GpioKeyboardPrivate;

  GpioKeyboardPrivate = GPIO_KEYBOARD_DEV_FROM_THIS (This);

  return CheckQueue (&GpioKeyboardPrivate->Queue);
}

/**
  Free keyboard notify list.

  @param  ListHead   The list head

  @retval EFI_SUCCESS           Free the notify list successfully
  @retval EFI_INVALID_PARAMETER ListHead is invalid.

**/
EFI_STATUS
GpioKeyboardFreeNotifyList (
  IN OUT LIST_ENTRY           *ListHead
  )
{
  GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY *NotifyNode;

  if (ListHead == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  while (!IsListEmpty (ListHead)) {
    NotifyNode = CR (
                   ListHead->ForwardLink,
                   GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY,
                   NotifyEntry,
                   GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY_SIGNATURE
                   );
    RemoveEntryList (ListHead->ForwardLink);
    gBS->FreePool (NotifyNode);
  }

  return EFI_SUCCESS;
}

/**
  Judge whether is a registed key

  @param RegsiteredData       A pointer to a buffer that is filled in with the keystroke
                              state data for the key that was registered.
  @param InputData            A pointer to a buffer that is filled in with the keystroke
                              state data for the key that was pressed.

  @retval TRUE                Key be pressed matches a registered key.
  @retval FLASE               Match failed.

**/
BOOLEAN
IsKeyRegistered (
  IN EFI_KEY_DATA  *RegsiteredData,
  IN EFI_KEY_DATA  *InputData
  )

{
  ASSERT (RegsiteredData != NULL && InputData != NULL);

  if ((RegsiteredData->Key.ScanCode    != InputData->Key.ScanCode) ||
      (RegsiteredData->Key.UnicodeChar != InputData->Key.UnicodeChar)) {
    return FALSE;
  }

  //
  // Assume KeyShiftState/KeyToggleState = 0 in Registered key data means these state could be ignored.
  //
  if (RegsiteredData->KeyState.KeyShiftState != 0 &&
      RegsiteredData->KeyState.KeyShiftState != InputData->KeyState.KeyShiftState) {
    return FALSE;
  }
  if (RegsiteredData->KeyState.KeyToggleState != 0 &&
      RegsiteredData->KeyState.KeyToggleState != InputData->KeyState.KeyToggleState) {
    return FALSE;
  }

  return TRUE;

}

/**
  Event notification function for SIMPLE_TEXT_IN.WaitForKey event
  Signal the event if there is key available

  @param Event    the event object
  @param Context  waitting context

**/
VOID
EFIAPI
GpioKeyboardWaitForKey (
  IN  EFI_EVENT               Event,
  IN  VOID                    *Context
  )
{
  //
  // Stall 1ms to give a chance to let other driver interrupt this routine for their timer event.
  // Csm will be used to check whether there is a key pending, but the csm will disable all
  // interrupt before switch to compatibility16, which mean all the efiCompatibility timer
  // event will stop work during the compatibility16. And If a caller recursivly invoke this function,
  // e.g. UI setup or Shell, other drivers which are driven by timer event will have a bad performance during this period,
  // e.g. usb keyboard driver.
  // Add a stall period can greatly increate other driver performance during the WaitForKey is recursivly invoked.
  // 1ms delay will make little impact to the thunk keyboard driver, and user can not feel the delay at all when input.
  //
  gBS->Stall (1000);
  //
  // Use TimerEvent callback function to check whether there's any key pressed
  //
  GpioKeyboardTimerHandler (NULL, GPIO_KEYBOARD_DEV_FROM_THIS (Context));

  if (!EFI_ERROR (GpioKeyboardCheckForKey (Context))) {
    gBS->SignalEvent (Event);
  }
}

/**
  Event notification function for SIMPLE_TEXT_INPUT_EX_PROTOCOL.WaitForKeyEx event
  Signal the event if there is key available

  @param Event    event object
  @param Context  waiting context

**/
VOID
EFIAPI
GpioKeyboardWaitForKeyEx (
  IN  EFI_EVENT               Event,
  IN  VOID                    *Context
  )

{
  GPIO_KEYBOARD_DEV                     *GpioKeyboardPrivate;

  GpioKeyboardPrivate = TEXT_INPUT_EX_GPIO_KEYBOARD_DEV_FROM_THIS (Context);
  GpioKeyboardWaitForKey (Event, &GpioKeyboardPrivate->SimpleTextIn);

}

//
// EFI Simple Text In Protocol Functions
//
/**
  Reset the Keyboard and do BAT test for it, if (ExtendedVerification == TRUE) then do some extra keyboard validations.

  @param  This                  Pointer of simple text Protocol.
  @param  ExtendedVerification  Whether perform the extra validation of keyboard. True: perform; FALSE: skip.

  @retval EFI_SUCCESS           The command byte is written successfully.
  @retval EFI_DEVICE_ERROR      Errors occurred during resetting keyboard.

**/
EFI_STATUS
EFIAPI
GpioKeyboardReset (
  IN  EFI_SIMPLE_TEXT_INPUT_PROTOCOL  *This,
  IN  BOOLEAN                         ExtendedVerification
  )
{
  GPIO_KEYBOARD_DEV *GpioKeyboardPrivate;
  EFI_STATUS        Status;
  EFI_TPL           OldTpl;
  GPIO_KBD_KEY      GpioKeyFirst, *GpioKey = NULL;
  LIST_ENTRY        *Next = NULL;

  GpioKeyboardPrivate = GPIO_KEYBOARD_DEV_FROM_THIS (This);

  //
  // Raise TPL to avoid mouse operation impact
  //
  OldTpl = gBS->RaiseTPL (TPL_NOTIFY);

  if (GpioKeyboardPrivate->PlatformGpio && GpioKeyboardPrivate->PlatformGpio->Register) {
    Status = GpioKeyboardPrivate->PlatformGpio->Register (&GpioKeyFirst);
    if (Status == EFI_SUCCESS) {
      GpioKey = &GpioKeyFirst;
      do {
        Status = GpioKeyboardPrivate->Gpio->Set (GpioKeyboardPrivate->Gpio, GpioKey->Pin, GPIO_MODE_INPUT);
        ASSERT_EFI_ERROR (Status);
        Next = &GpioKey->Next;
        GpioKey = CR (Next, GPIO_KBD_KEY, Next, GPIO_KBD_KEY_NEXT_SIGNATURE);
      } while (&GpioKeyFirst != GpioKey);
    }
  } else {
    Status = EFI_INVALID_PARAMETER;
  }

  //
  // resume priority of task level
  //
  gBS->RestoreTPL (OldTpl);

  return Status;

}

/**
  Reset the input device and optionaly run diagnostics

  @param  This                  Protocol instance pointer.
  @param  ExtendedVerification  Driver may perform diagnostics on reset.

  @retval EFI_SUCCESS           The device was reset.
  @retval EFI_DEVICE_ERROR      The device is not functioning properly and could-
                                not be reset.

**/
EFI_STATUS
EFIAPI
GpioKeyboardResetEx (
  IN EFI_SIMPLE_TEXT_INPUT_EX_PROTOCOL  *This,
  IN BOOLEAN                            ExtendedVerification
  )
{
  GPIO_KEYBOARD_DEV                     *GpioKeyboardPrivate;
  EFI_STATUS                            Status;
  EFI_TPL                               OldTpl;

  GpioKeyboardPrivate = TEXT_INPUT_EX_GPIO_KEYBOARD_DEV_FROM_THIS (This);

  Status = GpioKeyboardPrivate->SimpleTextIn.Reset (
                                               &GpioKeyboardPrivate->SimpleTextIn,
                                               ExtendedVerification
                                               );
  if (EFI_ERROR (Status)) {
    return EFI_DEVICE_ERROR;
  }

  OldTpl = gBS->RaiseTPL (TPL_NOTIFY);

  gBS->RestoreTPL (OldTpl);

  return EFI_SUCCESS;

}

/**
  Reads the next keystroke from the input device. The WaitForKey Event can
  be used to test for existance of a keystroke via WaitForEvent () call.

  @param  GpioKeyboardPrivate   Gpiokeyboard driver private structure.
  @param  KeyData               A pointer to a buffer that is filled in with the keystroke
                                state data for the key that was pressed.

  @retval EFI_SUCCESS           The keystroke information was returned.
  @retval EFI_NOT_READY         There was no keystroke data availiable.
  @retval EFI_DEVICE_ERROR      The keystroke information was not returned due to
                                hardware errors.
  @retval EFI_INVALID_PARAMETER KeyData is NULL.

**/
EFI_STATUS
KeyboardReadKeyStrokeWorker (
  IN GPIO_KEYBOARD_DEV  *GpioKeyboardPrivate,
  OUT EFI_KEY_DATA      *KeyData
  )
{
  EFI_STATUS                            Status;
  EFI_TPL                               OldTpl;
  if (KeyData == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  //
  // Use TimerEvent callback function to check whether there's any key pressed
  //

  //
  // Stall 1ms to give a chance to let other driver interrupt this routine for their timer event.
  // Csm will be used to check whether there is a key pending, but the csm will disable all
  // interrupt before switch to compatibility16, which mean all the efiCompatibility timer
  // event will stop work during the compatibility16. And If a caller recursivly invoke this function,
  // e.g. OS loader, other drivers which are driven by timer event will have a bad performance during this period,
  // e.g. usb keyboard driver.
  // Add a stall period can greatly increate other driver performance during the WaitForKey is recursivly invoked.
  // 1ms delay will make little impact to the thunk keyboard driver, and user can not feel the delay at all when input.
  //
  gBS->Stall (1000);

  OldTpl = gBS->RaiseTPL (TPL_NOTIFY);

  GpioKeyboardTimerHandler (NULL, GpioKeyboardPrivate);
  //
  // If there's no key, just return
  //
  Status = CheckQueue (&GpioKeyboardPrivate->Queue);
  if (EFI_ERROR (Status)) {
    gBS->RestoreTPL (OldTpl);
    return EFI_NOT_READY;
  }

  Status = Dequeue (&GpioKeyboardPrivate->Queue, KeyData);

  gBS->RestoreTPL (OldTpl);

  return EFI_SUCCESS;
}

/**
  Read out the scan code of the key that has just been stroked.

  @param  This        Pointer of simple text Protocol.
  @param  Key         Pointer for store the key that read out.

  @retval EFI_SUCCESS The key is read out successfully.
  @retval other       The key reading failed.

**/
EFI_STATUS
EFIAPI
GpioKeyboardReadKeyStroke (
  IN  EFI_SIMPLE_TEXT_INPUT_PROTOCOL  *This,
  OUT EFI_INPUT_KEY                   *Key
  )
{
  GPIO_KEYBOARD_DEV     *GpioKeyboardPrivate;
  EFI_STATUS            Status;
  EFI_KEY_DATA          KeyData;

  GpioKeyboardPrivate = GPIO_KEYBOARD_DEV_FROM_THIS (This);

  Status = KeyboardReadKeyStrokeWorker (GpioKeyboardPrivate, &KeyData);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  //
  // Convert the Ctrl+[a-z] to Ctrl+[1-26]
  //
  if ((KeyData.KeyState.KeyShiftState & (EFI_LEFT_CONTROL_PRESSED | EFI_RIGHT_CONTROL_PRESSED)) != 0) {
    if (KeyData.Key.UnicodeChar >= L'a' && KeyData.Key.UnicodeChar <= L'z') {
      KeyData.Key.UnicodeChar = (CHAR16) (KeyData.Key.UnicodeChar - L'a' + 1);
    } else if (KeyData.Key.UnicodeChar >= L'A' && KeyData.Key.UnicodeChar <= L'Z') {
      KeyData.Key.UnicodeChar = (CHAR16) (KeyData.Key.UnicodeChar - L'A' + 1);
    }
  }

  CopyMem (Key, &KeyData.Key, sizeof (EFI_INPUT_KEY));

  return EFI_SUCCESS;
}

/**
  Reads the next keystroke from the input device. The WaitForKey Event can
  be used to test for existance of a keystroke via WaitForEvent () call.

  @param  This         Protocol instance pointer.
  @param  KeyData      A pointer to a buffer that is filled in with the keystroke
                       state data for the key that was pressed.

  @retval  EFI_SUCCESS           The keystroke information was returned.
  @retval  EFI_NOT_READY         There was no keystroke data availiable.
  @retval  EFI_DEVICE_ERROR      The keystroke information was not returned due to
                                 hardware errors.
  @retval  EFI_INVALID_PARAMETER KeyData is NULL.

**/
EFI_STATUS
EFIAPI
GpioKeyboardReadKeyStrokeEx (
  IN  EFI_SIMPLE_TEXT_INPUT_EX_PROTOCOL *This,
  OUT EFI_KEY_DATA                      *KeyData
  )
{
  GPIO_KEYBOARD_DEV                     *GpioKeyboardPrivate;

  if (KeyData == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  GpioKeyboardPrivate = TEXT_INPUT_EX_GPIO_KEYBOARD_DEV_FROM_THIS (This);

  return KeyboardReadKeyStrokeWorker (GpioKeyboardPrivate, KeyData);

}

/**
  Set certain state for the input device.

  @param  This              Protocol instance pointer.
  @param  KeyToggleState    A pointer to the EFI_KEY_TOGGLE_STATE to set the-
                            state for the input device.

  @retval EFI_SUCCESS           The device state was set successfully.
  @retval EFI_DEVICE_ERROR      The device is not functioning correctly and could-
                                not have the setting adjusted.
  @retval EFI_UNSUPPORTED       The device does not have the ability to set its state.
  @retval EFI_INVALID_PARAMETER KeyToggleState is NULL.

**/
EFI_STATUS
EFIAPI
GpioKeyboardSetState (
  IN EFI_SIMPLE_TEXT_INPUT_EX_PROTOCOL  *This,
  IN EFI_KEY_TOGGLE_STATE               *KeyToggleState
  )
{
  if (KeyToggleState == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  //
  // Thunk keyboard driver doesn't support partial keystroke.
  //
  if ((*KeyToggleState & EFI_TOGGLE_STATE_VALID) != EFI_TOGGLE_STATE_VALID ||
      (*KeyToggleState & EFI_KEY_STATE_EXPOSED) == EFI_KEY_STATE_EXPOSED
      ) {
    return EFI_UNSUPPORTED;
  }

  return EFI_SUCCESS;
}

/**
  Register a notification function for a particular keystroke for the input device.

  @param  This                    Protocol instance pointer.
  @param  KeyData                 A pointer to a buffer that is filled in with the keystroke
                                  information data for the key that was pressed.
  @param  KeyNotificationFunction Points to the function to be called when the key
                                  sequence is typed specified by KeyData.
  @param  NotifyHandle            Points to the unique handle assigned to the registered notification.


  @retval EFI_SUCCESS             The notification function was registered successfully.
  @retval EFI_OUT_OF_RESOURCES    Unable to allocate resources for necesssary data structures.
  @retval EFI_INVALID_PARAMETER   KeyData or NotifyHandle is NULL.

**/
EFI_STATUS
EFIAPI
GpioKeyboardRegisterKeyNotify (
  IN EFI_SIMPLE_TEXT_INPUT_EX_PROTOCOL  *This,
  IN EFI_KEY_DATA                       *KeyData,
  IN EFI_KEY_NOTIFY_FUNCTION            KeyNotificationFunction,
  OUT VOID                              **NotifyHandle
  )
{
  EFI_STATUS                            Status;
  GPIO_KEYBOARD_DEV                     *GpioKeyboardPrivate;
  EFI_TPL                               OldTpl;
  GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY    *NewNotify;
  LIST_ENTRY                            *Link;
  GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY    *CurrentNotify;

  if (KeyData == NULL || NotifyHandle == NULL || KeyNotificationFunction == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  GpioKeyboardPrivate = TEXT_INPUT_EX_GPIO_KEYBOARD_DEV_FROM_THIS (This);

  //
  // Enter critical section
  //
  OldTpl = gBS->RaiseTPL (TPL_NOTIFY);

  //
  // Return EFI_SUCCESS if the (KeyData, NotificationFunction) is already registered.
  //
  for (Link = GpioKeyboardPrivate->NotifyList.ForwardLink; Link != &GpioKeyboardPrivate->NotifyList; Link = Link->ForwardLink) {
    CurrentNotify = CR (
                      Link,
                      GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY,
                      NotifyEntry,
                      GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY_SIGNATURE
                      );
    if (IsKeyRegistered (&CurrentNotify->KeyData, KeyData)) {
      if (CurrentNotify->KeyNotificationFn == KeyNotificationFunction) {
        *NotifyHandle = CurrentNotify;
        Status = EFI_SUCCESS;
        goto Exit;
      }
    }
  }

  //
  // Allocate resource to save the notification function
  //

  NewNotify = (GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY *) AllocateZeroPool (sizeof (GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY));
  if (NewNotify == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  NewNotify->Signature         = GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY_SIGNATURE;
  NewNotify->KeyNotificationFn = KeyNotificationFunction;
  CopyMem (&NewNotify->KeyData, KeyData, sizeof (EFI_KEY_DATA));
  InsertTailList (&GpioKeyboardPrivate->NotifyList, &NewNotify->NotifyEntry);

  *NotifyHandle                = NewNotify;
  Status                       = EFI_SUCCESS;

Exit:
  //
  // Leave critical section and return
  //
  gBS->RestoreTPL (OldTpl);
  return Status;

}

/**
  Remove a registered notification function from a particular keystroke.

  @param  This                 Protocol instance pointer.
  @param  NotificationHandle   The handle of the notification function being unregistered.

  @retval EFI_SUCCESS             The notification function was unregistered successfully.
  @retval EFI_INVALID_PARAMETER   The NotificationHandle is invalid.

**/
EFI_STATUS
EFIAPI
GpioKeyboardUnregisterKeyNotify (
  IN EFI_SIMPLE_TEXT_INPUT_EX_PROTOCOL  *This,
  IN VOID                               *NotificationHandle
  )
{
  EFI_STATUS                            Status;
  GPIO_KEYBOARD_DEV                     *GpioKeyboardPrivate;
  EFI_TPL                               OldTpl;
  LIST_ENTRY                            *Link;
  GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY    *CurrentNotify;

  //
  // Check incoming notification handle
  //
  if (NotificationHandle == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  if (((GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY *) NotificationHandle)->Signature != GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY_SIGNATURE) {
    return EFI_INVALID_PARAMETER;
  }

  GpioKeyboardPrivate = TEXT_INPUT_EX_GPIO_KEYBOARD_DEV_FROM_THIS (This);

  //
  // Enter critical section
  //
  OldTpl = gBS->RaiseTPL (TPL_NOTIFY);

  for (Link = GpioKeyboardPrivate->NotifyList.ForwardLink; Link != &GpioKeyboardPrivate->NotifyList; Link = Link->ForwardLink) {
    CurrentNotify = CR (
                      Link,
                      GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY,
                      NotifyEntry,
                      GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY_SIGNATURE
                      );
    if (CurrentNotify == NotificationHandle) {
      //
      // Remove the notification function from NotifyList and free resources
      //
      RemoveEntryList (&CurrentNotify->NotifyEntry);

      Status = EFI_SUCCESS;
      goto Exit;
    }
  }

  //
  // Can not find the specified Notification Handle
  //
  Status = EFI_INVALID_PARAMETER;

Exit:
  //
  // Leave critical section and return
  //
  gBS->RestoreTPL (OldTpl);
  return Status;
}

/**
  Timer event handler: read a series of scancodes from 8042
  and put them into memory scancode buffer.
  it read as much scancodes to either fill
  the memory buffer or empty the keyboard buffer.
  It is registered as running under TPL_NOTIFY

  @param Event       The timer event
  @param Context     A KEYBOARD_CONSOLE_IN_DEV pointer

**/
VOID
EFIAPI
GpioKeyboardTimerHandler (
  IN EFI_EVENT    Event,
  IN VOID         *Context
  )
{
  EFI_STATUS                         Status;
  EFI_TPL                            OldTpl;
  LIST_ENTRY                         *Link;
  EFI_KEY_DATA                       KeyData;
  GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY *CurrentNotify;
  GPIO_KEYBOARD_DEV                  *GpioKeyboardPrivate;
  UINTN                              Value;
  GPIO_KBD_KEY                       GpioKeyFirst, *GpioKey = NULL;
  LIST_ENTRY                         *Next;

  GpioKeyboardPrivate = Context;

  //
  // Enter critical section
  //
  OldTpl = gBS->RaiseTPL (TPL_NOTIFY);

  if (GpioKeyboardPrivate->PlatformGpio && GpioKeyboardPrivate->PlatformGpio->Register) {
    Status = GpioKeyboardPrivate->PlatformGpio->Register (&GpioKeyFirst);
    if (Status == EFI_SUCCESS) {
      GpioKey = &GpioKeyFirst;
      do {
        Status = GpioKeyboardPrivate->Gpio->Get (GpioKeyboardPrivate->Gpio, GpioKey->Pin, &Value);
        if (EFI_ERROR (Status)) {
          goto Exit;
        }
        if (Value == GpioKey->Value) {
          KeyData.Key.ScanCode = GpioKey->Key.ScanCode;
          KeyData.Key.UnicodeChar = GpioKey->Key.UnicodeChar;
          KeyData.KeyState.KeyShiftState  = EFI_SHIFT_STATE_VALID;
          KeyData.KeyState.KeyToggleState = EFI_TOGGLE_STATE_VALID;
          break;
        }
        Next = &GpioKey->Next;
        GpioKey = CR (Next, GPIO_KBD_KEY, Next, GPIO_KBD_KEY_NEXT_SIGNATURE);
      } while (&GpioKeyFirst != GpioKey);
    }
  } else {
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }

  //
  // Signal KeyNotify process event if this key pressed matches any key registered.
  //
  for (Link = GpioKeyboardPrivate->NotifyList.ForwardLink; Link != &GpioKeyboardPrivate->NotifyList; Link = Link->ForwardLink) {
    CurrentNotify = CR (
                      Link,
                      GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY,
                      NotifyEntry,
                      GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY_SIGNATURE
                      );
    if (IsKeyRegistered (&CurrentNotify->KeyData, &KeyData)) {
      //
      // The key notification function needs to run at TPL_CALLBACK
      // while current TPL is TPL_NOTIFY. It will be invoked in
      // KeyNotifyProcessHandler() which runs at TPL_CALLBACK.
      //
      Enqueue (&GpioKeyboardPrivate->QueueForNotify, &KeyData);
      gBS->SignalEvent (GpioKeyboardPrivate->KeyNotifyProcessEvent);
    }
  }

  Enqueue (&GpioKeyboardPrivate->Queue, &KeyData);

Exit:
  //
  // Leave critical section and return
  //
  gBS->RestoreTPL (OldTpl);
}

/**
  Process key notify.

  @param  Event                 Indicates the event that invoke this function.
  @param  Context               Indicates the calling context.
**/
VOID
EFIAPI
KeyNotifyProcessHandler (
  IN  EFI_EVENT                 Event,
  IN  VOID                      *Context
  )
{
  EFI_STATUS                            Status;
  GPIO_KEYBOARD_DEV                     *GpioKeyboardPrivate;
  EFI_KEY_DATA                          KeyData;
  LIST_ENTRY                            *Link;
  LIST_ENTRY                            *NotifyList;
  GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY    *CurrentNotify;
  EFI_TPL                               OldTpl;

  GpioKeyboardPrivate = (GPIO_KEYBOARD_DEV *) Context;

  //
  // Invoke notification functions.
  //
  NotifyList = &GpioKeyboardPrivate->NotifyList;
  while (TRUE) {
    //
    // Enter critical section
    //
    OldTpl = gBS->RaiseTPL (TPL_NOTIFY);
    Status = Dequeue (&GpioKeyboardPrivate->QueueForNotify, &KeyData);
    //
    // Leave critical section
    //
    gBS->RestoreTPL (OldTpl);
    if (EFI_ERROR (Status)) {
      break;
    }
    for (Link = GetFirstNode (NotifyList); !IsNull (NotifyList, Link); Link = GetNextNode (NotifyList, Link)) {
      CurrentNotify = CR (Link, GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY, NotifyEntry, GPIO_KEYBOARD_CONSOLE_IN_EX_NOTIFY_SIGNATURE);
      if (IsKeyRegistered (&CurrentNotify->KeyData, &KeyData)) {
        CurrentNotify->KeyNotificationFn (&KeyData);
      }
    }
  }
}

/**
  The user Entry Point for module GpioKeyboard. The user code starts with this function.

  @param[in] ImageHandle    The firmware allocated handle for the EFI image.
  @param[in] SystemTable    A pointer to the EFI System Table.

  @retval EFI_SUCCESS       The entry point is executed successfully.
  @retval other             Some error occurs when executing this entry point.

**/
EFI_STATUS
EFIAPI
InitializeGpioKeyboard(
  IN EFI_HANDLE           ImageHandle,
  IN EFI_SYSTEM_TABLE     *SystemTable
  )
{
  EFI_STATUS              Status;

  //
  // Install driver model protocol(s).
  //
  Status = EfiLibInstallDriverBindingComponentName2 (
             ImageHandle,
             SystemTable,
             &gGpioKeyboardDriverBinding,
             ImageHandle,
             &gGpioKeyboardComponentName,
             &gGpioKeyboardComponentName2
             );
  ASSERT_EFI_ERROR (Status);

  return Status;
}
