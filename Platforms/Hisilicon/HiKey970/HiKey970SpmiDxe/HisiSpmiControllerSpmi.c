/*
 * HisiSpmiControllerSpmi.c
 *
 * Spmi Cotroller Driver.
 *
 * Copyright (c) 2001-2017, Huawei Tech. Co., Ltd. All rights reserved.
 *
 */
#include <Library/BaseLib.h>
#include <Library/TimerLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Protocol/DriverBinding.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <HisiSpmiControllerSpmi.h>


STATIC
INTN
SpmiControllerWaitForDone( UINT8 Channel, UINT8 Sid )
{
	unsigned int status = 0;
	unsigned int timeout = SPMI_STATUS_TIMEOUT_US;
	unsigned long status_addr = SPMI_APB_SPMI_STATUS_BASE_ADDR + SPMI_CHANNEL_OFFSET * Channel
		+ SPMI_SLAVE_OFFSET * Sid;

	while (timeout--) {
		SPMI_READL( status_addr, status );
		if ( status & SPMI_APB_TRANS_DONE ) {
			if ( status & SPMI_APB_TRANS_FAIL ) {
				DEBUG ((DEBUG_ERROR, "SPMI:Channel:0x%X Sid:0x%X transfer fail\n" , Channel, Sid ));
				return -1;
			}
			return 0;
		}
		MicroSecondDelay(1);
	}
	DEBUG (( DEBUG_ERROR, "SPMI: Sid:0x%X transfer timeout\n" , Sid ));
	return -1;

}

STATIC
INTN
SpmiRegisterRead( UINT8 Channel, UINT8 Sid, UINT32 Addr, UINT8 *Buf, UINT32 Len )
{
	UINTN Cmd, Data;
	UINT16 Opc, i;

	if ( 1 == Len ) {
		if ( 0x1f >= Addr ) {
			Opc = SPMI_CMD_REG_READ;
		} else if ( 0xff >= Addr ) {
			Opc = SPMI_CMD_EXT_REG_READ;
		} else if ( 0xfff >= Addr ) {
			Opc = SPMI_CMD_EXT_REG_READ_L;
		} else {
			DEBUG (( DEBUG_ERROR, "SPMI: invalid Addr\n"));
			return -1;
		}
	} else {
		if ( 0xff >= Addr ) {
			Opc = SPMI_CMD_EXT_REG_READ;
		} else if ( 0xfff >= Addr ) {
			Opc = SPMI_CMD_EXT_REG_READ_L;
		} else {
			DEBUG (( DEBUG_ERROR, "SPMI: invalid Addr\n"));
			return -1;
		}
	}
	Cmd = SPMI_APB_SPMI_CMD_EN |										    /* cmd_en */
		 ( Opc << SPMI_APB_SPMI_CMD_TYPE_OFFSET ) |						    /* cmd_type */
		 ( ( Len - 1 ) << SPMI_APB_SPMI_CMD_LENGTH_OFFSET ) |				/* byte_cnt */
		 ( ( Sid & 0xf ) << SPMI_APB_SPMI_CMD_SLAVEID_OFFSET ) |			/* slvid */
		 ( ( Addr & 0xffff )  << SPMI_APB_SPMI_CMD_ADDR_OFFSET );			/* slave_addr */

	SPMI_WRITEL( SPMI_APB_SPMI_CMD_BASE_ADDR + SPMI_CHANNEL_OFFSET * Channel, Cmd );

	if ( 0 != SpmiControllerWaitForDone( Channel, Sid ) ) {
		return -1;
	}

	i = 0;
	do {
		SPMI_READL( SPMI_APB_SPMI_RDATA0_BASE_ADDR  + SPMI_CHANNEL_OFFSET * Channel + SPMI_SLAVE_OFFSET * Sid + i * 4, Data );/*lint !e665 !e717 */
		Data = Tranverse32( Data );
		if ( ( Len - i * 4 ) >> 2 ) {
			CopyMem( Buf, &Data, sizeof( Data ) ); // unsafe_function_ignore: memcpy
			Buf += sizeof( Data );
		} else {
			CopyMem( Buf, &Data, (UINT32)Len % 4 ); // unsafe_function_ignore: memcpy
			Buf += ( Len % 4 );
		}
		i++;
	} while ( Len > i * 4 );

	return 0;

}

STATIC
INTN
SpmiRegisterWrite( UINT8 Channel, UINT8 Sid, UINT32 Addr, UINT8 *Buf, UINT32 Len )
{
	UINT32 Cmd;
	UINT32 Data =0 ;
	UINT16 Opc, i;

	if ( 1 == Len ) {
		if ( 0x1f >= Addr ) {
			Opc = SPMI_CMD_REG_WRITE;
		} else if ( 0xff >= Addr ) {
			Opc = SPMI_CMD_EXT_REG_WRITE;
		} else if ( 0xfff >= Addr ) {
			Opc = SPMI_CMD_EXT_REG_WRITE_L;
		} else {
			DEBUG (( DEBUG_ERROR, "SPMI: invalid Addr\n"));
			return -1;
		}
	} else {
		if ( 0xff >= Addr ) {
			Opc = SPMI_CMD_EXT_REG_WRITE;
		} else if ( 0xfff >= Addr ) {
			Opc = SPMI_CMD_EXT_REG_WRITE_L;
		} else {
			DEBUG (( DEBUG_ERROR, "SPMI: invalid Addr\n"));
			return -1;
		}
	}
	Cmd = SPMI_APB_SPMI_CMD_EN |									 /* cmd_en */
		 ( Opc << SPMI_APB_SPMI_CMD_TYPE_OFFSET ) |					 /* cmd_type */
		 ( ( Len - 1 ) << SPMI_APB_SPMI_CMD_LENGTH_OFFSET ) |	     /* byte_cnt */
		 ( ( Sid & 0xf ) << SPMI_APB_SPMI_CMD_SLAVEID_OFFSET ) |	 /* slvid */
		 ( ( Addr & 0xffff )  << SPMI_APB_SPMI_CMD_ADDR_OFFSET );	 /* slave_addr */

	i = 0;
	do {
		SetMem( &Data, 0, sizeof( Data ) );
		if ( ( Len - i * 4 ) >> 2 ) {
			CopyMem( &Data, Buf, sizeof( Data ) );
			Buf += sizeof( Data );
		} else {
			CopyMem( &Data, Buf, Len % 4 );
			Buf += ( Len % 4 );
		}

		Data = Tranverse32(Data);
		SPMI_WRITEL( SPMI_APB_SPMI_WDATA0_BASE_ADDR + SPMI_CHANNEL_OFFSET * Channel + i * 4, Data );
		i++;
	} while ( Len > i * 4 );

	SPMI_WRITEL( SPMI_APB_SPMI_CMD_BASE_ADDR + SPMI_CHANNEL_OFFSET * Channel, Cmd );

	return SpmiControllerWaitForDone( Channel, Sid );
}

EFI_STATUS
EFIAPI
SpmiRead(
  IN  HISI_SPMI_CONTROLLER_PROTOCOL       *This,
  IN  UINT8               Sid,
  IN  UINT32                Addr,
  IN  UINT8               *Buf,
  IN  UINT32                Len
  )
{
	int Ret;

	/* check sid */
	if ( SPMI_SLAVEID_MAX < Sid ) {
		 DEBUG ((DEBUG_ERROR, "SPMI: invalid slave id\n"));
		 return EFI_UNSUPPORTED;
	}

	/* check buf */
	if ( NULL == Buf ) {
		DEBUG ((DEBUG_ERROR, "SPMI: NULL buf\n"));
		return EFI_UNSUPPORTED;
	}

	/* check length */
	if ( Len == 0 ) {
		DEBUG ((DEBUG_ERROR, "SPMI: Please input invalid len\n"));/*[false alarm]: need check len*/
		return EFI_UNSUPPORTED;
	}

	while ( Len > SPMI_TRANS_BYTE_LEN_MAX ) {
		Ret = SpmiRegisterRead( SPMI_UEFI_CHANNEL, Sid, Addr, Buf, SPMI_TRANS_BYTE_LEN_MAX );
		if ( Ret < 0 )
			return EFI_UNSUPPORTED;

		Len -= SPMI_TRANS_BYTE_LEN_MAX;
		Buf += SPMI_TRANS_BYTE_LEN_MAX;
	}
	Ret = SpmiRegisterRead( SPMI_UEFI_CHANNEL, Sid, Addr, Buf, Len );
	return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
SpmiWrite(
  IN  HISI_SPMI_CONTROLLER_PROTOCOL       *This,
  IN  UINT8               Sid,
  IN  UINT32                Addr,
  IN  UINT8               *Buf,
  IN  UINT32                Len
  )
{
	int Ret;

	/* check length */
	if ( Len == 0 ) {
		DEBUG ((DEBUG_ERROR, "SPMI: Please input invalid len\n"));/*[false alarm]: need check len*/
		return EFI_UNSUPPORTED;
	}

	/* check sid */
	if ( SPMI_SLAVEID_MAX < Sid ) {
		DEBUG ((DEBUG_ERROR, "SPMI: invalid slave id\n"));
		return EFI_UNSUPPORTED;
	}

	/* check buf */
	if ( NULL == Buf ) {
		DEBUG ((DEBUG_ERROR, "SPMI: NULL buf\n"));
		return EFI_UNSUPPORTED;
	}
	while ( Len > SPMI_TRANS_BYTE_LEN_MAX ) {
		Ret = SpmiRegisterWrite( SPMI_UEFI_CHANNEL, Sid, Addr, Buf, SPMI_TRANS_BYTE_LEN_MAX );
		if ( Ret < 0 )
			return EFI_UNSUPPORTED;

		Len -= SPMI_TRANS_BYTE_LEN_MAX;
		Buf += SPMI_TRANS_BYTE_LEN_MAX;
	}
	Ret = SpmiRegisterWrite( SPMI_UEFI_CHANNEL, Sid, Addr, Buf, Len );
	return EFI_SUCCESS;
}

HISI_SPMI_CONTROLLER_PROTOCOL gSpmiController = {
    SpmiRead,
    SpmiWrite
};

EFI_STATUS
EFIAPI
SpmiControllerEntryPoint (
  IN EFI_HANDLE                            ImageHandle,
  IN EFI_SYSTEM_TABLE                      *SystemTable
  )
{
  EFI_STATUS        Status;

  Status = gBS->InstallProtocolInterface (
                &ImageHandle,
                &gHisiSpmiControllerProtocolGuid,
				EFI_NATIVE_INTERFACE,
                &gSpmiController
                );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  return Status;
}


