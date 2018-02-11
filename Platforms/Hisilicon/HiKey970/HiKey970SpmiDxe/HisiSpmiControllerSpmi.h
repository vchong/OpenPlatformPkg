/*
 * HisiSpmiControllerSpmi.h
 *
 * Function prototype definition.
 *
 * Copyright (c) 2001-2017, Huawei Tech. Co., Ltd. All rights reserved.
 *
 */

#ifndef __SPMI_H_
#define __SPMI_H_

/* 
 * slave id
*/
#define	SPMI_SLAVEID_HI6421v600  (0)

/*
 * SPMI register addr
 */
#define SOC_ACPU_SPMI_BASE_ADDR                (0xFFF24000)
#define SPMI_BASE_ADDR						    SOC_ACPU_SPMI_BASE_ADDR

#define SPMI_CHANNEL_OFFSET					    0x0300
#define SPMI_SLAVE_OFFSET						0x20

#define SPMI_APB_SPMI_CMD_BASE_ADDR				(SPMI_BASE_ADDR + 0x0100)
#define SPMI_APB_SPMI_WDATA0_BASE_ADDR			(SPMI_BASE_ADDR + 0x0104)
#define SPMI_APB_SPMI_WDATA1_BASE_ADDR			(SPMI_BASE_ADDR + 0x0108)
#define SPMI_APB_SPMI_WDATA2_BASE_ADDR			(SPMI_BASE_ADDR + 0x010c)
#define SPMI_APB_SPMI_WDATA3_BASE_ADDR			(SPMI_BASE_ADDR + 0x0110)

#define SPMI_APB_SPMI_STATUS_BASE_ADDR			( SPMI_BASE_ADDR + 0x0200 )

#define SPMI_APB_SPMI_RDATA0_BASE_ADDR			( SPMI_BASE_ADDR + 0x0204 )
#define SPMI_APB_SPMI_RDATA1_BASE_ADDR			( SPMI_BASE_ADDR + 0x0208 )
#define SPMI_APB_SPMI_RDATA2_BASE_ADDR			( SPMI_BASE_ADDR + 0x020c )
#define SPMI_APB_SPMI_RDATA3_BASE_ADDR			( SPMI_BASE_ADDR + 0x0210 )

/*
 * SPMI cmd register
 */
#define SPMI_APB_SPMI_CMD_EN					(1 << 31)
#define SPMI_APB_SPMI_CMD_TYPE_OFFSET			24
#define SPMI_APB_SPMI_CMD_LENGTH_OFFSET			20
#define SPMI_APB_SPMI_CMD_SLAVEID_OFFSET		16
#define SPMI_APB_SPMI_CMD_ADDR_OFFSET			0

/*
 * SPMI status register
 */
#define SPMI_APB_TRANS_DONE						(1)
#define SPMI_APB_TRANS_FAIL						(1 << 2)

#define SPMI_UEFI_CHANNEL					    2
#define SPMI_TRANS_BYTE_LEN_MAX					8
#define SPMI_SLAVEID_MAX                        15
#define SPMI_STATUS_TIMEOUT_US					1000

#define SPMI_WRITEL( addr, reg_val )	\
	do { \
		MmioWrite32( ( addr ), ( reg_val ) ); \
	} while (0)

#define  SPMI_READL( addr, reg_val )	\
	do { \
		reg_val = MmioRead32( addr );\
	} while (0)

#define Tranverse32(X)                 ((((UINT32)(X) & 0xff000000) >> 24) | \
                                                           (((UINT32)(X) & 0x00ff0000) >> 8) | \
                                                           (((UINT32)(X) & 0x0000ff00) << 8) | \
                                                           (((UINT32)(X) & 0x000000ff) << 24))

/* Command Opcodes */
enum spmi_controller_cmd_op_code {
	SPMI_CMD_REG_ZERO_WRITE = 0,
	SPMI_CMD_REG_WRITE = 1,
	SPMI_CMD_REG_READ = 2,
	SPMI_CMD_EXT_REG_WRITE = 3,
	SPMI_CMD_EXT_REG_READ = 4,
	SPMI_CMD_EXT_REG_WRITE_L = 5,
	SPMI_CMD_EXT_REG_READ_L = 6,
	SPMI_CMD_RESET = 7,
	SPMI_CMD_SLEEP = 8,
	SPMI_CMD_SHUTDOWN = 9,
	SPMI_CMD_WAKEUP = 10,
};

typedef struct _HISI_SPMI_CONTROLLER_PROTOCOL       HISI_SPMI_CONTROLLER_PROTOCOL;

//
// Function Prototypes
//

typedef
EFI_STATUS
(EFIAPI *HISI_SPMI_CONTROLLER_READ) (
  IN  HISI_SPMI_CONTROLLER_PROTOCOL       *This,
  IN  UINT8               Sid,
  IN  UINT32                Addr,
  IN  UINT8               *Buf,
  IN  UINT32                Len
  );
/*++

Routine Description:

  Read the Value of a Reg

Arguments:

  This  - pointer to protocol
  Sid   - slvid
  Addr  - slave_addr
  Buf   - Reg Value
  Len   - Data Lengths

Returns:

  EFI_SUCCESS - Reg Read as requested

--*/

typedef
EFI_STATUS
(EFIAPI *HISI_SPMI_CONTROLLER_WRITE) (
  IN  HISI_SPMI_CONTROLLER_PROTOCOL       *This,
  IN  UINT8               Sid,
  IN  UINT32                Addr,
  IN  UINT8               *Buf,
  IN  UINT32                Len
  );
/*++

Routine Description:

  Write the Value to a Reg

Arguments:

  This  - pointer to protocol
  Sid   - slvid
  Addr  - slave_addr
  Buf   - Reg Value
  Len   - Data Lengths


Returns:

  EFI_SUCCESS - Reg Write as requested

--*/

struct _HISI_SPMI_CONTROLLER_PROTOCOL{
  HISI_SPMI_CONTROLLER_READ  Read;
  HISI_SPMI_CONTROLLER_WRITE  Write;
};

extern EFI_GUID gHisiSpmiControllerProtocolGuid;

#endif
