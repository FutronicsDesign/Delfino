/*
 *  @file pl455.h
 *
 *  @author Stephen Holland - Texas Instruments Inc.
 *  @date June 2015
 *  @version 1.0 Initial version
 *  @note Built with CCS for Hercules Version: 5.5.0
 */

/*****************************************************************************
**
**  Copyright (c) 2011-2015 Texas Instruments
**
******************************************************************************/


#ifndef PL455_H_
#define PL455_H_

//typedefs
#include "stdint.h"

//typrdefs
typedef unsigned char BYTE;
typedef unsigned int BOOL;
typedef signed int HANDLE;


// User defines
#define FRMWRT_SGL_R	0x00 // single device write with response
#define FRMWRT_SGL_NR	0x10 // single device write without response
#define FRMWRT_GRP_R	0x20 // group broadcast with response
#define FRMWRT_GRP_NR	0x30 // group broadcast without response
#define FRMWRT_ALL_R	0x60 // general broadcast with response
#define FRMWRT_ALL_NR	0x70 // general broadcast without response

#define TOTALBOARDS	16
#define BAUDRATE 250000

// Function Prototypes
void ResetPL455();
void WakePL455();
 void CommClear(void);
void CommReset(void);
BOOL GetFaultStat();

//uint16  B2SWORD(uint16_t wIN);
//uint32 B2SDWORD(uint32_t dwIN);
//uint32 B2SINT24(uint32_t dwIN24);

int  WriteReg(BYTE bID, uint16_t wAddr, uint64_t dwData, BYTE bLen, BYTE bWriteType);
int  ReadReg(BYTE bID, uint16_t wAddr, void * pData, BYTE bLen, uint32_t dwTimeOut);

int  WriteFrame(BYTE bID, uint16_t wAddr, BYTE * pData, BYTE bLen, BYTE bWriteType);
int  ReadFrameReq(BYTE bID, uint16_t wAddr, BYTE bByteToReturn);

void delayms(uint16_t ms);
void delayus(uint16_t us);

#endif /* PL455_H_ */
//EOF
