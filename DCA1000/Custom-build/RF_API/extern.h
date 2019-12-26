/**
 * @file extern.h
 *
 * @author JP
 *
 * @version 0.1
 *
 * @brief This file contains extern declarations for DCA1000EVM system
 *
 * @par
 * NOTE:
 *     (C) Copyright 2019 Texas Instruments, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


///*****************************************************************************
/// HISTORY :
/// VERSION        DATE              AUTHOR      CHANGE DESCRIPTION
/// 0.1            07 Dec 2018       JP          Created
/// 0.2            17 Jan 2018       JP          Inline processing added and
///                                              Review comments incorporated
///*****************************************************************************

#ifndef EXTERN_H
#define EXTERN_H

///****************
/// Includes
///****************

#include "../Common/rf_api_internal.h"
#include "../Common/Osal_Utils/osal.h"

///****************
/// Externs
///****************

/** Socket IDs structure                                */
extern strRFDCCard_SockInfo sRFDCCard_SockInfo;

/** Callback event handler - Command & Async status     */
extern EVENT_HANDLER        RFDCARD_Callback;

/** Callback event handler - Inline processing summary  */
extern INLINE_PROC_HANDLER RecordInlineProc_Callback;

/** Start record config structure                       */
extern strStartRecConfigMode sRFDCCard_StartRecConfig;

/** Packets dropped count structure                     */
extern strRFDCCard_InlineProcStats sRFDCCard_InlineStats;

/** Maximum file size to capture                        */
extern UINT32 u32MaxFileSizeToCapture;

/** Record stop command sent flag                       */
extern bool gbRecStopCmdSent;

/** Fill zero bytes array for dropped packets           */
extern SINT8 s8ZeroBuf[MAX_BYTES_PER_PACKET];

/** Osal class object                                   */
extern osal osalObj;

/** Record - Lane number                                */
extern UINT8 u8LaneNumber;

/** Inline process log file pointer                     */
extern FILE *pInlineLogFile;

/** Duration stop mode timeout wait event
 * Thread and wait event enabled when duration stop mode config is set for recording
 */
extern OSAL_SIGNAL_HANDLE_TYPE sgnDurationStopModeWaitEvent;

/** Capture timeout wait event.
 *  To handle system disconnect/idleness while recording
 */
extern OSAL_SIGNAL_HANDLE_TYPE sgnCaptureTimeoutWaitEvent;


#endif // EXTERN_H

