/**
 * @file osal.h
 *
 * @author JP
 *
 * @version 0.2
 *
 * @brief This file contains windows support API declarations for DCA1000EVM 
 * system OSAL services such as shared memory, timer for CLI application
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

#ifndef OSAL_H
#define OSAL_H

#include "../globals.h"

///****************
/// Typedefs
///****************

#ifdef _WIN32

/** Definition for shared memory handle  (Windows)      */
typedef HANDLE OSAL_SHM_HANDLE_TYPE;

/** Definition for event handle (Windows)               */
typedef HANDLE OSAL_SIGNAL_HANDLE_TYPE;

#else

/** Definition for pthread condition variable handle    */
typedef pthread_cond_t OSAL_CONDN_HANDLE_TYPE;

/** Definition for pthread mutex variable handle        */
typedef pthread_mutex_t OSAL_MUTEX_HANDLE_TYPE;

/** Definition for shared memory handle (Linux)         */
typedef SINT32 OSAL_SHM_HANDLE_TYPE;

/** Structure definition for event handle (Linux)       */
typedef struct
{
    /** Condition variable handle  */
    OSAL_CONDN_HANDLE_TYPE condnVar;

    /** Mutex variable handle      */
    OSAL_MUTEX_HANDLE_TYPE mutexVar;

    /** Signalled state flag       */
    bool bSignalled;
}OSAL_SIGNAL_HANDLE_TYPE;

#endif


/** @class osal
 * @brief This class provides support APIs for shared memory and  <!--
 * --> timer event handling.
 */
class osal
{
    /** Shared memory name          */
    SINT8 s8ShmName[MAX_NAME_LEN];

    /** Shared memory structure     */
    SHM_PROC_STATES *shm_proc_states;

    /** Shared memory handle        */
    OSAL_SHM_HANDLE_TYPE hShmem;

    /** Shared memory inline stats update status variable*/
    SINT32 s16StatusInline = SUCCESS_STATUS;

public:

    /** @fn osal()
     * @brief This constructor is used to initialize variables
     */
    osal();

    /** @fn SINT32 QueryRecordProcStatus(UINT16 u16ConfigPort, SHM_PROC_STATES *procStates)
     * @brief This function is to read the record process status from shared <!--
     * --> memory
     * @param [in] u16ConfigPort [UINT16] - Config port
     * @param [out] procStates [SHM_PROC_STATES *] - Structure filled with process status
     * @return SINT32 value
     */
    SINT32 QueryRecordProcStatus(UINT16 u16ConfigPort, SHM_PROC_STATES *procStates);

    /** @fn SINT32 WriteDefaultRecordProcStatus(UINT16 u16ConfigPort)
     * @brief This function is to write the default value for the record <!--
     * --> process status in the shared memory when it is created
     * @param [in] u16ConfigPort [UINT16] - Config port
     * @return SINT32 value
     */
    SINT32 WriteDefaultRecordProcStatus(UINT16 u16ConfigPort);

    /** @fn  SINT32 WriteRecordProcStatus(UINT16 u16ConfigPort, SINT32 s32CmdStatus)
     * @brief This function is to write the command status of record <!--
     * --> process in the shared memory
     * @param [in] u16ConfigPort [UINT16] - Config port
     * @param [in] s32CmdStatus [SINT32] - Command status
     * @return SINT32 value
     */
    SINT32 WriteRecordProcStatus(UINT16 u16ConfigPort, SINT32 s32CmdStatus);

    /** @fn SINT32 WriteRecAsyncStatus(UINT16 u16ConfigPort, UINT32 u32AsyncStatus)
     * @brief This function is to write the async status from FPGA <!--
     * --> in the shared memory
     * @param [in] u16ConfigPort [UINT16] - Config port
     * @param [in] u32AsyncStatus [UINT32] - FPGA Async status
     * @return SINT32 value
     */
    SINT32 WriteRecAsyncStatus(UINT16 u16ConfigPort, UINT32 u32AsyncStatus);

#if 0
    /** @fn SINT32 WriteRecProcPostProcStats(UINT16 u16ConfigPort, UINT32 u32AsyncStatus, UINT8 u8DataIndex)
     * @brief This function is to write the out of sequence status from FPGA <!--
     * --> in the shared memory along with the out of sequence count for each of the ports
     * @param [in] u16ConfigPort [UINT16] - Config port
     * @param [in] u32AsyncStatus [UINT32] - FPGA Async status
     * @param [in] u8DataIndex [UINT8] - Data type Index
     * @return SINT32 value
     */
    SINT32 WriteRecProcPostProcStats(UINT16 u16ConfigPort, UINT32 u32AsyncStatus,
                                         UINT8 u8DataIndex);
#endif

    /** @fn SINT32 DestroyShm(UINT16 u16ConfigPort)
     * @brief This function is to destroy the shared memory that <!--
     * --> already created
     * @param [in] u16ConfigPort [UINT16] - Config port
     * @return SINT32 value
     */
    SINT32 DestroyShm(UINT16 u16ConfigPort);

    /** @fn  SINT32 CreateShm(UINT16 u16ConfigPort)
     * @brief This function is to create the shared memory using <!--
     * --> config port number
     * @param [in] u16ConfigPort [UINT16] - Config port
     * @return SINT32 value
     */
    SINT32 CreateShm(UINT16 u16ConfigPort);

    /** @fn SINT32 WriteRecProcInlineStats(UINT16 u16ConfigPort, <!--
     * --> strRFDCCard_InlineProcStats strInlineStats, bool bOutOfSeqSetFlag <!--
     * --> UINT8 u8DataIndex)
     * @brief This function is to write the inline process summary of record process <!--
     * --> in the shared memory
     * @param [in] u16ConfigPort [UINT16] - Config port
     * @param [in] strInlineStats [strRFDCCard_InlineProcStats *] - Inline process summary
     * @param [in] bOutOfSeqSetFlag [bool] - Out of sequence set flag
     * @param [in] u8DataIndex [UINT8] - Data type index
     * @return SINT32 value
     */
    SINT32 WriteRecProcInlineStats(UINT16 u16ConfigPort,
                                   strRFDCCard_InlineProcStats *strInlineStats,
                                   bool bOutOfSeqSetFlag,
                                   UINT8 u8DataIndex);

    /** @fn  void SleepInMilliSec(UINT32 u32MilliSec)
     * @brief This function is to add delay in milli second
     * @param [in] u32MilliSec [UINT32] - Milli second
     */
    void SleepInMilliSec(UINT32 u32MilliSec);

    /** @fn SINT32 IsValidDir(SINT8 *s8FileBasePath)
     * @brief This function is to validate the directory path
     * @param [in] s8FileBasePath [SINT8 *] - Directory path
     * @return SINT32 value
     */
    SINT32 IsValidDir(SINT8 *s8FileBasePath);

    /** @fn void WRITE_TO_LOG_FILE(const SINT8 *s8Msg)
     * @brief This function is to write the CLI messages in the logfile
     * @param [in] s8Msg [const SINT8 *] - Message to display
     */
    void WRITE_TO_LOG_FILE(const SINT8 *s8Msg);

    /** @fn SINT32 MapShm(UINT16 u16ConfigPort)
     * @brief This function is to map shared memory for updating record process status
     * @param [in] u16ConfigPort [UINT16] - Config port
     */
    SINT32 MapShm(UINT16 u16ConfigPort);

    /** @fn void UnmapShm()
     * @brief This function is to unmap the handle from shared memory
     */
    void UnmapShm();

    /** @fn void SignalEvent(OSAL_SIGNAL_HANDLE_TYPE *event)
     * @brief This function is to trigger a signal for the waiting event
     * @param [in] event [OSAL_SIGNAL_HANDLE_TYPE *] - Event
     */
    void SignalEvent(OSAL_SIGNAL_HANDLE_TYPE *event);

    /** @fn SINT32 osal::WaitForSignal(OSAL_SIGNAL_HANDLE_TYPE *event, UINT32 u32Sec)
     * @brief This function is to make the event wait for a signal for a defined time
     * @param [in] event [OSAL_SIGNAL_HANDLE_TYPE *] - Event
     * @param [in] u32Sec [UINT32] - second
     * @return SINT32 value
     */
    SINT32 WaitForSignal(OSAL_SIGNAL_HANDLE_TYPE *event, UINT32 u32Sec);

    /** @fn void InitEvent(OSAL_SIGNAL_HANDLE_TYPE *event)
     * @brief This function is to initialize an event
     * @param [in] event [OSAL_SIGNAL_HANDLE_TYPE *] - Event
     */
    void InitEvent(OSAL_SIGNAL_HANDLE_TYPE *event);

    /** @fn void DeInitEvent(OSAL_SIGNAL_HANDLE_TYPE *event)
     * @brief This function is to deinitialize an event
     * @param [in] event [OSAL_SIGNAL_HANDLE_TYPE *] - Event
     */
    void DeInitEvent(OSAL_SIGNAL_HANDLE_TYPE *event);

    /** @fn SINT32 sock_Close(SINT32 s32SocketId)
     * @brief This function is to close the socket
     * @param [in] s32SocketId [SINT32] - Socket ID
     * @return SINT32 value
     */
    SINT32 sock_Close(SINT32 s32SocketId);

    /** @fn SINT32 GetLastErrNo()
     * @brief This function is to get the last error system code
     * @return SINT32 value
     */
    SINT32 GetLastErrNo();

    /** @fn bool IsCmdTimeout()
     * @brief This function is to check whether timeout occurs for command response
     * @return boolean value
     */
    bool IsCmdTimeout();

    /** @fn SINT32 sock_setopt(SINT32 s32SocketId, UINT32 u32Seconds)
     * @brief This function is to set timeout options for the given socket ID
     * @param [in] s32SocketId [SINT32] - Socket ID
     * @param [in] u32Seconds [UINT32] - Timeout in Seconds
     * @return SINT32 value
     */
    SINT32 sock_setopt(SINT32 s32SocketId, UINT32 u32Seconds);
};

#endif // OSAL_H
