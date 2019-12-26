/**
 * @file osal_win.cpp
 *
 * @author JP
 *
 * @version 0.3
 *
 * @brief This file contains API definitions for DCA1000EVM system OSAL
 * services such as shared memory, timer for CLI application
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
/// 0.2            27 Dec 2018       JP          Linux support APIs added
/// 0.3            17 Jan 2018       JP          Inline processing added and
///                                              Review comments incorporated
///*****************************************************************************

#ifdef _WIN32
#include <stdio.h>
#include <stdlib.h>
#include "osal.h"
#include "../errcodes.h"


/** @fn osal::osal()
 * @brief This constructor is used to initialize variables
 */
osal::osal()
{
    hShmem = NULL;
    shm_proc_states = NULL;
    s16StatusInline = SUCCESS_STATUS;
}

/** @fn SINT32 osal::DestroyShm(UINT16 u16ConfigPort)
 * @brief This function is to destroy the shared memory that <!--
 * --> already created
 * @param [in] u16ConfigPort [UINT16] - Config port
 * @return SINT32 value
 */
SINT32 osal::DestroyShm(UINT16 u16ConfigPort)
{
	SINT32 s32ErrStatus = 0;
    SINT8 s8DebugMsg[MAX_NAME_LEN];

	memset(s8ShmName, '\0', MAX_NAME_LEN);

	sprintf(s8ShmName, "%s_%d", CLI_SHM_PREFIX_NAME, u16ConfigPort);
	
    if(hShmem == NULL)
    {
        /* Shared memory -                              */
        hShmem = OpenFileMappingA(
                   FILE_MAP_ALL_ACCESS,   // read/write access
                   FALSE,                 // do not inherit the name
                   s8ShmName);               // name of mapping object

        if (hShmem == NULL)
        {
            sprintf(s8DebugMsg,
                  "\nCould not open shared memory mapping object (%d). [error %d]\n",
                  s32ErrStatus, CLI_SHM_NOT_AVAIL_ERR);
            WRITE_TO_LOG_FILE(s8DebugMsg);

            return CLI_SHM_NOT_AVAIL_ERR;
        }
    }

   CloseHandle(hShmem);

   hShmem = NULL;
   shm_proc_states = NULL;

   return SUCCESS_STATUS;
}

/** @fn SINT32 osal::CreateShm(UINT16 u16ConfigPort)
 * @brief This function is to create the shared memory using <!--
 * --> config port number
 * @param [in] u16ConfigPort [UINT16] - Config port
 * @return SINT32 value
 */
SINT32 osal::CreateShm(UINT16 u16ConfigPort)
{
	SINT32 s32ErrStatus = 0;
    SINT8 s8DebugMsg[MAX_NAME_LEN];

	memset(s8ShmName, '\0', MAX_NAME_LEN);

	sprintf(s8ShmName, "%s_%d", CLI_SHM_PREFIX_NAME, u16ConfigPort);

    if(hShmem == NULL)
    {
        /* Shared memory -                              */
        hShmem = CreateFileMappingA(INVALID_HANDLE_VALUE,
                                    NULL,
                                    PAGE_READWRITE,
                                    0,
                                    sizeof(SHM_PROC_STATES),
                                    s8ShmName
                                    );

        if (hShmem == NULL)
        {
            sprintf(s8DebugMsg,
                   "\nCould not create shared memory mapping object (%d). [error %d]\n",
                   s32ErrStatus, CLI_SHM_CREATION_FAILURE_ERR);
            WRITE_TO_LOG_FILE(s8DebugMsg);

            return CLI_SHM_CREATION_FAILURE_ERR;
        }
        shm_proc_states = NULL;
    }

	WriteDefaultRecordProcStatus(u16ConfigPort);

	return SUCCESS_STATUS;
}

/** @fn SINT32 osal::QueryRecordProcStatus(UINT16 u16ConfigPort, SHM_PROC_STATES *procStates)
 * @brief This function is to read the record process status from shared <!--
 * --> memory
 * @param [in] u16ConfigPort [UINT16] - Config port
 * @param [out] procStates [SHM_PROC_STATES *] - Structure filled with process status
 * @return SINT32 value
 */
SINT32 osal::QueryRecordProcStatus(UINT16 u16ConfigPort, SHM_PROC_STATES *procStates)
{
    SINT32 s32ErrStatus = 0;
    SINT8 s8DebugMsg[MAX_NAME_LEN];

	memset(s8ShmName, '\0', MAX_NAME_LEN);

	sprintf(s8ShmName, "%s_%d", CLI_SHM_PREFIX_NAME, u16ConfigPort);

    if(hShmem == NULL)
    {
        /* Shared memory -                              */
        hShmem = OpenFileMappingA(
                   FILE_MAP_ALL_ACCESS,   // read/write access
                   FALSE,                 // do not inherit the name
                   s8ShmName);               // name of mapping object

        if (hShmem == NULL)
        {
            sprintf(s8DebugMsg,
            "\nCould not open shared memory mapping object (%d). [error %d]\n",
            s32ErrStatus, CLI_SHM_NOT_AVAIL_ERR);
            //WRITE_TO_LOG_FILE(s8DebugMsg);

            return CLI_SHM_NOT_AVAIL_ERR;
        }
    }

    if(shm_proc_states == NULL)
    {
        shm_proc_states = (SHM_PROC_STATES *)MapViewOfFile(hShmem,
                                            FILE_MAP_ALL_ACCESS, 0, 0,
                                            sizeof(SHM_PROC_STATES));

        if (shm_proc_states == NULL)
        {
            s32ErrStatus = GetLastError();

            sprintf(s8DebugMsg,
                   "\nCould not map shared memory mapping object (%d). [error %d]\n",
                   s32ErrStatus, CLI_SHM_MAPPING_FAILURE_ERR);
            WRITE_TO_LOG_FILE(s8DebugMsg);

            return CLI_SHM_MAPPING_FAILURE_ERR;
        }
    }

	procStates->u16CommandCode = shm_proc_states->u16CommandCode;
	procStates->s32CommandStatus = shm_proc_states->s32CommandStatus;
	procStates->u32AsyncStatus = shm_proc_states->u32AsyncStatus;
    for(int i = 0; i < NUM_DATA_TYPES; i ++)
    {
        strcpy(procStates->strInlineProcStats.s8HeaderId[i],
                shm_proc_states->strInlineProcStats.s8HeaderId[i]);
        procStates->strInlineProcStats.u32FirstPktId[i] =
                shm_proc_states->strInlineProcStats.u32FirstPktId[i];
        procStates->strInlineProcStats.u32LastPktId[i] =
                shm_proc_states->strInlineProcStats.u32LastPktId[i];
        procStates->strInlineProcStats.u64OutOfSeqCount[i] =
                shm_proc_states->strInlineProcStats.u64OutOfSeqCount[i];
        procStates->strInlineProcStats.u64NumOfRecvdPackets[i] =
                shm_proc_states->strInlineProcStats.u64NumOfRecvdPackets[i];
        procStates->strInlineProcStats.u64NumOfZeroFilledPackets[i] =
                shm_proc_states->strInlineProcStats.u64NumOfZeroFilledPackets[i];
        procStates->strInlineProcStats.u64NumOfZeroFilledBytes[i] =
                shm_proc_states->strInlineProcStats.u64NumOfZeroFilledBytes[i];
        procStates->strInlineProcStats.u32OutOfSeqPktFromOffset[i] =
                shm_proc_states->strInlineProcStats.u32OutOfSeqPktFromOffset[i];
        procStates->strInlineProcStats.u32OutOfSeqPktToOffset[i] =
                shm_proc_states->strInlineProcStats.u32OutOfSeqPktToOffset[i];
        procStates->strInlineProcStats.StartTime[i] =
                shm_proc_states->strInlineProcStats.StartTime[i];
        procStates->strInlineProcStats.EndTime[i] =
                shm_proc_states->strInlineProcStats.EndTime[i];
    }

    // release
    UnmapShm();

    return SUCCESS_STATUS;
}

/** @fn SINT32 osal::WriteDefaultRecordProcStatus(UINT16 u16ConfigPort)
 * @brief This function is to write the default value for the record <!--
 * --> process status in the shared memory when it is created
 * @param [in] u16ConfigPort [UINT16] - Config port
 * @return SINT32 value
 */
SINT32 osal::WriteDefaultRecordProcStatus(UINT16 u16ConfigPort)
{
    SINT16 s16Status = SUCCESS_STATUS;

    if (hShmem == NULL)
    {
        s16Status = CreateShm(u16ConfigPort);
        if(s16Status == SUCCESS_STATUS)
        {
            s16Status = MapShm(u16ConfigPort);
            if(s16Status != SUCCESS_STATUS)
            {
                return s16Status;
            }
        }
        else
            return s16Status;
    }
    if (shm_proc_states == NULL)
    {
        s16Status = MapShm(u16ConfigPort);
        if(s16Status != SUCCESS_STATUS)
        {
            return s16Status;
        }
    }

    shm_proc_states->u16CommandCode = CMD_CODE_START_RECORD;
    shm_proc_states->s32CommandStatus = STS_CLI_REC_PROC_STOPPED;
    shm_proc_states->u32AsyncStatus = 0;
    for(int i = 0; i < NUM_DATA_TYPES; i ++)
    {
        strcpy(shm_proc_states->strInlineProcStats.s8HeaderId[i], "");
        shm_proc_states->strInlineProcStats.u32FirstPktId[i] = 0;
        shm_proc_states->strInlineProcStats.u32LastPktId[i] = 0;
        shm_proc_states->strInlineProcStats.u64OutOfSeqCount[i] = 0;
        shm_proc_states->strInlineProcStats.u64NumOfRecvdPackets[i] = 0;
        shm_proc_states->strInlineProcStats.u64NumOfZeroFilledPackets[i] = 0;
        shm_proc_states->strInlineProcStats.u64NumOfZeroFilledBytes[i] = 0;
        shm_proc_states->strInlineProcStats.u32OutOfSeqPktFromOffset[i] = 0;
        shm_proc_states->strInlineProcStats.u32OutOfSeqPktToOffset[i] = 0;
        shm_proc_states->strInlineProcStats.StartTime[i] = time(NULL);
        shm_proc_states->strInlineProcStats.EndTime[i] = time(NULL);
    }

    return SUCCESS_STATUS;
}

/** @fn SINT32 osal::WriteRecordProcStatus(UINT16 u16ConfigPort, SINT32 s32CmdStatus)
 * @brief This function is to write the command status of record <!--
 * --> process in the shared memory
 * @param [in] u16ConfigPort [UINT16] - Config port
 * @param [in] s32CmdStatus [SINT32] - Command status
 * @return SINT32 value
 */
SINT32 osal::WriteRecordProcStatus(UINT16 u16ConfigPort, SINT32 s32CmdStatus)
{
    SINT16 s16Status = SUCCESS_STATUS;

    if (hShmem == NULL)
    {
        s16Status = CreateShm(u16ConfigPort);
        if(s16Status == SUCCESS_STATUS)
        {
            s16Status = MapShm(u16ConfigPort);
            if(s16Status != SUCCESS_STATUS)
            {
                return s16Status;
            }
        }
        else
            return s16Status;
    }
    if (shm_proc_states == NULL)
    {
        s16Status = MapShm(u16ConfigPort);
        if(s16Status != SUCCESS_STATUS)
        {
            return s16Status;
        }
    }

   	shm_proc_states->s32CommandStatus = s32CmdStatus;

    return SUCCESS_STATUS;
}

/** @fn SINT32 osal::WriteRecAsyncStatus(UINT16 u16ConfigPort, UINT32 u32AsyncStatus)
 * @brief This function is to write the async status from FPGA <!--
 * --> in the shared memory
 * @param [in] u16ConfigPort [UINT16] - Config port
 * @param [in] u32AsyncStatus [UINT32] - FPGA Async status
 * @return SINT32 value
 */
SINT32 osal::WriteRecAsyncStatus(UINT16 u16ConfigPort, UINT32 u32AsyncStatus)
{
    SINT32 s16Status = SUCCESS_STATUS;

    if (hShmem == NULL)
    {
        s16Status = CreateShm(u16ConfigPort);
        if(s16Status == SUCCESS_STATUS)
        {
            s16Status = MapShm(u16ConfigPort);
            if(s16Status != SUCCESS_STATUS)
            {
                return s16Status;
            }
        }
        else
            return s16Status;
    }
    if (shm_proc_states == NULL)
    {
        s16Status = MapShm(u16ConfigPort);
        if(s16Status != SUCCESS_STATUS)
        {
            return s16Status;
        }
    }

	shm_proc_states->u32AsyncStatus |= u32AsyncStatus;

    return s16Status;
}

/** @fn SINT32 osal::WriteRecProcInlineStats(UINT16 u16ConfigPort, <!--
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
SINT32 osal::WriteRecProcInlineStats(UINT16 u16ConfigPort,
                                strRFDCCard_InlineProcStats *strInlineStats,
                                bool bOutOfSeqSetFlag,
                                UINT8 u8DataIndex)
{
    s16StatusInline = SUCCESS_STATUS;

    if (hShmem == NULL)
    {
        s16StatusInline = CreateShm(u16ConfigPort);
        if(s16StatusInline == SUCCESS_STATUS)
        {
            s16StatusInline = MapShm(u16ConfigPort);
            if(s16StatusInline != SUCCESS_STATUS)
            {
                return s16StatusInline;
            }
        }
        else
            return s16StatusInline;
    }
    if (shm_proc_states == NULL)
    {
        s16StatusInline = MapShm(u16ConfigPort);
        if(s16StatusInline != SUCCESS_STATUS)
        {
            return s16StatusInline;
        }
    }

    if(bOutOfSeqSetFlag)
        shm_proc_states->u32AsyncStatus |= (1 << STS_REC_PKT_OUT_OF_SEQ);

    strcpy(shm_proc_states->strInlineProcStats.s8HeaderId[u8DataIndex],
           strInlineStats->s8HeaderId[u8DataIndex]);
    shm_proc_states->strInlineProcStats.u32FirstPktId[u8DataIndex] =
            strInlineStats->u32FirstPktId[u8DataIndex];
    shm_proc_states->strInlineProcStats.u32LastPktId[u8DataIndex] =
            strInlineStats->u32LastPktId[u8DataIndex];
    shm_proc_states->strInlineProcStats.u64OutOfSeqCount[u8DataIndex] =
            strInlineStats->u64OutOfSeqCount[u8DataIndex];
    shm_proc_states->strInlineProcStats.u64NumOfRecvdPackets[u8DataIndex] =
            strInlineStats->u64NumOfRecvdPackets[u8DataIndex];
    shm_proc_states->strInlineProcStats.u64NumOfZeroFilledPackets[u8DataIndex] =
            strInlineStats->u64NumOfZeroFilledPackets[u8DataIndex];
    shm_proc_states->strInlineProcStats.u64NumOfZeroFilledBytes[u8DataIndex] =
            strInlineStats->u64NumOfZeroFilledBytes[u8DataIndex];
    shm_proc_states->strInlineProcStats.u32OutOfSeqPktFromOffset[u8DataIndex] =
            strInlineStats->u32OutOfSeqPktFromOffset[u8DataIndex];
    shm_proc_states->strInlineProcStats.u32OutOfSeqPktToOffset[u8DataIndex] =
            strInlineStats->u32OutOfSeqPktToOffset[u8DataIndex];
    shm_proc_states->strInlineProcStats.StartTime[u8DataIndex] =
            strInlineStats->StartTime[u8DataIndex];
    shm_proc_states->strInlineProcStats.EndTime[u8DataIndex] =
            strInlineStats->EndTime[u8DataIndex];

    return s16StatusInline;
}

/** @fn SINT32 osal::MapShm(UINT16 u16ConfigPort)
 * @brief This function is to map shared memory for updating record process status
 * @param [in] u16ConfigPort [UINT16] - Config port
 * @return SINT32 value
 */
SINT32 osal::MapShm(UINT16 u16ConfigPort)
{
    SINT32 s32ErrStatus = 0;
    SINT8 s8DebugMsg[MAX_NAME_LEN];

    memset(s8ShmName, '\0', MAX_NAME_LEN);

    sprintf(s8ShmName, "%s_%d", CLI_SHM_PREFIX_NAME, u16ConfigPort);

    if(hShmem == NULL)
    {
        /* Shared memory -                              */
        hShmem = OpenFileMappingA(
                   FILE_MAP_ALL_ACCESS,   // read/write access
                   FALSE,                 // do not inherit the name
                   s8ShmName);               // name of mapping object

        if (hShmem == NULL)
        {
            s32ErrStatus = GetLastError();

            sprintf(s8DebugMsg,
              "\nCould not open shared memory mapping object (%d). [error %d]\n",
              s32ErrStatus, CLI_SHM_NOT_AVAIL_ERR);

            return CLI_SHM_NOT_AVAIL_ERR;
       }
    }

    shm_proc_states = (SHM_PROC_STATES *)MapViewOfFile(hShmem,
                                        FILE_MAP_ALL_ACCESS, 0, 0,
                                        sizeof(SHM_PROC_STATES));
    if (shm_proc_states == NULL)
    {
        s32ErrStatus = GetLastError();

        sprintf(s8DebugMsg,
               "\nCould not map shared memory mapping object (%d). [error %d]\n",
               s32ErrStatus, CLI_SHM_MAPPING_FAILURE_ERR);
        WRITE_TO_LOG_FILE(s8DebugMsg);
        return CLI_SHM_MAPPING_FAILURE_ERR;
    }

    return SUCCESS_STATUS;
}

/** @fn void osal::UnmapShm()
 * @brief This function is to unmap the handle from shared memory
 */
void osal::UnmapShm()
{
    if(shm_proc_states != NULL)
    {
        UnmapViewOfFile(shm_proc_states);
        shm_proc_states = NULL;
    }
}

/** @fn void osal::SleepInMilliSec(UINT32 u32MilliSec)
 * @brief This function is to add delay in milli second
 * @param [in] u32MilliSec [UINT32] - Milli second
 */
void osal::SleepInMilliSec(UINT32 u32MilliSec)
{
    Sleep(u32MilliSec);
}

/** @fn SINT32 osal::IsValidDir(SINT8 *s8FileBasePath)
 * @brief This function is to validate the directory path
 * @param [in] s8FileBasePath [SINT8 *] - Directory path
 * @return SINT32 value
 */
SINT32 osal::IsValidDir(SINT8 *s8FileBasePath)
{
    DWORD dwAttr = GetFileAttributesA(s8FileBasePath);
    return ((dwAttr != 0xffffffff) && (dwAttr & FILE_ATTRIBUTE_DIRECTORY));
}

/** @fn void osal::WRITE_TO_LOG_FILE(const SINT8 *s8Msg)
 * @brief This function is to write the CLI messages in the logfile
 * @param [in] s8Msg [const SINT8 *] - Message to display
 */
void osal::WRITE_TO_LOG_FILE(const SINT8 *s8Msg)
{
    time_t loggingTime = time(NULL);
    FILE * pDebugFile;
    pDebugFile = fopen(CLI_LOG_NAME, "a+");

    if(NULL != pDebugFile)
    {
        fprintf(pDebugFile, "\n%s%s", ctime(&loggingTime), s8Msg);
    }

    fclose(pDebugFile);
}

/** @fn void osal::SignalEvent(OSAL_SIGNAL_HANDLE_TYPE *event)
 * @brief This function is to trigger a signal for the waiting event
 * @param [in] event [OSAL_SIGNAL_HANDLE_TYPE *] - Event
 */
void osal::SignalEvent(OSAL_SIGNAL_HANDLE_TYPE *event)
{
    if(*event)
        SetEvent(*event);
}

/** @fn SINT32 osal::WaitForSignal(OSAL_SIGNAL_HANDLE_TYPE *event, UINT32 u32Sec)
 * @brief This function is to make the event wait for a signal for a defined time
 * @param [in] event [OSAL_SIGNAL_HANDLE_TYPE *] - Event
 * @param [in] u32Sec [UINT32] - second
 * @return SINT32 value
 */
SINT32 osal::WaitForSignal(OSAL_SIGNAL_HANDLE_TYPE *event, UINT32 u32Sec)
{
    if(*event)
    {
        if(u32Sec == NON_STOP)
        {
            return WaitForSingleObject(*event, INFINITE);
        }
        else
        {
            if(WAIT_TIMEOUT == WaitForSingleObject(*event,
                                                   u32Sec * SEC_TO_MILLI_SEC_CONVERSION))
                return STS_RFDCCARD_EVENT_TIMEOUT_ERR;
        }
    }
    return SUCCESS_STATUS;
}

/** @fn void osal::InitEvent(OSAL_SIGNAL_HANDLE_TYPE *event)
 * @brief This function is to initialize an event
 * @param [in] event [OSAL_SIGNAL_HANDLE_TYPE] - Event
 */
void osal::InitEvent(OSAL_SIGNAL_HANDLE_TYPE *event)
{
    *event = CreateEvent(NULL, false, false, NULL);
}

/** @fn void osal::DeInitEvent(OSAL_SIGNAL_HANDLE_TYPE *event)
 * @brief This function is to deinitialize an event
 * @param [in] event [OSAL_SIGNAL_HANDLE_TYPE *] - Event
 */
void osal::DeInitEvent(OSAL_SIGNAL_HANDLE_TYPE *event)
{
    if(*event)
    {
        CloseHandle(*event);
        *event = 0;
    }
}

/** @fn SINT32 osal::sock_Close(SINT32 s32SocketId)
 * @brief This function is to close the socket
 * @param [in] s32SocketId [SINT32] - Socket ID
 * @return SINT32 value
 */
SINT32 osal::sock_Close(SINT32 s32SocketId)
{
    if(s32SocketId != -1)
    {
        return closesocket(s32SocketId);
    }

    return 0;
}

/** @fn SINT32 osal::GetLastErrNo()
 * @brief This function is to get the last error system code
 * @return SINT32 value
 */
SINT32 osal::GetLastErrNo()
{
    return WSAGetLastError();
}

/** @fn bool osal::IsCmdTimeout()
 * @brief This function is to check whether timeout occurs for command response
 * @return boolean value
 */
bool osal::IsCmdTimeout()
{
    if(WSAGetLastError() == WSAETIMEDOUT)
        return true;
    else
        return false;
}

/** @fn SINT32 sock_setopt(SINT32 s32SocketId, UINT32 u32Seconds)
 * @brief This function is to set timeout options for the given socket ID
 * @param [in] s32SocketId [SINT32] - Socket ID
 * @param [in] u32Seconds [UINT32] - Timeout in Seconds
 * @return SINT32 value
 */
SINT32 osal::sock_setopt(SINT32 s32SocketId, UINT32 u32Seconds)
{
    SINT32 nTimeout = (u32Seconds * SEC_TO_MILLI_SEC_CONVERSION) ;
    return (setsockopt(s32SocketId, SOL_SOCKET, SO_RCVTIMEO,
                  (const char*)&nTimeout, sizeof(int)) < 0);
}

#endif
