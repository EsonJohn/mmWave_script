/**
 * @file rf_api_internal.h
 *
 * @author JP
 *
 * @version 0.2
 *
 * @brief This file contains API declarations and macro definitions for  
 * DCA1000EVM system which can be accessible to DLL and CLI application
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


///****************************************************************************
/// HISTORY :
/// VERSION        DATE              AUTHOR      CHANGE DESCRIPTION
/// 0.1            15 Feb 2019       JP          Created
/// 0.2            09 Apr 2019       JP          Removed reordering compiler
///                                              option
///****************************************************************************

#ifndef RF_API_INTERNAL_H
#define RF_API_INTERNAL_H

///****************
/// Includes
///****************

/** Mistral standard data type declarations                                  */
#include "DCA1000_API/rf_api.h"

/** Socket includes for Windows / Linux                                      */
#if defined _WIN32          //(_MSC_VER)
    #include <winsock2.h>
    #include <ws2tcpip.h>
#else                       // GNUC

    #include <stdio.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <string.h>
    #include <stdlib.h>
    #include <sys/stat.h>
    #include  <sys/ipc.h>
    #include  <sys/shm.h>
    #include <unistd.h>

    #define SOCKET_ERROR -1
#endif

/** C++11 thread header include */
#include <thread>

/** String manipulation header  */
#include <string>

/** Math manipulation header    */
#include <math.h>


/** To avoid structure padding                                               */
#pragma pack (1)

///****************
/// Defines
///****************

/** Command packet size                                                      */
#define FIXED_PACKET_SIZE                   8

/** Command packet size excluding footer                                     */
#define PACKET_FIXED_SIZE_EXC_FOOTER        6

/** Command packet footer size                                               */
#define PACKET_FOOTER_DATA_SIZE             2

/** Maximum length of a string                                               */
#define MAX_PARAMS_LEN                      100

/** Maximum file update length                                               */
#define MAX_FILE_UPDATE_LEN                 4000

/** Milli second to micro second conversion                                  */
#define MILLI_TO_MICRO_SEC_CONVERSION       1000

/** Second to milli second conversion                                        */
#define SEC_TO_MILLI_SEC_CONVERSION         1000

/** Data payload per packet from FPGA                                        */
#define PAYLOAD_BYTES_PER_PACKET            1456

/** Size for inline processing buffer                                        */
#define INLINE_BUF_SIZE                     (75 * 1000000) // 75 MB

/** Size for inline processing log buffer                                    */
#define INLINE_LOG_BUF_SIZE                 (12 * 100000) // 1.2 MB

/** Capture thread timeout duration in sec                                   */
#define CAPTURE_TIMEOUT_DURATION_SEC        80

/** Socket timeout duration for record mode in sec                           */
#define SOCKET_THREAD_TIMEOUT_DURATION_SEC  (CAPTURE_TIMEOUT_DURATION_SEC + 10)

/** Enable for CLI testing (in debug mode)                                   */
//#define CLI_TESTING_MODE

/** Enable/Disable inline processing                                         */
//#define POST_PROCESSING

/** Log out of sequence metadata which is to be processed
    [This logging was added before the log buffer implementation] 			 */
//#define LOG_OUT_OF_SEQ_OFFSET

/** Log dropped packets offset                                               */
#define LOG_DROPPED_PKTS_OFFSET

///****************
/// Stucture Declarations
///****************

/** Inline processing log file statistics                                    */
typedef struct
{
    /** Zero filled packet present flag     */
    bool bIsZeroFilledPktPresent;

    /** Log file stats size                 */
    UINT32 u32LogSize;

    /** Valid packet flag                   */
    bool bValidPacket[INLINE_LOG_BUF_SIZE];

    /** Offset of current packet            */
    ULONG64 u64CtPktOffset[INLINE_LOG_BUF_SIZE];

    /** Size of current packet              */
    UINT16 u32CtPktSize[INLINE_LOG_BUF_SIZE];
}strInlineProcLogFileStats;

/** Socket IDs for all data ports                                            */
typedef struct RFDCCARD_SOCKInfo
{
    /** Config socket - To read and write over the config port               */
    SINT32 s32EthConfSock;

    /** Config socket - To write async command over the local config port    */
    SINT32 s32EthConfAsyncSock;

    /** Raw data socket - To read over the data ports                        */
    SINT32 s32DataSock[NUM_DATA_TYPES];

}strRFDCCard_SockInfo;

///*****************
/// API Declarations
///*****************

#ifdef __cplusplus
extern "C" {
#endif

/** @fn void THROW_ERROR_STATUS(const SINT8 *s8Msg)
 * @brief This function is to display the error message in console or file(if debug mode enabled)
 * @param [in] s8Msg [const SINT8 *] - Error message to display
 */
void THROW_ERROR_STATUS(const SINT8 *s8Msg);

#ifdef ENABLE_DEBUG
/** @fn void DEBUG_FILE_WRITE(const SINT8 *s8Msg)
 * @brief This function is to write the error message in a file (if debug mode enabled)
 * @param [in] s8Msg [const SINT8 *]  - Error message to display
 */
void DEBUG_FILE_WRITE(const SINT8 *s8Msg);
#endif

/** @fn void WriteInlineProcSummaryInLogFile()
 * @brief This function is to write the inline processing summary in a log file
 */
void WriteInlineProcSummaryInLogFile();

/** @fn void WriteRecordSettingsInLogFile()
 * @brief This function is to write the record settings in a log file
 */
void WriteRecordSettingsInLogFile();

/** @fn STATUS SendConfigCmdRequest(SINT8 *s8Data, UINT16 u16PacketLen)
 * @brief This function is to send command request through config port <!--
 * --> to RF data capture card
 * @param [in] s8Data [SINT8 *] - Command data buffer
 * @param [in] u16PacketLen [UINT16] - Command data buffer size
 * @return SINT32 value
 */
STATUS SendConfigCmdRequest(SINT8 *s8Data, UINT16 u16PacketLen);

/** @fn STATUS GetConfigCmdResponse(const SINT8 *s8Cmd)
 * @brief This function is to wait for a certain timeout and receive command <!--
 * --> response through config port from RF data capture card till it timeouts
 * @param [in] s8Cmd [SINT8 *] - Command name
 * @return SINT32 value
 */
STATUS GetConfigCmdResponse(const SINT8 *s8Cmd);

/** @fn void startDurationStopModeTimer()
 * @brief This function is to initiate timer to handle duration stop mode <!--
 * --> of data recording
 */
void startDurationStopModeTimer();

/** @fn void handleCaptureThreadTimeout()
 * @brief This function is to handle capture timeout when the system is  <!--
 * --> disconnected during capture process
 */
void handleCaptureThreadTimeout();

/** @fn void UpdateInlineStatus(bool bOutOfSeqFlag, UINT8 u8DataIndex)
 * @brief This function is to update status of data capture based on data index
 * @param [in] bOutOfSeqFlag [bool] Out of sequence enable flag
 * @param [in] u8DataIndex [UINT8]  Data Index
 */
void UpdateInlineStatus(bool bOutOfSeqFlag, UINT8 u8DataIndex);

#ifdef __cplusplus
}
#endif

#endif // RF_API_INTERNAL_H
