/**
 * @file rf_api.cpp
 *
 * @author JP
 *
 * @version 0.5
 *
 * @brief This file contains exported APIs definitions for DCA1000EVM system
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
/// 0.2            27 Dec 2019       JP          Linux support APIs added
/// 0.3            17 Jan 2019       JP          Inline processing added and
///                                              Review comments incorporated
/// 0.4            29 Mar 2019       JP          Stop record timeout logic
///                                              updated and clode cleanup
/// 0.5            09 Apr 2019       JP          Reordering record data as
///                                              user configurable option
///*****************************************************************************

///****************
/// Includes
///****************

#include "../Common/rf_api_internal.h"
#include "defines.h"
#include "commandsprotocol.h"
#include "configdatarecv.h"
#include "recorddatarecv.h"
#include "../Common/Validate_Utils/validate_params.h"
#include "../Common/Osal_Utils/osal.h"

#include <iostream>

///****************
/// Global variables
///****************

/** Socket IDs for config and data ports                */
strRFDCCard_SockInfo	sRFDCCard_SockInfo;

/** Record process - Inline stats                       */
strRFDCCard_InlineProcStats sRFDCCard_InlineStats;

/** Start record config structure                       */
strStartRecConfigMode   sRFDCCard_StartRecConfig;

/** Callback function - To handle Config port async status */
EVENT_HANDLER           RFDCARD_Callback;

/** Callback function - To handle recording inline stats */
INLINE_PROC_HANDLER     RecordInlineProc_Callback;

/** Commands protocol -  class object                   */
cCommandsProtocol       objCmdsProto;

/** ADC data record process -  class object             */
cUdpDataReceiver        objUdpDataRecv(RAW_DATA_INDEX);

/** CP data record process -  class object              */
cUdpDataReceiver		objUdpCpDataRecv(CP_DATA_1_INDEX);

/** CQ data record process -  class object              */
cUdpDataReceiver		objUdpCqDataRecv(CQ_DATA_2_INDEX);

/** R4F data record process -  class object             */
cUdpDataReceiver		objUdpR4fDataRecv(R4F_DATA_3_INDEX);

/** DSP data record process -  class object             */
cUdpDataReceiver		objUdpDspDataRecv(DSP_DATA_4_INDEX);

/** Command response data handling -  class object      */
cUdpReceiver            objUdpConfigRecv;

/** Osal class object                                   */
osal osalObj;

/** Config port - socket address to send config commands    */
struct sockaddr_in      ethConf_PortAddress;

/** Config port - socket address to send config commands    */
struct sockaddr_in      ethConf_ServAddr;

/** Config port - socket address to send stop async command */
struct sockaddr_in      ethConf_PortAsyncAddress;

/** Record - Maximum file size config                       */
UINT32 u32MaxFileSizeToCapture = 0;

/** Record - Lane number                                    */
UINT8 u8LaneNumber = 4;

/** Record stop command sent                            */
bool gbRecStopCmdSent = false;

/** Log msg - char array declaration for writing out of seq metadat */
SINT8 s8MetaDataLogMsg[MAX_FILE_UPDATE_LEN];

/** Fill zero bytes array for dropped packets           */
SINT8 s8ZeroBuf[MAX_BYTES_PER_PACKET];

/** Enable Debug - Macro is used to debug DLL progress when integrated
 * in other application. Provide details about list of commands for
 * the current session till RF_API.dll is unloaded/closed by the calling application
 */
#ifdef ENABLE_DEBUG
/** DLL Debug file pointer                              */
FILE *pDebugFile;

/** Log msg - Char array declaration                    */
SINT8 s8DebugMsg[MAX_NAME_LEN];

/** Debug file name                                     */
SINT8 s8DebugFileName[MAX_NAME_LEN] = "api_debug.txt";
#endif

/** Inline process log file pointer                     */
FILE *pInlineLogFile = NULL;

/** Configuration commands response - Start record command.
 * Response will be updated in this structure by config port response thread
 * and returned to the calling application. The scope is global since it is
 * updated by the other thread function */
DATA_CAPTURE_RESP configResp;

/** Start record command timeout wait event
 * Separate thread will be running for handling config port response packets,
 * hence start record command responses will be handled using the wait event
 */
OSAL_SIGNAL_HANDLE_TYPE sgnCmdTimeoutWaitEvent;

/** Duration stop mode timeout wait event
 * Thread and wait event enabled when duration stop mode config is set for recording
 */
OSAL_SIGNAL_HANDLE_TYPE sgnDurationStopModeWaitEvent;

/** Capture timeout wait event.
 *  To handle system disconnect/idleness while recording
 */
OSAL_SIGNAL_HANDLE_TYPE sgnCaptureTimeoutWaitEvent;

/** @fn STATUS ConnectRFDCCard_ConfigMode (strEthConfigMode	sEthConfigMode)
 * @brief This function is to create a socket communication to DCA1000EVM <!--
 * --> system over the config port with the following configuration @n <!--
 * -->	1.	FPGA IP Address @n  <!--
 * -->	2.	Record port number @n  <!--
 * -->	3.	Configuration port number <!--
 * --> This function is intended for handling config command request <!--
 * --> and responses over the config port. This assumes that another <!--
 * --> process has not called \ref ConnectRFDCCard_RecordMode in the <!--
 * --> background as that process could be holding onto the config port <!--
 * --> for reading async reports from DCA1000EVM system
 * @param [in] sEthConfigMode [strEthConfigMode] - <!--
 * --> Structure filled with Ethernet config data
 * @return SINT32 value
 */
STATUS ConnectRFDCCard_ConfigMode
(
strEthConfigMode sEthConfigMode
)
{
#ifdef ENABLE_DEBUG
    pDebugFile = fopen(s8DebugFileName, "wb");
    if (NULL == pDebugFile)
    {
        RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                            STS_REC_FILE_CREATION_ERR);
        return false;
    }
    fclose(pDebugFile);
    DEBUG_FILE_WRITE("Application started ... \n");

    sprintf(s8DebugMsg, "\nConnectRFDCCard_ConfigMode : ");
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nau8Dca1000IpAddr : %d.%d.%d.%d",
        sEthConfigMode.au8Dca1000IpAddr[0],
        sEthConfigMode.au8Dca1000IpAddr[1],
        sEthConfigMode.au8Dca1000IpAddr[2],
        sEthConfigMode.au8Dca1000IpAddr[3]);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu32ConfigPortNo : %d",
        sEthConfigMode.u32ConfigPortNo);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu32RecordPortNo : %d",
        sEthConfigMode.u32RecordPortNo);
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    SINT32 s32RecvBuf = 0x7FFFFFFF;
    SINT32 s32SendBuf = 0xFFFFF;
    SINT8  s8IpAddr[IP_ADDR_MAX_SIZE_BYTES];
    strEthConfigMode sRFDCCard_EthConfig;

    /** Reset the socket IDs and socket address structure                    */
    sRFDCCard_SockInfo.s32EthConfSock = 0;
    memset((void *)&ethConf_PortAddress, '0', sizeof(ethConf_PortAddress));
    memset((void *)&ethConf_ServAddr, '0', sizeof(ethConf_ServAddr));

#if defined _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != NO_ERROR)
    {
        THROW_ERROR_STATUS("\nServer: WSAStartup failed with error");
        return (STS_RFDCCARD_OS_ERR);
    }
#endif

    /** Validate the input parameters                                        */

    /** Validate Record Port number                                          */
    if (SUCCESS_STATUS != validatePortNumber(sEthConfigMode.u32RecordPortNo))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConnectRFDCCard_ConfigMode(): "
            "Invalid input parameters (u32RecordPortNo) : %d",
            sEthConfigMode.u32RecordPortNo);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConnectRFDCCard_ConfigMode(): "
            "Invalid input parameters (u32RecordPortNo) : %d",
            sEthConfigMode.u32RecordPortNo);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    /** Validate Config Port number                                          */
    if (SUCCESS_STATUS != validatePortNumber(sEthConfigMode.u32ConfigPortNo))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConnectRFDCCard_ConfigMode(): "
            "Invalid input parameters (u32ConfigPortNo) : %d",
            sEthConfigMode.u32ConfigPortNo);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConnectRFDCCard_ConfigMode(): "
            "Invalid input parameters (u32ConfigPortNo) : %d",
            sEthConfigMode.u32ConfigPortNo);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    if(SUCCESS_STATUS != validatePortNumsForConflicts(sEthConfigMode.u32RecordPortNo,
                                                      sEthConfigMode.u32ConfigPortNo))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConnectRFDCCard_ConfigMode(): "
            "Invalid input parameters (Port numbers are same)");
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConnectRFDCCard_ConfigMode(): "
            "Invalid input parameters (Port numbers are same)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    if(SUCCESS_STATUS != validateIPAddr(sEthConfigMode.au8Dca1000IpAddr))
    {
        printf("\nConnectRFDCCard_RecordMode(): "
            "Invalid input parameters (au8Dca1000IpAddr)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    memcpy(sRFDCCard_EthConfig.au8Dca1000IpAddr,
           sEthConfigMode.au8Dca1000IpAddr, 4);
    sRFDCCard_EthConfig.u32ConfigPortNo = sEthConfigMode.u32ConfigPortNo;
    sRFDCCard_EthConfig.u32RecordPortNo = sEthConfigMode.u32RecordPortNo;

    sprintf(s8IpAddr, "%d.%d.%d.%d", sEthConfigMode.au8Dca1000IpAddr[0],
        sEthConfigMode.au8Dca1000IpAddr[1], sEthConfigMode.au8Dca1000IpAddr[2],
        sEthConfigMode.au8Dca1000IpAddr[3]);

    /** Create socket for config port */
    sRFDCCard_SockInfo.s32EthConfSock = socket(AF_INET, SOCK_DGRAM,
        IPPROTO_UDP);
    if (sRFDCCard_SockInfo.s32EthConfSock < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_ConfigMode(): "
            "Socket creation error (Config port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Prepare the sockaddr_in structure                                    */
    ethConf_ServAddr.sin_family = AF_INET;
    ethConf_ServAddr.sin_addr.s_addr = inet_addr(s8IpAddr);
    ethConf_ServAddr.sin_port = htons(sRFDCCard_EthConfig.u32ConfigPortNo);

    /** Prepare the sockaddr_in structure                                    */
    ethConf_PortAddress.sin_family = AF_INET;
    ethConf_PortAddress.sin_addr.s_addr = inet_addr("0.0.0.0");
    ethConf_PortAddress.sin_port = htons(sRFDCCard_EthConfig.u32ConfigPortNo);

    /** Bind                                                                 */
    if (bind(sRFDCCard_SockInfo.s32EthConfSock,
        (struct sockaddr *)&ethConf_PortAddress,
        sizeof(ethConf_PortAddress)) < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_ConfigMode(): "
            "Bind failed (Config port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32EthConfSock, SOL_SOCKET, SO_RCVBUF,
        (char*)&s32RecvBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_ConfigMode(): "
            "setsockopt failed (Config port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32EthConfSock, SOL_SOCKET, SO_SNDBUF,
        (char*)&s32SendBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_ConfigMode(): "
            "setsockopt failed (Config port)");
        return (STS_RFDCCARD_OS_ERR);
    }


    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS ConnectRFDCCard_AsyncCommandMode (strEthConfigMode sEthConfigMode)
 * @brief This function is to create a socket communication to the local <!--
 * --> system over the config port with the following configuration @n <!--
 * -->	1.	FPGA IP Address @n  <!--
 * -->	2.	Record port number @n  <!--
 * -->	3.	Configuration port number <!--
 * --> This function is intended for sending async stop message to <!--
 * --> interrupt the ongoing record process (which was started using <!--
 * --> \ref ConnectRFDCCard_RecordMode). The message is sent over the <!--
 * --> local config port since the record process would already be <!--
 * --> listening to DCA1000EVM async messages over that port
 * @param [in] sEthConfigMode [strEthConfigMode] - <!--
 * --> Structure filled with Ethernet config data
 * @return SINT32 value
 */
STATUS ConnectRFDCCard_AsyncCommandMode
(
strEthConfigMode sEthConfigMode
)
{
#ifdef ENABLE_DEBUG
    pDebugFile = fopen(s8DebugFileName, "wb");
    if (NULL == pDebugFile)
    {
        RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                            STS_REC_FILE_CREATION_ERR);
        return false;
    }
    fclose(pDebugFile);
    DEBUG_FILE_WRITE("Application started ... \n");

    sprintf(s8DebugMsg, "\nConnectRFDCCard_AsyncCommandMode : ");
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nau8Dca1000IpAddr : %d.%d.%d.%d",
        sEthConfigMode.au8Dca1000IpAddr[0],
        sEthConfigMode.au8Dca1000IpAddr[1],
        sEthConfigMode.au8Dca1000IpAddr[2],
        sEthConfigMode.au8Dca1000IpAddr[3]);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu32ConfigPortNo : %d",
        sEthConfigMode.u32ConfigPortNo);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu32RecordPortNo : %d",
        sEthConfigMode.u32RecordPortNo);
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    SINT32 s32RecvBuf = SOCK_RECV_BUF_SIZE;
    SINT32 s32SendBuf = SOCK_SEND_BUF_SIZE;
    SINT8  s8IpAddr[IP_ADDR_MAX_SIZE_BYTES];
    strEthConfigMode sRFDCCard_EthConfig;

    /** Reset the socket IDs and socket address structure                    */
    sRFDCCard_SockInfo.s32EthConfAsyncSock = 0;
    memset((void *)&ethConf_PortAsyncAddress, '0',
            sizeof(ethConf_PortAsyncAddress));

#if defined _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != NO_ERROR)
    {
        THROW_ERROR_STATUS("\nServer: WSAStartup failed with error");
        return (STS_RFDCCARD_OS_ERR);
    }
#endif

    /** Validate the input parameters                                        */

    /** Validate Record Port number                                          */
    if (SUCCESS_STATUS != validatePortNumber(sEthConfigMode.u32RecordPortNo))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConnectRFDCCard_AsyncCommandMode(): "
            "Invalid input parameters (u32RecordPortNo) : %d",
            sEthConfigMode.u32RecordPortNo);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConnectRFDCCard_AsyncCommandMode(): "
            "Invalid input parameters (u32RecordPortNo) : %d",
            sEthConfigMode.u32RecordPortNo);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    /** Validate Config Port number                                          */
    if (SUCCESS_STATUS != validatePortNumber(sEthConfigMode.u32ConfigPortNo))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConnectRFDCCard_AsyncCommandMode(): "
            "Invalid input parameters (u32ConfigPortNo) : %d",
            sEthConfigMode.u32ConfigPortNo);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConnectRFDCCard_AsyncCommandMode(): "
            "Invalid input parameters (u32ConfigPortNo) : %d",
            sEthConfigMode.u32ConfigPortNo);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    if(SUCCESS_STATUS != validatePortNumsForConflicts(sEthConfigMode.u32RecordPortNo,
                                                      sEthConfigMode.u32ConfigPortNo))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConnectRFDCCard_AsyncCommandMode(): "
            "Invalid input parameters (Port numbers are same)");
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConnectRFDCCard_AsyncCommandMode(): "
            "Invalid input parameters (Port numbers are same)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }
    if(SUCCESS_STATUS != validateIPAddr(sEthConfigMode.au8Dca1000IpAddr))
    {
        printf("\nConnectRFDCCard_AsyncCommandMode(): "
            "Invalid input parameters (au8Dca1000IpAddr)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    memcpy(sRFDCCard_EthConfig.au8Dca1000IpAddr, sEthConfigMode.au8Dca1000IpAddr,
        4);
    sRFDCCard_EthConfig.u32ConfigPortNo = sEthConfigMode.u32ConfigPortNo;
    sRFDCCard_EthConfig.u32RecordPortNo = sEthConfigMode.u32RecordPortNo;

    sprintf(s8IpAddr, "%d.%d.%d.%d", sEthConfigMode.au8Dca1000IpAddr[0],
        sEthConfigMode.au8Dca1000IpAddr[1], sEthConfigMode.au8Dca1000IpAddr[2],
        sEthConfigMode.au8Dca1000IpAddr[3]);

    /** Create socket for config port */
    sRFDCCard_SockInfo.s32EthConfAsyncSock = socket(AF_INET, SOCK_DGRAM,
        IPPROTO_UDP);
    if (sRFDCCard_SockInfo.s32EthConfAsyncSock < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_AsyncCommandMode(): "
            "Socket creation error (Config port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    ethConf_PortAsyncAddress.sin_family = AF_INET;
    ethConf_PortAsyncAddress.sin_port = htons(sRFDCCard_EthConfig.u32ConfigPortNo);
    struct hostent *pHost = gethostbyname("localhost");
       memcpy(&ethConf_PortAsyncAddress.sin_addr.s_addr, pHost->h_addr,
              pHost->h_length);

    if (setsockopt(sRFDCCard_SockInfo.s32EthConfAsyncSock, SOL_SOCKET, SO_RCVBUF,
        (char*)&s32RecvBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_AsyncCommandMode(): "
            "setsockopt failed (Config port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32EthConfAsyncSock, SOL_SOCKET, SO_SNDBUF,
        (char*)&s32SendBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_AsyncCommandMode(): "
            "setsockopt failed (Config port)");
        return (STS_RFDCCARD_OS_ERR);
    }


    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS ConnectRFDCCard_RecordMode (strEthConfigMode sEthConfigMode)
 * @brief This function is to create a socket communication to DCA1000EVM <!--
 * --> system over the config and data ports with the following <!--
 * --> configuration @n <!--
 * -->	1.	FPGA IP Address @n  <!--
 * -->	2.	Record port number @n  <!--
 * -->	3.	Configuration port number <!--
 * --> This function is intended for handling config and record command <!--
 * --> request and responses over the config port and recording of data <!--
 * --> over the data ports. Interactive/UI Application can use this API <!--
 * --> to execute config and recording commands. Once a process has <!--
 * --> initiated recording using this API, another process can communicate <!--
 * --> with this process using \ref ConnectRFDCCard_AsyncCommandMode
 * @param [in] sEthConfigMode [strEthConfigMode] - <!--
 * --> Structure filled with Ethernet config data
 * @return SINT32 value
 */
STATUS ConnectRFDCCard_RecordMode
(
strEthConfigMode sEthConfigMode
)
{
#ifdef ENABLE_DEBUG
    pDebugFile = fopen(s8DebugFileName, "wb");
    if (NULL == pDebugFile)
    {
        RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                            STS_REC_FILE_CREATION_ERR);
        return false;
    }
    fclose(pDebugFile);
    DEBUG_FILE_WRITE("Application started ... \n");

    sprintf(s8DebugMsg, "\nConnectRFDCCard_RecordMode : ");
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nau8Dca1000IpAddr : %d.%d.%d.%d",
        sEthConfigMode.au8Dca1000IpAddr[0],
        sEthConfigMode.au8Dca1000IpAddr[1],
        sEthConfigMode.au8Dca1000IpAddr[2],
        sEthConfigMode.au8Dca1000IpAddr[3]);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu32ConfigPortNo : %d",
        sEthConfigMode.u32ConfigPortNo);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu32RecordPortNo : %d",
        sEthConfigMode.u32RecordPortNo);
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    SINT32 s32RecvBuf = SOCK_RECV_BUF_SIZE;
    SINT32 s32SendBuf = SOCK_SEND_BUF_SIZE;
    strEthConfigMode sRFDCCard_EthConfig;

    struct sockaddr_in      ethRaw_ServAddr;
    struct sockaddr_in      ethDT1_ServAddr;
    struct sockaddr_in      ethDT2_ServAddr;
    struct sockaddr_in      ethDT3_ServAddr;
    struct sockaddr_in      ethDT4_ServAddr;
    SINT8  s8IpAddr[IP_ADDR_MAX_SIZE_BYTES];

    /** Reset the socket IDs and socket address structure                    */
    sRFDCCard_SockInfo.s32EthConfSock = 0;
    sRFDCCard_SockInfo.s32DataSock[RAW_DATA_INDEX] = 0;
    sRFDCCard_SockInfo.s32DataSock[CP_DATA_1_INDEX] = 0;
    sRFDCCard_SockInfo.s32DataSock[CQ_DATA_2_INDEX] = 0;
    sRFDCCard_SockInfo.s32DataSock[R4F_DATA_3_INDEX] = 0;
    sRFDCCard_SockInfo.s32DataSock[DSP_DATA_4_INDEX] = 0;
    memset((void *)&ethConf_PortAddress, '0', sizeof(ethConf_PortAddress));
    memset((void *)&ethConf_ServAddr, '0', sizeof(ethConf_ServAddr));
    memset((void *)&ethRaw_ServAddr, '0', sizeof(ethRaw_ServAddr));
    memset((void *)&ethDT1_ServAddr, '0', sizeof(ethDT1_ServAddr));
    memset((void *)&ethDT2_ServAddr, '0', sizeof(ethDT2_ServAddr));
    memset((void *)&ethDT3_ServAddr, '0', sizeof(ethDT3_ServAddr));
    memset((void *)&ethDT4_ServAddr, '0', sizeof(ethDT4_ServAddr));

    /** Initialize waiting events             */
    osalObj.InitEvent(&sgnCmdTimeoutWaitEvent);
    osalObj.InitEvent(&sgnDurationStopModeWaitEvent);
    osalObj.InitEvent(&sgnCaptureTimeoutWaitEvent);

#if defined _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != NO_ERROR)
    {
        THROW_ERROR_STATUS("\nServer: WSAStartup failed with error");
        return (STS_RFDCCARD_OS_ERR);
    }
#endif

    /** Validate the input parameters                                        */

    /** Validate Record Port number                                          */
    if (SUCCESS_STATUS != validatePortNumber(sEthConfigMode.u32RecordPortNo))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConnectRFDCCard_RecordMode(): "
            "Invalid input parameters (u32RecordPortNo) : %d",
            sEthConfigMode.u32RecordPortNo);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConnectRFDCCard_RecordMode(): "
            "Invalid input parameters (u32RecordPortNo) : %d",
            sEthConfigMode.u32RecordPortNo);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    /** Validate Config Port number                                          */
    if (SUCCESS_STATUS != validatePortNumber(sEthConfigMode.u32ConfigPortNo))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConnectRFDCCard_RecordMode(): "
            "Invalid input parameters (u32ConfigPortNo) : %d",
            sEthConfigMode.u32ConfigPortNo);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConnectRFDCCard_RecordMode(): "
            "Invalid input parameters (u32ConfigPortNo) : %d",
            sEthConfigMode.u32ConfigPortNo);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    if(SUCCESS_STATUS != validatePortNumsForConflicts(sEthConfigMode.u32RecordPortNo,
                                                      sEthConfigMode.u32ConfigPortNo))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConnectRFDCCard_RecordMode(): "
            "Invalid input parameters (Port numbers are same)");
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConnectRFDCCard_RecordMode(): "
            "Invalid input parameters (Port numbers are same)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    if(SUCCESS_STATUS != validateIPAddr(sEthConfigMode.au8Dca1000IpAddr))
    {
        printf("\nConnectRFDCCard_RecordMode(): "
            "Invalid input parameters (au8Dca1000IpAddr)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    memcpy(sRFDCCard_EthConfig.au8Dca1000IpAddr,
           sEthConfigMode.au8Dca1000IpAddr, 4);
    sRFDCCard_EthConfig.u32ConfigPortNo = sEthConfigMode.u32ConfigPortNo;
    sRFDCCard_EthConfig.u32RecordPortNo = sEthConfigMode.u32RecordPortNo;

    sprintf(s8IpAddr, "%d.%d.%d.%d", sEthConfigMode.au8Dca1000IpAddr[0],
        sEthConfigMode.au8Dca1000IpAddr[1], sEthConfigMode.au8Dca1000IpAddr[2],
        sEthConfigMode.au8Dca1000IpAddr[3]);

    /** Create socket for raw port                                           */
    sRFDCCard_SockInfo.s32DataSock[RAW_DATA_INDEX] = socket(AF_INET, SOCK_DGRAM,
        IPPROTO_UDP);
    if (sRFDCCard_SockInfo.s32DataSock[RAW_DATA_INDEX] < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode():  "
            "Socket creation error (ADC data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Prepare the sockaddr_in structure                                    */
    ethRaw_ServAddr.sin_family = AF_INET;
    ethRaw_ServAddr.sin_addr.s_addr = inet_addr("0.0.0.0");
    ethRaw_ServAddr.sin_port = htons(sRFDCCard_EthConfig.u32RecordPortNo);

    /** Setting the timeout for ADC data port */
    if(osalObj.sock_setopt(sRFDCCard_SockInfo.s32DataSock[RAW_DATA_INDEX],
                           SOCKET_THREAD_TIMEOUT_DURATION_SEC) < 0)
    {
        THROW_ERROR_STATUS("\n\nConnectRFDCCard_RecordMode(): "
                           "setsockopt timeout failed (ADC data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Bind                                                                 */
    if (bind(sRFDCCard_SockInfo.s32DataSock[RAW_DATA_INDEX],
        (struct sockaddr *)&ethRaw_ServAddr,
        sizeof(ethRaw_ServAddr)) < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "Bind failed (ADC data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32DataSock[RAW_DATA_INDEX], SOL_SOCKET, SO_RCVBUF,
        (char*)&s32RecvBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "setsockopt failed (ADC data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32DataSock[RAW_DATA_INDEX], SOL_SOCKET, SO_SNDBUF,
        (char*)&s32SendBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "setsockopt failed (ADC data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Create socket for Data type 1 (CP)                                   */
    sRFDCCard_SockInfo.s32DataSock[CP_DATA_1_INDEX] = socket(AF_INET, SOCK_DGRAM,
        IPPROTO_UDP);
    if (sRFDCCard_SockInfo.s32DataSock[CP_DATA_1_INDEX] < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "Socket creation error (CP Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Prepare the sockaddr_in structure                                    */
    ethDT1_ServAddr.sin_family = AF_INET;
    ethDT1_ServAddr.sin_addr.s_addr = inet_addr("0.0.0.0");
    ethDT1_ServAddr.sin_port = htons(sEthConfigMode.u32RecordPortNo + 1);

    /** Setting the timeout for CP data port */
    if(osalObj.sock_setopt(sRFDCCard_SockInfo.s32DataSock[CP_DATA_1_INDEX],
                           SOCKET_THREAD_TIMEOUT_DURATION_SEC) < 0)
    {
        THROW_ERROR_STATUS("\n\nConnectRFDCCard_RecordMode(): "
                           "setsockopt timeout failed (CP data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Bind                                                                 */
    if (bind(sRFDCCard_SockInfo.s32DataSock[CP_DATA_1_INDEX],
        (struct sockaddr *)&ethDT1_ServAddr,
        sizeof(ethDT1_ServAddr)) < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "Bind failed (CP Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32DataSock[CP_DATA_1_INDEX], SOL_SOCKET, SO_RCVBUF,
        (char*)&s32RecvBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "setsockopt failed (CP Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32DataSock[CP_DATA_1_INDEX], SOL_SOCKET, SO_SNDBUF,
        (char*)&s32SendBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "setsockopt failed (CP Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Create socket for Data type 2 (CQ)                                   */
    sRFDCCard_SockInfo.s32DataSock[CQ_DATA_2_INDEX] = socket(AF_INET, SOCK_DGRAM,
        IPPROTO_UDP);
    if (sRFDCCard_SockInfo.s32DataSock[CQ_DATA_2_INDEX] < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "Socket creation error (CQ Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Prepare the sockaddr_in structure                                    */
    ethDT2_ServAddr.sin_family = AF_INET;
    ethDT2_ServAddr.sin_addr.s_addr = inet_addr("0.0.0.0");
    ethDT2_ServAddr.sin_port = htons(sEthConfigMode.u32RecordPortNo + 2);

    /** Setting the timeout for CQ data port */
    if(osalObj.sock_setopt(sRFDCCard_SockInfo.s32DataSock[CQ_DATA_2_INDEX],
                           SOCKET_THREAD_TIMEOUT_DURATION_SEC) < 0)
    {
        THROW_ERROR_STATUS("\n\nConnectRFDCCard_RecordMode(): "
                           "setsockopt timeout failed (CQ data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Bind                                                                 */
    if (bind(sRFDCCard_SockInfo.s32DataSock[CQ_DATA_2_INDEX],
        (struct sockaddr *)&ethDT2_ServAddr,
        sizeof(ethDT2_ServAddr)) < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "Bind failed (CQ Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32DataSock[CQ_DATA_2_INDEX], SOL_SOCKET, SO_RCVBUF,
        (char*)&s32RecvBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "setsockopt failed (CQ Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32DataSock[CQ_DATA_2_INDEX], SOL_SOCKET, SO_SNDBUF,
        (char*)&s32SendBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "setsockopt failed (CQ Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Create socket for Data type 3 (R4F) */
    sRFDCCard_SockInfo.s32DataSock[R4F_DATA_3_INDEX] = socket(AF_INET, SOCK_DGRAM,
        IPPROTO_UDP);
    if (sRFDCCard_SockInfo.s32DataSock[R4F_DATA_3_INDEX] < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "Socket creation error (R4F Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Prepare the sockaddr_in structure                                    */
    ethDT3_ServAddr.sin_family = AF_INET;
    ethDT3_ServAddr.sin_addr.s_addr = inet_addr("0.0.0.0");
    ethDT3_ServAddr.sin_port = htons(sEthConfigMode.u32RecordPortNo + 3);


    /** Setting the timeout for R4F data port */
    if(osalObj.sock_setopt(sRFDCCard_SockInfo.s32DataSock[R4F_DATA_3_INDEX],
                           SOCKET_THREAD_TIMEOUT_DURATION_SEC) < 0)
    {
        THROW_ERROR_STATUS("\n\nConnectRFDCCard_RecordMode(): "
                           "setsockopt timeout failed (R4F data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Bind                                                                 */
    if (bind(sRFDCCard_SockInfo.s32DataSock[R4F_DATA_3_INDEX],
        (struct sockaddr *)&ethDT3_ServAddr,
        sizeof(ethDT3_ServAddr)) < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "Bind failed (R4F Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32DataSock[R4F_DATA_3_INDEX], SOL_SOCKET, SO_RCVBUF,
        (char*)&s32RecvBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "setsockopt failed (R4F Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32DataSock[R4F_DATA_3_INDEX], SOL_SOCKET, SO_SNDBUF,
        (char*)&s32SendBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "setsockopt failed (R4F Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Create socket for Data type 4 (DSP) */
    sRFDCCard_SockInfo.s32DataSock[DSP_DATA_4_INDEX] = socket(AF_INET, SOCK_DGRAM,
        IPPROTO_UDP);
    if (sRFDCCard_SockInfo.s32DataSock[DSP_DATA_4_INDEX] < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "Socket creation error (DSP Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Prepare the sockaddr_in structure*/
    ethDT4_ServAddr.sin_family = AF_INET;
    ethDT4_ServAddr.sin_addr.s_addr = inet_addr("0.0.0.0");
    ethDT4_ServAddr.sin_port = htons(sEthConfigMode.u32RecordPortNo + 4);


    /** Setting the timeout for DSP data port */
    if(osalObj.sock_setopt(sRFDCCard_SockInfo.s32DataSock[DSP_DATA_4_INDEX],
                           SOCKET_THREAD_TIMEOUT_DURATION_SEC) < 0)
    {
        THROW_ERROR_STATUS("\n\nConnectRFDCCard_RecordMode(): "
                           "setsockopt timeout failed (DSP data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Bind                                                                  */
    if (bind(sRFDCCard_SockInfo.s32DataSock[DSP_DATA_4_INDEX],
        (struct sockaddr *)&ethDT4_ServAddr,
        sizeof(ethDT4_ServAddr)) < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "Bind failed (DSP Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32DataSock[DSP_DATA_4_INDEX], SOL_SOCKET, SO_RCVBUF,
        (char*)&s32RecvBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "setsockopt failed (DSP Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32DataSock[DSP_DATA_4_INDEX], SOL_SOCKET, SO_SNDBUF,
        (char*)&s32SendBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "setsockopt failed (DSP Data port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Create socket for config port */
    sRFDCCard_SockInfo.s32EthConfSock = socket(AF_INET, SOCK_DGRAM,
        IPPROTO_UDP);
    if (sRFDCCard_SockInfo.s32EthConfSock < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "Socket creation error (Config port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Prepare the sockaddr_in structure                                    */
    ethConf_ServAddr.sin_family = AF_INET;
    ethConf_ServAddr.sin_addr.s_addr = inet_addr(s8IpAddr);
    ethConf_ServAddr.sin_port = htons(sEthConfigMode.u32ConfigPortNo);

    /** Prepare the sockaddr_in structure                                    */
    ethConf_PortAddress.sin_family = AF_INET;
    ethConf_PortAddress.sin_addr.s_addr = inet_addr("0.0.0.0");
    ethConf_PortAddress.sin_port = htons(sRFDCCard_EthConfig.u32ConfigPortNo);


    /** Setting the timeout for config port response */
    if(osalObj.sock_setopt(sRFDCCard_SockInfo.s32EthConfSock,
                           SOCKET_THREAD_TIMEOUT_DURATION_SEC) < 0)
    {
        THROW_ERROR_STATUS("\n\nConnectRFDCCard_RecordMode(): "
                           "setsockopt timeout failed (config port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    /** Bind                                                                 */
    if (bind(sRFDCCard_SockInfo.s32EthConfSock,
        (struct sockaddr *)&ethConf_PortAddress,
        sizeof(ethConf_PortAddress)) < 0)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "Bind failed (Config port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32DataSock[DSP_DATA_4_INDEX], SOL_SOCKET, SO_RCVBUF,
        (char*)&s32RecvBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "setsockopt failed (Config port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    if (setsockopt(sRFDCCard_SockInfo.s32DataSock[DSP_DATA_4_INDEX], SOL_SOCKET, SO_SNDBUF,
        (char*)&s32SendBuf, sizeof(SINT32)) == -1)
    {
        THROW_ERROR_STATUS("\nConnectRFDCCard_RecordMode(): "
            "setsockopt failed (Config port)");
        return (STS_RFDCCARD_OS_ERR);
    }

    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS ConfigureRFDCCard_Fpga(strFpgaConfigMode sConfigMode)
 * @brief This function is to configure the DCA1000EVM system <!--
 * --> with the following mode configuration @n <!--
 * -->	1.	Logging Mode 		– RAW/MULTI @n <!--
 * -->	2.	LVDS Mode    		- FOUR/TWO LANE @n <!--
 * -->	3.	Data Transfer Mode 	– LVDS Capture/Playback @n <!--
 * -->	4.	Data Capture Mode 	– Ethernet Streaming/SD Card Storage @n  <!--
 * -->	5.	Data Format Mode  	- 12/14/16 bit @n <!--
 * -->  6.  Timer               - Timeout for LVDS data
 * @param [in] sConfigMode [strConfigMode] - Structure filled with config data
 * @pre Application should be connected to DCA1000EVM system
 * @return SINT32 value
 */
STATUS ConfigureRFDCCard_Fpga
(
strFpgaConfigMode      sConfigMode
)
{
#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nConfigureRFDCCard_Mode : ");
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\neLogMode : %d", sConfigMode.eLogMode);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\neLvdsMode : %d", sConfigMode.eLvdsMode);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\neDataXferMode : %d", sConfigMode.eDataXferMode);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\neDataCaptureMode : %d",
        sConfigMode.eDataCaptureMode);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\neDataFormatMode : %d",
        sConfigMode.eDataFormatMode);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu8Timer : %d", sConfigMode.u8Timer);
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    /** Validate the Logging Mode                                            */
    if (!((RAW_MODE == sConfigMode.eLogMode) ||
        (MULTI_MODE == sConfigMode.eLogMode)))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConfigureRFDCCard_Mode(): "
            "Invalid input parameters (eLogMode) : %d",
            sConfigMode.eLogMode);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConfigureRFDCCard_Mode(): "
            "Invalid input parameters (eLogMode) : %d",
            sConfigMode.eLogMode);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    /** Validate the LVDS Mode                                               */
    if (!((FOUR_LANE != sConfigMode.eLvdsMode) ||
        (TWO_LANE != sConfigMode.eLvdsMode)))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConfigureRFDCCard_Mode(): "
            "Invalid input parameters (eLvdsMode) : %d",
            sConfigMode.eLvdsMode);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConfigureRFDCCard_Mode(): "
            "Invalid input parameters (eLvdsMode) : %d",
            sConfigMode.eLvdsMode);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    /** Validate the Data Transfer Mode                                      */
    if (!((CAPTURE != sConfigMode.eDataXferMode) ||
        (PLAYBACK != sConfigMode.eDataXferMode)))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConfigureRFDCCard_Mode(): "
            "Invalid input parameters (eDataXferMode) : %d",
            sConfigMode.eDataXferMode);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConfigureRFDCCard_Mode(): "
            "Invalid input parameters (eDataXferMode) : %d",
            sConfigMode.eDataXferMode);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    /** Validate the Data Capture Mode                                       */
    if (!((SD_STORAGE != sConfigMode.eDataCaptureMode) ||
        (ETH_STREAM != sConfigMode.eDataCaptureMode)))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConfigureRFDCCard_Mode(): "
            "Invalid input parameters (eDataCaptureMode) : %d",
            sConfigMode.eDataCaptureMode);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConfigureRFDCCard_Mode(): "
            "Invalid input parameters (eDataCaptureMode) : %d",
            sConfigMode.eDataCaptureMode);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    /* Validate the Data Format Mode                                        */
    if (!((BIT12 != sConfigMode.eDataFormatMode) ||
        (BIT14 != sConfigMode.eDataFormatMode) ||
        (BIT16 != sConfigMode.eDataFormatMode)))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConfigureRFDCCard_Mode(): "
            "Invalid input parameters (eDataFormatMode) : %d",
            sConfigMode.eDataFormatMode);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConfigureRFDCCard_Mode(): "
            "Invalid input parameters (eDataFormatMode) : %d",
            sConfigMode.eDataFormatMode);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    /** Copy the configuration mode info to Command packet                   */
    UINT16 u16DataSize = 0;
    UINT8 u8PacketData[MAX_DATA_BYTES];

    u8PacketData[u16DataSize++] = sConfigMode.eLogMode;
    u8PacketData[u16DataSize++] = sConfigMode.eLvdsMode;
    u8PacketData[u16DataSize++] = sConfigMode.eDataXferMode;
    u8PacketData[u16DataSize++] = sConfigMode.eDataCaptureMode;
    u8PacketData[u16DataSize++] = sConfigMode.eDataFormatMode;
    u8PacketData[u16DataSize++] = sConfigMode.u8Timer;

    SINT8 s8Data[sizeof(DATA_CAPTURE_REQ)];
    UINT16 u16PacketLen = objCmdsProto.configFpgaCommand(
        s8Data, u8PacketData, u16DataSize);

    /** Send the command packet to FPGA                                      */
    SINT32 s32BytesSent = SendConfigCmdRequest(s8Data, u16PacketLen);

    if(s32BytesSent <= SOCKET_ERROR)
    {
        THROW_ERROR_STATUS("\n\nConfigureRFDCCard_Mode(): UDP write failed");
        return (STS_RFDCCARD_UDP_WRITE_ERR);
    }
    else
    {
        /** Receive the command response from FPGA                           */
        return GetConfigCmdResponse("ConfigureRFDCCard_Fpga");
    }

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nConfigureRFDCCard_Mode: Sent successfully");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS SendConfigCmdRequest(SINT8 *s8Data, UINT16 u16PacketLen)
 * @brief This function is to send command request through config port to RF  <!--
 * --> data capture card
 * @param [in] s8Data [SINT8 *] - Command data buffer
 * @param [in] u16PacketLen [UINT16] - Command data buffer size
 * @return SINT32 value
 */
STATUS SendConfigCmdRequest(SINT8 *s8Data, UINT16 u16PacketLen)
{
    return (sendto(sRFDCCard_SockInfo.s32EthConfSock, s8Data, u16PacketLen,
        0, (struct sockaddr *)&ethConf_ServAddr, sizeof(ethConf_ServAddr)));
}

/** @fn STATUS GetConfigCmdResponse(const SINT8 *s8Cmd)
 * @brief This function is to wait for a certain timeout and receive command <!--
 * --> response through config port from RF data capture card till it timeouts
 * @param [in] s8Cmd [SINT8 *] - Command name
 * @return SINT32 value
 */
STATUS GetConfigCmdResponse(const SINT8 *s8Cmd)
{
    SINT8 s8Msg[MAX_NAME_LEN];
    DATA_CAPTURE_RESP strConfigResp;

    /** Setting the timeout for command response */
    if(osalObj.sock_setopt(sRFDCCard_SockInfo.s32EthConfSock,
                           CMD_TIMEOUT_DURATION_SEC) < 0)
    {
        THROW_ERROR_STATUS("\n\nConfigureRFDCCard_Mode(): UDP write failed");
        return (STS_RFDCCARD_OS_ERR);
    }

    struct sockaddr_in config_SenderAddr;
    socklen_t s32Config_SenderAddrSize = sizeof(config_SenderAddr);

    SINT32 s32BytesRecvd = recvfrom(sRFDCCard_SockInfo.s32EthConfSock,
                               (SINT8 *)&strConfigResp,
                               sizeof(DATA_CAPTURE_RESP), 0,
                               (struct sockaddr *)&config_SenderAddr,
                               &s32Config_SenderAddrSize);

    if(s32BytesRecvd <= SOCKET_ERROR)
    {
        if(osalObj.IsCmdTimeout())
        {
            return STS_RFDCCARD_TIMEOUT_ERR;
        }
        else
        {
            sprintf(s8Msg, "\n\n%s(): UDP recvfrom failed", s8Cmd);
            THROW_ERROR_STATUS(s8Msg);
            return STS_RFDCCARD_OS_ERR;
        }
    }
    else
    {
        return strConfigResp.u16Status;
    }
}

/** @fn STATUS ConfigureRFDCCard_Eeprom (strEthConfigMode sEthConfigMode)
 * @brief This function is to configure the EEPROM of DCA1000EVM  <!--
 * --> system with the following configuration @n <!--
 * -->	1.	FPGA MAC ID @n   <!--
 * -->	2.	PC and FPGA IP Address @n  <!--
 * -->	3.	Record port number @n  <!--
 * -->	4.	Configuration port number
 * @param [in] sEthConfigMode [strEthConfigMode] - <!--
 * --> Structure filled with config data
 * @pre Application should be connected to DCA1000EVM system
 * @return SINT32 value
 */
STATUS ConfigureRFDCCard_Eeprom
(
strEthConfigMode      sEthConfigMode
)
{
#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nConfigureRFDCCard_EEPROM : ");
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nau8PcIpAddr : %d.%d.%d.%d",
        sEthConfigMode.au8PcIpAddr[0],
        sEthConfigMode.au8PcIpAddr[1],
        sEthConfigMode.au8PcIpAddr[2],
        sEthConfigMode.au8PcIpAddr[3]);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nau8Dca1000IpAddr : %d.%d.%d.%d",
        sEthConfigMode.au8Dca1000IpAddr[0],
        sEthConfigMode.au8Dca1000IpAddr[1],
        sEthConfigMode.au8Dca1000IpAddr[2],
        sEthConfigMode.au8Dca1000IpAddr[3]);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nau8MacId : %d-%d-%d-%d-%d-%d",
        sEthConfigMode.au8MacId[0],
        sEthConfigMode.au8MacId[1],
        sEthConfigMode.au8MacId[2],
        sEthConfigMode.au8MacId[3],
        sEthConfigMode.au8MacId[4],
        sEthConfigMode.au8MacId[5]);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu32ConfigPortNo : %d",
        sEthConfigMode.u32ConfigPortNo);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu32RecordPortNo : %d",
        sEthConfigMode.u32RecordPortNo);
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    /** Validate Record Port number                                          */
    if (SUCCESS_STATUS != validatePortNumber(sEthConfigMode.u32RecordPortNo))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConfigureRFDCCard_EEPROM(): "
            "Invalid input parameters (u32RecordPortNo) : %d",
            sEthConfigMode.u32RecordPortNo);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConfigureRFDCCard_EEPROM(): "
            "Invalid input parameters (u32RecordPortNo) : %d",
            sEthConfigMode.u32RecordPortNo);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    /** Validate Config Port number                                          */
    if (SUCCESS_STATUS != validatePortNumber(sEthConfigMode.u32ConfigPortNo))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConfigureRFDCCard_EEPROM(): "
            "Invalid input parameters (u32ConfigPortNo) : %d",
            sEthConfigMode.u32ConfigPortNo);
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConfigureRFDCCard_EEPROM(): "
            "Invalid input parameters (u32ConfigPortNo) : %d",
            sEthConfigMode.u32ConfigPortNo);
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    if(SUCCESS_STATUS != validatePortNumsForConflicts(sEthConfigMode.u32RecordPortNo,
                                                      sEthConfigMode.u32ConfigPortNo))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nConfigureRFDCCard_EEPROM(): "
            "Invalid input parameters (Port numbers are same)");
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nConfigureRFDCCard_EEPROM(): "
            "Invalid input parameters (Port numbers are same)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    /** Copy the configuration mode info to Command packet                   */
    SINT8 s8Data[sizeof(DATA_CAPTURE_REQ)];
    UINT8 u8PacketData[MAX_DATA_BYTES];
    UINT16 u16DataSize = 0;

    if(SUCCESS_STATUS != validateIPAddr(sEthConfigMode.au8Dca1000IpAddr))
    {
        printf("\nConfigureRFDCCard_Eeprom(): "
            "Invalid input parameters (au8Dca1000IpAddr)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    if(SUCCESS_STATUS != validateIPAddr(sEthConfigMode.au8PcIpAddr))
    {
        printf("\nConfigureRFDCCard_Eeprom(): "
            "Invalid input parameters (au8PcIpAddr)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    u8PacketData[u16DataSize++] = sEthConfigMode.au8PcIpAddr[3];
    u8PacketData[u16DataSize++] = sEthConfigMode.au8PcIpAddr[2];
    u8PacketData[u16DataSize++] = sEthConfigMode.au8PcIpAddr[1];
    u8PacketData[u16DataSize++] = sEthConfigMode.au8PcIpAddr[0];

    u8PacketData[u16DataSize++] = sEthConfigMode.au8Dca1000IpAddr[3];
    u8PacketData[u16DataSize++] = sEthConfigMode.au8Dca1000IpAddr[2];
    u8PacketData[u16DataSize++] = sEthConfigMode.au8Dca1000IpAddr[1];
    u8PacketData[u16DataSize++] = sEthConfigMode.au8Dca1000IpAddr[0];

    u8PacketData[u16DataSize++] = sEthConfigMode.au8MacId[5];
    u8PacketData[u16DataSize++] = sEthConfigMode.au8MacId[4];
    u8PacketData[u16DataSize++] = sEthConfigMode.au8MacId[3];
    u8PacketData[u16DataSize++] = sEthConfigMode.au8MacId[2];
    u8PacketData[u16DataSize++] = sEthConfigMode.au8MacId[1];
    u8PacketData[u16DataSize++] = sEthConfigMode.au8MacId[0];

    memcpy(&u8PacketData[u16DataSize], &sEthConfigMode.u32ConfigPortNo,
           sizeof(UINT16));
    u16DataSize += UINT16_DATA_SIZE;
    memcpy(&u8PacketData[u16DataSize], &sEthConfigMode.u32RecordPortNo,
           sizeof(UINT16));
    u16DataSize += UINT16_DATA_SIZE;

    /** Copy the EEPROM config info to Command packet                        */
    UINT16 u16PacketLen = objCmdsProto.configEepromCommand(
        s8Data, u8PacketData, u16DataSize);

    /** Send the command packet to FPGA                                      */
    SINT32 s32BytesSent = SendConfigCmdRequest(s8Data, u16PacketLen);

    if(s32BytesSent <= SOCKET_ERROR)
    {
        THROW_ERROR_STATUS("\n\nConfigureRFDCCard_EEPROM(): UDP write failed");
        return (STS_RFDCCARD_UDP_WRITE_ERR);
    }
    else
    {
        return GetConfigCmdResponse("ConfigureRFDCCard_EEPROM");
    }

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nConfigureRFDCCard_EEPROM: Sent successfully");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return (STS_RFDCCARD_SUCCESS);
}


/** @fn EXPORT STATUS HandshakeRFDCCard(void)
 * @brief This function is to verify the DCA1000EVM system connectivity
 * @pre Application should be connected to DCA1000EVM system
 * @return SINT32 value
 */
STATUS HandshakeRFDCCard
(
void
)
{
    /** Copy the system aliveness check info to Command packet               */
    SINT8 s8Data[sizeof(DATA_CAPTURE_REQ)];
    UINT16 u16PacketLen = objCmdsProto.systemAlivenessCommand(s8Data);

    /** Send the command packet to FPGA                                      */
    SINT32 s32BytesSent = SendConfigCmdRequest(s8Data, u16PacketLen);
    if(s32BytesSent <= SOCKET_ERROR)
    {
        THROW_ERROR_STATUS("\n\nHandshakeRFDCCard(): UDP write failed");
        return (STS_RFDCCARD_UDP_WRITE_ERR);
    }
    else
    {
        /** Receive the command response from FPGA                          */
        return GetConfigCmdResponse("HandshakeRFDCCard");
    }

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nHandshakeRFDCCard: Sent successfully");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS DisconnectRFDCCard_RecordMode(void)
 * @brief This function is to close config and data port socket connection <!--
 * --> and disconnect from the DCA1000EVM system.
 * @pre Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_RecordMode API
 * @return SINT32 value
 */
STATUS DisconnectRFDCCard_RecordMode
(
void
)
{
    /** Deinitialize waiting events             */
    osalObj.DeInitEvent(&sgnCmdTimeoutWaitEvent);
    osalObj.DeInitEvent(&sgnDurationStopModeWaitEvent);
    osalObj.DeInitEvent(&sgnCaptureTimeoutWaitEvent);

    
    objUdpConfigRecv.setSocketClose();
    objUdpDataRecv.setSocketClose();
    objUdpCpDataRecv.setSocketClose();
    objUdpCqDataRecv.setSocketClose();
    objUdpR4fDataRecv.setSocketClose();
    objUdpDspDataRecv.setSocketClose();

    /** Close ports */
    if (osalObj.sock_Close(sRFDCCard_SockInfo.s32EthConfSock) != 0)
    {
        THROW_ERROR_STATUS("\n osalObj.sock_Close(): (Config port)");
    }
    if (osalObj.sock_Close(sRFDCCard_SockInfo.s32DataSock[RAW_DATA_INDEX]) != 0)
    {
        THROW_ERROR_STATUS("\n osalObj.sock_Close(): (Data port)");
    }
    if (osalObj.sock_Close(sRFDCCard_SockInfo.s32DataSock[CP_DATA_1_INDEX]) != 0)
    {
        THROW_ERROR_STATUS("\n osalObj.sock_Close(): (CP Data port)");
    }
    if (osalObj.sock_Close(sRFDCCard_SockInfo.s32DataSock[CQ_DATA_2_INDEX]) != 0)
    {
        THROW_ERROR_STATUS("\n osalObj.sock_Close(): (CQ Data port)");
    }
    if (osalObj.sock_Close(sRFDCCard_SockInfo.s32DataSock[R4F_DATA_3_INDEX]) != 0)
    {
        THROW_ERROR_STATUS("\n osalObj.sock_Close(): (R4F Data port)");
    }
    if (osalObj.sock_Close(sRFDCCard_SockInfo.s32DataSock[DSP_DATA_4_INDEX]) != 0)
    {
        THROW_ERROR_STATUS("\n osalObj.sock_Close(): (DSP Data port)");
    }

    sRFDCCard_SockInfo.s32EthConfSock = -1;
    sRFDCCard_SockInfo.s32DataSock[RAW_DATA_INDEX] = -1;
    sRFDCCard_SockInfo.s32DataSock[CP_DATA_1_INDEX] = -1;
    sRFDCCard_SockInfo.s32DataSock[CQ_DATA_2_INDEX] = -1;
    sRFDCCard_SockInfo.s32DataSock[R4F_DATA_3_INDEX] = -1;
    sRFDCCard_SockInfo.s32DataSock[DSP_DATA_4_INDEX] = -1;

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nDisconnectRFDCCard_RecordMode: Success");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS DisconnectRFDCCard_AsyncCommandMode(void)
 * @brief This function is to close unbound config port socket connection <!--
 * --> which is used to send stop record async message to CLI Record tool.
 * @pre Application should be connected to CLI Record process using <!--
 * --> \ref ConnectRFDCCard_AsyncCommandMode API
 * @return SINT32 value
 */
STATUS DisconnectRFDCCard_AsyncCommandMode
(
void
)
{
    /* Close ports */
   if (osalObj.sock_Close(sRFDCCard_SockInfo.s32EthConfAsyncSock) != 0)
    {
        THROW_ERROR_STATUS("\n osalObj.sock_Close(): (Config port)");
    }

    sRFDCCard_SockInfo.s32EthConfAsyncSock = -1;

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nDisconnectRFDCCard_AsyncCommandMode: Success");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS DisconnectRFDCCard_ConfigMode(void)
 * @brief This function is to close bound config port socket connection <!--
 * --> and disconnect from the DCA1000EVM system.
 * @pre Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_ConfigMode API
 * @return SINT32 value
 */
STATUS DisconnectRFDCCard_ConfigMode
(
void
)
{
    /* Close ports */

    if (osalObj.sock_Close(sRFDCCard_SockInfo.s32EthConfSock) != 0)
    {
        THROW_ERROR_STATUS("\n osalObj.sock_Close(): (Config port)");
    }

    sRFDCCard_SockInfo.s32EthConfSock = -1;

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nDisconnectRFDCCard_ConfigMode: Success");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS StartRecordData(strStartRecConfigMode sStartRecConfigMode)
 * @brief This function is to start recording the data streamed over <!--
 * --> Ethernet from the DCA1000EVM system with the \ref <!--
 * --> strStartRecConfigMode structure configuration
 * @param [in] sStartRecConfigMode [strStartRecConfigMode] - <!--
 * --> Structure filled with record config data
 * @pre Application should be connected to DCA1000EVM system and <!--
 * --> FPGA and record packet delay should be configured
 * @return SINT32 value
 */
STATUS StartRecordData
(
strStartRecConfigMode sStartRecConfigMode
)
{
#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nStartRecordData : ");
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nbMsbToggleEnable : %d", sStartRecConfigMode.bMsbToggleEnable);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nbReorderEnable : %d", sStartRecConfigMode.bReorderEnable);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nbSequenceNumberEnable : %d", sStartRecConfigMode.bSequenceNumberEnable);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\neConfigLogMode : %d", sStartRecConfigMode.eConfigLogMode);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\neLvdsMode : %d", sStartRecConfigMode.eLvdsMode);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\neRecordStopMode : %d", sStartRecConfigMode.eRecordStopMode);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\ns8FileBasePath : %s", sStartRecConfigMode.s8FileBasePath);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\ns8FilePrefix : %s", sStartRecConfigMode.s8FilePrefix);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu16MaxRecFileSize : %d", sStartRecConfigMode.u16MaxRecFileSize);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu32BytesToCapture : %d", sStartRecConfigMode.u32BytesToCapture);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu32DurationToCapture : %d", sStartRecConfigMode.u32DurationToCapture);
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nu32FramesToCapture : %d", sStartRecConfigMode.u32FramesToCapture);
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    /** Resetting record global status    */
    gbRecStopCmdSent = false;

    /** Fill zero bytes array for dropped packets  */
    memset(s8ZeroBuf, 0, (MAX_BYTES_PER_PACKET * sizeof(SINT8)));

    /** Validate the file base path             */
    if (!osalObj.IsValidDir(sStartRecConfigMode.s8FileBasePath))
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg, "\nStartRecordData(): "
            "Invalid input parameters (sStartRecConfigMode.s8FileBasePath)");
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\nStartRecordData(): "
            "Invalid input parameters (sStartRecConfigMode.s8FileBasePath)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }


    if(SUCCESS_STATUS != validateFileMaxSize(sStartRecConfigMode.u16MaxRecFileSize))
    {
        printf("\nStartRecordData(): "
            "Invalid input parameters (sStartRecConfigMode.u16MaxRecFileSize)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    if(sStartRecConfigMode.eRecordStopMode == BYTES)
    {
        if(SUCCESS_STATUS != validateBytesStopConfig(
                                    sStartRecConfigMode.u32BytesToCapture,
                                    sStartRecConfigMode.eLvdsMode))
        {
            printf("\nStartRecordData(): "
                "Invalid input parameters (sStartRecConfigMode.u32BytesToCapture)");
            return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
        }
    }

    else if(sStartRecConfigMode.eRecordStopMode == FRAMES)
    {
        if( SUCCESS_STATUS != validateFramesStopConfig(
                                    sStartRecConfigMode.u32FramesToCapture))
        {
            printf("\nStartRecordData(): "
                "Invalid input parameters (sStartRecConfigMode.u32FramesToCapture)");
            return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
        }
    }
    else if(sStartRecConfigMode.eRecordStopMode == DURATION)
    {
        if(SUCCESS_STATUS != validateDurationStopConfig(
                                sStartRecConfigMode.u32DurationToCapture))
        {
            printf("\nStartRecordData(): "
                "Invalid input parameters (sStartRecConfigMode.u32DurationToCapture)");
            return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
        }
    }
    else if(sStartRecConfigMode.eRecordStopMode != NON_STOP)
    {
        printf("\nStartRecordData(): "
            "Invalid input parameters (sStartRecConfigMode.eRecordStopMode)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }


    memcpy(&sRFDCCard_StartRecConfig, &sStartRecConfigMode,
           sizeof(strStartRecConfigMode));

    u32MaxFileSizeToCapture = sRFDCCard_StartRecConfig.u16MaxRecFileSize *
                                1024 * 1024;

    if(sRFDCCard_StartRecConfig.eLvdsMode == TWO_LANE)
        u8LaneNumber = 2;
    else
        u8LaneNumber = 4;

    /** Logging the record configuration in log file */
    WriteRecordSettingsInLogFile();

    /** Resetting the inline stats structure */
    for(int i = 0; i < NUM_DATA_TYPES; i++)
    {
        sRFDCCard_InlineStats.u64OutOfSeqCount[i] = 0;
        sRFDCCard_InlineStats.u32FirstPktId[i] = 0;
        sRFDCCard_InlineStats.u32LastPktId[i] = 0;
        sRFDCCard_InlineStats.u64NumOfRecvdPackets[i] = 0;
        sRFDCCard_InlineStats.u64NumOfZeroFilledBytes[i] = 0;
        sRFDCCard_InlineStats.u64NumOfZeroFilledPackets[i] = 0;
        sRFDCCard_InlineStats.u32OutOfSeqPktFromOffset[i] = 0;
        sRFDCCard_InlineStats.u32OutOfSeqPktToOffset[i] = 0;
        sRFDCCard_InlineStats.EndTime[i] = time(NULL);
        sRFDCCard_InlineStats.StartTime[i] = time(NULL);
    }

    /** Start thread to record data into file for ADC data port */
    objUdpConfigRecv.setSocketOpen();
    std::thread tConfigData([&] { objUdpConfigRecv.readConfigDatagrams(); });
    tConfigData.detach();
    objUdpDataRecv.setSocketOpen();
    std::thread tRawData([&] { objUdpDataRecv.readData(); });
    tRawData.detach();
    objUdpDataRecv.setThreadStart();
#ifndef POST_PROCESSING
    /** Start thread for writing data into file from buffer for ADC port (inline processing) */
    std::thread tRawData2([&] { objUdpDataRecv.Thread_WriteDataToFile(); });
    tRawData2.detach();
#endif

    if(sStartRecConfigMode.eConfigLogMode == MULTI_MODE)
    {
        /** Start thread to record data into file for each of the data ports */
        objUdpCpDataRecv.setSocketOpen();
        std::thread tCpData([&] { objUdpCpDataRecv.readData(); });
        tCpData.detach();
        objUdpCqDataRecv.setSocketOpen();
        std::thread tCqData([&] { objUdpCqDataRecv.readData(); });
        tCqData.detach();
        objUdpR4fDataRecv.setSocketOpen();
        std::thread tR4fData([&] { objUdpR4fDataRecv.readData(); });
        tR4fData.detach();
        objUdpDspDataRecv.setSocketOpen();
        std::thread tDspData([&] { objUdpDspDataRecv.readData(); });
        tDspData.detach();
        objUdpCpDataRecv.setThreadStart();
        objUdpCqDataRecv.setThreadStart();
        objUdpR4fDataRecv.setThreadStart();
        objUdpDspDataRecv.setThreadStart();
#ifndef POST_PROCESSING
        /** Start thread for writing data into file from buffer for each of the
         * data ports (Inline processing)
         */
        std::thread tCpData2([&] { objUdpCpDataRecv.Thread_WriteDataToFile(); });
        tCpData2.detach();
        std::thread tCqData2([&] { objUdpCqDataRecv.Thread_WriteDataToFile(); });
        tCqData2.detach();
        std::thread tR4fData2([&] { objUdpR4fDataRecv.Thread_WriteDataToFile(); });
        tR4fData2.detach();
        std::thread tDspData2([&] { objUdpDspDataRecv.Thread_WriteDataToFile(); });
        tDspData2.detach();
#endif
    }

    /** Copy the start record mode info to Command packet                    */
    SINT8 s8Data[sizeof(DATA_CAPTURE_REQ)];
    UINT16 u16PacketLen = objCmdsProto.startRecordCommand(s8Data);

    /** Send the command packet to FPGA                                      */
    SINT32 s32BytesSent = SendConfigCmdRequest(s8Data, u16PacketLen);

    /** Verifies the start record command sent status */
    if(s32BytesSent <= SOCKET_ERROR)
    {
        objUdpDataRecv.setThreadStop();
        if(sStartRecConfigMode.eConfigLogMode == MULTI_MODE)
        {
            objUdpCpDataRecv.setThreadStop();
            objUdpCqDataRecv.setThreadStop();
            objUdpR4fDataRecv.setThreadStop();
            objUdpDspDataRecv.setThreadStop();
        }

        THROW_ERROR_STATUS("\n\nStartRecordData(): UDP write failed");
        return (STS_RFDCCARD_UDP_WRITE_ERR);
    }
    else
    {
        /** Start duration stop mode timer if enabled */
        if(sRFDCCard_StartRecConfig.eRecordStopMode == DURATION)
        {
            std::thread tTimerToStopRecCp([&] { startDurationStopModeTimer(); });
            tTimerToStopRecCp.detach();
        }

        /** Wait for the start record command response  */
        if (STS_RFDCCARD_EVENT_TIMEOUT_ERR == osalObj.WaitForSignal(
                    &sgnCmdTimeoutWaitEvent, CMD_TIMEOUT_DURATION_SEC))
        {
            /** Exit from the duration stop mode timer on unsuccessful start */
            if(sRFDCCard_StartRecConfig.eRecordStopMode == DURATION)
            {
                sRFDCCard_StartRecConfig.u32DurationToCapture = 1;
                osalObj.SignalEvent(&sgnDurationStopModeWaitEvent);
            }

            return STS_RFDCCARD_TIMEOUT_ERR;
        }
        else
        {
            /** Start a timer to handle data capture timeout(if system disconnected) */
            std::thread tToHandleCaptureTimeout([&] { handleCaptureThreadTimeout(); });
            tToHandleCaptureTimeout.detach();

            /** Return the command response to the calling application */
            return configResp.u16Status;
        }
    }

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nStartRecordData: Sent successfully");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return (STS_RFDCCARD_SUCCESS);
}

/** @fn void startDurationStopModeTimer()
 * @brief This function is to initiate timer to handle duration stop mode <!--
 * --> of data recording
 */
void startDurationStopModeTimer()
{
    /** Wait for the first data packet to be received in any of the data ports*/
    osalObj.WaitForSignal(&sgnDurationStopModeWaitEvent, NON_STOP);

    /** Timer to wait for the specified duration stop mode config   */
    osalObj.SleepInMilliSec(sRFDCCard_StartRecConfig.u32DurationToCapture);

    /** Send the record completed status unless stop command is not sent */
    if(!gbRecStopCmdSent)
        RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                     STS_REC_COMPLETED);
}

/** @fn  void handleCaptureThreadTimeout()
 * @brief This function is to handle capture timeout when the system is  <!--
 * --> disconnected during capture process
 */
void handleCaptureThreadTimeout()
{
    while(!gbRecStopCmdSent)
    {
        if (STS_RFDCCARD_EVENT_TIMEOUT_ERR == osalObj.WaitForSignal(
                    &sgnCaptureTimeoutWaitEvent, CAPTURE_TIMEOUT_DURATION_SEC))
        {
            /** Callback when Timeout occurs */
            if(!gbRecStopCmdSent)
            {
                RFDCARD_Callback(STS_CAPTURE_THREAD_TIMEOUT, 0);
            }
        }
    }
}

/** @fn STATUS StopRecordAsyncCmd(void)
 * @brief This function is to send a message to CLI record process over the <!--
 * --> config port to initiate stop recording data from the DCA1000EVM system <!--
 * --> using \ref StopRecordData API
 * @pre Record process should be running using \ref StartRecordData API. <!--
 * --> Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_AsyncCommandMode API
 * @return SINT32 value
 */
STATUS StopRecordAsyncCmd(void)
{
    /** Copy the start record mode info to Command packet                    */
    SINT8 s8Data[sizeof(DATA_CAPTURE_REQ)];
    UINT16 u16PacketLen = objCmdsProto.stopRecordAsyncCommand(s8Data);

    /** Send the stop record async command packet to record process */
    if (sendto(sRFDCCard_SockInfo.s32EthConfAsyncSock, s8Data, u16PacketLen,
        0, (struct sockaddr *)&ethConf_PortAsyncAddress,
        sizeof(ethConf_PortAsyncAddress)) < u16PacketLen)
    {
        THROW_ERROR_STATUS("\n\nStopRecordAsyncCmd(): UDP write failed");
        return (STS_RFDCCARD_UDP_WRITE_ERR);
    }

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nStopRecordAsyncCmd: Sent successfully");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS StopRecordData(void)
 * @brief This function is to send command to FPGA to stop recording the <!--
 * --> data streamed over Ethernet from the DCA1000EVM system
 * @pre Record process should be running using \ref StartRecordData API
 * @return SINT32 value
 */
STATUS StopRecordData
(
void
)
{
    /** To avoid multiple stop record being called  */
    if(gbRecStopCmdSent)
        return (STS_RFDCCARD_SUCCESS);

    /** Resetting record global status    */
    gbRecStopCmdSent = true;

    osalObj.SignalEvent(&sgnCaptureTimeoutWaitEvent);

    /** Copy the start record mode info to Command packet                    */
    SINT8 s8Data[sizeof(DATA_CAPTURE_REQ)];
    UINT16 u16PacketLen = objCmdsProto.stopRecordCommand(s8Data);

    objUdpDataRecv.setThreadStop();
    if(sRFDCCard_StartRecConfig.eConfigLogMode == MULTI_MODE)
    {
        objUdpCpDataRecv.setThreadStop();
        objUdpCqDataRecv.setThreadStop();
        objUdpR4fDataRecv.setThreadStop();
        objUdpDspDataRecv.setThreadStop();
    }

    /** Verifies the stop record command sent status */
    if(SendConfigCmdRequest(s8Data, u16PacketLen) <= SOCKET_ERROR)
    {
        THROW_ERROR_STATUS("\n\nStopRecordData(): UDP write failed");
        return (STS_RFDCCARD_UDP_WRITE_ERR);
    }
    else
    {
        /** Wait for the stop record command response  */
        if (STS_RFDCCARD_EVENT_TIMEOUT_ERR == osalObj.WaitForSignal(
                    &sgnCmdTimeoutWaitEvent, CMD_TIMEOUT_DURATION_SEC))
        {
#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nStopRecordData: Response timeout error");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

            WriteInlineProcSummaryInLogFile();

            RFDCARD_Callback(STS_CLI_REC_PROC_STOP_FAILED, 0);

            return STS_RFDCCARD_TIMEOUT_ERR;
        }
        else
        {
#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nStopRecordData: Sent successfully");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

            WriteInlineProcSummaryInLogFile();

            RFDCARD_Callback(STS_CLI_REC_PROC_STOPPED, 0);

            /** Return the command response to the calling application */
            return configResp.u16Status;
        }
    }

    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS ResetRFDCCard_FPGA(void)
 * @brief This function is to reset DCA1000EVM FPGA
 * @pre Application should be connected to DCA1000EVM system
 * @return SINT32 value
 */
STATUS ResetRFDCCard_FPGA
(
void
)
{
    /** Copy the reset DC card FPGA info to Command packet                   */
    SINT8 s8Data[sizeof(DATA_CAPTURE_REQ)];
    UINT16 u16PacketLen = objCmdsProto.resetFpgaCommand(s8Data);

    /** Send the command packet to FPGA                                      */
    SINT32 s32BytesSent = SendConfigCmdRequest(s8Data, u16PacketLen);

    if(s32BytesSent <= SOCKET_ERROR)
    {
        THROW_ERROR_STATUS("\n\nResetDCCard_FPGA(): UDP write failed");
        return (STS_RFDCCARD_UDP_WRITE_ERR);
    }
    else
    {
        /** Receive the command response from FPGA                          */
        return GetConfigCmdResponse("ResetDCCard_FPGA");
    }


#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nResetDCCard_FPGA: Sent successfully");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS ResetRadarEVM(void)
 * @brief This function is to reset RADAR AR device
 * @pre Application should be connected to DCA1000EVM system
 * @return SINT32 value
 */
STATUS ResetRadarEVM
(
void
)
{
    /** Copy the reset Radar EVM info to Command packet                      */
    SINT8 s8Data[sizeof(DATA_CAPTURE_REQ)];
    UINT16 u16PacketLen = objCmdsProto.resetArDeviceCommand(s8Data);

    /** Send the command packet to FPGA                                      */
    SINT32 s32BytesSent = SendConfigCmdRequest(s8Data, u16PacketLen);

    if(s32BytesSent <= SOCKET_ERROR)
    {
        THROW_ERROR_STATUS("\n\nResetRadarEVM(): UDP write failed");
        return (STS_RFDCCARD_UDP_WRITE_ERR);
    }
    else
    {
        /** Recieve the command response from FPGA */
        return GetConfigCmdResponse("ResetRadarEVM");
    }

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nResetRadarEVM: Sent successfully");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif
    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS StatusRFDCCard_EventRegister (EVENT_HANDLER RFDCCard_EventCallback)
 * @brief This function is to register user event callback for handling <!--
 * --> async status from FPGA and stop record command response
 * @param [in] RFDCCard_EventCallback  [EVENT_HANDLER] - Callback function
 * @return SINT32 value
 */
STATUS StatusRFDCCard_EventRegister
(
    EVENT_HANDLER		RFDCCard_EventCallback
)
{
    if (NULL == RFDCCard_EventCallback)
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg,
            "\n\nStatusDCCard_EventRegister(RFDCCard_EventCallback is null)");
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\n\nStatusDCCard_EventRegister(RFDCCard_EventCallback is null)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    RFDCARD_Callback = RFDCCard_EventCallback;

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nStatusDCCard_EventRegister: Sent successfully");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return STS_RFDCCARD_SUCCESS;
}

/** @fn STATUS RecInlineProcStats_EventRegister (INLINE_PROC_HANDLER RecordStats_Callback)
 * @brief This function is to register user event callback for updating <!--
 * --> inline processing statistics of recording process
 * @param [in] RecordStats_Callback  [INLINE_PROC_HANDLER] - <!--
 * --> Callback function
 * @pre Record process should be running using \ref StartRecordData API
 * @return SINT32 value
 */
STATUS RecInlineProcStats_EventRegister
(
    INLINE_PROC_HANDLER   RecordStats_Callback
)
{
    if (NULL == RecordStats_Callback)
    {
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsg,
            "\n\nRecInlineProcStats_EventRegister(RecordStats_Callback is null)");
        DEBUG_FILE_WRITE(s8DebugMsg);
#endif
        printf("\n\nRecInlineProcStats_EventRegister(RecordStats_Callback is null)");
        return (STS_RFDCCARD_INVALID_INPUT_PARAMS);
    }

    RecordInlineProc_Callback = RecordStats_Callback;

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nRecInlineProcStats_EventRegister: Sent successfully");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return STS_RFDCCARD_SUCCESS;
}

/** @fn STATUS ConfigureRFDCCard_Record(strRecConfigMode sRecConfigMode)
 * @brief This function is to configure record data packet delay <!--
 * --> in data recording with the following configuration @n <!--
 * -->	1.	Record delay
 * @param [in] sRecConfigMode [strRecConfigMode] - <!--
 * --> Structure filled with config data
 * @pre Application should be connected to DCA1000EVM system
 * @return SINT32 value
 */
STATUS ConfigureRFDCCard_Record
(
strRecConfigMode sRecConfigMode
)
{
#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nConfigureRFDCCard_Record : ");
    DEBUG_FILE_WRITE(s8DebugMsg);
    sprintf(s8DebugMsg, "\nRecord packet delay : %d", sRecConfigMode.u16RecDelay);
    DEBUG_FILE_WRITE(s8DebugMsg);

#endif

    /** Copy the configuration mode info to Command packet                   */
    UINT16 u16DataSize = 0;
    UINT16 u16Data = 0;
    UINT8 u8PacketData[MAX_DATA_BYTES];

    u16Data = MAX_BYTES_PER_PACKET;
    memcpy(&u8PacketData[u16DataSize], &u16Data,
           sizeof(UINT16));
    u16DataSize += UINT16_DATA_SIZE;

    u16Data = ((UINT16)sRecConfigMode.u16RecDelay *
                         FPGA_CLK_CONVERSION_FACTOR /
                         FPGA_CLK_PERIOD_IN_NANO_SEC);
    memcpy(&u8PacketData[u16DataSize], &u16Data,
           sizeof(UINT16));
    u16DataSize += UINT16_DATA_SIZE;
    u16Data = 0;
    memcpy(&u8PacketData[u16DataSize], &u16Data,
           sizeof(UINT16));
    u16DataSize += UINT16_DATA_SIZE;

    SINT8 s8Data[sizeof(DATA_CAPTURE_REQ)];
    UINT16 u16PacketLen = objCmdsProto.configDataPacketCommand(s8Data,
        u8PacketData, u16DataSize);

    /** Send the command packet to FPGA                                      */
    SINT32 s32BytesSent = SendConfigCmdRequest(s8Data, u16PacketLen);

    if(s32BytesSent <= SOCKET_ERROR)
    {
        THROW_ERROR_STATUS("\n\nConfigureRFDCCard_Record(): UDP write failed");
        return (STS_RFDCCARD_UDP_WRITE_ERR);
    }
    else
    {
        /** Receive the command response        */
        return GetConfigCmdResponse("ConfigureRFDCCard_Record");
    }

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nConfigureRFDCCard_Record: Sent successfully");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS ReadRFDCCard_DllVersion(SINT8 *s8DllVersion)
 * @brief This function is to read API DLL version
 * @param [out] s8DllVersion [SINT8 *] - Array filled with version
 * @return SINT32 value
 */
STATUS ReadRFDCCard_DllVersion
(
SINT8 *s8DllVersion
)
{
    memset(s8DllVersion, 0, 10);
    memcpy(s8DllVersion, DCA1000EVM_DLL_VERSION, 10);
    return (STS_RFDCCARD_SUCCESS);
}

/** @fn STATUS ReadRFDCCard_FpgaVersion(SINT8 *s8FpgaVersion)
 * @brief This function is to read FPGA version
 * @param [out] s8FpgaVersion [SINT8 *] - Array filled with version
 * @pre Application should be connected to DCA1000EVM system
 * @return SINT32 value
 */
STATUS ReadRFDCCard_FpgaVersion
(
SINT8 *s8FpgaVersion
)
{
    /** Copy the read FPGA version command info to Command packet            */
    SINT8 s8Data[sizeof(DATA_CAPTURE_REQ)];
    UINT16 u16PacketLen = objCmdsProto.readFpgaVersionCommand(s8Data);

    /** Send the command packet to FPGA                                      */
    SINT32 s32BytesSent = SendConfigCmdRequest(s8Data, u16PacketLen);

    if(s32BytesSent <= SOCKET_ERROR)
    {
        THROW_ERROR_STATUS("\n\nReadRFDCCard_FpgaVersion(): UDP write failed");
        return (STS_RFDCCARD_UDP_WRITE_ERR);
    }
    else
    {
        /** Receive the command response        */
        SINT32 s32Status = GetConfigCmdResponse("ReadRFDCCard_FpgaVersion");

        if(s32Status < SUCCESS_STATUS)
        {
            sprintf(s8FpgaVersion, "\nUnable to read FPGA Version. [error %d]\n",
                    s32Status);
        }
        else
        {
            /** Parse the version from the command response */
            UINT8 u8MajorVersion = (UINT8)(s32Status & VERSION_BITS_DECODE);
            UINT8 u8MinorVersion = (UINT8)((s32Status >> VERSION_NUM_OF_BITS) &
                                           VERSION_BITS_DECODE);

            memset(s8FpgaVersion, 0, 50);

            if((s32Status & PLAYBACK_BIT_DECODE) == PLAYBACK_BIT_DECODE)
            {
                sprintf(s8FpgaVersion, "\nFPGA Version : %d.%d [Playback]\n",
                        u8MajorVersion,u8MinorVersion);
            }
            else
            {
                sprintf(s8FpgaVersion, "\nFPGA Version : %d.%d [Record]\n",
                        u8MajorVersion,u8MinorVersion);
            }
        }
        return s32Status;
    }

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "\n\nReadRFDCCard_FpgaVersion: Sent successfully");
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif

    return (STS_RFDCCARD_SUCCESS);
}

/** @fn void THROW_ERROR_STATUS(const SINT8 *s8Msg)
 * @brief This function is to display the error message in console or file(if debug mode enabled)
 * @param [in] s8Msg [const SINT8 *] - Error message to display
 */
void THROW_ERROR_STATUS
(
const SINT8 *s8Msg
)
{
    SINT32 s32LastErrorMain = 0;

    s32LastErrorMain = osalObj.GetLastErrNo();

#ifdef ENABLE_DEBUG
    sprintf(s8DebugMsg, "%s : %d", s8Msg, s32LastErrorMain);
    DEBUG_FILE_WRITE(s8DebugMsg);
#endif
    printf("\n%s : %d", s8Msg, s32LastErrorMain);
}

#ifdef ENABLE_DEBUG
/** @fn void DEBUG_FILE_WRITE(const SINT8 *s8Msg)
 * @brief This function is to write the error message in a file (if debug mode enabled)
 * @param [in] s8Msg [const SINT8 *]  - Error message to display
 */
void DEBUG_FILE_WRITE
(
const SINT8 *s8Msg
)
{
    pDebugFile = fopen(s8DebugFileName, "a");

    if (NULL != pDebugFile)
    {
        fprintf(pDebugFile, "%s", s8Msg);
    }

    fclose(pDebugFile);
}
#endif

/** @fn void WriteRecordSettingsInLogFile()
 * @brief This function is to write the record settings in a log file
 */
void WriteRecordSettingsInLogFile()
{
    SINT8 s8LogMsg[MAX_FILE_UPDATE_LEN];
    SINT8 s8LogMsg2[MAX_NAME_LEN];
    strcpy(s8LogMsg, sRFDCCard_StartRecConfig.s8FileBasePath);
#ifdef _WIN32
    strcat(s8LogMsg, "\\");
#else
    strcat(s8LogMsg, "/");
#endif
    strcat(s8LogMsg, sRFDCCard_StartRecConfig.s8FilePrefix);

    if(sRFDCCard_StartRecConfig.eConfigLogMode == RAW_MODE)
    {
        strcat(s8LogMsg, RAW_MODE_FILE_NAME);
    }
    else
    {
        strcat(s8LogMsg, MULTI_MODE_FILE_NAME);
    }
    strcat(s8LogMsg, "LogFile.csv");
    s8LogMsg[strlen(s8LogMsg)] = '\0';

    pInlineLogFile = fopen(s8LogMsg, "w");

    if (NULL == pInlineLogFile)
    {
        sprintf(s8LogMsg2, "\nNot able to write inline processing summary.\n %s ",
                s8LogMsg);
    }
    else
    {
        sprintf(s8LogMsg2, "Start record configuration : \n,");
        strcpy(s8LogMsg, s8LogMsg2);
        if(sRFDCCard_StartRecConfig.eConfigLogMode == RAW_MODE)
            sprintf(s8LogMsg2, "\nLog mode : Raw");
        else if(sRFDCCard_StartRecConfig.eConfigLogMode == MULTI_MODE)
                sprintf(s8LogMsg2, "\nLog mode : Multi");
        strcat(s8LogMsg, s8LogMsg2);
        if(sRFDCCard_StartRecConfig.eLvdsMode == FOUR_LANE)
            sprintf(s8LogMsg2, "\nLVDS lane mode : 4 lane");
        else if(sRFDCCard_StartRecConfig.eLvdsMode == TWO_LANE)
                sprintf(s8LogMsg2, "\nLVDS lane mode : 2 lane");
        strcat(s8LogMsg, s8LogMsg2);
        if(sRFDCCard_StartRecConfig.eRecordStopMode == NON_STOP)
            sprintf(s8LogMsg2, "\nRecord stop mode : Infinite");
        else if(sRFDCCard_StartRecConfig.eRecordStopMode == BYTES)
            sprintf(s8LogMsg2, "\nBytes mode : Bytes (%d)",
                    sRFDCCard_StartRecConfig.u32BytesToCapture);
        else if(sRFDCCard_StartRecConfig.eRecordStopMode == FRAMES)
            sprintf(s8LogMsg2, "\nFrames mode : Frames (%d)",
                    sRFDCCard_StartRecConfig.u32FramesToCapture);
        else if(sRFDCCard_StartRecConfig.eRecordStopMode == DURATION)
            sprintf(s8LogMsg2, "\nDuration mode : Duration (%d)",
                    sRFDCCard_StartRecConfig.u32DurationToCapture);
        strcat(s8LogMsg, s8LogMsg2);
#ifdef POST_PROCESSING
        if(sRFDCCard_StartRecConfig.bSequenceNumberEnable)
            sprintf(s8LogMsg2, "\nSequence number : true");
        else
            sprintf(s8LogMsg2, "\nSequence number : false");
        strcat(s8LogMsg, s8LogMsg2);
#endif
        sprintf(s8LogMsg2, "\nMax file size (MB) : %d,",
                sRFDCCard_StartRecConfig.u16MaxRecFileSize);
        strcat(s8LogMsg, s8LogMsg2);
#ifndef POST_PROCESSING

#ifdef LOG_DROPPED_PKTS_OFFSET
        if(sRFDCCard_StartRecConfig.eConfigLogMode == RAW_MODE)
            sprintf(s8LogMsg2, "\n,*DT 1,");
        else
            sprintf(s8LogMsg2, "\n,*DT 1,,*DT 2,,*DT 3,,*DT 4,,*DT 5,");
#else //LOG_OUT_OF_SEQ_OFFSET
        if(sRFDCCard_StartRecConfig.eConfigLogMode == RAW_MODE)
            sprintf(s8LogMsg2, "\n,*DT 1,");
        else
            sprintf(s8LogMsg2, "\n,*DT 1,,,,,*DT 2,,,,,*DT 3,,,,,*DT 4,,,,,*DT 5,,,,,");
#endif
        strcat(s8LogMsg, s8LogMsg2);
#endif
        fprintf(pInlineLogFile, "%s", s8LogMsg);
    }
}

/** @fn void WriteInlineProcSummaryInLogFile()
 * @brief This function is to write the inline processing summary in a log file
 */
void WriteInlineProcSummaryInLogFile()
{
    SINT8 s8LogMsg[MAX_FILE_UPDATE_LEN];
    SINT8 s8LogMsg2[MAX_NAME_LEN];
    UINT8 u8NumDataTypes = NUM_DATA_TYPES;

    if(pInlineLogFile != NULL)
    {
        if(sRFDCCard_StartRecConfig.eConfigLogMode == RAW_MODE)
        {
            u8NumDataTypes = 1;
        }

        for(int i = 0; i < u8NumDataTypes; i ++)
        {
            if(u8NumDataTypes == 1)
            {
                sprintf(s8LogMsg2, "\nRaw Data :");
            }else
            {
                if(strcmp(sRFDCCard_InlineStats.s8HeaderId[i],
                       "") == 0)
                {
                    sprintf(s8LogMsg2, "\nDT %d Header Data : (No data received)",
                            i+1);
                }
                else
                {
                    sprintf(s8LogMsg2, "\n%s Header Data :",
                        sRFDCCard_InlineStats.s8HeaderId[i]);
                }
            }
            strcpy(s8LogMsg, s8LogMsg2);
            sprintf(s8LogMsg2, "\nOut of sequence count - %llu",
                    sRFDCCard_InlineStats.u64OutOfSeqCount[i]);
            strcat(s8LogMsg, s8LogMsg2);
#ifndef POST_PROCESSING
            sprintf(s8LogMsg2, "\nOut of sequence seen from %u to %u",
                    sRFDCCard_InlineStats.u32OutOfSeqPktFromOffset[i],
                    sRFDCCard_InlineStats.u32OutOfSeqPktToOffset[i]);
            strcat(s8LogMsg, s8LogMsg2);
#endif
            sprintf(s8LogMsg2, "\nFirst Packet ID - %d",
                    sRFDCCard_InlineStats.u32FirstPktId[i]);
            strcat(s8LogMsg, s8LogMsg2);
            sprintf(s8LogMsg2, "\nLast Packet ID - %d",
                    sRFDCCard_InlineStats.u32LastPktId[i]);
            strcat(s8LogMsg, s8LogMsg2);
            sprintf(s8LogMsg2, "\nNumber of received packets - %llu",
                    sRFDCCard_InlineStats.u64NumOfRecvdPackets[i]);
            strcat(s8LogMsg, s8LogMsg2);
#ifndef POST_PROCESSING
            sprintf(s8LogMsg2, "\nNumber of zero filled packets - %llu",
                    sRFDCCard_InlineStats.u64NumOfZeroFilledPackets[i]);
            strcat(s8LogMsg, s8LogMsg2);
            sprintf(s8LogMsg2, "\nNumber of zero filled bytes - %llu",
                    sRFDCCard_InlineStats.u64NumOfZeroFilledBytes[i]);
            strcat(s8LogMsg, s8LogMsg2);
#endif
            sprintf(s8LogMsg2, "\nCapture start time - %s",
                    ctime(&sRFDCCard_InlineStats.StartTime[i]));
            strcat(s8LogMsg, s8LogMsg2);
            sprintf(s8LogMsg2, "Capture end time - %s",
                    ctime(&sRFDCCard_InlineStats.EndTime[i]));
            strcat(s8LogMsg, s8LogMsg2);
            ULONG64 seconds = difftime(sRFDCCard_InlineStats.EndTime[i],
                                      sRFDCCard_InlineStats.StartTime[i]);
            sprintf(s8LogMsg2, "Duration(sec) - %llu", seconds);
            strcat(s8LogMsg, s8LogMsg2);

            fprintf(pInlineLogFile, "\n%s", s8LogMsg);
        }

        fclose(pInlineLogFile);
    }
}

/** @fn void UpdateInlineStatus(bool bOutOfSeqFlag, UINT8 u8DataIndex)
 * @brief This function is to update status of data capture based on data index
 * @param [in] bOutOfSeqFlag [bool] Out of sequence enable flag
 * @param [in] u8DataIndex [UINT8]  Data Index
 */
void UpdateInlineStatus(bool bOutOfSeqFlag, UINT8 u8DataIndex)
{
    RecordInlineProc_Callback(sRFDCCard_InlineStats, bOutOfSeqFlag,
                              u8DataIndex);
}
