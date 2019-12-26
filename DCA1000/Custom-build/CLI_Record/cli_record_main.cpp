/**
 * @file cli_record_main.cpp
 *
 * @author JP
 *
 * @version 0.5
 *
 * @brief This file supports data capture process. This file handles start and
 * stop of the recording using UDP packets, storing in files based on the
 * capture configuration in DCA1000EVM system
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
/// 0.1            07 Dec 2018       JP          Created
/// 0.2            27 Dec 2018       JP          Linux support APIs added
/// 0.3            17 Jan 2019       JP          Inline processing added and
///                                              Review comments incorporated
/// 0.4            29 Mar 2019       JP          Asynchronous status updated
/// 0.5            09 Apr 2019       JP          Reordering record data as
///                                              user configurable option
///****************************************************************************

///****************
/// Includes
///****************

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "../Common/globals.h"
#include "../Common/Validate_Utils/validate_params.h"
#include "../Common/errcodes.h"
#include "../Common/Json_Utils/dist/json/json.h"
#include "../Common/Json_Utils/dist/json/json-forwards.h"
#include "../Common/Osal_Utils/osal.h"

///****************
/// Variable declarations
///****************

/** Quiet mode enable/disable                   */
bool gbCliQuietMode = false;

/** Osal class object                           */
osal osalObj_Rec;

/** Ethernet config mode stucture object        */
strEthConfigMode gsEthConfigMode;

/** Start record config mode stucture object    */
strStartRecConfigMode gsStartRecConfigMode;

/** JSON file data                              */
std::stringstream fileData;

/** Record in progress status to update inline status       */
bool bRecProgress = false;

/** Ready status to update inline status                    */
bool bReady = true;

/** Stop command sent flag to avoid multiple time sending   */
bool bStopCmdSent = false;

/** Inline status structure to be updated       */
strRFDCCard_InlineProcStats gInlineStats;

/** Inline status - Out of sequence set flag    */
bool gbOutOfSeqSetFlag = true;

/** Inline status - Data type index             */
UINT8 gu8DataIndex = 0;

/** Record stop event flag                                               */
OSAL_SIGNAL_HANDLE_TYPE sgnRecStopWaitEvent;

/** Inline status update event flag                                      */
OSAL_SIGNAL_HANDLE_TYPE sgnInlineStsUpdateWaitEvent;

/** void WRITE_TO_CONSOLE_REC(const SINT8 *msg)
 * @brief This function is to write the CLI messages in the console  <!--
 * --> if quiet mode is not enabled
 * @param [in] msg [const SINT8 *] - Message to display
 * @return SINT32 value
 */
void WRITE_TO_CONSOLE_REC(const SINT8 *msg)
{
    if(!gbCliQuietMode)
    {
        printf("\n%s\n",msg);
    }
}

/** @fn void WRITE_TO_LOG_FILE_REC(const SINT8 *s8Msg)
 * @brief This function is to write the CLI messages in the logfile
 * @param [in] s8Msg [const SINT8 *] - Message to display
 */
void WRITE_TO_LOG_FILE_REC
(
    const SINT8 *s8Msg
)
{
    time_t loggingTime = time(NULL);
    FILE * pDebugFile;
    pDebugFile = fopen(CLI_LOG_NAME, "a+");

    if(NULL != pDebugFile)
    {
        fprintf(pDebugFile, "\n%s%s\n", ctime(&loggingTime), s8Msg);
    }

    fclose(pDebugFile);
}

/** @fn void ListOfCmds_Rec()
 * @brief This function is to list commands supported by CLI Record tool
 */
void ListOfCmds_Rec()
{
    printf("Cli_Record [options]");
    printf("\nOptions:");
    printf("\n%s\t\t%s", CMD_START_RECORD, "Start Record");
    printf("\n%s\t\t%s", CMD_READ_DLL_VER, "Read DLL version");
    printf("\n%s\t\t%s", CMD_READ_CLI_VER, "Read CLI_Record tool version");
    printf("\n%s\t\t\t%s", CMD_HELP_S_CLI_APP, "List of commands supported");
    printf("\n%s\t\t\t%s", CMD_QUIET_MODE_CLI_APP,
                        "Quiet mode - No status display in the console\n");
}

/** @fn void StopRecordProc_Thread()
 * @brief This thread function is to handle execute stop record command <!--
 * --> and signal to exit the record tool
 */
void StopRecordProc_Thread()
{
    /** Write shared memory */
    osalObj_Rec.WriteRecordProcStatus(gsEthConfigMode.u32ConfigPortNo,
                                  STS_CLI_REC_PROC_STOP_INIT);

    /** API Call - Stop Record                                           */
    SINT32 s32CliStatusCb = StopRecordData();

    if(s32CliStatusCb == STS_RFDCCARD_UDP_WRITE_ERR)
    {
        osalObj_Rec.WriteRecordProcStatus(gsEthConfigMode.u32ConfigPortNo,
                                    STS_CLI_REC_PROC_STOP_FAILED);
        WRITE_TO_CONSOLE_REC("Record stop failed");
    }

    /** Exit the record process         */
    osalObj_Rec.SignalEvent(&sgnRecStopWaitEvent);
}

/** @fn void StopRecordProc_Callback()
 * @brief This function is to create a thread to send stop record command <!--
 * --> through Ethernet over config port. Thread process is created to  <!--
 * --> execute the command without blocking other callback status
 */
void StopRecordProc_Callback()
{
    if(!bStopCmdSent)
    {
        bStopCmdSent = true;
        std::thread tStopRecData([&] { StopRecordProc_Thread(); });
        tStopRecData.detach();
    }
}

/** @fn void cmdRecvCallback(UINT16 u16CmdCode, UINT16 u16Status)
 * @brief This function is to handle the responses from FPGA using callbacks
 * @param [in] u16CmdCode [UINT16] - Command code
 * @param [in] u16Status [UINT16] - Command status
 */
void cmdRecvCallback(UINT16 u16CmdCode, UINT16 u16Status)
{
    char strData[MAX_NAME_LEN] = "System status :";

    switch(u16CmdCode)
    {
    case CMD_CODE_CLI_ASYNC_RECORD_STOP:
        //WRITE_TO_CONSOLE_REC("Record Process : \nUser stop initiated");
        //WRITE_TO_LOG_FILE_REC("Record Process : \nUser stop initiated");
        StopRecordProc_Callback();
        break;
    case STS_CAPTURE_THREAD_TIMEOUT:
        osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                         (1 << STS_REC_PROC_TIMEOUT));
        WRITE_TO_CONSOLE_REC(
                "Record Process : \nTimeout Error! System disconnected");
        WRITE_TO_LOG_FILE_REC(
                "Record Process : \nTimeout Error! System disconnected");
        /** Write shared memory */
        osalObj_Rec.WriteRecordProcStatus(gsEthConfigMode.u32ConfigPortNo,
                                      STS_CLI_REC_PROC_STOP_FAILED);

        if(!bStopCmdSent)
        {
            StopRecordProc_Callback();
        }
        else
        {
            /** Force stop the process */
            osalObj_Rec.SignalEvent(&sgnRecStopWaitEvent);
        }

        break;
    case STS_CLI_REC_PROC_STOP_INIT:
        osalObj_Rec.WriteRecordProcStatus(gsEthConfigMode.u32ConfigPortNo,
                                      STS_CLI_REC_PROC_STOP_INIT);
        WRITE_TO_CONSOLE_REC("Record stop is initiated");
        WRITE_TO_LOG_FILE_REC("Record stop is initiated");
        break;
    case STS_CLI_REC_PROC_STOPPED:
        osalObj_Rec.WriteRecordProcStatus(gsEthConfigMode.u32ConfigPortNo,
                                      STS_CLI_REC_PROC_STOPPED);
        WRITE_TO_CONSOLE_REC("Record stop is done successfully");
        WRITE_TO_LOG_FILE_REC("Record stop is done successfully");

        /** Exit the record process         */
        osalObj_Rec.SignalEvent(&sgnRecStopWaitEvent);
        break;
    case STS_CLI_REC_PROC_STOP_FAILED:
        osalObj_Rec.WriteRecordProcStatus(gsEthConfigMode.u32ConfigPortNo,
                                    STS_CLI_REC_PROC_STOP_FAILED);
        WRITE_TO_CONSOLE_REC("Record stop failed");
        WRITE_TO_LOG_FILE_REC("Record stop failed");

        /** Exit the record process         */
        osalObj_Rec.SignalEvent(&sgnRecStopWaitEvent);
        break;

    case CMD_CODE_SYSTEM_ASYNC_STATUS:
        switch(u16Status)
        {
        case STS_NO_LVDS_DATA:
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_NO_LVDS_DATA));

            strcpy(strData, "No LVDS data");

            StopRecordProc_Callback();

            break;
        case STS_NO_HEADER:
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_NO_HEADER));

            strcpy(strData, "No Header");

            StopRecordProc_Callback();

            break;
        case STS_EEPROM_FAILURE:
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_EEPROM_FAILURE));

            strcpy(strData, "EEPROM Failure");
            break;
        case STS_SD_CARD_DETECTED:
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_SD_CARD_DETECTED));

            strcpy(strData, "SD card detected");
            break;
        case STS_SD_CARD_REMOVED:
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_SD_CARD_REMOVED));

            strcpy(strData, "SD card removed");
            break;
        case STS_SD_CARD_FULL:
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_SD_CARD_FULL));
            strcpy(strData, "SD card full");
            break;
        case STS_MODE_CONFIG_FAILURE:
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_MODE_CONFIG_FAILURE));

            strcpy(strData, "Mode configuration failure");
            break;
        case STS_DDR_FULL:
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_DDR_FULL));

            strcpy(strData, "DDR is full");
            break;
        case STS_REC_COMPLETED:
            strcpy(strData, "Record is completed");
            StopRecordProc_Callback();

            break;
        case STS_LVDS_BUFFER_FULL:
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_LVDS_BUFFER_FULL));

            strcpy(strData, "LVDS buffer full");
            break;
        case STS_REC_FILE_CREATION_ERR:
            strcpy(strData, "Record file creation failed.");
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_REC_FILE_CREATION_ERR));

            StopRecordProc_Callback();

            break;
        case STS_REC_REORDERING_ERR:
            strcpy(strData, "Record process reordering failed.");
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_REC_REORDERING_ERR));

            StopRecordProc_Callback();

            break;
        case STS_INVALID_RESP_PKT_ERR:
            strcpy(strData, "Invalid packet received.");
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_INVALID_RESP_PKT_ERR));
            break;
        case STS_REC_INLINE_BUF_ALLOCATION_ERR:
            strcpy(strData, "Inline buffer allocation failed.");
            osalObj_Rec.WriteRecAsyncStatus(gsEthConfigMode.u32ConfigPortNo,
                             (1 << STS_REC_INLINE_BUF_ALLOCATION_ERR));
            break;
        default:
            sprintf(strData, "%d None", u16Status);
            break;
        }
        WRITE_TO_CONSOLE_REC(strData);
        WRITE_TO_LOG_FILE_REC(strData);

        break;
    default:
        WRITE_TO_CONSOLE_REC("New command response - not handled");
        WRITE_TO_LOG_FILE_REC("New command response - not handled");
        break;
    }
}

/** @fn void DecodeCommandStatus_Rec(SINT32 s32Status, const SINT8 * strCommand)
 * @brief This function is to decode command response status as success or failure
 * @param [in] s32Status [SINT32] - Command status
 * @param [in] strCommand [const SINT8 *] - Command name
 */
void DecodeCommandStatus_Rec(SINT32 s32Status, const SINT8 * strCommand)
{
    SINT8 msgData[MAX_NAME_LEN];

    if(s32Status == SUCCESS_STATUS)
    {
        sprintf(msgData, "%s command : Success",strCommand);
    }
    else if(s32Status == FAILURE_STATUS)
    {
        sprintf(msgData, "%s command : Failed",strCommand);
    }
    else if(s32Status == STS_RFDCCARD_INVALID_INPUT_PARAMS)
    {
        sprintf(msgData, "%s : \nVerify the input parameters - %d",
                strCommand, s32Status);
    }
    else if(s32Status == STS_RFDCCARD_OS_ERR)
    {
        sprintf(msgData, "%s : \nOS error - %d",
                strCommand, s32Status);
    }
    else if(s32Status == STS_RFDCCARD_UDP_WRITE_ERR)
    {
        sprintf(msgData, "%s : \nSending command failed - %d",
                strCommand, s32Status);
    }
    else if(s32Status == STS_RFDCCARD_TIMEOUT_ERR)
    {
        sprintf(msgData, "%s : \nTimeout Error! System disconnected",
                strCommand);
    }
    else if(s32Status == STS_INVALID_RESP_PKT_ERR)
    {
        sprintf(msgData, "%s : \nInvalid response packet error code",
                strCommand);
    }
    else
    {
        sprintf(msgData, "%s Cmd", strCommand);
    }

    WRITE_TO_CONSOLE_REC(msgData);
    WRITE_TO_LOG_FILE_REC(msgData);
    sprintf(msgData, "Return status : %d", s32Status);
    WRITE_TO_LOG_FILE_REC(msgData);
}

/** @fn void inlineStatsCallback(strRFDCCard_InlineProcStats inlineStats, bool bOutOfSeqSetFlag)
 * @brief This function is to handle the update of inline processing summary using callbacks
 * @param [in] inlineStats [strRFDCCard_InlineProcStats] - Status code
 * @param [in] bOutOfSeqSetFlag [bool] - If OutOfSeq occured then set flag
*/
void inlineStatsCallback(strRFDCCard_InlineProcStats inlineStats,
                         bool bOutOfSeqSetFlag, UINT8 u8DataIndex)
{
    if(bReady)
    {
        /** Copying global variables and trigger the event to update    */
        memcpy(&gInlineStats, &inlineStats, sizeof(strRFDCCard_InlineProcStats));
        gbOutOfSeqSetFlag = bOutOfSeqSetFlag;
        gu8DataIndex = u8DataIndex;

        osalObj_Rec.SignalEvent(&sgnInlineStsUpdateWaitEvent);
    }
}

/** @fn SINT32 ValidateJsonFileData_Rec(SINT8 *configFile, UINT16 u16CmdCode)
 * @brief This function is to read and validate JSON file data corresponding <!--
 * --> to the command executed
 * @param [in] configFile [SINT8 *] - JSON config file
 * @param [in] u16CmdCode [UINT16] - Command code
 * @return SINT32 value
 */
SINT32 ValidateJsonFileData_Rec(SINT8 *configFile, UINT16 u16CmdCode)
{
    SINT16 s16Status = 0;
    SINT8 buffer[MAX_FILE_UPDATE_LEN];
    SINT8 s8DebugMsg[MAX_NAME_LEN*2];
    Json::Value root;
    Json::Value node;
    Json::CharReaderBuilder reader;
    Json::Value obj;
    SINT8 nodeData[MAX_PARAMS_LEN];
    SINT8 *token;
    SINT32 k = 0;
    std::string errs = "";
    ULONG lFileSize = 0;

    FILE *fp = fopen(configFile, "r");

    /** Read contents from JSON file */
    if(fp == NULL)
    {
        sprintf(s8DebugMsg, "Unable to open the JSON file (%s). error[%d]",
                configFile, CLI_JSON_FILE_OPEN_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        return CLI_JSON_FILE_OPEN_ERR;
    }

    fseek(fp,0,SEEK_END);
    lFileSize = ftell(fp);
    rewind(fp);
    fread(buffer, 1, lFileSize, fp);
    buffer[lFileSize] = '\0';
    fclose(fp);

    fileData << buffer;

    /** Parse the JSON file */
    if(!Json::parseFromStream(reader, fileData, &obj, &errs))
    {
        sprintf(s8DebugMsg,  "Invalid JSON config file. [error %d].\n(%s)",
                CLI_INVALID_JSON_FILE_ERR, errs.c_str());
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        s16Status = CLI_INVALID_JSON_FILE_ERR;
        return s16Status;
    }

    /** Parent node - DCA1000Config */
    root = obj.get("DCA1000Config",0);
    if(root.size() < 1)
    {
        sprintf(s8DebugMsg, "Invalid JSON config file - DCA1000Config node is missing. [error %d]", 
							CLI_JSON_DCA_CONFIG_NODE_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        s16Status = CLI_JSON_DCA_CONFIG_NODE_ERR;
        return s16Status;
    }

    node = root.get("ethernetConfig",0);
    if(node.size() < 1)
    {
        sprintf(s8DebugMsg, "Invalid JSON config file - ethernetConfig node is missing. [error %d]", 
							CLI_JSON_ETH_CONFIG_NODE_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        s16Status = CLI_JSON_ETH_CONFIG_NODE_ERR;
        return s16Status;
    }

    if(!node.isMember("DCA1000IPAddress"))
    {
        sprintf(s8DebugMsg, "Invalid JSON config file - Ethernet config - DCA1000IPAddress node is missing. [error %d]",CLI_JSON_ETH_DCA_IPADDR_NODE_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        s16Status = CLI_JSON_ETH_DCA_IPADDR_NODE_ERR;
        return s16Status;
    }

    strcpy(nodeData, root["ethernetConfig"]["DCA1000IPAddress"].asString().c_str());
    s16Status = validateIpAddress(nodeData);
    if(s16Status != SUCCESS_STATUS)
    {
        sprintf(s8DebugMsg, "Invalid Ethernet config DCA1000IPAddress value (%s). [error %d]" ,
            root["ethernetConfig"]["DCA1000IPAddress"].asString().c_str(),CLI_JSON_ETH_INVALID_DCA_IPADDR_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        s16Status = CLI_JSON_ETH_INVALID_DCA_IPADDR_ERR;
        return s16Status;
    }
    token = strtok(nodeData, ".");
    k = 0;
    while (token != NULL) {
        gsEthConfigMode.au8Dca1000IpAddr[k++] = atoi(token);
        token = strtok(NULL, ".");
    }

    /**  Config port number */
    if(!node.isMember("DCA1000ConfigPort"))
    {
        sprintf(s8DebugMsg, "Invalid JSON config file - Ethernet config - DCA1000ConfigPort node is missing. [error %d]",CLI_JSON_ETH_DCA_CONFIG_PORT_NODE_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        s16Status = CLI_JSON_ETH_DCA_CONFIG_PORT_NODE_ERR;
        return s16Status;
    }

    strcpy(nodeData,root["ethernetConfig"]["DCA1000ConfigPort"].asString().c_str());
    s16Status = validatePortNum(nodeData);
    if(s16Status != SUCCESS_STATUS)
    {
        sprintf(s8DebugMsg, "Invalid Ethernet config DCA1000ConfigPort value (%s). [error %d]" ,
                nodeData,CLI_JSON_ETH_INVALID_DCA_CONFIG_PORT_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        s16Status = CLI_JSON_ETH_INVALID_DCA_CONFIG_PORT_ERR;
        return s16Status;
    }
    gsEthConfigMode.u32ConfigPortNo = atoi(nodeData);

    /** Data port number */
    if(!node.isMember("DCA1000DataPort"))
    {
        sprintf(s8DebugMsg, "Invalid JSON config file - Ethernet config - DCA1000DataPort node is missing. [error %d]",CLI_JSON_ETH_DCA_DATA_PORT_NODE_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        s16Status = CLI_JSON_ETH_DCA_DATA_PORT_NODE_ERR;
        return s16Status;
    }

    strcpy(nodeData,root["ethernetConfig"]["DCA1000DataPort"].asString().c_str());
    s16Status = validatePortNum(nodeData);
    if(s16Status != SUCCESS_STATUS)
    {
        sprintf(s8DebugMsg, "Invalid Ethernet config DCA1000DataPort value (%s). [error %d]" ,
                nodeData,CLI_JSON_ETH_INVALID_DCA_DATA_PORT_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        s16Status = CLI_JSON_ETH_INVALID_DCA_DATA_PORT_ERR;
        return s16Status;
    }
    gsEthConfigMode.u32RecordPortNo = atoi(nodeData);

    s16Status = validatePortNumsForConflicts(gsEthConfigMode.u32RecordPortNo,
                                        gsEthConfigMode.u32ConfigPortNo);
    if(s16Status != SUCCESS_STATUS)
    {
        sprintf(s8DebugMsg, "Invalid Ethernet config port numbers - One of the data ports is same as config port(%d). [error %d]" ,
                gsEthConfigMode.u32ConfigPortNo, CLI_JSON_ETH_PORT_NUM_CONFLICT_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        s16Status = CLI_JSON_ETH_PORT_NUM_CONFLICT_ERR;
        return s16Status;
    }

    switch(u16CmdCode)
    {
    case CMD_CODE_START_RECORD:
        if(!root.isMember("dataLoggingMode"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - dataLoggingMode node is missing. [error %d]", 
								CLI_JSON_DATA_LOGGING_MODE_NODE_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_DATA_LOGGING_MODE_NODE_ERR;
            return s16Status;
        }
        strcpy(nodeData, root["dataLoggingMode"].asString().c_str());
        if(strcmp(nodeData, "raw") == 0)
            gsStartRecConfigMode.eConfigLogMode = RAW_MODE;
        else if(strcmp(nodeData, "multi") == 0)
            gsStartRecConfigMode.eConfigLogMode = MULTI_MODE;
        else
        {
            sprintf(s8DebugMsg, "Invalid dataLoggingMode value (%s). [error %d]", 
								nodeData, CLI_JSON_INVALID_DATALOGGING_MODE_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_INVALID_DATALOGGING_MODE_ERR;
            return s16Status;
        }

        if(!root.isMember("lvdsMode"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - lvdsMode node is missing. [error %d]", 
								CLI_JSON_LVDS_MODE_NODE_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_LVDS_MODE_NODE_ERR;
            return s16Status;
        }
        strcpy(nodeData, root["lvdsMode"].asString().c_str());
        if(strcmp(nodeData, "1") == 0)
            gsStartRecConfigMode.eLvdsMode = FOUR_LANE;
        else if(strcmp(nodeData, "2") == 0)
            gsStartRecConfigMode.eLvdsMode = TWO_LANE;
        else
        {
            sprintf(s8DebugMsg, "Invalid lvdsMode value (%s). [error %d]", nodeData, 
								CLI_JSON_INVALID_LVDS_MODE_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_INVALID_LVDS_MODE_ERR;
            return s16Status;
        }

        node = root.get("captureConfig",0);
        if(node.size() < 1)
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - captureConfig node is missing. [error %d]", 
								CLI_JSON_CAPTURE_CONFIG_NODE_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_CAPTURE_CONFIG_NODE_ERR;
            return s16Status;
        }

        if(!node.isMember("fileBasePath"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - fileBasePath node is missing. [error %d]", 
								CLI_JSON_REC_FILE_BASE_PATH_NODE_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_REC_FILE_BASE_PATH_NODE_ERR;
            return s16Status;
        }
        strcpy(gsStartRecConfigMode.s8FileBasePath,
               root["captureConfig"]["fileBasePath"].asString().c_str());

        if(!osalObj_Rec.IsValidDir(gsStartRecConfigMode.s8FileBasePath))
        {
            sprintf(s8DebugMsg, "Invalid fileBasePath value (%s). [error %d]",
                    gsStartRecConfigMode.s8FileBasePath,CLI_JSON_REC_INVALID_FILE_BASE_PATH_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            return CLI_JSON_REC_INVALID_FILE_BASE_PATH_ERR;
        }

        if(!node.isMember("filePrefix"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - filePrefix node is missing. [error %d]", 
								CLI_JSON_REC_FILE_PREFIX_NODE_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_REC_FILE_PREFIX_NODE_ERR;
            return s16Status;
        }
        strcpy(gsStartRecConfigMode.s8FilePrefix,
               root["captureConfig"]["filePrefix"].asString().c_str());
        if(strcmp(gsStartRecConfigMode.s8FilePrefix, "") == 0)
        {
            sprintf(s8DebugMsg, "FilePrefix value is empty. [error %d]", 
								CLI_JSON_REC_INVALID_FILE_PREFIX_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_REC_INVALID_FILE_PREFIX_ERR;
            return s16Status;
        }

        if(!node.isMember("maxRecFileSize_MB"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - maxRecFileSize_MB node is missing. [error %d]", 
								CLI_JSON_REC_MAX_FILE_SIZE_NODE_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_REC_MAX_FILE_SIZE_NODE_ERR;
            return s16Status;
        }
        memset(nodeData, '\0', MAX_PARAMS_LEN);
        strcpy(nodeData,
               root["captureConfig"]["maxRecFileSize_MB"].asString().c_str());
        s16Status = validateRecFileMaxsize(nodeData);
        if(s16Status != SUCCESS_STATUS)
        {
            sprintf(s8DebugMsg, "Invalid maxRecFileSize_MB value (%s). [error %d]",
                    nodeData,CLI_JSON_REC_INVALID_MAX_FILE_SIZE_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_REC_INVALID_MAX_FILE_SIZE_ERR;
            return s16Status;
        }
        gsStartRecConfigMode.u16MaxRecFileSize = atoi(nodeData);

        /** Sequence number   */
        if(!node.isMember("sequenceNumberEnable"))
        {
            sprintf( s8DebugMsg,"Invalid JSON config file - sequenceNumberEnable node is missing. [error %d]",
                     CLI_JSON_REC_EN_SEQ_NUM_NODE_ERR);
            s16Status = CLI_JSON_REC_EN_SEQ_NUM_NODE_ERR;
            return s16Status;
        }

        memset(nodeData, '\0', MAX_PARAMS_LEN);
        strcpy(nodeData,
               root["captureConfig"]["sequenceNumberEnable"].asString().c_str());
        if(strcmp(nodeData, "1") == 0)
        {
            gsStartRecConfigMode.bSequenceNumberEnable = true;
        }
        else if(strcmp(nodeData, "0") == 0)
        {
            gsStartRecConfigMode.bSequenceNumberEnable = false;
        }
        else
        {
            sprintf(s8DebugMsg, "Invalid sequenceNumberEnable value (%s). [error %d]",
                    nodeData,CLI_JSON_REC_INVALID_EN_SEQ_NUM_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_REC_INVALID_EN_SEQ_NUM_ERR;
            return s16Status;
        }

        if(!node.isMember("captureStopMode"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - captureStopMode node is missing. [error %d]", 
								CLI_JSON_REC_CAPTURE_STOP_MODE_NODE_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_REC_CAPTURE_STOP_MODE_NODE_ERR;
            return s16Status;
        }
        memset(nodeData, '\0', MAX_PARAMS_LEN);
        strcpy(nodeData,
               root["captureConfig"]["captureStopMode"].asString().c_str());
        if(strcmp(nodeData, "bytes") == 0)
        {
            gsStartRecConfigMode.eRecordStopMode = BYTES;
            if(!node.isMember("bytesToCapture"))
            {
                sprintf( s8DebugMsg, "Invalid JSON config file - bytesToCapture node is missing. [error %d]", 
									CLI_JSON_REC_BYTE_CAPTURE_NODE_ERR);
                WRITE_TO_CONSOLE_REC(s8DebugMsg);
                s16Status = CLI_JSON_REC_BYTE_CAPTURE_NODE_ERR;
                return s16Status;
            }
            memset(nodeData, '\0', MAX_PARAMS_LEN);
            strcpy(nodeData,
                   root["captureConfig"]["bytesToCapture"].asString().c_str());
            s16Status = validateBytesRecStopConfig(nodeData,
                                            gsStartRecConfigMode.eLvdsMode);
            if(s16Status != SUCCESS_STATUS)
            {
                sprintf(s8DebugMsg, "Invalid bytesToCapture value (%s). [error %d]",
                        nodeData,CLI_JSON_REC_INVALID_BYTE_CAPTURE_ERR);
                WRITE_TO_CONSOLE_REC(s8DebugMsg);
                s16Status = CLI_JSON_REC_INVALID_BYTE_CAPTURE_ERR;
                return s16Status;
            }
            gsStartRecConfigMode.u32BytesToCapture = atoi(nodeData);
        }
        else if(strcmp(nodeData, "duration") == 0)
        {
            gsStartRecConfigMode.eRecordStopMode = DURATION;
            if(!node.isMember("durationToCapture_ms"))
            {
                sprintf(s8DebugMsg, "Invalid JSON config file - durationToCapture_ms node is missing. [error %d]",CLI_JSON_REC_DUR_CAPTURE_NODE_ERR);
                WRITE_TO_CONSOLE_REC(s8DebugMsg);
                s16Status = CLI_JSON_REC_DUR_CAPTURE_NODE_ERR;
                return s16Status;
            }
            memset(nodeData, '\0', MAX_PARAMS_LEN);
            strcpy(nodeData,
                   root["captureConfig"]["durationToCapture_ms"].asString().c_str());
            s16Status = validateDurationRecStopConfig(nodeData);
            if(s16Status != SUCCESS_STATUS)
            {
                sprintf(s8DebugMsg, "Invalid durationToCapture_ms value (%s). [error %d]",
                        nodeData,CLI_JSON_REC_INVALID_DUR_CAPTURE_ERR);
                WRITE_TO_CONSOLE_REC(s8DebugMsg);
                s16Status = CLI_JSON_REC_INVALID_DUR_CAPTURE_ERR;
                return s16Status;
            }
            gsStartRecConfigMode.u32DurationToCapture = atoi(nodeData);

        }
        else if(strcmp(nodeData, "frames") == 0)
        {
            if(gsStartRecConfigMode.eConfigLogMode != MULTI_MODE)
            {
                sprintf(s8DebugMsg, "Record stopping using number of frames is valid only in raw mode. [error %d]",CLI_JSON_REC_CAPTURE_STOP_MODE_NODE_ERR);
                WRITE_TO_CONSOLE_REC(s8DebugMsg);
                s16Status = CLI_JSON_REC_CAPTURE_STOP_MODE_NODE_ERR;
                return s16Status;
            }

            gsStartRecConfigMode.eRecordStopMode = FRAMES;
            if(!node.isMember("framesToCapture"))
            {
                sprintf(s8DebugMsg,"Invalid JSON config file - framesToCapture node is missing. [error %d]", 
									CLI_JSON_REC_FRAME_CAPTURE_NODE_ERR);
                WRITE_TO_CONSOLE_REC(s8DebugMsg);
                s16Status = CLI_JSON_REC_FRAME_CAPTURE_NODE_ERR;
                return s16Status;
            }
            memset(nodeData, '\0', MAX_PARAMS_LEN);
            strcpy(nodeData,
                   root["captureConfig"]["framesToCapture"].asString().c_str());
            s16Status = validateFramesRecStopConfig(nodeData);
            if(s16Status != SUCCESS_STATUS)
            {
                sprintf(s8DebugMsg, "Invalid framesToCapture value (%s). [error %d]",
                        nodeData,CLI_JSON_REC_INVALID_FRAME_CAPTURE_ERR);
                WRITE_TO_CONSOLE_REC(s8DebugMsg);
                s16Status = CLI_JSON_REC_INVALID_FRAME_CAPTURE_ERR;
                return s16Status;

            }
            gsStartRecConfigMode.u32FramesToCapture = atoi(nodeData);

        }
        else if(strcmp(nodeData, "infinite") == 0)
        {
            gsStartRecConfigMode.eRecordStopMode = NON_STOP;
        }
        else
        {
            sprintf(s8DebugMsg, "Invalid captureStopMode value (%s). [error %d]",
                    nodeData,CLI_JSON_REC_INVALID_CAPTURE_STOP_MODE_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_REC_INVALID_CAPTURE_STOP_MODE_ERR;
            return s16Status;
        }

        /** Read MSB togglig enable/disable status */
        node = root.get("dataFormatConfig",0);
        if(node.size() < 1)
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - dataFormatConfig node is missing. [error %d]",
                    CLI_JSON_DATA_FORMAT_CONFIG_NODE_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            s16Status = CLI_JSON_DATA_FORMAT_CONFIG_NODE_ERR;
            return s16Status;
        }
        if(!node.isMember("MSBToggle"))
        {
            /** MSB toggling is disabled if the field is missig in JSON file */
            gsStartRecConfigMode.bMsbToggleEnable = false;
        }
        else
        {
            strcpy(nodeData,
                   root["dataFormatConfig"]["MSBToggle"].asString().c_str());
            if(strcmp(nodeData, "0") == 0)
            {
                gsStartRecConfigMode.bMsbToggleEnable = false;
            }
            else if(strcmp(nodeData, "1") == 0)
            {
                gsStartRecConfigMode.bMsbToggleEnable = true;
            }
            else
            {
                sprintf(s8DebugMsg, "Invalid MsbToggleEnable status (%s). [error %d]",
                        nodeData, CLI_JSON_REC_INVALID_EN_MSB_TOGGLE_ERR);
                WRITE_TO_CONSOLE_REC(s8DebugMsg);
                s16Status = CLI_JSON_REC_INVALID_EN_MSB_TOGGLE_ERR;
                return s16Status;
            }
        }
        if(!node.isMember("reorderEnable"))
        {
            /** Reordering is enabled if the field is missig in JSON file */
            gsStartRecConfigMode.bReorderEnable = true;
        }
        else
        {
            strcpy(nodeData,
                   root["dataFormatConfig"]["reorderEnable"].asString().c_str());
            if(strcmp(nodeData, "0") == 0)
            {
                gsStartRecConfigMode.bReorderEnable = false;
            }
            else if(strcmp(nodeData, "1") == 0)
            {
                gsStartRecConfigMode.bReorderEnable = true;
            }
            else
            {
                sprintf(s8DebugMsg, "Invalid bReorderEnable status (%s). [error %d]",
                        nodeData, CLI_JSON_REC_INVALID_EN_REORDER_ERR);
                WRITE_TO_CONSOLE_REC(s8DebugMsg);
                s16Status = CLI_JSON_REC_INVALID_EN_REORDER_ERR;
                return s16Status;
            }
        }

        break;
    }

    return s16Status;
}

/** @fn bool IsRecordProcRunning_Rec()
 * @brief This function is to check whether the record process is runnning
 * @return boolean value
 */
bool IsRecordProcRunning_Rec()
{
    SHM_PROC_STATES procStates;
    SINT8 s8DebugMsg[MAX_NAME_LEN];

    SINT32 s32Status = osalObj_Rec.QueryRecordProcStatus(gsEthConfigMode.u32ConfigPortNo,
                                                     &procStates);
    if(SUCCESS_STATUS == s32Status)
    {
        // Parse the command status
        if(!((procStates.s32CommandStatus == STS_CLI_REC_PROC_STOPPED) ||
            (procStates.s32CommandStatus == STS_CLI_REC_PROC_START_FAILED) ||
             (procStates.s32CommandStatus == STS_CLI_REC_PROC_STOP_FAILED)))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        if(s32Status == CLI_SHM_NOT_AVAIL_ERR)
        {
         //   WRITE_TO_CONSOLE_REC( "No record process is running. [error %d]",
            //CLI_SHM_NOT_AVAIL_ERR);
        }
        else
        {
            sprintf(s8DebugMsg, "Unable to read record process states. [error %d]",
                   s32Status);
            WRITE_TO_LOG_FILE_REC(s8DebugMsg);
        }

        return false;
    }
}

/** @fn void updateRecInlineStatus()
 * @brief This function is to update inline status in shared memory <!--
 * --> when record is in progress
 */
void updateRecInlineStatus()
{
    while(bRecProgress)
    {
        osalObj_Rec.WaitForSignal(&sgnInlineStsUpdateWaitEvent, NON_STOP);

        bReady = false;

        if(bRecProgress)
        {
            osalObj_Rec.WriteRecProcInlineStats(gsEthConfigMode.u32ConfigPortNo,
                    &gInlineStats, gbOutOfSeqSetFlag, gu8DataIndex);
        }
        bReady = true;
    }
}

/** @fn SINT32 main(SINT32 argc, SINT8* argv[])
 * @brief This function is the main function to handle start and stop <!--
 * --> record, validation of JSON file
 * @param [in] argc [SINT32] - Number of arguments
 * @param [in] argv [SINT8 *] - Arguments array
 * @return SINT32 value
 */
SINT32 main(SINT32 argc, SINT8* argv[])
{
    /** Command line argument  1 - Command name */
    SINT8 s8Command[MAX_NAME_LEN];

    /** Command line argument  2 - JSON file name */
    SINT8 s8CommandArg[MAX_NAME_LEN];

    SINT8 s8DebugMsg[MAX_NAME_LEN];
    SINT32 s32CliStatus = SUCCESS_STATUS;
    SINT8 s8CliVersion[MAX_VERSION_BUF_LEN];
    strcpy(s8CliVersion, CLI_REC_VERSION);

    /** Initialize waiting events             */
    osalObj_Rec.InitEvent(&sgnRecStopWaitEvent);
    osalObj_Rec.InitEvent(&sgnInlineStsUpdateWaitEvent);

    if(argc < 2)
    {
        ListOfCmds_Rec();
#ifndef CLI_TESTING_MODE
        return CLI_NO_ARG_ERR;
#endif
    }
    else
    {
        strcpy(s8Command, argv[1]);
        if(argc > 2)
            strcpy(s8CommandArg, argv[2]);
        if(argc > 3)
        {
            if(strcmp(argv[3], CMD_QUIET_MODE_CLI_APP) == 0)
                gbCliQuietMode = true;
            else
                gbCliQuietMode = false;
        }
    }


#ifdef CLI_TESTING_MODE
    SINT8 s8Version[MAX_VERSION_BUF_LEN];
    if(argc < 2)
    {
        printf("\nEnter the command : ");
        scanf("%s", s8Command);

        if((strcmp(s8Command, CMD_HELP_CLI_APP) == 0) ||
                (strcmp(s8Command, CMD_HELP_S_CLI_APP) == 0))
        {
            ListOfCmds_Rec();
            return SUCCESS_STATUS;
        }
        else if(strcmp(s8Command, CMD_READ_DLL_VER) == 0)
        {
            /** API Call - Read API DLL version                              */
            memset(s8Version, '\0', MAX_VERSION_BUF_LEN);
            WRITE_TO_LOG_FILE_REC("Read DLL Verison Command (req)");
            if(ReadRFDCCard_DllVersion(s8Version) == SUCCESS_STATUS)
            {
                sprintf(s8DebugMsg, "DLL Version : %s", s8Version);
                WRITE_TO_CONSOLE_REC(s8DebugMsg);
                WRITE_TO_LOG_FILE_REC(s8DebugMsg);
                return SUCCESS_STATUS;
            }
            else
            {
                sprintf(s8DebugMsg, "Not able to read DLL Version. error[%d]", 
									CLI_READ_DLL_VERSION_ERR);
                WRITE_TO_CONSOLE_REC(s8DebugMsg);
                WRITE_TO_LOG_FILE_REC(s8DebugMsg);
                return CLI_READ_DLL_VERSION_ERR;
            }
        }
        else if(strcmp(s8Command, CMD_READ_CLI_VER) == 0)
        {
            WRITE_TO_LOG_FILE_REC("Read CLI Verison Command (req)");
            sprintf(s8DebugMsg, "Record CLI Version : %s", s8CliVersion);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            WRITE_TO_LOG_FILE_REC(s8DebugMsg);
            return SUCCESS_STATUS;
        }
        else
        {
            printf("\nEnter the argument : ");
            scanf("%s", s8CommandArg);
        }
    }
#endif

    if((strcmp(s8Command, CMD_HELP_CLI_APP) == 0) ||
            (strcmp(s8Command, CMD_HELP_S_CLI_APP) == 0))
    {
        ListOfCmds_Rec();
        return SUCCESS_STATUS;
    }
    else if(strcmp(s8Command, CMD_READ_DLL_VER) == 0)
    {
        SINT8 s8Version[MAX_VERSION_BUF_LEN];
        /* API Call - Read API DLL version                                   */
        memset(s8Version, '\0', MAX_VERSION_BUF_LEN);
        WRITE_TO_LOG_FILE_REC("Read DLL Verison Command (req)");
        if(ReadRFDCCard_DllVersion(s8Version) == SUCCESS_STATUS)
        {
            sprintf(s8DebugMsg, "DLL Version : %s", s8Version);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            WRITE_TO_LOG_FILE_REC(s8DebugMsg);
            return SUCCESS_STATUS;
        }
        else
        {
            sprintf(s8DebugMsg, "Not able to read DLL Version. error[%d]",
                    CLI_READ_DLL_VERSION_ERR);
            WRITE_TO_CONSOLE_REC(s8DebugMsg);
            WRITE_TO_LOG_FILE_REC(s8DebugMsg);
            return CLI_READ_DLL_VERSION_ERR;
        }
    }
    else if(strcmp(s8Command, CMD_READ_CLI_VER) == 0)
    {
        WRITE_TO_LOG_FILE_REC( "Read CLI Verison Command (req)");
        sprintf(s8DebugMsg, "Record CLI Version : %s", s8CliVersion);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        WRITE_TO_LOG_FILE_REC(s8DebugMsg);
        return SUCCESS_STATUS;
    }

    s32CliStatus = ValidateJsonFileData_Rec(s8CommandArg, CMD_CODE_START_RECORD);
    if(s32CliStatus < 0)
        return s32CliStatus;

    if(IsRecordProcRunning_Rec())
    {
        WRITE_TO_LOG_FILE_REC(s8Command);
        WRITE_TO_CONSOLE_REC( "Stop the already running process.");
        WRITE_TO_LOG_FILE_REC("Stop the already running process.");
        return CLI_SHM_REC_PROC_RUNNING_STS;
    }
    else
    {
        SHM_PROC_STATES procStates;

        if(osalObj_Rec.QueryRecordProcStatus(gsEthConfigMode.u32ConfigPortNo,
                &procStates) == CLI_SHM_NOT_AVAIL_ERR)
        {
            osalObj_Rec.CreateShm(gsEthConfigMode.u32ConfigPortNo);
        }
    }

    /* API Call - Register UI callback                                      */
    s32CliStatus = StatusRFDCCard_EventRegister(
                                (EVENT_HANDLER)cmdRecvCallback);
    if(s32CliStatus != SUCCESS_STATUS)
    {
        WRITE_TO_LOG_FILE_REC("Callback function register (req)");
        sprintf(s8DebugMsg, "Registering Callback function failed (Cmd Sts & Async). error[%d]",
                CLI_CMD_CALLBACK_REG_FAILED_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        WRITE_TO_LOG_FILE_REC(s8DebugMsg);
        return CLI_CMD_CALLBACK_REG_FAILED_ERR;
    }
    s32CliStatus = RecInlineProcStats_EventRegister(
                                (INLINE_PROC_HANDLER)inlineStatsCallback);
    if(s32CliStatus != SUCCESS_STATUS)
    {
        WRITE_TO_LOG_FILE_REC("Callback function register (req)");
        sprintf(s8DebugMsg, "Registering Callback function failed (Inline status). error[%d]",
                CLI_INLINE_CALLBACK_REG_FAILED_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        WRITE_TO_LOG_FILE_REC(s8DebugMsg);
        return CLI_INLINE_CALLBACK_REG_FAILED_ERR;
    }

    /** API Call - Ethernet connection                                       */
    s32CliStatus = ConnectRFDCCard_RecordMode(gsEthConfigMode);
    if(s32CliStatus != SUCCESS_STATUS)
    {
        WRITE_TO_LOG_FILE_REC("Ethernet connection (req)");
        sprintf(s8DebugMsg, "Ethernet connection failed. [error %d]", 
								CLI_ETH_CONNECT_FAIL_ERR);
        WRITE_TO_CONSOLE_REC(s8DebugMsg);
        WRITE_TO_LOG_FILE_REC(s8DebugMsg);
        DisconnectRFDCCard_RecordMode();
        return CLI_ETH_CONNECT_FAIL_ERR;
    }

    /** Timestamp logging                                                */
    WRITE_TO_LOG_FILE_REC("Start Record Command (req)");

    /** Map to shared memory        */
    osalObj_Rec.MapShm(gsEthConfigMode.u32ConfigPortNo);

    /** Reset shared memory         */
    osalObj_Rec.WriteDefaultRecordProcStatus(gsEthConfigMode.u32ConfigPortNo);

    /** Write shared memory         */
    osalObj_Rec.WriteRecordProcStatus(gsEthConfigMode.u32ConfigPortNo,
                                  STS_CLI_REC_PROC_START_INIT);

    bStopCmdSent = false;

    /** API Call - Start Record                                     */
    s32CliStatus = StartRecordData(gsStartRecConfigMode);

    /** Handling command response                                        */
    DecodeCommandStatus_Rec(s32CliStatus, "Start Record");

    /** Write shared memory     */
    if(s32CliStatus == SUCCESS_STATUS)
    {
        osalObj_Rec.WriteRecordProcStatus(gsEthConfigMode.u32ConfigPortNo,
                                          STS_CLI_REC_PROC_IS_IN_PROG);

        bRecProgress = true;

        /** Updation of record process stats in shared memory is handled
         * in a thread. Whenever the callback is recieved from DLL, stats will
         * be updated.
         */
        std::thread tUpdateInline([&] { updateRecInlineStatus(); });
        tUpdateInline.detach();

        /** Record process will wait till the stop command is recieved  */
        osalObj_Rec.WaitForSignal(&sgnRecStopWaitEvent, NON_STOP);
    }
    else
    {
        osalObj_Rec.WriteRecordProcStatus(gsEthConfigMode.u32ConfigPortNo,
                                          STS_CLI_REC_PROC_START_FAILED);
    }

    /** Reset record status             */
    bRecProgress = false;
    osalObj_Rec.SignalEvent(&sgnInlineStsUpdateWaitEvent);

    /** Unmap from shared memory        */
    osalObj_Rec.UnmapShm();

    /** Deinitialize waiting events             */
    osalObj_Rec.DeInitEvent(&sgnRecStopWaitEvent);
    osalObj_Rec.DeInitEvent(&sgnInlineStsUpdateWaitEvent);

    DisconnectRFDCCard_RecordMode();

    return s32CliStatus;
}

