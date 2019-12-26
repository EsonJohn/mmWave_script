/**
 * @file cli_control_main.cpp
 *
 * @author JP
 *
 * @version 0.5
 *
 * @brief This file supports configuration of DCA1000EVM and start recording.
 * This file handles the CLI Control tool command request and responses
 * in DCA1000EVM system using UDP packets.
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

/** Macro to support ULONG64 data type specifier */
#ifdef _WIN32
#define __USE_MINGW_ANSI_STDIO 1
#endif

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "../Common/globals.h"
#include "../Common/Osal_Utils/osal.h"
#include "../Common/Validate_Utils/validate_params.h"
#include "../Common/errcodes.h"
#include "../Common/Json_Utils/dist/json/json.h"
#include "../Common/Json_Utils/dist/json/json-forwards.h"

///****************
/// Variable declarations
///****************

/** Quiet mode enable/disable                   */
bool gbCliQuietMode = false;

/** Osal class object                           */
osal osalObj;

/** FPGA config mode stucture object            */
strFpgaConfigMode gsFpgaConfigMode;

/** Ethernet config mode stucture object        */
strEthConfigMode gsEthConfigMode;

/** Ethernet update mode stucture object        */
strEthConfigMode gsEthUpdateMode;

/** Record config mode stucture object          */
strRecConfigMode gsRecConfigMode;

/** Start record config mode stucture object    */
strStartRecConfigMode gsStartRecConfigMode;

/** JSON file data                              */
std::stringstream fileData;

/** @fn void WRITE_TO_CONSOLE(const SINT8 *msg)
 * @brief This function is to write the CLI messages in the console  <!--
 * --> if quiet mode is not enabled
 * @param [in] msg [const SINT8 *] - Message to display
 * @return SINT32 value
 */
void WRITE_TO_CONSOLE(const SINT8 *msg)
{
    if(!gbCliQuietMode)
    {
        printf("\n%s\n",msg);
    }
}

/** @fn void WRITE_TO_LOG_FILE(const SINT8 *s8Msg)
 * @brief This function is to write the CLI messages in the logfile
 * @param [in] s8Msg [const SINT8 *] - Message to display
 */
void WRITE_TO_LOG_FILE
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

/** @fn void ListOfCmds()
 * @brief This function is to list commands supported by CLI Control tool
 */
void ListOfCmds()
{
    printf("Cli_Control [options]");
    printf("\nOptions:");
    printf("\n%s\t\t\t\t%s", CMD_CONFIG_FPGA, "Configure FPGA");
    printf("\n%s\t\t\t\t%s", CMD_CONFIG_EEPROM, "Update EEPROM");
    printf("\n%s\t\t\t%s", CMD_RESET_FPGA, "Reset FPGA");
    printf("\n%s\t\t\t%s", CMD_RESET_AR_DEV, "Reset AR Device");
    printf("\n%s\t\t\t%s", CMD_START_RECORD, "Start Record");
    printf("\n%s\t\t\t%s", CMD_STOP_RECORD, "Stop Record");
    printf("\n%s\t\t\t\t%s", CMD_CONFIG_RECORD, "Configure Record delay");
    printf("\n%s\t\t\t%s", CMD_READ_DLL_VER, "Read DLL version");
    printf("\n%s\t\t\t%s", CMD_READ_CLI_VER, "Read CLI_Control tool version");
    printf("\n%s\t\t\t%s", CMD_READ_FPGA_VER, "Read FPGA version");
    printf("\n%s\t\t\t%s", CMD_QUERY_CLI_PROC_STATUS,
                        "Read status of record process");
    printf("\n%s\t\t%s", CMD_QUERY_SYSTEM_ALIVENESS,
                        "DCA1000EVM System aliveness");
    printf("\n%s\t\t\t\t%s", CMD_HELP_S_CLI_APP, "List of commands supported");
    printf("\n%s\t\t\t\t%s", CMD_QUIET_MODE_CLI_APP,
                        "Quiet mode - No status display in the console\n");
}

/** @fn void cli_DisconnectRFDCCard(UINT16 u16CmdCode)
 * @brief This function is to disconnect from DCA1000EVM sysytem
 * @param [in] u16CmdCode [UINT16] - Command code
 */
void cli_DisconnectRFDCCard(UINT16 u16CmdCode)
{
    if(u16CmdCode == CMD_CODE_CLI_ASYNC_RECORD_STOP)
    {
        DisconnectRFDCCard_AsyncCommandMode();
    }
    else
    {
        DisconnectRFDCCard_ConfigMode();
    }
}

/** @fn SINT32 ValidateJsonFileData(SINT8 *configFile, UINT16 u16CmdCode)
 * @brief This function is to read and validate JSON file data <!--
 * --> corresponding to the command executed
 * @param [in] configFile [SINT8 *] - JSON config file
 * @param [in] u16CmdCode [UINT16] - Command code
 * @return SINT32 value
 */
SINT32 ValidateJsonFileData(SINT8 *configFile, UINT16 u16CmdCode)
{
    SINT16 s16Status = 0;
    SINT8 buffer[MAX_FILE_UPDATE_LEN];
    SINT8 s8DebugMsg[MAX_NAME_LEN];
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
        WRITE_TO_CONSOLE(s8DebugMsg);
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
        WRITE_TO_CONSOLE(s8DebugMsg);
        s16Status = CLI_INVALID_JSON_FILE_ERR;
        return s16Status;
    }

    /** Parent node - DCA1000Config */
    root = obj.get("DCA1000Config",0);
    if(root.size() < 1)
    {
        sprintf(s8DebugMsg, "Invalid JSON config file - DCA1000Config node is missing. [error %d]",CLI_JSON_DCA_CONFIG_NODE_ERR);
        WRITE_TO_CONSOLE(s8DebugMsg);
        s16Status = CLI_JSON_DCA_CONFIG_NODE_ERR;
        return s16Status;
    }

    node = root.get("ethernetConfig",0);
    if(node.size() < 1)
    {
        sprintf(s8DebugMsg, "Invalid JSON config file - ethernetConfig node is missing. [error %d]",CLI_JSON_ETH_CONFIG_NODE_ERR);
        WRITE_TO_CONSOLE(s8DebugMsg);
        s16Status = CLI_JSON_ETH_CONFIG_NODE_ERR;
        return s16Status;
    }

    if(!node.isMember("DCA1000IPAddress"))
    {
        sprintf(s8DebugMsg, "Invalid JSON config file - Ethernet config - DCA1000IPAddress node is missing. [error %d]",CLI_JSON_ETH_DCA_IPADDR_NODE_ERR);
        WRITE_TO_CONSOLE(s8DebugMsg);
        s16Status = CLI_JSON_ETH_DCA_IPADDR_NODE_ERR;
        return s16Status;
    }

    strcpy(nodeData, root["ethernetConfig"]["DCA1000IPAddress"].asString().c_str());
    s16Status = validateIpAddress(nodeData);
    if(s16Status != SUCCESS_STATUS)
    {
        sprintf(s8DebugMsg, "Invalid Ethernet config DCA1000IPAddress value (%s). [error %d]" ,
            root["ethernetConfig"]["DCA1000IPAddress"].asString().c_str(),CLI_JSON_ETH_INVALID_DCA_IPADDR_ERR);
        WRITE_TO_CONSOLE(s8DebugMsg);
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
        WRITE_TO_CONSOLE(s8DebugMsg);
        s16Status = CLI_JSON_ETH_DCA_CONFIG_PORT_NODE_ERR;
        return s16Status;
    }

    strcpy(nodeData,root["ethernetConfig"]["DCA1000ConfigPort"].asString().c_str());
    s16Status = validatePortNum(nodeData);
    if(s16Status != SUCCESS_STATUS)
    {
        sprintf(s8DebugMsg, "Invalid Ethernet config DCA1000ConfigPort value (%s). [error %d]" ,
                nodeData,CLI_JSON_ETH_INVALID_DCA_CONFIG_PORT_ERR);
        WRITE_TO_CONSOLE(s8DebugMsg);
        s16Status = CLI_JSON_ETH_INVALID_DCA_CONFIG_PORT_ERR;
        return s16Status;
    }
    gsEthConfigMode.u32ConfigPortNo = atoi(nodeData);

    /** Data port number */
    if(!node.isMember("DCA1000DataPort"))
    {
        sprintf(s8DebugMsg, "Invalid JSON config file - Ethernet config - DCA1000DataPort node is missing. [error %d]",CLI_JSON_ETH_DCA_DATA_PORT_NODE_ERR);
        WRITE_TO_CONSOLE(s8DebugMsg);
        s16Status = CLI_JSON_ETH_DCA_DATA_PORT_NODE_ERR;
        return s16Status;
    }

    strcpy(nodeData,root["ethernetConfig"]["DCA1000DataPort"].asString().c_str());
    s16Status = validatePortNum(nodeData);
    if(s16Status != SUCCESS_STATUS)
    {
        sprintf(s8DebugMsg, "Invalid Ethernet config DCA1000DataPort value (%s). [error %d]" ,
                nodeData,CLI_JSON_ETH_INVALID_DCA_DATA_PORT_ERR);
        WRITE_TO_CONSOLE(s8DebugMsg);
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
        WRITE_TO_CONSOLE(s8DebugMsg);
        s16Status = CLI_JSON_ETH_PORT_NUM_CONFLICT_ERR;
        return s16Status;
    }

    switch(u16CmdCode)
    {
    case CMD_CODE_CONFIG_FPGA:
        if(!root.isMember("dataLoggingMode"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - dataLoggingMode node is missing. [error %d]",
                        CLI_JSON_DATA_LOGGING_MODE_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_DATA_LOGGING_MODE_NODE_ERR;
            return s16Status;
        }
        strcpy(nodeData, root["dataLoggingMode"].asString().c_str());

         if(strcmp(nodeData, "raw") == 0)
            gsFpgaConfigMode.eLogMode = RAW_MODE;
        else if(strcmp(nodeData, "multi") == 0)
            gsFpgaConfigMode.eLogMode = MULTI_MODE;
        else
        {
            sprintf(s8DebugMsg, "Invalid dataLoggingMode value (%s). [error %d]", 
								nodeData, CLI_JSON_INVALID_DATALOGGING_MODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_INVALID_DATALOGGING_MODE_ERR;
            return s16Status;
        }

        if(!root.isMember("dataTransferMode"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - dataTransferMode node is missing. [error %d]",CLI_JSON_DATA_TRANSFER_MODE_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_DATA_TRANSFER_MODE_NODE_ERR;
            return s16Status;
        }
        strcpy(nodeData, root["dataTransferMode"].asString().c_str());
        if(strcmp(nodeData, "LVDSCapture") == 0)
            gsFpgaConfigMode.eDataXferMode = CAPTURE;
        else if(strcmp(nodeData, "playback") == 0)
            gsFpgaConfigMode.eDataXferMode = PLAYBACK;
        else
        {
            sprintf(s8DebugMsg, "Invalid dataTransferMode value (%s). [error %d]", nodeData,CLI_JSON_INVALID_DATA_TRANSFER_MODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_INVALID_DATA_TRANSFER_MODE_ERR;
            return s16Status;
        }

        if(!root.isMember("dataCaptureMode"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - dataCaptureMode node is missing. [error %d]",CLI_JSON_DATA_CAPTURE_MODE_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_DATA_CAPTURE_MODE_NODE_ERR;
            return s16Status;
        }
        strcpy(nodeData, root["dataCaptureMode"].asString().c_str());
        if(strcmp(nodeData, "SDCardStorage") == 0)
            gsFpgaConfigMode.eDataCaptureMode = SD_STORAGE;
        else if(strcmp(nodeData, "ethernetStream") == 0)
            gsFpgaConfigMode.eDataCaptureMode = ETH_STREAM;
        else
        {
            sprintf(s8DebugMsg, "Invalid datacapturemode value (%s). [error %d]", nodeData,CLI_JSON_INVALID_DATA_CAPTURE_MODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_INVALID_DATA_CAPTURE_MODE_ERR;
            return s16Status;
        }

        if(!root.isMember("lvdsMode"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - lvdsMode node is missing. [error %d]",CLI_JSON_LVDS_MODE_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_LVDS_MODE_NODE_ERR;
            return s16Status;
        }
        strcpy(nodeData, root["lvdsMode"].asString().c_str());
        if(strcmp(nodeData, "1") == 0)
            gsFpgaConfigMode.eLvdsMode = FOUR_LANE;
        else if(strcmp(nodeData, "2") == 0)
            gsFpgaConfigMode.eLvdsMode = TWO_LANE;
        else
        {
            sprintf(s8DebugMsg, "Invalid lvdsMode value (%s). [error %d]", 
								nodeData, CLI_JSON_INVALID_LVDS_MODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_INVALID_LVDS_MODE_ERR;
            return s16Status;
        }

        if(!root.isMember("dataFormatMode"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - dataFormatMode node is missing. [error %d]",CLI_JSON_DATA_FORMAT_MODE_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_DATA_FORMAT_MODE_NODE_ERR;
            return s16Status;
        }
        strcpy(nodeData, root["dataFormatMode"].asString().c_str());
        if(strcmp(nodeData, "1") == 0)
            gsFpgaConfigMode.eDataFormatMode = BIT12;
        else if(strcmp(nodeData, "2") == 0)
            gsFpgaConfigMode.eDataFormatMode = BIT14;
        else if(strcmp(nodeData, "3") == 0)
           gsFpgaConfigMode.eDataFormatMode = BIT16;
        else
        {
            sprintf(s8DebugMsg, "Invalid dataFormatMode value (%s). [error %d]", nodeData,CLI_JSON_INVALID_DATA_FORMAT_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status =CLI_JSON_INVALID_DATA_FORMAT_NODE_ERR;
            return s16Status;
        }

        gsFpgaConfigMode.u8Timer = FPGA_CONFIG_DEFAULT_TIMER; // by default

        break;
    case CMD_CODE_CONFIG_EEPROM:
        node = root.get("ethernetConfigUpdate",0);

        if(node.size() < 1)
        {
            sprintf(s8DebugMsg,"Invalid JSON config file - ethernetConfigUpdate node is missing. [error %d]",CLI_JSON_EEPROM_CONFIG_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_EEPROM_CONFIG_NODE_ERR;
            return s16Status;
        }

        if(!node.isMember("systemIPAddress"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - Ethernet Update - systemIPAddress node is missing. [error %d]",CLI_JSON_EEPROM_SYS_IPADDR_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status =CLI_JSON_EEPROM_SYS_IPADDR_NODE_ERR;
            return s16Status;
        }
        strcpy(nodeData, root["ethernetConfigUpdate"]["systemIPAddress"].asString().c_str());
        s16Status = validateIpAddress(nodeData);
        if(s16Status != SUCCESS_STATUS)
        {
            sprintf(s8DebugMsg, "Invalid Ethernet Update systemIPAddress value (%s). [error %d]" ,
                    root["ethernetConfigUpdate"]["systemIPAddress"].asString().c_str(),CLI_JSON_EEPROM_INVALID_SYS_IPADDR_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_EEPROM_INVALID_SYS_IPADDR_ERR;
            return s16Status;
        }

        token = strtok(nodeData, ".");
        k = 0;
        while (token != NULL)
        {
            gsEthUpdateMode.au8PcIpAddr[k++] = atoi(token);
            token = strtok(NULL, ".");
         }

        /** FPGA IP address */
        if(!node.isMember("DCA1000IPAddress"))
        {
            sprintf(s8DebugMsg,
                    "Invalid JSON config file - Ethernet Update - DCA1000IPAddress node is missing. [error %d]",CLI_JSON_EEPROM_DCA_IPADDR_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_EEPROM_DCA_IPADDR_NODE_ERR;
            return s16Status;
        }

        strcpy(nodeData, root["ethernetConfigUpdate"]["DCA1000IPAddress"].asString().c_str());
        s16Status = validateIpAddress(nodeData);
        if(s16Status != SUCCESS_STATUS)
        {
            sprintf(s8DebugMsg, "Invalid Ethernet Update DCA1000IPAddress value (%s). [error %d]" ,
                    root["ethernetConfigUpdate"]["DCA1000IPAddress"].asString().c_str(),CLI_JSON_EEPROM_INVALID_DCA_IPADDR_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_EEPROM_INVALID_DCA_IPADDR_ERR;
            return s16Status;
        }
        token = strtok(nodeData, ".");
        k = 0;
        while (token != NULL) {
            gsEthUpdateMode.au8Dca1000IpAddr[k++] = atoi(token);
            token = strtok(NULL, ".");
        }

        /** MAC address */
        if(!node.isMember("DCA1000MACAddress"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - DCA1000MACAddress node is missing. [error %d]",CLI_JSON_EEPROM_DCA_MACADDR_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_EEPROM_DCA_MACADDR_NODE_ERR;
            return s16Status;
        }
        strcpy(nodeData, root["ethernetConfigUpdate"]["DCA1000MACAddress"].asString().c_str());
        s16Status = validateMacAddress(nodeData);
        if(s16Status != SUCCESS_STATUS)
        {
            sprintf(s8DebugMsg, "Invalid Ethernet Update DCA1000MACAddress value (%s). [error %d]" ,
                    root["ethernetConfigUpdate"]["DCA1000MACAddress"].asString().c_str(),CLI_JSON_EEPROM_INVALID_DCA_MACADDR_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_EEPROM_INVALID_DCA_MACADDR_ERR;
            return s16Status;
        }

        token = strtok(nodeData, ".");
        k = 0;
        while (token != NULL) {
            gsEthUpdateMode.au8MacId[k++] = atoi(token);
            token = strtok(NULL, ".");
        }        

        /** Config port number */
        if(!node.isMember("DCA1000ConfigPort"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - Ethernet Update - DCA1000ConfigPort node is missing. [error %d]",CLI_JSON_EEPROM_DCA_CONFIG_PORT_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_EEPROM_DCA_CONFIG_PORT_NODE_ERR;
            return s16Status;
        }
        strcpy(nodeData,root["ethernetConfigUpdate"]["DCA1000ConfigPort"].asString().c_str());
        s16Status = validatePortNum(nodeData);
        if(s16Status != SUCCESS_STATUS)
        {
            sprintf(s8DebugMsg, "Invalid Ethernet Update DCA1000ConfigPort value (%s). [error %d]" ,
                        nodeData,CLI_JSON_EEPROM_INVALID_DCA_CONFIG_PORT_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_EEPROM_INVALID_DCA_CONFIG_PORT_ERR;
            return s16Status;
        }
        gsEthUpdateMode.u32ConfigPortNo = atoi(nodeData);

        /** Data port number */
        if(!node.isMember("DCA1000DataPort"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - Ethernet Update - DCA1000DataPort node is missing. [error %d]",CLI_JSON_EEPROM_DCA_DATA_PORT_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_EEPROM_DCA_DATA_PORT_NODE_ERR;
            return s16Status;
        }
        strcpy(nodeData,root["ethernetConfigUpdate"]["DCA1000DataPort"].asString().c_str());
        s16Status = validatePortNum(nodeData);
        if(s16Status != SUCCESS_STATUS)
        {
            sprintf(s8DebugMsg, "Invalid Ethernet config DCA1000DataPort value (%s). [error %d]" ,
                    nodeData,CLI_JSON_EEPROM_INVALID_DCA_DATA_PORT_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_EEPROM_INVALID_DCA_DATA_PORT_ERR;
            return s16Status;
        }
        gsEthUpdateMode.u32RecordPortNo = atoi(nodeData);

        s16Status = validatePortNumsForConflicts(gsEthUpdateMode.u32RecordPortNo,
                                            gsEthUpdateMode.u32ConfigPortNo);
        if(s16Status != SUCCESS_STATUS)
        {
            sprintf(s8DebugMsg, "Invalid Ethernet Update port numbers - One of the data ports is same as config port(%d). [error %d]" ,
                    gsEthUpdateMode.u32ConfigPortNo, CLI_JSON_EEPROM_PORT_NUM_CONFLICT_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_EEPROM_PORT_NUM_CONFLICT_ERR;
            return s16Status;
        }
        break;
    case CMD_CODE_CONFIG_RECORD:
        if(!root.isMember("packetDelay_us"))
        {
            sprintf( s8DebugMsg,"Invalid JSON config file - packetDelay_us node is missing. [error %d]",CLI_JSON_PACKET_DELAY_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_PACKET_DELAY_NODE_ERR;
            return s16Status;
        }
        strcpy(nodeData, root["packetDelay_us"].asString().c_str());
        s16Status = validatePacketDelay(nodeData);
        if(s16Status != SUCCESS_STATUS)
        {
            sprintf(s8DebugMsg, "Invalid packetDelay_us value (%s). [error %d]" ,
                    nodeData,CLI_JSON_INVALID_PACKET_DELAY_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_INVALID_PACKET_DELAY_ERR;
            return s16Status;
        }
        gsRecConfigMode.u16RecDelay = atoi(nodeData);

        break;
    case CMD_CODE_START_RECORD:
        if(!root.isMember("dataLoggingMode"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - dataLoggingMode node is missing. [error %d]",CLI_JSON_DATA_LOGGING_MODE_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
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
            sprintf(s8DebugMsg, "Invalid dataLoggingMode value (%s). [error %d]", nodeData,CLI_JSON_INVALID_DATALOGGING_MODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_INVALID_DATALOGGING_MODE_ERR;
            return s16Status;
        }

        if(!root.isMember("lvdsMode"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - lvdsMode node is missing. [error %d]",CLI_JSON_LVDS_MODE_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
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
            sprintf(s8DebugMsg, "Invalid lvdsMode value (%s). [error %d]", 
								nodeData, CLI_JSON_INVALID_LVDS_MODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_INVALID_LVDS_MODE_ERR;
            return s16Status;
        }

        node = root.get("captureConfig",0);
        if(node.size() < 1)
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - captureConfig node is missing. [error %d]",CLI_JSON_CAPTURE_CONFIG_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_CAPTURE_CONFIG_NODE_ERR;
            return s16Status;
        }

        if(!node.isMember("fileBasePath"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - fileBasePath node is missing. [error %d]",CLI_JSON_REC_FILE_BASE_PATH_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_REC_FILE_BASE_PATH_NODE_ERR;
            return s16Status;
        }
        strcpy(gsStartRecConfigMode.s8FileBasePath,
               root["captureConfig"]["fileBasePath"].asString().c_str());

        if(!osalObj.IsValidDir(gsStartRecConfigMode.s8FileBasePath))
        {
            sprintf(s8DebugMsg, "Invalid fileBasePath value (%s). [error %d]",
                    gsStartRecConfigMode.s8FileBasePath,CLI_JSON_REC_INVALID_FILE_BASE_PATH_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            return CLI_JSON_REC_INVALID_FILE_BASE_PATH_ERR;
        }

        if(!node.isMember("filePrefix"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - filePrefix node is missing. [error %d]",CLI_JSON_REC_FILE_PREFIX_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_REC_FILE_PREFIX_NODE_ERR;
            return s16Status;
        }
        strcpy(gsStartRecConfigMode.s8FilePrefix,
               root["captureConfig"]["filePrefix"].asString().c_str());
        if(strcmp(gsStartRecConfigMode.s8FilePrefix, "") == 0)
        {
            sprintf(s8DebugMsg, "FilePrefix value is empty. [error %d]",CLI_JSON_REC_INVALID_FILE_PREFIX_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_REC_INVALID_FILE_PREFIX_ERR;
            return s16Status;
        }

        if(!node.isMember("maxRecFileSize_MB"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - maxRecFileSize_MB node is missing. [error %d]",CLI_JSON_REC_MAX_FILE_SIZE_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
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
            WRITE_TO_CONSOLE(s8DebugMsg);
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
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_REC_INVALID_EN_SEQ_NUM_ERR;
            return s16Status;
        }

        if(!node.isMember("captureStopMode"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - captureStopMode node is missing. [error %d]",CLI_JSON_REC_CAPTURE_STOP_MODE_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
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
                sprintf( s8DebugMsg, "Invalid JSON config file - bytesToCapture node is missing. [error %d]",CLI_JSON_REC_BYTE_CAPTURE_NODE_ERR);
                WRITE_TO_CONSOLE(s8DebugMsg);
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
                WRITE_TO_CONSOLE(s8DebugMsg);
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
                WRITE_TO_CONSOLE(s8DebugMsg);
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
                WRITE_TO_CONSOLE(s8DebugMsg);
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
                WRITE_TO_CONSOLE(s8DebugMsg);
                s16Status = CLI_JSON_REC_CAPTURE_STOP_MODE_NODE_ERR;
                return s16Status;
            }

            gsStartRecConfigMode.eRecordStopMode = FRAMES;
            if(!node.isMember("framesToCapture"))
            {
                sprintf(s8DebugMsg,"Invalid JSON config file - framesToCapture node is missing. [error %d]",CLI_JSON_REC_FRAME_CAPTURE_NODE_ERR);
                WRITE_TO_CONSOLE(s8DebugMsg);
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
                WRITE_TO_CONSOLE(s8DebugMsg);
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
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_REC_INVALID_CAPTURE_STOP_MODE_ERR;
            return s16Status;
        }

        /** Read MSB togglig and reordering enable/disable status */
        node = root.get("dataFormatConfig",0);
        if(node.size() < 1)
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - dataFormatConfig node is missing. [error %d]",
                    CLI_JSON_DATA_FORMAT_CONFIG_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
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
                WRITE_TO_CONSOLE(s8DebugMsg);
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
                WRITE_TO_CONSOLE(s8DebugMsg);
                s16Status = CLI_JSON_REC_INVALID_EN_REORDER_ERR;
                return s16Status;
            }
        }

        break;
    case CMD_CODE_CLI_PROC_STATUS_SHM:
        if(!root.isMember("dataLoggingMode"))
        {
            sprintf(s8DebugMsg, "Invalid JSON config file - dataLoggingMode node is missing. [error %d]",CLI_JSON_DATA_LOGGING_MODE_NODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
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
								nodeData,CLI_JSON_INVALID_DATALOGGING_MODE_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            s16Status = CLI_JSON_INVALID_DATALOGGING_MODE_ERR;
            return s16Status;
        }
        break;
    }

    return s16Status;
}


/** @fn bool IsRecordProcStopped()
 * @brief This function is to check whether the record process is stopped
 * @return boolean value
 */
bool IsRecordProcStopped()
{
    SHM_PROC_STATES procStates;
    SINT32 s32Status = osalObj.QueryRecordProcStatus(
								gsEthConfigMode.u32ConfigPortNo, &procStates);
    if(SUCCESS_STATUS == s32Status)
    {
        /** Parse the command status */
        if(procStates.s32CommandStatus == STS_CLI_REC_PROC_STOPPED)
        {
            //WRITE_TO_LOG_FILE( "Record stop is done successfully");
            //WRITE_TO_CONSOLE( "Record stop is done successfully");
            return true;
        }
        else if(procStates.s32CommandStatus == STS_CLI_REC_PROC_STOP_FAILED)
        {
            //WRITE_TO_LOG_FILE( "Record stop failed");
            //WRITE_TO_CONSOLE( "Record stop failed");
            return true;
        }
    }
    return false;
}

/** @fn bool IsRecordProcRunning()
 * @brief This function is to check whether the record process is runnning

 * @return boolean value
 */
bool IsRecordProcRunning()
{
    SHM_PROC_STATES procStates;
    SINT8 s8DebugMsg[MAX_NAME_LEN];

    SINT32 s32Status = osalObj.QueryRecordProcStatus(
								gsEthConfigMode.u32ConfigPortNo, &procStates);
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
         //   WRITE_TO_CONSOLE( "No record process is running. [error %d]",
            //CLI_SHM_NOT_AVAIL_ERR);
        }
        else
        {
            sprintf(s8DebugMsg, "Unable to read record process states. [error %d]",
                   s32Status);
            WRITE_TO_LOG_FILE(s8DebugMsg);
        }

        return false;
    }
}

/** @fn bool ReadShmForQueryStatusCmd()
 * @brief This function is to read and display the record process status
 * @return SINT32 value
 */
SINT32 ReadShmForQueryStatusCmd()
{
    SINT32 s32Status = 0;
    SINT8 s8LogMsg[MAX_NAME_LEN];

    WRITE_TO_LOG_FILE( "Read Record Process Status (req)");

    SHM_PROC_STATES procStates;
    s32Status = osalObj.QueryRecordProcStatus(gsEthConfigMode.u32ConfigPortNo,
                                              &procStates);
    if(SUCCESS_STATUS == s32Status)
    {
        /** Parse the command status */
        if(procStates.u16CommandCode == CMD_CODE_START_RECORD)
        {
            if(procStates.s32CommandStatus == STS_CLI_REC_PROC_IS_IN_PROG)
            {
                sprintf(s8LogMsg, "Record is in progress. [status %d]",
                        CLI_SHM_REC_IN_PROG_STS);
                s32Status = CLI_SHM_REC_IN_PROG_STS;
            }
            else if(procStates.s32CommandStatus == STS_CLI_REC_PROC_STOPPED)
            {
                sprintf(s8LogMsg, "Record process is stopped. [status %d]",
                        CLI_SHM_REC_STOPPED_STS);
                s32Status = CLI_SHM_REC_STOPPED_STS;
            }
            else if(procStates.s32CommandStatus == STS_CLI_REC_PROC_START_INIT)
            {
                sprintf(s8LogMsg, "Record process is initiated. [status %d]",
                        CLI_SHM_REC_START_INITATED_STS);
                s32Status = CLI_SHM_REC_START_INITATED_STS;
            }
            else if(procStates.s32CommandStatus == STS_CLI_REC_PROC_STOP_INIT)
            {
                sprintf(s8LogMsg, "Record stopping is initiated. [status %d]",
                        CLI_SHM_REC_STOP_INITATED_STS);
                s32Status = CLI_SHM_REC_STOP_INITATED_STS;
            }
            else if(procStates.s32CommandStatus == STS_CLI_REC_PROC_START_FAILED)
            {
                sprintf(s8LogMsg,
                        "Start record process is failed. [error %d]",
                        CLI_SHM_REC_START_FAILED_STS);
                s32Status = CLI_SHM_REC_START_FAILED_STS;
            }
            else if(procStates.s32CommandStatus == STS_CLI_REC_PROC_STOP_FAILED)
            {
                sprintf(s8LogMsg,
                            "Stop record process is failed. [error %d]",
                            CLI_SHM_REC_STOP_FAILED_STS);
                s32Status = CLI_SHM_REC_STOP_FAILED_STS;
            }
            else
            {
                sprintf(s8LogMsg,
                        "Unknown record process status (%d). [error %d]",
                        CLI_SHM_REC_UNKNOWN_STS, procStates.s32CommandStatus);
                s32Status = CLI_SHM_REC_UNKNOWN_STS;
            }

             //STS_NO_LVDS_DATA
            if((procStates.u32AsyncStatus & (1 << STS_NO_LVDS_DATA)) ==
                        (1 << STS_NO_LVDS_DATA))
            {
                WRITE_TO_CONSOLE( "No LVDS data");
                WRITE_TO_LOG_FILE( "No LVDS data");
            }
            //STS_NO_HEADER
            if((procStates.u32AsyncStatus & (1 << STS_NO_HEADER)) ==
                    (1 << STS_NO_HEADER))
            {
                WRITE_TO_CONSOLE( "No header");
                WRITE_TO_LOG_FILE( "No header");
            }
            //STS_EEPROM_FAILURE
            if((procStates.u32AsyncStatus & (1 << STS_EEPROM_FAILURE)) ==
                    (1 << STS_EEPROM_FAILURE))
            {
                WRITE_TO_CONSOLE( "EEPROM failure");
                WRITE_TO_LOG_FILE( "EEPROM failure");
            }
            //STS_SD_CARD_DETECTED
            if((procStates.u32AsyncStatus & (1 << STS_SD_CARD_DETECTED)) ==
                    (1 << STS_SD_CARD_DETECTED))
            {
                WRITE_TO_CONSOLE( "SD card detected");
                WRITE_TO_LOG_FILE( "SD card detected");
            }
            //STS_SD_CARD_REMOVED
            if((procStates.u32AsyncStatus & (1 << STS_SD_CARD_REMOVED)) ==
                    (1 << STS_SD_CARD_REMOVED))
            {
                WRITE_TO_CONSOLE( "SD card removed");
                WRITE_TO_LOG_FILE( "SD card removed");
            }
            //STS_MODE_CONFIG_FAILURE
            if((procStates.u32AsyncStatus & (1 << STS_MODE_CONFIG_FAILURE)) ==
                    (1 << STS_MODE_CONFIG_FAILURE))
            {
                WRITE_TO_CONSOLE( "Mode configuration failed");
                WRITE_TO_LOG_FILE( "Mode configuration failed");
            }
            //STS_DDR_FULL
            if((procStates.u32AsyncStatus & (1 << STS_DDR_FULL)) ==
                    (1 << STS_DDR_FULL))
            {
                WRITE_TO_CONSOLE( "DDR full");
                WRITE_TO_LOG_FILE( "DDR full");
            }
            //STS_LVDS_BUFFER_FULL
            if((procStates.u32AsyncStatus & (1 << STS_LVDS_BUFFER_FULL)) ==
                    (1 << STS_LVDS_BUFFER_FULL))
            {
                WRITE_TO_CONSOLE( "LVDS buffer full");
                WRITE_TO_LOG_FILE( "LVDS buffer full");
            }
            //STS_RECORD_PKT_OUT_OF_SEQ
            if((procStates.u32AsyncStatus & (1 << STS_REC_PKT_OUT_OF_SEQ)) ==
                    (1 << STS_REC_PKT_OUT_OF_SEQ))
            {
                WRITE_TO_CONSOLE( "Record packet out of sequence");
                WRITE_TO_LOG_FILE( "Record packet out of sequence");

                UINT8 u8NumDataTypes = NUM_DATA_TYPES;
                SINT8 s8LogBuf[MAX_FILE_UPDATE_LEN];
                memset(s8LogBuf, '\0', MAX_FILE_UPDATE_LEN);
                ULONG64 seconds = 0;

                if(gsStartRecConfigMode.eConfigLogMode == RAW_MODE)
                {
                    u8NumDataTypes = 1;
                }

                for(int i = 0; i < u8NumDataTypes; i ++)
                {
                    if(u8NumDataTypes == 1)
                        sprintf(s8LogMsg, "Raw Data :");
                    else
                    {
                        if(strcmp(procStates.strInlineProcStats.s8HeaderId[i],
                                  "") == 0)
                        {
                            sprintf(s8LogMsg, "DT %d Header Data : (No data received)",
                                i+1);
                        }
                        else
                        {
                            sprintf(s8LogMsg, "%s Header Data :",
                                procStates.strInlineProcStats.s8HeaderId[i]);
                        }
                    }
                    strcpy(s8LogBuf, s8LogMsg);
                    sprintf(s8LogMsg, "\nOut of sequence count - %llu",
                            procStates.strInlineProcStats.u64OutOfSeqCount[i]);
                    strcat(s8LogBuf, s8LogMsg);
                    sprintf(s8LogMsg, "\nFirst Packet ID - %u",
                            procStates.strInlineProcStats.u32FirstPktId[i]);
                    strcat(s8LogBuf, s8LogMsg);
#ifndef POST_PROCESSING
                    sprintf(s8LogMsg, "\nOut of sequence from %u to %u",
                            procStates.strInlineProcStats.u32OutOfSeqPktFromOffset[i],
                            procStates.strInlineProcStats.u32OutOfSeqPktToOffset[i]);
                    strcat(s8LogBuf, s8LogMsg);
#endif
                    sprintf(s8LogMsg, "\nLast Packet ID - %d",
                            procStates.strInlineProcStats.u32LastPktId[i]);
                    strcat(s8LogBuf, s8LogMsg);
                    sprintf(s8LogMsg, "\nNumber of received packets - %llu",
                            procStates.strInlineProcStats.u64NumOfRecvdPackets[i]);
                    strcat(s8LogBuf, s8LogMsg);
#ifndef POST_PROCESSING
                    sprintf(s8LogMsg, "\nNumber of zero filled packets - %llu",
                            procStates.strInlineProcStats.u64NumOfZeroFilledPackets[i]);
                    strcat(s8LogBuf, s8LogMsg);
                    sprintf(s8LogMsg, "\nNumber of zero filled bytes - %llu",
                            procStates.strInlineProcStats.u64NumOfZeroFilledBytes[i]);
                    strcat(s8LogBuf, s8LogMsg);
#endif
                    sprintf(s8LogMsg, "\nCapture start time - %s",
                            ctime(&procStates.strInlineProcStats.StartTime[i]));
                    strcat(s8LogBuf, s8LogMsg);
                    sprintf(s8LogMsg, "Capture end time - %s",
                            ctime(&procStates.strInlineProcStats.EndTime[i]));
                    strcat(s8LogBuf, s8LogMsg);
                    seconds = difftime(procStates.strInlineProcStats.EndTime[i],
                                              procStates.strInlineProcStats.StartTime[i]);
                    sprintf(s8LogMsg, "Capture Duration(sec) - %llu", seconds);
                    strcat(s8LogBuf, s8LogMsg);

                    WRITE_TO_CONSOLE(s8LogBuf);
                    WRITE_TO_LOG_FILE(s8LogBuf);
                    strcpy(s8LogBuf, "\0");
                    strcpy(s8LogMsg, "\0");
                }
            }
            //STS_CAPTURE_THREAD_TIMEOUT
            if((procStates.u32AsyncStatus & (1 << STS_REC_PROC_TIMEOUT)) ==
                                (1 << STS_REC_PROC_TIMEOUT))
            {
                WRITE_TO_CONSOLE( "Record Process : \nTimeout Error! System disconnected");
                WRITE_TO_LOG_FILE( "Record Process : \nTimeout Error! System disconnected");
            }
            //STS_RECORD_FILE_CREATION_ERR
            if((procStates.u32AsyncStatus & (1 << STS_REC_FILE_CREATION_ERR)) ==
                                (1 << STS_REC_FILE_CREATION_ERR))
            {
                WRITE_TO_CONSOLE( "Record file creation error");
                WRITE_TO_LOG_FILE( "Record file creation error");
            }
            //STS_REC_REORDERING_ERR
            if((procStates.u32AsyncStatus & (1 << STS_REC_REORDERING_ERR)) ==
                                (1 << STS_REC_REORDERING_ERR))
            {
                WRITE_TO_CONSOLE( "Record process - Recordering error");
                WRITE_TO_LOG_FILE( "Record process - Recordering error");
            }
            //STS_REC_INLINE_BUF_ALLOCATION_ERR
            if((procStates.u32AsyncStatus &
                            (1 << STS_REC_INLINE_BUF_ALLOCATION_ERR)) ==
                            (1 << STS_REC_INLINE_BUF_ALLOCATION_ERR))
            {
                WRITE_TO_CONSOLE(
                            "Record process - Inline buffer allocation error");
                WRITE_TO_LOG_FILE(
                            "Record process - Inline buffer allocation error");
            }
        }
        else  /** invalid command set */
        {
            sprintf(s8LogMsg, "Unable to read record process states. [error %d]",
                    CLI_SHM_REC_PROC_STS_READ_ERR);
            s32Status = CLI_SHM_REC_PROC_STS_READ_ERR;
        }
    }
    else
    {
        if(s32Status == CLI_SHM_NOT_AVAIL_ERR)
        {
             sprintf(s8LogMsg,"No record process is running. [error %d]",
                     CLI_SHM_NOT_AVAIL_ERR);
        }
        else
        {
            sprintf(s8LogMsg, "Unable to read record process states. [error %d]",
                   s32Status);
        }
    }

    WRITE_TO_CONSOLE(s8LogMsg);
    WRITE_TO_LOG_FILE(s8LogMsg);

    return s32Status;
}

/** @fn void DecodeCommandStatus(SINT32 s32Status, const SINT8 * strCommand)
 * @brief This function is to decode command response status as success or failure
 * @param [in] s32Status [SINT32] - Command status
 * @param [in] strCommand [const SINT8 *] - Command name
 */
void DecodeCommandStatus(SINT32 s32Status, const SINT8 * strCommand)
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

    WRITE_TO_CONSOLE(msgData);
    WRITE_TO_LOG_FILE(msgData);
    sprintf(msgData, "Return status : %d", s32Status);
    WRITE_TO_LOG_FILE(msgData);
}

/** @fn SINT32 main(SINT32 argc, SINT8* argv[])
 * @brief This function is the main function to handle configuration <!--
 * --> commands to be executed, validation of JSON file and command line  <!--
 * --> arguments
 * @param [in] argc [SINT32] - Number of arguments
 * @param [in] argv [SINT8 *] - Arguments array
 * @return SINT32 value
 */
SINT32 main(SINT32 argc, SINT8* argv[])
{
    /** Command line argument  1 - Command name */
    SINT8 s8Command[MAX_PARAMS_LEN];

    /** Command line argument  2 - JSON file name */
    SINT8 s8CommandArg[MAX_PARAMS_LEN];

    SINT8 s8DebugMsg[MAX_NAME_LEN];
    SINT32 s32CliStatus = SUCCESS_STATUS;
    SINT8 s8Version[MAX_NAME_LEN];
    SINT8 s8CliVersion[MAX_NAME_LEN];
    UINT16 u16CmdCode = 0;
    strcpy(s8CliVersion, CLI_CTRL_VERSION);
    bool bValue = false;

    if(argc < 2)
    {
        ListOfCmds();
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
    if(argc < 2)
    {
        printf("\nEnter the command : ");
        scanf("%s", s8Command);

        if((strcmp(s8Command, CMD_HELP_CLI_APP) == 0) ||
                (strcmp(s8Command, CMD_HELP_S_CLI_APP) == 0))
        {
            ListOfCmds();
            return SUCCESS_STATUS;
        }
        if(strcmp(s8Command, CMD_READ_DLL_VER) == 0)
        {
            /** API Call - Read API DLL version                              */
            memset(s8Version, '\0', MAX_NAME_LEN);
            WRITE_TO_LOG_FILE( "Read DLL Verison Command (req)");
            if(ReadRFDCCard_DllVersion(s8Version) == SUCCESS_STATUS)
            {
                sprintf(s8DebugMsg, "\nDLL Version : %s\n", s8Version);
                WRITE_TO_CONSOLE(s8DebugMsg);
                WRITE_TO_LOG_FILE(s8DebugMsg);
                return SUCCESS_STATUS;
            }
            else
            {
                sprintf(s8DebugMsg, "\nNot able to read DLL Version. error[%d]\n",
                        CLI_READ_DLL_VERSION_ERR);
                WRITE_TO_CONSOLE(s8DebugMsg);
                WRITE_TO_LOG_FILE(s8DebugMsg);
                return CLI_READ_DLL_VERSION_ERR;
            }
        }
        else if(strcmp(s8Command, CMD_READ_CLI_VER) == 0)
        {
            u16CmdCode = CMD_CODE_CLI_READ_CLI_VERSION;
            WRITE_TO_LOG_FILE( "Read CLI Verison Command (req)");
            sprintf(s8DebugMsg, "\nControl CLI Version : %s\n", s8CliVersion);
            WRITE_TO_CONSOLE(s8DebugMsg);
            WRITE_TO_LOG_FILE(s8DebugMsg);
            return SUCCESS_STATUS;
        }
        else
        {
            printf("\nEnter the argument : ");
            scanf("%s", s8CommandArg);
        }
    }
#endif

    u16CmdCode = 0;

    if(strcmp(s8Command, CMD_QUERY_SYSTEM_ALIVENESS) == 0)
    {
        u16CmdCode = CMD_CODE_SYSTEM_ALIVENESS;
    }
    else if(strcmp(s8Command, CMD_RESET_FPGA) == 0)
    {
        u16CmdCode = CMD_CODE_RESET_FPGA;
    }
    else if(strcmp(s8Command, CMD_RESET_AR_DEV) == 0)
    {
        u16CmdCode = CMD_CODE_RESET_AR_DEV;
    }
    else if(strcmp(s8Command, CMD_CONFIG_FPGA) == 0)
    {
        u16CmdCode = CMD_CODE_CONFIG_FPGA;
    }
    else if(strcmp(s8Command, CMD_CONFIG_EEPROM) == 0)
    {
        u16CmdCode = CMD_CODE_CONFIG_EEPROM;
    }
    else if(strcmp(s8Command, CMD_START_RECORD) == 0)
    {
        u16CmdCode = CMD_CODE_START_RECORD;
    }
    else if(strcmp(s8Command, CMD_STOP_RECORD) == 0)
    {
        u16CmdCode = CMD_CODE_CLI_ASYNC_RECORD_STOP;
    }
    else if(strcmp(s8Command, CMD_CONFIG_RECORD) == 0)
    {
        u16CmdCode = CMD_CODE_CONFIG_RECORD;
    }
    else if(strcmp(s8Command, CMD_READ_FPGA_VER) == 0)
    {
        u16CmdCode = CMD_CODE_READ_FPGA_VERSION;
    }
    else if(strcmp(s8Command, CMD_QUERY_CLI_PROC_STATUS) == 0)
    {
        u16CmdCode = CMD_CODE_CLI_PROC_STATUS_SHM;
    }
    else if((strcmp(s8Command, CMD_HELP_CLI_APP) == 0) ||
            (strcmp(s8Command, CMD_HELP_S_CLI_APP) == 0))
    {
        ListOfCmds();
        return SUCCESS_STATUS;
    }
    else if(strcmp(s8Command, CMD_READ_DLL_VER) == 0)
    {
        /* API Call - Read API DLL version                                   */
        memset(s8Version, '\0', MAX_NAME_LEN);
        WRITE_TO_LOG_FILE( "Read DLL Verison Command (req)");
        if(ReadRFDCCard_DllVersion(s8Version) == SUCCESS_STATUS)
        {
            sprintf(s8DebugMsg, "DLL Version : %s", s8Version);
            WRITE_TO_CONSOLE(s8DebugMsg);
            WRITE_TO_LOG_FILE(s8DebugMsg);
            return SUCCESS_STATUS;
        }
        else
        {
            sprintf(s8DebugMsg, "Not able to read DLL Version. error[%d]",
                    CLI_READ_DLL_VERSION_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            WRITE_TO_LOG_FILE(s8DebugMsg);
            return CLI_READ_DLL_VERSION_ERR;
        }
    }
    else if(strcmp(s8Command, CMD_READ_CLI_VER) == 0)
    {
        u16CmdCode = CMD_CODE_CLI_READ_CLI_VERSION;
        WRITE_TO_LOG_FILE( "Read CLI Verison Command (req)");
        sprintf(s8DebugMsg, "Control CLI Version : %s", s8CliVersion);
        WRITE_TO_CONSOLE(s8DebugMsg);
        WRITE_TO_LOG_FILE(s8DebugMsg);
        return SUCCESS_STATUS;
    }
    else
    {
        sprintf(s8DebugMsg, "Invalid Command (%s). error[%d]", s8Command,
                CLI_INVALID_CMD_ARG_ERR);
        WRITE_TO_CONSOLE(s8DebugMsg);
        WRITE_TO_LOG_FILE(s8DebugMsg);
        return CLI_INVALID_CMD_ARG_ERR;
    }

    s32CliStatus = ValidateJsonFileData(s8CommandArg, u16CmdCode);
    if(s32CliStatus < 0)
        return s32CliStatus;

    if(u16CmdCode != CMD_CODE_CLI_PROC_STATUS_SHM)
    {
        bValue = IsRecordProcRunning();
        if(u16CmdCode == CMD_CODE_CLI_ASYNC_RECORD_STOP)
        {
            if(!bValue)
            {
                WRITE_TO_LOG_FILE(s8Command);
                WRITE_TO_CONSOLE( "No record process is running.");
                WRITE_TO_LOG_FILE("No record process is running.");
                return CLI_SHM_REC_PROC_NOT_RUNNING_STS;
            }
        }
        else
        {
            if(bValue)
            {
                WRITE_TO_LOG_FILE(s8Command);
                WRITE_TO_CONSOLE( "Stop the already running process.");
                WRITE_TO_LOG_FILE( "Stop the already running process.");
                return CLI_SHM_REC_PROC_RUNNING_STS;
            }
        }

        /* API Call - Ethernet connection */
        if(u16CmdCode == CMD_CODE_CLI_ASYNC_RECORD_STOP)
        {
            s32CliStatus = ConnectRFDCCard_AsyncCommandMode(gsEthConfigMode);
        }
        else
        {
            s32CliStatus = ConnectRFDCCard_ConfigMode(gsEthConfigMode);
        }
        if(s32CliStatus != SUCCESS_STATUS)
        {
            WRITE_TO_LOG_FILE( "Ethernet connection");
            sprintf(s8DebugMsg, "Ethernet connection failed. [error %d]",
                    CLI_ETH_CONNECT_FAIL_ERR);
            WRITE_TO_CONSOLE(s8DebugMsg);
            WRITE_TO_LOG_FILE(s8DebugMsg);
            cli_DisconnectRFDCCard(u16CmdCode);
            return CLI_ETH_CONNECT_FAIL_ERR;
        }
    }

    switch(u16CmdCode)
    {
    case CMD_CODE_SYSTEM_ALIVENESS:

        WRITE_TO_LOG_FILE( "Query system status Command (req)");

        /** API Call - System aliveness                                          */
        s32CliStatus = HandshakeRFDCCard();
        if(s32CliStatus != SUCCESS_STATUS)
        {
            sprintf(s8DebugMsg, "System is disconnected.");
        }
        else
        {
            sprintf(s8DebugMsg, "System is connected.");
        }
        WRITE_TO_CONSOLE(s8DebugMsg);
        WRITE_TO_LOG_FILE(s8DebugMsg);
        sprintf(s8DebugMsg, "Return status : %d", s32CliStatus);
        WRITE_TO_LOG_FILE(s8DebugMsg);

        break;

    case CMD_CODE_RESET_FPGA:
        /** Timestamp logging                                                */
        WRITE_TO_LOG_FILE( "Reset FPGA Command (req)");

        /** API Call - Reset FPGA                                            */
        s32CliStatus = ResetRFDCCard_FPGA();

        /** Handling command response                                        */
        DecodeCommandStatus(s32CliStatus, "Reset FPGA");

        break;

    case CMD_CODE_RESET_AR_DEV:
        /** Timestamp logging                                                */
        WRITE_TO_LOG_FILE( "Reset AR Device Command (req)");

        /** API Call - Reset AR Deice                                        */
        s32CliStatus = ResetRadarEVM();

        /** Handling command response                                        */
        DecodeCommandStatus(s32CliStatus, "Reset AR Device");

        break;

    case CMD_CODE_CONFIG_FPGA:
        /** Timestamp logging                                                */
        WRITE_TO_LOG_FILE( "FPGA Configuration Command (req)");

        /** API Call - Configure FPGA                                        */
        s32CliStatus = ConfigureRFDCCard_Fpga(gsFpgaConfigMode);

        /** Handling command response                                        */
        DecodeCommandStatus(s32CliStatus, "FPGA Configuration");

        break;

    case CMD_CODE_CONFIG_EEPROM:
        /** Timestamp logging                                                */
        WRITE_TO_LOG_FILE( "EEPROM Configuration Command (req)");

        /** API Call - Configure EEPROM                                      */
        s32CliStatus = ConfigureRFDCCard_Eeprom(gsEthUpdateMode);

        /** Handling command response                                        */
        DecodeCommandStatus(s32CliStatus, "EEPROM Configuration");

        break;

    case CMD_CODE_START_RECORD:
        /** Timestamp logging                                                */
        WRITE_TO_LOG_FILE( "Start Record Command (req)");

        s32CliStatus = HandshakeRFDCCard();
        DisconnectRFDCCard_ConfigMode();

        if(s32CliStatus != SUCCESS_STATUS)
        {
            sprintf(s8DebugMsg, "System is disconnected.");

            WRITE_TO_CONSOLE(s8DebugMsg);
            WRITE_TO_LOG_FILE(s8DebugMsg);
            sprintf(s8DebugMsg, "Return status : %d", s32CliStatus);
            WRITE_TO_LOG_FILE(s8DebugMsg);
        }
        else
        {
            osalObj.CreateShm(gsEthConfigMode.u32ConfigPortNo);

            /** API Call - Start Command                                     */
#ifdef _WIN32
            if(gbCliQuietMode)
                sprintf(s8DebugMsg, "start DCA1000EVM_CLI_Record.exe start_record %s -q &",
                        s8CommandArg);
            else
                sprintf(s8DebugMsg, "start DCA1000EVM_CLI_Record.exe start_record %s",
                        s8CommandArg);
#else
            if(gbCliQuietMode)
                sprintf(s8DebugMsg, "./DCA1000EVM_CLI_Record start_record %s -q &",
                        s8CommandArg);
            else
                sprintf(s8DebugMsg, "gnome-terminal -x ./DCA1000EVM_CLI_Record start_record %s",
                        s8CommandArg);
#endif
            if(system(s8DebugMsg) == SUCCESS_STATUS)
            {
                SINT32 timeoutCount = CLI_CMD_TIMEOUT_DURATION / MILLI_SEC_TO_READ_SHM;

                SINT32 iLoop = 0;
                do
                {
                    if(IsRecordProcRunning())
                        break;
                    else
                        osalObj.SleepInMilliSec(MILLI_SEC_TO_READ_SHM);
                    iLoop ++;
                }while(iLoop <= timeoutCount);

                if(iLoop > timeoutCount)
                {
                    sprintf(s8DebugMsg, "Start Record command : Timeout Error! Couldnt read the record process status. [error 4071]");
                    WRITE_TO_LOG_FILE(s8DebugMsg);
                    WRITE_TO_CONSOLE(s8DebugMsg);
                    return -4071;
                }
                else
                {
                    sprintf(s8DebugMsg, "Start Record command : Success");
                    WRITE_TO_LOG_FILE(s8DebugMsg);
                    WRITE_TO_CONSOLE(s8DebugMsg);
                    return SUCCESS_STATUS;
                }
            }
        }
        break;

    case CMD_CODE_CLI_ASYNC_RECORD_STOP:

        /** Timestamp logging                                                */
        WRITE_TO_LOG_FILE( "Stop Record Command (req)");

        /** API Call - Stop Record                                           */
        s32CliStatus = StopRecordAsyncCmd();

        if(s32CliStatus != SUCCESS_STATUS)
        {
            /** Handling command response                                        */
            DecodeCommandStatus(s32CliStatus, "Stop Record Command");
        }
        else
        {
            int timeoutCount = CLI_CMD_TIMEOUT_DURATION / MILLI_SEC_TO_READ_SHM;

            int iLoop = 0;
            do
            {
                if(IsRecordProcStopped())
                    break;
                else
                    osalObj.SleepInMilliSec(MILLI_SEC_TO_READ_SHM);
                iLoop ++;
            }while(iLoop <= timeoutCount);

            if(iLoop > timeoutCount)
            {
                sprintf(s8DebugMsg,
                        "Stop Record command : Timeout Error! Couldnt read the record process status. [error %d]",
                        CLI_CMD_REC_STOP_TIMEOUT_ERR);
                osalObj.DestroyShm(gsEthConfigMode.u32ConfigPortNo);
                WRITE_TO_LOG_FILE(s8DebugMsg);
                WRITE_TO_CONSOLE(s8DebugMsg);
                return CLI_CMD_REC_STOP_TIMEOUT_ERR;
            }
            else
            {
                sprintf(s8DebugMsg, "Stop Record command : Success");
                osalObj.DestroyShm(gsEthConfigMode.u32ConfigPortNo);
                WRITE_TO_LOG_FILE(s8DebugMsg);
                WRITE_TO_CONSOLE(s8DebugMsg);
                return SUCCESS_STATUS;
            }
        }
        break;
    case CMD_CODE_CONFIG_RECORD:
        /** Timestamp logging                                                */
        WRITE_TO_LOG_FILE( "Configure Record Command (req)");

        /** API Call - Configure Record                                      */
        s32CliStatus = ConfigureRFDCCard_Record(gsRecConfigMode);

        /** Handling command response                                        */
        DecodeCommandStatus(s32CliStatus, "Configure Record");

        break;
    case CMD_CODE_READ_FPGA_VERSION:
        /** Timestamp logging                                                */
        WRITE_TO_LOG_FILE( "Read FPGA version Command (req)");

        /** API Call - Read FPGA version                                     */
        memset(s8Version, '\0', MAX_NAME_LEN);
        s32CliStatus = ReadRFDCCard_FpgaVersion(s8Version);

        WRITE_TO_CONSOLE(s8Version);
        WRITE_TO_LOG_FILE(s8Version);
        break;
    case CMD_CODE_CLI_PROC_STATUS_SHM:
        s32CliStatus = ReadShmForQueryStatusCmd();
        break;
    default:
        sprintf(s8DebugMsg, "Invalid command %d.", u16CmdCode);
        WRITE_TO_CONSOLE(s8DebugMsg);
        WRITE_TO_LOG_FILE(s8DebugMsg);
        break;
    }

    if((u16CmdCode != CMD_CODE_CLI_PROC_STATUS_SHM) &&
            (u16CmdCode != CMD_CODE_START_RECORD))
    {
        cli_DisconnectRFDCCard(u16CmdCode);
    }
    return s32CliStatus;
}

