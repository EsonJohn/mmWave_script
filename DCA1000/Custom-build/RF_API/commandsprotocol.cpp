/**
 * @file commandsprotocol.cpp
 *
 * @author JP
 *
 * @version 0.1
 *
 * @brief This file contains definition of APIs for constructing UDP
 * packets based on defined protocols for DCA1000EVM commands
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
///*****************************************************************************


///****************
/// Includes
///****************

#include "commandsprotocol.h"

/** @fn void cCommandsProtocol::copyStructureIntoArray(SINT8 *ps8Data,      <!--
 * -->                                       DATA_CAPTURE_REQ structCmdReq)
 * @brief This function is to copy the command packet structure into an <!--
 * --> array to send as UDP datagram
 * @param [out] ps8Data [SINT8 *] - Array for command packet data
 * @param [in] structCmdReq [DATA_CAPTURE_REQ] - Structure filled with command data
 */
void cCommandsProtocol::copyStructureIntoArray(SINT8 *ps8Data,
                                        DATA_CAPTURE_REQ structCmdReq)
{
    memcpy(ps8Data, &structCmdReq, PACKET_FIXED_SIZE_EXC_FOOTER);
    if(structCmdReq.u16DataSize)
        memcpy(&ps8Data[PACKET_FIXED_SIZE_EXC_FOOTER],
               structCmdReq.strData, structCmdReq.u16DataSize);
    memcpy(&ps8Data[PACKET_FIXED_SIZE_EXC_FOOTER + structCmdReq.u16DataSize],
           &structCmdReq.u16Footer, PACKET_FOOTER_DATA_SIZE);
}


/** @fn UINT16 cCommandsProtocol::systemAlivenessCommand(SINT8 *s8Data)
 * @brief This function is to construct system aliveness command
 * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
 * @return UINT16 value
 */
UINT16 cCommandsProtocol::systemAlivenessCommand(SINT8 *s8Data)
{
    DATA_CAPTURE_REQ structDcReq;

    memset(&structDcReq, 0, sizeof(DATA_CAPTURE_REQ));
    memset(s8Data, 0, sizeof(DATA_CAPTURE_REQ));

    structDcReq.u16Header     = HEADER_START_BYTES;
    structDcReq.u16CmdCode    = CMD_CODE_SYSTEM_ALIVENESS;
    structDcReq.u16DataSize   = 0;
    structDcReq.u16Footer     = FOOTER_STOP_BYTES;

    /** Copying the command packet into array */
    copyStructureIntoArray(s8Data, structDcReq);

    /** Returning packet size */
    return (FIXED_PACKET_SIZE);
}

/** @fn UINT16 cCommandsProtocol::resetFpgaCommand(SINT8 *s8Data)
 * @brief This function is to construct reset FPGA command
 * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
 * @return UINT16 value
 */
UINT16 cCommandsProtocol::resetFpgaCommand(SINT8 *s8Data)
{
    DATA_CAPTURE_REQ structDcReq;

    memset(&structDcReq, 0, sizeof(DATA_CAPTURE_REQ));
    memset(s8Data, 0, sizeof(DATA_CAPTURE_REQ));

    structDcReq.u16Header     = HEADER_START_BYTES;
    structDcReq.u16CmdCode    = CMD_CODE_RESET_FPGA;
    structDcReq.u16DataSize   = 0;
    structDcReq.u16Footer     = FOOTER_STOP_BYTES;

    /** Copying the command packet into array  */
    copyStructureIntoArray(s8Data, structDcReq);

    /** Returning packet size   */
    return (FIXED_PACKET_SIZE);
}


/** @fn UINT16 cCommandsProtocol::resetArDeviceCommand(SINT8 *s8Data)
 * @brief This function is to construct reset AR device command
 * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
 * @return UINT16 value
 */
UINT16 cCommandsProtocol::resetArDeviceCommand(SINT8 *s8Data)
{
    DATA_CAPTURE_REQ structDcReq;

    memset(&structDcReq, 0, sizeof(DATA_CAPTURE_REQ));
    memset(s8Data, 0, sizeof(DATA_CAPTURE_REQ));

    structDcReq.u16Header     = HEADER_START_BYTES;
    structDcReq.u16CmdCode    = CMD_CODE_RESET_AR_DEV;
    structDcReq.u16DataSize   = 0;
    structDcReq.u16Footer     = FOOTER_STOP_BYTES;

    /** Copying the command packet into array  */
    copyStructureIntoArray(s8Data, structDcReq);

    /** Returning packet size   */
    return (FIXED_PACKET_SIZE);
}

/** @fn UINT16 cCommandsProtocol::configFpgaCommand(SINT8 *s8Data, <!--
 * -->                                 UINT8 *pu8Data, UINT16 u16DataLen)
 * @brief This function is to construct FPGA config command
 * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
 * @param [in] pu8Data [UINT8 *] - Array filled with command packet data
 * @param [in] u16DataLen [UINT16] - Length of command packet data
 * @return UINT16 value
 */
UINT16 cCommandsProtocol::configFpgaCommand(SINT8 *s8Data,
                                          UINT8 *pu8Data, UINT16 u16DataLen)
{
    DATA_CAPTURE_REQ structDcReq;

    memset(&structDcReq, 0, sizeof(DATA_CAPTURE_REQ));
    memset(s8Data, 0, sizeof(DATA_CAPTURE_REQ));

    structDcReq.u16Header     = HEADER_START_BYTES;
    structDcReq.u16CmdCode    = CMD_CODE_CONFIG_FPGA;
    structDcReq.u16DataSize   = u16DataLen;
    structDcReq.u16Footer     = FOOTER_STOP_BYTES;

    /** Copying the command data into request structure   */
    memcpy(structDcReq.strData, pu8Data, u16DataLen);

    /** Copying the command packet into array  */
    copyStructureIntoArray(s8Data, structDcReq);

    /** Returning packet size   */
    return (u16DataLen + FIXED_PACKET_SIZE);
}

/** @fn UINT16 cCommandsProtocol::configEepromCommand(SINT8 *s8Data, <!--
 * -->                                 UINT8 *pu8Data, UINT16 u16DataLen)
 * @brief This function is to construct EEPROM config command
 * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
 * @param [in] pu8Data [UINT8 *] - Array filled with command packet data
 * @param [in] u16DataLen [UINT16] - Length of command packet data
 * @return UINT16 value
 */
UINT16 cCommandsProtocol::configEepromCommand(SINT8 *s8Data,
                                            UINT8 *pu8Data, UINT16 u16DataLen)
  {
    DATA_CAPTURE_REQ structDcReq;

    memset(&structDcReq, 0, sizeof(DATA_CAPTURE_REQ));
    memset(s8Data, 0, sizeof(DATA_CAPTURE_REQ));

    structDcReq.u16Header     = HEADER_START_BYTES;
    structDcReq.u16CmdCode    = CMD_CODE_CONFIG_EEPROM;
    structDcReq.u16DataSize   = u16DataLen;
    structDcReq.u16Footer     = FOOTER_STOP_BYTES;

    /** Copying the command data into request structure   */
    memcpy(structDcReq.strData, pu8Data, u16DataLen);

    /** Copying the command packet into array  */
    copyStructureIntoArray(s8Data, structDcReq);

    /** Returning packet size   */
    return (u16DataLen + FIXED_PACKET_SIZE);
}


/** @fn UINT16 cCommandsProtocol::startRecordCommand(SINT8 *s8Data)
 * @brief This function is to construct start record command
 * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
 * @return UINT16 value
 */
UINT16 cCommandsProtocol::startRecordCommand(SINT8 *s8Data)
{
    DATA_CAPTURE_REQ structDcReq;

    memset(&structDcReq, 0, sizeof(DATA_CAPTURE_REQ));
    memset(s8Data, 0, sizeof(DATA_CAPTURE_REQ));

    structDcReq.u16Header     = HEADER_START_BYTES;
    structDcReq.u16CmdCode    = CMD_CODE_START_RECORD;
    structDcReq.u16DataSize   = 0;
    structDcReq.u16Footer     = FOOTER_STOP_BYTES;

    /** Copying the command packet into array  */
    copyStructureIntoArray(s8Data, structDcReq);

    /** Returning packet size   */
    return (FIXED_PACKET_SIZE);
}


/** @fn UINT16 cCommandsProtocol::stopRecordCommand(SINT8 *s8Data)
 * @brief This function is to construct stop record command
 * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
 * @return UINT16 value
 */
UINT16 cCommandsProtocol::stopRecordCommand(SINT8 *s8Data)
{
    DATA_CAPTURE_REQ structDcReq;

    memset(&structDcReq, 0, sizeof(DATA_CAPTURE_REQ));
    memset(s8Data, 0, sizeof(DATA_CAPTURE_REQ));

    structDcReq.u16Header     = HEADER_START_BYTES;
    structDcReq.u16CmdCode    = CMD_CODE_STOP_RECORD;
    structDcReq.u16DataSize   = 0;
    structDcReq.u16Footer     = FOOTER_STOP_BYTES;

    /** Copying the command packet into array  */
    copyStructureIntoArray(s8Data, structDcReq);

    /** Returning packet size   */
    return  (FIXED_PACKET_SIZE);
}

/** @fn UINT16 cCommandsProtocol::stopRecordAsyncCommand(SINT8 *s8Data)
 * @brief This function is to construct stop record from cli_control tool
 * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
 * @return UINT16 value
 */
UINT16 cCommandsProtocol::stopRecordAsyncCommand(SINT8 *s8Data)
{
    DATA_CAPTURE_REQ structDcReq;

    memset(&structDcReq, 0, sizeof(DATA_CAPTURE_REQ));
    memset(s8Data, 0, sizeof(DATA_CAPTURE_REQ));

    structDcReq.u16Header     = HEADER_START_BYTES;
    structDcReq.u16CmdCode    = CMD_CODE_CLI_ASYNC_RECORD_STOP;
    structDcReq.u16DataSize   = 0;
    structDcReq.u16Footer     = FOOTER_STOP_BYTES;

    /** Copying the command packet into array  */
    copyStructureIntoArray(s8Data, structDcReq);

    /** Returning packet size   */
    return  (FIXED_PACKET_SIZE);
}


/** @fn UINT16 cCommandsProtocol::readFpgaVersionCommand(SINT8 *s8Data)
 * @brief This function is to construct read FPGA version command
 * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
 * @return UINT16 value
 */
UINT16 cCommandsProtocol::readFpgaVersionCommand(SINT8 *s8Data)
{
    DATA_CAPTURE_REQ structDcReq;

    memset(&structDcReq, 0, sizeof(DATA_CAPTURE_REQ));
    memset(s8Data, 0, sizeof(DATA_CAPTURE_REQ));

    structDcReq.u16Header     = HEADER_START_BYTES;
    structDcReq.u16CmdCode    = CMD_CODE_READ_FPGA_VERSION;
    structDcReq.u16DataSize   = 0;
    structDcReq.u16Footer     = FOOTER_STOP_BYTES;

    /** Copying the command packet into array  */
    copyStructureIntoArray(s8Data, structDcReq);

    /** Returning packet size   */
    return  (FIXED_PACKET_SIZE);
}


/** @fn UINT16 cCommandsProtocol::configDataPacketCommand(SINT8 *s8Data, <!--
 * -->                                 UINT8 *pu8Data, UINT16 u16DataLen)
 * @brief This function is to construct record delay config command
 * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
 * @param [in] pu8Data [UINT8 *] - Array filled with command packet data
 * @param [in] u16DataLen [UINT16] - Length of command packet data
 * @return UINT16 value
 */
UINT16 cCommandsProtocol::configDataPacketCommand(SINT8 *s8Data,
                                          UINT8 *pu8Data, UINT16 u16DataLen)
{
    DATA_CAPTURE_REQ structDcReq;

    memset(&structDcReq, 0, sizeof(DATA_CAPTURE_REQ));
    memset(s8Data, 0, sizeof(DATA_CAPTURE_REQ));

    structDcReq.u16Header     = HEADER_START_BYTES;
    structDcReq.u16CmdCode    = CMD_CODE_CONFIG_RECORD;
    structDcReq.u16DataSize   = u16DataLen;
    structDcReq.u16Footer     = FOOTER_STOP_BYTES;

    /** Copying the command data into request structure   */
    memcpy(structDcReq.strData, pu8Data, u16DataLen);

    /** Copying the command packet into array  */
    copyStructureIntoArray(s8Data, structDcReq);

    /** Returning packet size   */
    return (u16DataLen + FIXED_PACKET_SIZE);
}

