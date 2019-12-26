/**
 * @file commandsprotocol.h
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


#ifndef CMDSPROTO_H
#define CMDSPROTO_H

///****************
/// Includes
///****************

#include "../Common/rf_api_internal.h"

/** @class cCommandsProtocol
 * @brief Describes the construction of UDP packets based on defined protocol  <!--
 * -->  structure for DCA1000EVM commands
 */
class cCommandsProtocol
{

public:

    /** @fn void copyStructureIntoArray(SINT8 *ps8Data,      <!--
     * -->                                       DATA_CAPTURE_REQ structCmdReq)
     * @brief This function is to copy the command packet structure into an <!--
     * --> array to send as UDP datagram
     * @param [out] ps8Data [SINT8 *] - Array for command packet data
     * @param [in] structCmdReq [DATA_CAPTURE_REQ] - Structure filled with command data
     */
    void copyStructureIntoArray(SINT8 *ps8Data, DATA_CAPTURE_REQ structCmdReq);

    /** @fn UINT16 systemAlivenessCommand(SINT8 *s8Data)
     * @brief This function is to construct system aliveness command
     * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
     * @return UINT16 value
     */
    UINT16 systemAlivenessCommand(SINT8 *s8Data);

    /** @fn UINT16 resetFpgaCommand(SINT8 *s8Data)
     * @brief This function is to construct reset FPGA command
     * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
     * @return UINT16 value
     */
    UINT16 resetFpgaCommand(SINT8 *s8Data);

    /** @fn UINT16 resetArDeviceCommand(SINT8 *s8Data)
     * @brief This function is to construct reset AR device command
     * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
     * @return UINT16 value
     */
    UINT16 resetArDeviceCommand(SINT8 *s8Data);

    /** @fn UINT16 configFpgaCommand(SINT8 *s8Data, <!--
     * -->                                 UINT8 *pu8Data, UINT16 u16DataLen)
     * @brief This function is to construct FPGA config command
     * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
     * @param [in] pu8Data [UINT8 *] - Array filled with command packet data
     * @param [in] u16DataLen [UINT16] - Length of command packet data
     * @return UINT16 value
     */
    UINT16 configFpgaCommand(SINT8 *s8Data, UINT8 *pu8Data, UINT16 u16DataLen);

    /** @fn UINT16 configEepromCommand(SINT8 *s8Data, <!--
     * -->                                 UINT8 *pu8Data, UINT16 u16DataLen)
     * @brief This function is to construct EEPROM config command
     * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
     * @param [in] pu8Data [UINT8 *] - Array filled with command packet data
     * @param [in] u16DataLen [UINT16] - Length of command packet data
     * @return UINT16 value
     */
    UINT16 configEepromCommand(SINT8 *s8Data, UINT8 *pu8Data,
                               UINT16 u16DataLen);

    /** @fn UINT16 startRecordCommand(SINT8 *s8Data)
     * @brief This function is to construct start record command
     * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
     * @return UINT16 value
     */
    UINT16 startRecordCommand(SINT8 *s8Data);

    /** @fn UINT16 stopRecordCommand(SINT8 *s8Data)
     * @brief This function is to construct stop record command
     * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
     * @return UINT16 value
     */
    UINT16 stopRecordCommand(SINT8 *s8Data);

    /** @fn UINT16 stopRecordAsyncCommand(SINT8 *s8Data)
     * @brief This function is to construct stop record from cli_control tool
     * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
     * @return UINT16 value
     */
    UINT16 stopRecordAsyncCommand(SINT8 *s8Data);

    /** @fn UINT16 readFpgaVersionCommand(SINT8 *s8Data)
     * @brief This function is to construct read FPGA version command
     * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
     * @return UINT16 value
     */
    UINT16 readFpgaVersionCommand(SINT8 *s8Data);

    /** @fn UINT16 configDataPacketCommand(SINT8 *s8Data, <!--
     * -->                                 UINT8 *pu8Data, UINT16 u16DataLen)
     * @brief This function is to construct record delay config command
     * @param [out] s8Data [SINT8 *] - UDP command packet filled in this array
     * @param [in] pu8Data [UINT8 *] - Array filled with command packet data
     * @param [in] u16DataLen [UINT16] - Length of command packet data
     * @return UINT16 value
     */
    UINT16 configDataPacketCommand(SINT8 *s8Data, UINT8 *pu8Data,
                                   UINT16 u16DataLen);

};

#endif // CMDSPROTO_H
