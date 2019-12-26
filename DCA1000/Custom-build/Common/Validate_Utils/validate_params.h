/**
 * @file validate_params.h
 *
 * @author JP
 *
 * @version 0.2
 *
 * @brief This file contains API definitions for validating JSON file
 * parameters for DCA1000EVM system
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
/// 0.1            22 Dec 2018       JP          Created
/// 0.2            29 Mar 2019       JP          DLL supportive APIs updated
///*****************************************************************************

#ifndef VALIDATE_PARAMS_H
#define VALIDATE_PARAMS_H

#include "../DCA1000_API/dca_types.h"

/** @fn SINT32 validateIpAddress(SINT8 *nodeData)
 * @brief This function is to validate the IP address in JSON file
 * @param [in] nodeData [SINT8 *] - IP address of String type
 * @return SINT32 value
 */
SINT32 validateIpAddress(SINT8 *nodeData);

/** @fn SINT32 validateIPAddr(UINT8 nodeData[])
 * @brief This function is to validate the IP address in JSON file
 * @param [in] nodeData [UINT8[] ] - IP address of Integer type
 * @return SINT32 value
 */
SINT32 validateIPAddr(UINT8 nodeData[]);

/** @fn SINT32 validatePortNumber(ULONG u32PortNum)
 * @brief This function is to validate the port number in JSON file
 * @param [in] u32PortNum [ULONG] - Port number of Integer type
 * @return SINT32 value
 */
SINT32 validatePortNumber(ULONG u32PortNum);

/** @fn SINT32 validatePortNum(SINT8 *nodeData)
 * @brief This function is to validate the port number in JSON file
 * @param [in] nodeData [SINT8 *] - Port number of String type
 * @return SINT32 value
 */
SINT32 validatePortNum(SINT8 *nodeData);

/** @fn SINT32 validateMacAddress(SINT8 *nodeData)
 * @brief This function is to validate the MAC address in JSON file
 * @param [in] nodeData [SINT8 *] - MAC address of String type
 * @return SINT32 value
 */
SINT32 validateMacAddress(SINT8 *nodeData);

/** @fn SINT32 validateMACAddr(UINT8 nodeData[])
 * @brief This function is to validate the MAC address in JSON file
 * @param [in] nodeData [UINT8[] ] - MAC address of Integer type
 * @return SINT32 value
 */
SINT32 validateMACAddr(UINT8 nodeData[]);

/** @fn SINT32 validateRecFileMaxsize(SINT8 *nodeData)
 * @brief This function is to validate maximum file size config in JSON file
 * @param [in] nodeData [SINT8 *] - Maximum file size of String type
 * @return SINT32 value
 */
SINT32 validateRecFileMaxsize(SINT8 *nodeData);

/** @fn SINT32 validateFileMaxSize(SINT32 s32FileSize)
 * @brief This function is to validate maximum file size config in JSON file
 * @param [in] s32FileSize [SINT32] - Maximum file size of Integer type
 * @return SINT32 value
 */
SINT32 validateFileMaxSize(SINT32 s32FileSize);

/** @fn SINT32 validatePacketDelay(SINT8 *nodeData)
 * @brief This function is to validate packet delay config in JSON file
 * @param [in] nodeData [SINT8 *] - Packet delay
 * @return SINT32 value
 */
SINT32 validatePacketDelay(SINT8 *nodeData);

/** @fn SINT32 validatePortNumsForConflicts(UINT32 u32RecordPort, UINT32 u32ConfigPort)
 * @brief This function is to verify any conflicts between two ports in JSON file
 * @param [in] u32RecordPort [UINT32] - Record port
 * @param [in] u32ConfigPort [UINT32] - Config port
 * @return SINT32 value
 */
SINT32 validatePortNumsForConflicts(UINT32 u32RecordPort, UINT32 u32ConfigPort);

/** @fn SINT32 validateBytesRecStopConfig(SINT8 *nodeData, SINT8 s8LvdsMode)
 * @brief This function is to validate bytes stop mode config in JSON file
 * @param [in] nodeData [SINT8 *] - Number of bytes of String type
 * @param [in] s8LvdsMode [SINT8] - LVDS mode
 * @return SINT32 value
 */
SINT32 validateBytesRecStopConfig(SINT8 *nodeData, SINT8 s8LvdsMode);

/** @fn SINT32 validateBytesStopConfig(ULONG u32Bytes,SINT8 s8LvdsMode)
 * @brief This function is to validate bytes stop mode config in JSON file
 * @param [in] u32Bytes [ULONG] - Number of bytes of Integer type
 * @param [in] s8LvdsMode [SINT8] - LVDS mode
 * @return SINT32 value
 */
SINT32 validateBytesStopConfig(ULONG u32Bytes, SINT8 s8LvdsMode);

/** @fn SINT32 validateFramesRecStopConfig(SINT8 *nodeData)
 * @brief This function is to validate frames stop mode config in JSON file
 * @param [in] nodeData [SINT8 *] - Number of frames of String type
 * @return SINT32 value
 */
SINT32 validateFramesRecStopConfig(SINT8 *nodeData);

/** @fn SINT32 validateFramesStopConfig(ULONG u32Frames)
 * @brief This function is to validate frames stop mode config in JSON file
 * @param [in] u32Frames [ULONG] - Number of frames of Integer type
 * @return SINT32 value
 */
SINT32 validateFramesStopConfig(ULONG u32Frames);


/** @fn SINT32 validateDurationRecStopConfig(SINT8 *nodeData)
 * @brief This function is to validate duration stop mode config in JSON file
 * @param [in] nodeData [SINT8 *] - Duration in seconds of String type
 * @return SINT32 value
 */
SINT32 validateDurationRecStopConfig(SINT8 *nodeData);

/** @fn SINT32 validateDurationStopConfig(ULONG u32Duration)
 * @brief This function is to validate duration stop mode config in JSON file
 * @param [in] u32Duration [ULONG] - Duration in seconds of Integer type
 * @return SINT32 value
 */
SINT32 validateDurationStopConfig(ULONG u32Duration);

#endif //VALIDATE_PARAMS_H
