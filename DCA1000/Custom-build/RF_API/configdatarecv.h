/**
 * @file configdatarecv.h
 *
 * @author JP
 *
 * @version 0.2
 *
 * @brief This file contains API definitions for handling configuration
 * command responses from DCA1000EVM system
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
///
/// 0.2            29 Mar 2019       JP          Stop record timeout logic
///                                              updated
///*****************************************************************************

///****************
/// Includes
///****************
#ifndef CONFIGDATARECV_H
#define CONFIGDATARECV_H

///****************
/// Includes
///****************

#include "../Common/DCA1000_API/dca_types.h"

/** @class cUdpReceiver
 * @brief This class provides support APIs for reading command response <!--
 * --> packets from DCA1000EVM system and signals GUI and corresponding process
 */
class cUdpReceiver
{
public:

    /** @fn cUdpReceiver()
     * @brief This constructor function is to initialize the class member variables
     */
    cUdpReceiver();

    /** @fn void setSocketOpen()
     * @brief This function is to set configuration command response socket <!--
     * --> as open
     */
    void setSocketOpen();

    /** @fn void setSocketClose()
     * @brief This function is to reset configuration command response  <!--
     * --> socket as closed
     */
    void setSocketClose();

    /** @fn void getThreadStatus()
     * @brief This function is to read configuration command response  <!--
     * --> thread status
     * @return boolean value
     */
    bool getThreadStatus();

    /** @fn void readConfigDatagrams()
     * @brief This function is a thread process to read configuration <!--
     * -->command response through UDP and handles signals from FPGA
     */
    void readConfigDatagrams();

};

#endif // CONFIGDATARECV_H
