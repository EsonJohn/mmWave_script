/**
 * @file configdatarecv.cpp
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


///****************************************************************************
/// HISTORY :
/// VERSION        DATE              AUTHOR      CHANGE DESCRIPTION
/// 0.1            07 Dec 2018       JP          Created
/// 0.2            29 Mar 2019       JP          Stop record timeout logic
///                                              updated
///****************************************************************************

///****************
/// Includes
///****************

#include "../Common/DCA1000_API/rf_api.h"

#include "defines.h"
#include "commandsprotocol.h"
#include "extern.h"

#include "configdatarecv.h"
#include "recorddatarecv.h"

///****************
/// Externs
///****************

/** ADC data record process -  class object             */
extern cUdpDataReceiver     objUdpDataRecv;

/** CP data record process -  class object              */
extern cUdpDataReceiver		objUdpCpDataRecv;

/** CQ data record process -  class object              */
extern cUdpDataReceiver		objUdpCqDataRecv;

/** R4F data record process -  class object             */
extern cUdpDataReceiver		objUdpR4fDataRecv;

/** DSP data record process -  class object             */
extern cUdpDataReceiver		objUdpDspDataRecv;

/** Command response structure - Start record command   */
extern DATA_CAPTURE_RESP    configResp;

/** Start record command timeout wait event
 * Separate thread will be running for handling config port response packets,
 * hence start record command responses will be handled using the wait event
 */
extern OSAL_SIGNAL_HANDLE_TYPE sgnCmdTimeoutWaitEvent;

/** Thread running state        */
bool bThreadState_config = false;

/** Socket aliveness state      */
bool bSocketState_config = false;

/** @fn cUdpReceiver::cUdpReceiver()
 * @brief This constructor function is to initialize the class member variables
 */
cUdpReceiver::cUdpReceiver()
{
    bSocketState_config = false;
    bThreadState_config = false;
}

/** @fn void cUdpReceiver::setSocketOpen()
 * @brief This function is to set configuration command response socket <!--
 * --> as open
 */
void cUdpReceiver::setSocketOpen()
{
    bSocketState_config   = true;
    bThreadState_config   = true;
}

/** @fn void cUdpReceiver::setSocketClose()
 * @brief This function is to reset configuration command response  <!--
 * --> socket as closed
 */
void cUdpReceiver::setSocketClose()
{
    bSocketState_config   = false;
    bThreadState_config   = false;
}

/** @fn void cUdpReceiver::getThreadStatus()
 * @brief This function is to read configuration command response  <!--
 * --> thread status
 * @return boolean value
 */
bool cUdpReceiver::getThreadStatus()
{
     return bThreadState_config;
}

/** @fn void cUdpReceiver::readConfigDatagrams()
 * @brief This function is a thread process to read configuration <!--
 * -->command response through UDP and handles signals from FPGA
 */
void cUdpReceiver::readConfigDatagrams()
{
    struct sockaddr_in SenderAddr_cfg;
    socklen_t s32SenderAddrSize_cfg = sizeof(SenderAddr_cfg);
    SINT32		s32RecvSize_cfg = 0;
    UINT16		u16Data = 0;
    UINT16      u16SysStatus = 0;
    SINT32      s32Result = 0;
    fd_set readset;
    struct timeval       timeout;

#ifdef ENABLE_DEBUG
    SINT8       s8DebugMsgs[MAX_NAME_LEN];
#endif

    while(bSocketState_config)
    {
        s32RecvSize_cfg = recvfrom(sRFDCCard_SockInfo.s32EthConfSock,
                                   (SINT8 *)&configResp,
                                   sizeof(DATA_CAPTURE_RESP), 0,
                                   (struct sockaddr *)&SenderAddr_cfg,
                                   &s32SenderAddrSize_cfg);

        if((s32RecvSize_cfg > 0) && (bThreadState_config))
		{
            if((configResp.u16Header == HEADER_START_BYTES) &&
                (configResp.u16Footer == FOOTER_STOP_BYTES))
			{

				switch(configResp.u16CmdCode)
                {
                case CMD_CODE_START_RECORD:
                    osalObj.SignalEvent(&sgnCmdTimeoutWaitEvent);

					if(configResp.u16Status != SUCCESS_STATUS)
					{
                        objUdpDataRecv.setThreadStop();
                        objUdpCpDataRecv.setThreadStop();
                        objUdpCqDataRecv.setThreadStop();
                        objUdpR4fDataRecv.setThreadStop();
                        objUdpDspDataRecv.setThreadStop();
					}
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nRECORD_START_CMD_CODE: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
					break;
                case CMD_CODE_CLI_ASYNC_RECORD_STOP:
                    RFDCARD_Callback(CMD_CODE_CLI_ASYNC_RECORD_STOP, 0);
                    break;
                case CMD_CODE_STOP_RECORD:
                    osalObj.SignalEvent(&sgnCmdTimeoutWaitEvent);

#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nRECORD_STOP_CMD_CODE: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
					break;
                case CMD_CODE_SYSTEM_ASYNC_STATUS:
                    for(u16SysStatus = 0; u16SysStatus < MAX_SYSTEM_STATUS;
                        u16SysStatus ++)
					{
						u16Data = 1;
                        u16Data <<= u16SysStatus;
						if((configResp.u16Status & u16Data) == u16Data)
						{
                            switch (u16SysStatus)
							{
							case STS_NO_LVDS_DATA:
                                RFDCARD_Callback(configResp.u16CmdCode,
                                    STS_NO_LVDS_DATA);
                                if(sRFDCCard_StartRecConfig.eRecordStopMode ==
                                        DURATION)
                                {
                                    sRFDCCard_StartRecConfig.u32DurationToCapture = 1;
                                    osalObj.SignalEvent(&
                                                sgnDurationStopModeWaitEvent);
                                }

#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nSTS_NO_LVDS_DATA: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
								break;

							case STS_NO_HEADER:
                                RFDCARD_Callback(configResp.u16CmdCode,
                                    STS_NO_HEADER);
                                if(sRFDCCard_StartRecConfig.eRecordStopMode ==
                                        DURATION)
                                {
                                    sRFDCCard_StartRecConfig.u32DurationToCapture = 1;
                                    osalObj.SignalEvent(&
                                                sgnDurationStopModeWaitEvent);
                                }

#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nSTS_NO_HEADER: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
								break;
                            case STS_EEPROM_FAILURE:
								RFDCARD_Callback(configResp.u16CmdCode,
									STS_EEPROM_FAILURE);
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nSTS_EEPROM_FAILURE: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
								break;
							case STS_SD_CARD_DETECTED:
								RFDCARD_Callback(configResp.u16CmdCode,
									STS_SD_CARD_DETECTED);
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nSTS_SD_CARD_DETECTED: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
								break;
							case STS_SD_CARD_REMOVED:
								RFDCARD_Callback(configResp.u16CmdCode,
									STS_SD_CARD_REMOVED);
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nSTS_SD_CARD_REMOVED: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
								break;
							case STS_SD_CARD_FULL:
								RFDCARD_Callback(configResp.u16CmdCode,
									STS_SD_CARD_FULL);
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nSTS_SD_CARD_FULL: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
								break;
							case STS_MODE_CONFIG_FAILURE:
								RFDCARD_Callback(configResp.u16CmdCode,
									STS_MODE_CONFIG_FAILURE);
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nSTS_MODE_CONFIG_FAILURE: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
								break;
							case STS_DDR_FULL:
								RFDCARD_Callback(configResp.u16CmdCode,
									STS_DDR_FULL);
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nSTS_DDR_FULL: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
                                break;
                            case STS_REC_COMPLETED:
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nValidating socket buffer empty");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif

                                /** Raw socket buffer                                                    */
                                do
                                {
                                    FD_ZERO(&readset);
                                    FD_SET(sRFDCCard_SockInfo.s32DataSock[RAW_DATA_INDEX],
                                           &readset);
                                    timeout.tv_sec  = 0;
                                    timeout.tv_usec = 0;
                                    s32Result = select(0, &readset, NULL, NULL,
                                                       &timeout);
                                } while ((s32Result > 0));

                                /** CP socket buffer                                                    */
                                do
                                {
                                    FD_ZERO(&readset);
                                    FD_SET(sRFDCCard_SockInfo.s32DataSock[CP_DATA_1_INDEX],
                                           &readset);
                                    timeout.tv_sec  = 0;
                                    timeout.tv_usec = 0;
                                    s32Result = select(0, &readset, NULL, NULL,
                                                       &timeout);
                                } while ((s32Result > 0));

                                /** CQ socket buffer                                                    */
                                do
                                {
                                    FD_ZERO(&readset);
                                    FD_SET(sRFDCCard_SockInfo.s32DataSock[CQ_DATA_2_INDEX],
                                           &readset);
                                    timeout.tv_sec  = 0;
                                    timeout.tv_usec = 0;
                                    s32Result = select(0, &readset, NULL, NULL,
                                                       &timeout);
                                } while ((s32Result > 0));

                                /** R4f socket buffer                                                   */
                                do
                                {
                                    FD_ZERO(&readset);
                                    FD_SET(sRFDCCard_SockInfo.s32DataSock[R4F_DATA_3_INDEX],
                                           &readset);
                                    timeout.tv_sec  = 0;
                                    timeout.tv_usec = 0;
                                    s32Result = select(0, &readset, NULL, NULL,
                                                       &timeout);
                                } while ((s32Result > 0));

                                /** Dsp socket buffer                                                   */
                                do
                                {
                                    FD_ZERO(&readset);
                                    FD_SET(sRFDCCard_SockInfo.s32DataSock[DSP_DATA_4_INDEX],
                                           &readset);
                                    timeout.tv_sec  = 0;
                                    timeout.tv_usec = 0;
                                    s32Result = select(0, &readset, NULL, NULL,
                                                       &timeout);
                                } while ((s32Result > 0));

                        #ifdef ENABLE_DEBUG
                                sprintf(s8DebugMsgs, "\n\ns32Result = %d",
                                        s32Result);
                                DEBUG_FILE_WRITE(s8DebugMsgs);
                        #endif

                                if(!gbRecStopCmdSent)
                                {
                                    RFDCARD_Callback(configResp.u16CmdCode,
                                                 STS_REC_COMPLETED);
                                }

                                if(sRFDCCard_StartRecConfig.eRecordStopMode ==
                                        DURATION)
                                {
                                    sRFDCCard_StartRecConfig.u32DurationToCapture = 1;
                                    osalObj.SignalEvent(&
                                                sgnDurationStopModeWaitEvent);
                                }

#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nSTS_REC_COMPLETED: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
                                break;
							case STS_LVDS_BUFFER_FULL:
								RFDCARD_Callback(configResp.u16CmdCode,
                                                 STS_LVDS_BUFFER_FULL);
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nSTS_LVDS_BUFFER_FULL: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
                                break;
							}
						}
					}

					break;
				}
			}
			else
			{
                RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                                 STS_INVALID_RESP_PKT_ERR);
#ifdef ENABLE_DEBUG
        sprintf(s8DebugMsgs, "\n\nINVALID_RESP_PKT_ERROR_CODE: Received");
        DEBUG_FILE_WRITE(s8DebugMsgs);
#endif
				break;
			}
		}
    }
}
