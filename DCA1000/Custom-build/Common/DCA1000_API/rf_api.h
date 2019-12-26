/**
 * @file rf_api.h
 *
 * @author JP
 *
 * @version 0.5
 *
 * @brief This file contains API declarations for DCA1000EVM system which can
 * be accessible and exported to any other calling application
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
/// 0.2            17 Jan 2019       JP          Inline processing added and
///                                              Review comments incorporated
/// 0.3            14 Feb 2019       JP          Maintain export functions and
///                                              Review comments incorporated
/// 0.4            29 Mar 2019       JP          Stop record timeout logic
///                                              updated and clode cleanup
/// 0.5            09 Apr 2019       JP          Reordering record data as
///                                              user configurable option
///****************************************************************************

#ifndef RF_API_H
#define RF_API_H

///****************
/// Includes
///****************

/** Standard data type declarations                                          */
#include "dca_types.h"

/** Standard declarations for date and time                                  */
#include <time.h>

/** Socket includes for Windows / Linux                                      */
#if defined _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>

    /** Export Macro definition for APIs */
    #define EXPORT __declspec(dllexport)
#else
    /** Export Macro definition for APIs */
    #define EXPORT __attribute__((visibility("default")))

#endif

/** To avoid structure padding                                               */
#pragma pack (1)

///****************
/// Defines
///****************

/** Macro definition for command return value                                */
#define STATUS                              SINT32

/** Success status for command                                               */
#define	STS_RFDCCARD_SUCCESS                (0)

/** Invalid input parameters status for command                              */
#define STS_RFDCCARD_INVALID_INPUT_PARAMS   (-1)

/** OS error status for command                                              */
#define STS_RFDCCARD_OS_ERR                 (-2)

/** UDP write failure status for command                                     */
#define STS_RFDCCARD_UDP_WRITE_ERR          (-3)

/** Ethernet cable connection error                                          */
#define STS_RFDCCARD_ETH_CABLE_CON_ERROR    (-4)

/** Command response timeout error                                           */
#define	STS_RFDCCARD_TIMEOUT_ERR            (-5)

/** Event timeout error                                                      */
#define	STS_RFDCCARD_EVENT_TIMEOUT_ERR      (-6)

/** Reset FPGA command code                                                  */
#define CMD_CODE_RESET_FPGA                 0x01

/** Reset AR device command code                                             */
#define CMD_CODE_RESET_AR_DEV               0x02

/** Configure FPGA data modes command code                                   */
#define CMD_CODE_CONFIG_FPGA                0x03

/** Configure EEPROM command code                                            */
#define CMD_CODE_CONFIG_EEPROM              0x04

/** Start record command code                                                */
#define CMD_CODE_START_RECORD               0x05

/** Stop record command code                                                 */
#define CMD_CODE_STOP_RECORD                0x06

/** Playback start command code                                              */
#define CMD_CODE_START_PLAYBACK             0x07

/** Playback stop command code                                               */
#define CMD_CODE_STOP_PLAYBACK              0x08

/** System aliveness check command code                                      */
#define CMD_CODE_SYSTEM_ALIVENESS           0x09

/** System status command code                                               */
#define CMD_CODE_SYSTEM_ASYNC_STATUS        0x0A

/** Configure packet data delay and size command code                        */
#define CMD_CODE_CONFIG_RECORD              0x0B

/** Configure AR device data mode command code                               */
#define CMD_CODE_CONFIG_AR_DEV              0x0C

/** Initiate FPGA playback command code                                      */
#define CMD_CODE_INIT_FPGA_PLAYBACK			0x0D

/** Read FPGA version command code                                           */
#define CMD_CODE_READ_FPGA_VERSION			0x0E

/** CLI - Read record status from shared memory command code                 */
#define CMD_CODE_CLI_PROC_STATUS_SHM        0x20

/** CLI - Read DLL version command code                                      */
#define CMD_CODE_CLI_READ_DLL_VERSION       0x21

/** CLI - Read CLI version command code                                      */
#define CMD_CODE_CLI_READ_CLI_VERSION       0x23

/** CLI - Async Record stop command code                                     */
#define CMD_CODE_CLI_ASYNC_RECORD_STOP      0x24

/** Record packet out of sequence error code                                 */
#define STS_RECORD_PKT_OUT_OF_SEQ           0xC3

/** Record progress status code                                              */
#define STS_RECORD_IS_IN_PROGRESS           0xC4

/** Playback from GUI to FPGA completed status code                          */
#define STS_GUI_PLAYBACK_COMPLETED			0xC5

/** Playback file open error code                                            */
#define STS_PLAYBACK_FILE_OPEN_ERR          0xC6

/** Playback UDP write failure error code                                    */
#define STS_PLAYBACK_UDP_WRITE_ERR          0xC7

/** Playback progress status code                                            */
#define STS_PLAYBACK_IS_IN_PROGRESS         0xC8

/** Record thread timeout error code                                         */
#define STS_CAPTURE_THREAD_TIMEOUT          0xC9

/** Command header start bytes                                               */
#define HEADER_START_BYTES                  0xA55A

/** Command footer start bytes                                               */
#define FOOTER_STOP_BYTES                   0xEEAA

/** Maximum length of a string                                               */
#define MAX_NAME_LEN                        255

/** Maximum length of version info                                           */
#define MAX_VERSION_BUF_LEN                 20

/** Number of data types for record                                          */
#define NUM_DATA_TYPES                      5

/** Maximum data bytes in command request                                    */
#define MAX_DATA_BYTES                      504

///****************
/// Enumerations
///****************

/** @enum SYS_ASYNC_STATUS
 *  @brief System async status from FPGA. @n <!--
 *  --> DCA1000EVM FPGA will send the system status as asynchronous UDP packet <!--
 *  --> through Ethernet over the config port. DLL will recieve it and <!--
 *  --> communicate the status using the callback to the calling application
 *  @pre Calling application must register a callback function with  <!--
 *  --> the DLL using StatusRFDCCard_EventRegister API
 */
enum SYS_ASYNC_STATUS
{
    /** No lvds data  status         */
    STS_NO_LVDS_DATA = 0,

    /** No header status             */
    STS_NO_HEADER,

    /** EEPROM failure status        */
    STS_EEPROM_FAILURE,

    /** SD card detected status      */
    STS_SD_CARD_DETECTED,

    /** SD card removed status       */
    STS_SD_CARD_REMOVED,

    /** SD card full status          */
    STS_SD_CARD_FULL,

    /** Mode config failure status   */
    STS_MODE_CONFIG_FAILURE,

    /** DDR full status              */
    STS_DDR_FULL,

    /** Record completed status      */
    STS_REC_COMPLETED,

    /** LVDS buffer full status      */
    STS_LVDS_BUFFER_FULL,

    /** Playback completed status    */
    STS_PLAYBACK_COMPLETED,

    /** Playback out of sequence     */
    STS_PLAYBACK_OUT_OF_SEQ
};

/** @enum DLL_ASYNC_STATUS
 *  @brief System async status from DLL. @n <!--
 *  --> DCA1000EVM DLL will send the system status during the record process <!--
 *  --> asynchronously using the callback to the calling application
 *  @pre Calling application must register a callback function with  <!--
 *  --> the DLL using StatusRFDCCard_EventRegister API
 */
enum DLL_ASYNC_STATUS
{
    /** Record packet out of sequence            */
    STS_REC_PKT_OUT_OF_SEQ = 0x10,

    /** Record process timeout                   */
    STS_REC_PROC_TIMEOUT,

    /** Record file creation failed              */
    STS_REC_FILE_CREATION_ERR,

    /** Record process reordering failed         */
    STS_REC_REORDERING_ERR,

    /** Invalid response packet                  */
    STS_INVALID_RESP_PKT_ERR,

    /** Record inline buffer allocation failed   */
    STS_REC_INLINE_BUF_ALLOCATION_ERR
};

/** @enum SYS_CLI_REC_PROC_STATUS
 *  @brief Record process states. @n <!--
 *  --> DCA1000EVM CLI Record tool will update the record process states <!--
 *  --> in the shared memory
 *  @post CLI Control tool can read the record process states using <!--
 *  --> shared memory
 */
enum SYS_CLI_REC_PROC_STATUS
{
    /** Record is in progress         */
    STS_CLI_REC_PROC_IS_IN_PROG = 0,

    /** Record is stopped             */
    STS_CLI_REC_PROC_STOPPED,

    /** Start cmd is initiated        */
    STS_CLI_REC_PROC_START_INIT,

    /** Start cmd is failed           */
    STS_CLI_REC_PROC_START_FAILED,

    /** Start cmd is initiated        */
    STS_CLI_REC_PROC_STOP_INIT,

    /** Stop cmd is failed            */
    STS_CLI_REC_PROC_STOP_FAILED
};

/** Data log mode                                                            */
typedef enum CONFIG_LOG_MODE
{
    /** Raw mode                     */
    RAW_MODE = 1,

    /** Multi mode                   */
    MULTI_MODE
}ConfigLogMode;


/** Data LVDS mode                                                           */
typedef enum CONFIG_LVDS_MODE
{
    /** AR1243 - 4 lane               */
    FOUR_LANE = 1,

    /** AR1642 - 2 lane               */
    TWO_LANE
}ConfigLvdsMode;


/** Data transfer mode                                                       */
typedef enum CONFIG_TRANSFER_MODE
{
    /** Capture mode                 */
    CAPTURE = 1,

    /** Playback mode                */
    PLAYBACK
}ConfigTransferMode;


/** Data format mode                                                         */
typedef enum CONFIG_FORMAT_MODE
{
    /** 12 bit mode                  */
    BIT12 = 1,

    /** 14 bit mode                  */
    BIT14,

    /** 16 bit mode                  */
    BIT16
}ConfigFormatMode;


/** Data capture mode                                                        */
typedef enum CONFIG_CAPTURE_MODE
{
    /** SD card storage              */
    SD_STORAGE =1,

    /** Ethernet stream              */
    ETH_STREAM
}ConfigCaptureMode;


/** Record stop mode                                                         */
typedef enum RECORD_STOP_MODE
{
    /** Bytes   */
    BYTES = 1,

    /** Duration */
    DURATION,

    /** Frames */
    FRAMES,

    /** Infinite */
    NON_STOP
}RecordStopMode;

///****************
/// Stucture Declarations
///****************

/** Data mode config for FPGA */
typedef struct FPGA_CONFIG_MODE
{
    /** Data log mode              */
    ConfigLogMode       	eLogMode;

    /** Data LVDS mode             */
    ConfigLvdsMode          eLvdsMode;

    /** Data transfer mode         */
    ConfigTransferMode     	eDataXferMode;

    /** Data capture mode          */
    ConfigCaptureMode      	eDataCaptureMode;

    /** Data format mode           */
    ConfigFormatMode       	eDataFormatMode;

    /** Timeout value for LVDS data*/
    UINT8                   u8Timer;
} strFpgaConfigMode;


/** System ethernet configuration                                            */
typedef struct ETH_CONFIG_MODE
{
    /** FPGA MAC address             */
    UINT8          	au8MacId[6];

    /** PC IP address                */
    UINT8       	au8PcIpAddr[4];

    /** FPGA IP address              */
    UINT8			au8Dca1000IpAddr[4];

    /** Record port number           */
    UINT32        	u32RecordPortNo;

    /** Config port number           */
    UINT32        	u32ConfigPortNo;
} strEthConfigMode;

/** Record mode configuration                                                */
typedef struct REC_CONFIG_MODE
{
    /** Record  packet delay         */
    UINT16          u16RecDelay;
} strRecConfigMode;

/** Start record modes configuration                                         */
typedef struct START_REC_CONFIG_MODE
{
    /** Record data file Base path                      */
    SINT8   s8FileBasePath[MAX_NAME_LEN];

    /** Record data file name prefix                    */
    SINT8 s8FilePrefix[MAX_NAME_LEN];

    /** Sequence number in file enable/disable          */
    bool bSequenceNumberEnable;

    /** MSB bit toggling in the captured data(16-bit)   */
    bool bMsbToggleEnable;

    /** Reordering data bytes in the captured data      */
    bool bReorderEnable;

    /** Sequence number in file enable/disable          */
    UINT16 u16MaxRecFileSize;

    /** Capture stop modes                              */
    RecordStopMode eRecordStopMode;

    /** Number of bytes to capture                      */
    UINT32 u32BytesToCapture;

    /** Duration to capture                             */
    UINT32 u32DurationToCapture;

    /** Number of frames to capture                     */
    UINT32 u32FramesToCapture;

    /** DataLogging Mode                                */
    ConfigLogMode eConfigLogMode;

    /** Data LVDS mode                                  */
    ConfigLvdsMode eLvdsMode;
}strStartRecConfigMode;

/** Inline processing statistics                                             */
typedef struct
{
    /** Data type header ID                         */
    SINT8 s8HeaderId[NUM_DATA_TYPES][MAX_NAME_LEN];

    /** First packet ID                             */
    UINT32 u32FirstPktId[NUM_DATA_TYPES];

    /** Last packet ID                              */
    UINT32 u32LastPktId[NUM_DATA_TYPES];

    /** Out of sequence count                       */
    ULONG64 u64OutOfSeqCount[NUM_DATA_TYPES];

    /** Received packets count                      */
    ULONG64 u64NumOfRecvdPackets[NUM_DATA_TYPES];

    /** Zero filled packets count                   */
    ULONG64 u64NumOfZeroFilledPackets[NUM_DATA_TYPES];

    /** Zero filled bytes count                     */
    ULONG64 u64NumOfZeroFilledBytes[NUM_DATA_TYPES];

    /** Packet start timestamp                      */
    time_t StartTime[NUM_DATA_TYPES];

    /** Packet end timestamp                        */
    time_t EndTime[NUM_DATA_TYPES];

    /** Packet out of sequence seen from offset     */
    UINT32 u32OutOfSeqPktFromOffset[NUM_DATA_TYPES];

    /** Packet out of sequence seen till offset     */
    UINT32 u32OutOfSeqPktToOffset[NUM_DATA_TYPES];

}strRFDCCard_InlineProcStats;

/** Command request protocol                                                 */
typedef struct
{
    /** Header                       */
    UINT16  u16Header;

    /** Command code                 */
    UINT16  u16CmdCode;

    /** Data size                    */
    UINT16  u16DataSize;

    /** Data                         */
    UINT8   strData[MAX_DATA_BYTES];

    /** Footer                       */
    UINT16  u16Footer;
}DATA_CAPTURE_REQ;


/** Command response protocol                                                */
typedef struct
{
    /** Header   					 */
    UINT16  u16Header;

    /** Command code                 */
    UINT16  u16CmdCode;

    /** Command status               */
    UINT16  u16Status;

    /** Footer                       */
    UINT16  u16Footer;
}DATA_CAPTURE_RESP;


/** Callback typedef for event handling                                      */
typedef void (*EVENT_HANDLER)
(
    /** Command code */
    UINT16 u16CmdCode,

    /** Command status */
    UINT16 u16Status
);

/** Callback typedef for record inlne processing handling                    */
typedef void (*INLINE_PROC_HANDLER)
(
    /** Inline process summary structure */
    strRFDCCard_InlineProcStats strInlineProcStats,

    /** Out of sequence set flag */
    bool u8OutOfSeqFlag,

    /** Data type index */
    UINT8 u8DataIndex
);

///*****************
/// API Declarations
///*****************

#ifdef __cplusplus
extern "C" {
#endif

/** @fn EXPORT STATUS ConnectRFDCCard_ConfigMode (strEthConfigMode sEthConfigMode)
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
EXPORT STATUS ConnectRFDCCard_ConfigMode
(
    strEthConfigMode	sEthConfigMode
);

/** @fn EXPORT STATUS ConnectRFDCCard_AsyncCommandMode (strEthConfigMode sEthConfigMode)
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
EXPORT STATUS ConnectRFDCCard_AsyncCommandMode
(
    strEthConfigMode	sEthConfigMode
);

/** @fn EXPORT STATUS ConnectRFDCCard_RecordMode (strEthConfigMode sEthConfigMode)
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
EXPORT STATUS ConnectRFDCCard_RecordMode
(
    strEthConfigMode	sEthConfigMode
);

/** @fn  EXPORT STATUS ConfigureRFDCCard_Fpga(strFpgaConfigMode sConfigMode)
 * @brief This function is to configure the DCA1000EVM system <!--
 * --> with the following mode configuration @n <!--
 * -->	1.	Logging Mode 		– RAW/MULTI @n <!--
 * -->	2.	LVDS Mode    		- FOUR/TWO LANE @n <!--
 * -->	3.	Data Transfer Mode 	– LVDS Capture/Playback @n <!--
 * -->	4.	Data Capture Mode 	– Ethernet Streaming/SD Card Storage @n  <!--
 * -->	5.	Data Format Mode  	- 12/14/16 bit @n <!--
 * -->  6.  Timer               - Timeout for LVDS data
 * @param [in] sConfigMode [strConfigMode] - Structure filled with config data
 * @pre Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_ConfigMode API
 * @return SINT32 value
 */
EXPORT STATUS ConfigureRFDCCard_Fpga
(
  strFpgaConfigMode      sConfigMode
);

/** @fn EXPORT STATUS ConfigureRFDCCard_Eeprom (strEthConfigMode sEthConfigMode)
 * @brief This function is to configure the EEPROM of DCA1000EVM  <!--
 * --> system with the following configuration @n <!--
 * -->	1.	FPGA MAC ID @n   <!--
 * -->	2.	PC and FPGA IP Address @n  <!--
 * -->	3.	Record port number @n  <!--
 * -->	4.	Configuration port number
 * @param [in] sEthConfigMode [strEthConfigMode] - <!--
 * --> Structure filled with config data
 * @pre Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_ConfigMode API
 * @return SINT32 value
 */
EXPORT STATUS ConfigureRFDCCard_Eeprom
(
  strEthConfigMode      sEthConfigMode
);

/** @fn EXPORT STATUS HandshakeRFDCCard(void)
 * @brief This function is to verify the DCA1000EVM system connectivity
 * @pre Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_ConfigMode API
 * @return SINT32 value
 */
EXPORT STATUS HandshakeRFDCCard(void);

/** @fn EXPORT STATUS DisconnectRFDCCard_RecordMode(void)
 * @brief This function is to close config and data port socket connection <!--
 * --> and disconnect from the DCA1000EVM system.
 * @pre Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_RecordMode API
 * @return SINT32 value
 */
EXPORT STATUS DisconnectRFDCCard_RecordMode (void);

/** @fn  EXPORT STATUS DisconnectRFDCCard_ConfigMode(void)
 * @brief This function is to close bound config port socket connection <!--
 * --> and disconnect from the DCA1000EVM system.
 * @pre Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_ConfigMode API
 * @return SINT32 value
 */
EXPORT STATUS DisconnectRFDCCard_ConfigMode (void);

/** @fn EXPORT STATUS DisconnectRFDCCard_AsyncCommandMode(void)
 * @brief This function is to close unbound config port socket connection <!--
 * --> which is used to send stop record async message to CLI Record tool.
 * @pre Application should be connected to CLI Record process using <!--
 * --> \ref ConnectRFDCCard_AsyncCommandMode API
 * @return SINT32 value
 */
EXPORT STATUS DisconnectRFDCCard_AsyncCommandMode (void);

/** @fn EXPORT STATUS StartRecordData(strStartRecConfigMode sStartRecConfigMode)
 * @brief This function is to start recording the data streamed over <!--
 * --> Ethernet from the DCA1000EVM system with the \ref <!--
 * --> strStartRecConfigMode structure configuration
 * @param [in] sStartRecConfigMode [strStartRecConfigMode] - <!--
 * --> Structure filled with record config data
 * @pre FPGA and record packet delay should be configured. <!--
 * --> Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_RecordMode API
 * @return SINT32 value
 */
EXPORT STATUS StartRecordData
(
    strStartRecConfigMode sStartRecConfigMode
);

/** @fn EXPORT STATUS StopRecordData(void)
 * @brief This function is to send command to FPGA to stop recording the <!--
 * --> data streamed over Ethernet from the DCA1000EVM system
 * @pre Record process should be running using \ref StartRecordData API
 * @return SINT32 value
 */
EXPORT STATUS StopRecordData (void);

/** @fn EXPORT STATUS StopRecordAsyncCmd(void)
 * @brief This function is to send a message to CLI record process over the <!--
 * --> config port to initiate stop recording data from the DCA1000EVM system <!--
 * --> using \ref StopRecordData API
 * @pre Record process should be running using \ref StartRecordData API. <!--
 * --> Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_AsyncCommandMode API
 * @return SINT32 value
 */
EXPORT STATUS StopRecordAsyncCmd (void);

/** @fn EXPORT STATUS ResetRFDCCard_FPGA(void)
 * @brief This function is to reset DCA1000EVM FPGA
 * @pre Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_ConfigMode API
 * @return SINT32 value
 */
EXPORT STATUS ResetRFDCCard_FPGA (void);

/** @fn  EXPORT STATUS ResetRadarEVM(void)
 * @brief This function is to reset RADAR AR device
 * @pre Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_ConfigMode API
 * @return SINT32 value
 */
EXPORT STATUS ResetRadarEVM (void);

/** @fn EXPORT STATUS StatusRFDCCard_EventRegister (EVENT_HANDLER RFDCCard_EventCallback)
 * @brief This function is to register user event callback for handling <!--
 * --> async status from FPGA and stop record command response
 * @param [in] RFDCCard_EventCallback  [EVENT_HANDLER] - Callback function
 * @return SINT32 value
 */
EXPORT STATUS StatusRFDCCard_EventRegister
(
    EVENT_HANDLER   RFDCCard_EventCallback
);

/** @fn EXPORT STATUS ReadRFDCCard_FpgaVersion(SINT8 *s8FpgaVersion)
 * @brief This function is to read FPGA version
 * @param [out] s8FpgaVersion [SINT8 *] - Array filled with version
 * @pre Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_ConfigMode API
 * @return SINT32 value
 */
EXPORT STATUS ReadRFDCCard_FpgaVersion(SINT8 *s8FpgaVersion);

/** @fn EXPORT STATUS ReadRFDCCard_DllVersion(SINT8 *s8DllVersion)
 * @brief This function is to read API DLL version
 * @param [out] s8DllVersion [SINT8 *] - Array filled with version
 * @return SINT32 value
 */
EXPORT STATUS ReadRFDCCard_DllVersion(SINT8 *s8DllVersion);

/** @fn EXPORT STATUS ConfigureRFDCCard_Record(strRecConfigMode sRecConfigMode)
 * @brief This function is to configure record data packet delay <!--
 * --> in data recording with the following configuration @n <!--
 * -->	1.	Record delay
 * @param [in] sRecConfigMode [strRecConfigMode] - <!--
 * --> Structure filled with config data
 * @pre Application should be connected to DCA1000EVM system using <!--
 * --> \ref ConnectRFDCCard_ConfigMode API
 * @return SINT32 value
 */
EXPORT STATUS ConfigureRFDCCard_Record(strRecConfigMode sRecConfigMode);

/** @fn EXPORT STATUS RecInlineProcStats_EventRegister (INLINE_PROC_HANDLER RecordStats_Callback)
 * @brief This function is to register user event callback for updating <!--
 * --> inline processing statistics of recording process
 * @param [in] RecordStats_Callback  [INLINE_PROC_HANDLER] - <!--
 * --> Callback function
 * @pre Record process should be running using \ref StartRecordData API
 * @return SINT32 value
 */
EXPORT STATUS RecInlineProcStats_EventRegister
(
    INLINE_PROC_HANDLER   RecordStats_Callback
);

#ifdef __cplusplus
}
#endif

#endif // RF_API_H
