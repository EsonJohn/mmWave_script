/**
 * @file globals.h
 *
 * @author JP
 *
 * @version 0.2
 *
 * @brief This file contains macros ad structures definitions
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
/// 0.2            17 Jan 2018       JP          Inline processing added and
///                                              Review comments incorporated
///*****************************************************************************


#ifndef GLOBALS_H
#define GLOBALS_H

///****************
/// Includes
///****************

#include "../Common/rf_api_internal.h"

/** To avoid structure padding                                               */
#pragma pack (1)

///****************
/// Defines
///****************

/** Shared memory prefix name followed by config port to read process states*/
#define CLI_SHM_PREFIX_NAME             "clishm_"

/** CLI log file name   */
#define CLI_LOG_NAME                    "CLI_LogFile.txt"

/** CLI log filename prefix */
#define CLI_LOG_PREFIX_NAME             "CLI_LogFile_"

/** Command - Configure FPGA                                                */
#define CMD_CONFIG_FPGA                 "fpga"

/** Command - Configure EEPROM                                              */
#define CMD_CONFIG_EEPROM               "eeprom"

/** Command - Reset FPGA                                                    */
#define CMD_RESET_FPGA                  "reset_fpga"

/** Command - Reset AR Device                                               */
#define CMD_RESET_AR_DEV                "reset_ar_device"

/** Command - Invoke cli_record tool as a separate process                  */
#define CMD_START_RECORD                "start_record"

/** Command - Trigger cli_record tool to stop the record process            */
#define CMD_STOP_RECORD                 "stop_record"

/** Command - Configure Record delay                                        */
#define CMD_CONFIG_RECORD               "record"

/** Command - Read DLL Version                                              */
#define CMD_READ_DLL_VER                "dll_version"

/** Command - Read DLL Version                                              */
#define CMD_READ_CLI_VER                "cli_version"

/** Command - Read FPGA Version                                             */
#define CMD_READ_FPGA_VER               "fpga_version"

/** Command - Query record/playback process from shared memory              */
#define CMD_QUERY_CLI_PROC_STATUS       "query_status"

/** Command - Query DCA1000EVM system status                                */
#define CMD_QUERY_SYSTEM_ALIVENESS      "query_sys_status"

/** Command - Help in the application                                       */
#define CMD_HELP_S_CLI_APP              "-h"

/** Command - Help in the application                                       */
#define CMD_HELP_CLI_APP                "help"

/** Command - Quiet Mode in the application                                 */
#define CMD_QUIET_MODE_CLI_APP          "-q"

/** Command - Enable post processing of the recording                       */
#define CMD_ENABLE_POST_PROCESSING      "-pp"

/** Minimum config timer - FPGA config                                      */
#define MIN_CONFIG_TIMER                    0

/** Maximum config timer - FPGA config                                      */
#define MAX_CONFIG_TIMER                    255

/** Minimum port number - FPGA config                                       */
#define MIN_CONFIG_PORT                     1

/** Maximum port number - FPGA config                                       */
#define MAX_CONFIG_PORT                     65535

/** Minimum record delay - Record config                                    */
#define MIN_RECORD_DELAY                    5

/** Maximum record delay - Record config                                    */
#define MAX_RECORD_DELAY                    500

/** Minimum File size in MB - Record config                                 */
#define MIN_RECORD_FILE_SIZE                1

/** Maximum File size in MB - Record config                                 */
#define MAX_RECORD_FILE_SIZE                1024

/** Minimum bytes stop mode in bytes - Record config                        */
#define MIN_RECORD_BYTES_STOP_MODE          128

/** Maximum bytes stop mode in bytes - Record config                        */
#define MAX_RECORD_BYTES_STOP_MODE          0xFFFFFFFF

/** Minimum frames stop mode - Record config                                */
#define MIN_RECORD_FRAMES_STOP_MODE         1

/** Maximum frames stop mode - Record config                                */
#define MAX_RECORD_FRAMES_STOP_MODE         0xFFFF

/** Minimum duration stop mode - Record config                              */
#define MIN_RECORD_DURATION_STOP_MODE       40

/** Maximum duration stop mode  - Record config                             */
#define MAX_RECORD_DURATION_STOP_MODE       0xFFFFFFFF

/** Fpga config - Timer value                                               */
#define FPGA_CONFIG_DEFAULT_TIMER           30

/** SUCCESS status (generic) */
#define SUCCESS_STATUS                      0

/** Failure status (generic) */
#define FAILURE_STATUS                      1

/** 16 bit data size in bytes */
#define UINT16_DATA_SIZE                    2

/** CLI command timeout duration in millisec */
#define CLI_CMD_TIMEOUT_DURATION            7000

/** Shared memory polling frequency in millisec for record status */
#define MILLI_SEC_TO_READ_SHM               500

/** Delay before printing stop record status */
#define MIN_MILLI_SEC_SLEEP_TO_DISP         100

/** CLI Control tool verison                        */
#define CLI_CTRL_VERSION 							"1.0"

/** CLI Record tool verison                         */
#define CLI_REC_VERSION 							"1.0"

///****************
/// Structures
///****************

/** Shared Memory - Reading states of record               */
typedef struct
{
    /** Command Code of the record command                 */
    UINT16 u16CommandCode;

    /** Command status of the record process               */
    SINT32 s32CommandStatus;

    /** Async FPGA status [if Record is in progress]
     *	 Each bit can be set for the defined async status from FPGA
     */
    UINT32 u32AsyncStatus;

    /** Inline process summary                             */
    strRFDCCard_InlineProcStats strInlineProcStats;

}SHM_PROC_STATES;

#endif // GLOBALS_H

