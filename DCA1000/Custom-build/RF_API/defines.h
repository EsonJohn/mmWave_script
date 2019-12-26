/**
 * @file defines.h
 *
 * @author JP
 *
 * @version 0.1
 *
 * @brief This file contains Macro definitions
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

#ifndef DEFINES_H
#define DEFINES_H

/** DCA1000EVM DLL version                                                   */
#define DCA1000EVM_DLL_VERSION              "1.0"

/** Success status of the command                                            */
#define SUCCESS_STATUS                      0

/** Failure status of the command                                            */
#define FAILURE_STATUS                      1

/** Raw data index                                                           */
#define RAW_DATA_INDEX                      0

/** CP data index                                                            */
#define CP_DATA_1_INDEX						1

/** CQ data index                                                            */
#define CQ_DATA_2_INDEX						2

/** R4F data index                                                           */
#define R4F_DATA_3_INDEX					3

/** DSP data index                                                           */
#define DSP_DATA_4_INDEX					4

/** Raw Mode file name                                                       */
#define RAW_MODE_FILE_NAME                  "_Raw_"

/** MULTI Mode file name                                                       */
#define MULTI_MODE_FILE_NAME                "_hdr_"

/** Record data file extension                                               */
#define REC_DATA_FILE_EXTENSION             ".bin"

/** Record data file extension characters length                             */
#define REC_FILE_EXTN_CHARACTERS			4

/** Data Index in the buffer     - With bytes count (6 bytes)                */
#define RECORD_DATA_BUF_INDEX				10

/** Multi mode data header ID size in bytes                                 */
#define NUM_OF_BYTES_DATA_HEADER            8

/** Maximum bytes to be recorded in the file  (1GB)                         */
#define MAX_BYTES_PER_REC_DATA_FILE         1073741824

/** Maximum bytes in the data packet                                         */
#define MAX_BYTES_PER_PACKET				1470

/** Size of unsigned short integer value                                     */
#define UINT16_DATA_SIZE                    2

/** Size of unsigned integer value                                           */
#define UINT32_DATA_SIZE                    4

/** Maximum bytes to store IP address                                        */
#define IP_ADDR_MAX_SIZE_BYTES              20

/** Maximum system status                                                    */
#define MAX_SYSTEM_STATUS                   16

/** Command timeout duration in second                                       */
#define CMD_TIMEOUT_DURATION_SEC            10

/** Command timeout duration in millisec                                     */
#define CMD_TIMEOUT_DURATION_MS             10000

/** Playback FPGA bitfile identifier bit                                    */
#define PLAYBACK_BIT_DECODE                 0x4000

/** Version bits decode                                                     */
#define VERSION_BITS_DECODE                 0x7F

/** Number of bits required for version                                     */
#define VERSION_NUM_OF_BITS                 7

/** Record packet delay clock conversion factor                             */
#define FPGA_CLK_CONVERSION_FACTOR          1000

/** Record packet delay clock period in ns                                  */
#define FPGA_CLK_PERIOD_IN_NANO_SEC         8

/** Socket receive buffer size                                              */
#define SOCK_RECV_BUF_SIZE                  0x7FFFFFFF

/** Socket send buffer size                                                 */
#define SOCK_SEND_BUF_SIZE                  0xFFFFF

/** Reordering buffer size for the purpose of swapping bytes algorithm      */
#define MAX_BYTES_FOR_REORDERING            8

#endif // DEFINES_H

