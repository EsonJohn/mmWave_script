/**
 * @file errcodes.h
 *
 * @author JP
 *
 * @version 0.1
 *
 * @brief This file contains DCA1000EVM error code definitions
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
///*****************************************************************************


#ifndef ERRCODES_H
#define ERRCODES_H


///****************
/// Defines
///****************

/** CLI - Shared memory read error                          */
#define CLI_SHM_REC_PROC_STS_READ_ERR				-4000

/** CLI - Shared memory creation error                      */
#define CLI_SHM_CREATION_FAILURE_ERR                -4001

/** CLI - Shared memory not available error                 */
#define CLI_SHM_NOT_AVAIL_ERR                       -4002

/** CLI - Shared memory mapping failure error               */
#define CLI_SHM_MAPPING_FAILURE_ERR                 -4003

/** CLI - Json file DcaConfig node eror                     */
#define CLI_JSON_DCA_CONFIG_NODE_ERR				-4004

/** CLI - Json file EthernetConfig node error               */
#define CLI_JSON_ETH_CONFIG_NODE_ERR				-4005

/** CLI - Json file datalogging mode node error             */
#define CLI_JSON_DATA_LOGGING_MODE_NODE_ERR			-4006

/** CLI - Json file datatransfer mode node error            */
#define CLI_JSON_DATA_TRANSFER_MODE_NODE_ERR		-4007

/** CLI - Json file datacapture mode node error             */
#define CLI_JSON_DATA_CAPTURE_MODE_NODE_ERR			-4008

/** CLI - Json file LVDS mode node error                    */
#define CLI_JSON_LVDS_MODE_NODE_ERR					-4009

/** CLI - Json file dataformat mode node error              */
#define CLI_JSON_DATA_FORMAT_MODE_NODE_ERR			-4010

/** CLI - Json file laneformat map node error               */
#define CLI_JSON_LANE_FORMAT_MAP_NODE_ERR			-4011

/** CLI - Json file dataformat config node error            */
#define CLI_JSON_DATA_FORMAT_CONFIG_NODE_ERR		-4012

/** CLI - Json file invalid data logging mode error         */
#define CLI_JSON_INVALID_DATALOGGING_MODE_ERR		-4013

/** CLI - Json file invalid data transfer mode error        */
#define CLI_JSON_INVALID_DATA_TRANSFER_MODE_ERR		-4014

/** CLI - Json file invalid data capture mode error         */
#define CLI_JSON_INVALID_DATA_CAPTURE_MODE_ERR		-4015

/** CLI - Json file invalid LVDS mode error                 */
#define CLI_JSON_INVALID_LVDS_MODE_ERR				-4016

/** CLI - Json file invalid data format node error          */
#define CLI_JSON_INVALID_DATA_FORMAT_NODE_ERR		-4017

/** CLI - Json file eeprom config node error                */
#define CLI_JSON_EEPROM_CONFIG_NODE_ERR				-4018

/** CLI - Json file eeprom sys Ipaddress node error         */
#define CLI_JSON_EEPROM_SYS_IPADDR_NODE_ERR			-4019

/** CLI - Json file eeprom Dca Ipaddress node error         */
#define CLI_JSON_EEPROM_DCA_IPADDR_NODE_ERR			-4020

/** CLI - Json file eeprom Dca macaddr node error           */
#define CLI_JSON_EEPROM_DCA_MACADDR_NODE_ERR		-4021

/** CLI - Json file eeprom Dca config port node error       */
#define CLI_JSON_EEPROM_DCA_CONFIG_PORT_NODE_ERR	-4022

/** CLI - Json file eeprom Dca data port node error         */
#define CLI_JSON_EEPROM_DCA_DATA_PORT_NODE_ERR		-4023

/** CLI - Json file eeprom invalid Dca config port erro     */
#define CLI_JSON_EEPROM_INVALID_DCA_CONFIG_PORT_ERR	-4024

/** CLI - Json file eeprom invalid Dca data port error      */
#define CLI_JSON_EEPROM_INVALID_DCA_DATA_PORT_ERR	-4025

/** CLI - Json file eeprom port number conflict error       */
#define CLI_JSON_EEPROM_PORT_NUM_CONFLICT_ERR		-4026

/** CLI - No argument while executing error                 */
#define CLI_NO_ARG_ERR                              -4027

/** CLI - Record process running status                     */
#define CLI_SHM_REC_PROC_RUNNING_STS				-4028

/** CLI - Record process is in progress status              */
#define CLI_SHM_REC_IN_PROG_STS                     -4029

/** CLI - Record process is stopped status                  */
#define CLI_SHM_REC_STOPPED_STS                     -4030

/** CLI - Record process start initiated status             */
#define CLI_SHM_REC_START_INITATED_STS              -4031

/** CLI - Record process stop initiated status              */
#define CLI_SHM_REC_STOP_INITATED_STS               -4032

/** CLI - Record process start failed status                */
#define CLI_SHM_REC_START_FAILED_STS                -4033

/** CLI - Json file eeprom invalid Dca macaddress error     */
#define CLI_JSON_EEPROM_INVALID_DCA_MACADDR_ERR		-4034

/** CLI - Json file eeprom invalid Dca ipaddress error      */
#define CLI_JSON_EEPROM_INVALID_DCA_IPADDR_ERR		-4035

/** CLI - Json file eeprom invalid sys ipaddress error      */
#define CLI_JSON_EEPROM_INVALID_SYS_IPADDR_ERR		-4036

/** CLI - Json file ethernet Dca ipaddress error            */
#define CLI_JSON_ETH_DCA_IPADDR_NODE_ERR			-4037

/** CLI - Json file ethernet invalid Dca ipaddress error    */
#define CLI_JSON_ETH_INVALID_DCA_IPADDR_ERR			-4038

/** CLI - Json file ethernet config port node error         */
#define CLI_JSON_ETH_DCA_CONFIG_PORT_NODE_ERR		-4039

/** CLI - Json file ethernet invalid config port error      */
#define CLI_JSON_ETH_INVALID_DCA_CONFIG_PORT_ERR	-4040

/** CLI - Json file ethernet data port node error           */
#define CLI_JSON_ETH_DCA_DATA_PORT_NODE_ERR			-4041

/** CLI - Json file ethernet data port error                */
#define CLI_JSON_ETH_INVALID_DCA_DATA_PORT_ERR		-4042

/** CLI - Json file ethernet port number conflicts error    */
#define CLI_JSON_ETH_PORT_NUM_CONFLICT_ERR			-4043

/** CLI - Json file packet delay node error                 */
#define CLI_JSON_PACKET_DELAY_NODE_ERR				-4044

/** CLI - Json file invalid packet delay error              */
#define CLI_JSON_INVALID_PACKET_DELAY_ERR			-4045

/** CLI - Json file capture config node error               */
#define CLI_JSON_CAPTURE_CONFIG_NODE_ERR			-4046

/** CLI - Json file read dll version error                  */
#define CLI_READ_DLL_VERSION_ERR					-4047

/** CLI - Invalid command argument error                    */
#define	CLI_INVALID_CMD_ARG_ERR                     -4048

/** CLI - Json file open error                              */
#define CLI_JSON_FILE_OPEN_ERR						-4049

/** CLI - Callback register failed error                    */
#define CLI_CMD_CALLBACK_REG_FAILED_ERR				-4050

/** CLI - Ethernet connect failed error                     */
#define CLI_ETH_CONNECT_FAIL_ERR					-4051

/** CLI - Shared memory record unknown status               */
#define CLI_SHM_REC_UNKNOWN_STS                     -4052

/** CLI - Json file invalid file base path error            */
#define CLI_JSON_REC_INVALID_FILE_BASE_PATH_ERR		-4053

/** CLI - Json file invalid max file size error             */
#define CLI_JSON_REC_INVALID_MAX_FILE_SIZE_ERR		-4054

/** CLI - Json file invalid sequence number enable error    */
#define CLI_JSON_REC_INVALID_EN_SEQ_NUM_ERR			-4055

/** CLI - Json file invalid capture stop mode error         */
#define CLI_JSON_REC_INVALID_CAPTURE_STOP_MODE_ERR	-4056

/** CLI - Json file invalid byte capture  error             */
#define CLI_JSON_REC_INVALID_BYTE_CAPTURE_ERR		-4057

/** CLI - Json file invalid duration capture error          */
#define CLI_JSON_REC_INVALID_DUR_CAPTURE_ERR		-4058

/** CLI - Json file invalid frame capture error             */
#define CLI_JSON_REC_INVALID_FRAME_CAPTURE_ERR		-4059

/** CLI - Json file record file base path node error        */
#define CLI_JSON_REC_FILE_BASE_PATH_NODE_ERR		-4060

/** CLI - Json file record file prefix node error           */
#define CLI_JSON_REC_FILE_PREFIX_NODE_ERR			-4061

/** CLI - Json file record max file size node error         */
#define CLI_JSON_REC_MAX_FILE_SIZE_NODE_ERR			-4062

/** CLI - Json file record enable sequence number error     */
#define CLI_JSON_REC_EN_SEQ_NUM_NODE_ERR	     	-4063

/** CLI - Json file record capture stop mode node error     */
#define CLI_JSON_REC_CAPTURE_STOP_MODE_NODE_ERR		-4064

/** CLI - Json file record byte capture node error          */
#define CLI_JSON_REC_BYTE_CAPTURE_NODE_ERR			-4065

/** CLI - Json file record duration capture node err        */
#define CLI_JSON_REC_DUR_CAPTURE_NODE_ERR			-4066

/** CLI - Json file record framecapture node error          */
#define CLI_JSON_REC_FRAME_CAPTURE_NODE_ERR			-4067

/** CLI - Record stop command timeout error                 */
#define CLI_CMD_REC_STOP_TIMEOUT_ERR                -4068

/** CLI - Record process stop failed status                 */
#define CLI_SHM_REC_STOP_FAILED_STS                 -4069

/** CLI - Record process not running status                 */
#define CLI_SHM_REC_PROC_NOT_RUNNING_STS			-4070

/** CLI - Record start command timeout error                */
#define CLI_CMD_REC_START_TIMEOUT_ERR               -4071

/** CLI - Json file invalid record stop mode for Raw mode   */
#define CLI_JSON_REC_INVALID_STOP_MODE_FOR_RAW		-4072

/** CLI - Json file invalid file prefix error               */
#define CLI_JSON_REC_INVALID_FILE_PREFIX_ERR		-4073

/** CLI - Invalid JSON file error                           */
#define CLI_INVALID_JSON_FILE_ERR					-4074

/** CLI - Inline callback register failed error             */
#define CLI_INLINE_CALLBACK_REG_FAILED_ERR			-4075

/** CLI - Json file invalid MSB toggle enable error         */
#define CLI_JSON_REC_INVALID_EN_MSB_TOGGLE_ERR      -4076

/** CLI - Json file invalid reorder enable error            */
#define CLI_JSON_REC_INVALID_EN_REORDER_ERR         -4077

#endif // ERRCODES_H
