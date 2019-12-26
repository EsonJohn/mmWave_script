/**
 * @file recorddatarecv.h
 *
 * @author JP
 *
 * @version 0.5
 *
 * @brief This file contains API declarations for reading data
 * from DCA1000EVM system over different data ports and recording in files. Each
 * data type will be assigned an unique index. 
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
/// 0.2            17 Jan 2019       JP          Inline processing added and
///                                              Review comments incorporated
/// 0.3            14 Feb 2019       JP          Inline processing logfile and
///                                              Review comments incorporated
/// 0.4            29 Mar 2019       JP          Inline buffer - dynamic
///                                              memory allocation
/// 0.5            09 Apr 2019       JP          Reordering record data as
///                                              user configurable option
///*****************************************************************************

#ifndef RECORDDATARECV_H
#define RECORDDATARECV_H

///****************
/// Includes
///****************

#include "../Common/rf_api_internal.h"

#include "defines.h"
#include "extern.h"


/** @class cUdpDataReceiver
 * @brief This class provides support APIs for reading different data UDP <!--
 * --> packets from DCA1000EVM system and record in files of defined size <!--
 * --> This is a Generic class for instantiation of different data ports index <!--
 * --> as per the requirements.
 */
class cUdpDataReceiver
{
	/** Data type index										*/
    UINT8 u8DataTypeId;

    /** Record data file pointer                            */
    FILE *pRecordDataFile;

    /** Record data file path                               */
    SINT8 strRecordFilePath[MAX_NAME_LEN * 2];

    /** Record data file name                               */
    SINT8 strFileName1 [MAX_NAME_LEN * 3];

    /** Socket state                */
    bool bSocketState;

    /** Thread state                */
    bool bThreadState;

    /** First packet state          */
    bool bFirstPktSeqSet;

    /** File Data count             */
    UINT32 u32DataCount;

    /** File count                  */
    UINT32 u32DataFileCount;

	/** File writing - Loop index	*/
    UINT32 u32ByteIndex;

    #ifndef POST_PROCESSING

    /** File write completion wait event                    */
    OSAL_SIGNAL_HANDLE_TYPE sgnFileWriteCompletionWaitEvent;

    /** File write init wait event                          */
    OSAL_SIGNAL_HANDLE_TYPE sgnFileWriteInitWaitEvent;

    /** Buffer 1 empty flag                                 */
    bool bBuf1Empty;

    /** Skip flag for the signal wait for first buffer swap */
    bool bWaitForSignal;

    /** Record buffer 1 for inline processing               */
    SINT8 *s8RecBuf1;

    /** Record buffer 2 for inline processing               */
    SINT8 *s8RecBuf2;

    /** Read data buffer size                               */
    UINT32 u32ReadPtrSize;

    /** Read data buffer index                              */
    UINT32 u32ReadPtrBufIndex;

    /** Write data buffer size                              */
    UINT32 u32WritePtrSize;

    /** Input Buffer for reordering                         */
    SINT16 s16DataReorderBuf[MAX_BYTES_FOR_REORDERING];

    /** Output Buffer for reordering                        */
    SINT16 s16DataReorderOutBuf[MAX_BYTES_FOR_REORDERING];

    /** Log msg - char array declaration for writing out of seq metadata */
    SINT8 s8LogMsg[MAX_NAME_LEN];

    /** Old index for log buffer                            */
    UINT32 u32LogBufOldIndex ;

    /** Log file - Handover offset                          */
    ULONG64 u64ZeroFilledStartHandoverOffset;

    /** Log buffer 1 for inline processing                  */
    strInlineProcLogFileStats *strLogBuf1;

    /** Log buffer 2 for inline processing                  */
    strInlineProcLogFileStats *strLogBuf2;

    /** Current packet size									*/
    SINT32  s32CtPktRecvSize;

    /** Bytes sent till current packet 					   	*/
    ULONG64 u64BytesSentTillCtPkt;

    /** Log buffer loop                                     */
    UINT32 u32Loop;

    /** Start  zero fill offset in Log file                 */
    ULONG64 u64StartZeroFillOffset;

    /** Zero fill bytes in log file                         */
    ULONG64 u64ZeroFillBytes;

    /** Total bytes till dropped packet                     */
    ULONG64 u64TotalBytesTillDroppedPkt;

    /** Reordering - Data stored in intermediate variable1  */
    UINT32 u32Data1;
    
    /** Reordering - Data stored in intermediate variable2  */
    UINT32 u32Data2;
    
    /** Loop index                                     */
    UINT32 u32Loop1;
    
    #endif

public:
    /** @fn cUdpDataReceiver(UINT8 u8DataTypeArg)
     * @brief This constructor function is to initialize the class member <!--
     * --> variables and assign the corresponding datatype index
     * @param [in] u8DataTypeArg [UINT8] - datatype index
     */
    cUdpDataReceiver(UINT8 u8DataTypeArg);
	
    /** @fn void setSocketOpen()
     * @brief This function is to reset member variables and set data socket <!--
     * --> as open for each record process
     */
    void setSocketOpen();

    /** @fn void setSocketClose()
     * @brief This function is to reset data socket as closed
     */
    void setSocketClose();

    /** @fn void setThreadStart()
     * @brief This function is to set data thread progress as recording started
     */
    void setThreadStart();

    /** @fn void setThreadStop()
     * @brief This function is to set data thread progress as recording <!--
     * --> stoppped and close the record file
     */
    void setThreadStop();

    /** @fn void getThreadStatus()
     * @brief This function is to get data thread progress status
     * @return boolean value
     */
    bool getThreadStatus();

    /** @fn void setFileName(SINT8 s8Value1, SINT8 s8Value2)
     * @brief This function is to set data file name for the record progress
     * @param [in] s8Value1 [SINT8] - Header value
     * @param [in] s8Value2 [SINT8] - Header value
     * @return boolean value
     */
    bool setFileName(SINT8 s8Value1, SINT8 s8Value2);

    /** @fn void readData()
     * @brief This function is a thread process to record data through UDP <!--
     * -->in the file, to check for packet out of sequence and to handle <!--
     * --> stop mode configs
     */
    void readData();

    /** @fn bool writeDataToFile(SINT8 *s8Buffer, UINT32 u32Size)
     * @brief This function is to handle recording data in files
     * @return boolean value
     */
    bool writeDataToFile(SINT8 *s8Buffer, UINT32 u32Size);

    /** @fn void writeDataToBuffer_Inline(SINT8 *s8Buffer, UINT32 u32Size, bool bOldPkt, bool bZeroFilledPkt)
     * @brief This function is to handle recording data in buffer (inline processing)
     * @param [in] s8Buffer [SINT8 *] - Buffer to write in buffer
     * @param [in] u32Size  [UINT32] - Size of buffer
     * @param [in] bOldPkt  [bool] - Old packet flag
     * @param [in] bZeroFilledPkt  [bool] - Zero filled packet flag
     */
    void writeDataToBuffer_Inline(SINT8 *s8Buffer, UINT32 u32Size,
                                                    bool bOldPkt, bool bZeroFilledPkt);

    /** @fn void Thread_WriteDataToFile()
     * @brief This thread function is to handle recording data in files (inline processing)
     */
    void Thread_WriteDataToFile();

    /** @fn bool writeDataToFile_Inline(SINT8 *s8Buffer, UINT32 u32Size)
     * @brief This function is to handle recording data in files along with  <!--
     * --> reordering of data (inline processing)
     * @param [in] s8Buffer [SINT8 *] - Buffer to write in file
     * @param [in] u32Size  [UINT32] - Size of buffer
     * @return boolean value
     */
    bool writeDataToFile_Inline(SINT8 *s8Buffer, UINT32 u32Size);

    /** @fn  bool ReorderAlgorithm(SINT8 *s8Buffer, UINT32 u32SizeReorder)
     * @brief This function is to reorder the recorded data based on number <!--
     * --> of lanes and format 0/1 (Complex, Format 0 algorithm)
     * @param [in] s8Buffer [SINT8 *] - 8-bit Data array to be reordered
     * @param [in] u32SizeReorder  [UINT32] - Data size
     */
    bool ReorderAlgorithm(SINT8 *s8Buffer, UINT32 u32SizeReorder);

    /** @fn SINT32 seekOldIndexReadBuf(SINT32 bytes)
     * @brief This function is to seek old index in the buffer (inline processing)
     * @param [in] bytes [SINT32] - Bytes to seek back in the buffer
     * @return SINT32 value
     */
    SINT32 seekOldIndexReadBuf(SINT32 bytes);

    /** @fn void writeLogToFile_Inline()
     * @brief This function is to handle recording data logs in files along with  <!--
     * --> zero filled bytes and offsets (inline processing)
     */
    void writeLogToFile_Inline();

    /** @fn void WriteOffsetLog(ULONG64 u64ZeroFilledOffset, ULONG64 u64ZeroFilledBytes)
     * @brief This function is to capture zero filled offset and bytes in log file
     * @param [in] u64ZeroFilledOffset [ULONG64] - Zero filled offset
     * @param [in] u64ZeroFilledBytes [ULONG64] - Number of Zero filled bytes
     */
    void WriteOffsetLog(ULONG64 u64ZeroFilledOffset, ULONG64 u64ZeroFilledBytes);

    /** @fn void WriteOffsetMetaData(ULONG64 u64PrevPktOffset, UINT32 u32PrevPktSize, ULONG64 u64CtPktOffset, UINT32 u32CtPktSize)
     * @brief This function is to capture the out of sequence packet metadata in a log file <!--
     * --> which will be processed for getting the offset and bytes dropped.
     * @param [in] u64PrevPktOffset [ULONG64] - Previous packet offset
     * @param [in] u32PrevPktSize [UINT32] - Previous packet size
     * @param [in] u64CtPktOffset [ULONG64] - Current packet offset
     * @param [in] u32CtPktSize [UINT32] - Current packet size
    */
    void WriteOffsetMetaData(ULONG64 u64PrevPktOffset, UINT32 u32PrevPktSize,
                                 ULONG64 u64CtPktOffset, UINT32 u32CtPktSize);
};

#endif //RECORDDATARECV_H
