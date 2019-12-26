/**
 * @file recorddatarecv.cpp
 *
 * @author JP
 *
 * @version 0.5
 *
 * @brief This file contains API definitions for reading data
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


///****************************************************************************
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
///****************************************************************************

///****************
/// Includes
///****************

#include "recorddatarecv.h"

#include "extern.h"

/** @fn cUdpDataReceiver::cUdpDataReceiver(UINT8 u8DataTypeArg)
 * @brief This constructor function is to initialize the class member <!--
 * --> variables and assign the corresponding datatype index
 * @param [in] u8DataTypeArg [UINT8] - datatype index
 */
cUdpDataReceiver::cUdpDataReceiver(UINT8 u8DataTypeArg)
{
    u8DataTypeId = u8DataTypeArg;
    pRecordDataFile = NULL;
    bSocketState = false;
    bThreadState = false;
    bFirstPktSeqSet = true;
    u32DataCount = 0;
    u32DataFileCount = 0;
	u32ByteIndex = 0;
    
#ifndef POST_PROCESSING
    bBuf1Empty = false;
    bWaitForSignal = false;
    u32ReadPtrSize = 0;
    u32ReadPtrBufIndex = 0;
    u32WritePtrSize = 0;
    u32LogBufOldIndex = 0;
    u64ZeroFilledStartHandoverOffset = 0;
    s32CtPktRecvSize = 0;
    u64BytesSentTillCtPkt = 0;
    u64StartZeroFillOffset = 0;
    u64ZeroFillBytes = 0;
    u64TotalBytesTillDroppedPkt = 0;
	u32Data1 = 0;
    u32Loop1 = 0;
    u32Data2 = 0;    
    u32Loop = 0;
    s8RecBuf1 = NULL;
    s8RecBuf2 = NULL;
    strLogBuf1 = NULL;
    strLogBuf2 = NULL;

#endif
}

/** @fn void cUdpDataReceiver::setSocketOpen()
 * @brief This function is to reset member variables and set data socket <!--
 * --> as open for each record process
 */
void cUdpDataReceiver::setSocketOpen()
{
    bSocketState   = true;
}

/** @fn void cUdpDataReceiver::setSocketClose()
 * @brief This function is to reset data socket as closed
 */
void cUdpDataReceiver::setSocketClose()
{
    bSocketState   = false;
}

/** @fn void cUdpDataReceiver::setThreadStart()
 * @brief This function is to set data thread progress as recording started
 */
void cUdpDataReceiver::setThreadStart()
{
    bThreadState   = true;

    bFirstPktSeqSet = true;
    u32DataCount = 0;
    u32DataFileCount = 0;

#ifndef POST_PROCESSING
    bBuf1Empty = true;
    bWaitForSignal = false;
    u32ReadPtrBufIndex = 0;
    u32ReadPtrSize = 0;
    u32WritePtrSize = 0;
    u32LogBufOldIndex = 0;
    u64ZeroFilledStartHandoverOffset = 0;
    memset(s16DataReorderBuf, 0, MAX_BYTES_FOR_REORDERING);
    memset(s16DataReorderOutBuf, 0, MAX_BYTES_FOR_REORDERING);
    osalObj.InitEvent(&sgnFileWriteCompletionWaitEvent);
    osalObj.InitEvent(&sgnFileWriteInitWaitEvent);

    s8RecBuf1 = (SINT8 *)malloc(INLINE_BUF_SIZE * sizeof(SINT8));
    s8RecBuf2 = (SINT8 *)malloc(INLINE_BUF_SIZE * sizeof(SINT8));
    strLogBuf1 = (strInlineProcLogFileStats *) malloc(
                                    sizeof(strInlineProcLogFileStats)) ;
    strLogBuf2 = (strInlineProcLogFileStats *) malloc(
                                    sizeof(strInlineProcLogFileStats));

    /** Callback if dynamic memmory allocation failed and packets received
     *  in the port will not be processed
     */
    if((s8RecBuf1 == NULL) || (s8RecBuf2 == NULL) ||
        (strLogBuf1 == NULL) || (strLogBuf2 == NULL))
    {
        RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS ,
                         STS_REC_INLINE_BUF_ALLOCATION_ERR);
        bThreadState   = false;
        return;
    }
    memset(s8RecBuf1, 0, INLINE_BUF_SIZE * sizeof(SINT8));
    memset(s8RecBuf2, 0, INLINE_BUF_SIZE * sizeof(SINT8));
    memset(strLogBuf1, 0, sizeof(strInlineProcLogFileStats));
    memset(strLogBuf2, 0, sizeof(strInlineProcLogFileStats));
#endif
}

/** @fn void cUdpDataReceiver::setThreadStop()
 * @brief This function is to set data thread progress as recording <!--
 * --> stoppped and close the record file
 */
void cUdpDataReceiver::setThreadStop()
{
     bThreadState   = false;

#ifndef POST_PROCESSING
     if(u32ReadPtrSize > 0)
     {
         if(bWaitForSignal)
         {
             /** wait for signal */
             osalObj.WaitForSignal(&sgnFileWriteCompletionWaitEvent, NON_STOP);
         }

         bBuf1Empty = !bBuf1Empty;

         if(bBuf1Empty)
         {
             writeDataToFile_Inline(s8RecBuf2, u32ReadPtrSize);
         }
         else
         {
             writeDataToFile_Inline(s8RecBuf1, u32ReadPtrSize);
         }
    }

    osalObj.DeInitEvent(&sgnFileWriteCompletionWaitEvent);
    osalObj.DeInitEvent(&sgnFileWriteInitWaitEvent);

    /** Delete the memory       */
    if(s8RecBuf1 != NULL)
    {
        free(s8RecBuf1);
        s8RecBuf1 = NULL;
    }
    if(s8RecBuf2 != NULL)
    {
        free(s8RecBuf2);
        s8RecBuf2 = NULL;
    }
    if(strLogBuf1 != NULL)
    {
        free(strLogBuf1);
        strLogBuf1 = NULL;
    }
    if(strLogBuf2 != NULL)
    {
        free(strLogBuf2);
        strLogBuf2 = NULL;
    }
#endif

     if (pRecordDataFile != NULL)
     {
         fclose(pRecordDataFile);
         pRecordDataFile = NULL;
     }
}

/** @fn void cUdpDataReceiver::getThreadStatus()
 * @brief This function is to get data thread progress status
 * @return boolean value
 */
bool cUdpDataReceiver::getThreadStatus()
{
     return bThreadState;
}

/** @fn void cUdpDataReceiver::setFileName(SINT8 s8Value1, SINT8 s8Value2)
 * @brief This function is to set  data file name for the record progress
 * @param [in] s8Value1 [SINT8] - Header value
 * @param [in] s8Value2 [SINT8] - Header value
 * @return boolean value
 */
bool cUdpDataReceiver::setFileName(SINT8 s8Value1, SINT8 s8Value2)
{
    SINT8 header[MAX_NAME_LEN];

    memcpy(strRecordFilePath, sRFDCCard_StartRecConfig.s8FileBasePath,
           strlen(sRFDCCard_StartRecConfig.s8FileBasePath));
#ifdef _WIN32
    strcat(strRecordFilePath, "\\");
#else
    strcat(strRecordFilePath, "/");
#endif
    strcat(strRecordFilePath, sRFDCCard_StartRecConfig.s8FilePrefix);
    if(sRFDCCard_StartRecConfig.eConfigLogMode == RAW_MODE)
    {
        strcat(strRecordFilePath, RAW_MODE_FILE_NAME);

        /** Header ID for storing globally */
        strcpy(sRFDCCard_InlineStats.s8HeaderId[u8DataTypeId], "Raw");
    }
    else
    {
        strcat(strRecordFilePath, MULTI_MODE_FILE_NAME);
        /** File name       */
        sprintf(header , "%02X%02X_", (s8Value1 & 0x0FF),
                (s8Value2 & 0x0FF));
        strcat(strRecordFilePath, header);

       /** Header ID for storing globally */
        sprintf(header , "%02X%02X", (s8Value1 & 0x0FF),
                (s8Value2 & 0x0FF));
        strcpy(sRFDCCard_InlineStats.s8HeaderId[u8DataTypeId], header);
    }
    strRecordFilePath[strlen(strRecordFilePath)] = '\0';

    strcpy(strFileName1, strRecordFilePath);
    strcat(strFileName1, std::to_string(u32DataFileCount).c_str());
    strcat(strFileName1, REC_DATA_FILE_EXTENSION);

    pRecordDataFile = fopen(strFileName1, "wb+");
    if (NULL == pRecordDataFile)
    {
        RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                         STS_REC_FILE_CREATION_ERR);
        return false;
    }

    return true;
}


#ifdef POST_PROCESSING

/** @fn void cUdpDataReceiver::readData()
 * @brief This function is a thread process to record data through UDP <!--
 * -->in the file, to check for packet out of sequence and to handle <!--
 * --> stop mode configs
 */
void cUdpDataReceiver::readData()
{
    struct sockaddr_in SenderAddr;
    socklen_t s32SenderAddrSize = sizeof(SenderAddr);
    SINT8       s8ReceiveBuf[MAX_BYTES_PER_PACKET];
    SINT8       s8HeaderBuf[NUM_OF_BYTES_DATA_HEADER];
    SINT32		s32CtPktRecvSize = 0;
    UINT32		u32CtPktNum = 0;
    UINT32		u32NextPktNum = 0;
    DOUBLE      dTotalBytes = 0;
    DOUBLE      dTotalFrames = 0;

    while(bSocketState)
    {
        START_LOOP:

        /** Receiving data from FPGA        */
        s32CtPktRecvSize = recvfrom(sRFDCCard_SockInfo.s32DataSock[u8DataTypeId],
                               s8ReceiveBuf, MAX_BYTES_PER_PACKET, 0,
                               (struct sockaddr *)&SenderAddr,
                               &s32SenderAddrSize);

        /** Handle the received packet till stop command is executed   */
        if ((s32CtPktRecvSize > 0) && (bThreadState))
        {

            /** Signal capture timeout event that data is been received from system */
            osalObj.SignalEvent(&sgnCaptureTimeoutWaitEvent);

            memcpy(&u32CtPktNum, &s8ReceiveBuf[0], sizeof(UINT32));

            /** First packet */
            if (bFirstPktSeqSet)
            {
                bFirstPktSeqSet = false;
                u32NextPktNum = u32CtPktNum;

                sRFDCCard_InlineStats.StartTime[u8DataTypeId] = time(NULL);
                sRFDCCard_InlineStats.u32FirstPktId[u8DataTypeId] = u32CtPktNum;

                setFileName(s8ReceiveBuf[RECORD_DATA_BUF_INDEX + 1],
                            s8ReceiveBuf[RECORD_DATA_BUF_INDEX]);

                if(sRFDCCard_StartRecConfig.eRecordStopMode == FRAMES)
                {
                    /** Save the Header ID */
                    memcpy(s8HeaderBuf, &s8ReceiveBuf[RECORD_DATA_BUF_INDEX],
                                           NUM_OF_BYTES_DATA_HEADER);
                }
                else if(sRFDCCard_StartRecConfig.eRecordStopMode == DURATION)
                {
                    osalObj.SignalEvent(&sgnDurationStopModeWaitEvent);
                }

            }

            /** Verify bytes stop mode */
            if(sRFDCCard_StartRecConfig.eRecordStopMode == BYTES)
            {
                if(sRFDCCard_StartRecConfig.bSequenceNumberEnable)
                {
                    if((dTotalBytes + s32CtPktRecvSize + UINT32_DATA_SIZE) >=
                            sRFDCCard_StartRecConfig.u32BytesToCapture)
                    {
                        /** Write remaining data to file and exit */
                        writeDataToFile(s8ReceiveBuf,
                                    (UINT32)(sRFDCCard_StartRecConfig.u32BytesToCapture -
                                             dTotalBytes - UINT32_DATA_SIZE -
                                              RECORD_DATA_BUF_INDEX));

                        /** Stop the recording */
                        bThreadState = false;
                        if(!gbRecStopCmdSent)
                            RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                                         STS_REC_COMPLETED);
                        goto START_LOOP;
                    }

                    dTotalBytes += (s32CtPktRecvSize + UINT32_DATA_SIZE);
                }
                else
                {
                    if((dTotalBytes + s32CtPktRecvSize - RECORD_DATA_BUF_INDEX) >=
                            sRFDCCard_StartRecConfig.u32BytesToCapture)
                    {
                        /** Write remaining data to file and exit */
                        writeDataToFile(s8ReceiveBuf,
                                    (UINT32)(sRFDCCard_StartRecConfig.u32BytesToCapture - dTotalBytes));
                        
                        /** Stop the recording */
                        bThreadState = false;
                        if(!gbRecStopCmdSent)
                            RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                                         STS_REC_COMPLETED);
                        goto START_LOOP;
                    }

                    dTotalBytes += (s32CtPktRecvSize - RECORD_DATA_BUF_INDEX) ;
                }
            }
			
			/** Verify frames stop mode */
			if((sRFDCCard_StartRecConfig.eConfigLogMode == MULTI_MODE) &&
				(sRFDCCard_StartRecConfig.eRecordStopMode == FRAMES))
			{
				if(memcmp(s8HeaderBuf, &s8ReceiveBuf[RECORD_DATA_BUF_INDEX],
                       NUM_OF_BYTES_DATA_HEADER) == 0)
                {
                    dTotalFrames ++;
                }

                if(dTotalFrames > sRFDCCard_StartRecConfig.u32FramesToCapture)
                {
                    /** Stop the recording */
                    bThreadState = false;
                    if(!gbRecStopCmdSent)
                        RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                                     STS_REC_COMPLETED);
                    goto START_LOOP;
                }
            }

            /** Updating record process status variable for every packet */
            sRFDCCard_InlineStats.u64NumOfRecvdPackets[u8DataTypeId] ++;
            sRFDCCard_InlineStats.EndTime[u8DataTypeId] = time(NULL);
            sRFDCCard_InlineStats.u32LastPktId[u8DataTypeId] = u32CtPktNum;

            /** Verify out of sequence packet   */
            if (u32NextPktNum != u32CtPktNum)
            {
                sRFDCCard_InlineStats.u64OutOfSeqCount[u8DataTypeId] ++;
                UpdateInlineStatus(true, u8DataTypeId);
            }
            else
            {
                UpdateInlineStatus(false, u8DataTypeId);
            }
            u32NextPktNum = (u32CtPktNum + 1);

            /** Write data to file */
            if(!writeDataToFile(s8ReceiveBuf,
                        (UINT32)(s32CtPktRecvSize - RECORD_DATA_BUF_INDEX)))
            {
                /** Stop the recording */
                bThreadState = false;
                if(!gbRecStopCmdSent)
                    RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                                 STS_REC_COMPLETED);
                goto START_LOOP;
            }           

        }
    }
}


/** @fn bool cUdpDataReceiver::writeDataToFile(SINT8 *s8Buffer, UINT32 u32Size)
 * @brief This function is to handle recording  data in files
 * @return boolean value
 */
bool cUdpDataReceiver::writeDataToFile(SINT8 *s8Buffer, UINT32 u32Size)
{
    /** Verifies for maximum file size */
    if((u32DataCount + u32Size) > u32MaxFileSizeToCapture)
    {
        /** Closes and open another file if maximum file size exceeds */
        fclose(pRecordDataFile);
        u32DataFileCount++;

        strcpy(strFileName1, strRecordFilePath);
        strcat(strFileName1, std::to_string(u32DataFileCount).c_str());
        strcat(strFileName1, REC_DATA_FILE_EXTENSION);

        pRecordDataFile = fopen (strFileName1, "wb+");
        if (NULL == pRecordDataFile)
        {
            RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                             STS_REC_FILE_CREATION_ERR);
            return false;
        }
        u32DataCount = 0;
    }
    if(sRFDCCard_StartRecConfig.bSequenceNumberEnable)
    {
        u32DataCount += fwrite((const SINT8 *)s8Buffer, 1,
                                    UINT32_DATA_SIZE, pRecordDataFile);
        memcpy(s8Buffer, &u32Size, UINT32_DATA_SIZE);
        u32DataCount += fwrite((const SINT8 *)s8Buffer, 1,
                            u32Size + RECORD_DATA_BUF_INDEX,
                            pRecordDataFile);
    }
    else
    {
        u32DataCount += fwrite(
                        (const SINT8 *) &s8Buffer[RECORD_DATA_BUF_INDEX],
                        1, u32Size, pRecordDataFile);
    }


    return true;
}

#else
/** Inline processing */

/** @fn void cUdpDataReceiver::readData()
 * @brief This function is a thread process to record  data through UDP <!--
 * -->in the file, to check for packet out of sequence and to handle <!--
 * --> stop mode configs
 */
void cUdpDataReceiver::readData()
{
    struct sockaddr_in SenderAddr;
    socklen_t s32SenderAddrSize = sizeof(SenderAddr);
    SINT8       s8ReceiveBuf[MAX_BYTES_PER_PACKET];
    SINT8       s8HeaderBuf[NUM_OF_BYTES_DATA_HEADER];
    s32CtPktRecvSize = 0;
    SINT32		s32PrevPktRecvSize = 0;
    UINT32		u32CtPktNum = 0;
    UINT32		u32NextPktNum = 0;
    UINT32		u32PrevPktNum = 0;
    UINT32      u32NumOfDroppedPkts = 0;
    DOUBLE      dTotalBytes = 0;
    DOUBLE      dTotalFrames = 0;
    u64BytesSentTillCtPkt = 0;
    ULONG64     u64BytesSentTillPrevPkt = 0;
    ULONG64     u64ZeroFilledBytes = 0;

    memset(s8ReceiveBuf, 0, (MAX_BYTES_PER_PACKET * sizeof(SINT8)));
    memset(s8HeaderBuf, 0, (NUM_OF_BYTES_DATA_HEADER * sizeof(SINT8)));

    while(bSocketState)
    {
        START_LOOP:

        /** Receiving data from FPGA        */
        s32CtPktRecvSize = recvfrom(sRFDCCard_SockInfo.s32DataSock[u8DataTypeId],
                               s8ReceiveBuf, MAX_BYTES_PER_PACKET, 0,
                               (struct sockaddr *)&SenderAddr,
                               &s32SenderAddrSize);

        /** Handle the received packet till stop command is executed   */
        if ((s32CtPktRecvSize > 0) && (bThreadState))
        {
            /** Signal capture timeout event that data is been received from system */
            osalObj.SignalEvent(&sgnCaptureTimeoutWaitEvent);

            memcpy(&u32CtPktNum, &s8ReceiveBuf[0], sizeof(UINT32));

            memcpy(&u64BytesSentTillCtPkt, &s8ReceiveBuf[4], 6);

            /** First packet */
            if (bFirstPktSeqSet)
            {
                /** Reset inline logic variables */
                u32NextPktNum = u32CtPktNum;
                u32PrevPktNum = (u32CtPktNum - 1);
                u32NumOfDroppedPkts = 0;
                u64BytesSentTillPrevPkt = 0;
                u64ZeroFilledBytes = 0;
                s32PrevPktRecvSize = 0;
                dTotalBytes = 0;
                dTotalFrames = 0;

                bFirstPktSeqSet = false;
                sRFDCCard_InlineStats.StartTime[u8DataTypeId] = time(NULL);
                sRFDCCard_InlineStats.u32FirstPktId[u8DataTypeId] = u32CtPktNum;

                setFileName(s8ReceiveBuf[RECORD_DATA_BUF_INDEX + 1],
                            s8ReceiveBuf[RECORD_DATA_BUF_INDEX]);

                if(sRFDCCard_StartRecConfig.eRecordStopMode == FRAMES)
                {
                    /** Save the Header ID   */
                    memcpy(s8HeaderBuf, &s8ReceiveBuf[RECORD_DATA_BUF_INDEX],
                                           NUM_OF_BYTES_DATA_HEADER);
                }
                else if(sRFDCCard_StartRecConfig.eRecordStopMode == DURATION)
                {
                    osalObj.SignalEvent(&sgnDurationStopModeWaitEvent);
                }

                /** Assigning buffer to read write pointers    */
                bBuf1Empty = true;
                u32ReadPtrSize = 0;
                u32WritePtrSize = 0;
                u32ReadPtrBufIndex = 0;
            }

            /** Updating inline status variable for every packet */
            sRFDCCard_InlineStats.EndTime[u8DataTypeId] = time(NULL);
            
			/** Verify frames stop mode */
			if((sRFDCCard_StartRecConfig.eConfigLogMode == MULTI_MODE) &&
				(sRFDCCard_StartRecConfig.eRecordStopMode == FRAMES))
			{
				if(memcmp(s8HeaderBuf, &s8ReceiveBuf[RECORD_DATA_BUF_INDEX],
                       NUM_OF_BYTES_DATA_HEADER) == 0)
                {
                    dTotalFrames ++;
                }

                if(dTotalFrames > sRFDCCard_StartRecConfig.u32FramesToCapture)
                {
                    /** Stop the recording */
                    bThreadState = false;
                    if(!gbRecStopCmdSent)
                        RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                                     STS_REC_COMPLETED);
                    goto START_LOOP;
                }
            }
			
			sRFDCCard_InlineStats.u32LastPktId[u8DataTypeId] = u32CtPktNum;
			
            /** Verifies out of sequence and increment the count */
            if((u32PrevPktNum + 1) != u32CtPktNum)
            {
                sRFDCCard_InlineStats.u64OutOfSeqCount[u8DataTypeId] ++;
            }
            u32PrevPktNum = u32CtPktNum;
			
			
            /** Handle received packets     */
            if (u32CtPktNum == u32NextPktNum)
            {
                sRFDCCard_InlineStats.u64NumOfRecvdPackets[u8DataTypeId] ++;

                /** Verifies bytes stop mode    */
                if(sRFDCCard_StartRecConfig.eRecordStopMode == BYTES)
                {
                    /** Stop recording if data matches the total number of bytes */
                    if((dTotalBytes + s32CtPktRecvSize - RECORD_DATA_BUF_INDEX) >=
                            sRFDCCard_StartRecConfig.u32BytesToCapture)
                    {
                        writeDataToBuffer_Inline(&s8ReceiveBuf[RECORD_DATA_BUF_INDEX],
                                  (UINT32)(sRFDCCard_StartRecConfig.u32BytesToCapture - dTotalBytes),
                                     false, false);

                        /** Updating inline processing summary */
                        UpdateInlineStatus(true, u8DataTypeId);

                        /** Stop the recording */
                        bThreadState = false;
                        if(!gbRecStopCmdSent)
                            RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                                         STS_REC_COMPLETED);
                        goto START_LOOP;
                    }

                    writeDataToBuffer_Inline(&s8ReceiveBuf[RECORD_DATA_BUF_INDEX],
                                      (s32CtPktRecvSize - RECORD_DATA_BUF_INDEX),
                                        false, false);
                    dTotalBytes += (s32CtPktRecvSize - RECORD_DATA_BUF_INDEX) ;
                }
                else
                {
                    writeDataToBuffer_Inline(&s8ReceiveBuf[RECORD_DATA_BUF_INDEX],
                                  (s32CtPktRecvSize - RECORD_DATA_BUF_INDEX),
                                             false, false);
                }

                u32NextPktNum = (u32CtPktNum + 1);
                u64BytesSentTillPrevPkt = u64BytesSentTillCtPkt;
                s32PrevPktRecvSize = s32CtPktRecvSize;

                /** Updating inline processing summary */
                UpdateInlineStatus(false, u8DataTypeId);
            }
            else if (u32CtPktNum < u32NextPktNum)
            {
                if(seekOldIndexReadBuf(
                    ((u64BytesSentTillPrevPkt + s32PrevPktRecvSize - RECORD_DATA_BUF_INDEX)
                     - u64BytesSentTillCtPkt))
                    != -1)
                {
                    sRFDCCard_InlineStats.u64NumOfRecvdPackets[u8DataTypeId] ++;

                    /** Calculatig old index for log buffer     */
                    u32LogBufOldIndex = u32NextPktNum - u32CtPktNum;

                    /** Writing the old packet in the data buffer */
                    writeDataToBuffer_Inline(&s8ReceiveBuf[RECORD_DATA_BUF_INDEX],
                                      (s32CtPktRecvSize - RECORD_DATA_BUF_INDEX),
                                             true, false);
                    sRFDCCard_InlineStats.
                            u64NumOfZeroFilledPackets[u8DataTypeId] --;
                    sRFDCCard_InlineStats.u64NumOfZeroFilledBytes[u8DataTypeId]
                           -= (s32CtPktRecvSize - RECORD_DATA_BUF_INDEX);

                    /** Set to current index */
                    u32ReadPtrBufIndex = u32ReadPtrSize;

                    /** Logging out of seq metadata         */
                    sRFDCCard_InlineStats.u32OutOfSeqPktFromOffset[u8DataTypeId] =
                        u32NextPktNum - 1;
                    sRFDCCard_InlineStats.u32OutOfSeqPktToOffset[u8DataTypeId] =
                        u32CtPktNum;
#ifdef LOG_OUT_OF_SEQ_OFFSET
                    WriteOffsetMetaData(u64BytesSentTillPrevPkt, s32PrevPktRecvSize - RECORD_DATA_BUF_INDEX,
                                            u64BytesSentTillCtPkt, s32CtPktRecvSize - RECORD_DATA_BUF_INDEX);
#endif
                }

                /** Updating inline processing summary */
                UpdateInlineStatus(true, u8DataTypeId);
            }
            else if (u32CtPktNum > u32NextPktNum)
            {              
                sRFDCCard_InlineStats.u64NumOfRecvdPackets[u8DataTypeId] ++;

                u32NumOfDroppedPkts = u32CtPktNum - u32NextPktNum;
                sRFDCCard_InlineStats.u64NumOfZeroFilledPackets[u8DataTypeId]
                        += u32NumOfDroppedPkts;

                u64ZeroFilledBytes = u64BytesSentTillCtPkt -
                        (u64BytesSentTillPrevPkt + (s32PrevPktRecvSize -
                         RECORD_DATA_BUF_INDEX));
                sRFDCCard_InlineStats.u64NumOfZeroFilledBytes[u8DataTypeId]
                        += u64ZeroFilledBytes;

                /** Write single packet to ensure not filling beyond buffer size */
                while(u64ZeroFilledBytes >= PAYLOAD_BYTES_PER_PACKET)
                {
                    /** Verifies bytes stop mode    */
                    if(sRFDCCard_StartRecConfig.eRecordStopMode == BYTES)
                    {
                        /** Stop recording if data matches the total number of bytes */
                        if((dTotalBytes + PAYLOAD_BYTES_PER_PACKET) >=
                                sRFDCCard_StartRecConfig.u32BytesToCapture)
                        {
                            writeDataToBuffer_Inline(s8ZeroBuf,
                              (UINT32)(sRFDCCard_StartRecConfig.u32BytesToCapture - dTotalBytes),
                                             false, true);

                            /** Updating inline processing summary */
                            UpdateInlineStatus(true, u8DataTypeId);

                            /** Stop the recording */
                            bThreadState = false;
                            if(!gbRecStopCmdSent)
                                RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                                             STS_REC_COMPLETED);
                            goto START_LOOP;
                        }

                        writeDataToBuffer_Inline(s8ZeroBuf, PAYLOAD_BYTES_PER_PACKET,
                                                 false, true);
                        dTotalBytes += (PAYLOAD_BYTES_PER_PACKET) ;
                    }
                    else
                    {
                        writeDataToBuffer_Inline(s8ZeroBuf, PAYLOAD_BYTES_PER_PACKET,
                                                 false, true);
                    }
                    u64ZeroFilledBytes -= PAYLOAD_BYTES_PER_PACKET;
                }
                if(u64ZeroFilledBytes > 0)
                {
                    /** Verifies bytes stop mode    */
                    if(sRFDCCard_StartRecConfig.eRecordStopMode == BYTES)
                    {
                        /** Stop recording if data matches the total number of bytes */
                        if((dTotalBytes + u64ZeroFilledBytes) >=
                                sRFDCCard_StartRecConfig.u32BytesToCapture)
                        {
                            writeDataToBuffer_Inline(s8ZeroBuf,
                                      (UINT32)(sRFDCCard_StartRecConfig.u32BytesToCapture - dTotalBytes),
                                                         false, true);

                            /** Updating inline processing summary */
                            UpdateInlineStatus(true, u8DataTypeId);

                            /** Stop the recording */
                            bThreadState = false;
                            if(!gbRecStopCmdSent)
                                RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                                             STS_REC_COMPLETED);
                            goto START_LOOP;
                        }

                        writeDataToBuffer_Inline(s8ZeroBuf, u64ZeroFilledBytes,
                                                 false, true);
                        dTotalBytes += (u64ZeroFilledBytes) ;
                    }
                    else
                    {
                        writeDataToBuffer_Inline(s8ZeroBuf, u64ZeroFilledBytes,
                                                 false, true);
                    }
                }

                /** Verifies bytes stop mode    */
                if(sRFDCCard_StartRecConfig.eRecordStopMode == BYTES)
                {
                    /** Stop recording if data matches the total number of bytes */
                    if((dTotalBytes + s32CtPktRecvSize - RECORD_DATA_BUF_INDEX) >=
                            sRFDCCard_StartRecConfig.u32BytesToCapture)
                    {
                        writeDataToBuffer_Inline(&s8ReceiveBuf[RECORD_DATA_BUF_INDEX],
                              (UINT32)(sRFDCCard_StartRecConfig.u32BytesToCapture - dTotalBytes),
                                         false, false);

                        /** Updating inline processing summary */
                        UpdateInlineStatus(true, u8DataTypeId);

                        /** Stop the recording */
                        bThreadState = false;
                        if(!gbRecStopCmdSent)
                            RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                                         STS_REC_COMPLETED);
                        goto START_LOOP;
                    }

                    writeDataToBuffer_Inline(&s8ReceiveBuf[RECORD_DATA_BUF_INDEX],
                                      (s32CtPktRecvSize - RECORD_DATA_BUF_INDEX),
                                             false, false);
                    dTotalBytes += (s32CtPktRecvSize - RECORD_DATA_BUF_INDEX) ;
                }
                else
                {
                    writeDataToBuffer_Inline(&s8ReceiveBuf[RECORD_DATA_BUF_INDEX],
                        (s32CtPktRecvSize - RECORD_DATA_BUF_INDEX),
                                             false, false);
                }

                /** Logging out of seq metadata         */
                sRFDCCard_InlineStats.u32OutOfSeqPktFromOffset[u8DataTypeId] =
                    u32NextPktNum - 1;
                sRFDCCard_InlineStats.u32OutOfSeqPktToOffset[u8DataTypeId] =
                    u32CtPktNum;
#ifdef LOG_OUT_OF_SEQ_OFFSET
                WriteOffsetMetaData(u64BytesSentTillPrevPkt, s32PrevPktRecvSize - RECORD_DATA_BUF_INDEX,
                                        u64BytesSentTillCtPkt, s32CtPktRecvSize - RECORD_DATA_BUF_INDEX);
#endif
                /** Stores the offset for verifying next packet */
                u32NextPktNum = (u32CtPktNum + 1);
                u64BytesSentTillPrevPkt = u64BytesSentTillCtPkt;
                s32PrevPktRecvSize = s32CtPktRecvSize;

                /** Updating inline processing summary */
                UpdateInlineStatus(true, u8DataTypeId);
            } // verify sequence and write
			
        } // If packet has some data

    }
}

/** @fn void cUdpDataReceiver::writeDataToBuffer_Inline(SINT8 *s8Buffer, UINT32 u32Size, bool bOldPkt, bool bZeroFilledPkt)
 * @brief This function is to handle recording  data in buffer (inline processing)
 * @param [in] s8Buffer [SINT8 *] - Buffer to write in buffer
 * @param [in] u32Size  [UINT32] - Size of buffer
 * @param [in] bOldPkt  [bool] - Old packet flag
 * @param [in] bZeroFilledPkt  [bool] - Zero filled packet flag
 */
void cUdpDataReceiver::writeDataToBuffer_Inline(SINT8 *s8Buffer, UINT32 u32Size,
                                                bool bOldPkt, bool bZeroFilledPkt)
{
    if((u32ReadPtrBufIndex + u32Size) > INLINE_BUF_SIZE)
    {        
        if(bWaitForSignal)
        {
            /** Wait for write completion signal */
            osalObj.WaitForSignal(&sgnFileWriteCompletionWaitEvent, NON_STOP);
        }
        else
        {
            bWaitForSignal = true;
        }

        /** Swapping buffers to continue read and parallel writing   */
        bBuf1Empty = !bBuf1Empty;

        u32WritePtrSize = u32ReadPtrSize;
        u32ReadPtrSize = 0;
        u32ReadPtrBufIndex = 0;

        /** Signal to write thread to start writing */
        osalObj.SignalEvent(&sgnFileWriteInitWaitEvent);
    }

    /** Store the current packet in buffer  */
    if(bBuf1Empty)
        memcpy(&s8RecBuf1[u32ReadPtrBufIndex], s8Buffer, u32Size);
    else
        memcpy(&s8RecBuf2[u32ReadPtrBufIndex], s8Buffer, u32Size);
    u32ReadPtrBufIndex += u32Size;
    if(!bOldPkt)
        u32ReadPtrSize += u32Size;

#ifdef LOG_DROPPED_PKTS_OFFSET
    /** Store the missed packet stats in log buffer */
    if(bBuf1Empty)
    {
        if(bZeroFilledPkt)
        {
            strLogBuf1->bIsZeroFilledPktPresent = true;
            strLogBuf1->bValidPacket[strLogBuf1->u32LogSize] = false;
            strLogBuf1->u32LogSize ++;
        }
        else
        {
            if(!bOldPkt)
            {
                strLogBuf1->bValidPacket[strLogBuf1->u32LogSize] = true;
                strLogBuf1->u32CtPktSize[strLogBuf1->u32LogSize] =
                        s32CtPktRecvSize - RECORD_DATA_BUF_INDEX;
                strLogBuf1->u64CtPktOffset[strLogBuf1->u32LogSize] =
                        u64BytesSentTillCtPkt;
                strLogBuf1->u32LogSize ++;
            }
            else
            {
                strLogBuf1->bValidPacket[strLogBuf1->u32LogSize -
                        u32LogBufOldIndex] = true;
                strLogBuf1->u32CtPktSize[strLogBuf1->u32LogSize -
                        u32LogBufOldIndex] =
                        s32CtPktRecvSize - RECORD_DATA_BUF_INDEX;
                strLogBuf1->u64CtPktOffset[strLogBuf1->u32LogSize -
                        u32LogBufOldIndex] = u64BytesSentTillCtPkt;
            }
        }
    }
    else
    {
        if(bZeroFilledPkt)
        {
            strLogBuf2->bIsZeroFilledPktPresent = true;
            strLogBuf2->bValidPacket[strLogBuf2->u32LogSize] = false;
            strLogBuf2->u32LogSize ++;
        }
        else
        {
            if(!bOldPkt)
            {
                strLogBuf2->bValidPacket[strLogBuf2->u32LogSize] = true;
                strLogBuf2->u32CtPktSize[strLogBuf2->u32LogSize] =
                        s32CtPktRecvSize - RECORD_DATA_BUF_INDEX;
                strLogBuf2->u64CtPktOffset[strLogBuf2->u32LogSize] =
                        u64BytesSentTillCtPkt;
                strLogBuf2->u32LogSize ++;
            }
            else
            {
                strLogBuf2->bValidPacket[strLogBuf2->u32LogSize -
                        u32LogBufOldIndex] = true;
                strLogBuf2->u32CtPktSize[strLogBuf2->u32LogSize -
                        u32LogBufOldIndex] =
                        s32CtPktRecvSize - RECORD_DATA_BUF_INDEX;
                strLogBuf2->u64CtPktOffset[strLogBuf2->u32LogSize -
                        u32LogBufOldIndex] = u64BytesSentTillCtPkt;
            }
        }
    }
#endif

}

/** @fn void cUdpDataReceiver::Thread_WriteDataToFile()
 * @brief This thread function is to handle recording  data in files (inline processing)
 */
void cUdpDataReceiver::Thread_WriteDataToFile()
{
    while(bSocketState)
    {
        /** Wait for ready to write signal      */
        osalObj.WaitForSignal(&sgnFileWriteInitWaitEvent, NON_STOP);

        if(bBuf1Empty)
        {
            writeDataToFile_Inline(s8RecBuf2, u32WritePtrSize);
        }
        else
        {
            writeDataToFile_Inline(s8RecBuf1, u32WritePtrSize);
        }

        /** Signal ready for next file write */
        osalObj.SignalEvent(&sgnFileWriteCompletionWaitEvent);
    }
}

/** @fn bool cUdpDataReceiver::writeDataToFile_Inline(SINT8 *s8Buffer, UINT32 u32Size)
 * @brief This function is to handle recording  data in files along with  <!--
 * --> reordering of data (inline processing)
 * @param [in] s8Buffer [SINT8 *] - Buffer to write in file
 * @param [in] u32Size  [UINT32] - Size of buffer
 * @return boolean value
 */
bool cUdpDataReceiver::writeDataToFile_Inline(SINT8 *s8Buffer, UINT32 u32Size)
{
    /** Reordering data bytes */
    if(sRFDCCard_StartRecConfig.bReorderEnable)
    {
        if(!ReorderAlgorithm(s8Buffer, u32Size))
            return false;
    }

	/** Data file writing */
	u32ByteIndex = 0;
    
    do
    {
        /** Verifies for maximum file size */
        if((u32DataCount + u32Size) <= u32MaxFileSizeToCapture)
        {
            fwrite((const SINT8 *)&s8Buffer[u32ByteIndex], 1,
                                        u32Size, pRecordDataFile);
            u32DataCount += u32Size;
            u32Size = 0;
        }
        else
        {
            /** Write chunk of packets in multiples of PAYLOAD_BYTES_PER_PACKET
             * to avoid i,q conflicts in multiple files
             */
            u32Loop = floor(((u32MaxFileSizeToCapture - u32DataCount) /
                                PAYLOAD_BYTES_PER_PACKET));
            if(u32Loop != 0)
            {
                u32DataCount = u32Loop * PAYLOAD_BYTES_PER_PACKET;
                fwrite((const SINT8 *)&s8Buffer[u32ByteIndex], 1, u32DataCount,
                                        pRecordDataFile);
                u32Size -= u32DataCount;
                u32ByteIndex += u32DataCount;
            }

            /** Closes and open another file if maximum file size exceeds */
            fclose(pRecordDataFile);
            u32DataFileCount ++;

            strcpy(strFileName1, strRecordFilePath);
            strcat(strFileName1, std::to_string(u32DataFileCount).c_str());
            strcat(strFileName1, REC_DATA_FILE_EXTENSION);

            pRecordDataFile = fopen (strFileName1, "wb+");
            if (NULL == pRecordDataFile)
            {
                RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS,
                                 STS_REC_FILE_CREATION_ERR);
                return false;
            }
            u32DataCount = 0;
        }
    }while(u32Size > 0);

#ifdef LOG_DROPPED_PKTS_OFFSET

	/** Logfile writing */
    writeLogToFile_Inline();
	
#endif

    return true;
}

/** @fn  bool ReorderAlgorithm(SINT8 *s8Buffer, UINT32 u32SizeReorder)
 * @brief This function is to reorder the recorded data based on number of <!--
 * --> lanes and format 0/1 (Complex, Format 0 algorithm)
 * @param [in] s8Buffer [SINT8 *] - 8-bit Data array to be reordered
 * @param [in] u32SizeReorder  [UINT32] - Data size
 */
bool cUdpDataReceiver::ReorderAlgorithm(SINT8 *s8Buffer, UINT32 u32SizeReorder)
{
    if((u32SizeReorder % (u8LaneNumber * 2 * 2)) != 0)
    {
       RFDCARD_Callback(CMD_CODE_SYSTEM_ASYNC_STATUS ,
                        STS_REC_REORDERING_ERR);
       return false;
    }

    for(u32Data1 = 0; u32Data1 < u32SizeReorder;
         u32Data1 += (2 * 2 * u8LaneNumber))
    {
        /** Write into data buffer before swapping   */
        memcpy(s16DataReorderBuf, &s8Buffer[u32Data1],
                    sizeof(SINT16) * (2 * u8LaneNumber));

        /** Swapping based on reordering algorithm for lane no */
		for(u32Loop1 = 0; u32Loop1 < (2 * u8LaneNumber); u32Loop1 ++)
        {
			u32Data2 = ((u32Loop1 % u8LaneNumber) * 2);
			u32Data2 += (u32Loop1 / u8LaneNumber);
			
            if(sRFDCCard_StartRecConfig.bMsbToggleEnable)
            {
                s16DataReorderOutBuf[u32Data2] =
                                        (s16DataReorderBuf[u32Loop1] + 0x8000);
            }
            else
            {
                s16DataReorderOutBuf[u32Data2] = s16DataReorderBuf[u32Loop1];
            }
        }
		
		/** Write into data buffer after swapping   */
        memcpy(&s8Buffer[u32Data1], s16DataReorderOutBuf,
                         sizeof(SINT16) * (2 * u8LaneNumber));

    } // end of for loop

    return true;
}

/** @fn SINT32 cUdpDataReceiver::seekOldIndexReadBuf(SINT32 bytes)
 * @brief This function is to seek old index in the buffer (inline processing)
 * @param [in] bytes [SINT32] - Bytes to seek back in the buffer
 * @return SINT32 value
 */
SINT32 cUdpDataReceiver::seekOldIndexReadBuf(SINT32 bytes)
{
    if(((SINT32)u32ReadPtrBufIndex - bytes) < 0)
        return -1;
    else
        u32ReadPtrBufIndex = u32ReadPtrBufIndex - bytes;

    return 0;
}

/** @fn void cUdpDataReceiver::writeLogToFile_Inline()
 * @brief This function is to handle recording  data logs in files along with  <!--
 * --> zero filled bytes and offsets (inline processing)
 */
void cUdpDataReceiver::writeLogToFile_Inline()
{
    if(!bBuf1Empty)
    {
        u64ZeroFillBytes = 0;
        u64StartZeroFillOffset = u64ZeroFilledStartHandoverOffset;
        u64TotalBytesTillDroppedPkt = u64ZeroFilledStartHandoverOffset;

        if(strLogBuf1->bIsZeroFilledPktPresent)
        {
            if(strLogBuf1->u32LogSize > 0)
            {
                if(strLogBuf1->bValidPacket[0])
                      u64StartZeroFillOffset = 0;
            }
            for(u32Loop = 0; u32Loop < strLogBuf1->u32LogSize; u32Loop += 1)
            {
                if(strLogBuf1->bValidPacket[u32Loop])
                {
                    if(u64StartZeroFillOffset != 0)
                    {
                        u64ZeroFillBytes = strLogBuf1->u64CtPktOffset[u32Loop] -
                                            u64TotalBytesTillDroppedPkt;
                        WriteOffsetLog(u64StartZeroFillOffset, u64ZeroFillBytes);
                        u64StartZeroFillOffset = 0;
                    }
                    u64TotalBytesTillDroppedPkt = (strLogBuf1->u64CtPktOffset[u32Loop] +
                            (strLogBuf1->u32CtPktSize[u32Loop]));
                }
                else
                {
                    if(u64StartZeroFillOffset == 0)
                    {
                        u64StartZeroFillOffset = u64TotalBytesTillDroppedPkt;
                    }
                }
            }

            /** Verify for the need of handover buffer  */
            if(u64StartZeroFillOffset != 0)
            {
                u64ZeroFilledStartHandoverOffset = u64StartZeroFillOffset;
            }
            else
            {
                u64ZeroFilledStartHandoverOffset =
                        strLogBuf1->u64CtPktOffset[strLogBuf1->u32LogSize - 1] +
                        (strLogBuf1->u32CtPktSize[strLogBuf1->u32LogSize - 1]);
            }
        }
        else
        {
            u64ZeroFilledStartHandoverOffset =
                    strLogBuf1->u64CtPktOffset[strLogBuf1->u32LogSize - 1] +
                    (strLogBuf1->u32CtPktSize[strLogBuf1->u32LogSize - 1]);
        }

        /** Resetting the log buffer    */
        memset(strLogBuf1, 0, sizeof(strInlineProcLogFileStats));
    }

    else
    {
        u64ZeroFillBytes = 0;
        u64StartZeroFillOffset = u64ZeroFilledStartHandoverOffset;
        u64TotalBytesTillDroppedPkt = u64ZeroFilledStartHandoverOffset;

        if(strLogBuf2->bIsZeroFilledPktPresent)
        {
            if(strLogBuf2->u32LogSize > 0)
            {
                if(strLogBuf2->bValidPacket[0])
                      u64StartZeroFillOffset = 0;
            }
            for(u32Loop = 0; u32Loop < strLogBuf2->u32LogSize; u32Loop += 1)
            {
                if(strLogBuf2->bValidPacket[u32Loop])
                {
                    if(u64StartZeroFillOffset != 0)
                    {
                        u64ZeroFillBytes = strLogBuf2->u64CtPktOffset[u32Loop] -
                                            u64TotalBytesTillDroppedPkt;
                        WriteOffsetLog(u64StartZeroFillOffset, u64ZeroFillBytes);
                        u64StartZeroFillOffset = 0;
                    }
                    u64TotalBytesTillDroppedPkt = (strLogBuf2->u64CtPktOffset[u32Loop] +
                            (strLogBuf2->u32CtPktSize[u32Loop]));
                }
                else
                {
                    if(u64StartZeroFillOffset == 0)
                    {
                        u64StartZeroFillOffset = u64TotalBytesTillDroppedPkt;
                    }
                }
            }

            /** Verify for the need of handover buffer  */
            if(u64StartZeroFillOffset != 0)
            {
                u64ZeroFilledStartHandoverOffset = u64StartZeroFillOffset;
            }
            else
            {
                u64ZeroFilledStartHandoverOffset =
                        strLogBuf2->u64CtPktOffset[strLogBuf2->u32LogSize - 1] +
                        (strLogBuf2->u32CtPktSize[strLogBuf2->u32LogSize - 1]);
            }
        }
        else
        {
            u64ZeroFilledStartHandoverOffset =
                    strLogBuf2->u64CtPktOffset[strLogBuf2->u32LogSize - 1] +
                    (strLogBuf2->u32CtPktSize[strLogBuf2->u32LogSize - 1]);
        }

        /** Resetting the log buffer    */
        memset(strLogBuf2, 0, sizeof(strInlineProcLogFileStats));
    }
}

/** @fn void cUdpDataReceiver::WriteOffsetLog(ULONG64 u64ZeroFilledOffset, ULONG64 u64ZeroFilledBytes)
 * @brief This function is to capture zero filled offset and bytes in log file
 * @param [in] u64ZeroFilledOffset [ULONG64] - Zero filled offset
 * @param [in] u64ZeroFilledBytes [ULONG64] - Number of Zero filled bytes
 */
void cUdpDataReceiver::WriteOffsetLog(ULONG64 u64ZeroFilledOffset,
                                          ULONG64 u64ZeroFilledBytes)
{
    if(pInlineLogFile != NULL)
    {
        if(u8DataTypeId == RAW_DATA_INDEX)
        {
			sprintf(s8LogMsg, "\n,%llu,%llu,",
                u64ZeroFilledOffset, u64ZeroFilledBytes);
        }
		else if(u8DataTypeId == CP_DATA_1_INDEX)
        {
			sprintf(s8LogMsg, "\n,,,%llu,%llu,",
                u64ZeroFilledOffset, u64ZeroFilledBytes);
        }
		else if(u8DataTypeId == CQ_DATA_2_INDEX)
        {
			sprintf(s8LogMsg, "\n,,,,,%llu,%llu,",
                u64ZeroFilledOffset, u64ZeroFilledBytes);
        }
		else if(u8DataTypeId == R4F_DATA_3_INDEX)
        {
			sprintf(s8LogMsg, "\n,,,,,,,%llu,%llu,",
                u64ZeroFilledOffset, u64ZeroFilledBytes);
        }
		else if(u8DataTypeId == DSP_DATA_4_INDEX)
        {
			sprintf(s8LogMsg, "\n,,,,,,,,,%llu,%llu,",
                u64ZeroFilledOffset, u64ZeroFilledBytes);
        }
		fprintf(pInlineLogFile, "%s", s8LogMsg);
    }
    else
    {
        printf("\n Not able to write inline processing logs\n");
    }
}

/** @fn void cUdpDataReceiver::WriteOffsetMetaData(ULONG64 u64PrevPktOffset, UINT32 u32PrevPktSize, ULONG64 u64CtPktOffset, UINT32 u32CtPktSize)
 * @brief This function is to capture the out of sequence packet metadata in a log file <!--
 * --> which will be processed for getting the offset and bytes dropped.
 * @param [in] u64PrevPktOffset [ULONG64] - Previous packet offset
 * @param [in] u32PrevPktSize [UINT32] - Previous packet size
 * @param [in] u64CtPktOffset [ULONG64] - Current packet offset
 * @param [in] u32CtPktSize [UINT32] - Current packet size
*/
void cUdpDataReceiver::WriteOffsetMetaData(ULONG64 u64PrevPktOffset, UINT32 u32PrevPktSize,
                             ULONG64 u64CtPktOffset, UINT32 u32CtPktSize)
{
    if(pInlineLogFile != NULL)
    {
		/** Dropped packets offset and bytes        */
		if(sRFDCCard_InlineStats.u32OutOfSeqPktFromOffset[u8DataTypeId] <
				sRFDCCard_InlineStats.u32OutOfSeqPktToOffset[u8DataTypeId])
		{
			if(u8DataTypeId == RAW_DATA_INDEX)
				sprintf(s8LogMsg, "\n,%llu,%llu,,,",
					(u64PrevPktOffset + u32PrevPktSize),
					(u64CtPktOffset - (u64PrevPktOffset + u32PrevPktSize)));
			else if(u8DataTypeId == CP_DATA_1_INDEX)
				sprintf(s8LogMsg, "\n,,,,,,%llu,%llu,,,",
					(u64PrevPktOffset + u32PrevPktSize),
					(u64CtPktOffset - (u64PrevPktOffset + u32PrevPktSize)));
			else if(u8DataTypeId == CQ_DATA_2_INDEX)
				sprintf(s8LogMsg, "\n,,,,,,,,,,,%llu,%llu,,,",
					(u64PrevPktOffset + u32PrevPktSize),
					(u64CtPktOffset - (u64PrevPktOffset + u32PrevPktSize)));
			else if(u8DataTypeId == R4F_DATA_3_INDEX)
				sprintf(s8LogMsg, "\n,,,,,,,,,,,,,,,,%llu,%llu,,,",
					(u64PrevPktOffset + u32PrevPktSize),
					(u64CtPktOffset - (u64PrevPktOffset + u32PrevPktSize)));
			else if(u8DataTypeId == DSP_DATA_4_INDEX)
				sprintf(s8LogMsg, "\n,,,,,,,,,,,,,,,,,,,,,%llu,%llu,,,",
					(u64PrevPktOffset + u32PrevPktSize),
					(u64CtPktOffset - (u64PrevPktOffset + u32PrevPktSize)));

		}
		/** Old packet offset and bytes             */
		else
		{
			if(u8DataTypeId == RAW_DATA_INDEX)
				sprintf(s8LogMsg, "\n,,,%llu,%u,", 
					u64CtPktOffset, u32CtPktSize);
			else if(u8DataTypeId == CP_DATA_1_INDEX)
				sprintf(s8LogMsg, "\n,,,,,,,,%llu,%u,", 
					u64CtPktOffset, u32CtPktSize);
			else if(u8DataTypeId == CQ_DATA_2_INDEX)
				sprintf(s8LogMsg, "\n,,,,,,,,,,,,,%llu,%u,", 
					u64CtPktOffset, u32CtPktSize);
			else if(u8DataTypeId == R4F_DATA_3_INDEX)
				sprintf(s8LogMsg, "\n,,,,,,,,,,,,,,,,,,%llu,%u,", 
					u64CtPktOffset, u32CtPktSize);
			else if(u8DataTypeId == DSP_DATA_4_INDEX)
				sprintf(s8LogMsg, "\n,,,,,,,,,,,,,,,,,,,,,,,%llu,%u,", 
					u64CtPktOffset, u32CtPktSize);
		}
		
        fprintf(pInlineLogFile, "%s", s8LogMsg);
    }
    else
    {
        printf("\nNot able to write inline processing out of seq metadata\n");
    }
}

#endif
