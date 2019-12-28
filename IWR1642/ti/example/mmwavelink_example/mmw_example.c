/****************************************************************************************
* FileName     : mmw_example.c
*
* Description  : This file implements mmwave link example application.
*
****************************************************************************************
* (C) Copyright 2014, Texas Instruments Incorporated. - TI web address www.ti.com
*---------------------------------------------------------------------------------------
*
*  Redistribution and use in source and binary forms, with or without modification,
*  are permitted provided that the following conditions are met:
*
*    Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of its
*    contributors may be used to endorse or promote products derived from this
*    software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
*  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
*  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR CONTRIBUTORS
*  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*  CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/
/******************************************************************************
* INCLUDE FILES
******************************************************************************
*/
#include <windows.h>
#include <stdio.h>
#include <share.h>
#include <string.h>
#include <stdlib.h>
#include "mmw_example.h"
#include "mmw_config.h"
#include <ti/control/mmwavelink/mmwavelink.h>
#include "rls_studio.h"
#include "rls_osi.h"
/* AWR1243 meta image file for ES 3.0 */
#include "firmware/xwr12xx_metaImage.h"

/****************************************************************************************
* MACRO DEFINITIONS
****************************************************************************************
*/
#define MMWL_FW_FIRST_CHUNK_SIZE (220U)
#define MMWL_FW_CHUNK_SIZE (228U)
#define MMWL_META_IMG_FILE_SIZE (sizeof(metaImage))

#define GET_BIT_VALUE(data, noOfBits, location)    ((((rlUInt32_t)(data)) >> (location)) &\
                                               (((rlUInt32_t)((rlUInt32_t)1U << (noOfBits))) - (rlUInt32_t)1U))
/* Async Event Timeouts */
#define MMWL_API_INIT_TIMEOUT                (2000) /* 2 Sec*/
#define MMWL_API_START_TIMEOUT               (1000) /* 1 Sec*/
#define MMWL_API_RF_INIT_TIMEOUT             (1000) /* 1 Sec*/

/* To Enable File download */
#define FIRMWARE_DOWNLOAD                      0

/* MAX unique chirp AWR1243 supports */
#define MAX_UNIQUE_CHIRP_INDEX                (512 -1)

/* MAX index to read back chirp config  */
#define MAX_GET_CHIRP_CONFIG_IDX              14

/* To enable TX2 */
#define ENABLE_TX2                             0

/******************************************************************************
* GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
******************************************************************************
*/
typedef int (*RL_P_OS_SPAWN_FUNC_PTR)(RL_P_OSI_SPAWN_ENTRY pEntry, const void* pValue, unsigned int flags);
typedef int (*RL_P_OS_DELAY_FUNC_PTR)(unsigned int delay);

/* Global Variable for Device Status */
static unsigned char mmwl_bInitComp = 0U;
static unsigned char mmwl_bStartComp = 0U;
static unsigned char mmwl_bRfInitComp = 0U;
static unsigned char mmwl_bSensorStarted = 0U;
static unsigned char mmwl_bGpadcDataRcv = 0U;

unsigned char gAwr1243CrcType = RL_CRC_TYPE_32BIT;

/* Enable Advanced frame config test in the application
TRUE -> config AdvFrame
FALSE -> config Legacy Frame */
unsigned char gLinkAdvanceFrameTest = FALSE;

/* Enable/Disable continuous mode config test in the application */
unsigned char gLinkContModeTest = FALSE;

/* Enable/Disable Dynamic Chirp confing & Dynamic Chirp Enable test in application */
unsigned char gLinkDynChirpTest = FALSE;

/* Enable/Disable Dynamic Profile config test in application */
unsigned char gLinkDynProfileTest = FALSE;

/* TRUE -> Enable LDO bypass. Support only on AWR1243 Rev B EVMs
   FALSE -> Disable LDO bypass.
   CAUTION : DON'T ENABLE LDO BYPASS ON UNSUPPORTED DEVICES. IT MAY DAMAGE EVM. */
unsigned char gLinkBypassLdo = FALSE;

/* Enable/Disable calibration data store restore test in application */
unsigned char gLinkCalibStoreRestoreTest = FALSE;

/* store frame periodicity */
unsigned int framePeriodicity = 0;
/* store frame count */
unsigned int frameCount = 0;

/* SPI Communication handle to AWR1243 device*/
rlComIfHdl_t mmwl_devHdl = NULL;

/* structure parameters of two profile confing and cont mode config are same */
rlProfileCfg_t profileCfgArgs[2] = { 0 };

/* strcture to store dynamic chirp configuration */
rlDynChirpCfg_t dynChirpCfgArgs[3] = { 0 };

/* Strcture to store async event config */
rlRfDevCfg_t rfDevCfg = { 0x0 };

/* Structure to store GPADC measurement data sent by device */
rlRecvdGpAdcData_t rcvGpAdcData = {0};

/* Structure to store mmWave device Calibration data */
rlCalibrationData_t calibData = { 0 };

uint64_t computeCRC(uint8_t *p, uint32_t len, uint8_t width);

/* Function to compare dynamically configured chirp data */
int MMWL_chirpParamCompare(rlChirpCfg_t * chirpData);

/******************************************************************************
* all function definations starts here
*******************************************************************************
*/

/** @fn void MMWL_asyncEventHandler(rlUInt8_t deviceIndex, rlUInt16_t sbId,
*    rlUInt16_t sbLen, rlUInt8_t *payload)
*
*   @brief Radar Async Event Handler callback
*   @param[in] msgId - Message Id
*   @param[in] sbId - SubBlock Id
*   @param[in] sbLen - SubBlock Length
*   @param[in] payload - Sub Block Payload
*
*   @return None
*
*   Radar Async Event Handler callback
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
void MMWL_asyncEventHandler(rlUInt8_t deviceIndex, rlUInt16_t sbId,
    rlUInt16_t sbLen, rlUInt8_t *payload)
{
    unsigned int deviceMap = 0;
    rlUInt16_t msgId = sbId / RL_MAX_SB_IN_MSG;
    rlUInt16_t asyncSB = RL_GET_SBID_FROM_MSG(sbId, msgId);

    /* Host can receive Async Event from RADARSS/MSS */
    switch (msgId)
    {
        /* Async Event from RADARSS */
        case RL_RF_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
            case RL_RF_AE_INITCALIBSTATUS_SB:
            {
                mmwl_bRfInitComp = 1U;
                printf("Async event: RF-init calibration status \n");
            }
            break;
            case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
            {
                mmwl_bSensorStarted = 1U;
                printf("Async event: Frame trigger \n");
            }
            break;
            case RL_RF_AE_FRAME_END_SB:
            {
                mmwl_bSensorStarted = 0U;
                printf("Async event: Frame stopped \n");
            }
            break;
            case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
            {
                printf("Aync event: Run time Calibration Report [0x%x]\n", ((rlRfRunTimeCalibReport_t*)payload)->calibErrorFlag);
            }
            break;
            case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
            {
                printf("Aync event: Monitoring Timing Failed Report\n");
                break;
            }
            case RL_RF_AE_GPADC_MEAS_DATA_SB:
            {
                mmwl_bGpadcDataRcv = 1U;
                /* store the GPAdc Measurement data which AWR1243 will read from the analog test pins
                    where user has fed the input signal */
                memcpy(&rcvGpAdcData, payload, sizeof(rlRecvdGpAdcData_t));
                break;
            }
            case RL_RF_AE_CPUFAULT_SB:
            {
                printf("BSS CPU fault \n");
                while(1);
                break;
            }
            case RL_RF_AE_ESMFAULT_SB:
            {
                printf("BSS ESM fault \n");
                break;
            }
            default:
                break;
            }

        }
        break;

        /* Async Event from MSS */
        case RL_DEV_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
                case RL_DEV_AE_MSSPOWERUPDONE_SB:
                {
                    mmwl_bInitComp = 1U;
                }
                break;
                case RL_DEV_AE_MSS_BOOTERRSTATUS_SB:
                {
                    mmwl_bInitComp = 1U;
                }
                break;
                case RL_DEV_AE_RFPOWERUPDONE_SB:
                {
                    mmwl_bStartComp = 1U;
                }
                break;
                case RL_DEV_AE_MSS_ESMFAULT_SB:
                {
                    printf("MSS ESM Error \n");
                }
                break;
                case RL_DEV_AE_MSS_CPUFAULT_SB:
                {
                    printf("MSS CPU Fault\n");
                }
                break;
            }
        }
        break;

        /* Async Event from MMWL */
        case RL_MMWL_ASYNC_EVENT_MSG:
        {
            case RL_MMWL_AE_MISMATCH_REPORT:
            {
                int errTemp = *(int32_t*)payload;
                /* CRC mismatched in the received Async-Event msg */
                if (errTemp == RL_RET_CODE_CRC_FAILED)
                {
                    printf("CRC mismatched in the received Async-Event msg \n" );
                }
                /* Checksum mismatched in the received msg */
                else if (errTemp == RL_RET_CODE_CHKSUM_FAILED)
                {
                    printf("Checksum mismatched in the received msg \n" );
                }
                /* Polling to HostIRQ is timed out,
                i.e. Device didn't respond to CNYS from the Host */
                else if (errTemp == RL_RET_CODE_HOSTIRQ_TIMEOUT)
                {
                    printf("HostIRQ polling timed out \n");
                }
                else if (errTemp == RL_RET_CODE_RADAR_OSIF_ERROR)
                {
                    printf("mmWaveLink error \n");
                }
                break;
            }
            break;
        }
        default:
        {
            printf("Unhandled Aync Event msgId: 0x%x, asyncSB:0x%x  \n", msgId, asyncSB);
            break;
        }
    }
}

/** @fn rlComIfHdl_t MMWL_spiOpen(unsigned char deviceIndex, unsigned in flags)
*
*   @brief SPI Open callback
*   @param[in] deviceIndex - Device Index
*   @param[in] flags - Optional Flags
*
*   @return INT32 Success - Handle to communication channel, Failure - NULL
*
*   SPI Open callback
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
rlComIfHdl_t MMWL_spiOpen(unsigned char deviceIndex, unsigned int flags)
{
    printf("rlComIfOpen Callback Called for Device Index [%d]\n", deviceIndex);
    mmwl_devHdl = rlsSpiOpen(&deviceIndex, flags);
    return mmwl_devHdl;
}

/** @fn int MMWL_enableDevice(unsigned char deviceIndex)
*
*   @brief Performs SOP and enables the device.
*
*   @param[in] deviceIndex
*
*   @return int Success - 0, Failure - Error Code
*
*   Power on Slave device API.
*/
int MMWL_enableDevice(unsigned char deviceIndex)
{
    int retVal = RL_RET_CODE_OK;
    /* Enable device in Functional Mode (SOP-4) */
    printf("rlDeviceEnable Callback is called by mmWaveLink for Device Index [%d]\n", deviceIndex);
    return rlsEnableDevice(deviceIndex);
}

/** @fn int MMWL_disableDevice(unsigned char deviceIndex)
*
*   @brief disables the device.
*
*   @param[in] deviceIndex
*
*   @return int Success - 0, Failure - Error Code
*
*   Power on Slave device API.
*/
int MMWL_disableDevice(unsigned char deviceIndex)
{
    printf("rlDeviceDisable Callback is called by mmWaveLink for Device Index [%d]\n", deviceIndex);
    return rlsDisableDevice(deviceIndex);
}

/** @fn int MMWL_computeCRC(unsigned char* data, unsigned int dataLen, unsigned char crcLen,
                        unsigned char* outCrc)
*
*   @brief Compute the CRC of given data
*
*   @param[in] data - message data buffer pointer
*    @param[in] dataLen - length of data buffer
*    @param[in] crcLen - length of crc 2/4/8 bytes
*    @param[out] outCrc - computed CRC data
*
*   @return int Success - 0, Failure - Error Code
*
*   Compute the CRC of given data
*/
int MMWL_computeCRC(unsigned char* data, unsigned int dataLen, unsigned char crcLen,
                        unsigned char* outCrc)
{
    uint64_t crcResult = computeCRC(data, dataLen, (16 << crcLen));
    memcpy(outCrc, &crcResult, (2 << crcLen));
    return 0;
}

/** @fn int MMWL_powerOnMaster(deviceMap)
*
*   @brief Power on Master API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Power on Master API.
*/
int MMWL_powerOnMaster(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
    /*
     \subsection     porting_step1   Step 1 - Define mmWaveLink client callback structure
    The mmWaveLink framework is ported to different platforms using mmWaveLink client callbacks. These
    callbacks are grouped as different structures such as OS callbacks, Communication Interface
    callbacks and others. Application needs to define these callbacks and initialize the mmWaveLink
    framework with the structure.

     Refer to \ref rlClientCbs_t for more details
     */
    rlClientCbs_t clientCtx = { 0 };

    /*Read all the parameters from config file*/
    MMWL_readPowerOnMaster(&clientCtx);

    /* store CRC Type which has been read from mmwaveconfig.txt file */
    gAwr1243CrcType = clientCtx.crcType;

    /*
    \subsection     porting_step2   Step 2 - Implement Communication Interface Callbacks
    The mmWaveLink device support several standard communication protocol among SPI and MailBox
    Depending on device variant, one need to choose the communication channel. For e.g
    xWR1443/xWR1642 requires Mailbox interface and AWR1243 supports SPI interface.
    The interface for this communication channel should include 4 simple access functions:
    -# rlComIfOpen
    -# rlComIfClose
    -# rlComIfRead
    -# rlComIfWrite

    Refer to \ref rlComIfCbs_t for interface details
    */
    clientCtx.comIfCb.rlComIfOpen = MMWL_spiOpen;
    clientCtx.comIfCb.rlComIfClose = rlsSpiClose;
    clientCtx.comIfCb.rlComIfRead = rlsSpiRead;
    clientCtx.comIfCb.rlComIfWrite = rlsSpiWrite;

    /*   \subsection     porting_step3   Step 3 - Implement Device Control Interface
    The mmWaveLink driver internally powers on/off the mmWave device. The exact implementation of
    these interface is platform dependent, hence you need to implement below functions:
    -# rlDeviceEnable
    -# rlDeviceDisable
    -# rlRegisterInterruptHandler

    Refer to \ref rlDeviceCtrlCbs_t for interface details
    */
    clientCtx.devCtrlCb.rlDeviceDisable = MMWL_disableDevice;
    clientCtx.devCtrlCb.rlDeviceEnable = MMWL_enableDevice;
    clientCtx.devCtrlCb.rlDeviceMaskHostIrq = rlsSpiIRQMask;
    clientCtx.devCtrlCb.rlDeviceUnMaskHostIrq = rlsSpiIRQUnMask;
    clientCtx.devCtrlCb.rlRegisterInterruptHandler = rlsRegisterInterruptHandler;
    clientCtx.devCtrlCb.rlDeviceWaitIrqStatus = rlsDeviceWaitIrqStatus;

    /*  \subsection     porting_step4     Step 4 - Implement Event Handlers
    The mmWaveLink driver reports asynchronous event indicating mmWave device status, exceptions
    etc. Application can register this callback to receive these notification and take appropriate
    actions

    Refer to \ref rlEventCbs_t for interface details*/
    clientCtx.eventCb.rlAsyncEvent = MMWL_asyncEventHandler;

    /*  \subsection     porting_step5     Step 5 - Implement OS Interface
    The mmWaveLink driver can work in both OS and NonOS environment. If Application prefers to use
    operating system, it needs to implement basic OS routines such as tasks, mutex and Semaphore


    Refer to \ref rlOsiCbs_t for interface details
    */
    /* Mutex */
    clientCtx.osiCb.mutex.rlOsiMutexCreate = osiLockObjCreate;
    clientCtx.osiCb.mutex.rlOsiMutexLock = osiLockObjLock;
    clientCtx.osiCb.mutex.rlOsiMutexUnLock = osiLockObjUnlock;
    clientCtx.osiCb.mutex.rlOsiMutexDelete = osiLockObjDelete;

    /* Semaphore */
    clientCtx.osiCb.sem.rlOsiSemCreate = osiSyncObjCreate;
    clientCtx.osiCb.sem.rlOsiSemWait = osiSyncObjWait;
    clientCtx.osiCb.sem.rlOsiSemSignal = osiSyncObjSignal;
    clientCtx.osiCb.sem.rlOsiSemDelete = osiSyncObjDelete;

    /* Spawn Task */
    clientCtx.osiCb.queue.rlOsiSpawn = (RL_P_OS_SPAWN_FUNC_PTR)osiSpawn;

    /* Sleep/Delay Callback*/
    clientCtx.timerCb.rlDelay = (RL_P_OS_DELAY_FUNC_PTR)osiSleep;

    /*  \subsection     porting_step6     Step 6 - Implement CRC Interface
    The mmWaveLink driver uses CRC for message integrity. If Application prefers to use
    CRC, it needs to implement CRC routine.

    Refer to \ref rlCrcCbs_t for interface details
    */
    clientCtx.crcCb.rlComputeCRC = MMWL_computeCRC;

    /*  \subsection     porting_step7     Step 7 - Define Platform
    The mmWaveLink driver can be configured to run on different platform by
    passing appropriate platform and device type
    */
    clientCtx.platform = RL_PLATFORM_HOST;
    clientCtx.arDevType = RL_AR_DEVICETYPE_12XX;

    /*clear all the interupts flag*/
    mmwl_bInitComp = 0;
    mmwl_bStartComp = 0U;
    mmwl_bRfInitComp = 0U;

    /*  \subsection     porting_step8     step 8 - Call Power ON API and pass client context
    The mmWaveLink driver initializes the internal components, creates Mutex/Semaphore,
    initializes buffers, register interrupts, bring mmWave front end out of reset.
    */
    retVal = rlDevicePowerOn(deviceMap, clientCtx);

    /*  \subsection     porting_step9     step 9 - Test if porting is successful
    Once configuration is complete and mmWave device is powered On, mmWaveLink driver receives
    asynchronous event from mmWave device and notifies application using
    asynchronous event callback

    Refer to \ref MMWL_asyncEventHandler for event details
    */
    while (mmwl_bInitComp == 0U)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    mmwl_bInitComp = 0U;
    return retVal;
}

int MMWL_fileWrite(unsigned char deviceMap,
                unsigned short remChunks,
                unsigned short chunkLen,
                unsigned char *chunk)
{
    int ret_val = -1;

    rlFileData_t fileChunk = { 0 };
    fileChunk.chunkLen = chunkLen;
    memcpy(fileChunk.fData, chunk, chunkLen);

    ret_val = rlDeviceFileDownload(deviceMap, &fileChunk, remChunks);
    return ret_val;
}

/** @fn int MMWL_fileDownload((unsigned char deviceMap,
                  mmwlFileType_t fileType,
                  unsigned int fileLen)
*
*   @brief Firmware Download API.
*
*   @param[in] deviceMap - Devic Index
*    @param[in] fileType - firmware/file type
*    @param[in] fileLen - firmware/file length
*
*   @return int Success - 0, Failure - Error Code
*
*   Firmware Download API.
*/
int MMWL_fileDownload(unsigned char deviceMap,
                  unsigned int fileLen)
{
    unsigned int imgLen = fileLen;
    int ret_val = -1;
    int mmwl_iRemChunks = 0;
    unsigned short usChunkLen = 0U;
    unsigned int iNumChunks = 0U;
    unsigned short usLastChunkLen = 0;
    unsigned short usFirstChunkLen = 0;
    unsigned short usProgress = 0;

    /*First Chunk*/
    unsigned char firstChunk[MMWL_FW_CHUNK_SIZE];
    unsigned char* pmmwl_imgBuffer = NULL;

    pmmwl_imgBuffer = (unsigned char*)&metaImage[0];

    if(pmmwl_imgBuffer == NULL)
    {
        printf("MMWL_fileDwld Fail : File Buffer is NULL \n\r");
        return -1;
    }

    /*Download to Device*/
    usChunkLen = MMWL_FW_CHUNK_SIZE;
    iNumChunks = (imgLen + 8) / usChunkLen;
    mmwl_iRemChunks = iNumChunks;

    if (mmwl_iRemChunks > 0)
    {
        usLastChunkLen = (imgLen + 8) % usChunkLen;
        usFirstChunkLen = MMWL_FW_CHUNK_SIZE;
    }
    else
    {
        usFirstChunkLen = imgLen + 8;
    }

    *((unsigned int*)&firstChunk[0]) = (unsigned int)MMWL_FILETYPE_META_IMG;
    *((unsigned int*)&firstChunk[4]) = (unsigned int)imgLen;
    memcpy((char*)&firstChunk[8], (char*)pmmwl_imgBuffer,
                usFirstChunkLen - 8);

    ret_val = MMWL_fileWrite(deviceMap, mmwl_iRemChunks, usFirstChunkLen,
                              firstChunk);
    if (ret_val < 0)
    {
        printf("MMWL_fileDwld Fail : Ftype: %d\n\r", MMWL_FILETYPE_META_IMG);
        return ret_val;
    }
    pmmwl_imgBuffer += MMWL_FW_FIRST_CHUNK_SIZE;
    mmwl_iRemChunks--;

    if(mmwl_iRemChunks > 0)
    {
        printf("Download in Progress: %d%%..", usProgress);
    }
    /*Remaining Chunk*/
    while (mmwl_iRemChunks > 0)
    {
        if ((((iNumChunks - mmwl_iRemChunks) * 100)/iNumChunks - usProgress) > 10)
        {
            usProgress += 10;
            printf("%d%%..", usProgress);
        }

        ret_val = MMWL_fileWrite(deviceMap, mmwl_iRemChunks,
                    MMWL_FW_CHUNK_SIZE, pmmwl_imgBuffer);

        if (ret_val < 0)
        {
            printf("\n\r MMWL_fileDwld rem chunk Fail : Ftype: %d\n\r",
                MMWL_FILETYPE_META_IMG);
            return ret_val;
        }
        pmmwl_imgBuffer += MMWL_FW_CHUNK_SIZE;
        mmwl_iRemChunks--;
    }

    /*Last Chunk*/
    if (usLastChunkLen > 0)
    {
        ret_val = MMWL_fileWrite(deviceMap, 0, usLastChunkLen,
                                  pmmwl_imgBuffer);
        if (ret_val < 0)
        {
            printf("MMWL_fileDwld last chunk Fail : Ftype: %d\n\r",
                MMWL_FILETYPE_META_IMG);
            return ret_val;
        }
    }
     printf("Done!\n");
    return ret_val;
}

/** @fn int MMWL_firmwareDownload(deviceMap)
*
*   @brief Firmware Download API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Firmware Download API.
*/
int MMWL_firmwareDownload(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;

    /* Meta Image download */
    printf("Meta Image download started for deviceMap %u\n",
        deviceMap);
    retVal = MMWL_fileDownload(deviceMap, MMWL_META_IMG_FILE_SIZE);
    printf("Meta Image download complete ret = %d\n", retVal);

    printf("\nWaiting for firmware update response from mmWave device \n");
    while (mmwl_bInitComp == 0)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    mmwl_bInitComp = 0;
    return retVal;
}

/** @fn int MMWL_rfEnable(deviceMap)
*
*   @brief RFenable API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RFenable API.
*/
int MMWL_rfEnable(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
    retVal = rlDeviceRfStart(deviceMap);
    while (mmwl_bStartComp == 0U)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_START_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }

    }
    mmwl_bStartComp = 0;

    if(retVal == RL_RET_CODE_OK)
    {
        rlVersion_t verArgs = {0};
        retVal = rlDeviceGetVersion(deviceMap,&verArgs);

        printf("\nRF Version [%2d.%2d.%2d.%2d] \nMSS version [%2d.%2d.%2d.%2d] \nmmWaveLink version [%2d.%2d.%2d.%2d]\n",
            verArgs.rf.fwMajor, verArgs.rf.fwMinor, verArgs.rf.fwBuild, verArgs.rf.fwDebug,
            verArgs.master.fwMajor, verArgs.master.fwMinor, verArgs.master.fwBuild, verArgs.master.fwDebug,
            verArgs.mmWaveLink.major, verArgs.mmWaveLink.minor, verArgs.mmWaveLink.build, verArgs.mmWaveLink.debug);
        printf("\nRF Patch Version [%2d.%2d.%2d.%2d] \nMSS Patch version [%2d.%2d.%2d.%2d]\n",
            verArgs.rf.patchMajor, verArgs.rf.patchMinor, ((verArgs.rf.patchBuildDebug & 0xF0) >> 4), (verArgs.rf.patchBuildDebug & 0x0F),
            verArgs.master.patchMajor, verArgs.master.patchMinor, ((verArgs.master.patchBuildDebug & 0xF0) >> 4), (verArgs.master.patchBuildDebug & 0x0F));
    }
    return retVal;
}

/** @fn int MMWL_dataFmtConfig(unsigned char deviceMap)
*
*   @brief Data Format Config API
*
*   @return Success - 0, Failure - Error Code
*
*   Data Format Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_dataFmtConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevDataFmtCfg_t dataFmtCfgArgs = { 0 };

    /*dataFmtCfgArgs from config file*/
    MMWL_readDataFmtConfig(&dataFmtCfgArgs);

    retVal = rlDeviceSetDataFmtConfig(deviceMap, &dataFmtCfgArgs);
    return retVal;
}

/** @fn int MMWL_ldoBypassConfig(unsigned char deviceMap)
*
*   @brief LDO Bypass Config API
*
*   @return Success - 0, Failure - Error Code
*
*   LDO Bypass Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_ldoBypassConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlRfLdoBypassCfg_t rfLdoBypassCfgArgs = { 0 };

	// Eson: Set always FALSE because LDO may damage EVM besides 1432.
    /*if (gLinkBypassLdo == TRUE)
    {
        rfLdoBypassCfgArgs.ldoBypassEnable = 1;
    }*/

    printf("Calling rlRfSetLdoBypassConfig With Bypass [%d] \n",
        rfLdoBypassCfgArgs.ldoBypassEnable);

    retVal = rlRfSetLdoBypassConfig(deviceMap, &rfLdoBypassCfgArgs);
    return retVal;
}

/** @fn int MMWL_adcOutConfig(unsigned char deviceMap)
*
*   @brief ADC Configuration API
*
*   @return Success - 0, Failure - Error Code
*
*   ADC Configuration API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_adcOutConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;

    rlAdcOutCfg_t adcOutCfgArgs = { 0 };

    /*read adcOutCfgArgs from config file*/
    MMWL_readAdcOutConfig(&adcOutCfgArgs);


    printf("Calling rlSetAdcOutConfig With [%d]ADC Bits and [%d]ADC Format \n",
        adcOutCfgArgs.fmt.b2AdcBits, adcOutCfgArgs.fmt.b2AdcOutFmt);

    retVal = rlSetAdcOutConfig(deviceMap, &adcOutCfgArgs);
    return retVal;
}

/** @fn int MMWL_channelConfig(unsigned char deviceMap,
                               unsigned short cascading)
*
*   @brief Channel Config API
*
*   @return Success - 0, Failure - Error Code
*
*   Channel Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_channelConfig(unsigned char deviceMap,
                       unsigned short cascade)
{
    int retVal = RL_RET_CODE_OK;
    /* TBD - Read GUI Values */
    rlChanCfg_t rfChanCfgArgs = { 0 };

    /*read arguments from config file*/
    MMWL_readChannelConfig(&rfChanCfgArgs, cascade);

#if (ENABLE_TX2)
    rfChanCfgArgs.txChannelEn |= (1 << 2); // Enable TX2
#endif

    printf("Calling rlSetChannelConfig With b[%d]Rx and b[%d]Tx Channel Enabled \n",
           rfChanCfgArgs.rxChannelEn, rfChanCfgArgs.txChannelEn);

    retVal = rlSetChannelConfig(deviceMap, &rfChanCfgArgs);
    return retVal;
}

/** @fn int MMWL_setAsyncEventDir(unsigned char deviceMap)
*
*   @brief Update async event message direction and CRC type of Async event
*           from AWR1243 radarSS to Host
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Update async event message direction and CRC type of Async event
*   from AWR1243 radarSS to Host
*/
int MMWL_setAsyncEventDir(unsigned char  deviceMap)
{
    int32_t         retVal;
    /* set global and monitoring async event direction to Host */
    rfDevCfg.aeDirection = 0x05;
    /* Set the CRC type of Async event received from radarSS */
    rfDevCfg.aeCrcConfig = gAwr1243CrcType;
    retVal = rlRfSetDeviceCfg(deviceMap, &rfDevCfg);

    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlSetAsyncEventDir retVal=%d\n", retVal);
        return -1;
    }

    printf("Debug: Finished rlSetAsyncEventDir\n");

    return 0;
}

/** @fn int MMWL_setDeviceCrcType(unsigned char deviceMap)
*
*   @brief Set CRC type of async event from AWR1243 MasterSS
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Set CRC type of async event from AWR1243 MasterSS
*/
int MMWL_setDeviceCrcType(unsigned char deviceMap)
{
    int32_t         retVal;
    rlDevMiscCfg_t devMiscCfg = {0};
    /* Set the CRC Type for Async Event from MSS */
    devMiscCfg.aeCrcConfig = gAwr1243CrcType;
    retVal = rlDeviceSetMiscConfig(deviceMap, &devMiscCfg);

    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlDeviceSetMiscConfig retVal=%d\n", retVal);
        return -1;
    }

    printf("Debug: Finished rlDeviceSetMiscConfig\n");

    return 0;
}

/** @fn int MMWL_basicConfiguration(unsigned char deviceMap, unsigned int cascade)
*
*   @brief Channel, ADC,Data format configuration API.
*
*   @param[in] deviceMap - Devic Index
*    @param[in] unsigned int cascade
*
*   @return int Success - 0, Failure - Error Code
*
*   Channel, ADC,Data format configuration API.
*/
int MMWL_basicConfiguration(unsigned char deviceMap, unsigned int cascade)
{
    int retVal = RL_RET_CODE_OK;

    /* Set which Rx and Tx channels will be enable of the device */
    retVal = MMWL_channelConfig(deviceMap, cascade);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Channel Config failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf(">>>>Channel Configuration success for deviceMap %u\n\n", deviceMap);
    }

    /* ADC out data format configuration */
    retVal = MMWL_adcOutConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("AdcOut Config failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf(">>>>AdcOut Configuration success for deviceMap %u\n\n", deviceMap);
    }

    /* LDO bypass configuration */
    retVal = MMWL_ldoBypassConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("LDO Bypass Config failed for deviceMap %u with error code %d\n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf(">>>>LDO Bypass Configuration success for deviceMap %u\n\n", deviceMap);
    }

    /* Data format configuration */
    retVal = MMWL_dataFmtConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Data format Configuration failed for deviceMap %u with error code %d\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf(">>>>Data format Configuration success for deviceMap %u\n", deviceMap);
    }

    /* low power configuration */
    retVal = MMWL_lowPowerConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Low Power Configuration failed for deviceMap %u with error %d \n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf(">>>>Low Power Configuration success for deviceMap %u \n", deviceMap);
    }

    /* Async event direction and control configuration for RadarSS */
    retVal = MMWL_setAsyncEventDir(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("AsyncEvent Configuration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf(">>>>AsyncEvent Configuration success for deviceMap %u \n\n", deviceMap);
    }
    return retVal;
}

/** @fn int MMWL_rfInit(unsigned char deviceMap)
*
*   @brief RFinit API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RFinit API.
*/
int MMWL_rfInit(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
    rlRfInitCalConf_t rfCalibCfgArgs = { 0 };
    /* Enable all calibrations which is already default setting in the device. Application can
       change this calibraiton setting if required. */
    rfCalibCfgArgs.calibEnMask = 0x1FF0;
    /* This API enables/disables the calibrations to run. If applicaiton needs to disable any
       specific boot-time calibration it can use following API by setting "calibEnMask" with
       appropriate value. All the calibrations are enabled by default in the device. */
    retVal = rlRfInitCalibConfig(deviceMap, &rfCalibCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("RF calib configuration failed for deviceMap %u with error code %d \n\n",
            deviceMap, retVal);
        return -1;
    }

    if (retVal == 0)
    {
        mmwl_bRfInitComp = 0;
        /* Run boot time calibrations */
        retVal = rlRfInit(deviceMap);
        while (mmwl_bRfInitComp == 0U)
        {
            osiSleep(1); /*Sleep 1 msec*/
            timeOutCnt++;
            if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
            {
                retVal = RL_RET_CODE_RESP_TIMEOUT;
                break;
            }
        }
        mmwl_bRfInitComp = 0;
    }
    return retVal;
}

/** @fn int MMWL_calibStoreRestore(unsigned char deviceMap)
*
*   @brief Calibration data restore test.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   This function provides the reference how the Host can use the factory 
*   calibration and inject that to mmWave device.
*/
int MMWL_calibStoreRestore(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
    rlRfInitCalConf_t rfCalibCfgArgs = { 0 };
    /* Disable all calibrations if Host has calibration data stored and 
       wish to avoid running calibrations on mmWave device. */
    rfCalibCfgArgs.calibEnMask = 0x0;

    /* Power on the device */
    retVal = MMWL_powerOnMaster(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("mmWave Device Power on failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
#if (FIRMWARE_DOWNLOAD)
    /* Download firmware */
    retVal = MMWL_firmwareDownload(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Firmware update failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
#endif
    /* Set CRC type */
    //retVal = MMWL_setDeviceCrcType(deviceMap);
    //if (retVal != RL_RET_CODE_OK)
    //{
    //    printf("CRC Type set for MasterSS failed for deviceMap %u with error code %d \n\n",
    //        deviceMap, retVal);
    //    return -1;
    //}
    ///* Power on radarss */
    //retVal = MMWL_rfEnable(deviceMap);
    //if (retVal != RL_RET_CODE_OK)
    //{
    //    printf("Radar/RF subsystem Power up failed for deviceMap %u with error %d \n\n",
    //        deviceMap, retVal);
    //    return -1;
    //}
    /* Configure channels, ADC data, data path, etc.. */
    retVal = MMWL_basicConfiguration(deviceMap, 0);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Basic/Static configuration failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    /* Disble all the calibrations as Host has already stored the calibration data from
       the mmWave device and tried to inject to the device. */
    retVal = rlRfInitCalibConfig(deviceMap, &rfCalibCfgArgs);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("RF calib configuration failed for deviceMap %u with error code %d \n\n",
            deviceMap, retVal);
        return -1;

    }
    mmwl_bRfInitComp = 0;
    /* Invoke this API to inject lastly stored calibration data which was either
       stored on sFlash of Host or embedded in the Host application. */
    retVal = rlRfCalibDataRestore(deviceMap, &calibData);
    
    /* Device will send an Async Event if it doesn't detect any error in the calibration data */
    while (mmwl_bRfInitComp == 0U)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    mmwl_bRfInitComp = 0;
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Calibration data re-store failed for deviceMap %u with error code %d\n\n",
            deviceMap, retVal);
        return -1;
    }
    /* Invoke this API when device will use the above sent calibration data and skip 
       initiating the calibration. */
    mmwl_bRfInitComp = 0;
    retVal = rlRfInit(deviceMap);
    while (mmwl_bRfInitComp == 0U)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    mmwl_bRfInitComp = 0;
    if (retVal != RL_RET_CODE_OK)
    {
        printf("RF Initialization/Calibration failed for deviceMap %u with error code %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    return 0;
}

/** @fn int MMWL_profileConfig(unsigned char deviceMap)
*
*   @brief Profile configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Profile configuration API.
*/
int MMWL_profileConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    double endFreqConst;

    /*read profileCfgArgs from config file*/
    MMWL_readProfileConfig(&profileCfgArgs[0]);

    endFreqConst = (double)(profileCfgArgs[0].startFreqConst * 53.6441803 + (profileCfgArgs[0].rampEndTime * 10) * \
                   (profileCfgArgs[0].freqSlopeConst * 48.2797623))/53.6441803;
    if ((profileCfgArgs[0].startFreqConst >= 1435388860U) && ((unsigned int)endFreqConst <= 1509954515U))
    {
        /* If start frequency is in between 77GHz to 81GHz use VCO2 */
        profileCfgArgs[0].pfVcoSelect = 0x02;
    }
    else
    {
        /* If start frequency is in between 76GHz to 78GHz use VCO1 */
        profileCfgArgs[0].pfVcoSelect = 0x00;
    }

    printf("Calling rlSetProfileConfig with \nProfileId[%d]\nStart Frequency[%f] GHz\nRamp Slope[%f] MHz/uS \n",
        profileCfgArgs[0].profileId, (float)((profileCfgArgs[0].startFreqConst * 53.6441803)/(1000*1000*1000)),
        (float)(profileCfgArgs[0].freqSlopeConst * 48.2797623)/1000.0);
    /* with this API we can configure 2 profiles (max 4 profiles) at a time */
    retVal = rlSetProfileConfig(deviceMap, 1U, &profileCfgArgs[0U]);
    return retVal;
}

/** @fn int MMWL_chirpConfig(unsigned char deviceMap)
*
*   @brief Chirp configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Chirp configuration API.
*/
int MMWL_chirpConfig(unsigned char deviceMap)
{
    int i, retVal = RL_RET_CODE_OK;
    rlChirpCfg_t setChirpCfgArgs[2] = {0};

    rlChirpCfg_t getChirpCfgArgs[MAX_GET_CHIRP_CONFIG_IDX+1] = {0};

    /*read chirpCfgArgs from config file*/
    MMWL_readChirpConfig(&setChirpCfgArgs[0]);

    /* Setup second chirp configuration based on first read from mmwaveconfig.txt */
    /* check if first chirpConfig end Index is not covering MAX chirp index */
    if (setChirpCfgArgs[0].chirpEndIdx < MAX_UNIQUE_CHIRP_INDEX)
    {
        setChirpCfgArgs[1].chirpStartIdx = setChirpCfgArgs[0].chirpEndIdx + 1U;
    }
    else
    {
        setChirpCfgArgs[1].chirpStartIdx = setChirpCfgArgs[0].chirpEndIdx;
    }
    setChirpCfgArgs[1].chirpEndIdx   = MAX_UNIQUE_CHIRP_INDEX;
    setChirpCfgArgs[1].txEnable  = 2;

    printf("Calling rlSetChirpConfig with \nProfileId[%d]\nStart Idx[%d]\nEnd Idx[%d] \n",
                setChirpCfgArgs[0].profileId, setChirpCfgArgs[0].chirpStartIdx,
                setChirpCfgArgs[0].chirpEndIdx);
    /* With this API we can configure max 512 chirp in one call */
    retVal = rlSetChirpConfig(deviceMap, 2U, &setChirpCfgArgs[0U]);

    /* read back Chirp config, to verify that setChirpConfig actually set to Device
      @Note - This examples read back (10+1) num of chirp config for demonstration,
               which user can raise to match with their requirement */
    retVal = rlGetChirpConfig(deviceMap, setChirpCfgArgs[0].chirpStartIdx,
                              setChirpCfgArgs[0].chirpStartIdx + MAX_GET_CHIRP_CONFIG_IDX, &getChirpCfgArgs[0]);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("GetChirp Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
    }
    else
    {
        for (i=0; i <= MAX_GET_CHIRP_CONFIG_IDX; i++)
        {
            /* @Note- This check assumes that all chirp configs are configured by single setChirpCfgArgs[0] */
            /* compare each chirpConfig parameters to lastly configured via rlDynChirpConfig API */
            if ((getChirpCfgArgs[i].profileId != setChirpCfgArgs[0].profileId) || \
                (getChirpCfgArgs[i].freqSlopeVar != setChirpCfgArgs[0].freqSlopeVar) || \
                (getChirpCfgArgs[i].txEnable != setChirpCfgArgs[0].txEnable) || \
                (getChirpCfgArgs[i].startFreqVar != setChirpCfgArgs[0].startFreqVar) || \
                (getChirpCfgArgs[i].idleTimeVar != setChirpCfgArgs[0].idleTimeVar) || \
                (getChirpCfgArgs[i].adcStartTimeVar != setChirpCfgArgs[0].adcStartTimeVar))
            {
                    printf("*** Failed - Parameters are mismatched GetChirpConfig compare to rlSetChirpConfig *** \n");
                    break;
            }

        }

        if (i > MAX_GET_CHIRP_CONFIG_IDX)
        {
            printf("Debug: Get chirp configurations are matching with parameters configured during rlSetChirpConfig \n");
        }
    }

    return retVal;
}

int MMWL_chirpParamCompare(rlChirpCfg_t * chirpData)
{
    int retVal = RL_RET_CODE_OK, i = 0,j = 0;
    /* compare each chirpConfig parameters to lastly configured via rlDynChirpConfig API */
    while (i <= MAX_GET_CHIRP_CONFIG_IDX)
    {
        if (dynChirpCfgArgs[0].chirpRowSelect == 0x00)
        {
            if ((chirpData->profileId != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 4, 0)) || \
                (chirpData->freqSlopeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 6, 8)) || \
                (chirpData->txEnable != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 3, 16)) || \
                (chirpData->startFreqVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 23, 0)) || \
                (chirpData->idleTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 12, 0)) || \
                (chirpData->adcStartTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 12, 16)))
            {
                break;
            }
            i++;
            chirpData++;
        }
        else if (dynChirpCfgArgs[0].chirpRowSelect == 0x10)
        {
            if ((chirpData->profileId != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 4, 0)) || \
                (chirpData->freqSlopeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 6, 8)) || \
                (chirpData->txEnable != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 3, 16)))
            {
                break;
            }
            i++;
            chirpData++;
            if ((chirpData->profileId != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 4, 0)) || \
                (chirpData->freqSlopeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 6, 8)) || \
                (chirpData->txEnable != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 3, 16)))
            {
                break;
            }
            i++;
            chirpData++;
            if ((chirpData->profileId != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 4, 0)) || \
                (chirpData->freqSlopeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 6, 8)) || \
                (chirpData->txEnable != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 3, 16)))
            {
                break;
            }
            i++;
            chirpData++;
        }
        else if (dynChirpCfgArgs[0].chirpRowSelect == 0x20)
        {
            if (chirpData->startFreqVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 23, 0))
            {
                break;
            }
            i++;
            chirpData++;
            if (chirpData->startFreqVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 23, 0))
            {
                break;
            }
            i++;
            chirpData++;
            if (chirpData->startFreqVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 23, 0))
            {
                break;
            }
            i++;
            chirpData++;
        }
        else if (dynChirpCfgArgs[0].chirpRowSelect == 0x30)
        {
            if ((chirpData->idleTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 12, 0)) || \
                (chirpData->adcStartTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR1, 12, 16)))
            {
                break;
            }
            i++;
            chirpData++;
            if ((chirpData->idleTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 12, 0)) || \
                (chirpData->adcStartTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR2, 12, 16)))
            {
                break;
            }
            i++;
            chirpData++;
            if ((chirpData->idleTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 12, 0)) || \
                (chirpData->adcStartTimeVar != GET_BIT_VALUE(dynChirpCfgArgs[0].chirpRow[j].chirpNR3, 12, 16)))
            {
                break;
            }
            i++;
            chirpData++;
        }
        j++;
    }
    if (i <= MAX_GET_CHIRP_CONFIG_IDX)
    {
        retVal = -1;
    }
    return retVal;
}
int MMWL_getDynChirpConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK,i = 0, j= 0, chirpNotMatch = 0;
    unsigned short chirpStartIdx;
    rlChirpCfg_t chirpCfgArgs[MAX_GET_CHIRP_CONFIG_IDX+1] = {0};
    if (dynChirpCfgArgs[0].chirpRowSelect == 0x00)
    {
        chirpStartIdx = (dynChirpCfgArgs[0].chirpSegSel * 16);
    }
    else
    {
        chirpStartIdx = (dynChirpCfgArgs[0].chirpSegSel * 48);
    }
    /* get the chirp config for (10+1) chirps for which it's being updated by dynChirpConfig API
       @Note - This examples read back (10+1) num of chirp config for demonstration,
               which user can raise to match with their requirement */
    retVal = rlGetChirpConfig(deviceMap, chirpStartIdx, chirpStartIdx + MAX_GET_CHIRP_CONFIG_IDX, &chirpCfgArgs[0]);

    if (retVal != RL_RET_CODE_OK)
    {
        printf("*** Failed - rlGetChirpConfig failed with %d*** \n",retVal);
    }

    retVal = MMWL_chirpParamCompare(&chirpCfgArgs[0]);

    if (retVal != RL_RET_CODE_OK)
    {
        printf("*** Failed - Parameters are mismatched GetChirpConfig compare to dynChirpConfig *** \n");
    }
    else
    {
        printf("Debug: Get chirp configurations are matching with parameters configured via dynChirpConfig \n");
    }

    return retVal;
}

/** @fn int MMWL_frameConfig(unsigned char deviceMap)
*
*   @brief Frame configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Frame configuration API.
*/
int MMWL_frameConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlFrameCfg_t frameCfgArgs = { 0 };

    /*read frameCfgArgs from config file*/
    MMWL_readFrameConfig(&frameCfgArgs);

    framePeriodicity = (frameCfgArgs.framePeriodicity * 5)/(1000*1000);
    frameCount = frameCfgArgs.numFrames;

    printf("Calling rlSetFrameConfig with \nStart Idx[%d]\nEnd Idx[%d]\nLoops[%d]\nPeriodicity[%d]ms \n",
        frameCfgArgs.chirpStartIdx, frameCfgArgs.chirpEndIdx,
        frameCfgArgs.numLoops, (frameCfgArgs.framePeriodicity * 5)/(1000*1000));

    retVal = rlSetFrameConfig(deviceMap, &frameCfgArgs);
    return retVal;
}

/** @fn int MMWL_advFrameConfig(unsigned char deviceMap)
*
*   @brief Advance Frame configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Frame configuration API.
*/
int MMWL_advFrameConfig(unsigned char deviceMap)
{
    int i, retVal = RL_RET_CODE_OK;
    rlAdvFrameCfg_t AdvframeCfgArgs = { 0 };
    rlAdvFrameCfg_t GetAdvFrameCfgArgs = { 0 };
    /* reset frame periodicity to zero */
    framePeriodicity = 0;

    /*read frameCfgArgs from config file*/
    MMWL_readAdvFrameConfig(&AdvframeCfgArgs);

    /* Add all subframes periodicity to get whole frame periodicity */
    for (i=0; i < AdvframeCfgArgs.frameSeq.numOfSubFrames; i++)
        framePeriodicity += AdvframeCfgArgs.frameSeq.subFrameCfg[i].subFramePeriodicity;

    framePeriodicity = (framePeriodicity * 5)/(1000*1000);
    /* store total number of frames configured */
    frameCount = AdvframeCfgArgs.frameSeq.numFrames;

    printf("Calling rlSetAdvFrameConfig with \nnumOfSubFrames[%d]\nforceProfile[%d]\nnumFrames[%d]\ntriggerSelect[%d]ms \n",
        AdvframeCfgArgs.frameSeq.numOfSubFrames, AdvframeCfgArgs.frameSeq.forceProfile,
        AdvframeCfgArgs.frameSeq.numFrames, AdvframeCfgArgs.frameSeq.triggerSelect);

    retVal = rlSetAdvFrameConfig(deviceMap, &AdvframeCfgArgs);
    if (retVal == 0)
    {
        retVal = rlGetAdvFrameConfig(deviceMap, &GetAdvFrameCfgArgs);
        if ((AdvframeCfgArgs.frameSeq.forceProfile != GetAdvFrameCfgArgs.frameSeq.forceProfile) || \
            (AdvframeCfgArgs.frameSeq.frameTrigDelay != GetAdvFrameCfgArgs.frameSeq.frameTrigDelay) || \
            (AdvframeCfgArgs.frameSeq.numFrames != GetAdvFrameCfgArgs.frameSeq.numFrames) || \
            (AdvframeCfgArgs.frameSeq.numOfSubFrames != GetAdvFrameCfgArgs.frameSeq.numOfSubFrames) || \
            (AdvframeCfgArgs.frameSeq.triggerSelect != GetAdvFrameCfgArgs.frameSeq.triggerSelect))
        {
            printf("MMWL_readAdvFrameConfig failed...\n");
            return retVal;
        }
    }
    return retVal;
}

/**
 *******************************************************************************
 *
 * \brief   Local function to enable the dummy input of objects from AWR143
 *
 * \param   None
 * /return  retVal   BSP_SOK if the test source is set correctly.
 *
 *******************************************************************************
*/
#if defined (ENABLE_TEST_SOURCE)
int MMWL_testSourceConfig(unsigned char deviceMap)
{
    rlTestSource_t tsArgs = {0};
    rlTestSourceEnable_t tsEnableArgs = {0};
    int retVal = RL_RET_CODE_OK;

    tsArgs.testObj[0].posX = 0;

    tsArgs.testObj[0].posY = 500;
    tsArgs.testObj[0].posZ = 0;

    tsArgs.testObj[0].velX = 0;
    tsArgs.testObj[0].velY = 0;
    tsArgs.testObj[0].velZ = 0;

    tsArgs.testObj[0].posXMin = -32700;
    tsArgs.testObj[0].posYMin = 0;
    tsArgs.testObj[0].posZMin = -32700;

    tsArgs.testObj[0].posXMax = 32700;
    tsArgs.testObj[0].posYMax = 32700;
    tsArgs.testObj[0].posZMax = 32700;

    tsArgs.testObj[0].sigLvl = 150;

    tsArgs.testObj[1].posX = 0;
    tsArgs.testObj[1].posY = 32700;
    tsArgs.testObj[1].posZ = 0;

    tsArgs.testObj[1].velX = 0;
    tsArgs.testObj[1].velY = 0;
    tsArgs.testObj[1].velZ = 0;

    tsArgs.testObj[1].posXMin = -32700;
    tsArgs.testObj[1].posYMin = 0;
    tsArgs.testObj[1].posZMin = -32700;

    tsArgs.testObj[1].posXMax = 32700;
    tsArgs.testObj[1].posYMax = 32700;
    tsArgs.testObj[1].posZMax = 32700;

    tsArgs.testObj[1].sigLvl = 948;

    tsArgs.rxAntPos[0].antPosX = 0;
    tsArgs.rxAntPos[0].antPosZ = 0;
    tsArgs.rxAntPos[1].antPosX = 32;
    tsArgs.rxAntPos[1].antPosZ = 0;
    tsArgs.rxAntPos[2].antPosX = 64;
    tsArgs.rxAntPos[2].antPosZ = 0;
    tsArgs.rxAntPos[3].antPosX = 96;
    tsArgs.rxAntPos[3].antPosZ = 0;

    printf("Calling rlSetTestSourceConfig with Simulated Object at X[%d]cm, Y[%d]cm, Z[%d]cm \n",
            tsArgs.testObj[0].posX, tsArgs.testObj[0].posY, tsArgs.testObj[0].posZ);

    retVal = rlSetTestSourceConfig(deviceMap, &tsArgs);

    tsEnableArgs.tsEnable = 1U;
    retVal = rlTestSourceEnable(deviceMap, &tsEnableArgs);

    return retVal;
}
#endif

/** @fn int MMWL_dataPathConfig(unsigned char deviceMap)
*
*   @brief Data path configuration API. Configures CQ data size on the
*           lanes and number of samples of CQ[0-2] to br transferred.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Data path configuration API. Configures CQ data size on the
*   lanes and number of samples of CQ[0-2] to br transferred.
*/
int MMWL_dataPathConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevDataPathCfg_t dataPathCfgArgs = { 0 };

    /* read dataPathCfgArgs from config file */
    MMWL_readDataPathConfig(&dataPathCfgArgs);

    printf("Calling rlDeviceSetDataPathConfig with HSI Interface[%d] Selected \n",
            dataPathCfgArgs.intfSel);

    /* same API is used to configure CQ data size on the
     * lanes and number of samples of CQ[0-2] to br transferred.
     */
    retVal = rlDeviceSetDataPathConfig(deviceMap, &dataPathCfgArgs);
    return retVal;
}

/** @fn int MMWL_lvdsLaneConfig(unsigned char deviceMap)
*
*   @brief Lane Config API
*
*   @return Success - 0, Failure - Error Code
*
*   Lane Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_lvdsLaneConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevLvdsLaneCfg_t lvdsLaneCfgArgs = { 0 };

    /*read lvdsLaneCfgArgs from config file*/
    MMWL_readLvdsLaneConfig(&lvdsLaneCfgArgs);

    retVal = rlDeviceSetLvdsLaneConfig(deviceMap, &lvdsLaneCfgArgs);
    return retVal;
}

/** @fn int MMWL_laneConfig(unsigned char deviceMap)
*
*   @brief Lane Enable API
*
*   @return Success - 0, Failure - Error Code
*
*   Lane Enable API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_laneConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevLaneEnable_t laneEnCfgArgs = { 0 };

    /*read laneEnCfgArgs from config file*/
    MMWL_readLaneConfig(&laneEnCfgArgs);

    retVal = rlDeviceSetLaneConfig(deviceMap, &laneEnCfgArgs);
    return retVal;
}

/** @fn int MMWL_hsiLaneConfig(unsigned char deviceMap)
*
*   @brief LVDS lane configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   LVDS lane configuration API.
*/
int MMWL_hsiLaneConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    /*lane configuration*/
    retVal = MMWL_laneConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("LaneConfig failed for deviceMap %u with error code %d\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("LaneConfig success for deviceMap %u\n", deviceMap);
    }
    /*LVDS lane configuration*/
    retVal = MMWL_lvdsLaneConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("LvdsLaneConfig failed for deviceMap %u with error code %d\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("LvdsLaneConfig success for deviceMap %u\n", deviceMap);
    }
    return retVal;
}

/** @fn int MMWL_setHsiClock(unsigned char deviceMap)
*
*   @brief High Speed Interface Clock Config API
*
*   @return Success - 0, Failure - Error Code
*
*   HSI Clock Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_setHsiClock(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevHsiClk_t hsiClkgs = { 0 };

    /*read hsiClkgs from config file*/
    MMWL_readSetHsiClock(&hsiClkgs);

    printf("Calling rlDeviceSetHsiClk with HSI Clock[%d] \n",
            hsiClkgs.hsiClk);

    retVal = rlDeviceSetHsiClk(deviceMap, &hsiClkgs);
    return retVal;
}

/** @fn int MMWL_hsiDataRateConfig(unsigned char deviceMap)
*
*   @brief LVDS/CSI2 Clock Config API
*
*   @return Success - 0, Failure - Error Code
*
*   LVDS/CSI2 Clock Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_hsiDataRateConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevDataPathClkCfg_t dataPathClkCfgArgs = { 0 };

    /*read lvdsClkCfgArgs from config file*/
    MMWL_readLvdsClkConfig(&dataPathClkCfgArgs);

    printf("Calling rlDeviceSetDataPathClkConfig with HSI Data Rate[%d] Selected \n",
            dataPathClkCfgArgs.dataRate);

    retVal = rlDeviceSetDataPathClkConfig(deviceMap, &dataPathClkCfgArgs);
    return retVal;
}

/** @fn int MMWL_hsiClockConfig(unsigned char deviceMap)
*
*   @brief Clock configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Clock configuration API.
*/
int MMWL_hsiClockConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, readAllParams = 0;

    /*LVDS clock configuration*/
    retVal = MMWL_hsiDataRateConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("LvdsClkConfig failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_hsiDataRateConfig success for deviceMap %u\n\n", deviceMap);
    }

    /*set high speed clock configuration*/
    retVal = MMWL_setHsiClock(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setHsiClock failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setHsiClock success for deviceMap %u\n\n", deviceMap);
    }

    return retVal;
}

/** @fn int MMWL_gpadcMeasConfig(unsigned char deviceMap)
*
*   @brief API to set GPADC configuration.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code.
*
*   API to set GPADC Configuration. And device will    send GPADC
*    measurement data in form of Asynchronous event over SPI to
*    Host. User needs to feed input signal on the device pins where
*    they want to read the measurement data inside the device.
*/
int MMWL_gpadcMeasConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    int timeOutCnt = 0;
    rlGpAdcCfg_t gpadcCfg = {0};

    /* enable all the sensors [0-6] to read gpADC measurement data */
    gpadcCfg.enable = 0x3F;
    /* set the number of samples device needs to collect to do the measurement */
    gpadcCfg.numOfSamples[0].sampleCnt = 32;
    gpadcCfg.numOfSamples[1].sampleCnt = 32;
    gpadcCfg.numOfSamples[2].sampleCnt = 32;
    gpadcCfg.numOfSamples[3].sampleCnt = 32;
    gpadcCfg.numOfSamples[4].sampleCnt = 32;
    gpadcCfg.numOfSamples[5].sampleCnt = 32;
    gpadcCfg.numOfSamples[6].sampleCnt = 32;

    retVal = rlSetGpAdcConfig(deviceMap, &gpadcCfg);

    if(retVal == RL_RET_CODE_OK)
    {
        /* The actual GPADC measurement is done at the end of current burst/frame,
		   so GPADC data async event might get delayed by max one burst/frame */
        while (mmwl_bGpadcDataRcv == 0U)
        {
            osiSleep(1); /*Sleep 1 msec*/
            timeOutCnt++;
            if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
            {
                retVal = RL_RET_CODE_RESP_TIMEOUT;
                break;
            }
        }
    }

    return retVal;
}

/** @fn int MMWL_sensorStart(unsigned char deviceMap)
*
*   @brief API to Start sensor.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Start sensor.
*/
int MMWL_sensorStart(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    int timeOutCnt = 0;
    mmwl_bSensorStarted = 0U;
    retVal = rlSensorStart(deviceMap);
#ifndef ENABLE_TEST_SOURCE
    if ((rfDevCfg.aeControl & 0x1) == 0x0)
    {
        while (mmwl_bSensorStarted == 0U)
        {
            osiSleep(1); /*Sleep 1 msec*/
            timeOutCnt++;
            if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
            {
                retVal = RL_RET_CODE_RESP_TIMEOUT;
                break;
            }
        }
    }
#endif
    return retVal;
}

/** @fn int MMWL_sensorStop(unsigned char deviceMap)
*
*   @brief API to Stop sensor.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Stop Sensor.
*/
int MMWL_sensorStop(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt =0;
    retVal = rlSensorStop(deviceMap);
#ifndef ENABLE_TEST_SOURCE
    if (retVal == RL_RET_CODE_OK)
    {
        if ((rfDevCfg.aeControl & 0x2) == 0x0)
        {
            while (mmwl_bSensorStarted == 1U)
            {
                osiSleep(1); /*Sleep 1 msec*/
                timeOutCnt++;
                if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
                {
                    retVal = RL_RET_CODE_RESP_TIMEOUT;
                    break;
                }
            }
        }
    }
#endif
    return retVal;
}

/** @fn int MMWL_setContMode(unsigned char deviceMap)
*
*   @brief API to set continuous mode.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to set continuous mode.
*/
int MMWL_setContMode(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlContModeCfg_t contModeCfgArgs = { 0 };
    contModeCfgArgs.digOutSampleRate = profileCfgArgs[0].digOutSampleRate;
    contModeCfgArgs.hpfCornerFreq1 = profileCfgArgs[0].hpfCornerFreq1;
    contModeCfgArgs.hpfCornerFreq2 = profileCfgArgs[0].hpfCornerFreq2;
    contModeCfgArgs.startFreqConst = profileCfgArgs[0].startFreqConst;
    contModeCfgArgs.txOutPowerBackoffCode = profileCfgArgs[0].txOutPowerBackoffCode;
    contModeCfgArgs.txPhaseShifter = profileCfgArgs[0].txPhaseShifter;

    /*read contModeCfgArgs from config file*/
    MMWL_readContModeConfig(&contModeCfgArgs);

    printf("Calling setContMode with\n digOutSampleRate[%d]\nstartFreqConst[%d]\ntxOutPowerBackoffCode[%d]\nRXGain[%d]\n\n", \
        contModeCfgArgs.digOutSampleRate, contModeCfgArgs.startFreqConst, contModeCfgArgs.txOutPowerBackoffCode, \
        contModeCfgArgs.rxGain);
    retVal = rlSetContModeConfig(deviceMap, &contModeCfgArgs);
    return retVal;
}

/** @fn int MMWL_dynChirpEnable(unsigned char deviceMap)
*
*   @brief API to enable Dynamic chirp feature.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to enable Dynamic chirp feature.
*/
int MMWL_dynChirpEnable(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDynChirpEnCfg_t dynChirpEnCfgArgs = { 0 };

    retVal = rlSetDynChirpEn(deviceMap, &dynChirpEnCfgArgs);
    return retVal;
}

/** @fn int MMWL_dynChirpConfig(unsigned char deviceMap)
*
*   @brief API to config chirp dynamically.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to config chirp dynamically.
*/
int  MMWL_setDynChirpConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    unsigned int cnt;
    rlDynChirpCfg_t * dataDynChirp[3U] = { &dynChirpCfgArgs[0], &dynChirpCfgArgs[1], &dynChirpCfgArgs[2]};

    dynChirpCfgArgs[0].programMode = 0;

    /* Configure NR1 for 48 chirps */
    dynChirpCfgArgs[0].chirpRowSelect = 0x10;
    dynChirpCfgArgs[0].chirpSegSel = 0;
    /* Copy this dynamic chirp config to other config and update chirp segment number */
    memcpy(&dynChirpCfgArgs[1], &dynChirpCfgArgs[0], sizeof(rlDynChirpCfg_t));
    memcpy(&dynChirpCfgArgs[2], &dynChirpCfgArgs[0], sizeof(rlDynChirpCfg_t));
    /* Configure NR2 for 48 chirps */
    dynChirpCfgArgs[1].chirpRowSelect = 0x20;
    dynChirpCfgArgs[1].chirpSegSel = 1;
    /* Configure NR3 for 48 chirps */
    dynChirpCfgArgs[2].chirpRowSelect = 0x30;
    dynChirpCfgArgs[2].chirpSegSel = 2;

    for (cnt = 0; cnt < 16; cnt++)
    {
        /* Reconfiguring frequency slope for 48 chirps */
        dynChirpCfgArgs[0].chirpRow[cnt].chirpNR1 |= (((3*cnt) & 0x3FU) << 8);
        dynChirpCfgArgs[0].chirpRow[cnt].chirpNR2 |= (((3*cnt + 1) & 0x3FU) << 8);
        dynChirpCfgArgs[0].chirpRow[cnt].chirpNR3 |= (((3*cnt + 2) & 0x3FU) << 8);
        /* Reconfiguring start frequency for 48 chirps */
        dynChirpCfgArgs[1].chirpRow[cnt].chirpNR1 |= 3*cnt;
        dynChirpCfgArgs[1].chirpRow[cnt].chirpNR2 |= 3*cnt + 1;
        dynChirpCfgArgs[1].chirpRow[cnt].chirpNR3 |= 3*cnt + 2;
        /* Reconfiguring ideal time for 48 chirps */
        dynChirpCfgArgs[2].chirpRow[cnt].chirpNR1 |= 3 * cnt;
        dynChirpCfgArgs[2].chirpRow[cnt].chirpNR2 |= 3 * cnt + 1;
        dynChirpCfgArgs[2].chirpRow[cnt].chirpNR3 |= 3 * cnt + 2;
    }

    printf("Calling DynChirpCfg with chirpSegSel[%d]\nchirpNR1[%d]\n\n", \
        dynChirpCfgArgs[0].chirpSegSel, dynChirpCfgArgs[0].chirpRow[0].chirpNR1);
    retVal = rlSetDynChirpCfg(deviceMap, 2U, &dataDynChirp[0]);
    return retVal;
}

/** @fn int MMWL_powerOff(unsigned char deviceMap)
*
*   @brief API to poweroff device.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to poweroff device.
*/
int MMWL_powerOff(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    retVal = rlDevicePowerOff();
    mmwl_bInitComp = 0;
    mmwl_bStartComp = 0U;
    mmwl_bRfInitComp = 0U;
    mmwl_devHdl = NULL;

    return retVal;
}

/** @fn int MMWL_lowPowerConfig(deviceMap)
*
*   @brief LowPower configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   LowPower configuration API.
*/
int MMWL_lowPowerConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    /* TBD - Read GUI Values */
    rlLowPowerModeCfg_t rfLpModeCfgArgs = { 0 };

    /*read rfLpModeCfgArgs from config file*/
    MMWL_readLowPowerConfig(&rfLpModeCfgArgs);

    retVal = rlSetLowPowerModeConfig(deviceMap, &rfLpModeCfgArgs);
    return retVal;
}


/** @fn int MMWL_App()
*
*   @brief mmWaveLink Example Application.
*
*   @return int Success - 0, Failure - Error Code
*
*   mmWaveLink Example Application.
*/
int MMWL_App()
{
    int retVal = RL_RET_CODE_OK;
    unsigned char deviceMap = RL_DEVICE_MAP_CASCADED_1;

    retVal = MMWL_openConfigFile();
    if (retVal != RL_RET_CODE_OK)
    {
        printf("failed to Open configuration file\n");
        return -1;
    }
	else
	{
		printf(">>>>success to Open configuration file\n");
	}

    /*  \subsection     api_sequence1     Seq 1 - Call Power ON API
    The mmWaveLink driver initializes the internal components, creates Mutex/Semaphore,
    initializes buffers, register interrupts, bring mmWave front end out of reset.
    */
    retVal = MMWL_powerOnMaster(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("mmWave Device Power on failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf(">>>>mmWave Device Power on success for deviceMap %u \n\n",
                deviceMap);
    }

    /*  \subsection     api_sequence2     Seq 2 - Download FIrmware/patch (Optional)
    The mmWave device firmware is ROMed and also can be stored in External Flash. This
    step is necessary if firmware needs to be patched and patch is not stored in serial
    Flash
    */
#if (FIRMWARE_DOWNLOAD)
    printf("==========================Firmware Download==========================\n");
    retVal = MMWL_firmwareDownload(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Firmware update failed for deviceMap %u with error %d \n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Firmware update successful for deviceMap %u \n",
                deviceMap);
    }
    printf("=====================================================================\n");
#endif
    /* Change CRC Type of Async Event generated by MSS to what is being requested by user in mmwaveconfig.txt */
    /*retVal = MMWL_setDeviceCrcType(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("CRC Type set for MasterSS failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("CRC Type set for MasterSS success for deviceMap %u \n\n", deviceMap);
    }*/

    /*  \subsection     api_sequence3     Seq 3 - Enable the mmWave Front end (Radar/RF subsystem)
    The mmWave Front end once enabled runs boot time routines and upon completion sends asynchronous event
    to notify the result
    */
    retVal = MMWL_rfEnable(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Radar/RF subsystem Power up failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf(">>>>Radar/RF subsystem Power up successful for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence4     Seq 4 - Basic/Static Configuration
    The mmWave Front end needs to be configured for mmWave Radar operations. basic
    configuration includes Rx/Tx channel configuration, ADC configuration etc
    */
    printf("======================Basic/Static Configuration======================\n");
    retVal = MMWL_basicConfiguration(deviceMap, 0);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Basic/Static configuration failed for deviceMap %u with error %d \n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf(">>>>Basic/Static configuration success for deviceMap %u \n\n",
                deviceMap);
    }

    /*  \subsection     api_sequence5     Seq 5 - Initializes the mmWave Front end
    The mmWave Front end once configured needs to be initialized. During initialization
    mmWave Front end performs calibration and once calibration is complete, it
    notifies the application using asynchronous event
    */
	printf("==========================RF Initilization============================\n");
    retVal = MMWL_rfInit(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("RF Initialization/Calibration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf(">>>>RF Initialization/Calibration successful for deviceMap %u \n\n", deviceMap);
    }

    /* mmwave sensor provides a feature to do the factory calibration, where application needs
       to send already stored calibration data to mmwave device. In this flow device will use 
       provided calibration data and skip doing actual boot-time calibration. With this option
       application can save the calibration time and the current (which consumes during default
       boot-time calibraiton).
       In this application, user needs to set this flag to run this test. This code flow is provided 
       as a reference which Host can implement.
       */
    if (gLinkCalibStoreRestoreTest == TRUE)
    {
        /* Host needs to invoke this API to get the calibration data from the device while device
           has done all the boot-time calibration. This function it needs to call to store the 
           calibration data to sFlash or in the Host application for later use.
           @Note: Make sure that before calling this API, rlRfInit API is being called with 
           all the calibrations ON */
        retVal = rlRfCalibDataStore(deviceMap, &calibData);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Calibration data store failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
            return -1;
        }
        /* Power of the device, if it's already Up. */
        retVal = MMWL_powerOff(deviceMap);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Device power off failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
            return -1;
        }
        /* This function is the reference where application needs to use lastly stored calibration 
           data and provide to the mmwave device so that it can skip the bootup time calibration 
           and rely on given calibration data. */
        retVal = MMWL_calibStoreRestore(deviceMap);
        printf("====================Calibration Data Store & Restore===================\n");
        if (retVal != RL_RET_CODE_OK)
        {
            printf("calibration data restore test for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
            return -1;
        }
        else
        {
            printf("calibration data restore test success for deviceMap %u \n\n", deviceMap);
        }
        printf("======================================================================\n\n");
    }

    /*  \subsection     api_sequence6     Seq 6 - FMCW profile configuration
    TI mmWave devices supports Frequency Modulated Continuous Wave(FMCW) Radar. User
    Need to define characteristics of FMCW signal using profile configuration. A profile
    contains information about FMCW signal such as Start Frequency (76 - 81 GHz), Ramp
    Slope (e.g 30MHz/uS). Idle Time etc. It also configures ADC samples, Sampling rate,
    Receiver gain, Filter configuration parameters

    \ Note - User can define upto 4 different profiles
    */
    printf("======================FMCW Configuration======================\n");
    retVal = MMWL_profileConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Profile Configuration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Profile Configuration success for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence7     Seq 7 - FMCW chirp configuration
    A chirp is always associated with FMCW profile from which it inherits coarse information
    about FMCW signal. Using chirp configuration user can further define fine
    variations to coarse parameters such as Start Frequency variation(0 - ~400 MHz), Ramp
    Slope variation (0 - ~3 MHz/uS), Idle Time variation etc. It also configures which transmit channels to be used
    for transmitting FMCW signal.

    \ Note - User can define upto 512 unique chirps
    */
    retVal = MMWL_chirpConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Chirp Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Chirp Configuration success for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence8     Seq 8 - Data Path (CSI2/LVDS) Configuration
    TI mmWave device supports CSI2 or LVDS interface for sending RAW ADC data. mmWave device
    can also send Chirp Profile and Chirp Quality data over LVDS/CSI2. User need to select
    the high speed interface and also what data it expects to receive.

    \ Note - This API is only applicable for AWR1243 when mmWaveLink driver is running on External Host
    */
    printf("==================Data Path(LVDS/CSI2) Configuration==================\n");
    retVal = MMWL_dataPathConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Data Path Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Data Path Configuration successful for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence9     Seq 9 - CSI2/LVDS CLock and Data Rate Configuration
    User need to configure what data rate is required to send the data on high speed interface. For
    e.g 150mbps, 300mbps etc.
    \ Note - This API is only applicable for AWR1243 when mmWaveLink driver is running on External Host
    */
    retVal = MMWL_hsiClockConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("CSI2/LVDS Clock Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("CSI2/LVDS Clock Configuration success for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence10     Seq 10 - CSI2/LVDS Lane Configuration
    User need to configure how many LVDS/CSI2 lane needs to be enabled
    \ Note - This API is only applicable for AWR1243 when mmWaveLink driver is running on External Host
    */
    retVal = MMWL_hsiLaneConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("CSI2/LVDS Lane Config failed for deviceMap %u with error %d \n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("CSI2/LVDS Lane Configuration success for deviceMap %u \n",
                deviceMap);
    }
    printf("======================================================================\n\n");

#ifdef ENABLE_TEST_SOURCE
    retVal = MMWL_testSourceConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Test Source Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Test source Configuration success for deviceMap %u \n\n", deviceMap);
    }
#endif

    /* Check for If Advance Frame Test is enabled */
    if(gLinkAdvanceFrameTest == FALSE)
    {
        /*  \subsection     api_sequence11     Seq 11 - FMCW frame configuration
        A frame defines sequence of chirps and how this sequence needs to be repeated over time.
        */
        retVal = MMWL_frameConfig(deviceMap);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Frame Configuration failed for deviceMap %u with error %d \n",
                deviceMap, retVal);
            return -1;
        }
        else
        {
            printf("Frame Configuration success for deviceMap %u \n", deviceMap);
        }
    }
    else
    {
        /*  \subsection     api_sequence11     Seq 11 - FMCW Advance frame configuration
        A frame defines sequence of chirps and how this sequence needs to be repeated over time.
        */
        retVal = MMWL_advFrameConfig(deviceMap);

        if (retVal != RL_RET_CODE_OK)
        {
            printf("Adv Frame Configuration failed for deviceMap %u with error %d \n",
                deviceMap, retVal);
            return -1;
        }
        else
        {
            printf("Adv Frame Configuration success for deviceMap %u \n", deviceMap);
        }
    }
    printf("======================================================================\n\n");

    if (gLinkContModeTest == TRUE)
    {
        retVal = MMWL_setContMode(deviceMap);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Continuous mode Config failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
            return -1;
        }
        else
        {
            printf("Continuous mode Config successful for deviceMap %u \n\n", deviceMap);
        }
    }
    /*  \subsection     api_sequence12     Seq 12 - Start mmWave Radar Sensor
    This will trigger the mmWave Front to start transmitting FMCW signal. Raw ADC samples
    would be received from Digital front end. For AWR1243, if high speed interface is
    configured, RAW ADC data would be transmitted over CSI2/LVDS. On xWR1443/xWR1642, it can
    be processed using HW accelerator or DSP
    */
    retVal = MMWL_sensorStart(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Sensor Start failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Sensor Start successful for deviceMap %u \n\n", deviceMap);
    }
    printf("======================================================================\n\n");

    if (gLinkDynProfileTest == TRUE)
    {
        /* Host can update profile configurations dynamically while frame is ongoing.
           This test has been added in this example to demostrate dynamic profile update feature
           of mmWave sensor device, developer must check the validity of parameters at the system
           level before implementing the application. */

        /* wait for few frames worth of time before updating profile config */
        osiSleep(3*framePeriodicity);

        /* update few of existing profile parameters */
        profileCfgArgs[0].rxGain = 222;
        profileCfgArgs[0].pfCalLutUpdate = 0x1; /* bit0: 1, bit1: 0 */
        profileCfgArgs[0].hpfCornerFreq1 = 1;
        profileCfgArgs[0].hpfCornerFreq2 = 1;
        profileCfgArgs[0].txStartTime = 2;
        profileCfgArgs[0].rampEndTime = 7000;

        /* Dynamically configure 1 profile (max 4 profiles) while frame is ongoing */
        retVal = rlSetProfileConfig(deviceMap, 1U, &profileCfgArgs[0U]);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Dynamic Profile Configuration failed for deviceMap %u with error code %d \n\n",
                    deviceMap, retVal);
            return -1;
        }
        else
        {
            printf("Dynamic Profile Configuration success for deviceMap %u \n\n", deviceMap);
        }

        /* wait for few frames worth of time before reading profile config.
           Dynamic profile configuration will come in effect during next frame, so wait for that time
           before reading back profile config */
        osiSleep(2*framePeriodicity);

        /* To verify that profile configuration parameters are applied to device while frame is ongoing,
           read back profile configurationn from device */
        retVal = rlGetProfileConfig(deviceMap, 0, &profileCfgArgs[1]);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Dynamic Get Profile Configuration failed for deviceMap %u with error code %d \n\n",
                    deviceMap, retVal);
            return -1;
        }
        else
        {
            printf("Dynamic Get Profile Configuration success for deviceMap %u \n\n", deviceMap);
            /* compare the read back profile configuration parameters to lastly configured parameters */
            if ((profileCfgArgs[0].rxGain != profileCfgArgs[1].rxGain) || \
                (profileCfgArgs[0].hpfCornerFreq1 != profileCfgArgs[1].hpfCornerFreq1) || \
                (profileCfgArgs[0].hpfCornerFreq2 != profileCfgArgs[1].hpfCornerFreq2) || \
                (profileCfgArgs[0].txStartTime != profileCfgArgs[1].txStartTime) || \
                (profileCfgArgs[0].rampEndTime != profileCfgArgs[1].rampEndTime))
                printf("Dynamic Profile Config mismatched !!! \n\n");
            else
                printf("Dynamic profile cfg matched \n\n");
        }
    }

    if (gLinkDynChirpTest == TRUE)
    {
        /* wait for few frames to elapse before invoking Dynamic chirp config API to update
           new chirp config to come in effect for next frames */

        /* wait for few frames worth of time */
        osiSleep(3*framePeriodicity);

        retVal = MMWL_setDynChirpConfig(deviceMap);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Dynamic Chirp config failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
            return -1;
        }
        else
        {
            printf("Dynamic Chirp config successful for deviceMap %u \n\n", deviceMap);
        }
        printf("======================================================================\n\n");

        retVal = MMWL_dynChirpEnable(deviceMap);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Dynamic Chirp Enable failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
            return -1;
        }
        else
        {
            printf("Dynamic Chirp Enable successful for deviceMap %u \n\n", deviceMap);
        }
        printf("======================================================================\n\n");

        /* wait for another few mSec so that dynamic chirp come in effect,
         If above API reached to BSS at the end of frame then new chirp config will come in effect
         during next frame only */
        osiSleep(2*framePeriodicity);

        /* read back Chirp config, which should be same as configured in dynChirpConfig for same segment */
        retVal = MMWL_getDynChirpConfig(deviceMap);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("GetChirp Configuration failed for deviceMap %u with error %d \n\n",
                    deviceMap, retVal);
            return -1;
        }
        else
        {
            printf("GetChirp Configuration success for deviceMap %u \n\n", deviceMap);
        }
    }

    /* @Note - all these SLeep is added in this demo application to demonstrate mmWave sensor features,
                user can change these sleep time values as per their requirement */

    /* wait for 10 frames worth of time */
    osiSleep(10*framePeriodicity);

    /* Stop the frame */
    retVal = MMWL_sensorStop(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        if (retVal == RL_RET_CODE_FRAME_ALREADY_ENDED)
        {
            printf("Frame is already stopped when sensorStop CMD was issued\n");
        }
        else
        {
            printf("Sensor Stop failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
            return -1;
        }
    }
    else
    {
        printf("Sensor Stop successful for deviceMap %u \n\n", deviceMap);
    }

    /* Note- Before Calling this API user must feed in input signal to device's pins,
    else device will return garbage data in GPAdc measurement over Async event.
    Measurement data is stored in 'rcvGpAdcData' structure after this API call. */
    retVal = MMWL_gpadcMeasConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("GPAdc measurement API failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("GPAdc measurement API success for deviceMap %u \n\n", deviceMap);
    }

    /* Switch off the device */
    retVal = MMWL_powerOff(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device power off failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device power off success for deviceMap %u \n\n", deviceMap);
    }

    /* Close Configuraiton file */
    MMWL_closeConfigFile();

    return 0;
}

/** @fn int main()
*
*   @brief Main function.
*
*   @return none
*
*   Main function.
*/
void main(void)
{
    int retVal;

    printf("=========== mmWaveLink Example Application =========== \n");
    retVal = MMWL_App();
    if(retVal == RL_RET_CODE_OK)
    {
        printf("=========== mmWaveLink Example Application execution Successful=========== \n");
    }
    else
    {
        printf("=========== mmWaveLink Example Application execution Failed =========== \n");
    }

    /* Wait for Enter click */
    getchar();
    printf("=========== mmWaveLink Example Application: Exit =========== \n");
}
