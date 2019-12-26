/****************************************************************************************
* FileName     : mmw_monitoring.c
*
* Description  : This file implements mmwavelink Advanced frame and monitoring feature of mmwave sensor.
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
#include "mmw_monitoring.h"
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

#define MMWL_MSS_FIRMWARE_SIZE  (sizeof(mssImg))
#define MMWL_CONFIG_FILE_SIZE (sizeof(configImg))
#define MMWL_RADARSS_FIRMWARE_SIZE   (sizeof(bssImg))
#define MMWL_META_IMG_FILE_SIZE (sizeof(metaImage))

/* Async Event Timeouts */
#define MMWL_API_INIT_TIMEOUT                (2000) /* 2 Sec*/
#define MMWL_API_START_TIMEOUT               (1000) /* 1 Sec*/
#define MMWL_API_RF_INIT_TIMEOUT             (1000) /* 1 Sec*/

/* To Enable File download */
#define FIRMWARE_DOWNLOAD                      1

/* Time unit for calibration/monitoring, it's based on Frame count */
#define CAL_MON_TIME_UNIT                      1

/* To enable TX2 */
#define ENABLE_TX2                             1

#define FALSE    0
#define TRUE     1

/******************************************************************************
* GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
******************************************************************************
*/
typedef int (*RL_P_OS_SPAWN_FUNC_PTR)(RL_P_OSI_SPAWN_ENTRY pEntry, const void* pValue, unsigned int flags);
typedef int (*RL_P_OS_DELAY_FUNC_PTR)(unsigned int delay);

/* Global Variable for Device Status */
static unsigned int mmwl_bInitComp = 0U;
static unsigned int mmwl_bStartComp = 0U;
static unsigned int mmwl_bRfInitComp = 0U;
static unsigned int mmwl_bSensorStarted = 0U;
static unsigned int mmwl_bRunTimeCalib = 0U;

unsigned char gAwr1243CrcType = RL_CRC_TYPE_32BIT;

/* Enable Advanced frame config test in the application
TRUE -> config AdvFrame
FALSE -> config Legacy Frame */
unsigned char gLinkAdvanceFrameTest = FALSE;

/* TRUE -> Enable LDO bypass. Support only on AWR1243 Rev B EVMs
   FALSE -> Disable LDO bypass.
   CAUTION : Don't enable LDO bypass for un supported device. It may damage EVM. */
unsigned char gLinkBypassLdo = FALSE;

/* monitoring report header count */
unsigned int gMonReportHdrCnt = 0U;
/* Frame count configured to AWR1243 device */
unsigned short gFrameCount = 0;

/* Strcture to store async event config */
rlRfDevCfg_t rfDevCfg = { 0x0 };

/* SPI Communication handle to AWR1243 device*/
rlComIfHdl_t mmwl_devHdl = NULL;

uint64_t computeCRC(uint8_t *p, uint32_t len, uint8_t width);

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
rlUInt32_t asyncEvntCnt0[32];
rlUInt32_t asyncEvntCnt1[32];
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
            asyncEvntCnt0[asyncSB]++;
            switch (asyncSB)
            {
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    mmwl_bRfInitComp = 1U;
                    printf("Async event: RF-init calibration status status: [0x%x] update: [0x%x]\n", \
                          ((rlRfInitComplete_t*)payload)->calibStatus, ((rlRfInitComplete_t*)payload)->calibUpdate);
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
                    mmwl_bRunTimeCalib = 1U;
                    printf("Aync event: Run time Calibration Report [0x%x]\n", ((rlRfRunTimeCalibReport_t*)payload)->calibErrorFlag);
                }
                break;
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    printf("Aync event: Monitoring Timing Failed Report [%d] \n", ((rlCalMonTimingErrorReportData_t*)payload)->timingFailCode);
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
                case RL_RF_AE_MON_TEMPERATURE_REPORT_SB:
                {
                    printf("Aync event: Temperature monitor Report [0x%x] \n", ((rlMonTempReportData_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_RX_GAIN_PHASE_REPORT:
                {
                    printf("Aync event: Rx Gain Phase Report [0x%x] \n", ((rlMonRxGainPhRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_RX_NOISE_FIG_REPORT:
                {
                    printf("Aync event: Rx Noise Fig Report [0x%x] \n", ((rlMonRxNoiseFigRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_RX_IF_STAGE_REPORT:
                {
                    printf("Aync event: Rx IF Stage Report [%ld] \n", ((rlMonRxIfStageRep_t*)payload)->hpfCutOffFreqEr);
                    break;
                }
                case RL_RF_AE_MON_TX0_POWER_REPORT:
                {
                    printf("Aync event: TX0 Power Report [0x%x] \n", ((rlMonTxPowRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_TX1_POWER_REPORT:
                {
                    printf("Aync event: TX1 Power Report [0x%x] \n", ((rlMonTxPowRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_TX2_POWER_REPORT:
                {
                    printf("Aync event: TX2 Power Report [0x%x] \n", ((rlMonTxPowRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_TX0_BALLBREAK_REPORT:
                {
                    printf("Aync event: TX0 Ballbreak Report [0x%x] \n", ((rlMonTxBallBreakRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_TX1_BALLBREAK_REPORT:
                {
                    printf("Aync event: TX1 Ballbreak Report [0x%x] \n", ((rlMonTxBallBreakRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_REPORT_HEADER_SB:
                {
                    printf("Aync event: Monitor Report Header FTTI count [%d] \n", ((rlMonReportHdrData_t*)payload)->fttiCount);
                    gMonReportHdrCnt++;
                    break;
                }
                default:
                {
                    printf("Unhandled RadarSS Aync Event msgId: 0x%x, asyncSB:0x%x  \n", msgId, asyncSB);
                    break;
                }
            }
        }
        break;
        /* async event from radarSS which sub-block IDs comes under 0x81 MsgID */
        case RL_RF_ASYNC_EVENT_1_MSG:
        {
            asyncEvntCnt1[asyncSB]++;
            switch (asyncSB)
            {
                case RL_RF_AE_MON_TX2_BALLBREAK_REPORT:
                {
                    printf("Aync event: TX2 Ballbreak Report [0x%x] \n", ((rlMonTxBallBreakRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_TX_GAIN_MISMATCH_REPORT:
                {
                    printf("Aync event: TX Gain Mismatch Report [0x%x] \n", ((rlMonTxGainPhaMisRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_TX0_BPM_REPORT:
                {
                    printf("Aync event: TX1 BPM Report [0x%x] \n", ((rlMonTxBpmRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_TX1_BPM_REPORT:
                {
                    printf("Aync event: TX1 BPM Report [0x%x] \n", ((rlMonTxBpmRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_TX2_BPM_REPORT:
                {
                    printf("Aync event: TX2 BPM Report [0x%x] \n", ((rlMonTxBpmRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_SYNTHESIZER_FREQ_REPORT:
                {
                    printf("Aync event: Synthesizeer freq Report [0x%x] \n", ((rlMonSynthFreqRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_EXT_ANALOG_SIG_REPORT:
                {
                    printf("Aync event: External Analog Signal Report [0x%x] \n", ((rlMonExtAnaSigRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_TX0_INT_ANA_SIG_REPORT:
                {
                    printf("Aync event: TX1 Internal Analog Signal Report [0x%x] \n", ((rlMonTxIntAnaSigRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_TX1_INT_ANA_SIG_REPORT:
                {
                    printf("Aync event: TX1 Internal Analog Signal Report [0x%x] \n", ((rlMonTxIntAnaSigRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_TX2_INT_ANA_SIG_REPORT:
                {
                    printf("Aync event: TX2 Internal Analog Signal Report [0x%x] \n", ((rlMonTxIntAnaSigRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_RX_INT_ANALOG_SIG_REPORT:
                {
                    printf("Aync event: RX Internal Analog Signal Report [0x%x] \n", ((rlMonRxIntAnaSigRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_PMCLKLO_INT_ANA_SIG_REPORT:
                {
                    printf("Aync event: PMCLKLO Internal Analog Signal Report [0x%x] \n", ((rlMonPmclkloIntAnaSigRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_GPADC_INT_ANA_SIG_REPORT:
                {
                    printf("Aync event: GPADC Internal Analog Signal Report [0x%x] \n", ((rlMonGpadcIntAnaSigRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_PLL_CONTROL_VOLT_REPORT:
                {
                    printf("Aync event: PLL Control Volt Report [0x%x] \n", ((rlMonPllConVoltRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_DCC_CLK_FREQ_REPORT:
                {
                    printf("Aync event: DCC CLK Freq Report [0x%x] \n", ((rlMonDccClkFreqRep_t*)payload)->statusFlags);
                    break;
                }
                case RL_RF_AE_MON_RX_MIXER_IN_PWR_REPORT:
                {
                    printf("Aync event: RX Mixer Input Power Report [0x%x] \n", ((rlMonRxMixrInPwrRep_t*)payload)->statusFlags);
                    break;
                }
                default:
                {
                    printf("Unhandled Aync Event msgId: 0x%x, asyncSB:0x%x  \n", msgId, asyncSB);
                    break;
                }
            }
            break;
        }
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

    if (pmmwl_imgBuffer == NULL)
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

    if (mmwl_iRemChunks > 0)
    {
        printf("Download in Progress: %d%%..", usProgress);
    }
    /*Remaining Chunk*/
    while (mmwl_iRemChunks > 0)
    {
        if ((((iNumChunks - mmwl_iRemChunks) * 100) / iNumChunks - usProgress) > 10)
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

    if (gLinkBypassLdo == TRUE)
    {
        rfLdoBypassCfgArgs.ldoBypassEnable = 1;
    }

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
    rfChanCfgArgs.txChannelEn |= (1<<2); // Enable TX2
#endif

    printf("Calling rlSetChannelConfig With [%d]Rx and [%d]Tx Channel Enabled \n",
           rfChanCfgArgs.rxChannelEn, rfChanCfgArgs.txChannelEn);

    retVal = rlSetChannelConfig(deviceMap, &rfChanCfgArgs);
    return retVal;
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
        printf("Channel Configuration success for deviceMap %u\n\n", deviceMap);
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
        printf("AdcOut Configuration success for deviceMap %u\n\n", deviceMap);
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
        printf("LDO Bypass Configuration success for deviceMap %u\n\n", deviceMap);
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
        printf("Data format Configuration success for deviceMap %u\n", deviceMap);
    }

    /* Data path configuration */
    retVal = MMWL_dataPathConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Data path Configuration failed for deviceMap %u with error code %d\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Data path Configuration success for deviceMap %u\n", deviceMap);
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
        printf("Low Power Configuration success for deviceMap %u \n", deviceMap);
    }

    /* HSI (LVDS/CSI-2) clock configuration
       - This API is only applicable for AWR1243 when mmWaveLink driver is running on External Host */
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

    /* Async event direction and control configuration */
    retVal = MMWL_setAsyncEventDir(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("AsyncEvent Configuration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("AsyncEvent Configuration success for deviceMap %u \n\n", deviceMap);
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
    rlProfileCfg_t profileCfgArgs = { 0 };
    double endFreqConst;

    /*read profileCfgArgs from config file*/
    MMWL_readProfileConfig(&profileCfgArgs);

    endFreqConst = (double)((profileCfgArgs.startFreqConst * 53.6441803 + (profileCfgArgs.rampEndTime * 10) * \
                   (profileCfgArgs.freqSlopeConst * 48.2797623))) / 53.6441803;
    if ((profileCfgArgs.startFreqConst >= 1435388860U) && ((unsigned int)endFreqConst <= 1509954515U))
    {
        /* If start frequency is in between 77GHz to 81GHz use VCO2 */
        profileCfgArgs.pfVcoSelect = 0x02;
    }
    else
    {
        /* If start frequency is in between 76GHz to 78GHz use VCO1 */
        profileCfgArgs.pfVcoSelect = 0x00;
    }

    printf("Calling rlSetProfileConfig with \nProfileId[%d]\nStart Frequency[%f] GHz\nRamp Slope[%f] MHz/uS \n",
        profileCfgArgs.profileId, (float)((profileCfgArgs.startFreqConst * 53.6441803)/(1000*1000*1000)),
        (float)(profileCfgArgs.freqSlopeConst * 48.2797623)/1000.0);
    retVal = rlSetProfileConfig(deviceMap, 1U, &profileCfgArgs);
    return retVal;
}

/** @fn int MMMWL_setCalMonConfig(unsigned char deviceMap)
*
*   @brief Calibration monitoring time unit and freaquency limit configuration
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Calibration monitoring time unit and freaquency limit configuration API.
*/
int32_t MMMWL_setCalMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlRfCalMonFreqLimitConf_t data2 = { 0 };
    rlRfCalMonTimeUntConf_t data = { 0 };

    /* set Frequency max and min limit */
    data2.freqLimitLow = 760U;
    data2.freqLimitHigh = 810U;

    /* set monitoring Unit etc. */
    data.calibMonTimeUnit = CAL_MON_TIME_UNIT;
    data.devId = 1U;
    data.numOfCascadeDev = 1U;
    data.reserved = 0U;

    retVal = rlRfSetCalMonTimeUnitConfig(RL_DEVICE_MAP_CASCADED_1,&data);
    if (retVal != 0)
    {
        printf("Error: Unable to rlRfSetCalMonTimeUnitConfig [Error %d]\n", retVal);
        return -1;
    }

    retVal = rlRfSetCalMonFreqLimitConfig(RL_DEVICE_MAP_CASCADED_1,
        (rlRfCalMonFreqLimitConf_t*)&data2);

    if (retVal != 0)
    {
        printf("Error: Unable to rlRfSetCalMonFreqLimitConfig[Error %d]\n", retVal);
        return -1;
    }

    return 0;
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
    int retVal = RL_RET_CODE_OK;
    rlChirpCfg_t chirpCfgArgs = {0};

    /*read chirpCfgArgs from config file*/
    MMWL_readChirpConfig(&chirpCfgArgs);

    printf("Calling rlSetChirpConfig with \nProfileId[%d]\nStart Idx[%d]\nEnd Idx[%d] \n",
        chirpCfgArgs.profileId, chirpCfgArgs.chirpStartIdx,
        chirpCfgArgs.chirpEndIdx);
    retVal = rlSetChirpConfig(deviceMap, 1U, &chirpCfgArgs);
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

    gFrameCount = frameCfgArgs.numFrames;

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
    int retVal = RL_RET_CODE_OK;
    rlAdvFrameCfg_t AdvframeCfgArgs = { 0 };
    rlAdvFrameCfg_t GetAdvFrameCfgArgs = { 0 };
    /*read frameCfgArgs from config file*/
    MMWL_readAdvFrameConfig(&AdvframeCfgArgs);
    gFrameCount = AdvframeCfgArgs.frameSeq.numFrames;

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

/** @fn int MMWL_rlRfAnaMonConfig(unsigned char deviceMap)
*
*   @brief consolidated configuration of all ana mon.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Sets the consolidated configuration of all analog monitoring..
*/
int MMWL_rlRfAnaMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlMonAnaEnables_t data =  {
                                (1 << 0) |      /* TEMPERATURE_MONITOR_EN */               \
                                (1 << 1) |      /* RX_GAIN_PHASE_MONITOR_EN */             \
                                (1 << 2) |      /* RX_NOISE_MONITOR_EN */                  \
                                (1 << 3) |      /* RX_IFSTAGE_MONITOR_EN */                \
                                (1 << 4) |      /* TX0_POWER_MONITOR_EN */                 \
                                (1 << 5) |      /* TX1_POWER_MONITOR_EN */                 \
                                (1 << 7) |      /* TX0_BALLBREAK_MONITOR_EN */             \
                                (1 << 8) |      /* TX1_BALLBREAK_MONITOR_EN */             \
                                (1 << 10) |      /* TX_GAIN_PHASE_MONITOR_EN */            \
                                (1 << 11) |      /* TX0_BPM_MONITOR_EN */                  \
                                (1 << 12) |      /* TX1_BPM_MONITOR_EN */                  \
                                (1 << 14) |      /* SYNTH_FREQ_MONITOR_EN */               \
                                (1 << 15) |      /* EXTERNAL_ANALOG_SIGNALS_MONITOR_EN */  \
                                (1 << 16) |      /* INTERNAL_TX0_SIGNALS_MONITOR_EN */     \
                                (1 << 17) |      /* INTERNAL_TX1_SIGNALS_MONITOR_EN */     \
                                (1 << 19) |      /* INTERNAL_RX_SIGNALS_MONITOR_EN */      \
                                (1 << 20) |      /* INTERNAL_PMCLKLO_SIGNALS_MONITOR_EN */ \
                                (1 << 21) |      /* INTERNAL_GPADC_SIGNALS_MONITOR_EN */   \
                                (1 << 22) |      /* PLL_CONTROL_VOLTAGE_MONITOR_EN */      \
                                (1 << 23) |      /* DCC_CLOCK_FREQ_MONITOR_EN  */          \
                                (1 << 24) |      /* RX_IF_SATURATION_MONITOR_EN */         \
                                (1 << 25) |      /* RX_SIG_IMG_BAND_MONITORING_EN */       \
                                (1 << 26) |      /* RX_MIXER_INPUT_POWER_MONITOR */        \
                                 0x0 };

#if (ENABLE_TX2)
    data.enMask |=              (1 << 6) |      /* TX2_POWER_MONITOR_EN */                 \
                                (1 << 9) |      /* TX2_BALLBREAK_MONITOR_EN */             \
                                (1 << 13) |     /* TX2_BPM_MONITOR_EN */                   \
                                (1 << 18)       /* INTERNAL_TX2_SIGNALS_MONITOR_EN */;

#endif
    /* Analog monitoring configuratoin configuration */
    retVal = rlRfAnaMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);

    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfAnaMonConfig retVal=%d\n", retVal);
        return -1;
    }

    printf("Debug: Finished analog monitoring configurations to BSS\n");

    return 0;
}

/** @fn int MMWL_setRfTempMonConfig(unsigned char deviceMap)
*
*   @brief Temperature monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Temperature monitoring configuration API.
*/
int MMWL_setRfTempMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;

    rlTempMonConf_t data = { 0x2, 0x0, -10, 60, -10, 60, 20, 0, 0 };

    /* Temperature monitoring configuration */
    retVal = rlRfTempMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);

    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfTempMonConfig retVal=%d\n", retVal);
        return -1;
    }

    printf("Debug: Finished temperature monitoring configurations to BSS\n");

    return 0;
}

/** @fn int MMWL_setRfRxGainPhaMonConfig(unsigned char deviceMap)
*
*   @brief RX Gain and Phase Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RX Gain and Phase Monitoring configuration API.
*/
int MMWL_setRfRxGainPhaMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    int32_t         i,j;
    rlRxGainPhaseMonConf_t data = { 0x0 };

    data.rfFreqBitMask = 0x7;
    data.txSel = 0x1;
    data.rxGainAbsThresh = 40U;
    data.rxGainMismatchErrThresh = 40U;
    data.rxGainFlatnessErrThresh = 40U;
    data.rxGainPhaseMismatchErrThresh = (30 * (1U << 16)) / 360U;
    for (i = 0; i <= 3; i++)
    {
        for (j = 0; j <= 2; j++)
        {
            data.rxGainMismatchOffsetVal[i][j] = 0x1A;
            data.rxGainPhaseMismatchOffsetVal[i][j] = 0x1A;
        }
    }

    /* RX Gain and Phase Monitoring configuration */
    retVal = rlRfRxGainPhMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);

    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfRxGainPhMonConfig retVal=%d\n", retVal);
        return -1;
    }

    printf("Debug: Finished RX gain phase monitoring configurations to BSS\n");

    return 0;
}

/** @fn int MMWL_setRfRxNoiseMonConfig(unsigned char deviceMap)
*
*   @brief RX Noise Monitoring  configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RX Noise Monitoring  configuration API.
*/
int32_t MMWL_setRfRxNoiseMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlRxNoiseMonConf_t data = { 0x0 };

    data.rfFreqBitMask = 0x7;
    data.reportMode = 0x2;
    data.noiseThresh = 250;

    /* RX Noise Monitoring configuration */
    retVal = rlRfRxNoiseMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);

    /* Check if there is more than one pending interrupt */

    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfRxNoiseMonConfig retVal=%d\n", retVal);
        return -1;
    }

    printf("Debug: Finished RX noise monitoring configurations to BSS\n");

    return 0;
}

/** @fn int MMWL_setRfRxIfStageMonConfig(unsigned char deviceMap)
*
*   @brief RX IF Stage Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RX IF Stage Monitoring configuration API.
*/
int32_t MMWL_setRfRxIfStageMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlRxIfStageMonConf_t data = { 0x0 };

    data.reportMode = 0x2;
    data.hpfCutoffErrThresh = 15;
    data.lpfCutoffErrThresh = 15;
    data.ifaGainErrThresh = 100;

    /* RX IF Stage Monitoring configuration */
    retVal = rlRfRxIfStageMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);

    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfRxIfStageMonConfig retVal=%d\n", retVal);
        return -1;
    }

    printf("Debug: Finished RX IF filter attenuation monitoring configurations to BSS\n");

    return 0;
}

/** @fn int MMWL_setRfTxPowMonConfig(unsigned char deviceMap)
*
*   @brief TX Power Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   TX Power Monitoring configuration API.
*/
int32_t MMWL_setRfTxPowMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlTxPowMonConf_t tx0PowrMonCfg = { 0, 7, 0, 2, 0, 30, 30, 0, 0 };
    rlTxPowMonConf_t tx1PowrMonCfg = { 0, 7, 0, 2, 0, 30, 30, 0, 0 };
#if (ENABLE_TX2)
    rlTxPowMonConf_t tx2PowrMonCfg = { 0, 7, 0, 2, 0, 30, 30, 0, 0 };
#else
    rlTxPowMonConf_t tx2PowrMonCfg = NULL;
#endif

    rlAllTxPowMonConf_t data = { &tx0PowrMonCfg, &tx1PowrMonCfg, &tx2PowrMonCfg };


    /* TX Power Monitoring configuration */
    retVal = rlRfTxPowrMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);

    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfTxPowrMonConfig retVal=%d\n", retVal);
        return -1;
    }

    printf("Debug: Finished information related to TX Power monitor configurations to BSS\n");
    return retVal;
}


/** @fn int MMWL_setRfTxBallbreakMonConfig(unsigned char deviceMap)
*
*   @brief TX Ballbreak Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   TX Ballbreak Monitoring configuration API.
*/
int32_t MMWL_setRfTxBallbreakMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlTxBallbreakMonConf_t tx0BallBrkMonCfg = { 2, 0, -90, 0, 0 };
    rlTxBallbreakMonConf_t tx1BallBrkMonCfg = { 2, 0, -90, 0, 0 };
#if (ENABLE_TX2)
    rlTxBallbreakMonConf_t tx2BallBrkMonCfg = { 2, 0, -90, 0, 0 };
#else
    rlTxBallbreakMonConf_t tx2BallBrkMonCfg = NULL;
#endif
    rlAllTxBallBreakMonCfg_t data = { &tx0BallBrkMonCfg, &tx1BallBrkMonCfg, &tx2BallBrkMonCfg };

    /* TX Ballbreak Monitoring configuration */
    retVal = rlRfTxBallbreakMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfTxPowrMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to TX power monitoring configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfTxGainPhaseMismatchMonConfig(unsigned char deviceMap)
*
*   @brief TX Gain Phase Mismatch Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   TX Gain Phase Mismatch Monitoring configuration API.
*/
int32_t MMWL_setRfTxGainPhaseMismatchMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlTxGainPhaseMismatchMonConf_t data = {0x0};

    data.profileIndx = 0U;
    data.txEn = 0x3U;
    data.rxEn = 0xFU;
    data.rfFreqBitMask = 0x7U;

    /* TX Gain Phase Mismatch Monitoring configuration */
    retVal = rlRfTxGainPhaseMismatchMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfTxGainPhaseMismatchMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to TX gain and phase mismatch monitoring configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfTxBpmMonConfig(unsigned char deviceMap)
*
*   @brief TX BPM Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   TX BPM Monitoring configuration API.
*/
int32_t MMWL_setRfTxBpmMonConfig(unsigned char deviceMap)
{
    rlTxBpmMonConf_t tx0BpmMonCfg = { 0x0, 0x04, 0x02, 0x02, 0x2, 0xF, 0x1555, 30, 0x0, 0x0 };
    rlTxBpmMonConf_t tx1BpmMonCfg = { 0x0, 0x04, 0x02, 0x02, 0x2, 0xF, 0x1555, 30, 0x0, 0x0 };
#if (ENABLE_TX2)
    rlTxBpmMonConf_t tx2BpmMonCfg = { 0x0, 0x04, 0x02, 0x02, 0x2, 0xF, 0x1555, 30, 0x0, 0x0 };
#else
    rlTxBpmMonConf_t tx2BpmMonCfg = NULL;
#endif
    rlAllTxBpmMonConf_t data = { &tx0BpmMonCfg, &tx1BpmMonCfg, &tx2BpmMonCfg };
    int32_t retVal;

    tx0BpmMonCfg.rxEn = 0xFU;
    tx1BpmMonCfg.rxEn = 0xFU;
#if (ENABLE_TX2)
    tx2BpmMonCfg.rxEn = 0xFU;
#endif

    /* TX BPM Monitoring configuration */
    retVal = rlRfTxBpmMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfTxBpmMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to TX BPM configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfTxIntAnaSignalMonConfig(unsigned char deviceMap)
*
*   @brief  TX Internal Analog Signals Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   TX Internal Analog Signals Monitoring configuration API.
*/
int32_t MMWL_setRfTxIntAnaSignalMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlTxIntAnaSignalsMonConf_t tx0IntAnaSgnlMonCfg = { 0x0, 0x2, 0x0, 0x0 };
    rlTxIntAnaSignalsMonConf_t tx1IntAnaSgnlMonCfg = { 0x0, 0x2, 0x0, 0x0 };
#if (ENABLE_TX2)
    rlTxIntAnaSignalsMonConf_t tx2IntAnaSgnlMonCfg = { 0x0, 0x2, 0x0, 0x0 };
#else
    rlTxIntAnaSignalsMonConf_t tx2IntAnaSgnlMonCfg = NULL;
#endif
    rlAllTxIntAnaSignalsMonConf_t data = { &tx0IntAnaSgnlMonCfg, &tx1IntAnaSgnlMonCfg, &tx2IntAnaSgnlMonCfg };

    /* TX Internal Analog Signals Monitoring configuration */
    retVal = rlRfTxIntAnaSignalsMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfTxIntAnaSignalsMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to TX Internal Analog Signals configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfRxIntAnaSignalMonConfig(unsigned char deviceMap)
*
*   @brief  RX Internal Analog Signals Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RX Internal Analog Signals Monitoring configuration API.
*/
int32_t MMWL_setRfRxIntAnaSignalMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlRxIntAnaSignalsMonConf_t data = { 0x0, 0x2, 0x0, 0x0 };

    /* RX Internal Analog Signals Monitoring configuration */
    retVal = rlRfRxIntAnaSignalsMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfRxIntAnaSignalsMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to RX Internal Analog Signals configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfPmClkLoIntAnaSignalsMonConfig(unsigned char deviceMap)
*
*   @brief PMClock configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   PM, CLK, LO Internal Analog Signals Monitoring configuration API.
*/
int32_t MMWL_setRfPmClkLoIntAnaSignalsMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlPmClkLoIntAnaSignalsMonConf_t data = { 0x0 };

    /* PM, CLK, LO Internal Analog Signals Monitoring configuration */
    retVal = rlRfPmClkLoIntAnaSignalsMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfPmClkLoIntAnaSignalsMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to Power Management, Clock generation and LO distributioncon configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfGpadcIntAnaSignalsMonConfig(unsigned char deviceMap)
*
*   @brief GPADC Internal Analog Signals Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   GPADC Internal Analog Signals Monitoring configuration API.
*/
int32_t MMWL_setRfGpadcIntAnaSignalsMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlGpadcIntAnaSignalsMonConf_t data = { 0x0 };

    data.reportMode = 0x2;

    /*  GPADC Internal Analog Signals Monitoring configuration */
    retVal = rlRfGpadcIntAnaSignalsMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfGpadcIntAnaSignalsMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to GPADC Internal Analog Signals monitoring configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfPllContrlVoltMonConfig(unsigned char deviceMap)
*
*   @brief APLL and Synthesizer’s control voltage signals monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   APLL and Synthesizer’s control voltage signals monitoring configuration API.
*/
int32_t MMWL_setRfPllContrlVoltMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlPllContrVoltMonConf_t data = { 0x0 };

    data.reportMode = 0x2;
    data.signalEnables = 0x7;

    /* APLL and Synthesizer’s control voltage signals monitoring configuration */
    retVal = rlRfPllContrlVoltMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfPllContrlVoltMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to APLL and Synthesizer’s control voltage signals monitoring configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfSynthFreqMonConfig(unsigned char deviceMap)
*
*   @brief Synth Freq Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Synth Freq Monitoring configuration API.
*/
int32_t MMWL_setRfSynthFreqMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlSynthFreqMonConf_t data = { 0x0 };

    data.reportMode = 0x2;
    data.freqErrThresh = 4000;
    data.monStartTime = 10;

    /* Synth Freq Monitoring configuration */
    retVal = rlRfSynthFreqMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfSynthFreqMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to synthesizer frequency configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfExtAnaSignalsMonConfig(unsigned char deviceMap)
*
*   @brief External Analog Signals Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   External Analog Signals Monitoring configuration API.
*/
int32_t MMWL_setRfExtAnaSignalsMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    int32_t         i;
    rlExtAnaSignalsMonConf_t data = { 0x0 };

    data.reportMode = 0x2;
    data.signalInpEnables = 0x3F;
    data.signalBuffEnables = 0x1F;
    for (i = 0; i <= 5; i++)
    {
        data.signalSettlingTime[i] = 10;
        data.signalThresh[i] = 0;
        data.signalThresh[i + 1] = 200;
    }

    /* External Analog Signals Monitoring configuration */
    retVal = rlRfExtAnaSignalsMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfExtAnaSignalsMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to external DC signals monitoring configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfDualClkCompMonConfig(unsigned char deviceMap)
*
*   @brief DCC based clock frequency monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   DCC based clock frequency monitoring configuration API.
*/
int32_t MMWL_setRfDualClkCompMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;

    rlDualClkCompMonConf_t data = { 0x2, 0x0, 63, 0x0 };

    /* DCC based clock frequency monitoring configuration */
    retVal = rlRfDualClkCompMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfExtAnaSignalsMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to external DC signals monitoring configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfRxIfSatMonConfig(unsigned char deviceMap)
*
*   @brief RX saturation detector monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RX saturation detector monitoring configuration API.
*/
int32_t MMWL_setRfRxIfSatMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;

    rlRxSatMonConf_t data = { 0x0 };
    data.satMonSel = 0x3;
    data.primarySliceDuration = 0x5;
    data.numSlices = 0xD;

    /* RX saturation detector monitoring configuration */
    retVal = rlRfRxIfSatMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: MMWL_setRfRxIfSatMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to RX saturation detector monitoring configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfRxSigImgMonConfig(unsigned char deviceMap)
*
*   @brief RX signal and image band energy monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RX signal and image band energy monitoring configuration API.
*/
int32_t MMWL_setRfRxSigImgMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;

    rlSigImgMonConf_t data = { 0x0 };
    data.numSlices = 0xD;
    data.timeSliceNumSamples = 0xC;

    /* RX signal image monitoring configuration */
    retVal = rlRfRxSigImgMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: MMWL_setRfRxSigImgMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to RX signal and image band energy monitoring configurations to BSS\n");
    return 0;
}

/** @fn int MMWL_setRfRxMixerInPwrConfig(unsigned char deviceMap)
*
*   @brief RX mixer input power monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RX mixer input power monitoring configuration API.
*/
int32_t MMWL_setRfRxMixerInPwrConfig(unsigned char deviceMap)
{
    int32_t         retVal;

    rlRxMixInPwrMonConf_t data = { 0x0 };
    data.reportMode = 0x2;
    data.txEnable = 0x3;
    data.thresholds = 0xFA00;

    /* RX mixer input power monitoring configuration */
    retVal = rlRfRxMixerInPwrConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfRxMixerInPwrConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished information related to RX mixer input power monitoring configurations to BSS\n");
    return 0;
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
#endif
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
    rlLowPowerModeCfg_t rfLpModeCfgArgs = { 0 };

    /*read rfLpModeCfgArgs from config file*/
    MMWL_readLowPowerConfig(&rfLpModeCfgArgs);

    retVal = rlSetLowPowerModeConfig(deviceMap, &rfLpModeCfgArgs);
    return retVal;
}

int MMWL_MonitoringConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;

    /* consolidated configuration of all analog monitoring
       This API SB sets the consolidated configuration of all analog monitoring
    */
    retVal = MMWL_rlRfAnaMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("consolidated Monitoring config failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("consolidated Monitoring config success for deviceMap %u \n", deviceMap);
    }
    printf("======================================================================\n\n");

    /* This API sets the monitoring periodicity based on frame count (FTTI) */
    retVal = MMMWL_setCalMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMMWL_setCalMonConfig failed for deviceMap %u with error code %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMMWL_setCalMonConfig success for deviceMap %u \n\n", deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API SB sets the Temperature sensor configuration    */
    retVal = MMWL_setRfTempMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfTempMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfTempMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets RX Gain and Phase Monitoring  configuration    */
    retVal = MMWL_setRfRxGainPhaMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfRxGainPhaMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfRxGainPhaMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets RX Noise Monitoring configuration    */
    retVal = MMWL_setRfRxNoiseMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfRxNoiseMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfRxNoiseMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets RX IF Stage Monitoring configuration    */
    retVal = MMWL_setRfRxIfStageMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfRxIfStageMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfRxIfStageMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets TX Power Monitoring configuration    */
    retVal = MMWL_setRfTxPowMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfTxPowMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfTxPowMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets TX Ballbreak Monitoring  configuration    */
    retVal = MMWL_setRfTxBallbreakMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfTxBallbreakMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfTxBallbreakMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets TX Gain Phase Mismatch Monitoring configuration    */
    retVal = MMWL_setRfTxGainPhaseMismatchMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfTxGainPhaseMismatchMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfTxGainPhaseMismatchMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets TX BPM Monitoring configuration    */
    retVal = MMWL_setRfTxBpmMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfTxPowMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfTxPowMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets TX Internal Analog Signals Monitoring configuration    */
    retVal = MMWL_setRfTxIntAnaSignalMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfTxIntAnaSignalMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfTxIntAnaSignalMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");


    /*   This API sets RX Internal Analog Signals Monitoring configuration    */
    retVal = MMWL_setRfRxIntAnaSignalMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfRxIntAnaSignalMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfRxIntAnaSignalMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets PM, CLK, LO Internal Analog Signals Monitoring configuration    */
    retVal = MMWL_setRfPmClkLoIntAnaSignalsMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfPmClkLoIntAnaSignalsMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfPmClkLoIntAnaSignalsMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets GPADC Internal Analog Signals Monitoring configuration */
    retVal = MMWL_setRfGpadcIntAnaSignalsMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfGpadcIntAnaSignalsMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfGpadcIntAnaSignalsMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");


    /*   This API sets APLL and Synthesizer’s control voltage signals monitoring configuration    */
    retVal = MMWL_setRfPllContrlVoltMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfPllContrlVoltMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfPllContrlVoltMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets Synth Freq Monitoring configuration    */
    retVal = MMWL_setRfSynthFreqMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfSynthFreqMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfSynthFreqMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets External Analog Signals Monitoring  configuration    */
    retVal = MMWL_setRfExtAnaSignalsMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfExtAnaSignalsMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfExtAnaSignalsMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets DCC based clock frequency monitoring configuration    */
    retVal = MMWL_setRfDualClkCompMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfDualClkCompMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfDualClkCompMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");


    /*   This API sets RX saturation detector monitoring configuration    */
    retVal = MMWL_setRfRxIfSatMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfRxIfSatMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfRxIfSatMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets RX signal image monitoring configuration    */
    retVal = MMWL_setRfRxSigImgMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfRxSigImgMonConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfRxSigImgMonConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");

    /*   This API sets RX mixer input power monitoring configuration    */
    retVal = MMWL_setRfRxMixerInPwrConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfRxMixerInPwrConfig failed for deviceMap %u with error %d \n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfRxMixerInPwrConfig success for deviceMap %u \n",
            deviceMap);
    }
    printf("======================================================================\n\n");


    return retVal;
}

int MML_RunTimeCalibConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
    rlRunTimeCalibConf_t runTimeCalibCfg = {0};

    runTimeCalibCfg.oneTimeCalibEnMask  = ((1 << 4) | (1 << 9) | (1 << 10));
    runTimeCalibCfg.periodicCalibEnMask = ((1 << 4) | (1 << 9) | (1 << 10));
    runTimeCalibCfg.calibPeriodicity    = 1;
    runTimeCalibCfg.reportEn = 5;

    mmwl_bRunTimeCalib = 0U;
    retVal = rlRfRunTimeCalibConfig(deviceMap, &runTimeCalibCfg);

    while (mmwl_bRunTimeCalib == 0U)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }

    return retVal;
}

/** @fn int MMWL_App()
*
*   @brief mmWaveLink monitoring Application.
*
*   @return int Success - 0, Failure - Error Code
*
*   mmWaveLink monitoring Application.
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
        printf("mmWave Device Power on success for deviceMap %u \n\n",
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
    retVal = MMWL_setDeviceCrcType(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("CRC Type set for MasterSS failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("CRC Type set for MasterSS success for deviceMap %u \n\n", deviceMap);
    }

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
        printf("Radar/RF subsystem Power up successful for deviceMap %u \n\n", deviceMap);
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
        printf("Basic/Static configuration success for deviceMap %u \n",
                deviceMap);
    }

    /*  \subsection     api_sequence5     Seq 5 - Initializes the mmWave Front end
    The mmWave Front end once configured needs to be initialized. During initialization
    mmWave Front end performs calibration and once calibration is complete, it
    notifies the application using asynchronous event
    */
    retVal = MMWL_rfInit(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("RF Initialization/Calibration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("RF Initialization/Calibration successful for deviceMap %u \n", deviceMap);
    }
    printf("======================================================================\n\n");

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

    /*  \subsection     api_sequence8     Seq 8 - Configure RF one time and periodic
    calibration of various RF/Analog aspects and trigger those. After this configuration, one
    time calibration report arrives immediately and periodic calibration report will arrive when
    frame starts in form of async event at every FTTI interval (here CAL_MON_TIME_UNIT) which
    is unit of frame count.
    @Note- This API must call after profileConfig API is called */
    retVal = MML_RunTimeCalibConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Run Time calibration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Run Time calibration successful for deviceMap %u \n\n", deviceMap);
    }

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
        /*  \subsection     api_sequence9     Seq 9 - FMCW frame configuration
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
        /*  \subsection     api_sequence9     Seq 9 - FMCW Advance frame configuration
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

    /*  \subsection     api_sequence10     Seq 10 - Configure monitoring time and frequncy unit
    and along with this enable misc monitoring feature of device.
    After this configuration when frame starts device will send monitoring data in form of
    async event at every FTTI interval (here CAL_MON_TIME_UNIT) which is unit of frame count */
    MMWL_MonitoringConfig(deviceMap);

    /*  \subsection     api_sequence11     Seq 11 - Start mmWave Radar Sensor
    This will trigger the mmWave Front to start transmitting FMCW signal. Raw ADC samples
    would be received from Digital front end. For AWR1243, if high speed interface is
    configured, RAW ADC data would be transmitted over CSI2/LVDS. On xWR1443/xWR1642, it can
    be processed using HW accelerator or DSP.
    After this API call, device will start sending monitoring data in form of Async Event message.
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

    /* wait till Host receives all monitoring report where number and periodicity of monitoring report is configured
    from rlRfSetCalMonTimeUnitConfig API. */
    while (gMonReportHdrCnt != (gFrameCount/CAL_MON_TIME_UNIT))
    {
        Sleep(1);
    }

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
