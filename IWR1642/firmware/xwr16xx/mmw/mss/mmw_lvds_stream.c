/**
 *   @file  mmw_lvds_stream.c
 *
 *   @brief
 *      Implements LVDS stream functionality.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2019 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/knl/Semaphore.h>

/* MMWSDK Include Files. */
#include <ti/drivers/soc/soc.h>
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/cbuff/cbuff.h>
#include <ti/utils/hsiheader/hsiheader.h>

/* MMWAVE Demo Include Files */
#include <ti/demo/xwr16xx/mmw/mss/mmw_mss.h>
#include <ti/demo/xwr16xx/mmw/mmw_res.h>

extern MmwDemo_MSS_MCB    gMmwMssMCB;

 /**
 *  @b Description
 *  @n
 *      This function initializes/configures the LVDS
 *      streaming EDMA resources.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
void MmwDemo_LVDSStream_EDMAInit (void)
{
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelAllocatorIndex = 0;
    gMmwMssMCB.lvdsStream.swSessionEDMAChannelAllocatorIndex = 0;
 
    /* Populate the LVDS Stream HW Session EDMA Channel Table: */              
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[0].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_0;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[0].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_0;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[1].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_1;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[1].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_1;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[2].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_2;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[2].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_2;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[3].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_3;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[3].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_3;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[4].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_4;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[4].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_4;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[5].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_5;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[5].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_5;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[6].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_6;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[6].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_6;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[7].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_7;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[7].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_7;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[8].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_8;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[8].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_8;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[9].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_9;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[9].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_9;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[10].chainChannelsId      = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_10;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[10].shadowLinkChannelsId = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_10;

    /* Populate the LVDS Stream SW Session EDMA Channel Table: */
    gMmwMssMCB.lvdsStream.swSessionEDMAChannelTable[0].chainChannelsId       = MMW_LVDS_STREAM_SW_SESSION_EDMA_CH_0;
    gMmwMssMCB.lvdsStream.swSessionEDMAChannelTable[0].shadowLinkChannelsId  = MMW_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_0;
    gMmwMssMCB.lvdsStream.swSessionEDMAChannelTable[1].chainChannelsId       = MMW_LVDS_STREAM_SW_SESSION_EDMA_CH_1;
    gMmwMssMCB.lvdsStream.swSessionEDMAChannelTable[1].shadowLinkChannelsId  = MMW_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_1;
    gMmwMssMCB.lvdsStream.swSessionEDMAChannelTable[2].chainChannelsId       = MMW_LVDS_STREAM_SW_SESSION_EDMA_CH_2;
    gMmwMssMCB.lvdsStream.swSessionEDMAChannelTable[2].shadowLinkChannelsId  = MMW_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_2;
} 
 
 /**
 *  @b Description
 *  @n
 *      This is the LVDS streaming init function. 
 *      It initializes the necessary modules
 *      that implement the streaming.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_LVDSStreamInit (void)
{
    CBUFF_InitCfg           initCfg;
    int32_t                 retVal = MINUS_ONE;
    int32_t                 errCode;
    Semaphore_Params        semParams;

    /*************************************************************************************
     * Open the CBUFF Driver:
     *************************************************************************************/
    memset ((void *)&initCfg, 0, sizeof(CBUFF_InitCfg));

    /* Populate the configuration: */
    initCfg.socHandle                 = gMmwMssMCB.socHandle;
    initCfg.enableECC                 = 0U;
    initCfg.crcEnable                 = 1U;
    /* Up to 1 SW session + 1 HW session can be configured for each frame. Therefore max session is 2. */
    initCfg.maxSessions               = 2U;
    initCfg.enableDebugMode           = false;
    initCfg.interface                 = CBUFF_Interface_LVDS;
    initCfg.outputDataFmt             = CBUFF_OutputDataFmt_16bit;
    initCfg.u.lvdsCfg.crcEnable       = 0U;
    initCfg.u.lvdsCfg.msbFirst        = 1U;
    /* Enable all lanes available on the platform*/
    initCfg.u.lvdsCfg.lvdsLaneEnable  = 0x3U;
    initCfg.u.lvdsCfg.ddrClockMode    = 1U;
    initCfg.u.lvdsCfg.ddrClockModeMux = 1U;

    /* Initialize the CBUFF Driver: */
    gMmwMssMCB.lvdsStream.cbuffHandle = CBUFF_init (&initCfg, &errCode);
    if (gMmwMssMCB.lvdsStream.cbuffHandle == NULL)
    {
        /* Error: Unable to initialize the CBUFF Driver */
        System_printf("Error: CBUFF_init failed with [Error=%d]\n", errCode);
        goto exit;
    }


    /* User data buffer is involved in LVDS streaming.
     * The 64 byte alignment is a speculative workaround for an issue where
     * EDMA is not completing transfer. */
    gMmwMssMCB.lvdsStream.userDataHeader = Memory_alloc(NULL,
                                                        sizeof(MmwDemo_LVDSUserDataHeader_t),
                                                        64U, /* alignment */
                                                        NULL);

    /* Initialize the HSI Header Module: */
    if (HSIHeader_init (&initCfg, &errCode) < 0)
    {
        /* Error: Unable to initialize the HSI Header Module */
        System_printf("Error: HSIHeader_init failed with [Error=%d]\n", errCode);
        goto exit;
    }

    /* Populate EDMA resources */
    MmwDemo_LVDSStream_EDMAInit();
    
    /* Initialize semaphores */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    gMmwMssMCB.lvdsStream.hwFrameDoneSemHandle = Semaphore_create(0, &semParams, NULL);
    gMmwMssMCB.lvdsStream.swFrameDoneSemHandle = Semaphore_create(0, &semParams, NULL);

    /* Check some assumptions about s/w session regarding sizes for user buffer which
     * are going to stream out in CBUFF units so must be even number of bytes */
    MmwDemo_debugAssert((sizeof(MmwDemo_LVDSUserDataHeader_t) & 1) == 0);
    MmwDemo_debugAssert((sizeof(DPIF_PointCloudCartesian) & 1) == 0);
    MmwDemo_debugAssert((sizeof(DPIF_PointCloudSideInfo) & 1) == 0);

    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function that allocates CBUFF-EDMA channel
 *
 *  @param[in]  ptrEDMAInfo
 *      Pointer to the EDMA Information
 *  @param[out]  ptrEDMACfg
 *      Populated EDMA channel configuration
 *
 */
static void MmwDemo_LVDSStream_EDMAAllocateCBUFFChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
)
{
    if(ptrEDMAInfo->dmaNum == 0)
    {
        ptrEDMACfg->chainChannelsId      = MMW_LVDS_STREAM_CBUFF_EDMA_CH_0;
        ptrEDMACfg->shadowLinkChannelsId = MMW_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0;        
    }
    else if(ptrEDMAInfo->dmaNum == 1)
    {
        ptrEDMACfg->chainChannelsId      = MMW_LVDS_STREAM_CBUFF_EDMA_CH_1;
        ptrEDMACfg->shadowLinkChannelsId = MMW_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_1;        
    }    
    else
    {
        /* Max of 2 CBUFF sessions can be configured*/
        MmwDemo_debugAssert (0);
    }
}

/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel allocation function
 *      which allocates EDMA channels for CBUFF HW Session
 *
 *  @param[in]  ptrEDMAInfo
 *      Pointer to the EDMA Information
 *  @param[out]  ptrEDMACfg
 *      Populated EDMA channel configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_LVDSStream_EDMAAllocateCBUFFHwChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
)
{
    int32_t         retVal = MINUS_ONE;
    MmwDemo_LVDSStream_MCB_t *streamMCBPtr =  &gMmwMssMCB.lvdsStream;

    if(ptrEDMAInfo->isFirstEDMAChannel)
    {
        MmwDemo_LVDSStream_EDMAAllocateCBUFFChannel(ptrEDMAInfo, ptrEDMACfg);
        retVal = 0;
    }
    else
    {

        /* Sanity Check: Are there sufficient EDMA channels? */
        if (streamMCBPtr->hwSessionEDMAChannelAllocatorIndex >= MMWDEMO_LVDS_STREAM_HW_SESSION_MAX_EDMA_CHANNEL)
        {
            /* Error: All the EDMA channels are allocated */
            System_printf ("Error: MmwDemo_LVDSStream_EDMAAllocateCBUFFChannel failed. HW channel index=%d\n", 
                            streamMCBPtr->hwSessionEDMAChannelAllocatorIndex);
            goto exit;
        }

        /* Copy over the allocated EDMA configuration. */
        memcpy ((void *)ptrEDMACfg,
                (void*)&streamMCBPtr->hwSessionEDMAChannelTable[streamMCBPtr->hwSessionEDMAChannelAllocatorIndex],
                sizeof(CBUFF_EDMAChannelCfg));

        /* Increment the allocator index: */
        streamMCBPtr->hwSessionEDMAChannelAllocatorIndex++;

        /* EDMA Channel allocated successfully */
        retVal = 0;
    }    

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel allocation function
 *      which allocates EDMA channels for CBUFF SW Session
 *
 *  @param[in]  ptrEDMAInfo
 *      Pointer to the EDMA Information
 *  @param[out]  ptrEDMACfg
 *      Populated EDMA channel configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_LVDSStream_EDMAAllocateCBUFFSwChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
)
{
    int32_t         retVal = MINUS_ONE;
    MmwDemo_LVDSStream_MCB_t *streamMCBPtr =  &gMmwMssMCB.lvdsStream;

    if(ptrEDMAInfo->isFirstEDMAChannel)
    {
        MmwDemo_LVDSStream_EDMAAllocateCBUFFChannel(ptrEDMAInfo,ptrEDMACfg);
        retVal = 0;
    }
    else
    {
        /* Sanity Check: Are there sufficient EDMA channels? */
        if (streamMCBPtr->swSessionEDMAChannelAllocatorIndex >= MMWDEMO_LVDS_STREAM_SW_SESSION_MAX_EDMA_CHANNEL)
        {
            /* Error: All the EDMA channels are allocated */
            System_printf ("Error: MmwDemo_LVDSStream_EDMAAllocateCBUFFChannel failed. SW channel index=%d\n", 
                            streamMCBPtr->swSessionEDMAChannelAllocatorIndex);
            goto exit;
        }
        
        /* Copy over the allocated EDMA configuration. */
        memcpy ((void *)ptrEDMACfg,
                (void*)&streamMCBPtr->swSessionEDMAChannelTable[streamMCBPtr->swSessionEDMAChannelAllocatorIndex],
                sizeof(CBUFF_EDMAChannelCfg));
        
        /* Increment the allocator index: */
        streamMCBPtr->swSessionEDMAChannelAllocatorIndex++;
        
        /* EDMA Channel allocated successfully */
        retVal = 0;
    }    

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel free function which frees EDMA channels
 *      which had been allocated for use by a CBUFF HW Session
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_LVDSStream_EDMAFreeCBUFFHwChannel (CBUFF_EDMAChannelCfg* ptrEDMACfg)
{
    uint8_t    index;
    MmwDemo_LVDSStream_MCB_t *streamMCBPtr =  &gMmwMssMCB.lvdsStream;

    if((ptrEDMACfg->chainChannelsId == MMW_LVDS_STREAM_CBUFF_EDMA_CH_0) ||
       (ptrEDMACfg->chainChannelsId == MMW_LVDS_STREAM_CBUFF_EDMA_CH_1))
    {
        /*This is the CBUFF trigger channel. It is not part of the resource table so
          nothing needs to be done*/
        goto exit;  
    }

    for (index = 0U; index < MMWDEMO_LVDS_STREAM_HW_SESSION_MAX_EDMA_CHANNEL; index++) 
    {
        /* Do we have a match? */
        if (memcmp ((void*)ptrEDMACfg,
                    (void*)&streamMCBPtr->hwSessionEDMAChannelTable[index],
                    sizeof(CBUFF_EDMAChannelCfg)) == 0)
        {
            /* Yes: Decrement the HW Session index */
            streamMCBPtr->hwSessionEDMAChannelAllocatorIndex--;
            goto exit;
        }
    }

    /* Sanity Check: We should have had a match. An assertion is thrown to indicate that the EDMA channel
     * being cleaned up does not belong to the table*/
    MmwDemo_debugAssert (0);

exit:
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel free function which frees EDMA channels
 *      which had been allocated for use by a CBUFF SW Session
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_LVDSStream_EDMAFreeCBUFFSwChannel (CBUFF_EDMAChannelCfg* ptrEDMACfg)
{
    uint8_t    index;
    MmwDemo_LVDSStream_MCB_t *streamMCBPtr =  &gMmwMssMCB.lvdsStream;

    if((ptrEDMACfg->chainChannelsId == MMW_LVDS_STREAM_CBUFF_EDMA_CH_0) ||
       (ptrEDMACfg->chainChannelsId == MMW_LVDS_STREAM_CBUFF_EDMA_CH_1))
    {
        /*This is the CBUFF trigger channel. It is not part of the resource table so
          nothing needs to be done*/
        goto exit;  
    }

    for (index = 0U; index < MMWDEMO_LVDS_STREAM_SW_SESSION_MAX_EDMA_CHANNEL; index++)
    {
        /* Do we have a match? */
        if (memcmp ((void*)ptrEDMACfg,
                    (void*)&streamMCBPtr->swSessionEDMAChannelTable[index],
                    sizeof(CBUFF_EDMAChannelCfg)) == 0)
        {
            /* Yes: Decrement the SW Session index */
            streamMCBPtr->swSessionEDMAChannelAllocatorIndex--;
            goto exit;
        }
    }

    /* Sanity Check: We should have had a match. An assertion is thrown to indicate that the EDMA channel
     * being cleaned up does not belong to the table*/
    MmwDemo_debugAssert (0);

exit:
    return;
}


/**
 *  @b Description
 *  @n
 *      This function deletes the hardware session and any HSI
 *      header associated with it. 
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_LVDSStreamDeleteHwSession (void)
{
    int32_t     errCode;
    MmwDemo_LVDSStream_MCB_t* streamMcb = &gMmwMssMCB.lvdsStream;
    
    /* Delete session*/
    if (CBUFF_deleteSession (streamMcb->hwSessionHandle, &errCode) < 0)
    {
        /* Error: Unable to delete the session. */
        System_printf ("Error: MmwDemo_LVDSStreamDeleteHwSession CBUFF_deleteSession failed. Error code %d\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }
    
    streamMcb->hwSessionHandle = NULL;
    
    /* Did we stream out with the HSI Header? */
    if (streamMcb->isHwSessionHSIHeaderAllocated == true)
    {
        /* Delete the HSI Header: */
        if (HSIHeader_deleteHeader (&streamMcb->hwSessionHSIHeader, &errCode) < 0)
        {
            /* Error: Unable to delete the HSI Header */
            System_printf ("Error: MmwDemo_LVDSStreamDeleteHwSession HSIHeader_deleteHeader failed. Error code %d\n", errCode);
            MmwDemo_debugAssert(0);
            return;
        }

        streamMcb->isHwSessionHSIHeaderAllocated = false;
    }
}

/**
 *  @b Description
 *  @n
 *      This function deletes the SW session and any HSI
 *      header associated with it. 
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_LVDSStreamDeleteSwSession (void)
{
    int32_t     errCode;
    MmwDemo_LVDSStream_MCB_t* streamMcb = &gMmwMssMCB.lvdsStream;
    
    /* Delete session*/
    if (CBUFF_deleteSession (streamMcb->swSessionHandle, &errCode) < 0)
    {
        /* Error: Unable to delete the session. */
        System_printf ("Error: MmwDemo_LVDSStreamDeleteSwSession CBUFF_deleteSession failed. Error code %d\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }
    
    streamMcb->swSessionHandle = NULL;
    
    /* Delete the HSI Header: */
    if (HSIHeader_deleteHeader (&streamMcb->swSessionHSIHeader, &errCode) < 0)
    {
        /* Error: Unable to delete the HSI Header */
        System_printf ("Error: MmwDemo_LVDSStreamDeleteSwSession HSIHeader_deleteHeader failed. Error code %d\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the registered callback function which is invoked after the
 *      frame done interrupt is received for the hardware session.
 *
 *  @param[in]  sessionHandle
 *      Handle to the session
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_LVDSStream_HwTriggerFrameDone (CBUFF_SessionHandle sessionHandle)
{
    int32_t     errCode;

    /* Increment stats*/
    gMmwMssMCB.lvdsStream.hwFrameDoneCount++;

    if(sessionHandle != NULL)
    {
        /* There are 2 cases to consider:
           Only one subframe configured (legacy frame):
           If there is a software session configured for this subframe, we need to 
           deactivate the HW session here. 
           
           More than one subframe configured:
           If there is more than one subframe we need to deactivate the HW
           session here as other subframes may have configured a HW session.
           In this case (more than one subframe), the HW session is always created
           and activated in the application code */
        if((gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames > 1) ||
            (gMmwMssMCB.subFrameCfg[0].lvdsStreamCfg.isSwEnabled == 1))
        {
            if(CBUFF_deactivateSession (sessionHandle, &errCode) < 0)
            {
                /* Error: Unable to deactivate the session. */
                DebugP_assert(0);
                return;
            }
        }    
    }
    else
    {
        DebugP_assert(0);
    }

    Semaphore_post(gMmwMssMCB.lvdsStream.hwFrameDoneSemHandle);
}

/**
 *  @b Description
 *  @n
 *      This is the registered callback function which is invoked after the
 *      frame done interrupt is received for the SW session.
 *
 *  @param[in]  sessionHandle
 *      Handle to the session
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_LVDSStream_SwTriggerFrameDone (CBUFF_SessionHandle sessionHandle)
{    
    int32_t     errCode;

    /* Increment stats*/
    gMmwMssMCB.lvdsStream.swFrameDoneCount++;
    
    if(sessionHandle != NULL)
    {
        if(CBUFF_deactivateSession (sessionHandle, &errCode) < 0)
        {
            /* Error: Unable to deactivate the session. */
            DebugP_assert(0);
            return;
        }
        
        /*If only one subframe has been configured (legacy frame) and
          a HW session has been configured for that subframe, we need to
          enable it here as for the legacy frame the HW is NOT reconfigured
          every frame. 
          If more than one subframe is configured, the HW
          session is reconfigured and activated every subframe
          in the application code, not here.*/
        if((gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames == 1) &&
           (gMmwMssMCB.lvdsStream.hwSessionHandle != NULL))
        {        
            if(CBUFF_activateSession (gMmwMssMCB.lvdsStream.hwSessionHandle, &errCode) < 0)
            {
                DebugP_assert(0);
            }
        }        
    }
    else
    {
        DebugP_assert(0);
    }    
    
    Semaphore_post(gMmwMssMCB.lvdsStream.swFrameDoneSemHandle);
}

/**
 *  @b Description
 *  @n
 *      This is the LVDS streaming config function. 
 *      It configures the sessions for the LVDS streaming.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_LVDSStreamHwConfig (uint8_t subFrameIndx)
{
    CBUFF_SessionCfg          sessionCfg;
    MmwDemo_LVDSStream_MCB_t* streamMcb = &gMmwMssMCB.lvdsStream;
    int32_t                   errCode;
    int32_t                   retVal = MINUS_ONE;
    MmwDemo_SubFrameCfg       *subFrameCfg = &gMmwMssMCB.subFrameCfg[subFrameIndx];

    memset ((void*)&sessionCfg, 0, sizeof(CBUFF_SessionCfg));
    
    /* Populate the configuration: */
    sessionCfg.executionMode          = CBUFF_SessionExecuteMode_HW;
    sessionCfg.edmaHandle             = gMmwMssMCB.edmaHandle;
    sessionCfg.allocateEDMAChannelFxn = MmwDemo_LVDSStream_EDMAAllocateCBUFFHwChannel;
    sessionCfg.freeEDMAChannelFxn     = MmwDemo_LVDSStream_EDMAFreeCBUFFHwChannel;
    sessionCfg.frameDoneCallbackFxn   = MmwDemo_LVDSStream_HwTriggerFrameDone;
    sessionCfg.dataType               = CBUFF_DataType_COMPLEX;
    sessionCfg.u.hwCfg.dataMode       = (CBUFF_DataMode)subFrameCfg->adcBufCfg.chInterleave;
    
    /* Populate the HW Session configuration: */
    sessionCfg.u.hwCfg.adcBufHandle      = gMmwMssMCB.adcBufHandle;
    sessionCfg.u.hwCfg.numADCSamples     = subFrameCfg->numAdcSamples;
    sessionCfg.u.hwCfg.numChirpsPerFrame = subFrameCfg->numChirpsPerSubFrame;
    sessionCfg.u.hwCfg.chirpMode         = subFrameCfg->adcBufCfg.chirpThreshold;
    sessionCfg.u.hwCfg.opMode            = CBUFF_OperationalMode_CHIRP;
    
    switch(subFrameCfg->lvdsStreamCfg.dataFmt)
    {
        case MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_ADC:
            sessionCfg.u.hwCfg.dataFormat = CBUFF_DataFmt_ADC_DATA;
        break;
        case MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC_CQ:
            sessionCfg.u.hwCfg.dataFormat = CBUFF_DataFmt_CP_ADC_CQ;
            sessionCfg.u.hwCfg.cqSize[0] = 0;
            sessionCfg.u.hwCfg.cqSize[1] = HSIHeader_toCBUFFUnits(subFrameCfg->sigImgMonTotalSize);
            sessionCfg.u.hwCfg.cqSize[2] = HSIHeader_toCBUFFUnits(subFrameCfg->satMonTotalSize);
        break;
        default:
            System_printf ("Error: lvdsStreamCfg dataFmt %d is invalid\n", subFrameCfg->lvdsStreamCfg.dataFmt);
            MmwDemo_debugAssert(0);
        break;
    }    
        
    if(subFrameCfg->lvdsStreamCfg.isHeaderEnabled)
    {    
        MmwDemo_debugAssert(streamMcb->isHwSessionHSIHeaderAllocated == false);

        /* Create the HSI Header to be used for the HW Session: */ 
        if (HSIHeader_createHeader (&sessionCfg, false, &(streamMcb->hwSessionHSIHeader), &errCode) < 0)
        {
            /* Error: Unable to create the HSI Header; report the error */
            System_printf("Error: MmwDemo_LVDSStream_config unable to create HW HSI header with [Error=%d]\n", errCode);
            goto exit;
        }
        
        streamMcb->isHwSessionHSIHeaderAllocated = true;

        /* Setup the header in the CBUFF session configuration: */
        sessionCfg.header.size    = HSIHeader_getHeaderSize(&streamMcb->hwSessionHSIHeader);
        sessionCfg.header.address = (uint32_t)&(streamMcb->hwSessionHSIHeader);
    }    
       
    /* Create the HW Session: */
    streamMcb->hwSessionHandle = CBUFF_createSession (gMmwMssMCB.lvdsStream.cbuffHandle, &sessionCfg, &errCode);
                                                      
    if (streamMcb->hwSessionHandle == NULL)
    {
        /* Error: Unable to create the CBUFF hardware session */
        System_printf("Error: MmwDemo_LVDSStream_config unable to create the CBUFF hardware session with [Error=%d]\n", errCode);
        goto exit;
    }

    /* Control comes here implies that the LVDS Stream has been configured successfully */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the LVDS sw streaming config function.
 *      It configures the sw session for the LVDS streaming.
 *
 *  @param[in]  numObjOut      Number of detected objects to stream out
 *  @param[in]  objOut         Pointer to detected objects point cloud
 *  @param[in]  objOutSideInfo Pointer to detected objects side information
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_LVDSStreamSwConfig (uint32_t numObjOut,
                                    DPIF_PointCloudCartesian *objOut,
                                    DPIF_PointCloudSideInfo *objOutSideInfo)
{
    CBUFF_SessionCfg          sessionCfg;
    MmwDemo_LVDSStream_MCB_t* streamMcb = &gMmwMssMCB.lvdsStream;
    int32_t                   errCode;
    int32_t                   retVal = MINUS_ONE;

    memset ((void*)&sessionCfg, 0, sizeof(CBUFF_SessionCfg));
    
    /* Populate the configuration: */
    sessionCfg.executionMode                     = CBUFF_SessionExecuteMode_SW;
    sessionCfg.edmaHandle                        = gMmwMssMCB.edmaHandle;
    sessionCfg.allocateEDMAChannelFxn            = MmwDemo_LVDSStream_EDMAAllocateCBUFFSwChannel;
    sessionCfg.freeEDMAChannelFxn                = MmwDemo_LVDSStream_EDMAFreeCBUFFSwChannel;
    sessionCfg.frameDoneCallbackFxn              = MmwDemo_LVDSStream_SwTriggerFrameDone;
    sessionCfg.dataType                          = CBUFF_DataType_COMPLEX; 
    sessionCfg.u.swCfg.userBufferInfo[0].size    = HSIHeader_toCBUFFUnits(sizeof(MmwDemo_LVDSUserDataHeader_t));
    sessionCfg.u.swCfg.userBufferInfo[0].address = (uint32_t)(streamMcb->userDataHeader);

    /* Note size and addresses have defaulted to 0 due to memset zero initialization above */
    if(numObjOut != 0)
    {
        sessionCfg.u.swCfg.userBufferInfo[1].size    = HSIHeader_toCBUFFUnits(numObjOut * sizeof(DPIF_PointCloudCartesian));
        sessionCfg.u.swCfg.userBufferInfo[1].address = (uint32_t)objOut;

        sessionCfg.u.swCfg.userBufferInfo[2].size    = HSIHeader_toCBUFFUnits(numObjOut * sizeof(DPIF_PointCloudSideInfo));
        sessionCfg.u.swCfg.userBufferInfo[2].address = (uint32_t)objOutSideInfo;
    }

    /* Create the HSI Header to be used for the SW Session: */
    if (HSIHeader_createHeader (&sessionCfg, true, &(streamMcb->swSessionHSIHeader), &errCode) < 0)
    {
        /* Error: Unable to create the HSI Header; report the error */
        System_printf("Error: MmwDemo_LVDSStream_config unable to create HW HSI header with [Error=%d]\n", errCode);
        goto exit;
    }
    
    /* Setup the header in the CBUFF session configuration: */
    sessionCfg.header.size    = HSIHeader_getHeaderSize(&streamMcb->swSessionHSIHeader);
    sessionCfg.header.address = (uint32_t)&(streamMcb->swSessionHSIHeader);

    /* Create the SW Session. */
    streamMcb->swSessionHandle = CBUFF_createSession (gMmwMssMCB.lvdsStream.cbuffHandle, &sessionCfg, &errCode);
    
    if (streamMcb->swSessionHandle == NULL)
    {
        /* Error: Unable to create the CBUFF SW session */
        System_printf("Error: MmwDemo_LVDSStream_config unable to create the CBUFF SW session with [Error=%d]\n", errCode);
        goto exit;
    }

    /* Control comes here implies that the LVDS Stream has been configured successfully */
    retVal = 0;

exit:
    return retVal;
}

/**
*  @b Description
*  @n
*      High level API for configuring Hw session. Deletes h/w session if it exists,
*      configures desired configuration input and activates the h/w session
*  @param[in]  subFrameIndx Index of sub-frame
*
*  @retval
*      None
*/
void MmwDemo_configLVDSHwData(uint8_t subFrameIndx)
{
    int32_t retVal;

    /* Delete previous CBUFF HW session if one was configured */
    if(gMmwMssMCB.lvdsStream.hwSessionHandle != NULL)
    {
        MmwDemo_LVDSStreamDeleteHwSession();
    }

    /* Configure HW session */
    if (MmwDemo_LVDSStreamHwConfig(subFrameIndx) < 0)
    {
        System_printf("Failed LVDS stream HW configuration\n");
        MmwDemo_debugAssert(0);
    }

    /* If HW LVDS stream is enabled, start the session here so that ADC samples will be
    streamed out as soon as the first chirp samples land on ADC*/
    if(CBUFF_activateSession(gMmwMssMCB.lvdsStream.hwSessionHandle, &retVal) < 0)
    {
        System_printf("Failed to activate CBUFF session for LVDS stream HW. errCode=%d\n",retVal);
        MmwDemo_debugAssert(0);
    }
}
