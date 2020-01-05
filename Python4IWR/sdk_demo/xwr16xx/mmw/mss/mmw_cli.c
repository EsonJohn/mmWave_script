/*
 *   @file  mmw_cli.c
 *
 *   @brief
 *      Mmw (Milli-meter wave) DEMO CLI Implementation
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
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

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* BIOS/XDC Include Files. */
#include <xdc/runtime/System.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/uart/UART.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/mathutils/mathutils.h>

/* Demo Include Files */
#include <ti/demo/xwr16xx/mmw/include/mmw_config.h>
#include <ti/demo/xwr16xx/mmw/mss/mmw_mss.h>
#include <ti/demo/utils/mmwdemo_adcconfig.h>
#include <ti/demo/utils/mmwdemo_rfparser.h>

/**************************************************************************
 *************************** Local function prototype****************************
 **************************************************************************/

/* CLI Extended Command Functions */
static int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIMultiObjBeamForming (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICalibDcRangeSig (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIClutterRemoval (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICfarFovCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIAoAFovCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIExtendedMaxVelocity (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIBpmCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIChirpQualitySigImgMonCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLILvdsStreamCfg (int32_t argc, char* argv[]);

/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/

extern MmwDemo_MSS_MCB    gMmwMssMCB;

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/


/**************************************************************************
 *************************** CLI  Function Definitions **************************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor start command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[])
{
    bool        doReconfig = true;
    int32_t     retVal = 0;

    /*  Only following command syntax will be supported 
        sensorStart
        sensorStart 0
    */
    if (argc == 2)
    {
        doReconfig = (bool) atoi (argv[1]);

        if (doReconfig == true)
        {
            CLI_write ("Error: Reconfig is not supported, only argument of 0 is\n"
                       "(do not reconfig, just re-start the sensor) valid\n");
            return -1;
        }
    }
    else
    {
        /* In case there is no argument for sensorStart, always do reconfig */
        doReconfig = true;
    }

    /***********************************************************************************
     * Do sensor state management to influence the sensor actions
     ***********************************************************************************/

    /* Error checking initial state: no partial config is allowed 
       until the first sucessful sensor start state */
    if ((gMmwMssMCB.sensorState == MmwDemo_SensorState_INIT) || 
         (gMmwMssMCB.sensorState == MmwDemo_SensorState_OPENED))
    {
        MMWave_CtrlCfg ctrlCfg;

        /* need to get number of sub-frames so that next function to check
         * pending state can work */
        CLI_getMMWaveExtensionConfig (&ctrlCfg);
        gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames =
            MmwDemo_RFParser_getNumSubFrames(&ctrlCfg);

        if (MmwDemo_isAllCfgInPendingState() == 0)
        {
            CLI_write ("Error: Full configuration must be provided before sensor can be started "
                       "the first time\n");

            /* Although not strictly needed, bring back to the initial value since we
             * are rejecting this first time configuration, prevents misleading debug. */
            gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames = 0;

            return -1;
        }
    }

    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: Sensor is already started\n");
        return 0;
    }

    if (doReconfig == false)
    {
         /* User intends to issue sensor start without config, check if no
            config was issued after stop and generate error if this is the case. */
         if (MmwDemo_isAllCfgInNonPendingState() == 0)
         {
             /* Message user differently if all config was issued or partial config was
                issued. */
             if (MmwDemo_isAllCfgInPendingState())
             {
                 CLI_write ("Error: You have provided complete new configuration, "
                            "issue \"sensorStart\" (without argument) if you want it to "
                            "take effect\n");
             }
             else
             {
                 CLI_write ("Error: You have provided partial configuration between stop and this "
                            "command and partial configuration cannot be undone."
                            "Issue the full configuration and do \"sensorStart\" \n");
             }
             return -1;
         }
    }
    else
    {
        /* User intends to issue sensor start with full config, check if all config
           was issued after stop and generate error if  is the case. */
        MMWave_CtrlCfg ctrlCfg;

        /* need to get number of sub-frames so that next function to check
         * pending state can work */
        CLI_getMMWaveExtensionConfig (&ctrlCfg);
        gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames =
            MmwDemo_RFParser_getNumSubFrames(&ctrlCfg);
        
        if (MmwDemo_isAllCfgInPendingState() == 0)
        {
            /* Message user differently if no config was issued or partial config was
               issued. */
            if (MmwDemo_isAllCfgInNonPendingState())
            {
                CLI_write ("Error: You have provided no configuration, "
                           "issue \"sensorStart 0\" OR provide "
                           "full configuration and issue \"sensorStart\"\n");
            }
            else
            {
                CLI_write ("Error: You have provided partial configuration between stop and this "
                           "command and partial configuration cannot be undone."
                           "Issue the full configuration and do \"sensorStart\" \n");
            }
            /* Although not strictly needed, bring back to the initial value since we
             * are rejecting this first time configuration, prevents misleading debug. */
            gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames = 0;
            return -1;
        }
    }

    /***********************************************************************************
     * Retreive and check mmwave Open related config before calling openSensor
     ***********************************************************************************/

    /*  Fill demo's MCB mmWave openCfg structure from the CLI configs*/
    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_INIT)
    {
        /* Get the open configuration: */
        CLI_getMMWaveExtensionOpenConfig (&gMmwMssMCB.cfg.openCfg);
        /* call sensor open */
        retVal = MmwDemo_openSensor(true);
        if(retVal != 0)
        {
            return -1;
        }
        gMmwMssMCB.sensorState = MmwDemo_SensorState_OPENED;    
    }
    else
    {
        /* openCfg related configurations like chCfg, lowPowerMode, adcCfg
         * are only used on the first sensor start. If they are different
         * on a subsequent sensor start, then generate a fatal error
         * so the user does not think that the new (changed) configuration
         * takes effect, the board needs to be reboot for the new
         * configuration to be applied.
         */
        MMWave_OpenCfg openCfg;
        CLI_getMMWaveExtensionOpenConfig (&openCfg);
        /* Compare openCfg->chCfg*/
        if(memcmp((void *)&gMmwMssMCB.cfg.openCfg.chCfg, (void *)&openCfg.chCfg,
                          sizeof(rlChanCfg_t)) != 0)
        {
            MmwDemo_debugAssert(0);
        }
        
        /* Compare openCfg->lowPowerMode*/
        if(memcmp((void *)&gMmwMssMCB.cfg.openCfg.lowPowerMode, (void *)&openCfg.lowPowerMode,
                          sizeof(rlLowPowerModeCfg_t)) != 0)
        {
            MmwDemo_debugAssert(0);
        }
        /* Compare openCfg->adcOutCfg*/
        if(memcmp((void *)&gMmwMssMCB.cfg.openCfg.adcOutCfg, (void *)&openCfg.adcOutCfg,
                          sizeof(rlAdcOutCfg_t)) != 0)
        {
            MmwDemo_debugAssert(0);
        }
    }

    

    /***********************************************************************************
     * Retrieve mmwave Control related config before calling startSensor
     ***********************************************************************************/
    /* Get the mmWave ctrlCfg from the CLI mmWave Extension */
    if(doReconfig)
    {
        /* if MmwDemo_openSensor has non-first time related processing, call here again*/
        /* call sensor config */
        CLI_getMMWaveExtensionConfig (&gMmwMssMCB.cfg.ctrlCfg);
        retVal = MmwDemo_configSensor();
        if(retVal != 0)
        {
            return -1;
        }
    }
    retVal = MmwDemo_startSensor();
    if(retVal != 0)
    {
        return -1;
    }

    /***********************************************************************************
     * Set the state
     ***********************************************************************************/
    gMmwMssMCB.sensorState = MmwDemo_SensorState_STARTED;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor stop command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[])
{
    if ((gMmwMssMCB.sensorState == MmwDemo_SensorState_STOPPED) ||
        (gMmwMssMCB.sensorState == MmwDemo_SensorState_INIT) ||
        (gMmwMssMCB.sensorState == MmwDemo_SensorState_OPENED))
    {
        CLI_write ("Ignored: Sensor is already stopped\n");
        return 0;
    }

    MmwDemo_stopSensor();

    MmwDemo_resetStaticCfgPendingState();

    gMmwMssMCB.sensorState = MmwDemo_SensorState_STOPPED;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function to get sub-frame number
 *
 *  @param[in] argc  Number of arguments
 *  @param[in] argv  Arguments
 *  @param[in] expectedArgc Expected number of arguments
 *  @param[out] subFrameNum Sub-frame Number (0 based)
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIGetSubframe (int32_t argc, char* argv[], int32_t expectedArgc,
                                       int8_t* subFrameNum)
{
    int8_t subframe;
    
    /* Sanity Check: Minimum argument check */
    if (argc != expectedArgc)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /*Subframe info is always in position 1*/
    subframe = (int8_t) atoi(argv[1]);

    if(subframe >= (int8_t)RL_MAX_SUBFRAMES)
    {
        CLI_write ("Error: Subframe number is invalid\n");
        return -1;
    }

    *subFrameNum = (int8_t)subframe;

    return 0;
}



/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for gui monitoring configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[])
{
    MmwDemo_GuiMonSel   guiMonSel;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 8, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the guiMonSel configuration: */
    memset ((void *)&guiMonSel, 0, sizeof(MmwDemo_GuiMonSel));

    /* Populate configuration: */
    guiMonSel.detectedObjects           = atoi (argv[2]);
    guiMonSel.logMagRange               = atoi (argv[3]);
    guiMonSel.noiseProfile              = atoi (argv[4]);
    guiMonSel.rangeAzimuthHeatMap       = atoi (argv[5]);
    guiMonSel.rangeDopplerHeatMap       = atoi (argv[6]);
    guiMonSel.statsInfo                 = atoi (argv[7]);

    MmwDemo_CfgUpdate((void *)&guiMonSel, MMWDEMO_GUIMONSEL_OFFSET,
        sizeof(MmwDemo_GuiMonSel), subFrameNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for CFAR configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[])
{
    DPU_CFARCAProc_CfarCfg   cfarCfg;
    uint32_t            procDirection;
    int8_t              subFrameNum;
    float               threshold;

    if(MmwDemo_CLIGetSubframe(argc, argv, 10, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfarCfg, 0, sizeof(cfarCfg));

    /* Populate configuration: */
    procDirection             = (uint32_t) atoi (argv[2]);
    cfarCfg.averageMode       = (uint8_t) atoi (argv[3]);
    cfarCfg.winLen            = (uint8_t) atoi (argv[4]);
    cfarCfg.guardLen          = (uint8_t) atoi (argv[5]);
    cfarCfg.noiseDivShift     = (uint8_t) atoi (argv[6]);
    cfarCfg.cyclicMode        = (uint8_t) atoi (argv[7]);
    threshold                 = (float) atof (argv[8]);
    cfarCfg.peakGroupingEn    = (uint8_t) atoi (argv[9]);

    if (threshold > 100.0)
    {
        CLI_write("Error: Maximum value for CFAR thresholdScale is 100.0 dB.\n");
        return -1;
    }   
    
    /* threshold is a float value from 0-100dB. It needs to
       be later converted to linear scale (conversion can only be done
       when the number of virtual antennas is known) before passing it
       to CFAR DPU.
       For now, the threshold will be coded in a 16bit integer in the following
       way:
       suppose threshold is a float represented as XYZ.ABC
       it will be saved as a 16bit integer XYZAB       
       that is, 2 decimal cases are saved.*/
    threshold = threshold * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR;   
    cfarCfg.thresholdScale    = (uint16_t) threshold;
    
    /* Save Configuration to use later */     
    if (procDirection == 0)
    {
        MmwDemo_CfgUpdate((void *)&cfarCfg, MMWDEMO_CFARCFGRANGE_OFFSET,
                          sizeof(cfarCfg), subFrameNum);
    }
    else
    {
        MmwDemo_CfgUpdate((void *)&cfarCfg, MMWDEMO_CFARCFGDOPPLER_OFFSET,
                          sizeof(cfarCfg), subFrameNum);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for CFAR FOV (Field Of View) configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLICfarFovCfg (int32_t argc, char* argv[])
{
    DPU_CFARCAProc_FovCfg   fovCfg;
    uint32_t            procDirection;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&fovCfg, 0, sizeof(fovCfg));

    /* Populate configuration: */
    procDirection             = (uint32_t) atoi (argv[2]);
    fovCfg.min                = (float) atof (argv[3]);
    fovCfg.max                = (float) atof (argv[4]);

    /* Save Configuration to use later */
    if (procDirection == 0)
    {
        MmwDemo_CfgUpdate((void *)&fovCfg, MMWDEMO_FOVRANGE_OFFSET,
                          sizeof(fovCfg), subFrameNum);
    }
    else
    {
        MmwDemo_CfgUpdate((void *)&fovCfg, MMWDEMO_FOVDOPPLER_OFFSET,
                          sizeof(fovCfg), subFrameNum);
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for AoA FOV (Field Of View) configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIAoAFovCfg (int32_t argc, char* argv[])
{
    DPU_AoAProc_FovAoaCfg   fovCfg;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&fovCfg, 0, sizeof(fovCfg));

    /* Populate configuration: */
    fovCfg.minAzimuthDeg      = (float) atoi (argv[2]);
    fovCfg.maxAzimuthDeg      = (float) atoi (argv[3]);
    fovCfg.minElevationDeg    = (float) atoi (argv[4]);
    fovCfg.maxElevationDeg    = (float) atoi (argv[5]);

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&fovCfg, MMWDEMO_FOVAOA_OFFSET,
                      sizeof(fovCfg), subFrameNum);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for extended maximum velocity configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIExtendedMaxVelocity (int32_t argc, char* argv[])
{
    DPU_AoAProc_ExtendedMaxVelocityCfg   cfg;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 3, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.enabled      = (uint8_t) atoi (argv[2]);

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&cfg, MMWDEMO_EXTMAXVEL_OFFSET,
                      sizeof(cfg), subFrameNum);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for multi object beam forming configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIMultiObjBeamForming (int32_t argc, char* argv[])
{
    DPU_AoAProc_MultiObjBeamFormingCfg cfg;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 4, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.enabled                     = (uint8_t) atoi (argv[2]);
    cfg.multiPeakThrsScal           = (float) atof (argv[3]);

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&cfg, MMWDEMO_MULTIOBJBEAMFORMING_OFFSET,
                      sizeof(cfg), subFrameNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for DC range calibration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLICalibDcRangeSig (int32_t argc, char* argv[])
{
    DPU_RangeProc_CalibDcRangeSigCfg cfg;
    uint32_t                   log2NumAvgChirps;
    int8_t                     subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);
    cfg.negativeBinIdx   = (int16_t)  atoi (argv[3]);
    cfg.positiveBinIdx   = (int16_t)  atoi (argv[4]);
    cfg.numAvgChirps     = (uint16_t) atoi (argv[5]);

    if (cfg.negativeBinIdx > 0)
    {
        CLI_write ("Error: Invalid negative bin index\n");
        return -1;
    }
    if ((cfg.positiveBinIdx - cfg.negativeBinIdx + 1) > DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE)
    {
        CLI_write ("Error: Number of bins exceeds the limit\n");
        return -1;
    }
    log2NumAvgChirps = (uint32_t) mathUtils_ceilLog2(cfg.numAvgChirps);
    if (cfg.numAvgChirps != (1U << log2NumAvgChirps))
    {
        CLI_write ("Error: Number of averaged chirps is not power of two\n");
        return -1;
    }

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&cfg, MMWDEMO_CALIBDCRANGESIG_OFFSET,
                      sizeof(cfg), subFrameNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Clutter removal Configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIClutterRemoval (int32_t argc, char* argv[])
{
    DPC_ObjectDetection_StaticClutterRemovalCfg_Base cfg;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 3, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for clutter removal */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&cfg, MMWDEMO_STATICCLUTTERREMOFVAL_OFFSET,
                      sizeof(cfg), subFrameNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for data logger set command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[])
{
    MmwDemo_ADCBufCfg   adcBufCfg;
    int8_t              subFrameNum;

    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    if(MmwDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&adcBufCfg, 0, sizeof(adcBufCfg));

    /* Populate configuration: */
    adcBufCfg.adcFmt          = (uint8_t) atoi (argv[2]);
    adcBufCfg.iqSwapSel       = (uint8_t) atoi (argv[3]);
    adcBufCfg.chInterleave    = (uint8_t) atoi (argv[4]);
    adcBufCfg.chirpThreshold  = (uint8_t) atoi (argv[5]);

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&adcBufCfg,
                      MMWDEMO_ADCBUFCFG_OFFSET,
                      sizeof(MmwDemo_ADCBufCfg), subFrameNum);
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for compensation of range bias and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    DPU_AoAProc_compRxChannelBiasCfg   cfg;
    int32_t Re, Im;
    int32_t argInd;
    int32_t i;

    /* Sanity Check: Minimum argument check */
    if (argc != (1+1+SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.rangeBias          = (float) atof (argv[1]);

    argInd = 2;
    for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        Re = (int32_t) (atof (argv[argInd++]) * 32768.);
        MATHUTILS_SATURATE16(Re);
        cfg.rxChPhaseComp[i].real = (int16_t) Re;

        Im = (int32_t) (atof (argv[argInd++]) * 32768.);
        MATHUTILS_SATURATE16(Im);
        cfg.rxChPhaseComp[i].imag = (int16_t) Im;

    }
    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.compRxChanCfg,
           &cfg, sizeof(cfg));

    gMmwMssMCB.objDetCommonCfg.isCompRxChannelBiasCfgPending = 1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for measurement configuration of range bias
 *      and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    DPC_ObjectDetection_MeasureRxChannelBiasCfg   cfg;

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.enabled          = (uint8_t) atoi (argv[1]);
    cfg.targetDistance   = (float) atof (argv[2]);
    cfg.searchWinSize   = (float) atof (argv[3]);

    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.measureRxChannelBiasCfg,
           &cfg, sizeof(cfg));

    gMmwMssMCB.objDetCommonCfg.isMeasureRxChannelBiasCfgPending = 1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for BPM configuration supported by the mmw Demo
 *      Note that there is a generic BPM configuration command supported by
 *      utils/cli and mmwave. The generic BPM command is not supported by the
 *      demo as the mmw demo assumes a specific BPM pattern for the TX antennas.
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIBpmCfg (int32_t argc, char* argv[])
{
    int8_t              subFrameNum;
    MmwDemo_BpmCfg      bpmCfg;

    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    if(MmwDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&bpmCfg, 0, sizeof(MmwDemo_BpmCfg));

    /* Populate configuration: */
    bpmCfg.isEnabled = (bool) atoi(argv[2]) ;
    bpmCfg.chirp0Idx = (uint16_t) atoi(argv[3]) ;
    bpmCfg.chirp1Idx = (uint16_t) atoi(argv[4]) ;

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&bpmCfg, MMWDEMO_BPMCFG_OFFSET,
                      sizeof(MmwDemo_BpmCfg), subFrameNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for configuring CQ RX Saturation monitor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[])
{
    rlRxSatMonConf_t        cqSatMonCfg;

    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cqSatMonCfg, 0, sizeof(rlRxSatMonConf_t));

    /* Populate configuration: */
    cqSatMonCfg.profileIndx                 = (uint8_t) atoi (argv[1]);

    if(cqSatMonCfg.profileIndx < RL_MAX_PROFILES_CNT)
    {

        cqSatMonCfg.satMonSel                   = (uint8_t) atoi (argv[2]);
        cqSatMonCfg.primarySliceDuration        = (uint16_t) atoi (argv[3]);
        cqSatMonCfg.numSlices                   = (uint16_t) atoi (argv[4]);
        cqSatMonCfg.rxChannelMask               = (uint8_t) atoi (argv[5]);

        /* Save Configuration to use later */
        gMmwMssMCB.cqSatMonCfg[cqSatMonCfg.profileIndx] = cqSatMonCfg;

        return 0;
    }
    else
    {
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for configuring CQ Singal & Image band monitor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIChirpQualitySigImgMonCfg (int32_t argc, char* argv[])
{
    rlSigImgMonConf_t       cqSigImgMonCfg;

    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cqSigImgMonCfg, 0, sizeof(rlSigImgMonConf_t));

    /* Populate configuration: */
    cqSigImgMonCfg.profileIndx              = (uint8_t) atoi (argv[1]);

    if(cqSigImgMonCfg.profileIndx < RL_MAX_PROFILES_CNT)
    {
        cqSigImgMonCfg.numSlices            = (uint8_t) atoi (argv[2]);
        cqSigImgMonCfg.timeSliceNumSamples  = (uint16_t) atoi (argv[3]);

        /* Save Configuration to use later */
        gMmwMssMCB.cqSigImgMonCfg[cqSigImgMonCfg.profileIndx] = cqSigImgMonCfg;

        return 0;
    }
    else
    {
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for enabling analog monitors
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[])
{
    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Save Configuration to use later */
    gMmwMssMCB.anaMonCfg.rxSatMonEn = atoi (argv[1]);
    gMmwMssMCB.anaMonCfg.sigImgMonEn = atoi (argv[2]);

    gMmwMssMCB.isAnaMonCfgPending = 1;

    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the High Speed Interface
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLILvdsStreamCfg (int32_t argc, char* argv[])
{
    MmwDemo_LvdsStreamCfg   cfg;
    int8_t                  subFrameNum;

    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    if(MmwDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_LvdsStreamCfg));

    /* Populate configuration: */
    cfg.isHeaderEnabled = (bool)    atoi(argv[2]);
    cfg.dataFmt         = (uint8_t) atoi(argv[3]);
    cfg.isSwEnabled     = (bool)    atoi(argv[4]);

    /* If both h/w and s/w are enabled, HSI header must be enabled, because
     * we don't allow mixed h/w session without HSI header
     * simultaneously with s/w session with HSI header (s/w session always
     * streams HSI header) */
    if ((cfg.isSwEnabled == true) && (cfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED))
    {
        if (cfg.isHeaderEnabled == false)
        {
            CLI_write("Error: header must be enabled when both h/w and s/w streaming are enabled\n");
            return -1;
        }
    }

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&cfg,
                      MMWDEMO_LVDSSTREAMCFG_OFFSET,
                      sizeof(MmwDemo_LvdsStreamCfg), subFrameNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_CLIInit (uint8_t taskPriority)
{
    CLI_Cfg     cliCfg;
    char        demoBanner[256];
    uint32_t    cnt;

    /* Create Demo Banner to be printed out by CLI */
    sprintf(&demoBanner[0], 
                       "******************************************\n" \
                       "xWR16xx MMW Demo %02d.%02d.%02d.%02d\n"  \
                       "******************************************\n", 
                        MMWAVE_SDK_VERSION_MAJOR,
                        MMWAVE_SDK_VERSION_MINOR,
                        MMWAVE_SDK_VERSION_BUGFIX,
                        MMWAVE_SDK_VERSION_BUILD
            );

    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "mmwDemo:/>";
    cliCfg.cliBanner                    = demoBanner;
    cliCfg.cliUartHandle                = gMmwMssMCB.commandUartHandle;
    cliCfg.taskPriority                 = taskPriority;
    cliCfg.socHandle                    = gMmwMssMCB.socHandle;
    cliCfg.mmWaveHandle                 = gMmwMssMCB.ctrlHandle;
    cliCfg.enableMMWaveExtension        = 1U;
    cliCfg.usePolledMode                = true;
    cliCfg.overridePlatform             = false;
    cliCfg.overridePlatformString       = NULL;
    
    cnt=0;
    cliCfg.tableEntry[cnt].cmd            = "sensorStart";
    cliCfg.tableEntry[cnt].helpString     = "[doReconfig(optional, default:enabled)]";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorStart;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "sensorStop";
    cliCfg.tableEntry[cnt].helpString     = "No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorStop;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "guiMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <detectedObjects> <logMagRange> <noiseProfile> <rangeAzimuthHeatMap> <rangeDopplerHeatMap> <statsInfo>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIGuiMonSel;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "cfarCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <procDirection> <averageMode> <winLen> <guardLen> <noiseDiv> <cyclicMode> <thresholdScale> <peakGroupingEn>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICfarCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "multiObjBeamForming";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <threshold>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMultiObjBeamForming;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "calibDcRangeSig";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <negativeBinIdx> <positiveBinIdx> <numAvgFrames>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICalibDcRangeSig;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "clutterRemoval";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIClutterRemoval;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "adcbufCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <adcOutputFmt> <SampleSwap> <ChanInterleave> <ChirpThreshold>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIADCBufCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "compRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<rangeBias> <Re00> <Im00> <Re01> <Im01> <Re02> <Im02> <Re03> <Im03> <Re10> <Im10> <Re11> <Im11> <Re12> <Im12> <Re13> <Im13> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "measureRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<enabled> <targetDistance> <searchWin>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "aoaFovCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <minAzimuthDeg> <maxAzimuthDeg> <minElevationDeg> <maxElevationDeg>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIAoAFovCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "cfarFovCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <procDirection> <min (meters or m/s)> <max (meters or m/s)>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICfarFovCfg;
    cnt++;
    cliCfg.tableEntry[cnt].cmd            = "extendedMaxVelocity";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIExtendedMaxVelocity;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "bpmCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <chirp0Idx> <chirp1Idx>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIBpmCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQRxSatMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <satMonSel> <priSliceDuration> <numSlices> <rxChanMask>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIChirpQualityRxSatMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQSigImgMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <numSlices> <numSamplePerSlice>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIChirpQualitySigImgMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "analogMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<rxSaturation> <sigImgBand>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIAnalogMonitorCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "lvdsStreamCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enableHeader> <dataFmt> <enableSW>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLILvdsStreamCfg;
    cnt++;

    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        System_printf ("Error: Unable to open the CLI\n");
        return;
    }
    System_printf ("Debug: CLI is operational\n");
    return;
}


