/****************************************************************************************
* FileName     : mmw_config.c
*
* Description  : This file reads the mmwave configuration from config file.
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
#include "mmw_config.h"

/****************************************************************************************
* MACRO DEFINITIONS
****************************************************************************************
*/

/******************************************************************************
* GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
******************************************************************************
*/

/* File pointer for config file*/
FILE *mmwl_configfPtr = NULL;

/******************************************************************************
* Function Definitions
*******************************************************************************
*/

/** @fn char *MMWL_trim(char * s)
*
*   @brief get rid of trailing and leading whitespace along with "\n"
*
*   @param[in] s - String pointer which needs to be trimed
*
*   @return int Success - 0, Failure - Error Code
*
*   get rid of trailing and leading whitespace along with "\n"
*/
char *MMWL_trim(char * s)
{
    /* Initialize start, end pointers */
    char *s1 = s, *s2 = &s[strlen(s) - 1];

    /* Trim and delimit right side */
    while ((isspace(*s2)) && (s2 >= s1))
        s2--;
    *(s2 + 1) = '\0';

    /* Trim left side */
    while ((isspace(*s1)) && (s1 < s2))
        s1++;

    /* Copy finished string */
    strcpy(s, s1);
    return s;
}

/** @fn unsigned int MMWL_getCascadeStatus(unsigned int cascade_enable)
*
*   @brief Read cascade_enable params from config file.
*
*   @param[in] cascade_enable.
*
*   @return int Success - 0, Failure - Error Code
*
*   API to poweroff device.
*/
unsigned int MMWL_getCascadeStatus(unsigned int cascade_enable)
{
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    int retVal = RL_RET_CODE_OK;
    unsigned int readAllParams = 0;
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL) 
            && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        /* Copy into correct entry in parameters struct */
        if (strcmp(name, "cascade_enable") == 0)
        {
            cascade_enable = atoi(value);
            readAllParams = 1;
        }
    }
    return cascade_enable;

}

/** @fn void MMWL_readPowerOnMaster(rlClientCbs_t *clientCtx)
*
*   @brief Read rlClientCbs_t params from config file.
*
*   @param[in] rlClientCbs_t *clientCtx
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read rlDevicePowerOn configuration params
*/
void MMWL_readPowerOnMaster(rlClientCbs_t *clientCtx)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL) 
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "crcType") == 0)
        {
            clientCtx->crcType = (rlCrcType_t)atoi(value);
        }
        
        if (strcmp(name, "ackTimeout") == 0)
        {
            clientCtx->ackTimeout = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readChannelConfig(rlChanCfg_t *rfChanCfgArgs, 
*                            unsigned short cascade)
*
*   @brief Read rlChanCfg_t params from config file.
*
*   @param[in] rlChanCfg_t *rfChanCfgArgs
*    @param[in] unsigned short cascade
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read channel configuration params
*/
void MMWL_readChannelConfig(rlChanCfg_t *rfChanCfgArgs, 
                            unsigned short cascade)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL) 
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

		if (strcmp(name, "channelTx") == 0){
			rfChanCfgArgs->txChannelEn = atoi(value);
		}

		if (strcmp(name, "channelRx") == 0)
		{
			rfChanCfgArgs->rxChannelEn = atoi(value);
		}

        if (strcmp(name, "cascading") == 0)
        {
			rfChanCfgArgs->cascading = atoi(value);
            readAllParams = 1;
        }
    }
    rfChanCfgArgs->cascading = cascade;
}

/** @fn void MMWL_readAdcOutConfig(rlAdcOutCfg_t *adcOutCfgArgs)
*
*   @brief Read rlAdcOutCfg_t params from config file.
*
*   @param[in] rlAdcOutCfg_t *adcOutCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read ADC configuration params
*/
void MMWL_readAdcOutConfig(rlAdcOutCfg_t *adcOutCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "adcBits") == 0)
            adcOutCfgArgs->fmt.b2AdcBits = atoi(value);

        if (strcmp(name, "adcFormat") == 0)
        {
            adcOutCfgArgs->fmt.b2AdcOutFmt = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readDataFmtConfig(rlDevDataFmtCfg_t *dataFmtCfgArgs)
*
*   @brief Read rlDevDataFmtCfg_t params from config file.
*
*   @param[in] rlDevDataFmtCfg_t *dataFmtCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read data format configuration params
*/
void MMWL_readDataFmtConfig(rlDevDataFmtCfg_t *dataFmtCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "rxChanEn") == 0)
            dataFmtCfgArgs->rxChannelEn = atoi(value);

        if (strcmp(name, "adcBitsD") == 0)
            dataFmtCfgArgs->adcBits = atoi(value);

        if (strcmp(name, "adcFmt") == 0)
            dataFmtCfgArgs->adcFmt = atoi(value);

        if (strcmp(name, "iqSwapSel") == 0)
            dataFmtCfgArgs->iqSwapSel = atoi(value);

        if (strcmp(name, "chInterleave") == 0)
        {
            dataFmtCfgArgs->chInterleave = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readLowPowerConfig(rlLowPowerModeCfg_t *rfLpModeCfgArgs)
*
*   @brief Read rlLowPowerModeCfg_t params from config file.
*
*   @param[in] rlLowPowerModeCfg_t *rfLpModeCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read low power configuration params
*/
void MMWL_readLowPowerConfig(rlLowPowerModeCfg_t *rfLpModeCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "lpAdcMode") == 0)
        {
            rfLpModeCfgArgs->lpAdcMode = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readDataPathConfig(rlDevDataPathCfg_t *dataPathCfgArgs)
*
*   @brief Read rlDevDataPathCfg_t params from config file.
*
*   @param[in] rlDevDataPathCfg_t *dataPathCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read data path configuration params
*/
void MMWL_readDataPathConfig(rlDevDataPathCfg_t *dataPathCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "intfSel") == 0)
            dataPathCfgArgs->intfSel = atoi(value);

        if (strcmp(name, "transferFmtPkt0") == 0)
            dataPathCfgArgs->transferFmtPkt0 = atoi(value);

        if (strcmp(name, "transferFmtPkt1") == 0)
            dataPathCfgArgs->transferFmtPkt1 = atoi(value);

        if (strcmp(name, "cqConfig") == 0)
            dataPathCfgArgs->cqConfig = atoi(value);
        
        if (strcmp(name, "cq0TransSize") == 0)
            dataPathCfgArgs->cq0TransSize = atoi(value);
        
        if (strcmp(name, "cq1TransSize") == 0)
            dataPathCfgArgs->cq1TransSize = atoi(value);
        
        if (strcmp(name, "cq2TransSize") == 0)
            dataPathCfgArgs->cq2TransSize = atoi(value);
    }
}

/** @fn void MMWL_readLvdsClkConfig(rlDevDataPathClkCfg_t *lvdsClkCfgArgs)
*
*   @brief Read rlDevDataPathClkCfg_t params from config file.
*
*   @param[in] rlDevDataPathClkCfg_t *lvdsClkCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read LVDS clock configuration params
*/
void MMWL_readLvdsClkConfig(rlDevDataPathClkCfg_t *lvdsClkCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "laneClk") == 0)
            lvdsClkCfgArgs->laneClkCfg = atoi(value);

        if (strcmp(name, "dataRate") == 0)
        {
            lvdsClkCfgArgs->dataRate = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readSetHsiClock(rlDevHsiClk_t *hsiClkgs)
*
*   @brief Read rlDevHsiClk_t params from config file.
*
*   @param[in] rlDevHsiClk_t *hsiClkgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read data path clock configuration params
*/
void MMWL_readSetHsiClock(rlDevHsiClk_t *hsiClkgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "hsiClk") == 0)
        {
            hsiClkgs->hsiClk = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readLaneConfig(rlDevLaneEnable_t *laneEnCfgArgs)
*
*   @brief Read rlDevLaneEnable_t params from config file.
*
*   @param[in] rlDevLaneEnable_t *laneEnCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read LVDS/CSI2 lane configuration params
*/
void MMWL_readLaneConfig(rlDevLaneEnable_t *laneEnCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "laneEn") == 0)
        {
            laneEnCfgArgs->laneEn = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readLvdsLaneConfig(rlDevLvdsLaneCfg_t *lvdsLaneCfgArgs)
*
*   @brief Read rlDevLvdsLaneCfg_t params from config file.
*
*   @param[in] rlDevLvdsLaneCfg_t *lvdsLaneCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read LVDS specific configuration params
*/
void MMWL_readLvdsLaneConfig(rlDevLvdsLaneCfg_t *lvdsLaneCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "laneFmtMap") == 0)
            lvdsLaneCfgArgs->laneFmtMap = atoi(value);

        if (strcmp(name, "laneParamCfg") == 0)
        {
            lvdsLaneCfgArgs->laneParamCfg = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readProfileConfig(rlProfileCfg_t *profileCfgArgs)
*
*   @brief Read rlProfileCfg_t params from config file.
*
*   @param[in] rlProfileCfg_t *profileCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read profile configuration
*/
void MMWL_readProfileConfig(rlProfileCfg_t *profileCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN], *ptr;
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "profileId") == 0)
            profileCfgArgs->profileId = atoi(value);

        if (strcmp(name, "startFreqConst") == 0)
            profileCfgArgs->startFreqConst = strtoul(value, &ptr, 10);

        if (strcmp(name, "idleTimeConst") == 0)
            profileCfgArgs->idleTimeConst = strtoul(value, &ptr, 10);

        if (strcmp(name, "adcStartTimeConst") == 0)
            profileCfgArgs->adcStartTimeConst = strtoul(value, &ptr, 10);

        if (strcmp(name, "rampEndTime") == 0)
            profileCfgArgs->rampEndTime = strtoul(value, &ptr, 10);

        if (strcmp(name, "txOutPowerBackoffCode") == 0)
            profileCfgArgs->txOutPowerBackoffCode = strtoul(value, &ptr, 10);

        if (strcmp(name, "txPhaseShifter") == 0)
            profileCfgArgs->txPhaseShifter = strtoul(value, &ptr, 10);

        if (strcmp(name, "freqSlopeConst") == 0)
            profileCfgArgs->freqSlopeConst = atoi(value);

        if (strcmp(name, "txStartTime") == 0)
            profileCfgArgs->txStartTime = atoi(value);

        if (strcmp(name, "numAdcSamples") == 0)
            profileCfgArgs->numAdcSamples = atoi(value);

        if (strcmp(name, "digOutSampleRate") == 0)
            profileCfgArgs->digOutSampleRate = atoi(value);

        if (strcmp(name, "hpfCornerFreq1") == 0)
            profileCfgArgs->hpfCornerFreq1 = atoi(value);

        if (strcmp(name, "hpfCornerFreq2") == 0)
            profileCfgArgs->hpfCornerFreq2 = atoi(value);

		if (strcmp(name, "txCalibEnCfg") == 0)
			profileCfgArgs->txCalibEnCfg = atoi(value);

        if (strcmp(name, "rxGain") == 0)
        {
            profileCfgArgs->rxGain = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readChirpConfig(rlChirpCfg_t *chirpCfgArgs)
*
*   @brief Read rlChirpCfg_t params from config file.
*

*   @param[in] rlChirpCfg_t *chirpCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read chirp configuration params
*/

void MMWL_readChirpConfig(rlChirpCfg_t *chirpCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN], *ptr;
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "chirpStartIdx") == 0)
            chirpCfgArgs->chirpStartIdx = atoi(value);

        if (strcmp(name, "chirpEndIdx") == 0)
            chirpCfgArgs->chirpEndIdx = atoi(value);

        if (strcmp(name, "profileIdCPCFG") == 0)
            chirpCfgArgs->profileId = atoi(value);

        if (strcmp(name, "startFreqVar") == 0)
            chirpCfgArgs->startFreqVar = strtoul(value, &ptr, 10);

        if (strcmp(name, "freqSlopeVar") == 0)
            chirpCfgArgs->freqSlopeVar = atoi(value);

        if (strcmp(name, "idleTimeVar") == 0)
            chirpCfgArgs->idleTimeVar = atoi(value);

        if (strcmp(name, "adcStartTimeVar") == 0)
            chirpCfgArgs->adcStartTimeVar = atoi(value);

        if (strcmp(name, "txEnable") == 0)
        {
            chirpCfgArgs->txEnable = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readFrameConfig(rlFrameCfg_t *frameCfgArgs)
*
*   @brief Read rlFrameCfg_t params from config file.
*
*   @param[in] rlFrameCfg_t *frameCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read frame configuration params
*/
void MMWL_readFrameConfig(rlFrameCfg_t *frameCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN], *ptr;
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "chirpStartIdxFCF") == 0)
            frameCfgArgs->chirpStartIdx = atoi(value);

        if (strcmp(name, "chirpEndIdxFCF") == 0)
            frameCfgArgs->chirpEndIdx = atoi(value);

        if (strcmp(name, "frameCount") == 0)
            frameCfgArgs->numFrames = atoi(value);

        if (strcmp(name, "loopCount") == 0)
            frameCfgArgs->numLoops = atoi(value);

        if (strcmp(name, "periodicity") == 0)
            frameCfgArgs->framePeriodicity = strtoul(value, &ptr, 10);

        if (strcmp(name, "triggerDelay") == 0)
            frameCfgArgs->frameTriggerDelay = strtoul(value, &ptr, 10);

        if (strcmp(name, "numAdcSamples") == 0)
            frameCfgArgs->numAdcSamples = atoi(value);

        if (strcmp(name, "triggerSelect") == 0)
        {
            frameCfgArgs->triggerSelect = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readAdvFrameConfig(rlAdvFrameCfg_t *rlAdvFrameCfgArgs)
*
*   @brief Read rlAdvFrameCfg_t params from config file.
*
*   @param[in] rlAdvFrameCfg_t *rlAdvFrameCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read frame configuration params
*/
void MMWL_readAdvFrameConfig(rlAdvFrameCfg_t *rlAdvFrameCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN], *ptr;
    unsigned char subFrameCfgCnt = 0, numsubframe = 0, advframe_flag = 0;
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
        && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "numOfSubFrames") == 0){
            rlAdvFrameCfgArgs->frameSeq.numOfSubFrames = atoi(value);
            subFrameCfgCnt = rlAdvFrameCfgArgs->frameSeq.numOfSubFrames;

            rlAdvFrameCfgArgs->frameData.numSubFrames = atoi(value);
            numsubframe = rlAdvFrameCfgArgs->frameData.numSubFrames;

            advframe_flag = 1;
        }

        if (strcmp(name, "forceProfile") == 0)
            rlAdvFrameCfgArgs->frameSeq.forceProfile = atoi(value);

        if (strcmp(name, "numFrames") == 0)
            rlAdvFrameCfgArgs->frameSeq.numFrames = atoi(value);

        if (strcmp(name, "loopBackCfg") == 0)
            rlAdvFrameCfgArgs->frameSeq.loopBackCfg = atoi(value);

        if (strcmp(name, "triggerSelect") == 0)
            rlAdvFrameCfgArgs->frameSeq.triggerSelect = atoi(value);

        if (strcmp(name, "frameTrigDelay") == 0)
            rlAdvFrameCfgArgs->frameSeq.frameTrigDelay = strtoul(value, &ptr, 10);

        if (strcmp(name, "forceProfileIdx") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[--subFrameCfgCnt].forceProfileIdx = atoi(value);

        if (strcmp(name, "chirpStartIdxAF") == 0)
            if (advframe_flag == 1){
                advframe_flag = 0;
                rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].chirpStartIdxOffset = atoi(value);
            }

        if (strcmp(name, "numOfChirps") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfChirps = atoi(value);

        if (strcmp(name, "numLoops") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numLoops = atoi(value);

        if (strcmp(name, "burstPeriodicity") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].burstPeriodicity = strtoul(value, &ptr, 10);

        if (strcmp(name, "chirpStartIdxOffset") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].chirpStartIdxOffset = atoi(value);

        if (strcmp(name, "numOfBurst") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfBurst = atoi(value);

        if (strcmp(name, "numOfBurstLoops") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfBurstLoops = atoi(value);

        if (strcmp(name, "subFramePeriodicity") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].subFramePeriodicity = strtoul(value, &ptr, 10);

        if (strcmp(name, "numAdcSamplesAF") == 0)
            rlAdvFrameCfgArgs->frameData.subframeDataCfg[--numsubframe].numAdcSamples = atoi(value);

        if (strcmp(name, "numChirpsInDataPacket") == 0)
        {
            rlAdvFrameCfgArgs->frameData.subframeDataCfg[numsubframe].numChirpsInDataPacket = atoi(value);

            /* Total number of chirps in one subframe */
            rlAdvFrameCfgArgs->frameData.subframeDataCfg[numsubframe].totalChirps =
                (rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfChirps *
                rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numLoops *
                rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfBurst *
                rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfBurstLoops);

            if (numsubframe == 0)
                readAllParams = 1;
        }

    }
}

/** @fn void MMWL_readContModeConfig(rlContModeCfg_t * rlContModeCfgArgs)
*
*   @brief Read rlContModeCfg_t params from config file.
*
*   @param[in] rlContModeCfg_t * rlContModeCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read continuous mode configuration params
*/
void MMWL_readContModeConfig(rlContModeCfg_t * rlContModeCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
        && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "vcoSelect") == 0)
            rlContModeCfgArgs->vcoSelect= atoi(value);

        if (strcmp(name, "contModeRxGain") == 0)
            rlContModeCfgArgs->rxGain = atoi(value);
    }

}

/** @fn void MMWL_dynChirpConfig(rlDynChirpCfg_t* rldynChirpCfgArgs)
*
*   @brief Read rlDynChirpCfg_t params from config file.
*
*   @param[in] rlDynChirpCfg_t* rldynChirpCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read dynamic chirp configuration params
*/
void MMWL_readDynChirpConfig(rlDynChirpCfg_t* rldynChirpCfgArgs)
{
    int readAllParams = 0;
    int chirpRowCnt = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
        && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "chirpRowSel") == 0)
            rldynChirpCfgArgs->chirpRowSelect = atoi(value);

        if (strcmp(name, "chirpSegSel") == 0)
            rldynChirpCfgArgs->chirpSegSel = atoi(value);

        if (strcmp(name, "chirpNR1") == 0)
        {
            for (chirpRowCnt = 0; chirpRowCnt < 16; chirpRowCnt++)
                rldynChirpCfgArgs->chirpRow[chirpRowCnt].chirpNR1 = atoi(value);
        }
        if (strcmp(name, "chirpNR2") == 0)
        {
            for (chirpRowCnt = 0; chirpRowCnt < 16; chirpRowCnt++)
                rldynChirpCfgArgs->chirpRow[chirpRowCnt].chirpNR2 = atoi(value);
        }
        if (strcmp(name, "chirpNR3") == 0)
        {
            for (chirpRowCnt = 0; chirpRowCnt < 16; chirpRowCnt++)
                rldynChirpCfgArgs->chirpRow[chirpRowCnt].chirpNR3 = atoi(value);
        }
    }

}

/** @fn int MMWL_openConfigFile()
*
*   @brief Opens MMWave config file
*
*   @return int Success - 0, Failure - Error Code
*
*   Opens MMWave config file
*/
int MMWL_openConfigFile()
{
    /*open config file to read parameters*/
    if (mmwl_configfPtr == NULL)
    {
        mmwl_configfPtr = fopen("mmwaveconfig.txt", "r");
        if (mmwl_configfPtr == NULL)
        {
            printf("failed to open config file\n");
            return -1;
        }
    }
    return 0;
}

/** @fn void MMWL_closeConfigFile()
*
*   @brief Close MMWave config file
*
*   Close MMWave config file
*/
void MMWL_closeConfigFile()
{
    /* Close config file */
    fclose(mmwl_configfPtr);
    mmwl_configfPtr = NULL;
}

