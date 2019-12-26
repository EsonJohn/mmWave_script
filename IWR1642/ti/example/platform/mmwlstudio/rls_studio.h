/*
 * rls_studio.h - mmWaveLink Device and Communication Interface Implementation
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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
 *
*/

/****************************************************************************************
 * FILE INCLUSION PROTECTION
 ****************************************************************************************
 */
#ifndef MMWL_STUDIO_H
#define MMWL_STUDIO_H

/******************************************************************************
 * INCLUDE FILES
 ******************************************************************************
 */
#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************************
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/* Port Description*/
#define PORT_A          (0)
#define PORT_B          (1)
#define PORT_C          (2)
#define PORT_D          (3)

/* RL STUDIO Version */
#define RL_STUDIO_VER               "1.1.5.0.20.7.18"

/* UART Flags */
#define UART_IF_OPEN_FLAG_NONE       (0)
#define UART_IF_OPEN_FLAG_RE_OPEN    (1)

/* Board Error Definition */
#define RLS_ERR_NERR                    (1)
#define RLS_ERR_PIN_MONITOR             (1<<1)
#define RLS_ERR_TSWCONN_MONITOR         (1<<2)

#define RLS_NUM_CONNECTED_DEVICES_MAX   (4U)
#define RLS_DEVICE_NAME_SIZE_MAX        (32U)

/* SOP Modes */
#define RLS_SOP_MODE_SCAN_APTG          (1)
#define RLS_SOP_MODE_DEVELOPMENT        (2)
#define RLS_SOP_MODE_THB                (3)
#define RLS_SOP_MODE_FUNCTIONAL         (4)
#define RLS_SOP_MODE_FLASH_DOWNLOAD     (5)
#define RLS_SOP_MODE_FUNCTIONAL_CPHA1   (6)

/* RadarLink Studio return Codes */
#define RLS_RET_CODE_OK                 (0)
#define RLS_RET_CODE_COMM_OPEN_ERROR    (-1)
#define RLS_RET_CODE_COMM_WR_ERROR      (-2)
#define RLS_RET_CODE_COMM_TIMEOUT       (-3)
#define RLS_RET_CODE_COMM_RD_ERROR      (-4)
#define RLS_RET_CODE_DEVICE_NOT_FOUND   (-5)
#define RLS_RET_CODE_MALLOC_FAILED      (-6)
#define RLS_RET_CODE_NULL_PTR_ERROR     (-7)
#define RLS_RET_CODE_INVALID_INPUT      (-8)


/******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 ******************************************************************************
 */

/*! \brief
* UART Interface Param structure
*/
typedef struct  
{
    unsigned int BaudRate;
    unsigned char FlowControlEnable;
    unsigned char CommPort;
}rlsUartIfParams_t;

/*! \brief
* RadarLink Event Handler
*/
typedef void (*RLS_P_EVENT_HANDLER)(unsigned char deviceIndex, void* pValue);

/*! \brief
* RadarLink Studio Device handle
*/
typedef void* rlsDevHandle_t;

/*! \brief
* UART interface handle
*/
typedef void* rlsUartHandle_t;

/******************************************************************************
 * FUNCTION DEFINITIONS
 ******************************************************************************
 */

/* Device/Board COntrol Interface*/
__declspec(dllexport) int rlsDisableDevice(unsigned char deviceIndex);
__declspec(dllexport) int rlsEnableDevice(unsigned char deviceIndex);

/* Enable/Disable device at time of firmware upgrade*/
__declspec(dllexport) void rlsEnableDeviceMap(unsigned char DeviceMap);
__declspec(dllexport) void rlsDisableDeviceMap(unsigned char DeviceMap);

/*get number of devices connected*/
__declspec(dllexport) int rlsGetNumofDevices(int *numofdevices);
__declspec(dllexport) unsigned int rlsGetNumofConnectedDevices(unsigned int DeviceMap);

/* Host Interupt Interface*/
__declspec(dllexport) int rlsRegisterInterruptHandler(unsigned char deviceIndex, 
                                                      RLS_P_EVENT_HANDLER InterruptHdl , 
                                                      void* pValue);
/*Block call, waits till interrupt level(high/low) occured on Irq line*/
__declspec(dllexport) int rlsDeviceWaitIrqStatus(rlsDevHandle_t hdl, unsigned char level);

/* SPI Interupt Interface*/
__declspec(dllexport) void rlsSpiIRQMask(rlsDevHandle_t hdl);
__declspec(dllexport) void rlsSpiIRQUnMask(rlsDevHandle_t hdl);

/* UART Interupt Interface*/
__declspec(dllexport) void rlsUartIRQMask(rlsDevHandle_t hdl);
__declspec(dllexport) void rlsUartIRQUnMask(rlsDevHandle_t hdl);

/* SPI Communication Channel Interface*/
__declspec(dllexport) rlsDevHandle_t rlsSpiOpen(void* ifParam, unsigned long flags);
__declspec(dllexport) int rlsSpiClose(rlsDevHandle_t hdl);
__declspec(dllexport) int rlsSpiRead(rlsDevHandle_t hdl, unsigned char *pBuff, unsigned short len);
__declspec(dllexport) int rlsSpiWrite(rlsDevHandle_t hdl, unsigned char *pBuff, unsigned short len);

/* UART Communication Channel Interface*/
__declspec(dllexport) rlsUartHandle_t rlsUartOpen(void* ifParam, unsigned long flags);
__declspec(dllexport) int rlsUartClose(rlsDevHandle_t hdl);
__declspec(dllexport) int rlsUartWrite(rlsDevHandle_t hdl, unsigned char* pBuffer, unsigned short BytesToWrite);
__declspec(dllexport) int rlsUartRead(rlsDevHandle_t hdl, unsigned char* pBuffer, unsigned short BytesToRead);

/* Get Version API*/
__declspec(dllexport) char* rlsGetVersion(void);

/* Get Handle API*/
__declspec(dllexport) rlsDevHandle_t rlsGetDeviceCtx(unsigned char ucDevId);


/* Board Controls Interface */
__declspec(dllexport) int rlsOpenBoardControlIf(rlsDevHandle_t hdl);
__declspec(dllexport) int rlsCloseBoardControlIf(rlsDevHandle_t hdl);
__declspec(dllexport) int rlsOpenGenericGpioIf(rlsDevHandle_t hdl);
__declspec(dllexport) int rlsCloseGenericGpioIf(rlsDevHandle_t hdl);
__declspec(dllexport) int rlsWarmReset(rlsDevHandle_t hdl, int status);
__declspec(dllexport) int rlsFullReset(rlsDevHandle_t hdl, int status);
__declspec(dllexport) int rlsWriteNErrIn(rlsDevHandle_t hdl, int state);
__declspec(dllexport) int rlsReadNErrOut(rlsDevHandle_t hdl);
__declspec(dllexport) int rlsEnOnbrdClkBuffer(rlsDevHandle_t hdl, int state);
__declspec(dllexport) int rlsPortDGPIOMode(rlsDevHandle_t hdl,int GPIOnum, int IOmode);
__declspec(dllexport) int rlsReadGPIO(rlsDevHandle_t hdl,int portnum,int pinnum);
__declspec(dllexport) int rlsWriteGPIO(rlsDevHandle_t hdl,int portnum,int pinnum, int state);
__declspec(dllexport) int rlsSOPControl(rlsDevHandle_t hdl, int SOPmode);
__declspec(dllexport) int rlsReadPinMonitor(rlsDevHandle_t hdl);
__declspec(dllexport) int rlsReadTSWMonitor(rlsDevHandle_t hdl);
__declspec(dllexport) int rlsEnablePMICReset(rlsDevHandle_t hdl, int state);
__declspec(dllexport) int rlsRegisterBrdInterruptHandler(RLS_P_EVENT_HANDLER InterruptHdl , 
                                                                            void* pValue);

/* I2C Interface */
__declspec(dllexport) int rlsOpenI2cIrqIf(rlsDevHandle_t hdl);
__declspec(dllexport) int rlsI2CWrite(rlsDevHandle_t hdl, unsigned char slaveAddress, 
                                      unsigned char regAddress, unsigned char msbData, 
                                      unsigned char lsbData, int datasize);

__declspec(dllexport) int rlsI2CRead(rlsDevHandle_t hdl, unsigned char slaveAddress, 
                                     unsigned char regAddress, unsigned char *msbData, 
                                     unsigned char *lsbData, int datasize);
__declspec(dllexport) int rlsCloseI2cIrqIf(rlsDevHandle_t hdl);

/* AR1243 4 Chip Cascade System Pin Mapping & Mux Control */
__declspec(dllexport) int rlsMuxControl( unsigned int DevId );

#ifdef  __cplusplus
}
#endif 


#endif 
/*
 * END OF MMWL_STUDIO_H FILE
 */
