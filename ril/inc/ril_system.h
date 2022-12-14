/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of Quectel Co., Ltd. 2019
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   ril_system.h 
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   The file is for OpenCPU RIL sytem definitions and APIs.
 *
 * Author:
 * -------
 * -------
 *
 *============================================================================
 *             HISTORY
 *----------------------------------------------------------------------------
 * 
 ****************************************************************************/
#ifndef __RIL_SYSTEM_H__
#define __RIL_SYSTEM_H__



typedef struct
{
    s32 capacity;
    s32 voltage;
}ST_SysPower;

/*****************************************************************
* Function:     RIL_QuerySysInitStatus 
* 
* Description:
*               Queries the initializing status of module.
*
* Parameters:       
*               SysInitStatus
*                   [Out] system init status.
*               0/1/2/3, the init status value, one value of "Enum_SysInitState".
*               Please refer to "AT+QINISTAT" in ATC document for the meanings.
* Return:        
*                RIL_AT_SUCCESS,send AT successfully.
*                RIL_AT_FAILED, send AT failed.
*                RIL_AT_TIMEOUT,send AT timeout.
*                RIL_AT_BUSY,   sending AT.
*                RIL_AT_INVALID_PARAM, invalid input parameter.
*                RIL_AT_UNINITIALIZED, RIL is not ready, need to wait for MSG_ID_RIL_READY
*                                      and then call Ql_RIL_Initialize to initialize RIL.
*****************************************************************/
s32 RIL_QuerySysInitStatus(s32* SysInitStatus);

/*****************************************************************
* Function:     RIL_GetPowerSupply 
* 
* Description:
*               This function queries the battery balance, and the battery voltage.
*
* Parameters:
*               capacity:      
*                   [out] battery balance, a percent, ranges from 1 to 100.
*
*               voltage:       
*                   [out] battery voltage, unit in mV
* Return:        
*                RIL_AT_SUCCESS,send AT successfully.
*                RIL_AT_FAILED, send AT failed.
*                RIL_AT_TIMEOUT,send AT timeout.
*                RIL_AT_BUSY,   sending AT.
*                RIL_AT_INVALID_PARAM, invalid input parameter.
*                RIL_AT_UNINITIALIZED, RIL is not ready, need to wait for MSG_ID_RIL_READY
*                                      and then call Ql_RIL_Initialize to initialize RIL.
*****************************************************************/
s32 RIL_GetPowerSupply(u32* capacity, u32* voltage);

/*****************************************************************
* Function:     RIL_GetIMEI 
* 
* Description:
*               Retrieves the IMEI number of module.
*
* Parameters:       
*               imei:
*                   [Out] buffer to store the imei number. The length 
*                         of buffer should be at least 15-byte.
* Return:        
*                RIL_AT_SUCCESS,send AT successfully.
*                RIL_AT_FAILED, send AT failed.
*                RIL_AT_TIMEOUT,send AT timeout.
*                RIL_AT_BUSY,   sending AT.
*                RIL_AT_INVALID_PARAM, invalid input parameter.
*                RIL_AT_UNINITIALIZED, RIL is not ready, need to wait for MSG_ID_RIL_READY
*                                      and then call Ql_RIL_Initialize to initialize RIL.
*****************************************************************/
s32 RIL_GetIMEI(char* imei);

/*****************************************************************
* Function:     QSDK_Get_Str 
* 
* Description:
*               This function get strings based on the location of comma.
* Parameters:
*               None.
* Return:        
*               
*
*eg:src_string="GPRMC,235945.799,V,,,,,0.00,0.00,050180,,,N"
*index =1 ,dest_string="235945.799"; return TRUE
*index =,return FALSE
*****************************************************************/
bool QSDK_Get_Str(char *src_string,  char *dest_string, unsigned char index);

#endif  //__RIL_SYSTEM_H__

