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
 *   ql_eint.h 
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   EINT API defines.
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


#ifndef __QL_EINT_H__
#define __QL_EINT_H__

#include "ql_gpio.h"


typedef enum{
    EINT_EDGE_FALLING_AND_RISING = 0, // Edge and falling or rising trigger. 
    EINT_EDGE_RISING,                 //Edge and rising trigger. 
    EINT_EDGE_FALLING,                // Edge and falling trigger. 
}Enum_EintType;


/*****************************************************************
* Description:
*               Definition for EINT callback function.
* 
* Parameters:
*               eintPinName:
*                   EINT pin name, one value of Enum_PinName, which is registered by 
*                   calling Ql_EINT_Register() or Ql_EINT_RegisterFast().
*
*               pinLevel:
*                   The EINT level value, one value of Enum_PinLevel. 
*
*               customParam:
*                   The customized parameter, which is passed in by calling 
*                   Ql_EINT_Register() or Ql_EINT_RegisterFast().
*****************************************************************/
typedef void (*Callback_EINT_Handle)(Enum_PinName eintPinName, Enum_PinLevel pinLevel, void* customParam);

/*****************************************************************
* Function:     Ql_EINT_Register 
* 
* Description:
*               This function registers an EINT I/O, and specifies the interrupt handler. 
*
* Parameters:
*               eintPinName:
*                   EINT pin name, one value of Enum_PinName that has 
*                   the interrupt function.
*
*               callback_eint:
*                   The interrupt handler or ISR.
*
*               customParam:
*                   A customized parameter, which can be use in interrupt handler. 
* Return:        
*               QL_RET_OK, this function succeeds.
*               QL_RET_ERR_PARAM, the input IO is invalid. 
*               QL_RET_ERR_NOSUPPORTEINT, did not supports eint.
*               QL_RET_ERR_USED, this EINT is registered already.
*               QL_RET_ERR_NOSUPPORTEINT,this pin not supports EINT function.
*****************************************************************/
s32 Ql_EINT_Register(Enum_PinName eintPinName, Callback_EINT_Handle callback_eint, void* customParam);

/*****************************************************************
* Function:     Ql_EINT_RegisterFast 
* 
* Description:
*               This function registers an EINT I/O, and specifies the interrupt handler. 
*               It creates a high priority interrupt.
*
*               IMPORTANT NOTES:
*               Please don't add any task schedule in the interrupt
*               handler.And the interrupt handler cannot consume much
*               CPU time. Or it causes system exception or reset.
*
* Parameters:
*               eintPinName:
*                   EINT pin name, one value of Enum_PinName that has 
*                   the interrupt function.
*
*               callback_eint:
*                   The interrupt handler or ISR.
*
*               customParam:
*                   A customized parameter, which can be use in interrupt handler. 
* Return:        
*               QL_RET_OK, this function succeeds.
*               QL_RET_ERR_PARAM, the input IO is invalid. 
*               QL_RET_ERR_NOSUPPORTEINT, did not supports eint.
*               QL_RET_ERR_USED, this EINT is registered already.
*               QL_RET_ERR_NOSUPPORTEINT,this pin not supports EINT function.
*****************************************************************/
s32 Ql_EINT_RegisterFast(Enum_PinName eintPinName, Callback_EINT_Handle callback_eint, void* customParam);

/*****************************************************************
* Function:     Ql_EINT_Init 
* 
* Description:
*               Initialize an external interrupt function. 
*
* Parameters:
*               eintPinName:
*                   EINT pin name, one value of Enum_PinName that has 
*                   the interrupt function.
*
*               eintType:
*                   Interrupt type, supports edge-triggered interrupt.
*
*		    automask:
*	               mask the Eint after the interrupt happened.
* Return:        
*               QL_RET_OK, this function succeeds.
**             QL_RET_ERR_PARAM, the input IO is invalid. 
*               QL_RET_ERR_NOSUPPORTEINT, did not supports eint.
*               QL_RET_ERR_USED, this EINT is registered already.
*               QL_RET_ERR_NOSUPPORTEINT,this pin not supports EINT function.
*****************************************************************/
s32 Ql_EINT_Init(Enum_PinName eintPinName, Enum_EintType eintType, bool automask);

/*****************************************************************
* Function:     Ql_EINT_Uninit 
* 
* Description:
*               This function releases the specified EINT pin that was 
*               initialized by calling Ql_EINT_Init() previously.
*               After releasing, the pin can be used for other purpose.
* Parameters:
*               eintPinName:
*                   EINT pin name, one value of Enum_PinName that has 
*                   the interrupt function.
* Return:        
*               QL_RET_OK, this function succeeds.
*               QL_RET_ERR_PARAM, the input GPIO is invalid. 
*               QL_RET_ERR_USED, the GPIO not used as GPIO, 
*               Maby is used by IIC or SPI,this function can't release it
*               QL_RET_ERR_NOSUPPORTEINT,this pin not supports EINT function.
*****************************************************************/
s32 Ql_EINT_Uninit(Enum_PinName eintPinName);

/*****************************************************************
* Function:     Ql_EINT_GetLevel 
* 
* Description:
*               This function gets the level of the specified EINT pin.
*
* Parameters:
*               eintPinName:
*                   EINT pin name, one value of Enum_PinName that has 
*                   the interrupt function.
* Return:        
*               The level value of the specified EINT pin, which is 
*               nonnegative integer.
*               QL_RET_ERR_PARAM, the input GPIO is invalid. 
*               QL_RET_ERR_NOT_INIT, can't operate,Maybe not Init to EINT .
*               QL_RET_ERR_NOSUPPORTEINT,this pin not supports EINT function.
*****************************************************************/
s32 Ql_EINT_GetLevel(Enum_PinName eintPinName);

/*****************************************************************
* Function:     Ql_EINT_Mask 
* 
* Description:
*               This function masks the specified EINT pin.
*
* Parameters:
*               eintPinName:
*                   EINT pin name, one value of Enum_PinName that has 
*                   the interrupt function.
* Return:        
*               None.
*****************************************************************/
void Ql_EINT_Mask(Enum_PinName eintPinName);

/*****************************************************************
* Function:     Ql_EINT_Unmask 
* 
* Description:
*               This function unmasks the specified EINT pin.
*
* Parameters:
*               eintPinName:
*                   EINT pin name, one value of Enum_PinName that has 
*                   the interrupt function.
* Return:        
*               None.
*****************************************************************/
void Ql_EINT_Unmask(Enum_PinName eintPinName);

#endif  // __QL_EINT_H__
