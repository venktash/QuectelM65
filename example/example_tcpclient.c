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
 *   example_tcpclient.c
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   This example demonstrates how to establish a TCP connection, when the module 
 *   is used for the client. Input the specified command through any serial port 
 *   and the result will be output through the debug port.We have adopted a timeout 
 *   mechanism,if in the process of connecting socket or getting the TCP socket ACK 
 *   number overtime 90s, the socket will be close and the network will be deactivated.
 *   In most of TCPIP functions,  return -2(QL_SOC_WOULDBLOCK) doesn't indicate failed.
 *   It means app should wait, till the callback function is called.
 *   The app can get the information of success or failure in callback function.
 *   Get more info about return value. Please read the "OPEN_CPU_DGD" document.
 *
 * Usage:
 * ------
 *   Compile & Run:
 *
 *     Set "C_PREDEF=-D __EXAMPLE_TCPCLIENT__" in gcc_makefile file. And compile the 
 *     app using "make clean/new".
 *     Download image bin to module to run.
 * 
 *   Operation:
 *            
 *     step1: set APN parameter.
 *            Command: Set_APN_Param=<APN>,<username>,<password>
 *     step2: set server parameter, which is you want to connect.
 *            Command:Set_Srv_Param=<srv ip>,<srv port>
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
#ifdef __EXAMPLE_TCPCLIENT__  
#include "custom_feature_def.h"
#include "ql_stdlib.h"
#include "ql_common.h"
#include "ql_type.h"
#include "ql_trace.h"
#include "ql_error.h"
#include "ql_uart.h"
#include "ql_uart.h"
#include "ql_gprs.h"
#include "ql_socket.h"
#include "ql_wtd.h"
#include "ql_timer.h"
#include "ril_sim.h"
#include "ril_network.h"
#include "ril.h"
#include "ril_util.h"
#include "ril_telephony.h"

#define DEBUG_ENABLE 1
#if DEBUG_ENABLE > 0
#define DEBUG_PORT  UART_PORT1
#define DBG_BUF_LEN   512
static char DBG_BUFFER[DBG_BUF_LEN];
#define APP_DEBUG(FORMAT,...) {\
    Ql_memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    Ql_sprintf(DBG_BUFFER,FORMAT,##__VA_ARGS__); \
    if (UART_PORT2 == (DEBUG_PORT)) \
    {\
        Ql_Debug_Trace(DBG_BUFFER);\
    } else {\
        Ql_UART_Write((Enum_SerialPort)(DEBUG_PORT), (u8*)(DBG_BUFFER), Ql_strlen((const char *)(DBG_BUFFER)));\
    }\
}
#else
#define APP_DEBUG(FORMAT,...) 
#endif



/*****************************************************************
* define process state
******************************************************************/
typedef enum{
    STATE_NW_GET_SIMSTATE,
    STATE_NW_QUERY_STATE,
    STATE_GPRS_REGISTER,
    STATE_GPRS_CONFIG,
    STATE_GPRS_ACTIVATE,
    STATE_GPRS_ACTIVATING,
    STATE_GPRS_GET_DNSADDRESS,
    STATE_GPRS_GET_LOCALIP,
    STATE_CHACK_SRVADDR,
    STATE_SOC_IP_GETTING,
   
    STATE_SOC_SEND,
    STATE_SOC_SENDING,
    STATE_SOC_ACK,
    STATE_SOC_CLOSE,
    STATE_GPRS_DEACTIVATE,
    STATE_SERV_LOGIN,
    STATE_TOTAL_NUM    
}Enum_TCPSTATE;
static u8 m_tcp_state = STATE_NW_GET_SIMSTATE;
/**************************************************************

*//////////////////////////////////////////////////////////////
//#define HTTP_REQUEST  1  // 0=http-get, 1=http-post, 2=http-file

//
// http url address
//
//#if (HTTP_REQUEST == 0)
// url for http-get
//#define HTTP_URL_ADDR   "http://www.quectel.com/\0"
//#elif (HTTP_REQUEST == 1)
// url for http-post
#define HTTP_URL_ADDR   "http://smartled.in/CMS/v2.php?" //
//#define HTTP_URL_ADDR   "http://ptsv2.com/t/qa4us-1659522538/post" //
//#elif (HTTP_REQUEST == 2)
// url for http-file
//#define HTTP_URL_ADDR   "http://124.74.xxx.xxx:5015/index.html\0"
//#endif


//#define HTTP_POST_MSG   "868277046799511,0,0,0,0,1,12.70,00.00,000.0,00.00,00.00,000.0,12.70,00.94,011.9,123.0,000,015,100,04,08,21,0,0,0,0,1,12.70,00.00,000.0,00.00,00.00,000.0,12.70,00.94,011.9,123.0,000,015,100,04,08,21,,#"
                        

u8 arrHttpRcvBuf[10*1024]; // 10K buffer for http data

/*****************************************************************
* UART Param
******************************************************************/
#define SERIAL_RX_BUFFER_LEN  512
static u8 m_RxBuf_Uart[SERIAL_RX_BUFFER_LEN];


int a2i(char* t_imei);
char Imei_m[16]="";

u32 rssi;           // varable for signal strength
u32 ber; 
s32 iRet = 0;
char strCCID[30];   // variable for CCID -SIM ID 20 DIGIT



/*****************************************************************
* timer param
******************************************************************/
#define TCP_TIMER_ID         TIMER_ID_USER_START
#define TIMEOUT_90S_TIMER_ID TIMER_ID_USER_START + 1   //timeout

#define TCP_TIMER_PERIOD     800
#define TIMEOUT_90S_PERIOD   90000

static s32 timeout_90S_monitor = FALSE;

/*****************************************************************
* APN Param
******************************************************************/
static u8 m_apn[MAX_GPRS_APN_LEN] = "M2MISAFE";        //for VI //"airtelgprs.com";
static u8 m_userid[MAX_GPRS_USER_NAME_LEN] = "";
static u8 m_passwd[MAX_GPRS_PASSWORD_LEN] = "";

static ST_GprsConfig  m_gprsCfg;

/*****************************************************************
* Server Param
******************************************************************/
#define SRVADDR_BUFFER_LEN  100
#define SEND_BUFFER_LEN     1024
#define RECV_BUFFER_LEN     2048

static u8 S_strngth[2];
static u8 m_send_buf[SEND_BUFFER_LEN];
static u8 m_recv_buf[RECV_BUFFER_LEN];
static u64 m_nSentLen  = 0;      // Bytes of number sent data through current socket    

static u8 m_transition_buf[SEND_BUFFER_LEN];

static u8  m_SrvADDR[SRVADDR_BUFFER_LEN] = "13.234.46.70\0";
//static u8  m_SrvADDR[SRVADDR_BUFFER_LEN] = "104.238.125.74\0"; 
//static u8  m_SrvADDR[SRVADDR_BUFFER_LEN] = 80;
static u32 m_SrvPort = 8008;

static u8  m_ipaddress[5];  //only save the number of server ip, remove the comma

static s32 m_socketid = -1; 

static s32 m_remain_len = 0;     // record the remaining number of bytes in send buffer.
static char *m_pCurrentPos = NULL; 

/*****************************************************************
* GPRS and socket callback function
******************************************************************/

void Callback_GPRS_Actived(u8 contexId, s32 errCode, void* customParam);
void CallBack_GPRS_Deactived(u8 contextId, s32 errCode, void* customParam );
void Callback_GetIpByName(u8 contexId, u8 requestId, s32 errCode,  u32 ipAddrCnt, u32* ipAddr);


ST_PDPContxt_Callback     callback_gprs_func = 
{
    Callback_GPRS_Actived,
    CallBack_GPRS_Deactived
};

s32  HTTP_Program(u8 get_post);
/*****************************************************************
* uart callback function
******************************************************************/
static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara);

/*****************************************************************
* timer callback function
******************************************************************/
static void Callback_Timer(u32 timerId, void* param);

/*****************************************************************
* other subroutines
******************************************************************/
extern s32 Analyse_Command(u8* src_str,s32 symbol_num,u8 symbol, u8* dest_buf);
static void proc_handle(char *pData,s32 len);


static s32 ret;
void proc_main_task(s32 taskId)
{
    s32 ret;
    ST_MSG msg;


    // Register & open UART port
    Ql_UART_Register(UART_PORT1, CallBack_UART_Hdlr, NULL);
    Ql_UART_Open(UART_PORT1, 9600, FC_NONE);
    
    APP_DEBUG("<--OpenCPU: TCP Client.-->\r\n");

    //register & start timer 
    Ql_Timer_Register(TCP_TIMER_ID, Callback_Timer, NULL);
    Ql_Timer_Start(TCP_TIMER_ID, TCP_TIMER_PERIOD, TRUE);

    Ql_Timer_Register(TIMEOUT_90S_TIMER_ID, Callback_Timer, NULL);
    timeout_90S_monitor = FALSE;
    
    while(TRUE)
    {
        Ql_OS_GetMessage(&msg);
        switch(msg.message)
        {
#ifdef __OCPU_RIL_SUPPORT__
        case MSG_ID_RIL_READY:
            APP_DEBUG("<-- RIL is ready -->\r\n");
            Ql_RIL_Initialize();
            break;
#endif
        default:
            break;
        }
    }
}

static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
{
	 char *p = NULL;
     char *q = NULL;

     //APP_DEBUG("this is in UART handler function !!!!!!!");
    switch (msg)
    {
    case EVENT_UART_READY_TO_READ:
        {

           s32 totalBytes = ReadSerialPort(port, m_RxBuf_Uart, sizeof(m_RxBuf_Uart));
           if (totalBytes > 0)
           {
			   p = Ql_strstr(m_RxBuf_Uart,"***");
			   if(p)
			   {
                
				  totalBytes = Ql_sprintf(m_send_buf,"f=%s%s\0",Imei_m,(m_RxBuf_Uart+3)); // need to add gps location on garbage and time for utc format as per sheet 
                 // Ql_sprintf(m_transition_buf,"%s",(m_RxBuf_Uart+2)); // need to add gps location on garbage and time for utc format as per sheet 
                  //  m_transition_buf[totalBytes-6]=NULL   ;
                   q = Ql_strstr(m_send_buf,"#");
                   Ql_sprintf(q,"%d,%s#",rssi,strCCID);

                  // *q=NULL;
                 // m_send_buf=m_transition_buf;
                    APP_DEBUG("m_send_buf =%s",m_send_buf);            
                                                                     // 
				 // proc_handle(m_transition_buf,sizeof(m_transition_buf));

			   }
               
           }
           break;
        }
    case EVENT_UART_READY_TO_WRITE:
        break;
    default:
        break;
    }
}

static void proc_handle(char *pData,s32 len)
{
    char *p = NULL;
    s32 iret;
    u8 srvport[10];

    //command: Set_APN_Param=<APN>,<username>,<password>
//     p = Ql_strstr(pData,"Set_APN_Param=");
//     if (p)
//     {
//         Ql_memset(m_apn, 0, MAX_GPRS_APN_LEN);
//         if (Analyse_Command(pData, 1, '>', m_apn))
//         {
//             APP_DEBUG("<--APN Parameter Error.-->\r\n");
//             return;
//         }
//         Ql_memset(m_userid, 0, MAX_GPRS_USER_NAME_LEN);
//         if (Analyse_Command(pData, 2, '>', m_userid))
//         {
//             APP_DEBUG("<--APN Username Parameter Error.-->\r\n");
//             return;
//         }
//         Ql_memset(m_passwd, 0, MAX_GPRS_PASSWORD_LEN);
//         if (Analyse_Command(pData, 3, '>', m_passwd))
//         {
//             APP_DEBUG("<--APN Password Parameter Error.-->\r\n");
//             return;
//         }
        
//         APP_DEBUG("<--Set APN Parameter Successfully<%s>,<%s>,<%s>.-->\r\n",m_apn,m_userid,m_passwd);

//         return;
//     }
    
    //command: Set_Srv_Param=<srv ip>,<srv port>
    p = Ql_strstr(pData,"Set_Srv_Param=");
    if (p)
    {
        Ql_memset(m_SrvADDR, 0, SRVADDR_BUFFER_LEN);
        if (Analyse_Command(pData, 1, '>', m_SrvADDR))
        {
            APP_DEBUG("<--Server Address Parameter Error.-->\r\n");
            return;
        }
        Ql_memset(srvport, 0, 10);
        if (Analyse_Command(pData, 2, '>', srvport))
        {
            APP_DEBUG("<--Server Port Parameter Error.-->\r\n");
            return;
        }
        m_SrvPort = Ql_atoi(srvport);
        APP_DEBUG("<--Set TCP Server Parameter Successfully<%s>,<%d>.-->\r\n",m_SrvADDR,m_SrvPort);

        m_tcp_state = STATE_NW_GET_SIMSTATE;
        APP_DEBUG("<--Restart the TCP connection process.-->\r\n");

        return;
    }

    //if not command,send it to server
    m_pCurrentPos = m_send_buf;
    Ql_strncpy(m_pCurrentPos + m_remain_len, pData,len);
    
    m_remain_len = Ql_strlen(m_pCurrentPos);
    
}

/*
static void checkErr_AckNumber(s32 err_code)
{
    if(SOC_INVALID_SOCKET == err_code)
    {
        APP_DEBUG("<-- Invalid socket ID -->\r\n");
    }
    else if(SOC_INVAL == err_code)
    {
        APP_DEBUG("<-- Invalid parameters for ACK number -->\r\n");
    }
    else if(SOC_ERROR == err_code)
    {
        APP_DEBUG("<-- Unspecified error for ACK number -->\r\n");
    }
    else
    {
        // get the socket option successfully
    }
}
*/

static void Callback_Timer(u32 timerId, void* param)
{
    //s32 wtdid;

    if (TIMEOUT_90S_TIMER_ID == timerId)
    {
        APP_DEBUG("<--90s time out!!!-->\r\n");
        APP_DEBUG("<-- Close socket.-->\r\n");
        
        Ql_SOC_Close(m_socketid);
        m_socketid = -1;

        m_tcp_state = STATE_GPRS_DEACTIVATE;

        timeout_90S_monitor = FALSE;
    }
    else if (TCP_TIMER_ID == timerId)
    {
        //APP_DEBUG("<--...........m_tcp_state=%d..................-->\r\n",m_tcp_state);
        switch (m_tcp_state)
        {
            case STATE_NW_GET_SIMSTATE:
            {
                s32 simStat = 0;
                RIL_SIM_GetSimState(&simStat);
                if (simStat == SIM_STAT_READY)
                {
                    m_tcp_state = STATE_NW_QUERY_STATE;
                    APP_DEBUG("<--SIM card status is normal!-->\r\n");
                }else
                {
                //    Ql_Timer_Stop(TCP_TIMER_ID);
                    APP_DEBUG("<--SIM card status is unnormal!-->\r\n");
                }
                break;
            }        
            case STATE_NW_QUERY_STATE:
            {
                s32 creg = 0;
                s32 cgreg = 0;

                ret = RIL_NW_GetGSMState(&creg);
                ret = RIL_NW_GetGPRSState(&cgreg);
                APP_DEBUG("<--Network State:creg=%d,cgreg=%d-->\r\n",creg,cgreg);
                if((cgreg == NW_STAT_REGISTERED)||(cgreg == NW_STAT_REGISTERED_ROAMING))
                {
                    /**********  SIGNAL QUALITY  *********************/ 
                     s32 nRet = RIL_NW_GetSignalQuality(&rssi, &ber);
                     APP_DEBUG("<-- Signal strength:%d, BER:%d -->\r\n", rssi, ber);
                     Ql_sprintf(S_strngth,"%d",rssi);
                     APP_DEBUG("s_strngth=%s",S_strngth);


                     /*********   SIM CCID/SIM ID   ******************/ 
                      Ql_memset(strCCID, 0x0, sizeof(strCCID));
	                  iRet = RIL_SIM_GetCCID(strCCID);
                      APP_DEBUG("<-- CCID:%s, iRet=%d -->\r\n", strCCID, iRet);
                    //m_tcp_state = STATE_GPRS_REGISTER;
                    m_tcp_state = STATE_SERV_LOGIN;
                }
                break;
            }
           /* case STATE_GPRS_REGISTER:
            {
                ret = Ql_GPRS_Register(0, &callback_gprs_func, NULL);
                if (GPRS_PDP_SUCCESS == ret)
                {
                    APP_DEBUG("<--Register GPRS callback function successfully.-->\r\n");
                    m_tcp_state = STATE_GPRS_CONFIG;
                }else if (GPRS_PDP_ALREADY == ret)
                {
                    APP_DEBUG("<--GPRS callback function has already been registered,ret=%d.-->\r\n",ret);
                    m_tcp_state = STATE_GPRS_CONFIG;
                }else
                {
                    APP_DEBUG("<--Register GPRS callback function failure,ret=%d.-->\r\n",ret);
                }
                break;
            }
            case STATE_GPRS_CONFIG:
            {
                Ql_strcpy(m_gprsCfg.apnName, m_apn);
                Ql_strcpy(m_gprsCfg.apnUserId, m_userid);
                Ql_strcpy(m_gprsCfg.apnPasswd, m_passwd);
                m_gprsCfg.authtype = 0;
                ret = Ql_GPRS_Config(0, &m_gprsCfg);
                if (GPRS_PDP_SUCCESS == ret)
                {
                    APP_DEBUG("<--configure GPRS param successfully.-->\r\n");
                }else
                {
                    APP_DEBUG("<--configure GPRS param failure,ret=%d.-->\r\n",ret);
                }
                
                m_tcp_state = STATE_GPRS_ACTIVATE;
                break;
            }
            case STATE_GPRS_ACTIVATE:
            {
                m_tcp_state = STATE_GPRS_ACTIVATING;
                ret = Ql_GPRS_Activate(0);
                if (ret == GPRS_PDP_SUCCESS)
                {
                    APP_DEBUG("<--Activate GPRS successfully.-->\r\n");
                    m_tcp_state = STATE_GPRS_GET_DNSADDRESS;
                }else if (ret == GPRS_PDP_WOULDBLOCK)
                {
                     APP_DEBUG("<--Waiting for the result of GPRS activated.,ret=%d.-->\r\n",ret);
                    //waiting Callback_GPRS_Actived
                }else if (ret == GPRS_PDP_ALREADY)
                {
                    APP_DEBUG("<--GPRS has already been activated,ret=%d.-->\r\n",ret);
                    m_tcp_state = STATE_GPRS_GET_DNSADDRESS;
                }else//error
                {
                    APP_DEBUG("<--Activate GPRS failure,ret=%d.-->\r\n",ret);
                    m_tcp_state = STATE_GPRS_ACTIVATE;
                }
                break;
            }*/
          /*  case STATE_GPRS_GET_DNSADDRESS:
            {            
                u8 primaryAddr[16] = {0};
                u8 bkAddr[16] = {0};
                ret =Ql_GPRS_GetDNSAddress(0, (u32*)primaryAddr,  (u32*)bkAddr);
                if (ret == GPRS_PDP_SUCCESS)
                {
                    APP_DEBUG("<--Get DNS address successfully,primaryAddr=%d.%d.%d.%d,bkAddr=%d.%d.%d.%d-->\r\n",primaryAddr[0],primaryAddr[1],primaryAddr[2],primaryAddr[3],bkAddr[0],bkAddr[1],bkAddr[2],bkAddr[3]);            
                    m_tcp_state = STATE_GPRS_GET_LOCALIP;
                }else
                {
                     APP_DEBUG("<--Get DNS address failure,ret=%d.-->\r\n",ret);
                    m_tcp_state = STATE_GPRS_DEACTIVATE;
                }
                break;
            }*/
           /* case STATE_GPRS_GET_LOCALIP:
            {
                u8 ip_addr[5];
                Ql_memset(ip_addr, 0, 5);
                ret = Ql_GPRS_GetLocalIPAddress(0, (u32 *)ip_addr);
                if (ret == GPRS_PDP_SUCCESS)
                {
                    APP_DEBUG("<--Get Local Ip successfully,Local Ip=%d.%d.%d.%d-->\r\n",ip_addr[0],ip_addr[1],ip_addr[2],ip_addr[3]);
                    
                   // m_tcp_state = STATE_SOC_SEND;
                }else
                {
                    APP_DEBUG("<--Get Local Ip failure,ret=%d.-->\r\n",ret);
                }
                break;
            }*/
          /*
            case STATE_CHACK_SRVADDR:
            {
                Ql_memset(m_ipaddress,0,5);
                ret = Ql_IpHelper_ConvertIpAddr(m_SrvADDR, (u32 *)m_ipaddress);
                if(ret == SOC_SUCCESS) // ip address, xxx.xxx.xxx.xxx
                {
                    APP_DEBUG("<--Convert Ip Address successfully,m_ipaddress=%d,%d,%d,%d-->\r\n",m_ipaddress[0],m_ipaddress[1],m_ipaddress[2],m_ipaddress[3]);
                    m_tcp_state = STATE_SOC_SEND;
                    
                }else  //domain name
                {
                    ret = Ql_IpHelper_GetIPByHostName(0, 0, m_SrvADDR, Callback_GetIpByName);
                    if(ret == SOC_SUCCESS)
                    {
                        APP_DEBUG("<--Get ip by hostname successfully.-->\r\n");
                    }
                    else if(ret == SOC_WOULDBLOCK)
                    {
                        APP_DEBUG("<--Waiting for the result of Getting ip by hostname,ret=%d.-->\r\n",ret);
                        //waiting CallBack_getipbyname
						m_tcp_state = STATE_SOC_IP_GETTING;
					}
                    else
                    {
                        APP_DEBUG("<--Get ip by hostname failure:ret=%d-->\r\n",ret);
                        if(ret == SOC_BEARER_FAIL)  
                        {
                             m_tcp_state = STATE_GPRS_DEACTIVATE;
                        }
                        else
                        {
                             m_tcp_state = STATE_GPRS_GET_DNSADDRESS;
                        } 
                    }
                }
                break;
            }
            ////////////////////////////////////////////////////////
            case STATE_SOC_CONNECT:
            {
                m_tcp_state = STATE_SOC_CONNECTING;
                ret = Ql_SOC_Connect(m_socketid,(u32) m_ipaddress, m_SrvPort);
                if(ret == SOC_SUCCESS)
                {
                    APP_DEBUG("<--The socket is already connected.-->\r\n");
                    m_tcp_state = STATE_SERV_LOGIN;
                    
                }else if(ret == SOC_WOULDBLOCK)
                {
                      if (!timeout_90S_monitor)//start timeout monitor
                      {
                        Ql_Timer_Start(TIMEOUT_90S_TIMER_ID, TIMEOUT_90S_PERIOD, FALSE);
                        timeout_90S_monitor = TRUE;
                      }
                      APP_DEBUG("<--Waiting for the result of socket connection,ret=%d.-->\r\n",ret);
                      //waiting CallBack_getipbyname
                      
                }else //error
                {
                    APP_DEBUG("<--Socket Connect failure,ret=%d.-->\r\n",ret);
                    APP_DEBUG("<-- Close socket.-->\r\n");
                    Ql_SOC_Close(m_socketid);
                    m_socketid = -1;
                    
                    if(ret == SOC_BEARER_FAIL)  
                    {
                        m_tcp_state = STATE_GPRS_DEACTIVATE;
                    }
                    else
                    {
                        m_tcp_state = STATE_GPRS_GET_DNSADDRESS;
                    }
                }
                break;
            }*/
            case STATE_SOC_SEND:
            {
                 //Ql_WTD_Feed(&wtdid);
                if (!Ql_strlen(m_send_buf))//no data need to send  m_send_buf
                    break;
                
                //m_tcp_state = STATE_SOC_SENDING;
                
                do
                {
                    ret = HTTP_Program(1);
                    //APP_DEBUG("<--Send data,socketid=%d,number of bytes sent=%d-->\r\n",m_socketid,ret);
                    if(ret == 0)//send compelete
                    {
                        m_remain_len = 0;
                        m_pCurrentPos = NULL;
                        m_nSentLen += ret;
                       // m_tcp_state = STATE_SOC_ACK;
                        break;
                    }
                    else if((ret <= 0) && (ret == SOC_WOULDBLOCK)) 
                    {
                        //waiting CallBack_socket_write, then send data;     
                        break;
                    }
                    else if(ret <= 0)
                    {
                        APP_DEBUG("<--Send data failure,ret=%d.-->\r\n",ret);
                        APP_DEBUG("<-- Close socket.-->\r\n");
                        Ql_SOC_Close(m_socketid);//error , Ql_SOC_Close
                        m_socketid = -1;

                        m_remain_len = 0;
                        m_pCurrentPos = NULL; 
                        if(ret == SOC_BEARER_FAIL)  
                        {
                            m_tcp_state = STATE_GPRS_DEACTIVATE;
                        }
                        else
                        {
                            m_tcp_state = STATE_GPRS_GET_DNSADDRESS;
                        }
                        break;
                    }
                    else if(ret < m_remain_len)//continue send, do not send all data
                    {
                        m_remain_len -= ret;
                        m_pCurrentPos += ret; 
                        m_nSentLen += ret;
                    }
                }while(1);
                break;
            }
          /*  case STATE_SOC_ACK:
            {
                u64 ackedNumCurr;
                ret = Ql_SOC_GetAckNumber(m_socketid, &ackedNumCurr);
                if (ret < 0)
                {
                    checkErr_AckNumber(ret);
                }
                if (m_nSentLen == ackedNumCurr)
                {
                    if (timeout_90S_monitor) //stop timeout monitor
                    {
                        Ql_Timer_Stop(TIMEOUT_90S_TIMER_ID);
                        timeout_90S_monitor = FALSE;
                    }
                    
                    APP_DEBUG("<-- ACK Number:%llu/", m_nSentLen);
                    APP_DEBUG("%llu. Server has received all data. -->\r\n",ackedNumCurr);

                    Ql_memset(m_send_buf,0,SEND_BUFFER_LEN);
                    m_tcp_state = STATE_SOC_SEND;
                }
                else
                {
                    if (!timeout_90S_monitor)//start timeout monitor
                    {
                        Ql_Timer_Start(TIMEOUT_90S_TIMER_ID, TIMEOUT_90S_PERIOD, FALSE);
                        timeout_90S_monitor = TRUE;
                    }
                    
                    APP_DEBUG("<-- ACK Number:%llu/", ackedNumCurr);
                    APP_DEBUG("%llu from socket[%d] -->\r\n",  m_nSentLen, m_socketid);
				}
                break;
            }*/
            case STATE_GPRS_DEACTIVATE:
            {
                APP_DEBUG("<--Deactivate GPRS.-->\r\n");
                Ql_GPRS_Deactivate(0);
                break;
            }
            case STATE_SERV_LOGIN:
            {
                //  ret = Ql_WTD_Init(0, PINNAME_NETLIGHT, 600);
                 if (0 == ret)
                      {
                          APP_DEBUG("\r\n<--OpenCPU: watchdog init OK!-->\r\n");         
                      }
                //wtdid = Ql_WTD_Start(500);
                    char pchData[30];                                    
                    char read_imei[16];   // to read imei 
                    //s32 imei_pswrd=0;  // stored pswrd int form
                    s32 sum = 0;
                     s32 j=0;
                     s32 digit, i; 
                  
                    Ql_memset(pchData, 0x0, sizeof(pchData));
                    RIL_GetIMEI(read_imei);
                    
                    s32 len=Ql_strlen(read_imei);
                   // APP_DEBUG("t_imei_len=%d",len)
                    for (i = 0; i < len; i++)
                    {
                        if((read_imei[i]<=0x39) && (read_imei[i]>=0x30))
                        {
                        Imei_m[j]=read_imei[i];
                        digit = read_imei[i] - 0x30;
                        sum = sum + digit; 
                        j++;
                        //APP_DEBUG("t_imei_len=%d",j);
                        }
                    }
                    Imei_m[j]='\0';
                   
                    m_tcp_state = STATE_SOC_SEND;
            }
            default:
                break;
        }    
    }
}




static u32 m_rcvDataLen = 0;
static void HTTP_RcvData(u8* ptrData, u32 dataLen, void* reserved)
{
    APP_DEBUG("<-- Data coming on http, total len:%d -->\r\n", m_rcvDataLen + dataLen);
    if ((m_rcvDataLen + dataLen) <= sizeof(arrHttpRcvBuf))
    {
        Ql_memcpy((void*)(arrHttpRcvBuf + m_rcvDataLen), (const void*)ptrData, dataLen);
    } else {
        if (m_rcvDataLen < sizeof(arrHttpRcvBuf))
        {// buffer is not enough
            u32 realAcceptLen = sizeof(arrHttpRcvBuf) - m_rcvDataLen;
            Ql_memcpy((void*)(arrHttpRcvBuf + m_rcvDataLen), (const void*)ptrData, realAcceptLen);
            APP_DEBUG("<-- Rcv-buffer is not enough, discard part of data (len:%d/%d) -->\r\n", dataLen - realAcceptLen, dataLen);
        } else {// No more buffer
            APP_DEBUG("<-- No more buffer, discard data (len:%d) -->\r\n", dataLen);
        }
    }
    m_rcvDataLen += dataLen;
}



s32 HTTP_Program(u8 http_action)
{
    s32 ret;
    u32 readDataLen = 0;
    
   // Set PDP context
   


    ret = RIL_NW_SetGPRSContext(Ql_GPRS_GetPDPContextId());
    APP_DEBUG("<-- Set GPRS PDP context, ret=%d -->\r\n", ret);

    // Set APN
    ret = RIL_NW_SetAPN(1, "M2MISAFE;", "", "");
    APP_DEBUG("<-- Set GPRS APN, ret=%d -->\r\n", ret);

    // Open/Activate PDP context
    ret = RIL_NW_OpenPDPContext();
    APP_DEBUG("<-- Open PDP context, ret=%d -->\r\n", ret);

    //Set HTTP server address (URL)
    ret = RIL_HTTP_SetServerURL(HTTP_URL_ADDR, Ql_strlen(HTTP_URL_ADDR));
    APP_DEBUG("<-- Set http server URL, ret=%d -->\r\n", ret);

    // Send get/post request
    m_rcvDataLen = 0;
    if (0 == http_action)
    {
        // get-request
        ret = RIL_HTTP_RequestToGet(100);   // 100s timetout
        APP_DEBUG("<-- Send get-request, ret=%d -->\r\n", ret);

        // Read response from server
        ret = RIL_HTTP_ReadResponse(120, HTTP_RcvData);
        APP_DEBUG("<-- Read http response data, ret=%d, dataLen=%d -->\r\n", ret, m_rcvDataLen);
    }
    else if (1 == http_action)
    {
        
        
        ret = RIL_HTTP_RequestToPost( m_send_buf,  Ql_strlen((char*)m_send_buf));
        APP_DEBUG("<-- Send post-request, postMsg=%s, ret=%d --size=>> %dr\n", (char*)m_send_buf, ret,Ql_strlen((char*)m_send_buf));

        // Read response from server
        ret = RIL_HTTP_ReadResponse(120, HTTP_RcvData);
        APP_DEBUG("<-- Read http response data, ret=%d, dataLen=%d,  response=%s-->\r\n", ret, m_rcvDataLen,HTTP_Rcv);
         Ql_memset(m_send_buf,0,SEND_BUFFER_LEN);
    }
    // else if (2 == http_action){
    //     // get-request
    //     ret = RIL_HTTP_RequestToGet(100);   // 100s timetout
    //     APP_DEBUG("<-- Send get-request, ret=%d -->\r\n", ret);

    //     // Download file from http server
    //     ret = RIL_HTTP_DownloadFile(RAM_FILE_NAME, RAM_FILE_SIZE, Callback_HTTP_DwnldFile);
    //     APP_DEBUG("<-- Download file from http server, ret=%d -->\r\n", ret);
    // }
    
    // Close PDP context
    ret = RIL_NW_ClosePDPContext();
    
    APP_DEBUG("<-- Close PDP context, ret=%d -->\r\n", ret);
    return ret;
 }


void Callback_GPRS_Actived(u8 contexId, s32 errCode, void* customParam)
{
    if(errCode == SOC_SUCCESS)
    {
        APP_DEBUG("<--CallBack: active GPRS successfully.-->\r\n");
        m_tcp_state = STATE_GPRS_GET_DNSADDRESS;
    }else
    {
        APP_DEBUG("<--CallBack: active GPRS successfully,errCode=%d-->\r\n",errCode);
        m_tcp_state = STATE_GPRS_ACTIVATE;
    }      
}





void CallBack_GPRS_Deactived(u8 contextId, s32 errCode, void* customParam )
{
	m_nSentLen  = 0;

    if (errCode == SOC_SUCCESS)
    {
        APP_DEBUG("<--CallBack: deactived GPRS successfully.-->\r\n"); 
        m_tcp_state = STATE_NW_GET_SIMSTATE;
    }else
    {
        APP_DEBUG("<--CallBack: deactived GPRS failure,(contexid=%d,error_cause=%d)-->\r\n",contextId,errCode); 
    }
}

#endif // __EXAMPLE_TCPCLIENT__

