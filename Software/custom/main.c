#ifdef __CUSTOMER_CODE__
#include "ril.h"
#include "ril_util.h"
#include "ril_sms.h"
#include "ril_telephony.h"
#include "ril_system.h"
#include "ql_stdlib.h"
#include "ql_error.h"
#include "ql_trace.h"
#include "ql_uart.h"
#include "ql_system.h"
#include "ql_memory.h"
#include "ql_timer.h"
#include "ql_fs.h"

#if (defined(__OCPU_RIL_SUPPORT__) && defined(__OCPU_RIL_SMS_SUPPORT__))

#define DEBUG_ENABLE 1
#if DEBUG_ENABLE > 0
#define DEBUG_PORT UART_PORT1
#define DBG_BUF_LEN 512
static char DBG_BUFFER[DBG_BUF_LEN];
#define APP_DEBUG(FORMAT, ...)                                                                                       \
    {                                                                                                                \
        Ql_memset(DBG_BUFFER, 0, DBG_BUF_LEN);                                                                       \
        Ql_sprintf(DBG_BUFFER, FORMAT, ##__VA_ARGS__);                                                               \
        if (UART_PORT2 == (DEBUG_PORT))                                                                              \
        {                                                                                                            \
            Ql_Debug_Trace(DBG_BUFFER);                                                                              \
        }                                                                                                            \
        else                                                                                                         \
        {                                                                                                            \
            Ql_UART_Write((Enum_SerialPort)(DEBUG_PORT), (u8 *)(DBG_BUFFER), Ql_strlen((const char *)(DBG_BUFFER))); \
        }                                                                                                            \
    }
#else
#define APP_DEBUG(FORMAT, ...)
#endif

#define SERIAL_RX_BUFFER_LEN 2048
// Define the UART port and the receive data buffer
static Enum_SerialPort m_myUartPort = UART_PORT1;
static u8 m_RxBuf_Uart1[SERIAL_RX_BUFFER_LEN];
static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void *customizedPara);

#define CLOCK PINNAME_PCM_OUT
#define LATCH PINNAME_PCM_IN
#define DATA PINNAME_PCM_SYNC

#define OUT_1 PINNAME_DTR
#define OUT_2 PINNAME_CTS
#define OUT_3 PINNAME_RTS

#define LSBFIRST 0
#define MSBFIRST 1

union
{
    struct
    {
        u8 out_1 : 1;
        u8 out_2 : 1;
        u8 out_3 : 1;
        u8 out_4 : 1;
        u8 out_5 : 1;
        u8 out_6 : 1;
    } set;
    u8 all;
} Data;

#define IN_1 PINNAME_RXD_AUX

char admin[22];
char users[4][15];

int mode = 10;

enum
{
    NORMAL = 10,
    SET_ADMIN = 15,
};

////work with file//////////////////////////////////////////////////////////////////////////////
#define DATA_FILE_PATH "data.txt"
#define LENGTH 60
s32 handle = -1;
u8 file_content[LENGTH] = {0};
s32 readenLen = 0;

////////////////////////////////////////////////////////////////////////////////////////////////

/***********************************************************************
 * MACRO CONSTANT DEFINITIONS
 ************************************************************************/

#define CON_SMS_BUF_MAX_CNT (1)
#define CON_SMS_SEG_MAX_CHAR (160)
#define CON_SMS_SEG_MAX_BYTE (4 * CON_SMS_SEG_MAX_CHAR)
#define CON_SMS_MAX_SEG (7)

/***********************************************************************
 * STRUCT TYPE DEFINITIONS
 ************************************************************************/
typedef struct
{
    u8 aData[CON_SMS_SEG_MAX_BYTE];
    u16 uLen;
} ConSMSSegStruct;

typedef struct
{
    u16 uMsgRef;
    u8 uMsgTot;

    ConSMSSegStruct asSeg[CON_SMS_MAX_SEG];
    bool abSegValid[CON_SMS_MAX_SEG];
} ConSMSStruct;

/***********************************************************************
 * FUNCTION DECLARATIONS
 ************************************************************************/
static bool ConSMSBuf_IsIntact(ConSMSStruct *pCSBuf, u8 uCSMaxCnt, u8 uIdx, ST_RIL_SMS_Con *pCon);
static bool ConSMSBuf_AddSeg(ConSMSStruct *pCSBuf, u8 uCSMaxCnt, u8 uIdx, ST_RIL_SMS_Con *pCon, u8 *pData, u16 uLen);
static s8 ConSMSBuf_GetIndex(ConSMSStruct *pCSBuf, u8 uCSMaxCnt, ST_RIL_SMS_Con *pCon);
static bool ConSMSBuf_ResetCtx(ConSMSStruct *pCSBuf, u8 uCSMaxCnt, u8 uIdx);

/***********************************************************************
 * GLOBAL DATA DEFINITIONS
 ************************************************************************/
ConSMSStruct g_asConSMSBuf[CON_SMS_BUF_MAX_CNT];

/***********************************************************************
 * MACRO FUNCTION DEFINITIONS
 ************************************************************************/

/*****************************************************************************
 * FUNCTION
 *  ConSMSBuf_ResetCtx
 *
 * DESCRIPTION
 *  This function is used to reset ConSMSBuf context
 *
 * PARAMETERS
 *  <pCSBuf>     The SMS index in storage,it starts from 1
 *  <uCSMaxCnt>  TRUE: The module should reply a SMS to the sender; FALSE: The module only read this SMS.
 *  <uIdx>       Index of <pCSBuf> which will be stored
 *
 * RETURNS
 *  FALSE:   FAIL!
 *  TRUE: SUCCESS.
 *
 * NOTE
 *  1. This is an internal function
 *****************************************************************************/
static bool ConSMSBuf_ResetCtx(ConSMSStruct *pCSBuf, u8 uCSMaxCnt, u8 uIdx)
{
    if ((NULL == pCSBuf) || (0 == uCSMaxCnt) || (uIdx >= uCSMaxCnt))
    {
        APP_DEBUG("Enter ConSMSBuf_ResetCtx,FAIL! Parameter is INVALID. pCSBuf:%x,uCSMaxCnt:%d,uIdx:%d\r\n", pCSBuf, uCSMaxCnt, uIdx);
        return FALSE;
    }

    // Default reset
    Ql_memset(&pCSBuf[uIdx], 0x00, sizeof(ConSMSStruct));

    // TODO: Add special reset here

    return TRUE;
}

/*****************************************************************************
 * FUNCTION
 *  SMS_Initialize
 *
 * DESCRIPTION
 *  Initialize SMS environment.
 *
 * PARAMETERS
 *  VOID
 *
 * RETURNS
 *  TRUE:  This function works SUCCESS.
 *  FALSE: This function works FAIL!
 *****************************************************************************/
static bool SMS_Initialize(void)
{
    s32 iResult = 0;
    u8 nCurrStorage = 0;
    u32 nUsed = 0;
    u32 nTotal = 0;

    // Delete all existed short messages (if needed)
    // iResult = RIL_SMS_DeleteSMS(0, RIL_SMS_DEL_ALL_MSG);
    iResult = RIL_SMS_DeleteSMS(1, RIL_SMS_DEL_ALL_MSG);
    if (iResult != RIL_AT_SUCCESS)
    {
        APP_DEBUG("Fail to delete all messages, iResult=%d,cause:%d\r\n", iResult, Ql_RIL_AT_GetErrCode());
        return FALSE;
    }
    APP_DEBUG("Delete all existed messages\r\n");

    return TRUE;
}

void SMS_TextMode_Send(char *phone, char *msg)
{
    s32 iResult;
    u32 nMsgRef;

    ST_RIL_SMS_SendExt sExt;

    // Initialize
    Ql_memset(&sExt, 0x00, sizeof(sExt));

    APP_DEBUG("< Send Normal Text SMS begin... >\r\n");

    iResult = RIL_SMS_SendSMS_Text(phone, Ql_strlen(phone), LIB_SMS_CHARSET_GSM, msg, Ql_strlen(msg), &nMsgRef);
    if (iResult != RIL_AT_SUCCESS)
    {
        APP_DEBUG("< Fail to send Text SMS, iResult=%d, cause:%d >\r\n", iResult, Ql_RIL_AT_GetErrCode());
        return;
    }
    APP_DEBUG("< Send Text SMS successfully, MsgRef:%u >\r\n", nMsgRef);
}

/*****************************************************************************
 * FUNCTION
 *  Hdlr_RecvNewSMS
 *
 * DESCRIPTION
 *  The handler function of new received SMS.
 *
 * PARAMETERS
 *  <nIndex>     The SMS index in storage,it starts from 1
 *  <bAutoReply> TRUE: The module should reply a SMS to the sender;
 *               FALSE: The module only read this SMS.
 *
 * RETURNS
 *  VOID
 *
 * NOTE
 *  1. This is an internal function
 *****************************************************************************/
static void Hdlr_RecvNewSMS(u32 nIndex, bool bAutoReply)
{
    s32 iResult = 0;
    u32 uMsgRef = 0;
    ST_RIL_SMS_TextInfo *pTextInfo = NULL;
    ST_RIL_SMS_DeliverParam *pDeliverTextInfo = NULL;
    char aPhNum[RIL_SMS_PHONE_NUMBER_MAX_LEN] = {
        0,
    };
    const char aReplyCon[] = {"Module has received SMS."};
    bool bResult = FALSE;

    pTextInfo = Ql_MEM_Alloc(sizeof(ST_RIL_SMS_TextInfo));
    if (NULL == pTextInfo)
    {
        APP_DEBUG("%s/%d:Ql_MEM_Alloc FAIL! size:%u\r\n", sizeof(ST_RIL_SMS_TextInfo), __func__, __LINE__);
        return;
    }
    Ql_memset(pTextInfo, 0x00, sizeof(ST_RIL_SMS_TextInfo));
    iResult = RIL_SMS_ReadSMS_Text(nIndex, LIB_SMS_CHARSET_GSM, pTextInfo);
    if (iResult != RIL_AT_SUCCESS)
    {
        Ql_MEM_Free(pTextInfo);
        APP_DEBUG("Fail to read text SMS[%d], cause:%d\r\n", nIndex, iResult);
        return;
    }

    if ((LIB_SMS_PDU_TYPE_DELIVER != (pTextInfo->type)) || (RIL_SMS_STATUS_TYPE_INVALID == (pTextInfo->status)))
    {
        Ql_MEM_Free(pTextInfo);
        APP_DEBUG("WARNING: NOT a new received SMS.\r\n");
        return;
    }

    pDeliverTextInfo = &((pTextInfo->param).deliverParam);

    APP_DEBUG("<-- RIL_SMS_ReadSMS_Text OK. eCharSet:LIB_SMS_CHARSET_GSM,nIndex:%u -->\r\n", nIndex);
    APP_DEBUG("status:%u,type:%u,alpha:%u,sca:%s,oa:%s,scts:%s,data length:%u\r\n",
              pTextInfo->status,
              pTextInfo->type,
              pDeliverTextInfo->alpha,
              pTextInfo->sca,
              pDeliverTextInfo->oa,
              pDeliverTextInfo->scts,
              pDeliverTextInfo->length);
    APP_DEBUG("data = %s\r\n", (pDeliverTextInfo->data));

    if (mode == SET_ADMIN)
    {
        if (Ql_strcmp((pDeliverTextInfo->data), "admin") == 0)
        {
            APP_DEBUG("admin = %s\r\n", pDeliverTextInfo->oa);
            Ql_strcpy(users[0], pDeliverTextInfo->oa);
            SMS_TextMode_Send(users[0], "now you are admin");
            mode = NORMAL;

            handle = Ql_FS_Open(DATA_FILE_PATH, QL_FS_CREATE_ALWAYS);
            if (handle > 0)
            {
                Ql_FS_Truncate(handle);
                Ql_FS_Flush(handle);
                Ql_FS_Seek(handle, 0, QL_FS_FILE_BEGIN);
                Ql_FS_Write(handle, users[0], Ql_strlen(users[0]), &readenLen);
                Ql_FS_Flush(handle);
                Ql_FS_Close(handle);
            }
            else
            {
                APP_DEBUG("ERROR WRITING File \r\n");
            }
        }
        else
        {
            APP_DEBUG("not equal\r\n");
        }
    }

    {
        APP_DEBUG("sender = %s\r\n", pDeliverTextInfo->oa);
        if (Ql_strncmp((pDeliverTextInfo->oa), users[0], 7) == 0 || Ql_strncmp((pDeliverTextInfo->oa), users[1], 7) == 0 || Ql_strncmp((pDeliverTextInfo->oa), users[2], 7) == 0 || Ql_strncmp((pDeliverTextInfo->oa), users[3], 7) == 0)
        {
            if (Ql_strncmp((pDeliverTextInfo->data), "add user ", 9) == 0)
            {
                if (Ql_strncmp((pDeliverTextInfo->oa), users[0], 13) == 0)
                {
                    if (Ql_strlen(pDeliverTextInfo->data) == 22)
                    {
                        handle = Ql_FS_Open(DATA_FILE_PATH, QL_FS_CREATE);
                        if (handle > 0)
                        {
                            Ql_FS_Seek(handle, 0, QL_FS_FILE_BEGIN);
                            Ql_FS_Read(handle, file_content, LENGTH - 1, &readenLen);
                            APP_DEBUG("file = %s; len :%d\r\n", file_content, readenLen);
                            if ((readenLen / 13) < 4)
                            {
                                APP_DEBUG("start write\r\n");
                                Ql_strcpy(users[(readenLen / 13)], (pDeliverTextInfo->data) + 9);
                                Ql_FS_Seek(handle, 0, QL_FS_FILE_END);
                                Ql_FS_Write(handle, (pDeliverTextInfo->data) + 9, Ql_strlen((pDeliverTextInfo->data) + 9), &readenLen);
                                Ql_FS_Flush(handle);
                                APP_DEBUG("end write\r\n");
                            }
                            else
                            {
                                APP_DEBUG("max users\r\n");
                                SMS_TextMode_Send(pDeliverTextInfo->oa, "max users");
                            }

                            Ql_FS_Close(handle);
                        }
                    }
                    else
                    {
                        APP_DEBUG("incorrect phone number\r\n");
                        SMS_TextMode_Send(pDeliverTextInfo->oa, "incorrect phone number");
                    }
                }
                else
                {
                    APP_DEBUG("You are not admin\r\n");
                    SMS_TextMode_Send(pDeliverTextInfo->oa, "You are not admin");
                }
            }
            if (Ql_strcmp((pDeliverTextInfo->data), "user list") == 0)
            {
                char temp[100];

                Ql_sprintf(temp, "admin : %s\r\n user 1 : %s\r\n user 2 : %s\r\n user 3 : %s\r\n\0", users[0], users[1], users[2], users[3]);
                APP_DEBUG("admin : %s\r\n user 1 : %s\r\n user 2 : %s\r\n user 3 : %s\r\n", users[0], users[1], users[2], users[3]);
                SMS_TextMode_Send(users[0], temp);
            }

            if (Ql_strncmp((pDeliverTextInfo->data), "on out ", 7) == 0)
            {
                APP_DEBUG("on = %d\r\n", pDeliverTextInfo->data[7]);
                if (pDeliverTextInfo->data[7] > 47 && pDeliverTextInfo->data[7] < 58)
                {
                    switch (pDeliverTextInfo->data[7] - 48)
                    {
                    case 1:
                        Data.set.out_1 = 1;
                        break;
                    case 2:
                        Data.set.out_2 = 1;
                        break;
                    case 3:
                        Data.set.out_3 = 1;
                        break;
                    case 4:
                        Data.set.out_4 = 1;
                        break;
                    case 5:
                        Data.set.out_5 = 1;
                        break;
                    case 6:
                        Data.set.out_6 = 1;
                        break;

                    default:
                        break;
                    }
                    update_IO();
                }
            }

            if (Ql_strncmp((pDeliverTextInfo->data), "pulse out ", 10) == 0)
            {
                APP_DEBUG("pulse = %d\r\n", pDeliverTextInfo->data[10]);
                if (pDeliverTextInfo->data[10] > 47 && pDeliverTextInfo->data[10] < 58)
                {
                    switch (pDeliverTextInfo->data[10] - 48)
                    {
                    case 1:
                        pcf8574_setoutputpin(0, 0, 0);
                        Ql_Sleep(400);
                        pcf8574_setoutputpin(0, 0, 1);
                        break;
                    case 2:
                        pcf8574_setoutputpin(0, 1, 0);
                        Ql_Sleep(400);
                        pcf8574_setoutputpin(0, 1, 1);
                        break;
                    case 3:
                        pcf8574_setoutputpin(0, 2, 0);
                        Ql_Sleep(400);
                        pcf8574_setoutputpin(0, 2, 1);
                        break;
                    case 4:
                        pcf8574_setoutputpin(0, 3, 0);
                        Ql_Sleep(400);
                        pcf8574_setoutputpin(0, 3, 1);
                        break;
                    case 5:
                        pcf8574_setoutputpin(0, 4, 0);
                        Ql_Sleep(400);
                        pcf8574_setoutputpin(0, 4, 1);
                        break;
                    case 6:
                        pcf8574_setoutputpin(0, 5, 0);
                        Ql_Sleep(400);
                        pcf8574_setoutputpin(0, 5, 1);
                        break;

                    default:
                        break;
                    }
                }
            }

            if (Ql_strncmp((pDeliverTextInfo->data), "off out ", 8) == 0)
            {
                APP_DEBUG("off = %d\r\n", pDeliverTextInfo->data[8]);
                if (pDeliverTextInfo->data[8] > 47 && pDeliverTextInfo->data[8] < 58)
                {
                    switch (pDeliverTextInfo->data[8] - 48)
                    {
                    case 1:
                        Data.set.out_1 = 0;
                        break;
                    case 2:
                        Data.set.out_2 = 0;
                        break;
                    case 3:
                        Data.set.out_3 = 0;
                        break;
                    case 4:
                        Data.set.out_4 = 0;
                        break;
                    case 5:
                        Data.set.out_5 = 0;
                        break;
                    case 6:
                        Data.set.out_6 = 0;
                        break;

                    default:
                        break;
                    }

                    update_IO();
                }
            }
        }
    }

    Ql_strcpy(aPhNum, pDeliverTextInfo->oa);
    Ql_MEM_Free(pTextInfo);

    return;
}

void proc_subtask1(s32 TaskId)
{
    int status = 0;
    while (TRUE)
    {
        if (mode == SET_ADMIN)
        {
            Ql_GPIO_SetLevel(OUT_1, PINLEVEL_HIGH);
            Ql_GPIO_SetLevel(OUT_2, PINLEVEL_HIGH);
            Ql_GPIO_SetLevel(OUT_3, PINLEVEL_HIGH);
            Ql_Sleep(200);
            Ql_GPIO_SetLevel(OUT_1, PINLEVEL_LOW);
            Ql_GPIO_SetLevel(OUT_2, PINLEVEL_LOW);
            Ql_GPIO_SetLevel(OUT_3, PINLEVEL_LOW);
            Ql_Sleep(200);
        }
        else if (mode == NORMAL)
        {
            status = Ql_GPIO_GetLevel(IN_1);
            if (status == 0)
            {
                Ql_Sleep(2000);
                status = Ql_GPIO_GetLevel(IN_1);
                if (status == 0)
                {
                    mode = SET_ADMIN;
                }
            }
        }
        Ql_Sleep(50);
    }
}

static s32 ReadSerialPort(Enum_SerialPort port, /*[out]*/ u8 *pBuffer, /*[in]*/ u32 bufLen)
{
    s32 rdLen = 0;
    s32 rdTotalLen = 0;
    if (NULL == pBuffer || 0 == bufLen)
    {
        return -1;
    }
    Ql_memset(pBuffer, 0x0, bufLen);
    while (1)
    {
        rdLen = Ql_UART_Read(port, pBuffer + rdTotalLen, bufLen - rdTotalLen);
        if (rdLen <= 0) // All data is read out, or Serial Port Error!
        {
            break;
        }
        rdTotalLen += rdLen;
        // Continue to read...
    }
    if (rdLen < 0) // Serial Port Error!
    {
        APP_DEBUG("Fail to read from port[%d]\r\n", port);
        return -99;
    }
    return rdTotalLen;
}

static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void *customizedPara)
{
    switch (msg)
    {
    case EVENT_UART_READY_TO_READ:
    {
        char *p = NULL;
        s32 totalBytes = ReadSerialPort(port, m_RxBuf_Uart1, sizeof(m_RxBuf_Uart1));
        if (totalBytes <= 0)
        {
            break;
        }

        APP_DEBUG("received %s\r\n", m_RxBuf_Uart1);

        switch (m_RxBuf_Uart1[0])
        {
        case '1':
            Data.set.out_1 = !Data.set.out_1;

            break;
        case '2':
            Data.set.out_2 = !Data.set.out_2;

            break;
        case '3':
            Data.set.out_3 = !Data.set.out_3;

            break;
        case '4':
            Data.set.out_4 = !Data.set.out_4;

            break;
        case '5':
            Data.set.out_5 = !Data.set.out_5;

            break;
        case '6':
            Data.set.out_6 = !Data.set.out_6;

            break;

        default:
            Data.all = 0;
            break;
        }
        update_IO();

        break;
    }
    }
}

static void InitSerialPort(void)
{
    s32 iResult = 0;

    // Register & Open UART port
    iResult = Ql_UART_Register(UART_PORT1, CallBack_UART_Hdlr, NULL);
    if (iResult != QL_RET_OK)
    {
        Ql_Debug_Trace("Fail to register UART port[%d]:%d\r\n", UART_PORT1);
    }

    iResult = Ql_UART_Open(UART_PORT1, 115200, FC_NONE);
    if (iResult != QL_RET_OK)
    {
        Ql_Debug_Trace("Fail to open UART port[%d], baud rate:115200, FC_NONE\r\n", UART_PORT1);
    }
}

void check_file()
{

    handle = Ql_FS_Open(DATA_FILE_PATH, QL_FS_READ_ONLY);
    if (handle > 0)
    {
        Ql_FS_Seek(handle, 0, QL_FS_FILE_BEGIN);
        Ql_FS_Read(handle, file_content, LENGTH - 1, &readenLen);
        Ql_FS_Close(handle);
        Ql_strncpy(users[0], file_content, 13);
        Ql_strncpy(users[1], file_content + 13, 13);
        Ql_strncpy(users[2], file_content + 26, 13);
        Ql_strncpy(users[3], file_content + 39, 13);
        APP_DEBUG("admin : %s\r\n user 1 : %s\r\n user 2 : %s\r\n user 3 : %s\r\n", users[0], users[1], users[2], users[3]);
        APP_DEBUG("file content :%s\r\n", file_content);
    }
    else
    {
        APP_DEBUG("file does not exist \r\n");
    }
}


void proc_main_task(s32 iTaskID)
{
    s32 iResult = 0;
    ST_MSG taskMsg;

    // Register & open UART port
    InitSerialPort();

    APP_DEBUG("SMS Controller (mahdi2001h)\r\n");

    iResult = Ql_IIC_Init(1, PINNAME_RI, PINNAME_DCD, FALSE);
    APP_DEBUG("IIC init %d\r\n", iResult);
    Ql_Sleep(100);
    iResult = Ql_IIC_Config(1, TRUE, 0x40, 100);
    APP_DEBUG("IIC Config %d\r\n", iResult);

    Data.all =0;


    // enable led
    Ql_GPIO_Init(OUT_1, PINDIRECTION_OUT, PINLEVEL_LOW, PINPULLSEL_PULLUP);
    Ql_GPIO_Init(OUT_2, PINDIRECTION_OUT, PINLEVEL_LOW, PINPULLSEL_PULLUP);
    Ql_GPIO_Init(OUT_3, PINDIRECTION_OUT, PINLEVEL_LOW, PINPULLSEL_PULLUP);

    Ql_GPIO_Init(IN_1, PINDIRECTION_IN, PINLEVEL_HIGH, PINPULLSEL_PULLUP);

    Ql_GPIO_SetLevel(OUT_1, PINLEVEL_HIGH);
    Ql_GPIO_SetLevel(OUT_2, PINLEVEL_HIGH);
    Ql_GPIO_SetLevel(OUT_3, PINLEVEL_HIGH);
    Ql_Sleep(200);
    Ql_GPIO_SetLevel(OUT_1, PINLEVEL_LOW);
    Ql_GPIO_SetLevel(OUT_2, PINLEVEL_LOW);
    Ql_GPIO_SetLevel(OUT_3, PINLEVEL_LOW);

    check_file();

    // START MESSAGE LOOP OF THIS TASK
    while (TRUE)
    {
        s32 i = 0;

        Ql_memset(&taskMsg, 0x0, sizeof(ST_MSG));
        Ql_OS_GetMessage(&taskMsg);
        switch (taskMsg.message)
        {
        case MSG_ID_RIL_READY:
        {
            APP_DEBUG("<-- RIL is ready -->\r\n");
            Ql_RIL_Initialize(); // MUST call this function

            for (i = 0; i < CON_SMS_BUF_MAX_CNT; i++)
            {
                ConSMSBuf_ResetCtx(g_asConSMSBuf, CON_SMS_BUF_MAX_CNT, i);
            }

            break;
        }
        case MSG_ID_URC_INDICATION:
            switch (taskMsg.param1)
            {
            case URC_SYS_INIT_STATE_IND:
            {
                APP_DEBUG("<-- Sys Init Status %d -->\r\n", taskMsg.param2);
                if (SYS_STATE_SMSOK == taskMsg.param2)
                {
                    APP_DEBUG("\r\n<-- SMS module is ready -->\r\n");
                    APP_DEBUG("\r\n<-- Initialize SMS-related options -->\r\n");
                    iResult = SMS_Initialize();
                    if (!iResult)
                    {
                        APP_DEBUG("Fail to initialize SMS\r\n");
                    }

                    // SMS_TextMode_Send();
                }
                break;
            }
            case URC_SIM_CARD_STATE_IND:
            {
                APP_DEBUG("\r\n<-- SIM Card Status:%d -->\r\n", taskMsg.param2);
            }
            break;

            case URC_GSM_NW_STATE_IND:
            {
                APP_DEBUG("\r\n<-- GSM Network Status:%d -->\r\n", taskMsg.param2);
                break;
            }

            case URC_GPRS_NW_STATE_IND:
            {
                APP_DEBUG("\r\n<-- GPRS Network Status:%d -->\r\n", taskMsg.param2);
                break;
            }

            case URC_CFUN_STATE_IND:
            {
                APP_DEBUG("\r\n<-- CFUN Status:%d -->\r\n", taskMsg.param2);
                break;
            }

            case URC_COMING_CALL_IND:
            {
                ST_ComingCall *pComingCall = (ST_ComingCall *)(taskMsg.param2);
                APP_DEBUG("\r\n<-- Coming call, number:%s, type:%d -->\r\n", pComingCall->phoneNumber, pComingCall->type);
                break;
            }

            case URC_NEW_SMS_IND:
            {
                APP_DEBUG("\r\n<-- New SMS Arrives: index=%d\r\n", taskMsg.param2);
                Hdlr_RecvNewSMS((taskMsg.param2), FALSE);
                break;
            }

            case URC_MODULE_VOLTAGE_IND:
            {
                APP_DEBUG("\r\n<-- VBatt Voltage Ind: type=%d\r\n", taskMsg.param2);
                break;
            }

            default:
                break;
            }
            break;

        default:
            break;
        }
    }
}

void update_IO()
{
    pcf8574_setoutput(0, ~Data.all);
}

#endif //  __CUSTOMER_CODE__
