/********************************************************************************
  * @file    gsm_engine.c
  * @author  SRIKANTH , Calixto Firmware Team - Updated with new AT commands
  * @version V1.1.0
  * @date    22-August-2022 - Updated with user AT commands
  * @brief   This file provides functions to manage GSM communication with updated AT commands
  *          - Updated AT command sequence for better network connectivity
  *          - Enhanced IMEI processing with AT+CGSN=2
  *          - Improved network registration with signal quality monitoring
  *          - Proper APN configuration for data connection
  *******************************************************************************/

#include "application.h"
#include "json.h"
#include  "middleware.h"
#include "flash.h"

#define GSM_DEBUG_EN         1

#define GSM_MOD_ON				1
#define GSM_MOD_OFF				0

#define GSM_ON                  0
#define GSM_RST                 1
#define GSM_INIT                2
#define GSM_DATA_TXRX			3
#define GSM_GET_TIME			4
#define GSM_FWVR_UP				5
#define GSM_RESTART				6
#define GSM_IDLE                7
#define GSM_TCP_COMM			8
#define GSM_GPS					9
#define GSM_TX_SMS			    10
#define GSM_RX_SMS			    11

#define GSM_CMD1                0
#define GSM_CMD2                1
#define GSM_CMD3                2
#define GSM_CMD4                3
#define GSM_CMD5                4
#define GSM_CMD6                5
#define GSM_CMD7                6
#define GSM_CMD8                7
#define GSM_CMD9                8
#define GSM_CMD10               9
#define GSM_CMD11               10
#define GSM_CMD12               11
#define GSM_CMD13               12

#define GSM_TIM_CMD1			0
#define GSM_TIM_CMD2			1

#define GSM_FWVR_CMD1			0
#define GSM_FWVR_CMD2			1
#define GSM_FWVR_CMD3			2
#define GSM_FWVR_CMD4			3
#define GSM_FWVR_CMD5			4
#define GSM_FWVR_CMD6			5
#define GSM_FWVR_CMD7			6
#define GSM_FWVR_CMD8			7
#define GSM_FWVR_CMD9			8
#define GSM_FWVR_CMD10			9

#define RESP_AT_CONNECT     	0
#define RESP_OK             	1
#define RESP_ERROR          	2
#define RESP_FAIL           	3
#define RESP_SEND_OK        	4
#define RESP_HTTPACTION     	5
#define RESP_HTTPREAD       	6
#define RESP_CPIN_READY     	7
#define RESP_CREG        		8
#define RESP_STATE        		9
#define RESP_CME_ERROR      	10
#define RESP_NO_CARRIER     	11
#define RESP_RDY         		12

#define ENGINE_TIM_MAX      	2000//50
#define ENGINE_TIM_MIN      	500

#define MAX_HTTP_DATA_LENGTH  	3000

// Global variables
uint16_t engine_tim_limit   = 0;
uint16_t engine_wait_tim    = 0;
uint8_t  gsm_device_state   = GSM_ON;
uint8_t  gsm_init_state     = 0;
uint8_t  gsm_get_time_state = 0;
uint8_t  gsm_fwvr_up_state  = 0;
uint8_t  gsm_tcp_comm_state = 0;
uint8_t  gsm_gps_state      = 0;
uint8_t  gsm_tx_sms_state   = 0;
uint8_t  gsm_rx_sms_state   = 0;

uint8_t  gsm_response_flag  = 0;
uint8_t  gsm_engine_initdone= 0;
uint8_t  gsm_engine_tick    = 0;
uint8_t  gsm_reset_count    = 0;
uint8_t  gsm_response_val   = 0;

uint8_t  time_get_set_flag  = 0;
uint8_t  imei_flag          = 0;
uint8_t  modem_type_get     = 0;
uint8_t  modem_type         = QUECTEL_MODEM;

extern uint8_t cloud_comm_start;

// Updated AT Commands based on user specification
unsigned char Command1[]       = "AT\r\0";                                    // Basic connectivity test
unsigned char Command2[]       = "ATE0\r\0";                                  // Disable echo
unsigned char Command3[]       = "AT+CGMR\r\0";                               // Get manufacturer revision
unsigned char Command4[]       = "AT+CFUN=1\r\0";                             // Enable full functionality
unsigned char Command5[]       = "AT+CSQ\r\0";                                // Check signal quality
unsigned char Command6[]       = "AT+CREG=1\r\0";                             // Network registration
unsigned char Command7[]       = "AT+COPS?\r\0";                              // Check network operator
unsigned char Command8[]       = "AT+COPS=4,2,\"40445\",7\r\0";               // Set network operator with auto fallback
unsigned char Command9[]       = "AT+CGSN=2\r\0";                             // Get IMEISV
unsigned char Command10[]      = "AT+CGCMOD=?\r\0";                           // Check CID associated with active context
unsigned char Command11[]      = "AT+CGATT=1\r\0";                            // Attach to packet domain service
unsigned char Command12[]      = "AT+CGACT=1,1\r\0";                          // Activate PDP context
unsigned char Command13[]      = "AT+CGDCONT=1,\"IP\",\"airtel\"\r\0";        // Set APN for data connection

// Time and other commands
unsigned char Commandtim1[]    = "AT+CCLK?\r\0";
unsigned char Commandtim2[]    = "AT+CTZU=1\r\0";

// TCP commands for Quectel modem
unsigned char q_tcp_command1[25] = "AT+QIDNSIP=1\r\0";
unsigned char q_tcp_command2[]   = "AT+QIMUX=0\r\0";
unsigned char q_tcp_command3[]   = "AT+QIREGAPP\r\0";
unsigned char q_tcp_command4[]   = "AT+QIACT\r\0";
unsigned char q_tcp_command5[]   = "AT+QIACT?\r\0";
unsigned char q_tcp_command6[]   = "AT+QIOPEN=\"TCP\",\"59.144.164.216\",7273\r\0";
unsigned char q_tcp_command7[20] = "AT+QISEND=5\r\0";
unsigned char q_tcp_command8[]   = "AT+QICLOSE\r\0";

// GPS commands
unsigned char q_gps_command1[]   = "AT+QGNSSC=1\r\0";
unsigned char q_gps_command2[]   = "AT+QGNSSRD=\"NMEA/GGA\"\r\0";
unsigned char q_gps_command3[]   = "AT+QGNSSRD=\"NMEA/RMC\"\r\0";

// SMS commands
unsigned char tx_sms_command1[]  = "AT+CSMS=1\r\0";
unsigned char tx_sms_command2[]  = "AT+CMGF=1\r\0";

// Buffer variables
uint8_t imei_char[20];
uint8_t imei_num_bytes[20];

// Function prototypes
void gsm_engine(void);
void gsm_init_engine(void);
void gsm_on(void);
void gsm_reset(void);
void gsm_restart(void);
void gsm_idle(void);
void gsm_init_flags(void);
void gsm_send_command(unsigned char *gsm_command);
void gsm_response_process(uint8_t response_type);
void gsm_get_time_engine(void);
void quectel_gsm_tcp_engine(void);
void gps_engine(void);
void gsm_tx_sms_engine(void);
void gsm_rx_sms_engine(void);

/********************************************************************************
  * @brief  GSM Engine Main Function with updated AT command sequence
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_engine(void)
{
    gsm_engine_tick++;

    switch(gsm_device_state)
    {
        case GSM_ON:
            gsm_on();
            gsm_engine_initdone = 0;
            gsm_init_state = GSM_CMD1;
            break;

        case GSM_INIT:
            gsm_init_engine();
            break;

        case GSM_GET_TIME:
            gsm_get_time_engine();
            break;

        case GSM_FWVR_UP:
            // Firmware update handling
            break;

        case GSM_RESTART:
            gsm_engine_initdone = 0;
            gsm_restart();
            break;

        case GSM_IDLE:
            gsm_idle();
            break;

        case GSM_TCP_COMM:
            quectel_gsm_tcp_engine();
            break;

        case GSM_GPS:
            gps_engine();
            break;

        case GSM_TX_SMS:
            gsm_tx_sms_engine();
            break;

        case GSM_RX_SMS:
            gsm_rx_sms_engine();
            break;

        default:
            gsm_device_state = GSM_RESTART;
            break;
    }
}

/********************************************************************************
  * @brief  GSM Initialization Engine with updated AT commands
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_init_engine(void)
{
    switch(gsm_init_state)
    {
        case GSM_CMD1:    // AT - Basic connectivity test
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command1);
            gsm_response_process(RESP_OK);
            break;

        case GSM_CMD2:    // ATE0 - Disable echo
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command2);
            gsm_response_process(RESP_OK);
            break;

        case GSM_CMD3:    // AT+CGMR - Get manufacturer revision
            modem_type_get = 1;
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command3);
            gsm_response_process(RESP_OK);
            break;

        case GSM_CMD4:    // AT+CFUN=1 - Enable full functionality
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command4);
            gsm_response_process(RESP_OK);
            break;

        case GSM_CMD5:    // AT+CSQ - Check signal quality
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command5);
            gsm_response_process(RESP_OK);
            break;

        case GSM_CMD6:    // AT+CREG=1 - Enable network registration status
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command6);
            gsm_response_process(RESP_OK);
            break;

        case GSM_CMD7:    // AT+COPS? - Query current network operator
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command7);
            gsm_response_process(RESP_OK);
            break;

        case GSM_CMD8:    // AT+COPS=4,2,"40445",7 - Set network operator with auto fallback
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command8);
            gsm_response_process(RESP_OK);
            break;

        case GSM_CMD9:    // AT+CGSN=2 - Get IMEISV
            imei_flag = 1;
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command9);
            gsm_response_process(RESP_OK);
            break;

        case GSM_CMD10:   // AT+CGCMOD=? - Check CID associated with active context
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command10);
            gsm_response_process(RESP_OK);
            break;

        case GSM_CMD11:   // AT+CGATT=1 - Attach to packet domain service
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command11);
            gsm_response_process(RESP_OK);
            break;

        case GSM_CMD12:   // AT+CGACT=1,1 - Activate PDP context for CID 1
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command12);
            gsm_response_process(RESP_OK);
            break;

        case GSM_CMD13:   // AT+CGDCONT=1,"IP","airtel" - Set APN for data connection
            engine_tim_limit = ENGINE_TIM_MAX;
            gsm_send_command(Command13);
            gsm_response_process(RESP_OK);
            // Initialization complete, move to next phase
            if(time_get_set_flag == 1)
            {
                time_get_set_flag = 0;
                gsm_device_state = GSM_GET_TIME;
                gsm_init_state = GSM_CMD1;
            }
            else
            {
                gsm_device_state = GSM_IDLE;
                gsm_init_state = GSM_CMD1;
            }
            cloud_comm_start = 1;
            break;

        default:
            gsm_device_state = GSM_RESTART;
            gsm_reset_count = 0;
            break;
    }
}

/********************************************************************************
  * @brief  GSM Response Processing with enhanced parsing for new commands
  * @param  response_type: Expected response type
  * @retval none
  *****************************************************************************/
void gsm_response_process(uint8_t response_type)
{
    extern uint8_t gsm_serialRXBuff[];
    extern uint16_t gsm_RxBuffWRPtr;
    
    uint16_t i = 0, j = 0, k = 0;
    uint16_t tmp_gsm_RxBuffWRPtr = gsm_RxBuffWRPtr;

    engine_wait_tim++;

    if(engine_wait_tim >= engine_tim_limit)
    {
        gsm_response_flag = 0;
        engine_wait_tim = 0;
        gsm_device_state = GSM_RESTART;
        gsm_reset_count = 0;
        return;
    }

    for(i = 0; i < tmp_gsm_RxBuffWRPtr; i++)
    {
        // Process manufacturer revision response from AT+CGMR
        if(strstr((char*)gsm_serialRXBuff, "+CGMR:") != NULL && modem_type_get == 1)
        {
            #ifdef GSM_DEBUG_EN
            printf("\r\nModem Revision Info Received\r\n");
            #endif
            modem_type_get = 0;
            gsm_response_flag = 1;
            i = tmp_gsm_RxBuffWRPtr + 1;
        }
        // Process signal quality response from AT+CSQ
        else if(strstr((char*)gsm_serialRXBuff, "+CSQ:") != NULL)
        {
            int signal_strength = 0, bit_error_rate = 0;
            if(sscanf((char*)gsm_serialRXBuff, "+CSQ: %d,%d", &signal_strength, &bit_error_rate) == 2)
            {
                #ifdef GSM_DEBUG_EN
                printf("\r\nSignal Strength: %d, BER: %d", signal_strength, bit_error_rate);
                if(signal_strength >= 20) {
                    printf(" - EXCELLENT\r\n");
                } else if(signal_strength >= 15) {
                    printf(" - GOOD\r\n");
                } else if(signal_strength >= 10) {
                    printf(" - FAIR\r\n");
                } else {
                    printf(" - POOR\r\n");
                }
                #endif
            }
            gsm_response_flag = 1;
            i = tmp_gsm_RxBuffWRPtr + 1;
        }
        // Process network registration response from AT+CREG
        else if(strstr((char*)gsm_serialRXBuff, "+CREG:") != NULL)
        {
            int n = 0, stat = 0;
            if(sscanf((char*)gsm_serialRXBuff, "+CREG: %d,%d", &n, &stat) == 2)
            {
                #ifdef GSM_DEBUG_EN
                printf("\r\nNetwork Registration Status: ");
                switch(stat) {
                    case 0: printf("NOT_SEARCHING\r\n"); break;
                    case 1: printf("HOME_NETWORK\r\n"); break;
                    case 2: printf("SEARCHING\r\n"); break;
                    case 3: printf("DENIED\r\n"); break;
                    case 5: printf("ROAMING\r\n"); break;
                    default: printf("UNKNOWN\r\n"); break;
                }
                #endif
            }
            gsm_response_flag = 1;
            i = tmp_gsm_RxBuffWRPtr + 1;
        }
        // Process IMEI response from AT+CGSN=2
        else if(imei_flag == 1)
        {
            if(strstr((char*)gsm_serialRXBuff, "+CGSN:") != NULL)
            {
                char *imei_start = strchr((char*)gsm_serialRXBuff, '"');
                if(imei_start != NULL)
                {
                    imei_start++;  // Move past the opening quote
                    char *imei_end = strchr(imei_start, '"');
                    if(imei_end != NULL)
                    {
                        int imei_len = imei_end - imei_start;
                        if(imei_len < 16)  // IMEI is typically 15 digits
                        {
                            strncpy((char*)imei_num_bytes, imei_start, imei_len);
                            imei_num_bytes[imei_len] = '\0';
                            imei_num_conv(imei_num_bytes);
                            #ifdef GSM_DEBUG_EN
                            printf("\r\nIMEI: %s\r\n", imei_num_bytes);
                            #endif
                            imei_flag = 0;
                            gsm_response_flag = 1;
                            i = tmp_gsm_RxBuffWRPtr + 1;
                        }
                    }
                }
            }
        }
        // Process CGCMOD response
        else if(strstr((char*)gsm_serialRXBuff, "+CGCMOD:") != NULL)
        {
            #ifdef GSM_DEBUG_EN
            printf("\r\nCGCMOD Response Received\r\n");
            #endif
            gsm_response_flag = 1;
            i = tmp_gsm_RxBuffWRPtr + 1;
        }
        // Process standard OK response
        else if((gsm_serialRXBuff[i] == 'K') && (gsm_serialRXBuff[i-1] == 'O'))
        {
            switch(response_type)
            {
                case RESP_OK:
                    gsm_init_state = gsm_init_state + 1;
                    gsm_response_flag = 1;
                    engine_wait_tim = 0;
                    i = tmp_gsm_RxBuffWRPtr + 1;
                    break;

                default:
                    break;
            }
        }
        // Process ERROR response
        else if(strstr((char*)gsm_serialRXBuff, "ERROR") != NULL)
        {
            gsm_response_flag = 0;
            engine_wait_tim = 0;
            gsm_device_state = GSM_RESTART;
            i = tmp_gsm_RxBuffWRPtr + 1;
        }
    }

    if(gsm_response_flag == 1)
    {
        gsm_init_flags();
        engine_wait_tim = 0;
        gsm_response_flag = 0;
    }
}

/********************************************************************************
  * @brief  GSM Send Command Function
  * @param  gsm_command: Command to send
  * @retval none
  *****************************************************************************/
void gsm_send_command(unsigned char *gsm_command)
{
    extern void SEND_GSM_BUFF(unsigned char *data, uint16_t len);
    
    #ifdef GSM_DEBUG_EN
    printf("\r\nTX: %s", gsm_command);
    #endif
    
    SEND_GSM_BUFF(gsm_command, strlen((char*)gsm_command));
}

/********************************************************************************
  * @brief  GSM Initialize Flags
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_init_flags(void)
{
    extern uint8_t gsm_serialRXBuff[];
    extern uint16_t gsm_RxBuffWRPtr;
    
    memset(gsm_serialRXBuff, 0, sizeof(gsm_serialRXBuff));
    gsm_RxBuffWRPtr = 0;
}

/********************************************************************************
  * @brief  GSM Turn On Function
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_on(void)
{
    GSM_MODEM_LDO_ON;
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gsm_device_state = GSM_INIT;
}

/********************************************************************************
  * @brief  GSM Reset Function
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_reset(void)
{
    GSM_MODEM_LDO_OFF;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    GSM_MODEM_LDO_ON;
    vTaskDelay(2000 / portTICK_PERIOD_MS);
}

/********************************************************************************
  * @brief  GSM Restart Function
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_restart(void)
{
    gsm_reset_count++;
    if(gsm_reset_count >= 3)
    {
        gsm_reset_count = 0;
        gsm_reset();
    }
    gsm_device_state = GSM_ON;
}

/********************************************************************************
  * @brief  GSM Idle Function
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_idle(void)
{
    // GSM module is in idle state, waiting for commands
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

/********************************************************************************
  * @brief  GSM Get Time Engine (placeholder)
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_get_time_engine(void)
{
    // Time synchronization implementation
    gsm_device_state = GSM_IDLE;
}

/********************************************************************************
  * @brief  Quectel GSM TCP Engine (placeholder)
  * @param  none
  * @retval none
  *****************************************************************************/
void quectel_gsm_tcp_engine(void)
{
    // TCP communication implementation
}

/********************************************************************************
  * @brief  GPS Engine (placeholder)
  * @param  none
  * @retval none
  *****************************************************************************/
void gps_engine(void)
{
    // GPS processing implementation
}

/********************************************************************************
  * @brief  GSM TX SMS Engine (placeholder)
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_tx_sms_engine(void)
{
    // SMS transmission implementation
}

/********************************************************************************
  * @brief  GSM RX SMS Engine (placeholder)
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_rx_sms_engine(void)
{
    // SMS reception implementation
}

// External function declaration for IMEI conversion
extern void imei_num_conv(uint8_t *imei_data);