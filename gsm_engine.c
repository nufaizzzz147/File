/********************************************************************************
  * @file    gsm_engine.c
  * @author  SRIKANTH , Calixto Firmware Team
  * @version V1.0.0
  * @date    22-August-2022
  * @brief   This file provides functions to manage the following
  *          breif the functionalities here:
  *           - function 1 ->gsm_RxBuffClear() clearing the gsm RX buffer
  *           - function 2 ->gsm_response_process() this checks the response or failure in the gsm
  *           - function 3 ->gsm_on() turns ON the GSM module
  *           - function 4 ->gsm_reset() resets the GSM module
  *           - function 5 ->gsm_init()	initializes the GSM module
  *           - function 6 ->gsm_send_command() sends the command to the cloud
  *           - function 7 ->gsm_init_flags() resets the flags each time the gsm engine sends a command and receives the response
  *           - function 8 ->gsm_http_txrx() transmits and receives the http commands
  *           - function 9 ->http_idle() the engine rests
  *           - function 10->gsm_get_time() used to change the state of the GSM for getting the time from the server
  *
  *  @verbatim
  *
  *          ===================================================================
  *                             Working of chip/peripheral/Algorithm
  *          ===================================================================
  *
  *          Heading No.1
  *          =============
  *          Explanation
  *
  *          Heading No.2
  *          =============
  *          Explanation
  *
  *          ===================================================================
  *                              How to use this driver / source
  *          ===================================================================
  *            -
  *            -
  *            -
  *            -
  *
  *  @endverbatim
  *
  ******************************************************************************
  *
  * <h2><center>&copy; COPYRIGHT 2015 Calixto Systems Pvt Ltd</center></h2>
  ******************************************************************************
  */

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
#define GSM_FWVR_CMD11          10
#define GSM_FWVR_CMD12          11
#define GSM_FWVR_CMD13          12
#define GSM_FWVR_CMD14          13
#define GSM_FWVR_CMD15          14

#define GSM_TCP_1             0
#define GSM_TCP_2             1
#define GSM_TCP_3             2
#define GSM_TCP_4             3
#define GSM_TCP_5             4
#define GSM_TCP_6             5
#define GSM_TCP_7             6
#define GSM_TCP_8             7
#define GSM_TCP_9             8
#define GSM_TCP_10            9
#define GSM_TCP_11             10



#define GPS_CMD1			  0
#define GPS_CMD2			  1
#define GPS_CMD3              2

#define GSM_TX_SMS_1 				0
#define GSM_TX_SMS_2				1
#define GSM_TX_SMS_3				2
#define GSM_TX_SMS_4				3
#define GSM_TX_SMS_5				4
#define GSM_TX_SMS_6				5

#define GSM_RX_SMS_1 				0
#define GSM_RX_SMS_2				1
#define GSM_RX_SMS_3				2
#define GSM_RX_SMS_4				3
#define GSM_RX_SMS_5				4
#define GSM_RX_SMS_6				5
#define GSM_RX_SMS_7				6





#define RESP_OK                 0
#define RESP_HTTP_OK            1
#define RESP_OFF				2
#define RESP_RESET				3
#define RESP_TIM_OK				4
#define RESP_FWVR_OK			5
#define RESP_TCP_OK             6
#define RESP_GPS_OK				7
#define RESP_TCP_DATA           8
#define RESP_TX_SMS_OK  		9
#define RESP_RX_SMS_OK  		10
#define Q_RESP_FWVR_OK          11


#define MAX_ERR_GSM				3

#define ENGINE_TIM_MAX			900
#define ENGINE_TIM_MIN			20
#define ENGINE_TIM_20S			200

#define GSM_SERIAL_BUFF_MIN	    20

#define SUCCESS 			  1
#define FAILURE				  0

uint32_t gsm_reset_count       = 0;
uint8_t gps_data[150];

#define SMS_GET_VERSION_STRING    	 "VERSION"
#define SMS_GET_STATUS_STRING      	 "STATUS"
#define SMS_GET_URL_STRING           "GPSURL"
#define SMS_SET_APN_STRING           "APN"
#define SMS_GET_APN_STRING           "APNGET"
#define SMS_SET_SERVER_STRING		 "SERVER"
#define SMS_GET_SERVER_STRING		 "SERVERGET"
#define SMS_SET_SPEED_STRING		 "SPEED"
#define SMS_GET_SPEED_STRING		 "SPEEDGET"
#define SMS_SET_TIMER_STRING		 "TIMER"
#define SMS_GET_TIMER_STRING		 "TIMERGET"
#define SMS_SET_CLEAR_STRING		 "CLEAR"
#define SMS_SET_FACTORY_STRING		 "FACTORY"
#define SMS_SET_REBOOT_STRING		 "REBOOT"
#define SMS_SET_FTP_STRING			 "FOTA"
#define SMS_GET_FTP_STRING           "FOTAGET"

#define SMS_GET_VERSION_TYPE    	1
#define SMS_GET_STATUS_TYPE      	2
#define SMS_GET_URL_TYPE            3
#define SMS_SET_APN_TYPE            4
#define SMS_GET_APN_TYPE            5
#define SMS_SET_SERVER_TYPE		    6
#define SMS_GET_SERVER_TYPE		    7
#define SMS_SET_SPEED_TYPE		    8
#define SMS_GET_SPEED_TYPE		    9
#define SMS_SET_TIMER_TYPE		    10
#define SMS_GET_TIMER_TYPE		    11
#define SMS_SET_CLEAR_TYPE		    12
#define SMS_SET_FACTORY_TYPE		13
#define SMS_SET_REBOOT_TYPE		    14
#define SMS_SET_FTP_TYPE 			15
#define SMS_GET_FTP_TYPE			16

uint8_t *gsm_data_json = (uint8_t*)"{\"DVID\":\"XXXXXXXXXXXXXXX\"}";

unsigned char Command1[]       = "AT\r\0";
unsigned char Command2[]       = "ATE0\r\0";
unsigned char Command3[]       = "ATI\r\0";
unsigned char Command4[]       = "AT+CREG=1\r\0";
unsigned char Command5[]       = "AT+CREG?\r\0";
unsigned char Command6[]       = "AT+CSQ\r\0";
unsigned char Command7[]       = "AT+COPS?\r\0";
unsigned char Command8[140]    = "AT+CSTT=\"letstrackgprs\"\r\0"; //airtelgprs.com	   //letstrackgprs
unsigned char Command8a[140]   = "AT+QICSGP=1,\"airtelgprs.com\"\r\0";
unsigned char Command9[]       = "AT+GSN\r\0";


unsigned char Commandtim1[]	   = "AT+CCLK?\r\0";


unsigned char Commandfwvr0[]    = "AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\0";
unsigned char Commandfwvr1[]    = "AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"\r\0";
unsigned char Commandfwvr2[]	= "AT+SAPBR=1,1\r\0";
unsigned char Commandfwvr3[]	= "AT+FTPTYPE=\"I\"\r\0";
unsigned char Commandfwvr4[]    = "AT+FTPMODE=0\r\0";                  // I - bin   A-ASCII
unsigned char Commandfwvr5[200]	= "AT+FTPSERV=\"ftp02-02.letstrack.com\"\r\0";    // 185.27.134.11 - ip
unsigned char Commandfwvr6[200]	= "AT+FTPUN=\"ftp02-02.letstrack.com|ftp01-01\"\r\0";
unsigned char Commandfwvr7[200]	= "AT+FTPPW=\"GDTEswfte!@33221@\"\r\0";
unsigned char Commandfwvr8[]    = "AT+FTPPORT=21\r\0";
unsigned char Commandfwvr9[] 	= "AT+FTPGETNAME=\"EVIOT00100.bin\"\r\0";
unsigned char Commandfwvr10[] 	= "AT+FTPGETPATH=\"/\"\r\0";
unsigned char Commandfwvr11[]	= "AT+FTPGET=1\r\0";
unsigned char Commandfwvr12[]	= "AT+FTPGET=2,1024\r\0";
unsigned char Commandfwvr13[]	= "AT+FTPQUIT\r\0";


unsigned char q_Commandfwvr0[]    = "AT+QFTPUSER=\"ftp01-01.letstrack.com|ftp01-01\"\r\0";
unsigned char q_Commandfwvr1[]    = "AT+QFTPPASS=\"GDTEswfte!@33221@\"\r\0";
unsigned char q_Commandfwvr2[]     = "AT+QFTPOPEN=\"ftp01-01.letstrack.com\",21\r\0";
unsigned char q_Commandfwvr3[]     = "AT+QFTPPATH=\"/\"\r\0";
unsigned char q_Commandfwvr4[]     = "AT+QFTPGET=\"EVIOT00100.bin\",0,1024\r\0";
unsigned char q_Commandfwvr5[]     = "AT+QFTPCLOSE\r\0";


unsigned char q_tcp_command1[25]="AT+QIDNSIP=1\r\0";
unsigned char q_tcp_command2[]="AT+QIMUX=0\r\0";
unsigned char q_tcp_command3[]="AT+QIREGAPP\r\0";
unsigned char q_tcp_command4[]="AT+QIACT\r\0";
unsigned char q_tcp_command5[]="AT+QIACT?\r\0";
unsigned char q_tcp_command6[]="AT+QIOPEN=\"TCP\",\"59.144.164.216\",7273\r\0";
unsigned char q_tcp_command7[20]="AT+QISEND=5\r\0";
unsigned char q_tcp_command8[]="AT+QICLOSE\r\0";

unsigned char tcp_command1[]="AT+ISLKVRSCAN\r\0";
unsigned char tcp_command2[]="AT+CGMI\r\0";
unsigned char tcp_command3[]="AT+CIICR\r\0";
unsigned char tcp_command4[]="AT+CIFSR\r\0";                                                                    //gpsdevice.letstrack.com 10401
unsigned char tcp_command5[]="AT+CIPSTART=\"TCP\",\"61.246.2.50\",2404\r\0";  //ip and port      7273 // echo.u-blox.com // 59.144.164.216 //61.246.2.50 2404
unsigned char tcp_command6[]="AT+CIPSEND=127\r\0";  // data length
unsigned char tcp_command6a[]="AT+CIPSEND?\r\0";
unsigned char tcp_command7[]="AT+CIPCLOSE\r\0";
unsigned char tcp_command8[]="AT+CIPCCFG=8,2,300,1\r\0";
unsigned char tcp_command9[]="AT+CIPMODE=1\r\0";

unsigned char gps_command1[]="AT+MGPSC=1\r\0";
unsigned char gps_command2[]="AT+GETGPS=\"GNGGA\"\r\0";
unsigned char gps_command3[]="AT+GETGPS=\"GNRMC\"\r\0";

unsigned char q_gps_command1[]="AT+QGNSSC=1\r\0";
unsigned char q_gps_command2[]="AT+QGNSSRD=\"NMEA/GGA\"\r\0";
unsigned char q_gps_command3[]="AT+QGNSSRD=\"NMEA/RMC\"\r\0";

unsigned char tx_sms_command1[]="AT+CSMS=1\r\0";
unsigned char tx_sms_command2[]="AT+CMGF=1\r\0";
unsigned char tx_sms_command2a[]="AT+CSCA?\r\0";
unsigned char tx_sms_command3[]="AT+CMGS=\"+919003837207\"";
uint8_t       tx_sms_command4[1]={0x0D};
unsigned char tx_sms_command5[]="866911125648749:Gps:fixedby14satellitesnormalcommunication";
uint8_t       tx_sms_command6[1]={0X1A};


unsigned char rx_sms_command1[]="AT+CMGL=\"REC UNREAD\"\r\0";
unsigned char rx_sms_command2[]="AT+CMGR=0\r\0";
unsigned char rx_sms_command3[]="AT+CMGD=2,0\r\0";

uint8_t gps_url_tx_sms[]="http://maps.google.com/maps?q=";
uint8_t fw_ver[]="FW:V1.3-21/11/2022#";

uint8_t gsm_tcp_state=GSM_TCP_1;
uint8_t gps_state=0;
uint8_t gps_data_flag=0;

uint8_t gsm_device_state = GSM_RESTART;//GSM_IDLE;
uint8_t gsm_init_state   = 0;

uint8_t gsm_txrx_state = 0;
uint8_t gsm_command_flag  = 0;
uint32_t gsm_engine_tick   = 0;
extern uint8_t gsm_response_flag, gsm_response_data_start_flag;
extern char gsm_serialRXBuff[GSM_SERIAL_BUFF_MAX];
extern uint8_t gsm_fwup_flag;
extern uint8_t gsm_failure_flag;

extern char csq_val[];


unsigned char rest_pos_char[12];

extern uint16_t rxbuff_len; // = 20;
uint16_t engine_tim_limit = 20;

uint8_t gsm_tim_flag=0;
uint8_t gsm_tim_state=0;

static uint8_t gsm_firmware_state = 0;


uint8_t reset_eng_cnt = 0;
uint8_t cloud_error_reset = 0;

uint8_t gsm_intrupt_flag;
uint8_t http_resp_return;

uint8_t gsm_fwup_intrpt_data = 0;

extern uint16_t gsm_RxBuffWRPtr;

extern char cloud_settime[];

uint8_t gsm_engine_initdone = 0;
uint8_t gsm_time_got_flag   = 0;
uint8_t time_engine_on      = 0;
uint8_t data_engine_on      = 0;

uint8_t time_engine_start_flag = 0;
uint8_t data_engine_start_flag = 0;

uint8_t gps_count=0;
uint8_t tcp_data_flag=0;
extern uint8_t cloud_tcp_tx_data[190];

extern uint8_t cloud_comm_start;
extern uint8_t send_tcp_data;
extern uint16_t cloud_commn_timer ;

uint8_t time_get_set_flag =1;
uint8_t tcp_server_connect=0;
uint8_t flash_store_started=0;
uint8_t gsm_reception=0;
uint8_t flash_data[190],ret_val1=0;
uint8_t current_data=0;
uint8_t imei_flag =0;
uint8_t imei_num_bytes[16];
uint8_t gsm_firmware_up=0;
uint8_t ftp_rxvd_flag=0;
uint8_t ftp_var=0;
uint8_t rtc_time_set_flag=0;
uint8_t gsm_time_get_flag=0;
uint8_t fw_update_started=0;
uint8_t ftp_write_data_flag=0;
uint8_t ftp_read_data_flag=0;
uint32_t gsm_firmware_count=0;
extern bool ota_update_approved;
uint8_t close_tcp_port=0;
uint8_t tcp_port=0;
extern uint8_t printf_buffer[400];
uint8_t gsm_tx_sms_state=0;
uint8_t gsm_rx_sms_state=0;
extern uint8_t gsm_sms_rxvd;

uint8_t gsm_sms_intr_recvd_flag=0;
uint8_t index_data_sms_flag=0;
uint8_t gsm_rx_sms_read_index[5];
uint8_t gsm_rx_sms_mobile_num[15];
uint8_t gsm_rx_sms_data[512];
uint8_t gsm_rx_msg_type=0;
uint8_t temp_sms_data1[32];
uint8_t temp_sms_data2[32];
uint8_t temp_sms_data3[45];
uint8_t temp_sms_data4[32];
uint8_t temp_sms_data5[64];
uint8_t temp_sms_data6[32];

uint8_t gsm_tx_sms_data[512];
uint8_t gsm_tx_sms_flag=0;
extern float lattitude_gps;
extern float longitude_gps;
extern uint8_t gps_lat_ind[2];
extern uint8_t gps_lon_ind[2];
uint8_t temp_buff[30];
uint16_t gsm_tx_sms_data_len=0;
extern uint8_t n_sat_in_use;
extern char final_bat_buff[10];
uint8_t restart_chip=0;
uint8_t reset_modem=0;
uint8_t gps_sms_rx_flag=0;
uint8_t get_gps_location_flag=0;
extern uint8_t bms_type[2];
extern uint8_t server_conn_flag;
uint8_t server_connected_update=0;
uint8_t modem_type_get=0;
uint8_t modem_type=0;
uint8_t q_ftp_response=0;
uint8_t q_ftp_data=0;
uint8_t q_ftp_data_tx_cmplt=0;
uint32_t q_ftp_get_bytes=0;

extern uint8_t ftp_error_bytes;
uint8_t flash_read_started=0;
extern uint8_t close_ftp_flag;
extern uint8_t gsm_apn_info[140];
extern uint8_t gsm_server_info[42];
extern uint8_t gsm_ftp_info[192];
extern uint8_t gsm_overspeed_info[20];
extern uint8_t gsm_srvr_comm_interval_info[15];
extern uint8_t over_spd_ind;
extern uint16_t over_speed_ind_km;
extern uint8_t over_speed_ind_to;
extern uint8_t cloud_comm_interval_moving;
extern uint8_t cloud_comm_interval_stopped;
extern uint8_t tcp_data_sent_get_can_data_flag;
uint8_t firmware_update_initiated=0;
uint8_t gsm_tx_sms_count=0;
extern uint8_t gsm_transmit_sms_after_ota_done;
uint8_t ftp_status_sms_indiction=0;

/*******************************************************************************/

/* Private function prototypes -----------------------------------------------*/
unsigned char gsm_response_process(unsigned char response_no);
void gsm_on(void);
void gsm_init_engine(void);
void gsm_send_command(unsigned char *gprs_command);
void gsm_init_flags(void);
void gsm_idle(void);
void gsm_get_time_engine(void);
void gsm_fwvr_up_engine(void);
void gsm_engine(void);
void gsm_receive_process(void);
void gsm_tcp_engine(void);
void gps_engine(void);
void data_send_to_server(void);
void gsm_tx_sms_engine(void);
void gsm_rx_sms_engine(void);
void decode_gsm_rx_sms_process(uint8_t *data,uint16_t data_len);
void rx_sms_command_data(uint8_t *data);
uint8_t rx_sms_get_type(uint8_t *data);
void rx_sms_store_process(uint8_t command);
void quectel_gsm_tcp_engine(void);
void quectel_gsm_fwvr_up_engine(void);
void transfer_data_to_server(uint8_t type);
void encode_config_data_to_store_in_flash(uint8_t num_data,uint8_t *data0,uint8_t *data1,uint8_t *data2,uint8_t *data3,uint8_t *data4,uint8_t *encrypted_data);
void decode_data_from_flash(uint8_t type,uint8_t *encrypted_data);
void lynq_ftp_reception(void);
void quectel_ftp_reception(void);
void tcp_data_reading_from_spi_flash(void);
void tcp_data_writing_to_spi_flash(void);

/* Private functions----------------------------------------------------------*/

void gsm_restart(void)
{


	gsm_reset_count++;


	if(gsm_reset_count == 10)
	{
		GSM_MODEM_LDO_OFF
	}
	if(gsm_reset_count == 110)
	{
		GSM_MODEM_LDO_ON
	}
	
	if (gsm_reset_count == 120)
	{

		gsm_device_state = GSM_ON;
		gsm_init_state   = GSM_CMD1;
		gsm_tcp_state =GSM_TCP_1;
		gps_state=GPS_CMD1;
		gsm_tim_state=GSM_TIM_CMD1;
		gsm_firmware_state = GSM_FWVR_CMD1;
		gsm_tx_sms_state =GSM_TX_SMS_1;
		gsm_rx_sms_state =GSM_RX_SMS_1;

		gps_count=0;
		gps_data_flag=0;
		gsm_reset_count  = 0;
		gsm_firmware_up=0;
		time_engine_on   = 0;
		data_engine_on   = 0;
		gsm_firmware_count=0;
		gps_sms_rx_flag=0;
		gsm_tx_sms_count=0;
		ftp_rxvd_flag=0;
		ftp_status_sms_indiction=0;

		gsm_sms_intr_recvd_flag=0;
	    index_data_sms_flag=0;
	}
}


/*****************************************************************************
  * @brief  Function to turn ON GSM											  *
  * @param  none															  *
  * @retval none															  *
  *****************************************************************************/
void gsm_on(void)
{
  gsm_reset_count++;


  if (gsm_reset_count <= 5)               //5
   {
	  gpio_set_level(GSM_UART_PWRON_PIN,0);
   }
   else if (gsm_reset_count <= 15)        //15
   {
	  gpio_set_level(GSM_UART_PWRON_PIN,1);
   }

   else if (gsm_reset_count <= 150)
   {
	  gpio_set_level(GSM_UART_PWRON_PIN,0);
   }

  else
   {
     gsm_intrupt_flag = 1;     
     rxbuff_len       = 20;	      
     gsm_sms_rxvd=0;


	 #ifdef GSM_DEBUG_EN
     APP_GSM_DEBUG((uint8_t*)"\n\rAPP_GSM> MODEM_ON",my_strlen("\n\rAPP_GSM> MODEM_ON"))
     #endif

	 
	 gsm_reset_count  = 0;
	 gsm_device_state = GSM_INIT;
	 
     gsm_init_flags();

	 time_engine_on   = 0;
	 data_engine_on   = 0;
   }
}
/* End gsm_on()*******************************************/

/******************************************************************************
  * @brief  Function for initializing GSM
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_init_engine(void)
{
	uint8_t i=0,j=0;
	switch(gsm_init_state)
	 {
	   case GSM_CMD1    :
		   	   	   	   	   engine_tim_limit = ENGINE_TIM_MAX;
		   	   	   	   	   gsm_send_command(Command1);
	   	   	   	   	   	   gsm_response_process(RESP_OK);
	                       break;

	   case GSM_CMD2    :
		                   engine_tim_limit = ENGINE_TIM_MAX;
	   	                   gsm_send_command(Command2);
	                       gsm_response_process(RESP_OK);
	   	   	   	   	   	   break;
	   case GSM_CMD3    :  modem_type_get=1;
		                   engine_tim_limit = ENGINE_TIM_MAX;
	   	                   gsm_send_command(Command3);
	                       gsm_response_process(RESP_OK);
	   	   	   	   	   	   break;
	   case GSM_CMD4	:  modem_type_get=0;
		   	   	   	   	   if(time_get_set_flag == 1)
		   	   	   	   	   {
		   	   	   	   		   engine_tim_limit = ENGINE_TIM_MAX;
		    		   	   	   gsm_send_command(Command4);
		    		   	   	   gsm_response_process(RESP_OK);
		   	   	   	   	   }
		   	   	   	   	   else
		   	   	   	   	   {
		   	   	   	   		   gsm_init_state=gsm_init_state+1;
		   	   	   	   	   }

		   	   	   	   	   break;
	   case GSM_CMD5    :
		   	   	   	   	   engine_tim_limit = ENGINE_TIM_MIN;
	   	   	   	   	   	   gsm_send_command(Command5);
	   		   	   	   	   gsm_response_process(RESP_OK);
	                       break;
	   case GSM_CMD6    :
		   	   	   	   	   engine_tim_limit = ENGINE_TIM_MAX;
		   		   	   	   gsm_send_command(Command6);
	   	   	   	   	   	   gsm_response_process(RESP_OK);
	                       break;

	   case GSM_CMD7    :  engine_tim_limit = ENGINE_TIM_MAX;
		   	   	   	   	   gsm_send_command(Command7);
	                       gsm_response_process(RESP_OK);
	                       break;

	   case GSM_CMD8    :
		   	   	   	   	   engine_tim_limit = ENGINE_TIM_MAX;
		   	   	   	   	   if(modem_type == LYNQ_MODEM)
		   	   	   	   	   {
		   	   	   	   		   gsm_send_command(Command8);
		   	   	   	   		   j=20;
		   	   	   	   		   for(i=9;Command8[i] != '"';i++)
		   	   	   	   		   {
		   	   	   	   			   	  Commandfwvr1[j]=Command8[i];
		   	   	   	   			   	  j++;
		   	   	   	   		   }
		   	   	   	   		   Commandfwvr1[j]='"';
		   	   	   	   		   Commandfwvr1[j+1]='\r';
		   	   	   	   		   Commandfwvr1[j+2]='\0';

		   	   	   	   	   }
		   	   	   	   	   else
		   	   	   	   	   {
		   	   	   	           gsm_send_command(Command8a);
		   	   	   	   	   }

	                       gsm_response_process(RESP_OK);

	                       break;

	   case GSM_CMD9    : imei_flag = 1;
		                  gsm_send_command(Command9);
		   	              gsm_response_process(RESP_OK);

	                       break;

	   case GSM_CMD10	:  imei_flag=0;
		           	   	   engine_tim_limit = ENGINE_TIM_MIN;
		           	   	   if(modem_type == LYNQ_MODEM)
		           	   	   {
		           	   		  //gsm_init_state=gsm_init_state+1;
			           	   	 gsm_send_command(gps_command1);
		           	   	   }
		           	   	   else
		           	   	   {
		           	   		  gsm_send_command(q_gps_command1);
		           	   	   }
		           	   	   gsm_response_process(RESP_OK);
		   	   	   	   	   break;


	   case GSM_CMD11	:   engine_tim_limit = ENGINE_TIM_MAX;
	   	   	   	   	   	   	//gsm_init_state=gsm_init_state+1;
		           	   	    gsm_send_command(tx_sms_command2);
		           	   	    gsm_response_process(RESP_OK);
		           	   	    break;

	   case GSM_CMD12    :engine_tim_limit = ENGINE_TIM_MAX;
		   	   	   	   	   if(time_get_set_flag == 1)
	   	   	   	   	   	   {
		   	   	   	   	   	    time_get_set_flag=0;
		   	   	   	   	   	   	gsm_device_state    = GSM_GET_TIME;
		   	   	   	   	   	    gsm_init_state=GSM_CMD1;
	   	   	   	   	   	   }
	   	   	   	   	   	   else
	   	   	   	   	   	   {

	   	   	   	   	   		   gsm_device_state    = GSM_IDLE; //GSM_FWVR_UP; //  GSM_TX_SMS;  //
	   	   	   	   	   		   gsm_init_state=GSM_CMD1;
	   	   	   	   	   	   }
		   	   	   	   	   cloud_comm_start=1;
	                       break;

	   default           : gsm_device_state = GSM_RESTART;
	                       gsm_reset_count  = 0;
	                       break;
	}
}


/* End gsm_on()*******************************************/


/******************************************************************************
  * @brief  Function for initializing GPS
  * @param  none
  * @retval none
  *****************************************************************************/
void gps_engine(void)
{
	gps_count++;
	if(gps_count >= 40)
	{

		switch(gps_state)
		{
		case GPS_CMD1:
						engine_tim_limit = ENGINE_TIM_MAX;
						gps_data_flag=1;
						if(modem_type == LYNQ_MODEM)
						{
							gsm_send_command(gps_command2);
						}
						else
						{
							gsm_send_command(q_gps_command2);
						}
						gsm_response_process(RESP_GPS_OK);
			            break;
		case GPS_CMD2:  gps_data_flag=1;
						engine_tim_limit = ENGINE_TIM_MAX;
						if(modem_type == LYNQ_MODEM)
						{
							gsm_send_command(gps_command3);
						}
						else
						{
							gsm_send_command(q_gps_command3);
						}
					    gsm_response_process(RESP_GPS_OK);
			            break;
		case GPS_CMD3: engine_tim_limit = ENGINE_TIM_MIN;
						if(get_gps_location_flag == 1)
						{
							get_gps_location_flag=0;
							gps_data_flag=0;
							gps_state=GPS_CMD1;
				            gps_count=0;
	                        gsm_device_state    = GSM_IDLE;
	                        rx_sms_store_process(SMS_GET_URL_TYPE);
						}
						else
						{
							gps_data_flag=0;
			                gps_state=GPS_CMD1;
						    gps_count=0;
						    gsm_device_state    =   GSM_TCP_COMM;
						}

			           break;

		default      :gps_count=0;
					  gsm_device_state = GSM_RESTART;
				      gsm_reset_count  = 0;
			          break;
		}
	}


}

/* End gsm_http_txrx()*******************************************/

/******************************************************************************
  * @brief  Function to Get TIME
  * @param  none
  * @retval none
  *****************************************************************************/
void quectel_gsm_tcp_engine(void)
{


	switch(gsm_tcp_state)
	{

		case GSM_TCP_1 :gsm_send_command(q_tcp_command1);
						gsm_response_process(RESP_TCP_OK);
						break;

		case GSM_TCP_2 :tcp_server_connect = 0;
						gsm_send_command(q_tcp_command2);
						gsm_response_process(RESP_TCP_OK);
						break;

		case GSM_TCP_3 :tcp_server_connect = 0;
			            engine_tim_limit = ENGINE_TIM_20S;
						gsm_send_command(q_tcp_command3);
						gsm_response_process(RESP_TCP_OK);

						break;
		case GSM_TCP_4 :tcp_server_connect = 1;
						engine_tim_limit = ENGINE_TIM_20S;
						gsm_send_command(q_tcp_command4);
						gsm_response_process(RESP_TCP_OK);
						break;

		case GSM_TCP_5 :tcp_server_connect = 1;
						engine_tim_limit = ENGINE_TIM_20S;
						gsm_tcp_state = gsm_tcp_state+1;
						//gsm_send_command(q_tcp_command5);
						//gsm_response_process(RESP_TCP_OK);
						break;

		case GSM_TCP_6 :tcp_server_connect = 1;
						tcp_data_flag = 1;
						engine_tim_limit = ENGINE_TIM_20S;
						gsm_send_command(q_tcp_command6);
						gsm_response_process(RESP_TCP_OK);
						break;
		case GSM_TCP_7 :
						tcp_data_flag=0;
						tcp_server_connect = 1;

						 if(send_tcp_data == 1 && gsm_command_flag == 0 )
						 {
							 send_tcp_data=0;
							 current_data=1;
							 gsm_device_state = GSM_GPS;
						 }
						 else
						 {
							#if BMS_TYPE_1 || BMS_TYPE_4
							 if(bms_type[0] == BMS_TYPE_1 ||  bms_type[0] == BMS_TYPE_4)
							 {
								 strcpy((char*)q_tcp_command7,"AT+QISEND=129\r\0");
							 }
							#endif
							#if BMS_TYPE_2
							 if(bms_type[0] == BMS_TYPE_2)
							 {
								 strcpy((char*)q_tcp_command7,"AT+QISEND=88\r\0");
							 }
							#endif
							#if BMS_TYPE_3
							 if(bms_type[0] == BMS_TYPE_3)
							 {
								 strcpy((char*)q_tcp_command7,"AT+QISEND=99\r\0");
							 }
							#endif
							#if BMS_TYPE_5
							 if(bms_type[0] == BMS_TYPE_5)
							 {
								 strcpy((char*)q_tcp_command7,"AT+QISEND=190\r\0");
							 }
							#endif
							#if BMS_TYPE_6
							 if(bms_type[0] == BMS_TYPE_6)
							 {
								 strcpy((char*)q_tcp_command7,"AT+QISEND=121\r\0");
							 }
							#endif
							#if BMS_TYPE_7
							 if(bms_type[0] == BMS_TYPE_7)
							 {
								 strcpy((char*)q_tcp_command7,"AT+QISEND=128\r\0");
							 }
							#endif
							#if BMS_TYPE_8
							 if(bms_type[0] == BMS_TYPE_8)
							 {
								 strcpy((char*)q_tcp_command7,"AT+QISEND=113\r\0");
							 }
							#endif
							#if BMS_TYPE_X
							 if(bms_type[0] == BMS_TYPE_X)
							 {
								 strcpy((char*)q_tcp_command7,"AT+QISEND=91\r\0");
							 }
							#endif
							 server_connected_update = 1;
								gsm_send_command(q_tcp_command7);
							 gsm_response_process(RESP_TCP_OK);
						 }


						break;
		case GSM_TCP_8 :
						tcp_server_connect = 1;
					    server_connected_update = 0;
						engine_tim_limit = ENGINE_TIM_20S;
						if(flash_read_started == 0)
						{
						    current_data=1;
						}
						data_send_to_server();
						current_data=0;
						break;
		case GSM_TCP_9 :
			            engine_tim_limit = ENGINE_TIM_MAX;
						if(close_tcp_port == 1)
						{
							gsm_send_command(q_tcp_command8);
							gsm_response_process(RESP_TCP_OK);
						}
						else
						{
							gsm_tcp_state = gsm_tcp_state+1;
						}


						break;
		case GSM_TCP_10 :
						if(close_tcp_port == 1)
						{
							close_tcp_port = 0;
							gsm_device_state    = GSM_IDLE;
							gsm_tcp_state =GSM_TCP_6;

						}
						else
						{
				            gsm_device_state    = GSM_IDLE;
							gsm_tcp_state =GSM_TCP_7;
						}

						break;
		default 		:gsm_device_state = GSM_RESTART;
				 	 	 gsm_reset_count  = 0;
				 	 	 break;

	}
}

/* End gsm_tcp_engine()*******************************************/



/******************************************************************************
  * @brief  Function to Get TIME
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_tcp_engine(void)
{


	switch(gsm_tcp_state)
	{

		case GSM_TCP_1 :tcp_server_connect = 1;
			            engine_tim_limit = ENGINE_TIM_MAX;
						gsm_send_command(tcp_command3);
						gsm_response_process(RESP_TCP_OK);

						break;
		case GSM_TCP_2:tcp_server_connect = 1;
						engine_tim_limit = ENGINE_TIM_MAX;
						gsm_send_command(tcp_command5);
						gsm_response_process(RESP_TCP_OK);
						break;
		case GSM_TCP_3 :

						 tcp_server_connect = 1;
						 if(send_tcp_data == 1 && gsm_command_flag == 0 )
						 {
							 send_tcp_data=0;
							 current_data=1;
							 gsm_device_state = GSM_GPS;
						 }
						 else
						 {
							#if BMS_TYPE_1 || BMS_type_4
							 if(bms_type[0] == BMS_TYPE_1 ||  bms_type[0] == BMS_TYPE_4)
							 {
								 strcpy((char*)tcp_command6,"AT+CIPSEND=129\r\0");
							 }
							#endif
							#if BMS_TYPE_2
							 if(bms_type[0] == BMS_TYPE_2)
							 {
								 strcpy((char*)tcp_command6,"AT+CIPSEND=88\r\0");
							 }
							#endif
							#if BMS_TYPE_3
							 if(bms_type[0] == BMS_TYPE_3)
							 {
								 strcpy((char*)tcp_command6,"AT+CIPSEND=99\r\0");
							 }
							#endif
							#if BMS_TYPE_5
							 if(bms_type[0] == BMS_TYPE_5)
							 {
								 strcpy((char*)tcp_command6,"AT+CIPSEND=190\r\0");
							 }
							#endif
							#if BMS_TYPE_6
							 if(bms_type[0] == BMS_TYPE_6)
							 {
								 strcpy((char*)tcp_command6,"AT+CIPSEND=121\r\0");
							 }
							#endif
							#if BMS_TYPE_7
							if(bms_type[0] == BMS_TYPE_7)
							{
								strcpy((char*)tcp_command6,"AT+CIPSEND=128\r\0");
							}
							#endif
							#if BMS_TYPE_8
							if(bms_type[0] == BMS_TYPE_8)
							{
								strcpy((char*)tcp_command6,"AT+CIPSEND=113\r\0");
							}
							#endif
							#if BMS_TYPE_X
							 if(bms_type[0] == BMS_TYPE_X)
							 {
								 strcpy((char*)tcp_command6,"AT+CIPSEND=91\r\0");
							 }
							#endif
							 server_connected_update = 1;
							 gsm_send_command(tcp_command6);
							 gsm_response_process(RESP_TCP_OK);
						 }


						break;
		case GSM_TCP_4 :

			             tcp_server_connect = 1;
		                 server_connected_update = 0;
			             engine_tim_limit = ENGINE_TIM_MAX;
			             if(flash_read_started == 0)
			             {
			            	 current_data=1;
			             }
			             data_send_to_server();
			             current_data=0;

						break;
		case GSM_TCP_5 :
			            tcp_server_connect = 0;
						engine_tim_limit = ENGINE_TIM_MAX;
						gsm_send_command(tcp_command6a);
						gsm_response_process(RESP_TCP_OK);
						break;
		case GSM_TCP_6 :
			            engine_tim_limit = ENGINE_TIM_MAX;
						if(close_tcp_port == 1)
						{
							gsm_send_command(tcp_command7);
							gsm_response_process(RESP_TCP_OK);
						}
						else
						{
							gsm_tcp_state = gsm_tcp_state+1;
						}
						break;
		case GSM_TCP_7 :
						if(close_tcp_port == 1)
						{
							close_tcp_port = 0;
							gsm_device_state    = GSM_IDLE;
							gsm_tcp_state =GSM_TCP_1;

						}
						else
						{
				            gsm_device_state    = GSM_IDLE;
							gsm_tcp_state =GSM_TCP_3;
						}

						break;
		default 		:gsm_device_state = GSM_RESTART;
				 	 	 gsm_reset_count  = 0;
				 	 	 break;

	}
}

/* End gsm_tcp_engine()*******************************************/

/******************************************************************************
  * @brief  Function to Get TIME
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_get_time_engine(void)
{
	switch(gsm_tim_state)
		 {
		   case GSM_TIM_CMD1    :  engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(Commandtim1);
		                       	   gsm_response_process(RESP_TIM_OK);
		                       	   break;

		   case GSM_TIM_CMD2	:  if(gsm_time_get_flag == 1)
		   	   	   	   	   	   	   {

			   	   	   	             	gsm_time_get_flag  = 0;
			   	   	   	             	gsm_device_state = GSM_GPS;
			   	   	   	                gsm_tim_state=GSM_TIM_CMD1;
	                         	   }
		                           else
		                           {
		                        	   cloud_comm_start=1;
		                        	   gsm_tim_state=GSM_TIM_CMD1;
		                        	   gsm_device_state = GSM_IDLE;//GSM_TCP_COMM;//GSM_GPS;// //GSM_TX_SMS;//GSM_IDLE; //GSM_IDLE;//
		                           }
			   	   	   	   	   	   break;

		   default:				  gsm_device_state = GSM_RESTART;
			   	   	   	   	   	  break;

		 }
}
/* End gsm_get_time()*******************************************/


/******************************************************************************
  * @brief  Function to connect to the server to download and update the firmware
  * @param  none
  * @retval none
  *****************************************************************************/
void quectel_gsm_fwvr_up_engine(void)
{
	gsm_firmware_count++;

	switch(gsm_firmware_state)
	{
		   case GSM_FWVR_CMD1   : engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(Commandfwvr0);
	 	   	   	   	   	   	   	   gsm_response_process(Q_RESP_FWVR_OK);
	 	   	   	   	   	   	   	   break;

		   case GSM_FWVR_CMD2   :  engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(q_Commandfwvr0);
		    	   	   	   	   	   gsm_response_process(Q_RESP_FWVR_OK);
		    	   	   	   	   	   break;

		   case GSM_FWVR_CMD3   :  engine_tim_limit = ENGINE_TIM_MAX;
				   	   	   	   	   gsm_send_command(q_Commandfwvr1);
				   	   	   	   	   gsm_response_process(Q_RESP_FWVR_OK);
    	    	   	   	   	   	   break;

		   case GSM_FWVR_CMD4	:  q_ftp_response=1;
		                           fw_update_started=1;
			                       engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(q_Commandfwvr2);
		   	   	   	   	   	   	   gsm_response_process(Q_RESP_FWVR_OK);
		   	   	   	   	   	   	   break;

		   case GSM_FWVR_CMD5	:
		   	   	   	   	   	   	   q_ftp_response=1;
			                       engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(q_Commandfwvr3);
		   	   	   	   	   	   	   gsm_response_process(Q_RESP_FWVR_OK);
		   	   	   	   	   	   	   break;

		   case GSM_FWVR_CMD6   :  q_ftp_data=1;
		   	   	   	   	   	   	   q_ftp_response=0;
		   	   	   	   	   	   	   if(gsm_firmware_count >=10)
		   	   	   	   	   	   	   {
		   	   	   	   	              engine_tim_limit = ENGINE_TIM_MAX;
		   	   	   	   	   			  gsm_send_command(q_Commandfwvr4);
		   	   	   	   	   		   	  gsm_response_process(Q_RESP_FWVR_OK);
		   	   	   	   	   	   	   }
		   	    	   	   	   	   break;

		   case GSM_FWVR_CMD7   :  q_ftp_response=1;
		                           q_ftp_data=0;
		                           ftp_write_data_flag=0;
		                           if(gsm_firmware_count >=20)
		                           {
			                         engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	     gsm_send_command(q_Commandfwvr5);
		   	   	   	   	   	   	     gsm_response_process(Q_RESP_FWVR_OK);
		                           }
		   	   	   	   	   	   	   break;

		   case GSM_FWVR_CMD8	:  gsm_firmware_state = GSM_FWVR_CMD1;
			   	   	   	   	   	   gsm_device_state = GSM_IDLE;
			   	   	   	   	   	   gsm_firmware_count=0;
 	   	   	   	   	   	   	   	  break;


		   default:				  gsm_device_state = GSM_RESTART;
			   	   	   	   	   	  break;
	}

}
/* End gsm_fwvr_up()*******************************************/




/******************************************************************************
  * @brief  Function to connect to the server to download and update the firmware
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_fwvr_up_engine(void)
{
	gsm_firmware_count++;

	switch(gsm_firmware_state)
	{
		   case GSM_FWVR_CMD1   : engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(Commandfwvr0);
	 	   	   	   	   	   	   	   gsm_response_process(RESP_FWVR_OK);
	 	   	   	   	   	   	   	   break;

		   case GSM_FWVR_CMD2   :  engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(Commandfwvr1);
		    	   	   	   	   	   gsm_response_process(RESP_FWVR_OK);
		    	   	   	   	   	   break;

		   case GSM_FWVR_CMD3   :  engine_tim_limit = ENGINE_TIM_MAX;
				   	   	   	   	   gsm_send_command(Commandfwvr2);
				   	   	   	   	   gsm_response_process(RESP_FWVR_OK);
    	    	   	   	   	   	   break;

		   case GSM_FWVR_CMD4	:  engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(Commandfwvr3);
		   	   	   	   	   	   	   gsm_response_process(RESP_FWVR_OK);
		   	   	   	   	   	   	   break;

		   case GSM_FWVR_CMD5	:  engine_tim_limit = ENGINE_TIM_MAX;
		   	   	   	   	   	   	   gsm_firmware_state=gsm_firmware_state+1;
			   	   	   	   	   	   //gsm_send_command(Commandfwvr4);
		   	   	   	   	   	   	   //gsm_response_process(RESP_FWVR_OK);
		   	   	   	   	   	   	   break;

		   case GSM_FWVR_CMD6   :  engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(Commandfwvr5);
		   	    	   	   	   	   gsm_response_process(RESP_FWVR_OK);
		   	    	   	   	   	   break;

		   case GSM_FWVR_CMD7   :  engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(Commandfwvr6);
		   	   	   	   	   	   	   gsm_response_process(RESP_FWVR_OK);
		   	   	   	   	   	   	   break;

		   case GSM_FWVR_CMD8   :  engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(Commandfwvr7);
		   	   	   	   	   	   	   gsm_response_process(RESP_FWVR_OK);
		   	   	   	   	   	   	   break;

		   case GSM_FWVR_CMD9	:  engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(Commandfwvr8);
		   	   	   	   	   	   	   gsm_response_process(RESP_FWVR_OK);
		   	   	   	   	   	   	   break;

		   case GSM_FWVR_CMD10   :  fw_update_started=1;
			                       engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	   gsm_send_command(Commandfwvr9);
		   	   	   	   	   	   	   gsm_response_process(RESP_FWVR_OK);
		   	   	   	   	   	   	   break;

		   case GSM_FWVR_CMD11	: engine_tim_limit = ENGINE_TIM_MAX;
			   	   	   	   	   	  gsm_send_command(Commandfwvr10);
 	   	   	   	   	   	   	   	  gsm_response_process(RESP_FWVR_OK);
 	   	   	   	   	   	   	   	  break;

		   case GSM_FWVR_CMD12	: ftp_rxvd_flag = 1;
		   	   	   	   	   	   	  ftp_status_sms_indiction=1;
			   	   	   	   	   	  engine_tim_limit = ENGINE_TIM_MAX;
    	   	   	  	  	  	  	  gsm_send_command(Commandfwvr11);
	   	   	   	  	  	  	  	  gsm_response_process(RESP_FWVR_OK);
	   	   	   	  	  	          gsm_firmware_count=0;
 	   	   	   	   	   	   	   	  break;

		   case GSM_FWVR_CMD13	:  if(gsm_firmware_count >= 10)
		   	   	   	   	   	   	   {
			   	   	   	   	   	   	   //gsm_firmware_count=0;
			   	   	   	   	   	   	   ftp_rxvd_flag = 1;
			   		   	   	   	   	   ftp_read_data_flag = 1;
			   			   	   	   	   engine_tim_limit = ENGINE_TIM_MAX;
			   	  	  	  	  	  	   gsm_send_command(Commandfwvr12);
			   	  	  	  	  	  	   gsm_response_process(RESP_FWVR_OK);
		   	   	   	   	   	   	   	}


 	   	   	   	   	   	   	   	  break;
		   case GSM_FWVR_CMD14	:  if(gsm_firmware_count >= 20)
		   	   	   	   	   	   	   {
			   	   	   	   	   	   	   //gsm_firmware_count=0;

			   			   	   	   	   engine_tim_limit = ENGINE_TIM_MAX;
			   	  	  	  	  	  	   gsm_send_command(Commandfwvr13);
			   	  	  	  	  	  	   gsm_response_process(RESP_FWVR_OK);
		   	   	   	   	   	   	   	}
 	   	   	   	   	   	   	   	  break;

		   case GSM_FWVR_CMD15	:  gsm_firmware_state = GSM_FWVR_CMD1;
			   	   	   	   	   	   gsm_device_state = GSM_IDLE;
 	   	   	   	   	   	   	   	  break;


		   default:				  gsm_device_state = GSM_RESTART;
			   	   	   	   	   	  break;
	}

}
/* End gsm_fwvr_up()*******************************************/

/******************************************************************************
  * @brief  Function to Keep the GSM in the IDLE state
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_idle(void)
{

  gsm_reset_count = 0;


  if(send_tcp_data == 1)
  {

	  send_tcp_data=0;
	  if(tcp_port == 1)
	  {
		  tcp_port=0;
		  close_tcp_port =1;
	  }

	  if(rtc_time_set_flag == 1)
	  {

		  gsm_time_get_flag=1;
		  gsm_device_state       = GSM_GET_TIME;
	  }
	  else
	  {
		  gsm_device_state       = GSM_GPS;
	  }

  }
  else if(gsm_tx_sms_flag == 1)
  {
	  gsm_tx_sms_flag=0;
	  gsm_device_state       = GSM_TX_SMS;
  }
  else if(gsm_sms_rxvd == 1)
  {
	  gsm_sms_rxvd=0;
	  gsm_device_state       = GSM_RX_SMS;
  }
  else if(firmware_update_initiated == 1)
  {
	  firmware_update_initiated =0;
	  gsm_device_state          = GSM_FWVR_UP;
  }
  else
  {

  }


}
/* End gsm_idle()*******************************************/

/******************************************************************************
  * @brief  Function for resetting the flags
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_init_flags(void)
{
  gsm_response_flag  = 0;
  gsm_command_flag   = 0;

  if(gsm_fwup_flag != 0)
    gsm_fwup_flag=0;
}
/* End gsm_init_flags()**************************************************/

/******************************************************************************
  * @brief  Function receiving sms from mobile to modem
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_rx_sms_engine(void)
{

	switch(gsm_rx_sms_state)
	{
		case GSM_RX_SMS_1     : engine_tim_limit = ENGINE_TIM_MAX;
								gsm_sms_intr_recvd_flag=1;
								index_data_sms_flag=0;
								gsm_send_command(rx_sms_command1);
								gsm_response_process(RESP_RX_SMS_OK);
		                        break;
		case GSM_RX_SMS_2     : engine_tim_limit = ENGINE_TIM_MAX;
								gsm_sms_intr_recvd_flag=1;
								index_data_sms_flag=1;
							    gsm_send_command(rx_sms_command2);
					            gsm_response_process(RESP_RX_SMS_OK);

		                        break;
		case GSM_RX_SMS_3    :  engine_tim_limit = ENGINE_TIM_MAX;
			                    gsm_send_command(rx_sms_command3);
								gsm_response_process(RESP_RX_SMS_OK);
		                        break;
		case GSM_RX_SMS_4    :

								rx_sms_command_data(gsm_rx_sms_data);
								gsm_rx_msg_type=rx_sms_get_type(temp_sms_data1);
								rx_sms_store_process(gsm_rx_msg_type);
								gsm_rx_sms_state=gsm_rx_sms_state+1;
		                        break;

		case GSM_RX_SMS_5    :  if(gps_sms_rx_flag==1)
								{
									 get_gps_location_flag = 1;
			 	 	 	 	 	 	 gsm_device_state = GSM_GPS;
						             gsm_rx_sms_state=GSM_RX_SMS_1;
								}
								else
								{
									gsm_device_state = GSM_IDLE;
		                            gsm_rx_sms_state=GSM_RX_SMS_1;
								}

		                        break;

		default            :gsm_device_state = GSM_RESTART;
			   	   	   	   	break;
	}


}
/* End gsm_rx_sms_engine()**************************************************/

/******************************************************************************
  * @brief  Function sending sms from modem to mobile
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_tx_sms_engine(void)
{
	switch(gsm_tx_sms_state)
	{

		case GSM_TX_SMS_1     : engine_tim_limit = ENGINE_TIM_MAX;
						     	if(gsm_command_flag == 0)
							    {

								gsm_command_flag             = 1;
								gsm_response_flag            = 0;
								gsm_engine_tick              = 0;
								gsm_response_data_start_flag = 0;
								gsm_RxBuffWRPtr  = 0;

								SEND_GSM_BUFF(tx_sms_command3, my_strlen((char*)tx_sms_command3));
								SEND_GSM_BUFF(tx_sms_command4, 1);

								#ifdef GSM_DEBUG_EN
								APP_GSM_DEBUG((unsigned char*) "\n\rGSM_AT> NUMBER ENTERED : ", my_strlen("\n\rGSM_AT> NUMBER ENTERED : "))
								#endif

							   }

		 	 	 	 	 	 gsm_response_process(RESP_TX_SMS_OK);
		                     break;
		case GSM_TX_SMS_2     :engine_tim_limit = ENGINE_TIM_MAX;

							if(gsm_command_flag == 0)
							{
								gsm_command_flag             = 1;
								gsm_response_flag            = 0;
								gsm_engine_tick              = 0;
								gsm_response_data_start_flag = 0;
								gsm_RxBuffWRPtr  = 0;

								SEND_GSM_BUFF(gsm_tx_sms_data,strlen((char*)gsm_tx_sms_data));
								SEND_GSM_BUFF(tx_sms_command6, 1);
								SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));

								#ifdef GSM_DEBUG_EN
								APP_GSM_DEBUG((unsigned char*) "\n\rGSM_AT> MESSAGE SENT:", my_strlen("\n\rGSM_AT> MESSAGE SENT:"))
								#endif

							}

							 gsm_response_process(RESP_TX_SMS_OK);

		                     break;
		case GSM_TX_SMS_3     : if(restart_chip == 1)
								{
									restart_chip=0;
									esp_restart();
								}
								else if(reset_modem==1)
								{
									reset_modem=0;
									gsm_device_state = GSM_RESTART;
								}
								else
								{
									gsm_transmit_sms_after_ota_done=1;
				                    gsm_tx_sms_state =GSM_TX_SMS_1;
								    gsm_device_state = GSM_IDLE;
								}


							break;

		default            :gsm_device_state = GSM_RESTART;
			   	   	   	   	break;
	}


}
/* End gsm_sms_engine()**************************************************/

/********************************************************************************
  * @brief  Function for sending GSM command
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_send_command(unsigned char *gprs_command)
{
  if(gsm_command_flag == 0)
   {
     gsm_command_flag             = 1;
     gsm_response_flag            = 0;    
     gsm_engine_tick              = 0;
     gsm_response_data_start_flag = 0;

     if(gprs_command[0] != '\0')
      {

		gsm_RxBuffWRPtr  = 0;

        #ifdef GSM_DEBUG_EN
		APP_GSM_DEBUG((unsigned char*) "\n\rGSM_AT> :", my_strlen("\n\rGSM_AT> :"))
		APP_GSM_DEBUG(gprs_command, my_strlen((char*)gprs_command))
        #endif

        SEND_GSM_BUFF(gprs_command, my_strlen((char*)gprs_command));
        SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
      }
   }
}
/* End gsm_send_command()*******************************************************/

/********************************************************************************
  * @brief  Function for the response of each GSM
  * @param  none
  * @retval none
  *****************************************************************************/
unsigned char gsm_response_process(unsigned char response_no)
{

  if(gsm_response_flag == 1)
   {
     #ifdef GSM_DEBUG_EN
	  APP_GSM_DEBUG((uint8_t*)"\n\rAPP_GSM> GSM_BUFF: ",my_strlen("\n\rAPP_GSM> GSM_BUFF: "))
	  APP_GSM_DEBUG((uint8_t*)gsm_serialRXBuff,gsm_RxBuffWRPtr)
     #endif

	  if(ftp_write_data_flag == 1)
	  {
		  ftp_write_data_flag=0;
		  if(modem_type == LYNQ_MODEM)
		  {
			  esp_firmware_update((uint8_t*)gsm_serialRXBuff,gsm_RxBuffWRPtr);
		  }
		  else
		  {
			  esp_firmware_update_frm_quectel((uint8_t*)gsm_serialRXBuff,gsm_RxBuffWRPtr);
		  }
	  }

	 if (gsm_time_got_flag == 1)
	 	{
          gsm_time_got_flag = 0;
		  app_set_rtc();
	 	}
	  if(fw_update_started == 1)
	  {
		  fw_update_started=0;
		  esp_fw_update_init();
	  }

	 switch(response_no)
      {
         case RESP_OK      		 : gsm_init_state = gsm_init_state + 1;
                                   break;

         case RESP_HTTP_OK 		 : gsm_txrx_state               = gsm_txrx_state + 1;
		                           gsm_response_data_start_flag = 0;
                                   break;

         case RESP_OFF		     :
                                   break;

         case RESP_RESET         :
                                   break;

         case RESP_TIM_OK 		 :  gsm_tim_state = gsm_tim_state + 1;
                                    break;

         case RESP_FWVR_OK		 : lynq_ftp_reception();

                                   break;
         case RESP_TCP_OK		 : gsm_tcp_state=gsm_tcp_state+1;
         	 	 	 	 	 	   if(server_connected_update ==1)
         	 	 	 	 	 	   {
         	 	 	 	 	 	 	  server_connected_update=0;
         	 	 	 	 		      server_conn_flag=1;
         	 	 	 	 	 	   }
                                   break;

         case RESP_GPS_OK		 : gps_state=gps_state+1;
                                   break;

         case RESP_TCP_DATA      : tcp_data_reading_from_spi_flash();
        	 	 	 	 	 	   break;

         case RESP_TX_SMS_OK     : gsm_tx_sms_state = gsm_tx_sms_state+1;
                                   break;

         case RESP_RX_SMS_OK     : gsm_rx_sms_state = gsm_rx_sms_state+1;
                                   break;

         case Q_RESP_FWVR_OK     : quectel_ftp_reception();
         	 	 	 	 	 	   break;

         default     			 : break;
      }
	  if(gsm_sms_intr_recvd_flag == 1)
	  {
		  gsm_sms_intr_recvd_flag=0;
		  decode_gsm_rx_sms_process((uint8_t*)gsm_serialRXBuff,gsm_RxBuffWRPtr);
	  }

     gsm_init_flags();
     gsm_reception=0;
     gsm_firmware_count=0;
     gsm_tx_sms_count=0;
     ftp_status_sms_indiction=0;

     return 1;
   }
  else if (gsm_engine_tick > engine_tim_limit)
   {

	  if(tcp_server_connect == 1)
	  {
		  tcp_data_writing_to_spi_flash();
		  tcp_server_connect=0;
	  }
	  else
	  {

		     gsm_failure_flag = 1;
		     gsm_fwup_flag    = 0;

			 gsm_init_flags();
			 gsm_RxBuffWRPtr  = 0;
			 cloud_comm_start=0;
			 cloud_commn_timer=0;
			 send_tcp_data=0;
			 gsm_reception=0;

			 gsm_device_state = GSM_RESTART;


		     #ifdef GSM_DEBUG_EN

			 APP_GSM_DEBUG((uint8_t*)"\n\rAPP_GSM> ERROR:",my_strlen("\n\rAPP_GSM> ERROR:"))
			 APP_GSM_DEBUG((uint8_t*)gsm_serialRXBuff, gsm_RxBuffWRPtr)
		     #endif
		  	  if(ftp_rxvd_flag == 1)
		  	  {
		  		  	ftp_rxvd_flag = 0;
	 	 	 		strcpy((char*)gsm_tx_sms_data,"FOTA UPDATION FAILED");
	 	 	 		gsm_tx_sms_flag=1;
	 	 	 		gsm_device_state = GSM_IDLE;
	 	 	 		reset_modem=1;

		  	  }
			 if(ftp_status_sms_indiction == 1)
			 {
				 ftp_status_sms_indiction=0;
				 strcpy((char*)gsm_tx_sms_data,"FOTA FTP SERVER CONNECTION FAILED");
				 gsm_tx_sms_flag=1;
				 gsm_device_state = GSM_IDLE;
				 reset_modem=1;
			 }

		     return 0;


   }

   }
  return 2;
}
/* End gsm_response_process()*******************************************/

void gsm_receive_process(void)
{
  uint16_t tmp_gsm_RxBuffWRPtr;
 // uint16_t tmp_startDataPtr=0;
  uint16_t i = 0;
  uint16_t j = 0;
  uint16_t k = 0;
  uint16_t gps_data_came=0;
  uint8_t gps_received_flag=0;
  uint16_t gps_index=0;

  tmp_gsm_RxBuffWRPtr  = gsm_RxBuffWRPtr;

  for (i=0; i<= tmp_gsm_RxBuffWRPtr; i++)
  	{
       if(ftp_rxvd_flag == 1)
       {
    	   if ((gsm_serialRXBuff[i] == 'K') && (gsm_serialRXBuff[i-1] == 'O'))
    	   {
    		   ftp_var=1;
    	   }
    	   if(ftp_var == 1)
    	   {
    		   if((gsm_serialRXBuff[i-1] == ','))
    		   {
    			   gsm_response_flag = 1;
    			   i                 = tmp_gsm_RxBuffWRPtr + 1;
    			   ftp_rxvd_flag=0;
    			   ftp_var=0;
    			   if(ftp_read_data_flag == 1)
    			   {
    				   ftp_read_data_flag=0;
    				   ftp_write_data_flag=1;
    			   }
    		   }
    	   }
       }
       else if(q_ftp_response==1)
       {
    	   if(gsm_serialRXBuff[i] == '\n' && gsm_serialRXBuff[i-1] == '\r' && gsm_serialRXBuff[i-2] == '0')
    	   {
    		   gsm_response_flag = 1;
    		   i = tmp_gsm_RxBuffWRPtr + 1;
    		   q_ftp_response=0;
    	   }
       }
       else if(q_ftp_data ==1)
       {
    	   if((gsm_serialRXBuff[i] == 'K') && (gsm_serialRXBuff[i-1] == 'O'))
    	   {
    		   gsm_response_flag = 1;
    		   i = tmp_gsm_RxBuffWRPtr + 1;
    		   ftp_write_data_flag=1;
    		   q_ftp_data=0;
    	   }
    	   else if((gsm_serialRXBuff[i-8] == ':') && (gsm_serialRXBuff[i-9] == 'T') && (gsm_serialRXBuff[i-10] == 'E') && (gsm_serialRXBuff[i-11] == 'G') && (gsm_serialRXBuff[i-12] == 'P') && (gsm_serialRXBuff[i-13] == 'T') && (gsm_serialRXBuff[i-14] == 'F') && (gsm_serialRXBuff[i-15] == 'Q') && (gsm_serialRXBuff[i-16] == '+'))
    	   {
    		   gsm_response_flag = 1;
    		   i = tmp_gsm_RxBuffWRPtr + 1;
    		   ftp_write_data_flag=1;
    		   q_ftp_data=0;
    		   q_ftp_data_tx_cmplt=1;
    	   }
    	   else
    	   {

    	   }
       }

       else if(gps_data_flag == 1)
        {
             if(gsm_serialRXBuff[i] == '$')
         	{
         	    gps_data_came =1;
         	    gps_index=0;
         	}
             if(gps_data_came == 1)
             {
             	gps_data[gps_index++]=gsm_serialRXBuff[i];
             	if(gsm_serialRXBuff[i] == '\n')
             	{
             		gps_data_came =0;
             		gps_index=0;
             		gps_received_flag=1;
             		gps_data_process(gps_data);
             	}
             }
             if(gps_received_flag==1)
             {
                 if ((gsm_serialRXBuff[i] == 'K') && (gsm_serialRXBuff[i-1] == 'O'))
                   {
                        gsm_response_flag = 1;
                       	 i                 = tmp_gsm_RxBuffWRPtr + 1;
                       	gps_data_flag=0;
                   }
             }
         }
       else if(tcp_data_flag==1)
       {
    	   if((gsm_serialRXBuff[i] == 'K') && (gsm_serialRXBuff[i-1] == 'O') && (gsm_serialRXBuff[i-2] == ' ') && (gsm_serialRXBuff[i-3] == 'T'))
    	   {
    		   gsm_response_flag = 1;
    		   i                 = tmp_gsm_RxBuffWRPtr + 1;
  			   tcp_data_flag=0;
    	   }
       }
       else if(modem_type_get == 1)
       {
    	   if ((gsm_serialRXBuff[i] == 'K') && (gsm_serialRXBuff[i-1] == 'O'))
    	   {
    		   if ((gsm_serialRXBuff[2] == 'Q') && (gsm_serialRXBuff[3] == 'u'))// && (gsm_serialRXBuff[i-8] == 'Y') && (gsm_serialRXBuff[i-9] == 'L'))
    		   {
    			   modem_type=QUECTEL_MODEM;
    			   nvs_read_config_parameters();
    		   }
    	       else
    		   {
    			   modem_type=LYNQ_MODEM;
    			   nvs_read_config_parameters();
    		   }

    		   gsm_response_flag = 1;
    		      			   i = tmp_gsm_RxBuffWRPtr + 1;
    		      			 modem_type_get=0;
    	   }

       }
       else if(imei_flag == 1)
       {
    	   if ((gsm_serialRXBuff[i] == 'K') && (gsm_serialRXBuff[i-1] == 'O'))
    	   {
    		   k=i-20;
    		   for(j=0;j<15;j++)
    		   {

    			   imei_num_bytes[j]=gsm_serialRXBuff[k];
    			   k++;
    		   }
    			   imei_num_conv(imei_num_bytes);
    			   imei_flag =0;
    			   gsm_response_flag = 1;
    			   i                 = tmp_gsm_RxBuffWRPtr + 1;

    	   }

       }

	   else			                         //for all the rest AT and Http commands with 'OK' response and '>>'
		{
 		  if ((gsm_serialRXBuff[i] == 'K') && (gsm_serialRXBuff[i-1] == 'O'))
 			{
 			  gsm_response_flag = 1; 	
			  i                 = tmp_gsm_RxBuffWRPtr + 1;
 			}

		  if ((gsm_serialRXBuff[i] == '>') && (gsm_serialRXBuff[i-1] == '>'))
			{
			  gsm_response_flag = 1;
			  i                 = tmp_gsm_RxBuffWRPtr + 1;
			}
			
		  if ((gsm_serialRXBuff[i] == 'Q') && (gsm_serialRXBuff[i-1] == 'S') && (gsm_serialRXBuff[i-2] == 'C'))
		  	{
		  	   csq_val[0] = gsm_serialRXBuff[i+3];
			   csq_val[1] = gsm_serialRXBuff[i+4];
			   csq_val[2] = gsm_serialRXBuff[i+5];
			   csq_val[3] = gsm_serialRXBuff[i+6];
			   csq_val[4] = gsm_serialRXBuff[i+7];
			   csq_val[5] = '\0';
		  	}

		  if((gsm_serialRXBuff[i] == 'K') && (gsm_serialRXBuff[i-1] == 'L') && (gsm_serialRXBuff[i-2] == 'C') && (gsm_serialRXBuff[i-3] == 'C'))     //+CCLK: "80/01/06,16:46:13+22"
			{
			   for (k=0; k<33; k++)
			   {
				   cloud_settime[k+4] = gsm_serialRXBuff[i+k];
			   }

			   gsm_time_got_flag = 1;                
		  	}
		  if (gsm_serialRXBuff[i] == '>')
			{
			  gsm_response_flag = 1;
			  i                 = tmp_gsm_RxBuffWRPtr + 1;
			}

		}
  	}
}

/********************************************************************************
  * @brief  Main Function for the GSM Engine
  * @param  none
  * @retval none
  *****************************************************************************/
void gsm_engine(void)
{
  gsm_engine_tick++;

  switch(gsm_device_state)
   {
     case GSM_ON        : gsm_on();
	                      gsm_engine_initdone = 0;
                          gsm_init_state      = GSM_CMD1;
                          break;

     case GSM_INIT      : gsm_init_engine();
                          break;

     case GSM_GET_TIME  : gsm_get_time_engine();
                          break;

     case GSM_FWVR_UP   : if(modem_type == LYNQ_MODEM)
     	 	 	 	 	  {
    	                     gsm_fwvr_up_engine();
     	 	 	 	 	  }
     	 	 	 	 	  else
     	 	 	 	 	  {
     	 	 	 	 		quectel_gsm_fwvr_up_engine();
     	 	 	 	 	  }

                          break;

	 case GSM_RESTART   : gsm_engine_initdone = 0;
	 	                  gsm_restart();
                          break;

     case GSM_IDLE      : gsm_idle();
                          break;

     case GSM_TCP_COMM  : if(modem_type == LYNQ_MODEM)
     	 	 	 	 	  {
    	                     gsm_tcp_engine();
     	 	 	 	 	  }
     	 	 	 	 	 else
     	 	 	 	 	 {
     	 	 	 	 	   quectel_gsm_tcp_engine();
     	 	 	 	 	 }

                          break;

     case GSM_GPS       : gps_engine();
                          break;

     case GSM_TX_SMS     : gsm_tx_sms_engine();
     	 	 	 	 	  break;

     case GSM_RX_SMS     : gsm_rx_sms_engine();
     	 	 	 	 	  break;


     default            : APP_GSM_DEBUG((unsigned char*) "\n\rGSM_ENGINE_DEFLT >\n\r", my_strlen("\n\rGSM_ENGINE_DEFLT >\n\r"))
                          break;
   }
}
/* End app_gsm_engine()*******************************************/

/********************************************************************************
  * @brief  lynq ftp reception function
  * @param  none
  * @retval none
  *****************************************************************************/
void lynq_ftp_reception(void)
{
	  uint16_t i=0;
    if(gsm_firmware_state == GSM_FWVR_CMD13)
	{
	 	 	if(ota_update_approved == false ||  close_ftp_flag == 1)
	 	 	{
	 	 	 	 gsm_firmware_state = GSM_FWVR_CMD14;
	 	 	 	 if(close_ftp_flag == 1)
	 	 	 	 {
	 	 	 		close_ftp_flag=0;
	 	 	 		strcpy((char*)gsm_tx_sms_data,"FOTA UPDATION FAILED");
	 	 	 		gsm_tx_sms_flag=1;

	 	 	 	 }
	 	 	}
	 	 	else
	 	 	{
	 	 	 	for(i=0;i<gsm_RxBuffWRPtr;i++)
	 	 	 	{
	 	 	 			if ((gsm_serialRXBuff[i] == 'K') && (gsm_serialRXBuff[i-1] == 'O'))
	 	 	     	    {
	 	 	     		   ftp_var=1;
	 	 	     	    }
	 	 	     	    if(ftp_var == 1)
	 	 	     	    {
	 	 	     		   if((gsm_serialRXBuff[i-1] == ','))
	 	 	     		   {
	 	 	     			   if(gsm_serialRXBuff[i] == '1')
	 	 	     			   {
	 	 	     			       ftp_var=0;

	 	 	     			       gsm_firmware_state = GSM_FWVR_CMD13;
	 	 	     			   }
	 	 	     			   else
	 	 	     			   {
	 	 	     			       gsm_firmware_state = GSM_FWVR_CMD14;
	 	 	     			   }

	 	 	     		   }
	 	 	     	   }
	 	 	 	}
	 	 	 }

	 }
	 else
	 {
 	 	 gsm_firmware_state = gsm_firmware_state + 1;
	 }

}

/* End *******lynq_ftp_reception()*******************************************/

/********************************************************************************
  * @brief  quectel ftp reception function
  * @param  none
  * @retval none
  *****************************************************************************/
void quectel_ftp_reception(void)
{
	  uint16_t i=0,j=0;
    if(gsm_firmware_state == GSM_FWVR_CMD6 && close_ftp_flag == 1)
	{
	 	gsm_firmware_state = GSM_FWVR_CMD7;
	}
    else if(gsm_firmware_state == GSM_FWVR_CMD6)
    {
    	q_ftp_get_bytes=q_ftp_get_bytes+1024-ftp_error_bytes;
    	sprintf((char*)temp_buff,"%d",q_ftp_get_bytes);
    	for(i=0;q_Commandfwvr4[i] != ',';i++)
    	{

    	}
    	for(j=0;temp_buff[j] != '\0';j++)
    	{
    		i++;
    		q_Commandfwvr4[i]=temp_buff[j];
    	}
    	q_Commandfwvr4[i+1]=',';
    	q_Commandfwvr4[i+2]='1';
    	q_Commandfwvr4[i+3]='0';
    	q_Commandfwvr4[i+4]='2';
    	q_Commandfwvr4[i+5]='4';
    	q_Commandfwvr4[i+6]='\r';
    	q_Commandfwvr4[i+7]='\0';
 	 }
 	 else
 	 {
 	 	 gsm_firmware_state = gsm_firmware_state + 1;
 	 }

}

/* End quectel_ftp_reception()*******************************************/


/********************************************************************************
  * @brief  tcp data transfer read from flash function
  * @param  none
  * @retval none
  *****************************************************************************/
void tcp_data_reading_from_spi_flash(void)
{
    flash_read_started=1;
	  #if BMS_TYPE_5
          ret_val1 = flash_data_read(flash_data,180);
	   #endif
	   #if BMS_TYPE_6
	  	            ret_val1 = flash_data_read(flash_data,111);
	   #endif
	   #if BMS_TYPE_3
	  			    ret_val1 = flash_data_read(flash_data,99);
	   #endif
	   #if BMS_TYPE_1 || BMS_TYPE_4
	         	ret_val1 = flash_data_read(flash_data,88);
	   #endif
	   #if BMS_TYPE_2
	  			    ret_val1 = flash_data_read(flash_data,88);
		#endif
		#if BMS_TYPE_X
			    ret_val1 = flash_data_read(flash_data,81);
		#endif
		#if BMS_TYPE_7
	    ret_val1 = flash_data_read(flash_data,118);
		#endif
		#if BMS_TYPE_8
	    ret_val1 = flash_data_read(flash_data,103);
		#endif

		if(ret_val1 == 0)
		{
			if(modem_type == LYNQ_MODEM)
			{
				gsm_tcp_state=GSM_TCP_3;
			}
			else
			{
				gsm_tcp_state=GSM_TCP_7;
			}

		}
		else
		{
			flash_store_started=0;
			if(modem_type == LYNQ_MODEM)
			{
				gsm_tcp_state=GSM_TCP_6;
			}
			else
			{
				gsm_tcp_state=GSM_TCP_10;
			}
		}

}

/* End tcp_data_reading_from_spi_flash()*******************************************/

/********************************************************************************
  * @brief  tcp data transfer read from flash function
  * @param  none
  * @retval none
  *****************************************************************************/
void tcp_data_writing_to_spi_flash(void)
{
	  server_conn_flag=0;
	  gsm_device_state= GSM_IDLE;
	  #if BMS_TYPE_1 || BMS_TYPE_4
		  flash_data_write(&cloud_tcp_tx_data[10],88);
	  #endif
	  #if BMS_TYPE_2
		  flash_data_write(cloud_tcp_tx_data,88);
	  #endif
	  #if BMS_TYPE_3
		  flash_data_write(cloud_tcp_tx_data,99);
	  #endif
	  #if BMS_TYPE_5
		  flash_data_write(&cloud_tcp_tx_data[10],180);
	  #endif
	  #if BMS_TYPE_6
		  flash_data_write(&cloud_tcp_tx_data[10],111);
	  #endif
	#if BMS_TYPE_7
	  flash_data_write(&cloud_tcp_tx_data[10],118);
	#endif
	#if BMS_TYPE_8
	  flash_data_write(&cloud_tcp_tx_data[10],103);
	#endif
	#if BMS_TYPE_X
	  flash_data_write(&cloud_tcp_tx_data[10],81);
	#endif

	  flash_store_started = 1;
	  gsm_init_flags();
	  gsm_RxBuffWRPtr  = 0;
	  gsm_reception=0;
	  tcp_data_sent_get_can_data_flag=1;
		if(modem_type == LYNQ_MODEM)
		{
			gsm_tcp_state=GSM_TCP_1;
		}
		else
		{
			gsm_tcp_state=GSM_TCP_4;
		}

}

/* End tcp_data_writing_to_spi_flash()*******************************************/


/********************************************************************************
  * @brief  sending data to server function using tcp
  * @param  none
  * @retval none
  *****************************************************************************/

void data_send_to_server(void)
{
uint8_t i=0,j=0;
	if(flash_store_started == 1)
	{
         if(gsm_command_flag == 0)
		 {

			 gsm_reception                = 1;
		 	 gsm_command_flag             = 1;
		 	 gsm_response_flag            = 0;
		 	 gsm_engine_tick              = 0;
		 	 gsm_response_data_start_flag = 0;
		 	 gsm_RxBuffWRPtr  = 0;

			 if(current_data == 1)
			 {

				 current_data=0;
				 //tcp_data_sent_get_can_data_flag=1;

#if BMS_TYPE_1 || BMS_TYPE_4

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 129);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				     APP_DEBUG_PRINT("\r\nCURRENT")
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,129);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,258)
					#endif


#endif
#if BMS_TYPE_2

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 88);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					 #ifdef APP_DEBUG
				     APP_DEBUG_PRINT("\r\nCURRENT")
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,88);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,176)
					#endif

#endif
#if BMS_TYPE_3

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 99);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					 #ifdef APP_DEBUG
				     APP_DEBUG_PRINT("\r\nCURRENT")
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,99);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,198)
					#endif

#endif
#if BMS_TYPE_5

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 190);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					 #ifdef APP_DEBUG
				     APP_DEBUG_PRINT("\r\nCURRENT")
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,190);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,380)
					#endif

#endif
#if BMS_TYPE_6

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 121);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					 #ifdef APP_DEBUG
				     APP_DEBUG_PRINT("\r\nCURRENT")
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,121);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,242)
					#endif

#endif
#if BMS_TYPE_X

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 91);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					 #ifdef APP_DEBUG
				     APP_DEBUG_PRINT("\r\nCURRENT")
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,91);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,182)
					#endif

#endif
#if BMS_TYPE_7

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 128);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					 #ifdef APP_DEBUG
				     APP_DEBUG_PRINT("\r\nCURRENT")
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,128);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,256)
					#endif

#endif
#if BMS_TYPE_8

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 113);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					 #ifdef APP_DEBUG
				     APP_DEBUG_PRINT("\r\nCURRENT")
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,113);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,226)
					#endif

#endif


			 }
			else
			{
				APP_DEBUG_PRINT("\r\nFLASH")
#if BMS_TYPE_1 || BMS_TYPE_4

					for(i=89;i<130;i++)
					{
						flash_data[i]=0;
					}
                    j=10;
			        for(i=0;i<129;i++)
				    {
					  cloud_tcp_tx_data[j+i]=flash_data[i];
				    }
					SEND_GSM_BUFF(cloud_tcp_tx_data, 129);
				    SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				    hex_str_conv(printf_buffer,cloud_tcp_tx_data,129);
				    APP_DEBUG_PRINT("\r\n\r\n")
				    APP_GSM_DEBUG(printf_buffer,258)
					#endif
#endif
#if BMS_TYPE_2

				    for(i=0;i<88;i++)
					{
						cloud_tcp_tx_data[i]=flash_data[i];
					}

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 88);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,88);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,176)
					#endif

#endif
#if BMS_TYPE_3

					    for(i=0;i<99;i++)
						{
							cloud_tcp_tx_data[i]=flash_data[i];
						}

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 99);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,99);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,198)
					#endif
#endif
#if BMS_TYPE_5
	                j=10;
				    for(i=0;i<180;i++)
					{
						cloud_tcp_tx_data[j+i]=flash_data[i];
					}
					SEND_GSM_BUFF(cloud_tcp_tx_data, 190);
				    SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				    hex_str_conv(printf_buffer,cloud_tcp_tx_data,188);
				    APP_DEBUG_PRINT("\r\n\r\n")
				    APP_GSM_DEBUG(printf_buffer,380)
					#endif
#endif
#if BMS_TYPE_6
                    j=10;
					for(i=0;i<111;i++)
					{
						cloud_tcp_tx_data[j+i]=flash_data[i];
					}

					SEND_GSM_BUFF(cloud_tcp_tx_data, 121);
				    SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				    hex_str_conv(printf_buffer,cloud_tcp_tx_data,121);
				    APP_DEBUG_PRINT("\r\n\r\n")
				    APP_GSM_DEBUG(printf_buffer,242)
					#endif
#endif
#if BMS_TYPE_X
                    j=10;
					for(i=0;i<92;i++)
					{
						cloud_tcp_tx_data[j+i]=flash_data[i];
					}

					SEND_GSM_BUFF(cloud_tcp_tx_data, 91);
				    SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				    hex_str_conv(printf_buffer,cloud_tcp_tx_data,91);
				    APP_DEBUG_PRINT("\r\n\r\n")
				    APP_GSM_DEBUG(printf_buffer,182)
					#endif
#endif
#if BMS_TYPE_7
	                j=10;
				    for(i=0;i<128;i++)
					{
						cloud_tcp_tx_data[j+i]=flash_data[i];
					}
					SEND_GSM_BUFF(cloud_tcp_tx_data, 128);
				    SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				    hex_str_conv(printf_buffer,cloud_tcp_tx_data,128);
				    APP_DEBUG_PRINT("\r\n\r\n")
				    APP_GSM_DEBUG(printf_buffer,256)
					#endif
#endif
#if BMS_TYPE_8
	                j=10;
				    for(i=0;i<114;i++)
					{
						cloud_tcp_tx_data[j+i]=flash_data[i];
					}
					SEND_GSM_BUFF(cloud_tcp_tx_data, 113);
				    SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				    hex_str_conv(printf_buffer,cloud_tcp_tx_data,113);
				    APP_DEBUG_PRINT("\r\n\r\n")
				    APP_GSM_DEBUG(printf_buffer,226)
					#endif
#endif

			 }

		 }
		 if(gsm_reception  == 1)
		 {
	         gsm_response_process(RESP_TCP_DATA);
		 }

	}
	else
	{
		if(gsm_command_flag == 0)
  	   	{

	 	 	 	 gsm_command_flag             = 1;
	 	 	 	 gsm_response_flag            = 0;
	 	 	 	 gsm_engine_tick              = 0;
	 	 	 	 gsm_response_data_start_flag = 0;
	 	 	 	 gsm_RxBuffWRPtr  = 0;
				 tcp_data_sent_get_can_data_flag=1;


#if BMS_TYPE_1 || BMS_TYPE_4
					 SEND_GSM_BUFF(cloud_tcp_tx_data, 129);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,129);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,258)
					#endif

#endif
#if BMS_TYPE_2
					 SEND_GSM_BUFF(cloud_tcp_tx_data, 88);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,86);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,176)
					#endif
#endif
#if BMS_TYPE_3
					 SEND_GSM_BUFF(cloud_tcp_tx_data, 99);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,99);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,198)
					#endif
#endif
#if BMS_TYPE_5
					 SEND_GSM_BUFF(cloud_tcp_tx_data, 190);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,190);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,380)
					#endif
#endif
#if BMS_TYPE_6
					 SEND_GSM_BUFF(cloud_tcp_tx_data, 121);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					#ifdef APP_DEBUG
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,121);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,242)
					#endif
#endif
#if BMS_TYPE_X

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 91);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					 #ifdef APP_DEBUG
				     APP_DEBUG_PRINT("\r\nCURRENT")
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,91);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,182)
					#endif

#endif

#if BMS_TYPE_7

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 128);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					 #ifdef APP_DEBUG
				     APP_DEBUG_PRINT("\r\nCURRENT")
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,128);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,256)
					#endif

#endif
#if BMS_TYPE_8

					 SEND_GSM_BUFF(cloud_tcp_tx_data, 113);
				     SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
					 #ifdef APP_DEBUG
				     APP_DEBUG_PRINT("\r\nCURRENT")
				     hex_str_conv(printf_buffer,cloud_tcp_tx_data,113);
				     APP_DEBUG_PRINT("\r\n\r\n")
				     APP_GSM_DEBUG(printf_buffer,226)
					#endif

#endif


  	   	}
		gsm_response_process(RESP_TCP_OK);

	}

}


/* End data_send_to_server()*******************************************/

/********************************************************************************
  * @brief  transfer data server
  * @param  type - bms_type
  * @param  None
  * @retval none
  *****************************************************************************/
void transfer_data_to_server(uint8_t type)
{

	switch(type)
	{
	     case 1 :
                 #if BMS_TYPE_1 || BMS_TYPE_4
	    	 	 SEND_GSM_BUFF(cloud_tcp_tx_data, 127);
	    	 	 SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
				 #ifdef APP_DEBUG
	    	 	 hex_str_conv(printf_buffer,cloud_tcp_tx_data,127);
	    	 	 APP_DEBUG_PRINT("\r\n\r\n")
	    	 	 APP_GSM_DEBUG(printf_buffer,254)
				 #endif
				 #endif
		         break;
	     case 2 :
                 #if BMS_TYPE_2
			 	 SEND_GSM_BUFF(cloud_tcp_tx_data, 86);
			 	 SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
				 #ifdef APP_DEBUG
			 	 hex_str_conv(printf_buffer,cloud_tcp_tx_data,86);
			 	 APP_DEBUG_PRINT("\r\n\r\n")
			 	 APP_GSM_DEBUG(printf_buffer,172)
				 #endif
				 #endif
		         break;
	     case 3 :
                 #if BMS_TYPE_3
	    	 	 SEND_GSM_BUFF(cloud_tcp_tx_data, 97);
	    	 	 SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
				 #ifdef APP_DEBUG
	    	 	 hex_str_conv(printf_buffer,cloud_tcp_tx_data,97);
	    	 	 APP_DEBUG_PRINT("\r\n\r\n")
	    	 	 APP_GSM_DEBUG(printf_buffer,194)
				 #endif

				 #endif
		         break;
	     case 5 :
                 #if BMS_TYPE_5
	    	 	 SEND_GSM_BUFF(cloud_tcp_tx_data, 188);
	    	 	 SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
				 #ifdef APP_DEBUG
	    	 	 hex_str_conv(printf_buffer,cloud_tcp_tx_data,188);
	    	 	 APP_DEBUG_PRINT("\r\n\r\n")
	    	 	 APP_GSM_DEBUG(printf_buffer,376)
				 #endif
				 #endif
		         break;
	     case 6 :
                 #if BMS_TYPE_6
	    	 	 SEND_GSM_BUFF(cloud_tcp_tx_data, 119);
	    	 	 SEND_GSM_BUFF("\r\n\0", my_strlen("\r\n\0"));
				 #ifdef APP_DEBUG
	    	 	 hex_str_conv(printf_buffer,cloud_tcp_tx_data,119);
	    	 	 APP_DEBUG_PRINT("\r\n\r\n")
	    	 	 APP_GSM_DEBUG(printf_buffer,238)
				 #endif
				 #endif
		         break;
	     default :
	    	      break;
	}
}
/* End transfer_data_to_server()*******************************************/



/********************************************************************************
  * @brief  decoding the reived sms function
  * @param  data- gsm buffer
  * @param  data_len- gsm buffer len
  * @retval none
  *****************************************************************************/

void decode_gsm_rx_sms_process(uint8_t *data,uint16_t data_len)
{
	uint16_t index_val=0,i=0,j=0;
	uint8_t data_start=0;

	if(data[2] == 'O' && data[3] == 'K')
	{
		gsm_rx_sms_state=GSM_RX_SMS_5;
	}
	else
	{
		if(index_data_sms_flag==0)
		{
			index_val=0;
			for(i=0;i<data_len;i++)
			{
					if(data[i] == ':')
					{
						index_val = i+2;
					}
					if(data[i] == ',')
					{
						for(j=0;j<i-index_val;j++)
						{

							gsm_rx_sms_read_index[j]=data[index_val+j];

						}

						i=data_len;

					}
			}

			j=8;
			for(i=0;gsm_rx_sms_read_index[i] != '\0';i++)
			{
				rx_sms_command2[j]=gsm_rx_sms_read_index[i];
				j++;
			}
			rx_sms_command2[j]='\r';
			rx_sms_command2[j+1]='\0';

			j=8;
			for(i=0;gsm_rx_sms_read_index[i] != '\0';i++)
			{
				rx_sms_command3[j]=gsm_rx_sms_read_index[i];
				j++;
			}
			rx_sms_command3[j]=',';
			rx_sms_command3[j+1]='0';
			rx_sms_command3[j+2]='\r';
			rx_sms_command3[j+3]='\0';


		}

		if(index_data_sms_flag == 1)
		{
			index_val=0;
			for(i=0;i<data_len;i++)
			{
				if(data[i]=='D')
				{
					index_val=i+3;
					i=i+3;
				}
				if(data[i+1]==',' && index_val !=0)
				{
					for(j=0;j<=i-index_val;j++)
					{

						gsm_rx_sms_mobile_num[j]=data[index_val+j];

					}
					i=data_len;
					gsm_rx_sms_mobile_num[j]='\0';
				}
			}

			index_val=0;
			data_start=0;
			for(i=0;i<data_len;i++)
			{
				if(data[i]=='+' && data[i+1] != 'C')
			    {
					data_start=1;
				}
				if(data[i] == '\n' && data_start == 1)
				{
					index_val=i+1;
				}
				if(data[i] == '\r' && index_val != 0)
				{

					for(j=0;j<i-index_val;j++)
					{

						gsm_rx_sms_data[j]=data[index_val+j];
						gsm_tx_sms_data[j]=data[index_val+j];
					 }
					i=data_len;
					gsm_rx_sms_data[j-1]='\0';
					gsm_tx_sms_data[j]='\0';

				}
			}

	#ifdef GSM_DEBUG_EN
	 APP_GSM_DEBUG((uint8_t*)"\n\rAPP_GSM> RXVD_MOBILE_NUM: ",my_strlen("\n\rAPP_GSM> RXVD_MOBILE_NUM: "))
	 APP_GSM_DEBUG(gsm_rx_sms_mobile_num,strlen((char*)gsm_rx_sms_mobile_num))
	 APP_GSM_DEBUG((uint8_t*)"\n\rAPP_GSM> RXVD_SMS_DATA: ",my_strlen("\n\rAPP_GSM> RXVD_SMS_DATA: "))
	 APP_GSM_DEBUG(gsm_rx_sms_data,strlen((char*)gsm_rx_sms_data))

	#endif


	 j=10;
	 for(i=1;gsm_rx_sms_mobile_num[i] != '\0';i++)
	 {
		 tx_sms_command3[j]=gsm_rx_sms_mobile_num[i];
		 j++;
	 }
		}

	}




}


/********************************************************************************
  * @brief  recived sms data parsing into different buffers
  * @param  data- recived data
  * @param  None
  * @retval none
  *****************************************************************************/

void rx_sms_command_data(uint8_t *data)
{
	   const char identifier[2] = ",";
	   char *token;
	   uint8_t i=0,temp_var=0;
	   temp_sms_data1[0]='\0';
	   temp_sms_data2[0]='\0';
	   temp_sms_data3[0]='\0';
	   temp_sms_data4[0]='\0';
	   temp_sms_data5[0]='\0';
	   temp_sms_data6[0]='\0';


	   token = strtok((char*)data, identifier);

	   while( token != NULL ) {

		  temp_var++;

		  switch(temp_var)
		  {
		  	  case 1 :
		  		  	  for(i=0;i<strlen(token);i++)
		  		  	  {
		  		  		  temp_sms_data1[i]=token[i];
		  		  	  }
		  		  	  temp_sms_data1[i]='\0';

		  		  	  break;
		  	  case 2 :
	  		  	  	  for(i=0;i<strlen(token);i++)
	  		  	  	  {
	  		  	  		  temp_sms_data2[i]=token[i];
	  		  	  	  }
	  		  	  	  temp_sms_data2[i]='\0';

		  		  	  break;
		  	  case 3 :
		  		  	  for(i=0;i<strlen(token);i++)
		  		  	  {
		  		  		  temp_sms_data3[i]=token[i];
		  		  	  }
		  		  	  temp_sms_data3[i]='\0';

		  		  	  break;
		  	  case 4 :
		  		  	  for(i=0;i<strlen(token);i++)
		  		  	  {
		  		  		  temp_sms_data4[i]=token[i];
		  		  	  }
		  		  	  temp_sms_data4[i]='\0';

		  		  	  break;
		  	  case 5 :
		  		  	  for(i=0;i<strlen(token);i++)
		  		  	  {
		  		  		  temp_sms_data5[i]=token[i];
		  		  	  }
		  		  	  temp_sms_data5[i]='\0';
		  		  	  break;
		  	  case 6 :
		  		  	  for(i=0;i<strlen(token);i++)
		  		  	  {
		  		  		  temp_sms_data6[i]=token[i];
		  		  	  }
		  		  	  temp_sms_data6[i]='\0';

		  		  	  break;

		  	 default :
		  		  	  break;

		  }



	      token = strtok(NULL, identifier);
	   }


}


/* End rx_sms_command_data()*******************************************/

/********************************************************************************
  * @brief  getting the type of data function
  * @param  data- first buffer data to find type
  * @param  None
  * @retval none
  *****************************************************************************/

uint8_t rx_sms_get_type(uint8_t *data)
{

	if(strcmp((char*)SMS_GET_VERSION_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_GET_VERSION_TYPE;
	}
	else if(strcmp((char*)SMS_GET_STATUS_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_GET_STATUS_TYPE;
	}
	else if(strcmp((char*)SMS_GET_URL_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_GET_URL_TYPE;
	}
	else if(strcmp((char*)SMS_SET_APN_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_SET_APN_TYPE;
	}
	else if(strcmp((char*)SMS_GET_APN_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_GET_APN_TYPE;
	}
	else if(strcmp((char*)SMS_SET_SERVER_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_SET_SERVER_TYPE;
	}
	else if(strcmp((char*)SMS_GET_SERVER_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_GET_SERVER_TYPE;
	}
	else if(strcmp((char*)SMS_SET_SPEED_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_SET_SPEED_TYPE;
	}
	else if(strcmp((char*)SMS_GET_SPEED_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_GET_SPEED_TYPE;
	}
	else if(strcmp((char*)SMS_SET_TIMER_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_SET_TIMER_TYPE;
	}
	else if(strcmp((char*)SMS_GET_TIMER_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_GET_TIMER_TYPE;
	}
	else if(strcmp((char*)SMS_SET_CLEAR_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_SET_CLEAR_TYPE;
	}
	else if(strcmp((char*)SMS_SET_FACTORY_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_SET_FACTORY_TYPE;
	}
	else if(strcmp((char*)SMS_SET_REBOOT_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_SET_REBOOT_TYPE;
	}
	else if(strcmp((char*)SMS_SET_FTP_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_SET_FTP_TYPE;
	}
	else if(strcmp((char*)SMS_GET_FTP_STRING,(char *)temp_sms_data1) == 0)
	{
		return SMS_GET_FTP_TYPE;
	}
	else
	{

	}

	return 0;

}


/* End rx_sms_get_type()*******************************************/

/********************************************************************************
  * @brief  storing the respective value in flash and coping to the main gprs commands
  * @param  data- message type
  * @param  None
  * @retval none
  *****************************************************************************/

void rx_sms_store_process(uint8_t command)
{
	uint16_t i=0,j=0;
	gsm_tx_sms_data_len=0;

	switch(command)
	{
		case SMS_GET_VERSION_TYPE : for(i=0;fw_ver[i] != '\0';i++)
									{
										gsm_tx_sms_data[i]=fw_ver[i];
									}
									gsm_tx_sms_data[i]='\0';
									gsm_tx_sms_flag=1;
									break;
		case SMS_GET_STATUS_TYPE  : gsm_tx_sms_data[0]='I';
									gsm_tx_sms_data[1]='M';
									gsm_tx_sms_data[2]='E';
									gsm_tx_sms_data[3]='I';
									i=3;
									for(j=0;j<15;j++)
									{
										i++;
										gsm_tx_sms_data[i]=imei_num_bytes[j];
									}

									i++;
									gsm_tx_sms_data[i]=',';
									sprintf((char*)temp_buff,",GPS:FIX by %d SATELLITES,",n_sat_in_use);
									for(j=0;temp_buff[j]!='\0';j++)
									{
										i++;
										gsm_tx_sms_data[i]=temp_buff[j];
									}

									strcat((char*)gsm_tx_sms_data,(char*)"ACC:ON\0");
									gsm_tx_sms_flag=1;

									break;
		case SMS_GET_URL_TYPE     : if(gps_sms_rx_flag == 0)
									{
										gps_sms_rx_flag = 1;
									}
									else
									{
										gps_sms_rx_flag=0;
										strcpy((char*)gsm_tx_sms_data,(char*)gps_url_tx_sms);
										i=strlen((char*)gsm_tx_sms_data);
										gsm_tx_sms_data[i]=gps_lat_ind[0];
										sprintf((char*)temp_buff,"%f",lattitude_gps);
										for(j=0;temp_buff[j]!='\0';j++)
										{
											i++;
											gsm_tx_sms_data[i]=temp_buff[j];
										}
										i++;
										gsm_tx_sms_data[i]=',';
										i++;
										gsm_tx_sms_data[i]=gps_lon_ind[0];
										sprintf((char*)temp_buff,"%f",longitude_gps);
								        for(j=0;temp_buff[j]!='\0';j++)
										{
											i++;
											gsm_tx_sms_data[i]=temp_buff[j];
										}
										gsm_tx_sms_data[i+1]='\0';
										gsm_tx_sms_flag=1;
									}

				                    break;


		case SMS_SET_APN_TYPE     : if(temp_sms_data2[0] == '0')
									{
										encode_config_data_to_store_in_flash(1,temp_sms_data3,'\0','\0','\0','\0',gsm_apn_info);
									}
									else
									{
										encode_config_data_to_store_in_flash(3,temp_sms_data3,temp_sms_data4,temp_sms_data5,'\0','\0',gsm_apn_info);
									}
									nvs_write_config_parameters();
									decode_data_from_flash(APN,gsm_apn_info);
		                            gsm_tx_sms_data_len=strlen((char*)gsm_tx_sms_data);
		                            gsm_tx_sms_data[gsm_tx_sms_data_len]='\n';
		                            gsm_tx_sms_data[gsm_tx_sms_data_len+1]='O';
		                            gsm_tx_sms_data[gsm_tx_sms_data_len+2]='K';
		                            gsm_tx_sms_data[gsm_tx_sms_data_len+3]='\0';
		                            reset_modem=1;
									gsm_tx_sms_flag=1;
									break;
		case SMS_GET_APN_TYPE     : strcpy((char*)gsm_tx_sms_data,(char*)SMS_SET_APN_STRING);
									gsm_tx_sms_data_len=strlen((char*)gsm_tx_sms_data);
									gsm_tx_sms_data[gsm_tx_sms_data_len]=',';
									gsm_tx_sms_data[gsm_tx_sms_data_len+1]='0';
									gsm_tx_sms_data[gsm_tx_sms_data_len+2]=',';
									if(modem_type == LYNQ_MODEM)
									{
										j=gsm_tx_sms_data_len+3;
										for(i=9;Command8[i] != '"';i++)
										{
											gsm_tx_sms_data[j]=Command8[i];
											j++;
										}
										if(Command8[i+1] != '\r')
										{
											gsm_tx_sms_data[gsm_tx_sms_data_len+2]='1';
											gsm_tx_sms_data[j]=',';
											j=j+1;
											for(i=i+3;Command8[i] != '"';i++)
											{
												gsm_tx_sms_data[j]=Command8[i];
												j++;
											}
											gsm_tx_sms_data[j]=',';
											j=j+1;
											for(i=i+3;Command8[i] != '"';i++)
											{
												gsm_tx_sms_data[j]=Command8[i];
												j++;
											}
										}
									}
									else
									{
										j=gsm_tx_sms_data_len+3;
										for(i=13;Command8a[i] != '"';i++)
										{
											gsm_tx_sms_data[j]=Command8a[i];
											j++;
										}
										if(Command8a[i+1] != '\r')
										{
											gsm_tx_sms_data[gsm_tx_sms_data_len+2]='1';
											gsm_tx_sms_data[j]=',';
											j=j+1;
											for(i=i+3;Command8a[i] != '"';i++)
											{
												gsm_tx_sms_data[j]=Command8a[i];
												j++;
											}
											gsm_tx_sms_data[j]=',';
											j=j+1;
											for(i=i+3;Command8a[i] != '"';i++)
											{
												gsm_tx_sms_data[j]=Command8a[i];
												j++;
											}
										}
									}
									gsm_tx_sms_data[j]='#';
									gsm_tx_sms_data[j+1]='\0';
									gsm_tx_sms_flag=1;
									break;
		case SMS_SET_SERVER_TYPE  : encode_config_data_to_store_in_flash(3,temp_sms_data2,temp_sms_data3,temp_sms_data4,'\0','\0',gsm_server_info);
									nvs_write_config_parameters();
									decode_data_from_flash(TCP_SERVER,gsm_server_info);
		                            gsm_tx_sms_data_len=strlen((char*)gsm_tx_sms_data);
		                            gsm_tx_sms_data[gsm_tx_sms_data_len]='\n';
		                            gsm_tx_sms_data[gsm_tx_sms_data_len+1]='O';
		                            gsm_tx_sms_data[gsm_tx_sms_data_len+2]='K';
		                            gsm_tx_sms_data[gsm_tx_sms_data_len+3]='\0';
									gsm_tx_sms_flag=1;
									reset_modem=1;
									break;
		case SMS_GET_SERVER_TYPE  : strcpy((char*)gsm_tx_sms_data,(char*)SMS_SET_SERVER_STRING);
									gsm_tx_sms_data_len=strlen((char*)gsm_tx_sms_data);
									gsm_tx_sms_data[gsm_tx_sms_data_len]=',';
									gsm_tx_sms_data[gsm_tx_sms_data_len+1]='1';
									gsm_tx_sms_data[gsm_tx_sms_data_len+2]=',';
									if(modem_type == LYNQ_MODEM)
									{
										j=gsm_tx_sms_data_len+3;
										for(i=19;tcp_command5[i]!='"';i++)
										{
											gsm_tx_sms_data[j]=tcp_command5[i];
											j++;
										}
										gsm_tx_sms_data[j]=',';
										j++;
										for(i=i+2;tcp_command5[i]!='\r';i++)
										{
											gsm_tx_sms_data[j]=tcp_command5[i];
											j++;
										}

									}
									else
									{
										j=gsm_tx_sms_data_len+3;
										for(i=17;q_tcp_command6[i]!='"';i++)
										{
											gsm_tx_sms_data[j]=q_tcp_command6[i];
											j++;
										}
										gsm_tx_sms_data[j]=',';
										j++;
										for(i=i+2;q_tcp_command6[i]!='\r';i++)
										{
											gsm_tx_sms_data[j]=q_tcp_command6[i];
											j++;
										}

									}
									if(gsm_tx_sms_data[gsm_tx_sms_data_len+3] == '.' || gsm_tx_sms_data[gsm_tx_sms_data_len+4] == '.' || gsm_tx_sms_data[gsm_tx_sms_data_len+5] == '.' || gsm_tx_sms_data[gsm_tx_sms_data_len+6] == '.')
									{
										gsm_tx_sms_data[gsm_tx_sms_data_len+1]='0';
									}
						     		gsm_tx_sms_data[j]='#';
									gsm_tx_sms_data[j+1]='\0';
								    gsm_tx_sms_flag=1;
									break;
		case SMS_SET_SPEED_TYPE   : encode_config_data_to_store_in_flash(3,temp_sms_data2,temp_sms_data3,temp_sms_data4,'\0','\0',gsm_overspeed_info);
									nvs_write_config_parameters();
									decode_data_from_flash(OVER_SPEED,gsm_overspeed_info);
									gsm_tx_sms_data_len=strlen((char*)gsm_tx_sms_data);
		                            gsm_tx_sms_data[gsm_tx_sms_data_len]='\n';
		                            gsm_tx_sms_data[gsm_tx_sms_data_len+1]='O';
		                            gsm_tx_sms_data[gsm_tx_sms_data_len+2]='K';
		                            gsm_tx_sms_data[gsm_tx_sms_data_len+3]='\0';
									gsm_tx_sms_flag=1;
									break;
		case SMS_GET_SPEED_TYPE   : strcpy((char*)gsm_tx_sms_data,(char*)SMS_SET_SPEED_STRING);
									gsm_tx_sms_data_len=strlen((char*)gsm_tx_sms_data);
									gsm_tx_sms_data[gsm_tx_sms_data_len]=',';
									j=gsm_tx_sms_data_len+1;
									sprintf((char*)temp_buff,"%d",over_spd_ind);
									for(i=0;temp_buff[i] != '\0';i++)
									{
										gsm_tx_sms_data[j]=temp_buff[i];
										j++;
									}
									gsm_tx_sms_data[j]=',';
									j=j+1;
									sprintf((char*)temp_buff,"%d",over_speed_ind_km);
									for(i=0;temp_buff[i] != '\0';i++)
									{
										gsm_tx_sms_data[j]=temp_buff[i];
										j++;
									}
									gsm_tx_sms_data[j]=',';
									j=j+1;
									sprintf((char*)temp_buff,"%d",over_speed_ind_to);
									for(i=0;temp_buff[i] != '\0';i++)
									{
										gsm_tx_sms_data[j]=temp_buff[i];
										j++;
									}
									gsm_tx_sms_data[j]='#';
									gsm_tx_sms_data[j+1]='\0';
									gsm_tx_sms_flag=1;
									break;
		case SMS_SET_TIMER_TYPE   :	encode_config_data_to_store_in_flash(2,temp_sms_data2,temp_sms_data3,'\0','\0','\0',gsm_srvr_comm_interval_info);
									nvs_write_config_parameters();
									decode_data_from_flash(COMMN_INFO,gsm_srvr_comm_interval_info);
									gsm_tx_sms_data_len=strlen((char*)gsm_tx_sms_data);
					                gsm_tx_sms_data[gsm_tx_sms_data_len]='\n';
						            gsm_tx_sms_data[gsm_tx_sms_data_len+1]='O';
						            gsm_tx_sms_data[gsm_tx_sms_data_len+2]='K';
						            gsm_tx_sms_data[gsm_tx_sms_data_len+3]='\0';
							        gsm_tx_sms_flag=1;
									break;
		case SMS_GET_TIMER_TYPE   : strcpy((char*)gsm_tx_sms_data,(char*)SMS_SET_TIMER_STRING);
									gsm_tx_sms_data_len=strlen((char*)gsm_tx_sms_data);
									gsm_tx_sms_data[gsm_tx_sms_data_len]=',';
									j=gsm_tx_sms_data_len+1;
									sprintf((char*)temp_buff,"%d",cloud_comm_interval_moving);
									for(i=0;temp_buff[i] != '\0';i++)
									{
										gsm_tx_sms_data[j]=temp_buff[i];
										j++;
									}
									gsm_tx_sms_data[j]=',';
									j=j+1;
									sprintf((char*)temp_buff,"%d",cloud_comm_interval_stopped);
									for(i=0;temp_buff[i] != '\0';i++)
									{
										gsm_tx_sms_data[j]=temp_buff[i];
										j++;
									}
									gsm_tx_sms_data[j]='#';
									gsm_tx_sms_data[j+1]='\0';
									gsm_tx_sms_flag=1;
									break;
		case SMS_SET_CLEAR_TYPE   : spi_flash_erase_range(WR_AD_STORAGE_ADDRS,0xf3000);
									gsm_tx_sms_data_len=strlen((char*)gsm_tx_sms_data);
									gsm_tx_sms_data[gsm_tx_sms_data_len]='\n';
									gsm_tx_sms_data[gsm_tx_sms_data_len+1]='O';
									gsm_tx_sms_data[gsm_tx_sms_data_len+2]='K';
									gsm_tx_sms_data[gsm_tx_sms_data_len+3]='\0';
									gsm_tx_sms_flag=1;
									break;
		case SMS_SET_FACTORY_TYPE : restart_chip=1;
									nvs_flash_erase();
									gsm_tx_sms_data_len=strlen((char*)gsm_tx_sms_data);
									gsm_tx_sms_data[gsm_tx_sms_data_len]='\n';
									gsm_tx_sms_data[gsm_tx_sms_data_len+1]='O';
									gsm_tx_sms_data[gsm_tx_sms_data_len+2]='K';
									gsm_tx_sms_data[gsm_tx_sms_data_len+3]='\0';
									gsm_tx_sms_flag=1;
									break;
		case SMS_SET_REBOOT_TYPE  : restart_chip=1;
									gsm_tx_sms_data_len=strlen((char*)gsm_tx_sms_data);
									gsm_tx_sms_data[gsm_tx_sms_data_len]='\n';
									gsm_tx_sms_data[gsm_tx_sms_data_len+1]='O';
									gsm_tx_sms_data[gsm_tx_sms_data_len+2]='K';
									gsm_tx_sms_data[gsm_tx_sms_data_len+3]='\0';
									gsm_tx_sms_flag=1;
									break;
		case SMS_SET_FTP_TYPE     : encode_config_data_to_store_in_flash(5,temp_sms_data2,temp_sms_data3,temp_sms_data4,temp_sms_data5,temp_sms_data6,gsm_ftp_info);
									nvs_write_config_parameters();
									decode_data_from_flash(FTP_SERVER,gsm_ftp_info);
		                            gsm_tx_sms_data_len=strlen((char*)gsm_tx_sms_data);
									gsm_tx_sms_data[gsm_tx_sms_data_len]='\n';
									gsm_tx_sms_data[gsm_tx_sms_data_len+1]='O';
									gsm_tx_sms_data[gsm_tx_sms_data_len+2]='K';
									gsm_tx_sms_data[gsm_tx_sms_data_len+3]='\0';
									gsm_tx_sms_flag=1;
									firmware_update_initiated=1;
			                        break;
		default :
				break;
	}


}


/* End rx_sms_store_process()*******************************************/

/********************************************************************************
  * @brief  encoding the configuration data to store in flash
  * @param  no of data - message type
  * @param  None
  * @retval none
  *****************************************************************************/
void encode_config_data_to_store_in_flash(uint8_t num_data,uint8_t *data0,uint8_t *data1,uint8_t *data2,uint8_t *data3,uint8_t *data4,uint8_t *encrypted_data)
{
	int  temp_len=0;
	uint8_t full_length=0,i=0,j=0;

       temp_len = my_strlen((char*)data0);
       full_length=temp_len;
       encrypted_data[1]=temp_len & 0XFF;
       j=2;
       for(i=0;i<temp_len;i++)
       {
    	   encrypted_data[j] = data0[i];
       		j++;
       }
       if(num_data >=2)
       {
    	   temp_len = my_strlen((char*)data1);
    	   encrypted_data[j]= temp_len & 0XFF;
    	   j=j+1;
    	   full_length=full_length+temp_len;
    	   for(i=0;i<temp_len;i++)
    	   {
    		   encrypted_data[j] = data1[i];
    	      j++;
    	   }
       }
       if(num_data >= 3)
       {
    	   temp_len = my_strlen((char*)data2);
    	   encrypted_data[j]= temp_len & 0XFF;
    	   j=j+1;
    	   full_length=full_length+temp_len;
    	   for(i=0;i<temp_len;i++)
    	   {
    		   encrypted_data[j] = data2[i];
    	      j++;
    	   }

       }
       if(num_data >=4)
       {
    	   temp_len = my_strlen((char*)data3);
    	   encrypted_data[j]= temp_len & 0XFF;
    	   j=j+1;
    	   full_length=full_length+temp_len;
    	   for(i=0;i<temp_len;i++)
    	   {
    		   encrypted_data[j] = data3[i];
    	      j++;
    	   }

       }
       if(num_data >= 5)
       {
    	   temp_len = my_strlen((char*)data4);
    	   encrypted_data[j]= temp_len & 0XFF;
    	   j=j+1;
    	   full_length=full_length+temp_len;
    	   for(i=0;i<temp_len;i++)
    	   {
    		   encrypted_data[j] = data4[i];
    	      j++;
    	   }

       }
       encrypted_data[0]=full_length & 0XFF;
       encrypted_data[j]='\0';

}


/* End encode_config_data_to_store_in_flash()*******************************************/

/********************************************************************************
  * @brief  decoding the data from flash
  * @param  no of data - message type
  * @param  None
  * @retval none
  *****************************************************************************/

void decode_data_from_flash(uint8_t type,uint8_t *encrypted_data)
{


	uint8_t temp_len=0,full_length=0;
	uint8_t i=0,j=0;
	uint8_t index=0;
	uint8_t temp_buff2[2],temp_value[10];


	switch(type)
	{
	case              APN:   index=1;
                             temp_buff2[0] = encrypted_data[index];
                             hex_str_conv(temp_buff,temp_buff2,1);
						     temp_len = hex_to_dec(temp_buff,2);
						   hex_str_conv(temp_buff,&encrypted_data[0],1);
						   full_length=hex_to_dec(temp_buff,2);
		                   if(modem_type == LYNQ_MODEM)
						   {
		                	   j=9;
		                	   for(i=0;i<temp_len;i++)
		                	   {
		                		   Command8[j] = encrypted_data[i+2];
		                		   j++;
		                	   }
		                	   Command8[j]='"';
		                	   j=j+1;
		                	   if(full_length != temp_len)
		                	   {
		                		   index=index+temp_len+1;
		                           temp_buff2[0] = encrypted_data[index];
		                		   hex_str_conv(temp_buff,temp_buff2,1);
		                		   temp_len=hex_to_dec(temp_buff,2);
		                		   Command8[j]=',';
		                		   j=j+1;
		                		   Command8[j]='"';
		                		   j=j+1;
		                		   for(i=0;i<temp_len;i++)
		                		   {
		                			   Command8[j]=encrypted_data[i+index+1];
		                			   j++;
		                		   }
		                		   Command8[j]='"';
		                		   j=j+1;
		                		   Command8[j]=',';
		                		   j=j+1;
		                		   Command8[j]='"';
		                		   index=index+temp_len+1;
		                		   temp_buff2[0] = encrypted_data[index];
		                		   hex_str_conv(temp_buff,temp_buff2,1);
		                		   temp_len=hex_to_dec(temp_buff,2);
		                		   j=j+1;
		                		   for(i=0;i<temp_len;i++)
		                		   {
		                			   Command8[j]=encrypted_data[i+index+1];
		                			   j++;
		                		   }
		                		   Command8[j]='"';
		                		   j=j+1;
		                	   }
		                	   Command8[j]='\r';
		                	   Command8[j+1]='\0';
		                	   Command8[j+2]='"';
						   }
						   else
						   {
		                	   j=13;
		                	   for(i=0;i<temp_len;i++)
		                	   {
		                		   Command8a[j] = encrypted_data[i+2];
		                		   j++;
		                	   }
		                	   Command8a[j]='"';
		                	   j=j+1;
		                	   if(full_length != temp_len)
		                	   {
		                		   index=index+temp_len+1;
		                           temp_buff2[0] = encrypted_data[index];
		                		   hex_str_conv(temp_buff,temp_buff2,1);
		                		   temp_len=hex_to_dec(temp_buff,2);
		                		   Command8a[j]=',';
		                		   j=j+1;
		                		   Command8a[j]='"';
		                		   j=j+1;
		                		   for(i=0;i<temp_len;i++)
		                		   {
		                			   Command8a[j]=encrypted_data[i+index+1];
		                			   j++;
		                		   }
		                		   Command8a[j]='"';
		                		   j=j+1;
		                		   Command8a[j]=',';
		                		   j=j+1;
		                		   Command8a[j]='"';
		                		   index=index+temp_len+1;
		                		   temp_buff2[0] = encrypted_data[index];
		                		   hex_str_conv(temp_buff,temp_buff2,1);
		                		   temp_len=hex_to_dec(temp_buff,2);
		                		   j=j+1;
		                		   for(i=0;i<temp_len;i++)
		                		   {
		                			   Command8a[j]=encrypted_data[i+index+1];
		                			   j++;
		                		   }
		                		   Command8a[j]='"';

		                		   j=j+1;
		                	   }
		                	   Command8a[j]='\r';
		                	   Command8a[j+1]='\0';
		                	   Command8a[j+2]='"';

						   }
					       break;

	case      TCP_SERVER :  index=3;
                            temp_buff2[0] = encrypted_data[index];
                            hex_str_conv(temp_buff,temp_buff2,1);
						    temp_len = hex_to_dec(temp_buff,2);
						   hex_str_conv(temp_buff,&encrypted_data[0],1);
						   full_length=hex_to_dec(temp_buff,2);
		                   if(modem_type == LYNQ_MODEM)
						   {
		                	   j=19;

		                	   for(i=0;i<temp_len;i++)
		                	   {
		                		   tcp_command5[j] = encrypted_data[i+4];
		                		   j++;
		                	   }
		                	   tcp_command5[j]='"';
		                	   j=j+1;
   		                       index=index+temp_len+1;
                               temp_buff2[0] = encrypted_data[index];
   		                       hex_str_conv(temp_buff,temp_buff2,1);
                               temp_len=hex_to_dec(temp_buff,2);
                               tcp_command5[j]=',';
                               j=j+1;
                               for(i=0;i<temp_len;i++)
    		                   {
   			                      tcp_command5[j]=encrypted_data[i+index+1];
                                  j++;
   		                       }
		                	   tcp_command5[j]='\r';
		                	   tcp_command5[j+1]='\0';
		                	   tcp_command5[j+2]='"';


						   }
						   else
						   {
                              q_tcp_command1[11]=encrypted_data[2];

		                	   j=17;

		                	   for(i=0;i<temp_len;i++)
		                	   {
		                		   q_tcp_command6[j] = encrypted_data[i+4];
		                		   j++;
		                	   }
		                	   q_tcp_command6[j]='"';
		                	   j=j+1;
   		                       index=index+temp_len+1;
                               temp_buff2[0] = encrypted_data[index];
   		                       hex_str_conv(temp_buff,temp_buff2,1);
                               temp_len=hex_to_dec(temp_buff,2);
                               q_tcp_command6[j]=',';
                               j=j+1;
                               for(i=0;i<temp_len;i++)
    		                   {
   			                      q_tcp_command6[j]=encrypted_data[i+index+1];
                                  j++;
   		                       }
		                	   q_tcp_command6[j]='\r';
		                	   q_tcp_command6[j+1]='\0';
		                	   q_tcp_command6[j+2]='"';

						   }
		                   break;
	   case     FTP_SERVER:
		                   if(modem_type == LYNQ_MODEM)
						   {
						   	 index=1;
                             temp_buff2[0] = encrypted_data[index];
                             hex_str_conv(temp_buff,temp_buff2,1);
						     temp_len = hex_to_dec(temp_buff,2);

						     hex_str_conv(temp_buff,&encrypted_data[0],1);
						     full_length=hex_to_dec(temp_buff,2);
                             j=12;
		                	   for(i=0;i<temp_len;i++)
		                	   {
		                		   Commandfwvr5[j] = encrypted_data[i+2];
		                		   j++;
		                	   }
		                	   Commandfwvr5[j]='"';
		                	   Commandfwvr5[j+1]='\r';
		                	   Commandfwvr5[j+2]='\0';
		                	   Commandfwvr5[j+3]='"';
		                	   j=11;
		                	   index=index+temp_len+1;
                               temp_buff2[0] = encrypted_data[index];
   		                       hex_str_conv(temp_buff,temp_buff2,1);
                               temp_len=hex_to_dec(temp_buff,2);
                               for(i=0;i<temp_len;i++)
    		                   {
   			                      Commandfwvr8[j]=encrypted_data[i+index+1];
                                  j++;
   		                       }
		                	   Commandfwvr8[j]='\r';
		                	   Commandfwvr8[j+1]='\0';
		                	   Commandfwvr8[j+2]='"';
						   }
						   else
						   {
						   	 index=1;
                             temp_buff2[0] = encrypted_data[index];
                             hex_str_conv(temp_buff,temp_buff2,1);
						     temp_len = hex_to_dec(temp_buff,2);

						     hex_str_conv(temp_buff,&encrypted_data[0],1);
						     full_length=hex_to_dec(temp_buff,2);
   	                         j=13;
		                	   for(i=0;i<temp_len;i++)
		                	   {
		                		   q_Commandfwvr2[j] = encrypted_data[i+2];
		                		   j++;
		                	   }
		                	   q_Commandfwvr2[j]='"';
                               j=j+1;
   		                       index=index+temp_len+1;
                               temp_buff2[0] = encrypted_data[index];
   		                       hex_str_conv(temp_buff,temp_buff2,1);
                               temp_len=hex_to_dec(temp_buff,2);
                               q_Commandfwvr2[j]=',';
                               j=j+1;
                               for(i=0;i<temp_len;i++)
    		                   {
   			                      q_Commandfwvr2[j]=encrypted_data[i+index+1];
                                  j++;
   		                       }
		                	   q_Commandfwvr2[j]='\r';
		                	   q_Commandfwvr2[j+1]='\0';
		                	   q_Commandfwvr2[j+2]='"';
						   }
						   if(modem_type == LYNQ_MODEM)
						   {
   		                       index=index+temp_len+1;
                               temp_buff2[0] = encrypted_data[index];
   		                       hex_str_conv(temp_buff,temp_buff2,1);
                               temp_len=hex_to_dec(temp_buff,2);
                               j=10;
                               for(i=0;i<temp_len;i++)
    		                   {
   			                      Commandfwvr6[j]=encrypted_data[i+index+1];
                                  j++;
   		                       }
   		                       Commandfwvr6[j]='"';
		                	   Commandfwvr6[j+1]='\r';
		                	   Commandfwvr6[j+2]='\0';
		                	   Commandfwvr6[j+3]='"';

						   }
						   else
						   {
						   	   index=index+temp_len+1;
                               temp_buff2[0] = encrypted_data[index];
   		                       hex_str_conv(temp_buff,temp_buff2,1);
                               temp_len=hex_to_dec(temp_buff,2);
							   j=13;
                               for(i=0;i<temp_len;i++)
    		                   {
   			                      q_Commandfwvr0[j]=encrypted_data[i+index+1];
                                  j++;
   		                       }
   		                       q_Commandfwvr0[j]='"';
		                	   q_Commandfwvr0[j+1]='\r';
		                	   q_Commandfwvr0[j+2]='\0';
		                	   q_Commandfwvr0[j+3]='"';

						   }
   						   if(modem_type == LYNQ_MODEM)
                           {
						   	   j=10;
   		                       index=index+temp_len+1;
                               temp_buff2[0] = encrypted_data[index];
   		                       hex_str_conv(temp_buff,temp_buff2,1);
                               temp_len=hex_to_dec(temp_buff,2);
                               for(i=0;i<temp_len;i++)
    		                   {
   			                      Commandfwvr7[j]=encrypted_data[i+index+1];
                                  j++;
   		                       }
   		                       Commandfwvr7[j]='"';
		                	   Commandfwvr7[j+1]='\r';
		                	   Commandfwvr7[j+2]='\0';
		                	   Commandfwvr7[j+3]='"';

                           }
                           else
						   {
						   	   j=13;
   		                       index=index+temp_len+1;
                               temp_buff2[0] = encrypted_data[index];
   		                       hex_str_conv(temp_buff,temp_buff2,1);
                               temp_len=hex_to_dec(temp_buff,2);
                               for(i=0;i<temp_len;i++)
    		                   {
   			                      q_Commandfwvr1[j]=encrypted_data[i+index+1];
                                  j++;
   		                       }
   		                       q_Commandfwvr1[j]='"';
		                	   q_Commandfwvr1[j+1]='\r';
		                	   q_Commandfwvr1[j+2]='\0';
		                	   q_Commandfwvr1[j+3]='"';

						   }
						   if(modem_type == LYNQ_MODEM)
						   {
						   	   j=15;
   		                       index=index+temp_len+1;
                               temp_buff2[0] = encrypted_data[index];
   		                       hex_str_conv(temp_buff,temp_buff2,1);
                               temp_len=hex_to_dec(temp_buff,2);
                               for(i=0;i<temp_len;i++)
    		                   {
   			                      Commandfwvr9[j]=encrypted_data[i+index+1];
                                  j++;
   		                       }
		                	   Commandfwvr9[j]='"';
		                	   Commandfwvr9[j+1]='\r';
		                	   Commandfwvr9[j+2]='\0';
		                	   Commandfwvr9[j+3]='"';
						   }
						   else
						   {
							   j=12;
   		                       index=index+temp_len+1;
                               temp_buff2[0] = encrypted_data[index];
   		                       hex_str_conv(temp_buff,temp_buff2,1);
                               temp_len=hex_to_dec(temp_buff,2);
                               for(i=0;i<temp_len;i++)
    		                   {
   			                      q_Commandfwvr4[j]=encrypted_data[i+index+1];
                                  j++;
   		                       }

   		                       temp_len = strlen((char*)q_Commandfwvr4);

		                	   q_Commandfwvr4[temp_len]='"';
		                	   q_Commandfwvr4[temp_len+1]=',';
		                	   q_Commandfwvr4[temp_len+2]='0';
		                	   q_Commandfwvr4[temp_len+3]=',';
		                	   q_Commandfwvr4[temp_len+4]='1';
		                	   q_Commandfwvr4[temp_len+5]='0';
		                	   q_Commandfwvr4[temp_len+6]='2';
		                	   q_Commandfwvr4[temp_len+7]='4';
		                	   q_Commandfwvr4[temp_len+8]='\r';
		                	   q_Commandfwvr4[temp_len+9]='\0';
		                	   q_Commandfwvr4[temp_len+10]='"';

						   }

					           break;
		case OVER_SPEED    :   index=1;
                              temp_buff2[0] = encrypted_data[index];
  		                       hex_str_conv(temp_buff,temp_buff2,1);
                              temp_len=hex_to_dec(temp_buff,2);
                              for(i=0;i<temp_len;i++)
							   {
                            	  temp_value[i]=encrypted_data[i+index+1];
							   }
                              temp_value[i]='\0';
                              over_spd_ind=atoi((char*)temp_value);
                              index=index+temp_len+1;
                              temp_buff2[0] = encrypted_data[index];
  		                       hex_str_conv(temp_buff,temp_buff2,1);
                              temp_len=hex_to_dec(temp_buff,2);
                              for(i=0;i<temp_len;i++)
							   {
                            	  temp_value[i]=encrypted_data[i+index+1];
							   }
                              temp_value[i]='\0';
                              over_speed_ind_km=atoi((char*)temp_value);
                              index=index+temp_len+1;
                              temp_buff2[0] = encrypted_data[index];
  		                       hex_str_conv(temp_buff,temp_buff2,1);
                              temp_len=hex_to_dec(temp_buff,2);
                              for(i=0;i<temp_len;i++)
							   {
                            	  temp_value[i]=encrypted_data[i+index+1];
							   }
                              temp_value[i]='\0';
                              over_speed_ind_to=atoi((char*)temp_value);
						       break;
		case COMMN_INFO     :   index=1;
                              temp_buff2[0] = encrypted_data[index];
  		                       hex_str_conv(temp_buff,temp_buff2,1);
                              temp_len=hex_to_dec(temp_buff,2);
                              for(i=0;i<temp_len;i++)
							   {
                            	  temp_value[i]=encrypted_data[i+index+1];
							   }
                              temp_value[i]='\0';
                              cloud_comm_interval_moving=atoi((char*)temp_value);
                              index=index+temp_len+1;
                              temp_buff2[0] = encrypted_data[index];
  		                       hex_str_conv(temp_buff,temp_buff2,1);
                              temp_len=hex_to_dec(temp_buff,2);
                              for(i=0;i<temp_len;i++)
							   {
                            	  temp_value[i]=encrypted_data[i+index+1];
							   }
                              temp_value[i]='\0';
                              cloud_comm_interval_stopped=atoi((char*)temp_value);
					       	   break;
		default :
			   break;
	}
}

/* End decode_data_from_flash()*******************************************/

/************** (C) COPYRIGHT 2015 Calixto Systems Pvt Ltd *****END OF FILE****/
