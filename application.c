/**
  ******************************************************************************
  * @file    application.c
  * @author  Srikanth , Calixto Firmware Team
  * @version V1.0.0
  * @date    02-JUL-2021
  * @brief   This file provides functions to manage the following
  *            breif the functionalities here:
  *           -
  *
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
  * <h2><center>&copy; COPYRIGHT 2022 Calixto Systems Pvt Ltd</center></h2>
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include"application.h"
#include "middleware.h"
#include "can.h"
#include <time.h>
#include "json.h"
#include "flash.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static TaskHandle_t cloud_comm_task_handle;
static TaskHandle_t gsm_engine_task_handle;
static TaskHandle_t uart_1_isr_task_handle;
static TaskHandle_t updation_task_handle;


esp_ota_handle_t update_handle = 0 ;
const esp_partition_t *update_partition = NULL;
const esp_partition_t *running_partition = NULL;
esp_err_t err;

uint32_t image_len=0;
uint8_t ota_write_data[1025];
bool image_header_verified=false;
bool ota_update_approved=false;
uint16_t led_timer=0,led_sequence_timer = 10;
uint8_t fixed_byte[2]={0x89,0x98};
uint8_t imei_num[7]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;  // {0x03,0x14,0x73,0x38,0x8D,0x8C,0x91}; 3   // {0x3,0x14,0x73,0x38,0x90,0xC0,0xAB};//1  //2 0x8D,0x71
uint8_t time_date[4]={0x00,0x00,0x00,0x00};  // 62CFEDAE
uint8_t lattitude[4]={0x00,0x00,0x00,0x00};
uint8_t longitude[4]={0x00,0x00,0x00,0x00};
uint8_t o_speed[1]={0x00};
uint8_t speed[4]={0x00,0x00,0x00,0x00};
uint8_t satellite_used[1]={0x00};
uint8_t reserved[1]={0x00};
uint8_t time_set_var=0,main_time_str[10];

uint8_t cloud_tcp_tx_data[190];
uint8_t bms_type[2]={0X08};

uint16_t count_timer_10min=0;
extern uint8_t tcp_port;

uint8_t  gsm_rx_data;
char 	gsm_serialRXBuff[GSM_SERIAL_BUFF_MAX];
uint16_t gsm_RxBuffWRPtr = 0;

uint8_t cloud_comm_start=0;
uint8_t send_tcp_data=0;
uint16_t cloud_commn_timer = 0;
bool rtc_time_set=false;
uint32_t rtc_time_sync_count=0;

uint8_t status_info[8];
uint8_t gps_status_flag=0;
uint8_t can_status_flag=0;


uint8_t gsm_response_flag = 0, gsm_response_data_start_flag=0;
extern uint8_t gsm_command_flag;
uint8_t gsm_fwup_flag         = 0;
uint16_t rxbuff_len = 20;
uint8_t gsm_failure_flag      = 0;

char cloud_settime[33] = "+CCLK:";
extern uint8_t rtc_time_set_flag;

uint8_t gsm_sms_rxvd=0;

float bat_voltage=0;
float lattitude_gps=0;
float longitude_gps=0;
uint8_t gps_lat_ind[2];
uint8_t gps_lon_ind[2];
uint8_t n_sat_in_use=0;

char final_bat_buff[10];

char cloud_rx_buffer[200];
char csq_val[6]                = "00,00\0";
/* Private structures --------------------------------------------------------*/
lwgps_t hgps;

can_message_t app_can_tx,app_can_rx;

uint32_t can_msg_identifier[17]={0x100,0x101,0x102,0x103,0x104,0x105,0x106,0x107,0x108,0x109,0x10A,0x10B,0x10C,0x10D,0x10E,0x10F,0x110};

uint8_t can_index[17]={31,37,43,49,55,57,63,69,75,81,87,93,99,105,111,117,123};

uint8_t temp_buff[10], temp_var=0;

char strftime_buf[65];
time_t now;
struct tm timeinfo;

uint8_t printf_buffer[400];
extern uint8_t can_timer_sec;

uint8_t gsm_apn_info[140];
uint8_t gsm_server_info[42];
uint8_t gsm_ftp_info[152];
uint8_t gsm_overspeed_info[20];
uint8_t gsm_srvr_comm_interval_info[15];

uint8_t over_spd_ind=0;
uint16_t over_speed_ind_km=50;
uint8_t over_speed_ind_to=1;

uint8_t cloud_comm_interval_moving=20;
uint8_t cloud_comm_interval_stopped=1;

uint8_t temp_notify_data[2];
extern bool dvc_ble_connected;
uint8_t server_conn_flag=0;
uint16_t temp_val=0;
char bms_type_value[2];

uint8_t battery_connected_flag_type_2=0;

uint8_t flash_stored_bms_type[2];
uint8_t dcd_detected=0;
extern uint8_t q_ftp_data_tx_cmplt;
uint8_t ftp_error_bytes=0;

extern uint8_t current_data;
uint8_t close_ftp_flag=0;
uint8_t tcp_data_sent_get_can_data_flag=0;
uint8_t gsm_transmit_sms_after_ota=0;
uint8_t gsm_transmit_sms_after_ota_done=0;
extern uint8_t gsm_tx_sms_flag;
extern uint8_t gsm_tx_sms_data[512];

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void reset_tcp_tx_data(void);
void bms_type_1_can_commn_process(void);

extern void gsm_receive_process();
extern void gsm_engine();
void notify_ble_parameters(void);
void notify_ble_parameters_type_3(void);
void notify_ble_parameters_type_5(void);
void notify_ble_parameters_type_6(void);
void notify_can_ble_parameters_as_zeros(void);

void all_types_can_communication(void);

void encode_notify_data(uint8_t *data,uint8_t len,uint8_t notify_data);
void decode_data_from_flash(uint8_t type,uint8_t *encrypted_data);




/********************************************************************************
  * @brief  application init function
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/
void app_init(void)
{

     if(nvs_read_bms_type_parameters() == 0)
     {
         if(flash_stored_bms_type[0] != bms_type[0])
         {
        	spi_flash_erase_range(0XB000,0xf3000);
    		#ifdef APP_DEBUG
    		APP_DEBUG_PRINT("\r\nBMS TYPE NOT EQUAL ")
    		#endif
        	if(nvs_write_bms_type_parameters() == 0)
        	{
        		#ifdef APP_DEBUG
        		APP_DEBUG_PRINT("\r\nBMS TYPE IS STORED")
        		#endif
        	}

         }

     }

 	flash_get_info();



	cloud_tcp_tx_data[0]=fixed_byte[0];
	cloud_tcp_tx_data[1]=fixed_byte[1];
	cloud_tcp_tx_data[2]= imei_num[0];
	cloud_tcp_tx_data[3]= imei_num[1];
	cloud_tcp_tx_data[4]= imei_num[2];
	cloud_tcp_tx_data[5]= imei_num[3];
	cloud_tcp_tx_data[6]= imei_num[4];
	cloud_tcp_tx_data[7]= imei_num[5];
	cloud_tcp_tx_data[8]= imei_num[6];
	cloud_tcp_tx_data[9]=bms_type[0];
	cloud_tcp_tx_data[10]=time_date[0];
	cloud_tcp_tx_data[11]=time_date[1];
	cloud_tcp_tx_data[12]=time_date[2];
	cloud_tcp_tx_data[13]=time_date[3];
	cloud_tcp_tx_data[14]=lattitude[0];
	cloud_tcp_tx_data[15]=lattitude[1];
	cloud_tcp_tx_data[16]=lattitude[2];
	cloud_tcp_tx_data[17]=lattitude[3];
	cloud_tcp_tx_data[18]=0x00;
	cloud_tcp_tx_data[19]=longitude[0];
	cloud_tcp_tx_data[20]=longitude[1];
	cloud_tcp_tx_data[21]=longitude[2];
	cloud_tcp_tx_data[22]=longitude[3];
	cloud_tcp_tx_data[23]=0x00;
	cloud_tcp_tx_data[24]=o_speed[0];;
	cloud_tcp_tx_data[25]=speed[0];
	cloud_tcp_tx_data[26]=speed[1];
	cloud_tcp_tx_data[27]=speed[2];
	cloud_tcp_tx_data[28]=speed[3];
	cloud_tcp_tx_data[29]=satellite_used[0];
	cloud_tcp_tx_data[30]=reserved[0];


	app_can_tx.format=0;
	app_can_tx.frame=1;
	app_can_tx.data_length=0;
	app_can_tx.identifier=0x100;

	set_led_sequence(LED_SERVER_NOT_CONNECTING);

	xTaskCreate(uart_1_isr_task, "uart_1_isr_task",3048, NULL,10, uart_1_isr_task_handle);

	xTaskCreate(cloud_comm_task, "cloud_comm_task", 3048, NULL,10, cloud_comm_task_handle);

	xTaskCreate(gsm_engine_task, "gsm_engine_task",4096, NULL,9, gsm_engine_task_handle);

	xTaskCreate(updation_task, "gsm_engine_task",4096, NULL,8, updation_task_handle);




}


/* End of Function app_init()*******************************************/



/********************************************************************************
  * @brief gsm communication task
  * @param  args
  * @param  None
  * @retval None
  *****************************************************************************/

void gsm_engine_task(void *args)
{


	while(1)
	{

		gsm_engine();

#if BMS_TYPE_2
	bms_type_2_can_rx_process();
#endif
#if BMS_TYPE_X
	bms_type_x_can_rx_process();
#endif
#if BMS_TYPE_7
	bms_type_7_can_rx_process();
#endif
#if BMS_TYPE_8
	bms_type_8_can_rx_process();
#endif
		vTaskDelay(100/ portTICK_PERIOD_MS);

	}

}

/* End of Function cloud_comm_task()*******************************************/

/********************************************************************************
  * @brief  cloud communication task
  * @param  args
  * @param  None
  * @retval None
  *****************************************************************************/

void cloud_comm_task(void *args)
{

	while(1)
	{

		if(cloud_comm_start == 1)
		{
			cloud_commn_timer++;
			count_timer_10min++;

			if(cloud_commn_timer >= cloud_comm_interval_moving*10)
			{

				#ifdef APP_DEBUG
				APP_DEBUG_PRINT("\r\nCLOUD COMMUNICATION STARTING.....")
				#endif
				 all_types_can_communication();
				 time_conversion_process();
				 send_tcp_data=1;
				 gsm_sms_rxvd=1;
				 cloud_commn_timer=0;
#ifdef APP_DEBUG
APP_DEBUG_PRINT("\r\nCURRENT")
hex_str_conv(printf_buffer,cloud_tcp_tx_data,113);
APP_DEBUG_PRINT("\r\n\r\n")
APP_GSM_DEBUG(printf_buffer,226)
#endif
			}
		}
		if(gsm_command_flag == 1)
		{
			gsm_receive_process();
		}

		vTaskDelay(100/ portTICK_PERIOD_MS);
	}

}

/* End of Function cloud_comm_task()*******************************************/

/********************************************************************************
  * @brief  Can communication function for all types
  * @param  none
  * @param  None
  * @retval None
  *****************************************************************************/

void all_types_can_communication(void)
{

		#if BMS_TYPE_1 || BMS_TYPE_4
	    reset_tcp_tx_data();
        bms_type_1_can_commn_process();
        if(can_status_flag == 0)
        {
        	notify_can_ble_parameters_as_zeros();
        }
        else
        {
        	notify_ble_parameters();
        }
		#endif

		#if BMS_TYPE_3
		 reset_tcp_tx_data();
		bms_type_3_can_tx_rx_process();
        if(can_status_flag == 0)
        {
        	notify_can_ble_parameters_as_zeros();
        }
        else
        {
			notify_ble_parameters_type_3();
        }
		#endif

		#if BMS_TYPE_5
		 reset_tcp_tx_data();
		bms_type_5_can_tx_rx_process();
        if(can_status_flag == 0)
        {
        	notify_can_ble_parameters_as_zeros();
        }
        else
        {
			notify_ble_parameters_type_5();
        }
		#endif
		#if BMS_TYPE_6
		 reset_tcp_tx_data();
        bms_type_6_can_tx_rx_process();
        if(can_status_flag == 0)
        {
        	notify_can_ble_parameters_as_zeros();
        }
        else
        {
     		notify_ble_parameters_type_6();
        }
		#endif

}
/* End of Function all_types_can_communication()*******************************************/


/********************************************************************************
  * @brief updation task
  * @param  args
  * @param  None
  * @retval None
  *****************************************************************************/

void updation_task(void *args)
{

	uint8_t i=0;
	while(1)
	{
		led_sequence_process();
		update_status_process();
		if(gsm_transmit_sms_after_ota == 1 && gsm_transmit_sms_after_ota_done == 1)
		{
			gsm_transmit_sms_after_ota=0;
			gsm_transmit_sms_after_ota_done=0;
			esp_restart();
		}

		if(dvc_ble_connected)
		{
			temp_notify_data[0]=bms_type[0];
			bms_value_notify_data(temp_notify_data,BLE_NOTIFY_BMS_BYTE,BLE_NOTIFY_DATA_LEN_1_BYTE);
		}

		if(rtc_time_set)
		{
			rtc_time_sync_count++;

			if(rtc_time_sync_count >= RTC_TIME_SYNCHRONIZATION*10)
			{

				rtc_time_sync_count = 0;
				rtc_time_set_flag=1;
				rtc_time_set = false;
			}
		}

		if(count_timer_10min >= TCP_PORT_CLOSE_OPEN_INTERVAL * 10)
	    {
			count_timer_10min=0;
			tcp_port = 1;
		}
	    if(can_timer_sec >= CAN_RX_NOT_RXVD_INTERVAL*10)
	    {
	    	   can_timer_sec=0;
	    	   reset_tcp_tx_data();
	     }


		vTaskDelay(100/ portTICK_PERIOD_MS);

	}

}

/* End of Function updation_task()*******************************************/


/********************************************************************************
  * @brief uart_1_isr_task
  * @param  args
  * @param  None
  * @retval None
  *****************************************************************************/

void uart_1_isr_task(void *args)
{

	    uart_event_t event;
	    uint8_t i=0;
	    uint8_t uart_temp_buff[1024];

	   while(1)
	   {

	        if(xQueueReceive(gsm_uart_1_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
	            switch(event.type) {
	                case UART_DATA:
	                	uart_read_bytes(EX_UART_NUM, uart_temp_buff, event.size, portMAX_DELAY);

	                	for(i=0;i<event.size;i++)
	                	{

	                		gsm_serialRXBuff[gsm_RxBuffWRPtr]=uart_temp_buff[i];
	                		if(gsm_RxBuffWRPtr < GSM_SERIAL_BUFF_MAX)
	                		{
	                			gsm_RxBuffWRPtr++;
	                		}

	                	}


	                    break;

	                case UART_FIFO_OVF:
	                    uart_flush_input(EX_UART_NUM);
	                    xQueueReset(gsm_uart_1_queue);
	                    break;
	                //Event of UART ring buffer full
	                case UART_BUFFER_FULL:
	                    uart_flush_input(EX_UART_NUM);
	                    xQueueReset(gsm_uart_1_queue);
	                    break;
	                //Event of UART RX break detected
	                case UART_BREAK:
	                    break;
	                //Event of UART parity check error
	                case UART_PARITY_ERR:
	                    break;
	                //Event of UART frame error
	                case UART_FRAME_ERR:
	                    break;
	                //UART_PATTERN_DET
	                case UART_PATTERN_DET:
	                    break;

	                default:

	                    break;
	            }
	        }

	    }
}

/* End of Function uart_1_isr_task()*******************************************/


/********************************************************************************
*  @brief  gpio isr handler
*  @param  args - isr arguements
*  @retval None
  *****************************************************************************/

void IRAM_ATTR gpio_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;

	if(gpio_num == GSM_UART_RNG_PIN)
	{
		//gsm_sms_rxvd=1;
    }

}

/* End of Function gpio_isr_handler()*******************************************/

/********************************************************************************
  * @brief  I2c master write to slave function
  * @param  slave address
  * @param  write reg address
  * @param  data to write
  * @param  size of the data
  * @retval ESP_OK - SUCCESS
  * 		ESP_FAIL-FAILURE
  * 		ESP_ERR_TIMEOUT-ON TIMEOUT
  *****************************************************************************/
esp_err_t i2c_master_write_slave(uint8_t slave_addr,uint8_t write_addr,uint8_t data[],size_t len)
{

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1 | WRITE_BIT, ACK_CHECK_EN);

        i2c_master_write_byte(cmd, write_addr, ACK_CHECK_EN);

    for (int i = 0; i < len; i++) {
        i2c_master_write_byte(cmd, data[i], ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
    {
		#ifdef APP_DEBUG
		APP_DEBUG_PRINT("\n\rI2C WRITE DONE")
		#endif
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
		#ifdef APP_DEBUG
		APP_DEBUG_PRINT("\n\rI2C Bus is busy")
		#endif
    }
    else
    {
		#ifdef APP_DEBUG
		APP_DEBUG_PRINT("\n\rI2C Write Failed")
		#endif
    }

    return ret;
}

/* End of Function i2c_master_write_slave()*******************************************/

/********************************************************************************
  * @brief  I2c master read from slave function
  * @param  slave address
  * @param  write reg address
  * @param  read data
  * @param  size of the read data
  * @retval ESP_OK - SUCCESS
  * 		ESP_FAIL-FAILURE
  * 		ESP_ERR_TIMEOUT-ON TIMEOUT
  *****************************************************************************/
esp_err_t i2c_master_read_slave(uint8_t slave_addr,uint8_t read_addr,uint8_t data[],size_t len)
{


    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

        i2c_master_write_byte(cmd, slave_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, read_addr, ACK_CHECK_EN);
        i2c_master_start(cmd);

    i2c_master_write_byte(cmd, slave_addr << 1 | READ_BIT, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/* End of Function i2c_master_read_slave()*******************************************/


/********************************************************************************
  * @brief  cloud communication task
  * @param  args
  * @param  None
  * @retval None
  *****************************************************************************/
#if BMS_TYPE_1 || BMS_TYPE_4
void bms_type_1_can_commn_process(void)
{
	uint8_t i=0;


	for(i=0;i<17;i++)
	{
		if(bms_type[0] == BMS_TYPE_4)
		{
			bms_type_4_can_battery_process();
		}

		app_can_tx.identifier=can_msg_identifier[i];
		if(bms_type_1_can_transmit_receive(&app_can_tx,&app_can_rx) == 1)
		{
			can_status_flag=1;

			#ifdef APP_DEBUG
			APP_DEBUG_PRINT("\r\nRECEPTION CAME")
			#endif
			set_led_sequence(LED_NORMAL_WORKING);
		}
		else
		{

			set_led_sequence(LED_CAN_NOT_WORKING);
			can_status_flag=0;
			#ifdef APP_DEBUG
			APP_DEBUG_PRINT("\r\nNOTHING CAME22")
		    #endif

		}

		vTaskDelay(10/ portTICK_PERIOD_MS);
	}





}
#endif
/* End of Function can_commn_process()*******************************************/

/********************************************************************************
  * @brief  notify the parameters to ble
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/
#if BMS_TYPE_6
void notify_ble_parameters_type_6(void)
{
	uint8_t temp_buff[5];
	uint8_t hex_data[2];
	uint16_t temp_val =0;
	uint8_t temp_val1=0;

	hex_data[0] = cloud_tcp_tx_data[31];
	hex_data[1] = cloud_tcp_tx_data[32];
	hex_str_conv(temp_buff,hex_data,2);
	temp_val=hex_to_dec16(temp_buff,4);
	temp_val=temp_val*0.1;
	temp_val=temp_val*100;

	temp_buff[0]= (temp_val>>8) & 0xFF;
	temp_buff[1]= temp_val & 0xFF;
	bms_value_notify_data(temp_buff,BLE_NOTIFY_BAT_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);
	get_bat_vol(temp_buff);

	hex_data[0] = cloud_tcp_tx_data[33];
	hex_data[1] = cloud_tcp_tx_data[34];
	hex_str_conv(temp_buff,hex_data,2);
	temp_val=hex_to_dec16(temp_buff,4);
	temp_val=temp_val*0.1;
	temp_val=temp_val*100;
	temp_buff[0]= (temp_val>>8) & 0xFF;
	temp_buff[1]= temp_val & 0xFF;

  	bms_value_notify_data(temp_buff,BLE_NOTIFY_CURRENT_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);

  	hex_data[0] = cloud_tcp_tx_data[40];
	hex_str_conv(temp_buff,hex_data,1);
	temp_val1=hex_to_dec(temp_buff,2);
	temp_val1= temp_val1-40;
	temp_buff[0]= temp_val1 & 0xFF;

  	bms_value_notify_data(temp_buff,BLE_NOTIFY_TEMP_BYTE,BLE_NOTIFY_DATA_LEN_1_BYTE);a

  	temp_buff[0]=cloud_tcp_tx_data[79];
  	temp_buff[1]=cloud_tcp_tx_data[80];
  	bms_value_notify_data(temp_buff,BLE_NOTIFY_CELL1_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);
  	temp_buff[0]=cloud_tcp_tx_data[81];
  	temp_buff[1]=cloud_tcp_tx_data[82];
  	bms_value_notify_data(temp_buff,BLE_NOTIFY_CELL2_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);

	hex_data[0] = cloud_tcp_tx_data[35];
	hex_str_conv(temp_buff,hex_data,1);
	temp_val1=hex_to_dec(temp_buff,2);

	temp_buff[0]= temp_val1 & 0xFF;
  	bms_value_notify_data(temp_buff,BLE_NOTIFY_SOC_BYTE,BLE_NOTIFY_DATA_LEN_1_BYTE);

}
#endif
/* End of Function notify_ble_parameters_type_5()*******************************************/


/********************************************************************************
  * @brief  notify the parameters to ble
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/
#if BMS_TYPE_5
void notify_ble_parameters_type_5(void)
{
	uint8_t temp_buff[5];
	uint8_t hex_data[2];
	uint16_t temp_val =0;
	uint8_t temp_val1=0;

	hex_data[0] = cloud_tcp_tx_data[31];
	hex_data[1] = cloud_tcp_tx_data[32];
	hex_str_conv(temp_buff,hex_data,2);
	temp_val=hex_to_dec16(temp_buff,4);
	temp_val=temp_val*0.1;
	temp_val=temp_val*100;

	temp_buff[0]= (temp_val>>8) & 0xFF;
	temp_buff[1]= temp_val & 0xFF;
	bms_value_notify_data(temp_buff,BLE_NOTIFY_BAT_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);
	//get_bat_volt(temp_buff);

	hex_data[0] = cloud_tcp_tx_data[35];
	hex_data[1] = cloud_tcp_tx_data[36];
	hex_str_conv(temp_buff,hex_data,2);
	temp_val=hex_to_dec16(temp_buff,4);
	temp_val=temp_val-30000;
	temp_val=temp_val*0.1;
	temp_val=temp_val*100;

	temp_buff[0]= (temp_val>>8) & 0xFF;
	temp_buff[1]= temp_val & 0xFF;


  	bms_value_notify_data(temp_buff,BLE_NOTIFY_CURRENT_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);

  	hex_data[0] = cloud_tcp_tx_data[158];
	hex_str_conv(temp_buff,hex_data,1);
	temp_val1=hex_to_dec(temp_buff,2);
	temp_val1= temp_val1-40;
	temp_buff[0]= temp_val1 & 0xFF;

  	bms_value_notify_data(temp_buff,BLE_NOTIFY_TEMP_BYTE,BLE_NOTIFY_DATA_LEN_1_BYTE);

	hex_data[0] = cloud_tcp_tx_data[37];
	hex_data[1] = cloud_tcp_tx_data[38];
	hex_str_conv(temp_buff,hex_data,2);
	temp_val=hex_to_dec16(temp_buff,4);
	temp_val=temp_val*0.1;
	temp_buff[0]= temp_val & 0xFF;
  	bms_value_notify_data(temp_buff,BLE_NOTIFY_SOC_BYTE,BLE_NOTIFY_DATA_LEN_1_BYTE);

  	temp_buff[0]=cloud_tcp_tx_data[62];
  	temp_buff[1]=cloud_tcp_tx_data[63];
  	bms_value_notify_data(temp_buff,BLE_NOTIFY_CELL1_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);
  	temp_buff[0]=cloud_tcp_tx_data[64];
  	temp_buff[1]=cloud_tcp_tx_data[65];
  	bms_value_notify_data(temp_buff,BLE_NOTIFY_CELL2_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);



}
#endif
/* End of Function notify_ble_parameters_type_3()*******************************************/


/********************************************************************************
  * @brief  notify the parameters to ble
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/
#if BMS_TYPE_3
void notify_ble_parameters_type_3(void)
{

  	temp_notify_data[0]=cloud_tcp_tx_data[43];
  	temp_notify_data[1]=cloud_tcp_tx_data[44];

  	encode_notify_data(temp_notify_data,2,BLE_NOTIFY_BAT_BYTE);

  	temp_notify_data[0]=cloud_tcp_tx_data[38];
  	temp_notify_data[1]=cloud_tcp_tx_data[39];
  	encode_notify_data(temp_notify_data,2,BLE_NOTIFY_CURRENT_BYTE);

  	temp_notify_data[0]=cloud_tcp_tx_data[45];
  	encode_notify_data(temp_notify_data,1,BLE_NOTIFY_TEMP_BYTE);

  	temp_notify_data[0]=cloud_tcp_tx_data[51];
  	temp_notify_data[1]=cloud_tcp_tx_data[52];
  	encode_notify_data(temp_notify_data,2,BLE_NOTIFY_CELL1_BYTE);

  	temp_notify_data[0]=cloud_tcp_tx_data[53];
  	temp_notify_data[1]=cloud_tcp_tx_data[54];
  	encode_notify_data(temp_notify_data,2,BLE_NOTIFY_CELL2_BYTE);


  	temp_notify_data[0]=cloud_tcp_tx_data[86];

  	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_SOC_BYTE,1);



}

/* End of Function notify_ble_parameters_type_3()*******************************************/
/********************************************************************************
  * @brief  notify the parameters to ble
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/

void encode_notify_data(uint8_t *data,uint8_t len,uint8_t notify_data)
{
	 uint8_t temp_buff[4],i=0;
	 uint8_t temp_buff1[2];
	 uint16_t temp_var=0;
	 uint8_t temp_val1=0,temp_val2=0;

	 hex_str_conv(temp_buff,data,len);

	 if(len == 2)
	 {

		 temp_buff1[0]=temp_buff[0];
		 temp_buff1[1]=temp_buff[1];
		 temp_val1=hex_to_dec(temp_buff1,2);
		 temp_buff1[0]=temp_buff[2];
		 temp_buff1[1]=temp_buff[3];
		 temp_val2=hex_to_dec(temp_buff1,2);
		sprintf((char*)temp_buff,"%d%d",temp_val1,temp_val2);
		for (i = 0; i < (len*2); i++)
		{
			temp_var = (temp_var << 1) + (temp_var <<3)+temp_buff[i] - '0';
		}

		temp_buff[0]= (temp_var>>8) & 0xFF;
		temp_buff[1]= temp_var & 0xFF;

      	bms_value_notify_data(temp_buff,notify_data,len);
      	if(notify_data == BLE_NOTIFY_BAT_BYTE)
      	{
      		get_bat_vol(temp_buff);
      	}
	 }
	 else
	 {
		 temp_buff1[0]=temp_buff[0];
		 temp_buff1[1]=temp_buff[1];
		 temp_val1=hex_to_dec(temp_buff1,2);
		sprintf((char*)temp_buff,"%d",temp_val1);
			for (i = 0; i < (len*2); i++)
			{
				temp_var = (temp_var << 1) + (temp_var <<3)+temp_buff[i] - '0';
			}

			temp_buff[0]= temp_var & 0xFF;

	      	bms_value_notify_data(temp_buff,notify_data,len);
	 }



}
#endif
/* End of Function encode_notify_data()*******************************************/

/********************************************************************************
  * @brief  notify_can_ble_parameters_as_zeros
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/

void notify_can_ble_parameters_as_zeros(void)
{

 	temp_notify_data[0]=0;
  	temp_notify_data[1]=0;

  	if(dvc_ble_connected)
  	{


      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_BAT_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);
      	final_bat_buff[0]='0';
      	final_bat_buff[1]='V';
      	final_bat_buff[2]='\0';

      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_CURRENT_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);

      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_TEMP_BYTE,BLE_NOTIFY_DATA_LEN_1_BYTE);

      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_CELL1_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);

      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_CELL2_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);

      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_SOC_BYTE,BLE_NOTIFY_DATA_LEN_1_BYTE);

  	}




}

/* End of Function notify_can_ble_parameters_as_zeros()*******************************************/




/********************************************************************************
  * @brief  notify the parameters to ble
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/
#if BMS_TYPE_1 || BMS_TYPE_4

void notify_ble_parameters(void)
{

  	temp_notify_data[0]=0;
  	temp_notify_data[1]=0;


  	if(dvc_ble_connected)
  	{

      	temp_notify_data[0]=cloud_tcp_tx_data[31];
      	temp_notify_data[1]=cloud_tcp_tx_data[32];
      	get_bat_vol(temp_notify_data);
      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_BAT_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);
      	temp_notify_data[0]=cloud_tcp_tx_data[33];
      	temp_notify_data[1]=cloud_tcp_tx_data[34];
      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_CURRENT_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);
      	temp_val = ((cloud_tcp_tx_data[57] << 8) | cloud_tcp_tx_data[58]) & (0XFFFF);
      	temp_val =  (temp_val*0.1)- 273.15;
      	temp_notify_data[0] = temp_val & 0XFF;
      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_TEMP_BYTE,BLE_NOTIFY_DATA_LEN_1_BYTE);
      	temp_notify_data[0]=cloud_tcp_tx_data[69];
      	temp_notify_data[1]=cloud_tcp_tx_data[70];
      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_CELL1_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);
      	temp_notify_data[0]=cloud_tcp_tx_data[71];
      	temp_notify_data[1]=cloud_tcp_tx_data[72];
      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_CELL2_BYTE,BLE_NOTIFY_DATA_LEN_2_BYTE);
      	temp_notify_data[0]=cloud_tcp_tx_data[42];
      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_SOC_BYTE,BLE_NOTIFY_DATA_LEN_1_BYTE);

  	}

}
#endif
/* End of Function notify_ble_parameters()*******************************************/


/********************************************************************************
  * @brief getting battery voltage
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/

void get_bat_vol(uint8_t *bat_vol)
{

	 uint16_t bat_volt=0;

	 hex_str_conv(temp_buff,bat_vol,2);
	 bat_volt=hex_to_dec16(temp_buff,4);

	 bat_volt=bat_volt*10;
	 bat_volt=bat_volt/1000;

	 sprintf(final_bat_buff,"%dV",bat_volt);



}

/* End of Function get_bat_vol()*******************************************/


/********************************************************************************
  * @brief  cloud communication task
  * @param  args
  * @param  None
  * @retval None
  *****************************************************************************/
void reset_tcp_tx_data(void)
{
	uint8_t i=31;
	for(i=31;i<190;i++)
	{
		cloud_tcp_tx_data[i]=0;
	}
}
/* End of Function reset_tcp_tx_data()*******************************************/


/********************************************************************************
  * @brief  imei num conv function
  * @param  imei num in string
  * @param  None
  * @retval None
  *****************************************************************************/
void imei_num_conv(uint8_t *imei_data)
{


	uint8_t i=0;
	uint64_t temp_val=0;

	for (i = 0; i < 15; i++)
	{
		temp_val = (temp_val << 1) + (temp_val <<3)+imei_data[i] - '0';
	}


	cloud_tcp_tx_data[2]= (temp_val>>48) & 0xFF;
	cloud_tcp_tx_data[3]= (temp_val>>40) & 0xFF;
	cloud_tcp_tx_data[4]= (temp_val>>32) & 0xFF;
	cloud_tcp_tx_data[5]= (temp_val>>24) & 0xFF;
	cloud_tcp_tx_data[6]= (temp_val>>16) & 0xFF;
	cloud_tcp_tx_data[7]= (temp_val>>8) & 0xFF;
	cloud_tcp_tx_data[8]= temp_val & 0xFF;



}
/* End of Function imei_num_conv()*******************************************/



/********************************************************************************
  * @brief  network service characteristics write function
  * @param  param - ble parameters
  * @retval None
  *****************************************************************************/
void bms_type_characteristics_read(esp_ble_gatts_cb_param_t *param,esp_gatt_if_t gatts_if,uint8_t *handler)
{
    esp_gatt_rsp_t read_data;
    memset(&read_data, 0, sizeof(esp_gatt_rsp_t));

	if((param->read.handle) == handler[0])
	{
		read_data.attr_value.handle = param->read.handle;
		read_data.attr_value.len = 1;
		read_data.attr_value.value[0]=bms_type[0];

	}

    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &read_data);

}

/* End of Function ble_service1_characteristics_read()*******************************************/


/********************************************************************************
  * @brief  network service characteristics write function
  * @param  param - ble parameters
  * @retval None
  *****************************************************************************/
void bms_type_characteristics_write(esp_ble_gatts_cb_param_t *param,uint8_t *handler)
{

	if((param->write.handle) == handler[0])
	{
		//bms_type[0]=param->write.value[0];
		//bms_type[1]='\0';
		//nvs_write_parameters();
		//reset_tcp_tx_data();
	}

}

/* End of Function ble_service1_characteristics_write()*******************************************/

/********************************************************************************
  * @brief  network service characteristics write function
  * @param  param - ble parameters
  * @retval None
  *****************************************************************************/
void bms_value_characteristics_read(esp_ble_gatts_cb_param_t *param,esp_gatt_if_t gatts_if,uint8_t *handler)
{
    esp_gatt_rsp_t read_data;
    memset(&read_data, 0, sizeof(esp_gatt_rsp_t));

	if((param->read.handle) == handler[0])          // error byte
	{
		read_data.attr_value.handle = param->read.handle;
		read_data.attr_value.len = 1;
		read_data.attr_value.value[0]=0x00;
	}


	if((param->read.handle) == handler[1])             // battery voltage
	{
		read_data.attr_value.handle = param->read.handle;
		read_data.attr_value.len = 2;

			read_data.attr_value.value[0]=0;
			read_data.attr_value.value[1]=0;

	}

	if((param->read.handle) == handler[2])          // current
	{
		read_data.attr_value.handle = param->read.handle;
		read_data.attr_value.len = 2;
		read_data.attr_value.value[0]=0;
		read_data.attr_value.value[1]=0;
	}

	if((param->read.handle) == handler[3])          // temperature
	{
		read_data.attr_value.handle = param->read.handle;
		read_data.attr_value.len = 1;

		read_data.attr_value.value[0]=0;
		read_data.attr_value.value[1]=0;
	}

	if((param->read.handle) == handler[4])         // cell voltage 1
	{
		read_data.attr_value.handle = param->read.handle;
		read_data.attr_value.len = 2;
		read_data.attr_value.value[0]=0;
		read_data.attr_value.value[1]=0;
	}

	if((param->read.handle) == handler[5])        // cell voltage 2
	{
		read_data.attr_value.handle = param->read.handle;
		read_data.attr_value.len = 2;
		read_data.attr_value.value[0]=0;
		read_data.attr_value.value[1]=0;
	}

	if((param->read.handle) == handler[6])        //SOC
	{
		read_data.attr_value.handle = param->read.handle;
		read_data.attr_value.len = 1;
		read_data.attr_value.value[0]=0;
		read_data.attr_value.value[1]=0;
	}

	if((param->read.handle) == handler[7])        //BMS_TYPE
	{
		read_data.attr_value.handle = param->read.handle;
		read_data.attr_value.len = 1;
		read_data.attr_value.value[0]=bms_type[0];

	}

    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &read_data);

}

/* End of Function bms_value_characteristics_read()*******************************************/




/********************************************************************************
  * @brief  firmware update init function
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/
void esp_fw_update_init(void)
{

	image_len=0;


    running_partition = esp_ota_get_running_partition();

    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);

    image_header_verified=false;
    ota_update_approved = false;
    cloud_comm_start=0;



}

/* End of Function esp_fw_update_init()*******************************************/

/********************************************************************************
  * @brief  firmware update  function
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/

void esp_firmware_update_frm_quectel(uint8_t *data,uint16_t data_size)
{
		#ifdef APP_DEBUG
       	sprintf((char*)temp_buff,"%d",data_size);
       	APP_DEBUG_PRINT("\r\nDATA SIZE  >>>>>>>>>>>>>>>>>>>>>.. ")
       	APP_DEBUG_PRINT((uint8_t*)temp_buff)
       	#endif


	uint16_t i=0,read_index_start=0,read_size=0;

	read_index_start=11;

	if(q_ftp_data_tx_cmplt ==1)
	{

		for(i=read_index_start;i<data_size;i++)
		{
			if((data[i] == '\r' && data[i+1] == '\n' && data[i+2] == '+' && data[i+3] == 'Q' && data[i+4] == 'F'))
			{
				break;
			}
		}
		read_size=i-read_index_start;
	     	 #ifdef APP_DEBUG
       	     sprintf((char*)temp_buff,"%d",read_size);
       	     APP_DEBUG_PRINT("\r\nREAD SIZE LAST >>>>>>>>>>>>>>>>>>>>>.. ")
       	     APP_DEBUG_PRINT((uint8_t*)temp_buff)
       	     #endif

		for(i=0;i<read_size;i++)
		{
			ota_write_data[i]=data[read_index_start+i];
		}
	}
	else
	{
		for(i=read_index_start;i<data_size;i++)
		{
			if((data[i] == '\r' && data[i+1] == '\n' && data[i+2] == 'O' && data[i+3] == 'K'))
			{
				break;
			}
		}

		   read_size=i-read_index_start;

			#ifdef APP_DEBUG
       	     sprintf((char*)temp_buff,"%d",read_size);
       	     APP_DEBUG_PRINT("\r\nREAD SIZE >>>>>>>>>>>>>>>>>>>>>.. ")
       	     APP_DEBUG_PRINT((uint8_t*)temp_buff)
       	     #endif

		  for(i=0;i<read_size;i++)
		  {
			ota_write_data[i]=data[read_index_start+i];
		  }

	}


	image_len=image_len+read_size;


    if(read_size > 0)
    {
    	if(image_header_verified == false)
    	{
            esp_app_desc_t new_app_info;
            if (read_size > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
            {
                memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
				#ifdef APP_DEBUG
    			APP_DEBUG_PRINT("\r\nNew firmware version")
    	        APP_DEBUG_PRINT((uint8_t*)new_app_info.version)
    			#endif

                esp_app_desc_t running_app_info;
                if (esp_ota_get_partition_description(running_partition, &running_app_info) == ESP_OK)
                {
					#ifdef APP_DEBUG
                	APP_DEBUG_PRINT("\r\nRunning firmware version:")
    	        	APP_DEBUG_PRINT((uint8_t*)running_app_info.version)
    			    #endif
                }

                if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0)
                {
					#ifdef APP_DEBUG
                	APP_DEBUG_PRINT("\r\nCurrent running version is the same as a new. We will  continue the update.")
    			    #endif
	                ota_update_approved = true;
                }
                else
                {
                	ota_update_approved = true;
                }

                if(ota_update_approved)
                {
                	image_header_verified = true;

                	err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                	if (err != ESP_OK)
                	{
							#ifdef APP_DEBUG
                			APP_DEBUG_PRINT("\r\nOTA BEGIN FAILED")
            	       	   	#endif
            	    		esp_ota_abort(update_handle);
                			close_ftp_flag = 1;

                	}
                	else
                	{
            	        #ifdef APP_DEBUG
            	        APP_DEBUG_PRINT("\r\nOTA BEGIN SUCCESS")
            	        #endif
                	}

                }
                else
                {
					#ifdef APP_DEBUG
                	APP_DEBUG_PRINT("\r\nOTA FAILED DUE TO SAME VERSION UPDATE  WILL NOT CONTINUE ROLL BACK ....")
					#endif
		            esp_ota_abort(update_handle);
                	close_ftp_flag = 1;
                }


            }
            else
            {
		     #ifdef APP_DEBUG
             APP_DEBUG_PRINT("\r\nreceived package is not fit len")
             #endif
             esp_ota_abort(update_handle);
             close_ftp_flag = 1;
            }

    	}

        if(ota_update_approved)
        {
        	err = esp_ota_write( update_handle, ota_write_data, read_size);
        	if (err != ESP_OK)
        	{
        	     #ifdef APP_DEBUG
        	     APP_DEBUG_PRINT("\r\nOTA DATA WRITE FAILED")
        	     #endif
        	     esp_ota_abort(update_handle);
        	     close_ftp_flag = 1;
        	}
        	else
        	{
        	     #ifdef APP_DEBUG
        	     sprintf((char*)temp_buff,"%d",image_len);
        	     APP_DEBUG_PRINT("\r\nIMAGE SIZE : ")
        	     APP_DEBUG_PRINT((uint8_t*)temp_buff)
        	     #endif
        	     #ifdef APP_DEBUG
        	     APP_DEBUG_PRINT("\r\nOTA DATA WRITE SUCCEESS")
        	     #endif
        	}
        }

   }

    if(q_ftp_data_tx_cmplt == 1)
    {
    	q_ftp_data_tx_cmplt=0;

		#ifdef APP_DEBUG
    	sprintf((char*)temp_buff,"%d",image_len);
    	APP_DEBUG_PRINT("\r\nTOTALIMAGE SIZE : ")
    	APP_DEBUG_PRINT((uint8_t*)temp_buff)
		#endif

		err = esp_ota_end(update_handle);
    	if (err != ESP_OK)
    	{
    		if (err == ESP_ERR_OTA_VALIDATE_FAILED)
    		{
				#ifdef APP_DEBUG
    			APP_DEBUG_PRINT("\r\nImage validation failed, image is corrupted")
				#endif
    		}
			#ifdef APP_DEBUG
    		APP_DEBUG_PRINT("\r\nesp_ota_end failed")
			#endif

    		close_ftp_flag = 1;
    	}
    	else
    	{
    		err = esp_ota_set_boot_partition(update_partition);
    		if (err != ESP_OK)
    		{
				#ifdef APP_DEBUG
    			APP_DEBUG_PRINT("\r\nesp_ota_set_boot_partition failed")
				#endif
    		}
    		else
    		{
				#ifdef APP_DEBUG
    			APP_DEBUG_PRINT("\r\nOTA FINISHED AND PREPARING TO RESTART.............")
				#endif
				esp_restart();

    		}

    	}

    }



}


/********************************************************************************
  * @brief  firmware update  function
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/

void esp_firmware_update(uint8_t *data,uint16_t data_size)
{

	uint16_t i=0,j=0,k=0;
	uint8_t temp_buff[10];

	uint16_t write_index_start =0,read_size=0;

	for(i=0;data[i] !=',';i++)
	{
		if(data[i+1] == ',')
		{
			k=0;
			for(j=i+2;data[j] != '\r';j++)
			{
				temp_buff[k]=data[j];
				k++;
			}
			write_index_start= j+2;
		}

	}

	read_size = atoi((char*)temp_buff);

	#ifdef APP_DEBUG
	APP_DEBUG_PRINT("\r\nSIZE : ")
	APP_DEBUG_PRINT((uint8_t*)temp_buff)
	#endif

	image_len=image_len+read_size;

		for(i=0;i<read_size;i++)
		{
			ota_write_data[i]=data[write_index_start+i];
		}


  //  strncpy(ota_data, (char*)&data[write_index_start], write_size);

    if(read_size > 0)
    {
    	if(image_header_verified == false)
    	{
            esp_app_desc_t new_app_info;
            if (read_size > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
            {
                memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
				#ifdef APP_DEBUG
    			APP_DEBUG_PRINT("\r\nNew firmware version")
    	        APP_DEBUG_PRINT((uint8_t*)new_app_info.version)
    			#endif

                esp_app_desc_t running_app_info;
                if (esp_ota_get_partition_description(running_partition, &running_app_info) == ESP_OK)
                {
					#ifdef APP_DEBUG
                	APP_DEBUG_PRINT("\r\nRunning firmware version:")
    	        	APP_DEBUG_PRINT((uint8_t*)running_app_info.version)
    			    #endif
                }


                if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0)
                {
					#ifdef APP_DEBUG
                	APP_DEBUG_PRINT("\r\nCurrent running version is the same as a new. We will continue the update.")
    			    #endif
	                ota_update_approved = true;
                }
                else
                {
                	ota_update_approved = true;
                }

                if(ota_update_approved)
                {
                	image_header_verified = true;

                	err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
                	if (err != ESP_OK)
                	{
							#ifdef APP_DEBUG
                			APP_DEBUG_PRINT("\r\nOTA BEGIN FAILED")
            	       	   	#endif
            	    		esp_ota_abort(update_handle);
                			close_ftp_flag = 1;
                	}
                	else
                	{
            	        #ifdef APP_DEBUG
            	        APP_DEBUG_PRINT("\r\nOTA BEGIN SUCCESS")
            	        #endif
                	}

                }
                else
                {
					#ifdef APP_DEBUG
                	APP_DEBUG_PRINT("\r\nOTA FAILED DUE TO SAME VERSION UPDATE  WILL NOT CONTINUE ROLL BACK ....")
					#endif
		            esp_ota_abort(update_handle);
                	close_ftp_flag = 1;
                }


            }
            else
            {
		     #ifdef APP_DEBUG
             APP_DEBUG_PRINT("\r\nreceived package is not fit len")
             #endif
             esp_ota_abort(update_handle);
             close_ftp_flag = 1;
            }

    	}

        if(ota_update_approved && data[11] != '1')
        {
        	err = esp_ota_write( update_handle, (const void *)ota_write_data, read_size);
        	if (err != ESP_OK)
        	{
        	     #ifdef APP_DEBUG
        	     APP_DEBUG_PRINT("\r\nOTA DATA WRITE FAILED")
        	     #endif
        	     esp_ota_abort(update_handle);
        	     close_ftp_flag=1;
        	}
        	else
        	{
        	     #ifdef APP_DEBUG
        	     sprintf((char*)temp_buff,"%d",image_len);
        	     APP_DEBUG_PRINT("\r\nTOTALIMAGE SIZE : ")
        	     APP_DEBUG_PRINT((uint8_t*)temp_buff)
        	     #endif
        	     #ifdef APP_DEBUG
        	     APP_DEBUG_PRINT("\r\nOTA DATA WRITE SUCCEESS")
        	     #endif
        	}
        }

   }

	if(data[data_size-3] == '0' || data[11] == '1')
	{

			#ifdef APP_DEBUG
		    sprintf((char*)temp_buff,"%d",image_len);
			APP_DEBUG_PRINT("\r\nIMAGE SIZE : ")
	        APP_DEBUG_PRINT((uint8_t*)temp_buff)
			#endif
		   err = esp_ota_end(update_handle);
		    if (err != ESP_OK)
		    {
		        if (err == ESP_ERR_OTA_VALIDATE_FAILED)
		        {
					#ifdef APP_DEBUG
		        	APP_DEBUG_PRINT("\r\nImage validation failed, image is corrupted")
					#endif

		        }
				#ifdef APP_DEBUG
		        APP_DEBUG_PRINT("\r\nesp_ota_end failed")
				#endif
		        close_ftp_flag = 1;
		    }
		    else
		    {
			    err = esp_ota_set_boot_partition(update_partition);
				    if (err != ESP_OK)
				    {
						#ifdef APP_DEBUG
				        APP_DEBUG_PRINT("\r\nesp_ota_set_boot_partition failed")
						#endif

				    }
				    else
				    {
						#ifdef APP_DEBUG
				    	APP_DEBUG_PRINT("\r\nOTA FINISHED AND PREPARING TO RESTART.............")
						#endif

	 		 	 	 		strcpy((char*)gsm_tx_sms_data,"FOTA UPDATION SUCCESS");
	 		 	 	 		gsm_tx_sms_flag=1;
	 		 	 	 		gsm_transmit_sms_after_ota=1;

						//esp_restart();

				    }

		    }



	}



}


/* End of Function esp_firmware_update()*******************************************/



/********************************************************************************
  * @brief  time conversion process
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/
void time_conversion_process(void)
{


	long time_value=0;


    time_t epoch_time;

    time(&now);
    localtime_r(&now, &timeinfo);

    epoch_time = mktime(&timeinfo);
    sprintf((char*)main_time_str,"%ld",epoch_time);

    time_value=atol((char*)main_time_str);
    time_value=time_value-19800;

	cloud_tcp_tx_data[13]= time_value & 0xFF;
	cloud_tcp_tx_data[12]= (time_value>>8) & 0xFF;
	cloud_tcp_tx_data[11]= (time_value>>16) & 0xFF;
	cloud_tcp_tx_data[10]= (time_value>>24) & 0xFF;

}

/* End of Function time_conversion_process()*******************************************/

/********************************************************************************
  * @brief  updating status through reserved bit
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/

void update_status_process(void)
{
	uint8_t i=0,j=1;

	long int temp=0, hexadecimal = 0, remain;


	if(server_conn_flag == 1)
	{
		status_info[7]=1;
	}
	else
	{
		status_info[7]=0;
	}
	if(gps_status_flag == 1)
	{
		status_info[6]=1;
	}
	else
	{
		status_info[6]=0;
	}
	if(can_status_flag == 1)
	{
		status_info[5]=1;
	}
	else
	{
		status_info[5]=0;
	}
	if(battery_connected_flag_type_2 == 1)
	{
		status_info[4]=1;
	}
	else
	{
		status_info[4]=0;
	}

    for (i = 0;i<8; i++)
	{
	   	temp = (temp <<1) + (temp <<3) + status_info[i] - '0';
	}


	while (temp != 0)
    {
      remain = temp % 10;
      hexadecimal = hexadecimal + remain * j;
      j = j * 2;
      temp = temp / 10;
    }


	cloud_tcp_tx_data[30]= hexadecimal & 0xFF;

  	if(dvc_ble_connected)
  	{
      	temp_notify_data[0]=cloud_tcp_tx_data[30];
      	bms_value_notify_data(temp_notify_data,BLE_NOTIFY_ERROR_BYTE,BLE_NOTIFY_DATA_LEN_1_BYTE);
  	}

}




/* End of Function time_conversion_process()*******************************************/


/********************************************************************************
  * @brief  gps data process
  * @param  gps string
  * @param  None
  * @retval None
  *****************************************************************************/

void gps_data_process(uint8_t *gps_rx_data)
{
	//APP_GSM_DEBUG(gps_rx_data, my_strlen((char*)gps_rx_data))


	unsigned long temp_double_val=0;
	float temp_float_val=0;

	  const char identifier[2] = ",";
	   char *token;
	   uint8_t temp_var=0,temp_int_var=0,i=0;


	lwgps_init(&hgps);
	lwgps_process(&hgps, gps_rx_data, strlen((char*)gps_rx_data));

	if(hgps.p.stat == STAT_GGA)
	{
		   gps_lat_ind[0]='\0';
		   gps_lon_ind[0]='\0';
		   n_sat_in_use=0;

/*

		temp_float_val=hgps.latitude;
		lattitude_gps=temp_float_val;
		temp_double_val = *(unsigned long*)&temp_float_val;

		cloud_tcp_tx_data[17]= temp_double_val & 0xFF;
		cloud_tcp_tx_data[16]= (temp_double_val>>8) & 0xFF;
		cloud_tcp_tx_data[15]= (temp_double_val>>16) & 0xFF;
		cloud_tcp_tx_data[14]= (temp_double_val>>24) & 0xFF;

		temp_float_val=hgps.longitude;
		longitude_gps = temp_float_val;
		temp_double_val = *(unsigned long*)&temp_float_val;

		cloud_tcp_tx_data[21]= temp_double_val & 0xFF;
		cloud_tcp_tx_data[20]= (temp_double_val>>8) & 0xFF;
		cloud_tcp_tx_data[19]= (temp_double_val>>16) & 0xFF;
		cloud_tcp_tx_data[18]= (temp_double_val>>24) & 0xFF;
*/

		cloud_tcp_tx_data[29] = hgps.sats_in_use & 0XFF;
		n_sat_in_use=hgps.sats_in_use;

		if(hgps.sats_in_use >= 4)
		{
			gps_status_flag=1;
			   token = strtok((char*)gps_rx_data, identifier);
			   while( token != NULL )
			    {
						  temp_var++;
						  switch(temp_var)
						  {
						  	  case 3:
								       temp_buff[0]=token[0];
						  	           temp_buff[1]=token[1];
						  	           temp_buff[2]='\0';
						  	           temp_int_var= atoi((char*)temp_buff);
						  	           cloud_tcp_tx_data[14] = temp_int_var & 0XFF;
								       temp_buff[0]=token[2];
						  	           temp_buff[1]=token[3];
						  	           temp_buff[2]='\0';
						  	           temp_int_var= atoi((char*)temp_buff);
						  	           cloud_tcp_tx_data[15] = temp_int_var & 0XFF;
								       temp_buff[0]=token[5];
						  	           temp_buff[1]=token[6];
						  	           temp_buff[2]='\0';
						  	           temp_int_var= atoi((char*)temp_buff);
						  	           cloud_tcp_tx_data[16] = temp_int_var & 0XFF;
								       temp_buff[0]=token[7];
						  	           temp_buff[1]=token[8];
						  	           temp_buff[2]='\0';
						  	           temp_int_var= atoi((char*)temp_buff);
						  	           cloud_tcp_tx_data[17] = temp_int_var & 0XFF;
						  	  	      break;

						  	  case 4 : if(token[0] == 'N')
			                          {
						  		        cloud_tcp_tx_data[18]=0X4E;
			      	                  }
			      	                  else
									  {
			      	                	cloud_tcp_tx_data[18]=0X53;
									   }
										 gps_lat_ind[0]=token[0];
						  		  	  gps_lat_ind[1]='\0';
						  		  	  break;
			                  case 5  : temp_buff[0]=token[0];
						  	           temp_buff[1]=token[1];
						  	           temp_buff[2]=token[2];
						  	           temp_buff[3]='\0';
						  	           temp_int_var= atoi((char*)temp_buff);
						  	           cloud_tcp_tx_data[19] = temp_int_var & 0XFF;
								       temp_buff[0]=token[3];
						  	           temp_buff[1]=token[4];
						  	           temp_buff[2]='\0';
						  	           temp_int_var= atoi((char*)temp_buff);
						  	           cloud_tcp_tx_data[20] = temp_int_var & 0XFF;
								       temp_buff[0]=token[6];
						  	           temp_buff[1]=token[7];
						  	           temp_buff[2]='\0';
						  	           temp_int_var= atoi((char*)temp_buff);
						  	           cloud_tcp_tx_data[21] = temp_int_var & 0XFF;
								       temp_buff[0]=token[8];
						  	           temp_buff[1]=token[9];
						  	           temp_buff[2]='\0';
						  	           temp_int_var= atoi((char*)temp_buff);
						  	           cloud_tcp_tx_data[22] = temp_int_var & 0XFF;

			                  	      break;
						  	  case 6 :if(token[0] == 'E')
			                          {
						  		  	  	  cloud_tcp_tx_data[23]=0X45;
			      	                  }
			      	                  else
									  {
			      	                	  cloud_tcp_tx_data[23]=0X57;
									  }
										gps_lon_ind[0]=token[0];
						  		  	  gps_lon_ind[1]='\0';
						  		  	  break;

						  	 default :
						  		  	  break;

						  }



					      token = strtok(NULL, identifier);
					   }



		}
		else
		{
			gps_status_flag=0;
			for(i=14;i<24;i++)
			{
				cloud_tcp_tx_data[i]=0;
			}
		}

	}
	if(hgps.p.stat == STAT_RMC)
	{

		if(hgps.speed >= 30)
		{
	   	   cloud_tcp_tx_data[24]=0x01;
		}
		else
		{
	   	   cloud_tcp_tx_data[24]=0x00;
		}

		temp_float_val=hgps.speed;
		temp_double_val = *(unsigned long*)&temp_float_val;

		cloud_tcp_tx_data[28]= temp_double_val & 0xFF;
		cloud_tcp_tx_data[27]= (temp_double_val>>8) & 0xFF;
		cloud_tcp_tx_data[26]= (temp_double_val>>16) & 0xFF;
		cloud_tcp_tx_data[25]= (temp_double_val>>24) & 0xFF;

	}

}


/* End of Function gps_data_process()*******************************************/


/********************************************************************************
* @brief  Function for setting rtc time and date.
* @param  hr  :- hour
* @param  min :- minute
* @param  sec :- seconds
* @param  day :- day
* @param  month:-month
* @param  year :-year
* @param  wkdy :- week_day
* @retval HAL_STATUS_OK    -> success
*		  HAL_STATUS_ERROR -> failure
  *****************************************************************************/
uint8_t esp32s2_rtc_set_timedate(uint8_t hr, uint8_t min, uint8_t sec,uint8_t day,uint8_t month,uint8_t year)
{


    struct tm time;

    uint16_t year_val= 2000+year;
    time.tm_year = year_val-1900;

    time.tm_mon = month-1;

    time.tm_mday = day;

    time.tm_hour = hr;

    time.tm_min = min;

    time.tm_sec = sec;


    time_t t = mktime(&time);

    struct timeval set_time = { .tv_sec = t };

    settimeofday(&set_time, NULL) ;


    return 0;

}

/* End of function esp32s2_rtc_set_time()***********************************************/



/********************************************************************************
  * @brief  Function for setting the RTC time
  * @param  none
  * @retval none
  *****************************************************************************/
void app_set_rtc(void)
{
	char yy[3], mm[3], dd[3];
	char hh[3], min[3], ss[3];
	//char temp_print[3];

	uint8_t temp_app_date_mm, temp_app_date_dd, temp_app_date_yy;
	uint8_t temp_app_time_min, temp_app_time_hour, temp_app_time_sec;

    //+CCLK: "80/01/06,16:46:13+22"
    cloud_settime[32] = '\0';

    yy[0] = cloud_settime[8];
    yy[1] = cloud_settime[9];
    yy[2] = '\0';

	temp_app_date_yy = (uint8_t)char_to_dec(yy);

	if (temp_app_date_yy != 04)
     {
		rtc_time_set = true;
		rtc_time_set_flag  = 0;

       mm[0] = cloud_settime[11];
       mm[1] = cloud_settime[12];
       mm[2] = '\0';
       temp_app_date_mm = (uint8_t)char_to_dec(mm);

       dd[0] = cloud_settime[14];
       dd[1] = cloud_settime[15];
       dd[2] = '\0';
       temp_app_date_dd = (uint8_t)char_to_dec(dd);

       hh[0] = cloud_settime[17];
       hh[1] = cloud_settime[18];
       hh[2] = '\0';
       temp_app_time_hour = (uint8_t)char_to_dec(hh);

       min[0] = cloud_settime[20];
       min[1] = cloud_settime[21];
       min[2] = '\0';
       temp_app_time_min = (uint8_t)char_to_dec(min);

       ss[0] = cloud_settime[23];
       ss[1] = cloud_settime[24];
       ss[2] = '\0';
       temp_app_time_sec = (uint8_t)char_to_dec(ss);



       esp32s2_rtc_set_timedate(temp_app_time_hour, temp_app_time_min, temp_app_time_sec,temp_app_date_dd, temp_app_date_mm, temp_app_date_yy);


     }
	else
	{
		rtc_time_set_flag = 1;
	}
}
/* End app_set_rtc()*******************************************/


/********************************************************************************
  * @brief  set led blink sequence depending on wifi modes
  * @param  led_seq  - state mode
  * @param  None
  * @retval None
  *****************************************************************************/

void set_led_sequence(uint8_t led_seq)
{
	 led_timer=0;
	  switch(led_seq)
	  {
	  	  case LED_NORMAL_WORKING				:led_sequence_timer = 30;     //3 sec
		                         	 	 	 	 break;
	  	  case LED_SERVER_NOT_CONNECTING	    :led_sequence_timer = 1;
	  	                               	   	 	 break;
	  	  case LED_CAN_NOT_WORKING				:led_sequence_timer = 10;      //1 sec
	  	                               	   	 	 break;

	  	  default:
	  		  	  break;
	  }
}
/* End of Function set_led_sequence()*******************************************/


/********************************************************************************
  * @brief  toggle led in different interval sequence
  * @param  None
  * @param  None
  * @retval None
  *****************************************************************************/

void led_sequence_process(void)
{
  static uint8_t led_status = 0;
  led_timer++;

  if(led_timer >= led_sequence_timer){
	  led_timer=0;

	  if(led_status == 0){

	    gpio_set_level(LED_PIN,led_status);  // LED reset
	    led_status =1;

	  }
	  else{

		 gpio_set_level(LED_PIN,led_status);  // LED set
		  led_status =0;

	  }

  }


}
/* End of Function led_sequence_process()*******************************************/

/********************************************************************************
  * @brief  write the parameters to nvs
  * @param  None
  * @param  None
  * @retval 0-success
  *         1-failure
  *****************************************************************************/

uint8_t nvs_write_bms_type_parameters(void)
{

	esp_err_t err;
	nvs_handle_t my_handle;
	err = nvs_open("nvs_operation", NVS_READWRITE, &my_handle);
	if (err == ESP_OK)
    {
			nvs_set_str(my_handle ,APN_INFO, (char*)gsm_apn_info);
			nvs_set_str(my_handle ,SERVER_INFO , (char*)gsm_server_info);
			nvs_set_str(my_handle ,FTP_INFO, (char*)gsm_ftp_info);
			nvs_set_str(my_handle ,OVER_SPEED_INFO, (char*)gsm_overspeed_info);
			nvs_set_str(my_handle ,COMMN_INTERVAL_INFO,(char*)gsm_srvr_comm_interval_info);
			nvs_set_str(my_handle ,BMS_TYPE_KEY_NVS, (char*)bms_type);
	        nvs_commit(my_handle);
	        nvs_close(my_handle);
	        return 0;
	 }
	 else
	 {
	    	return 1;
	 }

}

/* End of Function nvs_write_bms_type_parameters()*******************************************/

/********************************************************************************
  * @brief  write the parameters to nvs
  * @param  None
  * @param  None
  * @retval 0-success
  *         1-failure
  *****************************************************************************/

uint8_t nvs_write_config_parameters(void)
{

	#ifdef APP_DEBUG
	APP_DEBUG_PRINT("\r\nWRITING CONFIG TO FLASH..........")
	#endif
	esp_err_t err;
	nvs_handle_t my_handle;
	err = nvs_open("NVS_STORE", NVS_READWRITE, &my_handle);
	if (err == ESP_OK)
    {
			nvs_set_str(my_handle ,APN_INFO, (char*)gsm_apn_info);
			nvs_set_str(my_handle ,SERVER_INFO , (char*)gsm_server_info);
			nvs_set_str(my_handle ,FTP_INFO, (char*)gsm_ftp_info);
			nvs_set_str(my_handle ,OVER_SPEED_INFO, (char*)gsm_overspeed_info);
			nvs_set_str(my_handle ,COMMN_INTERVAL_INFO,(char*)gsm_srvr_comm_interval_info);
	        nvs_commit(my_handle);
	        nvs_close(my_handle);

			#ifdef APP_DEBUG
			APP_DEBUG_PRINT("\r\nCONFIGURATION PARAMETERS WRITE TO FLASH SUCCESS")
			#endif
	        return 0;
	 }
	 else
	 {
			#ifdef APP_DEBUG
		    APP_DEBUG_PRINT("\r\nCONFIGURATION PARAMETERS WRITE TO FLASH FAILED")
			#endif
	    	return 1;
	 }

}

/* End of Function nvs_write_config_parameters()*******************************************/


/********************************************************************************
  * @brief  read the parameters from nvs
  * @param  None
  * @param  None
  * @retval 0-success
  *         1-failure
  *****************************************************************************/


uint8_t nvs_read_bms_type_parameters(void)
{
	esp_err_t err;
	nvs_handle_t my_handle;
	size_t data_len=0;


	    err = nvs_open("nvs_operation", NVS_READWRITE, &my_handle);
	    if (err == ESP_OK)
	    {
	        nvs_get_str(my_handle, BMS_TYPE_KEY_NVS,NULL,&data_len);
		    nvs_get_str(my_handle, BMS_TYPE_KEY_NVS,(char*)flash_stored_bms_type,&data_len);

	        nvs_commit(my_handle);
	        nvs_close(my_handle);
	        return 0 ;
	    }
	    else
	    {
	    	return 1;

	    }


}
/* End of Function nvs_read_bms_type_parameters()*******************************************/

/********************************************************************************
  * @brief  read the parameters from nvs
  * @param  None
  * @param  None
  * @retval 0-success
  *         1-failure
  *****************************************************************************/


uint8_t nvs_read_config_parameters(void)
{

	#ifdef APP_DEBUG
    APP_DEBUG_PRINT("\r\nREADING CONFIG FROM FLASH..........")
	#endif
	esp_err_t err;
	nvs_handle_t my_handle;
	size_t data_len=0;

	    err = nvs_open("NVS_STORE", NVS_READWRITE, &my_handle);
	    if (err == ESP_OK)
	    {

	        nvs_get_str(my_handle, APN_INFO,NULL,&data_len);
            nvs_get_str(my_handle, APN_INFO, (char*)gsm_apn_info,&data_len);

            if(data_len > 1)
            {
               decode_data_from_flash(APN,gsm_apn_info);
            }
            else
            {
    		   #ifdef APP_DEBUG
               APP_DEBUG_PRINT("\r\nNO DATA OF APN INFO IN FLASH")
				#endif
            }

	        nvs_get_str(my_handle, SERVER_INFO,NULL,&data_len);
	        nvs_get_str(my_handle, SERVER_INFO , (char*)gsm_server_info,&data_len);
            if(data_len > 1)
            {
            	decode_data_from_flash(TCP_SERVER,gsm_server_info);
            	APP_DEBUG_PRINT(gsm_server_info)
            }
            else
            {
				#ifdef APP_DEBUG
            	APP_DEBUG_PRINT("\r\nNO DATA OF SERVER_INFO  IN FLASH")
				#endif
            }

	        nvs_get_str(my_handle, FTP_INFO,NULL,&data_len);
		    nvs_get_str(my_handle, FTP_INFO, (char*)gsm_ftp_info,&data_len);
            if(data_len > 1)
            {
            	decode_data_from_flash(FTP_SERVER,gsm_ftp_info);
            }
            else
            {
				#ifdef APP_DEBUG
            	APP_DEBUG_PRINT("\r\nNO DATA OF FTP_INFO  IN FLASH")
				#endif
            }

	        nvs_get_str(my_handle, OVER_SPEED_INFO,NULL,&data_len);
		    nvs_get_str(my_handle, OVER_SPEED_INFO, (char*)gsm_overspeed_info,&data_len);
            if(data_len > 1)
            {
            	decode_data_from_flash(OVER_SPEED,gsm_overspeed_info);
            }
            else
            {
				#ifdef APP_DEBUG
            	APP_DEBUG_PRINT("\r\nNO DATA OF OVER_SPEED_INFO  IN FLASH")
				#endif
            }


	        nvs_get_str(my_handle, COMMN_INTERVAL_INFO,NULL,&data_len);
		    nvs_get_str(my_handle, COMMN_INTERVAL_INFO,(char*)gsm_srvr_comm_interval_info,&data_len);
            if(data_len > 1)
            {
            	decode_data_from_flash(COMMN_INFO,gsm_srvr_comm_interval_info);
            }
            else
            {
				#ifdef APP_DEBUG
            	APP_DEBUG_PRINT("\r\nNO DATA OF COMMN_INTERVAL_INFO  IN FLASH")
				#endif
            }

	        nvs_commit(my_handle);
	        nvs_close(my_handle);
	        return 0 ;
	    }
	    else
	    {
			#ifdef APP_DEBUG
	    	APP_DEBUG_PRINT("\r\nREADING CONFIG FROM FLASH FAILED")
			#endif
	    	return 1;

	    }

}
/* End of Function nvs_read_config_parameters()*******************************************/



