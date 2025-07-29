/**
  ******************************************************************************
  * @file    application.h
  * @author  Srikanth , Calixto Firmware Team
  * @version V1.0.0
  * @date    07-APR-2022
  * @brief   This file contains all the functions prototypes for the ____.
  ******************************************************************************
  *
  * <h2><center>&copy; COPYRIGHT 2022 Calixto Systems Pvt Ltd</center></h2>
  ******************************************************************************
  */

#ifndef COMPONENTS_APPLICATION_APPLICATION_H_
#define COMPONENTS_APPLICATION_APPLICATION_H_

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "can.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwgps.h"
#include "esp_spi_flash.h"

#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"



/* Exported types ------------------------------------------------------------*/

#define HAL_DEBUG     1
#define APP_DEBUG     1
#define CAN_DEBUG     1
#define GPS_GSM_DEBUG 1
#define BLE_DEBUG     1


#define GSM_MODEM_LDO_ON    gpio_set_level(GSM_UART_ENB_PIN,0);      //  LYNQ == 0 // MC60 1
#define GSM_MODEM_LDO_OFF   gpio_set_level(GSM_UART_ENB_PIN,1);      //  LYNQ == 1 // MC60 0

#define CLOUD_COMM_DATA_INTERVAL             20  // in 20 secs
#define TCP_PORT_CLOSE_OPEN_INTERVAL         600  // 10 min in secs

#define  CAN_MSG1        24
#define  CAN_MSG2        30
#define  CAN_MSG3        36
#define  CAN_MSG4        42
#define  CAN_MSG5        48
#define  CAN_MSG6        50
#define  CAN_MSG7        56
#define  CAN_MSG8        62
#define  CAN_MSG9        68
#define  CAN_MSG10       74
#define  CAN_MSG11       80
#define  CAN_MSG12       86
#define  CAN_MSG13       92
#define  CAN_MSG14       98
#define  CAN_MSG15       104
#define  CAN_MSG16       110
#define  CAN_MSG17       116

#define GSM_SERIAL_BUFF_MAX    1500

#define EX_UART_NUM  UART_NUM_1

#define RTC_TIME_SYNCHRONIZATION        28800     // in seconds (8hours)

//#define BMS_TYPE_1    0x01
//#define BMS_TYPE_2    0x02
//#define BMS_TYPE_3    0X03
//#define BMS_TYPE_4    0x04
//#define BMS_TYPE_5    0x05
//#define BMS_TYPE_6    0x06
//#define BMS_TYPE_7    0x07
#define BMS_TYPE_8      0x08
//#define BMS_TYPE_X    0x22
//#define BMS_TYPE_X1   0x08



#define LED_NORMAL_WORKING         0
#define LED_SERVER_NOT_CONNECTING  1
#define LED_CAN_NOT_WORKING        2


#define WRITE_BIT            I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT             I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN               0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS              0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL                    0x0                             /*!< I2C ack value */
#define NACK_VAL                   0x1                            /*!< I2C nack value */

#define APN_INFO 				"APN"
#define SERVER_INFO				"TCPSERVER"
#define FTP_INFO				"FTPSERVER"
#define OVER_SPEED_INFO			"OVERSPEED"
#define COMMN_INTERVAL_INFO		"COMMN_INFO"
#define BMS_TYPE_KEY_NVS        "BMS_TYPE_KEY"


#define BLE_NOTIFY_ERROR_BYTE      0
#define BLE_NOTIFY_BAT_BYTE        1
#define BLE_NOTIFY_CURRENT_BYTE    2
#define BLE_NOTIFY_TEMP_BYTE       3
#define BLE_NOTIFY_CELL1_BYTE      4
#define BLE_NOTIFY_CELL2_BYTE      5
#define BLE_NOTIFY_SOC_BYTE        6
#define BLE_NOTIFY_BMS_BYTE        7


#define BLE_NOTIFY_DATA_LEN_1_BYTE   1
#define BLE_NOTIFY_DATA_LEN_2_BYTE   2

#define QUECTEL_MODEM           1
#define LYNQ_MODEM              0

#define APN           	0
#define TCP_SERVER    	1
#define FTP_SERVER    	2
#define OVER_SPEED      3
#define COMMN_INFO      4

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported structures -------------------------------------------------------*/

QueueHandle_t gsm_uart_1_queue;


/* Exported functions --------------------------------------------------------*/

void app_init(void);
void time_conversion_process(void);
void gps_data_process(uint8_t *gps_data);
void cloud_comm_task(void *args);
void gsm_engine_task(void *args);
void uart_1_isr_task(void *args);
void app_set_rtc(void);
uint8_t esp32s2_rtc_set_timedate(uint8_t hr, uint8_t min, uint8_t sec,uint8_t day,uint8_t month,uint8_t year);
void imei_num_conv(uint8_t *imei_data);
void esp_fw_update_init(void);
void esp_firmware_update(uint8_t *data,uint16_t data_size);
void update_status_process(void);
void set_led_sequence(uint8_t led_seq);
void led_sequence_process(void);
void IRAM_ATTR gpio_isr_handler(void* arg);
esp_err_t i2c_master_write_slave(uint8_t slave_addr,uint8_t write_addr,uint8_t data[],size_t len);
esp_err_t i2c_master_read_slave(uint8_t slave_addr,uint8_t read_addr,uint8_t data[],size_t len);
uint8_t nvs_write_bms_type_parameters(void);
uint8_t nvs_read_bms_type_parameters(void);
uint8_t nvs_write_config_parameters(void);
uint8_t nvs_read_config_parameters(void);
void esp_firmware_update_frm_quectel(uint8_t *data,uint16_t data_size);
void updation_task(void *args);
void get_bat_vol(uint8_t *bat_vol);



/*  Functions used to ___________________________________________________ *****/

/* Initialization and Configuration functions *********************************/





#endif /* COMPONENTS_APPLICATION_APPLICATION_H_ */
