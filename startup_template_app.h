/**
 * \file
 * \brief Startup Template declarations
 * Copyright (c) 2014-2016 Atmel Corporation. All rights reserved.
 * \asf_license_start
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel
 * Support</a>
 */

#ifndef __STARTUP_TEMPLATE_H__
#define __STARTUP_TEMPLATE_H__

/**
 * @brief Blood Pressure service UUID
 */
//#define BLOOD_PRESSURE_SERVICE_UUID                                                 (0x1810)

/**
 * @brief Blood Pressure measurement characteristic UUID
 */
#define BLOOD_PRESSURE_MEASUREMENT_CHAR_UUID						(0x2A35)

/**
 * @brief Intermediate Cuff Pressure characteristic UUID
 */
#define INTERMEDIATE_CUFF_PRESSURE_CHAR_UUID						(0x2A36)

/**
 * @brief Blood Pressure Feature characteristic UUID
 */
#define BLOOD_PRESSURE_FEATURE_CHAR_UUID						(0x2A49)

/**
 * @brief Total no of characteristics in Blood Pressure service
 */
#define BLP_TOTAL_CHARATERISTIC_NUM							(3)


/**
 * @brief Notification Mask
 */
#define BLP_NOTIFICATION								(1)

/**
 * @brief Disabling the Indication
 */
#define BLP_INDICATION									(2)

/** 
 * @brief notification enable
 *
 */
#define BLP_NOTIFICATION_ENABLE								(1)

#define BLP_NOTIFICATION_DISABLE							(2)

#define BLP_INDICATION_ENABLE								(3)

#define BLP_INDICATION_DISABLE								(4)

/**
 * @brief maximum size of flags field of bp mm characteristic in bytes
 */
#define BLP_MM_FLAGS_SIZE                                                            (1)

/**
 * @brief maximum size of systolic  field of blp mm characteristic in bytes
 */
#define BLP_SYSTOLIC_VAL_SIZE                                                        (2)

/**
 * @brief maximum size of diastolic  field of blp mm characteristic in bytes
 */
#define BLP_DIASTOLIC_VAL_SIZE                                                       (2)

/**
 * @brief maximum size of map  field of blp mm characteristic in bytes
 */
#define BLP_MAP_VAL_SIZE		                                             (2)

/**
 * @brief maximum size of time stamp field of  blp mm characteristic in bytes
 */
#define BLP_MAX_TIME_STAMP_SIZE                                                      (7)

/**
 * @brief maximum size of pulse rate in characteristic in bytes
 */
#define BLP_MAX_PULSE_RATE_SIZE                                                      (2)

/**
 * @brief maximum size of user id in characteristic in bytes
 */
#define BLP_MAX_USER_ID_SIZE	                                                     (1)

/**
 * @brief maximum size of measurement status in characteristic in bytes
 */
#define BLP_MAX_MM_STATUS_SIZE	                                                     (2)


#define DEFAULT_VALUE																 (1)
/************************************************************************/
/*							Types										*/
/************************************************************************/

/**
 * @brief hr_gatt_service_handler is the service handler function
 */
typedef struct hr_gatt_service_handler {
	/** service uuid */
	at_ble_uuid_t serv_uuid;
	/** service handle */
	at_ble_handle_t serv_handle;
	/** characteristic handle */
	at_ble_characteristic_t serv_chars[BLP_TOTAL_CHARATERISTIC_NUM];
} blp_gatt_service_handler_t;

/************************************************************************/
/*				Functions		 		*/
/************************************************************************/

/**@brief Blood Pressure service and characteristic initialization(Called only once
 * by user).
 *
 * @param[in] Blood Pressure service instance
 *
 * @return none
 */
void blp_init_service(blp_gatt_service_handler_t *blood_pressure_serv);

/**@brief Register a blp service instance inside stack.
 *
 * @param[in] dis_primary_service blp service instance
 *
 * @return @ref AT_BLE_SUCCESS operation completed successfully
 * @return @ref AT_BLE_FAILURE Generic error.
 */
at_ble_status_t blp_primary_service_define(
		blp_gatt_service_handler_t *blp_primary_service);

/**@brief handler for characteristic write called by profile
 *
 * @param[in] hr_gatt_service_handler_t servcie handler
 *
 * @return number representing the changed characteristic
 */
uint8_t blp_char_change_handler(blp_gatt_service_handler_t *blp_primary_service,
		at_ble_characteristic_changed_t *params);

/** @brief APP_FAST_ADV between 0x0020 and 0x4000 in 0.625 ms units (20ms to 10.24s). */
#define APP_FAST_ADV						(1600)

/** @brief APP_ADV_TIMEOUT Advertising time-out between 0x0001 and 0x028F in seconds, 0x0000 disables time-out.*/
#define APP_ADV_TIMEOUT						(655)


/**
 * @brief Timer Interval which is 1 second
 */
#define TIMER_INTERVAL								(1)

/**
 * @brief Indication timer the time taken to send an indication
 */
#define INDICATION_TIMER_VAL							(10)

/**
 * @brief APP_DEFAULT_VAL default value for data in application
 */
#define APP_DEFAULT_VAL								(1)

/**
 * @brief BLP_DATA_LEN the blp measurment characteristic data length
 */
#define BLP_DATA_LEN								(19)

/**
 * @brief Mask for flags field in the blp measurement value
 */
#define BLOOD_PRESSURE_UNITS_FLAG_MASK					(0x1 << 0)

#define BLOOD_PRESSURE_TIME_FLAG_MASK					(0x1 << 1)

#define BLOOD_PRESSURE_PULSE_FLAG_MASK					(0x1 << 2)

#define BLOOD_PRESSURE_USERID_FLAG_MASK					(0x1 << 3)

#define BLOOD_PRESSURE_MMT_STATUS_FLAG_MASK				(0x1 << 4)

/**
 * @brief min and max fields for blp mm in both mmhg and kpa
 */

#define SYSTOLIC_MIN_MMHG									(525)
#define SYSTOLIC_MAX_MMHG									(825)

#define DIASTOLIC_MIN_MMHG									(525)
#define DIASTOLIC_MAX_MMHG									(825)

#define MAP_MIN_MMHG										(525)
#define MAP_MAX_MMHG										(825)

#define SYSTOLIC_MIN_KPA									(70)
#define SYSTOLIC_MAX_KPA									(110)

#define DIASTOLIC_MIN_KPA									(70)
#define DIASTOLIC_MAX_KPA									(110)

#define MAP_MIN_KPA										(70)
#define MAP_MAX_KPA										(110)

#define PULSE_RATE_MIN										(60)
#define PULSE_RATE_MAX										(120)

#define USERID_1										(1)
#define USERID_2										(2)

/**
 * @brief Default Time stamp Values
 */
							

/**
 * @brief Max Time stamp Values for time stamp calculation
 */
#define SECOND_MAX											(59)
#define MINUTE_MAX											(59)
#define HOUR_MAX											(23)
#define DAY_MAX												(31)
#define MONTH_MAX											(12)
#define YEAR_MAX											(9999)

/**
 * @brief Blood pressure parameters
 */
#define SYSTOLIC_MMHG										(0)
#define DIASTOLIC_MMHG										(1)
#define MAP_MMHG										(2)
#define SYSTOLIC_KPA										(3)
#define DIASTOLIC_KPA										(4)
#define MAP_KPA											(5)
#define PULSE_RATE										(6)
#define INTERIM_SYS_MMHG									(7)
#define INTERIM_SYS_KPA										(8)

/**
 * @brief app_connected_state ble manger notifies the application about state
 * @param[in] connected parameters
 */
static at_ble_status_t app_connected_state_handler(void *params);

/**
 * @brief app_connected_state ble manger notifies the application about state
 * @param[in] disconnection parameters
 */
static at_ble_status_t app_disconnected_state_handler(void *param);

/** @brief app_notification_confirmation_handler called by ble manager 
 *	to give the status of notification sent
 *  @param[in] at_ble_cmd_complete_event_t address of the cmd completion
 */	
static at_ble_status_t app_notification_confirmation_handler(void *params);

/** @brief app_indication_confirmation_handler called by ble manager 
 *	to give the status of notification sent
 *  @param[in] at_ble_cmd_complete_event_t address of the cmd completion
 */	
static at_ble_status_t app_indication_confirmation_handler(void *params);


/** @brief APP_BLP_SENSOR_FAST_ADV	 between 0x0020 and 0x4000 in 0.625 ms units (20ms to 10.24s). */
//	<o> Fast Advertisement Interval <100-1000:50>
//	<i> Defines inteval of Fast advertisement in ms.
//	<i> Default: 100
//	<id> blp_sensor_fast_adv
#define BLP_SENSOR_FAST_ADV								(3200) //2s

/** @brief BLP_SENSOR_ADV_TIMEOUT Advertising time-out between 0x0001 and 0x028F in 
 *seconds, 0x0000 disables time-out.*/
//	<o> Advertisement Timeout <1-655>
//	<i> Defines interval at which advertisement timeout in sec.
//	<i> Default: 655
//	<id> blp_sensor_adv_timeout
#define BLP_SENSOR_ADV_TIMEOUT								(655) // 10 min

/** @brief scan_resp_len is the length of the scan response data */
//	<o> Scan Response Buffer <1-20>
//	<i> Defines size of buffer for scan response.
//	<i> Default: 10
//	<id> blp_scan_resp_len
#define SCAN_RESP_LEN									(10)
	
/** @brief ADV_DATA_LEN */
#define ADV_DATA_LEN									(18)

/** @brief ADV_TYPE_LEN */
#define ADV_TYPE_LEN									(0x01)

#define UUID_16_BIT_LEN									(0x02)


/** @brief HR_SENSOR_ADV_DATA_UUID_TYPE is complete 16 bit uuid type*/
#define BLP_SENSOR_ADV_DATA_COMP_16_UUID_TYPE				(0x03)

/** @brief HR_SENSOR_ADV_DATA_UUID_LEN the total length for hr uuid and dis uuid */
#define BLP_SENSOR_ADV_DATA_UUID_LEN					(4)

/** @brief DEVICE_INFORMATION_SERVICE_UUID **/
#define DEVICE_INFORMATION_SERVICE_UUID					(0x180A)

/** @brief BLP_SENSOR_ADV_DATA_NAME_TYPE the gap ad data type */
#define BLP_SENSOR_ADV_DATA_NAME_TYPE					(0x09)

/** @brief BLP_SENSOR_ADV_DATA_NAME_LEN the  length of the device name */
#define BLP_SENSOR_ADV_DATA_NAME_LEN					(9)

/* @brief BLP_ADV_DATA_NAME_DATA the actual name of device */
//	<s.9>	Advertising String
//	<i>	String Descriptor describing in advertising packet.
//	<id> blp_sensor_adv_data_name_data
#define BLP_SENSOR_ADV_DATA_NAME_DATA					("ATMEL-BLP")

/* @brief call back handler type  */
typedef void (*blp_notification_callback_t)(bool);

/* @brief call back handler type  */
typedef void (*blp_indication_callback_t)(bool);


/****************************************************************************************
*							        Function Prototypes	                                *                                                        *
****************************************************************************************/

/** @brief blp_sensor_init initializes and defines the services of the blp profile
 *
 *  @param[in] params are unused.
 *
 */
void blp_sensor_init(void *param);

/** @brief blp_sensor_service_init initializes the services of the profile
 *
 */
void blp_sensor_service_init(void);

/** @brief blp_sensor_service_define defines the services of the profile
 *
 */
void blp_sensor_service_define(void);

/** @brief blp_sensor_adv adds the advertisement data of the profile and starts
 * advertisement
 *
 */
void blp_sensor_adv(void);

/** @brief blp_sensor_send_notification disconnects with the peer device called by the
 * application
 *
 */

void blp_sensor_send_notification(uint8_t *hr_data, uint8_t length);

/** @brief blp_sensor_send_indication adds the new characteristic value and
 * sends the indication
 *  @param[in] blp the new blp characteristic value needs to be updated
 *  @param[in] length length of new characteristic value
 */
void blp_sensor_send_indication(uint8_t *blp_data, uint8_t length);

/** @brief register_blp_indication_handler registers the indication handler
 * passed by the application
 *  @param[in] blp_indication_callback_t address of the indication handler
 *function to be called
 */
void register_blp_indication_handler(
		blp_indication_callback_t blp_indication_handler);
		
/** @brief register_blp_notification_handler registers the notification handler
 * passed by the application
 *  @param[in] blp_notification_callback_t address of the notification handler
 *function to be called
 */
void register_blp_notification_handler(
		blp_notification_callback_t blp_notificaton_handler);

/** @brief blp_sensor_char_changed_handler called by the ble manager after a
 *  change in the characteristic
 *  @param[in] at_ble_characteristic_changed_t which contains handle of
 *  characteristic and new value
 *  @return AT_BLE_SUCCESS
 */
at_ble_status_t blp_sensor_char_changed_handler(
					void *char_handle);

/** @brief blp_sensor_disconnect_event_handler called by ble manager after
 * disconnection event received
 *	@param[in] at_ble_disconnected_t	which has connection handle and
 *reason for disconnection
 */
at_ble_status_t blp_sensor_disconnect_event_handler(
					void *disconnect);

/** @brief blp_sensor_connected_state_handler called by ble manager after a
 * change in characteristic
 *  @param[in] at_ble_connected_t which has connection handle and the peer
 *device address
 */
at_ble_status_t blp_sensor_connected_state_handler(
		void *conn_params);

/** @brief blp_notification_confirmation_handler called by ble manager 
 *	to give the status of notification sent
 *  @param[in] at_ble_cmd_complete_event_t address of the cmd completion
 *  @return AT_BLE_SUCCESS on success
 */	
at_ble_status_t blp_notification_confirmation_handler(void *params);

/** @brief blp_indication_confirmation_handler called by ble manager 
 *	to give the status of notification sent
 *  @param[in] at_ble_cmd_complete_event_t address of the cmd completion
 *  @return AT_BLE_SUCCESS on success
 */	
at_ble_status_t blp_indication_confirmation_handler(void *params);

/** @brief blp_disconnection called by the application to disconnect
 *
 */
void blp_disconnection(void);



#include "ble_manager.h"

/****************************************************************************************
*							        Macros	                                     							*
****************************************************************************************/

/** @brief User should set the default manufacturer name before calling dis_init_service API */
//	<s.20>Manufacture Name
//	<i> String Descriptor describing Manufacture Name.
//	<id> default_manufacturer_name
#define DEFAULT_MANUFACTURER_NAME				("ATMEL")

/** @brief User should set the length based on DEFAULT_MANUFACTURER_NAME */
#define DIS_CHAR_MANUFACTURER_NAME_INIT_LEN			(0x05)

/** @brief User can set the maximum length for manufacture name */
#define DIS_CHAR_MANUFACTURER_NAME_MAX_LEN			(0x14)

/** @brief User should set the default model number before calling dis_init_service API */
//	<s.20>Model Number
//	<i>String Descriptor describing Model Number.
//	<id> default_model_number
#define DEFAULT_MODEL_NUMBER					("BLE DEVICE")

/** @brief User should set the length based on DEFAULT_MODEL_NUMBER*/
#define DIS_CHAR_MODEL_NUMBER_INIT_LEN				(0x0a)

/** @brief User can set the maximum length for model number */
#define DIS_CHAR_MODEL_NUMBER_MAX_LEN				(0x14)

/** @brief User should set the default serial number before calling dis_init_service API */
//  <s.20>Serial Number
//  <i>String Descriptor describing Serial Number.
//	<id> default_serial_number
#define DEFAULT_SERIAL_NUMBER				("BTLC1000/SAMB11")

/** @brief User should set the length based on  DEFAULT_SERIAL_NUMBER*/
#define DIS_CHAR_SERIAL_NUMBER_INIT_LEN			0x0f

/** @brief User can set the maximum length for serial number */
#define DIS_CHAR_SERIAL_NUMBER_MAX_LEN			0x14

/** @brief User should set the default hardware revision before calling dis_init_service API */
//  <s.20>Hardware Revision
//  <i>String Descriptor describing Hardware Revision.
//	<id> default_hardware_revision
#define DEFAULT_HARDWARE_REVISION			("Rev A")

/** @brief User should set the length based on  DEFAULT_HARDWARE_REVISION*/
#define DIS_CHAR_HARDWARE_REVISION_INIT_LEN		0x05

/** @brief User can set the maximum length for hardware revision */
#define DIS_CHAR_HARDWARE_REVISION_MAX_LEN		0x14

/** @brief User should set the default software revision before calling dis_init_service API */
//  <s.20>Software Revision
//  <i>String Descriptor describing Software Revision.
//	<id> default_software_revision
#define DEFAULT_SOFTWARE_REVISION			("SW_BETA")

/** @brief User should set the length based on  DEFAULT_SOFTWARE_REVISION*/
#define DIS_CHAR_SOFTWARE_REVISION_INIT_LEN		0x07

/** @brief User can set the maximum length for software revision */
#define DIS_CHAR_SOFTWARE_REVISION_MAX_LEN		0x14

/** @brief User should set the default firmware revision before calling dis_init_service API */
//  <s.20>Firmware Revision
//  <i>String Descriptor describing Firmware Revision.
//	<id> default_firmware_reivsion
#define DEFAULT_FIRMWARE_REIVSION			("FW_BETA")

/** @brief User should set the length based on  DEFAULT_FIRMWARE_REIVSION*/
#define DIS_CHAR_FIRMWARE_REIVSION_INIT_LEN		0x07

/** @brief User can set the maximum length for firmware revision */
#define DIS_CHAR_FIRMWARE_REIVSION_MAX_LEN		0x14

/** @brief number of device information service characteristics */
#define DIS_TOTAL_CHARATERISTIC_NUM          		0x09

/** @brief system id characteristic initial length*/
#define DIS_CHAR_SYSTEM_ID_INIT_LEN sizeof(system_id_char_value_t)

/** @brief system id characteristic maximum length*/
#define DIS_CHAR_SYSTEM_ID_MAX_LEN	sizeof(system_id_char_value_t)

/** @brief PnP id characteristic initial length */
#define DIS_CHAR_PNP_ID_INIT_LEN			0x07

/** @brief PnP id characteristic maximum length */
#define DIS_CHAR_PNP_ID_MAX_LEN				0x07

/** @brief IEEE regulatory certification data list characteristic minimum length */
#define DIS_CHAR_IEEE_REG_CERT_DATA_LIST_INIT_LEN	0x01

/** @brief IEEE regulatory certification data list characteristic maximum length */
#define DIS_CHAR_IEEE_REG_CERT_DATA_LIST_MAX_LEN	0x0a

/** @brief PnP ID characteristic value configure by user*/
#define PNP_ID_VENDOR_ID_SOURCE		0x01
#define PNP_ID_VENDOR_ID		0x2222
#define PNP_ID_PRODUCT_ID		0x3333
#define PNP_ID_PRODUCT_VERSION		0x0001

/** @brief system ID characteristic default values */
#define SYSTEM_ID_MANUFACTURER_ID_LEN	0x05
#define SYSTEM_ID_ORG_UNIQUE_ID_LEN	0x03	
#define SYSTEM_ID_MANUFACTURER_ID	"\x00\x00\x00\x00\x00"
#define SYSTEM_ID_ORG_UNIQUE_ID		"\x00\x04\x25"

/** @brief Macro used for updating manufacturing string after defining the service using dis_primary_service_define*/
#define UPDATE_MANUFACTURER_STRING(ptr,info_data, conn_handle) do {  \
	if ( (dis_info_update(ptr,DIS_MANUFACTURER_NAME,info_data, conn_handle)) != AT_BLE_SUCCESS) { \
		DBG_LOG("Updating Manufacturer string failed");  \
	}\
} while (0)

/** @brief Macro used for updating model number after defining the service using dis_primary_service_define*/
#define UPDATE_MODEL_NUMBER(ptr,info_data, conn_handle) do {   \
	if (dis_info_update(ptr,DIS_MODEL_NUMBER,info_data, conn_handle) != AT_BLE_SUCCESS) { \
		DBG_LOG("Updating model number failed");  \
	}\
} while (0)

/** @brief Macro used for updating serial number after defining the service using dis_primary_service_define*/
#define UPDATE_SERIAL_NUMBER(ptr,info_data, conn_handle) do{   \
	if (dis_info_update(ptr,DIS_SERIAL_NUMBER,info_data, conn_handle) != AT_BLE_SUCCESS) { \
		DBG_LOG("Updating serial number failed");  \
	}\
} while (0)

/** @brief Macro used for updating hardware revision after defining the service using dis_primary_service_define*/
#define UPDATE_HARDWARE_REVISION(ptr,info_data, conn_handle) do{   \
	if (dis_info_update(ptr,DIS_HARDWARE_REVISION,info_data, conn_handle) != AT_BLE_SUCCESS) { \
		DBG_LOG("Updating hardware revision failed");  \
	}\
} while (0)

/** @brief Macro used for updating firmware revision after defining the service using dis_primary_service_define*/
#define UPDATE_FIRMWARE_REVISION(ptr,info_data, conn_handle) do{   \
	if (dis_info_update(ptr,DIS_FIRMWARE_REVISION,info_data, conn_handle) != AT_BLE_SUCCESS) { \
		DBG_LOG("Updating firmware revision failed");  \
	}\
} while (0)

/** @brief Macro used for updating software revision after defining the service using dis_primary_service_define*/
#define UPDATE_SOFTWARE_REVISION(ptr,info_data, conn_handle) do{   \
	if (dis_info_update(ptr,DIS_SOFTWARE_REVISION,info_data, conn_handle) != AT_BLE_SUCCESS) { \
		DBG_LOG("Updating software revision failed");  \
	}\
} while (0)

/** @brief Macro used for updating system ID after defining the service using dis_primary_service_define*/
#define UPDATE_SYSTEM_ID(ptr,info_data, conn_handle) do {   \
	if (dis_info_update(ptr,DIS_SYSTEM_ID,info_data, conn_handle) != AT_BLE_SUCCESS) { \
		DBG_LOG("Updating system id failed");  \
	}\
} while (0)

/** @brief Macro used for updating PnP ID after defining the service using dis_primary_service_define*/
#define UPDATE_PNP_ID(ptr,info_data, conn_handle) do {   \
	if (dis_info_update(ptr,DIS_PNP_ID,info_data, conn_handle) != AT_BLE_SUCCESS) { \
		DBG_LOG("Updating PnP ID failed");  \
	}\
} while (0)

/** @brief Macro used for updating IEEE regulatory certification data list after defining the service using dis_primary_service_define*/
#define UPDATE_IEEE_REG_CERT_DATA_LIST(ptr,info_data, conn_handle) do {   \
	if (dis_info_update(ptr,DIS_IEEE_REG_CERT_DATA_LIST,info_data, conn_handle) != AT_BLE_SUCCESS) { \
		DBG_LOG("Updating IEEE regulatory certification data list failed");  \
	}\
} while (0)


/****************************************************************************************
*							        Enumerations	                                       							*
****************************************************************************************/

/**@brief The type of the information*/
typedef enum {
	/* manufacturer name characteristic */
	DIS_MANUFACTURER_NAME= 0,
	/* model number characteristic */
	DIS_MODEL_NUMBER,
	/* serial number characteristic */
	DIS_SERIAL_NUMBER,
	/* Hardware revision characteristic */
	DIS_HARDWARE_REVISION,
	/* Firmware revision characteristic */
	DIS_FIRMWARE_REVISION,
	/* Software revision characteristic */
	DIS_SOFTWARE_REVISION,
	/* System id characteristic */
	DIS_SYSTEM_ID,
	/* PnP ID characteristic */
	DIS_PNP_ID,
	/* IEEE regulatory certification data list characteristic */
	DIS_IEEE_REG_CERT_DATA_LIST,
	/* must be the last element */
	DIS_END_VALUE,
} dis_info_type;


/****************************************************************************************
*							        Structures                                     							*
****************************************************************************************/


typedef struct dis_gatt_service_handler
{
		at_ble_uuid_t	serv_uuid;
		at_ble_handle_t	serv_handle;
		at_ble_characteristic_t	serv_chars[DIS_TOTAL_CHARATERISTIC_NUM];	
}dis_gatt_service_handler_t;


/** @brief system ID characteristic value information */
typedef struct{
	/// manufacturer identifier
	uint8_t manufacturer_id[5];
	/// organizational unique identifier 
	uint8_t org_unique_id[3];
}system_id_char_value_t;



/** @brief pnp characteristic value information */
typedef struct {
	/// vendor id source
	uint8_t vendor_id_source;
	/// vendor id
	uint16_t vendor_id;
	/// product id
	uint16_t product_id;
	/// product version
	uint16_t product_version;
}pnp_id_char_value_t;



/** @brief characteristic value information */
typedef struct {
	/// manufacturer name 
	uint8_t manufacturer_name[DIS_CHAR_MANUFACTURER_NAME_MAX_LEN];
	/// model number
	uint8_t default_model_number[DIS_CHAR_MODEL_NUMBER_MAX_LEN];
	/// serial number
	uint8_t default_serial_number[DIS_CHAR_SERIAL_NUMBER_MAX_LEN];
	/// hardware revision
	uint8_t default_hardware_revision[DIS_CHAR_HARDWARE_REVISION_MAX_LEN];
	/// default firmware revision
	uint8_t default_firmware_revision[DIS_CHAR_FIRMWARE_REIVSION_MAX_LEN];
	/// software revision
	uint8_t default_software_revision[DIS_CHAR_SOFTWARE_REVISION_MAX_LEN];
	/// system id
	system_id_char_value_t default_system_id;
	/// PnP ID
	pnp_id_char_value_t default_pnp_id;
	/// ieee regulatory certification data list
	uint8_t ieee_reg_cert_data_list[DIS_CHAR_IEEE_REG_CERT_DATA_LIST_MAX_LEN];
}device_info_char_value_t;

/** @brief Configurable Client characteristic data for a given dis info type*/
typedef struct{
	// length of the data to be updated
	uint16_t data_len;
	// data to be updated
	uint8_t *info_data;
}dis_info_data;

/****************************************************************************************
*                                       Functions                                       *
****************************************************************************************/

/**@brief Update the DIS characteristic value after defining the services using dis_primary_service_define
 *
 * @param[in] dis_serv  dis service instance
 * @param[in] info_type dis characteristic type to be updated
 * @param[in] info_data data need to be updated
 * @return @ref AT_BLE_SUCCESS operation completed successfully .
 * @return @ref AT_BLE_FAILURE Generic error.
 */
at_ble_status_t dis_info_update(dis_gatt_service_handler_t *dis_serv , dis_info_type info_type, dis_info_data* info_data, at_ble_handle_t conn_handle);


/**@brief DIS service and characteristic initialization(Called only once by user).
 *
 * @param[in] device_info_serv dis service instance
 *
 * @return none
 */
void dis_init_service(dis_gatt_service_handler_t *device_info_serv );

/**@brief Register a dis service instance inside stack. 
 *
 * @param[in] dis_primary_service dis service instance
 *
 * @return @ref AT_BLE_SUCCESS operation completed successfully
 * @return @ref AT_BLE_FAILURE Generic error.
 */
at_ble_status_t dis_primary_service_define(dis_gatt_service_handler_t *dis_primary_service);
#endif /*__BLP_SENSOR_H__ */

//#endif /* __STARTUP_TEMPLATE_H__ */
