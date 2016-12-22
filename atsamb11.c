
/**
 * \file
 * \brief BLE Startup Template
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
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

/*- Includes ---------------------------------------------------------------*/

#include <asf.h>
#include <string.h>
#include <dma_sam_b.h>
#include "platform.h"
#include "at_ble_api.h"
#include "console_serial.h"
#include "timer_hw.h"
#include "ble_manager.h"
#include "ble_utils.h"
#include "button.h"
#include "startup_template_app.h"

void configure_gpio_pins(void);


volatile at_ble_status_t status;
at_ble_handle_t htpt_conn_handle;
volatile bool Timer_Flag = false;
volatile bool Temp_Notification_Flag = false;
//! [module_inst]
struct uart_module uart_instance;
//! [module_inst]

at_ble_connection_params_t change_conn_params;

//settings to save power
#define CONNECTION_INTERVAL_MIN 80	//time is 80*1.25 = 100ms
#define CONNECTION_INTERVAL_MAX 160	//time is 160*1.25 = 200ms
#define SLAVE_LATENCY 3	//slave needs to connect once in atleast 200+3(200) = 800ms
#define TRANSMIT_POWER AT_BLE_TX_PWR_LVL_POS_03_DB	//(-6dBm)
#define ADVERT_INTERVAL 3200	//(3200*0.625 = 2 seconds)

volatile int a = 0;
//! [dma_resource]
struct dma_resource uart_dma_resource_tx;
struct dma_resource uart_dma_resource_rx;
//! [dma_resource]

//! [usart_buffer]
#define BUFFER_LEN    6
static uint8_t string[BUFFER_LEN];
float integer_part;
float fractional_part;
float integer_part_press1;
float integer_part_press2;
float fractional_part_press;
float integer_press;
float actual_pressure;

uint8_t Temperature = 0;
float temp;
bool led_status;
//! [usart_buffer]

static at_ble_status_t ble_paired_cb(void *param);

//! [transfer_descriptor]
struct dma_descriptor example_descriptor_tx;
struct dma_descriptor example_descriptor_rx;
//! [transfer_descriptor]


/*blp_sensor_App.c*/
#define APP_STACK_SIZE  (1024)
volatile unsigned char app_stack_patch[APP_STACK_SIZE];
bool isTimer = false;
volatile at_ble_status_t status;
at_ble_handle_t htpt_conn_handle;
at_ble_tx_power_level_t power;

/** flag to check if indication has been sent successfully over the air*/
/*volatile*/ bool indication_sent = true;

/** flag to check if notification has been sent successfully over the air*/
/*volatile*/ bool notification_sent = true;

/** Flag to change the events from mmgh to kpa and vice versa*/
/*volatile*/ bool units = APP_DEFAULT_VAL;

/** flag to send notifications */
/*volatile*/ bool notification_flag = APP_DEFAULT_VAL;

/** flag to send indication */
/*volatile*/ bool indication_flag = APP_DEFAULT_VAL;

/** Flag to identify user request for indication and notification*/
/*volatile*/ bool user_request_flag =  APP_DEFAULT_VAL;

/** Counter to maintain interval of indication*/
/*volatile*/ uint8_t timer_count = APP_DEFAULT_VAL;

/** flag to send one notification for one second*/
/*volatile*/ bool notify = 0;

/** flag to check the app state*/
/*volatile*/ bool app_state;

/** flags for reversing the direction of characteristic*
 *       change for indication*/
/*volatile*/ int8_t operator_blp[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};

/** Current systolic value  in mmhg*/
uint16_t systolic_val_mmhg = SYSTOLIC_MIN_MMHG;

/** Current diastolic value in mmhg*/
uint16_t diastolic_val_mmhg = DIASTOLIC_MIN_MMHG;

/** Current map value in mmhg*/
uint16_t map_val_mmhg = MAP_MIN_MMHG;

/** Current systolic in kpa*/
uint16_t systolic_val_kpa = SYSTOLIC_MIN_KPA;

/** current diastolic value in kpa*/
uint16_t diastolic_val_kpa = DIASTOLIC_MIN_KPA;

/** current map value in kpa*/
uint16_t map_val_kpa = MAP_MIN_KPA;

/** Current pulse rate value in kpa*/
uint16_t pulse_rate_val = PULSE_RATE_MIN;

/** Current time stamp */
at_ble_prf_date_time_t time_stamp;

/* Intermediate Cuff Pressure Values for notification */
uint16_t interim_diastolic_mmhg = DIASTOLIC_MIN_MMHG;

uint16_t interim_diastolic_kpa = DIASTOLIC_MIN_KPA; 

uint16_t interim_systolic_mmhg = SYSTOLIC_MIN_MMHG;

uint16_t interim_systolic_kpa = SYSTOLIC_MIN_KPA;

uint16_t interim_map_mmhg = MAP_MIN_MMHG;

uint16_t interim_map_kpa = MAP_MIN_KPA;

static /*const*/ ble_event_callback_t app_gap_handle[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	app_connected_state_handler,
	app_disconnected_state_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

static /*const*/ ble_event_callback_t app_gatt_server_handle[] = {
	app_notification_confirmation_handler,
	app_indication_confirmation_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

/* To keep the app in execution mode */
bool app_exec = true;

bool isButton = false;

bool isIndication = false;
uint8_t g_blp_data[BLP_DATA_LEN];
uint8_t g_idx = 0;

/**
 * @brief app_connected_state profile notifies the application about state
 * @param[in] connected
 */
static at_ble_status_t app_connected_state_handler(void *params)
{
	app_state = true;
	return AT_BLE_SUCCESS;
}

static at_ble_status_t app_disconnected_state_handler(void *param)
{
	app_state = false;
	//Resetting all the simulated values
		
		interim_diastolic_mmhg = DIASTOLIC_MIN_MMHG;
		interim_diastolic_kpa = DIASTOLIC_MIN_KPA;
		interim_systolic_mmhg = SYSTOLIC_MIN_MMHG;
		interim_systolic_kpa = SYSTOLIC_MIN_KPA;
		interim_map_mmhg = MAP_MIN_MMHG;
		interim_map_kpa = MAP_MIN_KPA;
		systolic_val_mmhg = SYSTOLIC_MIN_MMHG; //systolic_val_mmhg = SYSTOLIC_MIN_MMHG;
		diastolic_val_mmhg = DIASTOLIC_MIN_MMHG; //diastolic_val_mmhg = DIASTOLIC_MIN_MMHG;
		map_val_mmhg = MAP_MIN_MMHG;
		systolic_val_kpa = actual_pressure/10.0;
		diastolic_val_kpa = actual_pressure/10.0;
		map_val_kpa = MAP_MIN_KPA;
		pulse_rate_val = PULSE_RATE_MIN;
		units = !units;
		indication_sent = true;
		notification_sent = true;
		notify = false;
		timer_count = APP_DEFAULT_VAL;
		user_request_flag =  APP_DEFAULT_VAL;
		indication_flag = APP_DEFAULT_VAL;
		notification_flag = APP_DEFAULT_VAL;
		
		/* Starting advertisement */
		blp_sensor_adv();
		//ALL_UNUSED(param);
		return AT_BLE_SUCCESS;
}

/** @brief Updating the time stamp
 *
 */
static void update_time_stamp(void)
{
	if (time_stamp.sec < SECOND_MAX)
	{
		time_stamp.sec++;
	}
	else
	{
		time_stamp.sec = 0;	
		if (time_stamp.min < MINUTE_MAX)
		{
			time_stamp.min++;
		}
		else
		{
			time_stamp.min = 0;
			if (time_stamp.hour < HOUR_MAX)
			{
				time_stamp.hour++;
			}
			else
			{
				time_stamp.hour = 0;
				if (time_stamp.day < DAY_MAX)
				{
					time_stamp.day++;
				}
				else
				{
					time_stamp.day = 1;
					if (time_stamp.month < MONTH_MAX)
					{
						time_stamp.month++;
					}
					else
					{
						time_stamp.month = 1;
						if (time_stamp.year < YEAR_MAX)
						{
							time_stamp.year++;
						} 
						else
						{
							time_stamp.year = 2016;
						}
					}
				}
			}
		}			
	}	
}

/** @brief initializes the time stamp with default time stamp
 *
 */
static void time_stamp_init(void)
{
	memset((uint8_t *)&time_stamp, 0, sizeof(at_ble_prf_date_time_t));
	time_stamp.year = 2015;
	time_stamp.day = 1;
	time_stamp.month = 1;
}

static at_ble_status_t app_notification_confirmation_handler(void *params)
{

	if (((at_ble_cmd_complete_event_t *)params)->status == AT_BLE_SUCCESS) {
		DBG_LOG_DEV("App Notification Successfully sent over the air");
		notification_sent = true;
	} else {
		DBG_LOG_DEV("Sending Notification over the air failed");
		notification_sent = false;
	}
	return AT_BLE_SUCCESS;
}



/** @brief app_indication_confirmation_handler called by ble manager 
 *	to give the status of notification sent
 *  @param[in] at_ble_cmd_complete_event_t address of the cmd completion
 */	
static at_ble_status_t app_indication_confirmation_handler(void *params)
{
	if (((at_ble_cmd_complete_event_t * )params)->status == AT_BLE_SUCCESS) {
		DBG_LOG_DEV("App Indication successfully sent over the air");
		indication_sent = true;
		user_request_flag = false;
		DBG_LOG("\r\nPress the button to receive the blood pressure parameters");
	} else {
		DBG_LOG_DEV("Sending indication over the air failed reason %x ",
								((at_ble_cmd_complete_event_t * )params)->status);
		indication_sent = false;
	}
	return AT_BLE_SUCCESS;
}

/** @brief blp_value_update which will change the blood pressure measurement operations
 *  @param[in] data to which the blood pressure parameter value to be appended
 *  @param[in] idx index where the value has to be updated
 *  @param[in] value_type which will determine which blood pressure parameter 
 */
static void blp_value_update(uint8_t *data, uint8_t idx, uint16_t value, uint8_t value_type)
{
	uint8_t min_val = 0, max_val = 0;
	
	switch(value_type) {
	case SYSTOLIC_MMHG:	
		min_val = SYSTOLIC_MIN_MMHG;
		max_val = SYSTOLIC_MAX_MMHG;
	break;
	
	case DIASTOLIC_MMHG:
		min_val = DIASTOLIC_MIN_MMHG;
		max_val = DIASTOLIC_MAX_MMHG;
	break;
	
	case MAP_MMHG:
		min_val = MAP_MIN_MMHG;
		max_val = MAP_MAX_MMHG;
	break;
	
	case PULSE_RATE:
		min_val = PULSE_RATE_MIN;
		max_val = PULSE_RATE_MAX;
	break;
	
	case SYSTOLIC_KPA:
		min_val = SYSTOLIC_MIN_KPA;
		max_val = SYSTOLIC_MAX_KPA;
	break;
	
	case DIASTOLIC_KPA:
		min_val = DIASTOLIC_MIN_KPA;
		max_val = DIASTOLIC_MAX_KPA;
	break;
	
	case MAP_KPA:
		min_val = MAP_MIN_KPA;
		max_val = MAP_MAX_KPA;
	break;
	
	case INTERIM_SYS_MMHG:
		min_val = SYSTOLIC_MIN_MMHG;
		max_val = SYSTOLIC_MAX_MMHG;
	break;
	
	case INTERIM_SYS_KPA:
		min_val = SYSTOLIC_MIN_KPA;
		max_val = SYSTOLIC_MAX_KPA;
	break;
	}
	
	if (value == max_val) {
		operator_blp[value_type] = -1;
	} else if (value == min_val) {
		operator_blp[value_type] = 1;
	}
	memcpy((data + idx),&value,2);
}
/** @brief sends the characteristic data for the profile to send indication
 *
 */
static void blp_char_indication(void)
{
	uint8_t blp_data[BLP_DATA_LEN];
	uint8_t idx = 0;

	memset(blp_data, 0, sizeof(uint8_t) * BLP_DATA_LEN);
	idx = 0;

	DBG_LOG("\n\n");
	
	DBG_LOG("The Blood Pressure Values are:");
	/* initializing flags to zero*/
	blp_data[0] = 0;
	/** Blood pressure measurement flags */
	if (units) {
		/** Units in mmhg*/
			blp_data[idx] |= (0x1)  & ~(BLOOD_PRESSURE_UNITS_FLAG_MASK);
	} else {
		/** Units in kpa*/
		    blp_data[idx] |= (0x1)  | BLOOD_PRESSURE_UNITS_FLAG_MASK ;
	} 
	
	/** Appending the flags for time stamp , pulse rate, user id , mm status */
	blp_data[idx]	|= BLOOD_PRESSURE_TIME_FLAG_MASK;
	blp_data[idx]	|= BLOOD_PRESSURE_PULSE_FLAG_MASK;
	blp_data[idx]	|= BLOOD_PRESSURE_USERID_FLAG_MASK;
	blp_data[idx++] |= BLOOD_PRESSURE_MMT_STATUS_FLAG_MASK;
	
	if (units) {
		systolic_val_mmhg = systolic_val_mmhg + (operator_blp[SYSTOLIC_MMHG]);
		blp_value_update(blp_data, idx, systolic_val_mmhg, SYSTOLIC_MMHG);
		idx += 2;
		DBG_LOG("%-12s", "Systolic");
		DBG_LOG_CONT("   %d mmhg", systolic_val_mmhg);

		diastolic_val_mmhg = diastolic_val_mmhg + (operator_blp[DIASTOLIC_MMHG]);
		blp_value_update(blp_data, idx, diastolic_val_mmhg, DIASTOLIC_MMHG);
		idx += 2;
		DBG_LOG("%-12s", "Diastolic");
		DBG_LOG_CONT("   %d mmhg", diastolic_val_mmhg);

		map_val_mmhg = map_val_mmhg + (operator_blp[MAP_MMHG]);
		blp_value_update(blp_data, idx, map_val_mmhg, MAP_MMHG);
		idx += 2;
		DBG_LOG("%-12s", "Map");
		DBG_LOG_CONT("   %d mmhg", map_val_mmhg);
	} else {
		systolic_val_kpa = systolic_val_kpa + (operator_blp[SYSTOLIC_KPA]);
		blp_value_update(blp_data, idx, systolic_val_kpa, SYSTOLIC_KPA);
		idx += 2;
		DBG_LOG("%-12s", "Systolic");
		DBG_LOG_CONT("   %02d kpa", systolic_val_kpa);
		diastolic_val_kpa = diastolic_val_kpa + (operator_blp[DIASTOLIC_KPA]);
		blp_value_update(blp_data, idx, diastolic_val_kpa, DIASTOLIC_KPA);
		idx += 2;
		DBG_LOG("%-12s", "Diastolic");
		DBG_LOG_CONT("   %02d kpa", diastolic_val_kpa);
		map_val_kpa = map_val_kpa + (operator_blp[MAP_KPA]);
		blp_value_update(blp_data, idx, map_val_kpa, MAP_KPA);
		idx += 2;
		DBG_LOG("%-12s", "Map");
		DBG_LOG_CONT("   %02d kpa", map_val_kpa);
	}
		memcpy(&blp_data[idx],(uint8_t*)&(time_stamp.year),2);
		idx += 2;
		memcpy(&blp_data[idx],(uint8_t*)&(time_stamp.month),1);
		idx += 1;
		memcpy(&blp_data[idx],(uint8_t*)&(time_stamp.day),1);
		idx += 1;
		memcpy(&blp_data[idx],(uint8_t*)&(time_stamp.hour),1);
		idx += 1;
		memcpy(&blp_data[idx],(uint8_t*)&(time_stamp.min),1);
		idx += 1;
		memcpy(&blp_data[idx],(uint8_t*)&(time_stamp.sec),1);
		idx += 1;
		
	pulse_rate_val = pulse_rate_val + (operator_blp[PULSE_RATE]);
	blp_value_update(blp_data, idx, pulse_rate_val, PULSE_RATE);
	idx += 2;
	DBG_LOG("%-12s", "Pulserate");
	DBG_LOG_CONT("   %d bpm", pulse_rate_val);

	/** Appending User id */
	if (units) {
		blp_data[idx++] = USERID_1;
	} else {
		blp_data[idx++] = USERID_2;
	}
	
	/** Appending Measurement status field */
	blp_data[idx++] = 0xf;
	blp_data[idx++] = 0x0;
	
	blp_sensor_send_indication(blp_data,idx);	
	
}

/** @brief sends the characteristic data for profile to send notification
 *
 */
static void blp_char_notification(void)
{
	uint8_t blp_data[BLP_DATA_LEN];	
	uint8_t idx = 0;

	memset(blp_data, 0, sizeof(uint8_t) * BLP_DATA_LEN);
	idx = 0;

	if (units) {
		/** Units in mmhg*/
		blp_data[idx++] |= (0x1)  & ~(BLOOD_PRESSURE_UNITS_FLAG_MASK);
		blp_data[0] = blp_data[0] & 1;
		DBG_LOG("Cuff pressure  %d mmhg", interim_systolic_mmhg);
		blp_value_update(blp_data, idx, interim_systolic_mmhg, INTERIM_SYS_MMHG);
		idx += 2;
		interim_systolic_mmhg = interim_systolic_mmhg + (operator_blp[7]);
	} else {
		/** Units in kpa*/
		blp_data[idx++] |=  (0x1)  | BLOOD_PRESSURE_UNITS_FLAG_MASK;
		blp_data[0] = blp_data[0] & 1;
		DBG_LOG("Cuff pressure  %02d kpa", interim_systolic_kpa);
		blp_value_update(blp_data, idx, interim_systolic_kpa, INTERIM_SYS_KPA);
		idx += 2;
		interim_systolic_kpa = interim_systolic_kpa + (operator_blp[8]);
	}

	/** Appending diastolic in kpa*/
	blp_data[idx++] = 0;
	blp_data[idx++] = 0;
	
	/** Appending map in kpa */
	blp_data[idx++] = 0;
	blp_data[idx++] = 0;
	
	blp_data[0]	|= BLOOD_PRESSURE_USERID_FLAG_MASK;
	
	/** Appending User id */
	if (units) {
		blp_data[idx++] = USERID_1;
		} else {
		blp_data[idx++] = USERID_2;
	}
	
	blp_sensor_send_notification(blp_data,idx);
}

/** @brief notification handler function called by the profile
 *	@param[in] enable will give weather notification has to enabled
 *  or disabled.
 */
static void app_notification_handler(bool enable)
{
	notification_flag = enable;
	
	if (notification_flag) {
		DBG_LOG("Notifications enabled by the remote device for interim cuff pressure");
	} else{
		DBG_LOG("Disabled notifications by the remote device for interim cuff pressure");
		timer_count = INDICATION_TIMER_VAL;
	}
}

/** @brief indication handler function called by the profile
 *	@param[in] enable will give weather indication has to enabled
 *  or disabled.
 */
static void app_indication_handler(bool enable)
{
	uint8_t blp_data[BLP_DATA_LEN];
	uint8_t idx = 0;	 

	memset(blp_data, 0, sizeof(uint8_t) * BLP_DATA_LEN);
	idx = 0;

	indication_flag = enable;

	if (indication_flag) {
		DBG_LOG("Indications enabled by the remote device for blood pressure\n ");
		if (units) {
			blp_data[idx++] = 0;
			DBG_LOG("Systolic       %02d mmhg",systolic_val_mmhg);
			memcpy(&blp_data[idx],&systolic_val_mmhg,2);
			idx += 2;
			DBG_LOG("Diastolic      %02d mmhg",diastolic_val_mmhg);
			memcpy(&blp_data[idx],&diastolic_val_mmhg,2);
			idx += 2;
			DBG_LOG("Map            %02d mmhg",map_val_mmhg);
			memcpy(&blp_data[idx],&map_val_mmhg,2);
			idx += 2;
		} else {
			blp_data[idx++] = 0x1;
			memcpy(&blp_data[idx],&systolic_val_kpa,2);
			idx += 2;
			DBG_LOG("Systolic       %02d kpa",systolic_val_kpa);
			memcpy(&blp_data[idx],&diastolic_val_kpa,2);
			idx += 2;
			DBG_LOG("Diastolic      %02d kpa",diastolic_val_kpa);
			memcpy(&blp_data[idx],&map_val_kpa,2);
			idx += 2;
			DBG_LOG("Map            %02d kpa",map_val_kpa);
		}
		
		blp_data[0]	|= BLOOD_PRESSURE_PULSE_FLAG_MASK;
			DBG_LOG("Pulse rate     %d bpm",pulse_rate_val);
		memcpy(&blp_data[idx],&pulse_rate_val,2);
		idx += 2;
		/* DBG_LOG("Flags are %d and length is %d",blp_data[0],idx); */

		isIndication = true;

		memcpy(g_blp_data, blp_data, sizeof(uint8_t) * BLP_DATA_LEN);
		g_idx = idx;
		send_plf_int_msg_ind(USER_TIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);

		/* Sending the default notification for the first time */
		/* blp_sensor_send_indication(blp_data,idx); */
	} else {
		DBG_LOG("Disabled indication by the remote server for blood pressure");
	}
}

/**
 * @brief Button Press Callback
 */
static void button_cb(void)
{
	isButton = true;
	send_plf_int_msg_ind(USER_TIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
}
/*blp_sensor_app.c ends*/

/*device_info.c*/
device_info_char_value_t char_value;

bool volatile dis_notification_flag[DIS_TOTAL_CHARATERISTIC_NUM] = {false};

/**@brief Initialize the dis service related information. */
void dis_init_service(dis_gatt_service_handler_t *device_info_serv)
{
	device_info_serv->serv_handle = 0;
	device_info_serv->serv_uuid.type = AT_BLE_UUID_16;
	device_info_serv->serv_uuid.uuid[0] = (uint8_t) DIS_SERVICE_UUID;
	device_info_serv->serv_uuid.uuid[1] = (uint8_t) (DIS_SERVICE_UUID >> 8);
	
	//Characteristic Info for Manufacturer Name String
	device_info_serv->serv_chars[0].char_val_handle = 0;          /* handle stored here */
	device_info_serv->serv_chars[0].uuid.type = AT_BLE_UUID_16;
	device_info_serv->serv_chars[0].uuid.uuid[0] = (uint8_t) DIS_CHAR_MANUFACTURER_NAME_UUID;          /* UUID : Manufacturer Name String */
	device_info_serv->serv_chars[0].uuid.uuid[1] = (uint8_t) (DIS_CHAR_MANUFACTURER_NAME_UUID >> 8);   /* UUID : Manufacturer Name String */
	device_info_serv->serv_chars[0].properties = AT_BLE_CHAR_READ; /* Properties */
	
	memcpy(char_value.manufacturer_name,DEFAULT_MANUFACTURER_NAME,DIS_CHAR_MANUFACTURER_NAME_INIT_LEN);
	device_info_serv->serv_chars[0].init_value = char_value.manufacturer_name;
	
	device_info_serv->serv_chars[0].value_init_len = DIS_CHAR_MANUFACTURER_NAME_INIT_LEN;
	device_info_serv->serv_chars[0].value_max_len = DIS_CHAR_MANUFACTURER_NAME_MAX_LEN;
	#if BLE_PAIR_ENABLE
	device_info_serv->serv_chars[0].value_permissions = AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR;   /* permissions */
	#else
	device_info_serv->serv_chars[0].value_permissions = AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR;   /* permissions */
	#endif
	device_info_serv->serv_chars[0].user_desc = NULL;           /* user defined name */
	device_info_serv->serv_chars[0].user_desc_len = 0;
	device_info_serv->serv_chars[0].user_desc_max_len = 0;
	device_info_serv->serv_chars[0].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;             /*user description permissions*/
	device_info_serv->serv_chars[0].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*client config permissions*/
	device_info_serv->serv_chars[0].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*server config permissions*/
	device_info_serv->serv_chars[0].user_desc_handle = 0;             /*user desc handles*/
	device_info_serv->serv_chars[0].client_config_handle = 0;         /*client config handles*/
	device_info_serv->serv_chars[0].server_config_handle = 0;         /*server config handles*/
	device_info_serv->serv_chars[0].presentation_format = NULL;       /* presentation format */
	
	
	//Characterisitc Info for Model Number String
	device_info_serv->serv_chars[1].char_val_handle = 0;          /* handle stored here */
	device_info_serv->serv_chars[1].uuid.type = AT_BLE_UUID_16;
	device_info_serv->serv_chars[1].uuid.uuid[0] = (uint8_t) DIS_CHAR_MODEL_NUMBER_UUID;          /* UUID : Serial Number String*/
	device_info_serv->serv_chars[1].uuid.uuid[1] = (uint8_t) (DIS_CHAR_MODEL_NUMBER_UUID >> 8);          /* UUID : Serial Number String*/
	device_info_serv->serv_chars[1].properties = AT_BLE_CHAR_READ; /* Properties */
	
	memcpy(char_value.default_model_number,DEFAULT_MODEL_NUMBER,DIS_CHAR_MODEL_NUMBER_INIT_LEN);
	device_info_serv->serv_chars[1].init_value = char_value.default_model_number;
	
	device_info_serv->serv_chars[1].value_init_len = DIS_CHAR_MODEL_NUMBER_INIT_LEN;
	device_info_serv->serv_chars[1].value_max_len = DIS_CHAR_MODEL_NUMBER_MAX_LEN;
	#if BLE_PAIR_ENABLE
	device_info_serv->serv_chars[1].value_permissions = AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR;   /* permissions */
	#else
	device_info_serv->serv_chars[1].value_permissions = AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR;   /* permissions */
	#endif
	device_info_serv->serv_chars[1].user_desc = NULL;           /* user defined name */
	device_info_serv->serv_chars[1].user_desc_len = 0;
	device_info_serv->serv_chars[1].user_desc_max_len = 0;
	device_info_serv->serv_chars[1].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;             /*user description permissions*/
	device_info_serv->serv_chars[1].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*client config permissions*/
	device_info_serv->serv_chars[1].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*server config permissions*/
	device_info_serv->serv_chars[1].user_desc_handle = 0;             /*user desc handles*/
	device_info_serv->serv_chars[1].client_config_handle = 0;         /*client config handles*/
	device_info_serv->serv_chars[1].server_config_handle = 0;         /*server config handles*/
	device_info_serv->serv_chars[1].presentation_format = NULL;       /* presentation format */


	//Characteristic Info for Serial String
	device_info_serv->serv_chars[2].char_val_handle = 0;          /* handle stored here */
	device_info_serv->serv_chars[2].uuid.type = AT_BLE_UUID_16;
	device_info_serv->serv_chars[2].uuid.uuid[0] = (uint8_t) DIS_CHAR_SERIAL_NUMBER_UUID;          /* UUID : Hardware Revision String*/
	device_info_serv->serv_chars[2].uuid.uuid[1] = (uint8_t) (DIS_CHAR_SERIAL_NUMBER_UUID >> 8);          /* UUID : Hardware Revision String*/
	device_info_serv->serv_chars[2].properties = AT_BLE_CHAR_READ; /* Properties */
	
	memcpy(char_value.default_serial_number,DEFAULT_SERIAL_NUMBER,DIS_CHAR_SERIAL_NUMBER_INIT_LEN);
	device_info_serv->serv_chars[2].init_value = char_value.default_serial_number;
	
	device_info_serv->serv_chars[2].value_init_len = DIS_CHAR_SERIAL_NUMBER_INIT_LEN;
	device_info_serv->serv_chars[2].value_max_len = DIS_CHAR_SERIAL_NUMBER_MAX_LEN;
	#if BLE_PAIR_ENABLE
	device_info_serv->serv_chars[2].value_permissions = AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR;   /* permissions */
	#else
	device_info_serv->serv_chars[2].value_permissions = AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR;   /* permissions */
	#endif
	device_info_serv->serv_chars[2].user_desc = NULL;           /* user defined name */
	device_info_serv->serv_chars[2].user_desc_len = 0;
	device_info_serv->serv_chars[2].user_desc_max_len = 0;
	device_info_serv->serv_chars[2].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;             /*user description permissions*/
	device_info_serv->serv_chars[2].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*client config permissions*/
	device_info_serv->serv_chars[2].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*server config permissions*/
	device_info_serv->serv_chars[2].user_desc_handle = 0;             /*user desc handles*/
	device_info_serv->serv_chars[2].client_config_handle = 0;         /*client config handles*/
	device_info_serv->serv_chars[2].server_config_handle = 0;         /*server config handles*/
	device_info_serv->serv_chars[2].presentation_format = NULL;       /* presentation format */


	//Characteristic Info for Hardware Revision String
	device_info_serv->serv_chars[3].char_val_handle = 0;          /* handle stored here */
	device_info_serv->serv_chars[3].uuid.type = AT_BLE_UUID_16;
	device_info_serv->serv_chars[3].uuid.uuid[0] = (uint8_t) DIS_CHAR_HARDWARE_REVISION_UUID;          /* UUID : Firmware Revision String*/
	device_info_serv->serv_chars[3].uuid.uuid[1] = (uint8_t) (DIS_CHAR_HARDWARE_REVISION_UUID >> 8);          /* UUID : Firmware Revision String*/
	device_info_serv->serv_chars[3].properties = AT_BLE_CHAR_READ; /* Properties */
	
	memcpy(char_value.default_hardware_revision,DEFAULT_HARDWARE_REVISION,DIS_CHAR_HARDWARE_REVISION_INIT_LEN);
	device_info_serv->serv_chars[3].init_value = char_value.default_hardware_revision;
	
	device_info_serv->serv_chars[3].value_init_len = DIS_CHAR_HARDWARE_REVISION_INIT_LEN;
	device_info_serv->serv_chars[3].value_max_len = DIS_CHAR_HARDWARE_REVISION_MAX_LEN;
	#if BLE_PAIR_ENABLE
	device_info_serv->serv_chars[3].value_permissions = AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR;   /* permissions */
	#else
	device_info_serv->serv_chars[3].value_permissions = AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR;   /* permissions */
	#endif
	device_info_serv->serv_chars[3].user_desc = NULL;           /* user defined name */
	device_info_serv->serv_chars[3].user_desc_len = 0;
	device_info_serv->serv_chars[3].user_desc_max_len = 0;
	device_info_serv->serv_chars[3].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;             /*user description permissions*/
	device_info_serv->serv_chars[3].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*client config permissions*/
	device_info_serv->serv_chars[3].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*server config permissions*/
	device_info_serv->serv_chars[3].user_desc_handle = 0;             /*user desc handles*/
	device_info_serv->serv_chars[3].client_config_handle = 0;         /*client config handles*/
	device_info_serv->serv_chars[3].server_config_handle = 0;         /*server config handles*/
	device_info_serv->serv_chars[3].presentation_format = NULL;       /* presentation format */
	
	
	//Characteristic Info for Firmware  Revision
	device_info_serv->serv_chars[4].char_val_handle = 0;          /* handle stored here */
	device_info_serv->serv_chars[4].uuid.type = AT_BLE_UUID_16;
	device_info_serv->serv_chars[4].uuid.uuid[0] = (uint8_t) DIS_CHAR_FIRMWARE_REIVSION_UUID;          /* UUID : Software Revision */
	device_info_serv->serv_chars[4].uuid.uuid[1] = (uint8_t) (DIS_CHAR_FIRMWARE_REIVSION_UUID >> 8);          /* UUID : Software Revision */
	device_info_serv->serv_chars[4].properties = AT_BLE_CHAR_READ; /* Properties */
	
	memcpy(char_value.default_firmware_revision,DEFAULT_FIRMWARE_REIVSION,DIS_CHAR_FIRMWARE_REIVSION_INIT_LEN);
	device_info_serv->serv_chars[4].init_value = char_value.default_firmware_revision;
	
	device_info_serv->serv_chars[4].value_init_len = DIS_CHAR_FIRMWARE_REIVSION_INIT_LEN;
	device_info_serv->serv_chars[4].value_max_len = DIS_CHAR_FIRMWARE_REIVSION_MAX_LEN;
	#if BLE_PAIR_ENABLE
	device_info_serv->serv_chars[4].value_permissions = AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR;   /* permissions */
	#else
	device_info_serv->serv_chars[4].value_permissions = AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR;   /* permissions */
	#endif
	device_info_serv->serv_chars[4].user_desc = NULL;           /* user defined name */
	device_info_serv->serv_chars[4].user_desc_len = 0;
	device_info_serv->serv_chars[4].user_desc_max_len = 0;
	device_info_serv->serv_chars[4].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;             /*user description permissions*/
	device_info_serv->serv_chars[4].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*client config permissions*/
	device_info_serv->serv_chars[4].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*server config permissions*/
	device_info_serv->serv_chars[4].user_desc_handle = 0;             /*user desc handles*/
	device_info_serv->serv_chars[4].client_config_handle = 0;         /*client config handles*/
	device_info_serv->serv_chars[4].server_config_handle = 0;         /*server config handles*/
	device_info_serv->serv_chars[4].presentation_format = NULL;       /* presentation format */
	
	//Characteristic Info for Software  Revision
	device_info_serv->serv_chars[5].char_val_handle = 0;          /* handle stored here */
	device_info_serv->serv_chars[5].uuid.type = AT_BLE_UUID_16;
	device_info_serv->serv_chars[5].uuid.uuid[0] = (uint8_t) DIS_CHAR_SOFTWARE_REVISION_UUID;          /* uuid : software revision */
	device_info_serv->serv_chars[5].uuid.uuid[1] = (uint8_t) (DIS_CHAR_SOFTWARE_REVISION_UUID >> 8);          /* uuid : software revision */
	device_info_serv->serv_chars[5].properties = AT_BLE_CHAR_READ; /* properties */
	
	memcpy(char_value.default_software_revision,DEFAULT_SOFTWARE_REVISION,DIS_CHAR_SOFTWARE_REVISION_INIT_LEN);
	device_info_serv->serv_chars[5].init_value = char_value.default_software_revision;
	
	device_info_serv->serv_chars[5].value_init_len = DIS_CHAR_SOFTWARE_REVISION_INIT_LEN;
	device_info_serv->serv_chars[5].value_max_len = DIS_CHAR_SOFTWARE_REVISION_MAX_LEN;
	#if BLE_PAIR_ENABLE
	device_info_serv->serv_chars[5].value_permissions = AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR;   /* permissions */
	#else
	device_info_serv->serv_chars[5].value_permissions = AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR;   /* permissions */
	#endif
	device_info_serv->serv_chars[5].user_desc = NULL;           /* user defined name */
	device_info_serv->serv_chars[5].user_desc_len = 0;
	device_info_serv->serv_chars[5].user_desc_max_len = 0;
	device_info_serv->serv_chars[5].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;             /*user description permissions*/
	device_info_serv->serv_chars[5].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*client config permissions*/
	device_info_serv->serv_chars[5].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*server config permissions*/
	device_info_serv->serv_chars[5].user_desc_handle = 0;             /*user desc handles*/
	device_info_serv->serv_chars[5].client_config_handle = 0;         /*client config handles*/
	device_info_serv->serv_chars[5].server_config_handle = 0;         /*server config handles*/
	device_info_serv->serv_chars[5].presentation_format = NULL;       /* presentation format */
	
	//Characteristic Info for SystemID  Number
	device_info_serv->serv_chars[6].char_val_handle = 0;          /* handle stored here */
	device_info_serv->serv_chars[6].uuid.type = AT_BLE_UUID_16;
	device_info_serv->serv_chars[6].uuid.uuid[0] = (uint8_t) DIS_CHAR_SYSTEM_ID_UUID;          /* UUID : Software Revision */
	device_info_serv->serv_chars[6].uuid.uuid[1] = (uint8_t) (DIS_CHAR_SYSTEM_ID_UUID >> 8);          /* UUID : Software Revision */
	device_info_serv->serv_chars[6].properties = AT_BLE_CHAR_READ; /* Properties */
	
	memcpy(char_value.default_system_id.manufacturer_id, SYSTEM_ID_MANUFACTURER_ID, SYSTEM_ID_MANUFACTURER_ID_LEN);
	memcpy(char_value.default_system_id.org_unique_id, SYSTEM_ID_ORG_UNIQUE_ID, SYSTEM_ID_ORG_UNIQUE_ID_LEN);
	device_info_serv->serv_chars[6].init_value = (uint8_t *) &char_value.default_system_id;					/*Initial Value*/
	
	device_info_serv->serv_chars[6].value_init_len = DIS_CHAR_SYSTEM_ID_INIT_LEN;
	device_info_serv->serv_chars[6].value_max_len = DIS_CHAR_SYSTEM_ID_MAX_LEN;
	#if BLE_PAIR_ENABLE
	device_info_serv->serv_chars[6].value_permissions = AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR;   /* permissions */
	#else
	device_info_serv->serv_chars[6].value_permissions = AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR;   /* permissions */
	#endif
	device_info_serv->serv_chars[6].user_desc = NULL;           /* user defined name */
	device_info_serv->serv_chars[6].user_desc_len = 0;
	device_info_serv->serv_chars[6].user_desc_max_len = 0;
	device_info_serv->serv_chars[6].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;             /*user description permissions*/
	device_info_serv->serv_chars[6].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*client config permissions*/
	device_info_serv->serv_chars[6].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*server config permissions*/
	device_info_serv->serv_chars[6].user_desc_handle = 0;             /*user desc handles*/
	device_info_serv->serv_chars[6].client_config_handle = 0;         /*client config handles*/
	device_info_serv->serv_chars[6].server_config_handle = 0;         /*server config handles*/
	device_info_serv->serv_chars[6].presentation_format = NULL;       /* presentation format */
	
	//Characteristic Info for PnP ID
	device_info_serv->serv_chars[7].char_val_handle = 0;          /* handle stored here */
	device_info_serv->serv_chars[7].uuid.type = AT_BLE_UUID_16;
	device_info_serv->serv_chars[7].uuid.uuid[0] = (uint8_t) DIS_CHAR_PNP_ID_UUID;          /* UUID : Software Revision */
	device_info_serv->serv_chars[7].uuid.uuid[1] = (uint8_t) (DIS_CHAR_PNP_ID_UUID >> 8);          /* UUID : Software Revision */
	device_info_serv->serv_chars[7].properties = AT_BLE_CHAR_READ; /* Properties */
	
	char_value.default_pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;					/*characteristic value initialization */
	char_value.default_pnp_id.vendor_id = PNP_ID_VENDOR_ID;
	char_value.default_pnp_id.product_id= PNP_ID_PRODUCT_ID;
	char_value.default_pnp_id.product_version= PNP_ID_PRODUCT_VERSION;
	device_info_serv->serv_chars[7].init_value = (uint8_t *) &char_value.default_pnp_id;					/*Initial Value*/
	
	device_info_serv->serv_chars[7].value_init_len = DIS_CHAR_PNP_ID_INIT_LEN;
	device_info_serv->serv_chars[7].value_max_len = DIS_CHAR_PNP_ID_MAX_LEN;
	#if BLE_PAIR_ENABLE
	device_info_serv->serv_chars[7].value_permissions = AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR;   /* permissions */
	#else
	device_info_serv->serv_chars[7].value_permissions = AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR;   /* permissions */
	#endif
	device_info_serv->serv_chars[7].user_desc = NULL;           /* user defined name */
	device_info_serv->serv_chars[7].user_desc_len = 0;
	device_info_serv->serv_chars[7].user_desc_max_len = 0;
	device_info_serv->serv_chars[7].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;             /*user description permissions*/
	device_info_serv->serv_chars[7].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*client config permissions*/
	device_info_serv->serv_chars[7].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*server config permissions*/
	device_info_serv->serv_chars[7].user_desc_handle = 0;             /*user desc handles*/
	device_info_serv->serv_chars[7].client_config_handle = 0;         /*client config handles*/
	device_info_serv->serv_chars[7].server_config_handle = 0;         /*server config handles*/
	device_info_serv->serv_chars[7].presentation_format = NULL;       /* presentation format */
	
	//Characteristic Info for IEEE 11073-20601 Regulatory Certification Data List
	device_info_serv->serv_chars[8].char_val_handle = 0;          /* handle stored here */
	device_info_serv->serv_chars[8].uuid.type = AT_BLE_UUID_16;
	device_info_serv->serv_chars[8].uuid.uuid[0] = (uint8_t) DIS_CHAR_IEEE_REG_CERT_DATA_LIST_UUID;          /* UUID : Software Revision */
	device_info_serv->serv_chars[8].uuid.uuid[1] = (uint8_t) (DIS_CHAR_IEEE_REG_CERT_DATA_LIST_UUID >> 8);          /* UUID : Software Revision */
	device_info_serv->serv_chars[8].properties = AT_BLE_CHAR_READ; /* Properties */
	device_info_serv->serv_chars[8].init_value = char_value.ieee_reg_cert_data_list;					/*Initial Value*/
	device_info_serv->serv_chars[8].value_init_len = DIS_CHAR_IEEE_REG_CERT_DATA_LIST_INIT_LEN;
	device_info_serv->serv_chars[8].value_max_len = DIS_CHAR_IEEE_REG_CERT_DATA_LIST_MAX_LEN;
	#if BLE_PAIR_ENABLE
	device_info_serv->serv_chars[8].value_permissions = AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR;   /* permissions */
	#else
	device_info_serv->serv_chars[8].value_permissions = AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR;   /* permissions */
	#endif
	device_info_serv->serv_chars[8].user_desc = NULL;           /* user defined name */
	device_info_serv->serv_chars[8].user_desc_len = 0;
	device_info_serv->serv_chars[8].user_desc_max_len = 0;
	device_info_serv->serv_chars[8].user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;             /*user description permissions*/
	device_info_serv->serv_chars[8].client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*client config permissions*/
	device_info_serv->serv_chars[8].server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*server config permissions*/
	device_info_serv->serv_chars[8].user_desc_handle = 0;             /*user desc handles*/
	device_info_serv->serv_chars[8].client_config_handle = 0;         /*client config handles*/
	device_info_serv->serv_chars[8].server_config_handle = 0;         /*server config handles*/
	device_info_serv->serv_chars[8].presentation_format = NULL;       /* presentation format */
}

/**@brief Register a dis service instance inside stack. */
at_ble_status_t dis_primary_service_define(dis_gatt_service_handler_t *dis_primary_service)
{
	
	return(at_ble_primary_service_define(&dis_primary_service->serv_uuid,
	&dis_primary_service->serv_handle,
	NULL, 0,
	dis_primary_service->serv_chars,DIS_TOTAL_CHARATERISTIC_NUM));
}

/**@brief  Update the DIS characteristic value after defining the services using dis_primary_service_define*/
at_ble_status_t dis_info_update(dis_gatt_service_handler_t *dis_serv , dis_info_type info_type, dis_info_data* info_data, at_ble_handle_t conn_handle)
{
	if (info_data->data_len > dis_serv->serv_chars[info_type].value_max_len)
	{
		DBG_LOG("invalid length parameter");
		return AT_BLE_FAILURE;
	}
	
	//updating application data
	memcpy(&(dis_serv->serv_chars[info_type].init_value), info_data->info_data,info_data->data_len);
	
	//updating the characteristic value
	if ((at_ble_characteristic_value_set(dis_serv->serv_chars[info_type].char_val_handle, info_data->info_data, info_data->data_len)) != AT_BLE_SUCCESS){
		DBG_LOG("updating the characteristic failed\r\n");
		} else {
		return AT_BLE_SUCCESS;
	}
	ALL_UNUSED(conn_handle);
	return AT_BLE_FAILURE;
}

/*device_info.c ends*/

/*blood pressure.c starts*/
/************************************************************************/
/*							Globals										 */
/************************************************************************/
/** initial heart rate measurement value */
uint16_t blp_measurement_value = DEFAULT_VALUE;

/** initial body sensor location value */
uint8_t intermediate_cuff_pressure_value = DEFAULT_VALUE;

/** initial blood pressure feature value */
uint16_t blood_pressure_feature_value = 0x001f;



/************************************************************************/
/*							Implementation								*/
/************************************************************************/

/**@brief Blood Pressure service and characteristic initialization(Called only once
 * by user).
 *
 * @param[in] Blood Pressure service instance
 *
 * @return none
 */
void blp_init_service(blp_gatt_service_handler_t *blood_pressure_serv)
{
	blp_measurement_value = DEFAULT_VALUE;
	intermediate_cuff_pressure_value = DEFAULT_VALUE;
	blood_pressure_feature_value = 0x001f;

	blood_pressure_serv->serv_handle = 0;
	blood_pressure_serv->serv_uuid.type = AT_BLE_UUID_16;
	blood_pressure_serv->serv_uuid.uuid[0] = (uint8_t)BLOOD_PRESSURE_SERVICE_UUID;
	blood_pressure_serv->serv_uuid.uuid[1]
		= (uint8_t)(BLOOD_PRESSURE_SERVICE_UUID >> 8);

	/*Characteristic Info for Blood Pressure Measurement*/

	/* handle stored here */
	blood_pressure_serv->serv_chars[0].char_val_handle = 0;
	blood_pressure_serv->serv_chars[0].uuid.type = AT_BLE_UUID_16;

	/* UUID : Blood Pressure Measurement Characteristic */
	blood_pressure_serv->serv_chars[0].uuid.uuid[0]
		= (uint8_t)BLOOD_PRESSURE_MEASUREMENT_CHAR_UUID;
	blood_pressure_serv->serv_chars[0].uuid.uuid[1]
		= (uint8_t)(BLOOD_PRESSURE_MEASUREMENT_CHAR_UUID >> 8);

	/* Properties */
	blood_pressure_serv->serv_chars[0].properties = AT_BLE_CHAR_INDICATE;

	blood_pressure_serv->serv_chars[0].init_value
		= (uint8_t *)&blp_measurement_value;
	blood_pressure_serv->serv_chars[0].value_init_len = sizeof(uint16_t);

	blood_pressure_serv->serv_chars[0].value_max_len = BLP_MM_FLAGS_SIZE +
		BLP_SYSTOLIC_VAL_SIZE + BLP_DIASTOLIC_VAL_SIZE + BLP_MAP_VAL_SIZE+
		BLP_MAX_TIME_STAMP_SIZE + BLP_MAX_PULSE_RATE_SIZE + BLP_MAX_USER_ID_SIZE+
		BLP_MAX_MM_STATUS_SIZE;
		
	/* Permissions for characteristics */
	blood_pressure_serv->serv_chars[0].value_permissions
										 	= AT_BLE_ATTR_NO_PERMISSIONS;
	/* user defined */
	blood_pressure_serv->serv_chars[0].user_desc = NULL;
	blood_pressure_serv->serv_chars[0].user_desc_len = 0;
	blood_pressure_serv->serv_chars[0].user_desc_max_len = 0;
	/*user description permissions*/
	blood_pressure_serv->serv_chars[0].user_desc_permissions
		= AT_BLE_ATTR_NO_PERMISSIONS;
	/*client config permissions*/
	#if BLE_PAIR_ENABLE
			blood_pressure_serv->serv_chars[0].client_config_permissions
			=	( AT_BLE_ATTR_WRITABLE_REQ_AUTHN_NO_AUTHR);
	#else 
			blood_pressure_serv->serv_chars[0].client_config_permissions
			= AT_BLE_ATTR_NO_PERMISSIONS;
	#endif 

	/*server config permissions*/
	blood_pressure_serv->serv_chars[0].server_config_permissions
		= AT_BLE_ATTR_NO_PERMISSIONS;
	/*user desc handles*/
	blood_pressure_serv->serv_chars[0].user_desc_handle = 0;
	/*client config handles*/
	blood_pressure_serv->serv_chars[0].client_config_handle = 0;
	/*server config handles*/
	blood_pressure_serv->serv_chars[0].server_config_handle = 0;
	/* presentation format */
	blood_pressure_serv->serv_chars[0].presentation_format = NULL;

	/* Characteristic Info for Intermediate Cuff Pressure */
	blood_pressure_serv->serv_chars[1].char_val_handle = 0; 
	         /* handle stored here */
	blood_pressure_serv->serv_chars[1].uuid.type = AT_BLE_UUID_16;
	/* UUID : Intermediate Cuff Pressure*/
	blood_pressure_serv->serv_chars[1].uuid.uuid[0]
		= (uint8_t)INTERMEDIATE_CUFF_PRESSURE_CHAR_UUID;
	/* UUID : Body Sensor location*/
	blood_pressure_serv->serv_chars[1].uuid.uuid[1]
		= (uint8_t)(INTERMEDIATE_CUFF_PRESSURE_CHAR_UUID >> 8);
	/* Properties */
	blood_pressure_serv->serv_chars[1].properties = AT_BLE_CHAR_NOTIFY;

	blood_pressure_serv->serv_chars[1].init_value = &intermediate_cuff_pressure_value;

	blood_pressure_serv->serv_chars[1].value_init_len = sizeof(uint8_t);
	blood_pressure_serv->serv_chars[1].value_max_len = BLP_MM_FLAGS_SIZE +
	BLP_SYSTOLIC_VAL_SIZE + BLP_DIASTOLIC_VAL_SIZE + BLP_MAP_VAL_SIZE+
	BLP_MAX_TIME_STAMP_SIZE + BLP_MAX_PULSE_RATE_SIZE + BLP_MAX_USER_ID_SIZE+
	BLP_MAX_MM_STATUS_SIZE;

		/* permissions */
		blood_pressure_serv->serv_chars[1].value_permissions
							= AT_BLE_ATTR_NO_PERMISSIONS;
	
	/* user defined name */
	blood_pressure_serv->serv_chars[1].user_desc = NULL;
	blood_pressure_serv->serv_chars[1].user_desc_len = 0;
	blood_pressure_serv->serv_chars[1].user_desc_max_len = 0;
	/*user description permissions*/
	blood_pressure_serv->serv_chars[1].user_desc_permissions
		= AT_BLE_ATTR_NO_PERMISSIONS;
		
	/*client config permissions*/
	#if BLE_PAIR_ENABLE
	blood_pressure_serv->serv_chars[1].client_config_permissions
									= (AT_BLE_ATTR_WRITABLE_REQ_AUTHN_NO_AUTHR);
	#else
	blood_pressure_serv->serv_chars[1].client_config_permissions
	= AT_BLE_ATTR_NO_PERMISSIONS;
	#endif

	/*server config permissions*/
	blood_pressure_serv->serv_chars[1].server_config_permissions
		= AT_BLE_ATTR_NO_PERMISSIONS;
	/*user desc handles*/
	blood_pressure_serv->serv_chars[1].user_desc_handle = 0;
	/*client config handles*/
	blood_pressure_serv->serv_chars[1].client_config_handle = 0;
	/*server config handles*/
	blood_pressure_serv->serv_chars[1].server_config_handle = 0;
	/* presentation format */
	blood_pressure_serv->serv_chars[1].presentation_format = NULL;

	/* Characteristic Info for blood Pressure feature Characteristic */
	/* handle stored here */
	blood_pressure_serv->serv_chars[2].char_val_handle = 0;
	blood_pressure_serv->serv_chars[2].uuid.type = AT_BLE_UUID_16;
	/* UUID : Heart Rate Control Point*/
	blood_pressure_serv->serv_chars[2].uuid.uuid[0]
		= (uint8_t)BLOOD_PRESSURE_FEATURE_CHAR_UUID;
	/* UUID : Heart Rate Control Point*/
	blood_pressure_serv->serv_chars[2].uuid.uuid[1]
		= (uint8_t)(BLOOD_PRESSURE_FEATURE_CHAR_UUID >> 8);
	/* Properties */
	blood_pressure_serv->serv_chars[2].properties = AT_BLE_CHAR_READ;

	/* Initial Value */
	blood_pressure_serv->serv_chars[2].init_value =(uint8_t *) &blood_pressure_feature_value;

	blood_pressure_serv->serv_chars[2].value_init_len = sizeof(uint16_t);
	blood_pressure_serv->serv_chars[2].value_max_len = sizeof(uint16_t);
	
	#if BLE_PAIR_ENABLE
			/* permissions */
			blood_pressure_serv->serv_chars[2].value_permissions
						= AT_BLE_ATTR_READABLE_REQ_AUTHN_NO_AUTHR;
	#else
			/* permissions */
			blood_pressure_serv->serv_chars[2].value_permissions
						= AT_BLE_ATTR_NO_PERMISSIONS;
	#endif

	/* user defined name */
	blood_pressure_serv->serv_chars[2].user_desc = NULL;
	blood_pressure_serv->serv_chars[2].user_desc_len = 0;
	blood_pressure_serv->serv_chars[2].user_desc_max_len = 0;
	/*user description permissions*/
	blood_pressure_serv->serv_chars[2].user_desc_permissions
		= AT_BLE_ATTR_NO_PERMISSIONS;
	/*client config permissions*/
	blood_pressure_serv->serv_chars[2].client_config_permissions
		= AT_BLE_ATTR_NO_PERMISSIONS;
	/*server config permissions*/
	blood_pressure_serv->serv_chars[2].server_config_permissions
		= AT_BLE_ATTR_NO_PERMISSIONS;
	/*user desc handles*/
	blood_pressure_serv->serv_chars[2].user_desc_handle = 0;
	/*client config handles*/
	blood_pressure_serv->serv_chars[2].client_config_handle = 0;
	/*server config handles*/
	blood_pressure_serv->serv_chars[2].server_config_handle = 0;
	/* presentation format */
	blood_pressure_serv->serv_chars[2].presentation_format = NULL;
}

/**@brief Register a BLP service instance inside stack.
 *
 * @param[in] blp_primary_service blp service instance
 *
 * @return @ref AT_BLE_SUCCESS operation completed successfully
 * @return @ref AT_BLE_FAILURE Generic error.
 */
at_ble_status_t blp_primary_service_define(
		blp_gatt_service_handler_t *blp_primary_service)
{
	return(at_ble_primary_service_define(&blp_primary_service->serv_uuid,
	       &blp_primary_service->serv_handle,
	       NULL, 0,
	       blp_primary_service->serv_chars, BLP_TOTAL_CHARATERISTIC_NUM));
}

/**@brief handler for characteristic write called by profile
 *
 * @param[in] blp_gatt_service_handler_t service handler
 *
 * @return number representing the changed characteristic
 */
uint8_t blp_char_change_handler(blp_gatt_service_handler_t *blp_primary_service,
		at_ble_characteristic_changed_t *params)
{
	if (params->char_handle ==
			blp_primary_service->serv_chars[0].client_config_handle) {
		if (params->char_new_value[0] == BLP_INDICATION) {
			return BLP_INDICATION_ENABLE;
		} else if (params->char_new_value[0] == false) {
			return BLP_INDICATION_DISABLE;
		} 
	}
	
	if (params->char_handle ==
	blp_primary_service->serv_chars[1].client_config_handle) {
		if (params->char_new_value[0] == BLP_NOTIFICATION) {
			return BLP_NOTIFICATION_ENABLE;
		} else if (params->char_new_value[0] == false) {
			return BLP_NOTIFICATION_DISABLE;
		}
	} 
	return 0xff;
}

/*blood_pressure.c ends*/

/*blp_sensor.c starts*/
/** @brief device information service handler **/
dis_gatt_service_handler_t dis_service_handler;

/** @brief blood pressure measurement service handler **/
blp_gatt_service_handler_t blp_service_handler;

/** @brief callback functions pointers contains the address of application
 *functions **/
blp_notification_callback_t notification_cb;

blp_indication_callback_t	indication_cb;


/** @brief contains the connection handle functions **/
at_ble_handle_t connection_handle;


static /*const*/ ble_event_callback_t blp_sensor_gap_handle[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	blp_sensor_connected_state_handler,
	blp_sensor_disconnect_event_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

static /*const*/ ble_event_callback_t blp_sensor_gatt_server_handle[] = {
	blp_notification_confirmation_handler,
	blp_indication_confirmation_handler,
	blp_sensor_char_changed_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};
/****************************************************************************************
*							        Implementations										*
****************************************************************************************/


/** @brief register_blp_notification_handler registers the notification handler
 * passed by the application
 *  @param[in] blp_notification_callback_t address of the notification handler
 *function to be called
 */
void register_blp_notification_handler(
		blp_notification_callback_t blp_notificaton_handler)
{
	notification_cb = blp_notificaton_handler;
}


/** @brief register_blp_indication_handler registers the indication handler
 * passed by the application
 *  @param[in] blp_indication_callback_t address of the indication handler
 *function to be called
 */
void register_blp_indication_handler(
		blp_indication_callback_t blp_indication_handler)
{
		indication_cb = blp_indication_handler;
}

/** @brief blp_notification_confirmation_handler called by ble manager 
 *	to give the status of notification sent
 *  @param[in] at_ble_cmd_complete_event_t address of the cmd completion
 *  @return AT_BLE_SUCCESS on success
 */	
at_ble_status_t blp_notification_confirmation_handler(void *params)
{
	at_ble_cmd_complete_event_t event_params;
	memcpy(&event_params,params,sizeof(at_ble_cmd_complete_event_t));
	
	if (event_params.status == AT_BLE_SUCCESS) {
		DBG_LOG_DEV("Notification Successfully sent over the air");
	} else {
		DBG_LOG_DEV("Sending Notification over the air failed");
	}
	return AT_BLE_SUCCESS;
}

/** @brief blp_indication_confirmation_handler called by ble manager 
 *	to give the status of notification sent
 *  @param[in] at_ble_cmd_complete_event_t address of the cmd completion
 *  @return AT_BLE_SUCCESS on success
 */	
at_ble_status_t blp_indication_confirmation_handler(void *params)
{
	at_ble_cmd_complete_event_t event_params;
	memcpy(&event_params,params,sizeof(at_ble_cmd_complete_event_t));
	
	if (event_params.status == AT_BLE_SUCCESS){
		DBG_LOG_DEV("Indication successfully sent over the air");
	} else {
		DBG_LOG_DEV("Sending indication over the air failed %d",event_params.status);
	}
	
	return AT_BLE_SUCCESS;
}


/** @brief blp_sensor_send_notification adds the new characteristic value and
 * sends the notification
 *  @param[in] blp_data the new hr characteristic value needs to be updated
 *  @param[in] length length of new characteristic value
 */
void blp_sensor_send_notification(uint8_t *blp_data, uint8_t length)
{
	at_ble_status_t status = AT_BLE_SUCCESS;

	/** Updating the new characteristic value */
	if ((status
				= at_ble_characteristic_value_set(
					blp_service_handler.serv_chars
					[1].char_val_handle, blp_data,
					length)) != AT_BLE_SUCCESS) {
		DBG_LOG("at_ble_characteristic_value_set for notification failed,reason %x",
				status);
				return;
	}

	/** Sending the notification for the updated characteristic */
	if ((status	= at_ble_notification_send(connection_handle,
					blp_service_handler.serv_chars[1]
					.char_val_handle)) != AT_BLE_SUCCESS) {
		DBG_LOG("Send notification failed,reason %x", status);
	}
}

/** @brief blp_sensor_send_indication adds the new characteristic value and
 * sends the indication
 *  @param[in] blp the new blp characteristic value needs to be updated
 *  @param[in] length length of new characteristic value
 */
void blp_sensor_send_indication(uint8_t *blp_data, uint8_t length)
{
	at_ble_status_t status = AT_BLE_SUCCESS;

	/** Updating the new characteristic value */
	if ((status
				= at_ble_characteristic_value_set(
					blp_service_handler.serv_chars
					[0].char_val_handle, blp_data,
					length)) != AT_BLE_SUCCESS) {
		DBG_LOG("at_ble_characteristic_value_set for indication failed,reason %x",
				status);
				return;
	}

	/** Sending the indication for the updated characteristic */
	if ((status     = at_ble_indication_send(connection_handle,
					blp_service_handler.serv_chars[0]
					.char_val_handle)) != AT_BLE_SUCCESS) {
		DBG_LOG("Send indication failed,reason %x", status);
	}
}


/** @brief blp_sensor_char_changed_handler called by the ble manager after a
 *  change in the characteristic
 *  @param[in] at_ble_characteristic_changed_t which contains handle of
 *  characteristic and new value
 *  @return AT_BLE_SUCCESS
 */
at_ble_status_t blp_sensor_char_changed_handler(
					void *char_handle)
{
	uint8_t action_event = 0;
	at_ble_characteristic_changed_t change_params = {0, };
	memcpy((uint8_t *)&change_params, char_handle,
			sizeof(at_ble_characteristic_changed_t));

	action_event = blp_char_change_handler(&blp_service_handler,
			&change_params);

	if ((action_event == BLP_NOTIFICATION_ENABLE) ||
			(action_event == BLP_NOTIFICATION_DISABLE)) {
		if (action_event == BLP_NOTIFICATION_ENABLE) {
			notification_cb(true);	
		} else if (action_event == BLP_NOTIFICATION_DISABLE){
			notification_cb(false);
		}
	} else if ((action_event == BLP_INDICATION_ENABLE) ||
			(action_event == BLP_INDICATION_DISABLE)) {
		if (action_event == BLP_INDICATION_ENABLE) {
			indication_cb(true);
		} else if (action_event == BLP_INDICATION_DISABLE) {
			indication_cb(false);
		}
	}

	return AT_BLE_SUCCESS;
}

/** @brief blp_sensor_disconnect_event_handler called by ble manager after
 * disconnection event received
 *	@param[in] at_ble_disconnected_t	which has connection handle and
 *reason for disconnection
 */
at_ble_status_t blp_sensor_disconnect_event_handler(
					void *disconnect)
{
	/* ALL_UNUSED(disconnect); */
	return AT_BLE_SUCCESS;
}

/** @brief blp_sensor_connected_state_handler called by ble manager after a
 * change in characteristic
 *  @param[in] at_ble_connected_t which has connection handle and the peer
 *device address
 */
at_ble_status_t blp_sensor_connected_state_handler(
		void *conn_params)
{
	connection_handle = (at_ble_handle_t)(((at_ble_connected_t *)conn_params)->handle);
	a=1;	
	return AT_BLE_SUCCESS;
}

/** @brief blp_disconnection called by the application to disconnect
 *
 */
void blp_disconnection(void)
{
	at_ble_status_t status;
	if ((status = at_ble_disconnect(connection_handle,AT_BLE_TERMINATED_BY_USER)) != AT_BLE_SUCCESS) {
		DBG_LOG("Disconnection not successuful, reason %x",status);
	}
}

/** @brief blp_sensor_adv starts advertisement
 *
 */
void blp_sensor_adv(void)
{
	at_ble_status_t status = AT_BLE_SUCCESS;
	
	/* Start of advertisement */
	if ((status
				= at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED,
					AT_BLE_ADV_GEN_DISCOVERABLE, NULL,
					AT_BLE_ADV_FP_ANY,
					BLP_SENSOR_FAST_ADV,
					BLP_SENSOR_ADV_TIMEOUT,
					0)) == AT_BLE_SUCCESS) {
		DBG_LOG("Bluetooth device is in Advertising Mode");
	} else {
		DBG_LOG(
				"Bluetooth LE Advertisement failed start Failed,reason %d",
				status);
	}
}

/** @brief blp_sensor_service_define defines the services of the profile
 *
 */
void blp_sensor_service_define(void)
{
	at_ble_status_t status = AT_BLE_SUCCESS;

	if ((status = blp_primary_service_define(&blp_service_handler)) !=
			AT_BLE_SUCCESS) {
		DBG_LOG("Blood Pressure Service definition Failed,reason: %x",
				status);
	}
	
	if ((status = dis_primary_service_define(&dis_service_handler)) !=
			AT_BLE_SUCCESS) {
		DBG_LOG("Dis Service definition failed,reason: %x", status);
	}
	
}

/**
 * \brief Initialization of profile services
 */
void blp_sensor_service_init(void)
{
	blp_init_service(&blp_service_handler);

	dis_init_service(&dis_service_handler);
}

/** @brief blp_sensor_init initializes and defines the services of the hr profile
 *
 *  @param[in] params are unused.
 *
 */
void blp_sensor_init(void *param)
{
	at_ble_status_t status;

	memset(&dis_service_handler, 0, sizeof(dis_gatt_service_handler_t));
	memset(&blp_service_handler, 0, sizeof(blp_gatt_service_handler_t));
	notification_cb = 0;
	indication_cb = 0;
	connection_handle = 0;

	blp_sensor_gap_handle[0] = NULL;
	blp_sensor_gap_handle[1] = NULL;
	blp_sensor_gap_handle[2] = NULL;
	blp_sensor_gap_handle[3] = NULL;
	blp_sensor_gap_handle[4] = NULL;
	blp_sensor_gap_handle[5] = blp_sensor_connected_state_handler;
	blp_sensor_gap_handle[6] = blp_sensor_disconnect_event_handler;
	blp_sensor_gap_handle[7] = NULL;
	blp_sensor_gap_handle[8] = NULL;
	blp_sensor_gap_handle[9] = NULL;
	blp_sensor_gap_handle[10] = NULL;
	blp_sensor_gap_handle[11] = NULL;
	blp_sensor_gap_handle[12] = NULL;
	blp_sensor_gap_handle[13] = NULL;
	blp_sensor_gap_handle[14] = NULL;
	blp_sensor_gap_handle[15] = NULL;
	blp_sensor_gap_handle[16] = NULL;
	blp_sensor_gap_handle[17] = NULL;
	blp_sensor_gap_handle[18] = NULL;

	blp_sensor_gatt_server_handle[0] = blp_notification_confirmation_handler;
	blp_sensor_gatt_server_handle[1] = blp_indication_confirmation_handler;
	blp_sensor_gatt_server_handle[2] = blp_sensor_char_changed_handler;
	blp_sensor_gatt_server_handle[3] = NULL;
	blp_sensor_gatt_server_handle[4] = NULL;
	blp_sensor_gatt_server_handle[5] = NULL;
	blp_sensor_gatt_server_handle[6] = NULL;
	blp_sensor_gatt_server_handle[7] = NULL;
	blp_sensor_gatt_server_handle[8] = NULL;
	blp_sensor_gatt_server_handle[9] = NULL;

	blp_sensor_service_init();
	blp_sensor_service_define();
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK,
	BLE_GAP_EVENT_TYPE,
	blp_sensor_gap_handle);
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK,
	BLE_GATT_SERVER_EVENT_TYPE,
	blp_sensor_gatt_server_handle);
							
	status = ble_advertisement_data_set();
	
	if (status != AT_BLE_SUCCESS) {
		DBG_LOG("Advertisement set failed reason %d",status);
	}
				
	/* Handles for the Blood pressure service */
	DBG_LOG_PTS("\n\nThe service handle for blp is 0x%04x",
	blp_service_handler.serv_handle);
	DBG_LOG_PTS("The characteristic handle for blp measurement is 0x%04x",
	blp_service_handler.serv_chars[0].char_val_handle - 1);
	DBG_LOG_PTS("The characteristic value handle for blp measurement is 0x%04x",
	blp_service_handler.serv_chars[0].char_val_handle);
	DBG_LOG_PTS("The characteristic handle for intermediate cuff pressure is "
	"0x%04x",blp_service_handler.serv_chars[1].char_val_handle - 1);
	DBG_LOG_PTS("The characteristic value handle for intermediate cuff pressure is "
	"0x%04x",blp_service_handler.serv_chars[1].char_val_handle);
	DBG_LOG_PTS("The characteristic handle for blood pressure feature is 0x%04x",
	blp_service_handler.serv_chars[2].char_val_handle - 1);
	DBG_LOG_PTS("The characteristic value handle for blood pressure feature is 0x%04x",
	blp_service_handler.serv_chars[2].char_val_handle);
	DBG_LOG_PTS("The descriptor handle for blp measurement is 0x%04x",
	blp_service_handler.serv_chars[0].client_config_handle);
	DBG_LOG_PTS("The descriptor handle for intermediate cuff press is 0x%04x",
	blp_service_handler.serv_chars[1].client_config_handle);
	/* The handles received for Device information */
	DBG_LOG_PTS("\r\nThe service handle for Device information service is 0x%04x",
	dis_service_handler.serv_handle);
	DBG_LOG_PTS("The Handles for the characteristics of DIS are given below\n");
	DBG_LOG_PTS("Characteristic 1 - 0x%04x",
	dis_service_handler.serv_chars[0].char_val_handle - 1);
	DBG_LOG_PTS("Characteristic 2 - 0x%04x",
	dis_service_handler.serv_chars[1].char_val_handle - 1);
	DBG_LOG_PTS("Characteristic 3 - 0x%04x",
	dis_service_handler.serv_chars[2].char_val_handle - 1);
	DBG_LOG_PTS("Characteristic 4 - 0x%04x",
	dis_service_handler.serv_chars[3].char_val_handle - 1);
	DBG_LOG_PTS("Characteristic 5 - 0x%04x",
	dis_service_handler.serv_chars[4].char_val_handle - 1);
	DBG_LOG_PTS("Characteristic 6 - 0x%04x",
	dis_service_handler.serv_chars[5].char_val_handle - 1);
	DBG_LOG_PTS("Characteristic 7 - 0x%04x",
	dis_service_handler.serv_chars[6].char_val_handle - 1);
	DBG_LOG_PTS("Characteristic 8 - 0x%04x",
	dis_service_handler.serv_chars[7].char_val_handle - 1);
	DBG_LOG_PTS("Characteristic 9 - 0x%04x",
	dis_service_handler.serv_chars[8].char_val_handle - 1);
							
	DBG_LOG_PTS("The default value of blood pressure feature is 0x%02x",
							*(blp_service_handler.serv_chars[2].init_value));					
							
							
	/* ALL_UNUSED(param); */
}
/*blp_sensor.c ends*/

static const ble_event_callback_t app_gatt_server_cb[] = {
	NULL,//AT_BLE_NOTIFICATION_CONFIRMED
	NULL,//AT_BLE_INDICATION_CONFIRMED
	NULL,//AT_BLE_CHARACTERISTIC_CHANGED,
	NULL,//AT_BLE_CHARACTERISTIC_CONFIGURATION_CHANGED
	NULL,//AT_BLE_SERVICE_CHANGED_INDICATION_SENT
	NULL,//AT_BLE_WRITE_AUTHORIZE_REQUEST
	NULL,//AT_BLE_MTU_CHANGED_INDICATION
	NULL,//AT_BLE_MTU_CHANGED_CMD_COMPLETE
	NULL,//AT_BLE_CHARACTERISTIC_WRITE_CMD_CMP,
	NULL //AT_BLE_READ_AUTHORIZE_REQUEST
};

static const ble_event_callback_t app_gatt_client_cb[] = {
	NULL,//AT_BLE_NOTIFICATION_CONFIRMED
	NULL,//AT_BLE_INCLUDED_SERVICE_FOUND
	NULL,//AT_BLE_CHARACTERISTIC_FOUND
	NULL,//AT_BLE_DESCRIPTOR_FOUND
	NULL,//AT_BLE_DISCOVERY_COMPLETE
	NULL,//AT_BLE_CHARACTERISTIC_READ_BY_UUID_RESPONSE
	NULL,//AT_BLE_CHARACTERISTIC_READ_MULTIBLE_RESPONSE
	NULL,//AT_BLE_CHARACTERISTIC_WRITE_RESPONSE
	NULL,//AT_BLE_NOTIFICATION_RECIEVED
	NULL //AT_BLE_INDICATION_RECIEVED
};

/* Callback registered for AT_BLE_CONNECTED event*/
static at_ble_status_t ble_paired_cb (void *param) {
	
	change_conn_params.con_intv_min = CONNECTION_INTERVAL_MIN;
	change_conn_params.con_intv_max = CONNECTION_INTERVAL_MAX;
	change_conn_params.con_latency = SLAVE_LATENCY;
	change_conn_params.ce_len_min = GAP_CE_LEN_MIN;
	change_conn_params.ce_len_max = GAP_CE_LEN_MAX;
	change_conn_params.superv_to = GAP_SUPERVISION_TIMOUT;
	
	at_ble_pair_done_t *pair_params = param;
	printf("\nAssignment 3.2: Application paired ");
	/* Enable the HTP Profile */
	printf("\nAssignment 4.1: enable health temperature service ");
	status = at_ble_htpt_enable(pair_params->handle, HTPT_CFG_INTERM_MEAS_NTF);
	if(status != AT_BLE_SUCCESS){
		printf("*** Failure in HTP Profile Enable");
		while(true);
	}
	at_ble_connection_param_update(pair_params -> handle, &change_conn_params);
	ALL_UNUSED(param);
	return AT_BLE_SUCCESS;
}

static void htp_init (void) {
	printf("\nAssignment 4.1: Init Health temperature service ");
	/* Create htp service in GATT database*/
	status = at_ble_htpt_create_db( HTPT_TEMP_TYPE_CHAR_SUP,
	HTP_TYPE_FINGER,
	1,
	30,
	1,
	HTPT_AUTH,
	&htpt_conn_handle);
	if (status != AT_BLE_SUCCESS){
		printf("HTP Data Base creation failed");
		while(true);
	}
}

/* Timer callback */
static void timer_callback_handler(void) {
	/* Stop timer */
	hw_timer_stop();
	/* Set timer Alarm flag */
	Timer_Flag = true;
	/* Restart Timer */

	isTimer = true;
	send_plf_int_msg_ind(USER_TIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
	hw_timer_start(10);
}

static void ble_advertise (void) {
	printf("\nAssignment 2.1 : Start Advertising");
	status = ble_advertisement_data_set();
	if(status != AT_BLE_SUCCESS) {
		printf("\n\r## Advertisement data set failed : error %x",status);
		while(1);
	}
	/* Start of advertisement */
	status = at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED,
	AT_BLE_ADV_GEN_DISCOVERABLE,
	NULL,
	AT_BLE_ADV_FP_ANY,
	3200,	//advertising interval of 2s (3200*0.625)
	655,
	0);
	if(status != AT_BLE_SUCCESS) {
		printf("\n\r## Advertisement data set failed : error %x",status);
		while(1);
	}
}

/*Callback registered for AT_BLE_CONNECTION_PARAM_UPDATE_REQUEST*/
at_ble_status_t ble_connection_param_update(at_ble_handle_t handle1, at_ble_connection_params_t *connections_params){
	connection_handle = (at_ble_handle_t)(((at_ble_connected_t *)handle1)->handle);
	connections_params -> con_intv_min = CONNECTION_INTERVAL_MIN;
	connections_params -> con_intv_min = CONNECTION_INTERVAL_MAX;
	connections_params -> con_latency = SLAVE_LATENCY;
	return AT_BLE_SUCCESS;
}


/* Callback registered for AT_BLE_DISCONNECTED event */
static at_ble_status_t ble_disconnected_cb (void *param) {
	printf("\nAssignment 3.2: Application disconnected ");
	ble_advertise();
	ALL_UNUSED(param);
	return AT_BLE_SUCCESS;
}

static at_ble_status_t app_htpt_cfg_indntf_ind_handler(void *params) {
	at_ble_htpt_cfg_indntf_ind_t htpt_cfg_indntf_ind_params;
	memcpy((uint8_t *)&htpt_cfg_indntf_ind_params,
	params,
	sizeof(at_ble_htpt_cfg_indntf_ind_t));
	
	if (htpt_cfg_indntf_ind_params.ntf_ind_cfg == 0x03) {
		printf("Started HTP Temperature Notification");
		Temp_Notification_Flag = true;
	}
	else {
		printf("HTP Temperature Notification Stopped");
		Temp_Notification_Flag = false;
	}
	return AT_BLE_SUCCESS;
}

static const ble_event_callback_t app_gap_cb[] = {
	NULL,// AT_BLE_UNDEFINED_EVENT
	NULL,// AT_BLE_SCAN_INFO
	NULL,// AT_BLE_SCAN_REPORT
	NULL,// AT_BLE_ADV_REPORT
	NULL,// AT_BLE_RAND_ADDR_CHANGED
	NULL,// AT_BLE_CONNECTED
	ble_disconnected_cb,// AT_BLE_DISCONNECTED
	NULL,//ble_connection_param_update,// AT_BLE_CONN_PARAM_UPDATE_DONE
	NULL,// AT_BLE_CONN_PARAM_UPDATE_REQUEST
	ble_paired_cb,//PAIR_DONE
	NULL,// AT_BLE_PAIR_REQUEST
	NULL,// AT_BLE_SLAVE_SEC_REQUEST
	NULL,// AT_BLE_PAIR_KEY_REQUEST
	NULL,// AT_BLE_ENCRYPTION_REQUEST
	NULL,// AT_BLE_ENCRYPTION_STATUS_CHANGED
	NULL,// AT_BLE_RESOLV_RAND_ADDR_STATUS
	NULL,// AT_BLE_SIGN_COUNTERS_IND
	NULL,// AT_BLE_PEER_ATT_INFO_IND
	NULL // AT_BLE_CON_CHANNEL_MAP_IND
};

static const ble_event_callback_t app_htpt_handle[] = {
	NULL, // AT_BLE_HTPT_CREATE_DB_CFM
	NULL, // AT_BLE_HTPT_ERROR_IND
	NULL, // AT_BLE_HTPT_DISABLE_IND
	NULL, // AT_BLE_HTPT_TEMP_SEND_CFM
	NULL, // AT_BLE_HTPT_MEAS_INTV_CHG_IND
	app_htpt_cfg_indntf_ind_handler, // AT_BLE_HTPT_CFG_INDNTF_IND
	NULL, // AT_BLE_HTPT_ENABLE_RSP
	NULL, // AT_BLE_HTPT_MEAS_INTV_UPD_RSP
	NULL // AT_BLE_HTPT_MEAS_INTV_CHG_REQ
};


/* Register GAP callbacks at BLE manager level*/
static void register_ble_callbacks (void) { /* Register GAP Callbacks */
	printf("\nAssignment 3.2: Register bluetooth events callbacks");
	status = ble_mgr_events_callback_handler(REGISTER_CALL_BACK,
	BLE_GAP_EVENT_TYPE,
	app_gap_cb);
	if (status != true) {
		printf("\n##Error when Registering SAMB11 gap callbacks");
	}
	
	status = ble_mgr_events_callback_handler(REGISTER_CALL_BACK,
	BLE_GATT_HTPT_EVENT_TYPE,
	app_htpt_handle);
	if (status != true) {
		printf("\n##Error when Registering SAMB11 htpt callbacks");
	}
}

//! [transfer_done_tx]
static void transfer_done_tx(struct dma_resource* const resource )
{
	dma_start_transfer_job(&uart_dma_resource_rx);
}
//! [transfer_done_tx]


//! [transfer_done_rx]
static void transfer_done_rx(struct dma_resource* const resource )
{
	dma_start_transfer_job(&uart_dma_resource_tx);
	integer_part = string[0];		//taking the integer part in string[0]
	fractional_part = string[1];	//taking the fractional part in string[1]
	temp = (float) (integer_part + ((float)((fractional_part)/10)));	//to compute the temperature	
	integer_part_press1 = string[2];
	integer_part_press2 = string[3];
	integer_press = integer_part_press1 * 10 + integer_part_press2;
	fractional_part_press = string[4];
	actual_pressure = (float) (integer_press + ((float)((fractional_part_press)/10)));
	
}
//! [transfer_done_rx]


static void htp_temperature_read(void) {
	//float temperature; /* Read Temperature Value from IO1 Xplained Pro */
	float temperature;
	//temperature = at30tse_read_temperature(); /* Display temperature on com port */
	temperature = temp;
	#ifdef HTPT_FAHRENHEIT
	printf("\nTemperature: %d Fahrenheit", (uint16_t)temperature);
	#else
	printf("\nTemperature: %d Deg Celsius", (uint16_t)temperature);
	#endif
}


/* Sending the temperature value after reading it from IO1 Xplained Pro */
static void htp_temperature_send(void) {
	at_ble_prf_date_time_t timestamp;
	float temperature;
	
	//if character received is 'a', the number received is a negative number
	if (string[5]==97)
	{
		temp=temp*(-1);
	}
	temperature = temp;
	#ifdef HTPT_FAHRENHEIT
	temperature = (((temperature * 9.0)/5.0) + 32.0);
	#endif
	/* Read Temperature Value from IO1 Xplained Pro */
	timestamp.day = 1;
	timestamp.hour = 9;
	timestamp.min = 2;
	timestamp.month = 8;
	timestamp.sec = 36;
	timestamp.year = 15;
	/* Read Temperature Value from IO1 Xplained Pro */
	if(at_ble_htpt_temp_send(convert_ieee754_ieee11073_float((float)temperature),
	&timestamp,
	#ifdef HTPT_FAHRENHEIT
	(at_ble_htpt_temp_flags)(HTPT_FLAG_FAHRENHEIT | HTPT_FLAG_TYPE),
	#else
	(at_ble_htpt_temp_flags)(HTPT_FLAG_CELSIUS | HTPT_FLAG_TYPE),
	#endif
	HTP_TYPE_FINGER,
	1
	) == AT_BLE_SUCCESS)
	{
		#ifdef HTPT_FAHRENHEIT
		printf("\nTemperature: %d Fahrenheit", (uint16_t)temperature);
		#else
		printf("\nTemperature: %d Deg Celsius", (uint16_t)temperature);
		#endif
	}
}


//! [config_dma_resource_tx]
static void configure_dma_resource_tx(struct dma_resource *resource)
{
	//! [setup_tx_1]
	struct dma_resource_config config;
	//! [setup_tx_1]

	//! [setup_tx_2]
	dma_get_config_defaults(&config);
	//! [setup_tx_2]

	//! [setup_tx_3]
	config.des.periph = UART0TX_DMA_PERIPHERAL;
	config.des.enable_inc_addr = false;
	config.src.periph = UART0TX_DMA_PERIPHERAL;
	//! [setup_tx_3]

	//! [setup_tx_4]
	dma_allocate(resource, &config);
	//! [setup_tx_4]
}
//! [config_dma_resource_tx]

//! [setup_dma_transfer_tx_descriptor]
static void setup_transfer_descriptor_tx(struct dma_descriptor *descriptor)
{

	//! [setup_tx_5]
	dma_descriptor_get_config_defaults(descriptor);
	//! [setup_tx_5]

	//! [setup_tx_6]
	descriptor->buffer_size = BUFFER_LEN;
	descriptor->read_start_addr = (uint32_t)string;
	descriptor->write_start_addr =
	(uint32_t)(&uart_instance.hw->TRANSMIT_DATA.reg);
	//! [setup_tx_6]
}
//! [setup_dma_transfer_tx_descriptor]

//! [config_dma_resource_rx]
static void configure_dma_resource_rx(struct dma_resource *resource)
{
	//! [setup_rx_1]
	struct dma_resource_config config;
	//! [setup_rx_1]

	//! [setup_rx_2]
	dma_get_config_defaults(&config);
	//! [setup_rx_2]

	//! [setup_rx_3]
	config.src.periph = UART0RX_DMA_PERIPHERAL;
	config.src.enable_inc_addr = false;
	config.src.periph_delay = 1;
	//! [setup_rx_3]

	//! [setup_rx_4]
	dma_allocate(resource, &config);
	//! [setup_rx_4]
}
//! [config_dma_resource_rx]

//! [setup_dma_transfer_rx_descriptor]
static void setup_transfer_descriptor_rx(struct dma_descriptor *descriptor)
{
	//! [setup_rx_5]
	dma_descriptor_get_config_defaults(descriptor);
	//! [setup_rx_5]

	//! [setup_tx_6]
	descriptor->buffer_size = BUFFER_LEN;
	descriptor->read_start_addr =
	(uint32_t)(&uart_instance.hw->RECEIVE_DATA.reg);
	descriptor->write_start_addr = (uint32_t)string;
	//! [setup_tx_6]
}
//! [setup_dma_transfer_rx_descriptor]

//! [setup_usart]
static void configure_usart(void)
{
	//! [setup_config]
	struct uart_config config_uart;
	//! [setup_config]

	//! [setup_config_defaults]
	uart_get_config_defaults(&config_uart);
	//! [setup_config_defaults]

	//using UART0 pin connections
	//! [setup_change_config]
	config_uart.baud_rate = 9600;
	config_uart.pin_number_pad[0] = EDBG_CDC_SERCOM_PIN_PAD0;
	config_uart.pin_number_pad[1] = EDBG_CDC_SERCOM_PIN_PAD1;
	config_uart.pin_number_pad[2] = EDBG_CDC_SERCOM_PIN_PAD2;
	config_uart.pin_number_pad[3] = EDBG_CDC_SERCOM_PIN_PAD3;
	config_uart.pinmux_sel_pad[0] = EDBG_CDC_SERCOM_MUX_PAD0;
	config_uart.pinmux_sel_pad[1] = EDBG_CDC_SERCOM_MUX_PAD1;
	config_uart.pinmux_sel_pad[2] = EDBG_CDC_SERCOM_MUX_PAD2;
	config_uart.pinmux_sel_pad[3] = EDBG_CDC_SERCOM_MUX_PAD3;
	//! [setup_change_config]

	//! [setup_set_config]
	while (uart_init(&uart_instance,
	EDBG_CDC_MODULE, &config_uart) != STATUS_OK) {
	}
	//! [setup_set_config]

	//! [enable_interrupt]
	uart_enable_transmit_dma(&uart_instance);
	uart_enable_receive_dma(&uart_instance);
	//! [enable_interrupt]
}
//! [setup_usart]

//! [setup_callback]
static void configure_dma_callback(void)
{
	//! [setup_callback_register]
	dma_register_callback(&uart_dma_resource_tx, transfer_done_tx, DMA_CALLBACK_TRANSFER_DONE);
	dma_register_callback(&uart_dma_resource_rx, transfer_done_rx, DMA_CALLBACK_TRANSFER_DONE);
	//! [setup_callback_register]

	//! [setup_enable_callback]
	dma_enable_callback(&uart_dma_resource_tx, DMA_CALLBACK_TRANSFER_DONE);
	dma_enable_callback(&uart_dma_resource_rx, DMA_CALLBACK_TRANSFER_DONE);
	//! [setup_enable_callback]

	//! [enable_inic]
	NVIC_EnableIRQ(PROV_DMA_CTRL0_IRQn);
	//! [enable_inic]
}
//! [setup_callback]

//! [setup]

//setting up the output gpio pin
void configure_gpio_pins(void)
{
	struct gpio_config config_gpio_pin;
	gpio_get_config_defaults(&config_gpio_pin);

	config_gpio_pin.direction = GPIO_PIN_DIR_OUTPUT;
	gpio_pin_set_config(LED_0_PIN, &config_gpio_pin);
}
//! [setup]


int main (void) {
		indication_sent = true;
		notification_sent = true;
		units = APP_DEFAULT_VAL;
		notification_flag = APP_DEFAULT_VAL;
		indication_flag = APP_DEFAULT_VAL;
		user_request_flag =  APP_DEFAULT_VAL;
		timer_count = APP_DEFAULT_VAL;
		notify = 0;
		app_state = 0;
		memset(operator_blp, 1, sizeof(int8_t) * 9);
		systolic_val_mmhg = SYSTOLIC_MIN_MMHG;
		diastolic_val_mmhg = DIASTOLIC_MIN_MMHG;
		map_val_mmhg = MAP_MIN_MMHG;
		systolic_val_kpa = SYSTOLIC_MIN_KPA;
		diastolic_val_kpa = DIASTOLIC_MIN_KPA;
		map_val_kpa = MAP_MIN_KPA;
		pulse_rate_val = PULSE_RATE_MIN;
		interim_diastolic_mmhg = DIASTOLIC_MIN_MMHG;
		interim_diastolic_kpa = DIASTOLIC_MIN_KPA;
		interim_systolic_mmhg = SYSTOLIC_MIN_MMHG;
		interim_systolic_kpa = SYSTOLIC_MIN_KPA;
		interim_map_mmhg = MAP_MIN_MMHG;
		interim_map_kpa = MAP_MIN_KPA;
		app_exec = true;
		isButton = false;
		isTimer = false;
		isIndication = false;
		memset(g_blp_data, 0, sizeof(uint8_t) * BLP_DATA_LEN);
		g_idx = 0;

		app_gap_handle[0] = NULL;
		app_gap_handle[1] = NULL;
		app_gap_handle[2] = NULL;
		app_gap_handle[3] = NULL;
		app_gap_handle[4] = NULL;
		app_gap_handle[5] = app_connected_state_handler;
		app_gap_handle[6] = app_disconnected_state_handler;
		app_gap_handle[7] = NULL;
		app_gap_handle[8] = NULL;
		app_gap_handle[9] = NULL;
		app_gap_handle[10] = NULL;
		app_gap_handle[11] = NULL;
		app_gap_handle[12] = NULL;
		app_gap_handle[13] = NULL;
		app_gap_handle[14] = NULL;
		app_gap_handle[15] = NULL;
		app_gap_handle[16] = NULL;
		app_gap_handle[17] = NULL;
		app_gap_handle[18] = NULL;
		
		app_gatt_server_handle[0] = app_notification_confirmation_handler;
		app_gatt_server_handle[1] = app_indication_confirmation_handler;
		app_gatt_server_handle[2] = NULL;
		app_gatt_server_handle[3] = NULL;
		app_gatt_server_handle[4] = NULL;
		app_gatt_server_handle[5] = NULL;
		app_gatt_server_handle[6] = NULL;
		app_gatt_server_handle[7] = NULL;
		app_gatt_server_handle[8] = NULL;
		app_gatt_server_handle[9] = NULL;
	
	platform_driver_init();
	//acquire_sleep_lock();
		gpio_init();
		button_init();
		button_register_callback(button_cb);
	
	configure_gpio_pins();
	/* Initialize serial console */
	serial_console_init();
	
	/* Hardware timer */
	hw_timer_init();
	
	/* Register the callback */
	hw_timer_register_callback(timer_callback_handler);
	/* Start timer */
	hw_timer_start(1);
	
	printf("\n\rSAMB11 BLE Application");
	
	/* initialize the BLE chip and Set the Device Address */
	ble_device_init(NULL);
		/* Initialize the blood pressure sensor profile */
		blp_sensor_init(NULL);

		/** Initializing the application time stamp*/
		time_stamp_init();
	
	
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	
	system_clock_config(CLOCK_RESOURCE_XO_26_MHZ, CLOCK_FREQ_26_MHZ );
	
	//! [setup_usart]
	configure_usart();
	//! [setup_usart]
	
	//! [setup_dma_resource]
	configure_dma_resource_tx(&uart_dma_resource_tx);
	configure_dma_resource_rx(&uart_dma_resource_rx);
	//! [setup_dma_resource]

	//! [setup_transfer_descriptor]
	setup_transfer_descriptor_tx(&example_descriptor_tx);
	setup_transfer_descriptor_rx(&example_descriptor_rx);
	//! [setup_transfer_descriptor]

	//! [add_descriptor_to_resource]
	dma_add_descriptor(&uart_dma_resource_tx, &example_descriptor_tx);
	dma_add_descriptor(&uart_dma_resource_rx, &example_descriptor_rx);
	//! [add_descriptor_to_resource]

	//! [configure_callback]
	configure_dma_callback();
	//! [configure_callback]
	//! [setup_init]

	//! [main]
		
	//! [main_1]
	dma_start_transfer_job(&uart_dma_resource_rx);
	//! [main_1]
	
	//Changing the power transmit level to -6 dBm
	if((status = at_ble_tx_power_set(TRANSMIT_POWER)) != AT_BLE_SUCCESS) {
		printf("BLE at_ble_tx_power_set failed =0x%x\n", status);
	}
	else {
		printf("Assignment 5: at_ble_tx_power_set = 0x%x\n", TRANSMIT_POWER);
	}
	
	/* Registering the app_notification_handler with the profile */
	register_blp_notification_handler(app_notification_handler);
	
	/* Registering the app_indication_handler with the profile */
	register_blp_indication_handler(app_indication_handler);

	/* Initialize the htp service */
	htp_init();
	/* Register Bluetooth events Callbacks */
	register_ble_callbacks();
	
	/* Triggering advertisement */
	blp_sensor_adv();
	
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK,
	BLE_GAP_EVENT_TYPE,
	app_gap_handle);
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK,
	BLE_GATT_SERVER_EVENT_TYPE,
	app_gatt_server_handle);

	

	/* Start Advertising process */
	ble_advertise();
	htp_temperature_read();
	
	register_ble_callbacks();
		
	while(true) {
		gpio_pin_set_output_level(LED_0_PIN, true);
		ble_event_task(655);
		//dma_start_transfer_job(&uart_dma_resource_rx);
		if (Timer_Flag & Temp_Notification_Flag) {
			htp_temperature_send();
		}
		
		
		if (isTimer == true) {
			if (user_request_flag) {
				timer_count++;
				notify = true;
			}

			update_time_stamp();

			hw_timer_start(TIMER_INTERVAL);

			isTimer = false;
		}
		
		if (isIndication == true) {
			/* Sending the default notification for the first time */
			blp_sensor_send_indication(g_blp_data, g_idx);

			isIndication = false;
		}

	}
	return 0;
}
