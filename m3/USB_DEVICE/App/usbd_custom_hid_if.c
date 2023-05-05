/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v1.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */
#include "main.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//static uint8_t l_no_data[3] = {1, 7, 0};		//7 - GET_SENSORS_DATA, 0 - no data
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */
#define wordReportCount(x) ((x) & 0xFF), ((x) >> 8 & 0xFF)
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */
	0x06, 0x00, 0xFF,            // (GLOBAL) USAGE_PAGE         0xFF00 Vendor-defined
	0x09, 0x02,          		 // Local   Usage (vendor usage 1)
	0xA1, 0x01,                  // (MAIN)   COLLECTION         0x01 Application (Usage=0x0: Page=, Usage=, Type=) <-- Warning: USAGE type should be CA (Application)
	0x15, 0x00,                  //   (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0
	0x26, 0xFF, 0x00,            //   (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)
	0x75, 0x08,                  //   (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field

	//From mc to host. Answer consist of sensors data.
	0x85, 0x01,                  //   (GLOBAL) REPORT_ID          0x01 (1)
	0x96, wordReportCount(D_ANSWER_LENGTH - 1),   //   (GLOBAL) REPORT_COUNT	 Number of fields
	0x09, 0x02,                  //   (LOCAL)  USAGE              0xFF000001
	0x81, 0x20,                  //   (MAIN)   INPUT              0x00000002 (71 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap

	//From mc to host. No sensors data.
	0x85, 0x03,                  //   (GLOBAL) REPORT_ID          0x03 (3)
	0x95, 0x00,					 //   (GLOBAL) REPORT_COUNT       0x00 (0) Number of fields
	0x09, 0x02,                  //   (LOCAL)  USAGE              0xFF000001
	0x81, 0x20,                  //   (MAIN)   INPUT              0x00000002 (1 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap

	//From mc to host. Command result: 1 - success, >1 - error code.
	0x85, 0x04,                  //   (GLOBAL) REPORT_ID          0x04 (4)
	0x95, 0x02,					 //   (GLOBAL) REPORT_COUNT       0x02 (2) Number of fields
	0x09, 0x02,                  //   (LOCAL)  USAGE              0xFF000001
	0x81, 0x20,                  //   (MAIN)   INPUT              0x00000002 (3 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap

	//From mc to host. Command result: 1 - success, >1 - error code. Plus some data.
	0x85, 0x06,                  //   (GLOBAL) REPORT_ID          0x06 (6)
	0x96, wordReportCount(D_ANSWER_LENGTH - 1),	//   (GLOBAL) REPORT_COUNT   Number of fields
	0x09, 0x02,                  //   (LOCAL)  USAGE              0xFF000001
	0x81, 0x20,                  //   (MAIN)   INPUT              0x00000002 (71 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap

	//From mc to host. Debug data.
	0x85, 0x07,                  //   (GLOBAL) REPORT_ID          0x06 (6)
	0x96, wordReportCount(D_DEBUG_LENGTH - 1),	//   (GLOBAL) REPORT_COUNT       Number of fields
	0x09, 0x02,                  //   (LOCAL)  USAGE              0xFF000001
	0x81, 0x20,                  //   (MAIN)   INPUT              0x00000002 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap

	//From host to mc. Contains commands and other accompanying information
	0x85, 0x02,                  //   (GLOBAL) REPORT_ID          0x02 (2)
	0x95, 0x3F,                  //   (GLOBAL) REPORT_COUNT       0x3F (63) Number of fields
	0x09, 0x02,                  //   (LOCAL)  USAGE              0xFF000001
	0x91, 0x20,                  //   (MAIN)   OUTPUT             0x00000002 (64 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap

	//From host to mc. Get sensors data
	0x85, 0x05,                  //   (GLOBAL) REPORT_ID          0x05 (5)
	0x95, 0x01,                  //   (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields
	0x09, 0x02,                  //   (LOCAL)  USAGE              0xFF000001
	0x91, 0x01,                  //   (MAIN)   OUTPUT             0x00000002 (2 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
	//0xC0,                        // (MAIN)   END_COLLECTION     Application
  /* USER CODE END 0 */
  0xC0    /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
extern void save_errors_in_report();
extern inline void init_compare_registers_for_sensors_timer();

extern uint8_t l_report[D_ANSWER_LENGTH];
extern t_i2c_ext l_i2c1_ext;
extern t_i2c_ext l_i2c2_ext;
extern t_i2c_ext l_i2c3_ext;
extern t_error l_error;
extern uint8_t l_command;
extern t_sensors_on_points l_sensors_points;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern t_off_on_slots_tick l_slots_tick;
extern uint8_t l_debug[D_DEBUG_LENGTH];
extern uint16_t l_dbg_pos;
extern uint8_t l_leds_off;

static uint8_t l_answer[D_ANSWER_LENGTH];

static uint16_t get_report_length(uint8_t p_report_id) {
	switch (p_report_id) {
		case 1:		return D_ANSWER_LENGTH;
		case 3:		return 1;
		case 4:		return 3;
		case 6:		return D_ANSWER_LENGTH;
		case 7:		return D_DEBUG_LENGTH;

		case 2:		return 64;
		case 5:		return 2;
		default:	return 3;
	}
}

inline static void copy_errors_data() {
	l_answer[0]	= 6;		//Command result plus some data

	l_answer[2]	= 8;		//Error occurs
	l_answer[3] = l_i2c1_ext.state;
	l_answer[4] = l_i2c1_ext.error;
	l_answer[5] = l_i2c2_ext.state;
	l_answer[6] = l_i2c2_ext.error;
	l_answer[7] = l_i2c3_ext.state;
	l_answer[8] = l_i2c3_ext.error;
	l_answer[9] = l_error;
	l_answer[10] = l_sensors_points.point_number;
}

inline static void copy_sensors_data(uint8_t p_reset_i2c_state) {
	/*NVIC_DisableIRQ(I2C1_EV_IRQn);
	NVIC_DisableIRQ(I2C1_ER_IRQn);

	NVIC_DisableIRQ(I2C2_EV_IRQn);
	NVIC_DisableIRQ(I2C2_ER_IRQn);

	NVIC_DisableIRQ(I2C3_EV_IRQn);
	NVIC_DisableIRQ(I2C3_ER_IRQn);*/

	if (	data_ready == l_i2c1_ext.state
		&&	data_ready == l_i2c2_ext.state
		&&	data_ready == l_i2c3_ext.state
		&&	no_error == l_i2c1_ext.error
		&&	no_error == l_i2c2_ext.error
		&&	no_error == l_i2c3_ext.error
		&&	no_error == l_error
	) {
		dbg(121)
		//save_errors_in_report();

		memcpy(l_answer, l_report, D_ANSWER_LENGTH);

		l_answer[0]	= 1;		//Answer consist of sensors data
		l_answer[1] = l_command;
		l_answer[70] = l_sensors_points.point_number;
	} else {
		dbg(122)
		copy_errors_data();
	}

	if (p_reset_i2c_state) {
		l_i2c1_ext.state = l_i2c2_ext.state = l_i2c3_ext.state = init_state;
		l_i2c1_ext.error = l_i2c2_ext.error = l_i2c3_ext.error = l_error = no_error;

		memset(l_report, 0, D_ANSWER_LENGTH);
	}
}

void need_send_data() {
	if (9 == l_command) {
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, l_debug, D_DEBUG_LENGTH);
	} else {
		copy_sensors_data(1);

		//After transfer complete call USBD_CUSTOM_HID_DataIn - rewritten in main.c
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, l_answer, get_report_length(l_answer[0]));
	}
}
/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
	static uint8_t l_res = 1;

	dbg(81)
	dbg(event_idx)

	/* Start next USB packet transfer once data processing is completed */
	USBD_CUSTOM_HID_ReceivePacket(&hUsbDeviceFS);

	switch (event_idx) {
		case 0x02:			//Command
			;
			USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;

			uint8_t *l_command_arr = hhid->Report_buf;
			l_command = l_command_arr[1];

			dbg(l_command)

			/*Command data format
			Common section for all commands:
			0 - USB report ID			= 2 (Host -> Device)
			1 - application report ID	= command index

			Common section for ON`s commands:
			2 - one tick length
			3 - pixels per tick
			4 - number of ticks
			5 - number of LEDs

			ON_LEDS (application report ID = 2) or COMMON_ON (application report ID = 6)
			6 - 7..4 slot number, 0..3 on points array length
			7 - on points at first 8 ticks
			8 - on points at next 8 ticks if number of ticks > 8, else 0

			9 - next slot number
			etc...

			ON_SENSORS (application report ID = 4) or COMMON_ON (application report ID = 6)
			i - number of time points
			i+1 - lsb first time point
			i+2 - msb first time point

			i+3 - lsb next time point
			etc...
			*/

			/*Answer format
			0 - USB report ID (Device -> Host)
			1 - Repeat command index
			2 - Result: 1 - success, >1 - error code
			*/

			l_answer[0] = 4;			//Command result
			l_answer[1] = l_command;

			l_res = 1;			//1 - success, >1 - error code

			switch (l_command) {
				case 2:		//ON_LEDS
				case 4:		//ON_SENSORS
				case 6:		//COMMON_ON
					;
					dbg(2)

					uint16_t l_one_tick_length	= l_command_arr[2] | (uint16_t) l_command_arr[3] << 8 ;
					uint16_t l_pixels_per_tick	= l_command_arr[4] | (uint16_t) l_command_arr[5] << 8 ;
					uint8_t l_number_of_ticks	= l_command_arr[6];
					uint8_t l_number_of_leds	= l_command_arr[7];
					uint8_t i = 8;

					if (	2 == l_command		//ON_LEDS
						||	6 == l_command		//COMMON_ON
					) {
						if (l_number_of_leds > 0) {
							l_slots_tick.number_of_ticks	= l_number_of_ticks;
							l_slots_tick.number_of_leds		= l_number_of_leds;
							l_slots_tick.current_tick		= 0;
							l_leds_off						= 0;

							uint8_t l_prev_turn[l_number_of_leds];
							uint16_t l_on_led_points;
							uint8_t l_slot;
							uint8_t l_turn;

							for (uint8_t j = 0; j < l_number_of_leds; j++, i+=3) {
								l_slot			= l_command_arr[i] - 1;
								l_on_led_points	= l_command_arr[i + 1] | (uint16_t) l_command_arr[i + 2] << 8;

								for (uint8_t k = 0; k < l_number_of_ticks; k++) {
									l_turn = (l_on_led_points & 1U << k) > 0;

									if (0 == j) {
										l_slots_tick.tick[k].on_slots = 0;
										l_slots_tick.tick[k].off_slots = 0;
									}

									if (l_prev_turn[j] != l_turn
										|| 0 == k
									) {
										if (l_turn)
											l_slots_tick.tick[k].on_slots |= (uint16_t) 1 << l_slot;
										else
											l_slots_tick.tick[k].off_slots |= (uint16_t) 1 << l_slot;

										l_prev_turn[j] = l_turn;
									}
								}
							}

							//TIM3->CR1 |= TIM_CR1_UDIS;
							TIM3->ARR = l_one_tick_length;
							TIM3->CNT = 0;
							//htim3.State = HAL_TIM_STATE_BUSY;
							TIM3->SR = 0;
							TIM3->CR1 |= TIM_CR1_CEN;
							//TIM3->CR1 &= ~TIM_CR1_UDIS;
						}
					}

					uint8_t l_number_of_time_points	= l_command_arr[i];

					if (	4 == l_command		//ON_SENSORS
						||	6 == l_command		//COMMON_ON
					) {
						if (l_number_of_time_points > 0) {
							l_i2c1_ext.state = l_i2c2_ext.state = l_i2c3_ext.state = init_state;
							l_i2c1_ext.error = l_i2c2_ext.error = l_i2c3_ext.error = l_error = no_error;

							l_i2c1_ext.i2c_pointer->State = l_i2c2_ext.i2c_pointer->State = l_i2c3_ext.i2c_pointer->State = HAL_I2C_STATE_READY;

							l_sensors_points.number_of_poits = l_number_of_time_points;
							l_sensors_points.point_number = 0;
							i++;

							memcpy(l_sensors_points.on_points_word_arr, &l_command_arr[i], l_number_of_time_points * 2);

							for (uint8_t j = 0; j < l_number_of_time_points; j++) {
								l_sensors_points.on_points_word_arr[j] = (uint32_t) l_sensors_points.on_points_word_arr[j] * l_one_tick_length / l_pixels_per_tick;
							}

							//TIM1->CR1 |= TIM_CR1_UDIS;
							TIM1->ARR = l_number_of_ticks * l_one_tick_length;

							init_compare_registers_for_sensors_timer();

							TIM1->CNT = 0;

							TIM1->DIER = TIM_IT_UPDATE
								| TIM_IT_CC1
								| ((l_sensors_points.number_of_poits >= 2) ? TIM_IT_CC2 : 0)
								| ((l_sensors_points.number_of_poits >= 3) ? TIM_IT_CC3 : 0)
								| ((l_sensors_points.number_of_poits >= 4) ? TIM_IT_CC4 : 0);

							TIM1->CCER = TIM_CCER_CC1E
								| ((l_sensors_points.number_of_poits >= 2) ? TIM_CCER_CC2E : 0)
								| ((l_sensors_points.number_of_poits >= 3) ? TIM_CCER_CC3E : 0)
								| ((l_sensors_points.number_of_poits >= 4) ? TIM_CCER_CC4E : 0);

							//htim1.State = HAL_TIM_STATE_BUSY;
							TIM1->SR = 0;
							TIM1->CR1 |= TIM_CR1_CEN;
							//TIM1->CR1 &= ~TIM_CR1_UDIS;
						}
					}

					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

					//TIM5->CR1 |= TIM_CR1_UDIS;
					TIM5->CNT = 0;
					//htim5.State = HAL_TIM_STATE_BUSY;
					TIM5->SR = 0;
					TIM5->CR1 |= TIM_CR1_CEN;
					//TIM5->CR1 &= ~TIM_CR1_UDIS;

					if (	2 == l_command		//ON_LEDS
						||	6 == l_command		//COMMON_ON
					)
						HAL_TIM_PeriodElapsedCallback(&htim3);

					break;
				case 5:		//COMMON_OFF
					if (TIM5->CR1 & TIM_CR1_CEN) {
						TIM5->CR1 &= ~TIM_CR1_CEN;
						//htim5.State = HAL_TIM_STATE_READY;
					 } else
						l_res = 6;		//LEDs and sensors points timer already stopped
				case 1:		//OFF_LEDS
					 if (l_leds_off) {
						l_res = 5;			//LEDs already stopped
					} else {
						l_leds_off = 1;

						#define slot_off(a) Led##a##_GPIO_Port->BSRR = (uint32_t) Led##a##_Pin << 16U;

						slot_off(10)
						slot_off(9)
						slot_off(8)
						slot_off(7)
						slot_off(6)
						slot_off(5)
						slot_off(4)
						slot_off(3)
						slot_off(2)
						slot_off(1)
					}

					if (1 == l_command)
						break;
				case 3:		//OFF_SENSORS
					if (TIM1->CR1 & TIM_CR1_CEN) {
						TIM1->CR1 &= ~TIM_CR1_CEN;
						//htim1.State = HAL_TIM_STATE_READY;
					} else if (1 == l_res)
						l_res = 7;		//Sensors points timer already stopped

					break;
				case 7:		//GET_SENSORS_ERRORS
					copy_errors_data();

					break;
				case 8:		//GET_SENSORS_DATA
					copy_sensors_data(0);

					l_answer[0]	= 1;		//Answer consist of sensors data
					l_answer[1]	= 8;		//Command

					break;
				case 9:		//GET_DEBUG_DATA
					USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, l_debug, D_DEBUG_LENGTH);

					break;
				default:
					l_res = 8;		//Command not supported
			}

			if (8 != l_command)
				l_answer[2] = l_res;

			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

			//After transfer complete call USBD_CUSTOM_HID_DataIn - rewritten in main.c
			if (9 != l_command) {
				USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, l_answer, get_report_length(l_answer[0]));

				dbg(19)
			}

			break;
		case 0x05:		//Get sensors data
			if (
				(		(l_i2c1_ext.state == data_ready	|| l_i2c1_ext.error != no_error)
					&&	(l_i2c2_ext.state == data_ready	|| l_i2c2_ext.error != no_error)
					&&	(l_i2c3_ext.state == data_ready	|| l_i2c3_ext.error != no_error)
				)
				||	l_error != no_error
			) {
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

				dbg(41)

				need_send_data();
			}

			break;
	}

	return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
/*
static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}
*/
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

