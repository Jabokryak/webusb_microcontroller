/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	init_state
	,set_continuous_mode_QMC5883L
	,start_timer_QMC5883L
	,read_QMC5883L
	,set_standby_mode_QMC5883L
	,preread_first_TLV493D_A1B6
	,preread_second_TLV493D_A1B6
	,start_timer_TLV493D_A1B6
	,read_first_TLV493D_A1B6
	,read_second_TLV493D_A1B6
	,data_ready
} t_state_i2c;

typedef enum {
	no_error
	,not_all_state_is_init_state
	,set_continuous_mode_QMC5883L_error
	,read_QMC5883L_error
	,set_standby_mode_QMC5883L_error
	,preread_first_TLV493D_A1B6_error
	,preread_second_TLV493D_A1B6_error
	,read_first_TLV493D_A1B6_error
	,read_second_TLV493D_A1B6_error
	,unsupported_state_in_do_next
	,i2c_error
	,unsupported_state_after_i2c_transmit
	,unsupported_state_after_i2c_receive
	,unknown_timer
	,init_QMC5883L_error
	,soft_reset_QMC5883L_error
	,set_mode_first_TLV493D_A1B6_error
	,set_mode_second_TLV493D_A1B6_error
	,QMC5883L_status_register_not_one
} t_error;

typedef struct {
	I2C_HandleTypeDef		*i2c_pointer;
	volatile t_state_i2c	state;
	/*TIM_HandleTypeDef*/
	TIM_TypeDef				*timer;
	uint8_t					*QMC5883L_array_pointer;
	uint8_t					*first_TLV493D_A1B6_array_pointer;
	uint8_t					first_TLV493D_A1B6_address;
	uint8_t					*second_TLV493D_A1B6_array_pointer;
	uint8_t					second_TLV493D_A1B6_address;
	volatile t_error		error;
} t_i2c_ext;

typedef struct {
	uint8_t			number_of_poits;		//Число точек включения датчиков в одном цикле
	uint8_t			current_point_index;	//�?ндекс, хранящий значение какой компаратор счётчика активирован
	uint8_t			point_number;			//Порядковый номер текущей точки включения датчиков
	union {									//Массив с числами микросекунд от начала периода, по истечении которых должен быть выполнен опрос датчиков
		uint16_t			on_points_word_arr[16];
		uint8_t				on_points_byte_arr[32];
	};
} t_sensors_on_points;

#define D_MAX_TICK_NUMBER				16U		//Максимальная длинна массива, элементы которого структуры с битами необходимости включения или выключения слота светодиода в каждом интервале (тике)

typedef struct {
	uint8_t	number_of_ticks;
	uint8_t	number_of_leds;
	int8_t	current_tick;
	struct {
		uint16_t	off_slots;
		uint16_t	on_slots;
	} tick[D_MAX_TICK_NUMBER];
} t_off_on_slots_tick;

#define dbg(l_anchor) //{if (l_dbg_pos >= D_DEBUG_LENGTH) l_dbg_pos = D_DEBUG_START_POSITION; l_debug[l_dbg_pos++] = l_anchor;}
//#define dbg(l_anchor) //{if (l_dbg_pos < D_DEBUG_LENGTH) l_debug[l_dbg_pos++] = l_anchor;}

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void error(t_i2c_ext * p_i2c_ext, t_error p_err);
void save_errors_in_report();

extern t_sensors_on_points l_sensors_points;

inline void init_compare_registers_for_sensors_timer() {
	TIM1->CCR1 = l_sensors_points.on_points_word_arr[0];	//Set the Capture Compare Register value

	l_sensors_points.current_point_index = 1;

	if (l_sensors_points.number_of_poits >= 2) {
		TIM1->CCR2 = l_sensors_points.on_points_word_arr[1];

		l_sensors_points.current_point_index++;
	}

	if (l_sensors_points.number_of_poits >= 3) {
		TIM1->CCR3 = l_sensors_points.on_points_word_arr[2];

		l_sensors_points.current_point_index++;
	}

	if (l_sensors_points.number_of_poits >= 4) {
		TIM1->CCR4 = l_sensors_points.on_points_word_arr[3];

		l_sensors_points.current_point_index++;
	}

	//__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE | TIM_FLAG_COM | TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4);
}

void do_read();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define Led1_Pin GPIO_PIN_1
#define Led1_GPIO_Port GPIOC
#define Led2_Pin GPIO_PIN_2
#define Led2_GPIO_Port GPIOC
#define Led3_Pin GPIO_PIN_3
#define Led3_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define Led4_Pin GPIO_PIN_1
#define Led4_GPIO_Port GPIOA
#define Led5_Pin GPIO_PIN_2
#define Led5_GPIO_Port GPIOA
#define Led6_Pin GPIO_PIN_3
#define Led6_GPIO_Port GPIOA
#define Led7_Pin GPIO_PIN_4
#define Led7_GPIO_Port GPIOA
#define Led8_Pin GPIO_PIN_5
#define Led8_GPIO_Port GPIOA
#define Led9_Pin GPIO_PIN_6
#define Led9_GPIO_Port GPIOA
#define Led10_Pin GPIO_PIN_7
#define Led10_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define TLV493D_A1B6_Power_2_Pin GPIO_PIN_15
#define TLV493D_A1B6_Power_2_GPIO_Port GPIOE
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define Blue_led_Pin GPIO_PIN_9
#define Blue_led_GPIO_Port GPIOD
#define Red_led_Pin GPIO_PIN_11
#define Red_led_GPIO_Port GPIOD
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define TLV493D_A1B6_Power_3_Pin GPIO_PIN_8
#define TLV493D_A1B6_Power_3_GPIO_Port GPIOC
#define I2C3_SDA_Pin GPIO_PIN_9
#define I2C3_SDA_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define TLV493D_A1B6_Power_1_Pin GPIO_PIN_7
#define TLV493D_A1B6_Power_1_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_9
#define I2C1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define CUSTOM_HID_EPIN_SIZE			0x40U
#define CUSTOM_HID_EPOUT_SIZE			0x40U

#define D_ANSWER_LENGTH					71U		//Максимальная длина массива, содержащего в т.ч. данные с датчиков, для отправки в мк
#define D_DEBUG_LENGTH					5*64U	//Длина массива с отладочной информацией
#define D_DEBUG_START_POSITION			4U		//�?ндекс массива с отладочной информацией для начала записи данных в массив
#define CCMRAM							__attribute__((section(".ccmram")))
#define CCMIDATA						__attribute__((section(".ccmidata")))
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
