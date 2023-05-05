/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "usbd_customhid.h"
#include "QMC5883L.h"
#include "TLV493D_A1B6.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim13;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CCMRAM static RTC_TimeTypeDef l_time;
CCMRAM static RTC_DateTypeDef l_date;

CCMRAM uint8_t l_report[D_ANSWER_LENGTH];
CCMIDATA uint8_t l_debug[D_DEBUG_LENGTH] = {7, 9};
CCMIDATA uint16_t l_dbg_pos		= D_DEBUG_START_POSITION;
CCMIDATA t_error l_error		= no_error;
CCMRAM uint8_t l_command		= 0;
CCMRAM t_sensors_on_points l_sensors_points;
CCMRAM t_off_on_slots_tick l_slots_tick;
CCMRAM uint8_t l_leds_off;

#define D_WAIT_MEASURE_QMC5883L 1050;
#define D_WAIT_MEASURE_TLV493D_A1B6 104;

#define i2c_dbg(x, y)					\
	dbg(x)								\
	dbg((y == &l_i2c1_ext) ? 1			\
		: (y == &l_i2c2_ext) ? 2		\
		: (y == &l_i2c3_ext) ? 3		\
		: 0								\
	)									\
	dbg(y->state)

#define get_i2c_ext_dbg(x, y)						\
	t_i2c_ext * const y =						\
			(p_hi2c == &hi2c1) ? &l_i2c1_ext	\
		:	(p_hi2c == &hi2c2) ? &l_i2c2_ext	\
		:	&l_i2c3_ext;						\
	i2c_dbg(x, y)

extern void need_send_data();

//Magnet sensors data array l_report:
//0 - USB report ID = 1 - MC output data
//1 - application report ID = 20 magnet sensors data
//2 .. 9 - time and day
//I2C1 data:
//10 .. 16 - QMC5883L #1 data
//17 .. 23 - TLV493D-A1B6 #1 data
//24 .. 30 - TLV493D-A1B6 #2 data
//31	   - error
//I2C2 data:
//32 .. 38 - QMC5883L #2 data
//39 .. 45 - TLV493D-A1B6 #3 data
//46 .. 52 - TLV493D-A1B6 #4 data
//53	   - error
//I2C3 data:
//54 .. 60 - TLV493D-A1B6 #5 data
//61 .. 67 - TLV493D-A1B6 #6 data
//68	   - error
//69	   - global error
//70	   - current sensors on point number
								//i2c ref,	state,		timer, QMC5883L data begin,		 first TLV493D-A1B6 data begin and i2c address,		  second TLV493D-A1B6 data begin and i2c address,		error
CCMIDATA t_i2c_ext l_i2c1_ext = {&hi2c1,	init_state,		0, (uint8_t *)&l_report[10], (uint8_t *)&l_report[17], TLV493D_A1B6_Slave_3_addr, (uint8_t *)&l_report[24], TLV493D_A1B6_Slave_7_addr,	no_error};
CCMIDATA t_i2c_ext l_i2c2_ext = {&hi2c2,	init_state,		0, (uint8_t *)&l_report[32], (uint8_t *)&l_report[39], TLV493D_A1B6_Slave_3_addr, (uint8_t *)&l_report[46], TLV493D_A1B6_Slave_7_addr,	no_error};
CCMIDATA t_i2c_ext l_i2c3_ext = {&hi2c3,	init_state,		0,						  0, (uint8_t *)&l_report[54], TLV493D_A1B6_Slave_3_addr, (uint8_t *)&l_report[61], TLV493D_A1B6_Slave_7_addr,	no_error};

extern USBD_ClassTypeDef  USBD_CUSTOM_HID;

void error(t_i2c_ext * const p_i2c_ext, t_error const p_err) {
	TIM5->CR1 &= ~TIM_CR1_CEN;
	TIM5->SR = 0;

	TIM1->CR1 &= ~TIM_CR1_CEN;
	TIM1->SR = 0;

	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM3->SR = 0;

	l_i2c1_ext.timer->CR1 &= ~TIM_CR1_CEN;
	l_i2c1_ext.timer->SR = 0;

	l_i2c2_ext.timer->CR1 &= ~TIM_CR1_CEN;
	l_i2c2_ext.timer->SR = 0;

	l_i2c3_ext.timer->CR1 &= ~TIM_CR1_CEN;
	l_i2c3_ext.timer->SR = 0;

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

	if (p_i2c_ext == 0) {
		if (no_error == l_error)
			l_error = p_err;
	} else {
		p_i2c_ext->error = p_err;
	}

	dbg(88)
	dbg(p_err)

	need_send_data();
}

static void init_variables_in_ccmram() {
	l_debug[0]		= 7;
	l_debug[1]		= 9;
	l_dbg_pos		= D_DEBUG_START_POSITION;
	l_error			= no_error;
	l_command		= 0;

	l_i2c1_ext = (t_i2c_ext){&hi2c1,	init_state,	htim6.Instance, (uint8_t *)&l_report[10], (uint8_t *)&l_report[17], TLV493D_A1B6_Slave_3_addr, (uint8_t *)&l_report[24], TLV493D_A1B6_Slave_7_addr,	no_error};
	l_i2c2_ext = (t_i2c_ext){&hi2c2,	init_state,	htim7.Instance, (uint8_t *)&l_report[32], (uint8_t *)&l_report[39], TLV493D_A1B6_Slave_3_addr, (uint8_t *)&l_report[46], TLV493D_A1B6_Slave_7_addr,	no_error};
	l_i2c3_ext = (t_i2c_ext){&hi2c3,	init_state,	htim13.Instance,                       0, (uint8_t *)&l_report[54], TLV493D_A1B6_Slave_3_addr, (uint8_t *)&l_report[61], TLV493D_A1B6_Slave_7_addr,	no_error};
}

//Copy from usbd_customhid.c and added user code
static uint8_t USBD_CUSTOM_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(epnum);

  /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_CUSTOM_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId])->state = CUSTOM_HID_IDLE;

	//user code begin
	if (	9 == l_command		//GET_DEBUG_DATA
	){
		memset(l_debug, 0, D_DEBUG_LENGTH);
		l_dbg_pos	= D_DEBUG_START_POSITION;
		l_debug[0]	= 7;
		l_debug[1]	= 9;
	}
	//user code end

  return (uint8_t)USBD_OK;
}

void save_errors_in_report() {
	l_report[31] = l_i2c1_ext.error;
	l_report[53] = l_i2c2_ext.error;
	l_report[68] = l_i2c3_ext.error;
	l_report[69] = l_error;
}

void static inline get_time() {
	HAL_RTC_GetTime(&hrtc, &l_time, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &l_date, RTC_FORMAT_BCD);

	l_report[2] = l_time.Hours;
	l_report[3] = l_time.Minutes;
	l_report[4] = l_time.Seconds;
	l_report[5] = l_time.SubSeconds >> 8;
	l_report[6] = l_time.SubSeconds;
	l_report[7] = l_time.SecondFraction >> 8;
	l_report[8] = l_time.SecondFraction;
	l_report[9] = l_date.Date;
}

void do_next(t_i2c_ext * const p_i2c_ext) {
	i2c_dbg(22, p_i2c_ext)

	I2C_HandleTypeDef * const l_hi2c = p_i2c_ext->i2c_pointer;

	switch (p_i2c_ext->state) {
		case init_state:
			if (QMC5883L_set_continuous_mode(l_hi2c) == HAL_OK)
				p_i2c_ext->state = set_continuous_mode_QMC5883L;
			else
				error(p_i2c_ext, set_continuous_mode_QMC5883L_error);

			break;
		case set_continuous_mode_QMC5883L:
			p_i2c_ext->timer->ARR = D_WAIT_MEASURE_QMC5883L;
			p_i2c_ext->timer->CR1 |= TIM_CR1_CEN;

			p_i2c_ext->state = start_timer_QMC5883L;

			break;
		case start_timer_QMC5883L:
			if (QMC5883L_read(l_hi2c, p_i2c_ext->QMC5883L_array_pointer) == HAL_OK)			//get magnetic induction
				p_i2c_ext->state = read_QMC5883L;
			else
				error(p_i2c_ext, read_QMC5883L_error);

			break;
		case read_QMC5883L:
			if (QMC5883L_set_standby_mode(l_hi2c) == HAL_OK)
				p_i2c_ext->state = set_standby_mode_QMC5883L;
			else
				error(p_i2c_ext, set_standby_mode_QMC5883L_error);

			break;
		case set_standby_mode_QMC5883L:
			if (TLV493D_A1B6_preread(l_hi2c, p_i2c_ext->first_TLV493D_A1B6_address) == HAL_OK)
				p_i2c_ext->state = preread_first_TLV493D_A1B6;
			else
				error(p_i2c_ext, preread_first_TLV493D_A1B6_error);

			break;
		case preread_first_TLV493D_A1B6:
			if (TLV493D_A1B6_preread(l_hi2c, p_i2c_ext->second_TLV493D_A1B6_address) == HAL_OK)
				p_i2c_ext->state = preread_second_TLV493D_A1B6;
			else
				error(p_i2c_ext, preread_second_TLV493D_A1B6_error);

			break;
		case preread_second_TLV493D_A1B6:
			p_i2c_ext->timer->ARR = D_WAIT_MEASURE_TLV493D_A1B6;
			p_i2c_ext->timer->CR1 |= TIM_CR1_CEN;

			p_i2c_ext->state = start_timer_TLV493D_A1B6;

			break;
		case start_timer_TLV493D_A1B6:
			if (TLV493D_A1B6_read(l_hi2c, p_i2c_ext->first_TLV493D_A1B6_address, p_i2c_ext->first_TLV493D_A1B6_array_pointer) == HAL_OK)
				p_i2c_ext->state = read_first_TLV493D_A1B6;
			else
				error(p_i2c_ext, read_first_TLV493D_A1B6_error);

			break;
		case read_first_TLV493D_A1B6:
			if (TLV493D_A1B6_read(l_hi2c, p_i2c_ext->second_TLV493D_A1B6_address, p_i2c_ext->second_TLV493D_A1B6_array_pointer) == HAL_OK)
				p_i2c_ext->state = read_second_TLV493D_A1B6;
			else
				error(p_i2c_ext, read_second_TLV493D_A1B6_error);

			break;
		case read_second_TLV493D_A1B6:
			p_i2c_ext->state = data_ready;

			if (	data_ready == l_i2c1_ext.state
				&&	data_ready == l_i2c2_ext.state
				&&	data_ready == l_i2c3_ext.state
			)
				need_send_data();

			break;
		default:
			error(p_i2c_ext, unsupported_state_in_do_next);
	}

}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef * p_hi2c) {
	if (HAL_I2C_GetError(p_hi2c) != HAL_I2C_ERROR_AF) {
		get_i2c_ext_dbg(87, l_i2c_ext)

		error(l_i2c_ext, i2c_error);

		Error_Handler();
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef * p_hi2c) {
	get_i2c_ext_dbg(30, l_i2c_ext)

	switch (l_i2c_ext->state) {
		case set_continuous_mode_QMC5883L:
		case set_standby_mode_QMC5883L:
			do_next(l_i2c_ext);

			break;
		default:
			error(l_i2c_ext, unsupported_state_after_i2c_transmit);
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * p_hi2c) {
	HAL_I2C_MasterRxCpltCallback(p_hi2c);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef * p_hi2c) {	// I2C data ready!
	get_i2c_ext_dbg(40, l_i2c_ext)

	switch (l_i2c_ext->state) {
		case read_QMC5883L:
		case preread_first_TLV493D_A1B6:
		case preread_second_TLV493D_A1B6:
		case read_first_TLV493D_A1B6:
		case read_second_TLV493D_A1B6:
			do_next(l_i2c_ext);

			break;
		default:
			error(l_i2c_ext, unsupported_state_after_i2c_receive);
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * p_htim) {
	dbg(50)
	dbg(p_htim ->Channel)

	#define set_ccr(n) TIM1->CCR##n = l_sensors_points.on_points_word_arr[l_sensors_points.current_point_index]; break;

	if (l_sensors_points.current_point_index < l_sensors_points.number_of_poits) {
		switch (l_sensors_points.current_point_index % 4) {
			case 0: set_ccr(1)
			case 1: set_ccr(2)
			case 2: set_ccr(3)
			case 3: set_ccr(4)
		}

		l_sensors_points.current_point_index++;
	}

	l_sensors_points.point_number++;

	do_read();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *p_htim) {
	static uint16_t	l_slots;

	if (p_htim == &htim5) {
		return;

	} else if (p_htim == &htim1) {
		dbg(60)

		l_sensors_points.point_number = 0;

		if (l_sensors_points.number_of_poits > 4)
			init_compare_registers_for_sensors_timer();

		return;
	} else if (p_htim == &htim3) {
		dbg(61)

		if (l_leds_off)
			return;

		#define slot_reset(a)								\
			if (l_slots & 1U << (a - 1))					\
				Led##a##_GPIO_Port->BSRR = (uint32_t) Led##a##_Pin << 16U;

		#define slot_set(a)									\
			if (l_slots & 1U << (a - 1))					\
				Led##a##_GPIO_Port->BSRR = Led##a##_Pin;

		l_slots	= l_slots_tick.tick[l_slots_tick.current_tick].off_slots;

		slot_reset(10)
		slot_reset(9)
		slot_reset(8)
		slot_reset(7)
		slot_reset(6)
		slot_reset(5)
		slot_reset(4)
		slot_reset(3)
		slot_reset(2)
		slot_reset(1)

		l_slots	= l_slots_tick.tick[l_slots_tick.current_tick].on_slots;

		slot_set(10)
		slot_set(9)
		slot_set(8)
		slot_set(7)
		slot_set(6)
		slot_set(5)
		slot_set(4)
		slot_set(3)
		slot_set(2)
		slot_set(1)

		l_slots_tick.current_tick++;

		if (l_slots_tick.current_tick >= l_slots_tick.number_of_ticks)
			l_slots_tick.current_tick = 0;

		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

		return;
	}

	TIM_TypeDef * const l_timer = p_htim->Instance;
	t_i2c_ext * const l_i2c_ext =
			(l_timer == l_i2c1_ext.timer) ? &l_i2c1_ext
		:	(l_timer == l_i2c2_ext.timer) ? &l_i2c2_ext
		:	(l_timer == l_i2c3_ext.timer) ? &l_i2c3_ext
		:	0;

	if (0 == l_i2c_ext) {
		error(0, unknown_timer);

		return;
	}

	i2c_dbg(62, l_i2c_ext)

	do_next(l_i2c_ext);
}

void do_read() {
	if (	l_i2c1_ext.state != init_state
		||	l_i2c2_ext.state != init_state
		||	l_i2c3_ext.state != init_state
	) {
		error(0, not_all_state_is_init_state);
	} else {
		do_next(&l_i2c1_ext);
		do_next(&l_i2c2_ext);

		l_i2c3_ext.state = set_standby_mode_QMC5883L;
		do_next(&l_i2c3_ext);

		//get_time();		//get time from RTC
	}
}

void QMC5883L_soft_reset_and_init(t_i2c_ext * p_i2c_ext) {
	if (QMC5883L_soft_reset(p_i2c_ext->i2c_pointer) == HAL_OK) {
		if (QMC5883L_init(p_i2c_ext->i2c_pointer) != HAL_OK)
			error(p_i2c_ext, init_QMC5883L_error);
 	} else
		error(p_i2c_ext, soft_reset_QMC5883L_error);
}

void TLV493D_A1B6_init(t_i2c_ext * p_i2c_ext) {
	if (TLV493D_A1B6_set_mode(p_i2c_ext->i2c_pointer, p_i2c_ext->first_TLV493D_A1B6_address, 0) != HAL_OK) {
		error(p_i2c_ext, set_mode_first_TLV493D_A1B6_error);

		return;
	}

	if (TLV493D_A1B6_set_mode(p_i2c_ext->i2c_pointer, p_i2c_ext->second_TLV493D_A1B6_address, 0) != HAL_OK) {
		error(p_i2c_ext, set_mode_second_TLV493D_A1B6_error);

		return;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	//Init_OnBoard_LEDs(); // calls LEDs GPIO pins initialization function
	//configure_Button(); // call Push button GPIO pins initialization function
	//GPIO_PinState state; // Define a enum struct which contain boolean states
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  	init_variables_in_ccmram();

  	TIM5->SR = 0;
	//TIM5->DIER = TIM_IT_UPDATE;

	TIM3->SR = 0;
	TIM3->DIER = TIM_IT_UPDATE;

	TIM6->SR = 0;
	TIM6->DIER = TIM_IT_UPDATE;

	TIM7->SR = 0;
	TIM7->DIER = TIM_IT_UPDATE;

	TIM13->SR = 0;
	TIM13->DIER = TIM_IT_UPDATE;

	USBD_CUSTOM_HID.DataIn = USBD_CUSTOM_HID_DataIn;

	TLV493D_A1B6_init(&l_i2c1_ext);
	TLV493D_A1B6_init(&l_i2c2_ext);
	TLV493D_A1B6_init(&l_i2c3_ext);

	QMC5883L_soft_reset_and_init(&l_i2c1_ext);
	QMC5883L_soft_reset_and_init(&l_i2c2_ext);

	HAL_Delay(1);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	//do_read();

	//HAL_GPIO_WritePin(Led1_GPIO_Port, Led1_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(Led6_GPIO_Port, Led6_Pin, GPIO_PIN_SET);

	/*if (HAL_I2C_Master_Receive(&hi2c2, l_i2c2_ext.first_TLV493D_A1B6_address, l_i2c2_ext.first_TLV493D_A1B6_array_pointer, 7, TLV493D_A1B6_I2C_timeout) != HAL_OK)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //HAL_PWR_EnableSleepOnExit();
    //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//dbg(80);
	//HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = I2C1_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(I2C1_SDA_GPIO_Port, &GPIO_InitStruct);
  }

  HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TLV493D_A1B6_Power_1_GPIO_Port, TLV493D_A1B6_Power_1_Pin, GPIO_PIN_SET);

  HAL_Delay(1);

  HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET);
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */
  {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = I2C2_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(I2C2_SDA_GPIO_Port, &GPIO_InitStruct);
  }

  HAL_GPIO_WritePin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TLV493D_A1B6_Power_2_GPIO_Port, TLV493D_A1B6_Power_2_Pin, GPIO_PIN_SET);

  HAL_Delay(1);

  HAL_GPIO_WritePin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_SET);
  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */
  {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = I2C3_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(I2C3_SDA_GPIO_Port, &GPIO_InitStruct);
  }

  HAL_GPIO_WritePin(I2C3_SDA_GPIO_Port, I2C3_SDA_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TLV493D_A1B6_Power_3_GPIO_Port, TLV493D_A1B6_Power_3_Pin, GPIO_PIN_SET);

  HAL_Delay(1);

  HAL_GPIO_WritePin(I2C3_SDA_GPIO_Port, I2C3_SDA_Pin, GPIO_PIN_SET);
  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x1;
  sTime.Minutes = 0x18;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x27;
  sDate.Year = 0x22;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  HAL_Delay(1);			//Otherwise, slave timers often start immediately
  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 104;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim6, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 104;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim7, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 83;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 104;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim13, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Led1_Pin|Led2_Pin|Led3_Pin|TLV493D_A1B6_Power_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Led4_Pin|Led5_Pin|Led6_Pin|Led7_Pin
                          |Led8_Pin|Led9_Pin|Led10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TLV493D_A1B6_Power_2_GPIO_Port, TLV493D_A1B6_Power_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|TLV493D_A1B6_Power_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Blue_led_Pin|Red_led_Pin|LD4_Pin|LD3_Pin
                          |LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin Led1_Pin Led2_Pin Led3_Pin
                           TLV493D_A1B6_Power_3_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|Led1_Pin|Led2_Pin|Led3_Pin
                          |TLV493D_A1B6_Power_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Led4_Pin Led5_Pin Led6_Pin Led7_Pin
                           Led8_Pin Led9_Pin Led10_Pin */
  GPIO_InitStruct.Pin = Led4_Pin|Led5_Pin|Led6_Pin|Led7_Pin
                          |Led8_Pin|Led9_Pin|Led10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TLV493D_A1B6_Power_2_Pin */
  GPIO_InitStruct.Pin = TLV493D_A1B6_Power_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TLV493D_A1B6_Power_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 TLV493D_A1B6_Power_1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14|TLV493D_A1B6_Power_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Blue_led_Pin Red_led_Pin */
  GPIO_InitStruct.Pin = Blue_led_Pin|Red_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	  dbg(75)
	  HAL_Delay(300);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
