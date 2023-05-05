/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef Lib_for_QMC5883L
#define Lib_for_QMC5883L

#define QMC5883L_Addr							0x1B		//0xD << 1 | 1
#define QMC5883L_I2C_timeout					10

/* Register numbers */
#define QMC5883L_X_LSB							0
#define QMC5883L_X_MSB							1
#define QMC5883L_Y_LSB							2
#define QMC5883L_Y_MSB							3
#define QMC5883L_Z_LSB							4
#define QMC5883L_Z_MSB							5
#define QMC5883L_Status							6
#define QMC5883L_Relative_temperature_LSB		7
#define QMC5883L_Relative_temperature_MSB		8
#define QMC5883L_Control_register				9
#define QMC5883L_Control_register_2				10
#define QMC5883L_Set_reset_period_register		11

#define QMC5883L_Mode_standby					0b00000000
#define QMC5883L_Mode_continuous				0b00000001

#define QMC5883L_Output_data_rate_10_Hz			0b00000000
#define QMC5883L_Output_data_rate_50_Hz			0b00000100
#define QMC5883L_Output_data_rate_100_Hz		0b00001000
#define QMC5883L_Output_data_rate_200_Hz		0b00001100

#define QMC5883L_Measurement_range_2_G			0b00000000
#define QMC5883L_Measurement_range_8_G			0b00010000

#define QMC5883L_Oversample_rate_512			0b00000000
#define QMC5883L_Oversample_rate_256			0b01000000
#define QMC5883L_Oversample_rate_128			0b10000000
#define QMC5883L_Oversample_rate_64				0b11000000

#define QMC5883L_Interrupt_enable				0b00000000
#define QMC5883L_Interrupt_disable				0b00000001
#define QMC5883L_Rollover_pointer_enable		0b01000000
#define QMC5883L_Soft_reset						0b10000000

#define QMC5883L_CCMRAM							__attribute__((section(".ccmram")))
#define QMC5883L_CCMIDATA						__attribute__((section(".ccmidata")))

static uint8_t p_read_init_buf[1] = {0};

QMC5883L_CCMIDATA static uint8_t p_continuous_mode_buf[2] = {
	QMC5883L_Control_register
	,QMC5883L_Mode_continuous | QMC5883L_Output_data_rate_200_Hz | QMC5883L_Measurement_range_2_G | QMC5883L_Oversample_rate_64
};

QMC5883L_CCMIDATA static uint8_t p_standby_mode_buf[2] = {
	QMC5883L_Control_register
	,QMC5883L_Mode_standby | QMC5883L_Output_data_rate_200_Hz | QMC5883L_Measurement_range_2_G | QMC5883L_Oversample_rate_64
};

HAL_StatusTypeDef QMC5883L_soft_reset(I2C_HandleTypeDef * const hi2c);
HAL_StatusTypeDef QMC5883L_init(I2C_HandleTypeDef * const hi2c);
HAL_StatusTypeDef QMC5883L_set_mode(I2C_HandleTypeDef * const hi2c, uint8_t const mode, uint8_t const odr, uint8_t const rng, uint8_t const osr);

static inline HAL_StatusTypeDef QMC5883L_set_continuous_mode(I2C_HandleTypeDef * const hi2c){
	return HAL_I2C_Master_Transmit_IT(hi2c, QMC5883L_Addr, p_continuous_mode_buf, 2);
}

static inline HAL_StatusTypeDef QMC5883L_set_standby_mode(I2C_HandleTypeDef * const hi2c){
	return HAL_I2C_Master_Transmit_IT(hi2c, QMC5883L_Addr, p_standby_mode_buf, 2);
}


static inline HAL_StatusTypeDef QMC5883L_read_init(I2C_HandleTypeDef * const hi2c){
	return HAL_I2C_Master_Transmit_IT(hi2c, QMC5883L_Addr, p_read_init_buf, 1);
}

static inline HAL_StatusTypeDef QMC5883L_read(I2C_HandleTypeDef * const hi2c, uint8_t * const p_in_buf){
	return HAL_I2C_Mem_Read_IT(hi2c, QMC5883L_Addr, 0, 1, p_in_buf, 7);
}

#endif  //Lib_for_QMC5883L
