#include "stm32f4xx.h"
#include "QMC5883L.h"

static void QMC5883L_init_variables_in_ccmram() {
	p_read_init_buf[0] = 0;

	p_continuous_mode_buf[0] = QMC5883L_Control_register;
	p_continuous_mode_buf[1] = QMC5883L_Mode_continuous | QMC5883L_Output_data_rate_200_Hz | QMC5883L_Measurement_range_2_G | QMC5883L_Oversample_rate_64;

	p_standby_mode_buf[0] = QMC5883L_Control_register;
	p_standby_mode_buf[1] = QMC5883L_Mode_standby | QMC5883L_Output_data_rate_200_Hz | QMC5883L_Measurement_range_2_G | QMC5883L_Oversample_rate_64;
}

HAL_StatusTypeDef QMC5883L_write_reg(I2C_HandleTypeDef * const hi2c, uint8_t const p_reg_addr, uint8_t const p_val){
	uint8_t const l_out_buf[2] = {p_reg_addr, p_val};

	return HAL_I2C_Master_Transmit(hi2c, QMC5883L_Addr, (uint8_t *)l_out_buf, 2, QMC5883L_I2C_timeout);
}

HAL_StatusTypeDef QMC5883L_soft_reset(I2C_HandleTypeDef * const hi2c){
	return QMC5883L_write_reg(hi2c, QMC5883L_Control_register_2, QMC5883L_Soft_reset);
}

HAL_StatusTypeDef QMC5883L_init(I2C_HandleTypeDef * const hi2c) {
	QMC5883L_init_variables_in_ccmram();

	HAL_StatusTypeDef l_ret;
	//static uint8_t l_out_buf[1] = {0};

	//Define Set/Reset period
	l_ret = QMC5883L_write_reg(hi2c, QMC5883L_Set_reset_period_register, 0x1);

	if (l_ret != HAL_OK)
		return l_ret;

	//Interrupt and roll-over pointer enable
	l_ret = QMC5883L_write_reg(hi2c, QMC5883L_Control_register_2, QMC5883L_Interrupt_enable | QMC5883L_Rollover_pointer_enable);

	if (l_ret != HAL_OK)
		return l_ret;

	l_ret = QMC5883L_write_reg(hi2c, QMC5883L_Control_register, QMC5883L_Mode_standby | QMC5883L_Output_data_rate_200_Hz | QMC5883L_Measurement_range_2_G | QMC5883L_Oversample_rate_64);

	if (l_ret != HAL_OK)
		return l_ret;

	//Init read start with 0 byte
	//l_ret = HAL_I2C_Master_Transmit(hi2c, QMC5883L_Addr, l_out_buf, 1, QMC5883L_I2C_timeout);

	return l_ret;
}

HAL_StatusTypeDef QMC5883L_set_mode(I2C_HandleTypeDef * const hi2c, uint8_t const mode, uint8_t const odr, uint8_t const rng, uint8_t const osr){
	return QMC5883L_write_reg(hi2c, QMC5883L_Control_register, mode | odr | rng | osr);
}
