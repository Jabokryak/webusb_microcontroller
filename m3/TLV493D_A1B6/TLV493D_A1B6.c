#include "stm32f4xx.h"
#include "TLV493D_A1B6.h"

static inline uint8_t is_even(uint8_t p_out_buf[4]) {
	uint8_t p = p_out_buf[0] ^ p_out_buf[1] ^ p_out_buf[2] ^ p_out_buf[3];
	p = p ^ (p << 4);
	p = p ^ (p << 2);
	p = p ^ (p << 1);

	return ~p & 0x80;
}

HAL_StatusTypeDef TLV493D_A1B6_set_mode(I2C_HandleTypeDef * const p_hi2c, uint8_t const p_addr, uint8_t const p_new_IICAddr_bits) {
	uint8_t l_out_buf[4] = {
		0
		,p_new_IICAddr_bits | TLV493D_A1B6_Interrupt_disable | TLV493D_A1B6_Master_controlled_mode
		,0
		,TLV493D_A1B6_Temperature_measurement_disabled | TLV493D_A1B6_Low_power_period_12_ms| TLV493D_A1B6_Parity_test_enabled
	};

	l_out_buf[1] |= is_even(l_out_buf);		//Parity check, sum of all 32 bits from write registers 0H, 1H, 2H and 3H must be odd

	return HAL_I2C_Master_Transmit(p_hi2c, p_addr, l_out_buf, 4, TLV493D_A1B6_I2C_timeout);
}

void TLV493D_A1B6_recovery(I2C_HandleTypeDef * const p_hi2c) {
	uint8_t l_out_buf[1] = {0};

	HAL_I2C_Master_Receive(p_hi2c, 0xFF, l_out_buf, 0, TLV493D_A1B6_I2C_timeout);
}

void TLV493D_A1B6_reset(I2C_HandleTypeDef * const p_hi2c, uint8_t const p_addr) {
	HAL_I2C_Master_Transmit(p_hi2c, 0, (uint8_t *)&p_addr, 1, TLV493D_A1B6_I2C_timeout);
}
