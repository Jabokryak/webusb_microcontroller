/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef Lib_for_TLV493D_A1B6
#define Lib_for_TLV493D_A1B6

//Need high SDA pin at power-up
#define TLV493D_A1B6_Slave_0_addr				0x95
#define TLV493D_A1B6_Slave_0_IICAddr_bits		0x60			//11
#define TLV493D_A1B6_Slave_1_addr				0x9D
#define TLV493D_A1B6_Slave_1_IICAddr_bits		0x40			//10
#define TLV493D_A1B6_Slave_2_addr				0xB5
#define TLV493D_A1B6_Slave_2_IICAddr_bits		0x20			//01
#define TLV493D_A1B6_Slave_3_addr				0xBD
#define TLV493D_A1B6_Slave_3_IICAddr_bits		0x00			//00

//Need low SDA pin at power-up
#define TLV493D_A1B6_Slave_4_addr				0x17
#define TLV493D_A1B6_Slave_4_IICAddr_bits		0x60			//11
#define TLV493D_A1B6_Slave_5_addr				0x1F
#define TLV493D_A1B6_Slave_5_IICAddr_bits		0x40			//10
#define TLV493D_A1B6_Slave_6_addr				0x37
#define TLV493D_A1B6_Slave_6_IICAddr_bits		0x20			//01
#define TLV493D_A1B6_Slave_7_addr				0x3F
#define TLV493D_A1B6_Slave_7_IICAddr_bits		0x00			//00

#define TLV493D_A1B6_I2C_timeout				10

#define TLV493D_A1B6_Power_down_mode			0
#define TLV493D_A1B6_Fast_mode_3300_Hz			0x2
#define TLV493D_A1B6_Low_power_mode_100_Hz		0x1
#define TLV493D_A1B6_Ultra_low_power_mode_10_Hz	0				//Interrupt must be enabled
#define TLV493D_A1B6_Master_controlled_mode		0x3

#define TLV493D_A1B6_Interrupt_enable			0x4
#define TLV493D_A1B6_Interrupt_disable			0x0

#define TLV493D_A1B6_Temperature_measurement_enabled	0
#define TLV493D_A1B6_Temperature_measurement_disabled	0x80

#define TLV493D_A1B6_Low_power_period_12_ms		0x40
#define TLV493D_A1B6_Low_power_period_100_ms	0

#define TLV493D_A1B6_Parity_test_enabled		0x20
#define TLV493D_A1B6_Parity_test_disabled		0

static inline HAL_StatusTypeDef TLV493D_A1B6_read(I2C_HandleTypeDef * const p_hi2c, uint8_t const p_addr, uint8_t * const p_in_buf) {
	return HAL_I2C_Master_Receive_IT(p_hi2c, p_addr, p_in_buf, 7);
}

static inline HAL_StatusTypeDef TLV493D_A1B6_preread(I2C_HandleTypeDef * const p_hi2c, uint8_t const p_addr){
	static uint8_t l_in_buf[1] = {0};

	return HAL_I2C_Master_Receive_IT(p_hi2c, p_addr, l_in_buf, 1);
}

HAL_StatusTypeDef TLV493D_A1B6_set_mode(I2C_HandleTypeDef * const p_hi2c, uint8_t const p_addr, uint8_t const p_new_IICAddr_bits);
void TLV493D_A1B6_recovery(I2C_HandleTypeDef * const p_hi2c);
void TLV493D_A1B6_reset(I2C_HandleTypeDef * const p_hi2c, uint8_t const p_addr);

#endif  //Lib_for_TLV493D_A1B6
