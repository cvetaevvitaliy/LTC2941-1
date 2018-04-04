/*
 * LTC2941.c
 *
 *  Created on: 24 mar 2018
 *      Author: Robert S
 *
 *      @description 	library written for LTC2941-1 LiIon charge estimator
 *      @warnings		strange incrementation of registers addresses in case of reading single register value (e.g. control register address 0x01 is incremented to 0x81, do not know why).
 *
 */


#include "main.h"
#include "stm32f4xx_hal.h"
#include "LTC2941.h"


/*
 * @brief			function gets all registers because of strange incrementation of address in case of getting single reg
 * @param			buffer which will contain 2 bytes of the data
 * @return			function returns control register value via pointer
*/
void read_all_registers(uint8_t *buf, I2C_HandleTypeDef *hi2c)
{
	  while( HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY);
	  HAL_I2C_Master_Sequential_Transmit_IT(hi2c, LTC2941_ADDR, 0x00, 1, I2C_FIRST_FRAME);
	  while( HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY);
	  HAL_I2C_Master_Sequential_Receive_IT(hi2c, LTC2941_ADDR, buf, 8, I2C_LAST_FRAME);
	  while( HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY);

}

/*
 * @brief			function gets status reg value from all registers value
 * @param			all registers value buffer @read_all_registers()
 * @return			function returns 1 byte status reg value
 */
uint8_t get_status_reg_value(uint8_t *all_reg_buf)
{
	return all_reg_buf[0];
}

/*
 * @brief			function gets control reg value from all registers value
 * @param			all registers value buffer @read_all_registers()
 * @return			function returns 1 byte control reg value
 */
uint8_t get_control_reg_value(uint8_t *all_reg_buf)
{
	return all_reg_buf[1];
}

/*
 * @brief			function gets charge reg value from all registers value
 * @param			all registers value buffer @read_all_registers()
 * @return			function returns 2 byte charge reg value via pointer
 */
void get_charge_reg_value(uint8_t *all_reg_buf, uint8_t *return_buf)
{
	return_buf[0] = all_reg_buf[2];
	return_buf[1] = all_reg_buf[3];
}

/*
 * @brief			function gets high threshold reg value from all registers value
 * @param			all registers value buffer @read_all_registers()
 * @return			function returns 2 byte high threshold reg value via pointer
 */
void get_high_thresh_reg_value(uint8_t *all_reg_buf, uint8_t *return_buf)
{
	return_buf[0] = all_reg_buf[4];
	return_buf[1] = all_reg_buf[5];
}

/*
 * @brief			function gets low threshold reg value from all registers value
 * @param			all registers value buffer @read_all_registers()
 * @return			function returns 2 byte low threshold reg value via pointer
 */
void get_low_thresh_reg_value(uint8_t *all_reg_buf, uint8_t *return_buf)
{
	return_buf[0] = all_reg_buf[6];
	return_buf[1] = all_reg_buf[7];
}

/*
 * @brief			function sets maximum value 0xFFFF of charge register (0x02 ... 0x03)
 * @param			Handle Type Def of I2C
 * @return			no value returned
 */
void set_charge_value_max(I2C_HandleTypeDef *hi2c)
{
	uint8_t buf[3];
	buf[0] = 0x02;
	buf[1] = 0xFF;
	buf[2] = 0xFF;

	  while( HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY);
	  HAL_I2C_Master_Transmit_IT(hi2c, 0xC8, buf, 3);
	  while( HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY);
}


/*
 * @brief			function sets minimum value 0x0000 of charge register (0x02 ... 0x03)
 * @param			Handle Type Def of I2C
 * @return			no value returned
 */

void set_charge_value_min(I2C_HandleTypeDef *hi2c)
{
	uint8_t buf[3];
	buf[0] = 0x02;
	buf[1] = 0x00;
	buf[2] = 0x00;

	  while( HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY);
	  HAL_I2C_Master_Transmit_IT(hi2c, 0xC8, buf, 3);
	  while( HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY);
}


/*
 * @brief			function sets desired value of charge register (0x02 ... 0x03)
 * @param			Handle Type Def of I2C
 * @param			buffer of charge value register to be written
 * @return			no value returned
 */
void set_charge_value(I2C_HandleTypeDef *hi2c, uint8_t *buf_charge_value)
{
	uint8_t buf[3];
	buf[0] = 0x02;
	buf[1] = buf_charge_value[0];
	buf[2] = buf_charge_value[1];

	  while( HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY);
	  HAL_I2C_Master_Transmit_IT(hi2c, 0xC8, buf, 3);
	  while( HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY);
}
