/*
 * LTC2941.h
 *
 *  Created on: 24 mar 2018
 *      Author: Robert S
 *
 *      @description 	library written for LTC2941-1 LiIon charge estimator
 *      @warnings		strange incrementation of registers addresses in case of reading single register value (e.g. control register address 0x01 is incremented to 0x81, do not know why).
 *
 */

#ifndef LTC2941_H_
#define LTC2941_H_

#include "main.h"
#include "stm32f4xx_hal.h"


#define LTC2941_ADDR 	0xC8
#define STATUS_REG		0x00
#define CONTROL_REG		0x01
#define ACC_CHARGE_MSB	0x02
#define ACC_CHARGE_LSB	0x03
#define CH_THR_H_MSB	0x04
#define CH_THR_H_LSB	0x05
#define CH_THR_L_MSB	0x06
#define CH_THR_L_LSB	0x07


void read_all_registers(uint8_t *buf, I2C_HandleTypeDef *hi2c);
void get_charge_reg_value(uint8_t *all_reg_buf, uint8_t *return_buf);
void get_high_thresh_reg_value(uint8_t *all_reg_buf, uint8_t *return_buf);
void get_low_thresh_reg_value(uint8_t *all_reg_buf, uint8_t *return_buf);
uint8_t get_status_reg_value(uint8_t *all_reg_buf);
uint8_t get_control_reg_value(uint8_t *all_reg_buf);
void set_charge_value_max(I2C_HandleTypeDef *hi2c);
void set_charge_value(I2C_HandleTypeDef *hi2c, uint8_t *buf_charge_value);
void set_charge_value_min(I2C_HandleTypeDef *hi2c);


#endif /* LTC2941_H_ */
