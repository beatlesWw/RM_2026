#include "dm4310_drv.h"
#include "can.h"
#include "bsp_can_1.h"
#include "CAN_receive.h"
void dm4310_enable(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			enable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			enable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
	}	
}

void dm4310_disable(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			disable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			disable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			disable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
	}	
	dm4310_clear_para(motor);
}

void dm4310_ctrl_send(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			mit_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case 1:
			pos_speed_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case 2:
			speed_ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
	}	
}

void dm4310_set(motor_t *motor)
{
	motor->ctrl.kd_set 	= motor->cmd.kd_set;
	motor->ctrl.kp_set	= motor->cmd.kp_set;
	motor->ctrl.pos_set	= motor->cmd.pos_set;
	motor->ctrl.vel_set	= motor->cmd.vel_set;
	motor->ctrl.tor_set	= motor->cmd.tor_set;

}

void dm4310_clear_para(motor_t *motor)
{
	motor->cmd.kd_set 	= 0;
	motor->cmd.kp_set	 	= 0;
	motor->cmd.pos_set 	= 0;
	motor->cmd.vel_set 	= 0;
	motor->cmd.tor_set 	= 0;
	
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.kp_set	= 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
}

void dm4310_clear_err(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			clear_err(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			clear_err(hcan, motor->id, POS_MODE);
			break;
		case 2:
			clear_err(hcan, motor->id, SPEED_MODE);
			break;
	}	
}

void dm4310_fbdata(motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_bsp_send_data(hcan, id, data, 8);
}

void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_send_data(hcan, id, data, 8);
}

