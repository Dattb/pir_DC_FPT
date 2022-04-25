/*
 * RD_Light_Sensor.c
 *
 *  Created on: May 10, 2021
 *      Author: Dat_UTC
 */




#include "RD_Light_Sensor.h"
#include "RD_Pir_AC_Control.h"

float Kp = 90;
extern u16 rd_node_addr;
extern u16 rd_box_addr;


u8 RD_i2c_rx_buff[2] = {0};
u8 RD_i2c_tx_buff[2] = {OPT3001_CONFIG_REG_HIGH,OPT3001_CONFIG_REG_LOW};
u8 light_sensor_to_gw_tx_buff[6]= {0};

extern u16 Lux_val_new;

unsigned int CalculateLux(unsigned int rsp_lux)
{
	unsigned int lux_LSB = 0;
	unsigned char lux_MSB = 0;
	unsigned int lux_Value = 0;
	unsigned int pow = 1;
	unsigned char i;
	lux_LSB = rsp_lux & 0x0FFF;
	lux_MSB = ((rsp_lux>>12) & 0x0F);
	//Lux_Value = 0.01 * pow(2,Lux_MSB) * Lux_LSB; //don't use
	for(i=0;i<lux_MSB;i++){
		pow=pow*2;
	}
	lux_Value = 0.01 * pow * lux_LSB;
	// RD_EDIT: calib lux
	#if	LUX_UART_DEBUG
		char UART_TempSend[128];
		sprintf(UART_TempSend,"Lux befor calib %d \n",lux_Value);
		uart_CSend(UART_TempSend);
	#endif
	if(lux_Value > 60){
		int calib_lux = 1.7339 * lux_Value - 39.868;
		lux_Value = calib_lux;
	}
	return lux_Value;
}


u16 light_sensor_read_cnt=0;
u32 time_send_lux;
unsigned int  real_lux_new = 0,real_lux_old = 0;
#define LUX_THRESH_HOLD			30
unsigned char send_lux_flag = 0;
void Lux_send(unsigned int threshold){
	real_lux_new = CalculateLux(RD_Light_Sensor_read_raw_val());
	if(real_lux_old != real_lux_new){
		if(real_lux_old >= real_lux_new){
			if(real_lux_old - real_lux_new >= threshold){
				#if	LUX_UART_DEBUG
					char UART_TempSend[128];
					sprintf(UART_TempSend,"Lux is %d \n",real_lux_old);
					uart_CSend(UART_TempSend);
				#endif
				//send lux here after 10 second cycle
				// pull up send lux flag
				send_lux_flag = 1;
				real_lux_old = real_lux_new;
			}
		}
		else {
			if((real_lux_new - real_lux_old >= threshold)){
				#if	LUX_UART_DEBUG
					char UART_TempSend[128];
					sprintf(UART_TempSend,"Lux is %d \n",real_lux_old);
					uart_CSend(UART_TempSend);
				#endif
				//send lux here after LUX_LOOP_TIME ms cycle
				// pull up send lux flag
				send_lux_flag = 1;
				real_lux_old = real_lux_new;
			}
		}
//		char UART_TempSend[128];
//		sprintf(UART_TempSend,"Lux change, current is: %d lux\n",real_lux_old);
//		uart_CSend(UART_TempSend);
	}
	if(send_lux_flag){
		send_lux_flag = 0;
		unsigned char lux_send_buff[4] = {0};
		lux_send_buff[0] = 0x0d;
		lux_send_buff[1] = real_lux_new;
		lux_send_buff[2] = real_lux_new >>8;
		mesh_tx_cmd2normal(0xe5,(u8 *) lux_send_buff,3, rd_node_addr ,rd_box_addr, 2);
		char UART_TempSend[128];
		sprintf(UART_TempSend,"Lux is %d \n",real_lux_new);
		uart_CSend(UART_TempSend);
	}

}


unsigned int RD_Light_Sensor_read_raw_val() {
	u16  lux_read_data;

		u8 RD_i2c_rx_buff[3] = {0};
		u8 RD_i2c_tx_buff[2] = {OPT3001_CONFIG_REG_HIGH,OPT3001_CONFIG_REG_LOW};
		time_send_lux = clock_time_s();
		light_sensor_read_cnt = 0;

		i2c_write_series(OPT3001_CONFIG_REGISTER,OPT3001_CONFIG_REGISTER_LEN,(u8 *)RD_i2c_tx_buff, 2);
		i2c_read_series(OPT3001_RESULT_REGISTER,OPT3001_RESULT_REGISTER_LEN, (u8 *)RD_i2c_rx_buff, 3);

		lux_read_data = (RD_i2c_rx_buff[0]<<8) | RD_i2c_rx_buff[1];
		#if	LUX_UART_DEBUG
				char UART_TempSend[128];
				sprintf(UART_TempSend,"Raw lux data %d s\n",lux_read_data);
				uart_CSend(UART_TempSend);
		#endif
	return lux_read_data;
}


void RD_Send_raw_Lux(u16 Lux_raw_val,unsigned int lux_real)
{
	*(light_sensor_to_gw_tx_buff)   = 0x04;
	*(light_sensor_to_gw_tx_buff+1) = 0x00;
	*(light_sensor_to_gw_tx_buff+2)   = (u8)(Lux_raw_val>>8);
	*(light_sensor_to_gw_tx_buff+3) = (u8)(Lux_raw_val);
	*(light_sensor_to_gw_tx_buff+4)   = (u8)(lux_real>>8);
	*(light_sensor_to_gw_tx_buff+5) = (u8)(lux_real);
	mesh_tx_cmd2normal_primary(SENSOR_STATUS, (u8 *)light_sensor_to_gw_tx_buff, 6, 0x0001, 2);
}

void Light_sensor_i2c_init(){
	i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
	i2c_master_init(SLAVE_DEVICE_ADDR,(unsigned char)(CLOCK_SYS_CLOCK_HZ/(4*I2C_CLK_SPEED)));
}
unsigned int Lux_clock_time_read = 0;
void Lux_loop (unsigned int Lux_loop_time){
	if(clock_time_ms() - Lux_clock_time_read >= Lux_loop_time){
		Lux_clock_time_read = clock_time_ms();

		#if	LUX_UART_DEBUG
			char UART_TempSend[128];
			uart_CSend("-------------------------------\n");
			sprintf(UART_TempSend,"Lux after calib %d  || clock time is %d \n",CalculateLux(RD_Light_Sensor_read_raw_val()),clock_time_s());
			uart_CSend(UART_TempSend);


//		char UART_TempSend[128];
//		uart_CSend("-------------------------------\n");
//		sprintf(UART_TempSend,"Lux after calib %d  || clock time is %d \n",CalculateLux(RD_Light_Sensor_read_raw_val()),clock_time_s());
//		uart_CSend(UART_TempSend);
//		Lux_clock_time_read = clock_time_ms();
		#endif
		Lux_send(LUX_THRESH_HOLD);
	}
}
