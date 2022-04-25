/*
 * RD_type_device.c
 *
 *  Created on: Apr 18, 2021
 *      Author: Dat_UTC
 */

#include"RD_type_device.h"
#include"Sensor_Flash.h"
#include "RD_Light_Sensor.h"
#include "RD_Pir_AC_Control.h"
extern u16 Gw_addr;
unsigned int confirm_fail_time = 0;
unsigned char confirm_fail = 0;
unsigned char bind_all_flag = 0;
unsigned int bind_time = 0;
u16 rd_node_addr = 0;
u16 time_to_end_config = 0;
u16 rd_box_addr = 0;
u8 type_device_to_gw_tx_buff[8];
u8 save_gw_addr_to_gw_tx_buff[8];
u8 save_gw_tx_buff[8] = {0};
u8 type_device_to_gw_tx_buff[8] = {0};


int RD_Messenger_Process_Type_Device(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	saveAddBox(par,par_len,cb_par);
	return 0;
}

int end_config(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	rd_flash_read_data(0x78000,&rd_node_addr);
	unsigned short confirm_data = rd_rogo_rule_confirm(par);
	if((confirm_data) && (rd_node_addr == confirm_data)){
		bind_all_flag = 0;
		//uart_CSend("confirm done!");
		u8 buff_ = 1;
		mesh_tx_cmd2normal(0xe1, (u8 *)(&buff_), 1, cb_par->adr_dst, cb_par->adr_src, 2);
	}
	else {
		confirm_fail = 1;
		confirm_fail_time = clock_time_ms();
		u8 buff_ = 0;
		mesh_tx_cmd2normal(0xe1, (u8 *)(&buff_), 1, cb_par->adr_dst, cb_par->adr_src, 2);
	}
	return 0;
}

int RD_Messenger_Process_Null_1(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	return 0;
}

unsigned char modulo_calculate(unsigned short par1,unsigned short par2,unsigned short *out){
	if(par1>par2){
		*out = par1 % par2;
		return 1;
	}
	else if(par1<par2){
		*out = par2 % par1;
		return 1;
	}
	else return 0;
}

int rd_rogo_rule_confirm(unsigned char *data){
	unsigned short Param[4];
	unsigned short Modulo;
	Param[0] = (data[0]<<8|data[1]);
	Param[1] = (data[2]<<8|data[3]);
	Param[2] = (data[4]<<8|data[5]);
	Param[3] = (data[6]<<8|data[7]);
	if(modulo_calculate(Param[0],Param[1],&Modulo)){
		if(	modulo_calculate(Modulo,Param[2],&Modulo)){
			if(modulo_calculate(Modulo,Param[3],&Modulo)){
				return Modulo;
			}
			else return 0;
		}
		else return 0;
	}
	else return 0;
}



int RD_mesh_cmd_sig_cfg_bind(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	bind_time = clock_time_ms();

	bind_all_flag = 1;
	time_to_end_config = clock_time_s();
	extern int mesh_cmd_sig_cfg_bind(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
	flash_erase_sector(0x78000);
	flash_write_page(0x78000, 2, par); //doc dia chi nut cua Node

	rd_flash_read_data(0x78000,&rd_node_addr);

	return(mesh_cmd_sig_cfg_bind(par, par_len,  cb_par));
}

void saveAddBox(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	//rd_box_addr = par[1]<<8|par[0];
	flash_erase_sector(0x79000);
	flash_write_page(0x79000, 2, par); //doc dia chi nut cua Node

	rd_flash_read_data(0x79000,&rd_box_addr);
	u8 respone_buff[] = {0};
	respone_buff[0] = 1;
	respone_buff[1] = rd_box_addr;
	respone_buff[2] = rd_box_addr>>8;
	mesh_tx_cmd2normal(0xe0, (u8 *)respone_buff, 3, cb_par->adr_dst, cb_par->adr_src, 2);// leng (1),tempDiaChiNut, addBox, 2;
}



void Join_confirm_check(){
	#if(CONFIRM_ENABLE)
		if(confirm_fail){
			if(clock_time_ms() - confirm_fail_time >= 1000){
				kick_out();
			}
			//kick_out();
		}
	#endif
}

void confirm_receive_check(){
	#if(CONFIRM_ENABLE)
		if(bind_all_flag){
			if(clock_time_ms() - bind_time >= 120000){
				char UART_TempSend[128];
				sprintf(UART_TempSend,"dont have confirm, kick out: time is %d\n",clock_time_s());
				uart_CSend(UART_TempSend);
				kick_out();
			}
		}
	#endif
}


void flash_init(){
	if(is_provision_success()){
		rd_flash_read_data(0x78000,&rd_node_addr);
		rd_flash_read_data(0x79000,&rd_box_addr);
	}
	char UART_TempSend[128];
	sprintf(UART_TempSend,"rd_node_addr : %d --- rd_box_addr %d\n",rd_node_addr,rd_box_addr);
	uart_CSend(UART_TempSend);
}




