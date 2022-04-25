/*
 * RD_pir_DC_Control.c
 *
 *  Created on: Apr 22, 2022
 *      Author: PC5
 */

#include "RD_pir_DC_Control.h"
#include  "RD_Pir_AC_Control.h"
#include "SoftUart.h"
#include "RD_Light_Sensor.h"

extern u16 rd_box_addr;
extern u16 rd_node_addr;
//unsigned char pirDetect(){
//
//}

pirPinTypeDef rdPir;


void pirDCInit(pirPinTypeDef *pir){
	pir->pinHigh = PIR_DC_HIGH;
	pir->pinLow = PIR_DC_LOW;
	pir->timeOut = pirGetTimeoutReg();
	pir->flag.data = pirGetFlagReg();
}



unsigned char pirDetect(pirPinTypeDef *pin){
	if(pin->timeOut <= SYS_32K_TICK_S){
		if(SYS_32K_TICK_S - pin->timeOut >= PIR_DC_FLAG_TIMEOUT ){
			softUartSend(&sUart1,"tim_1\n");
			pin->timeOut = SYS_32K_TICK_S;
			//softUartSendInt(pin->timeOut);
			pirSaveTimeoutReg(pin->timeOut);
			pin->flag.pirHighFlag = 0;
			pin->flag.pirLowFlag = 0;
			pirSaveFlagReg(pin->flag.data);
		}
	}
	else{
		if((0xffffffff - pin->timeOut + SYS_32K_TICK_S) >= PIR_DC_FLAG_TIMEOUT){
			softUartSend(&sUart1,"tim_2\n");
			pin->timeOut = SYS_32K_TICK_S;
			//softUartSendInt(pin->timeOut);
			pirSaveTimeoutReg(pin->timeOut);
			pin->flag.pirHighFlag = 0;
			pin->flag.pirLowFlag = 0;
			pirSaveFlagReg(pin->flag.data);
		}
	}

	if(pin->state.pirHighState){
		//softUartSend(&sUart1,"r_Hi\n");
		unsigned char checkPullupTime = 0;
		for(unsigned short i=0;i<100;i++){
			sleep_us(50);
			if(gpio_read(pin->pinHigh)){
				checkPullupTime++;
				if(checkPullupTime >= PIR_THESHOLD){
					//softUartSend(&sUart1,"f_Hi\n");
					pin ->flag.pirHighFlag = 1;
					pin->timeOut = SYS_32K_TICK_S;
					pirSaveFlagReg(pin ->flag.data);
					pirSaveTimeoutReg(pin->timeOut);
					break;
				}
			}
			else {
				//softUartSend(&sUart1,"H_fa\n");
				pin ->flag.pirHighFlag = 0;
				pirSaveFlagReg(pin->flag.data);
				checkPullupTime = 0;
				break;
			}
		}
	}

	if(pin->state.pirLowState){
		//softUartSend(&sUart1,"r_Low\n");
		unsigned char checkPullupTime = 0;
		for(unsigned short i=0;i<100;i++){
			sleep_us(50);
			if(gpio_read(pin->pinLow)){
				checkPullupTime++;
				if(checkPullupTime >= PIR_THESHOLD){
					//softUartSend(&sUart1,"f_Low\n");
					pin ->flag.pirLowFlag = 1;
					pin->timeOut = SYS_32K_TICK_S;
					pirSaveFlagReg(pin->flag.data);
					pirSaveTimeoutReg(pin->timeOut);
					break;
				}
			}
			else {
				//softUartSend(&sUart1,"L_fa\n");
				pin ->flag.pirLowFlag = 0;
				pirSaveFlagReg(pin->flag.data);
				checkPullupTime = 0;
				break;
			}
		}
	}
	sUartInit(&sUart1);
	if(pin->flag.pirHighFlag && pin->flag.pirLowFlag){
		softUartSend(&sUart1,"M\n");
		pin->flag.motionFlag = 1;
	}
	return (pin->flag.motionFlag);
}


void sendMotion(unsigned char motionEdge){
	if(motionEdge){
		unsigned char pir_send_buff[2] = {0x0c,0x01};
		mesh_tx_cmd2normal(0xe5,(u8 *) pir_send_buff,2,rd_node_addr,rd_box_addr,2);
	}
}

void pirSaveTimeoutReg(unsigned short data){
	analog_write(PIR_TIMEOUT_REG0,(unsigned char)(data>>8));
	analog_write(PIR_TIMEOUT_REG1,(unsigned char)(data));
}

void pirSaveFlagReg(unsigned char data){
	analog_write(PIR_FLAG_SAVE_REG,data);
}

unsigned short pirGetTimeoutReg(){
	 unsigned long rdTick_32k = 0;
	 rdTick_32k  = analog_read(PIR_TIMEOUT_REG0)<<8 |analog_read(PIR_TIMEOUT_REG1);
	 return rdTick_32k;
}

unsigned char pirGetFlagReg(){
	return analog_read(PIR_FLAG_SAVE_REG);
}





