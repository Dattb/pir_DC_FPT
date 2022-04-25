#include "RD_Light_Sensor.h"
#include "RD_Pir_AC_Control.h"
#include "Sensor_Flash.h"
#include "RD_type_device.h"

RD_SensorSencesStoreTypedef RD_Sence_Store_obj[NUMBER_OF_SCENE];
u8 SceneID_Store[NUMBER_OF_SCENE];
u8  RD_Sence_Flash_Read_Buff [FLASH_BUFF_LEN];
u8  RD_Sence_Flash_Write_Buff[FLASH_BUFF_LEN];

u8  RD_Save_GW_Flash_Write_Buff[GW_SAVE_BUFF_LEN];
u8  RD_Save_GW_Flash_Read_Buff[GW_SAVE_BUFF_LEN];
extern u16 Gw_addr;
extern u16 Balcane_lux;
extern u16 Group_id;
extern u16 pir_time;
extern u16 friend_poll;


void RD_FlashSaveSenceData(u8 *RD_ScenePar)
{

	RD_Sensor_data_tdef  COndition;
	COndition.data = (RD_ScenePar[4]<<24)|(RD_ScenePar[5]<<16)|(RD_ScenePar[6]<<8);

    RD_SensorSencesStoreTypedef *RD_Sence_Store_pointer;
    RD_Sence_Store_pointer = (RD_SensorSencesStoreTypedef *) RD_ScenePar;
	RD_Sence_Store_obj[COndition.Pir_Conditon]  = *RD_Sence_Store_pointer;


    RD_Sence_Store_pointer = (RD_SensorSencesStoreTypedef *)RD_Sence_Flash_Write_Buff;
    for(u8 i=0;i<NUMBER_OF_SCENE;i++) *(RD_Sence_Store_pointer+i) = RD_Sence_Store_obj[i];

	flash_erase_sector(SCENE_FLASH_ADDR);
	flash_write_page(SCENE_FLASH_ADDR,FLASH_BUFF_LEN,RD_Sence_Flash_Write_Buff);
	RD_FlashReadSceneData ();

	#define RSP_MAX              2
	mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_RSP,(u8 *)RD_ScenePar,BYTE_OF_ONE_SCENE_OBJ, GATEWAY_ADDR, RSP_MAX);

}




void RD_FlashWriteGwAddr(u16 addr){

	RD_Save_GW_Flash_Write_Buff[0]=(u8)(addr);
	RD_Save_GW_Flash_Write_Buff[1]=(u8)(addr>>8);

	RD_Save_GW_Flash_Write_Buff[2]=(u8)(pir_time);
	RD_Save_GW_Flash_Write_Buff[3]=(u8)(pir_time>>8);

	flash_erase_sector(RD_SAVE_GW_FLASH_ADDR);

	flash_write_page(RD_SAVE_GW_FLASH_ADDR,GW_SAVE_BUFF_LEN,RD_Save_GW_Flash_Write_Buff);

	RD_Flash_read_GW_Addr();

}

void RD_FlashWrite_pir_time(u16 time){


	RD_Save_GW_Flash_Write_Buff[0]=(u8)(Gw_addr);
	RD_Save_GW_Flash_Write_Buff[1]=(u8)(Gw_addr>>8);

	RD_Save_GW_Flash_Write_Buff[2]=(u8)(time);
	RD_Save_GW_Flash_Write_Buff[3]=(u8)(time>>8);

	flash_erase_sector(RD_SAVE_GW_FLASH_ADDR);

	flash_write_page(RD_SAVE_GW_FLASH_ADDR,GW_SAVE_BUFF_LEN,RD_Save_GW_Flash_Write_Buff);

	RD_Flash_read_GW_Addr();

}




void RD_Flash_read_GW_Addr(){

	/*
	 * RD_EDIT: - Kiem tra xem ham luu scene, goi scene, xoa scene da chay ok chua
	 * 			- Kiem tra xem da luu duoc dia chi GW chua
	 * 			- Kiem tra xem khi xoa khoi mang thi Scene va dia chi GW da duoc xoa di chua.
	 */

		flash_read_page (RD_SAVE_GW_FLASH_ADDR, GW_SAVE_BUFF_LEN, RD_Save_GW_Flash_Read_Buff);

		u16 Flash_GwAddr;
		u16 Pir_time;

		Flash_GwAddr = 	(RD_Save_GW_Flash_Read_Buff[1]<<8)|(RD_Save_GW_Flash_Read_Buff[0]);
		if(Flash_GwAddr!=0xffff && Flash_GwAddr!=0x0000) Gw_addr = Flash_GwAddr;
		else Gw_addr=0x0001;

		Pir_time = 	(RD_Save_GW_Flash_Read_Buff[3]<<8)|(RD_Save_GW_Flash_Read_Buff[2]);
		if(Pir_time==0xffff||Pir_time<15) pir_time = 15;
		else pir_time = Pir_time;

}




extern u8 reset_cnt;
void RD_FlashReadSceneData ()
{

	flash_read_page (SCENE_FLASH_ADDR, FLASH_BUFF_LEN, RD_Sence_Flash_Read_Buff);

    RD_SensorSencesStoreTypedef *RD_Sence_Store_pointer;
    RD_Sence_Store_pointer = (RD_SensorSencesStoreTypedef *)RD_Sence_Flash_Read_Buff;
	for(u8 i=0;i<NUMBER_OF_SCENE;i++) RD_Sence_Store_obj[i] = *(RD_Sence_Store_pointer + i);

	//RD_Sensor_data_tdef COndition;
//		u8  check_data_buff[6];
//		u32 low =RD_Sence_Store_obj[1].sensor_data.Lux_low;
//		u32 hi =RD_Sence_Store_obj[1].sensor_data.Lux_hi;
//		check_data_buff[0] = RD_Sence_Store_obj[1].sensor_data.Pir_Conditon;
//		check_data_buff[1] = RD_Sence_Store_obj[1].sensor_data.Light_Conditon;
//		check_data_buff[2] = (u8)(low>>8);
//		check_data_buff[3] = (u8)(low);
//		check_data_buff[4] = (u8) (hi>>8);
//		check_data_buff[5] = (u8)(hi);
//		mesh_tx_cmd2normal_primary(0x52,(u8 *)check_data_buff,6, GATEWAY_ADDR, RSP_MAX);

}

void RD_ClearAllSceneInFlash()
{
	flash_erase_sector(SCENE_FLASH_ADDR);
	//flash_write_page(SCENE_FLASH_ADDR,FLASH_BUFF_LEN,RD_Sence_Flash_Clear_Buff);
	//RD_FlashWriteGwAddr(Gw_addr);
	RD_FlashReadSceneData ();
}

void RD_ClearSceneInFlash(u8 *RD_ScenePar)
{

    RD_SensorSencesStoreTypedef *RD_Sence_Store_pointer,Scene_obj;
    RD_Sence_Store_pointer = (RD_SensorSencesStoreTypedef *) RD_ScenePar;
    Scene_obj=*RD_Sence_Store_pointer;

	RD_SensorSencesStoreTypedef RD_Clear_scene = {0};
	 u16 Sid = Scene_obj.SceneID[1]<<8|Scene_obj.SceneID[0];
	 if(Sid != 0x0000 && Sid != 0xffff){
		for(u8 i=0;i<NUMBER_OF_SCENE;i++){
			if(RD_Sence_Store_obj[i].SceneID[0]==Scene_obj.SceneID[0]&&RD_Sence_Store_obj[i].SceneID[1]==Scene_obj.SceneID[1]){
				RD_Sence_Store_obj[i]= RD_Clear_scene;
				#define RSP_MAX              2
				mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_RSP,(u8 *)RD_ScenePar,BYTE_OF_ONE_SCENE_OBJ, GATEWAY_ADDR, RSP_MAX);
			}
		}
	 }

    RD_SensorSencesStoreTypedef *RD_Sence_Write_pointer;
    RD_Sence_Write_pointer = (RD_SensorSencesStoreTypedef *) RD_Sence_Flash_Write_Buff;
    for(u8 i=0;i<NUMBER_OF_SCENE;i++) *(RD_Sence_Write_pointer+i) = RD_Sence_Store_obj[i];

	flash_erase_sector(SCENE_FLASH_ADDR);
	flash_write_page(SCENE_FLASH_ADDR,FLASH_BUFF_LEN,RD_Sence_Flash_Write_Buff);
	RD_FlashReadSceneData ();

}



extern u8 reset_cnt;
void RD_Write_reset_cnt(u8 data)
{
	RD_Factory_Reset_Flash_Write_Buff[0]=data;
	flash_erase_sector(0x23000);
	flash_write_page(0x23000,1,RD_Factory_Reset_Flash_Write_Buff);
	RD_FlashReadSceneData ();
}

void rd_flash_read_data(unsigned long addr, u16 *out){
	u8 buff_read[2] = {0};
	flash_read_page(addr,2,buff_read);
	*out = buff_read[1]<<8|buff_read[0];
}


