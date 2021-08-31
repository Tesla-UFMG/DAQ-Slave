#include "general_can.h"
#include "CAN_handler.h"

static FDCAN_HandleTypeDef* can_ptr;;

static FDCAN_TxHeaderTypeDef TxHeader;

uint8_t RxData[8];
FDCAN_RxHeaderTypeDef RxHeader;
int16_t datageneral[4];
uint32_t idgeneral;



//função que inicializa a can geral, chamada em initializer.c
void initialize_general_CAN(FDCAN_HandleTypeDef* can_ref) {
	can_ptr = can_ref;
	void CAN_general_receive_callback(FDCAN_HandleTypeDef*, uint32_t);
	initialize_CAN(can_ptr, CAN_general_receive_callback, &TxHeader);
}



//função usada para transmitir alguma mensagem
void general_can_transmit(uint32_t id, uint16_t* data) {
	can_transmit(can_ptr, &TxHeader, id, data);
}
