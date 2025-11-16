///*
// * CAN.h
// * Header File
// *
// * Created on: 11th November 2025 -- 11:31pm
// * "I FLY OUT 2AM and lukes gonna touch me"
// * Author:Yodiddy & Harrmish
// *
// */
//
//
//#include "CAN.h"
//
//CAN_TxHeaderTypeDef TxHeader = {0}; //Setting for code in User code 4
//CAN_RxHeaderTypeDef RxHeader = {0};
//uint32_t mailbox;
//
//void CAN_SendTestFrame(void)
//{
//	static uint8_t counter = 0; //Counter set at *4:24
//	//Why dont we need CAN_RxHeader TypeDef TxHeader = {0} *here? like RX header is below
//	uint8_t data[8] = {0};
//
//	TxHeader.StdId = 0x30; //Do i need to change to pdm id
//	TxHeader.IDE = CAN_ID_STD;
//	TxHeader.RTR = CAN_RTR_DATA;
//	TxHeader.DLC = 8;
//
//	data[0] = counter++; //changes on each send
//
//	(void)HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &mailbox);
//}
//
////MCU LED
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
//{
//	uint8_t d[8];
//	if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &RxHeader, d) == HAL_OK)
//	{
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	}
//}
