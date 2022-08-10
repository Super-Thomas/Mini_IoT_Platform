#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "drivers_lora.h"
#include "esensor.h"
#include "main.h"
#include "lora_packet.h"
#include "task_priority.h"
#include "lora.h"

static uint8_t g_u8LoRaTxStartFlag = 0;
static uint8_t g_u8LoRaRxStartFlag = 0;

static void LoraTask(void *pvParam);
void LoRa_RxDoneAction(void);
void LoRa_TxDoneAction(void);
void LoRa_TxStart(void);

void SetLoRaTxStartFlag(uint8_t u8Value)
{
	g_u8LoRaTxStartFlag = u8Value;
}

void SetLoRaRxStartFlag(uint8_t u8Value)
{
	g_u8LoRaRxStartFlag = u8Value;
}

uint8_t GetLoRaTxStartFlag(void)
{
	return g_u8LoRaTxStartFlag;
}

uint8_t GetLoRaRxStartFlag(void)
{
	return g_u8LoRaRxStartFlag;
}

void LoRa_RxDoneAction(void)
{
	uint16_t u16Size = LORA_PACKET_SIZE;
	double fRSSIvalue = (double)0.0f;
	
	LP_InitPacket();
	LORAM_GetRxPacket(LP_GetPacket(), (unsigned short int *)&u16Size);
	
	if (u16Size > 0) {
		//printf("[%s] Received sensor data came from Device via LoRa... (%d bytes)\n", __FILE__, u16Size);
		
		fRSSIvalue = LORAM_GetPacketRssi();
		LP_UpdatePacket(GetReceivedSensorData());
		
		//printf("[%s] RSSI value: %.2f\n", __FILE__, fRSSIvalue);
	}
}

void LoRa_TxDoneAction(void)
{
	LORAM_RX_Init();
}

void LoRa_TxStart(void)
{
	//printf("[%s] Send sensor data to Gateway via LoRa... \n", __FILE__);
	
	LP_UpdatePacket(NULL);
  LORAM_SetTxPacket(LP_GetPacket(), LORA_PACKET_SIZE);
	LORAM_TX_Init();
}

void CreateLoRaTask(void)
{
	BaseType_t ret;
	
	ret = xTaskCreate(LoraTask, "LoraTask", configMINIMAL_STACK_SIZE, (void *)NULL, LORA_TASK_PRIORITY, NULL);
	if (ret == pdPASS) {
		printf("Created LoraTask\r\n");
	}
	else {
		printf("Can not create LoraTask: %ld\r\n", ret);
	}
}

static void LoraTask(void *pvParam)
{
	printf("Start %s...\r\n", __func__);
	
	if (GetDeviceID() != 0x00) { // If this device id is non 0, it will work for Device role.
		vTaskDelay(3000);
		LoRa_TxStart();
	}
	
	while (1)
	{
		if (GetDeviceID() == 0x00) { // If this device id is 0, it will work for Gateway role.
			if (GetLoRaRxStartFlag()/* == TRUE*/) {
				SetLoRaRxStartFlag(0/*FALSE*/);
				LORAM_RX_Done();
				LoRa_RxDoneAction();
				LORAM_RX_Init();
			}
			
			vTaskDelay(50);
		}
		else { // If this device id is non 0, it will work for Device role.
			if (GetLoRaTxStartFlag()/* == TRUE*/) {
				SetLoRaTxStartFlag(0/*FALSE*/);
				LORAM_TX_Done();
				LoRa_TxDoneAction();
				LoRa_TxStart();
			}
			
			vTaskDelay(1000);
		}
	}
}
