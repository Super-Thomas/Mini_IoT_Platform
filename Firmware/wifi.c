#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "drivers_wifi.h"
#include "esensor.h"
#include "common.h"
#include "main.h"
#include "settings.h"
#include "wifi_packet.h"
#include "task_priority.h"
#include "wifi.h"

typedef enum {
	WS_UNKNOW = 0,
	WS_NOT_CONNECTED_TO_AP,
	WS_CONNECTED_TO_AP,
	WS_NOT_CONNECTED_TO_SERVER,
	WS_CONNECTED_TO_SERVER
} WIFI_STATUS;

static WIFI_STATUS g_wifiStatus = WS_UNKNOW;

static void WifiTask(void *pvParam);
uint8_t ConnectToAP(uint8_t* pAPName, uint8_t* pAPPasswd);
uint8_t ConnectToServer(uint8_t* pServerIP, uint32_t u32Port);

uint8_t ConnectToAP(uint8_t* pAPName, uint8_t* pAPPasswd)
{
	uint8_t u8Ret = 0;
	uint8_t u8Command[64] = { 0, };
	
	printf("[%s] Connect To AP... ", __FILE__);
	
	WIFIM_QueueClear();
	WIFIM_Send((uint8_t *)"AT+CWMODE=1\r\n", 14);
	u8Ret = WIFIM_CheckOK();
	WIFIM_QueueClear();
	if (u8Ret != 0) {
		printf("[Failed]\n");
		return u8Ret;
	}
	
	vTaskDelay(1000); // 1000 ms delay
	
	WIFIM_QueueClear();
	sprintf((char *)&u8Command[0], "AT+CWJAP=\"%s\",\"%s\"\r\n", pAPName, pAPPasswd);
	WIFIM_Send(&u8Command[0], strlen((char *)u8Command));
	u8Ret = WIFIM_CheckOK();
	WIFIM_QueueClear();
	if (u8Ret != 0) {
		printf("[Failed]\n");
		return u8Ret;
	}

	printf("[OK]\n");
	
	return 0;
}

uint8_t ConnectToServer(uint8_t* pServerIP, uint32_t u32Port)
{
	uint8_t u8Ret = 0;
	uint8_t u8Command[64] = { 0, };
	
	printf("[%s] Connect To Server... ", __FILE__);
	
	WIFIM_QueueClear();
	sprintf((char *)&u8Command[0], "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", pServerIP, u32Port);
	WIFIM_Send(&u8Command[0], strlen((char *)u8Command));
	u8Ret = WIFIM_CheckOK();
	if (u8Ret != 0) {
		printf("[Failed]\n");
		return u8Ret;
	}
	
	printf("[OK]\n");
	
	return 0;
}

uint8_t SendWiFiPacket(void)
{
	uint8_t u8Ret = 0;
	uint8_t u8Command[64] = { 0, };
	
	printf("[%s] Send Wi-Fi Packet... ", __FILE__);
	
	WP_UpdatePacket(GetReceivedSensorData()); // Update Wi-Fi packet
	
	WIFIM_QueueClear();
	sprintf((char *)&u8Command[0], "AT+CIPSEND=%d\r\n", TCPIP_PACKET_SIZE);
	WIFIM_Send(&u8Command[0], strlen((char *)u8Command));
	u8Ret = WIFIM_CheckOK();
	if (u8Ret != 0) {
		printf("[Failed]\n");
		return u8Ret;
	}
	
	WIFIM_QueueClear();
	WIFIM_Send(WP_GetPacket(), TCPIP_PACKET_SIZE);
	u8Ret = WIFIM_CheckOK();
	if (u8Ret != 0) {
		printf("[Failed]\n");
		return u8Ret;
	}
	
	printf("[OK]\n");
	
	return 0;
}

void CreateWiFiTask(void)
{
	BaseType_t ret;
	
	ret = xTaskCreate(WifiTask, "WifiTask", configMINIMAL_STACK_SIZE, (void *)NULL, WIFI_TASK_PRIORITY, NULL);
	if (ret == pdPASS) {
		printf("Created WifiTask\r\n");
	}
	else {
		printf("Can not create WifiTask: %ld\r\n", ret);
	}
}

static void WifiTask(void *pvParam)
{
	uint8_t u8Ret = 0;
	
	printf("Start %s...\r\n", __func__);
	
	while (1)
	{
		switch (g_wifiStatus)
		{
			case WS_UNKNOW:
				g_wifiStatus = WS_NOT_CONNECTED_TO_AP;
				WIFIM_QueueClear();
				WIFIM_Reset();
				vTaskDelay(50);
				break;
			
			case WS_NOT_CONNECTED_TO_AP:
				u8Ret = ConnectToAP((uint8_t *)WIFI_AP_NAME, (uint8_t *)WIFI_AP_PASSWORD);
				if (u8Ret == 0)
					g_wifiStatus = WS_CONNECTED_TO_AP;
				else
					g_wifiStatus = WS_NOT_CONNECTED_TO_AP;
				vTaskDelay(3000);
				break;
				
			case WS_CONNECTED_TO_AP:
			case WS_NOT_CONNECTED_TO_SERVER:
				if (ConnectToServer((uint8_t *)TCPIP_SERVER_IP, (uint32_t)TCPIP_SERVER_PORT) == 0)
					g_wifiStatus = WS_CONNECTED_TO_SERVER;
				else
					g_wifiStatus = WS_NOT_CONNECTED_TO_SERVER;
				vTaskDelay(3000);
				break;
				
			case WS_CONNECTED_TO_SERVER:
				if (SendWiFiPacket() != 0)
					g_wifiStatus = WS_NOT_CONNECTED_TO_SERVER;
				vTaskDelay(3000);
				break;
				
			default:
				g_wifiStatus = WS_UNKNOW;
				vTaskDelay(50);
				break;
		}
	}
}
