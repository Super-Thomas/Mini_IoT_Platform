#include <stdio.h>
#include <string.h>
#include <NuMicro.h>
#include "drivers_esensor.h"
#include "esensor.h"
#include "main.h"
#include "lora_packet.h"

uint8_t g_u8LoRaPacketData[LORA_PACKET_SIZE] = { 0, };

void LP_InitPacket(void)
{
	memset(&g_u8LoRaPacketData[0], 0, sizeof(g_u8LoRaPacketData));
}

void LP_UpdatePacket(RECEIVED_SENSOR_DATA* prsd)
{
	if (prsd == NULL) { // Update packet for send
		// Copy header
		memcpy(&g_u8LoRaPacketData[0], LORA_PACKET_HEADER, strlen(LORA_PACKET_HEADER));
		
		// device id
		g_u8LoRaPacketData[4] = 'd';
		g_u8LoRaPacketData[5] = 'e';
		g_u8LoRaPacketData[6] = 'v';
		g_u8LoRaPacketData[7] = 0x30 + GetDeviceID();
		
		// temperature
		g_u8LoRaPacketData[8] = (uint8_t)(((ESENS_Get_Temperature() & 0xFF00) >> 8) & 0xFF);
		g_u8LoRaPacketData[9] = (uint8_t)(((ESENS_Get_Temperature() & 0x00FF)) & 0xFF);
		
		// pressure
		g_u8LoRaPacketData[10] = (uint8_t)(((ESENS_Get_Pressure() & 0xFF000000) >> 24) & 0xFF);
		g_u8LoRaPacketData[11] = (uint8_t)(((ESENS_Get_Pressure() & 0x00FF0000) >> 16) & 0xFF);
		g_u8LoRaPacketData[12] = (uint8_t)(((ESENS_Get_Pressure() & 0x0000FF00) >> 8) & 0xFF);
		g_u8LoRaPacketData[13] = (uint8_t)(((ESENS_Get_Pressure() & 0x000000FF)) & 0xFF);
		
		// humidity
		g_u8LoRaPacketData[14] = (uint8_t)(((ESENS_Get_Humidity() & 0xFF000000) >> 24) & 0xFF);
		g_u8LoRaPacketData[15] = (uint8_t)(((ESENS_Get_Humidity() & 0x00FF0000) >> 16) & 0xFF);
		g_u8LoRaPacketData[16] = (uint8_t)(((ESENS_Get_Humidity() & 0x0000FF00) >> 8) & 0xFF);
		g_u8LoRaPacketData[17] = (uint8_t)(((ESENS_Get_Humidity() & 0x000000FF)) & 0xFF);
		
		// resistrance
		g_u8LoRaPacketData[18] = (uint8_t)(((ESENS_Get_Gas_Resistance() & 0xFF000000) >> 24) & 0xFF);
		g_u8LoRaPacketData[19] = (uint8_t)(((ESENS_Get_Gas_Resistance() & 0x00FF0000) >> 16) & 0xFF);
		g_u8LoRaPacketData[20] = (uint8_t)(((ESENS_Get_Gas_Resistance() & 0x0000FF00) >> 8) & 0xFF);
		g_u8LoRaPacketData[21] = (uint8_t)(((ESENS_Get_Gas_Resistance() & 0x000000FF)) & 0xFF);
		
		// air quality socre
		g_u8LoRaPacketData[22] = (uint8_t)(((ESENS_Get_Air_Quality_Score() & 0xFF00) >> 8) & 0xFF);
		g_u8LoRaPacketData[23] = (uint8_t)(((ESENS_Get_Air_Quality_Score() & 0x00FF)) & 0xFF);
		
		// For debug
		//printf("\nLoRa \n");
		//int i;
		//for (i=0; i<24; i++)
		//{
		//	printf("%02x ", g_u8LoRaPacketData[i]);
		//}
		//printf("\n");
	}
	else { // Update sensor data for received packet
		if (strncmp(&g_u8LoRaPacketData[0], LORA_PACKET_HEADER, strlen(LORA_PACKET_HEADER)) == 0) // Check header
		{
			prsd->u8DeviceID = g_u8LoRaPacketData[7] - 0x30;
			
			prsd->u8Temperature[0] = g_u8LoRaPacketData[8];
			prsd->u8Temperature[1] = g_u8LoRaPacketData[9];
			
			prsd->u8Pressure[0] = g_u8LoRaPacketData[10];
			prsd->u8Pressure[1] = g_u8LoRaPacketData[11];
			prsd->u8Pressure[2] = g_u8LoRaPacketData[12];
			prsd->u8Pressure[3] = g_u8LoRaPacketData[13];
			
			prsd->u8Humidity[0] = g_u8LoRaPacketData[14];
			prsd->u8Humidity[1] = g_u8LoRaPacketData[15];
			prsd->u8Humidity[2] = g_u8LoRaPacketData[16];
			prsd->u8Humidity[3] = g_u8LoRaPacketData[17];
			
			prsd->u8Resistrance[0] = g_u8LoRaPacketData[18];
			prsd->u8Resistrance[1] = g_u8LoRaPacketData[19];
			prsd->u8Resistrance[2] = g_u8LoRaPacketData[20];
			prsd->u8Resistrance[3] = g_u8LoRaPacketData[21];
			
			prsd->u8AirQualityScore[0] = g_u8LoRaPacketData[22];
			prsd->u8AirQualityScore[1] = g_u8LoRaPacketData[23];
			
			// For debug
			//printf("\nLoRa \n");
			//int i;
			//for (i=0; i<24; i++)
			//{
			//	printf("%02x ", g_u8LoRaPacketData[i]);
			//}
			//printf("\n");
		}
	}
}

uint8_t* LP_GetPacket(void)
{
	return &g_u8LoRaPacketData[0];
}
