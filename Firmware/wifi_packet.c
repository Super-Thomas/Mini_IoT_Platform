#include <stdio.h>
#include <string.h>
#include <NuMicro.h>
#include "drivers_esensor.h"
#include "esensor.h"
#include "main.h"
#include "common.h"
#include "wifi_packet.h"

uint8_t g_u8WiFiPacketData[TCPIP_PACKET_SIZE] = { 0, };

void WP_InitPacket(void)
{
	memset(&g_u8WiFiPacketData[0], 0, sizeof(g_u8WiFiPacketData));
}

void WP_UpdatePacket(RECEIVED_SENSOR_DATA* prsd)
{
	uint8_t i;
	
	// Copy header
	memcpy(&g_u8WiFiPacketData[0], TCPIP_PACKET_HEADER, strlen(TCPIP_PACKET_HEADER));
	g_u8WiFiPacketData[4] = TOTAL_SENSOR_COUNT; // Sensor count
	
	for (i=0; i<TOTAL_SENSOR_COUNT; i++)
	{
		// device id
		g_u8WiFiPacketData[5 + (20 * i)] = 'd';
		g_u8WiFiPacketData[6 + (20 * i)] = 'e';
		g_u8WiFiPacketData[7 + (20 * i)] = 'v';
		g_u8WiFiPacketData[8 + (20 * i)] = i == 0 ? 0x30 + GetDeviceID() : 0x30 + prsd->u8DeviceID;
		
		// temperature
		g_u8WiFiPacketData[9 + (20 * i)] 	= i == 0 ? (uint8_t)(((ESENS_Get_Temperature() & 0xFF00) >> 8) & 0xFF) : prsd->u8Temperature[0];
		g_u8WiFiPacketData[10 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Temperature() & 0x00FF)) & 0xFF) : prsd->u8Temperature[1];
		
		// pressure
		g_u8WiFiPacketData[11 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Pressure() & 0xFF000000) >> 24) & 0xFF) : prsd->u8Pressure[0];
		g_u8WiFiPacketData[12 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Pressure() & 0x00FF0000) >> 16) & 0xFF) : prsd->u8Pressure[1];
		g_u8WiFiPacketData[13 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Pressure() & 0x0000FF00) >> 8) & 0xFF) : prsd->u8Pressure[2];
		g_u8WiFiPacketData[14 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Pressure() & 0x000000FF)) & 0xFF) : prsd->u8Pressure[3];
		
		// humidity
		g_u8WiFiPacketData[15 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Humidity() & 0xFF000000) >> 24) & 0xFF) : prsd->u8Humidity[0];
		g_u8WiFiPacketData[16 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Humidity() & 0x00FF0000) >> 16) & 0xFF) : prsd->u8Humidity[1];
		g_u8WiFiPacketData[17 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Humidity() & 0x0000FF00) >> 8) & 0xFF) : prsd->u8Humidity[2];
		g_u8WiFiPacketData[18 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Humidity() & 0x000000FF)) & 0xFF) : prsd->u8Humidity[3];
		
		// resistrance
		g_u8WiFiPacketData[19 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Gas_Resistance() & 0xFF000000) >> 24) & 0xFF) : prsd->u8Resistrance[0];
		g_u8WiFiPacketData[20 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Gas_Resistance() & 0x00FF0000) >> 16) & 0xFF) : prsd->u8Resistrance[1];
		g_u8WiFiPacketData[21 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Gas_Resistance() & 0x0000FF00) >> 8) & 0xFF) : prsd->u8Resistrance[2];
		g_u8WiFiPacketData[22 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Gas_Resistance() & 0x000000FF)) & 0xFF) : prsd->u8Resistrance[3];
		
		// air quality socre
		g_u8WiFiPacketData[23 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Air_Quality_Score() & 0xFF00) >> 8) & 0xFF) : prsd->u8AirQualityScore[0];
		g_u8WiFiPacketData[24 + (20 * i)] = i == 0 ? (uint8_t)(((ESENS_Get_Air_Quality_Score() & 0x00FF)) & 0xFF) : prsd->u8AirQualityScore[1];
	}
	
	// For debug
  //printf("\n");
  //for (i=0; i<TCPIP_PACKET_SIZE; i++) {
	//	printf("%02x ", g_u8WiFiPacketData[i]);
  //}
  //printf("\n");
}

uint8_t* WP_GetPacket(void)
{
	return &g_u8WiFiPacketData[0];
}