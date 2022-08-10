#ifndef __WIFI_PACKET_H_
#define __WIFI_PACKET_H_

void WP_InitPacket(void);
void WP_UpdatePacket(RECEIVED_SENSOR_DATA* prsd);
uint8_t* WP_GetPacket(void);

#endif //__WIFI_PACKET_H_
