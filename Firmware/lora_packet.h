#ifndef __LORA_PACKET_H_
#define __LORA_PACKET_H_

#define LORA_PACKET_SIZE 			(24)
#define LORA_PACKET_HEADER		"SENS"

#define TOTAL_DEVICE_COUNT		1

void LP_InitPacket(void);
void LP_UpdatePacket(RECEIVED_SENSOR_DATA* prsd);
uint8_t* LP_GetPacket(void);

#endif //__LORA_PACKET_H_
