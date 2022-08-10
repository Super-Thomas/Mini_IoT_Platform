#ifndef __COMMON_H_
#define __COMMON_H_

#define TCPIP_PACKET_SIZE 			(45)
#define TCPIP_PACKET_HEADER		  "SENS"

#define TOTAL_SENSOR_COUNT      (2)

typedef struct tagDevice_ID {
    char string[5];
} DEVICE_ID;

#endif //__COMMON_H_
