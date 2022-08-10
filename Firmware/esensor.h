#ifndef __ESENSOR_H_
#define __ESENSOR_H_

typedef struct tagReceived_Sensor_Data {
	uint8_t u8DeviceID;
	uint8_t u8Temperature[2];
	uint8_t u8Pressure[4];
	uint8_t u8Humidity[4];
	uint8_t u8Resistrance[4];
	uint8_t u8AirQualityScore[2];
} RECEIVED_SENSOR_DATA;

RECEIVED_SENSOR_DATA* GetReceivedSensorData(void);
void SetReceivedSensorData(RECEIVED_SENSOR_DATA rsd);
void CreateSensorTask(void);

#endif //__ESENSOR_H_
