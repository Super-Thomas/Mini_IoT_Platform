#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "drivers_esensor.h"
#include "main.h"
#include "task_priority.h"
#include "esensor.h"

RECEIVED_SENSOR_DATA g_receivedSensorData;

static void SensorTask(void *pvParam);

RECEIVED_SENSOR_DATA* GetReceivedSensorData(void)
{
	return &g_receivedSensorData;
}

void SetReceivedSensorData(RECEIVED_SENSOR_DATA rsd)
{
	g_receivedSensorData = rsd;
}

void CreateSensorTask(void)
{
	BaseType_t ret;
	
	ret = xTaskCreate(SensorTask, "SensorTask", configMINIMAL_STACK_SIZE, (void *)NULL, SENSOR_TASK_PRIORITY, NULL);
	if (ret == pdPASS) {
		printf("Created SensorTask\r\n");
	}
	else {
		printf("Can not create SensorTask: %ld\r\n", ret);
	}
}

static void SensorTask(void *pvParam)
{
	printf("Start %s...\r\n", __func__);
	
	while (1)
	{
		//Lock();
		ESENS_Measure();
		//Unlock();
		vTaskDelay(50);
	}
}