#ifndef __WIFIMODULE_H_
#define __WIFIMODULE_H_

uint8_t WIFIM_QueueEmpty(void);
uint8_t WIFIM_QueueFull(void);
void WIFIM_QueueClear(void);
void WIFIM_QueuePush(uint8_t x);
uint8_t WIFIM_QueuePop(void);
int16_t WIFIM_QueueSize(void);
void WIFIM_Reset(void);
void WIFIM_Init(void);
void WIFIM_UartRx_Handle(void);
void WIFIM_Send(uint8_t* u8Buffer, uint32_t u32Length);
uint8_t WIFIM_CheckReady(void);
uint8_t WIFIM_CheckOK(void);

#endif //__WIFIMODULE_H_
