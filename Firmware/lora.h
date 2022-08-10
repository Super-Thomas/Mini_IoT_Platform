#ifndef __LORA_H_
#define __LORA_H_

void SetLoRaTxStartFlag(uint8_t u8Value);
void SetLoRaRxStartFlag(uint8_t u8Value);
uint8_t GetLoRaTxStartFlag(void);
uint8_t GetLoRaRxStartFlag(void);
void CreateLoRaTask(void);

#endif //__LORA_H_
