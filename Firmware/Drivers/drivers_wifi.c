/*
	Drivers for ESP-12F
*/
#include <stdio.h>
#include <string.h>
#include <NuMicro.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "drivers_wifi.h"

#define WIFIM_PU								(PE12 = 1) // Power-Up Wi-Fi Module using GPIO
#define WIFIM_PD								(PE12 = 0) // Power-Down Wi-Fi Module using GPIO
#define WIFIM_RX_BUFFER_SIZE		512
#define WIFIM_TIMEOUT						15000

volatile static uint8_t g_u8WifiRxBuf[WIFIM_RX_BUFFER_SIZE] = { 0, };
volatile static int16_t g_s16WifiRxHead  = 0;
volatile static int16_t g_s16WifiRxTail  = 0;

uint8_t WIFIM_QueueEmpty(void)
{
	return (g_s16WifiRxHead == g_s16WifiRxTail);
}

uint8_t WIFIM_QueueFull(void)
{
	return (((g_s16WifiRxTail + 1) % WIFIM_RX_BUFFER_SIZE) == g_s16WifiRxHead);
}

void WIFIM_QueueClear(void)
{
	memset((char *)&g_u8WifiRxBuf[0], 0, sizeof(g_u8WifiRxBuf));
	g_s16WifiRxHead = 0;
	g_s16WifiRxTail = 0;
}

void WIFIM_QueuePush(uint8_t x)
{
	if (!WIFIM_QueueFull()) {
		g_s16WifiRxTail = ((g_s16WifiRxTail + 1) % WIFIM_RX_BUFFER_SIZE);
		g_u8WifiRxBuf[g_s16WifiRxTail] = x;
	}
}

uint8_t WIFIM_QueuePop(void)
{
	if (!WIFIM_QueueEmpty()) {
		g_s16WifiRxHead = ((g_s16WifiRxHead + 1) % WIFIM_RX_BUFFER_SIZE);
		return g_u8WifiRxBuf[g_s16WifiRxHead];
	}

	return 0;
}

int16_t WIFIM_QueueSize(void)
{
	if (g_s16WifiRxHead > g_s16WifiRxTail)
		return (g_s16WifiRxHead - g_s16WifiRxTail);
	else if (g_s16WifiRxHead < g_s16WifiRxTail)
		return (g_s16WifiRxTail - g_s16WifiRxHead);
	
	return 0;
}

void WIFIM_Reset(void)
{
	WIFIM_PD; // WiFi_RST set low
	TIMER_Delay(TIMER0, 100000);
	WIFIM_PU; // WiFi_RST set high
	TIMER_Delay(TIMER0, 100000);
}

void WIFIM_Init(void)
{
	// Unlock protected registers
  SYS_UnlockReg();
	
	// Enable UART4 clock
	CLK->APBCLK0 |= CLK_APBCLK0_UART4CKEN_Msk;
	
	// Switch UART4 clock source to HIRC
  CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_UART4SEL_Msk)) | CLK_CLKSEL3_UART4SEL_HIRC;
	
	// Set PE multi-function pin for UART4
	SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE13MFP_Msk)) | (SYS_GPE_MFPH_PE13MFP_UART4_nRTS);
	
	// Set PC multi-function pins for UART4
	SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC7MFP_Msk)) | (SYS_GPC_MFPL_PC6MFP_UART4_RXD | SYS_GPC_MFPL_PC7MFP_UART4_TXD);
	SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC8MFP_Msk)) | (SYS_GPC_MFPH_PC8MFP_UART4_nCTS);
	
	// Set PE multi-function pin for WiFi_RST
	SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE12MFP_Msk)) | (SYS_GPE_MFPH_PE12MFP_GPIO);
	
	// WiFi_RST set ouput
	GPIO_SetMode(PE, BIT12, GPIO_MODE_OUTPUT);
	WIFIM_Reset();
	
	// Lock protected registers
  SYS_LockReg();
	
	UART_Open(UART4, 115200);
	
	memset((char *)&g_u8WifiRxBuf[0], 0, sizeof(g_u8WifiRxBuf));
	g_s16WifiRxHead  = 0;
	g_s16WifiRxTail  = 0;
	
	// Enable RX interrupt
	UART_EnableInt(UART4, (UART_INTEN_RDAIEN_Msk));
}

void WIFIM_UartRx_Handle(void)
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UART4->INTSTS;

    // Receive Data Available Interrupt Handle    
    if(u32IntSts & UART_INTSTS_RDAINT_Msk) {
        // Get all the input characters
        while(UART_IS_RX_READY(UART4)) {          
            // Receive Line Status Error Handle 
            if(u32IntSts & UART_INTSTS_RLSINT_Msk) {                
                // Clear Receive Line Status Interrupt
                UART_ClearIntFlag(UART4, UART_INTSTS_RLSINT_Msk);   
            }              
            
            // Get the character from UART Buffer
            u8InChar = (uint8_t)UART_READ(UART4);

            WIFIM_QueuePush(u8InChar);
        }
    }

    // Buffer Error Interrupt Handle    
    if (u32IntSts & UART_INTSTS_BUFERRINT_Msk) {
        // Clear Buffer Error Interrupt
        UART_ClearIntFlag(UART4, UART_INTSTS_BUFERRINT_Msk);             
    }
}

void WIFIM_Send(uint8_t* u8Buffer, uint32_t u32Length)
{
	UART_Write(UART4, &u8Buffer[0], u32Length);
}

uint8_t WIFIM_CheckReady(void)
{
	uint8_t u8Buffer = 0, u8Done = 0, u8Counter = 0;
	uint32_t u32Timeout = 0;
	
	while (!u8Done) {
		if (WIFIM_QueueSize() > 0) {
			u8Buffer = WIFIM_QueuePop();
			
			switch (u8Counter) {
				case 0:
					if (u8Buffer == 'r')
						u8Counter++;
					break;
				case 1:
					if (u8Buffer == 'e')
						u8Counter++;
					else
						u8Counter = 0;
					break;
				case 2:
					if (u8Buffer == 'a')
						u8Counter++;
					else
						u8Counter = 0;
					break;
				case 3:
					if (u8Buffer == 'd')
						u8Counter++;
					else
						u8Counter = 0;
					break;
				case 4:
					if (u8Buffer == 'y')
						u8Done = 1;
					else
						u8Counter = 0;
					break;
			}
		}
		
		u32Timeout++;
		vTaskDelay(1); // 1 ms delay
		if (u32Timeout > WIFIM_TIMEOUT)
			return 1;
	}
	
	return 0;
}

uint8_t WIFIM_CheckOK(void)
{
	uint8_t u8Buffer = 0, u8Done = 0, u8Counter = 0;
	uint32_t u32Timeout = 0;
	
	while (!u8Done) {
		if (WIFIM_QueueSize() > 0)
		{
			u8Buffer = WIFIM_QueuePop();
			
			switch (u8Counter) {
				case 0:
					if (u8Buffer == 'O')
						u8Counter++;
					else if(u8Buffer == 'C')
						u8Counter++;
					break;
				case 1:
					if (u8Buffer == 'K')
						u8Done = 1;
					else if(u8Buffer == 'L')
						u8Counter++;
					else
						u8Counter = 0;
					break;
				case 2:
					if (u8Buffer == 'O')
						u8Counter++;
					else
						u8Counter = 0;
					break;
				case 3:
					if (u8Buffer == 'S')
						u8Counter++;
					else
						u8Counter = 0;
					break;
				case 4:
					if (u8Buffer == 'E')
						u8Counter++;
					else
						u8Counter = 0;
					break;
				case 5:
					if (u8Buffer == 'D')
						return 2;
					else
						u8Counter = 0;
					break;
			}
		}
		
		u32Timeout++;
		vTaskDelay(1); // 1 ms delay
		if (u32Timeout > WIFIM_TIMEOUT)
			return 1;
	}
	
	return 0;
}
