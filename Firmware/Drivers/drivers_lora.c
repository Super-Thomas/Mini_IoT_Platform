/*
	Drivers for SX1276
	Thanks to EC_ML51_LoRa_Control_V1.00(https://www.nuvoton.com/resource-download.jsp?tp_GUID=EC0120200103114211)
*/
#include <stdio.h>
#include <string.h>
#include <NuMicro.h>
#include "esensor.h"
#include "lora_packet.h"
#include "drivers_lora.h"

unsigned char SpiInOut(unsigned char outData);
void SX1276Init(void);
void SX1276InitIo(void);
void SX1276Reset(void);
void SX1276SetReset(unsigned char state);
void SX1276SetLoRaOn(void);
void SX1276LoRaSetOpMode(unsigned char opMode);
void SX1276Write(unsigned char addr, unsigned char data_in);
void SX1276WriteBuffer(unsigned char addr, unsigned char *buffer, unsigned char size_in);
void SX1276Read(unsigned char addr, unsigned char *data_in);
void SX1276ReadBuffer(unsigned char addr, unsigned char *buffer, unsigned char size_in);
void SX1276LoRaInit(void);
void SX1276LoRaSetDefaults(void);
void SX1276LoRaSetRFFrequency(unsigned long int freq);
void SX1276LoRaSetSpreadingFactor(unsigned char factor);
void SX1276LoRaSetErrorCoding(unsigned char value);
void SX1276LoRaSetPacketCrcOn(unsigned char enable);
void SX1276LoRaSetSignalBandwidth(unsigned char bw);
void SX1276LoRaSetImplicitHeaderOn(unsigned char enable);
void SX1276LoRaSetSymbTimeout(unsigned short int value);
void SX1276LoRaSetPayloadLength(unsigned char value);
void SX1276LoRaSetPreambleLength(unsigned short int value);
void SX1276LoRaSetLowDatarateOptimize(unsigned char enable);
void SX1276LoRaSetPAOutput(unsigned char outputPin);
void SX1276LoRaSetPa20dBm(unsigned char enale);
void SX1276LoRaSetRFPower(char power);
void SX1276LoRaSetNbTrigPeaks(unsigned char value);
void SX1276StartRx(void);
void SX1276LoRaSetRFState(unsigned char state);
void SX1276LoRaGetRxPacket(void *buffer, unsigned short int *size_in);
void SX1276LoRaSetTxPacket(const void *buffer, unsigned short int size_in);
void SX1276WriteFifo(unsigned char *buffer, unsigned char size_in);
void SX1276ReadFifo(unsigned char *buffer, unsigned char size_in);
double SX1276LoRaGetPacketRssi(void);

#define SPI_SS0_PIN																	PH7

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164.0
#define RSSI_OFFSET_HF                              -157.0

/*!
 * RF packet definition
 */
#define RF_BUFFER_SIZE_MAX                          256
#define RF_BUFFER_SIZE                              256

/*!
 * SX1276 LoRa General parameters definition
 */
typedef struct sLoRaSettings
{
    unsigned long int RFFrequency;
    unsigned char Power;
    unsigned char SignalBw;                   // LORA [0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
    // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
    unsigned char SpreadingFactor;            // LORA [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    unsigned char ErrorCoding;                // LORA [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    unsigned char CrcOn;                         // [0: OFF, 1: ON]
    unsigned char ImplicitHeaderOn;              // [0: OFF, 1: ON]
    unsigned char RxSingleOn;                    // [0: Continuous, 1 Single]
    unsigned char FreqHopOn;                     // [0: OFF, 1: ON]
    unsigned char HopPeriod;                  // Hops every frequency hopping period symbols
    unsigned long int TxPacketTimeout;
    unsigned long int RxPacketTimeout;
    unsigned char PayloadLength;
    unsigned char PreambleLength;               //Added by Joe
} tLoRaSettings;

typedef struct sSX1276LR
{
    unsigned char RegFifo;                                // 0x00
    // Common settings
    unsigned char RegOpMode;                              // 0x01
    unsigned char RegRes02;                               // 0x02
    unsigned char RegRes03;                               // 0x03
    unsigned char RegBandSetting;                         // 0x04
    unsigned char RegRes05;                               // 0x05
    unsigned char RegFrfMsb;                              // 0x06
    unsigned char RegFrfMid;                              // 0x07
    unsigned char RegFrfLsb;                              // 0x08
    // Tx settings
    unsigned char RegPaConfig;                            // 0x09
    unsigned char RegPaRamp;                              // 0x0A
    unsigned char RegOcp;                                 // 0x0B
    // Rx settings
    unsigned char RegLna;                                 // 0x0C
    // LoRa registers
    unsigned char RegFifoAddrPtr;                         // 0x0D
    unsigned char RegFifoTxBaseAddr;                      // 0x0E
    unsigned char RegFifoRxBaseAddr;                      // 0x0F
    unsigned char RegFifoRxCurrentAddr;                   // 0x10
    unsigned char RegIrqFlagsMask;                        // 0x11
    unsigned char RegIrqFlags;                            // 0x12
    unsigned char RegNbRxBytes;                           // 0x13
    unsigned char RegRxHeaderCntValueMsb;                 // 0x14
    unsigned char RegRxHeaderCntValueLsb;                 // 0x15
    unsigned char RegRxPacketCntValueMsb;                 // 0x16
    unsigned char RegRxPacketCntValueLsb;                 // 0x17
    unsigned char RegModemStat;                           // 0x18
    unsigned char RegPktSnrValue;                         // 0x19
    unsigned char RegPktRssiValue;                        // 0x1A
    unsigned char RegRssiValue;                           // 0x1B
    unsigned char RegHopChannel;                          // 0x1C
    unsigned char RegModemConfig1;                        // 0x1D
    unsigned char RegModemConfig2;                        // 0x1E
    unsigned char RegSymbTimeoutLsb;                      // 0x1F
    unsigned char RegPreambleMsb;                         // 0x20
    unsigned char RegPreambleLsb;                         // 0x21
    unsigned char RegPayloadLength;                       // 0x22
    unsigned char RegMaxPayloadLength;                    // 0x23
    unsigned char RegHopPeriod;                           // 0x24
    unsigned char RegFifoRxByteAddr;                      // 0x25
    unsigned char RegModemConfig3;                        // 0x26
    unsigned char RegTestReserved27;                      // 0x27
    unsigned char RegFeiMsb;                              // 0x28
    unsigned char RegFeiMib;                              // 0x29
    unsigned char RegFeiLsb;                              // 0x2A
    unsigned char RegTestReserved2B[0x30 - 0x2B + 1];        // 0x2B-0x30
    unsigned char RegDetectOptimize;                      // 0x31
    unsigned char RegTestReserved32;                      // 0x32
    unsigned char RegInvertIQ;                            // 0x33
    unsigned char RegTestReserved34[0x36 - 0x34 + 1];        // 0x34-0x36
    unsigned char RegDetectionThreshold;                  // 0x37
    unsigned char RegTestReserved38[0x3F - 0x38 + 1];        // 0x38-0x3F
    // I/O settings
    unsigned char RegDioMapping1;                         // 0x40
    unsigned char RegDioMapping2;                         // 0x41
    // Version
    unsigned char RegVersion;                             // 0x42
    // Test
    unsigned char RegTestReserved43;                      // 0x43
    // Additional settings
    unsigned char RegPllHop;                              // 0x44
    // Test
    unsigned char RegTestReserved45[0x4A - 0x45 + 1];        // 0x45-0x4A
    // Additional settings
    unsigned char RegTcxo;                                // 0x4B
    // Test
    unsigned char RegTestReserved4C;                      // 0x4C
    // Additional settings
    unsigned char RegPaDac;                               // 0x4D
    // Test
    unsigned char RegTestReserved4E[0x5A - 0x4E + 1];        // 0x4E-0x5A
    // Additional settings
    unsigned char RegFormerTemp;                          // 0x5B
    // Test
    unsigned char RegTestReserved5C;                      // 0x5C
    // Additional settings
    unsigned char RegBitrateFrac;                         // 0x5D
    // Additional settings
    unsigned char RegTestReserved5E[0x60 - 0x5E + 1];        // 0x5E-0x60
    // Additional settings
    unsigned char RegAgcRef;                              // 0x60
    unsigned char RegAgcThresh1;                          // 0x61
    unsigned char RegAgcThresh2;                          // 0x62
    unsigned char RegAgcThresh3;                          // 0x63
    // Test
    unsigned char RegTestReserved64[0x70 - 0x64 + 1];        // 0x64-0x70
} tSX1276LR;

/*!
 * SX1276 LoRa registers variable
 */
tSX1276LR *SX1276LR;

/*!
 * Local RF buffer for communication support
 */
static unsigned char RFBuffer[RF_BUFFER_SIZE];

/*!
 * Tx management support variables
 */
static unsigned int TxPacketSize = 0;

/*!
 * Rx management support variables
 */
static unsigned int RxPacketSize = 0;
static char RxPacketSnrEstimate;
static double RxPacketRssiValue;
static unsigned char RxGain = 1;

static unsigned char LoRaOnState = FALSE;

/*!
 * RF state machine variable
 */
static unsigned char RFLRState = RFLR_STATE_IDLE;

/*!
 * SX1276 registers variable
 */
unsigned char SX1276Regs[0x70] = {0};

unsigned char F_RxStart = FALSE;
unsigned char F_TxStart = FALSE;

/*!
 * Frequency hopping frequencies table
 */
const unsigned long int HoppingFrequencies[] =
{
    470500000
};

tLoRaSettings LoRaSettings =
{
    470500000,        // RFFrequency
    17,               // Power
    7,                // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
    // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
    10,               // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    1,                // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    1,                // CrcOn [0: OFF, 1: ON]
    0,                // ImplicitHeaderOn [0: OFF, 1: ON]
    0,                // RxSingleOn [0: Continuous, 1 Single]
    0,                // FreqHopOn [0: OFF, 1: ON]
    4,                // HopPeriod Hops every frequency hopping period symbols
    100,              // TxPacketTimeout (default, data will be update by TimeOutSetting())
    500,              // RxPacketTimeout (the same as TxPacketTimeout)
    LORA_PACKET_SIZE,                // PayloadLength
    12,               // PreambleLength     //Added by Joe
};

void LORAM_Init(void)
{
	SX1276Init();
  SX1276StartRx();
	LORAM_RX_Init();
}

void SX1276Init(void)
{
	SX1276LR = (tSX1276LR *)SX1276Regs;
	SX1276InitIo();
	SPI_Open(SPI1, SPI_MASTER, SPI_MODE_0, 8, 1000000);
	SX1276Reset();
	SX1276SetLoRaOn();
	// Initialize LoRa modem
	SX1276LoRaInit();
}

unsigned char LORAM_ReadID(void)
{
	unsigned char u8Data = 0;

	SX1276Read(0x42, &u8Data);
	
	return u8Data;
}

void SX1276InitIo(void)
{
	// Unlock protected registers
  SYS_UnlockReg();
	
	// Enable SPI1 clock
	CLK->APBCLK0 |= CLK_APBCLK0_SPI1CKEN_Msk;
	
	// Switch UART4 clock source to HIRC
  CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_HIRC;
	
	// Set PH multi-function pin for SPI1
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH7MFP_Msk | SYS_GPH_MFPL_PH6MFP_Msk | SYS_GPH_MFPL_PH5MFP_Msk | SYS_GPH_MFPL_PH4MFP_Msk)) | 
		(SYS_GPH_MFPL_PH7MFP_GPIO | SYS_GPH_MFPL_PH6MFP_SPI1_CLK | SYS_GPH_MFPL_PH5MFP_SPI1_MOSI | SYS_GPH_MFPL_PH4MFP_SPI1_MISO);
	
	// Set PF multi-function pin for GPIO
	SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF6MFP_Msk | SYS_GPF_MFPL_PF7MFP_Msk)) | (SYS_GPF_MFPL_PF6MFP_GPIO | SYS_GPF_MFPL_PF7MFP_GPIO);
	SYS->GPF_MFPH = (SYS->GPF_MFPH & ~(SYS_GPF_MFPH_PF8MFP_Msk | SYS_GPF_MFPH_PF9MFP_Msk | SYS_GPF_MFPH_PF10MFP_Msk | SYS_GPF_MFPH_PF11MFP_Msk)) | 
		(SYS_GPF_MFPH_PF8MFP_GPIO | SYS_GPF_MFPH_PF9MFP_GPIO | SYS_GPF_MFPH_PF10MFP_GPIO | SYS_GPF_MFPH_PF11MFP_GPIO);
	
	SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE9MFP_Msk)) | (SYS_GPE_MFPH_PE9MFP_GPIO);
	
	SYS->GPD_MFPH = (SYS->GPD_MFPH & ~(SYS_GPD_MFPH_PD12MFP_Msk)) | (SYS_GPD_MFPH_PD12MFP_GPIO);
	
	// Lock protected registers
  SYS_LockReg();
	
	GPIO_SetMode(PH, BIT7, GPIO_MODE_OUTPUT);
	GPIO_SetPullCtl(PH, BIT7, GPIO_PUSEL_PULL_DOWN);
	
	// DIO0
	GPIO_SetMode(PF, BIT7, GPIO_MODE_INPUT);
	GPIO_SetPullCtl(PF, BIT7, GPIO_PUSEL_PULL_DOWN);
	GPIO_EnableInt(PF, 7, GPIO_INT_RISING);
	NVIC_EnableIRQ(GPF_IRQn);
	
	// DIO1
	GPIO_SetMode(PF, BIT8, GPIO_MODE_INPUT);
	GPIO_SetPullCtl(PF, BIT8, GPIO_PUSEL_PULL_DOWN);
	
	// DIO2
	GPIO_SetMode(PF, BIT9, GPIO_MODE_INPUT);
	GPIO_SetPullCtl(PF, BIT9, GPIO_PUSEL_PULL_DOWN);
	
	// DIO3
	GPIO_SetMode(PF, BIT10, GPIO_MODE_INPUT);
	GPIO_SetPullCtl(PF, BIT10, GPIO_PUSEL_PULL_DOWN);
	
	// DIO4
	GPIO_SetMode(PD, BIT12, GPIO_MODE_INPUT);
	GPIO_SetPullCtl(PD, BIT12, GPIO_PUSEL_PULL_DOWN);
	
	// DIO5
	GPIO_SetMode(PF, BIT11, GPIO_MODE_INPUT);
	GPIO_SetPullCtl(PF, BIT11, GPIO_PUSEL_PULL_DOWN);
}

void SX1276Reset(void)
{
    SX1276SetReset(0);
    TIMER_Delay(TIMER0, 1000);
    SX1276SetReset(1);
		TIMER_Delay(TIMER0, 6000);
}

void SX1276SetReset(unsigned char state)
{
    if (state)
    {
			// Reset
			GPIO_SetMode(PF, BIT6, GPIO_MODE_OUTPUT);
			GPIO_SetPullCtl(PF, BIT6, GPIO_PUSEL_DISABLE);
			PF6 = 1;
    }
    else
    {
			GPIO_SetMode(PF, BIT6, GPIO_MODE_OUTPUT);
			GPIO_SetPullCtl(PF, BIT6, GPIO_PUSEL_DISABLE);
			PF6 = 0;
    }
}

void SX1276SetLoRaOn(void)
{
    if (LoRaOnState == 1)
    {
        return;
    }

    LoRaOnState = 1;

    SX1276LoRaSetOpMode(RFLR_OPMODE_SLEEP);

    SX1276LR->RegOpMode = (SX1276LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON;
    SX1276Write(REG_LR_OPMODE, SX1276LR->RegOpMode);

    SX1276LoRaSetOpMode(RFLR_OPMODE_STANDBY);
    //                         RxDone                     RxTimeout                  FhssChangeChannel          CadDone
    SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
    //                         CadDetected                ModeReady
    SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
    SX1276WriteBuffer(REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2);

    SX1276ReadBuffer(REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1);
}

void SX1276LoRaSetOpMode(unsigned char opMode)
{
    static unsigned char opModePrev = RFLR_OPMODE_STANDBY;
    opModePrev = SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;

    if (opMode != opModePrev)
    {
        SX1276LR->RegOpMode = (SX1276LR->RegOpMode & RFLR_OPMODE_MASK) | opMode;

        SX1276Write(REG_LR_OPMODE, SX1276LR->RegOpMode);
    }
}

unsigned char SpiInOut(unsigned char outData)
{
	SPI_WRITE_TX(SPI1, outData);
	while(SPI_IS_BUSY(SPI1));
	
	return SPI_READ_RX(SPI1);
}

void SX1276Write(unsigned char addr, unsigned char data_in)
{
    SX1276WriteBuffer(addr, &data_in, 1);
}

void SX1276WriteBuffer(unsigned char addr, unsigned char *buffer, unsigned char size_in)
{
    unsigned char i;
	
		SPI_SS0_PIN = 0;
    SpiInOut(addr | 0x80);
	
    for (i = 0; i < size_in; i++)
    {
			SpiInOut(buffer[i]);
    }
		
		SPI_SS0_PIN = 1;
}

void SX1276Read(unsigned char addr, unsigned char *data_in)
{
    SX1276ReadBuffer(addr, data_in, 1);
}

void SX1276ReadBuffer(unsigned char addr, unsigned char *buffer, unsigned char size_in)
{
    unsigned char i;

    SPI_SS0_PIN = 0;
    SpiInOut(addr & 0x7F);

    for (i = 0; i < size_in; i++)
    {
			buffer[i] = SpiInOut(0);
    }

    SPI_SS0_PIN = 1;
}

void SX1276LoRaInit(void)
{
    RFLRState = RFLR_STATE_IDLE;

    SX1276LoRaSetDefaults();

    SX1276ReadBuffer(REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1);

    SX1276LR->RegLna = RFLR_LNA_GAIN_G1;

    SX1276WriteBuffer(REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1);

    // set the RF settings
    SX1276LoRaSetRFFrequency(LoRaSettings.RFFrequency);
    SX1276LoRaSetSpreadingFactor(LoRaSettings.SpreadingFactor);   // SF6 only operates in implicit header mode.
    SX1276LoRaSetErrorCoding(LoRaSettings.ErrorCoding);
    SX1276LoRaSetPacketCrcOn(LoRaSettings.CrcOn);
    SX1276LoRaSetSignalBandwidth(LoRaSettings.SignalBw);

    SX1276LoRaSetImplicitHeaderOn(LoRaSettings.ImplicitHeaderOn);
    SX1276LoRaSetSymbTimeout(0x3FF);
    SX1276LoRaSetPayloadLength(LoRaSettings.PayloadLength);
    SX1276LoRaSetPreambleLength(LoRaSettings.PreambleLength);    //Added by Joe
    SX1276LoRaSetLowDatarateOptimize(TRUE);

    SX1276LoRaSetPAOutput(RFLR_PACONFIG_PASELECT_PABOOST);
    SX1276LoRaSetPa20dBm(TRUE);
    SX1276LoRaSetRFPower(LoRaSettings.Power);

    SX1276LoRaSetOpMode(RFLR_OPMODE_STANDBY);
}

void SX1276LoRaSetDefaults(void)
{
    // REMARK: See SX1276 datasheet for modified default values.
    SX1276Read(REG_LR_VERSION, &SX1276LR->RegVersion);
}

void SX1276LoRaSetRFFrequency(unsigned long int freq)
{
    LoRaSettings.RFFrequency = freq;

    freq = (unsigned long int)((signed long int)freq / (double)FREQ_STEP);
    SX1276LR->RegFrfMsb = (unsigned char)((freq >> 16) & 0xFF);
    SX1276LR->RegFrfMid = (unsigned char)((freq >> 8) & 0xFF);
    SX1276LR->RegFrfLsb = (unsigned char)(freq & 0xFF);
    SX1276WriteBuffer(REG_LR_FRFMSB, &SX1276LR->RegFrfMsb, 3);
}

void SX1276LoRaSetSpreadingFactor(unsigned char factor)
{

    if (factor > 12)
    {
        factor = 12;
    }
    else if (factor < 6)
    {
        factor = 6;
    }

    if (factor == 6)
    {
        SX1276LoRaSetNbTrigPeaks(5);
    }
    else
    {
        SX1276LoRaSetNbTrigPeaks(3);
    }

    SX1276Read(REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2);
    SX1276LR->RegModemConfig2 = (SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK) | (factor << 4);
    SX1276Write(REG_LR_MODEMCONFIG2, SX1276LR->RegModemConfig2);
    LoRaSettings.SpreadingFactor = factor;
}

void SX1276LoRaSetErrorCoding(unsigned char value)
{
    SX1276Read(REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1);
    SX1276LR->RegModemConfig1 = (SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK) | (value << 1);
    SX1276Write(REG_LR_MODEMCONFIG1, SX1276LR->RegModemConfig1);
    LoRaSettings.ErrorCoding = value;
}

void SX1276LoRaSetPacketCrcOn(unsigned char enable)
{
    SX1276Read(REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2);
    SX1276LR->RegModemConfig2 = (SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) | (enable << 2);
    SX1276Write(REG_LR_MODEMCONFIG2, SX1276LR->RegModemConfig2);
    LoRaSettings.CrcOn = enable;
}

void SX1276LoRaSetSignalBandwidth(unsigned char bw)
{
    SX1276Read(REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1);
    SX1276LR->RegModemConfig1 = (SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK) | (bw << 4);
    SX1276Write(REG_LR_MODEMCONFIG1, SX1276LR->RegModemConfig1);
    LoRaSettings.SignalBw = bw;
}

void SX1276LoRaSetImplicitHeaderOn(unsigned char enable)
{
    SX1276Read(REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1);
    SX1276LR->RegModemConfig1 = (SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) | (enable);
    SX1276Write(REG_LR_MODEMCONFIG1, SX1276LR->RegModemConfig1);
    LoRaSettings.ImplicitHeaderOn = enable;
}
void SX1276LoRaSetSymbTimeout(unsigned short int value)
{
    SX1276ReadBuffer(REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2, 2);

    SX1276LR->RegModemConfig2 = (SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) | ((value >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK);
    SX1276LR->RegSymbTimeoutLsb = value & 0xFF;
    SX1276WriteBuffer(REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2, 2);
}

void SX1276LoRaSetPayloadLength(unsigned char value)
{
    SX1276LR->RegPayloadLength = value;
    SX1276Write(REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength);
    LoRaSettings.PayloadLength = value;
}

void SX1276LoRaSetPreambleLength(unsigned short int value)
{
    SX1276ReadBuffer(REG_LR_PREAMBLEMSB, &SX1276LR->RegPreambleMsb, 2);

    SX1276LR->RegPreambleMsb = (value >> 8) & 0x00FF;
    SX1276LR->RegPreambleLsb = value & 0xFF;
    SX1276WriteBuffer(REG_LR_PREAMBLEMSB, &SX1276LR->RegPreambleMsb, 2);
}

void SX1276LoRaSetLowDatarateOptimize(unsigned char enable)
{
    SX1276Read(REG_LR_MODEMCONFIG3, &SX1276LR->RegModemConfig3);
    SX1276LR->RegModemConfig3 = (SX1276LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) | (enable << 3);
    SX1276Write(REG_LR_MODEMCONFIG3, SX1276LR->RegModemConfig3);
}

void SX1276LoRaSetPAOutput(unsigned char outputPin)
{
    SX1276Read(REG_LR_PACONFIG, &SX1276LR->RegPaConfig);
    SX1276LR->RegPaConfig = (SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK) | outputPin;
    SX1276Write(REG_LR_PACONFIG, SX1276LR->RegPaConfig);
}

void SX1276LoRaSetPa20dBm(unsigned char enale)
{
    SX1276Read(REG_LR_PADAC, &SX1276LR->RegPaDac);
    SX1276Read(REG_LR_PACONFIG, &SX1276LR->RegPaConfig);

    if ((SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST) == RFLR_PACONFIG_PASELECT_PABOOST)
    {
        if (enale == TRUE)
        {
            SX1276LR->RegPaDac = 0x87;
        }
    }
    else
    {
        SX1276LR->RegPaDac = 0x84;
    }

    SX1276Write(REG_LR_PADAC, SX1276LR->RegPaDac);
}

void SX1276LoRaSetRFPower(char power)
{
    SX1276Read(REG_LR_PACONFIG, &SX1276LR->RegPaConfig);
    SX1276Read(REG_LR_PADAC, &SX1276LR->RegPaDac);

    if ((SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST) == RFLR_PACONFIG_PASELECT_PABOOST)
    {
        if ((SX1276LR->RegPaDac & 0x87) == 0x87)
        {
            if (power < 5)
            {
                power = 5;
            }

            if (power > 20)
            {
                power = 20;
            }

            SX1276LR->RegPaConfig = (SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK) | 0x70;
            SX1276LR->RegPaConfig = (SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK) | (unsigned char)((unsigned short int)(power - 5) & 0x0F);
        }
        else
        {
            if (power < 2)
            {
                power = 2;
            }

            if (power > 17)
            {
                power = 17;
            }

            SX1276LR->RegPaConfig = (SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK) | 0x70;
            SX1276LR->RegPaConfig = (SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK) | (unsigned char)((unsigned short int)(power - 2) & 0x0F);
        }
    }
    else
    {
        if (power < -1)
        {
            power = -1;
        }

        if (power > 14)
        {
            power = 14;
        }

        SX1276LR->RegPaConfig = (SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK) | 0x70;
        SX1276LR->RegPaConfig = (SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK) | (unsigned char)((unsigned short int)(power + 1) & 0x0F);
    }

    SX1276Write(REG_LR_PACONFIG, SX1276LR->RegPaConfig);
    LoRaSettings.Power = power;
}

void SX1276LoRaSetNbTrigPeaks(unsigned char value)
{
    SX1276Read(0x31, &SX1276LR->RegDetectOptimize);
    SX1276LR->RegDetectOptimize = (SX1276LR->RegDetectOptimize & 0xF8) | value;
    SX1276Write(0x31, SX1276LR->RegDetectOptimize);
}

void SX1276StartRx(void)
{
    SX1276LoRaSetRFState(RFLR_STATE_RX_INIT);
}

void SX1276LoRaSetRFState(unsigned char state)
{
    RFLRState = state;
}

void LORAM_RX_Init(void)
{
    SX1276LoRaSetOpMode(RFLR_OPMODE_STANDBY);

    SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                //RFLR_IRQFLAGS_RXDONE |
                                //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                RFLR_IRQFLAGS_VALIDHEADER |
                                RFLR_IRQFLAGS_TXDONE |
                                RFLR_IRQFLAGS_CADDONE |
                                //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                RFLR_IRQFLAGS_CADDETECTED;
    SX1276Write(REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask);

    if (LoRaSettings.FreqHopOn == TRUE)
    {
        SX1276LR->RegHopPeriod = LoRaSettings.HopPeriod;

        SX1276Read(REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel);
        SX1276LoRaSetRFFrequency(HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK]);
    }
    else
    {
        SX1276LR->RegHopPeriod = 255;
    }

    SX1276Write(REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod);

    //                         RxDone                     RxTimeout                  FhssChangeChannel          CadDone
    SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
    //                         CadDetected                ModeReady
    SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
    SX1276WriteBuffer(REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2);

    if (LoRaSettings.RxSingleOn == TRUE)   // Rx single mode
    {

        SX1276LoRaSetOpMode(RFLR_OPMODE_RECEIVER_SINGLE);
    }
    else // Rx continuous mode
    {
        SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;
        SX1276Write(REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr);

        SX1276LoRaSetOpMode(RFLR_OPMODE_RECEIVER);
    }

    memset(RFBuffer, 0, (size_t)RF_BUFFER_SIZE);

    RFLRState = RFLR_STATE_RX_RUNNING;
}

void LORAM_TX_Init(void)
{
    SX1276LoRaSetOpMode(RFLR_OPMODE_STANDBY);

    if (LoRaSettings.FreqHopOn == TRUE)
    {
        SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    //RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1276LR->RegHopPeriod = LoRaSettings.HopPeriod;

        SX1276Read(REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel);
        SX1276LoRaSetRFFrequency(HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK]);
    }
    else
    {
        SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    //RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1276LR->RegHopPeriod = 0;
    }

    SX1276Write(REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod);
    SX1276Write(REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask);

    // Initializes the payload size
    SX1276LR->RegPayloadLength = TxPacketSize;
    SX1276Write(REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength);

    SX1276LR->RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
    SX1276Write(REG_LR_FIFOTXBASEADDR, SX1276LR->RegFifoTxBaseAddr);

    SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoTxBaseAddr;
    SX1276Write(REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr);

    // Write payload buffer to LORA modem
    SX1276WriteFifo(RFBuffer, SX1276LR->RegPayloadLength);
    //                         TxDone                     RxTimeout                  FhssChangeChannel          ValidHeader
    SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
    //                         PllLock                    Mode Ready
    SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
    SX1276WriteBuffer(REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2);

    SX1276LoRaSetOpMode(RFLR_OPMODE_TRANSMITTER);

    RFLRState = RFLR_STATE_TX_RUNNING;
}

void SX1276LoRaGetRxPacket(void *buffer, unsigned short int *size_in)
{
    *size_in = RxPacketSize;
    RxPacketSize = 0;
    memcpy((void *)buffer, (void *)RFBuffer, (size_t)*size_in);
}

void SX1276LoRaSetTxPacket(const void *buffer, unsigned short int size_in)
{
    TxPacketSize = size_in;
    memcpy((void *)RFBuffer, buffer, (size_t)TxPacketSize);
    RFLRState = RFLR_STATE_TX_INIT;
}

void SX1276WriteFifo(unsigned char *buffer, unsigned char size_in)
{
    SX1276WriteBuffer(0, buffer, size_in);
}

void SX1276ReadFifo(unsigned char *buffer, unsigned char size_in)
{
    SX1276ReadBuffer(0, buffer, size_in);
}

unsigned char SX1276LoRaGetRFState(void)
{
    return RFLRState;
}

void LORAM_RX_Done(void)
{
    if (LoRaSettings.FreqHopOn == TRUE)
    {
        SX1276Read(REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel);
        SX1276LoRaSetRFFrequency(HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK]);
    }

    // Clear Irq
    SX1276Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

    SX1276Read(REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags);

    if ((SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR) == RFLR_IRQFLAGS_PAYLOADCRCERROR)
    {
        // Clear Irq
        SX1276Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);
        RFLRState = RFLR_STATE_RX_RUNNING;
    }
    else
    {
        unsigned char rxSnrEstimate;
        SX1276Read(REG_LR_PKTSNRVALUE, &rxSnrEstimate);

        if (rxSnrEstimate & 0x80)   // The SNR sign bit is 1
        {
            // Invert and divide by 4
            RxPacketSnrEstimate = ((~rxSnrEstimate + 1) & 0xFF) >> 2;
            RxPacketSnrEstimate = -RxPacketSnrEstimate;
        }
        else
        {
            // Divide by 4
            RxPacketSnrEstimate = (rxSnrEstimate & 0xFF) >> 2;
        }

        SX1276Read(REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue);

        if (LoRaSettings.RFFrequency < 860000000)   // LF
        {
            if (RxPacketSnrEstimate < 0)
            {
                RxPacketRssiValue = (double)RSSI_OFFSET_LF + ((double)SX1276LR->RegPktRssiValue) + RxPacketSnrEstimate;
            }
            else
            {
                RxPacketRssiValue = (double)RSSI_OFFSET_LF + (1.0666 * ((double)SX1276LR->RegPktRssiValue));
            }
        }
        else                                        // HF
        {
            if (RxPacketSnrEstimate < 0)
            {
                RxPacketRssiValue = (double)RSSI_OFFSET_HF + ((double)SX1276LR->RegPktRssiValue) + RxPacketSnrEstimate;
            }
            else
            {
                RxPacketRssiValue = (double)RSSI_OFFSET_HF + (1.0666 * ((double)SX1276LR->RegPktRssiValue));
            }
        }

        // Rx continuous mode
        SX1276Read(REG_LR_FIFORXCURRENTADDR, &SX1276LR->RegFifoRxCurrentAddr);

        if (LoRaSettings.ImplicitHeaderOn == TRUE)
        {
            RxPacketSize = SX1276LR->RegPayloadLength;
            SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
            SX1276Write(REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr);
            SX1276ReadFifo(RFBuffer, SX1276LR->RegPayloadLength);
        }
        else
        {
            SX1276Read(REG_LR_NBRXBYTES, &SX1276LR->RegNbRxBytes);
            RxPacketSize = SX1276LR->RegNbRxBytes;
            SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
            SX1276Write(REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr);
            SX1276ReadFifo(RFBuffer, SX1276LR->RegNbRxBytes);
        }

        RFLRState = RFLR_STATE_RX_RUNNING;
    }
}

void LORAM_GetRxPacket(void *buffer, unsigned short int *size_in)
{
    SX1276LoRaGetRxPacket(buffer, size_in);
}

double LORAM_GetPacketRssi(void)
{
    return SX1276LoRaGetPacketRssi();
}

double SX1276LoRaGetPacketRssi(void)
{
    return RxPacketRssiValue;
}

void LORAM_SetTxPacket(const void *buffer, unsigned short int size_in)
{
    SX1276LoRaSetTxPacket(buffer, size_in);
}

void LORAM_TX_Done(void)
{
    SX1276Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
}
