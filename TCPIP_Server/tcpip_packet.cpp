#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common.h"
#include "tcpip_packet.h"

CTCPIP_Packet::CTCPIP_Packet() {
    m_pBuffer = NULL;
}

CTCPIP_Packet::~CTCPIP_Packet() {
    if (m_pBuffer != NULL) {
        delete []m_pBuffer;
    }
}

int CTCPIP_Packet::Init(int nBufferSize)
{
    int nRet = 0;

    if (m_pBuffer != NULL || nBufferSize <= 0) {
        nRet = -1;
        return nRet;
    }

    m_pBuffer = new char[nBufferSize];
    memset(&m_pBuffer[0], 0, sizeof(m_pBuffer));

    return nRet;
}

char* CTCPIP_Packet::GetBuffer(void)
{
    return m_pBuffer;
}

int CTCPIP_Packet::Update(DEVICE_ID* pID, float* fTemp, float* fPres, float* fHum, unsigned long* nGas, unsigned short* nAir)
{
    int nRet = 0;
    int i;

    if (pID == NULL || fTemp == NULL || fPres == NULL || fHum == NULL || nGas == NULL || nAir == NULL) {
        nRet = -1;
        return nRet;
    }

    // For debug
    //printf("\n");
    //for (i=0; i<TCPIP_PACKET_SIZE; i++) {
    //    printf("%02x ", m_pBuffer[i]);
    //}
    //printf("\n");

    // Check header
    if (strncmp(&m_pBuffer[0], TCPIP_PACKET_HEADER, strlen(TCPIP_PACKET_HEADER)) != 0) {
        nRet = -2;
        return nRet;
    }

    for (i=0; i<TOTAL_SENSOR_COUNT/*m_pBuffer[4]*/; i++) {
        pID[i].string[0] = m_pBuffer[5 + (20 * i)];
        pID[i].string[1] = m_pBuffer[6 + (20 * i)];
        pID[i].string[2] = m_pBuffer[7 + (20 * i)];
        pID[i].string[3] = m_pBuffer[8 + (20 * i)];
        pID[i].string[4] = 0;
        
        unsigned short u16Temp;
        unsigned long u32Temp;

        u16Temp = 0;
        u16Temp = (unsigned short)(((m_pBuffer[9 + (20 * i)] << 8) & 0xFF00 | 
            ((m_pBuffer[10 + (20 * i)]) & 0x00FF)));
        fTemp[i] = (float)u16Temp / 100.0f;

        u32Temp = 0;
        u32Temp = (unsigned long)(((m_pBuffer[11 + (20 * i)] << 24) & 0xFF000000) | 
            ((m_pBuffer[12 + (20 * i)] << 16) & 0x00FF0000) | 
            ((m_pBuffer[13 + (20 * i)] << 8) & 0x0000FF00) | 
            ((m_pBuffer[14 + (20 * i)]) & 0x000000FF));
        fPres[i] = (float)u32Temp / 100.0f;

        u32Temp = 0;
        u32Temp = (unsigned long)(((m_pBuffer[15 + (20 * i)] << 24) & 0xFF000000) | 
            ((m_pBuffer[16 + (20 * i)] << 16) & 0x00FF0000) | 
            ((m_pBuffer[17 + (20 * i)] << 8) & 0x0000FF00) | 
            ((m_pBuffer[18 + (20 * i)]) & 0x000000FF));
        fHum[i] = (float)u32Temp / 1000.0f;
        
        u32Temp = 0;
        u32Temp = (unsigned long)(((m_pBuffer[19 + (20 * i)] << 24) & 0xFF000000) |
            ((m_pBuffer[20 + (20 * i)] << 16) & 0x00FF0000) |
            ((m_pBuffer[21 + (20 * i)] << 8) & 0x0000FF00) | 
            ((m_pBuffer[22 + (20 * i)]) & 0x000000FF));
        nGas[i] = u32Temp;

        u16Temp = 0;
        u16Temp = (unsigned short)(((m_pBuffer[23 + (20 * i)] << 8) & 0xFF00 | 
            ((m_pBuffer[24 + (20 * i)]) & 0x00FF)));
        nAir[i] = u16Temp;
    }

    return nRet;
}