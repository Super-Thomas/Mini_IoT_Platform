#ifndef __TCPIP_PACKET_H_
#define __TCPIP_PACKET_H_

class CTCPIP_Packet {
private:
    char* m_pBuffer;

public:
     CTCPIP_Packet();
    ~CTCPIP_Packet();

    int Init(int nBufferSize);
    char* GetBuffer(void);
    int Update(DEVICE_ID* pID, float* fTemp, float* fPres, float* fHum, unsigned long* nGas, unsigned short* nAir);
};

#endif //__TCPIP_PACKET_H_