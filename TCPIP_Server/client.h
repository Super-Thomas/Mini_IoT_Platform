#ifndef __CLIENT_H_
#define __CLIENT_H_

class CTCPIP_Packet;

class CClient {
private:
    pthread_t m_handle;
    int m_nSocketFd;
    CTCPIP_Packet* m_pPacket;

public:
     CClient();
    ~CClient();

    int SetSocket(int nNewSocketFd);
    int GetSocket(void);
    pthread_t* GetThreadHandle(void);
    char* GetPacketBuffer(void);
    CTCPIP_Packet* GetPacket(void);
};

#endif //__CLIENT_H_