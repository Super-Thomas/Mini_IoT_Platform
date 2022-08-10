#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common.h"
#include "tcpip_packet.h"
#include "client.h"

CClient::CClient() {
    m_nSocketFd = -1;
    m_pPacket = new CTCPIP_Packet;
    m_pPacket->Init(TCPIP_PACKET_SIZE);
}

CClient::~CClient() {
    if (m_pPacket != NULL) {
        delete m_pPacket;
        m_pPacket = NULL;
    }
}

int CClient::SetSocket(int nNewSocketFd)
{
    m_nSocketFd = nNewSocketFd;
    return m_nSocketFd;
}

int CClient::GetSocket(void)
{
    return m_nSocketFd;
}

pthread_t* CClient::GetThreadHandle(void)
{
    return &m_handle;
}

char* CClient::GetPacketBuffer(void)
{
    return m_pPacket->GetBuffer();
}

CTCPIP_Packet* CClient::GetPacket(void)
{
    return m_pPacket;
}