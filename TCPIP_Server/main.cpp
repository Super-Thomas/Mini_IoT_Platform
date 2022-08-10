#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include "common.h"
#include "db_mysql.h"
#include "tcpip_packet.h"
#include "client.h"
#include "settings.h"
#include <vector>
using namespace std;

CDB_MySQL*          g_pMySQL = NULL; // MySQL
vector<CClient>     g_clientList;
pthread_mutex_t     g_mutexForClientList;

void ErrorMsg(const char* pstrMsg);
void Release(void);

void* ClientThread(void* iter_client)
{
    vector<CClient>::iterator iter = *(vector<CClient>::iterator *)iter_client;
    int nRet;

    while (1) {
        char buffer[256];
        // Read packet
        if ((nRet = read(iter->GetSocket(), iter->GetPacketBuffer(), TCPIP_PACKET_SIZE)) < 0) {
            // Read error
            printf("Client[%d]: %d\n", iter->GetSocket(), nRet);
            ErrorMsg("Read");
        }
        else {
            if (nRet == 0) { // Disconnect
                printf("Client[%d]: Disconnect\n", iter->GetSocket());

                pthread_mutex_lock(&g_mutexForClientList);
                vector<CClient>::iterator jter;
                // Find client you want to disconnect and then remove it in the client list
                for (jter=g_clientList.begin(); jter!=g_clientList.end(); jter++) {
                    if (jter->GetSocket() == iter->GetSocket())
                        break;
                }
                g_clientList.erase(jter);
                pthread_mutex_unlock(&g_mutexForClientList);

                pthread_exit((void *)0);
            }
            else { // Processing
                printf("Client[%d]: Processing\n", iter->GetSocket());

                DEVICE_ID deviceId[TOTAL_SENSOR_COUNT];
                float fTemp[TOTAL_SENSOR_COUNT];
                float fPres[TOTAL_SENSOR_COUNT];
                float fHum[TOTAL_SENSOR_COUNT];
                unsigned long nGas[TOTAL_SENSOR_COUNT];
                unsigned short nAir[TOTAL_SENSOR_COUNT];

                nRet = iter->GetPacket()->Update(&deviceId[0], &fTemp[0], &fPres[0], &fHum[0], &nGas[0], &nAir[0]);
                if (nRet != 0) {
                    printf("Client[%d]: Update %d\n", iter->GetSocket(), nRet);
                    ErrorMsg("Failed update");
                }

                nRet = g_pMySQL->WriteSensorData(&deviceId[0], &fTemp[0], &fPres[0], &fHum[0], &nGas[0], &nAir[0]);
                if (nRet != 0) {
                    printf("Client[%d]: Write sensor data %d\n", iter->GetSocket(), nRet);
                    ErrorMsg("Failed write sensor data");
                }
            }
        }
    }
}

void ErrorMsg(const char* pstrMsg)
{
    printf("[Error] %s\n", pstrMsg);
    Release();
    exit(1);
}

void Release(void)
{
    if (g_pMySQL != NULL) {
        delete g_pMySQL;
        g_pMySQL = NULL;
    }
}

int main(void)
{
    int socket_fd, client_fd;
    sockaddr_in server_addr, client_addr;
    int addr_len;

    printf("\r\n");
	printf("------------------------------------------------------\n");
	printf("TCP/IP Server\n");
	printf("------------------------------------------------------\n");

    // Init and connect to DB
    g_pMySQL = new CDB_MySQL;
    if (g_pMySQL->ConnectAndSelectDB((char *)DB_SERVER_IP, (char *)DB_SERVER_ID, (char *)DB_SERVER_PASSWORD, (char *)DB_NAME) != 0) {
        ErrorMsg("Can not connect to DB");
        return -1;
    }

    // Create socket for server
    if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        ErrorMsg("Can not create socket for server");
        return -1;
    }
 
    // Server setting
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(TCPIP_SERVER_PORT);
 
    // Bind
    if (bind(socket_fd, (sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ErrorMsg("Bind");
        return -1;
    }

    // Listen
    if (listen(socket_fd, 10) < 0) {
        ErrorMsg("Listen");
        return -1;
    }

    pthread_mutex_init(&g_mutexForClientList, NULL);
    
    while (1)
    {
        CClient client;

        // Accept
        addr_len = sizeof(client_addr);
        client_fd = accept(socket_fd, (sockaddr *)&client_addr, (socklen_t *)&addr_len);
        if (client_fd < 0) {
            ErrorMsg("New client socket is wrong");
            return -1;
        }

        pthread_mutex_lock(&g_mutexForClientList);
        g_clientList.push_back(client);

        vector<CClient>::iterator iter = g_clientList.end() - 1;
        iter->SetSocket(client_fd);
        // Create thread for new client
        if (pthread_create(iter->GetThreadHandle(), NULL, ClientThread, (void *)&iter) < 0) {
            ErrorMsg("Can not create thread for new client");
            return -1;
        }
        pthread_mutex_unlock(&g_mutexForClientList);

        printf("Welcome for new client! socket=%d thread=%ld\n", iter->GetSocket(), *(iter->GetThreadHandle()));
    }

    Release();
 
    return 0;
}
