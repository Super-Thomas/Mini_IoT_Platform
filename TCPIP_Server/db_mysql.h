#ifndef __DB_MYSQL_H_
#define __DB_MYSQL_H_

#include <mysql.h>

class CDB_MySQL {
private:
    MYSQL* m_pMysqlConn;
    pthread_mutex_t m_mutex;

public:
     CDB_MySQL();
    ~CDB_MySQL();

    int ConnectAndSelectDB(char* pIP, char* pID, char* pPassword, char* pDBName);
    int DisconnectDB(void);
    int WriteSensorData(DEVICE_ID* pID, float* fTemp, float* fPres, float* fHum, unsigned long* nGas, unsigned short* nAir);
};

#endif //__DB_MYSQL_H_