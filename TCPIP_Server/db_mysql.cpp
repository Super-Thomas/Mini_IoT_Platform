#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "common.h"
#include "db_mysql.h"

CDB_MySQL::CDB_MySQL() {
    m_pMysqlConn = NULL;
}

CDB_MySQL::~CDB_MySQL() {
    DisconnectDB();
}

int CDB_MySQL::ConnectAndSelectDB(char* pIP, char* pID, char* pPassword, char* pDBName)
{
    int nRet = 0;

    if (m_pMysqlConn != NULL || pIP == NULL || pID == NULL || pPassword == NULL || pDBName == NULL) {
        nRet = -1;
        return nRet;
    }

    m_pMysqlConn = mysql_init(NULL);
    if (!mysql_real_connect(m_pMysqlConn, pIP, pID, pPassword, NULL, 0, NULL, 0)) {
        nRet = -2; // Connect error
        return nRet;
    }
    else {
        if (mysql_select_db(m_pMysqlConn, pDBName)) {
            nRet = -3; // Select DB error
            return nRet;
        }
    }

    pthread_mutex_init(&m_mutex, NULL);

    return nRet;
}

int CDB_MySQL::DisconnectDB(void)
{
    int nRet = 0;

    if (m_pMysqlConn == NULL) {
        nRet = -1;
        return nRet;
    }

    mysql_close(m_pMysqlConn);
    m_pMysqlConn = NULL;

    return nRet;
}

int CDB_MySQL::WriteSensorData(DEVICE_ID* pID, float* fTemp, float* fPres, float* fHum, unsigned long* nGas, unsigned short* nAir)
{
    int nRet = 0;
    int i;
    MYSQL_RES* pResult;
    MYSQL_ROW row;
    char strQuery[256] = { 0, };
    char strDate[32] = { 0, };

    if (m_pMysqlConn == NULL || pID == NULL || fTemp == NULL || fPres == NULL || fHum == NULL || nGas == NULL || nAir == NULL) {
        nRet = -1;
        return nRet;
    }

    // Get Date
    memset(&strQuery[0], 0, sizeof(strQuery));
    sprintf(strQuery, "select now() as DATE");
    pthread_mutex_lock(&m_mutex);
    if (mysql_query(m_pMysqlConn, strQuery) == 0) {
	    pResult = mysql_store_result(m_pMysqlConn);
	    while ((row = mysql_fetch_row(pResult)) != NULL) {
            strcpy(strDate, row[0]);
        }
	    mysql_free_result(pResult);
    }
    else {
	    pthread_mutex_unlock(&m_mutex);
        nRet = -2; // Query for Get Date failed
        return nRet;
    }
    pthread_mutex_unlock(&m_mutex);

    // Write Sensor Data
    for (i=0; i<TOTAL_SENSOR_COUNT; i++) {
	    memset(&strQuery[0], 0, sizeof(strQuery));
	    sprintf(strQuery, "insert into sensor_table (temp, pres, hum, gas, score, id, regtime) value (%.2f, %.2f, %.2f, %ld, %d, \"%s\", \"%s\");", 
            fTemp[i], fPres[i], fHum[i], nGas[i], nAir[i], pID[i].string, strDate);
	    
        pthread_mutex_lock(&m_mutex);
	    if (mysql_query(m_pMysqlConn, strQuery))
	    {
		    pthread_mutex_unlock(&m_mutex);
            nRet = -3; // Query for Insert Data failed
            return nRet;
	    }
	    pthread_mutex_unlock(&m_mutex);
    }

    return nRet;
}