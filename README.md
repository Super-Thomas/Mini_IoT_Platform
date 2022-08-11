# Mini_IoT_Platform

## Overview
There have two sensor devices. One device will work for Gateway role. Another one device will work for only sensor role. Each device have environment sensor and can get air-quality data. Device for only sensor role will send air-quality data to Device for Gateway role via LoRa. Device for Gateway role will send air-quality data to TCP/IP server via Wi-Fi. TCP/IP server will processing air-quality data for send query to Database(MySQL). PHP web page will read air-quality data and draw charts using Google Charts.

![image](https://user-images.githubusercontent.com/99227045/184053585-e00711ea-8840-4ca0-b125-fc4e2a2181ad.png)

If you have Linux server(MySQL, Apache, TCP/IP server) and two NuMaker-IoT-M264A boards, You can try to operate Mini IoT platform using this project.

## Development environment
### Hardware
|Name|Description|Note|
|:------|:---|:---|
|NuMaker-IoT-M253A V1.1|Development board with ARM Cortex-M23 provided by Nuvoton|[Link](https://www.nuvoton.com/products/iot-solution/iot-platform/numaker-iot-m263a/)|
### Software
|Name|Description|Note|
|:------|:---|:---|
|Firmware|Based SDK for M261 series provided by Nuvoton|Version 3.00.003|
|Keil MDK|IDE for Firmware|uVision V5.34.0.0|
|Ubuntu|Linux OS|Version 18.04.6|
|g++|Compiler for TCP/IP Server|Version 7.5.0|
|MySQL|Database|Version 14.14|
|PHP|Script for Web page|Version 7.2.24|

## Settings
You can modify values for IoT Platform.

Include/settings.h
```c
#define WIFI_AP_NAME        "Your AP Name"
#define WIFI_AP_PASSWORD    "Your AP Password"

#define TCPIP_SERVER_IP     "Your Server IP"
#define TCPIP_SERVER_PORT   3360 // You can change it

#define DB_SERVER_IP        "127.0.0.1"
#define DB_SERVER_ID        "Your MySQL Account ID"
#define DB_SERVER_PASSWORD  "Your MySQL Account Password"
#define DB_NAME             "Your Database Name"
```

PHP_Webpage/index.php
```php
// connect to mysql
$db = mysqli_connect('Your MySQL IP', 'Your MySQL Account ID', 'Your MySQL Account Password', 'Your DB Name'); 

```

## Firmware
Device will read GPIO(PB7 to PB4) when boot-up. Read value will be set to Device ID value. If Device ID is 0, Device will start work as Gateway role. If Device ID is non 0, Device will start work as only sensor role. Sensor device will send Air-quality data to Gateway via LoRa. Gateway device will send Air-quality data to TCP/IP Server via Wi-Fi. You need to modify information of Server and AP in "settings.h" file.

## TCPIP_Server
TCP/IP Server will save Air-quality data to MySQL(Database) after receive Air-quality data from Gateway. You need to modify information of Server in "settings.h" file.

## MySQL(Database)
You must have the following table in your database:

Table Name: sensor_table
|Columns Name|idx|temp|pres|hum|gas|score|id|regtime|
|---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|Data Type|int(11)|float|float|float|int(11)|Tinyint(4)|varchar(32)|datetime|
|Note|Pri. key||||||||

## PHP_Webpage
If you built web server and PHP, you can access PHP file and view charts of air-quality data using web browser. You need to modify information of Server in "index.php" file.

![image](https://user-images.githubusercontent.com/99227045/184070533-fbb92330-5cf6-4487-929a-1e83de99bc9b.png)
