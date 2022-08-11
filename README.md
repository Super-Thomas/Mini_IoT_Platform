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
