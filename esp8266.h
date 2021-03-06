
/********************************************************
 * Author: Jacob Pagel
 * Date: 03/05/2015
 * esp8266.h (esp8266 wifi library include file)
 ********************************************************/

typedef unsigned char  u08;
typedef unsigned short u16;



extern unsigned int unIPD_id;

void espsend(u08 *SendData, int d);
void espstop(u08 *SendData);
void make_html(u08 *msg, u08 *data);
char *trimwhitespace(char *str);



/*
#ifndef _ESP8266_H_
#define _ESP8266_H_

//#include "config/USART3_Config.h"
//#include "stm32f10x_usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include "custom_libs/mytime.h"
//#include "globalDefines.h"
//#include "stm32f10x_gpio.h"
//#include "WebServer.h"
//#include "dht22.h"
//#include "bmp180.h"

//volatile uint8_t waitingForReponse;
//volatile uint8_t OKFound;
//volatile uint8_t ERRORFound;

//#define ESP_ResponseTimeout_ms 3000
//#define ESP_USART USART3


const typedef enum {ESP_RESPONSE_READY,ESP_RESPONSE_Link,ESP_RESPONSE_Unlink,ESP_RESPONSE_OK,
					ESP_RESPONSE_SEND_OK,ESP_RESPONSE_IPD,ESP_RESPONSE_ERROR,
					ESP_RESPONSE_Wrong_Syntax,ESP_RESPONSE_BUSY_P,ESP_RESPONSE_BUSY_INET}ESP_Messages;

typedef enum
{
	WIFI_CHECK_MODULE_CONNECTION = 0,
	WIFI_CURRENT_STATUS, //Get current status
	WIFI_AP_LIST, // Get available AP's
	WIFI_FIRMWARE_VERSION, // Get current firmware version on ESP8266
	WIFI_GET_CURRENT_MODE, // Gets the current mode (Access Point, Station, both)
	WIFI_SET_MODE_BOTH_AP_ST, // Set Mode as both, Access Point, Station
	WIFI_JOIN_NONYA,
	WIFI_SHOW_CURRENT_CONNECTED_AP,
	WIFI_MODULE_RESET,
	WIFI_SET_MULTICONNECTION,
	WIFI_SET_BAUD_115200,
	WIFI_START_LOCAL_SERVER_PORT_80,
	WIFI_GET_CURRENT_IP,
	WIFI_SHOW_ACTIVE_CONNECTIONS,
	WIFI_LIST_CONNECTED_DEVICES_inAPModeOnly,
	WIFI_QUIT_CURRENT_AP,
	WIFI_START_ACCESS_POINT,
	WIFI_DISABLE_ECHO,
	WIFI_CLOSE_CONNECTION
}Wifi_Commands;

typedef enum
{
	OPEN,
	WEP,
	WPA_PSK,
	WPA2_PSK,
	WPA_WPA2_PSK
}Available_Encyption;

//TYPEDEF DECLARATIONS
typedef struct{
	uint8_t ConnectionNum;
	char *DataSize;
	char *RequestType; //ie.. POST, GET, PUT, DELETE
	char *URI; //ie.. /api/foo?id=123
	char *Headers;
	char *Body;
	uint8_t Valid;
}IPD_Data;

typedef struct{
 char *AccessPoint_IP;
 char *AccessPoint_MAC;
 char *Station_IP;
 char *Station_MAC;

}ESP_Status;

extern const char *ATCommandsArray[19];

#define WIFI_COMMAND(commandEnum) (ATCommandsArray[(commandEnum)])


void Wifi_Init();
void Wifi_OFF();
void Wifi_ON();

void Wifi_ReadyWaitForAnswer();
void Wifi_WaitForAnswer();
void Wifi_WaitForAnswerCMD(char *cmdToWaitFor, uint16_t cmdSize);
void Wifi_WaitForAnswer_SEND_OK(uint16_t cmdSize);
void Wifi_CloseConnection(uint8_t connectionNum);
void Wifi_SendCustomCommand(char *customMessage);
void Wifi_SendCustomCommand_External_Wait(char *customMessage);
void Wifi_SendCommand(Wifi_Commands command );
void Wifi_CheckDMABuff_ForCIFSRData();
uint8_t Wifi_CheckDMABuff_ForReady();
IPD_Data Wifi_CheckDMABuff_ForIPDData(DHT22_Data *Current_DHT22_Reading, BMP180_Data *Current_BMP180_Reading);
void ConnectToAP(char *apName, char *password);
void StartLocalAP(char *SSID, char *password, uint8_t channel, Available_Encyption encypt);


#endif //_ESP8266_H_

*/
