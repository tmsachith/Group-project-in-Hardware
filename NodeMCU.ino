//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT
#include <ESP8266WiFi.h>

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "bimalA"
#define REMOTEXY_WIFI_PASSWORD ""
#define REMOTEXY_SERVER_PORT 6377

// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 98 bytes
{
    255,5,0,0,0,91,0,16,31,1,1,0,27,11,12,12,31,8,76,105,
    103,104,116,32,70,111,108,108,111,119,0,1,0,27,30,12,12,6,31,83,
    111,117,110,100,32,70,111,108,108,111,119,0,1,0,27,49,12,12,135,31,
    76,105,110,101,32,70,111,108,108,111,119,0,1,0,6,71,12,12,24,31,
    78,101,120,116,0,1,0,48,71,12,12,1,31,72,111,109,101,0
};

// this structure defines all the variables and events of your control interface 
struct {
    // input variables
    uint8_t button_1; // =1 if button pressed, else =0 
    uint8_t button_2; // =1 if button pressed, else =0 
    uint8_t button_3; // =1 if button pressed, else =0 
    uint8_t button_4; // =1 if button pressed, else =0 
    uint8_t button_5; // =1 if button pressed, else =0 

    // other variable
    uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

int value = 0;

void setup() {
    Serial.begin(9600);

    // RemoteXY initialization
    RemoteXY_Init();
}

void loop() {
    // RemoteXY communication
    RemoteXY_Handler();

    // Assign values based on button presses
    if (RemoteXY.button_1) {
        value = 10;
        Serial.println(value);
        delay(2000);
        value = 0;
    } else if (RemoteXY.button_2) {
        value = 20;
        Serial.println(value);
        delay(2000);
        value = 0;
    } else if (RemoteXY.button_3) {
        value = 30;
        Serial.println(value);
        delay(2000);
        value = 0;
    } else if (RemoteXY.button_4) {
        value = 40;
        Serial.println(value);
        delay(2000);
        value = 0;
    } else if (RemoteXY.button_5) {
        value = 50;
        Serial.println(value);
        delay(2000);
        value = 0;
    } 



}
