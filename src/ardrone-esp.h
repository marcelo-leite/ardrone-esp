#ifndef ARDRONE_ESP_H 
#define ARDRONE_ESP_H

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <IPAddress.h>
#include <WiFiUdp.h>

#include <ardata.h>

#define NUM_MAX_ARGS  8
const IPAddress drone(192, 168, 1, 1);


typedef struct {
  String type;
  String cmdargs[NUM_MAX_ARGS];

} ATCommand;

class ArdroneESP{

  private: 
    // Temp Variables
    unsigned int sequence;          // Current client-side sequence number
    unsigned int lNav;           // Last sequence number received from AR Drone
    unsigned long lSend;          // millis since last command packet sent
    unsigned long lReceive;      // millis since last nav packet received
    unsigned long littlecounter;

    // Variables Comunication
    int navPort;
    int atPort;
    int videoPort;
    char stream[10000];
    WiFiUDP NavUdp;
    WiFiUDP AT;
    WiFiClient Video;
    String ssid;
    
    // Buffer Capture Data
    char incoming[4096];
    char block[2048];
  
    // ARdrone Flight Data
    navdata_t navdata; // Complete Data
    ardata_t ardata;   // Someone Data
    uint16_t *id;      
    uint16_t *siz;

  public:
    ArdroneESP(void);
    ardata_t getArdata(void){ return this->ardata; };
    void videoStream(void);
    void ardroneConnect(void);

    // Comunication and Transmission
    void serializeAT(ATCommand command, int Nargs);
    void sendPacket(String command);
    void captureNavdata(void);
    void showNavdata(void);

    // Command ARDrone
    void refCommand(bool takeoff, bool emergency);
    void pcmdCommand(bool mode, float vel[4]);
    void ftrimCommand(void);
    void comwdgCommand(void);
    void ledCommand(void);
    void calibCommand(void);
    void pwmCommand(int pwm[4]);
    void configCommand(String key, String value);

    // Base Actions
    void land(void);
    void takeoff(void);
    void emergency(void);
};

#endif