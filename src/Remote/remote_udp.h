#ifndef _REMOTE_UDP_H_
#define _REMOTE_UDP_H_
#include <WiFiUdp.h>
#include <Arduino.h>

class RemoteUDP {
public:
    RemoteUDP(const char* ssid, const char* password, unsigned int localPort);
    void begin();
    void handleIncomingPackets();
    void udp_printf(const char *format, ...);
    void udp_justfloat(uint8_t count, ...);
    uint8_t New_packet_flag;

private:
    const char* _ssid;
    const char* _password;
    unsigned int _localPort;
    WiFiUDP _udp;
    char _incomingPacket[255];
    void connectToWiFi();
    void parsePacket(uint8_t *buffer, int len);
};

typedef struct {
    uint8_t id;
    float value1;
    float value2;
} DecodedData;

extern DecodedData globalData_1; // id为01的结构体
extern DecodedData globalData_2; // id为02的结构体
extern DecodedData globalData_3; // id为03的结构体

#endif //_REMOTE_UDP_H_
