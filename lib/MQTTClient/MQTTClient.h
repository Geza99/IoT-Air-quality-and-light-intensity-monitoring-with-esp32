/*
  MQTTClient.h - A simple client for MQTT.
  Nicholas O'Leary
  http://knolleary.net

  Ported to mbed by Zoltan Hudak <hudakz@outlook.com>
*/
#ifndef MQTTClient_h
#define MQTTClient_h

#include "mbed.h"
#include "TcpClient.h"

#define MQTT_MAX_PACKET_SIZE     256    // MQTT_MAX_PACKET_SIZE : Maximum packet size
#define MQTT_KEEPALIVE            15    // MQTT_KEEPALIVE : keepAlive interval in Seconds
#define MQTTPROTOCOLVERSION        3
#define MQTTCONNECT           1 << 4    // Client request to connect to Server
#define MQTTCONNACK           2 << 4    // Connect Acknowledgment
#define MQTTPUBLISH           3 << 4    // Publish message
#define MQTTPUBACK            4 << 4    // Publish Acknowledgment
#define MQTTPUBREC            5 << 4    // Publish Received (assured delivery part 1)
#define MQTTPUBREL            6 << 4    // Publish Release (assured delivery part 2)
#define MQTTPUBCOMP           7 << 4    // Publish Complete (assured delivery part 3)
#define MQTTSUBSCRIBE         8 << 4    // Client Subscribe request
#define MQTTSUBACK            9 << 4    // Subscribe Acknowledgment
#define MQTTUNSUBSCRIBE      10 << 4    // Client Unsubscribe request
#define MQTTUNSUBACK         11 << 4    // Unsubscribe Acknowledgment
#define MQTTPINGREQ          12 << 4    // PING Request
#define MQTTPINGRESP         13 << 4    // PING Response
#define MQTTDISCONNECT       14 << 4    // Client is Disconnecting
#define MQTTReserved         15 << 4    // Reserved
#define MQTTQOS0              0 << 1
#define MQTTQOS1              1 << 1
#define MQTTQOS2              2 << 1

class MQTTClient
{
    TcpClient  _client;
    uint8_t    _buffer[MQTT_MAX_PACKET_SIZE];
    uint16_t   _nextMsgId;
    time_t     _lastOutActivity;
    time_t     _lastInActivity;
    bool       _pingOutstanding;
    IpAddress  _ip;
    char*      _domain;
    uint16_t   _port;
    Stream*    _stream;

    uint16_t   _readPacket(uint8_t*);
    uint8_t    _readByte();
    bool       _write(uint8_t header, uint8_t* buf, uint16_t length);
    uint16_t   _writeString(const char* string, uint8_t* buf, uint16_t length);

    Callback<void (char*, uint8_t*, uint16_t)> _onMessage;
public:
    MQTTClient();
    MQTTClient(IpAddress& ip, uint16_t port);
    MQTTClient(IpAddress& ip, uint16_t port , Stream& stream);
    MQTTClient(const char* domain, uint16_t port);
    MQTTClient(const char* domain, uint16_t port, Stream& stream);
    bool    connect(const char*);
    bool    connect(const char* , const char* , const char*);
    bool    connect(const char* , const char* , uint8_t, uint8_t, const char*);
    bool    connect(const char* , const char* , const char* , const char* , uint8_t, uint8_t, const char*);
    void    disconnect();
    bool    publish(const char* , const char*);
    bool    publish(const char* , uint8_t* , uint16_t);
    bool    publish(const char* , uint8_t* , uint16_t, bool);
    bool    subscribe(const char*);
    bool    subscribe(const char* , uint8_t qos);
    bool    unsubscribe(const char*);
    bool    poll();
    bool    connected();
    void    attach(Callback<void (char*, uint8_t*, uint16_t)>);
    template<typename T, typename M>
    void    attach(T *obj, M method ) { attach(callback(obj, method)); }
};
#endif
