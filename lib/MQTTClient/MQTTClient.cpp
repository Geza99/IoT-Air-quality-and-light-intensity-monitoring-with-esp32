/*
 MQTTClient.cpp - A simple client for MQTT.
  Nicholas O'Leary
  http://knolleary.net

  Ported to mbed by Zoltan Hudak <hudakz@outlook.com>
*/
#include "MQTTClient.h"
#include <string.h>
#include <time.h>

/**
 * @brief
 * @note
 * @param
 * @retval
 */
MQTTClient::MQTTClient() :
    _client(NULL),
    _stream(NULL),
    _onMessage(NULL)
{ }

/**
 * @brief
 * @note
 * @param
 * @retval
 */
MQTTClient::MQTTClient(IpAddress& ip, uint16_t port) :
    _ip(ip),
    _domain(NULL),
    _port(port),
    _stream(NULL),
    _onMessage(NULL)
{ }

/**
 * @brief
 * @note
 * @param
 * @retval
 */
MQTTClient::MQTTClient(const char* domain, uint16_t port) :
    _domain((char*)domain),
    _port(port),
    _stream(NULL),
    _onMessage(NULL)
{ }

/**
 * @brief
 * @note
 * @param
 * @retval
 */
MQTTClient::MQTTClient(IpAddress& ip, uint16_t port, Stream& stream) :
    _ip(ip),
    _domain(NULL),
    _port(port),
    _stream(&stream),
    _onMessage(NULL)
{ }

/**
 * @brief
 * @note
 * @param
 * @retval
 */
MQTTClient::MQTTClient(const char* domain, uint16_t port, Stream& stream) :
    _domain((char*)domain),
    _port(port),
    _stream(&stream),
    _onMessage(NULL)
{ }

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::connect(const char* id)
{
    return connect(id, NULL, NULL, 0, 0, 0, 0);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::connect(const char* id, const char* user, const char* pass)
{
    return connect(id, user, pass, 0, 0, 0, 0);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::connect
(
    const char*     id,
    const char*     willTopic,
    uint8_t         willQos,
    uint8_t         willRetain,
    const char*     willMessage
)
{
    return connect(id, NULL, NULL, willTopic, willQos, willRetain, willMessage);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::connect
(
    const char*     id,
    const char*     user,
    const char*     pass,
    const char*     willTopic,
    uint8_t         willQos,
    uint8_t         willRetain,
    const char*     willMessage
)
{
    if (!connected()) {
        int result = 0;

        if (_domain != NULL) {
            result = !_client.connect(this->_domain, this->_port);
        }
        else {
            result = _client.connect(this->_ip, this->_port);
        }

        if (result) {
            _nextMsgId = 1;

            uint8_t     d[] = { 0x00, 0x06, 'M', 'Q', 'I', 's', 'd', 'p', MQTTPROTOCOLVERSION };

            // Leave room in the buffer for header and variable length field
            uint16_t    pos = 5;
            uint16_t    j;
            for (j = 0; j < sizeof(d); j++) {
                _buffer[pos++] = d[j];
            }

            uint8_t v;
            if (willTopic) {
                v = 0x06 | (willQos << 3) | (willRetain << 5);
            }
            else {
                v = 0x02;
            }

            if (user != NULL) {
                v = v | 0x80;

                if (pass != NULL) {
                    v = v | (0x80 >> 1);
                }
            }

            _buffer[pos++] = v;

            _buffer[pos++] = ((MQTT_KEEPALIVE) >> 8);
            _buffer[pos++] = ((MQTT_KEEPALIVE) & 0xFF);
            pos = _writeString(id, _buffer, pos);
            if (willTopic) {
                pos = _writeString(willTopic, _buffer, pos);
                pos = _writeString(willMessage, _buffer, pos);
            }

            if (user != NULL) {
                pos = _writeString(user, _buffer, pos);
                if (pass != NULL) {
                    pos = _writeString(pass, _buffer, pos);
                }
            }

            _write(MQTTCONNECT, _buffer, pos - 5);

            _lastInActivity = _lastOutActivity = time(NULL);

            while (!_client.available()) {
                unsigned long   t = time(NULL);
                if (t - _lastInActivity > MQTT_KEEPALIVE) {
                    _client.stop();
                    return false;
                }
            }

            uint8_t len;

            if (_readPacket(&len) == 4 && _buffer[3] == 0) {
                _lastInActivity = time(NULL);
                _pingOutstanding = false;
                return true;
            }
        }

        _client.stop();
    }

    return false;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
uint8_t MQTTClient::_readByte()
{
    while (!_client.available()) { }

    return _client.recv();
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
uint16_t MQTTClient::_readPacket(uint8_t* length)
{
    uint16_t    len = 0;
    _buffer[len++] = _readByte();

    bool        isPublish = (_buffer[0] & 0xF0) == MQTTPUBLISH;
    uint32_t    multiplier = 1;
    uint16_t    pos = 0;
    uint8_t     digit = 0;
    uint16_t    skip = 0;
    uint8_t     start = 0;

    do {
        digit = _readByte();
        _buffer[len++] = digit;
        pos += (digit & 127) * multiplier;
        multiplier *= 128;
    } while ((digit & 128) != 0);
    *length = len - 1;

    if (isPublish) {
        // Read in topic length to calculate bytes to skip over for Stream writing
        _buffer[len++] = _readByte();
        _buffer[len++] = _readByte();
        skip = (_buffer[*length + 1] << 8) + _buffer[*length + 2];
        start = 2;
        if (_buffer[0] & MQTTQOS1) {
            // skip message id
            skip += 2;
        }
    }

    for (uint16_t i = start; i < pos; i++) {
        digit = _readByte();
        if (this->_stream) {
            if (isPublish && len -*length - 2 > skip) {
                this->_stream->putc(digit);
            }
        }

        if (len < MQTT_MAX_PACKET_SIZE) {
            _buffer[len] = digit;
        }

        len++;
    }

    if (!this->_stream && len > MQTT_MAX_PACKET_SIZE) {
        len = 0;    // This will cause the packet to be ignored.
    }

    return len;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::poll()
{
    if (connected()) {
        time_t  now = time(NULL);
        if ((now - _lastInActivity > MQTT_KEEPALIVE) || (now - _lastOutActivity > MQTT_KEEPALIVE)) {
            if (_pingOutstanding) {
                _client.stop();
                return false;
            }
            else {
                _buffer[0] = MQTTPINGREQ;
                _buffer[1] = 0;
                _client.send(_buffer, 2);
                _lastOutActivity = now;
                _lastInActivity = now;
                _pingOutstanding = true;
            }
        }

        if (_client.available()) {
            uint8_t     len;
            uint16_t    length = _readPacket(&len);
            uint16_t    msgId = 0;
            uint8_t*    payload;
            if (length > 0) {
                _lastInActivity = now;

                uint8_t type = _buffer[0] & 0xF0;
                if (type == MQTTPUBLISH) {
                    if (_onMessage) {
                        uint16_t    topicLen = (_buffer[len + 1] << 8) + _buffer[len + 2];
                        char        topic[topicLen + 1];
                        for (uint16_t i = 0; i < topicLen; i++) {
                            topic[i] = _buffer[len + 3 + i];
                        }

                        topic[topicLen] = '\0';

                        // msgId only present for QOS>0
                        if ((_buffer[0] & 0x06) == MQTTQOS1) {
                            msgId = (_buffer[len + 3 + topicLen] << 8) + _buffer[len + 3 + topicLen + 1];
                            payload = _buffer + len + 3 + topicLen + 2;
                            _onMessage(topic, payload, length - len - 3 - topicLen - 2);

                            _buffer[0] = MQTTPUBACK;
                            _buffer[1] = 2;
                            _buffer[2] = (msgId >> 8);
                            _buffer[3] = (msgId & 0xFF);
                            _client.send(_buffer, 4);
                            _lastOutActivity = now;
                        }
                        else {
                            payload = _buffer + len + 3 + topicLen;
                            _onMessage(topic, payload, length - len - 3 - topicLen);
                        }
                    }
                }
                else
                if (type == MQTTPINGREQ) {
                    _buffer[0] = MQTTPINGRESP;
                    _buffer[1] = 0;
                    _client.send(_buffer, 2);
                }
                else
                if (type == MQTTPINGRESP) {
                    _pingOutstanding = false;
                }
            }
        }

        return true;
    }

    return false;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::publish(const char* topic, const char* payload)
{
    return publish(topic, (uint8_t*)payload, strlen(payload), false);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::publish(const char* topic, uint8_t* payload, uint16_t length)
{
    return publish(topic, payload, length, false);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::publish(const char* topic, uint8_t* payload, uint16_t plength, bool retained)
{
    if (connected()) {
        // Leave room in the buffer for header and variable length field
        uint16_t    length = 5;
        length = _writeString(topic, _buffer, length);

        uint16_t    i;
        for (i = 0; i < plength; i++) {
            _buffer[length++] = payload[i];
        }

        uint8_t header = MQTTPUBLISH;
        if (retained) {
            header |= 1;
        }

        return _write(header, _buffer, length - 5);
    }

    return false;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::_write(uint8_t header, uint8_t* buf, uint16_t length)
{
    uint8_t digitBuf[4];
    uint8_t digitLen = 0;
    uint8_t digit;
    uint8_t pos = 0;
    uint8_t rc;
    uint8_t len = length;
    do {
        digit = len % 128;
        len = len / 128;
        if (len > 0) {
            digit |= 0x80;
        }

        digitBuf[pos++] = digit;
        digitLen++;
    } while (len > 0);

    buf[4 - digitLen] = header;
    for (int i = 0; i < digitLen; i++) {
        buf[5 - digitLen + i] = digitBuf[i];
    }

    rc = _client.send(buf + (4 - digitLen), length + 1 + digitLen);

    _lastOutActivity = time(NULL);
    return(rc == 1 + digitLen + length);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::subscribe(const char* topic)
{
    bool    result = subscribe(topic, 0);
#if MBED_MAJOR_VERSION == 2
    wait_ms(50);
#else
    thread_sleep_for(50);
#endif
    
    return result;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::subscribe(const char* topic, uint8_t qos)
{
    if (qos > 1)
        return false;

    if (connected()) {
        // Leave room in the buffer for header and variable length field
        uint16_t    length = 5;
        _nextMsgId++;
        if (_nextMsgId == 0) {
            _nextMsgId = 1;
        }

        _buffer[length++] = (_nextMsgId >> 8);
        _buffer[length++] = (_nextMsgId & 0xFF);
        length = _writeString(topic, _buffer, length);
        _buffer[length++] = qos;
        return _write(MQTTSUBSCRIBE | MQTTQOS1, _buffer, length - 5);
    }

    return false;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::unsubscribe(const char* topic)
{
    if (connected()) {
        uint16_t    length = 5;
        _nextMsgId++;
        if (_nextMsgId == 0) {
            _nextMsgId = 1;
        }

        _buffer[length++] = (_nextMsgId >> 8);
        _buffer[length++] = (_nextMsgId & 0xFF);
        length = _writeString(topic, _buffer, length);
        return _write(MQTTUNSUBSCRIBE | MQTTQOS1, _buffer, length - 5);
    }

    return false;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void MQTTClient::disconnect()
{
    _buffer[0] = MQTTDISCONNECT;
    _buffer[1] = 0;
    _client.send(_buffer, 2);
    _client.stop();
    _lastInActivity = _lastOutActivity = time(NULL);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
uint16_t MQTTClient::_writeString(const char* string, uint8_t* buf, uint16_t length)
{
    char*       idp = (char*)string;
    uint16_t    i = 0;

    length += 2;
    while (*idp) {
        buf[length++] = *idp++;
        i++;
    }

    buf[length - i - 2] = (i >> 8);
    buf[length - i - 1] = (i & 0xFF);
    return length;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
bool MQTTClient::connected()
{
    bool    rc = (int)_client.connected();

    if (!rc)
        _client.stop();

    return rc;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void MQTTClient::attach(Callback<void (char *, uint8_t *, uint16_t)> fnc)
{
    _onMessage = fnc;
}
