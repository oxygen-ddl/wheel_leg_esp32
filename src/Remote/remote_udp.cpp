#include "remote_udp.h"
#include <WiFi.h>

DecodedData globalData_1;//id为01的结构体
DecodedData globalData_2;//id为02的结构体
DecodedData globalData_3;//id为03的结构体

RemoteUDP::RemoteUDP(const char *ssid, const char *password, unsigned int localPort) 
{
    _ssid = ssid;
    _password = password;
    _localPort = localPort;
    _udp = WiFiUDP(); // 初始化 WiFiUDP 对象
}

void RemoteUDP::connectToWiFi()
{
    Serial.print("Connecting to WiFi SSID: ");
    Serial.println(_ssid);
    WiFi.begin(_ssid, _password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());
}

void RemoteUDP::begin()
{
    connectToWiFi(); // 连接到 Wi-Fi
    _udp.begin(_localPort);
    Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), _localPort);
}

//printf函数
void RemoteUDP::udp_printf(const char *format, ...)
{
    char buffer[256];
    va_list args;// 定义可变参数列表
    va_start(args, format);// 初始化可变参数列表
    vsnprintf(buffer, sizeof(buffer), format, args);// 格式化字符串
    va_end(args);// 清理可变参数列表

    _udp.beginPacket(_udp.remoteIP(), _udp.remotePort());
    _udp.write((uint8_t *)buffer, strlen(buffer));
    _udp.endPacket();
}

/**
 * @brief 通过 UDP 发送多个 float 值，符合 vofa + 的 justfloat 格式
 * @param count 要发送的 float 值的数量
 * @param ... 可变参数列表，包含要发送的 float 值
 */
void RemoteUDP::udp_justfloat(uint8_t count, ...)
{
    uint8_t _tx_buf[256];
    va_list args;
    va_start(args, count);
    // 构建数据包
    uint8_t *p = _tx_buf;
    for (uint8_t i = 0; i < count; i++)
    {
        // float 在可变参数里会被提升为 double
        double tmp = va_arg(args, double);
        float v = (float)tmp;
        memcpy(p, &v, sizeof(v)); // 小端复制
        p += sizeof(v);
    }
    va_end(args);

    // 末尾追加帧尾 0x00,0x00,0x80,0x7F
    *p++ = 0x00;
    *p++ = 0x00;
    *p++ = 0x80;
    *p++ = 0x7F;

    size_t len = p - _tx_buf;
    _udp.beginPacket(_udp.remoteIP(), _udp.remotePort());
    _udp.write(_tx_buf, len);
    _udp.endPacket();
}

void RemoteUDP::handleIncomingPackets()
{
    int packetSize = _udp.parsePacket();
    if (packetSize)
    {
        // Serial.printf("Received %d bytes from %s, port %d\n", packetSize, _udp.remoteIP().toString().c_str(), _udp.remotePort());
        // 如果数据包不来自10.244.178.203，则忽略
        IPAddress allowedIP(10, 244, 178, 203);
        if (_udp.remoteIP() != allowedIP)
        {
            // Serial.println("Packet from unauthorized IP, ignoring.");
            return;
        }
        // 读取数据
        int len = _udp.read(_incomingPacket, 255);
        
        New_packet_flag = 1; // 设置新数据包标志

        parsePacket((uint8_t *)_incomingPacket, len);// 解析数据包
    }
}


void RemoteUDP::parsePacket(uint8_t *buffer, int len)
{
    if (len < 7)
        return;
    for (int i = 0; i < len - 1; i++)
    {
        if (buffer[i] == 0xFF)
        {
            uint8_t id = buffer[i + 1]; // 获取 ID
            // 查找帧尾 0xFE
            int dataStart = i + 2; // 数据起始位置
            // 根据 ID 不同解析不同长度的数据
            switch (id)
            {
            // ID 01 包含两个 float (每个4字节)
            // 数据段：00 80 3A 44 (float1) 和 00 00 A2 42 (float2)
            case 0x01:
            {
                
                if (buffer[i + 10] == 0xFE)// 验证帧尾位置
                { 
                    float f1, f2;
                    // 使用 memcpy 处理小端序转换
                    memcpy(&f1, &buffer[dataStart], 4);
                    memcpy(&f2, &buffer[dataStart + 4], 4);
                    globalData_1.id = id;
                    globalData_1.value1 = f1;
                    globalData_1.value2 = f2;
                    //udp_printf("ID: %02X, Val1: %.2f, Val2: %.2f\n", id, f1, f2);
                    i += 10; // 跳过已处理的字节
                }
                break;
            }
            // ID 02 包含两个 float (每个4字节)
            case 0x02:
            {

                if (buffer[i + 10] == 0xFE)
                {
                    float f1, f2;
                    memcpy(&f1, &buffer[dataStart], 4);
                    memcpy(&f2, &buffer[dataStart + 4], 4);
                    globalData_2.id = id;
                    globalData_2.value1 = f1;
                    globalData_2.value2 = f2;
                    //udp_printf("ID: %02X, Val1: %.2f, Val2: %.2f\n", id, f1, f2);
                    i += 10; // 跳过已处理的字节    
                }
                break;
            }
            //ID 03 包含一个float
            case 0x03:
            {
                if (buffer[i + 6] == 0xFE)
                {
                    float f1;
                    memcpy(&f1, &buffer[dataStart], 4);
                    // 处理 ID 03 的数据
                    globalData_3.id = id;
                    globalData_3.value1 = f1;
                    //udp_printf("ID: %02X, Val1: %.2f\n", id, f1);
                    i += 6; // 跳过已处理的字节    
                }
                break;
            }

            default:
                
                break;
            }
        }
    }
}

