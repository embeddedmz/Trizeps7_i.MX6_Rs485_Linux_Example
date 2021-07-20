#pragma once

#include <cstdint>
#include <string>
#include <vector>

class Rs485SerialPort final
{
public:
    enum RTSMode
    {
        None = 0,
        HwUp = 1, // Hardware up
        HwDown = 2, // Hardware down
        SwUp = 3, // Software up
        SwDown = 4 // Software down
    };

    explicit Rs485SerialPort();
    ~Rs485SerialPort();

    bool open(const std::string& port, const int baudrate, const char parity, 
        const int dataBit, const int stopBit);
    void flush();
    void close();

    bool setRtsMode(const RTSMode rtsMode);
    RTSMode getRtsMode();
    void setDebug(const bool enable);
    bool getDebug();

    ssize_t send(const uint8_t* pData, const size_t uSize);
    ssize_t send(const std::string& strData);
    ssize_t send(const std::vector<uint8_t>& Data);
    ssize_t receive(uint8_t* pData, const int uSize);

    void setRcvTimeout(const uint32_t msecTimeout);
    uint32_t getRcvTimeout();

    void setRTSSoftwareDelay(const uint32_t msecTimeout);
    uint32_t getRTSSoftwareDelay();

    bool setRTSHardwareDelayBeforeSend(const uint32_t msecTimeout);
    bool setRTSHardwareDelayAfterSend(const uint32_t msecTimeout);
    uint32_t getRTSHardwareDelayBeforeSend();
    uint32_t getRTSHardwareDelayAfterSend();

private:
    struct Internals;
    Internals* const m_Internals;
};
