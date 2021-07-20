//#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>

#include <unistd.h>

#include "Rs485SerialPort.h"

int main()
{
    srand(static_cast<unsigned>(time(nullptr)));

#define M6P_TEST
#ifdef M6P_TEST
    Rs485SerialPort sport;
    sport.setDebug(true);
    sport.setRcvTimeout(1000);
    if (sport.open("/dev/ttymxc0", 57600, 'N', 8, 1))
    {
        sport.setRtsMode(Rs485SerialPort::RTSMode::HwUp);
        bool test = sport.setRTSHardwareDelayBeforeSend(0);
        assert(test == true);
        test = sport.setRTSHardwareDelayAfterSend(0);
        assert(test == true);

        uint8_t m6pAddrByte = 0x00;
        uint8_t analogReqByte = 0x0F;
        uint8_t req[19] = { 0x24, 0x44, 0x49, 0x13, m6pAddrByte, 0x01, 0x00,
            analogReqByte, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        ushort checksum = 0;
        for (int i = 1; i < 17; ++i)
        {
            checksum += req[i];
        }
        // BE 16-bit
        req[17] = checksum >> 8;
        req[18] = checksum & 0xff;

        int i = 0;
        while (i++ < 10)
        {
            ssize_t ret = sport.send(req, 19);
            uint8_t resp[19] = { 0 };
            //std::generate(resp, resp + 19, [] { return (std::rand() % 256); });

            ret = sport.receive(resp, 19);

            if (resp[3] != 19)
            {
                printf("BCOUNT NOK\n");
            }
            else
            {
                printf("BCOUNT OK\n");
            }

            if (resp[0] != 0x24)
            {
                printf("$      NOK\n");
            }
            else
            {
                printf("$      OK\n");
            }

            ushort checksumRes = (ushort)(resp[18] + (resp[17] << 8));
            ushort checksumExp = 0;
            for (int i = 1; i < 17; ++i)
            {
                checksumExp += resp[i];
            }

            if (checksumRes == checksumExp)
            {
                printf("CHKSUM OK\n");
            }
            else
            {
                printf("CHKSUM NOK\n");
            }
            usleep(500000);
        }
        sport.close();
    }
#else
    Rs485SerialPort sport;
    sport.setDebug(true);
    sport.setRcvTimeout(10000);
    if (sport.open("/dev/ttymxc0", 57600, 'N', 8, 1))
    {
        sport.setRtsMode(Rs485SerialPort::RTSMode::HwUp);
        
        ssize_t ret = sport.send(reinterpret_cast<const uint8_t*>("Hello Amine !"), strlen("Hello Amine !"));
        printf("Sent '%zd' to Amine's computer ! (expected: '%zu').\n", ret, strlen("Hello Amine !"));

        printf("Waiting for Amine's computer to send a response (10 seconds timeout).\n");
        uint8_t resp[1024];
        ret = sport.receive(resp, 1024);
        if (ret > 0)
        {
            printf("Response: %s", std::string(reinterpret_cast<const char*>(resp), ret).c_str());
        }
        
        sport.close();
    }
    printf("\n");
#endif
    return 0;
}
