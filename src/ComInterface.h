#ifndef COMINTERFACE_H
#define COMINTERFACE_H

#include "RebaType.h"
#ifdef DJALLIL_CONF

#define OKC_HOST "192.168.0.100"
#define LEFTARM_IP "192.168.0.10"
#define RIGHTARM_IP "192.168.0.20"

#else

#define OKC_HOST "192.168.10.123"
#define LEFTARM_IP "192.168.10.10"
#define RIGHTARM_IP "192.168.10.11"
#endif

#define OKC_PORT "49938"

class ComInterface
{
public:
    ComInterface();
    virtual void connect() = 0;
    virtual bool isConnected() = 0;
    virtual void waitForFinished() = 0;
    float cycle_time;
};

#endif // COMINTERFACE_H
