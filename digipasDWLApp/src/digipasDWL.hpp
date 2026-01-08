#pragma once
#include <asynPortDriver.h>
#include <array>

constexpr int BUFFER_SIZE = 12; // bytes

class DigipasDWL : public asynPortDriver {
   public:
    DigipasDWL(const char* asyn_port);
    virtual void poll(void);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
private:
    asynUser* pasynUserDWL_;
    std::array<uint8_t, BUFFER_SIZE> buffer;

    void init();
};
