#include "digipasDWL.hpp"

#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <asynOctetSyncIO.h>

static void poll_thread_C(void *pPvt) {
    DigipasDWL *pDigipasDWL = (DigipasDWL *)pPvt;
    pDigipasDWL->poll();
}

constexpr int MAX_CONTROLLERS = 1;

DigipasDWL::DigipasDWL(const char* asyn_port)
    : asynPortDriver((std::string(asyn_port)+"_internal").c_str(), MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask | asynInt32ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynInt32ArrayMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                     1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
                     0, 0)
    {

    asynStatus status = pasynOctetSyncIO->connect(asyn_port, 0, &pasynUserDWL_, NULL);

    epicsThreadCreate("DigipasDWLPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void DigipasDWL::poll() {
    while (true) {
        lock();

	// do stuff ...

        callParamCallbacks();
        unlock();
        epicsThreadSleep(1.0);
    }
}

void DigipasDWL::init() {
    buffer.fill(0x0);
    buffer[0] = 0x06;
    buffer[1] = 0x24;
    // TODO: write to device
}

asynStatus DigipasDWL::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    bool comm_ok = true;

    callParamCallbacks();
    return comm_ok ? asynSuccess : asynError;
}

asynStatus DigipasDWL::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;
    bool comm_ok = true;

    callParamCallbacks();
    return comm_ok ? asynSuccess : asynError;
}

// register function for iocsh
extern "C" int DigipasDWLConfig(const char *asyn_port_name) {
    DigipasDWL *pDigipasDWL = new DigipasDWL(asyn_port_name);
    pDigipasDWL = NULL;
    return (asynSuccess);
}

static const iocshArg DigipasDWLArg0 = {"asyn port name", iocshArgString};
static const iocshArg *const DigipasDWLArgs[1] = {&DigipasDWLArg0};
static const iocshFuncDef DigipasDWLFuncDef = {"DigipasDWLConfig", 1, DigipasDWLArgs};

static void DigipasDWLCallFunc(const iocshArgBuf *args) { DigipasDWLConfig(args[0].sval); }

void DigipasDWLRegister(void) { iocshRegister(&DigipasDWLFuncDef, DigipasDWLCallFunc); }

extern "C" {
epicsExportRegistrar(DigipasDWLRegister);
}
