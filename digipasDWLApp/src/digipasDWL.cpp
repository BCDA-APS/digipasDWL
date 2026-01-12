#include "digipasDWL.hpp"

#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <algorithm>
#include <chrono>

static void poll_thread_C(void* pPvt) {
    DigipasDWL* pDigipasDWL = (DigipasDWL*)pPvt;
    pDigipasDWL->poll();
}

constexpr int MAX_CONTROLLERS = 1;
constexpr int INTERFACE_MASK = asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask;
constexpr int INTERRUPT_MASK = asynInt32Mask | asynFloat64Mask | asynOctetMask;
constexpr int ASYN_FLAGS = ASYN_MULTIDEVICE | ASYN_CANBLOCK;

DigipasDWL::DigipasDWL(const char* conn_port, const char* driver_port, std::string mode_str, int country, int city)
    : asynPortDriver(driver_port, MAX_CONTROLLERS, INTERFACE_MASK, INTERRUPT_MASK, ASYN_FLAGS, 1, 0, 0) {

    asynStatus status = pasynOctetSyncIO->connect(conn_port, 0, &pasynUserDriver_, NULL);
    if (status) {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "Failed to connect to sensor\n");
        return;
    }

    createParam(X_DEG_STRING, asynParamFloat64, &xdegId_);
    createParam(Y_DEG_STRING, asynParamFloat64, &ydegId_);

    // TODO:
    // - device version argument to constructor
    // - member variables: std::function<double(const std::array<char, 12>&)> compute_x, compute_y
    // - here in constructor, set those functions based on value of device version

    status = init_sensor();
    if (status)
        return;

    status = set_location(country, city);
    printf("Location: 0x%X, 0x%X\n", country, city);
    if (status)
        return;

    std::transform(mode_str.begin(), mode_str.end(), mode_str.begin(), [](auto& c){
	return std::tolower(c);
    });


    uint8_t sensor_mode = ModeNone;
    if (mode_str == "single") {
	sensor_mode = ModeSingle;
	compute_angles = [this](){
	    return Angles{
		(((this->in_buffer_[5]<< 24) + (this->in_buffer_ [4] << 16) + (this->in_buffer_ [3]<< 8) + this->in_buffer_ [2]) -18000000.0) / 100000.0,
		0.0
	    };
	};
    } else if (mode_str == "dual") {
	sensor_mode = ModeDual;
	compute_angles = [this](){
	    return Angles {
		(((this->in_buffer_[7] << 16) + (this->in_buffer_[6] << 8) + this->in_buffer_[5]) - 3000000) / 100000.0,
		(((this->in_buffer_[4] << 16) + (this->in_buffer_[3] << 8) + this->in_buffer_[2]) - 3000000) / 100000.0
	    };
	};
    } else if (mode_str == "calibration") {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "Calibration mode not supported\n");
        return;
    } else {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "Unknown mode %s\n", mode_str.c_str());
        return;
    }

    status = set_mode(sensor_mode);
    if (status)
        return;

    epicsThreadCreate("DigipasDWLPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void DigipasDWL::poll() {
    while (true) {
        lock();

	constexpr size_t TEMP_BUFFER_SIZE = 1024;
	constexpr double QUICK_READ_TIMEOUT = 0.05;
	std::array<uint8_t, TEMP_BUFFER_SIZE> temp_buffer;
	size_t nbytesin = 0;
	int eom_reason;
	asynStatus status = pasynOctetSyncIO->read(pasynUserDriver_, (char*)temp_buffer.data(), temp_buffer.size(),
						   QUICK_READ_TIMEOUT, &nbytesin, &eom_reason);
	// TODO: implement actual timeout
	if ((status != asynSuccess && status != asynTimeout) || nbytesin == 0) {
	    asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "pasynOctetSyncIO->read() failed[%d]. Read %ld bytes\n",
		      eom_reason, nbytesin);
	} else {
	    processing_buffer_.insert(processing_buffer_.end(), temp_buffer.begin(), temp_buffer.begin() + nbytesin);

	    bool new_data = false;
	    while (processing_buffer_.size() >= BUFFER_SIZE) {
		const uint8_t mode_check = mode_ == ModeDual ? ReplyModeDual : mode_ == ModeSingle ? ReplyModeSingle : ModeNone;
		if (processing_buffer_[0] == ReplyStart && processing_buffer_[1] == mode_check) {
		    std::copy(processing_buffer_.begin(), processing_buffer_.begin()+BUFFER_SIZE, in_buffer_.begin());
		    processing_buffer_.erase(processing_buffer_.begin(),processing_buffer_.begin()+BUFFER_SIZE);
		    new_data = true;
		} else {
		    processing_buffer_.erase(processing_buffer_.begin());
		}
	    }

	    if (processing_buffer_.size() > PROCESS_BUFFER_MAX) {
		processing_buffer_.clear();
	    }

	    if (new_data) {
		if (compute_angles) {
		    auto angles = compute_angles();
		    printf("X: %f\n", angles.x);
		    printf("Y: %f\n\n", angles.y);
		    setDoubleParam(xdegId_, angles.x);
		    setDoubleParam(ydegId_, angles.y);
		}
	    }
	}

        callParamCallbacks();
	unlock();

	epicsThreadSleep(0.1);
    }
}

asynStatus DigipasDWL::init_sensor() {
    out_buffer_.fill(0x0);
    out_buffer_[0] = 0x06;
    out_buffer_[1] = 0x24;
    return write();
}

asynStatus DigipasDWL::set_location(uint8_t country, uint8_t city) {
    out_buffer_.fill(0x0);
    out_buffer_[0] = 0x06;
    out_buffer_[1] = 0x01;
    out_buffer_[2] = ModeLocation;
    out_buffer_[3] = country;
    out_buffer_[4] = city;
    out_buffer_[5] = 0x00;
    out_buffer_[6] = 0x00;
    out_buffer_[7] = 0x5A;
    return write();
}


asynStatus DigipasDWL::set_mode(uint8_t mode) {
    out_buffer_.fill(0x0);
    out_buffer_[0] = 0x06;
    out_buffer_[1] = 0x01;
    out_buffer_[2] = mode;
    out_buffer_[3] = 0xAA;

    asynStatus status = write();
    if (status)
        return status;

    mode_ = mode;
    return status;
}

asynStatus DigipasDWL::write() {
    size_t nbytesout;
    asynStatus status = pasynOctetSyncIO->write(pasynUserDriver_, (char*)out_buffer_.data(), out_buffer_.size(), IO_TIMEOUT, &nbytesout);
    if (status) {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR,
                  "pasynOctetSyncIO->write() failed. Wrote %ld\n", nbytesout);
    }
    return status;
}

asynStatus DigipasDWL::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    // int function = pasynUser->reason;
    bool comm_ok = true;

    callParamCallbacks();
    return comm_ok ? asynSuccess : asynError;
}

asynStatus DigipasDWL::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {
    // int function = pasynUser->reason;
    bool comm_ok = true;

    callParamCallbacks();
    return comm_ok ? asynSuccess : asynError;
}

// register function for iocsh
extern "C" int DigipasDWLConfig(const char* conn_port, const char* driver_port, const char* mode, int country, int city) {
    DigipasDWL* pDigipasDWL = new DigipasDWL(conn_port, driver_port, mode, country, city);
    pDigipasDWL = NULL;
    (void)pDigipasDWL;
    return (asynSuccess);
}

static const iocshArg DigipasDWLArg0 = {"Connection asyn port", iocshArgString};
static const iocshArg DigipasDWLArg1 = {"Driver asyn port", iocshArgString};
static const iocshArg DigipasDWLArg2 = {"Mode", iocshArgString};
static const iocshArg DigipasDWLArg3 = {"Country code", iocshArgInt};
static const iocshArg DigipasDWLArg4 = {"City code", iocshArgInt};
static const iocshArg* const DigipasDWLArgs[5] = {&DigipasDWLArg0, &DigipasDWLArg1, &DigipasDWLArg2,
                                                  &DigipasDWLArg3, &DigipasDWLArg4};
static const iocshFuncDef DigipasDWLFuncDef = {"DigipasDWLConfig", 5, DigipasDWLArgs};

static void DigipasDWLCallFunc(const iocshArgBuf* args) {
    DigipasDWLConfig(args[0].sval, args[1].sval, args[2].sval, args[3].ival, args[4].ival);
}

void DigipasDWLRegister(void) { iocshRegister(&DigipasDWLFuncDef, DigipasDWLCallFunc); }

extern "C" {
epicsExportRegistrar(DigipasDWLRegister);
}
