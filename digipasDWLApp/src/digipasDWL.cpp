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

    char sensor_mode = ModeNone;
    if (mode_str == "single") {
	sensor_mode = ModeSingle;
    } else if (mode_str == "dual") {
	sensor_mode = ModeDual;
    } else if (mode_str == "calibration") {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "Calibration mode not supported\n");
        return;
    }

    status = set_mode(sensor_mode);
    if (status)
        return;

    epicsThreadCreate("DigipasDWLPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

double compute_x(const std::array<uint8_t, BUFFER_SIZE>& data) {
    return (((data[7] << 16) + (data[6] << 8) + data[5]) - 3000000) / 100000.0;
}

double compute_y(const std::array<uint8_t, BUFFER_SIZE>& data) {
    return (((data[4] << 16) + (data[3] << 8) + data[2]) - 3000000) / 100000.0;
}

void DigipasDWL::poll() {
    while (true) {
	// auto start = std::chrono::steady_clock::now();
        lock();

	std::array<uint8_t, 1024> temp_buffer;
	size_t nbytesin = 0;
	int eom_reason;
	asynStatus status = pasynOctetSyncIO->read(pasynUserDriver_, (char*)temp_buffer.data(), temp_buffer.size(),
						   0.05, &nbytesin, &eom_reason);
	printf("nread = %ld\n", nbytesin);
	if ((status != asynSuccess && status != asynTimeout) || nbytesin == 0) {
	    asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "pasynOctetSyncIO->read() failed[%d]. Read %ld bytes\n",
		      eom_reason, nbytesin);
	} else {
	    processing_buffer_.insert(processing_buffer_.end(), temp_buffer.begin(), temp_buffer.begin() + nbytesin);

	    bool new_data = false;
	    while (processing_buffer_.size() >= BUFFER_SIZE) {
		if (processing_buffer_[0] == 0x61 && processing_buffer_[1] == 0x22) {
		    std::copy(processing_buffer_.begin(), processing_buffer_.begin()+BUFFER_SIZE, in_buffer_.begin());
		    processing_buffer_.erase(processing_buffer_.begin(),processing_buffer_.begin()+BUFFER_SIZE);
		    new_data = true;
		} else {
		    processing_buffer_.erase(processing_buffer_.begin());
		}
	    }

	    if (processing_buffer_.size() > 2048) {
		processing_buffer_.clear();
	    }

	    if (new_data) {
		double x_deg = compute_x(in_buffer_);
		double y_deg = compute_y(in_buffer_);

		printf("X: %f\n", x_deg);
		printf("Y: %f\n\n", y_deg);

		setDoubleParam(xdegId_, x_deg);
		setDoubleParam(ydegId_, y_deg);
	    }
	}


        callParamCallbacks();
	unlock();

	// auto finish = std::chrono::steady_clock::now();
	// auto elap = std::chrono::duration<double>(finish - start);
	// printf("elap = %lf sec\n", elap.count());
	// printf("Process buffer size = %ld\n", processing_buffer_.size());
	epicsThreadSleep(0.1);
    }
}

asynStatus DigipasDWL::init_sensor() {
    out_buffer_.fill(0x0);
    out_buffer_[0] = 0x06;
    out_buffer_[1] = 0x24;
    return write_read();
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
    return write_read();
}


asynStatus DigipasDWL::get_angles() {
    asynStatus status = read();
    if (status)
        return status;

    double x_deg = compute_x(in_buffer_);
    double y_deg = compute_y(in_buffer_);

    setDoubleParam(xdegId_, x_deg);
    setDoubleParam(ydegId_, y_deg);

    return status;
}

asynStatus DigipasDWL::set_mode(uint8_t mode) {
    out_buffer_.fill(0x0);
    out_buffer_[0] = 0x06;
    out_buffer_[1] = 0x01;
    out_buffer_[2] = mode;
    out_buffer_[3] = 0xAA;

    asynStatus status = write_read();
    if (status)
        return status;

    return status;

    mode_ = mode;
}

asynStatus DigipasDWL::read() {
    size_t nbytesin;
    int eom_reason;
    asynStatus status = pasynOctetSyncIO->read(pasynUserDriver_, (char*)in_buffer_.data(), in_buffer_.size(),
                                               IO_TIMEOUT, &nbytesin, &eom_reason);
    if (status) {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "pasynOctetSyncIO->read() failed[%d]. Read %ld bytes\n",
                  eom_reason, nbytesin);
    }
    return status;
}

asynStatus DigipasDWL::write_read() {
    size_t nbytesin;
    size_t nbytesout;
    int eom_reason;
    asynStatus status = pasynOctetSyncIO->writeRead(pasynUserDriver_, (char*)out_buffer_.data(), out_buffer_.size(),
                                                    (char*)in_buffer_.data(), in_buffer_.size(), IO_TIMEOUT,
                                                    &nbytesout, &nbytesin, &eom_reason);
    if (status) {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR,
                  "pasynOctetSyncIO->writeRead() failed[%d]. Wrote %ld, read %ld bytes\n", eom_reason,
                  nbytesout, nbytesin);
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
