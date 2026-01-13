#pragma once
#include <array>
#include <asynPortDriver.h>
#include <functional>
#include <tuple>

// asyn parameter names
inline constexpr char DUAL_X_DEG_STRING[] = "DUAL_X_DEG";
inline constexpr char DUAL_Y_DEG_STRING[] = "DUAL_Y_DEG";
inline constexpr char SINGLE_DEG_STRING[] = "SINGLE_DEG";

constexpr size_t BUFFER_SIZE = 12; // bytes
constexpr double IO_TIMEOUT = 1.0; // sec

constexpr uint8_t ModeSingle = 0x01;
constexpr uint8_t ModeDual = 0x02;
constexpr uint8_t ModeVibro = 0x03;
constexpr uint8_t ModeCalibration = 0x0B;
constexpr uint8_t ModeAltZeroSingle = 0x07;
constexpr uint8_t ModeAltZeroDual = 0x0A;
constexpr uint8_t ModeLocation = 0x08;
constexpr uint8_t ModeNone = 0x00;
constexpr uint8_t ReplyStart = 0x61;
constexpr uint8_t ReplyModeSingle = 0x11;
constexpr uint8_t ReplyModeDual = 0x22;

constexpr size_t PROCESS_BUFFER_MAX = 2048;

class DigipasDWL : public asynPortDriver {
  public:
    DigipasDWL(const char* conn_port, const char* driver_port, const char* mode_str, int country, int city);
    virtual void poll(void);
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);

  private:
    asynUser* pasynUserDriver_; // pointer to the asynUser for this driver
    uint8_t mode_ = ModeNone;   // The active mode. Shouldn't change after class construction
    std::array<uint8_t, BUFFER_SIZE> out_buffer_; // Output data buffer
    std::array<uint8_t, BUFFER_SIZE> in_buffer_;  // Input data buffer
    std::vector<uint8_t> processing_buffer_;      // Processing data buffer (sliding window)

    // Sends the initialization command to the sensor
    asynStatus init_sensor();

    // Sets the sensor mode given a string name
    // which is provided as argument to iocsh function
    asynStatus set_mode(std::string mode_str);

    // Sets the sensor sensor with the given country
    // and city codes. Consult the manual for a list of codes.
    asynStatus set_location(uint8_t country, uint8_t city);

    // Writes out_buffer_ the sensor
    asynStatus write();

    // Reads in_buffer_ from the sensor
    asynStatus read();

    // Function to compute the angles from the input data.
    // In single mode, the the returned tuple is [angle, 0.0]
    // In dual mode, the returned tuple is [x, y]
    std::function<std::tuple<double, double>()> compute_angles = []() { return std::tuple{0.0, 0.0}; };

  protected:
    // Indices for parameters in asyn parameter library
    int dualXdegId_;
    int dualYdegId_;
    int singleDegId_;
};
