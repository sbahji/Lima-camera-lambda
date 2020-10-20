//
// Copyright (c) 2018-2019 X-Spectrum GmbH. All rights reserved.
//

#ifndef LIBXSP_LIBXSP_H
#define LIBXSP_LIBXSP_H

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <functional>


namespace xsp {


#define LIBXSP_VERSION_MAJOR 1
#define LIBXSP_VERSION_MINOR 3
#define LIBXSP_VERSION_PATCH 1

extern std::string libraryVersion();
extern int libraryMajor();
extern int libraryMinor();
extern int libraryPatch();


enum class LogLevel {
    ERROR = 0,
    WARN = 1,
    INFO = 2,
    DEBUG = 3
};

extern void setLogHandler(const std::function<void(LogLevel, const std::string&)>& handler);
extern void clearLogHandler();


enum class StatusCode: std::uint32_t {
    GOOD = 0x00000000,
    BAD_UNEXPECTED_ERROR = 0x80010000,
    BAD_INTERNAL_ERROR = 0x80020000,
    BAD_OUT_OF_MEMORY = 0x80030000,
    BAD_RESOURCE_UNAVAILABLE = 0x80040000,
    BAD_COMMUNICATION_ERROR = 0x80050000,
    BAD_DEVICE_NOT_CONNECTED = 0x80100000,
    BAD_DEVICE_NOT_SUPPORTED = 0x80110000,
    BAD_DEVICE_NOT_READY = 0x80120000,
    BAD_DEVICE_BUSY = 0x80130000,
    BAD_DEVICE_FAILURE = 0x80140000,
    BAD_COMMAND_NOT_ACCEPTED = 0x80400000,
    BAD_COMMAND_NOT_IMPLEMENTED = 0x80410000,
    BAD_COMMAND_NOT_SUPPORTED = 0x80420000,
    BAD_COMMAND_TIMED_OUT = 0x80430000,
    BAD_COMMAND_FAILED = 0x80440000,
    BAD_ARG_OUT_OF_RANGE = 0x804a0000,
    BAD_ARG_INVALID = 0x804b0000
};

extern std::string toString(const StatusCode& code);
extern std::ostream& operator<<(std::ostream& os, const StatusCode& code);


class Exception : public std::runtime_error
{
public:
    explicit Exception(const std::string& msg) : std::runtime_error(msg) {}
    ~Exception() noexcept override = default;
    Exception(const Exception&) = default;

    const char* what() const noexcept override { return std::runtime_error::what(); }
};


class ConfigError : public Exception
{
public:
    explicit ConfigError(const std::string& msg) : Exception(msg) {}
    ~ConfigError() noexcept override = default;
    ConfigError(const ConfigError&) = default;
};


class RuntimeError : public Exception
{
public:
    explicit RuntimeError(const std::string& msg, StatusCode code) : status_code(code), Exception(msg) {}
    ~RuntimeError() noexcept override = default;
    RuntimeError(const RuntimeError&) = default;

    std::uint32_t code() const noexcept { return static_cast<std::uint32_t>(status_code); }

private:
    StatusCode status_code;
};


enum class EventType {
    READY,
    START,
    STOP
};


enum class FrameStatusCode
{
    FRAME_OK,
    FRAME_INCOMPLETE,
    FRAME_MISSING,
    FRAME_COMPRESSION_FAILED
};


enum class ShuffleMode
{
    NO_SHUFFLE,
    BYTE_SHUFFLE,
    BIT_SHUFFLE,
    AUTO_SHUFFLE
};


enum class Interpolation {
    OFF,
    ON
};


struct Position {
    double x;
    double y;
    double z;
};


struct Rotation {
    double alpha;
    double beta;
    double gamma;
};


class Frame
{
public:
    virtual ~Frame() = default;
    virtual std::uint64_t nr() const = 0;
    virtual std::uint64_t trigger() const = 0;
    virtual FrameStatusCode status() const = 0;
    virtual const std::uint8_t* data() const = 0;
    virtual std::size_t size() const = 0;
};


class Detector
{
public:
    virtual ~Detector() = default;
    virtual std::string id() const = 0;
    virtual std::string type() const = 0;
    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void initialize() = 0;
    virtual bool isConnected() const = 0;
    virtual bool isReady() const = 0;
    virtual bool isBusy() const = 0;
    virtual void setEventHandler(const std::function<void(EventType, const void *)>& handler) = 0;
    virtual void clearEventHandler() = 0;
    virtual std::string userData(const std::string& key) const = 0;
    virtual std::uint64_t frameCount() const = 0;
    virtual void setFrameCount(std::uint64_t count) = 0;
    virtual double shutterTime() const = 0;
    virtual void setShutterTime(double time_ms) = 0;
    virtual void reset() = 0;
    virtual void startAcquisition() = 0;
    virtual void stopAcquisition() = 0;
    virtual int liveFrameWidth() const = 0;
    virtual int liveFrameHeight() const = 0;
    virtual int liveFrameDepth() const = 0;
    virtual int liveFramesQueued() const = 0;
    virtual const Frame* liveFrame(int timeout_ms) = 0;
    virtual void release(std::uint64_t nr) = 0;
};


class Receiver
{
public:
    virtual ~Receiver() = default;
    virtual std::string id() const = 0;
    virtual std::string type() const = 0;
    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void initialize() = 0;
    virtual bool isConnected() const = 0;
    virtual bool isReady() const = 0;
    virtual bool isBusy() const = 0;
    virtual void setEventHandler(const std::function<void(EventType, const void *)>& handler) = 0;
    virtual void clearEventHandler() = 0;
    virtual std::string userData(const std::string& key) const = 0;
    virtual bool pixelMaskApplied() const = 0;
    virtual const std::vector<std::uint32_t>& pixelMask() const = 0;
    virtual bool flatFieldApplied() const = 0;
    virtual const std::vector<double>& flatField() const = 0;
    virtual const std::vector<double>& flatFieldError() const = 0;
    virtual Position position() const = 0;
    virtual Rotation rotation() const = 0;
    virtual std::string compressor() const = 0;
    virtual int compressionLevel() const = 0;
    virtual void setCompressionLevel(int level) = 0;
    virtual ShuffleMode shuffleMode() const = 0;
    virtual void setShuffleMode(ShuffleMode mode) = 0;
    virtual Interpolation interpolation() const = 0;
    virtual void setInterpolation(Interpolation i) = 0;
    virtual int ramBufferSizeToFrames() const = 0;

    virtual int frameWidth() const = 0;
    virtual int frameHeight() const = 0;
    virtual int frameDepth() const = 0;
    virtual int framesQueued() const = 0;
    virtual const Frame* frame(int timeout_ms) = 0;
    virtual void release(std::uint64_t nr) = 0;
};


class System
{
public:
    virtual ~System() = default;
    virtual std::string id() const = 0;
    virtual std::vector<std::string> detectorIds() const = 0;
    virtual std::vector<std::string> receiverIds() const = 0;
    virtual std::shared_ptr<Detector> detector(const std::string& id) const = 0;
    virtual std::shared_ptr<Receiver> receiver(const std::string& id) const = 0;
    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void initialize() = 0;
    virtual bool isConnected() const = 0;
    virtual bool isReady() const = 0;
    virtual bool isBusy() const = 0;
    virtual void reset() = 0;
    virtual void startAcquisition() = 0;
    virtual void stopAcquisition() = 0;
};

std::unique_ptr<System> createSystem(const std::string& config_file);



namespace lambda {

enum class Feature : std::uint16_t {
    FEAT_HV = 0x0001u,
    FEAT_1_6_BIT = 0x0002u,
    FEAT_MEDIPIX_DAC_IO = 0x0004u,
    FEAT_EXTENDED_GATING = 0x0008u
};

enum class BitDepth {
    DEPTH_1 = 1,
    DEPTH_6 = 6,
    DEPTH_12 = 12,
    DEPTH_24 = 24
};

enum class ChargeSumming {
    OFF,
    ON
};

enum class CounterMode {
    SINGLE,
    DUAL
};

enum class Pitch {
    PITCH_55,
    PITCH_110
};


enum class TrigMode : std::uint8_t
{
    SOFTWARE,
    EXT_SEQUENCE,
    EXT_FRAMES
};


enum class Gating {
    OFF,
    ON
};


struct OperationMode {

    OperationMode() = default;
    explicit OperationMode(
            BitDepth d,
            ChargeSumming cs = ChargeSumming::OFF,
            CounterMode cm = CounterMode::SINGLE,
            Pitch p = Pitch::PITCH_55)
        : bit_depth(d), charge_summing(cs), counter_mode(cm), pitch(p)
        {}

    BitDepth bit_depth = BitDepth::DEPTH_12;
    ChargeSumming charge_summing = ChargeSumming::OFF;
    CounterMode counter_mode = CounterMode::SINGLE;
    Pitch pitch = Pitch::PITCH_55;
};


class Detector : virtual public xsp::Detector
{
public:
    virtual ~Detector() = default;
    virtual int numberOfModules() const = 0;
    virtual bool isModuleConnected(int module_nr) const = 0;
    virtual bool isModuleReady(int module_nr) const = 0;
    virtual bool isModuleBusy(int module_nr) const = 0;
    virtual std::string firmwareVersion(int module_nr) const = 0;
    virtual bool hasFeature(int module_nr, Feature f) const = 0;
    virtual std::vector<std::string> chipIds(int module_nr) const = 0;
    virtual OperationMode operationMode() const = 0;
    virtual void setOperationMode(const OperationMode& mode) = 0;
    virtual std::vector<double> thresholds() const = 0;
    virtual void setThresholds(const std::vector<double>& thresholds_kev) = 0;
    virtual TrigMode  triggerMode() const = 0;
    virtual void setTriggerMode(TrigMode mode) = 0;
    virtual int framesPerTrigger() const = 0;
    virtual void setFramesPerTrigger(int n) = 0;
    virtual Gating  gatingMode() const = 0;
    virtual void setGatingMode(Gating mode) = 0;
    [[deprecated]] virtual int ioDelay(int module_nr) const = 0;
    [[deprecated]] virtual void setIoDelay(int module_nr, int taps) = 0;
    virtual double voltage(int module_nr) const = 0;
    virtual void setVoltage(int module_nr, double voltage) = 0;
    virtual std::vector<double> temperature(int module_nr) const = 0;
    virtual double sensorCurrent(int module_nr) const = 0;
    virtual double humidity(int module_nr) const = 0;
    virtual double dacOut(int module_nr, int chip_nr) const = 0;
    virtual void setDacIn(int module_nr, int chip_nr, double voltage) = 0;
};

} // namespace lambda

} // namespace xsp

#endif //LIBXSP_LIBXSP_H
