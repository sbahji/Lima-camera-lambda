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
#include <stdexcept>


namespace xsp {


#define LIBXSP_VERSION_MAJOR 2
#define LIBXSP_VERSION_MINOR 1
#define LIBXSP_VERSION_PATCH 0

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

extern std::string toString(const LogLevel& l);
extern std::ostream& operator<<(std::ostream& os, const LogLevel& l);

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


class Error : public std::runtime_error
{
public:
    explicit Error(const std::string& msg) : std::runtime_error(msg) {}
    ~Error() noexcept override = default;
    Error(const Error&) = default;

    const char* what() const noexcept override { return std::runtime_error::what(); }
};


class ConfigError : public Error
{
public:
    explicit ConfigError(const std::string& msg) : Error(msg) {}
    ~ConfigError() noexcept override = default;
    ConfigError(const ConfigError&) = default;
};


class RuntimeError : public Error
{
public:
    explicit RuntimeError(const std::string& msg, StatusCode code) : Error(msg), status_code(code) {}
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
    virtual int subframe() const = 0;
    virtual int connector() const = 0;
    virtual std::uint64_t seq() const = 0;
    virtual std::uint64_t trigger() const = 0;
    virtual FrameStatusCode status() const = 0;
    virtual const std::uint8_t* data() const = 0;
    virtual std::size_t size() const = 0;
};


class Detector
{
public:
    virtual ~Detector() = default;

    // general infos
    virtual std::string id() const = 0;
    virtual std::string type() const = 0;
    virtual std::string userData(const std::string& key) const = 0;

    // event handling
    virtual void setEventHandler(const std::function<void(EventType, const void *)>& handler) = 0;
    virtual void clearEventHandler() = 0;

    // commands
    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void initialize() = 0;
    virtual void reset() = 0;
    virtual void startAcquisition() = 0;
    virtual void stopAcquisition() = 0;

    // status
    virtual bool isConnected() const = 0;
    virtual bool isReady() const = 0;
    virtual bool isBusy() const = 0;

    // acquisition parameters
    virtual std::uint64_t frameCount() const = 0;
    virtual void setFrameCount(std::uint64_t count) = 0;
    virtual double shutterTime() const = 0;
    virtual void setShutterTime(double time_ms) = 0;

    // live frames
    virtual int liveFrameWidth() const = 0;
    virtual int liveFrameHeight() const = 0;
    virtual int liveFrameDepth() const = 0;
    virtual int liveFramesQueued() const = 0;
    virtual void liveFrameSelect(int subframe, int connector) = 0;
    virtual const Frame* liveFrame(int timeout_ms) = 0;
    virtual void release(const Frame* f) = 0;
};


class Receiver
{
public:
    virtual ~Receiver() = default;

    // general infos
    virtual std::string id() const = 0;
    virtual std::string type() const = 0;
    virtual std::string userData(const std::string& key) const = 0;
    virtual bool ramAllocated() const = 0;

    // event handling
    virtual void setEventHandler(const std::function<void(EventType, const void *)>& handler) = 0;
    virtual void clearEventHandler() = 0;

    // commands
    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void initialize() = 0;

    // status
    virtual bool isConnected() const = 0;
    virtual bool isReady() const = 0;
    virtual bool isBusy() const = 0;

    // acquired data
    virtual int frameWidth() const = 0;
    virtual int frameHeight() const = 0;
    virtual int frameDepth() const = 0;
    virtual int framesQueued() const = 0;
    virtual const Frame* frame(int timeout_ms) = 0;
    virtual void release(const Frame* f) = 0;
};


class PostDecoder
{
public:
    virtual ~PostDecoder() = default;

    // general infos
    virtual std::string id() const = 0;

    // event handling
    virtual void setEventHandler(const std::function<void(EventType, const void *)>& handler) = 0;
    virtual void clearEventHandler() = 0;

    // status
    virtual bool isReady() const = 0;
    virtual bool isBusy() const = 0;

    // commands
    virtual void initialize() = 0;

    // compression info
    virtual std::string compressor() const = 0;
    virtual bool compressionEnabled() const = 0;
    virtual int compressionLevel() const = 0;
    virtual ShuffleMode shuffleMode() const = 0;
    virtual void setCompressionLevel(int level) = 0;
    virtual void setShuffleMode(ShuffleMode mode) = 0;

    // frame summing
    virtual bool frameSummingEnabled() const = 0;
    virtual unsigned int summedFrames() const = 0;
    virtual void setSummedFrames(unsigned int n_frames) = 0;

    // processed data
    virtual int numberOfSubFrames() const = 0;
    virtual int numberOfConnectedSensors() const = 0;
    virtual int frameWidth() const = 0;
    virtual int frameHeight() const = 0;
    virtual int frameDepth() const = 0;
    virtual int framesQueued() const = 0;
    virtual const Frame* frame(int timeout_ms) = 0;
    virtual void release(const Frame* f) = 0;

};

class System
{
public:
    virtual ~System() = default;

    // general infos
    virtual std::string id() const = 0;
    virtual std::vector<std::string> detectorIds() const = 0;
    virtual std::vector<std::string> receiverIds(const std::string& detector="") const = 0;
    virtual std::shared_ptr<Detector> detector(const std::string& id) const = 0;
    virtual std::shared_ptr<Receiver> receiver(const std::string& id) const = 0;

    // commands
    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void initialize() = 0;
    virtual void reset() = 0;
    virtual void startAcquisition() = 0;
    virtual void stopAcquisition() = 0;

    // status
    virtual bool isConnected() const = 0;
    virtual bool isReady() const = 0;
    virtual bool isBusy() const = 0;

    //TODO: move methods on next major release
    virtual std::vector<std::string> postDecoderIds() const = 0;
    virtual std::shared_ptr<PostDecoder> postDecoder(const std::string& id) const = 0;

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


enum class Counter {
    COUNTER_L,
    COUNTER_H
};


enum class Threshold {
    LOWER = 0,
    UPPER = 1
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


enum class ModuleFlag : unsigned int {
    IGNORE_ERRORS = 1 << 0
};

inline auto operator~ (ModuleFlag op) {
    using FT =  std::underlying_type<ModuleFlag>;
    return ~static_cast<FT::type>(op);
}

template <typename T>
inline T operator& (T lhs, ModuleFlag rhs) {
    using FT =  std::underlying_type<ModuleFlag>;
    return lhs & static_cast<FT::type>(rhs);
}

inline ModuleFlag operator| (ModuleFlag lhs, ModuleFlag rhs) {
    using FT =  std::underlying_type<ModuleFlag>;
    return static_cast<ModuleFlag>(static_cast<FT::type>(lhs) | static_cast<FT::type>(rhs));
}

template <typename T>
inline T& operator|= (T& lhs, ModuleFlag rhs) {
    using FT =  std::underlying_type<ModuleFlag>;
    return lhs = lhs | static_cast<FT::type>(rhs);
}

template <typename T>
inline T& operator&= (T& lhs, ModuleFlag rhs) {
    using FT =  std::underlying_type<ModuleFlag>;
    return lhs = lhs | static_cast<FT::type>(rhs);
}


class Detector : virtual public xsp::Detector
{
public:
    virtual ~Detector() = default;

    // general infos
    virtual int numberOfModules() const = 0;
    virtual std::string firmwareVersion(int module_nr) const = 0;
    virtual bool hasFeature(int module_nr, Feature f) const = 0;
    virtual void enableModuleFlag(int module_nr, ModuleFlag flag) = 0;
    virtual void disableModuleFlag(int module_nr, ModuleFlag flag) = 0;
    virtual bool moduleFlagEnabled(int module_nr, ModuleFlag flag) const = 0;
    virtual std::vector<std::string> chipIds(int module_nr) const = 0;
    virtual double voltage(int module_nr) const = 0;
    virtual bool voltageSettled(int module_nr) const = 0;
    virtual std::vector<double> temperature(int module_nr) const = 0;
    virtual double sensorCurrent(int module_nr) const = 0;
    virtual double humidity(int module_nr) const = 0;

    // status
    virtual bool isModuleConnected(int module_nr) const = 0;
    virtual bool isModuleReady(int module_nr) const = 0;
    virtual bool isModuleBusy(int module_nr) const = 0;

    // decoding settings
    virtual void enableSaturationFlag() = 0;
    virtual void disableSaturationFlag() = 0;
    virtual bool saturationFlagEnabled() const = 0;
    virtual int saturationThreshold(int module_nr) const = 0;
    virtual void setSaturationThreshold(int module_nr, int n) = 0;
    virtual const std::vector<int> saturationThresholdPerPixel(int module_nr) const = 0;
    virtual void setSaturationThresholdPerPixel(int module_nr, const std::vector<int>& v) = 0;
    virtual void enableCountrateCorrection() = 0;
    virtual void disableCountrateCorrection() = 0;
    virtual bool countrateCorrectionEnabled() const = 0;
    virtual void enableFlatfield() = 0;
    virtual void disableFlatfield() = 0;
    virtual bool flatfieldEnabled() const = 0;
    virtual void enableInterpolation() = 0;
    virtual void disableInterpolation() = 0;
    virtual bool interpolationEnabled() const = 0;

    // acquisition parameters
    virtual BitDepth bitDepth() const = 0;
    virtual void setBitDepth(BitDepth depth) = 0;
    virtual ChargeSumming chargeSumming() const = 0;
    virtual void setChargeSumming(ChargeSumming cs) = 0;
    virtual CounterMode counterMode() const = 0;
    virtual void setCounterMode(CounterMode cm) = 0;
    virtual OperationMode operationMode() const = 0;
    virtual void setOperationMode(const OperationMode& mode) = 0;
    virtual std::vector<double> thresholds() const = 0;
    virtual void setThresholds(const std::vector<double>& thresholds_kev) = 0;
    virtual double beamEnergy() const = 0;
    virtual void setBeamEnergy(double e_kev) = 0;
    virtual TrigMode  triggerMode() const = 0;
    virtual void setTriggerMode(TrigMode mode) = 0;
    virtual int framesPerTrigger() const = 0;
    virtual void setFramesPerTrigger(int n) = 0;
    virtual Gating  gatingMode() const = 0;
    virtual void setGatingMode(Gating mode) = 0;
    virtual int roiRows() const = 0;
    virtual void setRoiRows(int rows) = 0;

    // for test and calibration only
    virtual std::vector<int> chipNumbers(int module_nr) const = 0;
    virtual std::vector<std::uint8_t> ioDelay(int module_nr) const = 0;
    virtual std::vector<std::uint8_t> ioDelay(int module_nr, int chip_nr) const = 0;
    virtual void setIoDelay(int module_nr, const std::vector<std::uint8_t>& values) = 0;
    virtual void setIoDelay(int module_nr, int chip_nr, const std::vector<std::uint8_t>& values) = 0;
    virtual void setVoltage(int module_nr, double voltage) = 0;
    virtual std::vector<unsigned> rawThresholds(int module_nr, int chip_nr) const = 0;
    virtual void setRawThresholds(int module_nr, int chip_nr, const std::vector<unsigned>& thresholds_9bit) = 0;
    virtual std::vector<std::uint8_t> dacDisc(int module_nr, int chip_nr) const = 0;
    virtual void setDacDisc(int module_nr, int chip_nr, std::vector<std::uint8_t> values) = 0;
    virtual std::vector<std::uint8_t> pixelMaskBit(int module_nr, int chip_nr) const = 0;
    virtual void setPixelMaskBit(int module_nr, int chip_nr, const std::vector<std::uint8_t>& values) = 0;
    virtual std::vector<std::uint8_t> pixelTestBit(int module_nr, int chip_nr) const = 0;
    virtual void setPixelTestBit(int module_nr, int chip_nr, const std::vector<std::uint8_t>& values) = 0;
    virtual std::vector<std::uint8_t> pixelDiscL(int module_nr, int chip_nr) const = 0;
    virtual void setPixelDiscL(int module_nr, int chip_nr, const std::vector<std::uint8_t>& values) = 0;
    virtual std::vector<std::uint8_t> pixelDiscH(int module_nr, int chip_nr) const = 0;
    virtual void setPixelDiscH(int module_nr, int chip_nr, const std::vector<std::uint8_t>& values) = 0;
    virtual double dacOut(int module_nr, int chip_nr) const = 0;
    virtual void setDacIn(int module_nr, int chip_nr, double voltage) = 0;
    virtual void loadTestPattern(const std::vector<std::uint16_t>& pattern) = 0;
    virtual void readTestPattern() = 0;
    virtual void enableTestMode() = 0;
    virtual void disableTestMode() = 0;
    virtual bool testModeEnabled() const = 0;
    virtual void enableLookup() = 0;
    virtual void disableLookup() = 0;
    virtual bool lookupEnabled() const = 0;
    virtual void enableEqualization() = 0;
    virtual void disableEqualization() = 0;
    virtual bool equalizationEnabled() const = 0;
    virtual void selectDiscL() = 0;
    virtual void selectDiscH() = 0;
};

class Receiver : virtual public xsp::Receiver
{
public:
    virtual ~Receiver() = default;

    // general infos
    virtual Position position() const = 0;
    virtual Rotation rotation() const = 0;
    virtual int maxFrames() const = 0;

    // decoding settings
    virtual int numberOfSubFrames() const = 0;
    virtual int numberOfConnectedSensors() const = 0;
    virtual bool pixelMaskEnabled() const = 0;
    virtual const std::vector<std::uint32_t>& pixelMask() const = 0;
    virtual bool saturationFlagEnabled() const = 0;
    virtual int saturationThreshold() const = 0;
    virtual const std::vector<int>& saturationThresholdPerPixel() const = 0;
    virtual bool countrateCorrectionEnabled() const = 0;
    virtual bool flatfieldEnabled() const = 0;
    virtual const std::vector<double>& flatfield(Threshold th) const = 0;
    virtual const std::vector<double>& flatfieldError(Threshold th) const = 0;
    virtual std::string flatfieldTimestamp(Threshold th) const = 0;
    virtual std::string flatfieldAuthor(Threshold th) const = 0;
    virtual bool interpolationEnabled() const = 0;
    virtual std::string compressor() const = 0;
    virtual bool compressionEnabled() const = 0;
    virtual int compressionLevel() const = 0;
    virtual void setCompressionLevel(int level) = 0;
    virtual ShuffleMode shuffleMode() const = 0;
    virtual void setShuffleMode(ShuffleMode mode) = 0;

    // acquisition parameters
    virtual double threshold(Threshold th) const = 0;
    virtual double beamEnergy() const = 0;

    // for testing only
    virtual bool testModeEnabled() const = 0;
    virtual bool lookupEnabled() const = 0;
};

} // namespace lambda

} // namespace xsp

#endif //LIBXSP_LIBXSP_H
