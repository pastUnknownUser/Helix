#pragma once

#include <string>
#include <array>
#include <cstdint>
#include <fstream>

namespace Helix {

/**
 * @brief Logging level for telemetry
 */
enum class LogLevel {
    DEBUG = 0,    // Detailed debugging information
    INFO = 1,     // General information
    WARN = 2,     // Warnings
    ERROR = 3     // Errors
};

/**
 * @brief Motion data point for logging PID and motor states
 *
 * Used to record a snapshot of the control system state at a given time.
 */
struct MotionData {
    uint32_t timestamp;     // Milliseconds since start
    float target;             // Target position/heading
    float current;            // Current position/heading
    float error;              // Current error
    float output;             // PID output
    float velocity;           // Current velocity (optional)
    float leftMotor;          // Left motor output
    float rightMotor;         // Right motor output
    float x;                  // X position (if odometry enabled)
    float y;                  // Y position (if odometry enabled)
    float theta;              // Heading (if odometry enabled)

    MotionData()
        : timestamp(0), target(0), current(0), error(0),
          output(0), velocity(0), leftMotor(0), rightMotor(0),
          x(0), y(0), theta(0) {}
};

/**
 * @brief Circular buffer for motion data logging
 *
 * Stores recent motion data points in a fixed-size buffer.
 * When full, oldest data is overwritten.
 *
 * Example:
 * @code
 * MotionBuffer buffer(1000);  // Store last 1000 samples (~20 seconds)
 * buffer.record(currentData);
 *
 * // Later, dump to serial
 * buffer.dumpToSerial();
 * @endcode
 */
template<size_t Capacity>
class MotionBuffer {
public:
    MotionBuffer() : head_(0), count_(0) {}

    /**
     * @brief Record a data point
     */
    void record(const MotionData& data) {
        buffer_[head_] = data;
        head_ = (head_ + 1) % Capacity;
        if (count_ < Capacity) {
            count_++;
        }
    }

    /**
     * @brief Get number of recorded samples
     */
    size_t size() const { return count_; }

    /**
     * @brief Check if buffer is empty
     */
    bool empty() const { return count_ == 0; }

    /**
     * @brief Clear the buffer
     */
    void clear() { head_ = 0; count_ = 0; }

    /**
     * @brief Access data by index (0 = oldest)
     */
    const MotionData& operator[](size_t index) const {
        if (index >= count_) {
            index = count_ - 1;
        }
        size_t actualIndex = (head_ + Capacity - count_ + index) % Capacity;
        return buffer_[actualIndex];
    }

    /**
     * @brief Get the most recent data point
     */
    const MotionData& back() const {
        size_t lastIdx = (head_ + Capacity - 1) % Capacity;
        return buffer_[lastIdx];
    }

    /**
     * @brief Dump all data to serial as CSV
     */
    void dumpToSerial() const;

    /**
     * @brief Dump all data to a file stream
     */
    void dumpToFile(std::ofstream& file) const;

    /**
     * @brief Get the buffer capacity
     */
    static constexpr size_t capacity() { return Capacity; }

private:
    std::array<MotionData, Capacity> buffer_;
    size_t head_;
    size_t count_;
};

/**
 * @brief Logger for PID tuning and debugging
 *
 * Records motion data during movements for post-match analysis.
 * Automatically writes to SD card if present, otherwise logs to terminal.
 *
 * SD Card Support:
 * - Automatically detects if SD card is inserted
 * - Creates timestamped log files (e.g., "log_00123.csv")
 * - Writes CSV data in real-time (not buffered in memory)
 * - Falls back to terminal output if no SD card
 *
 * Example:
 * @code
 * // In initialize():
 * Helix::Logger logger;
 * logger.setLevel(Helix::LogLevel::DEBUG);
 * logger.initSDCard();  // Optional: check if SD card available
 *
 * // Start recording
 * logger.startRecording();
 *
 * // Run autonomous...
 *
 * // Stop and save
 * logger.stopRecording();  // Automatically saves to SD if present
 * @endcode
 */
class Logger {
public:
    /**
     * @brief Output mode for logging
     */
    enum class OutputMode {
        SERIAL_ONLY,      // Log to terminal only
        SD_CARD_ONLY,     // Log to SD card only (if available)
        SERIAL_AND_SD     // Log to both (default)
    };

    Logger();
    ~Logger();

    /**
     * @brief Initialize SD card and check if available
     *
     * @return true if SD card is present and writable
     *
     * Call this in initialize() to set up SD card logging.
     * If not called, SD card will be checked on first write.
     */
    bool initSDCard();

    /**
     * @brief Check if SD card is available
     */
    bool isSDCardAvailable() const { return sdCardAvailable_; }

    /**
     * @brief Get the current log file path (if SD card logging)
     */
    std::string getLogFilePath() const { return currentLogFile_; }

    /**
     * @brief Set output mode
     */
    void setOutputMode(OutputMode mode) { outputMode_ = mode; }

    /**
     * @brief Set minimum log level
     */
    void setLevel(LogLevel level) { minLevel_ = level; }

    /**
     * @brief Log a debug message
     */
    void debug(const char* msg);
    void debug(const std::string& msg) { debug(msg.c_str()); }

    /**
     * @brief Log an info message
     */
    void info(const char* msg);
    void info(const std::string& msg) { info(msg.c_str()); }

    /**
     * @brief Log a warning
     */
    void warn(const char* msg);
    void warn(const std::string& msg) { warn(msg.c_str()); }

    /**
     * @brief Log an error
     */
    void error(const char* msg);
    void error(const std::string& msg) { error(msg.c_str()); }

    /**
     * @brief Record motion data (if recording enabled)
     *
     * Writes to SD card in real-time if available, otherwise buffers.
     */
    void recordMotion(const MotionData& data);

    /**
     * @brief Start recording motion data
     *
     * Opens log file on SD card if available.
     */
    void startRecording();

    /**
     * @brief Stop recording motion data
     *
     * Closes log file and flushes buffer.
     */
    void stopRecording();

    /**
     * @brief Check if currently recording
     */
    bool isRecording() const { return recording_; }

    /**
     * @brief Get recording duration in milliseconds
     */
    uint32_t getRecordingDuration() const;

    /**
     * @brief Dump recorded motion data
     *
     * If SD card logging, this flushes any remaining data.
     * Otherwise, dumps to serial.
     */
    void dumpMotionData();

    /**
     * @brief Dump motion data to SD card file
     *
     * @param filename Name of file (will be prefixed with /usd/)
     * @return true if successful
     */
    bool saveToSDCard(const std::string& filename);

    /**
     * @brief Clear recorded motion data
     */
    void clearMotionData();

    /**
     * @brief Get number of recorded samples
     */
    size_t getMotionSampleCount() const;

    /**
     * @brief Print CSV header for motion data
     */
    static void printMotionHeader();

private:
    LogLevel minLevel_;
    OutputMode outputMode_;
    bool recording_;
    bool sdCardAvailable_;
    uint32_t recordStartTime_;
    std::string currentLogFile_;

    // SD card file stream
    std::ofstream sdFile_;
    bool sdFileOpen_;

    // Fixed-size buffer for motion data (used when SD not available)
    static constexpr size_t BUFFER_SIZE = 1000;
    MotionBuffer<BUFFER_SIZE> motionBuffer_;

    void log(LogLevel level, const char* msg);
    void writeToSD(const char* msg);
    void writeMotionToSD(const MotionData& data);
    const char* levelToString(LogLevel level);
    std::string generateLogFileName();

    // Prevent copying
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
};

/**
 * @brief Global logger instance (optional)
 *
 * Use this if you don't need multiple loggers
 */
extern Logger gLogger;

} // namespace Helix
