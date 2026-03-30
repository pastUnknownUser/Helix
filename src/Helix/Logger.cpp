#include "Helix/Logger.hpp"
#include "pros/rtos.hpp"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace Helix {

// Global logger instance
Logger gLogger;

Logger::Logger()
    : minLevel_(LogLevel::INFO),
      outputMode_(OutputMode::SERIAL_AND_SD),
      recording_(false),
      sdCardAvailable_(false),
      recordStartTime_(0),
      sdFileOpen_(false) {}

Logger::~Logger() {
    if (sdFileOpen_) {
        stopRecording();
    }
}

bool Logger::initSDCard() {
    // Check if SD card is available by trying to open a test file
    // PROS uses /usd/ prefix for SD card
    std::ifstream testFile("/usd/.test");
    sdCardAvailable_ = testFile.good() || std::ofstream("/usd/.test").good();

    if (sdCardAvailable_) {
        info("SD card detected");
    } else {
        info("No SD card detected - logging to terminal only");
    }

    return sdCardAvailable_;
}

std::string Logger::generateLogFileName() {
    // Generate unique filename based on timestamp
    uint32_t timestamp = pros::millis();
    std::stringstream ss;
    ss << "/usd/log_" << std::setfill('0') << std::setw(5) << (timestamp / 1000) << ".csv";
    return ss.str();
}

void Logger::log(LogLevel level, const char* msg) {
    if (static_cast<int>(level) < static_cast<int>(minLevel_)) {
        return;
    }

    const char* levelStr = levelToString(level);

    // Always log to serial
    if (outputMode_ == OutputMode::SERIAL_ONLY || outputMode_ == OutputMode::SERIAL_AND_SD) {
        std::printf("[%s] %s\n", levelStr, msg);
    }

    // Also write to SD if available and mode allows
    if ((outputMode_ == OutputMode::SD_CARD_ONLY || outputMode_ == OutputMode::SERIAL_AND_SD)
        && sdCardAvailable_ && sdFileOpen_) {
        writeToSD(msg);
    }
}

void Logger::writeToSD(const char* msg) {
    if (sdFileOpen_ && sdFile_.is_open()) {
        sdFile_ << msg << std::endl;
        sdFile_.flush();
    }
}

void Logger::writeMotionToSD(const MotionData& data) {
    if (sdFileOpen_ && sdFile_.is_open()) {
        sdFile_ << data.timestamp << ","
                << data.target << ","
                << data.current << ","
                << data.error << ","
                << data.output << ","
                << data.velocity << ","
                << data.leftMotor << ","
                << data.rightMotor << ","
                << data.x << ","
                << data.y << ","
                << data.theta << std::endl;

        // Flush periodically to ensure data is written
        if (data.timestamp % 500 < 20) {  // Every ~500ms
            sdFile_.flush();
        }
    }
}

const char* Logger::levelToString(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO";
        case LogLevel::WARN:  return "WARN";
        case LogLevel::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void Logger::debug(const char* msg) {
    log(LogLevel::DEBUG, msg);
}

void Logger::info(const char* msg) {
    log(LogLevel::INFO, msg);
}

void Logger::warn(const char* msg) {
    log(LogLevel::WARN, msg);
}

void Logger::error(const char* msg) {
    log(LogLevel::ERROR, msg);
}

void Logger::recordMotion(const MotionData& data) {
    if (!recording_) {
        return;
    }

    MotionData dataCopy = data;
    dataCopy.timestamp = pros::millis() - recordStartTime_;

    // If SD card is available and file is open, write directly
    if (sdCardAvailable_ && sdFileOpen_) {
        writeMotionToSD(dataCopy);
    } else {
        // Otherwise buffer in memory
        motionBuffer_.record(dataCopy);
    }
}

void Logger::startRecording() {
    if (recording_) {
        return;
    }

    recording_ = true;
    recordStartTime_ = pros::millis();
    motionBuffer_.clear();

    // Try to open SD card file if available
    if (sdCardAvailable_ || initSDCard()) {
        currentLogFile_ = generateLogFileName();
        sdFile_.open(currentLogFile_, std::ios::out | std::ios::trunc);

        if (sdFile_.is_open()) {
            sdFileOpen_ = true;
            // Write CSV header
            sdFile_ << "timestamp,target,current,error,output,velocity,leftMotor,rightMotor,x,y,theta" << std::endl;
            std::printf("[INFO] Started recording to SD: %s\n", currentLogFile_.c_str());
        } else {
            sdCardAvailable_ = false;
            sdFileOpen_ = false;
            warn("Failed to open SD card file, buffering in memory");
        }
    } else {
        info("Recording to memory buffer (no SD card)");
    }
}

void Logger::stopRecording() {
    if (!recording_) {
        return;
    }

    recording_ = false;
    uint32_t duration = pros::millis() - recordStartTime_;

    if (sdFileOpen_) {
        sdFile_.flush();
        sdFile_.close();
        sdFileOpen_ = false;
        std::printf("[INFO] Recording saved to %s (%lu ms, ~%zu samples)\n",
                   currentLogFile_.c_str(),
                   static_cast<unsigned long>(duration),
                   duration / 20);  // Approximate sample count at 50Hz
    } else {
        std::printf("[INFO] Recording stopped. Duration: %lu ms, Samples: %zu\n",
                   static_cast<unsigned long>(duration),
                   motionBuffer_.size());
    }
}

uint32_t Logger::getRecordingDuration() const {
    if (!recording_) {
        return 0;
    }
    return pros::millis() - recordStartTime_;
}

void Logger::printMotionHeader() {
    std::printf("timestamp,target,current,error,output,velocity,leftMotor,rightMotor,x,y,theta\n");
}

void Logger::dumpMotionData() {
    if (sdFileOpen_ && sdFile_.is_open()) {
        sdFile_.flush();
        std::printf("[INFO] Data written to SD card: %s\n", currentLogFile_.c_str());
        return;
    }

    if (motionBuffer_.empty()) {
        info("No motion data recorded");
        return;
    }

    std::printf("\n========== MOTION DATA DUMP ==========\n");
    printMotionHeader();

    for (size_t i = 0; i < motionBuffer_.size(); i++) {
        const MotionData& d = motionBuffer_[i];
        std::printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.2f,%.2f,%.2f\n",
                   static_cast<unsigned long>(d.timestamp),
                   d.target, d.current, d.error, d.output,
                   d.velocity, d.leftMotor, d.rightMotor,
                   d.x, d.y, d.theta);
    }

    std::printf("========== END MOTION DATA ==========\n\n");
}

bool Logger::saveToSDCard(const std::string& filename) {
    if (!sdCardAvailable_ && !initSDCard()) {
        error("No SD card available");
        return false;
    }

    std::string fullPath = "/usd/" + filename;
    std::ofstream file(fullPath);

    if (!file.is_open()) {
        error("Failed to open file for writing");
        return false;
    }

    // Write header
    file << "timestamp,target,current,error,output,velocity,leftMotor,rightMotor,x,y,theta" << std::endl;

    // Write data
    for (size_t i = 0; i < motionBuffer_.size(); i++) {
        const MotionData& d = motionBuffer_[i];
        file << d.timestamp << ","
             << d.target << ","
             << d.current << ","
             << d.error << ","
             << d.output << ","
             << d.velocity << ","
             << d.leftMotor << ","
             << d.rightMotor << ","
             << d.x << ","
             << d.y << ","
             << d.theta << std::endl;
    }

    file.close();
    std::printf("[INFO] Saved %zu samples to %s\n", motionBuffer_.size(), fullPath.c_str());
    return true;
}

void Logger::clearMotionData() {
    motionBuffer_.clear();
    info("Motion data cleared");
}

size_t Logger::getMotionSampleCount() const {
    if (sdFileOpen_) {
        // Estimate based on duration
        return (pros::millis() - recordStartTime_) / 20;
    }
    return motionBuffer_.size();
}

// MotionBuffer dump implementations
template<size_t Capacity>
void MotionBuffer<Capacity>::dumpToSerial() const {
    if (empty()) {
        std::printf("No data in buffer\n");
        return;
    }

    Logger::printMotionHeader();
    for (size_t i = 0; i < size(); i++) {
        const MotionData& d = (*this)[i];
        std::printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.2f,%.2f,%.2f\n",
                   static_cast<unsigned long>(d.timestamp),
                   d.target, d.current, d.error, d.output,
                   d.velocity, d.leftMotor, d.rightMotor,
                   d.x, d.y, d.theta);
    }
}

template<size_t Capacity>
void MotionBuffer<Capacity>::dumpToFile(std::ofstream& file) const {
    if (empty() || !file.is_open()) {
        return;
    }

    for (size_t i = 0; i < size(); i++) {
        const MotionData& d = (*this)[i];
        file << d.timestamp << ","
             << d.target << ","
             << d.current << ","
             << d.error << ","
             << d.output << ","
             << d.velocity << ","
             << d.leftMotor << ","
             << d.rightMotor << ","
             << d.x << ","
             << d.y << ","
             << d.theta << std::endl;
    }
}

// Explicit instantiations
template void MotionBuffer<1000>::dumpToSerial() const;
template void MotionBuffer<1000>::dumpToFile(std::ofstream&) const;

} // namespace Helix
