#pragma once
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <string>
#include <stdexcept>
#include <chrono>
#include <cstring>
#include <array>
#include <algorithm>

// ********** ROS 2 官方 serial_driver **********
#include "io_context/io_context.hpp"
#include "serial_driver/serial_port.hpp"
#include "serial_driver/serial_driver.hpp"

class MecanumMotorDriver
{
public:
    explicit MecanumMotorDriver(const std::string & device_name,
                                uint32_t baud = 115200,
                                std::chrono::milliseconds timeout = std::chrono::milliseconds(50));
    ~MecanumMotorDriver();

    void writeSpeed(const std::array<int16_t, 4> & pwm);
    std::array<int32_t, 4> readEncoder();

private:
    /* 协议常量 */
    static constexpr uint8_t HEAD1  = 0xAA;
    static constexpr uint8_t HEAD2  = 0x55;
    static constexpr uint8_t WRITE_ID = 0x04;
    static constexpr uint8_t READ_ID  = 0x04;
    static constexpr uint8_t CMD_SPEED = 0x01;
    static constexpr uint8_t CMD_ENC   = 0x04;

    void readThreadFunc();
    std::vector<uint8_t> buildWriteFrame(const std::array<int16_t, 4> & pwm);
    std::vector<uint8_t> buildReadFrame();
    bool waitForEncoder(std::array<int32_t, 4> & enc,
                        std::chrono::milliseconds timeout);
    static uint8_t crc8(const std::vector<uint8_t> & data, size_t len);

    /* 成员变量 */
    io_context::IoContext io_ctx_;      // 必须最先构造
    drivers::serial_driver::SerialDriver driver_;
    std::shared_ptr<drivers::serial_driver::SerialPort> port_;

    std::thread thread_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool stop_ = false;

    std::vector<uint8_t> rxBuf_;
    std::array<std::queue<int32_t>, 4> encQueues_;
};