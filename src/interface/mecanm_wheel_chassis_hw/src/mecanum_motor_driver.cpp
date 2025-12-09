#include "mecanum_wheel_chassis_hw/mecanum_motor_driver.hpp"
#include <spdlog/spdlog.h>   // 可改成 printf/rclcpp

using namespace std::chrono;

/* ---------- 构造/析构 ---------- */
MecanumMotorDriver::MecanumMotorDriver(const std::string & device_name,
                                       uint32_t baud,
                                       milliseconds timeout)
    : io_ctx_(2), driver_(io_ctx_)
{
    /* 配置串口参数 */
    drivers::serial_driver::SerialPortConfig cfg(
        baud,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);

    driver_.init_port(device_name, cfg);
    port_ = driver_.port();
    if (!port_->is_open())
        throw std::runtime_error("Cannot open serial port: " + device_name);

    /* 启动解析线程 */
    thread_ = std::thread(&MecanumMotorDriver::readThreadFunc, this);
}

MecanumMotorDriver::~MecanumMotorDriver()
{
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (port_ && port_->is_open()) port_->close();
}

/* ---------- 公有接口 ---------- */
void MecanumMotorDriver::writeSpeed(const std::array<int16_t, 4> & pwm)
{
    auto frame = buildWriteFrame(pwm);
    port_->send(frame.data(), frame.size());
}

std::array<int32_t, 4> MecanumMotorDriver::readEncoder()
{
    auto frame = buildReadFrame();
    port_->send(frame.data(), frame.size());

    std::array<int32_t, 4> enc{};
    if (!waitForEncoder(enc, milliseconds(50)))
        throw std::runtime_error("readEncoder timeout");
    return enc;
}

/* ---------- 私有实现 ---------- */
std::vector<uint8_t> MecanumMotorDriver::buildWriteFrame(const std::array<int16_t, 4> & pwm)
{
    std::vector<uint8_t> f;
    f.reserve(3 + 1 + 1 + 1 + 4 * 3 + 1);
    f.insert(f.end(), {HEAD1, HEAD2, WRITE_ID, CMD_SPEED});
    f.push_back(4 * 3);                 // 数据长度
    for (size_t i = 0; i < 4; ++i)
    {
        f.push_back(static_cast<uint8_t>(i));
        f.push_back(static_cast<uint8_t>(pwm[i] & 0xFF));
        f.push_back(static_cast<uint8_t>((pwm[i] >> 8) & 0xFF));
    }
    f.push_back(crc8(f, f.size()));
    return f;
}

std::vector<uint8_t> MecanumMotorDriver::buildReadFrame()
{
    std::vector<uint8_t> f;
    f.insert(f.end(), {HEAD1, HEAD2, READ_ID, CMD_ENC, 4});
    for (uint8_t i = 0; i < 4; ++i) f.push_back(i);
    f.push_back(crc8(f, f.size()));
    return f;
}

uint8_t MecanumMotorDriver::crc8(const std::vector<uint8_t> & data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1;
    }
    return crc;
}

/* ---------- 后台解析线程 ---------- */
void MecanumMotorDriver::readThreadFunc()
{
    std::vector<uint8_t> raw;
    raw.reserve(512);
    while (true)
    {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (stop_) break;
        }

        /* 同步收一包（阻塞 5 ms） */
        uint8_t chunk[128];
        size_t n = port_->receive(chunk, sizeof(chunk), milliseconds(5));
        if (n) raw.insert(raw.end(), chunk, chunk + n);

        /* 拆帧逻辑完全保持旧代码 */
        while (raw.size() >= 7)
        {
            auto it = std::search(raw.begin(), raw.end(),
                                  &HEAD1, &HEAD1 + 1);
            if (it == raw.end()) { raw.clear(); break; }
            if (it != raw.begin()) raw.erase(raw.begin(), it);
            if (raw.size() < 3 || raw[0] != HEAD1 || raw[1] != HEAD2)
            { raw.erase(raw.begin()); continue; }

            uint8_t len = raw[4];
            if (raw.size() < 5 + len + 1) break;
            if (crc8(raw, 5 + len) != raw[5 + len])
            { raw.erase(raw.begin()); continue; }

            if (raw[3] == CMD_ENC && len == 4 * 5)   // 假设每电机 5 字节
            {
                std::lock_guard<std::mutex> lk(mtx_);
                for (size_t i = 0; i < 4; ++i)
                {
                    int32_t val;
                    std::memcpy(&val, &raw[5 + i * 5 + 1], 4);
                    encQueues_[i].push(val);
                }
                cv_.notify_all();
            }
            raw.erase(raw.begin(), raw.begin() + 6 + len);
        }
    }
}

/* ---------- 同步等待 4 路编码器 ---------- */
bool MecanumMotorDriver::waitForEncoder(std::array<int32_t, 4> & enc,
                                        milliseconds timeout)
{
    auto deadline = steady_clock::now() + timeout;
    std::unique_lock<std::mutex> lk(mtx_);
    bool ok = cv_.wait_until(lk, deadline, [&] {
        return std::all_of(encQueues_.begin(), encQueues_.end(),
                           [](const auto & q) { return !q.empty(); });
    });
    if (ok)
        for (size_t i = 0; i < 4; ++i)
        {
            enc[i] = encQueues_[i].front();
            encQueues_[i].pop();
        }
    return ok;
}