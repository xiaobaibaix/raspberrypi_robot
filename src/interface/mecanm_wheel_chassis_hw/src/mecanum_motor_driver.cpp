#include "mecanum_wheel_chassis_hw/mecanum_motor_driver.hpp"
#include <iostream>
#include <spdlog/spdlog.h>

using namespace std::chrono;

enum RecvDataStatus{
    DATA_hand1,
    DATA_hand2,
    DATA_equipment,
    DATA_len,
    DATA_CRC,
    DTAT_data
};

enum EquipmentType{
    Led,
    Key,
    SteeringGear,
    EncodeMotor=0x04,
    None
};

enum MotorCMDType{
    SET_pwm=0x01,
    GET_encode=0x04,
};

static constexpr uint8_t HEAD1  = 0xAA;
static constexpr uint8_t HEAD2  = 0x55;

/* ---------- 构造/析构 ---------- */
MecanumMotorDriver::MecanumMotorDriver(const std::string & device_name,uint32_t baud)
    : io_ctx_(2),  // IoContext 构造函数可能需要参数
      driver_(io_ctx_)  // 传入 io_ctx_ 给 SerialDriver
{
    /* 配置串口参数 */
    drivers::serial_driver::SerialPortConfig cfg(
        baud,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);

    try {
        // 注意：在 ROS 2 Jazzy 中，可能需要直接创建 SerialPort
        // 而不是通过 SerialDriver
        port_ = std::make_shared<drivers::serial_driver::SerialPort>(
            io_ctx_, device_name, cfg);
        
        // 打开串口
        port_->open();
        
        std::cout << "Serial port opened successfully: " << device_name 
                  << " at " << baud << " baud" << std::endl;
    } catch (const std::exception & e) {
        std::cerr << "Error opening serial port: " << e.what() << std::endl;
        throw std::runtime_error("Cannot open serial port: " + device_name + " - " + e.what());
    }

    if (!port_->is_open()) {
        throw std::runtime_error("Cannot open serial port: " + device_name);
    }

    /* 启动解析线程 */
    thread_ = std::thread(&MecanumMotorDriver::readThreadFunc, this);
    std::cout << "Read thread started" << std::endl;
}

MecanumMotorDriver::~MecanumMotorDriver()
{
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = true;
    }
    cv_.notify_all();
    
    if (thread_.joinable()) {
        thread_.join();
        std::cout << "Read thread joined" << std::endl;
    }
    
    if (port_ && port_->is_open()) {
        port_->close();
        std::cout << "Serial port closed" << std::endl;
    }
}

/* ---------- 公有接口 ---------- */
void MecanumMotorDriver::writeSpeed(const std::array<int16_t, 4> & pwm)
{
    auto frame = buildWriteFrame(pwm);
    try {
        port_->send(frame);  // 只传递 vector
        // std::cout << "Sent speed command, frame size: " << frame.size() << std::endl;
    } catch (const std::exception & e) {
        std::cerr << "Send error: " << e.what() << std::endl;
    }
}

std::array<int32_t, 4> MecanumMotorDriver::readEncoder()
{
    auto frame = buildReadFrame();
    try {
        port_->send(frame);
        // std::cout << "Sent encoder read command, frame size: " << frame.size() << std::endl;
    } catch (const std::exception & e) {
        std::cerr << "Send error: " << e.what() << std::endl;
    }

    std::array<int32_t, 4> enc{};
    if (!waitForEncoder(enc, milliseconds(50))) {
        std::cerr << "readEncoder timeout" << std::endl;
        // 返回默认值
        enc = {0, 0, 0, 0};
    } else {
        // std::cout << "Encoder values: " 
        //           << enc[0] << ", " << enc[1] << ", " 
        //           << enc[2] << ", " << enc[3] << std::endl;
    }
    return enc;
}

/* ---------- 私有实现 ---------- */
std::vector<uint8_t> MecanumMotorDriver::buildWriteFrame(const std::array<int16_t, 4> & pwm)
{
    // 0xaa 0x55 motor(1) datalen(1) cmd(1) timer(2) motorszie(1) | id(1) data(2) id(1) data(2) id(1) data(2) id(1) data(2) crc(1)
    std::vector<uint8_t> f;
    f.reserve(21);
    f.insert(f.end(), {HEAD1, HEAD2, EquipmentType::EncodeMotor});

    std::vector<uint8_t> data;
    data.push_back(MotorCMDType::SET_pwm);
    data.push_back(0x0a);
    data.push_back(0x00);
    data.push_back(4);
    for (size_t i = 0; i < 4; ++i)
    {
        data.push_back(static_cast<uint8_t>(i));
        data.push_back(static_cast<uint8_t>(pwm[i] & 0xFF));
        data.push_back(static_cast<uint8_t>((pwm[i] >> 8) & 0xFF));
    }
    f.push_back(data.size());                 // 数据长度
    f.insert(f.end(),
        std::make_move_iterator(data.begin()),
        std::make_move_iterator(data.end()));
    f.push_back(crc8(f, f.size()));
    return f;
}

std::vector<uint8_t> MecanumMotorDriver::buildReadFrame()
{
    // 0xaa 0x55 motor(1) datalen(1) cmd(1) timer(2) motorszie(1) | id(1) id(1) id(1) id(1) crc(1)
    // back :0xaa 0x55 motor(1) datalen(1) cmd(1) timer(2) motorszie(1) | id(1) data(4) id(1) data(4) id(1) data(4) id(1) data(4) crc(1)
    std::vector<uint8_t> f;
    f.insert(f.end(), {HEAD1, HEAD2, EquipmentType::EncodeMotor});
    std::vector<uint8_t> data;
    data.insert(data.end(),{MotorCMDType::GET_encode, 0x00, 0x00, 0x04, 0x01, 0x02, 0x03, 0x04});
    f.push_back(data.size());
    f.insert(f.end(),
        std::make_move_iterator(data.begin()),
        std::make_move_iterator(data.end()));
    f.push_back(crc8(f, f.size()));
    return f;
}

uint8_t MecanumMotorDriver::crc8(const std::vector<uint8_t> & data, size_t len)
{
    uint8_t crc = 0xFF;
    size_t data_len = std::min(len, data.size());
    for (size_t i = 0; i < data_len; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1;
        }
    }
    return crc;
}


/* ---------- 后台解析线程 ---------- */
void MecanumMotorDriver::readThreadFunc()
{
    std::vector<uint8_t> raw;
    raw.reserve(512);
    
    RecvDataStatus status=RecvDataStatus::DATA_hand1;

    std::unordered_map<EquipmentType, std::function<void(const std::vector<uint8_t>& data)>> handlers;

    handlers[EquipmentType::EncodeMotor]=[&](const std::vector<uint8_t>& data){
        int count=data.at(0);
        for(int i=0;i<count;i++){
            int32_t encode_dat=(data.at(i*5+2))|
                                (data.at(i*5+3)<<8)|
                                (data.at(i*5+4)<<16)|
                                (data.at(i*5+5)<<24);
            if(data.at(i*5+1)>4)continue;
            this->encQueues_.at(data.at(i*5+1)-1).push(encode_dat);
        }
    };
    handlers[EquipmentType::Led]=[&](const std::vector<uint8_t>& data){
        (void)data;
        
    };

    handlers[EquipmentType::Key]=[&](const std::vector<uint8_t>& data){
        (void)data;

    };

    handlers[EquipmentType::SteeringGear]=[&](const std::vector<uint8_t>& data){
        (void)data;

    };

    while (true)
    {
        {
            std::unique_lock<std::mutex> lk(mtx_);
            if (stop_) break;
            lk.unlock();  // 解锁后执行 IO 操作
        }

        /* 接收数据 */
        std::vector<uint8_t> chunk(128);
        try {
            size_t n = port_->receive(chunk);
            
            if (n > 0) {
                // 调整 chunk 大小并添加到 raw
                chunk.resize(n);
                raw.insert(raw.end(), chunk.begin(), chunk.end());
                // std::cout << "Received " << n << " bytes, total: " << raw.size() << std::endl;
            }
        } catch (const std::exception & e) {
            std::cerr << "Receive error: " << e.what() << std::endl;
            // 短暂延迟后继续
            std::this_thread::sleep_for(milliseconds(10));
            continue;
        }

        /* 拆帧逻辑 */
        while (raw.size()>0)
        {
            switch (status)
            {
                case RecvDataStatus::DATA_hand1:
                {
                    auto it = std::find(raw.begin(), raw.end(), HEAD1);
                    if (it == raw.end()) {
                        raw.clear();
                        break;
                    }
                    if (it != raw.begin()) {
                        raw.erase(raw.begin(), it);
                    }
                    status=RecvDataStatus::DATA_hand2;
                }
                break;
                case RecvDataStatus::DATA_hand2:
                {
                    if(raw.at(1)==HEAD2){
                        status=RecvDataStatus::DATA_equipment;
                    }else{
                        raw.erase(raw.begin());
                        status=RecvDataStatus::DATA_hand1;
                    }
                }
                break;
                case RecvDataStatus::DATA_equipment:
                {
                    if(raw.at(2)<=EquipmentType::None){
                        status=RecvDataStatus::DATA_len;
                    }else{
                        status=RecvDataStatus::DATA_hand1;
                    }
                }
                break;
                case RecvDataStatus::DATA_len:
                {
                    int len=raw.size()-4;
                    if(raw.at(3)>=len){
                        status=RecvDataStatus::DATA_CRC;
                    }else{
                        break;
                    }
                }
                break;
                case RecvDataStatus::DATA_CRC:
                {
                    uint8_t crc=raw.at(3+raw.at(3)+1);
                    if (crc8(std::vector<uint8_t>(raw.begin()+3,raw.begin()+3+raw.at(3)),3+raw.at(3))!= crc) {
                        std::cerr << "CRC error, skipping frame" << std::endl;
                        raw.erase(raw.begin());
                        status=RecvDataStatus::DATA_hand1;
                    }else{
                        status=RecvDataStatus::DTAT_data;
                    }
                }
                break;
                case RecvDataStatus::DTAT_data:
                {
                    auto it=handlers.find((EquipmentType)raw.at(2));
                    if(it!=handlers.end()){
                        it->second(std::vector<uint8_t>(raw.begin()+3,raw.begin()+3+raw.at(3)));
                        raw.erase(raw.begin(),raw.begin()+2);
                        status=RecvDataStatus::DATA_hand1;
                    }else{
                        std::cerr << "handlers use: " << (EquipmentType)raw.at(2) << std::endl;
                    }
                }
                break;
            }   
        }
        // 短暂延迟
        std::this_thread::sleep_for(milliseconds(1));
    }
    std::cout << "Read thread exiting" << std::endl;
}

/* ---------- 同步等待 4 路编码器 ---------- */
bool MecanumMotorDriver::waitForEncoder(std::array<int32_t, 4> & enc,milliseconds timeout)
{
    auto deadline = steady_clock::now() + timeout;
    std::unique_lock<std::mutex> lk(mtx_);
    bool ok = cv_.wait_until(lk, deadline, [&] {
        for (const auto & q : encQueues_) {
            if (q.empty()) return false;
        }
        return true;
    });
    
    if (ok) {
        for (size_t i = 0; i < 4; ++i) {
            enc[i] = encQueues_[i].front();
            encQueues_[i].pop();
        }
    }
    return ok;
}