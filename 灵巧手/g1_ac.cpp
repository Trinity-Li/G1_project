/**
 * G1 Dual Window Arm Control
 * * Window 1: Monitor (Shows all motor states)
 * Window 2: Controller (Input vectors for Right Arm)
 * * Dimensions: Upper Arm = 0.20m, Forearm = 0.22m
 */

#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <array>

// Unitree SDK Headers
#include "gamepad.hpp"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

// ==========================================
// 1. 机器人参数定义
// ==========================================
const float L_UPPER = 0.20f;   // 大臂长 20cm
const float L_FOREARM = 0.22f; // 小臂长 22cm
const int G1_NUM_MOTOR = 29;

// Kp (刚度) 设置: 手臂设为软模式(20)以防抖动
std::array<float, G1_NUM_MOTOR> Kp_User = {
    60, 60, 60, 100, 40, 40,      // Left Leg
    60, 60, 60, 100, 40, 40,      // Right Leg
    60, 40, 40,                   // Waist
    20, 20, 20, 20, 20, 20, 20,   // Left Arm
    20, 20, 20, 20, 20, 20, 20    // Right Arm (Soft Control)
};

// Kd (阻尼) 设置: 手臂设为高阻尼(2)以吸收震荡
std::array<float, G1_NUM_MOTOR> Kd_User = {
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    1, 1, 1,
    2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2
};

// 关节 ID 映射
enum G1JointIndex {
    // Right Arm Joints
    R_ShPitch = 22, R_ShRoll = 23, R_ShYaw = 24,
    R_Elbow = 25,
    R_WrRoll = 26, R_WrPitch = 27, R_WrYaw = 28
};

// 所有电机名称 (用于 Monitor 显示)
const std::vector<std::string> MOTOR_NAMES = {
    "L_HipPitch", "L_HipRoll ", "L_HipYaw  ", "L_Knee    ", "L_AnkPitch", "L_AnkRoll ",
    "R_HipPitch", "R_HipRoll ", "R_HipYaw  ", "R_Knee    ", "R_AnkPitch", "R_AnkRoll ",
    "WaistYaw  ", "WaistRoll ", "WaistPitch",
    "L_ShPitch ", "L_ShRoll  ", "L_ShYaw   ", "L_Elbow   ", "L_WrRoll  ", "L_WrPitch ", "L_WrYaw   ",
    "R_ShPitch ", "R_ShRoll  ", "R_ShYaw   ", "R_Elbow   ", "R_WrRoll  ", "R_WrPitch ", "R_WrYaw   "
};

// ==========================================
// 2. 辅助工具
// ==========================================
inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
    uint32_t xbit = 0; uint32_t data = 0; uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31; data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) { CRC32 <<= 1; CRC32 ^= dwPolynomial; } else CRC32 <<= 1;
            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

// 简单的线程安全缓冲区
template <typename T>
class DataBuffer {
    std::shared_ptr<T> data;
    std::shared_mutex mutex;
public:
    void Set(const T &newData) {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data = std::make_shared<T>(newData);
    }
    std::shared_ptr<const T> Get() {
        std::shared_lock<std::shared_mutex> lock(mutex);
        return data;
    }
};

struct MotorData {
    std::array<float, G1_NUM_MOTOR> q;  // Position
    std::array<float, G1_NUM_MOTOR> dq; // Velocity
};

// ==========================================
// 3. 窗口 1: 状态监控器 (Monitor)
// ==========================================
class G1Monitor {
    ChannelSubscriberPtr<LowState_> sub_;
    DataBuffer<MotorData> buffer_;

public:
    G1Monitor(std::string net) {
        ChannelFactory::Instance()->Init(0, net);
        sub_.reset(new ChannelSubscriber<LowState_>("rt/lowstate"));
        sub_->InitChannel(std::bind(&G1Monitor::Handler, this, std::placeholders::_1), 1);
    }

    void Handler(const void *msg) {
        auto state = *(const LowState_ *)msg;
        if (state.crc() != Crc32Core((uint32_t *)&state, (sizeof(LowState_) >> 2) - 1)) return;
        
        MotorData md;
        for(int i=0; i<G1_NUM_MOTOR; ++i) {
            md.q[i] = state.motor_state()[i].q();
            md.dq[i] = state.motor_state()[i].dq();
        }
        buffer_.Set(md);
    }

    void StartLoop() {
        std::cout << "Waiting for robot data...\n";
        while(true) {
            auto data = buffer_.Get();
            if(data) {
                // 清屏并打印
                std::cout << "\033[2J\033[H"; 
                std::cout << "========================================\n";
                std::cout << "      G1 ROBOT MOTOR STATES (ALL)       \n";
                std::cout << "========================================\n";
                std::cout << " ID | Name       | Pos(deg) | Vel(rad/s)\n";
                std::cout << "----|------------|----------|-----------\n";
                
                for(int i=0; i<G1_NUM_MOTOR; ++i) {
                    std::string name = (i < MOTOR_NAMES.size()) ? MOTOR_NAMES[i] : "Unknown";
                    float deg = data->q[i] * 180.0f / M_PI;
                    
                    // 高亮显示右臂 (ID 22-28)
                    if(i >= 22 && i <= 28)
                        printf("\033[1;32m%3d | %-10s | %8.2f | %8.2f\033[0m\n", i, name.c_str(), deg, data->dq[i]);
                    else
                        printf("%3d | %-10s | %8.2f | %8.2f\n", i, name.c_str(), deg, data->dq[i]);
                }
                std::cout << "========================================\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20Hz 刷新
        }
    }
};

// ==========================================
// 4. 窗口 2: 运动控制器 (Controller)
// ==========================================
class G1Controller {
    ChannelPublisherPtr<LowCmd_> pub_;
    ChannelSubscriberPtr<LowState_> sub_;
    DataBuffer<MotorData> state_buf_;
    ThreadPtr cmd_thread_, ctrl_thread_;
    
    std::mutex target_mtx_;
    std::array<float, G1_NUM_MOTOR> q_target_final_; // 用户设定的最终目标
    std::array<float, G1_NUM_MOTOR> q_target_current_; // 插值过程中的当前目标
    
    bool initialized_ = false;
    uint8_t mode_machine_ = 0;

public:
    G1Controller(std::string net) {
        ChannelFactory::Instance()->Init(0, net);
        
        // 释放高级控制权
        auto msc = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
        msc->SetTimeout(5.0f); msc->Init();
        std::string s; 
        while(msc->CheckMode(s, s), !s.empty()) { msc->ReleaseMode(); sleep(1); }

        q_target_final_.fill(0);
        q_target_current_.fill(0);

        pub_.reset(new ChannelPublisher<LowCmd_>("rt/lowcmd"));
        pub_->InitChannel();
        sub_.reset(new ChannelSubscriber<LowState_>("rt/lowstate"));
        sub_->InitChannel(std::bind(&G1Controller::StateHandler, this, std::placeholders::_1), 1);

        cmd_thread_ = CreateRecurrentThreadEx("cmd", UT_CPU_ID_NONE, 2000, &G1Controller::CmdLoop, this);
        ctrl_thread_ = CreateRecurrentThreadEx("ctrl", UT_CPU_ID_NONE, 2000, &G1Controller::CtrlLoop, this);
    }

    // 接收状态用于初始化
    void StateHandler(const void *msg) {
        auto state = *(const LowState_ *)msg;
        if (state.crc() != Crc32Core((uint32_t *)&state, (sizeof(LowState_) >> 2) - 1)) return;
        
        MotorData md;
        for(int i=0; i<G1_NUM_MOTOR; ++i) md.q[i] = state.motor_state()[i].q();
        state_buf_.Set(md);
        if(state.mode_machine() != mode_machine_) mode_machine_ = state.mode_machine();
    }

    // 控制回路 (500Hz): 执行平滑插值
    void CtrlLoop() {
        auto data = state_buf_.Get();
        if(!data) return;

        // 上电初始化：锁定当前位置
        if(!initialized_) {
            std::unique_lock<std::mutex> lock(target_mtx_);
            q_target_final_ = data->q;
            q_target_current_ = data->q;
            initialized_ = true;
            std::cout << "[INFO] Robot Locked. Ready for Vector Input.\n";
        }

        std::unique_lock<std::mutex> lock(target_mtx_);
        // 低通滤波插值 (平滑运动)
        float alpha = 0.002f; // 值越小越慢
        for(int i=0; i<G1_NUM_MOTOR; ++i) {
            q_target_current_[i] = (1.0f - alpha) * q_target_current_[i] + alpha * q_target_final_[i];
        }
    }

    // 发送回路 (500Hz): 发送指令
    void CmdLoop() {
        if(!initialized_) return;

        LowCmd_ cmd;
        cmd.mode_machine() = mode_machine_;
        
        std::unique_lock<std::mutex> lock(target_mtx_); // 读取当前插值目标
        auto current_q = q_target_current_;
        lock.unlock();

        for(int i=0; i<G1_NUM_MOTOR; ++i) {
            cmd.motor_cmd()[i].mode() = 1; // Enable
            cmd.motor_cmd()[i].q() = current_q[i];
            cmd.motor_cmd()[i].kp() = Kp_User[i];
            cmd.motor_cmd()[i].kd() = Kd_User[i];
            cmd.motor_cmd()[i].tau() = 0;
            cmd.motor_cmd()[i].dq() = 0;
        }
        cmd.crc() = Crc32Core((uint32_t *)&cmd, (sizeof(LowCmd_) >> 2) - 1);
        pub_->Write(cmd);
    }

    // 逆运动学求解
    // Input 1: v1 (Upper Arm Vector) relative to Shoulder
    // Input 2: v2 (Forearm Vector) relative to Elbow (Local Frame)
    bool SolveIK(float x1, float y1, float z1, float x2, float y2, float z2) {
        // 1. 校验长度
        float l1 = std::sqrt(x1*x1 + y1*y1 + z1*z1);
        float l2 = std::sqrt(x2*x2 + y2*y2 + z2*z2);
        
        // 允许 10% 的误差，否则拒绝执行以保护机械结构
        if (std::abs(l1 - L_UPPER) > 0.02f) {
            printf("[ERROR] Upper arm vector length %.2fm != %.2fm\n", l1, L_UPPER);
            return false;
        }
        if (std::abs(l2 - L_FOREARM) > 0.02f) {
            printf("[ERROR] Forearm vector length %.2fm != %.2fm\n", l2, L_FOREARM);
            return false;
        }

        // 2. 计算肩部角度 (Shoulder)
        // 假设原点在肩部：Z轴向上，X轴向前，Y轴向左
        // Pitch: 绕Y轴旋转。 atan2(x, -z) -> (0, -0.2)即下垂时为0度
        float q_sh_pitch = std::atan2(x1, -z1);
        // Yaw: 绕Z轴旋转 (侧向抬起)。 atan2(y, -z)
        float q_sh_yaw = std::atan2(y1, -z1);
        
        // 3. 计算肘部角度 (Elbow)
        // 肘部是单自由度(Pitch)，只能改变小臂相对于大臂的夹角
        // 这里的 (x2, y2, z2) 是相对于肘部(大臂末端)的局部坐标
        // 假设：大臂竖直向下时，小臂向前伸直 (0.22, 0, 0) 对应 Elbow=0? 
        // 不，通常 G1 Elbow=0 是伸直状态。
        // 计算 v2 在局部 X-Z 平面上的角度
        // 如果 v2 = (0, 0, -0.22) (向下) -> 角度 0 (伸直)
        // 如果 v2 = (0.22, 0, 0) (向前) -> 角度 90 (弯曲)
        float q_elbow = std::atan2(x2, -z2); 

        // 4. 限制与赋值
        // G1 肘部范围限制 (防止自碰撞)
        q_elbow = std::fmax(0.0f, std::fmin(q_elbow, 2.5f)); 

        std::unique_lock<std::mutex> lock(target_mtx_);
        q_target_final_[R_ShPitch] = q_sh_pitch;
        q_target_final_[R_ShYaw]   = q_sh_yaw;
        q_target_final_[R_ShRoll]  = 0.0f; // 简化：Roll 保持 0
        q_target_final_[R_Elbow]   = q_elbow;
        
        // 手腕保持水平 (可选)
        q_target_final_[R_WrPitch] = 0.0f;
        
        return true;
    }
};

// ==========================================
// 修改后的 main 函数 (交互式选择模式)
// ==========================================
int main(int argc, char const *argv[]) {
    // 1. 只读取 1 个参数：网卡名称
    if (argc < 2) {
        std::cout << "Usage: sudo ./g1_ac [network_interface]" << std::endl;
        std::cout << "Example: sudo ./g1_ac lo" << std::endl;
        return 0;
    }

    std::string net = argv[1];

    // 2. 交互式询问用户模式 (避开 SDK 的参数解析冲突)
    std::cout << "========================================" << std::endl;
    std::cout << "Select Mode:" << std::endl;
    std::cout << "  [1] Monitor  (View Motor States)" << std::endl;
    std::cout << "  [2] Control  (Arm Vector Control)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Enter choice (1 or 2): ";

    int choice;
    std::cin >> choice;

    // 3. 根据输入启动对应功能
    if (choice == 1) {
        std::cout << "Starting Monitor Mode...\n";
        G1Monitor mon(net);
        mon.StartLoop();
    } 
    else if (choice == 2) {
        std::cout << "Starting Control Mode...\n";
        G1Controller ctrl(net);
        
        // 这一段是原来的 Control 逻辑
        std::cout << "Initializing (3s)...\n";
        sleep(3);

        // 清除输入缓冲区的换行符，防止 getline 直接读取空行
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        while(true) {
            std::cout << "\n=== Right Arm Vector Control ===\n";
            std::cout << "Format: v [x1] [y1] [z1] [x2] [y2] [z2]\n";
            std::cout << "Example (Hang): v 0 0 -0.2 0 0 -0.22\n";
            std::cout << "> ";

            std::string line;
            if(!std::getline(std::cin, line)) break;
            
            std::stringstream ss(line);
            std::string cmd; ss >> cmd;
            
            if(cmd == "v") {
                float x1, y1, z1, x2, y2, z2;
                if(ss >> x1 >> y1 >> z1 >> x2 >> y2 >> z2) {
                    if(ctrl.SolveIK(x1, y1, z1, x2, y2, z2)) {
                        std::cout << "Command Sent.\n";
                    }
                } else {
                    std::cout << "Invalid Input.\n";
                }
            } else if (cmd == "exit") {
                break;
            }
        }
    } 
    else {
        std::cout << "Invalid choice. Exiting.\n";
    }

    return 0;
}