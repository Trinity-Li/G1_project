/**
 * G1 Dual Window Arm Control (Final Robust Version)
 * * User Coordinate System:
 * X+: Right (向右)
 * Y+: Forward (向前)
 * Z+: Up (向上)
 * * Features:
 * - Spherical Coordinate IK (No Gimbal Lock)
 * - Safety Clamping (Prevents Rebound)
 * - Singularity Protection
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
#include <algorithm> // for clamping logic

// Unitree SDK Headers
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

// ==========================================
// 1. 机器人参数与配置
// ==========================================
const float L_UPPER = 0.20f;   
const float L_FOREARM = 0.22f; 
const int G1_NUM_MOTOR = 29;

// 刚度设置 (Arm Soft Mode)
std::array<float, G1_NUM_MOTOR> Kp_User = {
    60, 60, 60, 100, 40, 40,      // Left Leg
    60, 60, 60, 100, 40, 40,      // Right Leg
    60, 40, 40,                   // Waist
    40, 40, 40, 40, 40, 40, 40,   // Left Arm
    40, 40, 40, 40, 40, 40, 40    // Right Arm
};

// 阻尼设置
std::array<float, G1_NUM_MOTOR> Kd_User = {
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    1, 1, 1,
    2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2
};

enum G1JointIndex {
    R_ShPitch = 22, R_ShRoll = 23, R_ShYaw = 24,
    R_Elbow = 25,
    R_WrRoll = 26, R_WrPitch = 27, R_WrYaw = 28
};

const std::vector<std::string> MOTOR_NAMES = {
    "L_HipPitch", "L_HipRoll ", "L_HipYaw  ", "L_Knee    ", "L_AnkPitch", "L_AnkRoll ",
    "R_HipPitch", "R_HipRoll ", "R_HipYaw  ", "R_Knee    ", "R_AnkPitch", "R_AnkRoll ",
    "WaistYaw  ", "WaistRoll ", "WaistPitch",
    "L_ShPitch ", "L_ShRoll  ", "L_ShYaw   ", "L_Elbow   ", "L_WrRoll  ", "L_WrPitch ", "L_WrYaw   ",
    "R_ShPitch ", "R_ShRoll  ", "R_ShYaw   ", "R_Elbow   ", "R_WrRoll  ", "R_WrPitch ", "R_WrYaw   "
};

// ==========================================
// 2. 基础工具
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
    std::array<float, G1_NUM_MOTOR> q;
    std::array<float, G1_NUM_MOTOR> dq;
};

// ==========================================
// 3. Monitor (显示当前状态)
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
                std::cout << "\033[2J\033[H"; 
                std::cout << "========================================\n";
                std::cout << "      G1 ROBOT MOTOR STATES (ALL)       \n";
                std::cout << "========================================\n";
                std::cout << " ID | Name       | Pos(deg) | Vel(rad/s)\n";
                std::cout << "----|------------|----------|-----------\n";
                for(int i=0; i<G1_NUM_MOTOR; ++i) {
                    std::string name = (i < MOTOR_NAMES.size()) ? MOTOR_NAMES[i] : "Unknown";
                    float deg = data->q[i] * 180.0f / M_PI;
                    // 高亮右臂
                    if(i >= 22 && i <= 28)
                        printf("\033[1;32m%3d | %-10s | %8.2f | %8.2f\033[0m\n", i, name.c_str(), deg, data->dq[i]);
                    else
                        printf("%3d | %-10s | %8.2f | %8.2f\n", i, name.c_str(), deg, data->dq[i]);
                }
                std::cout << "========================================\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
};

// ==========================================
// 4. Controller (逆运动学控制)
// ==========================================
class G1Controller {
    ChannelPublisherPtr<LowCmd_> pub_;
    ChannelSubscriberPtr<LowState_> sub_;
    DataBuffer<MotorData> state_buf_;
    ThreadPtr cmd_thread_, ctrl_thread_;
    std::mutex target_mtx_;
    std::array<float, G1_NUM_MOTOR> q_target_final_;
    std::array<float, G1_NUM_MOTOR> q_target_current_;
    bool initialized_ = false;
    uint8_t mode_machine_ = 0;

public:
    G1Controller(std::string net) {
        ChannelFactory::Instance()->Init(0, net);
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

    void StateHandler(const void *msg) {
        auto state = *(const LowState_ *)msg;
        if (state.crc() != Crc32Core((uint32_t *)&state, (sizeof(LowState_) >> 2) - 1)) return;
        MotorData md;
        for(int i=0; i<G1_NUM_MOTOR; ++i) md.q[i] = state.motor_state()[i].q();
        state_buf_.Set(md);
        if(state.mode_machine() != mode_machine_) mode_machine_ = state.mode_machine();
    }

    void CtrlLoop() {
        auto data = state_buf_.Get();
        if(!data) return;
        if(!initialized_) {
            std::unique_lock<std::mutex> lock(target_mtx_);
            q_target_final_ = data->q;
            q_target_current_ = data->q;
            initialized_ = true;
            std::cout << "[INFO] Robot Locked. Ready.\n";
        }
        std::unique_lock<std::mutex> lock(target_mtx_);
        // 插值平滑系数
        float alpha = 0.005f; 
        for(int i=0; i<G1_NUM_MOTOR; ++i) {
            q_target_current_[i] = (1.0f - alpha) * q_target_current_[i] + alpha * q_target_final_[i];
        }
    }

    void CmdLoop() {
        if(!initialized_) return;
        LowCmd_ cmd;
        cmd.mode_machine() = mode_machine_;
        std::unique_lock<std::mutex> lock(target_mtx_);
        auto current_q = q_target_current_;
        lock.unlock();

        for(int i=0; i<G1_NUM_MOTOR; ++i) {
            cmd.motor_cmd()[i].mode() = 1; 
            cmd.motor_cmd()[i].q() = current_q[i];
            cmd.motor_cmd()[i].kp() = Kp_User[i];
            cmd.motor_cmd()[i].kd() = Kd_User[i];
            cmd.motor_cmd()[i].tau() = 0;
            cmd.motor_cmd()[i].dq() = 0;
        }
        cmd.crc() = Crc32Core((uint32_t *)&cmd, (sizeof(LowCmd_) >> 2) - 1);
        pub_->Write(cmd);
    }

    // ----------------------------------------------------------------------
    // 逆运动学核心求解 (Spherical + Protection)
    // ----------------------------------------------------------------------
    bool SolveIK(float x1, float y1, float z1, float x2, float y2, float z2) {
        // 1. 长度校验
        float l1 = std::sqrt(x1*x1 + y1*y1 + z1*z1);
        float l2 = std::sqrt(x2*x2 + y2*y2 + z2*z2);
        
        if (std::abs(l1 - L_UPPER) > 0.03f) {
            printf("[ERROR] Upper len %.3f != %.2f\n", l1, L_UPPER);
            return false;
        }
        if (std::abs(l2 - L_FOREARM) > 0.03f) {
            printf("[ERROR] Forearm len %.3f != %.2f\n", l2, L_FOREARM);
            return false;
        }

        // 2. Shoulder Roll (左右张开) - 使用 asin 避免死锁
        // User X+ (Right) -> Roll -90 deg
        float val_roll = -x1 / l1;
        // 数值保护
        if (val_roll > 1.0f) val_roll = 1.0f;
        if (val_roll < -1.0f) val_roll = -1.0f;
        
        float q_sh_roll = std::asin(val_roll);

        // 3. Shoulder Pitch (前后上下) - 使用 atan2
        // User Y+ (Fwd) -> Pitch -90
        // User Z- (Down) -> Pitch 0
        // 奇异点保护: 当手臂完全侧向水平时 (yz_norm -> 0)，Pitch 保持 0
        float yz_norm = std::sqrt(y1*y1 + z1*z1);
        float q_sh_pitch = 0.0f;

        if (yz_norm < 0.02f) {
            q_sh_pitch = 0.0f; 
        } else {
            q_sh_pitch = std::atan2(-y1, -z1);
        }

        // 4. Shoulder Yaw (小臂弯曲平面)
        // 投影法: 计算 v2 在垂直于 v1 的平面上的投影
        float inv_l1 = 1.0f / l1;
        // v1 单位向量
        float ux = x1 * inv_l1, uy = y1 * inv_l1, uz = z1 * inv_l1;
        float dot = x2 * ux + y2 * uy + z2 * uz;
        // 投影向量 p
        float px = x2 - dot * ux;
        float py = y2 - dot * uy;
        float pz = z2 - dot * uz; // 虽然不用pz，但为了完整性

        // 简化 Yaw 计算: 假设大臂主要垂直，主要看水平投影
        // Palm In (X=0) -> Yaw 0
        // Palm Fwd (X>0 in Robot Frame logic) -> Yaw -90
        // 这里的 px, py 是 User Frame 的投影
        float q_sh_yaw = std::atan2(-px, py);

        // 5. Elbow (肘部)
        // 1.57 (直) -> 0 (弯)
        float cos_alpha = (x1*x2 + y1*y2 + z1*z2) / (l1 * l2);
        if(cos_alpha > 1.0f) cos_alpha = 1.0f;
        if(cos_alpha < -1.0f) cos_alpha = -1.0f;
        float alpha = std::acos(cos_alpha);
        float q_elbow = 1.5708f - alpha;

        // 6. 安全限位 (Safety Clamping)
        // 防止计算出的角度超出物理极限导致回弹
        const float P_MIN = -2.8f; // 后/上
        const float P_MAX =  1.0f; // 前/下限
        const float R_MIN = -2.0f; // 外张
        const float R_MAX =  0.5f; // 内收
        const float E_MIN = -0.5f; 
        const float E_MAX =  2.8f; 

        q_sh_pitch = std::fmax(P_MIN, std::fmin(q_sh_pitch, P_MAX));
        q_sh_roll  = std::fmax(R_MIN, std::fmin(q_sh_roll, R_MAX));
        q_elbow    = std::fmax(E_MIN, std::fmin(q_elbow, E_MAX));

        // 赋值
        std::unique_lock<std::mutex> lock(target_mtx_);
        q_target_final_[R_ShPitch] = q_sh_pitch;
        q_target_final_[R_ShRoll]  = q_sh_roll;
        q_target_final_[R_ShYaw]   = q_sh_yaw;
        q_target_final_[R_Elbow]   = q_elbow;
        q_target_final_[R_WrPitch] = 0.0f; 
        
        // 调试打印
        printf("In: (%.2f, %.2f, %.2f)\n", x1, y1, z1);
        printf("Out: P=%.2f, R=%.2f, Y=%.2f, E=%.2f\n", q_sh_pitch, q_sh_roll, q_sh_yaw, q_elbow);

        return true;
    }
};

// ==========================================
// Main
// ==========================================
int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: sudo ./g1_ac [network_interface]" << std::endl;
        return 0;
    }
    std::string net = argv[1];

    std::cout << "Select Mode: [1] Monitor  [2] Control" << std::endl;
    int choice; std::cin >> choice;

    if (choice == 1) {
        G1Monitor mon(net);
        mon.StartLoop();
    } 
    else if (choice == 2) {
        G1Controller ctrl(net);
        std::cout << "Initializing...\n";
        sleep(2);
        
        // 清理输入缓冲区
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        while(true) {
            std::cout << "\n=== User Frame: X=Right, Y=Fwd, Z=Up ===\n";
            std::cout << "Format: v [x1] [y1] [z1] [x2] [y2] [z2]\n";
            std::cout << "> ";
            std::string line;
            if(!std::getline(std::cin, line)) break;
            std::stringstream ss(line);
            std::string cmd; ss >> cmd;
            if(cmd == "v") {
                float x1, y1, z1, x2, y2, z2;
                if(ss >> x1 >> y1 >> z1 >> x2 >> y2 >> z2) {
                    ctrl.SolveIK(x1, y1, z1, x2, y2, z2);
                }
            } else if (cmd == "exit") break;
        }
    } 
    return 0;
}