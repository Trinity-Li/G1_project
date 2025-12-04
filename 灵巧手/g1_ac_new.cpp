/**
 * G1 Dual Window Arm Control (Position Mode Integrated)
 * Features:
 * 1. Vector Control (v x1 y1 z1 x2 y2 z2)
 * 2. Position Control (p x y z) -> Auto-calculates Elbow
 * * User Coordinate System: X=Right, Y=Forward, Z=Up
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
#include <algorithm>

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
// 1. 机器人参数
// ==========================================
const float L_UPPER = 0.20f;   
const float L_FOREARM = 0.22f; 
const int G1_NUM_MOTOR = 29;

std::array<float, G1_NUM_MOTOR> Kp_User = {
    60, 60, 60, 100, 40, 40,      // Legs
    60, 60, 60, 100, 40, 40,      
    60, 40, 40,                   // Waist
    40, 40, 40, 40, 40, 40, 40,   // Left Arm
    40, 40, 40, 40, 40, 40, 40    // Right Arm
};

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
// 3. Monitor
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
                std::cout << "G1 ROBOT STATES\n";
                // 简化显示
                for(int i=22; i<=28; ++i) {
                     printf("R_ARM_%d: %.2f\n", i, data->q[i]);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

// ==========================================
// 4. Controller (整合数学求解)
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
        float alpha = 0.01f; // 响应速度
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
    // 核心函数 1: 计算肘部坐标 (替代你的 Python 脚本)
    // ----------------------------------------------------------------------
    // 输入: 手部坐标 target (x,y,z)
    // 输出: 调用 SolveIK 执行运动
    bool SolvePosition(float tx, float ty, float tz) {
        // 1. 计算肩部到手部的距离
        float d_sq = tx*tx + ty*ty + tz*tz;
        float d = std::sqrt(d_sq);

        // 2. 三角形不等式检查 (Reachability)
        if (d > (L_UPPER + L_FOREARM) - 0.001f) {
            printf("[ERROR] Target too far! Dist=%.3f, Max=%.3f\n", d, L_UPPER+L_FOREARM);
            return false;
        }
        if (d < fabs(L_UPPER - L_FOREARM) + 0.001f) {
            printf("[ERROR] Target too close!\n");
            return false;
        }

        // 3. 计算肩部内角 (Law of Cosines)
        // L2^2 = L1^2 + d^2 - 2*L1*d*cos(theta)
        // cos(theta) = (L1^2 + d^2 - L2^2) / (2*L1*d)
        float cos_theta = (L_UPPER*L_UPPER + d_sq - L_FOREARM*L_FOREARM) / (2 * L_UPPER * d);
        // 限制范围防NaN
        if(cos_theta > 1.0f) cos_theta = 1.0f;
        if(cos_theta < -1.0f) cos_theta = -1.0f;
        float theta = std::acos(cos_theta);

        // 4. 确定肘部坐标
        // 我们需要找到一个中间点 E，使得 |E|=L1，|E-T|=L2
        // 并且满足你的 Python 脚本中的偏好：X > 0 (向右/外张)
        
        // 算法：
        // (1) 构建基向量
        // U = T / |T| (指向目标的单位向量)
        float ux = tx/d, uy = ty/d, uz = tz/d;
        
        // (2) 寻找旋转轴
        // 我们需要一个垂直于 U 的向量 V，用来定义“肘部平面”。
        // 为了让 x > 0，我们利用 Z 轴辅助。
        // R = U x (0,0,1)。如果 U 也是竖直的，用 (1,0,0)
        float rx, ry, rz;
        if (std::abs(ux) < 0.01f && std::abs(uy) < 0.01f) {
            // 目标垂直向上/下，旋转轴设为 X 轴 (让肘部向右弯)
            rx = 1.0f; ry = 0.0f; rz = 0.0f;
        } else {
            // R = Cross(U, Z_up) -> 产生一个水平向量
            rx = uy * 1.0f - uz * 0.0f; // y*1 - z*0
            ry = uz * 0.0f - ux * 1.0f; // z*0 - x*1
            rz = ux * 0.0f - uy * 0.0f; // 0
            
            // 归一化 R
            float r_len = std::sqrt(rx*rx + ry*ry + rz*rz);
            rx /= r_len; ry /= r_len; rz /= r_len;
        }
        
        // (3) 旋转向量 U 得到肘部方向
        // 此时 R 是旋转轴。我们需要将 U 绕 R 旋转 theta 角度。
        // Rodrigues' Rotation Formula (简化版，因为 R 垂直于 U)
        // V_elbow_dir = U * cos(theta) + (R x U) * sin(theta)
        // 注意：方向判定。
        // R x U 也是垂直于 U 的。
        // 我们要让 X 尽可能大。
        // 简单策略：U 已经有了。计算 R x U (Cross2)。
        float c2x = ry*uz - rz*uy;
        float c2y = rz*ux - rx*uz;
        float c2z = rx*uy - ry*ux;
        
        // 肘部向量 E = L1 * (U*cos_theta + Cross2*sin_theta)
        // 我们选择加 Cross2 还是减 Cross2？
        // 你的 Python 脚本要求 x > 0。
        // 我们可以试两个解，选 x 大的那个。
        float sin_theta = std::sin(theta);
        
        // 解 1 (+)
        float ex1 = L_UPPER * (ux * cos_theta + c2x * sin_theta);
        float ey1 = L_UPPER * (uy * cos_theta + c2y * sin_theta);
        float ez1 = L_UPPER * (uz * cos_theta + c2z * sin_theta);
        
        // 解 2 (-)
        float ex2 = L_UPPER * (ux * cos_theta - c2x * sin_theta);
        float ey2 = L_UPPER * (uy * cos_theta - c2y * sin_theta);
        float ez2 = L_UPPER * (uz * cos_theta - c2z * sin_theta);
        
        // 选择 X 较大的解 (符合 x>0 偏好)
        float ex, ey, ez;
        if (ex1 > ex2) { ex=ex1; ey=ey1; ez=ez1; }
        else           { ex=ex2; ey=ey2; ez=ez2; }

        // 5. 生成 V1 和 V2
        // V1 = Elbow - Shoulder(0,0,0) = (ex, ey, ez)
        // V2 = Hand - Elbow
        float vx2 = tx - ex;
        float vy2 = ty - ey;
        float vz2 = tz - ez;

        printf("Python-Like Logic: Hand(%.2f,%.2f,%.2f) -> Elbow(%.2f,%.2f,%.2f)\n", 
               tx, ty, tz, ex, ey, ez);

        // 6. 调用底层 IK 执行
        return SolveIK(ex, ey, ez, vx2, vy2, vz2);
    }

    // ----------------------------------------------------------------------
    // 核心函数 2: 向量转电机角度 (你之前的健壮版 IK)
    // ----------------------------------------------------------------------
    bool SolveIK(float x1, float y1, float z1, float x2, float y2, float z2) {
        float l1 = std::sqrt(x1*x1 + y1*y1 + z1*z1);
        float l2 = std::sqrt(x2*x2 + y2*y2 + z2*z2);
        
        float val_roll = -x1 / l1;
        if (val_roll > 1.0f) val_roll = 1.0f;
        if (val_roll < -1.0f) val_roll = -1.0f;
        float q_sh_roll = std::asin(val_roll);

        float yz_norm = std::sqrt(y1*y1 + z1*z1);
        float q_sh_pitch = (yz_norm < 0.02f) ? 0.0f : std::atan2(-y1, -z1);

        float inv_l1 = 1.0f / l1;
        float ux = x1 * inv_l1, uy = y1 * inv_l1, uz = z1 * inv_l1;
        float dot = x2 * ux + y2 * uy + z2 * uz;
        float px = x2 - dot * ux;
        float py = y2 - dot * uy;
        float q_sh_yaw = std::atan2(-px, py);

        float cos_alpha = (x1*x2 + y1*y2 + z1*z2) / (l1 * l2);
        if(cos_alpha > 1.0f) cos_alpha = 1.0f;
        if(cos_alpha < -1.0f) cos_alpha = -1.0f;
        float q_elbow = 1.5708f - std::acos(cos_alpha);

        const float P_MIN = -2.8f, P_MAX = 1.0f;
        const float R_MIN = -2.0f, R_MAX = 0.5f;
        const float E_MIN = -0.5f, E_MAX = 2.8f; 

        q_sh_pitch = std::fmax(P_MIN, std::fmin(q_sh_pitch, P_MAX));
        q_sh_roll  = std::fmax(R_MIN, std::fmin(q_sh_roll, R_MAX));
        q_elbow    = std::fmax(E_MIN, std::fmin(q_elbow, E_MAX));

        std::unique_lock<std::mutex> lock(target_mtx_);
        q_target_final_[R_ShPitch] = q_sh_pitch;
        q_target_final_[R_ShRoll]  = q_sh_roll;
        q_target_final_[R_ShYaw]   = q_sh_yaw;
        q_target_final_[R_Elbow]   = q_elbow;
        q_target_final_[R_WrPitch] = 0.0f; 

        return true;
    }
};

// ==========================================
// Main
// ==========================================
int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: sudo ./g1_ac [net]" << std::endl;
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
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        while(true) {
            std::cout << "\n=== Control Mode ===\n";
            std::cout << "1. Vector Mode:   v [x1] [y1] [z1] [x2] [y2] [z2]\n";
            std::cout << "2. Position Mode: p [x] [y] [z]  (Hand Position)\n";
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
            } 
            else if (cmd == "p") {
                float x, y, z;
                if(ss >> x >> y >> z) {
                    // 调用新增的函数
                    ctrl.SolvePosition(x, y, z);
                }
            } 
            else if (cmd == "exit") break;
        }
    } 
    return 0;
}