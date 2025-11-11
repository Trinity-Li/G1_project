#include <cmath>
#include <memory>
#include <mutex> // 我们需要一个标准的 mutex
#include <shared_mutex>
#include <map>         // 用于 motor_map_
#include <string>      // 用于 motor_name
#include <iostream>    // 用于 std::cin, std::cout
#include <sstream>     // 用于解析用户输入
#include <iomanip>     // 用于 std::setw (美化输出)

#include "gamepad.hpp"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

/* * DataBuffer 模板类 (与原文件相同)
 */
template <typename T>
class DataBuffer {
 public:
  void SetData(const T &newData) {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = std::make_shared<T>(newData);
  }
  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex);
    return data ? data : nullptr;
  }
  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = nullptr;
  }
 private:
  std::shared_ptr<T> data;
  std::shared_mutex mutex;
};

/* * 常量, 结构体定义 (与原文件相同)
 */
const int G1_NUM_MOTOR = 29;
struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
};
struct MotorCommand {
  std::array<float, G1_NUM_MOTOR> q_target = {};
  std::array<float, G1_NUM_MOTOR> dq_target = {};
  std::array<float, G1_NUM_MOTOR> kp = {};
  std::array<float, G1_NUM_MOTOR> kd = {};
  std::array<float, G1_NUM_MOTOR> tau_ff = {};
};
struct MotorState {
  std::array<float, G1_NUM_MOTOR> q = {};
  std::array<float, G1_NUM_MOTOR> dq = {};
};

// Kp, Kd 数组 (与原文件相同)
std::array<float, G1_NUM_MOTOR> Kp{
    60, 60, 60, 100, 40, 40, 60, 60, 60, 100, 40, 40, 60, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40};
std::array<float, G1_NUM_MOTOR> Kd{1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1,
                                   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                   1, 1, 1, 1, 1};

// G1JointIndex 枚举 (与原文件相同)
enum G1JointIndex {
  LeftHipPitch = 0, LeftHipRoll = 1, LeftHipYaw = 2, LeftKnee = 3, LeftAnklePitch = 4, LeftAnkleB = 4,
  LeftAnkleRoll = 5, LeftAnkleA = 5, RightHipPitch = 6, RightHipRoll = 7, RightHipYaw = 8,
  RightKnee = 9, RightAnklePitch = 10, RightAnkleB = 10, RightAnkleRoll = 11, RightAnkleA = 11,
  WaistYaw = 12, WaistRoll = 13, WaistA = 13, WaistPitch = 14, WaistB = 14,
  LeftShoulderPitch = 15, LeftShoulderRoll = 16, LeftShoulderYaw = 17, LeftElbow = 18,
  LeftWristRoll = 19, LeftWristPitch = 20, LeftWristYaw = 21, RightShoulderPitch = 22,
  RightShoulderRoll = 23, RightShoulderYaw = 24, RightElbow = 25, RightWristRoll = 26,
  RightWristPitch = 27, RightWristYaw = 28
};

// Crc32Core 函数 (与原文件相同)
inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else
        CRC32 <<= 1;
      if (data & xbit) CRC32 ^= dwPolynomial;
      xbit >>= 1;
    }
  }
  return CRC32;
};

/* * =====================================================================
 * =================== G1MotorControl 类 (已修改) ====================
 * =====================================================================
 */
class G1MotorControl {
 private:
  int counter_;
  uint8_t mode_machine_;
  Gamepad gamepad_;
  REMOTE_DATA_RX rx_;

  // 线程安全的数据缓冲区
  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;

  // DDS 发布/订阅
  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
  ChannelSubscriberPtr<IMUState_> imutorso_subscriber_;
  
  // 核心线程
  ThreadPtr command_writer_ptr_, control_thread_ptr_;
  
  // 运动切换
  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;
  
  // ---------------- 新增的成员变量 -----------------
  std::mutex target_mutex_; // 用于保护 user_q_target_ 和 user_dq_target_
  std::array<float, G1_NUM_MOTOR> user_q_target_;  // [新] 用户的目标位置
  std::array<float, G1_NUM_MOTOR> user_dq_target_; // [新] 用户的目标速度
  std::map<std::string, G1JointIndex> motor_map_;  // [新] 字符串到ID的映射
  bool initialized_; // [新] 安全保持标志

 public:
  // 构造函数 (已修改)
  G1MotorControl(std::string networkInterface)
      : counter_(0),
        mode_machine_(0),
        initialized_(false) // [新] 
  {
    ChannelFactory::Instance()->Init(0, networkInterface);

    // [新] 初始化目标为 0
    user_q_target_.fill(0.0f);
    user_dq_target_.fill(0.0f);
    // [新] 初始化电机名称映射
    InitMotorMap();

    // 释放模式 (与原文件相同)
    msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    msc_->SetTimeout(5.0f);
    msc_->Init();
    std::string form, name;
    while (msc_->CheckMode(form, name), !name.empty()) {
      if (msc_->ReleaseMode())
        std::cout << "Failed to switch to Release Mode\n";
      sleep(5);
    }

    // 创建 publisher (与原文件相同)
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();

    // 创建 subscriber (与原文件相同)
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(std::bind(&G1MotorControl::LowStateHandler, this, std::placeholders::_1), 1);
    imutorso_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    imutorso_subscriber_->InitChannel(std::bind(&G1MotorControl::imuTorsoHandler, this, std::placeholders::_1), 1);

    // 创建 threads (与原文件相同)
    command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &G1MotorControl::LowCommandWriter, this);
    control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, &G1MotorControl::Control, this);
  }

  // imuTorsoHandler 回调 (与原文件相同)
  void imuTorsoHandler(const void *message) {
    IMUState_ imu_torso = *(const IMUState_ *)message;
    auto &rpy = imu_torso.rpy();
    if (counter_ % 500 == 0)
      printf("IMU.torso.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);
  }

  // LowStateHandler 回调 (与原文件相同)
  void LowStateHandler(const void *message) {
    LowState_ low_state = *(const LowState_ *)message;
    if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
      std::cout << "[ERROR] CRC Error" << std::endl;
      return;
    }

    // get motor state
    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      ms_tmp.q.at(i) = low_state.motor_state()[i].q();
      ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
      if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll)
        std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
    }
    motor_state_buffer_.SetData(ms_tmp);

    // get imu state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();
    imu_state_buffer_.SetData(imu_tmp);

    // update gamepad
    memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
    gamepad_.update(rx_.RF_RX);

    // update mode machine
    if (mode_machine_ != low_state.mode_machine()) {
      if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
      mode_machine_ = low_state.mode_machine();
    }

    // 调试打印 (与原文件相同)
    if (++counter_ % 500 == 0) {
      counter_ = 0;
      auto &rpy = low_state.imu_state().rpy();
      printf("IMU.pelvis.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);
      
      printf("gamepad_.A.pressed: %d\n", static_cast<int>(gamepad_.A.pressed));
      printf("gamepad_.B.pressed: %d\n", static_cast<int>(gamepad_.B.pressed));
      printf("gamepad_.X.pressed: %d\n", static_cast<int>(gamepad_.X.pressed));
      printf("gamepad_.Y.pressed: %d\n", static_cast<int>(gamepad_.Y.pressed));

      // Motor
      auto &ms = low_state.motor_state();
      printf("All %d Motors:", G1_NUM_MOTOR);
      printf("\nmode: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,", ms[i].mode());
      printf("\npos: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].q());
      printf("\nvel: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].dq());
      printf("\ntau_est: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].tau_est());
      printf("\ntemperature: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%d,%d;", ms[i].temperature()[0], ms[i].temperature()[1]);
      printf("\nvol: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].vol());
      printf("\nsensor: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,%u;", ms[i].sensor()[0], ms[i].sensor()[1]);
      printf("\nmotorstate: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,", ms[i].motorstate());
      printf("\nreserve: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,%u,%u,%u;", ms[i].reserve()[0], ms[i].reserve()[1], ms[i].reserve()[2], ms[i].reserve()[3]);
      printf("\n");
    }
  }

  // LowCommandWriter 线程 (与原文件相同, mode_pr_ 除外)
  void LowCommandWriter() {
    LowCmd_ dds_low_command;
    // dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_); // [已移除]
    dds_low_command.mode_machine() = mode_machine_;

    const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
    if (mc) {
      for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
        dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
        dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
        dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
        dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
        dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);
        dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);
      }

      dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
      lowcmd_publisher_->Write(dds_low_command);
    }
  }

  // -------------------------------------------------------------------
  // ------------------ Control 线程 (已修改) ------------------------
  // -------------------------------------------------------------------
  void Control() {
    // [1] 获取当前传感器状态
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
    if (!ms) {
      // 如果没有收到任何状态信息，立即返回，不做任何事
      return;
    }

    // [2] [安全功能] 第一次运行时，将所有电机的目标设为当前位置
    if (!initialized_) {
      std::unique_lock<std::mutex> lock(target_mutex_);
      user_q_target_ = ms->q; // 将目标设为当前位置
      user_dq_target_.fill(0.0f); // 目标速度设为 0
      initialized_ = true;
      lock.unlock(); // 立即释放锁

      std::cout << "[INFO] Robot Initialized. Holding current position." << std::endl;
      std::cout << "[INFO] Ready for user commands." << std::endl;
      return; // 跳过此循环，等待下一次
    }

    // [3] 初始化一个临时的电机命令
    MotorCommand motor_command_tmp;

    // [4] (线程安全地) 获取用户设定的目标
    target_mutex_.lock();
    std::array<float, G1_NUM_MOTOR> current_q_targets = user_q_target_;
    std::array<float, G1_NUM_MOTOR> current_dq_targets = user_dq_target_;
    target_mutex_.unlock();

    // [5] 填充命令
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      motor_command_tmp.tau_ff.at(i) = 0.0;
      motor_command_tmp.q_target.at(i) = current_q_targets.at(i);
      motor_command_tmp.dq_target.at(i) = current_dq_targets.at(i);
      motor_command_tmp.kp.at(i) = Kp[i];
      motor_command_tmp.kd.at(i) = Kd[i];
    }

    // [6] "生产" 命令: 将命令放入缓冲区，供 LowCommandWriter 线程消费
    motor_command_buffer_.SetData(motor_command_tmp);
  }

  // -------------------------------------------------------------------
  // ------------------ 新增的公共方法 ------------------------
  // -------------------------------------------------------------------
 private:
  // [新] 内部帮助函数，用于填充 motor_map_
  void InitMotorMap() {
    motor_map_["LeftHipPitch"] = LeftHipPitch;
    motor_map_["LeftHipRoll"] = LeftHipRoll;
    motor_map_["LeftHipYaw"] = LeftHipYaw;
    motor_map_["LeftKnee"] = LeftKnee;
    motor_map_["LeftAnklePitch"] = LeftAnklePitch;
    motor_map_["LeftAnkleRoll"] = LeftAnkleRoll;
    motor_map_["RightHipPitch"] = RightHipPitch;
    motor_map_["RightHipRoll"] = RightHipRoll;
    motor_map_["RightHipYaw"] = RightHipYaw;
    motor_map_["RightKnee"] = RightKnee;
    motor_map_["RightAnklePitch"] = RightAnklePitch;
    motor_map_["RightAnkleRoll"] = RightAnkleRoll;
    motor_map_["WaistYaw"] = WaistYaw;
    motor_map_["WaistRoll"] = WaistRoll;
    motor_map_["WaistPitch"] = WaistPitch;
    motor_map_["LeftShoulderPitch"] = LeftShoulderPitch;
    motor_map_["LeftShoulderRoll"] = LeftShoulderRoll;
    motor_map_["LeftShoulderYaw"] = LeftShoulderYaw;
    motor_map_["LeftElbow"] = LeftElbow;
    motor_map_["LeftWristRoll"] = LeftWristRoll;
    motor_map_["LeftWristPitch"] = LeftWristPitch;
    motor_map_["LeftWristYaw"] = LeftWristYaw;
    motor_map_["RightShoulderPitch"] = RightShoulderPitch;
    motor_map_["RightShoulderRoll"] = RightShoulderRoll;
    motor_map_["RightShoulderYaw"] = RightShoulderYaw;
    motor_map_["RightElbow"] = RightElbow;
    motor_map_["RightWristRoll"] = RightWristRoll;
    motor_map_["RightWristPitch"] = RightWristPitch;
    motor_map_["RightWristYaw"] = RightWristYaw;
  }

 public:
  // [新] 公共方法，用于 main 函数获取电机列表
  std::map<std::string, G1JointIndex> GetMotorMap() {
    return motor_map_;
  }

  // [新] 公公共方法，用于 main 函数设置目标 (线程安全)
  void SetMotorTarget(int motor_id, float q_target_rad, float dq_target_rad) {
    if (motor_id < 0 || motor_id >= G1_NUM_MOTOR) {
      std::cout << "[ERROR] Invalid motor ID: " << motor_id << std::endl;
      return;
    }

    std::unique_lock<std::mutex> lock(target_mutex_);
    // 只更新你指定的电机
    // 其他电机的目标值保持不变 (即保持它们上次的位置)
    user_q_target_.at(motor_id) = q_target_rad;
    user_dq_target_.at(motor_id) = dq_target_rad;
  }
};

/* * =====================================================================
 * ====================== main 函数 (已修改) ==========================
 * =====================================================================
 */
int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_motor_control_example network_interface" << std::endl;
    exit(0);
  }
  std::string networkInterface = argv[1];

  // [1] 创建 G1MotorControl 实例 (这会自动启动所有线程)
  G1MotorControl custom(networkInterface);

  // [2] 等待 Control 线程完成“上电保持”初始化
  std::cout << "Initializing... Waiting for first robot state message..." << std::endl;
  sleep(2); // 等待2秒，让 `initialized_` 标志有足够时间被设置

  // [3] 获取并打印所有可用的电机名称
  std::map<std::string, G1JointIndex> motor_map = custom.GetMotorMap();
  std::cout << "===============================================" << std::endl;
  std::cout << "Available motors (Name -> ID):" << std::endl;
  for (auto const& [name, id] : motor_map) {
      std::cout << "  " << std::setw(20) << std::left << name << " -> " << id << std::endl;
  }
  std::cout << "===============================================" << std::endl;

  // [4] 进入交互式命令循环
  while (true) {
    std::cout << "\nEnter command (or 'exit'): [MotorName] [Angle_deg] [Velocity_rad_s]" << std::endl;
    std::cout << "Example: LeftKnee 30 0" << std::endl;
    std::cout << "> ";

    std::string line;
    if (!std::getline(std::cin, line)) {
      break; // 捕获 Ctrl+D
    }

    if (line == "exit") {
      break; // 退出
    }

    std::stringstream ss(line);
    std::string motor_name;
    float angle_deg, vel_rad_s;

    // [5] 解析用户输入
    if (ss >> motor_name >> angle_deg >> vel_rad_s) {
      // 检查电机名称是否存在
      if (motor_map.count(motor_name)) {
        int motor_id = motor_map[motor_name];
        
        // [6] 将角度从“度”转换为“弧度” (M_PI 是 cmath 库中的 π)
        float angle_rad = angle_deg * M_PI / 180.0;

        // [7] 调用公共方法，将命令发送给 Control 线程
        custom.SetMotorTarget(motor_id, angle_rad, vel_rad_s);
        
        printf("[OK] Sending command: %s (id %d) -> Angle=%.2f rad (%.1f deg), Vel=%.2f rad/s\n",
               motor_name.c_str(), motor_id, angle_rad, angle_deg, vel_rad_s);
      } else {
        std::cout << "[ERROR] Unknown motor name: " << motor_name << std::endl;
      }
    } else {
      std::cout << "[ERROR] Invalid input format. Please try again." << std::endl;
    }
  }

  std::cout << "Exiting program." << std::endl;
  return 0;
}