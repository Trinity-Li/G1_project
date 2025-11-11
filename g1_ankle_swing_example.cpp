#include <cmath>
#include <memory> // 智能指针
#include <mutex> //互斥锁 保护数据
#include <shared_mutex> //共享互斥锁 一个线程写入

#include "gamepad.hpp"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp> //发送数据
#include <unitree/robot/channel/channel_subscriber.hpp>//接收（订阅） 某个“主题”上的数据

// IDL 它们定义了DDS通信中传输的具体数据消息格式。
#include <unitree/idl/hg/IMUState_.hpp> //惯性测量单元
#include <unitree/idl/hg/LowCmd_.hpp>  //底层控制命令
#include <unitree/idl/hg/LowState_.hpp>   //底层状态数据
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>  //运动模式切换客户端

//这三行代码就是用 C++ 的 static const std::string 来定义三个常量字符串，用作 DDS 通信的主题名称：
static const std::string HG_CMD_TOPIC = "rt/lowcmd"; 
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
static const std::string HG_STATE_TOPIC = "rt/lowstate";
//using namespace 的作用是简化代码，让你在后续写代码时不必每次都写完整（且冗长）的命名空间前缀。
using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

template <typename T> //线程安全的数据缓冲区 t代表一个未定的数据类型
class DataBuffer {
 public:
  void SetData(const T &newData) {
    std::unique_lock<std::shared_mutex> lock(mutex); // 独占锁，写入数据时阻塞其他读写操作
    data = std::make_shared<T>(newData); // 使用智能指针管理数据内存
  }

  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex);  // 共享锁，允许多个线程同时读取数据
    return data ? data : nullptr; // 返回数据的智能指针，如果数据为空则返回nullptr
  }

  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex); // 独占锁，清除数据时阻塞其他读写操作
    data = nullptr;  // 清除数据
  }

 private:
  std::shared_ptr<T> data;//  数据智能指针
  std::shared_mutex mutex;//    读写锁
};

const int G1_NUM_MOTOR = 29; //这很可能是宇树 G1 机器人的电机（马达）总数
struct ImuState {
  std::array<float, 3> rpy = {}; //代表 Roll (翻滚角), Pitch (俯仰角), Yaw (偏航角)。这是描述物体在三维空间中姿态的三个角度
  std::array<float, 3> omega = {}; //物理学中通常用来表示角速度的符号
};
struct MotorCommand { //电机命令  用于向机器人电机发送控制指令的数据结构
  std::array<float, G1_NUM_MOTOR> q_target = {}; //作用：设置 29 个电机的目标位置/角度。
  std::array<float, G1_NUM_MOTOR> dq_target = {}; //作用：设置 29 个电机的目标速度。
  std::array<float, G1_NUM_MOTOR> kp = {};    //作用：为 29 个电机设置位置控制的比例增益（Kp）。
  std::array<float, G1_NUM_MOTOR> kd = {};   //作用：为 29 个电机设置速度控制的微分增益（Kd）。
  std::array<float, G1_NUM_MOTOR> tau_ff = {}; //作用：为 29 个电机设置前馈力矩（tau_ff），用于补偿负载或实现更精确的控制。
}; //这是一种“预先”施加的力矩（例如，为了抵消重力），它不依赖于误差，而是直接叠加到 PD 控制的输出上。
struct MotorState { //电机状态  用于存储机器人电机当前状态的数据结构
  std::array<float, G1_NUM_MOTOR> q = {};     //作用：存储 29 个电机的当前位置/角度。
  std::array<float, G1_NUM_MOTOR> dq = {};   //作用：存储 29 个电机的当前速度。
};

// Stiffness for all G1 Joints    
std::array<float, G1_NUM_MOTOR> Kp{   
    60, 60, 60, 100, 40, 40,      // legs
    60, 60, 60, 100, 40, 40,      // legs
    60, 40, 40,                   // waist
    40, 40, 40, 40,  40, 40, 40,  // arms
    40, 40, 40, 40,  40, 40, 40   // arms
};

// Damping for all G1 Joints
std::array<float, G1_NUM_MOTOR> Kd{
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1,              // waist
    1, 1, 1, 1, 1, 1, 1,  // arms
    1, 1, 1, 1, 1, 1, 1   // arms
};

enum class Mode {  
  //enum 允许你创建一个自定义的类型，并为这些数字命名
  //enum class (推荐使用) : 目的：enum 的安全升级版。
  //它强制你使用 Mode::PR 这样的完整名称（解决了命名冲突），并且禁止它和整数（如 0）进行不安全的比较（解决了类型安全）。
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

enum G1JointIndex {  //pitch 俯仰角 roll 横滚角 yaw 偏航角
                      //依旧命名为枚举类型，但没有使用 class 关键字，因此这些名称可以直接使用，而不需要前缀。
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleB = 4,
  LeftAnkleRoll = 5,
  LeftAnkleA = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleB = 10,
  RightAnkleRoll = 11,
  RightAnkleA = 11,
  WaistYaw = 12,
  WaistRoll = 13,        // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistA = 13,           // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistPitch = 14,       // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistB = 14,           // NOTE INVALID for g1 23dof/29dof with waist locked
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20,   // NOTE INVALID for g1 23dof
  LeftWristYaw = 21,     // NOTE INVALID for g1 23dof
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27,  // NOTE INVALID for g1 23dof
  RightWristYaw = 28     // NOTE INVALID for g1 23dof
};

inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len)//inline: 这是一个给编译器的优化建议，告诉它“如果可能，请把这个函数的代码直接嵌入到调用它的地方”，以减少函数调用的开销。
 {
  uint32_t xbit = 0; // 用于在每次迭代中检查数据的每一位
  uint32_t data = 0;// 存储当前处理的数据字
  uint32_t CRC32 = 0xFFFFFFFF;  // 初始化 CRC32 校验值为全 1
  const uint32_t dwPolynomial = 0x04c11db7; // CRC32 多项式，用于计算校验值
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31; // 将 xbit 初始化为最高位（第 32 位）
    //xbit (从 1 << 31 开始) 是一个掩码，用于依次提取 data 的第 31 位、第 30 位、...、第 0 位。
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1; // 2. 左移一位
        CRC32 ^= dwPolynomial; // 3. 如果最高位是 1, 就 XOR 多项式
      } else
        CRC32 <<= 1; // 2. 左移一位
      if (data & xbit) CRC32 ^= dwPolynomial;

      xbit >>= 1;
    }//逻辑的巧妙之处： 步骤 (1-3) 和 (4-5) 看起来是分开的，但它们组合起来的效果等价于： if ( (CRC最高位) XOR (当前数据位) == 1 ) CRC = (CRC << 1) ^ Polynomial; else CRC = (CRC << 1); 这正是标准 CRC-32 算法（非反射）的逐位实现。
  }
  return CRC32;
};

class G1Example {
 private:
  double time_;    //跟踪当前时间
  double control_dt_;  // [2ms] //控制时间步长，表示控制循环的时间间隔，单位为秒。在这个例子中，control_dt_ 被设置为 0.002 秒，即 2 毫秒。这意味着控制循环每隔 2 毫秒执行一次。
  double duration_;    // [3 s] //持续时间，单位为秒。在这个例子中，duration_ 被设置为 3.0 秒。表明这个变量可能用来设置某个动作或程序运行的总时长
  int counter_; //计数器 用于跟踪控制循环的迭代次数，通常用于定期执行某些操作或打印状态信息。
  Mode mode_pr_; //当前的控制模式，类型为 Mode 枚举。在这个例子中，mode_pr_ 用于指示机器人当前是处于 PR 模式（Pitch/Roll 控制）还是 AB 模式（A/B 控制）。
  uint8_t mode_machine_; //机器人的当前模式机状态，类型为 uint8_t（无符号 8 位整数）。这个变量可能用于表示机器人所处的不同工作模式或状态机的状态。

  Gamepad gamepad_;  //游戏手柄对象 用于读取和处理游戏手柄的输入数据
  REMOTE_DATA_RX rx_;//无线遥控器接收数据的联合体对象 用于存储从无线遥控器接收到的数据

  DataBuffer<MotorState> motor_state_buffer_; //电机状态数据缓冲区 用于存储和管理机器人的电机状态数据
  DataBuffer<MotorCommand> motor_command_buffer_; //电机命令数据缓冲区 用于存储和管理发送给机器人的电机控制命令数据
  DataBuffer<ImuState> imu_state_buffer_; //IMU状态数据缓冲区 用于存储和管理机器人的惯性测量单元（IMU）状态数据

  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_; //一个发布者（Publisher）智能指针。它的任务是向 HG_CMD_TOPIC ("rt/lowcmd") 主题发送 LowCmd_ 消息（底层命令）。
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;//一个订阅者（Subscriber）智能指针。它的任务是从 HG_STATE_TOPIC ("rt/lowstate") 主题接收 LowState_ 消息（底层状态）。
  ChannelSubscriberPtr<IMUState_> imutorso_subscriber_;//一个订阅者（Subscriber）智能指针。它的任务是从 HG_IMU_TORSO ("rt/secondary_imu") 主题接收 IMUState_ 消息（躯干惯性测量单元状态）。
  ThreadPtr command_writer_ptr_, control_thread_ptr_;//两个线程智能指针。command_writer_ptr_ 负责周期性地发送底层命令，control_thread_ptr_ 负责执行控制逻辑。

  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_; //运动模式切换客户端智能指针。它用于与机器人的运动模式切换服务进行通信，以便在不同的控制模式之间切换。

 public:
  G1Example(std::string networkInterface)
      : time_(0.0),
        control_dt_(0.002),
        duration_(3.0),
        counter_(0),
        mode_pr_(Mode::PR),
        mode_machine_(0) {
    ChannelFactory::Instance()->Init(0, networkInterface); //这行代码告诉 DDS 系统：“开始工作，并使用 networkInterface（例如 "eth0" 或 "wlan0"）这张网卡来进行所有机器人通信。”

    // try to shutdown motion control-related service
    msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    msc_->SetTimeout(5.0f);
    msc_->Init();
    std::string form, name;
    while (msc_->CheckMode(form, name), !name.empty()) {
      if (msc_->ReleaseMode())
        std::cout << "Failed to switch to Release Mode\n";
      sleep(5);
    }
//     它创建了一个 MotionSwitcherClient (msc_)，这是一个高级控制器。

// while 循环是一个安全检查。它在问机器人：“你现在在执行什么高级任务吗？”（CheckMode）

// 如果机器人回答“是”（!name.empty()），程序就会命令它“停止你正在做的事并释放控制权”（ReleaseMode），然后等待 5 秒，再重新检查。

// 目的：这个循环确保在你的程序接管底层控制（LowCmd）之前，机器人必须处于一个安全、空闲的状态，防止两条不同的高级命令发生冲突。

    // create publisher
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();

//     这部分代码创建了 lowcmd_publisher_ 对象。

// 作用：它建立了一个**“发送通道”，你的程序将通过这个发布者，向 HG_CMD_TOPIC（即 "rt/lowcmd"）主题发送 LowCmd_ 消息**。

// InitChannel() 最终激活这个通道。

    // create subscriber
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(std::bind(&G1Example::LowStateHandler, this, std::placeholders::_1), 1);
    imutorso_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    imutorso_subscriber_->InitChannel(std::bind(&G1Example::imuTorsoHandler, this, std::placeholders::_1), 1);
// 这部分创建了两个**“接收通道”**：

// lowstate_subscriber_: 订阅 HG_STATE_TOPIC ("rt/lowstate") 主题，用来接收机器人的底层状态（LowState_）。

// imutorso_subscriber_: 订阅 HG_IMU_TORSO ("rt/secondary_imu") 主题，用来接收 IMU 姿态数据（IMUState_）。

// 最关键的部分是 std::bind：

// std::bind(&G1Example::LowStateHandler, ...) 是一种**“回调” (Callback)** 机制。

// 含义：它告诉 lowstate_subscriber_：“一旦你收到任何 LowState_ 消息，请立即自动调用 G1Example 类的 LowStateHandler 这个函数来处理它。”

// imuTorsoHandler 也是同理。

// 这实现了数据驱动：数据一到，对应的处理函数就自动执行，你不需要手动去检查。

    // create threads
    command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &G1Example::LowCommandWriter, this); 
    //一个循环线程，专门负责向机器人底层发送控制指令（比如电机的目标位置或速度）。
    control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, &G1Example::Control, this);
  }//一个循环线程，这是主要的控制逻辑发生的地方，比如计算机器人的步态、平衡等。

  void imuTorsoHandler(const void *message) {
    IMUState_ imu_torso = *(const IMUState_ *)message;
    auto &rpy = imu_torso.rpy();
    if (counter_ % 500 == 0)
      printf("IMU.torso.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);
 }
 //imuTorsoHandler 是一个回调函数，当收到躯干 IMU 传感器的数据时，它会被调用。

// 它会读取 IMU 数据中的 rpy（Roll 翻滚角, Pitch 俯仰角, Yaw 偏航角）。

// 为了避免刷屏，它每收到 500 次数据才打印一次当前的 rpy 值到控制台

  void LowStateHandler(const void *message) {
    LowState_ low_state = *(const LowState_ *)message;
    if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
      std::cout << "[ERROR] CRC Error" << std::endl;
      return;
    }

// 处理底层状态数据（LowState）：

// LowStateHandler 是一个回调函数，当收到机器人的底层状态数据（如电机位置、速度、足底力等）时，它会被调用。

// CRC 校验：在处理数据之前，它首先进行了一个 CRC（循环冗余校验）。这是一个非常重要的数据完整性检查。它通过计算接收到的数据（low_state）的校验和，并将其与数据包中自带的 crc() 值进行比较。

// 如果两者不一致，意味着数据在传输过程中可能发生了损坏或丢失，程序会打印一个错误信息并 return（即放弃处理这个损坏的数据包），以防止错误的传感器数据导致控制系统出问题。

    // get motor state
    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      ms_tmp.q.at(i) = low_state.motor_state()[i].q(); //电机位置
      ms_tmp.dq.at(i) = low_state.motor_state()[i].dq(); //电机速度
      if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll)
        std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n"; //检测电机状态码，如果有错误则打印出来
    } //遍历所有电机，提取它们的当前位置（q）和速度（dq），并存储到 ms_tmp 结构中。
    motor_state_buffer_.SetData(ms_tmp);

    // get imu state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope(); //角速度
    imu_tmp.rpy = low_state.imu_state().rpy();  //姿态角
    imu_state_buffer_.SetData(imu_tmp);   //存储 IMU 状态数据到缓冲区

    // update gamepad
    memcpy(rx_.buff, &low_state.wireless_remote()[0], 40); //将无线遥控器数据复制到 rx_ 结构中
    gamepad_.update(rx_.RF_RX); //使用无线遥控器数据更新游戏手柄状态

    // update mode machine
    if (mode_machine_ != low_state.mode_machine()) {
      if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl; //打印当前机器人类型
      mode_machine_ = low_state.mode_machine();
    }
// 检测变化：这段代码检查从机器人收到的模式（low_state.mode_machine()）是否与程序本地存储的模式（mode_machine_）不同。

// 打印 G1 型号：这里有一个巧妙的初始化操作。mode_machine_ 本地变量很可能在程序启动时被初始化为 0。当它第一次从 low_state 收到一个非 0 的模式码时，if (mode_machine_ == 0) 条件会成立，程序会打印出 "G1 type: ..."。这可能是在启动时用来确认机器人型号（例如 G1、G1 EDU 等）的。

// 更新模式：在检测到变化后，它会更新本地的 mode_machine_ 变量，使其与机器人当前的状态保持一致。
    // report robot status every second
    if (++counter_ % 500 == 0) {
      counter_ = 0; //每 500 次调用（假设控制循环频率为 500Hz，则大约每秒一次），打印机器人的状态信息
      // IMU
      auto &rpy = low_state.imu_state().rpy();
      printf("IMU.pelvis.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);

      // RC
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
//     IMU 状态： printf("IMU.pelvis.rpy: ...") 打印骨盆（pelvis）IMU 的姿态角（Roll, Pitch, Yaw）。

// RC (遥控器) 状态： printf("gamepad_.A.pressed: ...") 打印 A、B、X、Y 四个按键的按压状态（static_cast<int> 会把 true/false 转换为 1 或 0）。

// 所有电机的详细状态： printf("All %d Motors:", G1_NUM_MOTOR); 这是一个重头戏。它会遍历机器人上的所有电机（数量为 G1_NUM_MOTOR），并以列表形式打印出每一台电机的：

// mode: 当前工作模式（例如，0=关闭, 1=位置模式, 2=速度模式...）

// pos: 当前位置 (q)

// vel: 当前速度 (dq)

// tau_est: 估算的力矩

// temperature: 温度（[0], [1] 可能代表电机绕组和驱动器两个位置）

// vol: 电压

// sensor: 原始传感器数据（[0], [1]）

// motorstate: 错误码（0 代表正常）

// reserve: 保留字段（用于未来扩展）
  }

  void LowCommandWriter() {
    LowCmd_ dds_low_command; //在栈上创建一个用于发送的、空的底层命令数据包。
    dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
    dds_low_command.mode_machine() = mode_machine_; //它首先设置了两个全局状态，这两个状态是在 Control() 函数中被更新的

    const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData(); //这是最关键的一步。它尝试从命令缓冲区 (motor_command_buffer_) 中获取 Control() 函数生产的最新命令（mc）。
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

  void Control() {
    MotorCommand motor_command_tmp;
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
// 首先，它从“状态缓冲区”（motor_state_buffer_）中获取由 LowStateHandler（在之前的代码中）放入的最新传感器数据（ms）
// ，这主要包含了所有电机的当前位置（ms->q）。
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      motor_command_tmp.tau_ff.at(i) = 0.0;
      motor_command_tmp.q_target.at(i) = 0.0;
      motor_command_tmp.dq_target.at(i) = 0.0;
      motor_command_tmp.kp.at(i) = Kp[i];
      motor_command_tmp.kd.at(i) = Kd[i];
    }
// 在计算具体动作之前，它首先创建了一个临时的命令 motor_command_tmp，并将所有电机的目标位置/速度/力矩（q_target, dq_target, tau_ff）全部清零。

// 关键：它为所有电机设置了 Kp 和 Kd 增益。这意味着这个程序始终在PD 控制模式下运行。这个 Control() 函数的工作就是为底层的 PD 控制器提供目标设定点（q_target）。
    if (ms) {
      time_ += control_dt_;
      if (time_ < duration_) {
        // [Stage 1]: set robot to zero posture
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
          double ratio = std::clamp(time_ / duration_, 0.0, 1.0);
          motor_command_tmp.q_target.at(i) = (1.0 - ratio) * ms->q.at(i);
        } //这是一个非常重要的启动程序。它通过一个线性插值 (1.0 - ratio)，将所有电机的目标位置从它们当前的实际位置（ms->q.at(i)）平滑地移动到零点位置（0.0，即 ratio * 0.0，被省略了）。

//目的：防止机器人在上电或启动时，因电机位置不在零点而突然“跳动”或“抖动”，确保平滑启动。
      } else if (time_ < duration_ * 2) {
        // [Stage 2]: swing ankle using PR mode
        mode_pr_ = Mode::PR;
        double max_P = M_PI * 30.0 / 180.0;
        double max_R = M_PI * 10.0 / 180.0;
        double t = time_ - duration_;
        double L_P_des = max_P * std::sin(2.0 * M_PI * t);
        double L_R_des = max_R * std::sin(2.0 * M_PI * t);
        double R_P_des = max_P * std::sin(2.0 * M_PI * t);
        double R_R_des = -max_R * std::sin(2.0 * M_PI * t);
//  mode_pr_ = Mode::PR;：设置一个全局标志，告诉 LowCommandWriter 线程现在处于 "PR" 模式（很可能是 Pitch/Roll 俯仰/翻滚）。

// max_P = ... / max_R = ...：设置了 Pitch 30度 和 Roll 10度 的最大摆动幅度。

// L_P_des = max_P * std::sin(...)：它使用正弦波（sin）为左脚的 Pitch（L_P_des）、Roll（L_R_des）以及右脚的 Pitch 和 Roll 计算出一个平滑的、周期性的目标角度。

// 目的：在 PR 模式下测试或演示脚踝的运动。
        motor_command_tmp.q_target.at(LeftAnklePitch) = L_P_des;
        motor_command_tmp.q_target.at(LeftAnkleRoll) = L_R_des;
        motor_command_tmp.q_target.at(RightAnklePitch) = R_P_des;
        motor_command_tmp.q_target.at(RightAnkleRoll) = R_R_des;
      } else {
        // [Stage 3]: swing ankle using AB mode
        mode_pr_ = Mode::AB;
        double max_A = M_PI * 30.0 / 180.0;
        double max_B = M_PI * 10.0 / 180.0;
        double t = time_ - duration_ * 2;
        double L_A_des = +max_A * std::sin(M_PI * t);
        double L_B_des = +max_B * std::sin(M_PI * t + M_PI);
        double R_A_des = -max_A * std::sin(M_PI * t);
        double R_B_des = -max_B * std::sin(M_PI * t + M_PI);

        motor_command_tmp.q_target.at(LeftAnkleA) = L_A_des;
        motor_command_tmp.q_target.at(LeftAnkleB) = L_B_des;
        motor_command_tmp.q_target.at(RightAnkleA) = R_A_des;
        motor_command_tmp.q_target.at(RightAnkleB) = R_B_des
//        ;mode_pr_ = Mode::AB; ：切换模式标志到 "AB" 模式（可能是另一种运动学模型，如 Actuator A/B）。

// 同样，它使用正弦波（sin）来计算目标角度，但频率（M_PI * t）和相位（... + M_PI，即反相 180 度）与 Stage 2 不同。

// 目的：在 AB 模式下测试或演示脚踝的运动。

      }

      // motor_command_buffer_.SetData(motor_command_tmp);
      // “生产”并发布命令：

motor_command_buffer_.SetData(motor_command_tmp);

// 在状态机（if/else if/else）计算完所有电机的目标位置（q_target）并存入 motor_command_tmp 后，这一行代码将这个完整的命令放入“命令缓冲区”（motor_command_buffer_）。

// 放在缓冲区后，Control() 函数本轮的工作就全部完成了。LowCommandWriter 线程会（在下一段代码中）从这个缓冲区中取出该命令，并将其发送给硬件。
    }
  }
};

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_ankle_swing_example network_interface" << std::endl;
    exit(0);
  }
  std::string networkInterface = argv[1];
  G1Example custom(networkInterface);
  while (true) sleep(10);
  return 0;
}
// if (argc < 2) 检查用户是否在运行程序时提供了网络接口名。

// 如果没有（argc < 2），它会打印 Usage...（使用方法）并退出（exit(0)）。

// 获取网络接口：

// std::string networkInterface = argv[1];

// 如果“门卫”检查通过，这行代码会读取用户提供的网络接口名（argv[1]，即用户输入的第二个词），并将其存储在 networkInterface 变量中。

// 启动机器人控制器：

// G1Example custom(networkInterface);

// 这是最关键的一行。它创建了一个 G1Example 类的实例（对象），并把网络接口名 networkInterface 传递给它。

// 回顾：我们之前看到的所有函数（Control, LowCommandWriter, LowStateHandler）都是 G1Example 类的成员。

// 在创建 custom 这个对象时，G1Example 类的构造函数（我们最早看到的 create threads 那段代码）会被调用，从而启动 Control（大脑）和 LowCommandWriter（信使）这两个核心线程。

// 防止程序退出：

// while (true) sleep(10);

// 此时，Control 和 LowCommandWriter 线程已经在后台并行运行了。

// main 函数（主线程）的工作已经完成，但它不能退出，否则整个程序将终止。

// 这行代码让 main 线程进入一个无限睡眠循环（每 10 秒醒一次，然后继续睡）。

// 目的：保持主线程“活着”，从而为其他工作线程提供运行的“容器”。