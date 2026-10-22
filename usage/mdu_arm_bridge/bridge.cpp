// 在 MDU 运行：ROS 2 标准角度指令 -> 固件 joint_msgs -> 本机 Iceoryx。
// 默认禁发，测试通道与 HAL 控制通道隔离；不加载策略、不发送腿/腰/头指令。
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>
#include "diagnostic_log.h"

#include "aimrt_module_c_interface/module_base.h"
#include "aimrt_module_cpp_interface/channel/channel_handle.h"
#include "aimrt_module_ros2_interface/util/ros2_type_support.h"
#include "aimrt_pkg_c_interface/pkg_main.h"
#include "joint_msgs/msg/joint_command.hpp"
#include "joint_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

using Clock = std::chrono::steady_clock;
using Command = joint_msgs::msg::JointCommand;
using State = joint_msgs::msg::JointState;
constexpr std::array<const char*, 14> names = {
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};
constexpr std::array<double, 14> lower = {-2.87979,-.08727,-2.79253,-.95993,-2.79253,-1.62316,-1.62316,
                                         -2.87979,-2.61799,-2.79253,-.95993,-2.79253,-1.62316,-1.62316};
constexpr std::array<double, 14> upper = {2.87979,2.61799,2.79253,1.74533,2.79253,1.62316,1.62316,
                                         2.87979,.08727,2.79253,1.74533,2.79253,1.62316,1.62316};
constexpr std::array<double, 7> kp = {40,40,30,30,30,20,20}, kd = {3,3,2,2,2,2,2};

class Bridge {
 public:
  bool Initialize(const aimrt_core_base_t* core) {
    const char* flag = std::getenv("A3_ENABLE_COMMANDS");
    enabled_ = flag && std::string(flag) == "1";
    Json::Value start;
    start["enabled"] = enabled_;
    for (double value : kp) start["kp"].append(value);
    for (double value : kd) start["kd"].append(value);
    diagnostics_.Write("program_started", std::move(start));
    const auto* channel = core->channel_handle(core->impl);
    auto publisher = channel->get_publisher(channel->impl, aimrt::util::ToAimRTStringView(
        enabled_ ? "/body_drive/arm_joint_command" : "/teleop/dry_run/arm_joint_command"));
    publisher_ = aimrt::channel::PublisherRef(publisher);
    if (!publisher_.RegisterPublishType(aimrt::GetRos2MessageTypeSupport<Command>())) return false;
    auto state = channel->get_subscriber(channel->impl,
                                        aimrt::util::ToAimRTStringView("/body_drive/arm_joint_state"));
    if (!aimrt::channel::SubscriberRef(state).Subscribe(aimrt::GetRos2MessageTypeSupport<State>(),
        [this](const aimrt_channel_context_base_t*, const void* msg, aimrt_function_base_t* release) {
          aimrt::channel::SubscriberReleaseCallback done(release);
          OnState(*static_cast<const State*>(msg));
          done();
        })) return false;
    if (!enabled_) {
      // 用独立 Iceoryx 订阅回读测试输出，验证消息序列化和本机通道，不接触 HAL command。
      auto test = channel->get_subscriber(channel->impl,
          aimrt::util::ToAimRTStringView("/teleop/dry_run/arm_joint_command"));
      if (!aimrt::channel::SubscriberRef(test).Subscribe(aimrt::GetRos2MessageTypeSupport<Command>(),
          [](const aimrt_channel_context_base_t*, const void* msg, aimrt_function_base_t* release) {
            aimrt::channel::SubscriberReleaseCallback done(release);
            const auto& command = *static_cast<const Command*>(msg);
            bool valid = command.joints.size() == names.size();
            for (size_t i = 0; valid && i < names.size(); ++i) {
              const auto& j = command.joints[i];
              valid = j.name == names[i] && std::isfinite(j.position)
                  && j.stiffness == kp[i % 7] && j.damping == kd[i % 7]
                  && j.velocity == 0. && j.effort == 0.;
            }
            std::cout << (valid ? "DRY_RUN_ICEORYX_RECEIVED joints=14 PD=OK" : "DRY_RUN_DECODE_FAILED") << std::endl;
            done();
          })) return false;
    }
    rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    node_ = std::make_shared<rclcpp::Node>("a3_mdu_arm_command_bridge");
    ready_ = node_->create_publisher<std_msgs::msg::Bool>("/teleop/mdu/arm_bridge_ready", rclcpp::SensorDataQoS());
    subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/teleop/mdu/arm_joint_command",
        rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { OnCommand(*msg); });
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this] {
      std::lock_guard lock(mutex_);
      // 指令中断后不重发旧目标，也不自动恢复；重新启动桥接才能清除故障。
      if (started_ && Clock::now() - last_command_ > std::chrono::seconds(1))
        Fail("PC command stream timed out");
      std_msgs::msg::Bool msg;
      msg.data = enabled_ && !fault_ && FreshState();
      ready_->publish(msg);
    });
    return true;
  }

  bool Start() {
    worker_ = std::thread([this] {
      try { rclcpp::spin(node_); }
      catch (const std::exception& e) {
        std::lock_guard lock(mutex_);
        Fail("ROS executor failed");
        std::cerr << e.what() << std::endl;
      }
    });
    std::cout << (enabled_ ? "LIVE: HAL command forwarding enabled" : "DRY RUN: HAL command forwarding DISABLED") << std::endl;
    return true;
  }
  void Shutdown() {
    { std::lock_guard lock(mutex_); fault_ = true; }
    if (rclcpp::ok()) rclcpp::shutdown();
    if (worker_.joinable()) worker_.join();
  }
 private:
  bool FreshState() const {
    return have_state_ && Clock::now() - last_state_ < std::chrono::milliseconds(100);
  }
  void Fail(const char* reason) {
    if (!fault_) {
      std::cerr << "BRIDGE FAULT: " << reason << "; restart required" << std::endl;
      Json::Value row;
      row["reason"] = reason;
      row["last_accepted_stamp_ns"] = Json::Int64(last_stamp_);
      if (started_) row["accepted_command_age_ms"] =
          std::chrono::duration<double, std::milli>(Clock::now() - last_command_).count();
      if (have_command_rx_) row["received_command_age_ms"] =
          std::chrono::duration<double, std::milli>(Clock::now() - last_command_rx_).count();
      if (have_state_) row["feedback_age_ms"] =
          std::chrono::duration<double, std::milli>(Clock::now() - last_state_).count();
      diagnostics_.Write("bridge_fault", std::move(row));
    }
    fault_ = true;
  }
  void OnState(const State& msg) {
    std::lock_guard lock(mutex_);
    if (msg.joints.size() != names.size()) return;
    std::array<double, 14> positions{};
    for (size_t i = 0; i < names.size(); ++i) {
      const auto found = std::find_if(msg.joints.begin(), msg.joints.end(),
          [i](const auto& j) { return j.name == names[i]; });
      if (found == msg.joints.end() || !std::isfinite(found->position)) return;
      positions[i] = found->position;
    }
    measured_ = positions;
    Json::Value row;
    row["stamp_ns"] = Json::Int64(int64_t(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec);
    for (const auto& joint : msg.joints) {
      row["names"].append(joint.name);
      row["position"].append(joint.position);
      row["velocity"].append(joint.velocity);
      row["effort"].append(joint.effort);
    }
    if (have_state_) row["receive_gap_ms"] =
        std::chrono::duration<double, std::milli>(Clock::now() - last_state_).count();
    diagnostics_.Write("feedback", std::move(row));
    if (!have_state_) std::cout << "ICEORYX_ARM_STATE_RECEIVED joints=14" << std::endl;
    last_state_ = Clock::now(); have_state_ = true;
  }
  void OnCommand(const sensor_msgs::msg::JointState& msg) {
    std::lock_guard lock(mutex_);
    const auto received_at = Clock::now();
    const int64_t stamp = int64_t(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec;
    Json::Value received;
    received["stamp_ns"] = Json::Int64(stamp);
    received["name_count"] = Json::UInt64(msg.name.size());
    received["position_count"] = Json::UInt64(msg.position.size());
    // 非法报文只记录大小，避免诊断本身放大畸形输入的处理开销。
    if (msg.name.size() == names.size() && msg.position.size() == names.size()) {
      for (const auto& name : msg.name) received["names"].append(name.substr(0, 128));
      for (double value : msg.position) received["position"].append(value);
    }
    if (have_command_rx_) received["receive_gap_ms"] =
        std::chrono::duration<double, std::milli>(received_at - last_command_rx_).count();
    diagnostics_.Write("command_received", std::move(received));
    last_command_rx_ = received_at; have_command_rx_ = true;
    if (fault_) { RecordRejected(stamp, "bridge fault already latched"); return; }
    if (!FreshState()) { Reject(stamp, "no fresh Iceoryx arm feedback"); return; }
    if (msg.name.size() != names.size() || msg.position.size() != names.size()) {
      Reject(stamp, "invalid joint count"); return;
    }
    if (stamp <= 0 || (started_ && stamp <= last_stamp_)) {
      RecordRejected(stamp, "nonpositive or non-increasing timestamp"); return;
    }
    const auto now = Clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_command_).count();
    for (size_t i = 0; i < names.size(); ++i) {
      const double q = msg.position[i];
      if (msg.name[i] != names[i] || !std::isfinite(q) || q < lower[i] || q > upper[i]) {
        Reject(stamp, "invalid joint name/order/angle"); return;
      }
    }
    Command output;
    output.header.stamp = node_->now();
    bool limited = false;
    const double max_step = .5 * std::min(elapsed, .1) + .005;
    for (size_t i = 0; i < names.size(); ++i) {
      joint_msgs::msg::Command joint;
      joint.name = names[i]; joint.sequence = sequence_;
      joint.position = msg.position[i];
      if (started_)
        joint.position = std::clamp(joint.position, last_positions_[i] - max_step,
                                    last_positions_[i] + max_step);
      limited = limited || joint.position != msg.position[i];
      joint.velocity = 0.; joint.effort = 0.;
      joint.stiffness = kp[i % 7]; joint.damping = kd[i % 7];
      output.joints.push_back(joint);
      last_positions_[i] = joint.position;
    }
    aimrt::channel::Context context;
    context.SetSerializationType("ros2");
    publisher_.Publish("ros2:joint_msgs/msg/JointCommand", context, &output);
    Json::Value submitted;
    submitted["stamp_ns"] = Json::Int64(stamp);
    submitted["output_stamp_ns"] = Json::Int64(int64_t(output.header.stamp.sec) * 1000000000LL
                                               + output.header.stamp.nanosec);
    submitted["output_sequence"] = sequence_;
    submitted["hal_enabled"] = enabled_;
    submitted["limited"] = limited;
    for (const auto& joint : output.joints) submitted["output_position"].append(joint.position);
    submitted["processing_ms"] = std::chrono::duration<double, std::milli>(Clock::now() - received_at).count();
    diagnostics_.Write("iceoryx_publish_returned", std::move(submitted));
    ++sequence_; started_ = true; last_stamp_ = stamp; last_command_ = now;
  }
  void RecordRejected(int64_t stamp, const char* reason) {
    Json::Value row;
    row["stamp_ns"] = Json::Int64(stamp); row["reason"] = reason;
    diagnostics_.Write("command_rejected", std::move(row));
  }
  void Reject(int64_t stamp, const char* reason) {
    RecordRejected(stamp, reason);
    Fail(reason);
  }
  DiagnosticLog diagnostics_;
  bool enabled_ = false, have_state_ = false, started_ = false, fault_ = false;
  uint32_t sequence_ = 0;
  int64_t last_stamp_ = 0;
  std::array<double, 14> measured_{}, last_positions_{};
  Clock::time_point last_state_{}, last_command_{};
  Clock::time_point last_command_rx_{};
  bool have_command_rx_ = false;
  std::mutex mutex_;
  aimrt::channel::PublisherRef publisher_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread worker_;
};

// 固件通过 AimRT C ABI 加载模块；不链接第二套 AimRT Core。
struct Module {
  Bridge bridge;
  aimrt_module_base_t api{
      .info = [](void*) { aimrt_module_info_t info{}; info.name = {"A3MduArmBridge", 14}; return info; },
      .initialize = [](void* self, const aimrt_core_base_t* core) {
        try { return static_cast<Module*>(self)->bridge.Initialize(core); }
        catch (const std::exception& e) { std::cerr << e.what() << std::endl; return false; }
      },
      .start = [](void* self) { return static_cast<Module*>(self)->bridge.Start(); },
      .shutdown = [](void* self) { static_cast<Module*>(self)->bridge.Shutdown(); },
      .impl = this};
};
extern "C" {
size_t AimRTDynlibGetModuleNum() { return 1; }
const aimrt_string_view_t* AimRTDynlibGetModuleNameList() {
  static const aimrt_string_view_t name{"A3MduArmBridge", 14}; return &name;
}
const aimrt_module_base_t* AimRTDynlibCreateModule(aimrt_string_view_t name) {
  if (std::string_view(name.str, name.len) != "A3MduArmBridge") return nullptr;
  return &(new Module)->api;
}
void AimRTDynlibDestroyModule(const aimrt_module_base_t* module) { delete static_cast<Module*>(module->impl); }
}
