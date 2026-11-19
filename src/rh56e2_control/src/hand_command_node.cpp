// Three independent 10x8 pad regions on /hand/tactile/{index,middle,ring}_pad.
// Each monitored region yields 80 pressure readings and one pressure-weighted CoP (cell coordinates).
// Thumb tactile monitoring is disabled; all five fingers still use force-based grasping.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class HandControlNode : public rclcpp::Node
{
public:
  HandControlNode() : Node("hand_control_node")
  {
    object_name_ = declare_parameter<std::string>("object_name", "unspecified");
    enable_motion_ = declare_parameter<bool>("enable_motion", true);
    enable_auto_regrasp_ = declare_parameter<bool>("enable_auto_regrasp", true);
    const double initial_min_force =
      declare_parameter<double>("initial_min_force", kInitialMinForce);
    const double initial_max_force =
      declare_parameter<double>("initial_max_force", kInitialMaxForce);
    // Thresholds are only INITIAL LAB GUESSES for raw uint16 T1 taxels.
    tactile_min_sum_ = declare_parameter<double>("tactile_min_sum", 500.0);
    slip_distance_cells_ = declare_parameter<double>("slip_distance_cells", 0.2);
    slip_confirm_frames_ = declare_parameter<int>("slip_confirm_frames", 3);
    pressure_drop_ratio_ = declare_parameter<double>("pressure_drop_ratio", 0.2);
    pressure_drop_confirm_frames_ = declare_parameter<int>("pressure_drop_confirm_frames", 3);
    if (!std::isfinite(initial_min_force) || initial_min_force <= 0.0 ||
        !std::isfinite(initial_max_force) || initial_max_force <= initial_min_force ||
        initial_max_force > kForceUpperLimit ||
        !std::isfinite(tactile_min_sum_) || tactile_min_sum_ <= 0.0 ||
        !std::isfinite(slip_distance_cells_) || slip_distance_cells_ <= 0.0 ||
        !std::isfinite(pressure_drop_ratio_) || pressure_drop_ratio_ <= 0.0 ||
        pressure_drop_ratio_ >= 1.0 || pressure_drop_confirm_frames_ < 2 ||
        slip_confirm_frames_ < 2) {
      throw std::invalid_argument("Invalid tactile thresholds");
    }

    min_force_.fill(initial_min_force);
    max_force_.fill(initial_max_force);
    velocity_.fill(std::numeric_limits<double>::quiet_NaN());

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/hand_joint_controller/target_joint_position", 10);
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        receive_state(*msg);
      });
    // Thumb still participates in force-based grasping, but not tactile monitoring.
    for (std::size_t f = 1; f <= 3; ++f) {
      tactile_sub_[f] = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/hand/tactile/" + finger_names_[f] + "_pad", rclcpp::SensorDataQoS(),
        [this, f](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
          receive_tactile(*msg, f);
        });
    }
    timer_ = create_wall_timer(100ms, [this]() { control_step(); });
    report_timer_ = create_wall_timer(1s, [this]() { report_state(); });

    RCLCPP_INFO(get_logger(),
      "object=%s | OPEN -> FORCE ZERO(20) -> FORCE(5 fingers) -> LOCKED -> "
      "TACTILE REFERENCE(first valid frame) -> SLIP MONITOR. "
      "motion=%s auto_regrasp=%s initial_force=[%.1f,%.1f]",
      object_name_.c_str(),
      enable_motion_ ? "ON" : "OFF",
      enable_auto_regrasp_ ? "ON" : "OFF",
      initial_min_force, initial_max_force);
    RCLCPP_INFO(get_logger(),
      "[TACTILE] Monitoring index/middle/ring only");
  }

private:
  static constexpr std::size_t kN = 6;
  static constexpr std::size_t kIndex = 2;
  static constexpr std::size_t kFingers = 5;
  const std::array<std::string, kFingers> finger_names_{"thumb", "index", "middle", "ring", "pinky"};
  static constexpr std::size_t kRows = 10;
  static constexpr std::size_t kCols = 8;
  static constexpr std::size_t kTaxels = kRows * kCols;
  static constexpr std::size_t rows(std::size_t) { return kRows; }
  static constexpr std::size_t taxels(std::size_t f) { return rows(f) * kCols; }
  static constexpr int kCalibrationSamples = 20;
  static constexpr double kInitialMinForce = 55.0;
  static constexpr double kInitialMaxForce = 300.0;
  static constexpr double kAngleStep = 0.008;       // rad per adjustment (0.08 rad/s)
  static constexpr double kAdjustmentInterval = 0.1; // sec
  static constexpr double kPositionTrackingTolerance = 0.02; // rad
  static constexpr double kForceHoldSeconds = 3.0;
  static constexpr double kThumb1Target = 1.0;     // rad
  static constexpr double kForceUpperLimit = 300.0; // EXAMPLE software ceiling: verify!
  static constexpr double kRegraspAngleStep = 0.04;
  static constexpr double kFeedbackTimeout = 1.0;
  static constexpr double kTactileTimeout = 1.5;
  static constexpr double kOpenTolerance = 0.1;
  static constexpr double kThumbTolerance = 0.05;
  static constexpr double kMaxStillVelocity = 0.03; // rad/s
  static constexpr double kLockSettleSeconds = 0.5;
  static constexpr double kSlipCooldownSeconds = 2.0;
  static constexpr double kMinTactileFrameGap = 0.04; // sec: don't count duplicate bursts

  enum class Phase { WAIT_DATA, OPEN, ZERO, FORCE, LOCKED, ERROR };

  const std::array<std::string, kN> joint_names_{
    "thumb_joint1", "thumb_joint2", "index_joint",
    "middle_joint", "ring_joint", "pinky_joint"};
  // EXAMPLE max angles from prior code. Must check real URDF/driver.
  const std::array<double, kN> max_close_{1.0, 0.5, 1.2, 1.2, 1.2, 1.2};

  std::array<double, kN> position_{};
  std::array<double, kN> velocity_{};
  std::array<double, kN> raw_force_{};
  std::array<double, kN> sum_force_{};
  std::array<double, kN> zero_force_{};
  std::array<double, kN> command_{};
  std::array<double, kN> min_force_{};
  std::array<double, kN> max_force_{};
  std::array<bool, kN> stopped_{};
  int calibration_count_ = 0;
  bool have_state_ = false;
  std::string object_name_ = "unspecified";
  bool enable_motion_ = true;
  bool enable_auto_regrasp_ = true;
  Phase phase_ = Phase::WAIT_DATA;

  // Per-finger contact-pressure center (CoP) in taxel-cell units.
  // xi, yi are FIXED cell coordinates; only raw pressure values change.
  double tactile_min_sum_ = 500.0;
  double slip_distance_cells_ = 0.2;
  int slip_confirm_frames_ = 3;
  double pressure_drop_ratio_ = 0.2;
  int pressure_drop_confirm_frames_ = 3;
  struct TactileState {
    bool have_tactile_ = false;
    bool cop_reference_valid_ = false;
    double cop_ref_x_ = 0.0;
    double cop_ref_y_ = 0.0;
    double cop_ref_sum_ = 0.0;
    double last_cop_x_ = 0.0;
    double last_cop_y_ = 0.0;
    double last_tactile_sum_ = 0.0;
    int moving_frames_ = 0;
    int pressure_drop_frames_ = 0;
    unsigned long long slip_events_ = 0;
    bool has_last_slip_ = false;

    std::chrono::steady_clock::time_point tactile_at_{};
    std::chrono::steady_clock::time_point last_tactile_frame_{};
    std::chrono::steady_clock::time_point last_slip_at_{};

    std::array<double, kTaxels> raw_{};
    std::array<double, kTaxels> pressure_{};
    bool cop_valid_ = false;
  };
  std::array<TactileState, kFingers> tactile_{};
  std::chrono::steady_clock::time_point state_at_{}, phase_at_{}, last_adjustment_{};
  std::chrono::steady_clock::time_point stable_since_{};
  std::chrono::steady_clock::time_point force_in_band_since_{};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  std::array<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr, kFingers> tactile_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  static double elapsed(std::chrono::steady_clock::time_point a,
                        std::chrono::steady_clock::time_point b)
  {
    return std::chrono::duration<double>(a - b).count();
  }

  void enter(Phase new_phase, std::chrono::steady_clock::time_point now)
  {
    stable_since_ = {};
    phase_ = new_phase;
    phase_at_ = now;

  }

  void error_stop(const std::string& why)
  {
    phase_ = Phase::ERROR;
    RCLCPP_ERROR(get_logger(), "%s. ROS commands stopped (NOT an E-stop).", why.c_str());
  }

  bool hand_is_ready_for_calibration() const
  {
    if (std::abs(position_[0] - kThumb1Target) > kThumbTolerance) return false;
    for (std::size_t i = 1; i < kN; ++i) {
      if (std::abs(position_[i]) > kOpenTolerance) return false;
    }
    return true;
  }

  double force(std::size_t i) const
  {
    return raw_force_[i] - zero_force_[i];
  }

  void receive_state(const sensor_msgs::msg::JointState& msg)
  {
    std::array<std::size_t, kN> idx{};
    for (std::size_t i = 0; i < kN; ++i) {
      bool found = false;
      for (std::size_t j = 0; j < msg.name.size(); ++j) {
        if (msg.name[j] != joint_names_[i]) continue;
        if (j >= msg.position.size() || j >= msg.effort.size() ||
            !std::isfinite(msg.position[j]) || !std::isfinite(msg.effort[j])) {
          return;
        }
        idx[i] = j;
        found = true;
        break;
      }
      if (!found) return;
    }
    for (std::size_t i = 0; i < kN; ++i) {
      position_[i] = msg.position[idx[i]];
      raw_force_[i] = msg.effort[idx[i]];
      velocity_[i] = (idx[i] < msg.velocity.size() &&
                      std::isfinite(msg.velocity[idx[i]])) ?
        msg.velocity[idx[i]] : std::numeric_limits<double>::quiet_NaN();
    }
    have_state_ = true;
    const auto now = std::chrono::steady_clock::now();
    const bool feedback_gap = have_state_ && elapsed(now, state_at_) > kFeedbackTimeout;
    state_at_ = now;
    // Check every feedback sample so even brief force excursions restart the hold.
    if (phase_ == Phase::FORCE) {
      if (feedback_gap || !all_forces_in_band()) {
        force_in_band_since_ = {};
      } else if (force_in_band_since_ == std::chrono::steady_clock::time_point{}) {
        force_in_band_since_ = now;
      }
    }
    if (phase_ == Phase::LOCKED && fingers_stationary() && !feedback_gap) {
      if (stable_since_ == std::chrono::steady_clock::time_point{}) stable_since_ = now;
    } else {
      stable_since_ = {};
    }
    for (std::size_t f = 1; f <= 3; ++f) {
      auto& t = tactile_[f];
      if (feedback_gap || !finger_tactile_ready(f)) {
        // Pause monitoring; preserve the reference.
        t.moving_frames_ = 0;
        t.pressure_drop_frames_ = 0;
      }
    }
    if (phase_ == Phase::ZERO && calibration_count_ < kCalibrationSamples) {
      if (!hand_is_ready_for_calibration()) {
        error_stop("Hand moved during zero calibration");
        return;
      }
      for (std::size_t i = 0; i < kN; ++i) sum_force_[i] += raw_force_[i];
      ++calibration_count_;
    }
  }

  // Must only operate after ALL fingers have reached their force bands and stopped.
  bool fingers_stationary() const
  {
    for (std::size_t i = 0; i < kN; ++i) {
      if (!std::isfinite(velocity_[i]) ||
          std::abs(velocity_[i]) > kMaxStillVelocity ||
          std::abs(command_[i] - position_[i]) > 0.03) {
        return false;
      }
    }
    return true;
  }

  bool finger_tactile_ready(std::size_t f) const
  {
    const std::size_t i = f + 1;
    if (phase_ != Phase::LOCKED) return false;
    return std::isfinite(velocity_[i]) && std::abs(velocity_[i]) <= kMaxStillVelocity &&
           std::abs(command_[i] - position_[i]) <= 0.005;
  }

  void confirmed_slip(
    std::chrono::steady_clock::time_point now, double shift,
    double x, double y, double total, std::size_t f, const char* source)
  {
    auto& t = tactile_[f];
    const double baseline_sum = t.cop_ref_sum_;
    if (t.has_last_slip_ && elapsed(now, t.last_slip_at_) < kSlipCooldownSeconds) {
      return;
    }
    t.last_slip_at_ = now;
    t.has_last_slip_ = true;
    RCLCPP_WARN(get_logger(),
      "[%s][SLIP SUSPECTED] object=%s source=%s CoP_shift=%.2f cells "
      "CoP=(%.2f,%.2f) pressure_sum=%.0f baseline_sum=%.0f",
      finger_names_[f].c_str(), object_name_.c_str(),
      source, shift, x, y, total, baseline_sum);

    if (!enable_auto_regrasp_) {
      RCLCPP_WARN(get_logger(),
        "[ALERT ONLY] Auto regrasp is disabled. No new motion commanded.");
      return;
    }
    if (!enable_motion_) {
      RCLCPP_WARN(get_logger(),
        "[ALERT ONLY] Motion is disabled. No new motion commanded.");
      return;
    }
    request_regrasp(f);
  }

  void receive_tactile(const std_msgs::msg::Float64MultiArray& msg, std::size_t f)
  {
    auto& t = tactile_[f];
    // Contract: 80 raw taxels per monitored pad, 10x8 row-major.
    if (f == 0 || f == 4) return;
    if (msg.data.size() != taxels(f)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[%s] Expected %zu taxels, received %zu", finger_names_[f].c_str(), taxels(f), msg.data.size());
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    for (double raw : msg.data) {
      if (!std::isfinite(raw) || raw < 0.0 || raw > 65535.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[%s][TACTILE INVALID] Taxel value must be finite and within [0,65535]",
          finger_names_[f].c_str());
        return;
      }
    }
    const bool first_frame = !t.have_tactile_;
    std::copy(msg.data.begin(), msg.data.end(), t.raw_.begin());
    if (t.have_tactile_ && elapsed(now, t.tactile_at_) > kTactileTimeout) {
      // Pause monitoring; preserve the reference.
      t.moving_frames_ = 0;
      t.pressure_drop_frames_ = 0;
    }
    t.have_tactile_ = true;
    t.tactile_at_ = now;
    double total = 0.0, weighted_x = 0.0, weighted_y = 0.0;
    for (std::size_t row = 0; row < rows(f); ++row) {
      for (std::size_t col = 0; col < kCols; ++col) {
        const auto i = row * kCols + col;
        const double p = msg.data[i];
        t.pressure_[i] = p;
        total += p;
        weighted_x += p * static_cast<double>(col);
        weighted_y += p * static_cast<double>(row);
      }
    }
    t.have_tactile_ = true;
    t.tactile_at_ = now;
    t.last_tactile_sum_ = total;
    if (first_frame) {
      RCLCPP_INFO(get_logger(),
        "[%s][TACTILE RX] Received %zu taxels on /hand/tactile/%s_pad, sum=%.0f",
        finger_names_[f].c_str(), msg.data.size(), finger_names_[f].c_str(), total);
    }

    t.cop_valid_ = total >= tactile_min_sum_;
    // Low pressure invalidates CoP, but must still reach pressure-loss monitoring.
    const double x = t.cop_valid_ ? weighted_x / total : t.last_cop_x_;
    const double y = t.cop_valid_ ? weighted_y / total : t.last_cop_y_;
    t.last_cop_x_ = x;
    t.last_cop_y_ = y;

    // After LOCKED, pause on motion/stale feedback without discarding collected samples.
    if (!have_state_ || elapsed(now, state_at_) > kFeedbackTimeout ||
        !finger_tactile_ready(f)) {
      // Pause monitoring; preserve the reference.
      t.moving_frames_ = 0;
      t.pressure_drop_frames_ = 0;
      return;
    }
    if (t.last_tactile_frame_ != std::chrono::steady_clock::time_point{} &&
        elapsed(now, t.last_tactile_frame_) < kMinTactileFrameGap) {
      return;
    }
    t.last_tactile_frame_ = now;

    if (t.cop_reference_valid_ && phase_ != Phase::LOCKED) return;
    if (t.cop_reference_valid_) {
      const bool pressure_drop = total <= t.cop_ref_sum_ * (1.0 - pressure_drop_ratio_);
      t.pressure_drop_frames_ = pressure_drop ? t.pressure_drop_frames_ + 1 : 0;
      if (t.pressure_drop_frames_ >= pressure_drop_confirm_frames_) {
        t.pressure_drop_frames_ = 0;
        t.moving_frames_ = 0;
        const double shift = t.cop_valid_ ?
          std::hypot(x - t.cop_ref_x_, y - t.cop_ref_y_) :
          std::numeric_limits<double>::quiet_NaN();
        confirmed_slip(now, shift, x, y, total, f, "pressure-drop");
        return;
      }
    }
    if (!t.cop_valid_) {
      t.moving_frames_ = 0;
      return;
    }

    if (!t.cop_reference_valid_) {
      t.cop_ref_x_ = x;
      t.cop_ref_y_ = y;
      t.cop_ref_sum_ = total;
      t.cop_reference_valid_ = true;
      t.moving_frames_ = 0;
      t.pressure_drop_frames_ = 0;
      RCLCPP_INFO(get_logger(),
        "[%s][TACTILE] Reference CoP set from first valid frame: (%.2f, %.2f) cells, pressure sum=%.0f",
        finger_names_[f].c_str(), t.cop_ref_x_, t.cop_ref_y_, t.cop_ref_sum_);
      RCLCPP_INFO(get_logger(),
        "[%s] CoP baseline ready", finger_names_[f].c_str());
      RCLCPP_INFO(get_logger(),
        "[%s][SLIP MONITOR] Monitoring CoP shift and pressure-sum drop against the grasp baseline.", finger_names_[f].c_str());
      return;
    }

    const double shift = std::hypot(x - t.cop_ref_x_, y - t.cop_ref_y_);
    if (shift >= slip_distance_cells_) {
      ++t.moving_frames_;
    } else {
      t.moving_frames_ = 0;
    }
    if (t.moving_frames_ < slip_confirm_frames_) return;
    t.moving_frames_ = 0;
    confirmed_slip(now, shift, x, y, total, f, "CoP-shift");
  }

  void request_regrasp(std::size_t f)
  {
    auto& t = tactile_[f];
    if (phase_ != Phase::LOCKED ||
        !std::all_of(stopped_.begin() + 1, stopped_.end(),
          [](bool stopped) {return stopped;})) {
      return;
    }
    const std::size_t i = f + 1; // thumb flexion maps to thumb_joint2
    const double current = std::max(command_[i], position_[i]);
    const double next = std::min(max_close_[i], current + kRegraspAngleStep);
    if (next <= current + 1e-6) {
      RCLCPP_WARN(get_logger(), "[%s] Regrasp blocked by angle limit: angle=%.3f limit=%.3f rad",
        finger_names_[f].c_str(), current, max_close_[i]);
      return;
    }
    command_[i] = next;
    stable_since_ = {};
    ++t.slip_events_;
    t.cop_reference_valid_ = false;
    t.moving_frames_ = 0;
    t.pressure_drop_frames_ = 0;
    t.last_tactile_frame_ = {};
    RCLCPP_INFO(get_logger(), "[%s][BASELINE] New target commanded; setting a new reference from the first valid frame after settling",
      finger_names_[f].c_str());
    RCLCPP_WARN(get_logger(), "[%s][REGRASP #%llu] %s %.3f -> %.3f rad (+%.3f)",
      finger_names_[f].c_str(), t.slip_events_, joint_names_[i].c_str(), current, next, next-current);
  }

  void publish_command()
  {
    std_msgs::msg::Float64MultiArray out;
    out.data.assign(command_.begin(), command_.end());
    command_pub_->publish(out);
  }

  void control_step()
  {
    if (!enable_motion_ || phase_ == Phase::ERROR) return;
    const auto now = std::chrono::steady_clock::now();
    const bool fresh_state = have_state_ && elapsed(now, state_at_) < kFeedbackTimeout;
    const bool controller = command_pub_->get_subscription_count() > 0;
    if (!fresh_state || !controller) {
      if (phase_ != Phase::WAIT_DATA) error_stop("Lost joint feedback or command subscriber");
      return;
    }
    if (phase_ == Phase::WAIT_DATA) {
      enter(Phase::OPEN, now);
      RCLCPP_INFO(get_logger(),
        "[OPEN] Moving to the initial open posture");
    }

    switch (phase_) {
      case Phase::OPEN:
        command_.fill(0.0);
        command_[0] = kThumb1Target;
        if (hand_is_ready_for_calibration()) {
          calibration_count_ = 0;
          sum_force_.fill(0.0);
          for (auto& t : tactile_) { t = TactileState{}; }
          enter(Phase::ZERO, now);
          RCLCPP_INFO(get_logger(),
            "[OPEN] Initial open posture reached");
          RCLCPP_INFO(get_logger(),
            "[FORCE ZERO] Collecting 20 unloaded force samples");
        } else if (elapsed(now, phase_at_) > 13.0) {
          error_stop("Opening timed out");
          return;
        }
        break;
      case Phase::ZERO:
        command_.fill(0.0);
        command_[0] = kThumb1Target;
        if (calibration_count_ >= kCalibrationSamples) {
          for (std::size_t i = 0; i < kN; ++i) {
            zero_force_[i] = sum_force_[i] / kCalibrationSamples;
          }
          for (std::size_t i = 1; i < kN; ++i) {
            command_[i] = std::clamp(position_[i], 0.0, max_close_[i]);
          }
          stopped_.fill(false);
          stopped_[0] = true;
          force_in_band_since_ = {};
          last_adjustment_ = now - 1s;
          enter(Phase::FORCE, now);
          RCLCPP_INFO(get_logger(),
            "[FORCE ZERO DONE] Tactile baseline collection will start after LOCKED");
          RCLCPP_INFO(get_logger(),
            "[FORCE] Starting other 5 fingers from current angles; "
            "shared force bands [%.1f, %.1f]",
            min_force_[kIndex], max_force_[kIndex]);
        } else if (elapsed(now, phase_at_) > 10.0) {
          error_stop("20-sample calibration timeout");
          return;
        }
        break;
      case Phase::FORCE:
        if (elapsed(now, last_adjustment_) >= kAdjustmentInterval) {
          for (std::size_t i = 1; i < kN; ++i) {
            const double measured = force(i);
            if (measured >= min_force_[i] && measured <= max_force_[i]) {
              if (!stopped_[i]) {
                command_[i] = std::clamp(position_[i], 0.0, max_close_[i]);
                if (i > 1) {
                  RCLCPP_INFO(get_logger(), "[HOLD] %s F=%.1f angle=%.3f",
                    joint_names_[i].c_str(), measured, command_[i]);
                }
              }
            } else if (measured < min_force_[i]) {
              // Wait for position response before moving again.
              if (std::abs(command_[i] - position_[i]) <= kPositionTrackingTolerance) {
                if (command_[i] >= max_close_[i] - 1e-6) {
                  error_stop(joint_names_[i] + " reached max angle without target force");
                  return;
                }
                command_[i] = std::min(max_close_[i], command_[i] + kAngleStep);
              }
            } else {
              command_[i] = std::max(0.0,
                std::min(command_[i], position_[i]) - kAngleStep);
            }
            stopped_[i] = measured >= min_force_[i] && measured <= max_force_[i];
          }
          last_adjustment_ = now;
          if (all_forces_in_band() &&
              force_in_band_since_ != std::chrono::steady_clock::time_point{} &&
              elapsed(now, force_in_band_since_) >= kForceHoldSeconds) {
            enter(Phase::LOCKED, now);
            RCLCPP_INFO(get_logger(),
              "================ ALL 5 FINGERS LOCKED ================");
            for (std::size_t f = 1; f < kFingers; ++f) {
              const auto& t = tactile_[f];
              RCLCPP_INFO(get_logger(),
                "[LOCKED][%s] force=%.1f band=[%.1f,%.1f] target=%.3f rad "
                "tactile_rx=%s pressure_sum=%.0f",
                finger_names_[f].c_str(), force(f + 1), min_force_[f + 1], max_force_[f + 1],
                command_[f + 1],
                (f == 0 || f == 4) ? "disabled" : (!t.have_tactile_ ? "none" :
                (elapsed(now, t.tactile_at_) >= kTactileTimeout ? "stale" : "fresh")),
                t.last_tactile_sum_);
            }
            RCLCPP_INFO(get_logger(),
              "[LOCKED] All 5 fingers maintained their force bands for 3 seconds; holding joint targets. "
              "Waiting for stable tactile CoP baseline. NOT proof of secure grasp.");
          }
        }
        break;
      case Phase::LOCKED:
        // Keep target angles. Tactile callback may initiate a restricted regrasp.
        break;
      case Phase::WAIT_DATA:
      case Phase::ERROR:
        return;
    }
    publish_command();
  }

  bool all_forces_in_band() const
  {
    for (std::size_t i = 1; i < kN; ++i) {
      if (force(i) < min_force_[i] || force(i) > max_force_[i]) return false;
    }
    return true;
  }

  void report_state()
  {
    const auto now = std::chrono::steady_clock::now();
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    for (std::size_t f = 1; f < kFingers; ++f) {
      if (f > 1) out << " ";
      const auto& t = tactile_[f];
      out << finger_names_[f] << "=";
      if (f == 0 || f == 4) {
        out << "DISABLED";
        continue;
      }
      if (!t.have_tactile_) {
        out << "N/A(no-data)";
      } else if (elapsed(now, t.tactile_at_) >= kTactileTimeout) {
        out << "N/A(stale,age=" << elapsed(now, t.tactile_at_) << "s)";
      } else if (!t.cop_valid_) {
        out << (t.last_tactile_sum_ == 0.0 ? "N/A(all-zero-frame,sum=" : "N/A(low-pressure,sum=")
            << t.last_tactile_sum_
            << ",min=" << tactile_min_sum_ << ")";
      } else {
        out << "(" << t.last_cop_x_ << "," << t.last_cop_y_ << ")";

      }
    }
    RCLCPP_INFO(get_logger(), "[CoP] %s", out.str().c_str());

  }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandControlNode>());
  rclcpp::shutdown();
  return 0;
}
