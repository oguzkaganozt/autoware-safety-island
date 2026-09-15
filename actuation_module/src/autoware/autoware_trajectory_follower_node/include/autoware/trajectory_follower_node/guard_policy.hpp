#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__GUARD_POLICY_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__GUARD_POLICY_HPP_

#include <cmath>
#include <cstddef>

namespace autoware::motion::control::trajectory_follower_node
{

enum class GuardState
{
  INIT = 0,
  FRESH,
  COMFORTABLE,
  EMERGENCY,
  HOLD,
};

inline const char * guard_state_name(const GuardState s)
{
  switch (s) {
    case GuardState::INIT:
      return "INIT";
    case GuardState::FRESH:
      return "FRESH";
    case GuardState::COMFORTABLE:
      return "COMFORTABLE";
    case GuardState::EMERGENCY:
      return "EMERGENCY";
    case GuardState::HOLD:
      return "HOLD";
  }
  return "INIT";
}

inline constexpr double kFollowerTimeoutMs = 500.0;
inline constexpr double kTrajTimeoutMs = 3000.0;
inline constexpr double kOdomTimeoutMs = 500.0;
inline constexpr double kBabbleRateHz = 40.0;
inline constexpr double kBabbleDvMps = 8.0;
inline constexpr double kBabbleDsteerRad = 0.4;
inline constexpr double kBabbleSteerVRefMps = 5.0;
inline constexpr double kBabbleDaMps2 = 6.0;
inline constexpr double kBabbleSteerGateMps = 0.5;
inline constexpr double kALonPosMax = 4.0;
inline constexpr double kALonNegMax = 6.0;
inline constexpr double kALatMax = 3.0;
inline constexpr double kEnvelopeMargin = 1.2;
inline constexpr double kVMaxMps = 40.0;
inline constexpr double kSteerMaxRad = 0.65;
inline constexpr int kClearFrames = 10;
inline constexpr double kComfortableDecel = -0.8;
inline constexpr double kEmergencyDecel = -3.0;
inline constexpr int kMaxSpeeds = 16;
inline constexpr int kArrivalCap = 8;

struct FollowerCmd
{
  double stamp_ms{0.0};
  double velocity{0.0};
  double accel{0.0};
  double steer{0.0};
};

struct TrajSnapshot
{
  double stamp_ms{0.0};
  double speeds[kMaxSpeeds]{};
  int n_speeds{0};
  double curvature{0.0};
};

struct EgoSnapshot
{
  double stamp_ms{0.0};
  double speed{0.0};
};

struct GuardOutput
{
  GuardState state{GuardState::INIT};
  const char * reason{"startup"};
  bool has_override{false};
  FollowerCmd override_cmd{};
};

inline bool guard_finite(const double v)
{
  return std::isfinite(v);
}

class GuardPolicy
{
public:
  GuardState state{GuardState::INIT};
  const char * reason{"startup"};
  int false_stop_entries{0};

  void on_follower(const FollowerCmd & cmd)
  {
    if (skip_jump_once_) {
      skip_jump_once_ = false;
      prev_follower_ = cmd;
      has_prev_ = true;
    } else {
      prev_follower_ = last_follower_;
      has_prev_ = has_follower_;
    }
    last_follower_ = cmd;
    has_follower_ = true;
    follower_at_ = cmd.stamp_ms;
    if (arrival_n_ < kArrivalCap) {
      arrival_times_[arrival_n_++] = cmd.stamp_ms;
    } else {
      for (int i = 1; i < kArrivalCap; ++i) {
        arrival_times_[i - 1] = arrival_times_[i];
      }
      arrival_times_[kArrivalCap - 1] = cmd.stamp_ms;
    }
  }

  void on_traj(const TrajSnapshot & traj, const double now_ms)
  {
    traj_ = traj;
    has_traj_ = true;
    traj_at_ = now_ms;
  }

  void on_odom(const EgoSnapshot & ego)
  {
    ego_ = ego;
    has_ego_ = true;
    ego_at_ = ego.stamp_ms;
  }

  GuardOutput step(const double now_ms, const bool re_enable = false)
  {
    const char * f_fault = follower_fault(now_ms);
    const char * t_fault = traj_fault(now_ms);
    const char * e_fault = ego_fault(now_ms);
    const bool ego_stopped = has_ego_ && ego_.speed < 0.1;

    if (state == GuardState::INIT) {
      if (f_fault[0] == '\0' && t_fault[0] == '\0' && e_fault[0] == '\0') {
        return enter(GuardState::FRESH, "inputs-valid", now_ms);
      }
      return make_out(GuardState::INIT, reason, true, brake_cmd(now_ms));
    }

    if (state == GuardState::FRESH) {
      if (e_fault[0] != '\0') {
        return enter(GuardState::EMERGENCY, e_fault, now_ms);
      }
      if (f_fault[0] != '\0') {
        return enter(GuardState::EMERGENCY, f_fault, now_ms);
      }
      if (t_fault[0] != '\0') {
        return enter(GuardState::COMFORTABLE, t_fault, now_ms);
      }
      clear_count_ = 0;
      return make_out(GuardState::FRESH, "ok", false, FollowerCmd{});
    }

    if (state == GuardState::COMFORTABLE) {
      if (f_fault[0] != '\0' || e_fault[0] != '\0') {
        return enter(GuardState::EMERGENCY, f_fault[0] != '\0' ? f_fault : e_fault, now_ms);
      }
      if (t_fault[0] == '\0') {
        clear_count_ += 1;
        if (clear_count_ >= kClearFrames) {
          return enter(GuardState::FRESH, "traj-recovered", now_ms);
        }
      } else {
        clear_count_ = 0;
      }
      return make_out(GuardState::COMFORTABLE, reason, true, comfortable_cmd(now_ms));
    }

    if (state == GuardState::EMERGENCY) {
      if (ego_stopped) {
        return enter(GuardState::HOLD, "emergency-stop", now_ms);
      }
      return make_out(GuardState::EMERGENCY, reason, true, brake_cmd(now_ms));
    }

    if (state == GuardState::HOLD) {
      if (re_enable && f_fault[0] == '\0' && t_fault[0] == '\0' && e_fault[0] == '\0') {
        return enter(GuardState::FRESH, "re-enabled", now_ms);
      }
      if (re_enable) {
        reason = "re-enable-refused-fault-present";
      }
      return make_out(GuardState::HOLD, reason, true, brake_cmd(now_ms));
    }

    return make_out(GuardState::INIT, "startup", true, brake_cmd(now_ms));
  }

private:
  bool skip_jump_once_{false};
  bool has_follower_{false};
  bool has_prev_{false};
  bool has_traj_{false};
  bool has_ego_{false};
  FollowerCmd last_follower_{};
  FollowerCmd prev_follower_{};
  double follower_at_{0.0};
  double arrival_times_[kArrivalCap]{};
  int arrival_n_{0};
  TrajSnapshot traj_{};
  double traj_at_{0.0};
  EgoSnapshot ego_{};
  double ego_at_{0.0};
  int clear_count_{0};

  static GuardOutput make_out(
    const GuardState s, const char * r, const bool has_ov, const FollowerCmd & ov)
  {
    GuardOutput o;
    o.state = s;
    o.reason = r;
    o.has_override = has_ov;
    o.override_cmd = ov;
    return o;
  }

  GuardOutput enter(const GuardState s, const char * r, const double now_ms)
  {
    state = s;
    reason = r;
    clear_count_ = 0;
    if (s == GuardState::FRESH) {
      skip_jump_once_ = true;
      arrival_n_ = 0;
    }
    if (s == GuardState::EMERGENCY) {
      false_stop_entries += 1;
    }
    if (s == GuardState::FRESH) {
      return make_out(s, r, false, FollowerCmd{});
    }
    if (s == GuardState::COMFORTABLE) {
      return make_out(s, r, true, comfortable_cmd(now_ms));
    }
    return make_out(s, r, true, brake_cmd(now_ms));
  }

  FollowerCmd comfortable_cmd(const double now_ms) const
  {
    FollowerCmd b;
    b.stamp_ms = now_ms;
    const double v =
      (has_follower_ && guard_finite(last_follower_.velocity)) ? last_follower_.velocity : 0.0;
    b.velocity = v > 0.0 ? v : 0.0;
    b.accel = kComfortableDecel;
    b.steer = (has_follower_ && guard_finite(last_follower_.steer)) ? last_follower_.steer : 0.0;
    return b;
  }

  FollowerCmd brake_cmd(const double now_ms) const
  {
    FollowerCmd b;
    b.stamp_ms = now_ms;
    b.velocity = 0.0;
    b.accel = kEmergencyDecel;
    b.steer = (has_follower_ && guard_finite(last_follower_.steer)) ? last_follower_.steer : 0.0;
    return b;
  }

  const char * follower_fault(const double now_ms) const
  {
    if (!has_follower_) {
      return "no-follower-yet";
    }
    if (now_ms - follower_at_ > kFollowerTimeoutMs) {
      return "stale-follower";
    }
    const FollowerCmd & cmd = last_follower_;
    if (!guard_finite(cmd.velocity) || !guard_finite(cmd.accel) || !guard_finite(cmd.steer)) {
      return "invalid-follower";
    }
    if (arrival_n_ >= 3) {
      const double span = (arrival_times_[arrival_n_ - 1] - arrival_times_[0]) / 1000.0;
      const double rate = static_cast<double>(arrival_n_ - 1) / (span > 1e-6 ? span : 1e-6);
      if (rate > kBabbleRateHz) {
        return "babbling-rate";
      }
    }
    if (has_prev_ && last_follower_.stamp_ms > prev_follower_.stamp_ms) {
      if (cmd.velocity - prev_follower_.velocity > kBabbleDvMps &&
          std::fabs(prev_follower_.velocity) >= kBabbleSteerGateMps) {
        return "babbling-jump-v";
      }
      const double v_prev = guard_finite(prev_follower_.velocity) ? std::fabs(prev_follower_.velocity) : 0.0;
      const double v_cmd = std::fabs(cmd.velocity);
      const bool near_stop = v_prev < kBabbleSteerGateMps && v_cmd < kBabbleSteerGateMps;
      const double v_scale = (v_prev > v_cmd ? v_prev : v_cmd);
      const double v_ref = v_scale < kBabbleSteerGateMps ? kBabbleSteerGateMps : v_scale;
      double scale = kBabbleSteerVRefMps / v_ref;
      if (scale > 2.0) {
        scale = 2.0;
      } else if (scale < 0.5) {
        scale = 0.5;
      }
      const double steer_lim = kBabbleDsteerRad * scale;
      if (!near_stop && std::fabs(cmd.steer - prev_follower_.steer) > steer_lim) {
        return "babbling-jump-steer";
      }
      if (std::fabs(cmd.accel - prev_follower_.accel) > kBabbleDaMps2) {
        return "babbling-jump-a";
      }
    }
    if (cmd.velocity < -0.5 || cmd.velocity > kVMaxMps) {
      return "follower-vel-range";
    }
    if (std::fabs(cmd.steer) > kSteerMaxRad) {
      return "follower-steer-range";
    }
    const double curv = (has_traj_ && guard_finite(traj_.curvature)) ? traj_.curvature : 0.0;
    const double lat_acc = cmd.velocity * cmd.velocity * curv;
    const double lon = cmd.accel >= 0.0 ? cmd.accel / kALonPosMax : cmd.accel / kALonNegMax;
    const double lat = lat_acc / kALatMax;
    if (lon * lon + lat * lat > kEnvelopeMargin * kEnvelopeMargin) {
      return "envelope-violation";
    }
    return "";
  }

  const char * traj_fault(const double now_ms) const
  {
    if (!has_traj_) {
      return "no-traj-yet";
    }
    if (now_ms - traj_at_ > kTrajTimeoutMs) {
      return "stale-traj";
    }
    if (traj_.n_speeds < 2) {
      return "invalid-traj";
    }
    for (int i = 0; i < traj_.n_speeds; ++i) {
      if (!guard_finite(traj_.speeds[i])) {
        return "invalid-traj";
      }
      if (traj_.speeds[i] < -0.5 || traj_.speeds[i] > kVMaxMps) {
        return "traj-range";
      }
    }
    return "";
  }

  const char * ego_fault(const double now_ms) const
  {
    if (!has_ego_) {
      return "no-odom-yet";
    }
    if (now_ms - ego_at_ > kOdomTimeoutMs) {
      return "stale-odom";
    }
    if (!guard_finite(ego_.speed) || ego_.speed < -1.0) {
      return "invalid-odom";
    }
    return "";
  }
};

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__GUARD_POLICY_HPP_
