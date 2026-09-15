#include "autoware/trajectory_follower_node/guard_policy.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using autoware::motion::control::trajectory_follower_node::FollowerCmd;
using autoware::motion::control::trajectory_follower_node::GuardPolicy;
using autoware::motion::control::trajectory_follower_node::GuardState;
using autoware::motion::control::trajectory_follower_node::TrajSnapshot;
using autoware::motion::control::trajectory_follower_node::EgoSnapshot;
using autoware::motion::control::trajectory_follower_node::kClearFrames;
using autoware::motion::control::trajectory_follower_node::kFollowerTimeoutMs;
using autoware::motion::control::trajectory_follower_node::kTrajTimeoutMs;
using autoware::motion::control::trajectory_follower_node::guard_state_name;

static int g_fails = 0;

static void expect(bool ok, const char * msg)
{
  if (!ok) {
    std::fprintf(stderr, "FAIL: %s\n", msg);
    g_fails += 1;
  }
}

static FollowerCmd cmd(double t, double v = 3.0, double a = 0.0, double s = 0.0)
{
  FollowerCmd c;
  c.stamp_ms = t;
  c.velocity = v;
  c.accel = a;
  c.steer = s;
  return c;
}

static TrajSnapshot traj(double t, double v = 3.0, int n = 5)
{
  TrajSnapshot tr;
  tr.stamp_ms = t;
  tr.n_speeds = n;
  for (int i = 0; i < n; ++i) {
    tr.speeds[i] = v;
  }
  tr.curvature = 0.0;
  return tr;
}

static EgoSnapshot ego(double t, double v = 3.0)
{
  EgoSnapshot e;
  e.stamp_ms = t;
  e.speed = v;
  return e;
}

static void live(GuardPolicy & g, double t, double v = 3.0)
{
  g.on_follower(cmd(t, v));
  g.on_traj(traj(t, v), t);
  g.on_odom(ego(t, v));
}

static double drive_comfortable(GuardPolicy & g)
{
  live(g, 1000.0);
  g.step(1010.0);
  const double step_t = g.state == GuardState::FRESH ? 1000.0 + kTrajTimeoutMs + 10.0 : 2010.0;
  (void)step_t;
  const double t = 1000.0 + kTrajTimeoutMs + 10.0;
  g.on_follower(cmd(t));
  g.on_odom(ego(t));
  auto out = g.step(t);
  expect(out.state == GuardState::COMFORTABLE, "drive_comfortable");
  return t;
}

int main()
{
  {
    GuardPolicy g;
    auto out = g.step(100.0);
    expect(out.state == GuardState::INIT, "startup INIT");
    expect(out.has_override, "startup override");
  }
  {
    GuardPolicy g;
    live(g, 1000.0);
    auto out = g.step(1010.0);
    expect(out.state == GuardState::FRESH, "fresh");
    expect(!out.has_override, "fresh passthrough");
  }
  {
    GuardPolicy g;
    live(g, 1000.0);
    g.step(1010.0);
    auto out = g.step(1000.0 + kFollowerTimeoutMs + 10.0);
    expect(out.state == GuardState::EMERGENCY, "stale follower");
    expect(out.has_override && out.override_cmd.velocity == 0.0, "emergency brake v");
  }
  {
    GuardPolicy g;
    drive_comfortable(g);
    expect(g.state == GuardState::COMFORTABLE, "stale traj comfortable");
  }
  {
    GuardPolicy g;
    live(g, 1000.0);
    g.step(1010.0);
    auto out = g.step(5000.0);
    expect(out.state == GuardState::EMERGENCY, "everything stale emergency");
  }
  {
    GuardPolicy g;
    live(g, 1000.0);
    g.step(1010.0);
    g.on_follower(cmd(1020.0, NAN));
    auto out = g.step(1020.0);
    expect(out.state == GuardState::EMERGENCY, "nan follower");
    expect(std::strcmp(out.reason, "invalid-follower") == 0, "nan reason");
  }
  {
    GuardPolicy g;
    live(g, 1000.0, 3.0);
    g.step(1010.0);
    g.on_follower(cmd(1020.0, 3.0));
    g.on_follower(cmd(1030.0, 25.0));
    g.on_odom(ego(1030.0));
    g.on_traj(traj(1030.0), 1030.0);
    auto out = g.step(1030.0);
    expect(out.state == GuardState::EMERGENCY, "babble jump");
  }
  {
    GuardPolicy g;
    live(g, 1000.0);
    g.step(1010.0);
    TrajSnapshot tr = traj(1020.0, 10.0);
    tr.curvature = 0.05;
    g.on_traj(tr, 1020.0);
    g.on_follower(cmd(1020.0, 10.0, 0.0));
    g.on_odom(ego(1020.0, 10.0));
    auto out = g.step(1020.0);
    expect(out.state == GuardState::EMERGENCY, "envelope");
    expect(std::strcmp(out.reason, "envelope-violation") == 0, "envelope reason");
  }
  {
    GuardPolicy g;
    const double base = drive_comfortable(g);
    autoware::motion::control::trajectory_follower_node::GuardOutput out{};
    for (int i = 1; i <= kClearFrames; ++i) {
      const double t = base + i * 100.0;
      g.on_follower(cmd(t));
      g.on_traj(traj(t), t);
      g.on_odom(ego(t));
      out = g.step(t);
    }
    expect(out.state == GuardState::FRESH, "traj recovered");
    expect(!out.has_override, "recovered passthrough");
  }
  {
    GuardPolicy g;
    live(g, 1000.0, 0.0);
    g.step(1010.0);
    g.on_follower(cmd(1020.0, NAN));
    g.on_odom(ego(1020.0, 0.0));
    g.on_traj(traj(1020.0, 0.0), 1020.0);
    auto out = g.step(1020.0);
    expect(out.state == GuardState::EMERGENCY, "nan at rest emergency");
    g.on_follower(cmd(1030.0, 0.0));
    g.on_odom(ego(1030.0, 0.0));
    g.on_traj(traj(1030.0, 0.0), 1030.0);
    out = g.step(1030.0);
    expect(out.state == GuardState::HOLD, "latches hold");
    live(g, 2000.0, 0.0);
    out = g.step(2010.0);
    expect(out.state == GuardState::HOLD, "hold without reenable");
    out = g.step(2020.0, true);
    expect(out.state == GuardState::FRESH, "re-enable");
  }
  {
    GuardPolicy g;
    live(g, 1000.0, 0.0);
    g.step(1010.0);
    auto out = g.step(5000.0);
    g.on_odom(ego(5001.0, 0.0));
    out = g.step(5001.0);
    expect(out.state == GuardState::HOLD, "stale to hold");
    out = g.step(5010.0, true);
    expect(out.state == GuardState::HOLD, "reenable refused");
    expect(std::strstr(out.reason, "refused") != nullptr, "refused reason");
  }
  {
    GuardPolicy g;
    const double base = drive_comfortable(g);
    auto out = g.step(base + kFollowerTimeoutMs + 100.0);
    expect(out.state == GuardState::EMERGENCY, "comfortable then follower death");
  }
  {
    GuardPolicy g;
    live(g, 1000.0, 16.0);
    g.step(1010.0);
    g.on_follower(cmd(1020.0, 16.0, -5.0));
    g.on_traj(traj(1020.0, 16.0), 1020.0);
    g.on_odom(ego(1020.0, 16.0));
    auto out = g.step(1020.0);
    expect(out.state == GuardState::FRESH, "hard stop not a fault");
  }
  {
    GuardPolicy g;
    live(g, 1000.0, 3.0);
    g.step(1010.0);
    g.on_follower(cmd(1020.0, 3.0, 5.0));
    g.on_traj(traj(1020.0), 1020.0);
    g.on_odom(ego(1020.0));
    auto out = g.step(1020.0);
    expect(out.state == GuardState::EMERGENCY, "runaway throttle");
    expect(std::strcmp(out.reason, "envelope-violation") == 0, "runaway reason");
  }
  {
    GuardPolicy g;
    live(g, 1000.0, 0.0);
    g.step(1010.0);
    g.on_follower(cmd(1020.0, NAN));
    g.step(1020.0);
    g.on_follower(cmd(1030.0, 0.0));
    g.on_odom(ego(1030.0, 0.0));
    g.on_traj(traj(1030.0, 0.0), 1030.0);
    g.step(1030.0);
    live(g, 2000.0, 0.0);
    auto out = g.step(2010.0, true);
    expect(out.state == GuardState::FRESH, "fresh after hold");
    out = g.step(2015.0);
    expect(out.state == GuardState::FRESH, "timer before next cmd");
    g.on_follower(cmd(2020.0, 2.0, 0.0, 0.5));
    g.on_traj(traj(2020.0, 2.0), 2020.0);
    g.on_odom(ego(2020.0, 0.5));
    out = g.step(2020.0);
    expect(out.state == GuardState::FRESH, "adopt no compare");
    g.on_follower(cmd(2030.0, 2.0, 0.0, -0.5));
    g.on_traj(traj(2030.0, 2.0), 2030.0);
    g.on_odom(ego(2030.0, 0.5));
    out = g.step(2030.0);
    expect(out.state == GuardState::EMERGENCY, "jump after baseline");
  }
  {
    GuardPolicy g;
    live(g, 1000.0);
    g.on_follower(cmd(1010.0, 3.0, 0.0, 0.2));
    auto out = g.step(5000.0);
    expect(out.has_override, "brake has override");
    expect(std::fabs(out.override_cmd.steer - 0.2) < 1e-9, "brake freezes steer");
  }

  if (g_fails != 0) {
    std::fprintf(stderr, "%d failed\n", g_fails);
    return 1;
  }
  std::printf("guard_policy_test: all passed\n");
  return 0;
}
