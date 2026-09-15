#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__RUN_MODE_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__RUN_MODE_HPP_

#include "platform/platform_config.h"

#if defined(PLATFORM_FREERTOS) && !defined(PLATFORM_FREERTOS_S32Z2) && \
  !defined(PLATFORM_FREERTOS_X5H)
#include <cstdlib>
#include <cstring>
#endif

namespace autoware::motion::control::trajectory_follower_node
{

enum class SiMode
{
  Run = 0,
  Autoware,
  Stop,
};

inline SiMode si_mode_compiled()
{
#if defined(CONFIG_SI_MODE_AUTOWARE) && CONFIG_SI_MODE_AUTOWARE
  return SiMode::Autoware;
#elif defined(CONFIG_SI_MODE_STOP) && CONFIG_SI_MODE_STOP
  return SiMode::Stop;
#else
  return SiMode::Run;
#endif
}

inline SiMode si_mode()
{
  SiMode mode = si_mode_compiled();
#if defined(PLATFORM_FREERTOS) && !defined(PLATFORM_FREERTOS_S32Z2) && \
  !defined(PLATFORM_FREERTOS_X5H)
  const char * env = std::getenv("MODE");
  if (env != nullptr && env[0] != '\0') {
    if (std::strcmp(env, "autoware") == 0 || std::strcmp(env, "AUTOWARE") == 0 ||
        env[0] == 'a' || env[0] == 'A')
    {
      return SiMode::Autoware;
    }
    if (std::strcmp(env, "stop") == 0 || std::strcmp(env, "STOP") == 0 || env[0] == 's' ||
        env[0] == 'S' || env[0] == '0')
    {
      return SiMode::Stop;
    }
    if (std::strcmp(env, "run") == 0 || std::strcmp(env, "RUN") == 0 ||
        std::strcmp(env, "follow") == 0 || env[0] == 'r' || env[0] == 'R' || env[0] == '1')
    {
      return SiMode::Run;
    }
  }
#endif
  return mode;
}

inline const char * si_mode_name(const SiMode mode)
{
  switch (mode) {
    case SiMode::Autoware:
      return "AUTOWARE";
    case SiMode::Stop:
      return "STOP";
    case SiMode::Run:
    default:
      return "RUN";
  }
}

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__RUN_MODE_HPP_
