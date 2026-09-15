#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__GUARD_ENABLE_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__GUARD_ENABLE_HPP_

#include "platform/platform_config.h"

#if defined(PLATFORM_FREERTOS) && !defined(PLATFORM_FREERTOS_S32Z2) && \
  !defined(PLATFORM_FREERTOS_X5H)
#include <cstdlib>
#include <cstring>
#endif

namespace autoware::motion::control::trajectory_follower_node
{

inline bool guard_compiled_in()
{
#if defined(CONFIG_GUARD_ENABLE) && CONFIG_GUARD_ENABLE
  return true;
#else
  return false;
#endif
}

inline bool guard_enabled()
{
  if (!guard_compiled_in()) {
    return false;
  }
#if defined(PLATFORM_FREERTOS) && !defined(PLATFORM_FREERTOS_S32Z2) && \
  !defined(PLATFORM_FREERTOS_X5H)
  const char * env = std::getenv("GUARD");
  if (env != nullptr && env[0] != '\0') {
    if (env[0] == '0' || env[0] == 'n' || env[0] == 'N' || env[0] == 'f' || env[0] == 'F' ||
        std::strcmp(env, "off") == 0 || std::strcmp(env, "OFF") == 0)
    {
      return false;
    }
  }
#endif
  return true;
}

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__GUARD_ENABLE_HPP_
