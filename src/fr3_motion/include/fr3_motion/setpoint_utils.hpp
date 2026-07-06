// Copyright (c) 2026 Dimas Abreu Archanjo Dutra
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>

namespace fr3_motion {

/**
 * Decides whether an incoming setpoint should replace the currently held one.
 *
 * A setpoint is rejected when its timestamp is older than the latest setpoint kept so
 * far, or when it is already older than `timeout_seconds` by the time it is received.
 * Rejections are logged at INFO level, as required for observability of dropped
 * setpoints.
 *
 * \param[in] new_stamp timestamp of the incoming setpoint message.
 * \param[in] latest_stamp timestamp of the setpoint currently held (zero if none yet).
 * \param[in] now current time, used to evaluate the timeout.
 * \param[in] timeout_seconds maximum allowed age of a setpoint at reception time. A
 * non-finite value (e.g. infinity) disables the timeout check.
 * \param[in] logger logger used to report dropped setpoints.
 * \param[in] what short, human-readable name of the setpoint kind (e.g. "joint state"),
 * used only in the log message.
 * \return true if the setpoint should be accepted, false if it should be dropped.
 */
inline bool accept_setpoint(const rclcpp::Time& new_stamp,
                            const rclcpp::Time& latest_stamp,
                            const rclcpp::Time& now,
                            double timeout_seconds,
                            const rclcpp::Logger& logger,
                            const char* what) {
  if (latest_stamp.nanoseconds() != 0 && new_stamp < latest_stamp) {
    RCLCPP_INFO(logger,
                "Dropping %s setpoint: its timestamp is older than the latest received "
                "setpoint.",
                what);
    return false;
  }

  if (std::isfinite(timeout_seconds)) {
    const double age = (now - new_stamp).seconds();
    if (age > timeout_seconds) {
      RCLCPP_INFO(logger, "Dropping %s setpoint: age %.3f s exceeds setpoint_timeout %.3f s.", what,
                  age, timeout_seconds);
      return false;
    }
  }

  return true;
}

}  // namespace fr3_motion
