#ifndef ROBOT_ADMITTANCE_CONTROL_ADMITTANCE_FREEZE_MODE_H
#define ROBOT_ADMITTANCE_CONTROL_ADMITTANCE_FREEZE_MODE_H

#include <algorithm>
#include <string>
#include <vector>

namespace robot_admittance_control
{

inline bool shouldFreezeAdmittanceForState(
  const std::string & state_value,
  const std::vector<std::string> & freeze_states)
{
  if (state_value.empty()) {
    return false;
  }
  return std::find(freeze_states.begin(), freeze_states.end(), state_value) !=
    freeze_states.end();
}

}  // namespace robot_admittance_control

#endif  // ROBOT_ADMITTANCE_CONTROL_ADMITTANCE_FREEZE_MODE_H
