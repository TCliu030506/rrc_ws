#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "robot_admittance_control/admittance_freeze_mode.h"

TEST(AdmittanceFreezeMode, FreezesOnlyForConfiguredStates)
{
  const std::vector<std::string> freeze_states{"approach", "pre_contact"};

  EXPECT_TRUE(robot_admittance_control::shouldFreezeAdmittanceForState(
    "approach", freeze_states));
  EXPECT_TRUE(robot_admittance_control::shouldFreezeAdmittanceForState(
    "pre_contact", freeze_states));
  EXPECT_FALSE(robot_admittance_control::shouldFreezeAdmittanceForState(
    "contact_settle", freeze_states));
  EXPECT_FALSE(robot_admittance_control::shouldFreezeAdmittanceForState(
    "contact_scan", freeze_states));
}

TEST(AdmittanceFreezeMode, DoesNotFreezeWithoutState)
{
  const std::vector<std::string> freeze_states{"approach", "pre_contact"};

  EXPECT_FALSE(robot_admittance_control::shouldFreezeAdmittanceForState(
    "", freeze_states));
}
