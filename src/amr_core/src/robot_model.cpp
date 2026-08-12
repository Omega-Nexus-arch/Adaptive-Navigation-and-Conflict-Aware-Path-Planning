// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Deliberately kept free of yaml-cpp so the enum helpers can be linked (and
// unit-tested) without dragging in the configuration loader.

#include "amr_core/robot_model.hpp"

#include <string>

namespace amr_core
{

RobotRole RoleFromString(const std::string & text)
{
  if (text == "mapper") {
    return RobotRole::kMapper;
  }
  if (text == "scout") {
    return RobotRole::kScout;
  }
  // Anything else - including a case-mismatched "Mapper" - is unknown rather
  // than a best guess. Silently promoting a typo to a real role is how a scout
  // ends up with a mapper's priority.
  return RobotRole::kUnknown;
}

const char * RoleToString(RobotRole role)
{
  switch (role) {
    case RobotRole::kMapper:
      return "mapper";
    case RobotRole::kScout:
      return "scout";
    case RobotRole::kUnknown:
    default:
      return "unknown";
  }
}

}  // namespace amr_core
