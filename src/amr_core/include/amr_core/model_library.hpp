// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_CORE__MODEL_LIBRARY_HPP_
#define AMR_CORE__MODEL_LIBRARY_HPP_

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "amr_core/robot_model.hpp"

namespace amr_core
{

/// \brief Raised when a configuration file is missing, malformed, or refers to
///        a model that does not exist.
///
/// Configuration errors are fatal by design. A navigation stack that starts
/// with half a profile is more dangerous than one that refuses to start.
class ConfigError : public std::runtime_error
{
public:
  explicit ConfigError(const std::string & message)
  : std::runtime_error("amr_core config error: " + message) {}
};

/// \brief Parsed form of `amr_description/config/robot_models.yaml`.
///
/// ### Why this class exists
///
/// Before this refactoring the same physical constants appeared in at least
/// four places: the xacro description, two nav2 parameter files, and hard-coded
/// defaults inside the control nodes. Nothing enforced agreement, so tuning
/// AMR-2's acceleration in nav2 silently left the smoother using the old value.
///
/// `ModelLibrary` reads the one YAML file that the xacro also reads and hands
/// out an immutable, fully-typed `RobotProfile`. Disagreement is no longer
/// possible, and a missing key is an exception at startup rather than a
/// silently defaulted zero at 3 a.m. See docs/REFACTORING.md.
class ModelLibrary
{
public:
  ModelLibrary() = default;

  /// \brief Load every model block from \p yaml_path.
  /// \throws ConfigError if the file cannot be read or a block is malformed.
  static ModelLibrary FromFile(const std::string & yaml_path);

  /// \brief Look up a model by key.
  /// \throws ConfigError if \p model_name is not in the library.
  const RobotProfile & Get(const std::string & model_name) const;

  bool Contains(const std::string & model_name) const;

  /// \brief Model keys, sorted, for logging and error messages.
  std::vector<std::string> Names() const;

  std::size_t Size() const {return profiles_.size();}

  const std::string & SourcePath() const {return source_path_;}

private:
  std::map<std::string, RobotProfile> profiles_;
  std::string source_path_;
};

}  // namespace amr_core

#endif  // AMR_CORE__MODEL_LIBRARY_HPP_
