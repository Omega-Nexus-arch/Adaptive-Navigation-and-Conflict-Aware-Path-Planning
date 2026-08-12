// Copyright 2026 RSE Candidate
// Licensed under the Apache License, Version 2.0.
//
// Style: Google C++ Style Guide.

#ifndef AMR_SENSOR_BSP__VALIDATOR_HPP_
#define AMR_SENSOR_BSP__VALIDATOR_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace amr_sensor_bsp
{

/// \brief Verdict on a single sensor message.
enum class ValidationStatus : std::uint8_t
{
  kValid = 0,      ///< Passed everything. Forwarded unchanged.
  kDegraded = 1,   ///< A soft check failed. Forwarded, flagged, counted.
  kInvalid = 2     ///< A hard check failed. Dropped; never reaches navigation.
};

const char * ValidationStatusToString(ValidationStatus status);

/// \brief The outcome of running one message through a validator.
struct ValidationResult
{
  ValidationStatus status = ValidationStatus::kValid;
  /// Human-readable names of the checks that failed, e.g. "angular_velocity".
  /// Names, not codes: these end up in an operator's log.
  std::vector<std::string> faults;

  bool IsValid() const {return status == ValidationStatus::kValid;}
  bool ShouldForward() const {return status != ValidationStatus::kInvalid;}

  /// \brief Record a failure, promoting the overall status if needed.
  ///
  /// \param hard True for a fault that makes the message unusable, false for
  ///             one that makes it merely suspect.
  void AddFault(const std::string & name, bool hard)
  {
    faults.push_back(name);
    const ValidationStatus candidate =
      hard ? ValidationStatus::kInvalid : ValidationStatus::kDegraded;
    if (static_cast<std::uint8_t>(candidate) > static_cast<std::uint8_t>(status)) {
      status = candidate;
    }
  }
};

/// \brief Running counters for one validated stream.
struct ValidationStatistics
{
  std::uint64_t received = 0;
  std::uint64_t rejected = 0;
  std::uint64_t degraded = 0;
  double measured_rate_hz = 0.0;

  double RejectRatio() const
  {
    return received == 0 ? 0.0 : static_cast<double>(rejected) / static_cast<double>(received);
  }
};

/// \brief Base class for every Board Support Package validation routine.
///
/// ### What a BSP layer is doing here
///
/// The brief asks for a BSP-style validation routine in place of a classic
/// HAL, and the distinction is the point. A HAL abstracts *how* you talk to a
/// device. A BSP asserts what a device on this board is physically capable of,
/// and treats anything outside that envelope as a fault rather than as data.
///
/// Concretely: this layer sits between the drivers and the navigation stack.
/// Sensors publish to `<name>_raw`; validators republish to `<name>`; nothing
/// in the navigation stack subscribes to a raw topic. The gate is therefore
/// structural rather than a convention someone has to remember - a node cannot
/// accidentally consume unvalidated data, because the topic it subscribes to
/// only exists downstream of a validator.
///
/// ### Hard versus soft faults
///
/// A hard fault means the message cannot be used: a NaN where a distance
/// belongs, an all-zero orientation quaternion. Forwarding it would put
/// nonsense into a costmap.
///
/// A soft fault means the message is usable but the sensor is misbehaving: a
/// handful of out-of-range returns, an imager drifting towards saturation.
/// Dropping these would be worse than passing them on, because the robot would
/// go blind over a cosmetic defect. They are forwarded, counted, and reported.
///
/// The one asymmetry worth stating: an IMU angular velocity beyond the
/// chassis's physical capability is a *soft* fault. The brief requires a
/// warning, and a warning is right - the reading is almost certainly a
/// timestamp or scaling problem rather than the robot actually spinning at
/// that rate, and dropping IMU messages mid-manoeuvre would destabilise the
/// very localisation trying to recover from it.
class SensorValidator
{
public:
  explicit SensorValidator(std::string sensor_name)
  : sensor_name_(std::move(sensor_name)) {}

  virtual ~SensorValidator() = default;

  SensorValidator(const SensorValidator &) = delete;
  SensorValidator & operator=(const SensorValidator &) = delete;

  const std::string & SensorName() const {return sensor_name_;}
  const ValidationStatistics & Statistics() const {return statistics_;}

  void ResetStatistics() {statistics_ = ValidationStatistics{};}

protected:
  /// \brief Fold one verdict into the running counters.
  ///
  /// Subclasses call this from their own Validate() so every validator reports
  /// consistently without each one reimplementing the bookkeeping.
  void RecordResult(const ValidationResult & result)
  {
    ++statistics_.received;
    if (result.status == ValidationStatus::kInvalid) {
      ++statistics_.rejected;
    } else if (result.status == ValidationStatus::kDegraded) {
      ++statistics_.degraded;
    }
  }

  /// \brief Update the measured publish rate from a message timestamp.
  ///
  /// An exponential moving average rather than an instantaneous reciprocal:
  /// one late message on a loaded machine should not read as a rate collapse.
  void RecordArrival(double stamp_seconds)
  {
    if (has_previous_stamp_) {
      const double interval = stamp_seconds - previous_stamp_;
      if (interval > 1e-6) {
        const double instantaneous = 1.0 / interval;
        statistics_.measured_rate_hz = (statistics_.measured_rate_hz <= 0.0) ?
          instantaneous :
          0.9 * statistics_.measured_rate_hz + 0.1 * instantaneous;
      }
    }
    previous_stamp_ = stamp_seconds;
    has_previous_stamp_ = true;
  }

private:
  std::string sensor_name_;
  ValidationStatistics statistics_;
  double previous_stamp_ = 0.0;
  bool has_previous_stamp_ = false;
};

}  // namespace amr_sensor_bsp

#endif  // AMR_SENSOR_BSP__VALIDATOR_HPP_
