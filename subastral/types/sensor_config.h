#pragma once

namespace substral {

/// Available sensor modalities, auto-detected from loaded data.
/// Used to select initialization and tracking strategies.
struct SensorConfig {
  bool has_depth = false;      // Depth images available (RGB-D sensor)
  bool has_imu = false;        // Full 6-axis IMU (accel+gyro) available
  bool has_accel_only = false;  // Accelerometer only (no gyro)

  /// Priority-ordered initialization mode.
  /// depth > imu > monocular (pure vision).
  enum class InitMode { DEPTH, IMU, MONOCULAR };

  InitMode bestInitMode() const {
    if (has_depth) return InitMode::DEPTH;
    if (has_imu) return InitMode::IMU;
    return InitMode::MONOCULAR;
  }

  const char* initModeName() const {
    switch (bestInitMode()) {
      case InitMode::DEPTH:
        return "Depth (RGB-D)";
      case InitMode::IMU:
        return "IMU-aided monocular";
      case InitMode::MONOCULAR:
        return "Pure monocular";
    }
    return "Unknown";
  }
};

}  // namespace substral
