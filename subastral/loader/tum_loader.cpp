#include "subastral/loader/tum_loader.h"

#include <fstream>
#include <iostream>
#include <sstream>

namespace substral {
namespace loader {

bool TUMLoader::load(const std::string& sequence_dir) {
  dataset_.base_path = sequence_dir;

  // Detect sensor from directory name
  dataset_.sensor_id = detectSensor(sequence_dir);
  if (dataset_.sensor_id == 0) {
    std::cerr << "Warning: Could not detect freiburg sensor from path: "
              << sequence_dir << "\n"
              << "  Using default intrinsics (freiburg1).\n";
    dataset_.sensor_id = 1;
  }
  setIntrinsics(dataset_.sensor_id);

  // Parse rgb.txt
  std::string rgb_file = sequence_dir + "/rgb.txt";
  if (!parseImageList(rgb_file, dataset_.rgb_frames)) {
    std::cerr << "Failed to parse " << rgb_file << std::endl;
    return false;
  }

  // Parse depth.txt (optional — not all sequences have depth)
  std::string depth_file = sequence_dir + "/depth.txt";
  if (!parseImageList(depth_file, dataset_.depth_frames)) {
    dataset_.depth_frames.clear();
  }

  // Parse groundtruth.txt (optional — some sequences may not have it)
  std::string gt_file = sequence_dir + "/groundtruth.txt";
  if (!parseGroundTruth(gt_file, dataset_.ground_truth)) {
    std::cerr << "Warning: No ground truth found at " << gt_file << std::endl;
  }

  // Parse accelerometer.txt (optional — TUM RGB-D only, 3-axis)
  std::string accel_file = sequence_dir + "/accelerometer.txt";
  parseAccelerometer(accel_file, dataset_.imu_data);

  return !dataset_.rgb_frames.empty();
}

void TUMLoader::printStats() const {
  std::cout << "TUM Dataset: " << dataset_.base_path << "\n"
            << "  Sensor: freiburg" << dataset_.sensor_id << "\n"
            << "  RGB frames: " << dataset_.rgb_frames.size() << "\n"
            << "  Depth frames: " << dataset_.depth_frames.size() << "\n"
            << "  Ground truth poses: " << dataset_.ground_truth.size() << "\n"
            << "  IMU measurements: " << dataset_.imu_data.size()
            << (dataset_.has_full_imu ? " (6-axis)" : dataset_.imu_data.empty() ? "" : " (accel-only)") << "\n"
            << "  Intrinsics: fx=" << dataset_.intrinsics.fx
            << " fy=" << dataset_.intrinsics.fy
            << " cx=" << dataset_.intrinsics.cx
            << " cy=" << dataset_.intrinsics.cy << "\n";
  if (!dataset_.intrinsics.isUndistorted()) {
    std::cout << "  Distortion: k1=" << dataset_.intrinsics.k1
              << " k2=" << dataset_.intrinsics.k2
              << " p1=" << dataset_.intrinsics.p1
              << " p2=" << dataset_.intrinsics.p2
              << " k3=" << dataset_.intrinsics.k3 << "\n";
  } else {
    std::cout << "  Distortion: none (pre-undistorted)\n";
  }
  if (!dataset_.rgb_frames.empty()) {
    double duration = dataset_.rgb_frames.back().timestamp -
                      dataset_.rgb_frames.front().timestamp;
    std::cout << "  Duration: " << duration << "s\n"
              << "  Avg FPS: " << dataset_.rgb_frames.size() / duration << "\n";
  }
}

bool TUMLoader::parseImageList(const std::string& filepath,
                               std::vector<ImageEntry>& entries) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    // Skip comments and empty lines
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);
    ImageEntry entry;
    if (iss >> entry.timestamp >> entry.relative_path) {
      entries.push_back(std::move(entry));
    }
  }

  return true;
}

bool TUMLoader::parseGroundTruth(const std::string& filepath,
                                 std::vector<GroundTruthPose>& poses) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    // Skip comments and empty lines
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);
    GroundTruthPose pose;
    if (iss >> pose.timestamp >> pose.tx >> pose.ty >> pose.tz >> pose.qx >>
        pose.qy >> pose.qz >> pose.qw) {
      poses.push_back(pose);
    }
  }

  return true;
}

int TUMLoader::detectSensor(const std::string& path) {
  // Look for "freiburg1", "freiburg2", or "freiburg3" in the path
  if (path.find("freiburg1") != std::string::npos) return 1;
  if (path.find("freiburg2") != std::string::npos) return 2;
  if (path.find("freiburg3") != std::string::npos) return 3;
  return 0;
}

void TUMLoader::setIntrinsics(int sensor_id) {
  // TUM RGB-D benchmark intrinsics.
  //
  // The TUM page recommends using the ROS default parameter set
  // (fx=fy=525, cx=319.5, cy=239.5, no distortion) for all sensors,
  // because the OpenNI driver already applies factory calibration
  // (including undistortion and depth-to-RGB registration) to the
  // images. Applying additional undistortion on top would be incorrect.
  //
  // See: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats
  //   "We recommend to use the ROS default parameter set (i.e., without
  //    undistortion), as undistortion of the pre-registered depth images
  //    is not trivial."
  //
  // The per-sensor calibrated values are preserved in comments below
  // for reference, but should NOT be used for undistortion.
  (void)sensor_id;  // Same defaults for all sensors

  dataset_.intrinsics.fx = 525.0;
  dataset_.intrinsics.fy = 525.0;
  dataset_.intrinsics.cx = 319.5;
  dataset_.intrinsics.cy = 239.5;
  // No distortion — images are already corrected by the OpenNI driver
  dataset_.intrinsics.k1 = 0.0;
  dataset_.intrinsics.k2 = 0.0;
  dataset_.intrinsics.p1 = 0.0;
  dataset_.intrinsics.p2 = 0.0;
  dataset_.intrinsics.k3 = 0.0;

  // Per-sensor calibrated values (for reference only):
  //   fr1: fx=517.3 fy=516.5 cx=318.6 cy=255.3 d0=0.2624 d1=-0.9531
  //   fr2: fx=520.9 fy=521.0 cx=325.1 cy=249.7 d0=0.2312 d1=-0.7849
  //   fr3: fx=535.4 fy=539.2 cx=320.1 cy=247.6 (already undistorted)
}

bool TUMLoader::parseAccelerometer(
    const std::string& filepath,
    std::vector<ImuMeasurement>& measurements) {
  std::ifstream file(filepath);
  if (!file.is_open()) return false;

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;

    std::istringstream iss(line);
    ImuMeasurement m{};
    if (iss >> m.timestamp >> m.ax >> m.ay >> m.az) {
      // gx, gy, gz remain zero (TUM RGB-D has no gyroscope)
      measurements.push_back(m);
    }
  }
  return !measurements.empty();
}

}  // namespace loader
}  // namespace substral
