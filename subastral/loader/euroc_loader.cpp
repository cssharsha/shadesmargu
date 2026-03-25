#include "subastral/loader/euroc_loader.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

namespace substral {
namespace loader {

bool EuRoCLoader::load(const std::string& sequence_dir) {
  dataset_.base_path = sequence_dir;

  // Detect intrinsics from directory name or sensor.yaml
  detectIntrinsics(sequence_dir);

  // Parse camera images (cam0)
  std::string cam0_csv = sequence_dir + "/mav0/cam0/data.csv";
  std::string cam0_data = "mav0/cam0/data";
  if (!parseImageCSV(cam0_csv, dataset_.rgb_frames, cam0_data)) {
    std::cerr << "Failed to parse " << cam0_csv << std::endl;
    return false;
  }

  // Parse IMU data
  std::string imu_csv = sequence_dir + "/mav0/imu0/data.csv";
  if (parseIMU(imu_csv, dataset_.imu_data) && !dataset_.imu_data.empty()) {
    // Check if we have gyroscope data (non-zero gx/gy/gz)
    bool has_gyro = false;
    for (const auto& m : dataset_.imu_data) {
      if (m.gx != 0.0 || m.gy != 0.0 || m.gz != 0.0) {
        has_gyro = true;
        break;
      }
    }
    dataset_.has_full_imu = has_gyro;
  }

  // Parse ground truth (optional — not all sequences have full GT)
  std::string gt_csv =
      sequence_dir + "/mav0/state_groundtruth_estimate0/data.csv";
  if (!parseGroundTruth(gt_csv, dataset_.ground_truth)) {
    // Try alternative GT location used by some EuRoC sequences
    std::string gt_alt = sequence_dir + "/mav0/mocap0/data.csv";
    parseGroundTruth(gt_alt, dataset_.ground_truth);
  }

  return !dataset_.rgb_frames.empty();
}

void EuRoCLoader::printStats() const {
  std::cout << "EuRoC Dataset: " << dataset_.base_path << "\n"
            << "  RGB frames: " << dataset_.rgb_frames.size() << "\n"
            << "  Ground truth poses: " << dataset_.ground_truth.size() << "\n"
            << "  IMU measurements: " << dataset_.imu_data.size()
            << (dataset_.has_full_imu ? " (6-axis)" : " (accel-only)")
            << "\n"
            << "  Intrinsics: fx=" << dataset_.intrinsics.fx
            << " fy=" << dataset_.intrinsics.fy
            << " cx=" << dataset_.intrinsics.cx
            << " cy=" << dataset_.intrinsics.cy << "\n";
  if (!dataset_.intrinsics.isUndistorted()) {
    std::cout << "  Distortion: k1=" << dataset_.intrinsics.k1
              << " k2=" << dataset_.intrinsics.k2
              << " p1=" << dataset_.intrinsics.p1
              << " p2=" << dataset_.intrinsics.p2 << "\n";
  } else {
    std::cout << "  Distortion: none\n";
  }
  if (!dataset_.rgb_frames.empty()) {
    double duration = dataset_.rgb_frames.back().timestamp -
                      dataset_.rgb_frames.front().timestamp;
    std::cout << "  Duration: " << duration << "s\n"
              << "  Avg FPS: " << dataset_.rgb_frames.size() / duration << "\n";
  }
}

bool EuRoCLoader::parseImageCSV(const std::string& filepath,
                                std::vector<ImageEntry>& entries,
                                const std::string& data_dir) {
  std::ifstream file(filepath);
  if (!file.is_open()) return false;

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;

    // Format: timestamp[ns],filename
    std::istringstream iss(line);
    std::string ts_str, filename;

    if (!std::getline(iss, ts_str, ',')) continue;
    if (!std::getline(iss, filename, ',')) continue;

    // Trim whitespace from filename
    while (!filename.empty() && filename.front() == ' ') filename.erase(0, 1);

    ImageEntry entry;
    entry.timestamp = std::stod(ts_str) * 1e-9;  // ns → seconds
    entry.relative_path = data_dir + "/" + filename;
    entries.push_back(std::move(entry));
  }

  return !entries.empty();
}

bool EuRoCLoader::parseIMU(const std::string& filepath,
                           std::vector<ImuMeasurement>& measurements) {
  std::ifstream file(filepath);
  if (!file.is_open()) return false;

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;

    // Format: timestamp[ns],gx,gy,gz,ax,ay,az
    // Replace commas with spaces for easier parsing
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream iss(line);

    double ts_ns;
    ImuMeasurement m{};
    if (iss >> ts_ns >> m.gx >> m.gy >> m.gz >> m.ax >> m.ay >> m.az) {
      m.timestamp = ts_ns * 1e-9;  // ns → seconds
      measurements.push_back(m);
    }
  }

  return !measurements.empty();
}

bool EuRoCLoader::parseGroundTruth(const std::string& filepath,
                                   std::vector<GroundTruthPose>& poses) {
  std::ifstream file(filepath);
  if (!file.is_open()) return false;

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;

    // Format: timestamp[ns],tx,ty,tz,qw,qx,qy,qz[,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz]
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream iss(line);

    double ts_ns;
    GroundTruthPose pose;
    double qw;
    if (iss >> ts_ns >> pose.tx >> pose.ty >> pose.tz >>
        qw >> pose.qx >> pose.qy >> pose.qz) {
      pose.timestamp = ts_ns * 1e-9;  // ns → seconds
      pose.qw = qw;  // EuRoC: w-first in file, we store w-last in struct
      poses.push_back(pose);
    }
  }

  return !poses.empty();
}

void EuRoCLoader::detectIntrinsics(const std::string& sequence_dir) {
  // TUM-VI exported 512x512 datasets use a pinhole model.
  // The calibration is embedded in sensor.yaml. For the 512_16 exported
  // versions, the images are undistorted and the intrinsics are:
  //
  // From TUM-VI calibration (512x512, cam0):
  //   fx=190.97847715128717, fy=190.9733070521226
  //   cx=254.93170605935475, cy=256.8974428996504
  //
  // These are the standard values for the exported 512x512 resolution.

  if (sequence_dir.find("512_16") != std::string::npos ||
      sequence_dir.find("512_8") != std::string::npos) {
    // TUM-VI 512x512 exported (undistorted pinhole)
    dataset_.intrinsics.fx = 190.978;
    dataset_.intrinsics.fy = 190.973;
    dataset_.intrinsics.cx = 254.932;
    dataset_.intrinsics.cy = 256.897;
    dataset_.intrinsics.k1 = 0.0;
    dataset_.intrinsics.k2 = 0.0;
    dataset_.intrinsics.p1 = 0.0;
    dataset_.intrinsics.p2 = 0.0;
    dataset_.intrinsics.k3 = 0.0;
  } else if (sequence_dir.find("1024_16") != std::string::npos ||
             sequence_dir.find("1024_8") != std::string::npos) {
    // TUM-VI 1024x1024 exported (undistorted pinhole)
    // Scaled by 2x from 512
    dataset_.intrinsics.fx = 381.956;
    dataset_.intrinsics.fy = 381.947;
    dataset_.intrinsics.cx = 509.864;
    dataset_.intrinsics.cy = 513.795;
    dataset_.intrinsics.k1 = 0.0;
    dataset_.intrinsics.k2 = 0.0;
    dataset_.intrinsics.p1 = 0.0;
    dataset_.intrinsics.p2 = 0.0;
    dataset_.intrinsics.k3 = 0.0;
  } else {
    // Default: EuRoC MAV cam0 intrinsics (MH/V1/V2 sequences)
    dataset_.intrinsics.fx = 458.654;
    dataset_.intrinsics.fy = 457.296;
    dataset_.intrinsics.cx = 367.215;
    dataset_.intrinsics.cy = 248.375;
    dataset_.intrinsics.k1 = -0.28340811;
    dataset_.intrinsics.k2 = 0.07395907;
    dataset_.intrinsics.p1 = 0.00019359;
    dataset_.intrinsics.p2 = 1.76187114e-05;
    dataset_.intrinsics.k3 = 0.0;
  }
}

}  // namespace loader
}  // namespace substral
