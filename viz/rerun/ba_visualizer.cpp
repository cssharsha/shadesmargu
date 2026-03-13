// =============================================================================
// BAVisualizer implementation — Rerun C++ SDK
// =============================================================================

#include "viz/rerun/ba_visualizer.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <rerun.hpp>
#include <vector>

namespace substral {
namespace rerun_viz {

// ---------------------------------------------------------------------------
// Pimpl — hides the rerun::RecordingStream from the header
// ---------------------------------------------------------------------------
struct BAVisualizer::Impl {
  std::unique_ptr<rerun::RecordingStream> rec;
};

BAVisualizer::~BAVisualizer() { delete impl_; }

bool BAVisualizer::init(const VizConfig& config) {
  if (!config.enabled()) return false;

  impl_ = new Impl();
  impl_->rec =
      std::make_unique<rerun::RecordingStream>("subastral_ba", "default");

  // Set up sink(s)
  if (!config.rrd_path.empty()) {
    impl_->rec->save(config.rrd_path).exit_on_failure();
    std::cout << "[viz] Saving recording to: " << config.rrd_path << std::endl;
  }

  if (!config.connect_addr.empty()) {
    impl_->rec->connect_grpc(config.connect_addr).exit_on_failure();
    std::cout << "[viz] Connecting to Rerun viewer at: " << config.connect_addr
              << std::endl;
  }

  return true;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Convert angle-axis vector to a 3x3 rotation matrix (row-major).
/// R[row][col] = R[row * 3 + col]
static void angleAxisToRotation(const double* aa, float* R) {
  double theta = std::sqrt(aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]);
  if (theta < 1e-12) {
    R[0] = 1;
    R[1] = 0;
    R[2] = 0;
    R[3] = 0;
    R[4] = 1;
    R[5] = 0;
    R[6] = 0;
    R[7] = 0;
    R[8] = 1;
    return;
  }
  double kx = aa[0] / theta, ky = aa[1] / theta, kz = aa[2] / theta;
  double c = std::cos(theta), s = std::sin(theta), v = 1.0 - c;

  // Row-major: R[row * 3 + col]
  R[0] = static_cast<float>(c + kx * kx * v);
  R[1] = static_cast<float>(kx * ky * v - kz * s);
  R[2] = static_cast<float>(kx * kz * v + ky * s);
  R[3] = static_cast<float>(kx * ky * v + kz * s);
  R[4] = static_cast<float>(c + ky * ky * v);
  R[5] = static_cast<float>(ky * kz * v - kx * s);
  R[6] = static_cast<float>(kx * kz * v - ky * s);
  R[7] = static_cast<float>(ky * kz * v + kx * s);
  R[8] = static_cast<float>(c + kz * kz * v);
}

/// Extract camera world position and viewing direction from BAL 9-param camera.
/// BAL convention: P_cam = R * P_world + t
/// Camera center in world: C = -R^T * t
/// Camera viewing direction in world: z_cam in world = R^T * [0,0,1]
///   (third column of R^T = third row of R)
static void cameraWorldPoseFromBAL(const double* cam, float* pos,
                                   float* view_dir) {
  float R[9];
  angleAxisToRotation(cam, R);  // cam[0..2] = angle-axis

  const double tx = cam[3], ty = cam[4], tz = cam[5];

  // R^T * t  (R is row-major, so R^T[i][j] = R[j][i])
  // pos = -R^T * t
  pos[0] = -static_cast<float>(R[0] * tx + R[3] * ty + R[6] * tz);
  pos[1] = -static_cast<float>(R[1] * tx + R[4] * ty + R[7] * tz);
  pos[2] = -static_cast<float>(R[2] * tx + R[5] * ty + R[8] * tz);

  // Viewing direction = R^T * [0,0,1] = third row of R (transposed = col)
  // R^T[i][2] = R[2][i] = R[2*3+i]
  view_dir[0] = R[6];
  view_dir[1] = R[7];
  view_dir[2] = R[8];
}

// ---------------------------------------------------------------------------
// Point cloud logging
// ---------------------------------------------------------------------------
void BAVisualizer::logPoints(const std::string& label, const double* points,
                             int num_points) {
  if (!impl_) return;

  std::vector<rerun::Position3D> positions(num_points);
  for (int i = 0; i < num_points; ++i) {
    positions[i] = rerun::Position3D(static_cast<float>(points[i * 3 + 0]),
                                     static_cast<float>(points[i * 3 + 1]),
                                     static_cast<float>(points[i * 3 + 2]));
  }

  // Distinct colors: initial = light blue, optimized = green
  rerun::Color color = (label == "optimized") ? rerun::Color(50, 220, 50)
                                              : rerun::Color(100, 160, 255);
  float radius = (label == "optimized") ? 0.03f : 0.02f;

  std::string entity = "world/points/" + label;

  // Use log_static so points are visible regardless of timeline position
  impl_->rec->log_static(
      entity,
      rerun::Points3D(positions).with_radii({radius}).with_colors({color}));
}

// ---------------------------------------------------------------------------
// Camera logging — positions as Points3D + viewing directions as Arrows3D
// ---------------------------------------------------------------------------
void BAVisualizer::logCameras(const std::string& label, const double* cameras,
                              int num_cameras) {
  if (!impl_) return;

  constexpr int CAM_STRIDE = 9;

  // Collect all camera positions and viewing directions in batch
  std::vector<rerun::Position3D> cam_positions(num_cameras);
  std::vector<rerun::Position3D> arrow_origins(num_cameras);
  std::vector<rerun::Vector3D> arrow_vectors(num_cameras);

  constexpr float kArrowLength = 2.0f;

  for (int i = 0; i < num_cameras; ++i) {
    const double* cam = cameras + i * CAM_STRIDE;

    float pos[3], view_dir[3];
    cameraWorldPoseFromBAL(cam, pos, view_dir);

    cam_positions[i] = rerun::Position3D(pos[0], pos[1], pos[2]);
    arrow_origins[i] = rerun::Position3D(pos[0], pos[1], pos[2]);
    arrow_vectors[i] =
        rerun::Vector3D(view_dir[0] * kArrowLength, view_dir[1] * kArrowLength,
                        view_dir[2] * kArrowLength);
  }

  // Distinct colors: initial = red, optimized = yellow
  rerun::Color color = (label == "optimized") ? rerun::Color(255, 220, 50)
                                              : rerun::Color(255, 80, 80);

  // Log camera positions as visible points
  std::string pos_entity = "world/cameras/" + label + "/positions";
  impl_->rec->log_static(
      pos_entity,
      rerun::Points3D(cam_positions).with_radii({0.15f}).with_colors({color}));

  // Log viewing directions as arrows
  std::string dir_entity = "world/cameras/" + label + "/directions";
  impl_->rec->log_static(dir_entity,
                         rerun::Arrows3D::from_vectors(arrow_vectors)
                             .with_origins(arrow_origins)
                             .with_colors({color}));
}

// ---------------------------------------------------------------------------
// Per-iteration solver metrics
// ---------------------------------------------------------------------------
void BAVisualizer::logIteration(int iteration, double cost, double rms,
                                double lambda) {
  if (!impl_) return;

  impl_->rec->set_time_sequence("iteration", iteration);

  impl_->rec->log("solver/cost", rerun::Scalars(static_cast<float>(cost)));
  impl_->rec->log("solver/rms", rerun::Scalars(static_cast<float>(rms)));
  impl_->rec->log("solver/lambda", rerun::Scalars(static_cast<float>(lambda)));
}

// ---------------------------------------------------------------------------
// Full state snapshot at a given iteration
// ---------------------------------------------------------------------------
void BAVisualizer::logState(int iteration, const double* cameras,
                            int num_cameras, const double* points,
                            int num_points) {
  if (!impl_) return;

  impl_->rec->set_time_sequence("iteration", iteration);

  // Log points with a fixed label (overwritten each iteration)
  std::vector<rerun::Position3D> positions(num_points);
  for (int i = 0; i < num_points; ++i) {
    positions[i] = rerun::Position3D(static_cast<float>(points[i * 3 + 0]),
                                     static_cast<float>(points[i * 3 + 1]),
                                     static_cast<float>(points[i * 3 + 2]));
  }
  impl_->rec->log("world/points/current",
                  rerun::Points3D(positions).with_radii({0.02f}).with_colors(
                      {rerun::Color(180, 180, 180)}));

  // Log cameras
  logCameras("current", cameras, num_cameras);
}

}  // namespace rerun_viz
}  // namespace substral
