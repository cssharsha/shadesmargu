// =============================================================================
// PGVisualizer implementation — Rerun C++ SDK
// =============================================================================

#include "viz/rerun/pg_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <rerun.hpp>
#include <vector>

#include "subastral/backend/lie/se3.hpp"

namespace substral {
namespace rerun_viz {

// ---------------------------------------------------------------------------
// Pimpl — hides the rerun::RecordingStream from the header
// ---------------------------------------------------------------------------
struct PGVisualizer::Impl {
  std::unique_ptr<rerun::RecordingStream> rec;
};

PGVisualizer::~PGVisualizer() { delete impl_; }

bool PGVisualizer::init(const VizConfig& config) {
  if (!config.enabled()) return false;

  impl_ = new Impl();
  impl_->rec =
      std::make_unique<rerun::RecordingStream>("subastral_pg", "default");

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
// Helper: collect sorted pose positions from problem
// ---------------------------------------------------------------------------
static std::vector<std::array<float, 3>> collectPosePositions(
    const backend::PoseGraphProblem& problem) {
  auto sorted_ids = problem.sorted_vertex_ids();
  std::vector<std::array<float, 3>> positions(sorted_ids.size());

  // Build id->pose lookup
  std::map<int, const backend::Pose*> id_to_pose;
  for (const auto& p : problem.poses) {
    id_to_pose[p->get_id()] = p.get();
  }

  for (size_t i = 0; i < sorted_ids.size(); ++i) {
    const auto* pose = id_to_pose.at(sorted_ids[i]);
    positions[i] = {static_cast<float>(pose->x()),
                    static_cast<float>(pose->y()),
                    static_cast<float>(pose->z())};
  }
  return positions;
}

// ---------------------------------------------------------------------------
// Pose logging
// ---------------------------------------------------------------------------
void PGVisualizer::logPoses(const std::string& label,
                            const backend::PoseGraphProblem& problem) {
  if (!impl_) return;

  auto positions = collectPosePositions(problem);

  std::vector<rerun::Position3D> rerun_positions(positions.size());
  for (size_t i = 0; i < positions.size(); ++i) {
    rerun_positions[i] =
        rerun::Position3D(positions[i][0], positions[i][1], positions[i][2]);
  }

  rerun::Color color = (label == "optimized") ? rerun::Color(50, 220, 50)
                                              : rerun::Color(100, 160, 255);
  float radius = (label == "optimized") ? 0.08f : 0.05f;

  std::string entity = "world/poses/" + label;
  impl_->rec->log_static(entity, rerun::Points3D(rerun_positions)
                                     .with_radii({radius})
                                     .with_colors({color}));
}

// ---------------------------------------------------------------------------
// Helper: Compute per-edge residual norm on CPU
// ---------------------------------------------------------------------------
//
// Residual: e = log(T_meas^{-1} · T_i^{-1} · T_j) ∈ ℝ⁶
// Returns ||e|| (the se(3) norm).
//
static double computeEdgeResidualNorm(const backend::Pose& pose_i,
                                      const backend::Pose& pose_j,
                                      const backend::PoseEdge& edge) {
  using namespace substral::backend::lie;

  // Build 4x4 matrices
  auto quatToMatrix = [](double x, double y, double z, double qx, double qy,
                         double qz, double qw) -> Eigen::Matrix4d {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    // Quaternion to rotation
    double s = 2.0;
    T(0, 0) = 1.0 - s * (qy * qy + qz * qz);
    T(0, 1) = s * (qx * qy - qz * qw);
    T(0, 2) = s * (qx * qz + qy * qw);
    T(1, 0) = s * (qx * qy + qz * qw);
    T(1, 1) = 1.0 - s * (qx * qx + qz * qz);
    T(1, 2) = s * (qy * qz - qx * qw);
    T(2, 0) = s * (qx * qz - qy * qw);
    T(2, 1) = s * (qy * qz + qx * qw);
    T(2, 2) = 1.0 - s * (qx * qx + qy * qy);
    T(0, 3) = x;
    T(1, 3) = y;
    T(2, 3) = z;
    return T;
  };

  Eigen::Matrix4d Ti =
      quatToMatrix(pose_i.x(), pose_i.y(), pose_i.z(), pose_i.qx(), pose_i.qy(),
                   pose_i.qz(), pose_i.qw());
  Eigen::Matrix4d Tj =
      quatToMatrix(pose_j.x(), pose_j.y(), pose_j.z(), pose_j.qx(), pose_j.qy(),
                   pose_j.qz(), pose_j.qw());

  const double* m = edge.measurement_data();
  Eigen::Matrix4d Tm = quatToMatrix(m[0], m[1], m[2], m[3], m[4], m[5], m[6]);

  // E = T_meas^{-1} · T_i^{-1} · T_j
  Eigen::Matrix4d E = inverseSE3(Tm) * inverseSE3(Ti) * Tj;
  Eigen::Matrix<double, 6, 1> e = logSE3(E);
  return e.norm();
}

// ---------------------------------------------------------------------------
// Helper: map a normalized value [0,1] to a color via a ramp
// ---------------------------------------------------------------------------
//
// odom_ramp:  low error = cool blue (40,80,180), high = bright yellow
// (255,240,60) loop_ramp:  low error = green (40,200,40), high = bright red
// (255,40,40)
//
// We use a sqrt mapping to spread out the low-error values and compress
// the high-error tail, giving better visual contrast across the range.
//
static rerun::Color odomColorRamp(float t) {
  t = std::max(0.0f, std::min(1.0f, t));
  t = std::sqrt(t);  // spread low values for better contrast
  uint8_t r = static_cast<uint8_t>(40 + t * 215);
  uint8_t g = static_cast<uint8_t>(80 + t * 160);
  uint8_t b = static_cast<uint8_t>(180 - t * 120);
  return rerun::Color(r, g, b);
}

static rerun::Color loopColorRamp(float t) {
  t = std::max(0.0f, std::min(1.0f, t));
  t = std::sqrt(t);  // spread low values for better contrast
  uint8_t r = static_cast<uint8_t>(40 + t * 215);
  uint8_t g = static_cast<uint8_t>(200 - t * 160);
  uint8_t b = static_cast<uint8_t>(40);
  return rerun::Color(r, g, b);
}

// ---------------------------------------------------------------------------
// Edge logging — separate lines per edge, colored by residual error
// ---------------------------------------------------------------------------
void PGVisualizer::logEdges(const std::string& label,
                            const backend::PoseGraphProblem& problem) {
  if (!impl_) return;

  // Build id->pose lookup
  std::map<int, const backend::Pose*> id_to_pose;
  for (const auto& p : problem.poses) {
    id_to_pose[p->get_id()] = p.get();
  }

  // Compute per-edge errors and collect geometry
  struct EdgeInfo {
    rerun::Position3D p1, p2;
    double error;
    bool is_loop;
  };
  std::vector<EdgeInfo> all_edges;
  all_edges.reserve(problem.edges.size());

  double max_odom_err = 0.0, max_loop_err = 0.0;

  for (const auto& edge : problem.edges) {
    const auto* pi = id_to_pose.at(edge->get_from_id());
    const auto* pj = id_to_pose.at(edge->get_to_id());

    double err = computeEdgeResidualNorm(*pi, *pj, *edge);
    bool is_loop = edge->is_loop_closure();

    if (is_loop) {
      max_loop_err = std::max(max_loop_err, err);
    } else {
      max_odom_err = std::max(max_odom_err, err);
    }

    all_edges.push_back({rerun::Position3D(static_cast<float>(pi->x()),
                                           static_cast<float>(pi->y()),
                                           static_cast<float>(pi->z())),
                         rerun::Position3D(static_cast<float>(pj->x()),
                                           static_cast<float>(pj->y()),
                                           static_cast<float>(pj->z())),
                         err, is_loop});
  }

  // Avoid division by zero
  if (max_odom_err < 1e-12) max_odom_err = 1.0;
  if (max_loop_err < 1e-12) max_loop_err = 1.0;

  // Separate into odometry and loop closure edge lists
  struct StripData {
    std::vector<rerun::LineStrip3D> strips;
    std::vector<rerun::Color> colors;
    std::vector<rerun::components::Text> labels;
  };
  StripData odom, loop;

  // Format helper
  auto fmtErr = [](double err) -> std::string {
    char buf[64];
    if (err < 0.001)
      snprintf(buf, sizeof(buf), "%.2e", err);
    else if (err < 10.0)
      snprintf(buf, sizeof(buf), "%.4f", err);
    else
      snprintf(buf, sizeof(buf), "%.2f", err);
    return std::string("e=") + buf;
  };

  for (const auto& ei : all_edges) {
    std::vector<rerun::Position3D> pts = {ei.p1, ei.p2};

    if (ei.is_loop) {
      float t = static_cast<float>(ei.error / max_loop_err);
      loop.strips.push_back(rerun::LineStrip3D(pts));
      loop.colors.push_back(loopColorRamp(t));
      loop.labels.push_back(rerun::components::Text(fmtErr(ei.error)));
    } else {
      float t = static_cast<float>(ei.error / max_odom_err);
      odom.strips.push_back(rerun::LineStrip3D(pts));
      odom.colors.push_back(odomColorRamp(t));
      odom.labels.push_back(rerun::components::Text(fmtErr(ei.error)));
    }
  }

  // Log odometry edges — per-strip color and label
  if (!odom.strips.empty()) {
    std::string entity = "world/edges/" + label + "/odometry";
    impl_->rec->log_static(entity, rerun::LineStrips3D(odom.strips)
                                       .with_colors(odom.colors)
                                       .with_labels(odom.labels)
                                       .with_radii({0.02f}));
  }

  // Log loop closure edges — per-strip color and label
  if (!loop.strips.empty()) {
    std::string entity = "world/edges/" + label + "/loop_closures";
    impl_->rec->log_static(entity, rerun::LineStrips3D(loop.strips)
                                       .with_colors(loop.colors)
                                       .with_labels(loop.labels)
                                       .with_radii({0.02f}));
  }

  // Log summary statistics
  double total_err = 0.0;
  for (const auto& ei : all_edges) total_err += ei.error;
  double mean_err = all_edges.empty() ? 0.0 : total_err / all_edges.size();

  std::cout << "[viz] " << label << " edge errors:"
            << " n_odom=" << odom.strips.size()
            << " n_loop=" << loop.strips.size() << " mean=" << mean_err
            << " max_odom=" << max_odom_err << " max_loop=" << max_loop_err
            << std::endl;
}

// ---------------------------------------------------------------------------
// Trajectory logging — line strip through poses in ID order
// ---------------------------------------------------------------------------
void PGVisualizer::logTrajectory(const std::string& label,
                                 const backend::PoseGraphProblem& problem) {
  if (!impl_) return;

  auto positions = collectPosePositions(problem);

  std::vector<rerun::Position3D> traj_points(positions.size());
  for (size_t i = 0; i < positions.size(); ++i) {
    traj_points[i] =
        rerun::Position3D(positions[i][0], positions[i][1], positions[i][2]);
  }

  rerun::Color color = (label == "optimized") ? rerun::Color(50, 220, 50)
                                              : rerun::Color(100, 160, 255);

  std::string entity = "world/trajectory/" + label;
  impl_->rec->log_static(
      entity, rerun::LineStrips3D(rerun::Collection<rerun::LineStrip3D>{
                                      rerun::LineStrip3D(traj_points)})
                  .with_colors({color})
                  .with_radii({0.03f}));
}

// ---------------------------------------------------------------------------
// Per-iteration solver metrics
// ---------------------------------------------------------------------------
void PGVisualizer::logIteration(int iteration, double cost, double lambda) {
  if (!impl_) return;

  impl_->rec->set_time_sequence("iteration", iteration);
  impl_->rec->log("solver/cost", rerun::Scalars(static_cast<float>(cost)));
  impl_->rec->log("solver/lambda", rerun::Scalars(static_cast<float>(lambda)));
}

// ---------------------------------------------------------------------------
// Full state snapshot at a given iteration
// ---------------------------------------------------------------------------
void PGVisualizer::logState(int iteration,
                            const backend::PoseGraphProblem& problem,
                            double cost, double lambda) {
  if (!impl_) return;

  impl_->rec->set_time_sequence("iteration", iteration);

  // Log current poses
  auto positions = collectPosePositions(problem);
  std::vector<rerun::Position3D> rerun_positions(positions.size());
  for (size_t i = 0; i < positions.size(); ++i) {
    rerun_positions[i] =
        rerun::Position3D(positions[i][0], positions[i][1], positions[i][2]);
  }

  impl_->rec->log("world/poses/current",
                  rerun::Points3D(rerun_positions)
                      .with_radii({0.06f})
                      .with_colors({rerun::Color(255, 220, 50)}));

  // Log current trajectory
  std::vector<rerun::Position3D> traj_points(positions.size());
  for (size_t i = 0; i < positions.size(); ++i) {
    traj_points[i] =
        rerun::Position3D(positions[i][0], positions[i][1], positions[i][2]);
  }
  impl_->rec->log("world/trajectory/current",
                  rerun::LineStrips3D(rerun::Collection<rerun::LineStrip3D>{
                                          rerun::LineStrip3D(traj_points)})
                      .with_colors({rerun::Color(255, 220, 50)})
                      .with_radii({0.03f}));

  // Log metrics
  impl_->rec->log("solver/cost", rerun::Scalars(static_cast<float>(cost)));
  impl_->rec->log("solver/lambda", rerun::Scalars(static_cast<float>(lambda)));
}

}  // namespace rerun_viz
}  // namespace substral
