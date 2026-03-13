#include "viz/opencv/ba_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace substral {
namespace viz {

// =============================================================================
// BASnapshot
// =============================================================================

BASnapshot BASnapshot::capture(const backend::BAProblem& problem) {
  BASnapshot snap;
  snap.num_cameras = static_cast<int>(problem.cameras.size());
  snap.num_points = static_cast<int>(problem.points.size());
  const auto& mem = *problem.memory_map;
  snap.cameras = mem.observers;
  snap.points = mem.scene_points;
  return snap;
}

// =============================================================================
// Internal helpers
// =============================================================================
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// ---- Basic 3D vector math ----

struct Vec3 {
  double x, y, z;
  Vec3() : x(0), y(0), z(0) {}
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3& o) const {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
  double norm() const { return std::sqrt(x * x + y * y + z * z); }
  Vec3 normalized() const {
    double n = norm();
    return n > 1e-15 ? Vec3{x / n, y / n, z / n} : Vec3{0, 0, 0};
  }
};

// ---- 3x3 matrix (row-major) stored as 3 row-vectors ----

struct Mat3 {
  Vec3 row[3];

  // Matrix-vector product: M * v
  Vec3 mul(const Vec3& v) const {
    return {row[0].dot(v), row[1].dot(v), row[2].dot(v)};
  }
};

// ---- Rodrigues: angle-axis → rotation matrix ----

Mat3 angleAxisToRotation(const double* aa) {
  double ax = aa[0], ay = aa[1], az = aa[2];
  double theta2 = ax * ax + ay * ay + az * az;
  double theta = std::sqrt(theta2);

  double st, ct;
  if (theta < 1e-10) {
    st = 1.0;
    ct = 0.5;
  } else {
    st = std::sin(theta) / theta;
    ct = (1.0 - std::cos(theta)) / theta2;
  }

  Mat3 R;
  R.row[0] = {1.0 - ct * (ay * ay + az * az), ct * ax * ay - st * az,
              ct * ax * az + st * ay};
  R.row[1] = {ct * ax * ay + st * az, 1.0 - ct * (ax * ax + az * az),
              ct * ay * az - st * ax};
  R.row[2] = {ct * ax * az - st * ay, ct * ay * az + st * ax,
              1.0 - ct * (ax * ax + ay * ay)};
  return R;
}

// Transpose of a Mat3.
Mat3 transpose(const Mat3& M) {
  Mat3 T;
  T.row[0] = {M.row[0].x, M.row[1].x, M.row[2].x};
  T.row[1] = {M.row[0].y, M.row[1].y, M.row[2].y};
  T.row[2] = {M.row[0].z, M.row[1].z, M.row[2].z};
  return T;
}

// BAL: camera sees point P as R*P + t.  Camera center = -R^T * t.
Vec3 cameraCenter(const double* cam) {
  Mat3 R = angleAxisToRotation(cam);
  Vec3 t(cam[3], cam[4], cam[5]);
  Mat3 Rt = transpose(R);
  return Rt.mul(t) * -1.0;
}

// Camera's forward direction in world frame: -R^T * e_z.
Vec3 cameraForward(const double* cam) {
  Mat3 R = angleAxisToRotation(cam);
  Mat3 Rt = transpose(R);
  return Rt.mul({0, 0, 1}) * -1.0;
}

// Camera's up direction in world frame: -R^T * e_y  (BAL has -y up in cam).
Vec3 cameraUp(const double* cam) {
  Mat3 R = angleAxisToRotation(cam);
  Mat3 Rt = transpose(R);
  return Rt.mul({0, 1, 0}) * -1.0;
}

// Camera's right direction in world frame: R^T * e_x.
Vec3 cameraRight(const double* cam) {
  Mat3 R = angleAxisToRotation(cam);
  Mat3 Rt = transpose(R);
  return Rt.mul({1, 0, 0});
}

// =============================================================================
// Virtual orbit camera for rendering
// =============================================================================
//
// The orbit camera is placed at a distance from the scene centroid, looking
// inward.  It uses a standard look-at construction:
//
//   eye    = centroid + distance * direction(azimuth, elevation)
//   target = centroid
//   up     = world Y
//
// The view matrix transforms world points into camera-local coordinates
// (right-handed: +X right, +Y up, -Z forward).  Then a pinhole projection
// maps to pixel coordinates.

struct OrbitCamera {
  Vec3 eye;
  Vec3 target;
  Vec3 up;

  // View basis vectors (world space).
  Vec3 forward;  // unit, from eye toward target
  Vec3 right;    // unit, camera right
  Vec3 cam_up;   // unit, camera up (not necessarily world Y)

  // Perspective intrinsics.
  double focal_px;  // focal length in pixels (derived from fov + image height)
  double cx, cy;    // principal point (panel center)

  // Panel offset in the full image.
  int panel_x0;

  // Initialize from orbit parameters.
  void init(const Vec3& centroid, double distance, double azimuth_rad,
            double elevation_rad, double fov_rad, int panel_w, int panel_h,
            int px0) {
    // Spherical coordinates: azimuth from +X axis in XZ plane, elevation up.
    double ce = std::cos(elevation_rad);
    double se = std::sin(elevation_rad);
    double ca = std::cos(azimuth_rad);
    double sa = std::sin(azimuth_rad);

    Vec3 dir(ce * ca, se, ce * sa);
    eye = centroid + dir * distance;
    target = centroid;
    up = {0, 1, 0};

    forward = (target - eye).normalized();
    right = forward.cross(up).normalized();
    // Handle degenerate case where forward ≈ up.
    if (right.norm() < 1e-10) {
      right = {1, 0, 0};
    }
    cam_up = right.cross(forward).normalized();

    // Focal length from vertical FOV.
    focal_px = (panel_h * 0.5) / std::tan(fov_rad * 0.5);
    cx = panel_w * 0.5;
    cy = panel_h * 0.5;
    panel_x0 = px0;
  }

  // Project a world point to pixel coordinates.
  // Returns false if the point is behind the camera.
  bool project(const Vec3& pw, double& px, double& py, double& depth) const {
    Vec3 d = pw - eye;
    // Camera-local coordinates: x=right, y=up, z=-forward.
    double cz = d.dot(forward);   // depth along viewing direction
    if (cz < 0.01) return false;  // behind camera or too close

    double cx_local = d.dot(right);
    double cy_local = d.dot(cam_up);

    px = panel_x0 + cx + focal_px * (cx_local / cz);
    py = cy - focal_px * (cy_local / cz);
    depth = cz;
    return true;
  }
};

// =============================================================================
// Scene analysis: compute centroid, robust bounding radius
// =============================================================================

struct SceneStats {
  Vec3 centroid;
  double radius;  // Radius enclosing ~98% of the scene
};

SceneStats analyzeScene(const BASnapshot& before, const BASnapshot& after) {
  // Strategy: compute a robust centroid from cameras + inlier points, then
  // find a radius that frames both cameras and the core point cloud.
  //
  // Two-pass approach:
  //   Pass 1: Compute centroid from cameras only (always reliable).
  //   Pass 2: Compute point cloud median center, blend with camera centroid.
  //   Radius: enclose cameras + 90th percentile of points from the centroid.

  // 1. Collect camera centers from both snapshots.
  std::vector<Vec3> cam_centers;
  cam_centers.reserve(before.num_cameras + after.num_cameras);
  for (int i = 0; i < before.num_cameras; ++i) {
    cam_centers.push_back(cameraCenter(&before.cameras[i * 9]));
  }
  for (int i = 0; i < after.num_cameras; ++i) {
    cam_centers.push_back(cameraCenter(&after.cameras[i * 9]));
  }

  // 2. Collect subsampled 3D points from both snapshots.
  std::vector<Vec3> sample_pts;
  int max_sample = 50000;
  auto samplePoints = [&](const BASnapshot& snap) {
    int step = 1;
    if (snap.num_points > max_sample) step = snap.num_points / max_sample + 1;
    for (int i = 0; i < snap.num_points; i += step) {
      sample_pts.push_back(
          {snap.points[i * 3], snap.points[i * 3 + 1], snap.points[i * 3 + 2]});
    }
  };
  samplePoints(before);
  samplePoints(after);

  // 3. Compute centroid as weighted blend of camera mean and point median.
  //    Using median for points is more robust to outliers than mean.
  Vec3 cam_mean{0, 0, 0};
  if (!cam_centers.empty()) {
    for (const auto& c : cam_centers) cam_mean = cam_mean + c;
    cam_mean = cam_mean * (1.0 / static_cast<double>(cam_centers.size()));
  }

  Vec3 pt_median{0, 0, 0};
  if (!sample_pts.empty()) {
    // Component-wise median.
    std::vector<double> xs, ys, zs;
    xs.reserve(sample_pts.size());
    ys.reserve(sample_pts.size());
    zs.reserve(sample_pts.size());
    for (const auto& p : sample_pts) {
      xs.push_back(p.x);
      ys.push_back(p.y);
      zs.push_back(p.z);
    }
    auto median = [](std::vector<double>& v) -> double {
      size_t n = v.size();
      std::nth_element(v.begin(), v.begin() + n / 2, v.end());
      return v[n / 2];
    };
    pt_median = {median(xs), median(ys), median(zs)};
  }

  // Blend: 30% camera mean + 70% point median (points define the scene).
  // If no points, use camera mean entirely.
  Vec3 centroid;
  if (sample_pts.empty()) {
    centroid = cam_mean;
  } else {
    centroid = cam_mean * 0.3 + pt_median * 0.7;
  }

  // 4. Compute radius from all entities (cameras + inlier points) relative
  //    to the blended centroid.  Use 95th percentile of the combined set
  //    to reject extreme outliers while still framing the scene well.
  std::vector<double> all_dists;
  all_dists.reserve(cam_centers.size() + sample_pts.size());
  for (const auto& c : cam_centers) {
    all_dists.push_back((c - centroid).norm());
  }
  for (const auto& p : sample_pts) {
    all_dists.push_back((p - centroid).norm());
  }
  std::sort(all_dists.begin(), all_dists.end());

  double radius = 1.0;
  if (!all_dists.empty()) {
    int idx = std::min(static_cast<int>(all_dists.size() * 0.95),
                       static_cast<int>(all_dists.size()) - 1);
    radius = all_dists[idx];
  }
  if (radius < 1e-6) radius = 1.0;

  return {centroid, radius};
}

// =============================================================================
// Drawing helpers
// =============================================================================

// Blend a color with the background based on depth (atmospheric perspective).
cv::Scalar depthFade(const cv::Scalar& color, double depth, double near_depth,
                     double far_depth, const cv::Scalar& bg,
                     double min_alpha = 0.3) {
  double t = (depth - near_depth) / (far_depth - near_depth + 1e-10);
  t = std::max(0.0, std::min(1.0, t));
  double alpha = 1.0 - t * (1.0 - min_alpha);
  return cv::Scalar(bg[0] + alpha * (color[0] - bg[0]),
                    bg[1] + alpha * (color[1] - bg[1]),
                    bg[2] + alpha * (color[2] - bg[2]));
}

// Compute point radius based on depth.
int depthRadius(double depth, double near_depth, double far_depth, int r_near,
                int r_far) {
  double t = (depth - near_depth) / (far_depth - near_depth + 1e-10);
  t = std::max(0.0, std::min(1.0, t));
  return static_cast<int>(r_near + t * (r_far - r_near) + 0.5);
}

// Draw a 3D line segment projected through the orbit camera.
void drawLine3D(cv::Mat& img, const OrbitCamera& cam, const Vec3& a,
                const Vec3& b, const cv::Scalar& color, int thickness = 1) {
  double ax, ay, az, bx, by, bz;
  bool va = cam.project(a, ax, ay, az);
  bool vb = cam.project(b, bx, by, bz);
  if (!va || !vb) return;

  cv::line(img, cv::Point(static_cast<int>(ax), static_cast<int>(ay)),
           cv::Point(static_cast<int>(bx), static_cast<int>(by)), color,
           thickness, cv::LINE_AA);
}

// Draw a 3D camera frustum as a wireframe pyramid.
//
// The frustum has 5 vertices: the camera center (apex) and 4 corners of the
// near plane.  We draw 4 edges from center to corners and 4 edges around
// the base rectangle.
void drawFrustum3D(cv::Mat& img, const OrbitCamera& orbit_cam,
                   const double* ba_cam, double frustum_len,
                   const cv::Scalar& color) {
  Vec3 center = cameraCenter(ba_cam);
  Vec3 fwd = cameraForward(ba_cam).normalized();
  Vec3 up_dir = cameraUp(ba_cam).normalized();
  Vec3 right_dir = cameraRight(ba_cam).normalized();

  // Near plane half-extents (aspect ratio ~4:3).
  double hw = frustum_len * 0.4;
  double hh = frustum_len * 0.3;

  Vec3 tip = center + fwd * frustum_len;
  Vec3 c0 = tip + right_dir * (-hw) + up_dir * (-hh);
  Vec3 c1 = tip + right_dir * (hw) + up_dir * (-hh);
  Vec3 c2 = tip + right_dir * (hw) + up_dir * (hh);
  Vec3 c3 = tip + right_dir * (-hw) + up_dir * (hh);

  // Edges from center to corners.
  drawLine3D(img, orbit_cam, center, c0, color, 1);
  drawLine3D(img, orbit_cam, center, c1, color, 1);
  drawLine3D(img, orbit_cam, center, c2, color, 1);
  drawLine3D(img, orbit_cam, center, c3, color, 1);

  // Base rectangle.
  drawLine3D(img, orbit_cam, c0, c1, color, 1);
  drawLine3D(img, orbit_cam, c1, c2, color, 1);
  drawLine3D(img, orbit_cam, c2, c3, color, 1);
  drawLine3D(img, orbit_cam, c3, c0, color, 1);

  // Draw a small filled circle at the camera center for visibility.
  double px, py, depth;
  if (orbit_cam.project(center, px, py, depth)) {
    cv::circle(img, cv::Point(static_cast<int>(px), static_cast<int>(py)), 3,
               color, -1, cv::LINE_AA);
  }
}

// Draw a ground plane grid in 3D.
void drawGroundGrid(cv::Mat& img, const OrbitCamera& cam, const Vec3& centroid,
                    double radius, double y_level, const cv::Scalar& color) {
  // Grid extent: slightly larger than the scene.
  double extent = radius * 1.2;

  // Grid spacing: ~8 lines across the extent.
  double raw = extent * 2.0 / 8.0;
  double log_s = std::floor(std::log10(raw));
  double spacing = std::pow(10.0, log_s);
  if (extent * 2.0 / spacing > 15) spacing *= 5;
  if (extent * 2.0 / spacing < 4) spacing *= 0.5;

  double x0 = centroid.x - extent;
  double x1 = centroid.x + extent;
  double z0 = centroid.z - extent;
  double z1 = centroid.z + extent;

  // Lines parallel to Z axis (constant X).
  double xs = std::ceil(x0 / spacing) * spacing;
  for (double x = xs; x <= x1; x += spacing) {
    drawLine3D(img, cam, {x, y_level, z0}, {x, y_level, z1}, color);
  }

  // Lines parallel to X axis (constant Z).
  double zs = std::ceil(z0 / spacing) * spacing;
  for (double z = zs; z <= z1; z += spacing) {
    drawLine3D(img, cam, {x0, y_level, z}, {x1, y_level, z}, color);
  }
}

// Draw XYZ axis indicator in the bottom-left corner of a panel.
void drawAxisIndicator(cv::Mat& img, const OrbitCamera& cam, int panel_x0,
                       int panel_h) {
  // We draw a small axis triad in screen space, but rotated according to
  // the orbit camera orientation so it shows the current viewing angle.
  int cx = panel_x0 + 50;
  int cy = panel_h - 50;
  double len = 30.0;

  // Project unit vectors through the camera's rotation (no translation).
  auto projectDir = [&](const Vec3& dir) -> cv::Point {
    double sx = dir.dot(cam.right);
    double sy = dir.dot(cam.cam_up);
    return cv::Point(cx + static_cast<int>(sx * len),
                     cy - static_cast<int>(sy * len));
  };

  cv::Point origin(cx, cy);
  cv::Point px = projectDir({1, 0, 0});
  cv::Point py = projectDir({0, 1, 0});
  cv::Point pz = projectDir({0, 0, 1});

  cv::line(img, origin, px, cv::Scalar(0, 0, 220), 2, cv::LINE_AA);  // X = red
  cv::line(img, origin, py, cv::Scalar(0, 220, 0), 2,
           cv::LINE_AA);  // Y = green
  cv::line(img, origin, pz, cv::Scalar(220, 100, 0), 2,
           cv::LINE_AA);  // Z = blue

  cv::putText(img, "X", px, cv::FONT_HERSHEY_SIMPLEX, 0.35,
              cv::Scalar(0, 0, 220), 1, cv::LINE_AA);
  cv::putText(img, "Y", py, cv::FONT_HERSHEY_SIMPLEX, 0.35,
              cv::Scalar(0, 220, 0), 1, cv::LINE_AA);
  cv::putText(img, "Z", pz, cv::FONT_HERSHEY_SIMPLEX, 0.35,
              cv::Scalar(220, 100, 0), 1, cv::LINE_AA);
}

// Draw text label at top of panel.
void drawLabel(cv::Mat& img, const std::string& text, int panel_x0, int width,
               const cv::Scalar& color) {
  int baseline = 0;
  double font_scale = 0.8;
  int thickness = 2;
  cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                                thickness, &baseline);
  int x = panel_x0 + (width - sz.width) / 2;
  int y = 30 + sz.height;
  cv::putText(img, text, {x, y}, cv::FONT_HERSHEY_SIMPLEX, font_scale, color,
              thickness, cv::LINE_AA);
}

// Draw stats text at bottom-right of panel.
void drawStats(cv::Mat& img, int num_cameras, int num_points, int panel_x0,
               int panel_w, int panel_h, const cv::Scalar& color) {
  std::ostringstream oss;
  oss << num_cameras << " cameras, " << num_points << " points";
  std::string s = oss.str();
  int baseline = 0;
  cv::Size sz = cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
  int x = panel_x0 + panel_w - sz.width - 15;
  int y = panel_h - 15;
  cv::putText(img, s, {x, y}, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
              cv::LINE_AA);
}

// =============================================================================
// Render a single panel (before or after)
// =============================================================================

struct PointDraw {
  double px, py, depth;
  int idx;
};

void renderPanel(cv::Mat& img, const OrbitCamera& cam, const BASnapshot& snap,
                 int panel_x0, int panel_w, int panel_h, double frustum_len,
                 const cv::Scalar& pt_color, const cv::Scalar& cam_color,
                 const cv::Scalar& bg_color, int r_near, int r_far) {
  // ---- Collect and depth-sort points ----
  int max_draw = 80000;
  int step = 1;
  if (snap.num_points > max_draw) {
    step = snap.num_points / max_draw + 1;
  }

  std::vector<PointDraw> draws;
  draws.reserve(snap.num_points / step + 1);

  double near_d = 1e30, far_d = -1e30;

  for (int i = 0; i < snap.num_points; i += step) {
    Vec3 pw(snap.points[i * 3], snap.points[i * 3 + 1], snap.points[i * 3 + 2]);
    double px, py, depth;
    if (cam.project(pw, px, py, depth)) {
      int ipx = static_cast<int>(px);
      int ipy = static_cast<int>(py);
      if (ipx >= panel_x0 && ipx < panel_x0 + panel_w && ipy >= 0 &&
          ipy < panel_h) {
        draws.push_back({px, py, depth, i});
        near_d = std::min(near_d, depth);
        far_d = std::max(far_d, depth);
      }
    }
  }

  // Sort back-to-front (farthest first) for painter's algorithm.
  std::sort(
      draws.begin(), draws.end(),
      [](const PointDraw& a, const PointDraw& b) { return a.depth > b.depth; });

  // Draw points.
  for (const auto& d : draws) {
    int r = depthRadius(d.depth, near_d, far_d, r_near, r_far);
    cv::Scalar c = depthFade(pt_color, d.depth, near_d, far_d, bg_color, 0.35);
    cv::circle(img, cv::Point(static_cast<int>(d.px), static_cast<int>(d.py)),
               r, c, -1, cv::LINE_AA);
  }

  // ---- Draw camera frustums ----
  // Sort cameras by depth (back-to-front) too.
  struct CamDraw {
    double depth;
    int idx;
  };
  std::vector<CamDraw> cam_draws;
  cam_draws.reserve(snap.num_cameras);

  for (int i = 0; i < snap.num_cameras; ++i) {
    Vec3 center = cameraCenter(&snap.cameras[i * 9]);
    double px, py, depth;
    if (cam.project(center, px, py, depth)) {
      cam_draws.push_back({depth, i});
    }
  }
  std::sort(
      cam_draws.begin(), cam_draws.end(),
      [](const CamDraw& a, const CamDraw& b) { return a.depth > b.depth; });

  for (const auto& cd : cam_draws) {
    cv::Scalar c = depthFade(cam_color, cd.depth, near_d, far_d, bg_color, 0.5);
    drawFrustum3D(img, cam, &snap.cameras[cd.idx * 9], frustum_len, c);
  }
}

}  // anonymous namespace

// =============================================================================
// BAVisualizer::renderBeforeAfter
// =============================================================================

bool BAVisualizer::renderBeforeAfter(const BASnapshot& before,
                                     const BASnapshot& after,
                                     const std::string& output_path,
                                     const VizConfig& cfg) {
  int panel_w = cfg.image_width;
  int panel_h = cfg.image_height;
  int total_w = panel_w * 2;

  // Create the full image.
  cv::Mat img(panel_h, total_w, CV_8UC3,
              cv::Scalar(cfg.bg_b, cfg.bg_g, cfg.bg_r));

  // Analyze scene to find centroid and radius.
  SceneStats stats = analyzeScene(before, after);

  // Place orbit camera at ~2.5x the scene radius for a good view.
  double distance = stats.radius * 2.5;
  double az_rad = cfg.azimuth_deg * kDegToRad;
  double el_rad = cfg.elevation_deg * kDegToRad;
  double fov_rad = cfg.fov_deg * kDegToRad;

  OrbitCamera cam_before, cam_after;
  cam_before.init(stats.centroid, distance, az_rad, el_rad, fov_rad, panel_w,
                  panel_h, 0);
  cam_after.init(stats.centroid, distance, az_rad, el_rad, fov_rad, panel_w,
                 panel_h, panel_w);

  // Frustum length for BA cameras (proportional to scene radius).
  double frustum_len = stats.radius * cfg.frustum_scale;

  // Colors.
  cv::Scalar bg_color(cfg.bg_b, cfg.bg_g, cfg.bg_r);
  cv::Scalar grid_color(cfg.grid_b, cfg.grid_g, cfg.grid_r);
  cv::Scalar text_color(cfg.text_b, cfg.text_g, cfg.text_r);

  cv::Scalar pt_before(cfg.before_colors.point_b, cfg.before_colors.point_g,
                       cfg.before_colors.point_r);
  cv::Scalar pt_after(cfg.after_colors.point_b, cfg.after_colors.point_g,
                      cfg.after_colors.point_r);
  cv::Scalar cam_before_c(cfg.before_colors.camera_b,
                          cfg.before_colors.camera_g,
                          cfg.before_colors.camera_r);
  cv::Scalar cam_after_c(cfg.after_colors.camera_b, cfg.after_colors.camera_g,
                         cfg.after_colors.camera_r);

  // Draw ground grid at the bottom of the scene (centroid.y - radius).
  double grid_y = stats.centroid.y - stats.radius * 0.5;
  drawGroundGrid(img, cam_before, stats.centroid, stats.radius, grid_y,
                 grid_color);
  drawGroundGrid(img, cam_after, stats.centroid, stats.radius, grid_y,
                 grid_color);

  // Render panels.
  renderPanel(img, cam_before, before, 0, panel_w, panel_h, frustum_len,
              pt_before, cam_before_c, bg_color, cfg.point_radius_near,
              cfg.point_radius_far);
  renderPanel(img, cam_after, after, panel_w, panel_w, panel_h, frustum_len,
              pt_after, cam_after_c, bg_color, cfg.point_radius_near,
              cfg.point_radius_far);

  // Separator line.
  cv::line(img, {panel_w, 0}, {panel_w, panel_h}, cv::Scalar(60, 60, 60), 2);

  // Labels.
  drawLabel(img, "Before Optimization", 0, panel_w, text_color);
  drawLabel(img, "After Optimization", panel_w, panel_w, text_color);

  // Stats.
  drawStats(img, before.num_cameras, before.num_points, 0, panel_w, panel_h,
            text_color);
  drawStats(img, after.num_cameras, after.num_points, panel_w, panel_w, panel_h,
            text_color);

  // Axis indicators.
  drawAxisIndicator(img, cam_before, 0, panel_h);
  drawAxisIndicator(img, cam_after, panel_w, panel_h);

  // Write output.
  bool ok = cv::imwrite(output_path, img);
  if (ok) {
    std::cout << "Visualization saved to: " << output_path << std::endl;
  } else {
    std::cerr << "Failed to write visualization to: " << output_path
              << std::endl;
  }
  return ok;
}

}  // namespace viz
}  // namespace substral
