#pragma once

#include <string>
#include <vector>

#include "subastral/backend/common.h"

namespace substral {
namespace viz {

// =============================================================================
// BAVisualizer — renders before/after BA results to a PNG image.
//
// Produces a side-by-side image:
//   Left  panel: "Before" — initial camera poses + 3D points
//   Right panel: "After"  — optimized camera poses + 3D points
//
// Both panels share the same virtual camera so the spatial relationship is
// directly comparable.
//
// Rendering:
//   - A virtual orbit camera is placed at an elevated angle looking at the
//     scene centroid, with perspective projection (pinhole model).
//   - 3D points are depth-sorted and drawn back-to-front with depth-based
//     sizing for natural depth cues.
//   - Cameras are drawn as 3D wireframe frustum pyramids showing position
//     and orientation.
//   - A ground plane grid is drawn in perspective for spatial reference.
//   - A small XYZ axis indicator is drawn in the corner.
//
// Usage:
//   // Before optimization — snapshot the initial state
//   viz::BASnapshot before = viz::BASnapshot::capture(problem);
//
//   // ... run optimization ...
//
//   // After optimization — snapshot again
//   viz::BASnapshot after = viz::BASnapshot::capture(problem);
//
//   // Render and save
//   viz::BAVisualizer::renderBeforeAfter(before, after, "output.png");
// =============================================================================

// Snapshot of camera poses and 3D point positions at a moment in time.
struct BASnapshot {
  // Camera data: N cameras x 9 parameters each (angle-axis, translation, f,
  // k1, k2). Stored as a flat vector in the same layout as
  // BAProblem::observers.
  std::vector<double> cameras;

  // Point data: M points x 3 parameters each (X, Y, Z).
  // Stored as a flat vector in the same layout as BAProblem::scene_points.
  std::vector<double> points;

  int num_cameras = 0;
  int num_points = 0;

  // Capture a snapshot of the current state of a BAProblem.
  static BASnapshot capture(const backend::BAProblem& problem);
};

// Configuration for the visualizer.
struct VizConfig {
  int image_width = 1920;   // Width of each panel (total image = 2x this)
  int image_height = 1080;  // Height of each panel
  int margin = 60;          // Margin around the scene in pixels

  // Virtual orbit camera parameters.
  double elevation_deg = 35.0;  // Elevation angle above horizon (degrees)
  double azimuth_deg = -30.0;   // Azimuth angle (degrees, 0 = looking along +X)
  double fov_deg = 50.0;        // Vertical field of view (degrees)

  // Camera frustum scale (fraction of scene radius).
  double frustum_scale = 0.04;

  // Point rendering.
  int point_radius_near = 3;  // Point radius at nearest depth
  int point_radius_far = 1;   // Point radius at farthest depth

  // Colors (BGR format for OpenCV).
  // Before panel.
  struct {
    int camera_b = 60, camera_g = 60, camera_r = 230;  // Red
    int point_b = 100, point_g = 100, point_r = 210;   // Light red
  } before_colors;

  // After panel.
  struct {
    int camera_b = 60, camera_g = 210, camera_r = 60;  // Green
    int point_b = 100, point_g = 210, point_r = 100;   // Light green
  } after_colors;

  // Background.
  int bg_b = 20, bg_g = 20, bg_r = 25;  // Near black

  // Grid.
  int grid_b = 45, grid_g = 45, grid_r = 45;  // Subtle gray

  // Text.
  int text_b = 200, text_g = 200, text_r = 200;  // Light gray
};

class BAVisualizer {
 public:
  // Render a side-by-side before/after image and write it to a PNG file.
  //
  // Returns true on success, false if the image could not be written.
  static bool renderBeforeAfter(const BASnapshot& before,
                                const BASnapshot& after,
                                const std::string& output_path,
                                const VizConfig& config = VizConfig{});
};

}  // namespace viz
}  // namespace substral
