#include "subastral/loader/bal_loader.h"

#include <fstream>
#include <iostream>

namespace substral {
namespace loader {

BALLoader::BALLoader(const std::string& filename) : filename_(filename) {}

bool BALLoader::Load(
    std::shared_ptr<backend::FactorGraphMemoryMap> memory_map,
    std::vector<std::shared_ptr<backend::Camera>>& cameras,
    std::vector<std::shared_ptr<backend::Point>>& points,
    std::vector<std::shared_ptr<backend::Observation>>& observations) {
  std::ifstream file(filename_);
  if (!file.is_open()) {
    std::cerr << "Error opening file: " << filename_ << std::endl;
    return false;
  }

  int num_cameras, num_points, num_observations;
  if (!(file >> num_cameras >> num_points >> num_observations)) {
    std::cerr << "Error reading header" << std::endl;
    return false;
  }

  std::cout << "Header: " << num_cameras << " cameras, " << num_points
            << " points, " << num_observations << " observations" << std::endl;

  // Read observations
  for (int i = 0; i < num_observations; ++i) {
    int cam_idx, point_idx;
    double x, y;
    if (!(file >> cam_idx >> point_idx >> x >> y)) {
      std::cerr << "Error reading observation " << i << std::endl;
      return false;
    }

    // Create Observation entity
    // Note: Observation constructor resizes the memory map
    auto obs = std::make_shared<backend::Observation>(i, cam_idx, point_idx,
                                                      memory_map);
    observations.push_back(obs);

    // Manually set the data in the memory map since Observation is immutable
    // via mutable_data The indices are managed by the Observation constructor
    // We need to access the memory map directly using the index stored in the
    // map
    int data_idx = memory_map->observation_indices[i];
    memory_map->observations[data_idx] = x;
    memory_map->observations[data_idx + 1] =
        y;  // Inverted y? BAL format is -y usually for pixel coords but let's
            // stick to raw
    // Actually BAL format is: <camera_index> <point_index> <x> <y>
    // The origin is at the center of the image.
    // The x-axis points to the right, and the y-axis points up.
    // We just store raw values here.
  }

  // Read camera parameters
  // 9 parameters per camera: R (3), t (3), f, k1, k2
  for (int i = 0; i < num_cameras; ++i) {
    auto cam = std::make_shared<backend::Camera>(i, memory_map);
    cameras.push_back(cam);

    double* data = cam->mutable_data();
    for (int j = 0; j < 9; ++j) {
      if (!(file >> data[j])) {
        std::cerr << "Error reading camera " << i << " parameter " << j
                  << std::endl;
        return false;
      }
    }
  }

  // Read point parameters
  // 3 parameters per point: X, Y, Z
  for (int i = 0; i < num_points; ++i) {
    auto point = std::make_shared<backend::Point>(i, memory_map);
    points.push_back(point);

    double* data = point->mutable_data();
    for (int j = 0; j < 3; ++j) {
      if (!(file >> data[j])) {
        std::cerr << "Error reading point " << i << " parameter " << j
                  << std::endl;
        return false;
      }
    }
  }

  return true;
}

}  // namespace loader
}  // namespace substral
