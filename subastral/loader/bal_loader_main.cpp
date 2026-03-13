#include <iostream>
#include <memory>
#include <vector>

#include "subastral/backend/common.h"
#include "subastral/loader/bal_loader.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_bal_dataset>" << std::endl;
    return 1;
  }

  std::string filename = argv[1];
  substral::loader::BALLoader loader(filename);

  auto memory_map = std::make_shared<substral::backend::FactorGraphMemoryMap>();
  std::vector<std::shared_ptr<substral::backend::Camera>> cameras;
  std::vector<std::shared_ptr<substral::backend::Point>> points;
  std::vector<std::shared_ptr<substral::backend::Observation>> observations;

  substral::backend::BAProblem problem;

  // if (loader.Load(memory_map, cameras, points, observations)) {
  if (loader.Load(problem)) {
    std::cout << "Successfully loaded BAL dataset." << std::endl;
    std::cout << "  Cameras: " << problem.cameras.size() << std::endl;
    std::cout << "  Points: " << problem.points.size() << std::endl;
    std::cout << "  Observations: " << problem.observations.size() << std::endl;

    // Print first camera details as a sanity check
    if (!problem.cameras.empty()) {
      const double* cam_data = problem.cameras[0]->data();
      std::cout << "Camera 0 params: ";
      for (int i = 0; i < 9; ++i) std::cout << cam_data[i] << " ";
      std::cout << std::endl;
    }
  } else {
    std::cerr << "Failed to load BAL dataset." << std::endl;
    return 1;
  }

  return 0;
}
