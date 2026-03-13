#pragma once

#include <memory>
#include <string>
#include <vector>

#include "subastral/backend/common.h"

namespace substral {
namespace loader {

class BALLoader {
 public:
  BALLoader(const std::string& filename);

  bool Load(std::shared_ptr<backend::FactorGraphMemoryMap> memory_map,
            std::vector<std::shared_ptr<backend::Camera>>& cameras,
            std::vector<std::shared_ptr<backend::Point>>& points,
            std::vector<std::shared_ptr<backend::Observation>>& observations);
  bool Load(backend::BAProblem& problem) {
    return Load(problem.memory_map, problem.cameras, problem.points,
                problem.observations);
  }

 private:
  std::string filename_;
};

}  // namespace loader
}  // namespace substral
