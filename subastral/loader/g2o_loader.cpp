#include "subastral/loader/g2o_loader.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

namespace substral {
namespace loader {

G2OLoader::G2OLoader(const std::string& filename) : filename_(filename) {}

bool G2OLoader::Load(backend::PoseGraphProblem& problem) {
  return Load(problem.memory_map, problem.poses, problem.edges);
}

bool G2OLoader::Load(std::shared_ptr<backend::FactorGraphMemoryMap> memory_map,
                     std::vector<std::shared_ptr<backend::Pose>>& poses,
                     std::vector<std::shared_ptr<backend::PoseEdge>>& edges) {
  std::ifstream file(filename_);
  if (!file.is_open()) {
    std::cerr << "Error opening g2o file: " << filename_ << std::endl;
    return false;
  }

  // First pass: count vertices and edges for reporting
  // (We parse in a single pass, but report at the end)

  std::string line;
  int edge_counter = 0;

  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;

    std::istringstream iss(line);
    std::string tag;
    iss >> tag;

    if (tag == "VERTEX_SE3:QUAT") {
      int id;
      double x, y, z, qx, qy, qz, qw;
      if (!(iss >> id >> x >> y >> z >> qx >> qy >> qz >> qw)) {
        std::cerr << "Error parsing VERTEX_SE3:QUAT line: " << line
                  << std::endl;
        return false;
      }

      auto pose = std::make_shared<backend::Pose>(id, memory_map);
      double* d = pose->mutable_data();
      d[0] = x;
      d[1] = y;
      d[2] = z;
      d[3] = qx;
      d[4] = qy;
      d[5] = qz;
      d[6] = qw;

      poses.push_back(pose);

    } else if (tag == "EDGE_SE3:QUAT") {
      int id_from, id_to;
      double dx, dy, dz, dqx, dqy, dqz, dqw;
      if (!(iss >> id_from >> id_to >> dx >> dy >> dz >> dqx >> dqy >> dqz >>
            dqw)) {
        std::cerr << "Error parsing EDGE_SE3:QUAT measurement: " << line
                  << std::endl;
        return false;
      }

      // Read 21 upper-triangular entries of the 6×6 information matrix.
      //
      // g2o convention: the info matrix rows/cols are ordered as
      //   (t_x, t_y, t_z, r_x, r_y, r_z) — translation first.
      //
      // Our internal convention (matching se3.hpp):
      //   (ω_x, ω_y, ω_z, v_x, v_y, v_z) — rotation first.
      //
      // So we need to permute: g2o rows [0,1,2] → our rows [3,4,5]
      //                        g2o rows [3,4,5] → our rows [0,1,2]
      //
      // We first read into a 6×6 with g2o ordering, then permute.

      double info_g2o[6][6] = {};
      for (int r = 0; r < 6; ++r) {
        for (int c = r; c < 6; ++c) {
          if (!(iss >> info_g2o[r][c])) {
            std::cerr << "Error parsing EDGE_SE3:QUAT info matrix: " << line
                      << std::endl;
            return false;
          }
          info_g2o[c][r] = info_g2o[r][c];  // symmetric
        }
      }

      // Permutation: g2o index → our index
      //   g2o [0,1,2,3,4,5] = (tx,ty,tz,rx,ry,rz)
      //   ours [0,1,2,3,4,5] = (rx,ry,rz,tx,ty,tz)
      //   perm[our_i] = g2o_i
      static constexpr int perm[6] = {3, 4, 5, 0, 1, 2};

      auto edge = std::make_shared<backend::PoseEdge>(edge_counter, id_from,
                                                      id_to, memory_map);

      // Set measurement
      double* m = edge->mutable_measurement_data();
      m[0] = dx;
      m[1] = dy;
      m[2] = dz;
      m[3] = dqx;
      m[4] = dqy;
      m[5] = dqz;
      m[6] = dqw;

      // Set permuted information matrix (row-major 6×6)
      double* info = edge->mutable_info_data();
      for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
          info[r * 6 + c] = info_g2o[perm[r]][perm[c]];
        }
      }

      edges.push_back(edge);
      ++edge_counter;

    } else if (tag == "FIX") {
      // Some g2o files have FIX lines — we ignore them and fix the first
      // vertex ourselves.
    } else {
      // Unknown tag — skip (could be 2D types, etc.)
      std::cerr << "Warning: skipping unknown g2o tag: " << tag << std::endl;
    }
  }

  if (poses.empty()) {
    std::cerr << "No VERTEX_SE3:QUAT entries found in " << filename_
              << std::endl;
    return false;
  }

  std::cout << "Loaded g2o file: " << filename_ << std::endl;
  std::cout << "  Vertices: " << poses.size() << std::endl;
  std::cout << "  Edges:    " << edges.size() << std::endl;

  // Count loop closures
  int loop_closures = 0;
  for (const auto& e : edges) {
    if (e->is_loop_closure()) ++loop_closures;
  }
  std::cout << "  Loop closures: " << loop_closures << std::endl;

  return true;
}

}  // namespace loader
}  // namespace substral
