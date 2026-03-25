#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace substral {

/// Exception thrown when a transform lookup fails.
class TransformException : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// A lightweight transform tree for managing named coordinate frames.
///
/// Stores rigid body transforms (SE(3) as Eigen::Matrix4d) between named
/// coordinate frames organized as a forest. Supports automatic chaining:
/// if transforms A→B and B→C exist, lookup("C", "A") returns
/// T_C_A = T_C_B * T_B_A.
///
/// Convention: lookup("target", "source") returns T_target_source,
/// the transform that takes points FROM source TO target:
///   p_target = T_target_source * p_source
///
/// Thread safety: concurrent reads are safe (shared_mutex). Static transforms
/// are typically set during initialization before concurrent access begins.
///
/// Not a singleton. Created during pipeline setup and passed by shared_ptr
/// to all components that need it.
///
/// Future: dynamic (time-varying) transforms can be added via setDynamic()
/// with a timestamped buffer and interpolation, without changing the
/// static lookup API.
class TransformTree {
 public:
  TransformTree() = default;

  // ---- Static transforms ----

  /// Register a static transform between parent and child frames.
  /// Parent frame is auto-created as a root if it doesn't exist yet.
  ///
  /// @param parent  Parent frame ID (e.g., "world")
  /// @param child   Child frame ID (e.g., "optical")
  /// @param T_parent_child  Transform: p_parent = T_parent_child * p_child
  /// @throws TransformException if child already has a different parent,
  ///         or if the edge would create a cycle.
  void setStatic(const std::string& parent, const std::string& child,
                 const Eigen::Matrix4d& T_parent_child) {
    std::unique_lock lock(mutex_);

    // Ensure parent node exists (create as root if not)
    if (nodes_.find(parent) == nodes_.end()) {
      nodes_[parent] = FrameNode{parent, "", Eigen::Matrix4d::Identity()};
    }

    // Check child doesn't already exist with a different parent
    auto it = nodes_.find(child);
    if (it != nodes_.end() && !it->second.parent.empty() &&
        it->second.parent != parent) {
      throw TransformException("Frame '" + child + "' already has parent '" +
                               it->second.parent + "', cannot reparent to '" +
                               parent + "'");
    }

    // Check for cycle: walk from parent to root, ensure child isn't encountered
    std::string current = parent;
    while (!current.empty()) {
      if (current == child) {
        throw TransformException("Cycle detected: '" + child +
                                 "' is an ancestor of '" + parent + "'");
      }
      auto node_it = nodes_.find(current);
      if (node_it == nodes_.end()) break;
      current = node_it->second.parent;
    }

    nodes_[child] = FrameNode{child, parent, T_parent_child};
  }

  // ---- Lookups ----

  /// Look up the transform from source frame to target frame.
  /// Auto-chains through the tree via LCA (Lowest Common Ancestor).
  ///
  /// @throws TransformException if either frame is unknown or no path exists.
  Eigen::Matrix4d lookup(const std::string& target,
                         const std::string& source) const {
    if (target == source) return Eigen::Matrix4d::Identity();

    std::shared_lock lock(mutex_);

    auto path_source = pathToRoot(source);
    auto path_target = pathToRoot(target);

    // Check both paths end at the same root
    if (path_source.back() != path_target.back()) {
      throw TransformException("No path from '" + source + "' to '" + target +
                               "': different trees");
    }

    // Walk backwards (from root) to find where paths diverge → LCA
    int is = static_cast<int>(path_source.size()) - 1;
    int it = static_cast<int>(path_target.size()) - 1;

    while (is > 0 && it > 0 && path_source[is - 1] == path_target[it - 1]) {
      --is;
      --it;
    }
    // LCA is at path_source[is] == path_target[it]

    // T_LCA_source: compose from source up to LCA
    Eigen::Matrix4d T_lca_source = Eigen::Matrix4d::Identity();
    for (int i = 0; i < is; ++i) {
      const auto& node = nodes_.at(path_source[i]);
      T_lca_source = node.T_parent_child * T_lca_source;
    }

    // T_LCA_target: compose from target up to LCA
    Eigen::Matrix4d T_lca_target = Eigen::Matrix4d::Identity();
    for (int i = 0; i < it; ++i) {
      const auto& node = nodes_.at(path_target[i]);
      T_lca_target = node.T_parent_child * T_lca_target;
    }

    // T_target_source = inv(T_LCA_target) * T_LCA_source
    return T_lca_target.inverse() * T_lca_source;
  }

  /// Non-throwing variant. Returns identity and sets ok=false on failure.
  Eigen::Matrix4d lookup(const std::string& target, const std::string& source,
                         bool& ok) const noexcept {
    try {
      ok = true;
      return lookup(target, source);
    } catch (const TransformException&) {
      ok = false;
      return Eigen::Matrix4d::Identity();
    }
  }

  // ---- Queries ----

  bool hasFrame(const std::string& frame_id) const {
    std::shared_lock lock(mutex_);
    return nodes_.count(frame_id) > 0;
  }

  std::vector<std::string> frames() const {
    std::shared_lock lock(mutex_);
    std::vector<std::string> result;
    result.reserve(nodes_.size());
    for (const auto& [id, _] : nodes_) {
      result.push_back(id);
    }
    return result;
  }

  bool hasDirectTransform(const std::string& parent,
                          const std::string& child) const {
    std::shared_lock lock(mutex_);
    auto it = nodes_.find(child);
    return it != nodes_.end() && it->second.parent == parent;
  }

 private:
  struct FrameNode {
    std::string id;
    std::string parent;              // empty for root frames
    Eigen::Matrix4d T_parent_child;  // transform: child → parent
  };

  /// Walk from frame_id to root, returning [frame_id, ..., root].
  std::vector<std::string> pathToRoot(const std::string& frame_id) const {
    auto it = nodes_.find(frame_id);
    if (it == nodes_.end()) {
      throw TransformException("Unknown frame: '" + frame_id + "'");
    }

    std::vector<std::string> path;
    std::string current = frame_id;
    while (!current.empty()) {
      path.push_back(current);
      auto node_it = nodes_.find(current);
      if (node_it == nodes_.end()) break;
      current = node_it->second.parent;
    }
    return path;
  }

  mutable std::shared_mutex mutex_;
  std::unordered_map<std::string, FrameNode> nodes_;
};

}  // namespace substral
