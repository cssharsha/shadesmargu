// Smoke test: verify the Rerun SDK links and basic types work.

#include <cassert>
#include <iostream>

#include "viz/rerun/ba_visualizer.hpp"

int main() {
  // Test 1: VizConfig disabled by default
  substral::rerun_viz::VizConfig config;
  assert(!config.enabled());
  std::cout << "PASS: VizConfig disabled by default\n";

  // Test 2: VizConfig enabled with rrd path
  config.rrd_path = "/tmp/test.rrd";
  assert(config.enabled());
  std::cout << "PASS: VizConfig enabled with rrd_path\n";

  // Test 3: BAVisualizer can be constructed/destructed
  {
    substral::rerun_viz::BAVisualizer viz;
    // Don't init — just verify construction/destruction works
  }
  std::cout << "PASS: BAVisualizer construct/destruct\n";

  std::cout << "All Rerun link tests passed.\n";
  return 0;
}
