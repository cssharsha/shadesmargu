#include <Eigen/Dense>
#include <cassert>
#include <iostream>

int main() {
  Eigen::Matrix2d a;
  a << 1, 2, 3, 4;
  Eigen::Vector2d b(1, 2);
  Eigen::Vector2d c = a * b;

  std::cout << "Eigen test passed: \n" << c << std::endl;

  assert(c(0) == 5);
  assert(c(1) == 11);

  return 0;
}
