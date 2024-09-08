#include "src/common_function/logger/logger.h"
#include "src/math/ToraMatrix33.h"

int main(int argc, char* argv[]) {
  using namespace ::Tora;
  ToraVector3<> test_vec3(0.0, 0.0, 100);
  auto test_quaX = Q_from_AngX(45.0 * DEG_TO_RAD);
  auto test_quaY = Q_from_AngY(180.0 * DEG_TO_RAD);
  auto test_quaZ = Q_from_AngZ(270.0 * DEG_TO_RAD);

  LOG_INFO("vec3:{}, rotate x  45deg, will be:{}", test_vec3, test_quaX.Rotate(test_vec3));
  LOG_INFO("vec3:{}, rotate y 180deg, will be:{}", test_vec3, test_quaY.Rotate(test_vec3));
  LOG_INFO("vec3:{}, rotate x 270deg, will be:{}", test_vec3, test_quaZ.Rotate(test_vec3));
  return 0;
}