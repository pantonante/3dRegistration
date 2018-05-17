#include "goicp.h"

#include "spdlog/spdlog.h"

// #include <Eigen/Core>
// #include <Eigen/Geometry>

namespace goicp {

Goicp::Goicp(options::GoicpOptions opts) {
  initNodeRot.a = -PI;
  initNodeRot.b = -PI;
  initNodeRot.c = -PI;
  initNodeRot.w = 2 * PI;
  initNodeRot.l = 0;
  initNodeRot.lb = 0;
  initNodeTrans.lb = 0;
  doTrim = true;

  // Build Distance Transform
  double *x = (double *)malloc(sizeof(double) * cloudP->size());
  double *y = (double *)malloc(sizeof(double) * cloudP->size());
  double *z = (double *)malloc(sizeof(double) * cloudP->size());
  for (int i = 0; i < cloudP->size(); i++) {
    x[i] = pModel[i].x;
    y[i] = pModel[i].y;
    z[i] = pModel[i].z;
  }
  dt.Build(x, y, z, cloudP->size());
  delete (x);
  delete (y);
  delete (z);
}

float icp(Eigen::Affine3f &transform) { return 0.0; }

float innerBnB(float *maxRotDisL, Node *nodeTransOut) { return 0.0; }

float outerBnB() { return 0.0; }

void initialize() {}

void clear() {}

}  // namespace goicp
