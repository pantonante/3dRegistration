#pragma once

#include "distance_transform.h"
#include "goicp_options.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

namespace goicp {

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

struct Node {
  float a, b, c, w;
  float ub, lb;
  int l;
  friend bool operator<(const Node &n1, const Node &n2) {
    if (n1.lb != n2.lb)
      return n1.lb > n2.lb;
    else
      return n1.w < n2.w;
  }
};

class Goicp {
 public:
  PointCloud::Ptr cloudP, cloudQ;
  Node initNodeRot;
  Node initNodeTrans;
  Node optNodeRot;
  Node optNodeTrans;
  distance_transform::DistanceTransform dt;
  float MSEThresh;
  float SSEThresh;
  float icpThresh;
  float optError;
  float trimFraction;
  int inlierNum;
  bool doTrim;
  Eigen::Affine3f transform;

  Goicp(GoicpOptions opts);
  float performRegistration();

  /**
   * @brief Redefine new operator (suggested by Eigen).
   * See https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
   *
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  float icp(Eigen::Affine3f &transform);
  float innerBnB(float *maxRotDisL, Node *nodeTransOut);
  float outerBnB();
  void initialize();
  void clear();
};

}  // namespace goicp
