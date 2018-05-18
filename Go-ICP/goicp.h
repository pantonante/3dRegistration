#pragma once

// #include "distance_transform.h"
#include "goicp_options.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "spdlog/spdlog.h"

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
  PointCloud::Ptr cloudP, cloudQ;  //!< normalized point clouds
  Node initNodeRot;
  Node initNodeTrans;
  Node optNodeRot;
  Node optNodeTrans;

  GoicpOptions opts;

  // distance_transform::DistanceTransform dt;
  // float MSEThresh;
  // float SSEThresh;
  // float icpThresh;
  // float optError;
  // float trimFraction;
  // int inlierNum;
  // bool doTrim;

  Goicp(PointCloud::Ptr cloud_p, PointCloud::Ptr cloud_q, GoicpOptions options);
  float performRegistration();
  Eigen::Matrix4f getTransform();

  /**
   * @brief Redefine new operator (suggested by Eigen).
   * See https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
   *
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  std::shared_ptr<spdlog::logger> logger_;
  Eigen::Matrix4f transform_;  //!< so-far best transformation matrix for normalized pt. clouds
  Eigen::Vector3f cloudP_centroid_, cloudQ_centroid_;  //!< point cloud centroids
  float global_scale_;  //!< scaling factor to normalize pt. clouds into [-1, 1]^3
  std::vector<std::vector<float> > rot_uncert;  //!< rotation uncertainty for each rotation subcube

  float icp(Eigen::Matrix4f &transform);
  float innerBnB(float *maxRotDisL, Node *nodeTransOut);
  float outerBnB();
};

}  // namespace goicp
