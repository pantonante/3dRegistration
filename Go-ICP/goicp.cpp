#define _USE_MATH_DEFINES

#include "goicp.h"

#include <cmath>
#include <limits>

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/incremental_registration.h>
// #include <Eigen/Geometry>

namespace goicp {

Goicp::Goicp(PointCloud::Ptr cloud_p, PointCloud::Ptr cloud_q, GoicpOptions options)
    : cloudP(new PointCloud()), cloudQ(new PointCloud()) {
  opts = options;
  initNodeRot.a = initNodeRot.b = initNodeRot.c = -M_PI;
  initNodeRot.w = 2 * M_PI;
  initNodeRot.l = 0;
  initNodeRot.lb = 0;
  initNodeTrans.lb = 0;
  doTrim = true;

  global_scale_ = 1;
  transform_ = Eigen::Matrix4f::Identity();

  logger_ = spdlog::stdout_color_mt("Go-ICP");

  // Build Distance Transform
  // logger_->trace("Building Distance Transform");
  // double *x = new double[cloudP->size()];
  // double *y = new double[cloudP->size()];
  // double *z = new double[cloudP->size()];
  // for (int i = 0; i < cloudP->size(); i++) {
  //   // TODO(pantonante): waste of memory, distance transform should accept point clouds
  //   x[i] = static_cast<double>(cloudP->points[i].x);
  //   y[i] = static_cast<double>(cloudP->points[i].y);
  //   z[i] = static_cast<double>(cloudP->points[i].z);
  // }
  // dt.build(x, y, z, cloudP->size());
  // delete[] x;
  // delete[] y;
  // delete[] z;

  // Demean point cloud
  logger_->trace("Demeaning point clouds");
  Eigen::Vector4f centroid;
  pcl::demeanPointCloud(*cloud_p, centroid, *cloudP);
  cloudP_centroid_ = centroid.head<3>();
  pcl::demeanPointCloud(*cloud_q, centroid, *cloudQ);
  cloudQ_centroid_ = centroid.head<3>();

  // Normalization
  logger_->trace("Normalizing point clouds");
  for (auto pt : cloudP->points) {  // find global scale
    const auto l2norm = pt.getVector3fMap().norm();
    if (l2norm > global_scale_) global_scale_ = l2norm;
  }
  for (auto pt : cloudQ->points) {
    const auto l2norm = pt.getVector3fMap().norm();
    if (l2norm > global_scale_) global_scale_ = l2norm;
  }
  const auto scaling = Eigen::Matrix4f::Identity() * (1 / global_scale_);
  pcl::transformPointCloud(*cloudP, *cloudP, scaling);
  pcl::transformPointCloud(*cloudQ, *cloudQ, scaling);
}

float Goicp::performRegistration() {
  Eigen::Matrix4f local_transform;
  float fitness = icp(local_transform);
  if (std::isfinite(fitness)) {
    transform_ = local_transform;
  }
  return fitness;
}

Eigen::Matrix4f Goicp::getTransform() {
  Eigen::Matrix3f rot(transform_.block<3, 3>(0, 0));
  Eigen::Vector3f t(transform_.block<3, 1>(0, 3));

  Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
  global_transform.block<3, 3>(0, 0) = rot;
  global_transform.block<3, 1>(0, 3) =
      -rot * cloudQ_centroid_ + t * global_scale_ + cloudP_centroid_;

  return global_transform;
}

float Goicp::icp(Eigen::Matrix4f &transform) {
  logger_->trace("ICP, max. iterations {}", opts.icp_max_iterations);
  auto fitness = std::numeric_limits<float>::infinity();
  pcl::IterativeClosestPoint<Point, Point> icp;
  icp.setMaximumIterations(opts.icp_max_iterations);
  icp.setInputSource(cloudP);
  icp.setInputTarget(cloudQ);
  icp.align(*cloudP);
  logger_->trace("ICP done, converged: {}", icp.hasConverged());

  if (icp.hasConverged()) {
    fitness = icp.getFitnessScore();
    transform = icp.getFinalTransformation();
  } else {
    logger_->error("ICP not converged");
  }

  return fitness;
}

float Goicp::innerBnB(float *maxRotDisL, Node *nodeTransOut) { return 0.0; }

float Goicp::outerBnB() { return 0.0; }

}  // namespace goicp
