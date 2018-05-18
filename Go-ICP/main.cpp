#include <iostream>

#include "art.h"
#include "goicp.h"
#include "goicp_options.h"

#include <spdlog/spdlog.h>

int main(int argc, char **argv) {
  std::cout << goicp::art::kCliHeader;

  // Instantiate and parse fast global registration options
  goicp::GoicpOptions options;

  auto rt = options.parse(argc, argv);
  if (rt == goicp::GoicpOptions::ReturnCode::OPTPARSE_HELP)
    return 1;
  else if (rt == goicp::GoicpOptions::ReturnCode::OPTPARSE_FAIL)
    return 0;

  // Setup logger
  auto logger = spdlog::stdout_color_mt("main");
  spdlog::set_level(options.getLogLevel());  // Set global log level
  // Set logging patter to "[hh:mm:ss:millisec:macrosec] [logger_name] [loglevel] message"
  spdlog::set_pattern("[%T:%e:%f] [%n] [%l] %v");

  // Instantiate and load point Clouds
  goicp::PointCloud::Ptr ptCloud_P(new goicp::PointCloud());
  goicp::PointCloud::Ptr ptCloud_Q(new goicp::PointCloud());

  if (pcl::io::loadPCDFile(options.ptCloudP_filename, *ptCloud_P) < 0) {
    logger->critical("Error loading point cloud {}.", options.ptCloudP_filename);
    return 0;
  }
  logger->info("Point Coud {} successfully loaded, {} pts.", options.ptCloudP_filename,
               ptCloud_P->width);

  if (pcl::io::loadPCDFile(options.ptCloudQ_filename, *ptCloud_Q) < 0) {
    logger->critical("Error loading point cloud {}.", options.ptCloudQ_filename);
    return 0;
  }
  logger->info("Point Coud {} successfully loaded, {} pts.", options.ptCloudQ_filename,
               ptCloud_Q->width);

  logger->debug("Warning: verbose execution enabled, timing information might be inflated.");

  // Starting Go-ICP
  logger->info("Starting Go-ICP...");
  goicp::Goicp go_icp(ptCloud_P, ptCloud_Q, options);
  go_icp.performRegistration();

  // show results
  auto transform = go_icp.getTransform();
  const Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  std::cout << "-------------------------------------------------------------" << std::endl;
  std::cout << "Transformation matrix: " << std::endl << std::endl;
  std::cout << transform.format(OctaveFmt) << std::endl;
  std::cout << "-------------------------------------------------------------" << std::endl;

  return 1;
}