#pragma once

#include <iostream>

#include <spdlog/spdlog.h>
#include <boost/program_options.hpp>

#include "default_config.h"

namespace goicp {

namespace po = boost::program_options;

class GoicpOptions {
 public:
  std::string ptCloudP_filename, ptCloudQ_filename, verbosity;

  enum ReturnCode { OPTPARSE_SUCCESS, OPTPARSE_FAIL, OPTPARSE_HELP };

  ReturnCode parse(int argc, char **argv);
  spdlog::level::level_enum getLogLevel();
  void printUsage(const char *self, po::options_description desc);
};

}  // namespace goicp