#include "goicp_options.h"

namespace goicp {

GoicpOptions::ReturnCode GoicpOptions::parse(int argc, char **argv) {
  po::options_description req("Required arguments");
  // clang-format off
  req.add_options()
    ("pointcloudP,p", po::value<std::string>(&ptCloudP_filename)->required(),
      "Point cloud filename [*.pcd].\n")
    ("pointcloudQ,q", po::value<std::string>(&ptCloudQ_filename)->required(),
      "Point cloud filename [*.pcd].\n");
  // clang-format on

  po::options_description alg("Algorith parameters");
  // clang-format off
  alg.add_options()
    ("fitness-threashold, f", po::value<float>(&fitness_threshold)->default_value(goicp::defaults::kFitnessThreashold),
      "Stopping criteria on fitness")
    ("rot-subcubes", po::value<int>(&rot_subcubes)->default_value(goicp::defaults::kRotSubcubes),
      "Max number of rotation subcubes")
    ("icp-max-iter", po::value<int>(&icp_max_iterations)->default_value(goicp::defaults::kIcpMaxIterations),
      "Vanilla ICP maximum iterations.\n");
  // clang-format on

  po::options_description misc("Miscellaneous");
  // clang-format off
  misc.add_options()
    ("help,h", "Print help messages\n")
    ("loglevel,l", po::value<std::string>(&verbosity)->required()->default_value("info"),
      "Loggin level.\n");
  // clang-format on

  po::options_description all_options;
  all_options.add(req);
  all_options.add(alg);
  all_options.add(misc);

  po::variables_map vm;
  try {
    po::store(po::command_line_parser(argc, argv).options(all_options).run(), vm);

    if (vm.count("help")) {
      printUsage(argv[0], all_options);
      return GoicpOptions::OPTPARSE_HELP;
    }

    po::notify(vm);

  } catch (po::error &e) {
    auto logger = spdlog::stdout_color_mt("options");
    logger->error(e.what());
    printUsage(argv[0], all_options);
    return GoicpOptions::OPTPARSE_FAIL;
  }

  return GoicpOptions::OPTPARSE_SUCCESS;
}

spdlog::level::level_enum GoicpOptions::getLogLevel() {
  static std::unordered_map<std::string, spdlog::level::level_enum>
      name_to_level =  // map string->level
      {{"trace", spdlog::level::trace}, {"debug", spdlog::level::debug},
       {"info", spdlog::level::info},   {"warning", spdlog::level::warn},
       {"error", spdlog::level::err},   {"critical", spdlog::level::critical},
       {"off", spdlog::level::off}};

  auto lvl_it = name_to_level.find(verbosity);
  return lvl_it != name_to_level.end() ? lvl_it->second : spdlog::level::off;
}

void GoicpOptions::printUsage(const char *self, po::options_description desc) {
  std::cout << "\nUsage: " << self << " [options]" << std::endl;
  std::cout << desc << std::endl;
}

}  // namespace goicp