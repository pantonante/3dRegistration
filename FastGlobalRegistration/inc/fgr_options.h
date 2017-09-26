#ifndef FGROPTIONS_H
#define FGROPTIONS_H

#include <boost/program_options.hpp>

namespace po = boost::program_options;

class FGROptions
{
public:
    int parse(int argc, char** argv);
private:
    std::string basename_(const std::string& p);
    std::vector<std::string> get_unlimited_positional_args_(const po::positional_options_description& p);
    std::string make_usage_string_(const std::string& program_name, const po::options_description& desc, po::positional_options_description& p);
public:
    enum ReturnCode {OPTPARSE_SUCCESS,OPTPARSE_FAIL,OPTPARSE_HELP};
    std::string ptCloudP_filename;
    std::string ptCloudQ_filename;
};

#endif