#ifndef FGROPTIONS_H
#define FGROPTIONS_H

#include <boost/program_options.hpp>

namespace po = boost::program_options;

class FGROptions
{
public:
	bool 	verbose;
	bool 	use_absolute_scale;
	bool 	closed_form;
	float 	div_factor;
	float 	max_corr_dist;
	int 	iteration_number;
	float 	tuple_scale;
	int 	tuple_max_count;
	float 	normals_search_radius;
	float 	fpfh_search_radius;
    std::string ptCloudP_filename, ptCloudQ_filename;
    std::string outputfile, fitnessfile, reportfile;


	enum ReturnCode {OPTPARSE_SUCCESS,OPTPARSE_FAIL,OPTPARSE_HELP};
    int parse(int argc, char** argv);
private:
    std::string basename_(const std::string& p);
    std::vector<std::string> get_unlimited_positional_args_(const po::positional_options_description& p);
    std::string make_usage_string_(const std::string& program_name, const po::options_description& desc, po::positional_options_description& p);    
};

#endif